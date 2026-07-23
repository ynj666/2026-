#include "encoder.h"            // 引入编码器模块自身的头文件（引脚枚举、函数声明）
#include "ti_msp_dl_config.h"   // 引入SysConfig生成的配置文件（GPIO/中断等引脚定义）

typedef struct {                // 定义编码器硬件配置结构体（描述一路编码器接在哪个端口、哪两个引脚）
    GPIO_Regs *port;            // 编码器所在的GPIO端口寄存器基地址（如GPIOB）
    uint32_t pinA;              // 编码器A相引脚掩码
    uint32_t pinB;              // 编码器B相引脚掩码
    int8_t dir;                 // 计数方向系数：1或-1，用于修正电机安装方向导致的计数反向
} Encoder_Config;               // 结构体类型命名为Encoder_Config

/* 换引脚、反方向只改这里 */
static const Encoder_Config encoder_cfg[ENCODER_NUM] = {          // 编码器硬件配置表：四路编码器的引脚映射，常量存于Flash
    {GPIOB, DL_GPIO_PIN_0, DL_GPIO_PIN_16,  1}, // ENC1_A ENC1_B   // 编码器1：A相=PB0，B相=PB16，方向为正（1）
    {GPIOB, DL_GPIO_PIN_1,  DL_GPIO_PIN_4,  -1}, // ENC2_A ENC2_B   // 编码器2：A相=PB1，B相=PB4，方向为正（1）
    {GPIOB, DL_GPIO_PIN_10, DL_GPIO_PIN_11, 1}, // ENC3_A ENC3_B   // 编码器3：A相=PB10，B相=PB11，方向为正（1），反了改这里
    {GPIOB, DL_GPIO_PIN_22, DL_GPIO_PIN_21, 1}, // ENC4_A ENC4_B   // 编码器4：A相=PB22，B相=PB21，方向为正（1），反了改这里
};                                                              // 配置表结束

static volatile int32_t encoder_count[ENCODER_NUM];             // 四路编码器的脉冲累计计数，volatile防止被编译器优化（中断中修改）
static uint8_t encoder_last[ENCODER_NUM];                       // 四路编码器上一次采样到的AB相电平状态（2位）

static uint8_t Encoder_ReadState(ENCODER_ID id)                 // 读取指定编码器当前AB相电平，组合成2位状态
{                                                               // 函数体开始
    uint8_t a = (DL_GPIO_readPins(encoder_cfg[id].port, encoder_cfg[id].pinA) != 0);  // 读A相引脚电平，非0则为1
    uint8_t b = (DL_GPIO_readPins(encoder_cfg[id].port, encoder_cfg[id].pinB) != 0);  // 读B相引脚电平，非0则为1
    return (a << 1) | b;                                        // 把A放在高位、B放在低位，合成0~3的状态值返回
}                                                               // Encoder_ReadState函数结束

static void Encoder_Update(ENCODER_ID id)                       // 编码器状态更新：根据AB相跳变方向对计数器加减1
{                                                               // 函数体开始
    static const int8_t table[16] = {                           // 正交解码查值表：索引=上次状态(2位)+本次状态(2位)，值=+1/-1/0
         0,  1, -1,  0,                                         // 上次状态00时的四种跳变结果
        -1,  0,  0,  1,                                         // 上次状态01时的四种跳变结果
         1,  0,  0, -1,                                         // 上次状态10时的四种跳变结果
         0, -1,  1,  0                                          // 上次状态11时的四种跳变结果
    };                                                          // 查值表结束

    uint8_t now = Encoder_ReadState(id);                        // 读取编码器当前AB相状态
    uint8_t index = (encoder_last[id] << 2) | now;              // 上次状态移高2位再拼上当前状态，得到0~15的查表索引

    encoder_count[id] += table[index] * encoder_cfg[id].dir;    // 查表得到增减量，乘方向系数后累加到脉冲计数
    encoder_last[id] = now;                                     // 保存当前状态，作为下次比较的"上次状态"
}                                                               // Encoder_Update函数结束

void Encoder_Init(void)                                         // 编码器初始化：清零计数并使能GPIOB中断
{                                                               // 函数体开始
    for (uint8_t i = 0; i < ENCODER_NUM; i++) {                 // 遍历所有编码器
        encoder_count[i] = 0;                                   // 脉冲计数清零
        encoder_last[i] = Encoder_ReadState((ENCODER_ID)i);     // 记录当前AB相状态作为初始"上次状态"
    }                                                           // for循环结束

    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);                       // 清除GPIOB中断挂起标志，避免上电残留的误触发
    NVIC_EnableIRQ(GPIOB_INT_IRQn);                             // 在NVIC中使能GPIOB外部中断（编码器AB相跳变触发）
}                                                               // Encoder_Init函数结束

int32_t Encoder_Get(ENCODER_ID id)                              // 读取指定编码器的累计脉冲数
{                                                               // 函数体开始
    if (id >= ENCODER_NUM) return 0;                            // 参数越界保护：非法编号直接返回0
    return encoder_count[id];                                   // 返回该编码器的累计脉冲计数
}                                                               // Encoder_Get函数结束

void Encoder_Clear(ENCODER_ID id)                               // 清零指定编码器的脉冲计数
{                                                               // 函数体开始
    if (id >= ENCODER_NUM) return;                              // 参数越界保护：非法编号直接返回
    encoder_count[id] = 0;                                      // 脉冲计数清零
}                                                               // Encoder_Clear函数结束

void Encoder_ClearAll(void)                                     // 清零所有编码器的脉冲计数
{                                                               // 函数体开始
    uint8_t i;                                                  // 循环计数变量

    for (i = 0; i < ENCODER_NUM; i++) {                         // 遍历全部四路编码器
        Encoder_Clear((ENCODER_ID)i);                           // 清零该路脉冲计数
    }                                                           // for循环结束
}                                                               // Encoder_ClearAll函数结束

void GROUP1_IRQHandler(void)                                    // GPIO中断组1的中断服务函数（GPIOA/GPIOB共用此入口）
{                                                               // 函数体开始
    uint32_t pending;                                           // 存放触发中断的引脚状态标志

    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {  // 查询中断组1中是哪个外设触发的中断
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:                    // 是GPIOB触发的中断（编码器引脚）
            pending = DL_GPIO_getEnabledInterruptStatus(GPIOB,  // 读取GPIOB上8个编码器引脚中哪些触发了中断
                DL_GPIO_PIN_0 | DL_GPIO_PIN_16 | DL_GPIO_PIN_1 | DL_GPIO_PIN_4 |
                DL_GPIO_PIN_10 | DL_GPIO_PIN_11 | DL_GPIO_PIN_22 | DL_GPIO_PIN_21);

            if (pending & (DL_GPIO_PIN_0 | DL_GPIO_PIN_16)) {   // 若编码器1的A相或B相引脚触发了中断
                Encoder_Update(ENCODER_1);                      // 更新编码器1的脉冲计数
            }                                                   // if结束

            if (pending & (DL_GPIO_PIN_1 | DL_GPIO_PIN_4)) {    // 若编码器2的A相或B相引脚触发了中断
                Encoder_Update(ENCODER_2);                      // 更新编码器2的脉冲计数
            }                                                   // if结束

            if (pending & (DL_GPIO_PIN_10 | DL_GPIO_PIN_11)) {  // 若编码器3的A相或B相引脚触发了中断
                Encoder_Update(ENCODER_3);                      // 更新编码器3的脉冲计数
            }                                                   // if结束

            if (pending & (DL_GPIO_PIN_22 | DL_GPIO_PIN_21)) {  // 若编码器4的A相或B相引脚触发了中断
                Encoder_Update(ENCODER_4);                      // 更新编码器4的脉冲计数
            }                                                   // if结束

            DL_GPIO_clearInterruptStatus(GPIOB,                 // 清除GPIOB各引脚的中断标志，否则会反复进中断
                DL_GPIO_PIN_0 | DL_GPIO_PIN_16 | DL_GPIO_PIN_1 | DL_GPIO_PIN_4 |
                DL_GPIO_PIN_10 | DL_GPIO_PIN_11 | DL_GPIO_PIN_22 | DL_GPIO_PIN_21 |
                DL_GPIO_PIN_15);                                // 顺便清除PB15（按键）的标志
            break;                                              // GPIOB分支处理完毕，跳出switch

        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:                    // 是GPIOA触发的中断（按键引脚）
            /* Keys on GPIOA: only clear flags here to avoid interrupt storm */  // 按键只清标志不处理，防抖交给主循环，避免中断风暴
            DL_GPIO_clearInterruptStatus(GPIOA,                 // 清除GPIOA上3个按键引脚的中断标志
                DL_GPIO_PIN_16 | DL_GPIO_PIN_17 | DL_GPIO_PIN_18);
            break;                                              // GPIOA分支处理完毕，跳出switch

        default:                                                // 其他未预期的中断源
            break;                                              // 不做任何处理
    }                                                           // switch结束
}                                                               // GROUP1_IRQHandler函数结束
