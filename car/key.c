#include "key.h"                  // 包含本模块自己的头文件，拿到 KEY_ID 枚举和函数声明
#include "ti_msp_dl_config.h"     // 包含 SysConfig 生成的配置头文件，拿到 GPIOA、DL_GPIO_PIN_x、DL_GPIO_readPins 等定义

#define KEY_PRESS_LEVEL     0     // 按键按下时的电平：0 表示低电平按下（按键一端接地，需上拉输入）
#define KEY_DEBOUNCE_COUNT  3    // 连续3次采样一致才确认，约30ms（KEY_Scan需每10ms调用一次）——消抖确认次数

typedef struct {                  // 定义按键的硬件配置结构体：描述一个按键接在哪个端口哪个引脚
    GPIO_Regs *port;              // 按键所在 GPIO 端口的寄存器基地址（如 GPIOA）
    uint32_t pin;                 // 按键引脚掩码（如 DL_GPIO_PIN_16）
} KEY_Config;                     // 结构体类型命名为 KEY_Config

/* 换引脚只改这里 */
static const KEY_Config key_config[KEY_NUM] = {   // 按键硬件配置表：每个元素对应一个按键，static const 表示只读且本文件内可见
    {GPIOA, DL_GPIO_PIN_16},   // KEY_1 接在 PA16
    {GPIOA, DL_GPIO_PIN_17},   // KEY_2 接在 PA17
    {GPIOA, DL_GPIO_PIN_18},   // KEY_3 接在 PA18
};                                // 配置表结束

static uint8_t key_state[KEY_NUM];   // 消抖后的稳定状态：1=按下，0=松开
static uint8_t key_count[KEY_NUM];   // 与稳定状态相反的电平连续计数

/* 读取实际电平：高=1，低=0 */
static uint8_t KEY_ReadLevel(KEY_ID key)   // 内部函数：直接读取按键引脚的实时电平（未消抖），返回 1=高电平，0=低电平
{
    if (key >= KEY_NUM) {         // 参数保护：按键编号非法
        return 1;                 // 返回 1（高电平=未按下），保证异常时不会误报按下
    }                             // if 参数检查结束

    return (DL_GPIO_readPins(key_config[key].port, key_config[key].pin) != 0) ? 1 : 0;   // 读该按键引脚电平：非0即高电平返回1，否则返回0
}                                 // KEY_ReadLevel 函数结束

void KEY_Init(void)               // 按键模块初始化函数：上电后调用一次
{
    uint8_t i;                    // 循环计数变量

    for (i = 0; i < KEY_NUM; i++) {   // 遍历所有按键
        key_state[i] = 0;         // 初始稳定状态设为 0（松开）
        key_count[i] = 0;         // 消抖计数清零
    }                             // for 循环结束
}                                 // KEY_Init 函数结束

/* 消抖后的当前状态：按下=1，松开=0（随KEY_Scan周期更新） */
uint8_t KEY_Read(KEY_ID key)      // 读取按键消抖后的稳定状态：返回 1=按下，0=松开（电平式查询）
{
    if (key >= KEY_NUM) {         // 参数保护：按键编号非法
        return 0;                 // 返回 0（未按下），保证异常时不会误触发
    }                             // if 参数检查结束

    return key_state[key];        // 返回该按键消抖后的稳定状态
}                                 // KEY_Read 函数结束

/* 每10ms调用一次：非阻塞消抖，确认按下一次只返回一次1 */
uint8_t KEY_Scan(KEY_ID key)      // 按键扫描函数（事件式）：需每 10ms 周期调用，确认按下的那一拍返回 1，其余返回 0
{
    uint8_t raw;                  // 本次采样到的逻辑状态：1=按下，0=松开（已根据 KEY_PRESS_LEVEL 换算）
    uint8_t event = 0;            // 按键事件标志：本拍是否新确认了一次按下，默认无事件

    if (key >= KEY_NUM) {         // 参数保护：按键编号非法
        return 0;                 // 返回 0（无事件）
    }                             // if 参数检查结束

    raw = (KEY_ReadLevel(key) == KEY_PRESS_LEVEL) ? 1 : 0;   // 读实时电平并换算成逻辑状态：电平等于按下电平则 raw=1（按下）

    if (raw != key_state[key]) {  // 当前采样与稳定状态不一致：可能正在抖动或状态真的变了
        key_count[key]++;         // 不一致计数加 1
        if (key_count[key] >= KEY_DEBOUNCE_COUNT) {   // 连续不一致达到消抖次数（约30ms），认为状态确实改变
            key_count[key] = 0;   // 消抖计数清零，为下次消抖做准备
            key_state[key] = raw; // 更新稳定状态为新采样值
            if (raw == 1) {       // 若新状态是"按下"
                event = 1;   // 只在"确认按下"的这一拍返回1
            }                     // if 按下判断结束
        }                         // if 消抖确认结束
    } else {                      // 当前采样与稳定状态一致：没有变化
        key_count[key] = 0;       // 消抖计数清零（抖动被滤除）
    }                             // if-else 状态比较结束

    return event;                 // 返回事件：确认按下返回 1，否则返回 0
}                                 // KEY_Scan 函数结束

/* 调试用：返回3个按键引脚的实时电平，bit0/bit1/bit2对应KEY1/2/3，1=高电平，0=低电平 */
uint8_t KEY_GetRawLevels(void)    // 读取全部按键的原始电平并打包：用于判断按键实际是"按下为低"还是"按下为高"
{
    uint8_t levels = 0;           // 电平打包结果，初始为0（3位都为低）
    uint8_t i;                    // 循环计数变量

    for (i = 0; i < KEY_NUM; i++) {   // 遍历3个按键
        if (KEY_ReadLevel((KEY_ID)i)) {   // 若该按键引脚当前为高电平
            levels |= (uint8_t)(1U << i); // 把结果中对应的位置1
        }                             // if 电平判断结束
    }                                 // for 循环结束

    return levels;                // 返回电平掩码：3个键都松开（上拉高电平）时应为 0b111 即 7
}                                 // KEY_GetRawLevels 函数结束
