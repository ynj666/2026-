#include "motor.h"                 // 包含本模块自己的头文件，拿到 MOTOR_ID 枚举和函数声明
#include "ti_msp_dl_config.h"      // 包含 SysConfig 生成的配置头文件，拿到 GPIOA、DL_GPIO_PIN_x、PWM_0_INST/PWM_1_INST 等定义

typedef struct {                  // 定义一个电机的硬件配置结构体，把一路电机用到的所有引脚/通道集中描述
    GPIO_Regs *in1Port;           // 方向控制脚 IN1 所在的 GPIO 端口寄存器基地址（如 GPIOA）
    uint32_t in1Pin;              // 方向控制脚 IN1 的引脚掩码（如 DL_GPIO_PIN_10）
    GPIO_Regs *in2Port;           // 方向控制脚 IN2 所在的 GPIO 端口寄存器基地址
    uint32_t in2Pin;              // 方向控制脚 IN2 的引脚掩码
    GPTIMER_Regs *pwmInst;        // 该电机 PWM 所用的定时器实例（PWM_0_INST 或 PWM_1_INST）
    DL_TIMER_CC_INDEX pwmCh;      // 该电机占空比对应的定时器捕获/比较通道（决定 PWMA/PWMB 输出）
    uint16_t pwmPeriod;           // 该定时器的 PWM 周期计数值（PWM_0=4000，PWM_1=1000，由 SysConfig 配置决定）
    int8_t dir;                   // 方向修正系数：+1 或 -1，用于接线反了时在软件里把方向翻回来
} Motor_Config;                   // 结构体类型命名为 Motor_Config

/* 换引脚、反方向只改这里 */
static const Motor_Config motor_cfg[MOTOR_NUM] = {   // 电机硬件配置表：每个元素对应一路电机，static const 表示只读且本文件内可见
    {GPIOA, DL_GPIO_PIN_10, GPIOA, DL_GPIO_PIN_11, PWM_0_INST, GPIO_PWM_0_C0_IDX, 4000, 1}, // 电机A：AIN1=PA10，AIN2=PA11，PWMA 用 PWM0 通道0，方向为正（1：正PWM=正转=编码器正计数）
    {GPIOA, DL_GPIO_PIN_28, GPIOA, DL_GPIO_PIN_31, PWM_0_INST, GPIO_PWM_0_C1_IDX, 4000, -1}, // 电机B：BIN1=PA28，BIN2=PA31，PWMB 用 PWM0 通道1，方向取反（-1）
    {GPIOB, DL_GPIO_PIN_23, GPIOA, DL_GPIO_PIN_23, PWM_1_INST, GPIO_PWM_1_C0_IDX, 1000, -1}, // 电机C：CIN_1=PB23，CIN_2=PA23，PWM 用 PWM1 通道0，方向取反（-1），反了改这里
    {GPIOA, DL_GPIO_PIN_21, GPIOA, DL_GPIO_PIN_29, PWM_1_INST, GPIO_PWM_1_C1_IDX, 1000, -1}, // 电机D：DIN_1=PA21，DIN_2=PA29，PWM 用 PWM1 通道1，方向取反（-1），反了改这里
};                                 // 配置表结束

static uint16_t Motor_AbsLimit(int16_t speed)   // 内部辅助函数：把速度限制在 ±1000 内并返回其绝对值（本文件内使用）
{
    if (speed > 1000) speed = 1000;    // 正向超过上限 1000 则截断为 1000
    if (speed < -1000) speed = -1000;  // 反向超过下限 -1000 则截断为 -1000
    return (speed >= 0) ? speed : -speed;   // 返回速度的绝对值，作为占空比计算的基准（0~1000）
}                                // Motor_AbsLimit 函数结束

void Motor_Init(void)            // 电机模块初始化函数：上电后调用一次
{
    Motor_StopAll();             // 先让四路电机全部停止（占空比0、方向脚都拉低），避免上电瞬间乱转
    DL_Timer_startCounter(PWM_0_INST);   // 启动 PWM0 定时器计数，PWMA/PWMB 引脚开始输出 PWM 波形
    DL_Timer_startCounter(PWM_1_INST);   // 启动 PWM1 定时器计数（SysConfig 里 PWM_1 默认不自动启动），电机C/D 的 PWM 开始输出
}                                // Motor_Init 函数结束

void Motor_Set(MOTOR_ID id, int16_t speed)   // 设置某路电机速度：id 选电机，speed 范围 -1000~1000，正负代表方向
{
    uint32_t compare;            // 局部变量：存放计算出的 PWM 比较值（决定占空比）

    if (id >= MOTOR_NUM) return; // 参数保护：电机编号非法则直接返回，不操作硬件

    compare = (uint32_t)Motor_AbsLimit(speed) * motor_cfg[id].pwmPeriod / 1000U;   // 把 0~1000 的速度绝对值按该定时器的周期换算成比较值
    DL_Timer_setCaptureCompareValue(motor_cfg[id].pwmInst, compare, motor_cfg[id].pwmCh);   // 写入对应定时器通道的比较寄存器，设置该电机 PWM 占空比（即速度大小）

    speed *= motor_cfg[id].dir;  // 乘上方向修正系数：若该电机接线反了（dir=-1），这里把逻辑方向翻正

    if (speed > 0) {             // 速度为正：设置成正转
        DL_GPIO_setPins(motor_cfg[id].in1Port, motor_cfg[id].in1Pin);     // IN1 输出高电平
        DL_GPIO_clearPins(motor_cfg[id].in2Port, motor_cfg[id].in2Pin);   // IN2 输出低电平，H桥驱动电机正转
    } else if (speed < 0) {      // 速度为负：设置成反转
        DL_GPIO_clearPins(motor_cfg[id].in1Port, motor_cfg[id].in1Pin);   // IN1 输出低电平
        DL_GPIO_setPins(motor_cfg[id].in2Port, motor_cfg[id].in2Pin);     // IN2 输出高电平，H桥驱动电机反转
    } else {                     // 速度为 0：停止
        DL_GPIO_clearPins(motor_cfg[id].in1Port, motor_cfg[id].in1Pin);   // IN1 拉低
        DL_GPIO_clearPins(motor_cfg[id].in2Port, motor_cfg[id].in2Pin);   // IN2 也拉低，两个方向脚都为低，电机滑行停止（配合占空比0）
    }                            // if-else 方向选择结束
}                                // Motor_Set 函数结束

void Motor_StopAll(void)         // 停止所有电机的便捷函数
{
    uint8_t i;                   // 循环计数变量

    for (i = 0; i < MOTOR_NUM; i++) {   // 遍历全部四路电机
        Motor_Set((MOTOR_ID)i, 0);      // 速度设 0：占空比清零且方向脚都拉低
    }                            // for 循环结束
}                                // Motor_StopAll 函数结束
