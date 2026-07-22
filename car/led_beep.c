#include "led_beep.h"  // 包含本模块头文件，获得LED和蜂鸣器控制函数声明
#include "ti_msp_dl_config.h"  // 包含SysConfig生成的配置头文件，提供GPIOB、DL_GPIO_PIN_x等定义

#define LED_PORT    GPIOB  // LED连接的GPIO端口：GPIOB
#define LED_PIN     DL_GPIO_PIN_14  // LED连接的引脚：PB14

#define BEEP_PORT   GPIOB  // 蜂鸣器连接的GPIO端口：GPIOB
#define BEEP_PIN    DL_GPIO_PIN_12  // 蜂鸣器连接的引脚：PB12

void LED_BEEP_Init(void)  // LED与蜂鸣器初始化（引脚方向由SysConfig配置，这里只设初始电平）
{
    LED_Off();  // 上电默认熄灭LED
    BEEP_Off();  // 上电默认关闭蜂鸣器
}

void LED_On(void)  // 点亮LED
{
    DL_GPIO_setPins(LED_PORT, LED_PIN);  // PB14输出高电平，LED点亮
}

void LED_Off(void)  // 熄灭LED
{
    DL_GPIO_clearPins(LED_PORT, LED_PIN);  // PB14输出低电平，LED熄灭
}

void LED_Toggle(void)  // 翻转LED状态（亮变灭、灭变亮）
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);  // 翻转PB14输出电平
}

void BEEP_On(void)  // 打开蜂鸣器
{
    DL_GPIO_clearPins(BEEP_PORT, BEEP_PIN);  // PB12输出低电平，蜂鸣器鸣响（蜂鸣器为低电平有效）
}

void BEEP_Off(void)  // 关闭蜂鸣器
{
    DL_GPIO_setPins(BEEP_PORT, BEEP_PIN);  // PB12输出高电平，蜂鸣器停止鸣响
}

void BEEP_Toggle(void)  // 翻转蜂鸣器状态（响变停、停变响）
{
    DL_GPIO_togglePins(BEEP_PORT, BEEP_PIN);  // 翻转PB12输出电平
}