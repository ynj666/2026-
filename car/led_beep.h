#ifndef __LED_BEEP_H__  // 头文件保护：防止本头文件被重复包含
#define __LED_BEEP_H__  // 定义保护宏，再次包含时跳过全部内容

void LED_BEEP_Init(void);  // 声明：LED与蜂鸣器初始化（默认LED灭、蜂鸣器关）

void LED_On(void);  // 声明：点亮LED
void LED_Off(void);  // 声明：熄灭LED
void LED_Toggle(void);  // 声明：翻转LED状态

void BEEP_On(void);  // 声明：打开蜂鸣器
void BEEP_Off(void);  // 声明：关闭蜂鸣器
void BEEP_Toggle(void);  // 声明：翻转蜂鸣器状态

#endif  // 头文件保护结束，对应开头的#ifndef __LED_BEEP_H__