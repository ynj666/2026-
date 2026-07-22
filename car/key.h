#ifndef __KEY_H__                 // 头文件保护：若 __KEY_H__ 未定义过则继续，防止重复包含导致重复定义
#define __KEY_H__                 // 定义 __KEY_H__ 宏，标记本头文件已被包含过

#include <stdint.h>               // 包含标准整数类型头文件，提供 uint8_t 等定长整数类型

typedef enum {                    // 定义按键编号枚举类型，作为 KEY_Read/KEY_Scan 等函数的参数
    KEY_1 = 0,                    // 按键1，编号 0，对应 key_config 表第 0 项（PA16）
    KEY_2,                        // 按键2，编号 1，对应 key_config 表第 1 项（PA17）
    KEY_3,                        // 按键3，编号 2，对应 key_config 表第 2 项（PA18）
    KEY_NUM                       // 按键总数（=3），用于数组大小和参数合法性判断
} KEY_ID;                         // 枚举类型命名为 KEY_ID

void KEY_Init(void);              // 函数声明：按键模块初始化，清零所有按键的状态和消抖计数
uint8_t KEY_Read(KEY_ID key);     // 函数声明：读取按键消抖后的稳定状态，返回 1=按下，0=松开（电平式查询）
uint8_t KEY_Scan(KEY_ID key);     // 函数声明：按键扫描（需每10ms调用一次），确认按下的那一拍返回 1，其余返回 0（事件式）
uint8_t KEY_GetRawLevels(void);   // 函数声明：调试用的原始电平读取，返回bit0/1/2对应KEY1/2/3的实时电平（1=高），空闲时应为7

#endif                            // 与开头的 #ifndef 配对，头文件保护块结束
