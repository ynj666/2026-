#ifndef __MOTOR_H__               // 头文件保护：若 __MOTOR_H__ 未定义过则继续，防止重复包含导致重复定义
#define __MOTOR_H__               // 定义 __MOTOR_H__ 宏，标记本头文件已被包含过

#include <stdint.h>               // 包含标准整数类型头文件，提供 int16_t、uint8_t 等定长整数类型

typedef enum {                    // 定义电机编号枚举类型，作为 Motor_Set 等函数的 id 参数
    MOTOR_A = 0,                  // 电机A，编号 0，对应 motor_cfg 表第 0 项
    MOTOR_B,                      // 电机B，编号 1，对应 motor_cfg 表第 1 项
    MOTOR_C,                      // 电机C，编号 2，对应 motor_cfg 表第 2 项（PWM_1 通道0）
    MOTOR_D,                      // 电机D，编号 3，对应 motor_cfg 表第 3 项（PWM_1 通道1）
    MOTOR_NUM                     // 电机总数（=4），用于数组大小和参数合法性判断
} MOTOR_ID;                       // 枚举类型命名为 MOTOR_ID

void Motor_Init(void);            // 函数声明：电机模块初始化，停车并启动 PWM 定时器
void Motor_Set(MOTOR_ID id, int16_t speed);   // 函数声明：设置指定电机速度，speed 范围 -1000~1000，正负代表方向
void Motor_StopAll(void);         // 函数声明：停止所有电机（四路速度都设为 0）

#endif                            // 与开头的 #ifndef 配对，头文件保护块结束
