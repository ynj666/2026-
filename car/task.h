#ifndef __TASK_H__                // 头文件保护：若 __TASK_H__ 未定义过则继续，防止重复包含导致重复定义
#define __TASK_H__                // 定义 __TASK_H__ 宏，标记本头文件已被包含过

#include <stdint.h>               // 包含标准整数类型头文件，提供 uint8_t 等定长整数类型

#define TASK_NUM    5             // 任务总数：当前最多5个任务，编号1~5

void Task_Init(void);             // 函数声明：任务模块初始化，当前任务号置为初始值5，暂停状态
uint8_t Task_GetCurrent(void);    // 函数声明：读取当前选中的任务号（1~5），用于OLED/串口/VOFA显示
uint8_t Task_GetRunning(void);    // 函数声明：读取任务运行状态，1=执行中，0=暂停
void Task_SwitchNext(void);       // 函数声明：切换任务（按键1调用），任务号加1，超过5回到1
void Task_ToggleRun(void);        // 函数声明：开始/暂停切换（按键2调用），奇数次按下开始，偶数次按下暂停
void Task_Tick(void);             // 函数声明：任务状态巡检（主循环周期调用），任务完成时自动回到暂停态
void Task_SoftReset(void);        // 函数声明：软件归零复位（按键3调用），整个系统回到刚上电复位时的状态

#endif                            // 与开头的 #ifndef 配对，头文件保护块结束
