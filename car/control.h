#ifndef __CONTROL_H__         // 头文件保护：防止重复包含（若未定义__CONTROL_H__则继续）
#define __CONTROL_H__         // 定义宏__CONTROL_H__，标记本头文件已被包含

#include <stdint.h>           // 引入标准整数类型定义（uint8_t、int16_t、int32_t等）

/* 四轮闭环映射：左侧=电机A+电机C（编码器1+3），右侧=电机B+电机D（编码器2+4），
   同侧两轮共用一个PID输出，速度/里程取同侧两轮的平均值 */

/* 控制周期 = TIMER_0 中断周期 = 10ms */
#define CONTROL_DT_MS        10U    // 控制周期10毫秒：Control_Tick每10ms被调用一次
/* 电机输出限幅（与 Motor_Set 输入范围一致：-1000 ~ +1000） */
#define CONTROL_MOTOR_MAX    1000   // PID输出限幅值：对应电机PWM满量程1000

/* 初始化：编码器、电机、PID默认参数（在main中调用一次） */
void Control_Init(void);            // 控制模块初始化

/* 控制节拍：必须在 TIMER_0 中断中每10ms调用一次
 * 完成：四路编码器测速、左右侧里程累计、左右侧速度闭环PID输出 */
void Control_Tick(void);            // 控制节拍：每10ms执行一次闭环计算

/* 闭环使能：0=开环(电机保持当前状态)，1=速度闭环 */
void Control_Enable(uint8_t en);    // 使能/关闭速度闭环

/* 设置左右侧目标速度，单位：编码器脉冲数/10ms（每轮） */
void Control_SetTarget(int16_t left, int16_t right);    // 设置左右侧目标速度

/* 当前左右侧目标速度，调速度环时用于观察目标/实测/误差 */
int16_t Control_GetTargetLeft(void);
int16_t Control_GetTargetRight(void);

/* 整定速度环PID参数（左右侧共用一组） */
void Control_SetPID(float kp, float ki, float kd);      // 设置速度环PID三个参数

/* 整定位置环PID参数（左右侧共用一组），位置环输出作为速度环目标 */
void Control_SetPosPID(float kp, float ki, float kd);   // 设置位置环PID三个参数

/* 启动位置+速度双闭环：里程清零后走到指定脉冲数自动停车（左右侧给相同值即走直线） */
void Control_StartPosition(int32_t left_pulses, int32_t right_pulses);  // 启动位置闭环

/* 清零左右累计里程，比赛导航分段时调用 */
void Control_ResetMileage(void);

/* 位置闭环到位查询：1=两侧都到达目标位置并已停车 */
uint8_t Control_PositionDone(void); // 查询位置闭环是否完成

/* 最近一个10ms周期的实测速度，单位：脉冲数/10ms（同侧两轮平均） */
int32_t Control_GetSpeedLeft(void);     // 获取左侧实测速度
int32_t Control_GetSpeedRight(void);    // 获取右侧实测速度

/* 最近一次输出到电机的控制量，范围 -1000 ~ +1000，可当作PWM指令查看 */
int16_t Control_GetOutputLeft(void);
int16_t Control_GetOutputRight(void);

/* 累计里程，单位：脉冲数（同侧两轮平均） */
int32_t Control_GetMileageLeft(void);   // 获取左侧累计里程
int32_t Control_GetMileageRight(void);  // 获取右侧累计里程

#endif                        // 头文件保护结束（与开头的#ifndef配对）
