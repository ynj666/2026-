#include "control.h"          // 引入控制模块自身的头文件（函数声明、宏定义）
#include "motor.h"            // 引入电机驱动模块（Motor_Init、Motor_Set等）
#include "encoder.h"          // 引入编码器模块（Encoder_Get、Encoder_Clear等）

#define CONTROL_HAS_4WD     0         // 是否装了4个电机/编码器：1=四驱（左右侧各取两轮平均），0=只有A/B两轮（左侧=编码器1，右侧=编码器2）

/* PID默认参数：只保证能跑起来，必须按实际车况整定 */
#define PID_KP_DEFAULT      10.0f     // 速度环比例系数默认值：误差越大输出越大，决定响应速度
#define PID_KI_DEFAULT      1.0f      // 速度环积分系数默认值：消除静差，累积历史误差
#define PID_KD_DEFAULT      0.0f      // 速度环微分系数默认值：抑制超调振荡，默认不用
#define PID_INTEGRAL_MAX    500.0f    // 积分限幅值：防止积分项无限累积导致饱和失控

/* 位置环（外环）默认参数：输出为速度目标，必须按实际车况整定 */
#define POS_KP_DEFAULT      2.0f      // 位置环比例系数默认值：位置误差 × kp = 目标速度
#define POS_KI_DEFAULT      0.1f      // 位置环积分系数默认值：消除末端静差，保证走到目标位置
#define POS_KD_DEFAULT      0.0f      // 位置环微分系数默认值：默认不用
#define POS_SPEED_MAX       25       // 位置环输出的速度目标限幅（脉冲数/10ms），防止起步冲太猛
#define POS_DONE_TOL        200        // 到位判定容差（脉冲数）：两侧位置误差都在此范围内认为已到达

/* 控制模式 */
#define CTRL_MODE_OPEN      0         // 开环：闭环不输出，电机保持当前状态
#define CTRL_MODE_SPEED     1         // 速度闭环：只有内环（速度环）
#define CTRL_MODE_POSITION  2         // 位置+速度双闭环：位置环输出作为速度环目标

typedef struct {            // 定义PID控制器状态结构体
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float integral;         // 误差积分累积值
    float prevErr;          // 上一次的误差（用于计算微分项）
} PidState;                 // 结构体类型命名为PidState

static volatile uint8_t  ctrl_mode = CTRL_MODE_OPEN;   // 控制模式：开环/速度闭环/位置+速度双闭环（volatile：中断和主循环都会访问）
static volatile int16_t  target_left = 0;   // 左侧速度目标（脉冲数/10ms），位置模式下由位置环算出，左侧=电机A+电机C
static volatile int16_t  target_right = 0;  // 右侧速度目标（脉冲数/10ms），右侧=电机B+电机D

static volatile int32_t  speed_left = 0;    // 左侧实测速度：编码器1和3最近一个10ms周期脉冲数的平均值
static volatile int32_t  speed_right = 0;   // 右侧实测速度：编码器2和4最近一个10ms周期脉冲数的平均值
static volatile int32_t  mileage_left = 0;  // 左侧累计里程（脉冲总数，取编码器1和3的平均）
static volatile int32_t  mileage_right = 0; // 右侧累计里程（脉冲总数，取编码器2和4的平均）

static volatile int32_t  pos_target_left = 0;   // 左侧位置目标（脉冲数）：位置模式下里程要走到这里
static volatile int32_t  pos_target_right = 0;  // 右侧位置目标（脉冲数）
static volatile uint8_t  pos_done = 0;          // 位置到位标志：1=两侧都到达目标位置并已停车

static PidState pid_left;                   // 左侧速度环PID控制器状态
static PidState pid_right;                  // 右侧速度环PID控制器状态
static PidState pid_pos_left;               // 左侧位置环PID控制器状态（外环）
static PidState pid_pos_right;              // 右侧位置环PID控制器状态（外环）

static int16_t PID_Update(PidState *pid, int32_t target, int32_t measured)  // PID计算：输入目标值和实测值，输出电机PWM量
{                               // 函数体开始
    float err = (float)target - (float)measured;  // 计算本次误差 = 目标值 - 实测值
    float out;                  // PID输出值（浮点，最后再转成整型）

    /* 积分限幅，防止饱和 */
    pid->integral += err;       // 误差累加到积分项
    if (pid->integral > PID_INTEGRAL_MAX)  pid->integral = PID_INTEGRAL_MAX;    // 积分超过上限则钳位到上限
    if (pid->integral < -PID_INTEGRAL_MAX) pid->integral = -PID_INTEGRAL_MAX;   // 积分低于下限则钳位到下限

    out = pid->kp * err                     // 比例项：kp × 当前误差
        + pid->ki * pid->integral           // 积分项：ki × 误差累积
        + pid->kd * (err - pid->prevErr);   // 微分项：kd × 误差变化量
    pid->prevErr = err;                     // 保存本次误差，供下一周期计算微分

    if (out > CONTROL_MOTOR_MAX)  out = CONTROL_MOTOR_MAX;    // 输出超过+1000则限幅（电机PWM上限）
    if (out < -CONTROL_MOTOR_MAX) out = -CONTROL_MOTOR_MAX;   // 输出低于-1000则限幅（电机PWM下限）

    return (int16_t)out;        // 浮点输出转成int16_t返回
}                               // PID_Update函数结束

void Control_Init(void)         // 控制模块初始化：初始化编码器、电机并设置默认PID参数
{                               // 函数体开始
    Encoder_Init();             // 初始化编码器（清零计数、使能GPIOB中断）
    Motor_Init();               // 初始化电机（PWM和方向引脚）
    Control_SetPID(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);  // 设置速度环默认PID参数（左右侧共用）
    Control_SetPosPID(POS_KP_DEFAULT, POS_KI_DEFAULT, POS_KD_DEFAULT);  // 设置位置环默认PID参数（左右侧共用）
}                               // Control_Init函数结束

void Control_Tick(void)         // 控制节拍函数：每10ms在定时器中断中调用一次
{                               // 函数体开始
    int32_t s1;                 // 编码器1本周期脉冲数（左前/电机A轮）
    int32_t s2;                 // 编码器2本周期脉冲数（右前/电机B轮）
#if CONTROL_HAS_4WD             // 只有四驱模式才需要读编码器3/4
    int32_t s3;                 // 编码器3本周期脉冲数（左后/电机C轮）
    int32_t s4;                 // 编码器4本周期脉冲数（右后/电机D轮）
#endif                          // CONTROL_HAS_4WD 声明结束
    int32_t sl;                 // 左侧本周期脉冲数
    int32_t sr;                 // 右侧本周期脉冲数
    int16_t out_l;              // 左侧PID输出（同时给电机A和电机C）
    int16_t out_r;              // 右侧PID输出（同时给电机B和电机D）

    /* 测速：读取编码器本周期脉冲数并清零 */
    s1 = Encoder_Get(ENCODER_1);        // 读取编码器1这10ms内累计的脉冲数
    s2 = Encoder_Get(ENCODER_2);        // 读取编码器2这10ms内累计的脉冲数
    Encoder_Clear(ENCODER_1);           // 清零编码器1计数，为下一周期测速做准备
    Encoder_Clear(ENCODER_2);           // 清零编码器2计数，为下一周期测速做准备
#if CONTROL_HAS_4WD             // 四驱模式才读写编码器3/4
    s3 = Encoder_Get(ENCODER_3);        // 读取编码器3这10ms内累计的脉冲数
    s4 = Encoder_Get(ENCODER_4);        // 读取编码器4这10ms内累计的脉冲数
    Encoder_Clear(ENCODER_3);           // 清零编码器3计数，为下一周期测速做准备
    Encoder_Clear(ENCODER_4);           // 清零编码器4计数，为下一周期测速做准备
#endif                          // CONTROL_HAS_4WD 读取结束

    /* 左右侧速度：四驱时取同侧两轮平均，只有A/B两轮时直接用编码器1/2
       （C/D没装时编码器3/4读数恒为0，算平均会把反馈减半，导致闭环拼命加速） */
#if CONTROL_HAS_4WD               // 四驱：同侧两轮平均
    sl = (s1 + s3) / 2;         // 左侧平均速度（电机A轮和电机C轮）
    sr = (s2 + s4) / 2;         // 右侧平均速度（电机B轮和电机D轮）
#else                             // 两轮：只用电机A/B对应的编码器1/2
    sl = s1;                    // 左侧速度 = 编码器1
    sr = s2;                    // 右侧速度 = 编码器2
#endif                            // CONTROL_HAS_4WD 分支结束

    speed_left  = sl;           // 保存左侧实测速度（脉冲数/10ms）
    speed_right = sr;           // 保存右侧实测速度（脉冲数/10ms）
    mileage_left  += sl;        // 左侧里程累加本周期的脉冲数
    mileage_right += sr;        // 右侧里程累加本周期的脉冲数

    if (ctrl_mode == CTRL_MODE_POSITION) {  // 位置+速度双闭环：先跑外环（位置环）
        int32_t err_l = pos_target_left  - mileage_left;    // 左侧位置误差（脉冲数）
        int32_t err_r = pos_target_right - mileage_right;   // 右侧位置误差（脉冲数）
        int16_t spd;            // 位置环算出的速度目标（限幅后）

        if (err_l > -POS_DONE_TOL && err_l < POS_DONE_TOL &&
            err_r > -POS_DONE_TOL && err_r < POS_DONE_TOL) {    // 两侧位置误差都在容差内：已到达
            pos_done = 1;                   // 置位到位标志，供任务层查询
            ctrl_mode = CTRL_MODE_OPEN;     // 退出闭环
            Motor_StopAll();                // 四轮全部停车
        } else {                            // 未到达：位置环输出作为速度环目标
            spd = PID_Update(&pid_pos_left, pos_target_left, mileage_left);     // 左侧位置环计算
            if (spd > POS_SPEED_MAX)  spd = POS_SPEED_MAX;    // 速度目标上限幅
            if (spd < -POS_SPEED_MAX) spd = -POS_SPEED_MAX;   // 速度目标下限幅
            target_left = spd;              // 作为左侧速度环的新目标

            spd = PID_Update(&pid_pos_right, pos_target_right, mileage_right);  // 右侧位置环计算
            if (spd > POS_SPEED_MAX)  spd = POS_SPEED_MAX;    // 速度目标上限幅
            if (spd < -POS_SPEED_MAX) spd = -POS_SPEED_MAX;   // 速度目标下限幅
            target_right = spd;             // 作为右侧速度环的新目标
        }                                   // if-else 到位判断结束
    }                                       // if 位置环结束

    if (ctrl_mode != CTRL_MODE_OPEN) {      // 速度闭环或位置+速度双闭环：都要跑内环（速度环）
        out_l = PID_Update(&pid_left,  target_left,  sl);   // 左侧速度环PID计算
        out_r = PID_Update(&pid_right, target_right, sr);   // 右侧速度环PID计算
        Motor_Set(MOTOR_A, out_l);      // 左前电机A输出左侧PWM
        Motor_Set(MOTOR_C, out_l);      // 左后电机C输出同样的左侧PWM
        Motor_Set(MOTOR_B, out_r);      // 右前电机B输出右侧PWM
        Motor_Set(MOTOR_D, out_r);      // 右后电机D输出同样的右侧PWM
    }                           // if结束
}                               // Control_Tick函数结束

void Control_Enable(uint8_t en) // 使能/关闭速度闭环控制
{                               // 函数体开始
    ctrl_mode = (en != 0) ? CTRL_MODE_SPEED : CTRL_MODE_OPEN;   // 非0进速度闭环，0回开环

    if (ctrl_mode == CTRL_MODE_OPEN) {  // 若是关闭闭环
        /* 退出闭环时清积分，避免下次使能时冲击 */
        pid_left.integral = 0.0f;   // 清左侧速度环积分累积，防止重新使能时输出突变
        pid_left.prevErr = 0.0f;    // 清左侧速度环上次误差，避免微分项计算错误
        pid_right.integral = 0.0f;  // 清右侧速度环积分累积，防止重新使能时输出突变
        pid_right.prevErr = 0.0f;   // 清右侧速度环上次误差，避免微分项计算错误
    }                           // if结束
}                               // Control_Enable函数结束

void Control_SetTarget(int16_t left, int16_t right)  // 设置左右侧目标速度（单位：脉冲数/10ms，每轮）
{                               // 函数体开始
    target_left = left;         // 更新左侧速度目标
    target_right = right;       // 更新右侧速度目标
}                               // Control_SetTarget函数结束

void Control_SetPID(float kp, float ki, float kd)    // 整定速度环PID参数（左右侧共用一组）
{                               // 函数体开始
    pid_left.kp = kp;           // 设置左侧速度环比例系数
    pid_left.ki = ki;           // 设置左侧速度环积分系数
    pid_left.kd = kd;           // 设置左侧速度环微分系数

    pid_right.kp = kp;          // 设置右侧速度环比例系数
    pid_right.ki = ki;          // 设置右侧速度环积分系数
    pid_right.kd = kd;          // 设置右侧速度环微分系数
}                               // Control_SetPID函数结束

void Control_SetPosPID(float kp, float ki, float kd) // 整定位置环PID参数（左右侧共用一组）
{                               // 函数体开始
    pid_pos_left.kp = kp;       // 设置左侧位置环比例系数
    pid_pos_left.ki = ki;       // 设置左侧位置环积分系数
    pid_pos_left.kd = kd;       // 设置左侧位置环微分系数

    pid_pos_right.kp = kp;      // 设置右侧位置环比例系数
    pid_pos_right.ki = ki;      // 设置右侧位置环积分系数
    pid_pos_right.kd = kd;      // 设置右侧位置环微分系数
}                               // Control_SetPosPID函数结束

void Control_StartPosition(int32_t left_pulses, int32_t right_pulses)  // 启动位置+速度双闭环：走完指定脉冲数后自动停车
{                               // 函数体开始
    mileage_left = 0;           // 左侧里程清零，从当前位置开始计数
    mileage_right = 0;          // 右侧里程清零
    pos_target_left = left_pulses;      // 设置左侧位置目标
    pos_target_right = right_pulses;    // 设置右侧位置目标
    pos_done = 0;               // 清到位标志

    /* 清四个环的积分和上次误差，避免带着历史状态起步 */
    pid_pos_left.integral = 0.0f;   // 清左侧位置环积分
    pid_pos_left.prevErr = 0.0f;    // 清左侧位置环上次误差
    pid_pos_right.integral = 0.0f;  // 清右侧位置环积分
    pid_pos_right.prevErr = 0.0f;   // 清右侧位置环上次误差
    pid_left.integral = 0.0f;       // 清左侧速度环积分
    pid_left.prevErr = 0.0f;        // 清左侧速度环上次误差
    pid_right.integral = 0.0f;      // 清右侧速度环积分
    pid_right.prevErr = 0.0f;       // 清右侧速度环上次误差

    ctrl_mode = CTRL_MODE_POSITION; // 进入位置+速度双闭环模式
}                               // Control_StartPosition函数结束

uint8_t Control_PositionDone(void)   // 查询位置闭环是否已到位停车
{                               // 函数体开始
    return pos_done;            // 1=两侧都到达目标位置并已停车
}                               // Control_PositionDone函数结束

int32_t Control_GetSpeedLeft(void)   // 获取左侧最近一个10ms周期的实测速度
{                               // 函数体开始
    return speed_left;          // 返回左侧速度（脉冲数/10ms，同侧两轮平均）
}                               // Control_GetSpeedLeft函数结束

int32_t Control_GetSpeedRight(void)  // 获取右侧最近一个10ms周期的实测速度
{                               // 函数体开始
    return speed_right;         // 返回右侧速度（脉冲数/10ms，同侧两轮平均）
}                               // Control_GetSpeedRight函数结束

int32_t Control_GetMileageLeft(void) // 获取左侧累计里程
{                               // 函数体开始
    return mileage_left;        // 返回左侧累计脉冲数（同侧两轮平均）
}                               // Control_GetMileageLeft函数结束

int32_t Control_GetMileageRight(void)    // 获取右侧累计里程
{                               // 函数体开始
    return mileage_right;       // 返回右侧累计脉冲数（同侧两轮平均）
}                               // Control_GetMileageRight函数结束
