#include "control.h"          // 引入控制模块自身的头文件（函数声明、宏定义）
#include "motor.h"            // 引入电机驱动模块（Motor_Init、Motor_Set等）
#include "encoder.h"          // 引入编码器模块（Encoder_Get、Encoder_Clear等）
#include "pid.h"

#define CONTROL_HAS_4WD     0         // 是否装了4个电机/编码器：1=四驱（左右侧各取两轮平均），0=只有A/B两轮（左侧=编码器1，右侧=编码器2）

/* 电机左右映射：如果实际左/右轮接反，只改这里，不动 PID 和编码器逻辑。 */
#define CONTROL_LEFT_FRONT_MOTOR     MOTOR_B
#define CONTROL_RIGHT_FRONT_MOTOR    MOTOR_A
#define CONTROL_LEFT_REAR_MOTOR      MOTOR_C
#define CONTROL_RIGHT_REAR_MOTOR     MOTOR_D

/* 位置环（外环）默认参数：输出为速度目标，必须按实际车况整定 */
#define POS_SPEED_MAX       25       // 位置环输出的速度目标限幅（脉冲数/10ms），防止起步冲太猛
#define POS_DONE_TOL        200        // 到位判定容差（脉冲数）：两侧位置误差都在此范围内认为已到达

/* 控制模式 */
#define CTRL_MODE_OPEN      0         // 开环：闭环不输出，电机保持当前状态
#define CTRL_MODE_SPEED     1         // 速度闭环：只有内环（速度环）
#define CTRL_MODE_POSITION  2         // 位置+速度双闭环：位置环输出作为速度环目标

static volatile uint8_t  ctrl_mode = CTRL_MODE_OPEN;   // 控制模式：开环/速度闭环/位置+速度双闭环（volatile：中断和主循环都会访问）
static volatile int16_t  target_left = 0;   // 左侧速度目标（脉冲数/10ms），位置模式下由位置环算出，左侧=电机A+电机C
static volatile int16_t  target_right = 0;  // 右侧速度目标（脉冲数/10ms），右侧=电机B+电机D
static volatile int16_t  output_left = 0;   // 左侧最近一次电机输出，范围 -1000~+1000
static volatile int16_t  output_right = 0;  // 右侧最近一次电机输出，范围 -1000~+1000

static volatile int32_t  speed_left = 0;    // 左侧实测速度：编码器1和3最近一个10ms周期脉冲数的平均值
static volatile int32_t  speed_right = 0;   // 右侧实测速度：编码器2和4最近一个10ms周期脉冲数的平均值
static volatile int32_t  mileage_left = 0;  // 左侧累计里程（脉冲总数，取编码器1和3的平均）
static volatile int32_t  mileage_right = 0; // 右侧累计里程（脉冲总数，取编码器2和4的平均）

static volatile int32_t  pos_target_left = 0;   // 左侧位置目标（脉冲数）：位置模式下里程要走到这里
static volatile int32_t  pos_target_right = 0;  // 右侧位置目标（脉冲数）
static volatile uint8_t  pos_done = 0;          // 位置到位标志：1=两侧都到达目标位置并已停车

/* PID 参数直接在这里调：
 * speed_left/right 是左右速度内环，pos_left/right 是位置外环。
 */
static PID_TypeDef pid_speed_left = {
    .kp = 75.0f,
    .ki = 1.1f,
    .kd = 0.0f,
    .integral_limit = 500.0f,
    .output_limit = CONTROL_MOTOR_MAX,
};

static PID_TypeDef pid_speed_right = {
    .kp = 65.0f,
    .ki = 1.0f,
    .kd = 0.0f,
    .integral_limit = 500.0f,
    .output_limit = CONTROL_MOTOR_MAX,
};

static PID_TypeDef pid_pos_left = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = 500.0f,
    .output_limit = POS_SPEED_MAX,
};

static PID_TypeDef pid_pos_right = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = 500.0f,
    .output_limit = POS_SPEED_MAX,
};

void Control_Init(void)         // 控制模块初始化：初始化编码器、电机并设置默认PID参数
{                               // 函数体开始
    Encoder_Init();             // 初始化编码器（清零计数、使能GPIOB中断）
    Motor_Init();               // 初始化电机（PWM和方向引脚）
    PID_Reset(&pid_speed_left);
    PID_Reset(&pid_speed_right);
    PID_Reset(&pid_pos_left);
    PID_Reset(&pid_pos_right);
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
            target_left = 0;
            target_right = 0;
            output_left = 0;
            output_right = 0;
            Motor_StopAll();                // 四轮全部停车
        } else {                            // 未到达：位置环输出作为速度环目标
            spd = (int16_t)PID_PositionCalc(&pid_pos_left, (float)pos_target_left, (float)mileage_left);     // 左侧位置环计算
            if (spd > POS_SPEED_MAX)  spd = POS_SPEED_MAX;    // 速度目标上限幅
            if (spd < -POS_SPEED_MAX) spd = -POS_SPEED_MAX;   // 速度目标下限幅
            target_left = spd;              // 作为左侧速度环的新目标

            spd = (int16_t)PID_PositionCalc(&pid_pos_right, (float)pos_target_right, (float)mileage_right);  // 右侧位置环计算
            if (spd > POS_SPEED_MAX)  spd = POS_SPEED_MAX;    // 速度目标上限幅
            if (spd < -POS_SPEED_MAX) spd = -POS_SPEED_MAX;   // 速度目标下限幅
            target_right = spd;             // 作为右侧速度环的新目标
        }                                   // if-else 到位判断结束
    }                                       // if 位置环结束

    if (ctrl_mode != CTRL_MODE_OPEN) {      // 速度闭环或位置+速度双闭环：都要跑内环（速度环）
        out_l = (int16_t)PID_PositionCalc(&pid_speed_left,  (float)target_left,  (float)sl);   // 左侧速度环PID计算
        out_r = (int16_t)PID_PositionCalc(&pid_speed_right, (float)target_right, (float)sr);   // 右侧速度环PID计算
        output_left = out_l;
        output_right = out_r;
        Motor_Set(CONTROL_LEFT_FRONT_MOTOR, out_l);      // 左前轮输出左侧PWM
        Motor_Set(CONTROL_LEFT_REAR_MOTOR, out_l);       // 左后轮输出同样的左侧PWM
        Motor_Set(CONTROL_RIGHT_FRONT_MOTOR, out_r);     // 右前轮输出右侧PWM
        Motor_Set(CONTROL_RIGHT_REAR_MOTOR, out_r);      // 右后轮输出同样的右侧PWM
    }                           // if结束
}                               // Control_Tick函数结束

void Control_Enable(uint8_t en) // 使能/关闭速度闭环控制
{                               // 函数体开始
    ctrl_mode = (en != 0) ? CTRL_MODE_SPEED : CTRL_MODE_OPEN;   // 非0进速度闭环，0回开环

    if (ctrl_mode == CTRL_MODE_OPEN) {  // 若是关闭闭环
        /* 退出闭环时清积分，避免下次使能时冲击 */
        PID_Reset(&pid_speed_left);
        PID_Reset(&pid_speed_right);
        target_left = 0;
        target_right = 0;
        output_left = 0;
        output_right = 0;
    }                           // if结束
}                               // Control_Enable函数结束

void Control_SetTarget(int16_t left, int16_t right)  // 设置左右侧目标速度（单位：脉冲数/10ms，每轮）
{                               // 函数体开始
    target_left = left;         // 更新左侧速度目标
    target_right = right;       // 更新右侧速度目标
}                               // Control_SetTarget函数结束

int16_t Control_GetTargetLeft(void)
{
    /* 只给调试/显示读取，速度环真正使用的是模块内部 target_left。 */
    return target_left;
}

int16_t Control_GetTargetRight(void)
{
    return target_right;
}

void Control_SetPID(float kp, float ki, float kd)    // 整定速度环PID参数（左右侧共用一组）
{                               // 函数体开始
    pid_speed_left.kp = kp;     // 设置左侧速度环比例系数
    pid_speed_left.ki = ki;     // 设置左侧速度环积分系数
    pid_speed_left.kd = kd;     // 设置左侧速度环微分系数

    pid_speed_right.kp = kp;    // 设置右侧速度环比例系数
    pid_speed_right.ki = ki;    // 设置右侧速度环积分系数
    pid_speed_right.kd = kd;    // 设置右侧速度环微分系数
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
    PID_Reset(&pid_pos_left);
    PID_Reset(&pid_pos_right);
    PID_Reset(&pid_speed_left);
    PID_Reset(&pid_speed_right);

    ctrl_mode = CTRL_MODE_POSITION; // 进入位置+速度双闭环模式
}                               // Control_StartPosition函数结束

void Control_ResetMileage(void)
{
    mileage_left = 0;
    mileage_right = 0;
}

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

int16_t Control_GetOutputLeft(void)
{
    /* 最近一次速度 PID 输出，可直接在 OLED/VOFA 观察 PWM 是否饱和或抖动。 */
    return output_left;
}

int16_t Control_GetOutputRight(void)
{
    return output_right;
}

int32_t Control_GetMileageLeft(void) // 获取左侧累计里程
{                               // 函数体开始
    return mileage_left;        // 返回左侧累计脉冲数（同侧两轮平均）
}                               // Control_GetMileageLeft函数结束

int32_t Control_GetMileageRight(void)    // 获取右侧累计里程
{                               // 函数体开始
    return mileage_right;       // 返回右侧累计脉冲数（同侧两轮平均）
}                               // Control_GetMileageRight函数结束
