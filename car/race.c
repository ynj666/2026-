#include "race.h"
#include "control.h"
#include "gray.h"
#include "led_beep.h"
#include "motor.h"
#include "pid.h"

#define RACE_PULSE_PER_CM          100U

#define RACE_AB_CM                 100U
#define RACE_CD_CM                 100U
#define RACE_AC_CM                 128U
#define RACE_BD_CM                 128U
#define RACE_ARC_CM                126U

#define RACE_STRAIGHT_SPEED        18
/* 巡线基础速度，单位：编码器脉冲数/10ms。 */
#define RACE_LINE_SPEED            14
#define RACE_SPEED_MAX             30

/* 题目1基础速度，单位：编码器脉冲数/10ms。 */
#define TASK1_BASE_SPEED           20

/* 题目1航向目标，单位：度；现在先固定为 0，方便你看 yaw 是否被拉回。 */
#define TASK1_TARGET_YAW           0.0f

/* 题目1启动前等待时间，单位：10ms tick；100 tick = 1秒。 */
#define TASK1_START_WAIT_TICKS     100U

/* 题目1终点黑线最少命中路数；1=任意一路灰度为黑线就停车。 */
#define TASK1_BLACK_MIN_COUNT      1U




/* 第二题巡线调参开关：1=一启动就巡线且不按距离结束；0=运行完整第二题路线。 */
#define TASK2_LINE_TUNE_MODE       0U

/* 第二题 A->B 直线 yaw 目标角，单位：度。 */
#define TASK2_AB_TARGET_YAW        0.0f

/* 第二题 C->D 直线 yaw 目标角，单位：度。 */
#define TASK2_CD_TARGET_YAW        179.0f

/* 第二题直线段最少命中几路黑线才切段。 */
#define TASK2_BLACK_MIN_COUNT      1U

/* 第二题每段开始后的保护时间，单位：10ms tick，避免刚起步误触发切段。 */
#define TASK2_SEGMENT_ARM_TICKS    20U




/* 第三题斜线基础速度，单位：编码器脉冲数/10ms。 */
#define TASK3_BASE_SPEED           18

/* 第三题 A->C 斜线 yaw 目标，单位：度；若原地转方向反了，优先改正负号。 */
#define TASK3_AC_TARGET_YAW        -30.0f

/* 第三题 B->D 斜线 yaw 目标，单位：度；和 AC 保持同一套 yaw 正方向。 */
#define TASK3_BD_TARGET_YAW        -153.0f

/* 第三题直线段最少命中几路黑线才切段。 */
#define TASK3_BLACK_MIN_COUNT      1U

/* 第三题每段开始后的保护时间，单位：10ms tick，避免刚起步误触发切段。 */
#define TASK3_SEGMENT_ARM_TICKS    20U

/* 第三题原地转向到位区间，单位：度；进区间就认为方向够用了。 */
#define TASK3_TURN_OK_ERROR_DEG    5.0f

/* 第三题原地转向稳定时间，单位：10ms tick；5 tick = 50ms。 */
#define TASK3_TURN_STABLE_TICKS    5U

/* 第三题原地转向方向修正；若转向总是反了，把 1 改成 -1。 */
#define TASK3_TURN_DIR             -1

/* 第三题第一段转向调试：1=只转到 AC 角度就停车；0=转完后进入直线。 */
#define TASK3_FIRST_TURN_TUNE_MODE 0U

/* 第三题 AC 直线调试：1=AC 直线检测到黑线就停车；0=进入下一段巡线。 */
#define TASK3_AC_STRAIGHT_TUNE_MODE 0U

/* 第三题 CB 巡线调试：1=巡线结束就停车；0=进入下一段。 */
#define TASK3_CB_LINE_TUNE_MODE    0U

/* 第三题 B 点摆正调试：1=巡线结束后先摆到 177 度；0=直接转 BD 角度。 */
#define TASK3_B_ALIGN_TUNE_MODE    1U

/* 第三题 B 点摆正目标角，单位：度。 */
#define TASK3_B_ALIGN_TARGET_YAW   177.0f

/* 第三题 BD 转向调试：1=转到 BD 角度就停车；0=进入 BD 直线。 */
#define TASK3_BD_TURN_TUNE_MODE    0U

/* 第三题 BD 直线调试：1=BD 直线检测到黑线就停车；0=进入 DA 巡线。 */
#define TASK3_BD_STRAIGHT_TUNE_MODE 0U

/* 第三题 A 点摆正调试：1=DA 巡线结束后摆回 0 度；0=到 A 后直接结束。 */
#define TASK3_A_ALIGN_TUNE_MODE    1U

/* 第三题 A 点摆正目标角，单位：度。 */
#define TASK3_A_ALIGN_TARGET_YAW   0.0f

/* 第四题圈数：第4题复用第3题路线，跑完这么多圈后结束。 */
#define TASK4_TARGET_LAPS          4

/* 第四题每多跑一圈，AC 角度向 0 收、BD 角度向负方向收多少。 */
#define TASK4_YAW_INNER_STEP_DEG   1.0f



#define RACE_YAW_KP                0.55f
#define RACE_POINT_PROMPT_TICKS    15U

/* 灰度模块黑线输出电平；你实测黑线=0。 */
#define RACE_GRAY_BLACK_LEVEL      0U
#define RACE_SEGMENT_START         1U
#define RACE_SEGMENT_ACTIVE        0U

typedef enum {
    RACE_SEG_STRAIGHT = 0,
    RACE_SEG_LINE
} Race_SegType;

typedef enum {
    RACE_POINT_NONE = 0,
    RACE_POINT_A = 1,
    RACE_POINT_B,
    RACE_POINT_C,
    RACE_POINT_D
} Race_Point;

typedef struct {
    Race_SegType type;
    uint16_t distance_cm;
    Race_Point point;
} Race_Segment;

static const Race_Segment route_req1[] = {
    {RACE_SEG_STRAIGHT, RACE_AB_CM, RACE_POINT_B},
};

static const Race_Segment route_req2[] = {
    {RACE_SEG_STRAIGHT, RACE_AB_CM,  RACE_POINT_B},
    {RACE_SEG_LINE,     RACE_ARC_CM, RACE_POINT_C},
    {RACE_SEG_STRAIGHT, RACE_CD_CM,  RACE_POINT_D},
    {RACE_SEG_LINE,     RACE_ARC_CM, RACE_POINT_A},
};

static const Race_Segment route_req2_line_tune[] = {
    {RACE_SEG_LINE, 0U, RACE_POINT_NONE},
};

static const Race_Segment route_req3[] = {
    {RACE_SEG_STRAIGHT, RACE_AC_CM,  RACE_POINT_C},
    {RACE_SEG_LINE,     RACE_ARC_CM, RACE_POINT_B},
    {RACE_SEG_STRAIGHT, RACE_BD_CM,  RACE_POINT_D},
    {RACE_SEG_LINE,     RACE_ARC_CM, RACE_POINT_A},
};

/* 题目1 yaw 外环 PID：输出是左右轮目标速度差速修正量。
 * 只调这个结构体：kp/ki/kd 是 yaw 环参数，output_limit 是最大修正速度。
 */
static PID_TypeDef pid_task1_yaw = {
    .kp = 1.0f,
    .ki = 0.1f,
    .kd = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 20.0f,
};

/* 灰度位置环 PID：输入是 LINE_ERROR，输出是左右轮目标速度的差速修正量。
 * 只调这个结构体：先调 kp；如果弧线摆动明显，再小幅加 kd；ki 先保持 0。
 */
static PID_TypeDef pid_line = {
    .kp = 0.30f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = 0.0f,
    .output_limit = 20.0f,
};

/* 第二题直线 yaw 外环 PID：输出是左右轮目标速度的差速修正量。 */
static PID_TypeDef pid_task2_yaw = {
    .kp = 1.0f,
    .ki = 0.1f,
    .kd = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 20.0f,
};

/* 第三题斜线 yaw 外环 PID：输出是左右轮目标速度的差速修正量。 */
static PID_TypeDef pid_task3_yaw = {
    .kp = 1.0f,
    .ki = 0.1f,
    .kd = 0.0f,
    .integral_limit = 100.0f,
    .output_limit = 20.0f,
};

/* 第三题原地转向 yaw PID：输出直接作为左右轮反向目标速度。 */
static PID_TypeDef pid_task3_turn = {
    .kp = 0.8f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = 0.0f,
    .output_limit = 18.0f,
};

static const Race_Segment *race_route = 0;
static uint8_t race_route_len = 0;
static uint8_t race_target_laps = 1;

static volatile uint8_t race_running = 0;
static volatile uint8_t race_done = 0;
static RACE_State race_state = RACE_STATE_IDLE;

static uint8_t race_requirement = 0;
static uint8_t race_segment = 0;
static uint8_t race_lap = 0;
static uint8_t race_prompt_ticks = 0;
static uint8_t race_finish_after_prompt = 0;
static uint8_t race_segment_phase = RACE_SEGMENT_START;
static Race_Point race_point = RACE_POINT_NONE;

static float race_target_yaw = 0.0f;
static uint16_t race_task1_wait_ticks = 0;
static int16_t race_last_line_error = 0;
static int16_t race_line_error = 0;
static int16_t race_line_corr = 0;
static uint8_t race_line_black_count = 0;
static uint16_t race_segment_arm_ticks = 0;
static uint16_t race_turn_ok_ticks = 0;
static uint8_t race_task3_b_aligned = 0;
static uint8_t race_task3_a_aligning = 0;

static int32_t Race_Abs32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int16_t Race_LimitSpeed(int16_t speed)
{
    if (speed < 0) return 0;
    if (speed > RACE_SPEED_MAX) return RACE_SPEED_MAX;
    return speed;
}

static int16_t Race_LimitSignedSpeed(int16_t speed)
{
    if (speed > RACE_SPEED_MAX) return RACE_SPEED_MAX;
    if (speed < -RACE_SPEED_MAX) return -RACE_SPEED_MAX;
    return speed;
}

static void Race_SetWheelSpeed(int16_t left, int16_t right)
{
    Control_SetTarget(Race_LimitSignedSpeed(left), Race_LimitSignedSpeed(right));
    Control_Enable(1);
}

static void Race_SetForwardSpeed(int16_t left, int16_t right)
{
    Control_SetTarget(Race_LimitSpeed(left), Race_LimitSpeed(right));
    Control_Enable(1);
}

static float Race_NormalizeAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static int32_t Race_DistanceToPulses(uint16_t cm)
{
    return (int32_t)cm * (int32_t)RACE_PULSE_PER_CM;
}

static int32_t Race_GetMileageAbs(void)
{
    int32_t left = Control_GetMileageLeft();
    int32_t right = Control_GetMileageRight();

    return (Race_Abs32(left) + Race_Abs32(right)) / 2;
}

static uint8_t Race_GrayIsBlack(uint8_t gray, uint8_t bit)
{
    uint8_t level = (gray >> bit) & 0x01U;

    return (level == RACE_GRAY_BLACK_LEVEL) ? 1U : 0U;
}

static uint8_t Race_CountBlackGray(uint8_t gray)
{
    uint8_t count = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (Race_GrayIsBlack(gray, i)) {
            count++;
        }
    }

    return count;
}

static uint8_t Race_Task1BlackLineDetected(void)
{
    return (Race_CountBlackGray(GRAY_ReadAll()) >= TASK1_BLACK_MIN_COUNT) ? 1U : 0U;
}

static uint8_t Race_IsTask2LineTuneMode(void)
{
    return ((TASK2_LINE_TUNE_MODE != 0U) && (race_requirement == 2U)) ? 1U : 0U;
}

static uint8_t Race_IsTask2FullMode(void)
{
    return ((TASK2_LINE_TUNE_MODE == 0U) && (race_requirement == 2U)) ? 1U : 0U;
}

static uint8_t Race_IsTask3LikeMode(void)
{
    return ((race_requirement == 3U) || (race_requirement == 4U)) ? 1U : 0U;
}

static uint8_t Race_UsesLineExitDetect(void)
{
    return (Race_IsTask2FullMode() || Race_IsTask3LikeMode()) ? 1U : 0U;
}

static uint8_t Race_Task2BlackLineDetected(void)
{
    return (Race_CountBlackGray(GRAY_ReadAll()) >= TASK2_BLACK_MIN_COUNT) ? 1U : 0U;
}

static uint8_t Race_Task3BlackLineDetected(void)
{
    return (Race_CountBlackGray(GRAY_ReadAll()) >= TASK3_BLACK_MIN_COUNT) ? 1U : 0U;
}

static float Race_Task2StraightTargetYaw(void)
{
    return (race_segment == 2U) ? TASK2_CD_TARGET_YAW : TASK2_AB_TARGET_YAW;
}

static float Race_Task3StraightTargetYaw(void)
{
    float task4_ac_yaw_adjust = 0.0f;
    float task4_bd_yaw_adjust = 0.0f;

    if (race_task3_a_aligning != 0U) {
        return TASK3_A_ALIGN_TARGET_YAW;
    }

    if (race_requirement == 4U) {
        task4_ac_yaw_adjust = (float)race_lap * TASK4_YAW_INNER_STEP_DEG;
        task4_bd_yaw_adjust = -((float)race_lap * TASK4_YAW_INNER_STEP_DEG);
    }

    if (race_segment == 2U) {
        if ((TASK3_B_ALIGN_TUNE_MODE != 0U) && (race_task3_b_aligned == 0U)) {
            return TASK3_B_ALIGN_TARGET_YAW;
        }

        return TASK3_BD_TARGET_YAW + task4_bd_yaw_adjust;
    }

    return TASK3_AC_TARGET_YAW + task4_ac_yaw_adjust;
}

static uint16_t Race_GetSegmentArmTicks(void)
{
    if (Race_IsTask2FullMode()) {
        return TASK2_SEGMENT_ARM_TICKS;
    }

    if (Race_IsTask3LikeMode() && (race_route[race_segment].type == RACE_SEG_LINE)) {
        return TASK3_SEGMENT_ARM_TICKS;
    }

    return 0U;
}

static uint8_t Race_SegmentArmed(void)
{
    if (race_segment_arm_ticks > 0U) {
        race_segment_arm_ticks--;
        return 0U;
    }

    return 1U;
}

static int16_t Race_CalcLineError(void)
{
    static const int16_t weight[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    uint8_t gray = GRAY_ReadAll();
    int16_t sum = 0;
    uint8_t count = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (Race_GrayIsBlack(gray, i)) {
            sum += weight[i];
            count++;
        }
    }

    race_line_black_count = count;

    if (count == 0) {
        race_last_line_error = 0;
        return 0;
    }

    race_last_line_error = sum / (int16_t)count;
    return race_last_line_error;
}

static void Race_StopMotion(void)
{
    Control_Enable(0);
    Motor_StopAll();
}

static void Race_BeginSegment(void)
{
    Control_ResetMileage();
    race_last_line_error = 0;
    race_line_error = 0;
    race_line_corr = 0;
    race_line_black_count = 0;
    race_segment_arm_ticks = Race_GetSegmentArmTicks();
    race_turn_ok_ticks = 0;
    race_segment_phase = RACE_SEGMENT_START;

    if (race_route[race_segment].type == RACE_SEG_STRAIGHT) {
        race_state = Race_IsTask3LikeMode() ? RACE_STATE_TURN : RACE_STATE_STRAIGHT;
    } else {
        race_state = RACE_STATE_LINE;
    }
}

static void Race_StartPointPrompt(Race_Point point)
{
    race_point = point;
    race_state = RACE_STATE_POINT_PROMPT;
    race_prompt_ticks = RACE_POINT_PROMPT_TICKS;
    Race_StopMotion();
    LED_On();
    BEEP_On();
}

static void Race_FinishSegment(void)
{
    Race_Point point = race_route[race_segment].point;

    race_segment++;
    if (race_segment >= race_route_len) {
        race_segment = 0;
        race_lap++;

        if (race_lap >= race_target_laps) {
            race_finish_after_prompt = 1;
        }
    }

    Race_StartPointPrompt(point);
}

static void Race_ServicePrompt(void)
{
    if (race_prompt_ticks > 0) {
        race_prompt_ticks--;
        return;
    }

    LED_Off();
    BEEP_Off();

    if ((TASK3_A_ALIGN_TUNE_MODE != 0U) &&
        Race_IsTask3LikeMode() &&
        (race_task3_a_aligning == 0U) &&
        (race_point == RACE_POINT_A)) {
        race_task3_a_aligning = 1U;
        PID_Reset(&pid_task3_turn);
        race_turn_ok_ticks = 0;
        race_segment_phase = RACE_SEGMENT_START;
        race_state = RACE_STATE_TURN;
        Race_StopMotion();
        return;
    }

    if (race_finish_after_prompt) {
        race_running = 0;
        race_done = 1;
        race_state = RACE_STATE_DONE;
        Race_StopMotion();
        return;
    }

    Race_BeginSegment();
}

static void Race_RunTask1Basic(const IMU_Data *imu)
{
    float yaw_error = 0.0f;
    int16_t corr = 0;

    if (race_segment_phase == RACE_SEGMENT_START) {
        Control_ResetMileage();
        PID_Reset(&pid_task1_yaw);
        race_target_yaw = TASK1_TARGET_YAW;
        race_task1_wait_ticks = TASK1_START_WAIT_TICKS;
        race_segment_phase = RACE_SEGMENT_ACTIVE;
    }

    if (race_task1_wait_ticks > 0U) {
        race_task1_wait_ticks--;
        Race_StopMotion();
        return;
    }

    if (Race_Task1BlackLineDetected()) {
        race_point = RACE_POINT_B;
        race_running = 0;
        race_done = 1;
        race_state = RACE_STATE_DONE;
        Race_StopMotion();
        return;
    }

    if (imu != 0) {
        yaw_error = Race_NormalizeAngle(race_target_yaw - imu->yaw);
        corr = (int16_t)PID_PositionCalc(&pid_task1_yaw, yaw_error, 0.0f);
    }

    Race_SetForwardSpeed(TASK1_BASE_SPEED + corr, TASK1_BASE_SPEED - corr);
}

static void Race_RunTask2Straight(const IMU_Data *imu)
{
    float yaw_error = 0.0f;
    int16_t corr = 0;

    if (race_segment_phase == RACE_SEGMENT_START) {
        PID_Reset(&pid_task2_yaw);
        race_target_yaw = Race_Task2StraightTargetYaw();
        race_segment_phase = RACE_SEGMENT_ACTIVE;
    }

    if (Race_SegmentArmed() && Race_Task2BlackLineDetected()) {
        Race_FinishSegment();
        return;
    }

    if (imu != 0) {
        yaw_error = Race_NormalizeAngle(race_target_yaw - imu->yaw);
        corr = (int16_t)PID_PositionCalc(&pid_task2_yaw, yaw_error, 0.0f);
    }

    Race_SetForwardSpeed(RACE_STRAIGHT_SPEED + corr, RACE_STRAIGHT_SPEED - corr);
}

static void Race_RunTask3Turn(const IMU_Data *imu)
{
    float yaw_error = 0.0f;
    int16_t turn_speed = 0;

    if (race_segment_phase == RACE_SEGMENT_START) {
        PID_Reset(&pid_task3_turn);
        race_target_yaw = Race_Task3StraightTargetYaw();
        race_turn_ok_ticks = 0;
        race_segment_phase = RACE_SEGMENT_ACTIVE;
    }

    if (imu == 0) {
        Race_StopMotion();
        return;
    }

    yaw_error = Race_NormalizeAngle(race_target_yaw - imu->yaw);

    if ((yaw_error > -TASK3_TURN_OK_ERROR_DEG) &&
        (yaw_error < TASK3_TURN_OK_ERROR_DEG)) {
        race_turn_ok_ticks++;
        Race_StopMotion();

        if (race_turn_ok_ticks >= TASK3_TURN_STABLE_TICKS) {
            if ((TASK3_FIRST_TURN_TUNE_MODE != 0U) && (race_segment == 0U)) {
                race_running = 0;
                race_done = 1;
                race_state = RACE_STATE_DONE;
                Race_StopMotion();
                return;
            }

            if ((TASK3_A_ALIGN_TUNE_MODE != 0U) &&
                (race_task3_a_aligning != 0U)) {
                race_task3_a_aligning = 0U;
                Race_StopMotion();

                if (race_lap >= race_target_laps) {
                    race_running = 0;
                    race_done = 1;
                    race_state = RACE_STATE_DONE;
                } else {
                    race_task3_b_aligned = 0U;
                    race_finish_after_prompt = 0U;
                    Race_BeginSegment();
                }

                return;
            }

            if ((TASK3_B_ALIGN_TUNE_MODE != 0U) &&
                (race_segment == 2U) &&
                (race_task3_b_aligned == 0U)) {
                race_task3_b_aligned = 1U;
                PID_Reset(&pid_task3_turn);
                race_turn_ok_ticks = 0;
                race_segment_phase = RACE_SEGMENT_START;
                race_state = RACE_STATE_TURN;
                Race_StopMotion();
                return;
            }

            if ((TASK3_BD_TURN_TUNE_MODE != 0U) &&
                (race_segment == 2U) &&
                (race_task3_b_aligned != 0U)) {
                race_running = 0;
                race_done = 1;
                race_state = RACE_STATE_DONE;
                Race_StopMotion();
                return;
            }

            PID_Reset(&pid_task3_yaw);
            Control_ResetMileage();
            race_segment_arm_ticks = TASK3_SEGMENT_ARM_TICKS;
            race_segment_phase = RACE_SEGMENT_START;
            race_state = RACE_STATE_STRAIGHT;
        }

        return;
    }

    race_turn_ok_ticks = 0;
    turn_speed = (int16_t)PID_PositionCalc(&pid_task3_turn, yaw_error, 0.0f);
    turn_speed = (int16_t)(turn_speed * TASK3_TURN_DIR);
    Race_SetWheelSpeed(-turn_speed, turn_speed);
}

static void Race_RunTask3Straight(const IMU_Data *imu)
{
    float yaw_error = 0.0f;
    int16_t corr = 0;

    if (race_segment_phase == RACE_SEGMENT_START) {
        PID_Reset(&pid_task3_yaw);
        race_target_yaw = Race_Task3StraightTargetYaw();
        race_segment_phase = RACE_SEGMENT_ACTIVE;
    }

    if (Race_SegmentArmed() && Race_Task3BlackLineDetected()) {
        if ((TASK3_AC_STRAIGHT_TUNE_MODE != 0U) && (race_segment == 0U)) {
            race_running = 0;
            race_done = 1;
            race_state = RACE_STATE_DONE;
            Race_StopMotion();
            return;
        }

        if ((TASK3_BD_STRAIGHT_TUNE_MODE != 0U) && (race_segment == 2U)) {
            race_running = 0;
            race_done = 1;
            race_state = RACE_STATE_DONE;
            Race_StopMotion();
            return;
        }

        Race_FinishSegment();
        return;
    }

    if (imu != 0) {
        yaw_error = Race_NormalizeAngle(race_target_yaw - imu->yaw);
        corr = (int16_t)PID_PositionCalc(&pid_task3_yaw, yaw_error, 0.0f);
    }

    Race_SetForwardSpeed(TASK3_BASE_SPEED + corr, TASK3_BASE_SPEED - corr);
}

static void Race_RunStraight(const IMU_Data *imu)
{
    float err = 0.0f;
    int16_t corr = 0;

    if (race_requirement == 1U) {
        Race_RunTask1Basic(imu);
        return;
    }

    if (Race_IsTask2FullMode()) {
        Race_RunTask2Straight(imu);
        return;
    }

    if (Race_IsTask3LikeMode()) {
        Race_RunTask3Straight(imu);
        return;
    }

    if (race_segment_phase == RACE_SEGMENT_START) {
        race_target_yaw = (imu != 0) ? imu->yaw : 0.0f;
        race_segment_phase = RACE_SEGMENT_ACTIVE;
    }

    if (Race_GetMileageAbs() >= Race_DistanceToPulses(race_route[race_segment].distance_cm)) {
        Race_FinishSegment();
        return;
    }

    if (imu != 0) {
        err = Race_NormalizeAngle(race_target_yaw - imu->yaw);
        corr = (int16_t)(err * RACE_YAW_KP);
    }

    Race_SetForwardSpeed(RACE_STRAIGHT_SPEED - corr, RACE_STRAIGHT_SPEED + corr);
}

static void Race_RunLine(void)
{
    if (race_segment_phase == RACE_SEGMENT_START) {
        PID_Reset(&pid_line);
        race_last_line_error = 0;
        race_line_error = 0;
        race_line_corr = 0;
        race_line_black_count = 0;
        race_segment_phase = RACE_SEGMENT_ACTIVE;
    }

    if ((!Race_IsTask2LineTuneMode()) &&
        (!Race_UsesLineExitDetect()) &&
        (Race_GetMileageAbs() >= Race_DistanceToPulses(race_route[race_segment].distance_cm))) {
        Race_FinishSegment();
        return;
    }

    race_line_error = Race_CalcLineError();

    if (Race_UsesLineExitDetect() && Race_SegmentArmed() && (race_line_black_count == 0U)) {
        if ((TASK3_CB_LINE_TUNE_MODE != 0U) &&
            Race_IsTask3LikeMode() &&
            (race_segment == 1U)) {
            race_running = 0;
            race_done = 1;
            race_state = RACE_STATE_DONE;
            Race_StopMotion();
            return;
        }

        Race_FinishSegment();
        return;
    }

    if (race_line_black_count == 0U) {
        PID_Reset(&pid_line);
        race_line_corr = 0;
    } else {
        race_line_corr = (int16_t)PID_PositionCalc(&pid_line, 0.0f, (float)race_line_error);
    }

    Race_SetForwardSpeed(RACE_LINE_SPEED - race_line_corr, RACE_LINE_SPEED + race_line_corr);
}

void Race_Init(void)
{
    race_route = 0;
    race_route_len = 0;
    race_target_laps = 1;
    race_running = 0;
    race_done = 0;
    race_state = RACE_STATE_IDLE;
    race_requirement = 0;
    race_segment = 0;
    race_lap = 0;
    race_prompt_ticks = 0;
    race_finish_after_prompt = 0;
    race_point = RACE_POINT_NONE;
    race_task1_wait_ticks = 0;
    race_line_error = 0;
    race_line_corr = 0;
    race_line_black_count = 0;
    race_segment_arm_ticks = 0;
    race_turn_ok_ticks = 0;
    race_task3_b_aligned = 0;
    race_task3_a_aligning = 0;
    race_segment_phase = RACE_SEGMENT_START;
    PID_Reset(&pid_task1_yaw);
    PID_Reset(&pid_task2_yaw);
    PID_Reset(&pid_task3_yaw);
    PID_Reset(&pid_task3_turn);
    PID_Reset(&pid_line);
    Race_StopMotion();
}

void Race_Start(uint8_t requirement)
{
    race_requirement = requirement;

    switch (requirement) {
        case 1:
            race_route = route_req1;
            race_route_len = sizeof(route_req1) / sizeof(route_req1[0]);
            race_target_laps = 1;
            break;

        case 2:
            if (TASK2_LINE_TUNE_MODE != 0U) {
                race_route = route_req2_line_tune;
                race_route_len = sizeof(route_req2_line_tune) / sizeof(route_req2_line_tune[0]);
            } else {
                race_route = route_req2;
                race_route_len = sizeof(route_req2) / sizeof(route_req2[0]);
            }
            race_target_laps = 1;
            break;

        case 3:
            race_route = route_req3;
            race_route_len = sizeof(route_req3) / sizeof(route_req3[0]);
            race_target_laps = 1;
            break;

        case 4:
            race_route = route_req3;
            race_route_len = sizeof(route_req3) / sizeof(route_req3[0]);
            race_target_laps = TASK4_TARGET_LAPS;
            break;

        default:
            Race_Init();
            return;
    }

    race_running = 1;
    race_done = 0;
    race_segment = 0;
    race_lap = 0;
    race_prompt_ticks = 0;
    race_finish_after_prompt = 0;
    race_point = RACE_POINT_NONE;
    race_task3_b_aligned = 0;
    race_task3_a_aligning = 0;
    LED_Off();
    BEEP_Off();
    Race_BeginSegment();
}

void Race_Stop(void)
{
    race_running = 0;
    race_done = 0;
    race_state = RACE_STATE_IDLE;
    race_requirement = 0;
    race_prompt_ticks = 0;
    race_task1_wait_ticks = 0;
    race_line_error = 0;
    race_line_corr = 0;
    race_line_black_count = 0;
    race_segment_arm_ticks = 0;
    race_turn_ok_ticks = 0;
    race_task3_b_aligned = 0;
    race_task3_a_aligning = 0;
    PID_Reset(&pid_task1_yaw);
    PID_Reset(&pid_task2_yaw);
    PID_Reset(&pid_task3_yaw);
    PID_Reset(&pid_task3_turn);
    PID_Reset(&pid_line);
    LED_Off();
    BEEP_Off();
    Race_StopMotion();
}

void Race_Tick(const IMU_Data *imu)
{
    if (!race_running) {
        return;
    }

    switch (race_state) {
        case RACE_STATE_STRAIGHT:
            Race_RunStraight(imu);
            break;

        case RACE_STATE_LINE:
            Race_RunLine();
            break;

        case RACE_STATE_TURN:
            Race_RunTask3Turn(imu);
            break;

        case RACE_STATE_POINT_PROMPT:
            Race_ServicePrompt();
            break;

        default:
            Race_Stop();
            break;
    }
}

uint8_t Race_IsRunning(void)
{
    return race_running;
}

uint8_t Race_IsDone(void)
{
    return race_done;
}

uint8_t Race_GetState(void)
{
    return (uint8_t)race_state;
}

uint8_t Race_GetSegment(void)
{
    if ((race_route == 0) || (race_state == RACE_STATE_IDLE) || (race_state == RACE_STATE_DONE)) {
        return 0;
    }

    return race_segment + 1U;
}

uint8_t Race_GetLap(void)
{
    if (race_route == 0) {
        return 0;
    }

    if (race_done) {
        return race_target_laps;
    }

    return race_lap + 1U;
}

uint8_t Race_GetPoint(void)
{
    return (uint8_t)race_point;
}

int16_t Race_GetLineError(void)
{
    return race_line_error;
}

int16_t Race_GetLineCorrection(void)
{
    return race_line_corr;
}
