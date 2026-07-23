#include "pid.h"

static float PID_Limit(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }

    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

void PID_Reset(PID_TypeDef *pid)
{
    if (pid == 0) {
        return;
    }

    pid->out = 0.0f;
    pid->error0 = 0.0f;
    pid->error1 = 0.0f;
    pid->error2 = 0.0f;
    pid->errorint = 0.0f;
}

float PID_PositionCalc(PID_TypeDef *pid, float target, float actual)
{
    if (pid == 0) {
        return 0.0f;
    }

    pid->error1 = pid->error0;
    pid->error0 = target - actual;

    pid->errorint += pid->error0;
    pid->errorint = PID_Limit(pid->errorint, pid->integral_limit);

    pid->out = pid->kp * pid->error0
             + pid->ki * pid->errorint
             + pid->kd * (pid->error0 - pid->error1);

    pid->out = PID_Limit(pid->out, pid->output_limit);

    return pid->out;
}

float PID_IncrementCalc(PID_TypeDef *pid, float target, float actual)
{
    if (pid == 0) {
        return 0.0f;
    }

    pid->error2 = pid->error1;
    pid->error1 = pid->error0;
    pid->error0 = target - actual;

    pid->out += pid->kp * (pid->error0 - pid->error1)
              + pid->ki * pid->error0
              + pid->kd * (pid->error0 - 2.0f * pid->error1 + pid->error2);

    pid->out = PID_Limit(pid->out, pid->output_limit);

    return pid->out;
}
