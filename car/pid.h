#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral_limit;
    float output_limit;

    float out;
    float error0;
    float error1;
    float error2;
    float errorint;
} PID_TypeDef;

void PID_Reset(PID_TypeDef *pid);
float PID_PositionCalc(PID_TypeDef *pid, float target, float actual);
float PID_IncrementCalc(PID_TypeDef *pid, float target, float actual);

#endif
