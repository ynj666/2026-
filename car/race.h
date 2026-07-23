#ifndef __RACE_H__
#define __RACE_H__

#include <stdint.h>
#include "imu.h"

typedef enum {
    RACE_STATE_IDLE = 0,
    RACE_STATE_STRAIGHT,
    RACE_STATE_LINE,
    RACE_STATE_TURN,
    RACE_STATE_POINT_PROMPT,
    RACE_STATE_DONE
} RACE_State;

void Race_Init(void);
void Race_Start(uint8_t requirement);
void Race_Stop(void);
void Race_Tick(const IMU_Data *imu);

uint8_t Race_IsRunning(void);
uint8_t Race_IsDone(void);
uint8_t Race_GetState(void);
uint8_t Race_GetSegment(void);
uint8_t Race_GetLap(void);
uint8_t Race_GetPoint(void);
int16_t Race_GetLineError(void);
int16_t Race_GetLineCorrection(void);

#endif
