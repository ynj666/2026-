#ifndef __TASK_H__
#define __TASK_H__

#include <stdint.h>
#include "imu.h"

#define TASK_NUM    4

void Task_Init(void);
uint8_t Task_GetCurrent(void);
uint8_t Task_GetRunning(void);
uint8_t Task_GetDone(void);
void Task_SetCurrent(uint8_t task);
void Task_SwitchNext(void);
void Task_Confirm(void);
void Task_Tick(const IMU_Data *imu);
void Task_ReturnSelect(void);
void Task_SoftReset(void);

#endif
