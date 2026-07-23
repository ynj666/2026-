#include "task.h"
#include "ti_msp_dl_config.h"
#include "race.h"

static uint8_t task_current = 0;
static uint8_t task_running = 0;
static uint8_t task_done = 0;

void Task_Init(void)
{
    task_current = 1;
    task_running = 0;
    task_done = 0;
}

uint8_t Task_GetCurrent(void)
{
    return task_current;
}

uint8_t Task_GetRunning(void)
{
    return task_running;
}

uint8_t Task_GetDone(void)
{
    return task_done;
}

void Task_SetCurrent(uint8_t task)
{
    if (task_running) {
        return;
    }

    if ((task >= 1U) && (task <= TASK_NUM)) {
        task_current = task;
        task_done = 0;
    }
}

void Task_SwitchNext(void)
{
    if (task_running) {
        return;
    }

    task_done = 0;
    task_current++;
    if (task_current > TASK_NUM) {
        task_current = 1;
    }
}

void Task_Confirm(void)
{
    if (task_running) {
        return;
    }

    task_done = 0;
    task_running = 1;
    Race_Start(task_current);
}

void Task_Tick(const IMU_Data *imu)
{
    if (!task_running) {
        return;
    }

    Race_Tick(imu);

    if (Race_IsDone()) {
        task_running = 0;
        task_done = 1;
    }
}

void Task_ReturnSelect(void)
{
    task_running = 0;
    task_done = 0;
    Race_Stop();
}

void Task_SoftReset(void)
{
    NVIC_SystemReset();
}
