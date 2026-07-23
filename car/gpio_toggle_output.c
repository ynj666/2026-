#include "ti_msp_dl_config.h"
#include "uart.h"
#include "led_beep.h"
#include "imu.h"
#include "oled.h"
#include "bmi270_spi.h"
#include "motor.h"
#include "encoder.h"
#include "control.h"
#include "key.h"
#include "task.h"
#include "race.h"
#include "gray.h"
#include "VOFA.h"

#define IMU_DT_S                0.01f
#define APP_PRINT_DIV_TICKS     10U
#define APP_OLED_DIV_TICKS      10U

#define APP_TEST_SPEED_LEFT     20
#define APP_TEST_SPEED_RIGHT    20
#define APP_TEST_BEEP_TICKS     50U

typedef enum {
    APP_STATE_SELECT = 0,
    APP_STATE_TASK1  = 1,
    APP_STATE_TASK2  = 2,
    APP_STATE_TASK3  = 3,
    APP_STATE_TASK4  = 4,
    APP_STATE_TEST   = 5,
    APP_STATE_ERROR  = 6
} APP_State;

/* 改这里可以直接进测试状态；按键只选择 1~4 题。 */
#define APP_START_STATE         APP_STATE_SELECT

volatile uint8_t g_imuUpdateFlag = 0;
volatile uint8_t g_keyEvent = 0;

static APP_State g_appState = APP_START_STATE;
static volatile uint8_t g_imuOk = 0;
static IMU_Data g_imuData;

static uint8_t App_IsTaskState(APP_State state)
{
    return ((state >= APP_STATE_TASK1) && (state <= APP_STATE_TASK4)) ? 1U : 0U;
}

static void App_EnterSelect(void)
{
    Task_ReturnSelect();
    g_appState = APP_STATE_SELECT;
    OLED_Clear();
}

static void App_StartTask(uint8_t task)
{
    Task_SetCurrent(task);
    Task_Confirm();
    g_appState = (APP_State)task;
    OLED_Clear();
}

static void App_StartTest(void)
{
    Control_ResetMileage();
    Control_SetTarget(APP_TEST_SPEED_LEFT, APP_TEST_SPEED_RIGHT);
    Control_Enable(1);
    OLED_Clear();
}

static void App_HandleKey(uint8_t key)
{
    if (g_appState == APP_STATE_SELECT) {
        if (key == 1U) {
            Task_SwitchNext();
        } else if (key == 2U) {
            App_StartTask(Task_GetCurrent());
        }
        return;
    }

    if (App_IsTaskState(g_appState)) {
        if (Task_GetDone() && (key == 3U)) {
            App_EnterSelect();
        }
        return;
    }

    if (g_appState == APP_STATE_ERROR) {
        if (key == 3U) {
            Task_SoftReset();
        }
    }
}

static void App_RunState(const IMU_Data *imu)
{
    switch (g_appState) {
        case APP_STATE_SELECT:
            break;

        case APP_STATE_TASK1:
        case APP_STATE_TASK2:
        case APP_STATE_TASK3:
        case APP_STATE_TASK4:
            Task_Tick(imu);
            break;

        case APP_STATE_TEST:
            Control_SetTarget(APP_TEST_SPEED_LEFT, APP_TEST_SPEED_RIGHT);
            Control_Enable(1);
            break;

        case APP_STATE_ERROR:
            Control_Enable(0);
            Motor_StopAll();
            BEEP_Off();
            break;

        default:
            g_appState = APP_STATE_ERROR;
            break;
    }
}

static uint32_t App_GrayToBinaryDigits(uint8_t gray)
{
    uint32_t value = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        value *= 10U;

        if ((gray & (1U << i)) != 0U) {
            value += 1U;
        }
    }

    return value;
}

static void App_SendVofa(const IMU_Data *imu)
{
    float vf[12];

    vf[0] = ((g_imuOk != 0U) && (imu != 0)) ? imu->yaw : 0.0f;
    vf[1] = (float)Control_GetTargetLeft();
    vf[2] = (float)Control_GetSpeedLeft();
    vf[3] = (float)Control_GetOutputLeft();
    vf[4] = (float)(Control_GetTargetLeft() - Control_GetSpeedLeft());
    vf[5] = (float)Control_GetTargetRight();
    vf[6] = (float)Control_GetSpeedRight();
    vf[7] = (float)Control_GetOutputRight();
    vf[8] = (float)(Control_GetTargetRight() - Control_GetSpeedRight());
    vf[9] = (float)App_GrayToBinaryDigits(GRAY_ReadAll());
    vf[10] = (float)Race_GetLineError();
    vf[11] = (float)Race_GetLineCorrection();

    VOFA_SendFrame(vf, 12);
}

static void App_ShowYaw(const IMU_Data *imu)
{
    if ((g_imuOk != 0U) && (imu != 0)) {
        OLED_ShowString(0, 0, "Y=          ", 16);
        OLED_ShowFloat2(16, 0, imu->yaw, 3, 16);
    } else {
        OLED_ShowString(0, 0, "IMU ERR     ", 16);
    }
}

static void App_ShowRunData(void)
{
    OLED_ShowString(0, 4, "VL", 16);
    OLED_ShowSignedNum(16, 4, Control_GetSpeedLeft(), 3, 16);
    OLED_ShowString(56, 4, "VR", 16);
    OLED_ShowSignedNum(72, 4, Control_GetSpeedRight(), 3, 16);

    OLED_ShowString(0, 6, "PL", 16);
    OLED_ShowSignedNum(16, 6, Control_GetOutputLeft(), 4, 16);
    OLED_ShowString(64, 6, "PR", 16);
    OLED_ShowSignedNum(80, 6, Control_GetOutputRight(), 4, 16);
}

static void App_UpdateOled(const IMU_Data *imu)
{
    uint8_t gray;

    App_ShowYaw(imu);

    switch (g_appState) {
        case APP_STATE_SELECT:
            OLED_ShowString(0, 2, "SELECT TASK ", 16);
            OLED_ShowNum(96, 2, Task_GetCurrent(), 1, 16);
            OLED_ShowString(0, 4, "K1 SELECT   ", 16);
            OLED_ShowString(0, 6, "K2 OK       ", 16);
            break;

        case APP_STATE_TASK1:
        case APP_STATE_TASK2:
        case APP_STATE_TASK3:
        case APP_STATE_TASK4:
            OLED_ShowString(0, 2, "TASK ", 16);
            OLED_ShowNum(40, 2, (uint8_t)g_appState, 1, 16);

            if (Task_GetDone()) {
                OLED_ShowString(56, 2, "DONE  ", 16);
                OLED_ShowString(0, 4, "FINISHED    ", 16);
                OLED_ShowString(0, 6, "K3 SELECT   ", 16);
            } else {
                OLED_ShowString(56, 2, "RUN   ", 16);
                App_ShowRunData();
            }
            break;

        case APP_STATE_TEST:
            gray = GRAY_ReadAll();
            OLED_ShowString(0, 2, "TEST RUN    ", 16);
            OLED_ShowString(0, 4, "GRAY ", 16);
            OLED_ShowNum(40, 4, gray, 3, 16);
            OLED_ShowString(0, 6, "VL", 16);
            OLED_ShowSignedNum(16, 6, Control_GetSpeedLeft(), 3, 16);
            OLED_ShowString(56, 6, "VR", 16);
            OLED_ShowSignedNum(72, 6, Control_GetSpeedRight(), 3, 16);
            break;

        case APP_STATE_ERROR:
            OLED_ShowString(0, 2, "APP ERROR   ", 16);
            OLED_ShowString(0, 4, "K3 RESET    ", 16);
            OLED_ShowString(0, 6, "            ", 16);
            break;

        default:
            g_appState = APP_STATE_ERROR;
            break;
    }
}

int main(void)
{
    int8_t rslt;
    uint8_t chip_id;
    IMU_Data imu_data;

    uint8_t print_div = 0;
    uint8_t oled_div = 0;
    uint8_t beep_div = 0;

    SYSCFG_DL_init();

    LED_BEEP_Init();
    KEY_Init();
    Task_Init();

    UART0_SendString("BOOT OK\r\n");

    OLED_Init();
    OLED_ShowString(0, 0, "OLED OK", 16);
    UART0_SendString("OLED OK\r\n");

    BMI270_SPI_Init();
    chip_id = BMI270_SPI_ReadID();

    UART0_SendString("IMU ID=0x");
    UART0_SendHex8(chip_id);
    UART0_SendString("\r\n");

    rslt = IMU_Init();

    UART0_SendString("IMU Init=");
    UART0_SendInt(rslt);
    UART0_SendString("\r\n");

    if (rslt != IMU_OK) {
        UART0_SendString("DRV ID=0x");
        UART0_SendHex8(IMU_DebugGetChipId());
        UART0_SendString("\r\n");
    }

    if (rslt == IMU_OK) {
        OLED_ShowString(0, 2, "Calibrating...", 16);
        rslt = IMU_Calibrate(200);

        UART0_SendString("IMU Cal=");
        UART0_SendInt(rslt);
        UART0_SendString("\r\n");
    }

    if ((rslt != IMU_OK) && (rslt != IMU_CAL_FALLBACK)) {
        OLED_ShowString(0, 2, "IMU ERR:", 16);
        OLED_ShowSignedNum(64, 2, rslt, 3, 16);
        g_appState = APP_STATE_ERROR;
    } else {
        g_imuOk = 1;

        if (rslt == IMU_CAL_FALLBACK) {
            OLED_ShowString(0, 2, "CAL FALLBK", 16);
            UART0_SendString("IMU Cal=FALLBACK\r\n");
        }
    }

    Control_Init();
    Race_Init();

    if (g_appState == APP_STATE_TEST) {
        App_StartTest();
    } else if (App_IsTaskState(g_appState)) {
        App_StartTask((uint8_t)g_appState);
    } else {
        OLED_Clear();
    }

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
        if (g_imuUpdateFlag == 0U) {
            continue;
        }

        g_imuUpdateFlag = 0;

        if (g_imuOk) {
            __disable_irq();
            imu_data = g_imuData;
            __enable_irq();
        }

        if (g_keyEvent != 0U) {
            App_HandleKey(g_keyEvent);
            g_keyEvent = 0;
        }

        App_RunState(g_imuOk ? &imu_data : 0);

        if (g_appState == APP_STATE_TEST) {
            beep_div++;
            if (beep_div >= APP_TEST_BEEP_TICKS) {
                beep_div = 0;
                LED_Toggle();
                BEEP_Toggle();
            }
        }

        print_div++;
        if (print_div >= APP_PRINT_DIV_TICKS) {
            print_div = 0;
            App_SendVofa(g_imuOk ? &imu_data : 0);
        }

        oled_div++;
        if (oled_div >= APP_OLED_DIV_TICKS) {
            oled_div = 0;
            App_UpdateOled(g_imuOk ? &imu_data : 0);
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            if (g_imuOk) {
                (void)IMU_ReadDataDt(&g_imuData, IMU_DT_S);
            }

            Control_Tick();

            if (KEY_Scan(KEY_1)) {
                g_keyEvent = 1;
            } else if (KEY_Scan(KEY_2)) {
                g_keyEvent = 2;
            } else if (KEY_Scan(KEY_3)) {
                g_keyEvent = 3;
            }

            g_imuUpdateFlag = 1;
            break;

        default:
            break;
    }
}
