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
#include "vofa.h"

#define MOTOR_TEST_PWM    400
#define IMU_DT_S          0.01f

typedef enum {
    APP_STATE_RUN = 0,
    APP_STATE_TEST,
    APP_STATE_ERROR
} APP_State;


typedef enum {
    TEST_OUTPUT_UART_TEXT = 0,
    TEST_OUTPUT_VOFA
} TEST_OutputMode;

static APP_State g_appState = APP_STATE_TEST;
//串口,vofa,切换
static TEST_OutputMode g_testOutput = TEST_OUTPUT_VOFA;

volatile uint8_t g_imuUpdateFlag = 0;
volatile uint8_t g_keyEvent = 0;

static volatile uint8_t g_imuOk = 0;
static IMU_Data g_imuData;

int main(void)
{
    int8_t rslt = IMU_OK;
    uint8_t chip_id;
    IMU_Data imu_data;

    uint8_t print_div = 0;
    uint8_t oled_div = 0;
    uint8_t led_div = 0;
    uint8_t last_key = 0;
    uint8_t key_levels = 0;

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

    if (rslt != IMU_OK && rslt != IMU_CAL_FALLBACK) {
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

    OLED_Clear();

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
        if (g_imuUpdateFlag) {
            g_imuUpdateFlag = 0;

            led_div++;
            if (led_div >= 100) {
                led_div = 0;
                LED_Toggle();
            }

            if (g_imuOk) {
                __disable_irq();
                imu_data = g_imuData;
                __enable_irq();
            }

            if (g_keyEvent) {
                last_key = g_keyEvent;

                switch (g_appState) {
                    case APP_STATE_RUN:
											
                    case APP_STATE_TEST:
                        if (g_keyEvent == 1) {
                            Task_SwitchNext();// 切换题目
                        } else if (g_keyEvent == 2) {
                            Task_ToggleRun();// 开始/暂停
                        } else if (g_keyEvent == 3) {
                            Task_SoftReset();// 软复位
                        }
                        break;

                    case APP_STATE_ERROR:
                        if (g_keyEvent == 3) {
                            Task_SoftReset();
                        }
                        break;

                    default:
                        g_appState = APP_STATE_ERROR;
                        break;
                }

                g_keyEvent = 0;
            }

            if ((g_appState == APP_STATE_RUN) || (g_appState == APP_STATE_TEST)) {
                Task_Tick();
            }

            print_div++;
            if (print_div >= 10) {
                print_div = 0;

                switch (g_appState) {
                    case APP_STATE_TEST:
                        if (g_testOutput == TEST_OUTPUT_UART_TEXT) {
                            if (g_imuOk) {
                                UART0_SendString("Y=");
                                UART0_SendFloat2(imu_data.yaw);
                                UART0_SendChar(' ');
                            } else {
                                UART0_SendString("IMU ERR ");
                            }

                            UART0_SendString("L=");
                            UART0_SendInt(Control_GetSpeedLeft());
                            UART0_SendString(" ML=");
                            UART0_SendInt(Control_GetMileageLeft());

                            UART0_SendString(" R=");
                            UART0_SendInt(Control_GetSpeedRight());
                            UART0_SendString(" MR=");
                            UART0_SendInt(Control_GetMileageRight());

                            UART0_SendString(" KEY=");
                            UART0_SendInt(last_key);

                            key_levels = KEY_GetRawLevels();

                            UART0_SendString(" KL=");
                            UART0_SendChar((key_levels & 0x01) ? '1' : '0');
                            UART0_SendChar((key_levels & 0x02) ? '1' : '0');
                            UART0_SendChar((key_levels & 0x04) ? '1' : '0');

                            UART0_SendString("\r\n");
                        } else {
                            float vf[12];
                            uint8_t i;

                            for (i = 0; i < 12; i++) {
                                vf[i] = 0.0f;
                            }

                            if (g_imuOk) {
                                vf[0] = imu_data.gyr_x_dps;
                                vf[1] = imu_data.gyr_y_dps;
                                vf[2] = imu_data.gyr_z_dps;
                                vf[3] = imu_data.yaw;
                                vf[4] = imu_data.pitch;
                                vf[5] = imu_data.roll;
                            }

                            vf[6]  = (float)Task_GetCurrent();
                            vf[7]  = (float)Task_GetRunning();
                            vf[8]  = (float)Control_GetSpeedLeft();
                            vf[9]  = (float)Control_GetSpeedRight();
                            vf[10] = (float)Control_GetMileageLeft();
                            vf[11] = (float)Control_GetMileageRight();

                            VOFA_SendFrame(vf, 12);
                        }
                        break;

                    case APP_STATE_RUN:
                        break;

                    case APP_STATE_ERROR:
                        UART0_SendString("APP ERROR\r\n");
                        break;

                    default:
                        g_appState = APP_STATE_ERROR;
                        break;
                }
            }

            oled_div++;
            if (oled_div >= 10) {
                oled_div = 0;

                switch (g_appState) {
                    case APP_STATE_RUN:
                    case APP_STATE_TEST:
                        if (g_imuOk) {
                            OLED_ShowString(0, 0, "Y=", 16);
                            OLED_ShowFloat2(16, 0, imu_data.yaw, 3, 16);
                        } else {
                            OLED_ShowString(0, 0, "IMU ERR", 16);
                        }

                        OLED_ShowString(0, 2, "L=", 16);
                        OLED_ShowSignedNum(16, 2, Control_GetSpeedLeft(), 4, 16);
                        OLED_ShowSignedNum(64, 2, Control_GetMileageLeft(), 6, 16);

                        OLED_ShowString(0, 4, "R=", 16);
                        OLED_ShowSignedNum(16, 4, Control_GetSpeedRight(), 4, 16);
                        OLED_ShowSignedNum(64, 4, Control_GetMileageRight(), 6, 16);

                        OLED_ShowString(64, 6, "T", 16);
                        OLED_ShowNum(72, 6, Task_GetCurrent(), 1, 16);

                        if (Task_GetRunning()) {
                            OLED_ShowString(80, 6, "RUN", 16);
                        } else {
                            OLED_ShowString(80, 6, "---", 16);
                        }
                        break;

                    case APP_STATE_ERROR:
                        OLED_ShowString(0, 0, "APP ERROR", 16);
                        OLED_ShowString(0, 2, "IMU ERR", 16);
                        Motor_Set(MOTOR_A, 0);
                        Motor_Set(MOTOR_B, 0);
                        BEEP_Off();
                        break;

                    default:
                        g_appState = APP_STATE_ERROR;
                        break;
                }
            }
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