/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMG0
#define PWM_0_INST_IRQHandler                                   TIMG0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_12
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM34)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_13
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM35)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMA1
#define PWM_1_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_1_C0_PORT                                                 GPIOB
#define GPIO_PWM_1_C0_PIN                                         DL_GPIO_PIN_17
#define GPIO_PWM_1_C0_IOMUX                                      (IOMUX_PINCM43)
#define GPIO_PWM_1_C0_IOMUX_FUNC                     IOMUX_PINCM43_PF_TIMA1_CCP0
#define GPIO_PWM_1_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_1_C1_PORT                                                 GPIOB
#define GPIO_PWM_1_C1_PIN                                         DL_GPIO_PIN_27
#define GPIO_PWM_1_C1_IOMUX                                      (IOMUX_PINCM58)
#define GPIO_PWM_1_C1_IOMUX_FUNC                     IOMUX_PINCM58_PF_TIMA1_CCP1
#define GPIO_PWM_1_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for CAPTURE_ECHO */
#define CAPTURE_ECHO_INST                                               (TIMG12)
#define CAPTURE_ECHO_INST_IRQHandler                           TIMG12_IRQHandler
#define CAPTURE_ECHO_INST_INT_IRQN                             (TIMG12_INT_IRQn)
#define CAPTURE_ECHO_INST_LOAD_VALUE                                   (599999U)
/* GPIO defines for channel 0 */
#define GPIO_CAPTURE_ECHO_C0_PORT                                          GPIOB
#define GPIO_CAPTURE_ECHO_C0_PIN                                  DL_GPIO_PIN_20
#define GPIO_CAPTURE_ECHO_C0_IOMUX                               (IOMUX_PINCM48)
#define GPIO_CAPTURE_ECHO_C0_IOMUX_FUNC             IOMUX_PINCM48_PF_TIMG12_CCP0





/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG6)
#define TIMER_0_INST_IRQHandler                                 TIMG6_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG6_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (49999U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_1
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_0
#define GPIO_UART_0_IOMUX_RX                                      (IOMUX_PINCM2)
#define GPIO_UART_0_IOMUX_TX                                      (IOMUX_PINCM1)
#define GPIO_UART_0_IOMUX_RX_FUNC                       IOMUX_PINCM2_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                       IOMUX_PINCM1_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_9
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_8
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM20)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM19)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM19_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_1_FBRD_40_MHZ_115200_BAUD                                      (45)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI1
#define SPI_0_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOB
#define GPIO_SPI_0_PICO_PIN                                       DL_GPIO_PIN_8
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM25)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_0_POCI_PORT                                              GPIOB
#define GPIO_SPI_0_POCI_PIN                                       DL_GPIO_PIN_7
#define GPIO_SPI_0_IOMUX_POCI                                   (IOMUX_PINCM24)
#define GPIO_SPI_0_IOMUX_POCI_FUNC                   IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOB
#define GPIO_SPI_0_SCLK_PIN                                       DL_GPIO_PIN_9
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM26)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM26_PF_SPI1_SCLK
#define GPIO_SPI_0_CS0_PORT                                               GPIOA
#define GPIO_SPI_0_CS0_PIN                                        DL_GPIO_PIN_2
#define GPIO_SPI_0_IOMUX_CS0                                     (IOMUX_PINCM7)
#define GPIO_SPI_0_IOMUX_CS0_FUNC                      IOMUX_PINCM7_PF_SPI1_CS0



/* Defines for AIN1: GPIOA.10 with pinCMx 21 on package pin 56 */
#define GPIO_OUTPUT_AIN1_PORT                                            (GPIOA)
#define GPIO_OUTPUT_AIN1_PIN                                    (DL_GPIO_PIN_10)
#define GPIO_OUTPUT_AIN1_IOMUX                                   (IOMUX_PINCM21)
/* Defines for AIN2: GPIOA.11 with pinCMx 22 on package pin 57 */
#define GPIO_OUTPUT_AIN2_PORT                                            (GPIOA)
#define GPIO_OUTPUT_AIN2_PIN                                    (DL_GPIO_PIN_11)
#define GPIO_OUTPUT_AIN2_IOMUX                                   (IOMUX_PINCM22)
/* Defines for LED1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GPIO_OUTPUT_LED1_PORT                                            (GPIOB)
#define GPIO_OUTPUT_LED1_PIN                                    (DL_GPIO_PIN_14)
#define GPIO_OUTPUT_LED1_IOMUX                                   (IOMUX_PINCM31)
/* Defines for BIN1: GPIOA.28 with pinCMx 3 on package pin 35 */
#define GPIO_OUTPUT_BIN1_PORT                                            (GPIOA)
#define GPIO_OUTPUT_BIN1_PIN                                    (DL_GPIO_PIN_28)
#define GPIO_OUTPUT_BIN1_IOMUX                                    (IOMUX_PINCM3)
/* Defines for BIN2: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_OUTPUT_BIN2_PORT                                            (GPIOA)
#define GPIO_OUTPUT_BIN2_PIN                                    (DL_GPIO_PIN_31)
#define GPIO_OUTPUT_BIN2_IOMUX                                    (IOMUX_PINCM6)
/* Defines for TRIG: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_OUTPUT_TRIG_PORT                                            (GPIOB)
#define GPIO_OUTPUT_TRIG_PIN                                    (DL_GPIO_PIN_13)
#define GPIO_OUTPUT_TRIG_IOMUX                                   (IOMUX_PINCM30)
/* Defines for BEEP: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_OUTPUT_BEEP_PORT                                            (GPIOB)
#define GPIO_OUTPUT_BEEP_PIN                                    (DL_GPIO_PIN_12)
#define GPIO_OUTPUT_BEEP_IOMUX                                   (IOMUX_PINCM29)
/* Defines for LED2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_OUTPUT_LED2_PORT                                            (GPIOA)
#define GPIO_OUTPUT_LED2_PIN                                    (DL_GPIO_PIN_14)
#define GPIO_OUTPUT_LED2_IOMUX                                   (IOMUX_PINCM36)
/* Defines for CIN_1: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_OUTPUT_CIN_1_PORT                                           (GPIOB)
#define GPIO_OUTPUT_CIN_1_PIN                                   (DL_GPIO_PIN_23)
#define GPIO_OUTPUT_CIN_1_IOMUX                                  (IOMUX_PINCM51)
/* Defines for CIN_2: GPIOA.23 with pinCMx 53 on package pin 24 */
#define GPIO_OUTPUT_CIN_2_PORT                                           (GPIOA)
#define GPIO_OUTPUT_CIN_2_PIN                                   (DL_GPIO_PIN_23)
#define GPIO_OUTPUT_CIN_2_IOMUX                                  (IOMUX_PINCM53)
/* Defines for DIN_1: GPIOA.21 with pinCMx 46 on package pin 17 */
#define GPIO_OUTPUT_DIN_1_PORT                                           (GPIOA)
#define GPIO_OUTPUT_DIN_1_PIN                                   (DL_GPIO_PIN_21)
#define GPIO_OUTPUT_DIN_1_IOMUX                                  (IOMUX_PINCM46)
/* Defines for DIN_2: GPIOA.29 with pinCMx 4 on package pin 36 */
#define GPIO_OUTPUT_DIN_2_PORT                                           (GPIOA)
#define GPIO_OUTPUT_DIN_2_PIN                                   (DL_GPIO_PIN_29)
#define GPIO_OUTPUT_DIN_2_IOMUX                                   (IOMUX_PINCM4)
/* Port definition for Pin Group GPIO_ENCODER */
#define GPIO_ENCODER_PORT                                                (GPIOB)

/* Defines for ENC1_A: GPIOB.0 with pinCMx 12 on package pin 47 */
// pins affected by this interrupt request:["ENC1_A","ENC1_B","ENC2_A","ENC2_B","ENC3_A","ENC3_B","ENC4_A","ENC4_B"]
#define GPIO_ENCODER_INT_IRQN                                   (GPIOB_INT_IRQn)
#define GPIO_ENCODER_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_ENC1_A_IIDX                             (DL_GPIO_IIDX_DIO0)
#define GPIO_ENCODER_ENC1_A_PIN                                  (DL_GPIO_PIN_0)
#define GPIO_ENCODER_ENC1_A_IOMUX                                (IOMUX_PINCM12)
/* Defines for ENC1_B: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_ENCODER_ENC1_B_IIDX                            (DL_GPIO_IIDX_DIO16)
#define GPIO_ENCODER_ENC1_B_PIN                                 (DL_GPIO_PIN_16)
#define GPIO_ENCODER_ENC1_B_IOMUX                                (IOMUX_PINCM33)
/* Defines for ENC2_A: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_ENCODER_ENC2_A_IIDX                             (DL_GPIO_IIDX_DIO1)
#define GPIO_ENCODER_ENC2_A_PIN                                  (DL_GPIO_PIN_1)
#define GPIO_ENCODER_ENC2_A_IOMUX                                (IOMUX_PINCM13)
/* Defines for ENC2_B: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_ENCODER_ENC2_B_IIDX                             (DL_GPIO_IIDX_DIO4)
#define GPIO_ENCODER_ENC2_B_PIN                                  (DL_GPIO_PIN_4)
#define GPIO_ENCODER_ENC2_B_IOMUX                                (IOMUX_PINCM17)
/* Defines for ENC3_A: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_ENCODER_ENC3_A_IIDX                            (DL_GPIO_IIDX_DIO10)
#define GPIO_ENCODER_ENC3_A_PIN                                 (DL_GPIO_PIN_10)
#define GPIO_ENCODER_ENC3_A_IOMUX                                (IOMUX_PINCM27)
/* Defines for ENC3_B: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_ENCODER_ENC3_B_IIDX                            (DL_GPIO_IIDX_DIO11)
#define GPIO_ENCODER_ENC3_B_PIN                                 (DL_GPIO_PIN_11)
#define GPIO_ENCODER_ENC3_B_IOMUX                                (IOMUX_PINCM28)
/* Defines for ENC4_A: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_ENCODER_ENC4_A_IIDX                            (DL_GPIO_IIDX_DIO22)
#define GPIO_ENCODER_ENC4_A_PIN                                 (DL_GPIO_PIN_22)
#define GPIO_ENCODER_ENC4_A_IOMUX                                (IOMUX_PINCM50)
/* Defines for ENC4_B: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_ENCODER_ENC4_B_IIDX                            (DL_GPIO_IIDX_DIO21)
#define GPIO_ENCODER_ENC4_B_PIN                                 (DL_GPIO_PIN_21)
#define GPIO_ENCODER_ENC4_B_IOMUX                                (IOMUX_PINCM49)
/* Defines for GRAY1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_GRAY_GRAY1_PORT                                             (GPIOA)
#define GPIO_GRAY_GRAY1_PIN                                     (DL_GPIO_PIN_25)
#define GPIO_GRAY_GRAY1_IOMUX                                    (IOMUX_PINCM55)
/* Defines for GRAY2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_GRAY_GRAY2_PORT                                             (GPIOA)
#define GPIO_GRAY_GRAY2_PIN                                     (DL_GPIO_PIN_26)
#define GPIO_GRAY_GRAY2_IOMUX                                    (IOMUX_PINCM59)
/* Defines for GRAY3: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_GRAY_GRAY3_PORT                                             (GPIOA)
#define GPIO_GRAY_GRAY3_PIN                                     (DL_GPIO_PIN_27)
#define GPIO_GRAY_GRAY3_IOMUX                                    (IOMUX_PINCM60)
/* Defines for GRAY4: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_GRAY_GRAY4_PORT                                             (GPIOB)
#define GPIO_GRAY_GRAY4_PIN                                     (DL_GPIO_PIN_24)
#define GPIO_GRAY_GRAY4_IOMUX                                    (IOMUX_PINCM52)
/* Defines for GRAY5: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_GRAY_GRAY5_PORT                                             (GPIOB)
#define GPIO_GRAY_GRAY5_PIN                                     (DL_GPIO_PIN_19)
#define GPIO_GRAY_GRAY5_IOMUX                                    (IOMUX_PINCM45)
/* Defines for GRAY6: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_GRAY_GRAY6_PORT                                             (GPIOA)
#define GPIO_GRAY_GRAY6_PIN                                     (DL_GPIO_PIN_22)
#define GPIO_GRAY_GRAY6_IOMUX                                    (IOMUX_PINCM47)
/* Defines for GRAY7: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_GRAY_GRAY7_PORT                                             (GPIOB)
#define GPIO_GRAY_GRAY7_PIN                                     (DL_GPIO_PIN_18)
#define GPIO_GRAY_GRAY7_IOMUX                                    (IOMUX_PINCM44)
/* Defines for GRAY8: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_GRAY_GRAY8_PORT                                             (GPIOA)
#define GPIO_GRAY_GRAY8_PIN                                     (DL_GPIO_PIN_24)
#define GPIO_GRAY_GRAY8_IOMUX                                    (IOMUX_PINCM54)
/* Port definition for Pin Group GPIO_KEY */
#define GPIO_KEY_PORT                                                    (GPIOA)

/* Defines for KEY1: GPIOA.16 with pinCMx 38 on package pin 9 */
// pins affected by this interrupt request:["KEY1","KEY2","KEY3"]
#define GPIO_KEY_INT_IRQN                                       (GPIOA_INT_IRQn)
#define GPIO_KEY_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_KEY_KEY1_IIDX                                  (DL_GPIO_IIDX_DIO16)
#define GPIO_KEY_KEY1_PIN                                       (DL_GPIO_PIN_16)
#define GPIO_KEY_KEY1_IOMUX                                      (IOMUX_PINCM38)
/* Defines for KEY2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_KEY_KEY2_IIDX                                  (DL_GPIO_IIDX_DIO17)
#define GPIO_KEY_KEY2_PIN                                       (DL_GPIO_PIN_17)
#define GPIO_KEY_KEY2_IOMUX                                      (IOMUX_PINCM39)
/* Defines for KEY3: GPIOA.18 with pinCMx 40 on package pin 11 */
#define GPIO_KEY_KEY3_IIDX                                  (DL_GPIO_IIDX_DIO18)
#define GPIO_KEY_KEY3_PIN                                       (DL_GPIO_PIN_18)
#define GPIO_KEY_KEY3_IOMUX                                      (IOMUX_PINCM40)
/* Port definition for Pin Group GPIO_OLED */
#define GPIO_OLED_PORT                                                   (GPIOB)

/* Defines for OLED_SCL: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_OLED_OLED_SCL_PIN                                   (DL_GPIO_PIN_2)
#define GPIO_OLED_OLED_SCL_IOMUX                                 (IOMUX_PINCM15)
/* Defines for OLED_SDA: GPIOB.3 with pinCMx 16 on package pin 51 */
#define GPIO_OLED_OLED_SDA_PIN                                   (DL_GPIO_PIN_3)
#define GPIO_OLED_OLED_SDA_IOMUX                                 (IOMUX_PINCM16)
/* Port definition for Pin Group GPIO_BMI270 */
#define GPIO_BMI270_PORT                                                 (GPIOB)

/* Defines for BMI270_INT: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_BMI270_BMI270_INT_PIN                              (DL_GPIO_PIN_15)
#define GPIO_BMI270_BMI270_INT_IOMUX                             (IOMUX_PINCM32)
/* Defines for BMI270_CS: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_BMI270_BMI270_CS_PIN                                (DL_GPIO_PIN_6)
#define GPIO_BMI270_BMI270_CS_IOMUX                              (IOMUX_PINCM23)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_CAPTURE_ECHO_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_SPI_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
