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


#define CPUCLK_FREQ                                                     32000000



/* Defines for Motor */
#define Motor_INST                                                         TIMA0
#define Motor_INST_IRQHandler                                   TIMA0_IRQHandler
#define Motor_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define Motor_INST_CLK_FREQ                                              4000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_C0_PORT                                                 GPIOA
#define GPIO_Motor_C0_PIN                                         DL_GPIO_PIN_21
#define GPIO_Motor_C0_IOMUX                                      (IOMUX_PINCM46)
#define GPIO_Motor_C0_IOMUX_FUNC                     IOMUX_PINCM46_PF_TIMA0_CCP0
#define GPIO_Motor_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_Motor_C1_PORT                                                 GPIOA
#define GPIO_Motor_C1_PIN                                          DL_GPIO_PIN_7
#define GPIO_Motor_C1_IOMUX                                      (IOMUX_PINCM14)
#define GPIO_Motor_C1_IOMUX_FUNC                     IOMUX_PINCM14_PF_TIMA0_CCP1
#define GPIO_Motor_C1_IDX                                    DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_Motor_C2_PORT                                                 GPIOB
#define GPIO_Motor_C2_PIN                                          DL_GPIO_PIN_4
#define GPIO_Motor_C2_IOMUX                                      (IOMUX_PINCM17)
#define GPIO_Motor_C2_IOMUX_FUNC                     IOMUX_PINCM17_PF_TIMA0_CCP2
#define GPIO_Motor_C2_IDX                                    DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_Motor_C3_PORT                                                 GPIOA
#define GPIO_Motor_C3_PIN                                         DL_GPIO_PIN_17
#define GPIO_Motor_C3_IOMUX                                      (IOMUX_PINCM39)
#define GPIO_Motor_C3_IOMUX_FUNC                     IOMUX_PINCM39_PF_TIMA0_CCP3
#define GPIO_Motor_C3_IDX                                    DL_TIMER_CC_3_INDEX

/* Defines for SMotor_1 */
#define SMotor_1_INST                                                      TIMG6
#define SMotor_1_INST_IRQHandler                                TIMG6_IRQHandler
#define SMotor_1_INST_INT_IRQN                                  (TIMG6_INT_IRQn)
#define SMotor_1_INST_CLK_FREQ                                             62500
/* GPIO defines for channel 0 */
#define GPIO_SMotor_1_C0_PORT                                              GPIOA
#define GPIO_SMotor_1_C0_PIN                                      DL_GPIO_PIN_29
#define GPIO_SMotor_1_C0_IOMUX                                    (IOMUX_PINCM4)
#define GPIO_SMotor_1_C0_IOMUX_FUNC                   IOMUX_PINCM4_PF_TIMG6_CCP0
#define GPIO_SMotor_1_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_SMotor_1_C1_PORT                                              GPIOA
#define GPIO_SMotor_1_C1_PIN                                      DL_GPIO_PIN_30
#define GPIO_SMotor_1_C1_IOMUX                                    (IOMUX_PINCM5)
#define GPIO_SMotor_1_C1_IOMUX_FUNC                   IOMUX_PINCM5_PF_TIMG6_CCP1
#define GPIO_SMotor_1_C1_IDX                                 DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA1)
#define TIMER_0_INST_IRQHandler                                 TIMA1_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (3999U)




/* Defines for OLED */
#define OLED_INST                                                           I2C1
#define OLED_INST_IRQHandler                                     I2C1_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C1_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 400000
#define GPIO_OLED_SDA_PORT                                                 GPIOA
#define GPIO_OLED_SDA_PIN                                         DL_GPIO_PIN_10
#define GPIO_OLED_IOMUX_SDA                                      (IOMUX_PINCM21)
#define GPIO_OLED_IOMUX_SDA_FUNC                       IOMUX_PINCM21_PF_I2C1_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOA
#define GPIO_OLED_SCL_PIN                                         DL_GPIO_PIN_11
#define GPIO_OLED_IOMUX_SCL                                      (IOMUX_PINCM22)
#define GPIO_OLED_IOMUX_SCL_FUNC                       IOMUX_PINCM22_PF_I2C1_SCL

/* Defines for MPU6050_JY61P_Tracking */
#define MPU6050_JY61P_Tracking_INST                                         I2C0
#define MPU6050_JY61P_Tracking_INST_IRQHandler                         I2C0_IRQHandler
#define MPU6050_JY61P_Tracking_INST_INT_IRQN                           I2C0_INT_IRQn
#define MPU6050_JY61P_Tracking_BUS_SPEED_HZ                                  400000
#define GPIO_MPU6050_JY61P_Tracking_SDA_PORT                                   GPIOA
#define GPIO_MPU6050_JY61P_Tracking_SDA_PIN                           DL_GPIO_PIN_0
#define GPIO_MPU6050_JY61P_Tracking_IOMUX_SDA                          (IOMUX_PINCM1)
#define GPIO_MPU6050_JY61P_Tracking_IOMUX_SDA_FUNC                IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_MPU6050_JY61P_Tracking_SCL_PORT                                   GPIOA
#define GPIO_MPU6050_JY61P_Tracking_SCL_PIN                           DL_GPIO_PIN_1
#define GPIO_MPU6050_JY61P_Tracking_IOMUX_SCL                          (IOMUX_PINCM2)
#define GPIO_MPU6050_JY61P_Tracking_IOMUX_SCL_FUNC                IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for BlueTooth */
#define BlueTooth_INST                                                     UART0
#define BlueTooth_INST_FREQUENCY                                        32000000
#define BlueTooth_INST_IRQHandler                               UART0_IRQHandler
#define BlueTooth_INST_INT_IRQN                                   UART0_INT_IRQn
#define GPIO_BlueTooth_RX_PORT                                             GPIOB
#define GPIO_BlueTooth_TX_PORT                                             GPIOB
#define GPIO_BlueTooth_RX_PIN                                      DL_GPIO_PIN_1
#define GPIO_BlueTooth_TX_PIN                                      DL_GPIO_PIN_0
#define GPIO_BlueTooth_IOMUX_RX                                  (IOMUX_PINCM13)
#define GPIO_BlueTooth_IOMUX_TX                                  (IOMUX_PINCM12)
#define GPIO_BlueTooth_IOMUX_RX_FUNC                   IOMUX_PINCM13_PF_UART0_RX
#define GPIO_BlueTooth_IOMUX_TX_FUNC                   IOMUX_PINCM12_PF_UART0_TX
#define BlueTooth_BAUD_RATE                                             (115200)
#define BlueTooth_IBRD_32_MHZ_115200_BAUD                                   (17)
#define BlueTooth_FBRD_32_MHZ_115200_BAUD                                   (23)
/* Defines for K230 */
#define K230_INST                                                          UART2
#define K230_INST_FREQUENCY                                             32000000
#define K230_INST_IRQHandler                                    UART2_IRQHandler
#define K230_INST_INT_IRQN                                        UART2_INT_IRQn
#define GPIO_K230_RX_PORT                                                  GPIOA
#define GPIO_K230_TX_PORT                                                  GPIOA
#define GPIO_K230_RX_PIN                                          DL_GPIO_PIN_24
#define GPIO_K230_TX_PIN                                          DL_GPIO_PIN_23
#define GPIO_K230_IOMUX_RX                                       (IOMUX_PINCM54)
#define GPIO_K230_IOMUX_TX                                       (IOMUX_PINCM53)
#define GPIO_K230_IOMUX_RX_FUNC                        IOMUX_PINCM54_PF_UART2_RX
#define GPIO_K230_IOMUX_TX_FUNC                        IOMUX_PINCM53_PF_UART2_TX
#define K230_BAUD_RATE                                                  (115200)
#define K230_IBRD_32_MHZ_115200_BAUD                                        (17)
#define K230_FBRD_32_MHZ_115200_BAUD                                        (23)
/* Defines for BLDC */
#define BLDC_INST                                                          UART3
#define BLDC_INST_FREQUENCY                                             32000000
#define BLDC_INST_IRQHandler                                    UART3_IRQHandler
#define BLDC_INST_INT_IRQN                                        UART3_INT_IRQn
#define GPIO_BLDC_RX_PORT                                                  GPIOB
#define GPIO_BLDC_TX_PORT                                                  GPIOB
#define GPIO_BLDC_RX_PIN                                           DL_GPIO_PIN_3
#define GPIO_BLDC_TX_PIN                                          DL_GPIO_PIN_12
#define GPIO_BLDC_IOMUX_RX                                       (IOMUX_PINCM16)
#define GPIO_BLDC_IOMUX_TX                                       (IOMUX_PINCM29)
#define GPIO_BLDC_IOMUX_RX_FUNC                        IOMUX_PINCM16_PF_UART3_RX
#define GPIO_BLDC_IOMUX_TX_FUNC                        IOMUX_PINCM29_PF_UART3_TX
#define BLDC_BAUD_RATE                                                  (115200)
#define BLDC_IBRD_32_MHZ_115200_BAUD                                        (17)
#define BLDC_FBRD_32_MHZ_115200_BAUD                                        (23)
/* Defines for Debug_Ex */
#define Debug_Ex_INST                                                      UART1
#define Debug_Ex_INST_FREQUENCY                                         32000000
#define Debug_Ex_INST_IRQHandler                                UART1_IRQHandler
#define Debug_Ex_INST_INT_IRQN                                    UART1_INT_IRQn
#define GPIO_Debug_Ex_RX_PORT                                              GPIOA
#define GPIO_Debug_Ex_TX_PORT                                              GPIOA
#define GPIO_Debug_Ex_RX_PIN                                       DL_GPIO_PIN_9
#define GPIO_Debug_Ex_TX_PIN                                       DL_GPIO_PIN_8
#define GPIO_Debug_Ex_IOMUX_RX                                   (IOMUX_PINCM20)
#define GPIO_Debug_Ex_IOMUX_TX                                   (IOMUX_PINCM19)
#define GPIO_Debug_Ex_IOMUX_RX_FUNC                    IOMUX_PINCM20_PF_UART1_RX
#define GPIO_Debug_Ex_IOMUX_TX_FUNC                    IOMUX_PINCM19_PF_UART1_TX
#define Debug_Ex_BAUD_RATE                                              (115200)
#define Debug_Ex_IBRD_32_MHZ_115200_BAUD                                    (17)
#define Debug_Ex_FBRD_32_MHZ_115200_BAUD                                    (23)





/* Port definition for Pin Group Servo */
#define Servo_PORT                                                       (GPIOA)

/* Defines for PIN1: GPIOA.28 with pinCMx 3 on package pin 35 */
#define Servo_PIN1_PIN                                          (DL_GPIO_PIN_28)
#define Servo_PIN1_IOMUX                                          (IOMUX_PINCM3)
/* Port definition for Pin Group Buzzer */
#define Buzzer_PORT                                                      (GPIOB)

/* Defines for PIN: GPIOB.5 with pinCMx 18 on package pin 53 */
#define Buzzer_PIN_PIN                                           (DL_GPIO_PIN_5)
#define Buzzer_PIN_IOMUX                                         (IOMUX_PINCM18)
/* Port definition for Pin Group BlueTooth_State */
#define BlueTooth_State_PORT                                             (GPIOB)

/* Defines for State_PIN: GPIOB.10 with pinCMx 27 on package pin 62 */
#define BlueTooth_State_State_PIN_PIN                           (DL_GPIO_PIN_10)
#define BlueTooth_State_State_PIN_IOMUX                          (IOMUX_PINCM27)
/* Defines for DIR1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define SMotor_IO_DIR1_PORT                                              (GPIOB)
#define SMotor_IO_DIR1_PIN                                      (DL_GPIO_PIN_14)
#define SMotor_IO_DIR1_IOMUX                                     (IOMUX_PINCM31)
/* Defines for DIR2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define SMotor_IO_DIR2_PORT                                              (GPIOA)
#define SMotor_IO_DIR2_PIN                                      (DL_GPIO_PIN_15)
#define SMotor_IO_DIR2_IOMUX                                     (IOMUX_PINCM37)
/* Defines for EN1: GPIOB.11 with pinCMx 28 on package pin 63 */
#define SMotor_IO_EN1_PORT                                               (GPIOB)
#define SMotor_IO_EN1_PIN                                       (DL_GPIO_PIN_11)
#define SMotor_IO_EN1_IOMUX                                      (IOMUX_PINCM28)
/* Defines for EN2: GPIOB.20 with pinCMx 48 on package pin 19 */
#define SMotor_IO_EN2_PORT                                               (GPIOB)
#define SMotor_IO_EN2_PIN                                       (DL_GPIO_PIN_20)
#define SMotor_IO_EN2_IOMUX                                      (IOMUX_PINCM48)
/* Defines for AIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define Motor_IO_AIN1_PORT                                               (GPIOA)
#define Motor_IO_AIN1_PIN                                       (DL_GPIO_PIN_16)
#define Motor_IO_AIN1_IOMUX                                      (IOMUX_PINCM38)
/* Defines for AIN2: GPIOB.17 with pinCMx 43 on package pin 14 */
#define Motor_IO_AIN2_PORT                                               (GPIOB)
#define Motor_IO_AIN2_PIN                                       (DL_GPIO_PIN_17)
#define Motor_IO_AIN2_IOMUX                                      (IOMUX_PINCM43)
/* Defines for BIN1: GPIOB.18 with pinCMx 44 on package pin 15 */
#define Motor_IO_BIN1_PORT                                               (GPIOB)
#define Motor_IO_BIN1_PIN                                       (DL_GPIO_PIN_18)
#define Motor_IO_BIN1_IOMUX                                      (IOMUX_PINCM44)
/* Defines for BIN2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define Motor_IO_BIN2_PORT                                               (GPIOB)
#define Motor_IO_BIN2_PIN                                       (DL_GPIO_PIN_19)
#define Motor_IO_BIN2_IOMUX                                      (IOMUX_PINCM45)
/* Defines for E1A: GPIOB.2 with pinCMx 15 on package pin 50 */
#define Motor_IO_E1A_PORT                                                (GPIOB)
// pins affected by this interrupt request:["E1A"]
#define Motor_IO_GPIOB_INT_IRQN                                 (GPIOB_INT_IRQn)
#define Motor_IO_GPIOB_INT_IIDX                 (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define Motor_IO_E1A_IIDX                                    (DL_GPIO_IIDX_DIO2)
#define Motor_IO_E1A_PIN                                         (DL_GPIO_PIN_2)
#define Motor_IO_E1A_IOMUX                                       (IOMUX_PINCM15)
/* Defines for E2A: GPIOA.22 with pinCMx 47 on package pin 18 */
#define Motor_IO_E2A_PORT                                                (GPIOA)
// pins affected by this interrupt request:["E2A"]
#define Motor_IO_GPIOA_INT_IRQN                                 (GPIOA_INT_IRQn)
#define Motor_IO_GPIOA_INT_IIDX                 (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Motor_IO_E2A_IIDX                                   (DL_GPIO_IIDX_DIO22)
#define Motor_IO_E2A_PIN                                        (DL_GPIO_PIN_22)
#define Motor_IO_E2A_IOMUX                                       (IOMUX_PINCM47)
/* Defines for E1B: GPIOA.2 with pinCMx 7 on package pin 42 */
#define Motor_IO_E1B_PORT                                                (GPIOA)
#define Motor_IO_E1B_PIN                                         (DL_GPIO_PIN_2)
#define Motor_IO_E1B_IOMUX                                        (IOMUX_PINCM7)
/* Defines for E2B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define Motor_IO_E2B_PORT                                                (GPIOA)
#define Motor_IO_E2B_PIN                                        (DL_GPIO_PIN_25)
#define Motor_IO_E2B_IOMUX                                       (IOMUX_PINCM55)
/* Defines for PIN_1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define Key_PIN_1_PORT                                                   (GPIOA)
#define Key_PIN_1_PIN                                           (DL_GPIO_PIN_14)
#define Key_PIN_1_IOMUX                                          (IOMUX_PINCM36)
/* Defines for PIN_2: GPIOB.22 with pinCMx 50 on package pin 21 */
#define Key_PIN_2_PORT                                                   (GPIOB)
#define Key_PIN_2_PIN                                           (DL_GPIO_PIN_22)
#define Key_PIN_2_IOMUX                                          (IOMUX_PINCM50)
/* Defines for PIN_3: GPIOB.24 with pinCMx 52 on package pin 23 */
#define Key_PIN_3_PORT                                                   (GPIOB)
#define Key_PIN_3_PIN                                           (DL_GPIO_PIN_24)
#define Key_PIN_3_IOMUX                                          (IOMUX_PINCM52)
/* Defines for PIN_4: GPIOB.25 with pinCMx 56 on package pin 27 */
#define Key_PIN_4_PORT                                                   (GPIOB)
#define Key_PIN_4_PIN                                           (DL_GPIO_PIN_25)
#define Key_PIN_4_IOMUX                                          (IOMUX_PINCM56)
/* Defines for Tracking_1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define Tracking_Tracking_1_PORT                                         (GPIOB)
#define Tracking_Tracking_1_PIN                                  (DL_GPIO_PIN_6)
#define Tracking_Tracking_1_IOMUX                                (IOMUX_PINCM23)
/* Defines for Tracking_2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define Tracking_Tracking_2_PORT                                         (GPIOB)
#define Tracking_Tracking_2_PIN                                  (DL_GPIO_PIN_7)
#define Tracking_Tracking_2_IOMUX                                (IOMUX_PINCM24)
/* Defines for Tracking_3: GPIOB.8 with pinCMx 25 on package pin 60 */
#define Tracking_Tracking_3_PORT                                         (GPIOB)
#define Tracking_Tracking_3_PIN                                  (DL_GPIO_PIN_8)
#define Tracking_Tracking_3_IOMUX                                (IOMUX_PINCM25)
/* Defines for Tracking_4: GPIOB.9 with pinCMx 26 on package pin 61 */
#define Tracking_Tracking_4_PORT                                         (GPIOB)
#define Tracking_Tracking_4_PIN                                  (DL_GPIO_PIN_9)
#define Tracking_Tracking_4_IOMUX                                (IOMUX_PINCM26)
/* Defines for Tracking_5: GPIOA.12 with pinCMx 34 on package pin 5 */
#define Tracking_Tracking_5_PORT                                         (GPIOA)
#define Tracking_Tracking_5_PIN                                 (DL_GPIO_PIN_12)
#define Tracking_Tracking_5_IOMUX                                (IOMUX_PINCM34)
/* Defines for Tracking_6: GPIOB.26 with pinCMx 57 on package pin 28 */
#define Tracking_Tracking_6_PORT                                         (GPIOB)
#define Tracking_Tracking_6_PIN                                 (DL_GPIO_PIN_26)
#define Tracking_Tracking_6_IOMUX                                (IOMUX_PINCM57)
/* Defines for Tracking_7: GPIOB.23 with pinCMx 51 on package pin 22 */
#define Tracking_Tracking_7_PORT                                         (GPIOB)
#define Tracking_Tracking_7_PIN                                 (DL_GPIO_PIN_23)
#define Tracking_Tracking_7_IOMUX                                (IOMUX_PINCM51)
/* Defines for Tracking_8: GPIOA.13 with pinCMx 35 on package pin 6 */
#define Tracking_Tracking_8_PORT                                         (GPIOA)
#define Tracking_Tracking_8_PIN                                 (DL_GPIO_PIN_13)
#define Tracking_Tracking_8_IOMUX                                (IOMUX_PINCM35)
/* Port definition for Pin Group SR04 */
#define SR04_PORT                                                        (GPIOA)

/* Defines for Trig: GPIOA.27 with pinCMx 60 on package pin 31 */
#define SR04_Trig_PIN                                           (DL_GPIO_PIN_27)
#define SR04_Trig_IOMUX                                          (IOMUX_PINCM60)
/* Defines for Echo: GPIOA.26 with pinCMx 59 on package pin 30 */
#define SR04_Echo_PIN                                           (DL_GPIO_PIN_26)
#define SR04_Echo_IOMUX                                          (IOMUX_PINCM59)
/* Defines for G: GPIOA.31 with pinCMx 6 on package pin 39 */
#define LED_G_PORT                                                       (GPIOA)
#define LED_G_PIN                                               (DL_GPIO_PIN_31)
#define LED_G_IOMUX                                               (IOMUX_PINCM6)
/* Defines for Y: GPIOB.27 with pinCMx 58 on package pin 29 */
#define LED_Y_PORT                                                       (GPIOB)
#define LED_Y_PIN                                               (DL_GPIO_PIN_27)
#define LED_Y_IOMUX                                              (IOMUX_PINCM58)
/* Defines for R: GPIOB.16 with pinCMx 33 on package pin 4 */
#define LED_R_PORT                                                       (GPIOB)
#define LED_R_PIN                                               (DL_GPIO_PIN_16)
#define LED_R_IOMUX                                              (IOMUX_PINCM33)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_Motor_init(void);
void SYSCFG_DL_SMotor_1_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_MPU6050_JY61P_Tracking_init(void);
void SYSCFG_DL_BlueTooth_init(void);
void SYSCFG_DL_K230_init(void);
void SYSCFG_DL_BLDC_init(void);
void SYSCFG_DL_Debug_Ex_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
