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



/* Defines for Motor_Left */
#define Motor_Left_INST                                                    TIMA0
#define Motor_Left_INST_IRQHandler                              TIMA0_IRQHandler
#define Motor_Left_INST_INT_IRQN                                (TIMA0_INT_IRQn)
#define Motor_Left_INST_CLK_FREQ                                         4000000
/* GPIO defines for channel 1 */
#define GPIO_Motor_Left_C1_PORT                                            GPIOA
#define GPIO_Motor_Left_C1_PIN                                     DL_GPIO_PIN_7
#define GPIO_Motor_Left_C1_IOMUX                                 (IOMUX_PINCM14)
#define GPIO_Motor_Left_C1_IOMUX_FUNC                IOMUX_PINCM14_PF_TIMA0_CCP1
#define GPIO_Motor_Left_C1_IDX                               DL_TIMER_CC_1_INDEX

/* Defines for Motor_Right */
#define Motor_Right_INST                                                   TIMG0
#define Motor_Right_INST_IRQHandler                             TIMG0_IRQHandler
#define Motor_Right_INST_INT_IRQN                               (TIMG0_INT_IRQn)
#define Motor_Right_INST_CLK_FREQ                                        4000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_Right_C0_PORT                                           GPIOB
#define GPIO_Motor_Right_C0_PIN                                   DL_GPIO_PIN_10
#define GPIO_Motor_Right_C0_IOMUX                                (IOMUX_PINCM27)
#define GPIO_Motor_Right_C0_IOMUX_FUNC               IOMUX_PINCM27_PF_TIMG0_CCP0
#define GPIO_Motor_Right_C0_IDX                              DL_TIMER_CC_0_INDEX

/* Defines for SMotor */
#define SMotor_INST                                                        TIMG6
#define SMotor_INST_IRQHandler                                  TIMG6_IRQHandler
#define SMotor_INST_INT_IRQN                                    (TIMG6_INT_IRQn)
#define SMotor_INST_CLK_FREQ                                              500000
/* GPIO defines for channel 0 */
#define GPIO_SMotor_C0_PORT                                                GPIOA
#define GPIO_SMotor_C0_PIN                                        DL_GPIO_PIN_29
#define GPIO_SMotor_C0_IOMUX                                      (IOMUX_PINCM4)
#define GPIO_SMotor_C0_IOMUX_FUNC                     IOMUX_PINCM4_PF_TIMG6_CCP0
#define GPIO_SMotor_C0_IDX                                   DL_TIMER_CC_0_INDEX




/* Defines for SMotor_QEI */
#define SMotor_QEI_INST                                                    TIMG8
#define SMotor_QEI_INST_IRQHandler                              TIMG8_IRQHandler
#define SMotor_QEI_INST_INT_IRQN                                (TIMG8_INT_IRQn)
/* Pin configuration defines for SMotor_QEI PHA Pin */
#define GPIO_SMotor_QEI_PHA_PORT                                           GPIOA
#define GPIO_SMotor_QEI_PHA_PIN                                   DL_GPIO_PIN_26
#define GPIO_SMotor_QEI_PHA_IOMUX                                (IOMUX_PINCM59)
#define GPIO_SMotor_QEI_PHA_IOMUX_FUNC               IOMUX_PINCM59_PF_TIMG8_CCP0
/* Pin configuration defines for SMotor_QEI PHB Pin */
#define GPIO_SMotor_QEI_PHB_PORT                                           GPIOA
#define GPIO_SMotor_QEI_PHB_PIN                                   DL_GPIO_PIN_27
#define GPIO_SMotor_QEI_PHB_IOMUX                                (IOMUX_PINCM60)
#define GPIO_SMotor_QEI_PHB_IOMUX_FUNC               IOMUX_PINCM60_PF_TIMG8_CCP1


/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA1)
#define TIMER_0_INST_IRQHandler                                 TIMA1_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (799U)




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

/* Defines for Gray_JY61P_I2C */
#define Gray_JY61P_I2C_INST                                                 I2C0
#define Gray_JY61P_I2C_INST_IRQHandler                           I2C0_IRQHandler
#define Gray_JY61P_I2C_INST_INT_IRQN                               I2C0_INT_IRQn
#define Gray_JY61P_I2C_BUS_SPEED_HZ                                       400000
#define GPIO_Gray_JY61P_I2C_SDA_PORT                                       GPIOA
#define GPIO_Gray_JY61P_I2C_SDA_PIN                                DL_GPIO_PIN_0
#define GPIO_Gray_JY61P_I2C_IOMUX_SDA                             (IOMUX_PINCM1)
#define GPIO_Gray_JY61P_I2C_IOMUX_SDA_FUNC                IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_Gray_JY61P_I2C_SCL_PORT                                       GPIOA
#define GPIO_Gray_JY61P_I2C_SCL_PIN                                DL_GPIO_PIN_1
#define GPIO_Gray_JY61P_I2C_IOMUX_SCL                             (IOMUX_PINCM2)
#define GPIO_Gray_JY61P_I2C_IOMUX_SCL_FUNC                IOMUX_PINCM2_PF_I2C0_SCL


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
/* Defines for Rpi_UART */
#define Rpi_UART_INST                                                      UART2
#define Rpi_UART_INST_FREQUENCY                                         32000000
#define Rpi_UART_INST_IRQHandler                                UART2_IRQHandler
#define Rpi_UART_INST_INT_IRQN                                    UART2_INT_IRQn
#define GPIO_Rpi_UART_RX_PORT                                              GPIOA
#define GPIO_Rpi_UART_TX_PORT                                              GPIOB
#define GPIO_Rpi_UART_RX_PIN                                      DL_GPIO_PIN_24
#define GPIO_Rpi_UART_TX_PIN                                      DL_GPIO_PIN_15
#define GPIO_Rpi_UART_IOMUX_RX                                   (IOMUX_PINCM54)
#define GPIO_Rpi_UART_IOMUX_TX                                   (IOMUX_PINCM32)
#define GPIO_Rpi_UART_IOMUX_RX_FUNC                    IOMUX_PINCM54_PF_UART2_RX
#define GPIO_Rpi_UART_IOMUX_TX_FUNC                    IOMUX_PINCM32_PF_UART2_TX
#define Rpi_UART_BAUD_RATE                                              (115200)
#define Rpi_UART_IBRD_32_MHZ_115200_BAUD                                    (17)
#define Rpi_UART_FBRD_32_MHZ_115200_BAUD                                    (23)





/* Defines for DMA_CH_OLED_TX */
#define DMA_CH_OLED_TX_CHAN_ID                                               (0)
#define OLED_INST_DMA_TRIGGER                                 (DMA_I2C1_TX_TRIG)


/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for G: GPIOA.23 with pinCMx 53 on package pin 24 */
#define LED_G_PIN                                               (DL_GPIO_PIN_23)
#define LED_G_IOMUX                                              (IOMUX_PINCM53)
/* Port definition for Pin Group Buzzer */
#define Buzzer_PORT                                                      (GPIOB)

/* Defines for PIN: GPIOB.5 with pinCMx 18 on package pin 53 */
#define Buzzer_PIN_PIN                                           (DL_GPIO_PIN_5)
#define Buzzer_PIN_IOMUX                                         (IOMUX_PINCM18)
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
/* Defines for ENC_R_A: GPIOA.28 with pinCMx 3 on package pin 35 */
#define Motor_IO_ENC_R_A_PORT                                            (GPIOA)
// pins affected by this interrupt request:["ENC_R_A","ENC_L_A"]
#define Motor_IO_INT_IRQN                                       (GPIOA_INT_IRQn)
#define Motor_IO_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Motor_IO_ENC_R_A_IIDX                               (DL_GPIO_IIDX_DIO28)
#define Motor_IO_ENC_R_A_PIN                                    (DL_GPIO_PIN_28)
#define Motor_IO_ENC_R_A_IOMUX                                    (IOMUX_PINCM3)
/* Defines for ENC_L_A: GPIOA.22 with pinCMx 47 on package pin 18 */
#define Motor_IO_ENC_L_A_PORT                                            (GPIOA)
#define Motor_IO_ENC_L_A_IIDX                               (DL_GPIO_IIDX_DIO22)
#define Motor_IO_ENC_L_A_PIN                                    (DL_GPIO_PIN_22)
#define Motor_IO_ENC_L_A_IOMUX                                   (IOMUX_PINCM47)
/* Defines for ENC_R_B: GPIOA.2 with pinCMx 7 on package pin 42 */
#define Motor_IO_ENC_R_B_PORT                                            (GPIOA)
#define Motor_IO_ENC_R_B_PIN                                     (DL_GPIO_PIN_2)
#define Motor_IO_ENC_R_B_IOMUX                                    (IOMUX_PINCM7)
/* Defines for ENC_L_B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define Motor_IO_ENC_L_B_PORT                                            (GPIOA)
#define Motor_IO_ENC_L_B_PIN                                    (DL_GPIO_PIN_25)
#define Motor_IO_ENC_L_B_IOMUX                                   (IOMUX_PINCM55)
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
/* Port definition for Pin Group SMotor_IO */
#define SMotor_IO_PORT                                                   (GPIOB)

/* Defines for DIR1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define SMotor_IO_DIR1_PIN                                      (DL_GPIO_PIN_14)
#define SMotor_IO_DIR1_IOMUX                                     (IOMUX_PINCM31)
/* Defines for EN1: GPIOB.11 with pinCMx 28 on package pin 63 */
#define SMotor_IO_EN1_PIN                                       (DL_GPIO_PIN_11)
#define SMotor_IO_EN1_IOMUX                                      (IOMUX_PINCM28)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_Motor_Left_init(void);
void SYSCFG_DL_Motor_Right_init(void);
void SYSCFG_DL_SMotor_init(void);
void SYSCFG_DL_SMotor_QEI_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_Gray_JY61P_I2C_init(void);
void SYSCFG_DL_Debug_Ex_init(void);
void SYSCFG_DL_Rpi_UART_init(void);
void SYSCFG_DL_DMA_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
