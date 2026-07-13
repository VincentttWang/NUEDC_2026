/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gMotorBackup;
DL_TimerG_backupConfig gSMotor_1Backup;
DL_TimerA_backupConfig gTIMER_0Backup;
DL_UART_Main_backupConfig gBLDCBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_Motor_init();
    SYSCFG_DL_SMotor_1_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_OLED_init();
    SYSCFG_DL_MPU6050_JY61P_Tracking_init();
    SYSCFG_DL_BlueTooth_init();
    SYSCFG_DL_K230_init();
    SYSCFG_DL_BLDC_init();
    SYSCFG_DL_Debug_Ex_init();
    SYSCFG_DL_SYSTICK_init();
    /* Ensure backup structures have no valid state */
	gMotorBackup.backupRdy 	= false;
	gSMotor_1Backup.backupRdy 	= false;
	gTIMER_0Backup.backupRdy 	= false;
	gBLDCBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(Motor_INST, &gMotorBackup);
	retStatus &= DL_TimerG_saveConfiguration(SMotor_1_INST, &gSMotor_1Backup);
	retStatus &= DL_TimerA_saveConfiguration(TIMER_0_INST, &gTIMER_0Backup);
	retStatus &= DL_UART_Main_saveConfiguration(BLDC_INST, &gBLDCBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(Motor_INST, &gMotorBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(SMotor_1_INST, &gSMotor_1Backup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMER_0_INST, &gTIMER_0Backup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(BLDC_INST, &gBLDCBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(Motor_INST);
    DL_TimerG_reset(SMotor_1_INST);
    DL_TimerA_reset(TIMER_0_INST);
    DL_I2C_reset(OLED_INST);
    DL_I2C_reset(MPU6050_JY61P_Tracking_INST);
    DL_UART_Main_reset(BlueTooth_INST);
    DL_UART_Main_reset(K230_INST);
    DL_UART_Main_reset(BLDC_INST);
    DL_UART_Main_reset(Debug_Ex_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(Motor_INST);
    DL_TimerG_enablePower(SMotor_1_INST);
    DL_TimerA_enablePower(TIMER_0_INST);
    DL_I2C_enablePower(OLED_INST);
    DL_I2C_enablePower(MPU6050_JY61P_Tracking_INST);
    DL_UART_Main_enablePower(BlueTooth_INST);
    DL_UART_Main_enablePower(K230_INST);
    DL_UART_Main_enablePower(BLDC_INST);
    DL_UART_Main_enablePower(Debug_Ex_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_C0_IOMUX,GPIO_Motor_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_C0_PORT, GPIO_Motor_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_C1_IOMUX,GPIO_Motor_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_C1_PORT, GPIO_Motor_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_C2_IOMUX,GPIO_Motor_C2_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_C2_PORT, GPIO_Motor_C2_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_C3_IOMUX,GPIO_Motor_C3_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_C3_PORT, GPIO_Motor_C3_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SMotor_1_C0_IOMUX,GPIO_SMotor_1_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SMotor_1_C0_PORT, GPIO_SMotor_1_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SMotor_1_C1_IOMUX,GPIO_SMotor_1_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SMotor_1_C1_PORT, GPIO_SMotor_1_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_OLED_IOMUX_SDA,
        GPIO_OLED_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_OLED_IOMUX_SCL,
        GPIO_OLED_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SCL);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU6050_JY61P_Tracking_IOMUX_SDA,
        GPIO_MPU6050_JY61P_Tracking_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU6050_JY61P_Tracking_IOMUX_SCL,
        GPIO_MPU6050_JY61P_Tracking_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_MPU6050_JY61P_Tracking_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_MPU6050_JY61P_Tracking_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_BlueTooth_IOMUX_TX, GPIO_BlueTooth_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_BlueTooth_IOMUX_RX, GPIO_BlueTooth_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_K230_IOMUX_TX, GPIO_K230_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_K230_IOMUX_RX, GPIO_K230_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_BLDC_IOMUX_TX, GPIO_BLDC_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_BLDC_IOMUX_RX, GPIO_BLDC_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_Debug_Ex_IOMUX_TX, GPIO_Debug_Ex_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_Debug_Ex_IOMUX_RX, GPIO_Debug_Ex_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(Servo_PIN1_IOMUX);

    DL_GPIO_initDigitalOutput(Buzzer_PIN_IOMUX);

    DL_GPIO_initDigitalInputFeatures(BlueTooth_State_State_PIN_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(SMotor_IO_DIR1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(SMotor_IO_DIR2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutput(SMotor_IO_EN1_IOMUX);

    DL_GPIO_initDigitalOutput(SMotor_IO_EN2_IOMUX);

    DL_GPIO_initDigitalOutputFeatures(Motor_IO_AIN1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(Motor_IO_AIN2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(Motor_IO_BIN1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(Motor_IO_BIN2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Motor_IO_E1A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Motor_IO_E2A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Motor_IO_E1B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Motor_IO_E2B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_PIN_1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_PIN_2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_PIN_3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_PIN_4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Tracking_Tracking_1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Tracking_Tracking_2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Tracking_Tracking_3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Tracking_Tracking_4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Tracking_Tracking_5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Tracking_Tracking_6_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Tracking_Tracking_7_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Tracking_Tracking_8_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(SR04_Trig_IOMUX);

    DL_GPIO_initDigitalInputFeatures(SR04_Echo_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(LED_G_IOMUX);

    DL_GPIO_initDigitalOutput(LED_Y_IOMUX);

    DL_GPIO_initDigitalOutput(LED_R_IOMUX);

    DL_GPIO_clearPins(GPIOA, Servo_PIN1_PIN |
		SMotor_IO_DIR2_PIN |
		Motor_IO_AIN1_PIN |
		SR04_Trig_PIN |
		LED_G_PIN);
    DL_GPIO_enableOutput(GPIOA, Servo_PIN1_PIN |
		SMotor_IO_DIR2_PIN |
		Motor_IO_AIN1_PIN |
		SR04_Trig_PIN |
		LED_G_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_22_EDGE_RISE);
    DL_GPIO_setUpperPinsInputFilter(GPIOA, DL_GPIO_PIN_22_INPUT_FILTER_3_CYCLES);
    DL_GPIO_clearInterruptStatus(GPIOA, Motor_IO_E2A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, Motor_IO_E2A_PIN);
    DL_GPIO_clearPins(GPIOB, Buzzer_PIN_PIN |
		SMotor_IO_DIR1_PIN |
		SMotor_IO_EN1_PIN |
		SMotor_IO_EN2_PIN |
		Motor_IO_AIN2_PIN |
		Motor_IO_BIN1_PIN |
		Motor_IO_BIN2_PIN |
		LED_Y_PIN |
		LED_R_PIN);
    DL_GPIO_enableOutput(GPIOB, Buzzer_PIN_PIN |
		SMotor_IO_DIR1_PIN |
		SMotor_IO_EN1_PIN |
		SMotor_IO_EN2_PIN |
		Motor_IO_AIN2_PIN |
		Motor_IO_BIN1_PIN |
		Motor_IO_BIN2_PIN |
		LED_Y_PIN |
		LED_R_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_2_EDGE_RISE);
    DL_GPIO_setLowerPinsInputFilter(GPIOB, DL_GPIO_PIN_2_INPUT_FILTER_3_CYCLES);
    DL_GPIO_clearInterruptStatus(GPIOB, Motor_IO_E1A_PIN);
    DL_GPIO_enableInterrupt(GPIOB, Motor_IO_E1A_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOA_INT_IRQn, 0);

}


/*
 * Timer clock configuration to be sourced by  / 8 (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gMotorClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gMotorConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 1000,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Motor_init(void) {

    DL_TimerA_setClockConfig(
        Motor_INST, (DL_TimerA_ClockConfig *) &gMotorClockConfig);

    DL_TimerA_initPWMMode(
        Motor_INST, (DL_TimerA_PWMConfig *) &gMotorConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(Motor_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(Motor_INST, 500, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(Motor_INST, 500, DL_TIMER_CC_1_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_2_INDEX);
    DL_TimerA_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_2_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_3_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_3_INDEX);

    DL_TimerA_enableClock(Motor_INST);


    
    DL_TimerA_setCCPDirection(Motor_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT | DL_TIMER_CC2_OUTPUT | DL_TIMER_CC3_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 8 (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   62500 Hz = 4000000 Hz / (8 * (63 + 1))
 */
static const DL_TimerG_ClockConfig gSMotor_1ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 63U
};

static const DL_TimerG_PWMConfig gSMotor_1Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 1000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_SMotor_1_init(void) {

    DL_TimerG_setClockConfig(
        SMotor_1_INST, (DL_TimerG_ClockConfig *) &gSMotor_1ClockConfig);

    DL_TimerG_initPWMMode(
        SMotor_1_INST, (DL_TimerG_PWMConfig *) &gSMotor_1Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(SMotor_1_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(SMotor_1_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SMotor_1_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(SMotor_1_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(SMotor_1_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SMotor_1_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(SMotor_1_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(SMotor_1_INST);


    
    DL_TimerG_setCCPDirection(SMotor_1_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000 Hz = 4000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerA_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (100ms * 40000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

    DL_TimerA_setClockConfig(TIMER_0_INST,
        (DL_TimerA_ClockConfig *) &gTIMER_0ClockConfig);

    DL_TimerA_initTimerMode(TIMER_0_INST,
        (DL_TimerA_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerA_enableInterrupt(TIMER_0_INST , DL_TIMERA_INTERRUPT_ZERO_EVENT);
	NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 3);
    DL_TimerA_enableClock(TIMER_0_INST);





}


static const DL_I2C_ClockConfig gOLEDClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_OLED_init(void) {

    DL_I2C_setClockConfig(OLED_INST,
        (DL_I2C_ClockConfig *) &gOLEDClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(OLED_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(OLED_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(OLED_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(OLED_INST, 7);
    DL_I2C_setControllerTXFIFOThreshold(OLED_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(OLED_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(OLED_INST);


    /* Enable module */
    DL_I2C_enableController(OLED_INST);


}
static const DL_I2C_ClockConfig gMPU6050_JY61P_TrackingClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_MPU6050_JY61P_Tracking_init(void) {

    DL_I2C_setClockConfig(MPU6050_JY61P_Tracking_INST,
        (DL_I2C_ClockConfig *) &gMPU6050_JY61P_TrackingClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(MPU6050_JY61P_Tracking_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(MPU6050_JY61P_Tracking_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(MPU6050_JY61P_Tracking_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(MPU6050_JY61P_Tracking_INST, 7);
    DL_I2C_setControllerTXFIFOThreshold(MPU6050_JY61P_Tracking_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(MPU6050_JY61P_Tracking_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(MPU6050_JY61P_Tracking_INST);


    /* Enable module */
    DL_I2C_enableController(MPU6050_JY61P_Tracking_INST);


}

static const DL_UART_Main_ClockConfig gBlueToothClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gBlueToothConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_BlueTooth_init(void)
{
    DL_UART_Main_setClockConfig(BlueTooth_INST, (DL_UART_Main_ClockConfig *) &gBlueToothClockConfig);

    DL_UART_Main_init(BlueTooth_INST, (DL_UART_Main_Config *) &gBlueToothConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(BlueTooth_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(BlueTooth_INST, BlueTooth_IBRD_32_MHZ_115200_BAUD, BlueTooth_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(BlueTooth_INST);
}
static const DL_UART_Main_ClockConfig gK230ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gK230Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_K230_init(void)
{
    DL_UART_Main_setClockConfig(K230_INST, (DL_UART_Main_ClockConfig *) &gK230ClockConfig);

    DL_UART_Main_init(K230_INST, (DL_UART_Main_Config *) &gK230Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(K230_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(K230_INST, K230_IBRD_32_MHZ_115200_BAUD, K230_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(K230_INST,
                                 DL_UART_MAIN_INTERRUPT_RX |
                                 DL_UART_MAIN_INTERRUPT_TX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(K230_INST_INT_IRQN, 0);


    DL_UART_Main_enable(K230_INST);
}
static const DL_UART_Main_ClockConfig gBLDCClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gBLDCConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_BLDC_init(void)
{
    DL_UART_Main_setClockConfig(BLDC_INST, (DL_UART_Main_ClockConfig *) &gBLDCClockConfig);

    DL_UART_Main_init(BLDC_INST, (DL_UART_Main_Config *) &gBLDCConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(BLDC_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(BLDC_INST, BLDC_IBRD_32_MHZ_115200_BAUD, BLDC_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(BLDC_INST);
}
static const DL_UART_Main_ClockConfig gDebug_ExClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gDebug_ExConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_Debug_Ex_init(void)
{
    DL_UART_Main_setClockConfig(Debug_Ex_INST, (DL_UART_Main_ClockConfig *) &gDebug_ExClockConfig);

    DL_UART_Main_init(Debug_Ex_INST, (DL_UART_Main_Config *) &gDebug_ExConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(Debug_Ex_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(Debug_Ex_INST, Debug_Ex_IBRD_32_MHZ_115200_BAUD, Debug_Ex_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(Debug_Ex_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
    /*
     * Initializes the SysTick period to 1.00 ms,
     * enables the interrupt, and starts the SysTick Timer
     */
    DL_SYSTICK_config(32000);
}

