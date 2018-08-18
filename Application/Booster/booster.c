/** @FILE NAME:    booster.c
 *  @DESCRIPTION:  This file for ...
 *
 *  Copyright (c) 2018 EES Ltd.
 *  All Rights Reserved This program is the confidential and proprietary
 *  product of EES Ltd. Any Unauthorized use, reproduction or transfer
 *  of this program is strictly prohibited.
 *
 *  @Author: HaiHoang
 *  @NOTE:   No Note at the moment
 *  @BUG:    No known bugs.
 *
 *<pre>
 *  MODIFICATION HISTORY:
 *
 *  Ver   Who       Date                Changes
 *  ----- --------- ------------------  ----------------------------------------
 *  1.00  HaiHoang  August 1, 2018      First release
 *
 *
 *</pre>
 ******************************************************************************/

/***************************** Include Files *********************************/
#include <booster.h>
#include <BSP.h>
#include <utils.h>

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/
extern __interrupt void epwm3_isr(void);
extern __interrupt void epwm4_isr(void);
extern __interrupt void epwm3_tzint_isr(void);
extern __interrupt void epwm4_tzint_isr(void);
/************************** Variable Definitions *****************************/

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
BRet Boost_Init(SBooster *pBst) {
    pBst->eState        = BS_IDLE;
    pBst->period        = (uint16_t)(FTBCLK / Boost_Pwm_Freq);
    pBst->periodIQ      = _IQ20(pBst->period / 2);
    pBst->dutyValue     = 0;
    pBst->dutyMaxPer    = _IQ24(Boost_Duty_Coeff);
    pBst->dutyCurrPer   = 0;
    pBst->setVolt       = _IQ20(Boost_Volt_Output);
    LREP("BSET OUT: %d\r\n", (long)Boost_Volt_Output);

    PID_Init(&pBst->sPid, Boost_Kp, Boost_Ki, Boost_Kd, (Inverter_Pwm_Freq / 2),
             KP_A_COEFF, KP_B_COEFF);

    pBst->pwmAHandle    = (PWM_REGS *)&EPwm3Regs;   //(PWM_REGS *)&EPwm1Regs;
    pBst->pwmBHandle    = (PWM_REGS *)&EPwm4Regs;   //(PWM_REGS *)&EPwm2Regs;

    PWM_Boost_Init(pBst->pwmAHandle, pBst->pwmBHandle, Boost_Pwm_Freq, 0);

    //Boost_Set(pBst, _IQ24(0.05));
    return BR_OK;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */


#define PWM_SETTING_USE_PWM_ALL
#ifdef PWM_SETTING_USE_PWM_ALL
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void PWM_Boost_Init(PWM_REGS *pwmA, PWM_REGS *pwmB, uint32_t freq, uint16_t phase) {

    (void)phase;
    EALLOW;

//    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;     // 0=GPIO,  1=EPWM4B,  2=SCIRX-A,  3=Resv
//    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;      // 1=OUTput,  0=INput
//    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;    // uncomment if --> Set Low initially
//    //GpioDataRegs.GPASET.bit.GPIO7 = 1;      // uncomment if --> Set High initially
//
//
//    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;     // 0=GPIO,  1=EPWM3B,  2=Resv,  3=ECAP1
//    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;      // 1=OUTput,  0=INput
//    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;    // uncomment if --> Set Low initially
//    //GpioDataRegs.GPASET.bit.GPIO5 = 1;      // uncomment if --> Set High initially


//    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;     // 0=GPIO,  1=EPWM4B,  2=SCIRX-A,  3=Resv
//    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;      // 1=OUTput,  0=INput
//    //GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;    // uncomment if --> Set Low initially
//    GpioDataRegs.GPASET.bit.GPIO5 = 1;      // uncomment if --> Set High initially
//
//
//    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;     // 0=GPIO,  1=EPWM3B,  2=Resv,  3=ECAP1
//    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;      // 1=OUTput,  0=INput
//    //GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;    // uncomment if --> Set Low initially
//    GpioDataRegs.GPASET.bit.GPIO7 = 1;      // uncomment if --> Set High initially


    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A

    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B


    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A

    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO6 (EPWM4B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO6 as EPWM4A

    EDIS;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    PWM_2ChCntUpDownBoostCfg(pwmA, freq, 0, 0, 1);   // PWM1 Inverter is setting as master,
                                                     // all others must set as slave
    PWM_2ChCntUpDownBoostCfg(pwmB, freq, 0, 0, 2);   // PWM1 Inverter is setting as master,

    COMP_BoosterTripZoneConfig((struct COMP_REGS*)&Comp2Regs,
                               (uint16_t)(1023 * (float)Boost_SC_Protect_Value *
                               (float)Adc_Booster_Shunt_Volt_Rat / (float)Adc_Reference_Volt));

    /* COMP_BoosterTripZoneConfig((struct COMP_REGS*)&Comp2Regs,
                               _IQ20int(_IQ20(1023 * 1.3 / Adc_Reference_Volt))); */

    PWM_BoosterConfigTripZone(pwmA);
    PWM_BoosterConfigTripZone(pwmB);

//    EALLOW;
//    pwmA->ETSEL.bit.INTEN = 1;              // Enable INT
//    pwmA->ETPS.bit.INTPRD = ET_2ND;
//    pwmA->ETSEL.bit.INTSEL = ET_CTR_ZERO;
//    EDIS;

    PieCtrlRegs.PIEIER2.bit.INTx3   = 1;    // epwm3 TZ interrupt
//    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;      // epwm3 interrupt

    EALLOW;            // This is needed to write to EALLOW protected registers

    PieVectTable.EPWM3_TZINT        = &epwm3_tzint_isr;
//    PieVectTable.EPWM3_INT          = &epwm3_isr;

    EDIS;               // This is needed to disable write to EALLOW protected registers

    IER |= M_INT2;
//    IER |= M_INT3;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

}

#endif


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
BRet Boost_Stop(SBooster *pBst) {
    pBst->eState = BS_IDLE;
    PWM_2ChUpDownBoostSetDuty(pBst->pwmAHandle, 0);
    PWM_2ChUpDownBoostSetDuty(pBst->pwmBHandle, 0);
    return BR_OK;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
BRet Boost_Start(SBooster *pBst) {
    pBst->eState = BS_RUNNING;
    Boost_Set(pBst, _IQ24(Boost_Start_Percen_F));
    return BR_OK;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
//BRet Boost_Process(SBooster *pBst, _iq measVoltage) {
//    _iq duty = 0;
//#if Boost_Method_Used == Boost_Method_Open_Loop
//    duty = _IQdiv(pBst->bCoeff - measVoltage, pBst->aCoeff);
//#elif Boost_Method_Used == Boost_Method_Close_Loop
//    PID_ProcessM(&pBst->sPid, pBst->setVolt, measVoltage);
//    duty = pBst->sPid.PIDOut;
//#endif
//
//    duty = MIN(duty, pBst->dutyMaxPer);
//    Boost_Set(pBst, duty);
//    return BR_OK;
//}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */


