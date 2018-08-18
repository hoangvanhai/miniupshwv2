/** @FILE NAME:    isr.c
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
#include <BSP.h>
#include <DSP28x_Project.h>
#include <Timer.h>
#include <app.h>
#include <SineGen.h>

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

__interrupt void epwm1_isr(void)
{

    GPIO_SET_HIGH_DISP_BATT_LOW();
    // Control all UPS behaviour
    //App_ProcessInput(&sApp);

    //static uint16_t counter = 0;

//    Adc_CalcRealValueIsr(sApp.battVolt,        AdcResult.ADCRESULT0);
//    Adc_CalcRealValueIsr(sApp.boostVolt,       AdcResult.ADCRESULT1);
//    Adc_CalcRealValueIsr(sApp.inverterCurr,    AdcResult.ADCRESULT2);
//    Adc_CalcRealValueIsr(sApp.boostCurr,       AdcResult.ADCRESULT3);
//    Adc_CalcRealValueIsr(sApp.lineDetectVolt,  AdcResult.ADCRESULT4);
//    Adc_CalcRealValueIsr(sApp.loadDetectVolt,  AdcResult.ADCRESULT5);


    Adc_PushAdcValue(sApp.battVolt,        AdcResult.ADCRESULT0);
    Adc_PushAdcValue(sApp.boostVolt,       AdcResult.ADCRESULT1);
    Adc_PushAdcValue(sApp.inverterCurr,    AdcResult.ADCRESULT2);
    Adc_PushAdcValue(sApp.boostCurr,       AdcResult.ADCRESULT3);
    Adc_PushAdcValue(sApp.lineDetectVolt,  AdcResult.ADCRESULT5);
    Adc_PushAdcValue(sApp.loadDetectVolt,  AdcResult.ADCRESULT6);

    /*
     * Control duty cycle
     */

#if 1
    if(sApp.sInverter.eState == INV_RUNNING)
    {

        Sin_GenValueM(&sApp.sInverter.sSine1Phase);
        sApp.sInverter.pwm1Handle->CMPA.half.CMPA =
                _IQ24mpy(sApp.sInverter.sSine1Phase.sinPwmA,
                         sApp.sInverter.pwm1Handle->TBPRD>>1);

        sApp.sInverter.pwm2Handle->CMPA.half.CMPA =
                _IQ24mpy(sApp.sInverter.sSine1Phase.sinPwmB,
                         sApp.sInverter.pwm2Handle->TBPRD>>1);

    }

    if(sApp.sBooster.eState == BS_RUNNING) {
        Boost_Process(&sApp.sBooster, sApp.boostVolt.realValue, sApp.boostCurr.realValue);
    }
#endif



    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    GPIO_SET_LOW_DISP_BATT_LOW();
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

__interrupt void epwm2_isr(void)
{

    EPwm2Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

__interrupt void epwm3_isr(void)
{
    static uint16_t counter = 0;
    GPIO_SET_HIGH_DISP_ACIN();
    if(counter++ >= 4) {
        counter = 0;
        if(sApp.eDevState == DS_RUN_UPS) {
            //Boost_Process(&sApp.sBooster, sApp.boostVolt.realValue, sApp.boostCurr.realValue);
        }
    }

    EPwm3Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    GPIO_SET_LOW_DISP_ACIN();
}



/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

__interrupt void epwm4_isr(void)
{

    EPwm4Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
__interrupt void epwm1_tzint_isr(void)
{
    LREP("Trip Zone 1\r\n");
    /**
     * because use EPwmxRegs.TZCLR.bit.INT; cause other interrupt do not work properly
     * so not recommend use the statement frequently
     */

    EALLOW;
    EPwm1Regs.TZCLR.bit.CBC     = 1;
    //EPwm1Regs.TZCLR.bit.INT     = 1;
    EPwm1Regs.TZCLR.bit.DCAEVT2 = 1;
    EDIS;


    if(sApp.numTripOccurs++ >= APP_NUM_TRIP_TO_LOCK) {
        sApp.bDevLocked = TRUE;
        App_StopUps(&sApp);
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;


}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
__interrupt void epwm2_tzint_isr(void)
{

    LREP("Trip Zone 2\r\n");
    EALLOW;
    EPwm2Regs.TZFRC.bit.CBC     = 1;
    //EPwm2Regs.TZCLR.bit.INT     = 1;
    EPwm2Regs.TZCLR.bit.DCAEVT2 = 1;
    EDIS;

    //App_StopUps(&sApp);


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
__interrupt void epwm3_tzint_isr(void)
{
    LREP("Trip Zone 3\r\n");
    /**
     * because use EPwmxRegs.TZCLR.bit.INT; cause other interrupt do not work properly
     * so not recommend use the statement frequently
     */

    EALLOW;
    EPwm3Regs.TZCLR.bit.CBC     = 1;
    //EPwm3Regs.TZCLR.bit.INT     = 1;
    EPwm3Regs.TZCLR.bit.DCAEVT2 = 1;
    EDIS;


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
__interrupt void epwm4_tzint_isr(void)
{
    LREP("Trip Zone 4\r\n");
    /**
     * because use EPwmxRegs.TZCLR.bit.INT; cause other interrupt do not work properly
     * so not recommend use the statement frequently
     */

    EALLOW;
    EPwm4Regs.TZCLR.bit.CBC     = 1;
    EPwm4Regs.TZCLR.bit.INT     = 1;
    EPwm4Regs.TZCLR.bit.DCAEVT2 = 1;
    EDIS;


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;


}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
__interrupt void cpu_timer0_isr(void)
{
    GPIO_SET_HIGH_DISP_UPS_RUN();
    CpuTimer0.InterruptCount++;
    sSysTick.u32SystemTickCount++;
    Timer_Update();
    App_Control(&sApp);
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GPIO_SET_LOW_DISP_UPS_RUN();
}

/*****************************************************************************/
/** @brief for high voltage, low current application
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    /* Control ups behaviours */
//    App_Control(&sApp);
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
__interrupt void cpu_timer2_isr(void)
{
    CpuTimer2.InterruptCount++;
}

/*****************************************************************************/
/** @brief read all adc value on channel, this routine triggered by cpu timer 0
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
__interrupt void adc_isr(void)
{
#if 0
    GPIO_SET_HIGH_DISP_BATT_LOW();
    // Control all UPS behaviour
    //App_ProcessInput(&sApp);

    //static uint16_t counter = 0;
#if Build_Option == Build_Boost_Only     // only boost

//    Adc_CalcRealValueIsr(sApp.battVolt,        AdcResult.ADCRESULT0);
    Adc_CalcRealValueIsr(sApp.boostVolt,       AdcResult.ADCRESULT1);
//    Adc_CalcRealValueIsr(sApp.inverterCurr,    AdcResult.ADCRESULT2);
    Adc_CalcRealValueIsr(sApp.boostCurr,       AdcResult.ADCRESULT3);
//    Adc_CalcRealValueIsr(sApp.lineDetectVolt,  AdcResult.ADCRESULT4);
//    Adc_CalcRealValueIsr(sApp.loadDetectVolt,  AdcResult.ADCRESULT5);

#elif Build_Option == Build_Inverter_Fix  // only inverter
    //Adc_CalcRealValueIsr(pApp->battVolt,        AdcResult.ADCRESULT0);
    Adc_CalcRealValueIsr(pApp->boostVolt,       AdcResult.ADCRESULT1);
    Adc_CalcRealValueIsr(pApp->inverterCurr,     AdcResult.ADCRESULT2);
    //Adc_CalcRealValueIsr(pApp->boostCurr,       AdcResult.ADCRESULT3);
    //Adc_CalcRealValueIsr(pApp->lineDetectVolt,  AdcResult.ADCRESULT4);
    //Adc_CalcRealValueIsr(pApp->loadDetectVolt,  AdcResult.ADCRESULT5);


#elif Build_Option == Build_Inverter_All // all feature regardless AC and LOAD
    //Adc_CalcRealValueIsr(pApp->battVolt,        AdcResult.ADCRESULT0);
    Adc_CalcRealValueIsr(pApp->boostVolt,       AdcResult.ADCRESULT1);
    Adc_CalcRealValueIsr(pApp->inverterCurr,     AdcResult.ADCRESULT2);
    //Adc_CalcRealValueIsr(pApp->boostCurr,       AdcResult.ADCRESULT3);
    //Adc_CalcRealValueIsr(pApp->lineDetectVolt,  AdcResult.ADCRESULT4);
    //Adc_CalcRealValueIsr(pApp->loadDetectVolt,  AdcResult.ADCRESULT5);

#elif Build_Option == Build_Ups_All      // all device feature
    Adc_CalcRealValueIsr(pApp->battVolt,        AdcResult.ADCRESULT0);
    Adc_CalcRealValueIsr(pApp->boostVolt,       AdcResult.ADCRESULT1);
    Adc_CalcRealValueIsr(pApp->inverterCurr,    AdcResult.ADCRESULT2);
    Adc_CalcRealValueIsr(pApp->boostCurr,       AdcResult.ADCRESULT3);
    Adc_CalcRealValueIsr(pApp->lineDetectVolt,  AdcResult.ADCRESULT4);
    Adc_CalcRealValueIsr(pApp->loadDetectVolt,  AdcResult.ADCRESULT5);
#endif

    /*
     * Control duty cycle
     */

#if 1
    if(sApp.sInverter.eState == INV_RUNNING) {
        //counter++;
        //if(counter >= pApp->sInverter.genSinRatio)
        {
            //counter = 0;
            Sin_GenValueM(&sApp.sInverter.sSine1Phase);
            sApp.sInverter.pwm1Handle->CMPA.half.CMPA =
                    _IQ24mpy(sApp.sInverter.sSine1Phase.sinPwmA,
                             sApp.sInverter.pwm1Handle->TBPRD>>1);

            sApp.sInverter.pwm2Handle->CMPA.half.CMPA =
                    _IQ24mpy(sApp.sInverter.sSine1Phase.sinPwmB,
                             sApp.sInverter.pwm2Handle->TBPRD>>1);

        }
    }

#endif

    if(sApp.eDevState == DS_RUN_UPS) {
        Boost_Process(&sApp.sBooster, sApp.boostVolt.realValue);
    }

#endif
    // Clear ADCINT1 flag reinitialize for next SOC
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    // Acknowledge interrupt to PIE
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}











/*
 * END OF FILE
 */


