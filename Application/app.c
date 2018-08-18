/** @FILE NAME:    app.c
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
#include <app.h>
#include <BSP.h>
#include <Timer.h>
/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/



/************************** Function Prototypes ******************************/
static void Clb_AppControl(uint32_t tick, void *p_arg);
static void Clb_AppProtectOverCurrent(uint32_t tick, void *p_arg);
static void Clb_AppProtectTripCurrent(uint32_t tick, void *p_arg);
/************************** Variable Definitions *****************************/
SApp sApp;

int             ChSel[ADC_NUM_CHAN_MAX] =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int             TrigSel[ADC_NUM_CHAN_MAX] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int             ACQPS[ADC_NUM_CHAN_MAX] =   {6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6};
long max  = -32767, min = 32767, tmp = 0;

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void App_Init(SApp *pApp) {
    // init state for device
    uint32_t devStateInit = 0;
    // application timer
    Timer_Init();
    // init inverter
    Inv_Init(&pApp->sInverter);
    // init booster
    Boost_Init(&pApp->sBooster);
    // init app instance
    pApp->counterAdc = 0;
    pApp->counterCtrl = 0;
    pApp->numRAOCL = APP_NUM_RA_OVER_CURR;
    pApp->bDevLocked = FALSE;
    pApp->numTripOccurs = 0;
    pApp->eDevState = DS_ERR_UNKNOWN;

    pApp->eDevSm = DSM_STOP_UPS;

    // init timer for control ups behaviours
    pApp->hTimerControl = Timer_Create(Clb_AppControl, pApp);
    Timer_SetRate(pApp->hTimerControl, 50000);

    // timer for protect over current event
    pApp->hTimerProtectOverCurrent = Timer_Create(Clb_AppProtectOverCurrent, pApp);
    Timer_SetRate(pApp->hTimerProtectOverCurrent, 50000);

    // timer for protect trip current
    pApp->hTimerProtectTripCurrent = Timer_Create(Clb_AppProtectTripCurrent, pApp);
    Timer_SetRate(pApp->hTimerProtectTripCurrent, 50000);


    // init input caclculating
    Adc_InitValue(pApp->battVolt,                   /* adc node */
                  1,                                /* type adc node 1 avg 0 filter  */
                  _IQ24(Batt_Volt_Adc_Coeff),       /* coefficient for calculate realvalue */
                  1000,                               /* average counter */
                  10,                               /* offset value */
                  Adc_Noise_Cuttoff_Freq);          /* cuttoff freq */

    Adc_InitValue(pApp->boostVolt,
                  0,
                  _IQ24(Boost_Voltage_Adc_Coeff),
                  170,
                  10,
                  Adc_Noise_Cuttoff_Freq);

    Adc_InitValue(pApp->boostCurr,
                  1,
                  _IQ24(Boost_Current_Adc_Coeff),
                  2048,
                  10,
                  Adc_Noise_Cuttoff_Freq);

    Adc_InitValue(pApp->inverterCurr,
                  1,
                  _IQ24(Adc_Inverter_Current_Rat),
                  500,
                  10,
                  Adc_Noise_Cuttoff_Freq);

    Adc_InitValue(pApp->lineDetectVolt,
                  1,
                  _IQ24(0.001),
                  800,
                  10,
                  Adc_Noise_Cuttoff_Freq);

    Adc_InitValue(pApp->loadDetectVolt,
                  1,
                  _IQ24(0.001),
                  1100,
                  10,
                  Adc_Noise_Cuttoff_Freq);


    pApp->maxBattVolt           = _IQ20(Batt_Over_Volt);
    pApp->minBattVolt           = _IQ20(Batt_Under_Volt);
    pApp->maxBoostVolt          = _IQ20(Inverter_Dcbus_Over);
    pApp->minBoostVolt          = _IQ20(Inverter_Dcbus_Under);

    pApp->maxBoostCurr          = _IQ20(Boost_Trip_Curr);
    pApp->maxInverterCurr       = _IQ20(Inverter_OCL_Cur_Value);
    pApp->maxInverterCurr2      = _IQ20(Inverter_OCS_Cur_Value);
    pApp->maxLineDetectVolt     = _IQ20(AC_Detect_Level);
    pApp->maxLoadDetectVolt     = _IQ20(AC_Detect_Level);

    pApp->lastBoostVolt         = 0;
    pApp->lastBattVolt          = 0;

    pApp->shortCurrent          = FALSE;
    pApp->overCurrent1          = FALSE;
    pApp->overCurrent2          = FALSE;

#if Build_Option == Build_Boost_Only
    pApp->battVolt.realValue            = pApp->maxBattVolt - _IQ20(1);
    pApp->boostCurr.realValue           = _IQ20(5);    //pApp->maxBoostCurr - _IQ20(200) ;
    pApp->boostVolt.realValue           = 0;    //pApp->maxBoostVolt - _IQ20(5) ;
    pApp->inverterCurr.realValue        = pApp->maxInverterCurr - _IQ20(10);
    pApp->lineDetectVolt.realValue      = pApp->maxLineDetectVolt + _IQ20(1);
    //pApp->loadDetectVolt.realValue      = pApp->maxLoadDetectVolt - _IQ(1);
    devStateInit = DS_RUN_UPS;
#elif Build_Option == Build_Inverter_Fix
    pApp->battVolt.realValue            = pApp->maxBattVolt - _IQ20(1);
    pApp->boostCurr.realValue           = pApp->maxBoostCurr - _IQ20(200) ;
    pApp->boostVolt.realValue           = pApp->maxBoostVolt - _IQ20(5) ;
    pApp->inverterCurr.realValue        = pApp->maxInverterCurr - _IQ20(10);
    pApp->lineDetectVolt.realValue      = pApp->maxLineDetectVolt + _IQ20(1);
    //pApp->loadDetectVolt.realValue    = pApp->maxLoadDetectVolt - _IQ(1)
    devStateInit = DS_RUN_UPS;
#elif Build_Option == Build_Inverter_All
    pApp->battVolt.realValue            = pApp->maxBattVolt - _IQ20(1);
    pApp->boostCurr.realValue           = pApp->maxBoostCurr - _IQ20(200) ;
    pApp->boostVolt.realValue           = pApp->maxBoostVolt - _IQ20(5) ;
    pApp->inverterCurr.realValue        = pApp->maxInverterCurr - _IQ20(10);
    pApp->lineDetectVolt.realValue      = pApp->maxLineDetectVolt + _IQ20(1);
    //pApp->loadDetectVolt.realValue    = pApp->maxLoadDetectVolt - _IQ(1)
#elif Build_Option == Build_Ups_All
    pApp->battVolt.realValue            = 0;
    pApp->boostCurr.realValue           = 0;
    pApp->boostVolt.realValue           = 0;
    pApp->inverterCurr.realValue        = 0;
    pApp->lineDetectVolt.realValue      = 0;
    pApp->loadDetectVolt.realValue      = 0;
    devStateInit =  DS_RUN_UPS | DS_AC_AVAIL |
                    DS_BATT_VOLT_LOW | DS_BOOST_VOLT_LOW |
                    DS_ERR_BOOST_CURR | DS_ERR_INV_CURR;    // | DS_ERR_LOAD_VOLT;
#endif

    App_SetDevState(pApp, devStateInit);


}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
static void Clb_AppControl(uint32_t tick, void *p_arg) {
    SApp *pApp = (SApp*)p_arg;
    (void)tick;

    if(pApp->eDevState == DS_RUN_UPS ||
       pApp->eDevState == (DS_RUN_UPS | DS_BOOST_VOLT_LOW)) {
        pApp->eDevSm = DSM_STARTING_BOOSTER;
        LREP("change to START BOOSTER\r\n");
    }

    if(pApp->eDevState & DS_INV_CURR_OVER_1) {
        LREP("Clear over current 1\r\n");
        App_ClearDevState(pApp, DS_INV_CURR_OVER_1);
        pApp->overCurrent1 = FALSE;
    }

    if(pApp->eDevState & DS_INV_CURR_OVER_2) {
        LREP("Clear over current 2\r\n");
        App_ClearDevState(pApp, DS_INV_CURR_OVER_2);
        pApp->overCurrent2 = FALSE;
    }

    Timer_Stop(pApp->hTimerControl);
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
static void Clb_AppProtectOverCurrent(uint32_t tick, void *p_arg) {
    SApp *pApp = (SApp *)p_arg;
    (void)tick;

    if(pApp->eDevState & DS_INV_CURR_OVER_1) {
        pApp->overCurrent1 = TRUE;
    }
    if(pApp->eDevState & DS_INV_CURR_OVER_2) {
        pApp->overCurrent2 = TRUE;
    }

    pApp->numRAOCL--;
    if(pApp->numRAOCL > 0) {
        LREP("start restart timer\r\n");
        Timer_Stop(pApp->hTimerControl);
        Timer_StartAt(pApp->hTimerControl, APP_TIME_DA_OVER_CURR);
    }

    Timer_Stop(pApp->hTimerProtectOverCurrent);

    App_StopUps(pApp);
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
static void Clb_AppProtectTripCurrent(uint32_t tick, void *p_arg) {
    (void)tick;
    (void)p_arg;


}

/*****************************************************************************/
/** @brief
 *
 *#if 0
        long value;
        value = _IQ20int(pApp->boostVolt.realValue); //AdcResult.ADCRESULT1;
        if(max <  value)
        {
            max = value;
        }
        else if( min > value)
        {
            min = value;
        }
        tmp++;
        if (tmp == 5000)
        {
            LREP("[Max %d - Min %d]\r\n", (long)max, (long)min);
            tmp = 0;
            max = -32767;
            min = 32767;
        }

        return;
#endif
 *  @param
 *  @return Void.
 *  @note
 */
void App_ProcessInput(SApp *pApp) {

#if 0
    //static uint16_t counter = 0;
#if Build_Option == Build_Boost_Only     // only boost

    Adc_CalcRealValueIsr(pApp->battVolt,        AdcResult.ADCRESULT0);
    Adc_CalcRealValueIsr(pApp->boostVolt,       AdcResult.ADCRESULT1);
    Adc_CalcRealValueIsr(pApp->inverterCurr,    AdcResult.ADCRESULT2);
    Adc_CalcRealValueIsr(pApp->boostCurr,       AdcResult.ADCRESULT3);
    Adc_CalcRealValueIsr(pApp->lineDetectVolt,  AdcResult.ADCRESULT4);
    Adc_CalcRealValueIsr(pApp->loadDetectVolt,  AdcResult.ADCRESULT5);

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


    if(pApp->sInverter.eState == INV_RUNNING) {
        //counter++;
        //if(counter >= pApp->sInverter.genSinRatio)
        {
            //counter = 0;
            Sin_GenValueM(&pApp->sInverter.sSine1Phase);

            #if Inverter_Switching_Type == Inverter_Type_Open_Half
            if(lastSinValue > 0 && sinValue <= 0) {
                PWM_2ChCntUpSetDutyHalf(pApp->sInverter.pwm1Handle, 1, 0);
                PWM_2ChCntUpSetDutyHalf(pApp->sInverter.pwm2Handle, 2, 0);
            } else if(lastSinValue <= 0 && sinValue > 0){
                PWM_2ChCntUpSetDutyHalf(pApp->sInverter.pwm1Handle, 2, 0);
                PWM_2ChCntUpSetDutyHalf(pApp->sInverter.pwm2Handle, 1, 0);
            }

            duty = _IQint(_IQabs(_IQmpy(pApp->.sInverter.sSine1Phase.sinPwmA,
                                        pApp->sInverter.periodIQ)));

            if(sinValue > 0) {
                PWM_2ChCntUpSetDutyHalf(pApp->sInverter.pwm1Handle, 1, duty);
                PWM_2ChCntUpSetDutyHalf(pApp->sInverter.pwm2Handle, 2, duty);
            } else {
                PWM_2ChCntUpSetDutyHalf(pApp->sInverter.pwm1Handle, 2, duty);
                PWM_2ChCntUpSetDutyHalf(pApp->sInverter.pwm2Handle, 1, duty);
            }

            lastSinValue = sinValue;

            #elif Inverter_Switching_Type == Inverter_Type_Open_Full

            pApp->sInverter.pwm1Handle->CMPA.half.CMPA =
                    _IQ24mpy(pApp->sInverter.sSine1Phase.sinPwmA,
                             pApp->sInverter.pwm1Handle->TBPRD>>1);

            pApp->sInverter.pwm2Handle->CMPA.half.CMPA =
                    _IQ24mpy(pApp->sInverter.sSine1Phase.sinPwmB,
                             pApp->sInverter.pwm2Handle->TBPRD>>1);

            #endif
        }
    }

#endif

}


/*****************************************************************************/
/** @brief
    > Max            //
    protect 2
    > Lev2           //
    if not protect, protect 1
    else
    keep prev state
    > Lev2Shake  //
    protect 1
    > Lev1       //
    do nothing (keep prev state)
    > Lev1Shake  //
    normal operate
    > 0

 *  @param
 *  @return Void.
 *  @note
 */

void App_Control(SApp *pApp) {

    static _iq currVal = 0;
    /*
     * CHECK CONDITION AND CONTROL
     */
    if(pApp->bDevLocked == TRUE)
        return;

    if(pApp->numTripOccurs > 0)
        return;
    /*
     * Check battery voltage
     * if volt < Batt_Under_Volt or volt > Batt_Over_Volt and booster is running -> stop Booster
     * if volt > Batt_Under_Volt + Batt_Hyst_Volt and booster is stopping -> start booster
     */

//    if(pApp->battVolt.realValue <= pApp->minBattVolt) { // under low volt
//        // Set error low voltage
//        App_SetDevState(pApp, DS_BATT_VOLT_LOW);
//        //GPIO_SET_HIGH_DISP_BATT_LOW();
//    } else if(pApp->battVolt.realValue > pApp->minBattVolt &&
//            pApp->battVolt.realValue <= (pApp->minBattVolt + _IQ20(Batt_Hyst_Volt))) { // upper low volt but under Hyst
//
//        if(pApp->eDevState == DS_RUN_UPS) { //device is running -> keep running
//            // Clear all error flag
//
//        } else { // Set error low volt
//            App_SetDevState(pApp, DS_BATT_VOLT_LOW);
//            //GPIO_SET_HIGH_DISP_BATT_LOW();
//        }
//
//    } else if(pApp->battVolt.realValue > (pApp->minBattVolt+ _IQ20(Batt_Hyst_Volt)) &&
//            pApp->battVolt.realValue <= pApp->maxBattVolt){
//            // Clear battery error flag
//            App_ClearDevState(pApp, DS_BATT_VOLT_HIGH);
//            App_ClearDevState(pApp, DS_BATT_VOLT_LOW);
//
//    } else if(pApp->battVolt.realValue > pApp->maxBattVolt) {
//        // Set error over volt
//        App_SetDevState(pApp, DS_BATT_VOLT_HIGH);
//    }

    if(pApp->battVolt.realValue <= pApp->minBattVolt) {
        App_SetDevState(pApp, DS_BATT_VOLT_LOW);
    } else if(pApp->battVolt.realValue > pApp->maxBattVolt) {
        App_SetDevState(pApp, DS_BATT_VOLT_HIGH);
    } else {
        App_ClearDevState(pApp, DS_BATT_VOLT_LOW);
        App_ClearDevState(pApp, DS_BATT_VOLT_HIGH);
    }

    /*
     * Check AC available
     */
    if(pApp->lineDetectVolt.realValue > pApp->maxLineDetectVolt) { // Detect ac loss
        App_ClearDevState(pApp, DS_AC_AVAIL);
        //GPIO_SET_LOW_DISP_ACIN();
    } else {    // Detect ac available
        App_SetDevState(pApp, DS_AC_AVAIL);
        //GPIO_SET_HIGH_DISP_ACIN();
    }

    /*
     * Check boost voltage
     */
    if(pApp->boostVolt.realValue < pApp->minBoostVolt) { // Under voltage after boost
        // only check low volt boost when usp is running
        App_SetDevState(pApp, DS_BOOST_VOLT_LOW);
    } else if(pApp->boostVolt.realValue > pApp->maxBoostVolt) { // Over voltage after boost
        App_SetDevState(pApp, DS_BOOST_VOLT_HIGH);
        currVal = pApp->boostVolt.realValue;
    } else { // Normal voltage
        App_ClearDevState(pApp, DS_BOOST_VOLT_LOW);
        App_ClearDevState(pApp, DS_BOOST_VOLT_HIGH);
    }

    /*
     * Check boost current
     */
    if(pApp->boostCurr.realValue < pApp->maxBoostCurr) { // Normal current
        App_ClearDevState(pApp, DS_ERR_BOOST_CURR);
    } else { // Over current after boost
        App_SetDevState(pApp, DS_ERR_BOOST_CURR);
    }

    /*
     * Check inverter current
     */
    // normal operate
    if(pApp->inverterCurr.realValue < pApp->maxInverterCurr) {
        // 1.normal operate
        if(pApp->overCurrent1 == FALSE) {
            App_ClearDevState(pApp, DS_INV_CURR_OVER_1);
        }
        if(pApp->overCurrent2 == FALSE) {
            App_ClearDevState(pApp, DS_INV_CURR_OVER_2);
        }
        if(Timer_IsRunning(pApp->hTimerProtectOverCurrent)) {
            LREP("Stop protect\r\n");
            Timer_Stop(pApp->hTimerProtectOverCurrent);
        }
    } else if(pApp->inverterCurr.realValue >= pApp->maxInverterCurr &&
            (pApp->inverterCurr.realValue < pApp->maxInverterCurr +
            _IQ20(Inverter_Current_Offset))) {
        // 2. keep prev state
            LREP("Unstable current\r\n");
    } else if((pApp->inverterCurr.realValue >= pApp->maxInverterCurr +
            _IQ20(Inverter_Current_Offset)) &&
            pApp->inverterCurr.realValue < pApp->maxInverterCurr2) {
        // 3. protect 1
        if(pApp->eDevState != (DS_RUN_UPS | DS_INV_CURR_OVER_1)) {
            App_SetDevState(pApp, DS_INV_CURR_OVER_1);
            App_ClearDevState(pApp, DS_INV_CURR_OVER_2);
            LREP("start protect OCL timer on zone 3\r\n");
            Timer_StartAt(pApp->hTimerProtectOverCurrent, Inverter_OCL_Wait_Time);
        }

    } else if((pApp->inverterCurr.realValue >= pApp->maxInverterCurr +
            _IQ20(Inverter_Current_Offset)) &&
            pApp->inverterCurr.realValue < pApp->maxInverterCurr2) {
        // 4. if not protect -> start protect 1
        if(!Timer_IsRunning(pApp->hTimerProtectOverCurrent)) {
            App_SetDevState(pApp, DS_INV_CURR_OVER_1);
            App_ClearDevState(pApp, DS_INV_CURR_OVER_2);
            Timer_StartAt(pApp->hTimerProtectOverCurrent, Inverter_OCL_Wait_Time);
            LREP("start protect OCL timer on zone 4\r\n");
        } else { // else keep prev state

        }

    } else if(pApp->inverterCurr.realValue >=
            (pApp->maxInverterCurr2 + _IQ20(Inverter_Current_Offset))) {
        // 5. protect 2
        if(pApp->eDevState != (DS_RUN_UPS | DS_INV_CURR_OVER_2)) {
            App_ClearDevState(pApp, DS_INV_CURR_OVER_1);
            App_SetDevState(pApp, DS_INV_CURR_OVER_2);
            Timer_StartAt(pApp->hTimerProtectOverCurrent, Inverter_OCS_Wait_Time);
            LREP("start protect OCS timer \r\n");
        }

    }

    /*
     * Check Load out available
     */
//    if(pApp->loadDetectVolt.realValue > AC_Detect_Level) { // Not detect Load voltage
//        App_SetDevState(pApp, DS_ERR_LOAD_VOLT);
//    } else {
//        App_ClearDevState(pApp, DS_ERR_LOAD_VOLT);
//    }


    switch (pApp->eDevSm) {
    case DSM_STOP_UPS:
        // if not yet kick start UPS event
        if(pApp->eDevState == DS_RUN_UPS ||
           pApp->eDevState == (DS_RUN_UPS | DS_BOOST_VOLT_LOW)) {
            //LREP("APP STATE: 0x%x\r\n", (long)pApp->eDevState);
            if(!Timer_IsRunning(pApp->hTimerControl)) {
                // kick start
                Timer_StartAt(pApp->hTimerControl, Inverter_Soft_Start_Time);
                LREP("Soft start inverter\r\n");
            }
        } else { // while waitting starting time, error event occours
            if(Timer_IsRunning(pApp->hTimerControl) &&
                    !pApp->overCurrent1 && !pApp->overCurrent2) {
                LREP("stop soft start timer APP STATE: 0x%x\r\n", (long)pApp->eDevState);
                Timer_Stop(pApp->hTimerControl);
            }
        }

        break;
    case DSM_STARTING_BOOSTER:
        if(pApp->eDevState == DS_RUN_UPS ||
                pApp->eDevState == (DS_RUN_UPS | DS_BOOST_VOLT_LOW)) {
            //LREP("APP STATE: 0x%x\r\n", (long)pApp->eDevState);

            if(pApp->sBooster.eState != BS_RUNNING) {
                Boost_Start(&pApp->sBooster);
            }
            if(pApp->boostVolt.realValue > pApp->minBoostVolt &&
                    pApp->boostVolt.realValue < pApp->maxBoostVolt    ) {
                //GPIO_SET_HIGH_CTRL_RELAY();
                pApp->eDevSm = DSM_STARTING_INVERTER;
                //LREP("change to START INVERTER\r\n");
                //LREP("V: %d ", (long)_IQ20int(pApp->boostVolt.realValue));
            }
        } else {
            Boost_Stop(&pApp->sBooster);
            //GPIO_SET_LOW_CTRL_RELAY();
            pApp->eDevSm = DSM_STOP_UPS;
        }
        break;
    case DSM_STARTING_INVERTER:

        if(pApp->sInverter.eState != INV_RUNNING) {
            Inv_Start(&pApp->sInverter);
        }

        if(pApp->eDevState == DS_RUN_UPS                        ||
           pApp->eDevState == (DS_RUN_UPS | DS_INV_CURR_OVER_1) ||
           pApp->eDevState == (DS_RUN_UPS | DS_INV_CURR_OVER_2)) {
            // calculate gain on starting up inverter
            _iq gain = _IQ24div(pApp->sInverter.bCoeff - pApp->boostVolt.realValue, pApp->sInverter.aCoeff);

            gain = MIN(gain, pApp->sInverter.gainMax);

            if(Inv_GetGain(&pApp->sInverter) < _IQ24(Inverter_Max_Mf) && Inv_GetGain(&pApp->sInverter) < gain) {

                pApp->sInverter.currGain = Inv_GetGain(&pApp->sInverter) + Inv_GetGainStep(&pApp->sInverter);

                Inv_SetGain(&pApp->sInverter, pApp->sInverter.currGain);

                //LREP("inc gain = %d\r\n", _IQ24int(_IQ24mpy(currGain, _IQ24(100))));
            } else {
                LREP("change to RUNNING \r\n");
                pApp->eDevSm = DSM_RUNNING_UPS;
                //GPIO_SET_HIGH_DISP_UPS_RUN();
            }
        } else if(pApp->eDevState == DS_RUN_UPS ||
                pApp->eDevState == DS_RUN_UPS | DS_BOOST_VOLT_LOW) {
            Inv_Stop(&pApp->sInverter);
            //LREP("VLOW: %d ", (long)_IQ20int(pApp->boostVolt.realValue));
            pApp->eDevSm = DSM_STARTING_BOOSTER;
        } else {
            LREP("VOLT: %d\r\n", (long)_IQ20int(currVal));
            App_StopUps(pApp);
        }
        break;
    case DSM_RUNNING_UPS:

        // Keep running inverter
        if(pApp->eDevState == DS_RUN_UPS                        ||
           pApp->eDevState == (DS_RUN_UPS | DS_INV_CURR_OVER_1) ||
           pApp->eDevState == (DS_RUN_UPS | DS_INV_CURR_OVER_2)) {

            // update inverter sine value factor
            if(pApp->lastBoostVolt != pApp->boostVolt.realValue && pApp->numTripOccurs == 0) {
                _iq gain = 0;
                //putchar1('R');
                if(_IQabs(pApp->boostVolt.realValue - pApp->lastBoostVolt) > _IQ20(2)) {

                     pApp->sInverter.currGain = _IQ24div(pApp->sInverter.bCoeff - pApp->boostVolt.realValue,
                                      pApp->sInverter.aCoeff);

                    //_iq gain = _IQ(0.8);
                    pApp->lastBoostVolt = pApp->boostVolt.realValue;
                }

                gain = pApp->sInverter.currGain +
                        _IQ24mpyIQX(pApp->sInverter.currFbFact,
                                    24, pApp->inverterCurr.realValue, 18);

                gain = MIN(gain, pApp->sInverter.gainMax);
                Inv_SetGain(&pApp->sInverter, gain);
                //Inv_SetGain(&pApp->sInverter, _IQ24(0.9));
            }

        } else if(pApp->eDevState == DS_RUN_UPS ||
                pApp->eDevState == DS_RUN_UPS | DS_BOOST_VOLT_LOW) {
            Inv_Stop(&pApp->sInverter);
            pApp->eDevSm = DSM_STARTING_BOOSTER;
        } else { // Stop device

            LREP("VOLT: %d\r\n", (long)_IQ20int(currVal));
            App_StopUps(pApp);
            return;
        }


        break;

    default:
        break;
    }

}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
//void Adc_LogAdcValue(SAdcValue adc, uint16_t value) {
//    adc.smpQueue[adc.smpNum++] = value;
//    if(adc.smpNum >= ADC_NUM_SAMP_HOLD) {
//        adc.smpNum = 0;
//    }
//    adc.isSmpWait = TRUE;
//}


