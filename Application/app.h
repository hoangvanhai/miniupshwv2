/** @FILE NAME:    app.h
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
#ifndef APPLICATION_APP_H_
#define APPLICATION_APP_H_


/***************************** Include Files *********************************/
#include <booster.h>
#include <inverter.h>
#include <SineGen.h>
#include <utils.h>
#include <console.h>
#include <MATH_EMAVG_IQ_C.h>
/************************** Constant Definitions *****************************/

#define ADC_NUM_CHAN_MAX        16
#define ADC_NUM_CHAN_USED       7
#define ADC_NUM_SAMP_HOLD       10
/**************************** Type Definitions *******************************/

typedef enum EDeviceState_ {
    DS_RUN_UPS           = 0x0001,
    DS_STATE_UPS         = 0x0002,
    DS_AC_AVAIL          = 0x0004,
    DS_BATT_VOLT_LOW     = 0x0008,
    DS_BATT_VOLT_HIGH    = 0x0010,
    DS_BOOST_VOLT_LOW    = 0x0020,
    DS_BOOST_VOLT_HIGH   = 0x0040,
    DS_ERR_BOOST_CURR    = 0x0080,
    DS_INV_CURR_OVER_1   = 0x0100,
    DS_INV_CURR_OVER_2   = 0x0200,
    DS_ERR_LOAD_VOLT     = 0x0400,
    DS_ERR_UNKNOWN       = 0x0000
}EDeviceState;



typedef enum EBattStatus_ {
    BATT_NORMAL = 0,
    BATT_LOW_VOLT,
    BATT_OVER_VOLT
}EBattStatus;

typedef enum EAcInStatus_ {
    ACIN_LOSS = 0,
    ACIN_AVAIL
}EAcInStatus;

typedef enum ELoadStatus_ {
    LOAD_NORMAL = 0,
    LOAD_ERROR
}ELoadStatus;

typedef enum EDeviceSM_ {
    DSM_STOP_UPS = 0,
    DSM_STARTING_BOOSTER,
    DSM_STARTING_INVERTER,
    DSM_RUNNING_UPS
}EDeviceSM;

typedef struct SAdcValue_{
    uint8_t         type;
    BOOL            isSmpWait;
    BOOL            isLocked;
    uint8_t         smpNum;
    uint16_t        smpQueue[ADC_NUM_SAMP_HOLD];
    uint16_t        count;
    uint16_t        maxCount;
    uint32_t        totalValue;
    uint16_t        avgValue;
    uint16_t        currValue;
    uint16_t        offset;
    _iq             realValue;
    _iq             coeff;
    SMATH_EMAVG_IQ_C sEMA;
}SAdcValue;

typedef struct SApp_ {
    EDeviceState    eDevState;
    EDeviceSM       eDevSm;

    SBooster        sBooster;
    SInverter       sInverter;

    uint16_t        counterAdc;
    uint16_t        counterCtrl;

    SAdcValue       battVolt;
    _iq             lastBattVolt;
    _iq             maxBattVolt;
    _iq             minBattVolt;

    SAdcValue       boostVolt;
    _iq             maxBoostVolt;
    _iq             minBoostVolt;
    _iq             lastBoostVolt;

    SAdcValue       boostCurr;
    _iq             maxBoostCurr;

    SAdcValue       inverterCurr;
    _iq             maxInverterCurr;
    _iq             maxInverterCurr2;

    SAdcValue       lineDetectVolt;
    _iq             maxLineDetectVolt;

    SAdcValue       loadDetectVolt;
    _iq             maxLoadDetectVolt;

    BOOL            shortCurrent;
    BOOL            overCurrent1;
    BOOL            overCurrent2;

    BOOL            bDevLocked;     // device is locked by error in operate

    int16_t         numRAOCL;       // times restart after overcurrent
    int16_t         numTripOccurs;  // inverter current trip

    void            *hTimerControl;
    void            *hTimerProtectOverCurrent;
    void            *hTimerProtectTripCurrent;
}SApp;

/***************** Macros (Inline Functions) Definitions *********************/
#define App_BoostStart(pApp)            Boost_Start(&((pApp)->sBooster))
#define App_BoostStop(pApp)             Boost_Stop(&((pApp)->sBooster))
#define App_InvStart(pApp)              Inv_Start(&((pApp)->sInverter))
#define App_InvStop(pApp)               Inv_Stop(&((pApp)->sInverter))

#define App_SetDevState(pApp, state)    (pApp)->eDevState |= (state)
#define App_ClearDevState(pApp, state)  (pApp)->eDevState &= ~(state)


#define Adc_PushAdcValue(adc, value) {                                                          \
                                            if(adc.isLocked == FALSE) {                         \
                                                adc.isLocked = TRUE;                            \
                                                adc.smpQueue[adc.smpNum++] = value;             \
                                                if(adc.smpNum >= ADC_NUM_SAMP_HOLD) {           \
                                                    adc.smpNum = 0;                             \
                                                }                                               \
                                                adc.isSmpWait = TRUE;                           \
                                                adc.isLocked = FALSE;                           \
                                                }                                               \
                                            }

#define Adc_CalcRealValueIsr(adc, value)       { \
                                            if(adc.type == 0) { \
                                                if(value <= adc.offset) adc.sEMA.In = 0; \
                                                else adc.sEMA.In = _IQ20(value - adc.offset); \
                                                MATH_EMAVG_IQ_C(adc.sEMA); \
                                                adc.realValue = _IQ20mpyIQX(adc.coeff, 24, adc.sEMA.Out, 20); \
                                            } else { \
                                                adc.count++; \
                                                adc.currValue = value;  \
                                                adc.totalValue += value; \
                                                if(adc.count >= adc.maxCount) \
                                                { \
                                                    adc.avgValue = (adc.totalValue / adc.maxCount) \
                                                    if(adc.avgValue <  adc.offset) adc.avgValue = 0; \
                                                    else adc.avgValue -= adc.offset; \
                                                    adc.realValue = _IQ20mpyIQX(_IQ20(adc.avgValue), 20, adc.coeff, 24); \
                                                    adc.count = 0; \
                                                    adc.totalValue = 0;\
                                                } \
                                            } \
                                            }


#define Adc_CalcRealValueBgrd(adc)       { \
                                            if((adc.isSmpWait == TRUE) && (adc.isLocked == FALSE)) {            \
                                            uint8_t i = 0;                                                  \
                                            adc.isLocked = TRUE;                                            \
                                            while(i < adc.smpNum) {                                         \
                                                if(adc.type == 0) {                                         \
                                                    if(adc.smpQueue[i] <= adc.offset) adc.sEMA.In = 0;      \
                                                    else adc.sEMA.In = _IQ20(adc.smpQueue[i] - adc.offset); \
                                                    MATH_EMAVG_IQ_C(adc.sEMA);                              \
                                                    adc.realValue = _IQ20mpyIQX(adc.coeff, 24, adc.sEMA.Out, 20); \
                                                } else {                                    \
                                                    adc.count++;                            \
                                                    adc.currValue = adc.smpQueue[i];                  \
                                                    adc.totalValue += adc.smpQueue[i];                \
                                                    if(adc.count >= adc.maxCount) {         \
                                                        adc.avgValue = (adc.totalValue / adc.maxCount); \
                                                        if(adc.avgValue <  adc.offset) adc.avgValue = 0; \
                                                        else adc.avgValue -= adc.offset; \
                                                        adc.realValue = _IQ20mpyIQX(_IQ20(adc.avgValue), 20, adc.coeff, 24); \
                                                        adc.count = 0; \
                                                        adc.totalValue = 0;\
                                                    }   \
                                                }       \
                                                i++;    \
                                            } \
                                            adc.smpNum = 0;             \
                                            adc.isSmpWait = FALSE;      \
                                            adc.isLocked = FALSE;       \
                                            } }

#define Adc_InitValue(adc, typ, coe, cnt, offs, fc)          {      \
                                            adc.type = typ;       \
                                            adc.isSmpWait = FALSE; \
                                            adc.isLocked = FALSE; \
                                            adc.smpNum = 0;       \
                                            adc.count = 0;        \
                                            adc.currValue = 0;    \
                                            adc.totalValue = 0;   \
                                            adc.maxCount = cnt;   \
                                            adc.avgValue = 0;     \
                                            adc.offset = offs;    \
                                            adc.realValue = 0;    \
                                            adc.coeff = coe;      \
                     MATH_EMAVG_IQ_C_INIT(adc.sEMA, _IQ30(2*PI* (fc) / (Inverter_Pwm_Freq / 2))); \
                                            }



#define App_StartBstInv(pApp)       { \
                                        Inv_Start(&(pApp)->sInverter);      \
                                        Boost_Start(&(pApp)->sBooster);}

#define App_StopBstInv(pApp)        { \
                                        Boost_Stop(&(pApp)->sBooster);      \
                                        Inv_Stop(&(pApp)->sInverter); }

#define App_StopUps(pApp)           { \
                                    App_StopBstInv(pApp); \
                                    GPIO_SET_LOW_DISP_UPS_RUN(); \
                                    GPIO_SET_LOW_CTRL_RELAY();  \
                                    (pApp)->eDevSm = DSM_STOP_UPS; \
                                    ASSERT(0); \
                                    }

/************************** Function Prototypes ******************************/
void App_Init(SApp *pApp);
void App_ScanDigitalInput(SApp *pApp);
void App_ScanAnalogInput(SApp *pApp);
void App_ProcessInput(SApp *pApp);
void App_Control(SApp *pApp);

/************************** Variable Definitions *****************************/
extern SApp sApp;

extern int     ChSel[ADC_NUM_CHAN_MAX];
extern int     TrigSel[ADC_NUM_CHAN_MAX];
extern int     ACQPS[ADC_NUM_CHAN_MAX];
extern volatile uint16_t Result[ADC_NUM_CHAN_USED];
/*****************************************************************************/




#endif /* APPLICATION_APP_H_ */
