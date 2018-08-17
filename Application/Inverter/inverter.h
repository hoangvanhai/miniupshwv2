/** @FILE NAME:    inverter.h
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

#ifndef INVERTER_INVERTER_H_
#define INVERTER_INVERTER_H_


/***************************** Include Files *********************************/
#include <IQMathlib.h>
#include <typedefs.h>
#include <DSP28x_Project.h>
#include <BSP.h>
#include <SineGen.h>
#include <console.h>
/************************** Constant Definitions *****************************/
// for [50 : 60] -> 34VAC
//#define A_COEFF         62.3917748105776
//#define B_COEFF         110.0
// for [320 : 350] -> 220VAC
#define A_COEFF         -359.981634058608
#define B_COEFF         670.0
// for [160 : 300] -> 100VAC
//#define A_COEFF         (-339.981634058608)
//#define B_COEFF         460

/**************************** Type Definitions *******************************/
typedef enum EInverterMode_ {
    INV_SET_HALF_HIGH = 0,
    INV_SET_HALF_LOW
}EInverterMode;

typedef enum EInverterState_ {
    INV_IDLE = 0,
    INV_RUNNING,
    INV_ERROR
}EInverterState;

typedef struct SInverter_ {
    EInverterState  eState;
    EInverterMode   eMode;
    PWM_REGS        *pwm1Handle;
    PWM_REGS        *pwm2Handle;
    SSin1Phase      sSine1Phase;
    uint16_t        genSinRatio;
    uint32_t        freq;           // pwm freq
    uint16_t        period;         // pwm period
    uint16_t        dutyValue;          // duty value
    _iq             currFbFact;
    _iq             currGain;
    _iq             gainMax;
    _iq             gainStep;
    _iq             aCoeff;
    _iq             bCoeff;
}SInverter;

/***************** Macros (Inline Functions) Definitions *********************/
#define Inv_SetMode(pInv, mode)     (pInv)->eMode = mode
#define Inv_SetState(pInv, state)   (pInv)->eState = state
#define Inv_GetGain(pInv)           (pInv)->sSine1Phase.gain
#define Inv_GetGainStep(pInv)       (pInv)->gainStep

#define Inv_SetGain(pInv, val)        { \
                                    (pInv)->sSine1Phase.gain = val; \
                                    }
/*
 Function to calculate value of duty for stable voltage output:

 y = -62.3917748105776*x + 110
 -> x = (110 - y) /  62.3917748105776
 -> duty = (110 - boostVolt) / 62.3917748105776

*/
/************************** Function Prototypes ******************************/
void Inv_Init(SInverter *pInv);
void Inv_Start(SInverter *pInv);
void Inv_Stop(SInverter *pInv);
void Inv_Set(SInverter *pInv, uint16_t chan, _iq percen);
void PWM_Inv_Init(PWM_REGS * pwm1, PWM_REGS * pwm2, uint32_t freq);


/************************** Variable Definitions *****************************/
extern SInverter *pThisInv;
/*****************************************************************************/


#endif /* INVERTER_INVERTER_H_ */
