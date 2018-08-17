/** @FILE NAME:    booster.h
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

#ifndef BOOSTER_BOOSTER_H_
#define BOOSTER_BOOSTER_H_


/***************************** Include Files *********************************/
#include <IQMathlib.h>
#include <typedefs.h>
#include <DSP28x_Project.h>
#include <BSP.h>
#include <pid.h>
#include <console.h>

/************************** Constant Definitions *****************************/
#define KP_A_COEFF       (0.012)
#define KP_B_COEFF       (0.01)
/**************************** Type Definitions *******************************/
typedef enum BoostRet_ {
    BR_OK = 0,
    BR_FAIL
}BRet;

typedef enum EBoostState_ {
    BS_IDLE = 0,
    BS_RUNNING
}EBoostState;



typedef struct SBooster_ {
    EBoostState eState;
    PWM_REGS    *pwmAHandle;
    PWM_REGS    *pwmBHandle;
    uint16_t    period;
    uint16_t    dutyValue;          // duty value
    _iq         periodIQ;
    _iq         dutyMaxPer;
    _iq         dutyCurrPer;
    _iq         setVolt;
    SPID        sPid;
}SBooster;
/***************** Macros (Inline Functions) Definitions *********************/
#define Boost_SetState(pBst, state)     (pBst)->eState = state

#define PWM_2ChUpDownBoostSetDuty(pwm, duty) { \
    /*LREP("SET DUTY %d\r\n", (long)duty); */ \
    if((duty) < (pwm)->TBPRD) { \
        (pwm)->CMPA.half.CMPA = (duty);   \
        (pwm)->CMPB = (pwm)->TBPRD - (duty);  \
    } \
}

#define Boost_Apply(pBst) { \
    PWM_2ChUpDownBoostSetDuty((pBst)->pwmAHandle, (pBst)->dutyValue); \
    PWM_2ChUpDownBoostSetDuty((pBst)->pwmBHandle, (pBst)->dutyValue); \
}

#define Boost_Set(pBst, percen) { \
    (pBst)->dutyValue = _IQ20int(_IQ20mpyIQX((percen), 24, (pBst)->periodIQ, 20)); \
    Boost_Apply(pBst); \
}


#define Boost_Process(pBst, measVoltage, measCurr) { \
    PID_ProcessM(&(pBst)->sPid, (pBst)->setVolt, measVoltage, measCurr);\
    (pBst)->dutyCurrPer = (pBst)->sPid.PIDOut;\
    (pBst)->dutyCurrPer = MIN((pBst)->dutyCurrPer, (pBst)->dutyMaxPer);\
    (pBst)->dutyCurrPer = MAX((pBst)->dutyCurrPer, 0);\
    Boost_Set((pBst), (pBst)->dutyCurrPer);   \
}

#define Boost_SetLevOut(pBst, lev) {  \
    (pBst)->setVolt = lev;    \
}

/************************** Function Prototypes ******************************/
BRet Boost_Init (SBooster *pBst);
BRet Boost_Stop(SBooster *pBst);
BRet Boost_Start(SBooster *pBst);
void PWM_Boost_Init(PWM_REGS * pwmA, PWM_REGS * pwmB, uint32_t freq, uint16_t phase);

/************************** Variable Definitions *****************************/

/*****************************************************************************/


#endif /* BOOSTER_BOOSTER_H_ */
