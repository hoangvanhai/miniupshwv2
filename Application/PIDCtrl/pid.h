/** @FILE NAME:    pid.h
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
#ifndef APPLICATION_PIDCTRL_PID_H_
#define APPLICATION_PIDCTRL_PID_H_


/***************************** Include Files *********************************/
#include <IQmathlib.h>
#include <typedefs.h>
#include <AppConfig.h>
/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

typedef struct STUN_ {
    _iq         Out;
    _iq         aCoeff;
    _iq         bCoeff;
}STUN;


typedef struct SPID_ {
    _iq         KP;
    _iq         KI;
    _iq         KD;
    _iq         currErr;
    _iq         prevErr;
    _iq         Integration;
    _iq         Derative;
    _iq         Proportional;
    _iq         PIDOut;
    STUN        kpTun;
}SPID;
/***************** Macros (Inline Functions) Definitions *********************/
#define PID_Reset(pid) {                        \
                        (pid)->currErr = 0;       \
                        (pid)->prevErr = 0;       \
                        (pid)->Integration = 0;   \
                        (pid)->Derative = 0;      \
                        (pid)->Proportional = 0;  \
                        (pid)->PIDOut = 0;        \
                      }


#define PID_Init(pid, Kp, Ki, Kd, F, aCo, bCo) {             \
    PID_Reset(pid);                                \
    (pid)->KP = _IQ24(Kp);                         \
    (pid)->KI = _IQ24((float)(Ki) * 0.5 * (1/(float)(F)) );         \
    (pid)->KD = _IQ24((float)(Kd) * (F));                 \
    (pid)->kpTun.aCoeff = _IQ24(aCo);  \
    (pid)->kpTun.bCoeff = _IQ24(bCo);  \
}



#define PID_ProcessM(pPid, setPoint, feedBackV, feedbackC) {                                       \
    (pPid)->currErr       = setPoint - feedBackV;                                                  \
    /*(pPid)->KP            = _IQ24mpyIQX((pPid)->kpTun.aCoeff, 24, feedbackC, 20) + (pPid)->kpTun.bCoeff; */ \
    (pPid)->Proportional  = _IQ24mpyIQX((pPid)->KP, 24, (pPid)->currErr, 18);                       \
    (pPid)->Integration  += _IQ24mpyIQX((pPid)->KI, 24, (pPid)->currErr + (pPid)->prevErr, 18);     \
    (pPid)->Derative      = _IQ24mpyIQX((pPid)->KD, 24, (pPid)->currErr - (pPid)->prevErr, 18);     \
    (pPid)->prevErr       = (pPid)->currErr;                                                        \
    (pPid)->PIDOut        = (pPid)->Proportional + (pPid)->Integration + (pPid)->Derative;          \
}

/************************** Function Prototypes ******************************/
void PID_Process(SPID *pPid, _iq setPoint, _iq feedBack);

/************************** Variable Definitions *****************************/

/*****************************************************************************/



#endif /* APPLICATION_PIDCTRL_PID_H_ */
