/** @FILE NAME:    singen.h
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

#ifndef APPLICATION_SINEGEN_H_
#define APPLICATION_SINEGEN_H_


/***************************** Include Files *********************************/
#include <IQMathlib.h>
#include <typedefs.h>
/************************** Constant Definitions *****************************/
#define PI          (3.1415926535897932384626433832795)
#define ANGLE_120   (2*PI/3)
#define ANGLE_240   (4*PI/3)
#define ANGLE_360   (2*PI)

#define RAD_PER_DEG (0.0174532925199433)

/**************************** Type Definitions *******************************/

typedef struct Sine1Phase_ {
    uint32_t        freqSin;          // freq of sin wave
    uint32_t        freqGenSin;       // carier freq
    _iq             gain;             // gain of value
    _iq             offset;
    _iq             angleRadUnit;     // radian per degree
    _iq             angle120;         // 2PI/3
    _iq             angle240;         // 4PI/3
    _iq             angle360;         // 2PI
    _iq             angleA;
    _iq             angleB;
    _iq             angleC;
    _iq             sinPwmA;
    _iq             sinPwmB;
    _iq             sinPwmC;
}SSin1Phase;

/***************** Macros (Inline Functions) Definitions *********************/

#define Sin_GenValueM(pSin) { \
    (pSin)->angleA += (pSin)->angleRadUnit; \
    if((pSin)->angleA >= (pSin)->angle360) { \
        (pSin)->angleA = 0; \
    } \
    (pSin)->angleB = (pSin)->angleA + (pSin)->angle120; \
    if((pSin)->angleB >= (pSin)->angle360) { \
        (pSin)->angleB -= (pSin)->angle360; \
    } \
    (pSin)->angleC = (pSin)->angleA + (pSin)->angle240; \
    if((pSin)->angleC >= (pSin)->angle360) { \
        (pSin)->angleC -= (pSin)->angle360; \
    } \
    (pSin)->sinPwmA = _IQ24sin((pSin)->angleA); \
    (pSin)->sinPwmA = (pSin)->offset + _IQ24mpy((pSin)->sinPwmA, (pSin)->gain); \
    (pSin)->sinPwmB = _IQ24sin((pSin)->angleB); \
    (pSin)->sinPwmB = (pSin)->offset + _IQ24mpy((pSin)->sinPwmB, (pSin)->gain); \
    (pSin)->sinPwmC = _IQ24sin((pSin)->angleC); \
    (pSin)->sinPwmC = (pSin)->offset + _IQ24mpy((pSin)->sinPwmC, (pSin)->gain); \
}


#define SINE1PHASE_RESET(sin)  {        \
        (sin)->angleA = 0;                \
        (sin)->angleB = 0;                \
        (sin)->angleC = 0;                \
        (sin)->sinPwmA = 0;               \
        (sin)->sinPwmB = 0;               \
        (sin)->sinPwmC = 0;               \
}

#define Sin_SetGain(pSin, gain)       (pSin)->gain = gain
#define Sin_GetGain(pSin)             (pSin)->gain
/************************** Function Prototypes ******************************/

void Sin_GenValue(SSin1Phase *pSin);
void Sin_Init(SSin1Phase *pSin, uint32_t freqSin, uint32_t freqCarr, _iq stepRad, _iq gain, _iq offset);

/************************** Variable Definitions *****************************/

/*****************************************************************************/


#endif /* APPLICATION_SINEGEN_H_ */
