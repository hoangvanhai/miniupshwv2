/** @FILE NAME:    Timer.h
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
#ifndef _TIMER_H_
#define _TIMER_H_

/***************************** Include Files *********************************/
#include "typedefs.h"
/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/


typedef uint32_t SYS_TICK;

typedef struct
{
        SYS_TICK   	 u32SystemTickCount;
        uint16_t     u16SystemTickResolution;
        uint32_t     u32SystemTickUpdateRate;

}SSysTick;

typedef void*    TimerHandle;  // handle to access the System Tick functions

//typedef void (*FClbTimer)(SYS_TICK currSysTick);

typedef void (*FClbTimer)(uint32_t currSysTick, void *pClbParam);


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/

void        Timer_Init();
TimerHandle Timer_Create    (FClbTimer fClbTimer, void *pClbParam);
BOOL        Timer_Delete    (TimerHandle tmrHandle);
BOOL        Timer_SetRate   (TimerHandle tmrHandle, uint16_t u16Rate);
uint16_t      Timer_GetRate   (TimerHandle tmrHandle);
void        Timer_Update    (void);
void        Timer_Start     (TimerHandle tmrHandle);
void        Timer_StartAt   (TimerHandle tmrHandle, uint16_t u16CurrCnt);
void        Timer_Stop      (TimerHandle tmrHandle);
//void 		Timer_Reset		(TimerHandle tmrHandle, uint16_t u16CurrCnt);
void        Timer_Reset     (TimerHandle tmrHandle);
BOOL        Timer_IsRunning (TimerHandle tmrHandle);

/************************** Variable Definitions *****************************/

extern SSysTick sSysTick;

#endif //_TIMER_H_
