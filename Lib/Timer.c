/** @FILE NAME:    Timer.c
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
#include <stdlib.h>
#include "Timer.h"
#include <AppConfig.h>
#include <F2802x_CpuTimers.h>

/************************** Constant Definitions *****************************/

#if		(APP_PLATFORM == APP_STAND_ALONE)

	#define TIMER_ENTER_CRITICAL()    StopCpuTimer0()
	#define TIMER_EXIT_CRITICAL()     StartCpuTimer0()
	
#elif 	(APP_PLATFORM	== APP_RTOS)


	#define TIMER_ENTER_CRITICAL()    //CPU_CRITICAL_ENTER()
	#define TIMER_EXIT_CRITICAL()     //CPU_CRITICAL_EXIT()
	
#else
	#error Please define Platform, RTOS or Stand alone in app_cfg.h	
#endif

/**************************** Type Definitions *******************************/

SSysTick sSysTick;

typedef struct _STickNode
{
        struct STickNode    *pNext;
        BOOL                bRun;
        uint16_t              u16Rate;           // rate of call
        uint16_t        		u16CurrCnt;        // current count value
        void*				pClbParam;
        void (*ClbFunc)(SYS_TICK currTick, void *pClbParam);      // function to be called

}STickNode;     // system tick registration node

typedef struct _STimerList
{
    STickNode*   tickHead;
    STickNode*   tickTail;
}STimerList;

STimerList sTimerList;
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
void Timer_Init()
{
    TIMER_ENTER_CRITICAL();

    sTimerList.tickHead = 0;
    sTimerList.tickTail = 0;

    TIMER_EXIT_CRITICAL();

    sSysTick.u32SystemTickCount = 0;

}
/*****************************************************************************/
/** @brief
 *          This function creates a timer based on the System Tick.
 *          It registrates a callback handler with the SystemTick ISR
 *          The handler will be called from within this ISR at specified rate.
 *
 *  @param  tickTimerHandler - handler to be called from the tick ISR
 *  @return a valid handle if the registration succeeded, NULL otherwise
 *  @note
 *          - All the registered handlers expiring in a current tick
 *            will be called in turn, sequentially.
 *            Therefore they have to be as short as possible.
 *          - The timer handler is called from within an ISR. All the
 *            ISR code restrictions apply.
 *          - The Timer_Create() should not be called from within an ISR.
 *          - The rate of which the System Tick based timer expires is set by the
 *            Timer_SetRate();
 */

TimerHandle Timer_Create(FClbTimer fClbTimer, void *pClbParam)
{
    STickNode*  newNode=0;

    if(fClbTimer)
    {
        newNode = malloc(sizeof(*newNode));
        if(newNode)
        {
            newNode->ClbFunc = fClbTimer;
            newNode->pClbParam = pClbParam;
            newNode->u16Rate = newNode->u16CurrCnt = 0;
            newNode->pNext = 0;
            newNode->bRun = FALSE;

            TIMER_ENTER_CRITICAL();

            // add tail
            if(sTimerList.tickTail == 0)
            {
                sTimerList.tickHead = sTimerList.tickTail= newNode;
            }
            else
            {
                sTimerList.tickTail->pNext= (struct STickNode *)newNode;
                sTimerList.tickTail= newNode;
            }

            TIMER_EXIT_CRITICAL();

        }
    }

    return (TimerHandle)newNode;
        
}
/*****************************************************************************/
/** @brief
 *          This function deletes a System Tick timer previously created by
 *          Timer_Create() and registered with the System Tick ISR.
 *
 *  @param  tmrHandle - handle to a Tick Timer to be unregistered from the tick ISR
 *                      The handle should have been obtained by Timer_Create()
 *  @return TRUE if the deletion succeeded, false otherwise
 *  @note
 *          The Timer_Delete() should not be called from within an ISR.
 */

BOOL Timer_Delete(TimerHandle tmrHandle)
{
    STickNode   *pTick, *prev;

    if(sTimerList.tickHead == 0)
    {
        return FALSE;   // no registered handlers
    }

    TIMER_ENTER_CRITICAL();

    if((pTick=sTimerList.tickHead) == (STickNode*)tmrHandle)
    {   // remove head
        if(sTimerList.tickHead==sTimerList.tickTail)
        {
            sTimerList.tickHead=sTimerList.tickTail=0;
        }
        else
        {
            sTimerList.tickHead=(STickNode*)pTick->pNext;
        }
    }        
    else
    {
        for(prev=(STickNode*)sTimerList.tickHead, pTick=(STickNode*)sTimerList.tickHead->pNext;
        		pTick!=0; prev=(STickNode*)pTick, pTick=(STickNode*)pTick->pNext)
        {   // search within the list
            if(pTick == (STickNode*)tmrHandle)
            {   // found it
                prev->pNext=pTick->pNext;
                if(sTimerList.tickTail==pTick)
                {   // adjust tail
                    sTimerList.tickTail=prev;
                }
                break;
            }
        }
    }
    TIMER_EXIT_CRITICAL();

    if(pTick)
    {
        free(pTick);
    }

    return (BOOL)(pTick != 0);
        
}

/*****************************************************************************/
/** @brief
 *          This function sets the rate of a System Tick timer previously created by
 *          Timer_Create() and registered with the System Tick ISR.
 *
 *  @param  tmrHandle - handle to a Tick Timer to update the rate for.
 *                      The handle should have been obtained by Timer_Create()
 *          u16Rate   - current timer rate, in System Tick counts
 *  @return TRUE if the update succeeded, false otherwise.
 *  @note
 *          A timer with rate == 0 is disabled
 */

BOOL Timer_SetRate(TimerHandle tmrHandle, uint16_t u16Rate)
{
    STickNode*  tNode;
    
    tNode = (STickNode*)tmrHandle;

    TIMER_ENTER_CRITICAL();

    tNode->u16Rate = tNode->u16CurrCnt = u16Rate;

    //LREP("\r\nRate = %d, Curr = %d",(uint32_t)tNode->u16Rate,(uint32_t)tNode->u16CurrCnt);
	
    TIMER_EXIT_CRITICAL();

    return TRUE;


}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 *              
 */
uint16_t Timer_GetRate(TimerHandle tmrHandle)
{
    return (uint16_t)(((STickNode*)tmrHandle)->u16Rate);
}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 *              SystemTickCount variable is updated
 */

void Timer_Update(void)
{
    STickNode   *pTick;

    for(pTick=(STickNode*)sTimerList.tickHead; pTick!=0; pTick=(STickNode*)pTick->pNext)
    {
        if((pTick->u16Rate != 0) && (pTick->bRun == TRUE))
        {
            if(--pTick->u16CurrCnt==0)
            {
                pTick->u16CurrCnt=pTick->u16Rate;
                /* Callback function is called */
                (*pTick->ClbFunc)(sSysTick.u32SystemTickCount, pTick->pClbParam);
            }
        }
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
void Timer_Start(TimerHandle tmrHandle)
{
    STickNode*  pTick;

    pTick = (STickNode*)tmrHandle;

    TIMER_ENTER_CRITICAL();
    pTick->bRun = TRUE;
    pTick->u16CurrCnt = pTick->u16Rate;
    TIMER_EXIT_CRITICAL();
}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void Timer_StartAt(TimerHandle tmrHandle, uint16_t u16CurrCnt)
{
    STickNode*  pTick;

    pTick = (STickNode*)tmrHandle;

    TIMER_ENTER_CRITICAL();
    pTick->bRun = TRUE;
    pTick->u16CurrCnt = u16CurrCnt;
    TIMER_EXIT_CRITICAL();
}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void Timer_Stop(TimerHandle tmrHandle)
{
    STickNode*  pTick;

    pTick = (STickNode*)tmrHandle;

    TIMER_ENTER_CRITICAL();
    pTick->bRun = FALSE;
    TIMER_EXIT_CRITICAL();
}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
//void Timer_Reset(TimerHandle tmrHandle, uint16_t u16CurrCnt)
void Timer_Reset(TimerHandle tmrHandle)
{
    STickNode*  pTick;

    pTick = (STickNode*)tmrHandle;

    TIMER_ENTER_CRITICAL();
    pTick->u16CurrCnt = pTick->u16Rate;
    //pTick->u16CurrCnt = u16CurrCnt;
    TIMER_EXIT_CRITICAL();
}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
BOOL Timer_IsRunning(TimerHandle tmrHandle)
{
    STickNode*  pTick;
    BOOL  bRunning = FALSE;
    pTick = (STickNode*)tmrHandle;

    TIMER_ENTER_CRITICAL();
    bRunning=pTick->bRun;
    TIMER_EXIT_CRITICAL();
    
    return bRunning;
}
