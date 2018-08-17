/** @FILE NAME:    dispather.c
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

#include <dispatcher.h>
#include <stdlib.h>

/***************** Macros (Inline Functions) Definitions *********************/

#define TASK_ENTER_CRITICAL()    //DisableIntT1
#define TASK_EXIT_CRITICAL()     //EnableIntT1

/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/

stask_desc_list task_list;


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void 		task_init() {

    TASK_ENTER_CRITICAL();
    task_list.taskHead = 0;
    task_list.taskTail = 0;
    TASK_EXIT_CRITICAL();
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
task_handle task_create(task_func_t task, void *arg, BOOL always) {

	task_desc*  newNode=0;
	if(task)
	{
		newNode = malloc(sizeof(*newNode));
		if(newNode)
		{
			newNode->func_ptr = task;
			newNode->always = always;
			newNode->arg = arg;
			TASK_ENTER_CRITICAL();

			// add tail
			if(task_list.taskTail == 0)
			{
				task_list.taskHead = task_list.taskTail= newNode;
				newNode->next = NULL;
			}
			else
			{
				task_list.taskTail->next= (struct task_desc *)newNode;
				task_list.taskTail= newNode;
				newNode->next = NULL;
			}

			TASK_EXIT_CRITICAL();

		} else {
			//ASSERT(FALSE);
		}
	}

	return (task_handle)newNode;
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
BOOL 		task_delete(task_handle handle) {
    task_desc   *pTick, *prev;

    if(task_list.taskHead == 0)
    {
        return FALSE;   // no registered handlers
    }

    TASK_ENTER_CRITICAL();

    if((pTick=task_list.taskHead) == (task_desc*)handle)
    {   // remove head
        if(task_list.taskHead==task_list.taskTail)
        {
        	task_list.taskHead=task_list.taskTail=0;
        }
        else
        {
        	task_list.taskHead=(task_desc*)pTick->next;
        }
    }
    else
    {
        for(prev=(task_desc*)task_list.taskHead, pTick=(task_desc*)task_list.taskHead->next;
        		pTick!=0; prev=(task_desc*)pTick, pTick=(task_desc*)pTick->next)
        {   // search within the list
            if(pTick == (task_desc*)handle)
            {   // found it
                prev->next=pTick->next;
                if(task_list.taskTail==pTick)
                {   // adjust tail
                	task_list.taskTail=prev;
                }
                break;
            }
        }
    }
    TASK_EXIT_CRITICAL();

    if(pTick)
    {
        free(pTick);
    }

    return (BOOL)(pTick != 0);
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void 		task_main_exec() {

    task_desc   *pTick;
    for(pTick=(task_desc*)task_list.taskHead; pTick!=0;
    		pTick=(task_desc*)pTick->next)
    {
        if(pTick->always || pTick->run > 0)
        {
			pTick->func_ptr(pTick->arg);
			if(pTick->always == FALSE) pTick->run--;
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
BOOL		task_is_running(task_handle handle) {
	task_desc*  pHandle;
	BOOL  bRunning = FALSE;
	pHandle = (task_desc*)handle;
	TASK_ENTER_CRITICAL();
	bRunning = pHandle->run > 0 ? TRUE : FALSE;
	TASK_EXIT_CRITICAL();
	return bRunning;
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void 		task_set_run(task_handle handle, uint8_t times) {
	task_desc*  pHandle;
	pHandle = (task_desc*)handle;
	TASK_ENTER_CRITICAL();
	pHandle->run += times;
	TASK_EXIT_CRITICAL();
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
uint8_t		task_get_run(task_handle handle) {
	task_desc*  pHandle;
	uint8_t  run = 0;
	pHandle = (task_desc*)handle;
	TASK_ENTER_CRITICAL();
	run = pHandle->run;
	TASK_EXIT_CRITICAL();
	return run;
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void 		task_cancel(task_handle handle) {
	task_desc*  pHandle;
	pHandle = (task_desc*)handle;
	TASK_ENTER_CRITICAL();
	pHandle->run = 0;
	TASK_EXIT_CRITICAL();
}
