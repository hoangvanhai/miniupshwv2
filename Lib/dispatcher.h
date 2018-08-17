/** @FILE NAME:    dispatcher.h
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

#ifndef LIB_DISPATCHER_H_
#define LIB_DISPATCHER_H_

/***************************** Include Files *********************************/
#include <typedefs.h>
/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

typedef void(*task_func_t)(void *arg);
typedef void* task_handle;

typedef struct task_desc_ {
	struct task_desc 	*next;
	uint8_t 			run;
	BOOL				always;
	void				*arg;
	task_func_t 		func_ptr;
}task_desc;


typedef struct stask_desc_list_ {
	task_desc *taskHead;
	task_desc *taskTail;
}stask_desc_list;


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/


void 		task_init();
task_handle task_create(task_func_t task_, void *arg, BOOL always);
BOOL 		task_delete(task_handle handle);
void 		task_main_exec();
BOOL		task_is_running(task_handle handle);
void 		task_set_run(task_handle handle, uint8_t times);
uint8_t		task_get_run(task_handle handle);
void 		task_cancel(task_handle handle);

/************************** Variable Definitions *****************************/

extern stask_desc_list task_list;

#endif /* LIB_DISPATCHER_H_ */
