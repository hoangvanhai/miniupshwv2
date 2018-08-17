/** @FILE NAME:    utils.c
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
#include "utils.h"

/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/


/*****************************************************************************/

 /** @brief RoundFloatToUint32
 *		    Round a floating-point number to the nearest integer.
 *
 *  @param f_x is floating number
 *  @return integer value
 *  @note
 */
uint32_t RoundFloatToUint32(float f_x) 
{
	uint32_t u32_y;
	
	u32_y = f_x;
	if ((f_x - u32_y) < 0.5) return u32_y;
	else return u32_y+1;
}

/*****************************************************************************/
 /** @brief RoundFloatToUint16
 *		    Round a floating-point number to the nearest integer.
 *
 *  @param f_x floating value
 *  @return integer value
 *  @note
 */
uint16_t RoundFloatToUint16(float f_x) 
{
	uint16_t u16_y;
	
	u16_y = f_x;
	if ((f_x - u16_y) < 0.5) return u16_y;
	else return u16_y+1;
}

