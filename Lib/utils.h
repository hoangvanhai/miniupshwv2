/** @FILE NAME:    utils.h
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

#ifndef _UTILS_H_
#define _UTILS_H_


/***************************** Include Files *********************************/
#include "typedefs.h"


/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/
/** Macro to convert a number to a string.	 */
#define TOSTRING(x) _TOSTRING(x)
/* A helper macro used by \ref TOSTRING.	*/
#define _TOSTRING(x) #x

#define BIT_SET_MASK(var, mask)          ((var) |= (mask))

#define BIT_CLEAR_MASK(var, mask)        ((var) &= (~(mask)))

#define BIT_TOGGLE_MASK(var, mask)       ((var) ^= (mask))

#define IS_BIT_SET_MASK(var, mask)       (((var) & (mask)))

#define IS_BIT_CLEAR_MASK(var, mask)     ((~(var) & (mask)))


#define BIT_SET(var, bitnum)             ((var) |= (1 << (bitnum)))

#define BIT_CLEAR(var, bitnum)           ((var) &= (~(1 << (bitnum))))

#define BIT_TOGGLE(var, bitnum)          ((var) ^= (1 << (bitnum)))

#define IS_BIT_SET(var, bitnum)          ((var) & (1 << (bitnum)))

#define IS_BIT_CLEAR(var, bitnum)        (~(var) & ((1 << (bitnum))))

#define BIT_WRITE(var, bitnum, value)	 ((var) = (var) & (~(1 << (bitnum))) \
														| ((value) << (bitnum)))
#define BIT_READ(var,bitnum)			 (((var) & (1 << (bitnum))) >> bitnum)

/* macros*/
#define	LBYTE(w)		( (BYTE)w )
#define HBYTE(w)		( (BYTE)(w >> 8) )

#define SWAP_BYTE(w)	{BYTE tmp; tmp = HBYTE(w); w = ((BYTE)(w) << 8) | tmp;}

#define MAX(a, b)		((a) > (b) ? (a) : (b))
#define MIN(a, b)		((a) > (b) ? (b) : (a))
  
#define ABS(a)          ((a) > 0 ? (a) : (-a))  

/************************** Function Prototypes ******************************/
extern uint16_t RoundFloatToUint16(float f_x);
extern uint32_t RoundFloatToUint32(float f_x);
/************************** Variable Definitions *****************************/

/*****************************************************************************/


#endif
