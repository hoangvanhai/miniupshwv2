//================================================================================
// Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
//  ALL RIGHTS RESERVED
//================================================================================
/**
 * Performs exponential moving average calculation.  This macro requires the use of the
 * MATH_EMAVG_IQ_C structure.  The structure should be initialized with the MATH_EMAVG_IQ_C_INIT
 * macro.
 */

#ifndef MATH_EMAVG_IQ_C_H_
#define MATH_EMAVG_IQ_C_H_

/**
 * Exponential-moving average structure
 */
typedef struct {
	_iq In;
	_iq Out;
	_iq Multiplier;
} SMATH_EMAVG_IQ_C;


/**
 * Performs exponential moving average calculation.
 * @param m - MATH_EMAVG_IQ_C structure with values.
 * @return MATH_EMAVG_IQ_C Out parameter.
 */
#define MATH_EMAVG_IQ_C(v)  {   \
                                v.Out = (_IQ20mpyIQX((v.In - v.Out) , 20, v.Multiplier, 30) + v.Out); \
                            }

/**
 * Initial values for MATH_EMAVG_IQ_C structure.
 */
#define MATH_EMAVG_IQ_C_INIT(v, mult)	\
	v.In = 0;						\
	v.Out = 0;						\
	v.Multiplier = mult

#endif /* MATH_EMAVG_IQ_C_H_ */
