/* --COPYRIGHT--,BSD
* Copyright (c) 2012, Texas Instruments Incorporated
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* *  Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* *  Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* *  Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* --/COPYRIGHT--*/
#ifndef __RMP_CNTL_H__
#define __RMP_CNTL_H__

/* =================================================================================
File name:        RMP_CNTL.H  (IQ version)

Originator:	Digital Control Systems Group
Texas Instruments

Description:
Header file containing constants, data type definitions, and
function prototypes for the RMPCNTL module.
=====================================================================================
History:
-------------------------------------------------------------------------------------
04-15-2005	Version 3.20
------------------------------------------------------------------------------*/

// **************************************************************************
// the includes

//modules
#include "modules/types/types.h"
#include "modules/math/math.h"

//!
//!
//! \defgroup RMPCNTL 
//!
//@{

// Include the algorithm overview defined in ...
//! \defgroup RMPCNTL_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

//! \brief Defines the RMPCNTL object
//!
typedef struct _RMPCNTL_Obj_
{ 
	float_t    			TargetValue; 	// Input: Target input (pu)
	uint_least32_t	RampDelayMax;	// Parameter: Maximum delay rate (Q0) - independently with global Q
	float_t    			RampLowLimit;	// Parameter: Minimum limit (pu)
	float_t    			RampHighLimit;	// Parameter: Maximum limit (pu)
	uint_least32_t	RampDelayCount; // Variable: Incremental delay (Q0) - independently with global Q
	float_t    			SetpointValue;	// Output: Target output (pu)
	uint_least32_t	EqualFlag;		// Output: Flag output (Q0) - independently with global Q
} RMPCNTL_Obj;

typedef struct _RMPCNTL_Obj_ *RMPCNTL_Handle;

// **************************************************************************
// the functions

extern RMPCNTL_Handle RMP_CNTL_Init(void *pMemory, const size_t numBytes);//! \Mod By Dl.K

static inline void RMP_CNTL_SetRampDelayMax(RMPCNTL_Handle handle, const uint_least32_t value)//! \Mod By Dl.K
{
	RMPCNTL_Obj *obj = (RMPCNTL_Obj *)handle;

	obj->RampDelayMax = value;

	return;
}

static inline void RMP_CNTL_SetTargetValue(RMPCNTL_Handle handle, const float_t target_value)//! \Mod By Dl.K
{
	RMPCNTL_Obj *obj = (RMPCNTL_Obj *)handle;

	obj->TargetValue = target_value;

	return;
}

static inline float_t RMP_CNTL_GetFreq(RMPCNTL_Handle handle)//! \Mod By Dl.K
{
	RMPCNTL_Obj *obj = (RMPCNTL_Obj *)handle;

	return (obj->SetpointValue);
}

static inline void RMP_CNTL_InitPara(RMPCNTL_Handle handle)//! \Mod By Dl.K
{
	RMPCNTL_Obj *obj = (RMPCNTL_Obj *)handle;

	obj->RampDelayCount = 0;
	obj->SetpointValue = (0.0f);
	obj->EqualFlag = 0;
	
	return ;
}

static inline void RMP_CNTL_Run(RMPCNTL_Handle handle)//! \Mod By Dl.K
{
	RMPCNTL_Obj *obj = (RMPCNTL_Obj *)handle;

	float_t tmp;

	tmp = obj->TargetValue - obj->SetpointValue;

	if (MATH_abs(tmp) > (0.0000305f))
	{
		obj->RampDelayCount += 1;
		if (obj->RampDelayCount >= obj->RampDelayMax)
		{
			if (obj->TargetValue >= obj->SetpointValue)
			{
				obj->SetpointValue += (0.0000305f);        // 0.0000305 is resolution(1/2^15) of Q15
				if (obj->SetpointValue > obj->RampHighLimit)
					obj->SetpointValue = obj->RampHighLimit;
				obj->RampDelayCount = 0;
			}
			else
			{
				obj->SetpointValue -= (0.0000305f);        // 0.0000305 is resolution of Q15
				if (obj->SetpointValue < obj->RampLowLimit)
					obj->SetpointValue = obj->RampLowLimit;
				obj->RampDelayCount = 0;
			}
		}
	}
	else  obj->EqualFlag = 0x7FFFFFFF;
	return;
}


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // __RMP_CNTL_H__
