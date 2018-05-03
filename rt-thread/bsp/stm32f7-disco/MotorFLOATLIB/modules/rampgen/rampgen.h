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
#ifndef __RAMPGEN_H__
#define __RAMPGEN_H__

/* =================================================================================
File name:        RAMPGEN.H  (IQ version)

Originator:	Digital Control Systems Group
Texas Instruments

Description:
This file contains type definitions, constants and initializers for
the ramp generation functions contained in RAMPGEN.C
=====================================================================================
History:
-------------------------------------------------------------------------------------
04-15-2005	Version 3.20
------------------------------------------------------------------------------*/

// **************************************************************************
// the includes

//modules
#include "modules/types/types.h"


//!
//!
//! \defgroup RAMPGEN 
//!
//@{

// Include the algorithm overview defined in ...
//! \defgroup RAMPGEN_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

//! \brief Defines the RAMPGEN object
//!
typedef struct _RAMPGEN_Obj_
{
	float_t  Freq; 		// Input: Ramp frequency (pu) 	
	float_t  StepAngleMax;	// Parameter: Maximum step angle (pu)		
	float_t  Angle;		// Variable: Step angle (pu)					  
	float_t  Gain;			// Input: Ramp gain (pu)
	float_t  Out;  	 	// Output: Ramp signal (pu) 	
	float_t  Offset;		// Input: Ramp offset (pu) 				 
} RAMPGEN_Obj;

typedef struct _RAMPGEN_Obj_ *RAMPGEN_Handle;

// **************************************************************************
// the functions

extern RAMPGEN_Handle RAMPGEN_Init(void *pMemory, const size_t numBytes);

static inline float_t RAMPGEN_GetAngleOut(RAMPGEN_Handle handle)//! \Mod By Dl.K
{
	RAMPGEN_Obj *obj = (RAMPGEN_Obj *)handle;

	return (obj->Out);
}

static inline void RAMPGEN_SetFreq(RAMPGEN_Handle handle, const float_t freq)//! \Mod By Dl.K
{
	RAMPGEN_Obj *obj = (RAMPGEN_Obj *)handle;

	obj->Freq = freq;

	return;
}

static inline void RAMPGEN_SetStepAngleMax(RAMPGEN_Handle handle, const float_t step_angle_max)//! \Mod By Dl.K
{
	RAMPGEN_Obj *obj = (RAMPGEN_Obj *)handle;

	obj->StepAngleMax = step_angle_max;

	return;
}

static inline void RAMPGEN_SetAngleOffset(RAMPGEN_Handle handle, const float_t offset)//! \Mod By Dl.K
{
	RAMPGEN_Obj *obj = (RAMPGEN_Obj *)handle;

	obj->Offset = offset;

	return;
}

static inline void RAMPGEN_Run(RAMPGEN_Handle handle)//! \Mod By Dl.K
{
	RAMPGEN_Obj *obj = (RAMPGEN_Obj *)handle;

	// Compute the angle rate
	obj->Angle += (obj->StepAngleMax * obj->Freq);

	// Saturate the angle rate within (-1,1)        
	if (obj->Angle>(1.0f))
		obj->Angle -= (1.0f);
	else if (obj->Angle<(-1.0f))
		obj->Angle += (1.0f);

	// Compute the ramp output
	obj->Out = (obj->Angle * obj->Gain) + obj->Offset;

	// Saturate the ramp output within (-1,1)     
	if (obj->Out>(1.0f))
		obj->Out -= (1.0f);
	else if (obj->Out<(-1.0f))
		obj->Out += (1.0f);

	return;
}


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // __RAMPGEN_H__
