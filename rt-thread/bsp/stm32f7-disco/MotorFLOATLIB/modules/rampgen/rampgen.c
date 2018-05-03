/*=====================================================================================
 File name:        RAMPGEN.C  (IQ version)                  
                    
 Originator:	LiYong, NorthwesternPolytechnical University (NPU)

 Description:  The Ramp Generation                   
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/rampgen/rampgen.h"

RAMPGEN_Handle RAMPGEN_Init(void *pMemory, const size_t numBytes)//! \Mod By Dl.K
{
	RAMPGEN_Handle handle;

	if (numBytes < sizeof(RAMPGEN_Obj))
		return((RAMPGEN_Handle)NULL);

	// assign the handle
	handle = (RAMPGEN_Handle)pMemory;

	handle->Freq = (0.0);
	handle->StepAngleMax = (0.0);
	handle->Angle = (0.0);
	handle->Gain = (1.0);
	handle->Out = (0.0);
	handle->Offset = (0.75);

	return(handle);
}



