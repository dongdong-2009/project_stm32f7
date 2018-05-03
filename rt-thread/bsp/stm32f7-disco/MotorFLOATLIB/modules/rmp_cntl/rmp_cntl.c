/*=====================================================================================
 File name:        RMP_CNTL.C  (IQ version)                  
                    
 Originator:	LiYong, NorthwesternPolytechnical University (NPU)

 Description:  The Ramp Control                   
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/rmp_cntl/rmp_cntl.h"

RMPCNTL_Handle RMP_CNTL_Init(void *pMemory, const size_t numBytes)//! \Mod By Dl.K
{
	RMPCNTL_Handle handle;

	if (numBytes < sizeof(RMPCNTL_Obj))
		return((RMPCNTL_Handle)NULL);

	// assign the handle
	handle = (RMPCNTL_Handle)pMemory;

	handle->TargetValue = (0.0f);
	handle->RampDelayMax = 150;
	handle->RampLowLimit = (-1.0f);
	handle->RampHighLimit = (1.0f);
	handle->RampDelayCount = 0;
	handle->SetpointValue = (0.0f);
	handle->EqualFlag = 0;

	return(handle);
}


