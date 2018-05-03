/*=====================================================================================
 File name:        Template.c  (IQ version)
                    
 Originator:	DL.K

 Description:  The ...
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/Speed/Speed.h"

SPEED_Handle Speed_Init(void *pMemory, const size_t numBytes)//! \Mod By Dl.K
{
	SPEED_Handle handle;

	if (numBytes < sizeof(SPEED_Obj))
		return((SPEED_Handle)NULL);

	// assign the handle
	handle = (SPEED_Handle)pMemory;

	handle->ElecTheta = (0.0f);
	handle->OldElecTheta = (0.0f);
	handle->ElecThetaChagTimeRatio = (0.0f);// Real:1/(T) PU:1/(T)/SpeedBaseFreq
	handle->ElecSpeed = (0.0f);
	handle->OldElecSpeed = (0.0f);
	handle->ElecFilterSpeed = (0.0f);
	handle->ElecFilterSpeedRatio = (0.0f);// 1/(1+2*pi*Fc*Ts)

	return(handle);
}

void Speed_Run(SPEED_Handle handle)
{
	SPEED_Obj *obj = (SPEED_Obj *)handle;

	Speed_UpdateElecSpeed( obj );
	// Need to Change the Time between Elec And OldElec According to Speed
	// 1kHz:1ms ElecSpeed:500Hz RealSpeed:125Hz@4poles, 2kHz:0.5ms ElecSpeed:500Hz RealSpeed:125Hz@4poles,
	if( MATH_abs(obj->ElecTheta - obj->OldElecTheta) > (0.5f) )
	{
		if( (obj->ElecTheta + (1.0f)- obj->OldElecTheta)<(0.5f) )
		{
			obj->ElecSpeed = ( obj->ElecThetaChagTimeRatio * (obj->ElecTheta +(1.0f)- obj->OldElecTheta) );
		}
		else if( (obj->ElecTheta - (1.0f) - obj->OldElecTheta)>(-0.5f) )
		{
			obj->ElecSpeed = ( obj->ElecThetaChagTimeRatio * (obj->ElecTheta -(1.0f)- obj->OldElecTheta) );
		}
	}
	else
	{
		obj->ElecSpeed = ( obj->ElecThetaChagTimeRatio * (obj->ElecTheta - obj->OldElecTheta) );
	}
	obj->OldElecSpeed = obj->ElecFilterSpeed;
	obj->ElecFilterSpeed = obj->ElecSpeed;
	// Need to Change the Time between ElecSpeed And OldElecSpeed According to Acceleration
	// 1.0f@10ms:0.1f@1ms=>FreqRespose100HzORTimeRespose10ms:0~1.0f(800Hz) And the FilterFreq must 10x;
	if( MATH_abs(obj->ElecFilterSpeed - obj->OldElecSpeed) > (0.1f) )
	{
		obj->ElecFilterSpeed = obj->OldElecSpeed;
	}
//	else
//	{
		Speed_UpdateElecTheta(obj);
//	}

//	obj->ElecFilterSpeed = (obj->ElecSpeed * ((1.0f)-obj->ElecFilterSpeedRatio)) + (obj->ElecFilterSpeed * obj->ElecFilterSpeedRatio);

//	obj->ElecFilterSpeed = MATH_sat(obj->ElecFilterSpeed, (2.0f), (-2.0f));

	return;
}
