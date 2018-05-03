/*=====================================================================================
 File name:        Speed.h  (IQ version)

 Originator:	DL.K

 Description:  The ...
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/
#ifndef __SPEED_H__
#define __SPEED_H__

// **************************************************************************
// the includes

#include "modules/types/types.h"
#include "modules/math/math.h"

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

// **************************************************************************
// the typedefs
typedef struct _SPEED_Obj_
{
	float_t					ElecTheta;
	float_t					OldElecTheta;
	float_t					ElecThetaChagTimeRatio;
	float_t					ElecSpeed;
	float_t					OldElecSpeed;
	float_t					ElecFilterSpeedRatio;
	float_t					ElecFilterSpeed;
} SPEED_Obj;

typedef struct _SPEED_Obj_ *SPEED_Handle;

// **************************************************************************
// the functions
extern SPEED_Handle Speed_Init(void *pMemory, const size_t numBytes);//! \Mod By Dl.K


static inline void Speed_SetElecTheta(SPEED_Handle handle, float_t theta)//! \Mod By Dl.K
{
	SPEED_Obj *obj = (SPEED_Obj *)handle;

	obj->ElecTheta = theta;

	return;
}

static inline void Speed_SetElecFilterSpeedRatio(SPEED_Handle handle, float_t ratio)//! \Mod By Dl.K
{
	SPEED_Obj *obj = (SPEED_Obj *)handle;

	obj->ElecFilterSpeedRatio = ratio;

	return;
}

static inline void Speed_SetElecThetaChagTimeRatio(SPEED_Handle handle, float_t ratio)//! \Mod By Dl.K
{
	SPEED_Obj *obj = (SPEED_Obj *)handle;

	obj->ElecThetaChagTimeRatio = ratio;

	return;
}

static inline void Speed_UpdateElecTheta(SPEED_Handle handle)//! \Mod By Dl.K
{
	SPEED_Obj *obj = (SPEED_Obj *)handle;

	obj->OldElecTheta = obj->ElecTheta;

	return;
}

static inline void Speed_UpdateElecSpeed(SPEED_Handle handle)//! \Mod By Dl.K
{
	SPEED_Obj *obj = (SPEED_Obj *)handle;

	obj->OldElecSpeed = obj->ElecSpeed;

	return;
}

static inline float_t Speed_GetElecSpeed(SPEED_Handle handle)//! \Mod By Dl.K
{
	SPEED_Obj *obj = (SPEED_Obj *)handle;

	return( obj->ElecSpeed );
}

extern void Speed_Run(SPEED_Handle handle);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // __SPEED_H__
