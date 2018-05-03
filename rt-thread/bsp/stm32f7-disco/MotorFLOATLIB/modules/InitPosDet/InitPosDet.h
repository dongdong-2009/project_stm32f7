/*=====================================================================================
 File name:        InitPosDet.h  (IQ version)

 Originator:	DL.K

 Description:  The Initial Position Detection
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/
#ifndef __INITPOSDET_H__
#define __INITPOSDET_H__

// **************************************************************************
// the includes

#include "modules/types/types.h"
#include "modules/math/math.h"

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines
#define SUBDIVIDE_ANGLE_PU 0.0833333f // 30
#define VOL_VEC_ANGLE_SCLAR 24
#define VOL_VEC_ANGLE_INTERVAL_TIME_SCLAR 10
#define ID_EVG_SCLAR1 50
#define ID_EVG_SCLAR2 5

// **************************************************************************
// the typedefs
//! \brief Defines the INITPOSDET object
//!
typedef struct _INITPOSDET_Obj_
{
	uint16_t	InitPosDetFlag;
	uint16_t	InitPosAngleCount;
	float_t			InitPosAngle;
	float_t			SubdivideAngle;
	uint16_t	VolVecAngleFlag;
	uint16_t	VolVecAngleCount;
	float_t			VolVecAngle;
	float_t			VolVecAngleOffset;
	uint16_t	VolVecAngleIntervalTime;
	uint16_t	IdSampAngleFlag;
	float_t			IdSampAngle;
	uint16_t			IdSampAngleCount;
	float_t			IdEvg[VOL_VEC_ANGLE_SCLAR/2];
	float_t			IdEvgCoeff;
	uint16_t	IdEvgCount;

	float_t			IdEvgTmp;

} INITPOSDET_Obj;

typedef struct _INITPOSDET_Obj_ *INITPOSDET_Handle;

// **************************************************************************
// the functions

extern INITPOSDET_Handle INITPOSDET_Init(void *pMemory, const size_t numBytes);//! \Mod By Dl.K

static inline float_t INITPOSDET_GetInitPosAngle(INITPOSDET_Handle handle)//! \Mod By Dl.K
{
	INITPOSDET_Obj *obj = (INITPOSDET_Obj *)handle;

	return (obj->InitPosAngle);
}

static inline uint16_t INITPOSDET_GetVolVecAngleFlag(INITPOSDET_Handle handle)//! \Mod By Dl.K
{
	INITPOSDET_Obj *obj = (INITPOSDET_Obj *)handle;

	return (obj->VolVecAngleFlag);
}

static inline float_t INITPOSDET_GetVolVecAngle(INITPOSDET_Handle handle)//! \Mod By Dl.K
{
	INITPOSDET_Obj *obj = (INITPOSDET_Obj *)handle;

	return (obj->VolVecAngle);
}

static inline float_t INITPOSDET_GetIdSampAngle(INITPOSDET_Handle handle)//! \Mod By Dl.K
{
	INITPOSDET_Obj *obj = (INITPOSDET_Obj *)handle;

	return (obj->IdSampAngle);
}

static inline uint16_t INITPOSDET_GetIdSampAngleFlag(INITPOSDET_Handle handle)//! \Mod By Dl.K
{
	INITPOSDET_Obj *obj = (INITPOSDET_Obj *)handle;

	return (obj->IdSampAngleFlag);
}

extern void INITPOSDET_Run(INITPOSDET_Handle handle, float_t value);

extern void INITPOSDET_VolVecAngleRun(INITPOSDET_Handle handle);

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // __INITPOSDET_H__
