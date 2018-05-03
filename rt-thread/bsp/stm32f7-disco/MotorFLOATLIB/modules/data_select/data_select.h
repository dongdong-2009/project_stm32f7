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
#ifndef __DATA_SELECT_H__
#define __DATA_SELECT_H__

/* =================================================================================
File name:        DATA_SELECT.H  (IQ version)

Originator:	Digital Control Systems Group
Texas Instruments

Description:
This file contains type definitions, constants and initializers for
the ramp generation functions contained in DATA_SELECT.C
=====================================================================================
History:
-------------------------------------------------------------------------------------
04-15-2005	Version 3.20
------------------------------------------------------------------------------*/

// **************************************************************************
// the includes

//modules
#include "modules/types/types.h"
#include "modules/hal/hal_obj.h"
#include "modules/ctrl/ctrl_obj.h"


//!
//!
//! \defgroup DATA_SELECT
//!
//@{

// Include the algorithm overview defined in ...
//! \defgroup DATA_SELECT_OVERVIEW

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************


// **************************************************************************
// the typedefs

//! \brief Defines the DATA_SELECT object
//!

typedef enum
{
	STA_DATA				= 0,
	IA_DATA					= 1,
	IB_DATA					= 2,
	IC_DATA					= 3,

	IALPHA_DATA				= 4,
	IBETA_DATA				= 5,

	ID_DATA					= 6,
	IQ_DATA					= 7,

	IDC_DATA				= 8,

	VA_DATA					= 11,
	VB_DATA					= 12,
	VC_DATA					= 13,

	VALPHA_DATA				= 14,
	VBETA_DATA				= 15,

	VD_DATA					= 16,
	VQ_DATA					= 17,

	VDC_DATA				= 18,

	QEP_DATA				= 21,
	ANGLE_DATA				= 22,
	SPEED_DATA				= 23,
	FILTER_SPEED_DATA		= 24,

	IA_OFFSET_DATA			= 31,

	TA_DATA					= 41,
	TB_DATA					= 42,
	TC_DATA					= 43,

	MNT_VALUE_1_DATA		= 51,
	MNT_VALUE_2_DATA		= 52,
	MNT_VALUE_3_DATA		= 53,
	MNT_VALUE_4_DATA		= 54,

	END_DATA
}DATA_e;

// **************************************************************************
// the functions
static inline int16_t DATA_Select(CTRL_Handle handle, HAL_Handle halHandle, const HAL_AdcData_t *pAdcData, const HAL_QepData_t *pQepData, DATA_e data_e)
{
	CTRL_Obj *obj = (CTRL_Obj *)handle;
	OFFSET_Obj *offset_obj = (OFFSET_Obj *)(halHandle->offsetHandle_I[0]);
	int16_t value;

	switch(data_e)
	{
		case IA_DATA:
			value = 32768.0f*(pAdcData->I.value[0]);
			break;
		case IB_DATA:
			value = 32768.0f*(pAdcData->I.value[1]);
			break;
		case IC_DATA:
			value = 32768.0f*(pAdcData->I.value[2]);
			break;
		case IALPHA_DATA:
			value = 32768.0f*(obj->Iab_in.value[0]);
			break;
		case IBETA_DATA:
			value = 32768.0f*(obj->Iab_in.value[1]);
			break;
		case ID_DATA:
			value = 32768.0f*(obj->Idq_in.value[0]);
			break;
		case IQ_DATA:
			value = 32768.0f*(obj->Idq_in.value[1]);
			break;
		case IDC_DATA:
			value = 32768.0f*(pAdcData->IdcBus);
			break;

		case VA_DATA:
			value = 32768.0f*(pAdcData->V.value[0]);
			break;
		case VB_DATA:
			value = 32768.0f*(pAdcData->V.value[1]);
			break;
		case VC_DATA:
			value = 32768.0f*(pAdcData->V.value[2]);
			break;
		case VALPHA_DATA:
			value = 32768.0f*(obj->Vab_in.value[0]);
			break;
		case VBETA_DATA:
			value = 32768.0f*(obj->Vab_in.value[1]);
			break;
		case VD_DATA:
			value = 4;
			break;
		case VQ_DATA:
			value = 5;
			break;
		case VDC_DATA:
			value = 32768.0f*(pAdcData->VdcBus);
			break;

		case QEP_DATA:
			value = 32768.0f*pQepData->qep_cnt;
			break;

		case ANGLE_DATA:
			value = 32768.0f*pQepData->qep_cnt*USER_MOTOR_NUM_POLE_PAIRS;
			value &= 0x7FFF;
			break;

		case SPEED_DATA:
			value = 32768.0f*(obj->Speed_handle->ElecSpeed);;
			break;

		case FILTER_SPEED_DATA:
			value = 32768.0f*(obj->Speed_handle->ElecFilterSpeed);;
			break;

		case IA_OFFSET_DATA:
			value = 32768.0f*(offset_obj->value);
			break;

		case TA_DATA:
			value = 32768.0f*(obj->Tabc.value[0]);
			break;

		case TB_DATA:
			value = 32768.0f*(obj->Tabc.value[1]);
			break;

		case TC_DATA:
			value = 32768.0f*(obj->Tabc.value[2]);
			break;

		case MNT_VALUE_1_DATA:
			value = 32768.0f*(obj->MntValue.value[0]);
			break;

		case MNT_VALUE_2_DATA:
			value = 32768.0f*(obj->MntValue.value[1]);
			break;

		case MNT_VALUE_3_DATA:
			value = 32768.0f*(obj->MntValue.value[2]);
			break;

		case MNT_VALUE_4_DATA:
			value = 32768.0f*(obj->InitPosDet.IdEvgTmp);
			break;

		default:
			break;
	}

	return value;
}

extern void DATA_RTXD_Run(CTRL_Handle handle, HAL_Handle halHandle, const HAL_AdcData_t *pAdcData, const HAL_QepData_t *pQepData);
extern void DATA_RTXD_Decode(CTRL_Handle handle, HAL_Handle halHandle);

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // __DATA_SELECT_H__
