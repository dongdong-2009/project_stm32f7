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
#ifndef __DATA_RTXD_H__
#define __DATA_RTXD_H__

/* =================================================================================
File name:        DATA_RTXD.H  (IQ version)

Originator:	Digital Control Systems Group
Texas Instruments

Description:
This file contains type definitions, constants and initializers for
the ramp generation functions contained in DATA_RTXD.C
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
//! \defgroup DATA_RTXD
//!
//@{

// Include the algorithm overview defined in ...
//! \defgroup DATA_RTXD_OVERVIEW

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines
#define DATA_RXD_SIZE 13
#define DATA_TXD_SIZE 13


// **************************************************************************
// the typedefs

//! \brief Defines the DATA_RTXD object
//!

typedef struct _DATA_RTXD_Obj_
{
	uint8_t	data_rxd_num;
	uint8_t	data_rxd[DATA_RXD_SIZE];
	uint8_t	data_txd_num;
	uint8_t	data_txd[DATA_TXD_SIZE];
	uint16_t	time;
} DATA_RTXD_Obj;

typedef struct _DATA_RTXD_Obj_ *DATA_RTXD_Handle;

// **************************************************************************
// the functions

extern DATA_RTXD_Handle DATA_RTXD_Init(void *pMemory, const size_t numBytes);

static inline void Reset_DATA_RXD(DATA_RTXD_Handle handle)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)handle;

	obj->data_rxd_num = 0;
	
	return;
}

static inline void Reset_DATA_TXD(DATA_RTXD_Handle handle)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)handle;

	obj->data_txd_num = 0;
	
	return;
}

static inline uint8_t DATA_RXD_Status(DATA_RTXD_Handle handle)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)handle;

	if( obj->data_rxd_num == DATA_RXD_SIZE )
	{
		return false;
	}
	else return true;
}

static inline uint8_t DATA_TXD_Status(DATA_RTXD_Handle handle)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)handle;

	if( obj->data_txd_num == DATA_TXD_SIZE )
	{
		return false;
	}
	else return true;
}

static inline void DATA_RXD_Run(DATA_RTXD_Handle handle, uint8_t value)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)handle;

	if( obj->data_rxd_num < DATA_RXD_SIZE )
	{
		obj->data_rxd[obj->data_rxd_num] =  value;
		obj->data_rxd_num++;
	}
	//else obj->data_rxd_num = 0;
	return;
}

static inline uint8_t DATA_TXD_Run(DATA_RTXD_Handle handle)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)handle;
	uint8_t value = 0;

	if( obj->data_txd_num < DATA_TXD_SIZE )
	{
		value = obj->data_txd[obj->data_txd_num];
		obj->data_txd_num++;
	}
	//else obj->data_txd_num = 0;

	return value;
}

static inline uint8_t* Get_DATA_TXD_Ptr(DATA_RTXD_Handle handle)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)handle;
	
	obj->data_txd_num = DATA_TXD_SIZE;

	return obj->data_txd;
}

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // __DATA_RTXD_H__
