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
//! \file   solutions/instaspin_foc/boards/drv8312kit_revD/f28x/f2806xF/src/hal.c
//! \brief Contains the various functions related to the HAL object (everything outside the CTRL system) 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

// drivers

// modules
#include "modules/hal/hal.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals

HAL_Obj hal;


// **************************************************************************
// the functions

HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{
  uint_least8_t cnt;
  HAL_Handle handle;
  HAL_Obj *obj;


  if(numBytes < sizeof(HAL_Obj))
    return((HAL_Handle)NULL);


  // assign the handle
  handle = (HAL_Handle)pMemory;


  // assign the object
  obj = (HAL_Obj *)handle;

  // initialize the current offset estimator handles
  for(cnt=0;cnt<USER_NUM_CURRENT_SENSORS;cnt++)
    {
      obj->offsetHandle_I[cnt] = OFFSET_init(&obj->offset_I[cnt],sizeof(obj->offset_I[cnt]));
    }


  // initialize the voltage offset estimator handles
  for(cnt=0;cnt<USER_NUM_VOLTAGE_SENSORS;cnt++)
    {
      obj->offsetHandle_V[cnt] = OFFSET_init(&obj->offset_V[cnt],sizeof(obj->offset_V[cnt]));
    }


  return(handle);
} // end of HAL_init() function


void HAL_setParams(HAL_Handle handle,const USER_Params *pUserParams)
{
  uint_least8_t cnt;
//  HAL_Obj *obj = (HAL_Obj *)handle;
	float_t beta_lp_pu = (pUserParams->offsetPole_rps/(float_t)pUserParams->ctrlFreq_Hz);//! \Mod By Dl.K

  HAL_setNumCurrentSensors(handle,pUserParams->numCurrentSensors);
  HAL_setNumVoltageSensors(handle,pUserParams->numVoltageSensors);


  for(cnt=0;cnt<HAL_getNumCurrentSensors(handle);cnt++)
	{
	  HAL_setOffsetBeta_lp_pu(handle,HAL_SensorType_Current,cnt,beta_lp_pu);
	  HAL_setOffsetInitCond(handle,HAL_SensorType_Current,cnt,(0.0f));
	  HAL_setOffsetValue(handle,HAL_SensorType_Current,cnt,(0.0f));
	}//! \Mod By Dl.K


  for(cnt=0;cnt<HAL_getNumVoltageSensors(handle);cnt++)
    {
      HAL_setOffsetBeta_lp_pu(handle,HAL_SensorType_Voltage,cnt,beta_lp_pu);
      HAL_setOffsetInitCond(handle,HAL_SensorType_Voltage,cnt,(0.0f));
      HAL_setOffsetValue(handle,HAL_SensorType_Voltage,cnt,(0.0f));
    }//! \Mod By Dl.K

	  HAL_setBias(handle,HAL_SensorType_Current,0,((I_A_offset)*(pUserParams->current_sf)));
	  HAL_setBias(handle,HAL_SensorType_Current,1,((I_B_offset)*(pUserParams->current_sf)));
	  HAL_setBias(handle,HAL_SensorType_Current,2,((I_C_offset)*(pUserParams->current_sf)));


   float_t current_sf = (pUserParams->current_sf);//! \Mod By Dl.K
  HAL_setCurrentScaleFactor(handle,current_sf);
 

	  HAL_setBias(handle,HAL_SensorType_Voltage,0,((V_A_offset)*(pUserParams->voltage_sf)));
	  HAL_setBias(handle,HAL_SensorType_Voltage,1,((V_B_offset)*(pUserParams->voltage_sf)));
	  HAL_setBias(handle,HAL_SensorType_Voltage,2,((V_C_offset)*(pUserParams->voltage_sf)));


	float_t voltage_sf = (pUserParams->voltage_sf);//! \Mod By Dl.K
  HAL_setVoltageScaleFactor(handle,voltage_sf);
 
 return;
} // end of HAL_setParams() function

// end of file
