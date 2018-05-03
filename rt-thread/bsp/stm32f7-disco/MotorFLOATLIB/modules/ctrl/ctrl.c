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

 
 
//! \file   solutions/instaspin_foc/src/ctrl.c
//! \brief  Contains the various functions related to the controller (CTRL) object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "modules/ctrl/ctrl.h"


//#ifdef FLASH
//#pragma CODE_SECTION(CTRL_run,"ramfuncs");
//#pragma CODE_SECTION(CTRL_setup,"ramfuncs");
//#endif


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

void CTRL_getGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                   float_t *pKp,float_t *pKi,float_t *pKd)
{

  *pKp = CTRL_getKp(handle,ctrlType);
  *pKi = CTRL_getKi(handle,ctrlType);
  *pKd = CTRL_getKd(handle,ctrlType);

  return;    
} // end of CTRL_getGains() function


void CTRL_getIab_filt_pu(CTRL_Handle handle,MATH_vec2 *pIab_filt_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIab_filt_pu->value[0] = obj->Iab_filt.value[0];
  pIab_filt_pu->value[1] = obj->Iab_filt.value[1];

  return;
} // end of CTRL_getIab_filt_pu() function


void CTRL_getIab_in_pu(CTRL_Handle handle,MATH_vec2 *pIab_in_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIab_in_pu->value[0] = obj->Iab_in.value[0];
  pIab_in_pu->value[1] = obj->Iab_in.value[1];

  return;
} // end of CTRL_getIab_in_pu() function


void CTRL_getIdq_in_pu(CTRL_Handle handle,MATH_vec2 *pIdq_in_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIdq_in_pu->value[0] = obj->Idq_in.value[0];
  pIdq_in_pu->value[1] = obj->Idq_in.value[1];

  return;
} // end of CTRL_getIdq_in_pu() function


void CTRL_getIdq_ref_pu(CTRL_Handle handle,MATH_vec2 *pIdq_ref_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIdq_ref_pu->value[0] = obj->Idq_ref.value[0];
  pIdq_ref_pu->value[1] = obj->Idq_ref.value[1];

  return;
} // end of CTRL_getIdq_ref_pu() function


float_t CTRL_getMagCurrent_pu(CTRL_Handle handle)
{

  return(CTRL_getIdRated_pu(handle));
} // end of CTRL_getMagCurrent_pu() function


float_t CTRL_getMaximumSpeed_pu(CTRL_Handle handle)
{

  return(CTRL_getSpd_max_pu(handle));
} // end of CTRL_getMaximumSpeed_pu() function


void CTRL_getVab_in_pu(CTRL_Handle handle,MATH_vec2 *pVab_in_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVab_in_pu->value[0] = obj->Vab_in.value[0];
  pVab_in_pu->value[1] = obj->Vab_in.value[1];

  return;
} // end of CTRL_getVab_in_pu() function


void CTRL_getVab_out_pu(CTRL_Handle handle,MATH_vec2 *pVab_out_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVab_out_pu->value[0] = obj->Vab_out.value[0];
  pVab_out_pu->value[1] = obj->Vab_out.value[1];

  return;
} // end of CTRL_getVab_out_pu() function


void CTRL_getVdq_out_pu(CTRL_Handle handle,MATH_vec2 *pVdq_out_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVdq_out_pu->value[0] = obj->Vdq_out.value[0];
  pVdq_out_pu->value[1] = obj->Vdq_out.value[1];

  return;
} // end of CTRL_getVdq_out_pu() function


void CTRL_getWaitTimes(CTRL_Handle handle,uint_least32_t *pWaitTimes)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  uint_least16_t stateCnt;

  for(stateCnt=0;stateCnt<CTRL_numStates;stateCnt++)
    {
      pWaitTimes[stateCnt] = obj->waitTimes[stateCnt];
    }

  return;
} // end of CTRL_getWaitTimes() function


void CTRL_run(CTRL_Handle handle,HAL_Handle halHandle,
              const HAL_AdcData_t *pAdcData,
              HAL_PwmData_t *pPwmData,
							HAL_QepData_t *pQepData)
{

	uint_least16_t count_isr = CTRL_getCount_isr(handle);
	uint_least16_t numIsrTicksPerCtrlTick = CTRL_getNumIsrTicksPerCtrlTick(handle);

	// if needed, run the controller
	if(count_isr >= numIsrTicksPerCtrlTick)
	{
		CTRL_State_e ctrlState = CTRL_getState(handle);

		// reset the isr count
		CTRL_resetCounter_isr(handle);

		// increment the state counter
		//CTRL_incrCounter_state(handle);

		// increment the trajectory count
		CTRL_incrCounter_traj(handle);
		// run the appropriate controller
		if(ctrlState == CTRL_State_Error)
		{
			// run the Error controller
			
		}
		else if(ctrlState == CTRL_State_Idle)
		{
			// run the Idle controller
			CTRL_runOffLine(handle,halHandle,pAdcData,pPwmData,pQepData);
		}
		else if(ctrlState == CTRL_State_OffLine)
		{
			// run the offline controller
			CTRL_runOffLine(handle,halHandle,pAdcData,pPwmData,pQepData);
		}
		else if(ctrlState == CTRL_State_InitDete)
		{
			// run the InitDete controller
			CTRL_runInitDete_User(handle,halHandle,pAdcData,pPwmData,pQepData);
		}
		else if(ctrlState == CTRL_State_StaticStatorInitDete)
		{
			// run the StaticStatorInitDete controller
			CTRL_runStaticStatorInitDete_User(handle,halHandle,pAdcData,pPwmData,pQepData);
		}
		else if(ctrlState == CTRL_State_OpenLoop)
		{
			// increment the speed count
			CTRL_incrCounter_speed(handle);

			CTRL_runOpenLoop_User(handle,halHandle,pAdcData,pPwmData,pQepData);
		}
		else if(ctrlState == CTRL_State_OnLine || ctrlState == CTRL_State_CurrentCloseLoop || ctrlState == CTRL_State_SpeedCloseLoop )
		{
			//CTRL_Obj *obj = (CTRL_Obj *)handle;

			// increment the current count
			CTRL_incrCounter_current(handle);

			// increment the speed count
			CTRL_incrCounter_speed(handle);

			if( ctrlState == CTRL_State_CurrentCloseLoop )
			{
				CTRL_setFlag_enableCurrentCtrl(handle,true);
				CTRL_setFlag_enableSpeedCtrl(handle,false);
			}
			else if( ctrlState == CTRL_State_SpeedCloseLoop )
			{
				CTRL_setFlag_enableCurrentCtrl(handle,true);
				CTRL_setFlag_enableSpeedCtrl(handle,true);
			}

				CTRL_runOnLine_User(handle,halHandle,pAdcData,pPwmData,pQepData);

		}


	}
	else
	{
		// increment the isr count
		CTRL_incrCounter_isr(handle);
	}

  return;
} // end of CTRL_run() function


void CTRL_setGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                   const float_t Kp,const float_t Ki,const float_t Kd)
{

  CTRL_setKp(handle,ctrlType,Kp);
  CTRL_setKi(handle,ctrlType,Ki);
  CTRL_setKd(handle,ctrlType,Kd);

  return;    
} // end of CTRL_setGains() function


void CTRL_setMagCurrent_pu(CTRL_Handle handle,const float_t magCurrent_pu)
{

  CTRL_setIdRated_pu(handle,magCurrent_pu);

  return;    
} // end of CTRL_setMagCurrent_pu() function


void CTRL_setMaximumSpeed_pu(CTRL_Handle handle,const float_t maxSpeed_pu)
{

  CTRL_setSpd_max_pu(handle,maxSpeed_pu);

  return;    
} // end of CTRL_setMaximumSpeed_pu() function


CTRL_Handle CTRL_init(void *pMemory, const size_t numBytes)
{
	CTRL_Handle handle;
	CTRL_Obj *obj;


	if (numBytes < sizeof(CTRL_Obj))
		return((CTRL_Handle)NULL);


	// assign the handle
	handle = (CTRL_Handle)pMemory;


	// assign the object
	obj = (CTRL_Obj *)handle;


	obj->clarkeHandle_I = CLARKE_init(&obj->clarke_I, sizeof(obj->clarke_I));
	obj->clarkeHandle_V = CLARKE_init(&obj->clarke_V, sizeof(obj->clarke_V));


	obj->parkHandle = PARK_init(&obj->park, sizeof(obj->park));


	// initialize the Id PI controller module
	obj->pidHandle_Id = PID_init(&obj->pid_Id, sizeof(obj->pid_Id));


	// initialize the Iq PI controller module
	obj->pidHandle_Iq = PID_init(&obj->pid_Iq, sizeof(obj->pid_Iq));


	// initialize the speed PI controller module
	obj->pidHandle_spd = PID_init(&obj->pid_spd, sizeof(obj->pid_spd));

	
	obj->iparkHandle = IPARK_init(&obj->ipark, sizeof(obj->ipark));


	obj->svgenHandle = SVGEN_init(&obj->svgen, sizeof(obj->svgen)); 


	obj->trajHandle_Id = TRAJ_init(&obj->traj_Id, sizeof(obj->traj_Id));


	obj->trajHandle_spd = TRAJ_init(&obj->traj_spd, sizeof(obj->traj_spd));


	obj->trajHandle_spdMax = TRAJ_init(&obj->traj_spdMax, sizeof(obj->traj_spdMax));


	obj->rmpcntl_handle = RMP_CNTL_Init(&obj->rmpcntl, sizeof(obj->rmpcntl));


	obj->rampgen_handle = RAMPGEN_Init(&obj->rampgen, sizeof(obj->rampgen));


	obj->data_rtxd_handle = DATA_RTXD_Init(&obj->data_rtxd, sizeof(obj->data_rtxd));


	obj->InitPosDet_handle = INITPOSDET_Init(&obj->InitPosDet, sizeof(obj->InitPosDet));


	obj->Speed_handle = Speed_Init(&obj->Speed, sizeof(obj->Speed));


	return(handle);
} // end of CTRL_init() function


void CTRL_setParams(CTRL_Handle handle,USER_Params *pUserParams)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  float_t Kp,Ki,Kd;
  float_t outMin,outMax;
  float_t maxModulation;

  MATH_vec2 Iab_out_pu = {(0.0f),(0.0f)};
  MATH_vec2 Idq_out_pu = {(0.0f),(0.0f)};
  MATH_vec2 Idq_ref_pu = {(0.0f),(0.0f)};
  MATH_vec2 Vab_in_pu  = {(0.0f),(0.0f)};
  MATH_vec2 Vab_out_pu = {(0.0f),(0.0f)};
  MATH_vec2 Vdq_out_pu = {(0.0f),(0.0f)};


  // assign the motor type
  CTRL_setMotorParams(handle,pUserParams->motor_type,
                      pUserParams->motor_numPolePairs,
                      pUserParams->motor_ratedFlux,
                      pUserParams->motor_Ls_d,
                      pUserParams->motor_Ls_q,
                      pUserParams->motor_Rr,
                      pUserParams->motor_Rs);


  // assign other controller parameters
  CTRL_setNumIsrTicksPerCtrlTick(handle,pUserParams->numIsrTicksPerCtrlTick);
  CTRL_setNumCtrlTicksPerCurrentTick(handle,pUserParams->numCtrlTicksPerCurrentTick);
  CTRL_setNumCtrlTicksPerSpeedTick(handle,pUserParams->numCtrlTicksPerSpeedTick);
  CTRL_setNumCtrlTicksPerTrajTick(handle,pUserParams->numCtrlTicksPerTrajTick);

  CTRL_setCtrlFreq_Hz(handle,pUserParams->ctrlFreq_Hz);
  CTRL_setTrajFreq_Hz(handle,pUserParams->trajFreq_Hz);
  CTRL_setTrajPeriod_sec(handle,(1.0/pUserParams->trajFreq_Hz));

  CTRL_setCtrlPeriod_sec(handle,pUserParams->ctrlPeriod_sec);

  CTRL_setMaxVsMag_pu(handle,(pUserParams->maxVsMag_pu));

  CTRL_setIab_in_pu(handle,&Iab_out_pu);
  CTRL_setIdq_in_pu(handle,&Idq_out_pu);
  CTRL_setIdq_ref_pu(handle,&Idq_ref_pu);

  CTRL_setIdRated_pu(handle,(pUserParams->IdRated/pUserParams->iqFullScaleCurrent_A));

  CTRL_setVab_in_pu(handle,&Vab_in_pu);
  CTRL_setVab_out_pu(handle,&Vab_out_pu);
  CTRL_setVdq_out_pu(handle,&Vdq_out_pu);

  CTRL_setSpd_out_pu(handle,(0.0f));

  CTRL_setRhf(handle,0.0f);
  CTRL_setLhf(handle,0.0f);
  CTRL_setRoverL(handle,0.0f);


  // reset the counters
  CTRL_resetCounter_current(handle);
  CTRL_resetCounter_isr(handle);
  CTRL_resetCounter_speed(handle);
  CTRL_resetCounter_state(handle);
  CTRL_resetCounter_traj(handle);


  // set the wait times for each state
  CTRL_setWaitTimes(handle,&pUserParams->ctrlWaitTime[0]);


  // set flags
  CTRL_setFlag_enablePowerWarp(handle,false);
  CTRL_setFlag_enableCtrl(handle,false);
  CTRL_setFlag_enableOffset(handle,true);
  CTRL_setFlag_enableSpeedCtrl(handle,false);
  CTRL_setFlag_enableCurrentCtrl(handle,false);
  CTRL_setFlag_enableUserMotorParams(handle,false);
  CTRL_setFlag_enableDcBusComp(handle,true);


  // initialize the controller error code
  CTRL_setErrorCode(handle,CTRL_ErrorCode_NoError);


  // set the default controller state
  CTRL_setState(handle,CTRL_State_OnLine);


  // set the number of current sensors
  CTRL_setupClarke_I(handle,pUserParams->numCurrentSensors);


  // set the number of voltage sensors
  CTRL_setupClarke_V(handle,pUserParams->numVoltageSensors);


  // set the default Id PID controller parameters
  Kp = 1.0f;//(0.1f);
  Ki = 0.2f;//(pUserParams->ctrlPeriod_sec/0.004f);
  Kd = (0.0f);
  outMin = (-0.95f);
  outMax = (0.95f);

  PID_setGains(obj->pidHandle_Id,Kp,Ki,Kd);
  PID_setUi(obj->pidHandle_Id,(0.0f));
  PID_setMinMax(obj->pidHandle_Id,outMin,outMax);
  CTRL_setGains(handle,CTRL_Type_PID_Id,Kp,Ki,Kd);


  // set the default the Iq PID controller parameters
  Kp = 1.0f;//(0.1f);
  Ki = 0.2f;//(pUserParams->ctrlPeriod_sec/0.004f);
  Kd = (0.0f);
  outMin = (-0.95f);
  outMax = (0.95f);

  PID_setGains(obj->pidHandle_Iq,Kp,Ki,Kd);
  PID_setUi(obj->pidHandle_Iq,(0.0f));
  PID_setMinMax(obj->pidHandle_Iq,outMin,outMax);
  CTRL_setGains(handle,CTRL_Type_PID_Iq,Kp,Ki,Kd);


  // set the default speed PID controller parameters
  Kp = 2.0f;//(0.02f*pUserParams->maxCurrent*pUserParams->iqFullScaleFreq_Hz/pUserParams->iqFullScaleCurrent_A);
  Ki = 0.00001f;//(2.0f*pUserParams->maxCurrent*pUserParams->iqFullScaleFreq_Hz*pUserParams->ctrlPeriod_sec/pUserParams->iqFullScaleCurrent_A);
  Kd = (0.0f);
  outMin = (-1.0f);
  outMax = (1.0f);

  PID_setGains(obj->pidHandle_spd,Kp,Ki,Kd);
  PID_setUi(obj->pidHandle_spd,(0.0f));
  PID_setMinMax(obj->pidHandle_spd,outMin,outMax);
  CTRL_setGains(handle,CTRL_Type_PID_spd,Kp,Ki,Kd);


  // set the speed reference
  CTRL_setSpd_ref_pu(handle,(0.00f));


  // set the default Id current trajectory module parameters
  TRAJ_setIntValue(obj->trajHandle_Id,(0.0f));
  TRAJ_setTargetValue(obj->trajHandle_Id,(0.0f));
  TRAJ_setMinValue(obj->trajHandle_Id,(0.0f));
  TRAJ_setMaxValue(obj->trajHandle_Id,(0.0f));
  TRAJ_setMaxDelta(obj->trajHandle_Id,(0.0f));


  // set the default the speed trajectory module parameters
  TRAJ_setIntValue(obj->trajHandle_spd,(0.0f));
  TRAJ_setTargetValue(obj->trajHandle_spd,(0.0f));
  TRAJ_setMinValue(obj->trajHandle_spd,(0.0f));
  TRAJ_setMaxValue(obj->trajHandle_spd,(0.0f));
  TRAJ_setMaxDelta(obj->trajHandle_spd,(0.0f));


  // set the default maximum speed trajectory module parameters
  TRAJ_setIntValue(obj->trajHandle_spdMax,(0.0f));
  TRAJ_setTargetValue(obj->trajHandle_spdMax,(0.0f));
  TRAJ_setMinValue(obj->trajHandle_spdMax,(0.0f)); // not used
  TRAJ_setMaxValue(obj->trajHandle_spdMax,(0.0f)); // not used
  TRAJ_setMaxDelta(obj->trajHandle_spdMax,(0.0f)); // not used

  
  // set the default estimator parameters
  CTRL_setEstParams(obj->estHandle,pUserParams);


  // set the maximum modulation for the SVGEN module
  maxModulation = SVGEN_4_OVER_3;
  SVGEN_setMaxModulation(obj->svgenHandle,maxModulation);

	obj->angle_offset_pu = 0.0f;

  //! \Mod By Dl.K
  RMP_CNTL_SetRampDelayMax(obj->rmpcntl_handle, 600);

  RAMPGEN_SetStepAngleMax(obj->rampgen_handle,(pUserParams->ctrlPeriod_sec*pUserParams->iqFullScaleFreq_Hz));
  RAMPGEN_SetAngleOffset(obj->rampgen_handle,(0.75f));

	// fc100Hz=>Need:FreqRespose10HzORTimeRespose100ms:0~1.0f(800Hz)
  Speed_SetElecFilterSpeedRatio(obj->Speed_handle, ( 1.0f/(1.0f+2.0f*3.14f*100.0f*pUserParams->numCtrlTicksPerSpeedTick/pUserParams->ctrlFreq_Hz) ) );
  Speed_SetElecThetaChagTimeRatio( obj->Speed_handle, ( pUserParams->ctrlFreq_Hz/( pUserParams->iqFullScaleFreq_Hz*pUserParams->numCtrlTicksPerSpeedTick) ) );

  obj->Tabc.value[0]=(0.0f);
  obj->Tabc.value[1]=(0.0f);
  obj->Tabc.value[2]=(0.0f);

  obj->VdqRef.value[0]=(0.0f);
  obj->VdqRef.value[1]=(0.0f);

  obj->AngleRef=(-0.25f);

  obj->MntValue.value[0]=(0.0f);
  obj->MntValue.value[1]=(0.0f);
  obj->MntValue.value[2]=(0.0f);
  //! \Mod By Dl.K

  return;
} // end of CTRL_setParams() function


void CTRL_setSpd_ref_pu(CTRL_Handle handle,const float_t spd_ref_pu)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->spd_ref = spd_ref_pu;

  return;
} // end of CTRL_setSpd_ref_pu() function


void CTRL_setSpd_ref_krpm(CTRL_Handle handle,const float_t spd_ref_krpm)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  float_t krpm_to_pu_sf = EST_get_krpm_to_pu_sf(obj->estHandle);

  float_t spd_ref_pu = (spd_ref_krpm*krpm_to_pu_sf);

  obj->spd_ref = spd_ref_pu;

  return;
} // end of CTRL_setSpd_ref_krpm() function


void CTRL_setup(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  uint_least16_t count_traj = CTRL_getCount_traj(handle);
  uint_least16_t numCtrlTicksPerTrajTick = CTRL_getNumCtrlTicksPerTrajTick(handle);


  // as needed, update the trajectory
  if(count_traj >= numCtrlTicksPerTrajTick)
    {
      float_t intValue_Id = TRAJ_getIntValue(obj->trajHandle_Id);

      // reset the trajectory count
      CTRL_resetCounter_traj(handle);

      // run the trajectories
      CTRL_runTraj(handle);
    } // end of if(gFlag_traj) block

  return;
} // end of CTRL_setup() function


void CTRL_setupClarke_I(CTRL_Handle handle,uint_least8_t numCurrentSensors)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t alpha_sf,beta_sf;
  

  // initialize the Clarke transform module for current
  if(numCurrentSensors == 3)
    {
      alpha_sf = (MATH_ONE_OVER_THREE);
      beta_sf = (MATH_ONE_OVER_SQRT_THREE);
    }
  else if(numCurrentSensors == 2)
    {
      alpha_sf = (1.0);
      beta_sf = (MATH_ONE_OVER_SQRT_THREE);
    }
  else 
    {
      alpha_sf = (0.0);
      beta_sf = (0.0);
    }

  // set the parameters
  CLARKE_setScaleFactors(obj->clarkeHandle_I,alpha_sf,beta_sf);
  CLARKE_setNumSensors(obj->clarkeHandle_I,numCurrentSensors);

  return;
} // end of CTRL_setupClarke_I() function


void CTRL_setupClarke_V(CTRL_Handle handle,uint_least8_t numVoltageSensors)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t alpha_sf,beta_sf;
  

  // initialize the Clarke transform module for current
  if(numVoltageSensors == 3)
    {
      alpha_sf = (MATH_ONE_OVER_THREE);
      beta_sf = (MATH_ONE_OVER_SQRT_THREE);
    }
 else 
    {
      alpha_sf = (0.0);
      beta_sf = (0.0);
    }

  // set the parameters
  CLARKE_setScaleFactors(obj->clarkeHandle_V,alpha_sf,beta_sf);
  CLARKE_setNumSensors(obj->clarkeHandle_V,numVoltageSensors);

  return;
} // end of CTRL_setupClarke_V() function


void CTRL_setup_user(CTRL_Handle handle,
                     const float_t angle_pu,
                     const float_t speed_ref_pu,
                     const float_t speed_fb_pu,
                     const float_t speed_outMax_pu,
                     const MATH_vec2 *pIdq_offset_pu,
                     const MATH_vec2 *pVdq_offset_pu,
                     const bool flag_enableSpeedCtrl,
                     const bool flag_enableCurrentCtrl)
{
  CTRL_State_e ctrlState = CTRL_getState(handle);
  uint_least16_t count_traj = CTRL_getCount_traj(handle);
  uint_least16_t numCtrlTicksPerTrajTick = CTRL_getNumCtrlTicksPerTrajTick(handle);


  // increment the state counter
  CTRL_incrCounter_state(handle);

  // increment the trajectory count
  CTRL_incrCounter_traj(handle);

  // run the appropriate controller
  if(ctrlState == CTRL_State_OnLine)
  {
    // increment the current count
    CTRL_incrCounter_current(handle);

    // increment the speed count
    CTRL_incrCounter_speed(handle);
  }

  // as needed, update the trajectory
  if(count_traj >= numCtrlTicksPerTrajTick)
  {
    // reset the trajectory count
    CTRL_resetCounter_traj(handle);

    // run the trajectories
    CTRL_runTraj(handle);
  } // end of if(gFlag_traj) block


  CTRL_setAngle_pu(handle,angle_pu);
  CTRL_setSpeed_ref_pu(handle,speed_ref_pu);
  CTRL_setSpeed_fb_pu(handle,speed_fb_pu);
  CTRL_setSpeed_outMax_pu(handle,speed_outMax_pu);

  CTRL_setIdq_offset_pu(handle,pIdq_offset_pu);
  CTRL_setVdq_offset_pu(handle,pVdq_offset_pu);

  CTRL_setFlag_enableSpeedCtrl(handle,flag_enableSpeedCtrl);
  CTRL_setFlag_enableCurrentCtrl(handle,flag_enableCurrentCtrl);


  return;
} // end of CTRL_setup_user() function


void CTRL_setWaitTimes(CTRL_Handle handle,const uint_least32_t *pWaitTimes)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  uint_least16_t stateCnt;

  for(stateCnt=0;stateCnt<CTRL_numStates;stateCnt++)
    {
      obj->waitTimes[stateCnt] = pWaitTimes[stateCnt];
    }

  return;
} // end of CTRL_setWaitTimes() function


bool CTRL_updateState(CTRL_Handle handle)
{
  CTRL_State_e ctrlState = CTRL_getState(handle);
  bool flag_enableCtrl = CTRL_getFlag_enableCtrl(handle);
  bool stateChanged = false;


  if(flag_enableCtrl)
    {
      uint_least32_t waitTime = CTRL_getWaitTime(handle,ctrlState);
      uint_least32_t counter_ctrlState = CTRL_getCount_state(handle);


      // check for errors
      CTRL_checkForErrors(handle);


      if(counter_ctrlState >= waitTime)
        {
          // reset the counter
          CTRL_resetCounter_state(handle);


          if(ctrlState == CTRL_State_OnLine)
            {
              CTRL_Obj *obj = (CTRL_Obj *)handle;
              float_t Id_target = TRAJ_getTargetValue(obj->trajHandle_Id);

              // update the estimator state
              bool flag_estStateChanged = EST_updateState(obj->estHandle,Id_target);

              if(flag_estStateChanged)
                {
                  // setup the controller
                  CTRL_setupCtrl(handle);

                  // setup the trajectory
                  CTRL_setupTraj(handle);
                }

              if(EST_isOnLine(obj->estHandle))
                {
                  // setup the estimator for online state
                  CTRL_setupEstOnLineState(handle);
                }

              if(EST_isLockRotor(obj->estHandle) || 
                 (EST_isIdle(obj->estHandle) && EST_isMotorIdentified(obj->estHandle)))
                {
                  // set the enable controller flag to false
                  CTRL_setFlag_enableCtrl(handle,false);

                  // set the next controller state
                  CTRL_setState(handle,CTRL_State_Idle);
                }
            }
          else if(ctrlState == CTRL_State_OffLine)
            {
              // set the next controller state
              CTRL_setState(handle,CTRL_State_OnLine);
            }
          else if(ctrlState == CTRL_State_Idle)
            {
              CTRL_Obj *obj = (CTRL_Obj *)handle;
              bool  flag_enableUserMotorParams = CTRL_getFlag_enableUserMotorParams(handle);

              if(flag_enableUserMotorParams)
                {
                  // initialize the motor parameters using values from the user.h file
                  CTRL_setUserMotorParams(handle);
                }

              if(EST_isIdle(obj->estHandle))
                {
                  // setup the estimator for idle state
                  CTRL_setupEstIdleState(handle);

                  if(EST_isMotorIdentified(obj->estHandle))
                    {
                      if(CTRL_getFlag_enableOffset(handle))
                        {
                          // set the next controller state
                          CTRL_setState(handle,CTRL_State_OffLine);
                        }
                      else
                        {
                          // set the next controller state
                          CTRL_setState(handle,CTRL_State_OnLine);
                        }
                    }
                  else
                    {
                      // set the next controller state
                      CTRL_setState(handle,CTRL_State_OffLine);
                    }
                }
              else if(EST_isLockRotor(obj->estHandle))
                {
                  // set the next controller state
                  CTRL_setState(handle,CTRL_State_OnLine);
                }
            }
        }  // if(counter_ctrlState >= waitTime) loop
    } 
  else
    {
      CTRL_Obj *obj = (CTRL_Obj *)handle;

      // set the next controller state
      CTRL_setState(handle,CTRL_State_Idle);

      // set the estimator to idle
      if(!EST_isLockRotor(obj->estHandle))
        {
          if(EST_isMotorIdentified(obj->estHandle))
            {
              EST_setIdle(obj->estHandle);
            }
          else
            {
              EST_setIdle_all(obj->estHandle);

              EST_setRs_pu(obj->estHandle,(0.0));
            }
        }
    }


  // check to see if the state changed
  if(ctrlState != CTRL_getState(handle))
    {
      stateChanged = true;
    }

  return(stateChanged);
} // end of CTRL_updateState() function

// end of file
