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
#ifndef _CTRL_H_
#define _CTRL_H_

//! \file   solutions/instaspin_foc/src/ctrl.h
//! \brief Contains the public interface, object and function definitions for 
//!        various functions related to the CTRL object 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include <math.h>

#include "modules/hal/hal.h"
#include "modules/user/user.h"
#include "modules/ctrl/ctrl_obj.h"
#include "modules/hal/hal_obj.h"
#include "modules/data_select/data_select.h"
//! \Mod By Dl.K


//!
//!
//! \defgroup CTRL CTRL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the function prototypes


//! \brief     Adds the Vdq offset to the Vdq values
//! \param[in] handle        The controller (CTRL) handle
static inline void CTRL_addVdq_offset(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Vdq_out.value[0] += obj->Vdq_offset_pu.value[0];
  obj->Vdq_out.value[1] += obj->Vdq_offset_pu.value[1];

  return;
} // return CTRL_addVdq_offset() function


//! \brief      Gets the current loop count
//! \param[in]  handle  The controller (CTRL) handle
//! \return    The current loop count
static inline uint_least16_t CTRL_getCount_current(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_current);
} // end of CTRL_getCount_current() function


//! \brief      Gets the isr count
//! \param[in]  handle  The controller (CTRL) handle
//! \return    The isr count
static inline uint_least16_t CTRL_getCount_isr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_isr);
} // end of CTRL_getCount_isr() function


//! \brief      Gets the speed loop count
//! \param[in]  handle  The controller (CTRL) handle
//! \return    The speed loop count
static inline uint_least16_t CTRL_getCount_speed(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_speed);
} // end of CTRL_getCount_speed() function


//! \brief      Gets the state count 
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The state count
static inline uint_least32_t CTRL_getCount_state(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_state);
} // end of CTRL_getCount_state() function


//! \brief      Gets the trajectory loop count
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The trajectory loop count
static inline uint_least16_t CTRL_getCount_traj(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_traj);
} // end of CTRL_getCount_traj() function


//! \brief      Gets the controller execution frequency
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The controller execution frequency, Hz
static inline uint_least32_t CTRL_getCtrlFreq(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->ctrlFreq_Hz);
} // end of CTRL_getCtrlFreq() function


//! \brief      Gets the controller execution period
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The controller execution period, sec
static inline float_t CTRL_getCtrlPeriod_sec(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->ctrlPeriod_sec);
} // end of CTRL_getCtrlPeriod_sec() function


//! \brief     Gets the error code from the controller (CTRL) object
//! \param[in] handle  The controller (CTRL) handle
//! \return    The error code
static inline CTRL_ErrorCode_e CTRL_getErrorCode(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->errorCode);
} // end of CTRL_getErrorCode() function


//! \brief     Gets the estimator handle for a given controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The estimator handle for the given controller
static inline EST_Handle CTRL_getEstHandle(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->estHandle);
} // end of CTRL_getEstHandle() function


//! \brief      Gets the feedback value for the speed controller
//! \param[in]  handle  The controller (CTRL) handle
static inline float_t CTRL_getFbackValue_spd_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->speed_fb_pu);
} // end of CTRL_getFbackValue_spd_pu() function


//! \brief     Gets the enable controller flag value from the estimator
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable controller flag value
static inline bool CTRL_getFlag_enableCtrl(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enableCtrl);
} // end of CTRL_getFlag_enableCtrl() function


//! \brief     Gets the enable current controllers flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable current controller flag value
static inline bool CTRL_getFlag_enableCurrentCtrl(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enableCurrentCtrl);
} // end of CTRL_getFlag_enableCurrentCtrl() function


//! \brief     Gets the enable DC bus compensation flag value from the estimator
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable DC bus compensation flag value
static inline bool CTRL_getFlag_enableDcBusComp(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enableDcBusComp);
} // end of CTRL_getFlag_enableDcBusComp() function


//! \brief     Gets the PowerWarp enable flag value from the estimator
//! \param[in] handle  The controller (CTRL) handle
//! \return    The PowerWarp enable flag value
static inline bool CTRL_getFlag_enablePowerWarp(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enablePowerWarp);
} // end of CTRL_getFlag_enablePowerWarp() function


//! \brief     Gets the enable offset flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable offset flag value
static inline bool CTRL_getFlag_enableOffset(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enableOffset);
} // end of CTRL_getFlag_enableOffset() function


//! \brief     Gets the enable speed control flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable speed control flag value
static inline bool CTRL_getFlag_enableSpeedCtrl(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enableSpeedCtrl);
} // end of CTRL_getFlag_enableSpeedCtrl() function


//! \brief     Gets the enable user motor parameters flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable user motor parameters flag value
static inline bool CTRL_getFlag_enableUserMotorParams(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enableUserMotorParams);
} // end of CTRL_getFlag_enableUserMotorParams() function


//! \brief     Gets the gain values for the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \param[in] pKp       The pointer for the Kp value, pu
//! \param[in] pKi       The pointer for the Ki value, pu
//! \param[in] pKd       The pointer for the Kd value, pu
void CTRL_getGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                   float_t *pKp,float_t *pKi,float_t *pKd);


//! \brief      Gets the alpha/beta filtered current vector values from the controller
//! \param[in]  handle        The controller (CTRL) handle
//! \param[out] pIab_filt_pu  The vector for the alpha/beta filtered current vector values, pu
void CTRL_getIab_filt_pu(CTRL_Handle handle,MATH_vec2 *pIab_filt_pu);


//! \brief     Gets the alpha/beta filtered current vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The alpha/beta filtered current vector memory address
static inline MATH_vec2 *CTRL_getIab_filt_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Iab_filt));
} // end of CTRL_getIab_filt_addr() function


//! \brief     Gets the alpha/beta current input vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The alpha/beta current input vector memory address
static inline MATH_vec2 *CTRL_getIab_in_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Iab_in));
} // end of CTRL_getIab_in_addr() function


//! \brief      Gets the alpha/beta current input vector values from the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[out] pIab_in_pu  The vector for the alpha/beta current input vector values, pu
void CTRL_getIab_in_pu(CTRL_Handle handle,MATH_vec2 *pIab_in_pu);


//! \brief     Gets the direct current input value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current input value, pu
static inline float_t CTRL_getId_in_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_in.value[0]);
} // end of CTRL_getId_in_pu() function


//! \brief     Gets the direct current (Id) reference value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current reference value, pu
static inline float_t CTRL_getId_ref_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_ref.value[0]);
} // end of CTRL_getId_ref_pu() function


//! \brief     Gets the direct/quadrature current input vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature current input vector memory address
static inline MATH_vec2 *CTRL_getIdq_in_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_in));
} // end of CTRL_getIdq_in_addr() function


//! \brief      Gets the direct/quadrature current input vector values from the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[out] pIdq_in_pu  The vector for the direct/quadrature input current vector values, pu
void CTRL_getIdq_in_pu(CTRL_Handle handle,MATH_vec2 *pIdq_in_pu);


//! \brief     Gets the direct/quadrature current reference vector values from the controller
//! \param[in] handle        The controller (CTRL) handle
//! \param[out] pIdq_ref_pu  The vector for the direct/quadrature current reference vector values, pu
void CTRL_getIdq_ref_pu(CTRL_Handle handle,MATH_vec2 *pIdq_ref_pu);


//! \brief     Gets the direct current (Id) offset value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current offset value, pu
static inline float_t CTRL_getId_offset_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_offset_pu.value[0]);
} // end of CTRL_getId_offset_pu() function


//! \brief     Gets the Id rated current value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The Id rated current value, pu
static inline float_t CTRL_getIdRated_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->IdRated);
} // end of CTRL_getIdRated_pu() function


//! \brief     Gets the quadrature current input value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current input value, pu
static inline float_t CTRL_getIq_in_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_in.value[1]);
} // end of CTRL_getIq_in_pu() function


//! \brief     Gets the quadrature current (Iq) offset value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current offset value, pu
static inline float_t CTRL_getIq_offset_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_offset_pu.value[1]);
} // end of CTRL_getIq_offset_pu() function


//! \brief     Gets the quadrature current (Iq) reference value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current reference value, pu
static inline float_t CTRL_getIq_ref_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_ref.value[1]);
} // end of CTRL_getIq_ref_pu() function


//! \brief      Gets the reference value for the Iq current controller
//! \param[in]  handle  The controller (CTRL) handle
static inline  float_t CTRL_getRefValue_Iq_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  float_t refValue_pu = CTRL_getIq_ref_pu(handle) + CTRL_getIq_offset_pu(handle);

  return(refValue_pu);
} // end of CTRL_getRefValue_Iq_pu() function


//! \brief     Gets the quadrature current (Iq) reference memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current reference memory address
static inline float_t *CTRL_getIq_ref_pu_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_ref.value[1]));
} // end of CTRL_getIq_ref_pu_addr() function


//! \brief     Gets the integral gain (Ki) value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Ki value
static inline float_t CTRL_getKi(CTRL_Handle handle,const CTRL_Type_e ctrlType)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t Ki = (0.0f);

  if(ctrlType == CTRL_Type_PID_spd)
    {
      Ki = obj->Ki_spd;
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      Ki = obj->Ki_Id;
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      Ki = obj->Ki_Iq;
    }

  return(Ki);
} // end of CTRL_getKi() function


//! \brief     Gets the derivative gain (Kd) value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Kd value
static inline float_t CTRL_getKd(CTRL_Handle handle,const CTRL_Type_e ctrlType)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t Kd = (0.0f);

  if(ctrlType == CTRL_Type_PID_spd)
    {
      Kd = obj->Kd_spd;
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      Kd = obj->Kd_Id;
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      Kd = obj->Kd_Iq;
    }

  return(Kd);
} // end of CTRL_getKd() function


//! \brief     Gets the proportional gain (Kp) value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Kp value
static inline float_t CTRL_getKp(CTRL_Handle handle,const CTRL_Type_e ctrlType)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t Kp = (0.0f);

  if(ctrlType == CTRL_Type_PID_spd)
    {
      Kp = obj->Kp_spd;
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      Kp = obj->Kp_Id;
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      Kp = obj->Kp_Iq;
    }

  return(Kp);
} // end of CTRL_getKp() function


//! \brief     Gets the high frequency inductance (Lhf) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The Lhf value
static inline float_t CTRL_getLhf(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Lhf);
} // end of CTRL_getLhf() function


//! \brief     Gets the magnetizing current value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The magnetizing current value, pu
float_t CTRL_getMagCurrent_pu(CTRL_Handle handle);


//! \brief      Gets the maximum voltage vector
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The maximum voltage vector (value betwen 0 and 4/3)
static inline float_t CTRL_getMaxVsMag_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->maxVsMag_pu);
} // end of CTRL_getMaxVsMag_pu() function


//! \brief     Gets the maximum speed value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The maximum speed value, pu
float_t CTRL_getMaximumSpeed_pu(CTRL_Handle handle);


//! \brief     Gets the motor rated flux from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The motor rated flux, V*sec
static inline float_t CTRL_getMotorRatedFlux(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->motorParams.ratedFlux_VpHz);
} // end of CTRL_getMotorRatedFlux() function


//! \brief     Gets the motor type from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The motor type
static inline MOTOR_Type_e CTRL_getMotorType(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->motorParams.type);
} // end of CTRL_getMotorType() function


//! \brief     Gets the number of controller clock ticks per current controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of controller clock ticks per estimator clock tick
static inline uint_least16_t CTRL_getNumCtrlTicksPerCurrentTick(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->numCtrlTicksPerCurrentTick);
} // end of CTRL_getNumCtrlTicksPerCurrentTick() function


//! \brief     Gets the number of controller clock ticks per speed controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of controller clock ticks per speed clock tick
static inline uint_least16_t CTRL_getNumCtrlTicksPerSpeedTick(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->numCtrlTicksPerSpeedTick);
} // end of CTRL_getNumCtrlTicksPerSpeedTick() function


//! \brief     Gets the number of controller clock ticks per trajectory clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of controller clock ticks per trajectory clock tick
static inline uint_least16_t CTRL_getNumCtrlTicksPerTrajTick(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->numCtrlTicksPerTrajTick);
} // end of CTRL_getNumCtrlTicksPerTrajTick() function


//! \brief     Gets the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
static inline uint_least16_t CTRL_getNumIsrTicksPerCtrlTick(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->numIsrTicksPerCtrlTick);
} // end of CTRL_getNumIsrTicksPerCtrlTick() function


//! \brief     Gets the reference value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The reference value, pu
static inline float_t CTRL_getRefValue_pu(CTRL_Handle handle,const CTRL_Type_e ctrlType)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t ref = (0.0f);

  if(ctrlType == CTRL_Type_PID_spd)
    {
      ref = PID_getRefValue(obj->pidHandle_spd);
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      ref = PID_getRefValue(obj->pidHandle_Id);
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      ref = PID_getRefValue(obj->pidHandle_Iq);
    }

  return(ref);
} // end of CTRL_getRefValue_pu() function


//! \brief      Gets the reference value for the Id current controller
//! \param[in]  handle  The controller (CTRL) handle
static inline  float_t CTRL_getRefValue_Id_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  float_t refValue_pu = (0.0f);

  if(CTRL_getFlag_enableSpeedCtrl(handle))
  {
    refValue_pu = CTRL_getId_ref_pu(handle) + CTRL_getId_offset_pu(handle);
  }
  else
  {
    refValue_pu = CTRL_getId_offset_pu(handle);
  }

  return(refValue_pu);
} // end of CTRL_getRefValue_Id_pu() function


//! \brief      Gets the reference value for the speed controller
//! \param[in]  handle  The controller (CTRL) handle
static inline  float_t CTRL_getRefValue_spd_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->speed_ref_pu);
} // end of CTRL_getRefValue_spd_pu() function


//! \brief     Gets the high frequency resistance (Rhf) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The Rhf value
static inline float_t CTRL_getRhf(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Rhf);
} // end of CTRL_getRhf() function


//! \brief     Gets the R/L value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The the R/L value
static inline float_t CTRL_getRoverL(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->RoverL);
} // end of CTRL_getRoverL() function


//! \brief     Gets the maximum speed value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The maximum speed value, pu
static inline float_t CTRL_getSpd_max_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->spd_max);
} // end of CTRL_getSpd_max_pu() function


//! \brief     Gets the output speed memory address from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed memory address
static inline float_t *CTRL_getSpd_out_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->spd_out));
} // end of CTRL_getSpd_out_addr() function


//! \brief     Gets the output speed value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed value, pu
static inline float_t CTRL_getSpd_out_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->spd_out);
} // end of CTRL_getSpd_out_pu() function


//! \brief     Gets the output speed reference value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed reference value, pu
static inline float_t CTRL_getSpd_ref_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->spd_ref);
} // end of CTRL_getSpd_ref_pu() function


//! \brief     Gets the output speed intermediate reference value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed intermediate reference value, pu
static inline float_t CTRL_getSpd_int_ref_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(TRAJ_getIntValue(obj->trajHandle_spd));
} // end of CTRL_getSpd_int_ref_pu() function


//! \brief     Gets the maximum output value for the speed controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The maximum output value, pu
static inline float_t CTRL_getSpeed_outMax_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->speed_outMax_pu);
} // end of CTRL_getSpeed_outMax_pu() function


//! \brief     Gets the controller state
//! \param[in] handle  The controller (CTRL) handle
//! \return    The controller state
static inline CTRL_State_e CTRL_getState(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->state);
} // end of CTRL_getState() function


//! \brief      Gets the trajectory execution frequency
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The trajectory execution frequency, Hz
static inline uint_least32_t CTRL_getTrajFreq(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->trajFreq_Hz);
} // end of CTRL_getTrajFreq() function


//! \brief      Gets the trajectory execution period
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The trajectory execution period, sec
static inline float_t CTRL_getTrajPeriod_sec(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->trajPeriod_sec);
} // end of CTRL_getTrajPeriod_sec() function


//! \brief     Gets the trajectory step size
//! \param[in] handle  The controller (CTRL) handle
void CTRL_getTrajStep(CTRL_Handle handle);


//! \brief     Gets the integrator (Ui) value from the specified controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Ui value
static inline float_t CTRL_getUi(CTRL_Handle handle,const CTRL_Type_e ctrlType)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t Ui = (0.0f);

  if(ctrlType == CTRL_Type_PID_spd)
    {
      Ui = obj->Ui_spd;
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      Ui = obj->Ui_Id;
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      Ui = obj->Ui_Iq;
    }

  return(Ui);
} // end of CTRL_getUi() function


//! \brief     Gets the alpha/beta voltage input vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The alpha/beta voltage input vector memory address
static inline MATH_vec2 *CTRL_getVab_in_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vab_in));
} // end of CTRL_getVab_in_addr() function


//! \brief      Gets the alpha/beta voltage input vector values from the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[out] pVab_in_pu  The vector for the alpha/beta voltage input vector values, pu
void CTRL_getVab_in_pu(CTRL_Handle handle,MATH_vec2 *pVab_in_pu);


//! \brief     Gets the alpha/beta voltage output vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The alpha/beta voltage output vector memory address
static inline MATH_vec2 *CTRL_getVab_out_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vab_out));
} // end of CTRL_getVab_out_addr() function

static inline float_t *CTRL_getVa_out_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vab_out.value[0]));
}
static inline float_t *CTRL_getVb_out_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vab_out.value[1]));
}
//! \Add By Dl.K

//! \brief      Gets the alpha/beta voltage output vector values from the controller
//! \param[in]  handle       The controller (CTRL) handle
//! \param[out] pVab_out_pu  The vector for the alpha/beta voltage output vector values, pu
void CTRL_getVab_out_pu(CTRL_Handle handle,MATH_vec2 *pVab_out_pu);


//! \brief     Gets the direct voltage output value memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct voltage output value memory address
static inline float_t *CTRL_getVd_out_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_out.value[0]));
} // end of CTRL_getVd_out_addr() function


//! \brief     Gets the direct voltage output value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct voltage output value, pu
static inline float_t CTRL_getVd_out_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Vdq_out.value[0]);
} // end of CTRL_getVd_out_pu() function


//! \brief     Gets the direct/quadrature voltage output vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature voltage output vector memory address
static inline MATH_vec2 *CTRL_getVdq_out_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_out));
} // end of CTRL_getVdq_out_pu() function


//! \brief      Gets the direct/quadrature voltage output vector values from the controller
//! \param[in]  handle       The controller (CTRL) handle
//! \param[out] pVdq_out_pu  The vector for the direct/quadrature voltage output vector values, pu
void CTRL_getVdq_out_pu(CTRL_Handle handle,MATH_vec2 *pVdq_out_pu);


//! \brief     Gets the controller version number
//! \param[in] handle     The controller (CTRL) handle
//! \param[in] pVersion   A pointer to the version 
void CTRL_getVersion(CTRL_Handle handle,CTRL_Version *pVersion);


//! \brief     Gets the quadrature voltage output value memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature voltage output value memory address
static inline float_t *CTRL_getVq_out_addr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_out.value[1]));
} // end of CTRL_getVq_out_addr() function


//! \brief     Gets the quadrature voltage output value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature voltage output value, pu
static inline float_t CTRL_getVq_out_pu(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Vdq_out.value[1]);
} // end of CTRL_getVq_out_pu() function


//! \brief     Gets the wait time for a given state
//! \param[in] handle     The controller (CTRL) handle
//! \param[in] ctrlState  The controller state
//! \return    The wait time, controller clock counts
static inline uint_least32_t CTRL_getWaitTime(CTRL_Handle handle,const CTRL_State_e ctrlState)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->waitTimes[ctrlState]);
} // end of CTRL_getWaitTime() function


//! \brief     Gets the wait times from the estimator
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pWaitTimes  A pointer to a vector for the wait times, estimator clock counts
void CTRL_getWaitTimes(CTRL_Handle handle,uint_least32_t *pWaitTimes);


//! \brief     Increments the current counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_current(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  uint_least16_t count = obj->counter_current;

  // increment the count
  count++;

  // save the count value
  obj->counter_current = count;

  return;
} // end of CTRL_incrCounter_current() function


//! \brief     Increments the isr counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_isr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  uint_least16_t count = obj->counter_isr;

  // increment the count
  count++;

  // save the count value
  obj->counter_isr = count;

  return;
} // end of CTRL_incrCounter_isr() function


//! \brief     Increments the speed counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_speed(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  uint_least16_t count = obj->counter_speed;

  // increment the count
  count++;

  // save the count value
  obj->counter_speed = count;

  return;
} // end of CTRL_incrCounter_speed() function


//! \brief     Increments the state counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_state(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  uint_least32_t count = obj->counter_state;

  // increment the count
  count++;

  // save the count value
  obj->counter_state = count;

  return;
} // end of CTRL_incrCounter_state() function


//! \brief     Increments the trajectory counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_traj(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  uint_least16_t count = obj->counter_traj;

  // increment the count
  count++;

  // save the count value
  obj->counter_traj = count;

  return;
} // end of CTRL_incrCounter_traj() function


//! \brief      Initializes a specified controller
//! \details    Initializes all handles required for field oriented control and FAST observer
//!             interface.  Returns a handle to the CTRL object.
//! \param[in]  ctrlNumber  The controller number
//! \param[in]  estNumber   The estimator number
//! \return     The controller (CTRL) object handle
#ifdef FAST_ROM_V1p6
CTRL_Handle CTRL_initCtrl(const uint_least8_t ctrlNumber, const uint_least8_t estNumber);
#else
CTRL_Handle CTRL_initCtrl(const uint_least8_t estNumber,void *pMemory,const size_t numBytes);
#endif


//! \brief     Determines if there is a controller error
//! \param[in] handle  The controller (CTRL) handle
//! \return    A boolean value denoting if there is a controller error (true) or not (false)
static inline bool CTRL_isError(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_State_e ctrlState = CTRL_getState(handle);
  bool state = false;


  // check for controller errors
  if(ctrlState == CTRL_State_Error)
    {
      state = true;
    }

  return(state);
} // end of CTRL_isError() function


//! \brief     Resets the current counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_current(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_current = 0;

  return;
} // end of CTRL_resetCounter_current() function


//! \brief     Resets the isr counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_isr(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_isr = 1;

  return;
} // end of CTRL_resetCounter_isr() function


//! \brief     Resets the speed counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_speed(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_speed = 0;

  return;
} // end of CTRL_resetCounter_speed() function


//! \brief     Resets the state counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_state(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_state = 0;

  return;
} // end of CTRL_resetCounter_state() function


//! \brief     Resets the trajectory counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_traj(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_traj = 0;

  return;
} // end of CTRL_resetCounter_traj() function


//! \brief      Runs the controller
//! \details    The CTRL_run function implements the field oriented
//!             control.  There are two parts to CTRL_run, CTRL_runOffline,
//!             CTRL_runOnline, and CTRL_runOnlineUser.
//! \param[in]  handle     The controller (CTRL) handle
//! \param[in]  halHandle  The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData   The pointer to the ADC data
//! \param[out] pPwmData   The pointer to the PWM data
void CTRL_run(CTRL_Handle handle,HAL_Handle halHandle,
              const HAL_AdcData_t *pAdcData,
              HAL_PwmData_t *pPwmData,
							HAL_QepData_t *pQepData);


//! \brief     Runs the trajectory
//! \param[in] handle   The controller (CTRL) handle
static inline void CTRL_runTraj(CTRL_Handle handle)
{
}


//! \brief      Sets the angle value, pu
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  angle_pu  The angle value, pu
static inline void CTRL_setAngle_pu(CTRL_Handle handle,const float_t angle_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->angle_pu = angle_pu;

  return;
} // end of CTRL_setAngle_pu() function


//! \brief      Sets the controller frequency
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  ctrlFreq_Hz  The controller frequency, Hz
static inline void CTRL_setCtrlFreq_Hz(CTRL_Handle handle,const uint_least32_t ctrlFreq_Hz)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->ctrlFreq_Hz = ctrlFreq_Hz;

  return;
} // end of CTRL_setCtrlFreq_Hz() function


//! \brief      Sets the controller execution period
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  ctrlPeriod_sec  The controller execution period, sec
static inline void CTRL_setCtrlPeriod_sec(CTRL_Handle handle,const float_t ctrlPeriod_sec)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->ctrlPeriod_sec = ctrlPeriod_sec;

  return;
} // end of CTRL_setCtrlPeriod_sec() function


//! \brief      Sets the error code in the controller
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  errorCode   The error code
static inline void CTRL_setErrorCode(CTRL_Handle handle,const CTRL_ErrorCode_e errorCode)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->errorCode = errorCode;

  return;
} // end of CTRL_setErrorCode() function


//! \brief      Sets the default estimator parameters
//! \details    Copies all scale factors that are defined in the file user.h and used
//!             by CTRL into the CTRL object.
//! \param[in]  estHandle    The estimator (EST) handle
//! \param[in]  pUserParams  The pointer to the user parameters
static inline void CTRL_setEstParams(EST_Handle estHandle,USER_Params *pUserParams)
{
}


//! \brief      Sets the enable controller flag value in the estimator
//! \details    Enables/Disables the sensorless controller.  The first time this function
//!             is called and the function CTRL_setFlag_enableUserMotorParams() is not set true,
//!             motor commissioning is performed and then subsequent times it is called the
//!             motor starts to run.
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
static inline void CTRL_setFlag_enableCtrl(CTRL_Handle handle,const bool state)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enableCtrl = state;

  return;
} // end of CTRL_setFlag_enableCtrl() function


//! \brief     Sets the enable current controllers flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_enableCurrentCtrl(CTRL_Handle handle,const bool state)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enableCurrentCtrl = state;

  return;
} // end of CTRL_setFlag_enableCurrentCtrl() function


//! \brief      Sets the enable DC bus compensation flag value in the estimator
//! \details    The DC bus compensation algorithm will compensate the Iq and Id PI controller's
//!             proportional gains for DC bus variations.
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
static inline void CTRL_setFlag_enableDcBusComp(CTRL_Handle handle,const bool state)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enableDcBusComp = state;

  return;
} // end of CTRL_setFlag_enableDcBusComp() function


//! \brief      Sets the PowerWarp enable flag value in the estimator
//! \details    PowerWarp is only used when controlling an induction motor. PowerWarp adjusts field
//!             levels so that the least amount of power is used according to the load on
//!             the motor.
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
static inline void CTRL_setFlag_enablePowerWarp(CTRL_Handle handle,const bool state)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enablePowerWarp = state;

  return;
} // end of CTRL_setFlag_enablePowerWarp() function


//! \brief      Sets the enable offset flag value in the estimator
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
static inline void CTRL_setFlag_enableOffset(CTRL_Handle handle,const bool state)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enableOffset = state;

  return;
} // end of CTRL_setFlag_enableOffset() function


//! \brief      Sets the enable speed control value in the estimator
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
static inline void CTRL_setFlag_enableSpeedCtrl(CTRL_Handle handle,const bool state)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enableSpeedCtrl = state;

  return;
} // end of CTRL_setFlag_enableSpeedCtrl() function


//! \brief      Sets the enable user motor parameters flag value in the estimator
//! \details    When this function is called with a true parameter, the motor parameters
//!             in user.h are used instead of performing a motor commissioning at
//!             startup.
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
static inline void CTRL_setFlag_enableUserMotorParams(CTRL_Handle handle,const bool state)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enableUserMotorParams = state;

  return;
} // end of CTRL_setFlag_enableUserMotorParams() function


//! \brief      Sets the gain values for the specified controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  ctrlType  The controller type
//! \param[in]  Kp        The Kp gain value, pu
//! \param[in]  Ki        The Ki gain value, pu
//! \param[in]  Kd        The Kd gain value, pu
void CTRL_setGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                   const float_t Kp,const float_t Ki,const float_t Kd);


//! \brief      Sets the alpha/beta current (Iab) input vector values in the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  pIab_in_pu  The vector of the alpha/beta current input vector values, pu
static inline void CTRL_setIab_in_pu(CTRL_Handle handle,const MATH_vec2 *pIab_in_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Iab_in.value[0] = pIab_in_pu->value[0];
  obj->Iab_in.value[1] = pIab_in_pu->value[1];

  return;
} // end of CTRL_setIab_in_pu() function


//! \brief      Sets the alpha/beta filtered current vector values in the controller
//! \param[in]  handle        The controller (CTRL) handle
//! \param[in]  pIab_filt_pu  The vector of the alpha/beta filtered current vector values, pu
void CTRL_setIab_filt_pu(CTRL_Handle handle,const MATH_vec2 *pIab_filt_pu);


//! \brief      Sets the direct current (Id) reference value in the controller
//! \param[in]  handle     The controller (CTRL) handle
//! \param[in]  Id_ref_pu  The direct current reference value, pu
static inline void CTRL_setId_ref_pu(CTRL_Handle handle,const float_t Id_ref_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_ref.value[0] = Id_ref_pu;

  return;
} // end of CTRL_setId_ref_pu() function


//! \brief      Sets the direct/quadrature current (Idq) input vector values in the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  pIdq_in_pu  The vector of the direct/quadrature current input vector values, pu
static inline void CTRL_setIdq_in_pu(CTRL_Handle handle,const MATH_vec2 *pIdq_in_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_in.value[0] = pIdq_in_pu->value[0];
  obj->Idq_in.value[1] = pIdq_in_pu->value[1];

  return;
} // end of CTRL_setIdq_in_pu() function


//! \brief     Sets the direct/quadrature current (Idq) offset vector values in the controller
//! \param[in] handle          The controller (CTRL) handle
//! \param[in] pIdq_offset_pu  The vector of the direct/quadrature current offset vector values, pu
static inline void CTRL_setIdq_offset_pu(CTRL_Handle handle,const MATH_vec2 *pIdq_offset_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_offset_pu.value[0] = pIdq_offset_pu->value[0];
  obj->Idq_offset_pu.value[1] = pIdq_offset_pu->value[1];

  return;
} // end of CTRL_setIdq_offset_pu() function


//! \brief      Sets the direct/quadrature current (Idq) reference vector values in the controller
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  pIdq_ref_pu  The vector of the direct/quadrature current reference vector values, pu
static inline void CTRL_setIdq_ref_pu(CTRL_Handle handle,const MATH_vec2 *pIdq_ref_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_ref.value[0] = pIdq_ref_pu->value[0];
  obj->Idq_ref.value[1] = pIdq_ref_pu->value[1];

  return;
} // end of CTRL_setIdq_ref_pu() function


//! \brief      Sets the Id rated current value in the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  IdRated_pu  The Id rated current value, pu
static inline void CTRL_setIdRated_pu(CTRL_Handle handle,const float_t IdRated_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->IdRated = IdRated_pu;

  return;
} // end of CTRL_setIdRated_pu() function


//! \brief     Sets the integrator (Ui) value in the specified controller immediately
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] Ui      The Ui value
static inline void CTRL_setImmediateUi(CTRL_Handle handle,const CTRL_Type_e ctrlType,const float_t Ui)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  if(ctrlType == CTRL_Type_PID_spd)
    {
      PID_setUi(obj->pidHandle_spd,Ui);
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
    PID_setUi(obj->pidHandle_Id,Ui);
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
    PID_setUi(obj->pidHandle_Iq,Ui);
    }

  return;
} // end of CTRL_setUi() function


//! \brief      Sets the quadrature current (Iq) reference value in the controller
//! \param[in]  handle     The controller (CTRL) handle
//! \param[in]  IqRef_pu  The quadrature current reference value, pu
static inline void CTRL_setIq_ref_pu(CTRL_Handle handle,const float_t IqRef_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_ref.value[1] = IqRef_pu;

  return;
} // end of CTRL_setIq_ref_pu() function


//! \brief      Sets the derivative gain (Kd) value for the specified controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  ctrlType  The controller type
//! \param[in]  Kd        The Kd value
static inline void CTRL_setKd(CTRL_Handle handle,const CTRL_Type_e ctrlType,const float_t Kd)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  if(ctrlType == CTRL_Type_PID_spd)
    {
      obj->Kd_spd = Kd;
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      obj->Kd_Id = Kd;
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      obj->Kd_Iq = Kd;
    }

  return;
} // end of CTRL_setKd() function


//! \brief      Sets the integral gain (Ki) value for the specified controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  ctrlType  The controller type
//! \param[in]  Ki        The Ki value
static inline void CTRL_setKi(CTRL_Handle handle,const CTRL_Type_e ctrlType,const float_t Ki)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  if(ctrlType == CTRL_Type_PID_spd)
    {
      obj->Ki_spd = Ki;
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      obj->Ki_Id = Ki;
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      obj->Ki_Iq = Ki;
    }

  return;
} // end of CTRL_setKi() function


//! \brief      Sets the proportional gain (Kp) value for the specified controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  ctrlType  The controller type
//! \param[in]  Kp        The Kp value
static inline void CTRL_setKp(CTRL_Handle handle,const CTRL_Type_e ctrlType,const float_t Kp)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  if(ctrlType == CTRL_Type_PID_spd)
    {
      obj->Kp_spd = Kp;
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      obj->Kp_Id = Kp;
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      obj->Kp_Iq = Kp;
    }

  return;
} // end of CTRL_setKp() function


//! \brief      Sets the high frequency inductance (Lhf) value in the controller
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  Lhf     The Lhf value
static inline void CTRL_setLhf(CTRL_Handle handle,const float_t Lhf)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Lhf = Lhf;

  return;
} // end of CTRL_setLhf() function


//! \brief      Sets the magnetizing current value in the controller
//! \param[in]  handle         The controller (CTRL) handle
//! \param[in]  magCurrent_pu  The magnetizing current value, pu
void CTRL_setMagCurrent_pu(CTRL_Handle handle,const float_t magCurrent_pu);


//! \brief      Sets the maximum voltage vector in the controller
//! \param[in]  handle        The controller (CTRL) handle
//! \param[in]  maxVsMag      The maximum voltage vector (value betwen 0 and 4/3)
static inline void CTRL_setMaxVsMag_pu(CTRL_Handle handle,const float_t maxVsMag)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->maxVsMag_pu = maxVsMag;

  return;
} // end of CTRL_setmaxVsMag_pu() function


//! \brief      Sets the maximum acceleration of the speed controller
//! \details    Sets the maximum acceleration rate of the speed reference.
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  maxAccel_pu  The maximum acceleration (value betwen 0 and 1)
static inline void CTRL_setMaxAccel_pu(CTRL_Handle handle,const float_t maxAccel_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->traj_spd.maxDelta = maxAccel_pu;

  return;
} // end of CTRL_setMaxAccel_pu() function


//! \brief     Sets the maximum speed value in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] maxSpeed_pu  The maximum speed value, pu
void CTRL_setMaximumSpeed_pu(CTRL_Handle handle,const float_t maxSpeed_pu);


//! \brief     Sets the parameters for the motor in the controller
//! \param[in] handle        The controller (CTRL) handle
//! \param[in] motorType     The motor type
//! \param[in] numPolePairs  The number of motor pole pairs
//! \param[in] ratedFlux     The rated flux value, V*sec
//! \param[in] Ls_d          The direct stator inductance, Henry
//! \param[in] Ls_q          The quadrature stator inductance, Henry
//! \param[in] Rr            The rotor resistance, ohm
//! \param[in] Rs            The stator resitance, ohm
static inline void CTRL_setMotorParams(CTRL_Handle handle,
                                const MOTOR_Type_e motorType,
                                const uint_least16_t numPolePairs,
                                const float_t ratedFlux,
                                const float_t Ls_d,
                                const float_t Ls_q,
                                const float_t Rr,
                                const float_t Rs)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->motorParams.type = motorType;
  obj->motorParams.numPolePairs = numPolePairs;
  obj->motorParams.ratedFlux_VpHz = ratedFlux;
  obj->motorParams.Ls_d_H = Ls_d;
  obj->motorParams.Ls_q_H = Ls_q;
  obj->motorParams.Rr_Ohm = Rr;
  obj->motorParams.Rs_Ohm = Rs;

  return;
} // end of CTRL_setMotorParams() function

extern CTRL_Handle CTRL_init(void *pMemory, const size_t numBytes);

//! \brief      Sets the controller parameters
//! \details    This function allows for updates in scale factors during real-time
//!             operation of the controller.
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  pUserParams  The pointer to the user parameters
void CTRL_setParams(CTRL_Handle handle,USER_Params *pUserParams);


//! \brief     Sets the number of controller clock ticks per current controller clock tick
//! \param[in] handle                      The controller (CTRL) handle
//! \param[in] numCtrlTicksPerCurrentTick  The number of controller clock ticks per estimator clock tick
static inline void CTRL_setNumCtrlTicksPerCurrentTick(CTRL_Handle handle,
                                               const uint_least16_t numCtrlTicksPerCurrentTick)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->numCtrlTicksPerCurrentTick = numCtrlTicksPerCurrentTick;

  return;
} // end of CTRL_setNumCtrlTicksPerCurrentTick() function


//! \brief     Sets the number of controller clock ticks per speed controller clock tick
//! \param[in] handle                    The controller (CTRL) handle
//! \param[in] numCtrlTicksPerSpeedTick  The number of controller clock ticks per speed clock tick
static inline void CTRL_setNumCtrlTicksPerSpeedTick(CTRL_Handle handle,
                                               const uint_least16_t numCtrlTicksPerSpeedTick)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->numCtrlTicksPerSpeedTick = numCtrlTicksPerSpeedTick;

  return;
} // end of CTRL_setNumCtrlTicksPerSpeedTick() function


//! \brief     Sets the number of controller clock ticks per trajectory clock tick
//! \param[in] handle                   The controller (CTRL) handle
//! \param[in] numCtrlTicksPerTrajTick  The number of controller clock ticks per trajectory clock tick
static inline void CTRL_setNumCtrlTicksPerTrajTick(CTRL_Handle handle,
                                               const uint_least16_t numCtrlTicksPerTrajTick)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->numCtrlTicksPerTrajTick = numCtrlTicksPerTrajTick;

  return;
} // end of CTRL_setNumCtrlTicksPerTrajTick() function


//! \brief     Sets the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
//! \param[in] handle                  The controller (CTRL) handle
//! \param[in] numIsrTicksPerCtrlTick  The number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
static inline void CTRL_setNumIsrTicksPerCtrlTick(CTRL_Handle handle,
                                           const uint_least16_t numIsrTicksPerCtrlTick)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->numIsrTicksPerCtrlTick = numIsrTicksPerCtrlTick;

  return;
} // end of CTRL_setNumIsrTicksPerCtrlTick() function


//! \brief     Sets the high frequency resistance (Rhf) value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] Rhf     The Rhf value
static inline void CTRL_setRhf(CTRL_Handle handle,const float_t Rhf)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Rhf = Rhf;

  return;
} // end of CTRL_setRhf() function


//! \brief     Sets the R/L value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] RoverL  The R/L value
static inline void CTRL_setRoverL(CTRL_Handle handle,const float_t RoverL)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->RoverL = RoverL;

  return;
} // end of CTRL_setRoverL() function


//! \brief     Sets the feedback speed value in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] speed_fb_pu  The feedback speed value, pu
static inline void CTRL_setSpeed_fb_pu(CTRL_Handle handle,const float_t speed_fb_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->speed_fb_pu = speed_fb_pu;

  return;
} // end of CTRL_setSpeed_fb_pu() function


//! \brief     Sets the maximum speed value in the controller
//! \param[in] handle     The controller (CTRL) handle
//! \param[in] maxSpd_pu  The maximum speed value, pu
static inline void CTRL_setSpd_max_pu(CTRL_Handle handle,const float_t maxSpd_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->spd_max = maxSpd_pu;

  return;
} // end of CTRL_setSpd_max_pu() function


//! \brief     Sets the output speed value in the controller
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] spd_out_pu  The output speed value, pu
static inline void CTRL_setSpd_out_pu(CTRL_Handle handle,const float_t spd_out_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->spd_out = spd_out_pu;

  return;
} // end of CTRL_setSpd_out_pu() function


//! \brief      Sets the maximum output value for the speed controller
//! \param[in]  handle           The controller (CTRL) handle
//! \param[in]  speed_outMax_pu  The maximum output value for the speed controller
static inline void CTRL_setSpeed_outMax_pu(CTRL_Handle handle,const float_t speed_outMax_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->speed_outMax_pu = speed_outMax_pu;

  return;
} // end of CTRL_setSpeed_outMax_pu() function


//! \brief     Sets the output speed reference value in the controller
//! \param[in] handle        The controller (CTRL) handle
//! \param[in] speed_ref_pu  The output speed reference value, pu
static inline void CTRL_setSpeed_ref_pu(CTRL_Handle handle,const float_t speed_ref_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->speed_ref_pu = speed_ref_pu;

  return;
} // end of CTRL_setSpeed_ref_pu() function


//! \brief     Sets the output speed reference value in the controller
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] spd_ref_pu  The output speed reference value, pu
extern void CTRL_setSpd_ref_pu(CTRL_Handle handle,const float_t spd_ref_pu);


//! \brief     Sets the output speed reference value in the controller
//! \param[in] handle        The controller (CTRL) handle
//! \param[in] spd_ref_krpm  The output speed reference value, krpm
void CTRL_setSpd_ref_krpm(CTRL_Handle handle,const float_t spd_ref_krpm);


//! \brief     Sets the controller state
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state       The new state
static inline void CTRL_setState(CTRL_Handle handle,const CTRL_State_e state)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->prevState = obj->state;
  obj->state = state;

  return;
} // end of CTRL_setState() function


//! \brief      Sets the trajectory execution frequency
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  trajFreq_Hz  The trajectory execution frequency, Hz
static inline void CTRL_setTrajFreq_Hz(CTRL_Handle handle,const uint_least32_t trajFreq_Hz)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->trajFreq_Hz = trajFreq_Hz;

  return;
} // end of CTRL_setTrajFreq_Hz() function


//! \brief      Sets the trajectory execution period
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  trajPeriod_sec  The trajectory execution period, sec
static inline void CTRL_setTrajPeriod_sec(CTRL_Handle handle,const float_t trajPeriod_sec)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->trajPeriod_sec = trajPeriod_sec;

  return;
} // end of CTRL_setTrajPeriod_sec() function


//! \brief     Sets the integrator (Ui) value in the specified controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \param[in] Ui      The Ui value
static inline void CTRL_setUi(CTRL_Handle handle,const CTRL_Type_e ctrlType,const float_t Ui)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  if(ctrlType == CTRL_Type_PID_spd)
    {
      obj->Ui_spd = Ui;
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      obj->Ui_Id = Ui;
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      obj->Ui_Iq = Ui;
    }

  return;
} // end of CTRL_setUi() function


//! \brief      Sets the number of current sensors
//! \details    Different algorithms are used for calculating the Clarke transform
//!             when different number of currents are read in.
//! \param[in]  handle             The controller (CTRL) handle
//! \param[in]  numCurrentSensors  The number of current sensors
void CTRL_setupClarke_I(CTRL_Handle handle,uint_least8_t numCurrentSensors);


//! \brief      Sets the number of voltage sensors
//! \details    Different algorithms are used for calculating the Clarke transform
//!             when different number of voltages are read in.
//! \param[in]  handle             The controller (CTRL) handle
//! \param[in]  numVoltageSensors  The number of voltage sensors
void CTRL_setupClarke_V(CTRL_Handle handle,uint_least8_t numVoltageSensors);


//! \brief Sets up the controller and trajectory generator for the estimator idle state
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_setupEstIdleState(CTRL_Handle handle)
{
}


//! \brief     Sets up the controller and trajectory generator for the estimator online state
//! \param[in] handle   The controller (CTRL) handle
static inline void CTRL_setupEstOnLineState(CTRL_Handle handle)
{
}


//! \brief     Sets up the speed and current controllers
void CTRL_setup_user(CTRL_Handle handle,
                     const float_t angle_pu,
                     const float_t speed_ref_pu,
                     const float_t speed_fb_pu,
                     const float_t speed_outMax_pu,
                     const MATH_vec2 *pIdq_offset_pu,
                     const MATH_vec2 *pVdq_offset_pu,
                     const bool flag_enableSpeedCtrl,
                     const bool flag_enableCurrentCtrl);


//! \brief Sets the controller and estimator with motor parameters from the user.h file
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_setUserMotorParams(CTRL_Handle handle)
{
}


//! \brief     Sets the alpha/beta voltage input vector values in the controller
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pVab_in_pu  The vector of alpha/beta voltage input vector values, pu
static inline void CTRL_setVab_in_pu(CTRL_Handle handle,const MATH_vec2 *pVab_in_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Vab_in.value[0] = pVab_in_pu->value[0];
  obj->Vab_in.value[1] = pVab_in_pu->value[1];

  return;
} // end of CTRL_setVab_in_pu() function


//! \brief     Sets the alpha/beta current output vector values in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] pVab_out_pu  The vector of alpha/beta current output vector values, pu
static inline void CTRL_setVab_out_pu(CTRL_Handle handle,const MATH_vec2 *pVab_out_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Vab_out.value[0] = pVab_out_pu->value[0];
  obj->Vab_out.value[1] = pVab_out_pu->value[1];

  return;
} // end of CTRL_setVab_out_pu() function


//! \brief     Sets the direct/quadrature voltage (Vdq) offset vector values in the controller
//! \param[in] handle          The controller (CTRL) handle
//! \param[in] pVdq_offset_pu  The vector of the direct/quadrature voltage offset vector values, pu
static inline void CTRL_setVdq_offset_pu(CTRL_Handle handle,const MATH_vec2 *pVdq_offset_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Vdq_offset_pu.value[0] = pVdq_offset_pu->value[0];
  obj->Vdq_offset_pu.value[1] = pVdq_offset_pu->value[1];

  return;
} // end of CTRL_setVdq_offset_pu() function


//! \brief     Sets the direct/quadrature voltage output vector values in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] pVdq_out_pu  The vector of direct/quadrature voltage output vector values, pu
static inline void CTRL_setVdq_out_pu(CTRL_Handle handle,const MATH_vec2 *pVdq_out_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Vdq_out.value[0] = pVdq_out_pu->value[0];
  obj->Vdq_out.value[1] = pVdq_out_pu->value[1];

  return;
} // end of CTRL_setVdq_out_pu() function


//! \brief     Sets the wait times for the controller states
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pWaitTimes  A pointer to a vector of wait times, controller clock counts
void CTRL_setWaitTimes(CTRL_Handle handle,const uint_least32_t *pWaitTimes);


//! \brief     Sets up the controller (CTRL) object and all of the subordinate objects
//! \details    Is responsible for updating the CTRL state machine and must be called
//!             in the same timing sequence as CTRL_run().
//! \param[in] handle   The controller (CTRL) handle
void CTRL_setup(CTRL_Handle handle);


//! \brief     Sets up the controllers
//! \param[in] handle   The controller (CTRL) handle
static inline void CTRL_setupCtrl(CTRL_Handle handle)
{
}


//! \brief     Sets up the estimator (CTRL) object
//! \param[in] handle   The controller (CTRL) handle
void CTRL_setupEst(CTRL_Handle handle);


//! \brief     Sets up the trajectory (TRAJ) object
//! \param[in] handle   The controller (CTRL) handle
static inline void CTRL_setupTraj(CTRL_Handle handle)
{
}


//! \brief      Updates the controller state
//! \param[in]  handle  The controller (CTRL) handle
//! \return     A boolean value denoting if the state has changed (true) or not (false)
bool CTRL_updateState(CTRL_Handle handle);


//! \brief     Determines if a zero Iq current reference should be used in the controller
//! \param[in] handle   The controller (CTRL) handle
//! \return    A boolean value denoting if a zero Iq current reference should be used (true) or not (false)
static inline bool CTRL_useZeroIq_ref(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(EST_useZeroIq_ref(obj->estHandle));
} // end of CTRL_useZeroIq_ref() function


//! \brief      Checks for any controller errors and, if found, sets the controller state to the error state
//! \param[in]  handle  The controller (CTRL) handle
static inline void CTRL_checkForErrors(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // check for estimator errors
  if(EST_isError(obj->estHandle))
    {
      // set the next controller state
      CTRL_setState(handle,CTRL_State_Error);

      // set the error code
      CTRL_setErrorCode(handle,CTRL_ErrorCode_EstError);
    }

  return;
} // end of CTRL_checkForErrors() function


//! \brief     Computes the Id controller output limits
//! \param[in] handle        The controller (CTRL) handle
//! \param[in] maxVsMag      The maximum output voltage vector, pu
//! \param[in] outMin        The pointer to the minimum output value
//! \param[in] outMax        The pointer to the maximum output value
static inline void CTRL_computeOutputLimits_Id(CTRL_Handle handle,
                                               const float_t maxVsMag,
                                               float_t *outMin,float_t *outMax)//! \Mod By Dl.K
{

  *outMin = -maxVsMag;
  *outMax = maxVsMag;

  return;
} // end of CTRL_computeOutputLimits_Id() function


//! \brief     Computes the Iq controller output limits
//! \param[in] handle        The controller (CTRL) handle
//! \param[in] maxVsMag      The maximum output voltage vector, pu
//! \param[in] Vd_out_pu     The Vd output voltage value, pu
//! \param[in] outMin        The pointer to the minimum output value
//! \param[in] outMax        The pointer to the maximum output value
static inline void CTRL_computeOutputLimits_Iq(CTRL_Handle handle,
                                               const float_t maxVsMag,
                                               const float_t Vd_out_pu,
                                               float_t *outMin,float_t *outMax)//! \Mod By Dl.K
{
  float_t tmp = sqrtf((maxVsMag*maxVsMag) - (Vd_out_pu*Vd_out_pu));

  *outMin = -tmp;
  *outMax = tmp;

  return;
} // end of CTRL_computeOutputLimits_Iq() function


//! \brief      Computes a phasor for a given angle
//! \param[in]  angle_pu  The angle, pu
//! \param[out] pPhasor   The pointer to the phasor vector values
static inline void CTRL_computePhasor(const float_t angle_pu,MATH_vec2 *pPhasor)//! \Mod By Dl.K
{

  pPhasor->value[0] = cosf(angle_pu);
  pPhasor->value[1] = sinf(angle_pu);

  return;
} // end of CTRL_computePhasor() function


//! \brief      Determines if the current controllers should be run
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The value denoting that the current controllers should be run (true) or not (false)
static inline bool CTRL_doCurrentCtrl(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  bool result = false;

  if(CTRL_getFlag_enableCurrentCtrl(handle) && (obj->counter_current >= obj->numCtrlTicksPerCurrentTick))
  {
    result = true;
  }

  return(result);
} // end of CTRL_doCurrentCtrl() function


//! \brief     Determines if the speed controller should be executed
//! \param[in] handle  The controller (CTRL) handle
//! \return    A boolean value denoting if the speed controller should be executed (true) or not (false)
static inline bool CTRL_doSpeedCtrl(CTRL_Handle handle)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  bool result = false;

  if((obj->counter_speed >= obj->numCtrlTicksPerSpeedTick))
  {
    result = true;
  }

  return(result);
} // end of CTRL_doSpeedCtrl() function


//! \brief      Runs the offline controller
//! ADC一阶滤波，设置theta为0，PWM输出设为0
//! \param[in]  handle     The controller (CTRL) handle
//! \param[in]  halHandle  The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData   The pointer to the ADC data
//! \param[out] pPwmData   The pointer to the PWM data

static inline void CTRL_runOffLine(CTRL_Handle handle,HAL_Handle halHandle,
                            const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData, HAL_QepData_t *pQepData)//! \Mod By Dl.K
{

	// run offset estimation
	//settestpin();
	HAL_runOffsetEst(halHandle,pAdcData);  //对ADC一阶滤波
	//cleartestpin();
  
	Speed_SetElecTheta(handle->Speed_handle,0.0f);
	Speed_UpdateElecTheta(handle->Speed_handle);
	
	// create PWM data
	pPwmData->Tabc.value[0] = (0.0f);
	pPwmData->Tabc.value[1] = (0.0f);
	pPwmData->Tabc.value[2] = (0.0f);

	handle->MntValue.value[0] = halHandle->offset_I[0].value;
	handle->MntValue.value[1] = halHandle->offset_I[1].value;
	handle->MntValue.value[2] = halHandle->offset_I[2].value;
  return;
} // end of CTRL_runOffLine() function


//! \brief      Sets maximum speed controller output
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  spdMax  The maximum allowed output of the speed controller
static inline void CTRL_setSpdMax(CTRL_Handle handle, const float_t spdMax)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // set the maximum output of the speed controller, or Maximum Iq reference
  TRAJ_setIntValue(obj->trajHandle_spdMax, spdMax);

  return;
} // end of CTRL_setSpdMax() function


//! \brief      Runs angle delay compensation
//! \details    This function takes the decimation rates to calculate a phase delay,
//!             and it compensates the delay. This allows the voltage output to do a
//!             better correction of the error.
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  angle_pu  The angle delayed
//! \return     The phase delay compensated angle
static inline float_t CTRL_angleDelayComp(CTRL_Handle handle, const float_t angle_pu)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t angleDelta_pu = (EST_getFm_pu(obj->estHandle)*(USER_IQ_FULL_SCALE_FREQ_Hz/(USER_PWM_FREQ_kHz*1000.0f)));
  float_t angleUncomp_pu = angle_pu;
  float_t angleCompFactor = (1.0f + (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK * (float_t)USER_NUM_ISR_TICKS_PER_CTRL_TICK * ((float_t)USER_NUM_CTRL_TICKS_PER_EST_TICK - 0.5f));
  float_t angleDeltaComp_pu = (angleDelta_pu*angleCompFactor);
//  uint32_t angleMask = ((uint32_t)0xFFFFFFFF >> (32 - GLOBAL_Q));//! \Mod By Dl.K
  float_t angleTmp_pu;
  float_t angleComp_pu;

  // increment the angle
  angleTmp_pu = angleUncomp_pu + angleDeltaComp_pu;

  // mask the angle for wrap around
  // note: must account for the sign of the angle
//  angleComp_pu = fabs(angleTmp_pu) & angleMask;////???_IQabs(angleTmp_pu) & angleMask;//! \Mod By Dl.K

  // account for sign
  if(angleTmp_pu < 0.0f)
    {
   	  angleComp_pu = -angleComp_pu;
    }

  return(angleComp_pu);
}


//! \brief      Runs the online controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  pAdcData  The pointer to the ADC data
//! \param[out] pPwmData  The pointer to the PWM data
static inline void CTRL_runOnLine(CTRL_Handle handle,
                           const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData, HAL_QepData_t *pQepData)//! \Mod By Dl.K
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  float_t angle_pu;

  MATH_vec2 phasor;


 // run Clarke transform on current
 CLARKE_run(obj->clarkeHandle_I,&pAdcData->I,CTRL_getIab_in_addr(handle));
  

 // run Clarke transform on voltage
 CLARKE_run(obj->clarkeHandle_V,&pAdcData->V,CTRL_getVab_in_addr(handle));


 // run the estimator
 EST_run(obj->estHandle,CTRL_getIab_in_addr(handle),CTRL_getVab_in_addr(handle),
         pAdcData->VdcBus,TRAJ_getIntValue(obj->trajHandle_spd));


 // generate the motor electrical angle
 angle_pu = EST_getAngle_pu(obj->estHandle);


 // compute the sin/cos phasor
 CTRL_computePhasor(angle_pu,&phasor);


 // set the phasor in the Park transform
 PARK_setPhasor(obj->parkHandle,&phasor);


 // run the Park transform
 PARK_run(obj->parkHandle,CTRL_getIab_in_addr(handle),CTRL_getIdq_in_addr(handle));

 
 // when appropriate, run the PID speed controller
 if(EST_doSpeedCtrl(obj->estHandle))
   {
     if(CTRL_doSpeedCtrl(handle))
       {
         float_t refValue = TRAJ_getIntValue(obj->trajHandle_spd);
         float_t fbackValue = EST_getFm_pu(obj->estHandle);
         float_t outMax = TRAJ_getIntValue(obj->trajHandle_spdMax);
         float_t outMin = -outMax;

         // reset the speed count
         CTRL_resetCounter_speed(handle);

         PID_setMinMax(obj->pidHandle_spd,outMin,outMax);

         PID_run_spd(obj->pidHandle_spd,refValue,fbackValue,CTRL_getSpd_out_addr(handle));
       }
   }
 else
   {
     // zero the speed command
     CTRL_setSpd_out_pu(handle,(0.0f));

     // reset the integrator 
     PID_setUi(obj->pidHandle_spd,(0.0f));
   }


 // when appropriate, run the PID Id and Iq controllers
 if(CTRL_doCurrentCtrl(handle) && EST_doCurrentCtrl(obj->estHandle))
   {
     float_t Kp_Id = CTRL_getKp(handle,CTRL_Type_PID_Id);
     float_t Kp_Iq = CTRL_getKp(handle,CTRL_Type_PID_Iq);
     float_t refValue;
     float_t fbackValue;
     float_t outMin,outMax;
         
     float_t maxVsMag = CTRL_getMaxVsMag_pu(handle);

     // reset the current count
     CTRL_resetCounter_current(handle);

     // ***********************************
     // configure and run the Id controller

     // compute the Kp gain
     // Scale Kp instead of output to prevent saturation issues
     if(CTRL_getFlag_enableDcBusComp(handle))
       {
         Kp_Id = (Kp_Id*EST_getOneOverDcBus_pu(obj->estHandle));
       }

     PID_setKp(obj->pidHandle_Id,Kp_Id);

     // compute the reference value
     refValue = TRAJ_getIntValue(obj->trajHandle_Id) + CTRL_getId_ref_pu(handle);

     // update the Id reference value
     EST_updateId_ref_pu(obj->estHandle,&refValue);

     // get the feedback value
     fbackValue = CTRL_getId_in_pu(handle);

     // set minimum and maximum for Id controller output
     outMax = maxVsMag;
     outMin = -outMax;

     // set the minimum and maximum values
     PID_setMinMax(obj->pidHandle_Id,outMin,outMax);

     // run the Id PID controller
     PID_run(obj->pidHandle_Id,refValue,fbackValue,CTRL_getVd_out_addr(handle));

     // set the Id reference value in the estimator
     EST_setId_ref_pu(obj->estHandle,refValue);

     // ***********************************
     // configure and run the Iq controller

     // compute the Kp gain
     // Scale Kp instead of output to prevent saturation issues
     if(CTRL_getFlag_enableDcBusComp(handle))
       {
         Kp_Iq = (Kp_Iq*EST_getOneOverDcBus_pu(obj->estHandle));
       }

     PID_setKp(obj->pidHandle_Iq,Kp_Iq);

     // get the reference value
     if(CTRL_getFlag_enableSpeedCtrl(handle))
       {
         if(!CTRL_useZeroIq_ref(handle))
           {
             refValue = CTRL_getSpd_out_pu(handle);
           }
         else
           {
             refValue = (0.0f);
           }
       }
     else
       {
         // get the Iq reference value
         refValue = CTRL_getIq_ref_pu(handle);
       }

     // get the feedback value
     fbackValue = CTRL_getIq_in_pu(handle);

     // generate the Iq PID output limits without square root
     outMax = maxVsMag - fabs(CTRL_getVd_out_pu(handle));
     outMin = -outMax;

     // set the minimum and maximum values
     PID_setMinMax(obj->pidHandle_Iq,outMin,outMax);

     // run the Iq PID controller
     PID_run(obj->pidHandle_Iq,refValue,fbackValue,CTRL_getVq_out_addr(handle));

     // set the Iq reference value in the estimator
     EST_setIq_ref_pu(obj->estHandle,refValue);
   }


 // set the phasor in the inverse Park transform
 IPARK_setPhasor(obj->iparkHandle,&phasor);


 // run the inverse Park module
 IPARK_run(obj->iparkHandle,CTRL_getVdq_out_addr(handle),CTRL_getVab_out_addr(handle));


 // run the space Vector Generator (SVGEN) module
 SVGEN_run(obj->svgenHandle,CTRL_getVab_out_addr(handle),&(pPwmData->Tabc));

 return;
} // end of CTRL_runOnLine() function


//! \brief      Runs the online user controller
//! \details    An implementation of the field oriented control.  The online user controller
//!             is executed in user's memory i.e. RAM/FLASH and can be changed in any way
//!             suited to the user.
//!             Dynamic Stator Initial Position Detection
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  pAdcData  The pointer to the ADC data
//! \param[out] pPwmData  The pointer to the PWM data
static inline void CTRL_runInitDete_User(CTRL_Handle handle,HAL_Handle halHandle,
                           const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData, HAL_QepData_t *pQepData)//! \Mod By Dl.K
{
	//! \Mod By Dl.K Dynamic Stator Initial Position Detection
	CTRL_Obj *obj = (CTRL_Obj *)handle;
	static uint16_t flag=true;
	MATH_vec2 phasor;
	MATH_vec2 vdq = {0.0f,0.0f};
	int16_t angle_temp_pu;

	// run Clarke transform on current
	CLARKE_run(obj->clarkeHandle_I,&pAdcData->I,CTRL_getIab_in_addr(handle));

	// clear the position counter
//	QEP_clear_posn_counter(halHandle->qepHandle[0]);
	
	angle_temp_pu = 32768.0f*pQepData->qep_cnt*USER_MOTOR_NUM_POLE_PAIRS;
	angle_temp_pu &= 0x7FFF;
	obj->angle_offset_pu = angle_temp_pu *0.0000305f;

	Speed_SetElecTheta(obj->Speed_handle,0.0f);
	Speed_UpdateElecTheta(obj->Speed_handle);
	
	// compute the sin/cos phasor
	CTRL_computePhasor(obj->AngleRef * MATH_TWO_PI,&phasor);//-0.25

	// set
	if( flag==true )
	{
		vdq.value[0] = (0.0f);
		vdq.value[1] = (0.1f);
	}
	else
	{
		vdq.value[0] = (0.0f);
		vdq.value[1] = (0.0f);
	}
	CTRL_setVdq_out_pu(obj,&vdq);

	// set the phasor in the inverse Park transform
	IPARK_setPhasor(obj->iparkHandle,&phasor);

	// run the inverse Park module
	IPARK_run(obj->iparkHandle,CTRL_getVdq_out_addr(obj),CTRL_getVab_out_addr(obj));

	// run the space Vector Generator (SVGEN) module
	SVGEN_run(obj->svgenHandle,CTRL_getVab_out_addr(handle),&(pPwmData->Tabc));

	obj->Tabc.value[0]=pPwmData->Tabc.value[0];
	obj->Tabc.value[1]=pPwmData->Tabc.value[1];
	obj->Tabc.value[2]=pPwmData->Tabc.value[2];
}


//! \brief      Runs the online user controller
//! \details    An implementation of the field oriented control.  The online user controller
//!             is executed in user's memory i.e. RAM/FLASH and can be changed in any way
//!             suited to the user.
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  pAdcData  The pointer to the ADC data
//! \param[out] pPwmData  The pointer to the PWM data
static inline void CTRL_runStaticStatorInitDete_User(CTRL_Handle handle,HAL_Handle halHandle,
                           const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData, HAL_QepData_t *pQepData)//! \Mod By Dl.K
{
	//! \Mod By Dl.K Static Stator Initial Position Detection
	CTRL_Obj *obj = (CTRL_Obj *)handle;
	float_t angle_pu;
	MATH_vec2 phasor;
	MATH_vec2 vdq = {0.0f,0.0f};
	static uint16_t sInitPosDetTick = 5,sInitPosDetSclar = 5;

	//GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_32);

	if( INITPOSDET_GetIdSampAngleFlag(obj->InitPosDet_handle) &&
			sInitPosDetTick==4 )
	{
//		GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_43);

		obj->MntValue.value[0] = pAdcData->I.value[0];
		obj->MntValue.value[1] = pAdcData->I.value[1];
		obj->MntValue.value[2] = pAdcData->I.value[2];

		// run Clarke transform on current
		CLARKE_run(obj->clarkeHandle_I,&pAdcData->I,CTRL_getIab_in_addr(obj));

		// generate the motor electrical angle
		angle_pu = INITPOSDET_GetIdSampAngle(obj->InitPosDet_handle);

		// compute the sin/cos phasor
		CTRL_computePhasor(angle_pu * MATH_TWO_PI,&phasor);

		// set the phasor in the Park transform
		PARK_setPhasor(obj->parkHandle,&phasor);

		// run the Park transform
		PARK_run(obj->parkHandle,CTRL_getIab_in_addr(obj),CTRL_getIdq_in_addr(obj));

		INITPOSDET_Run(obj->InitPosDet_handle,CTRL_getId_in_pu(obj));

		obj->angle_offset_pu = - INITPOSDET_GetInitPosAngle(obj->InitPosDet_handle);
		
		Speed_SetElecTheta(obj->Speed_handle,-obj->angle_offset_pu);
		Speed_UpdateElecTheta(obj->Speed_handle);
//		GPIO_setLow(halHandle->gpioHandle,GPIO_Number_43);
	}

	if(sInitPosDetTick>=sInitPosDetSclar)
	{
		sInitPosDetTick=1;
		INITPOSDET_VolVecAngleRun(obj->InitPosDet_handle);
	}
	else sInitPosDetTick++;

	// set
	if( INITPOSDET_GetVolVecAngleFlag(obj->InitPosDet_handle) )
	{
		vdq.value[0] = obj->VdqRef.value[0];
		vdq.value[1] = (0.0f);

		// generate the motor electrical angle
		angle_pu = INITPOSDET_GetVolVecAngle(obj->InitPosDet_handle);

		// compute the sin/cos phasor
		CTRL_computePhasor(angle_pu * MATH_TWO_PI,&phasor);
	}
	else
	{
		vdq.value[0] = (0.0f);
		vdq.value[1] = (0.0f);

		// generate the motor electrical angle
		angle_pu = (0.0f);

		// compute the sin/cos phasor
		CTRL_computePhasor(angle_pu * MATH_TWO_PI,&phasor);
	}
	CTRL_setVdq_out_pu(obj,&vdq);

	// set the phasor in the inverse Park transform
	IPARK_setPhasor(obj->iparkHandle,&phasor);

	// run the inverse Park module
	IPARK_run(obj->iparkHandle,CTRL_getVdq_out_addr(obj),CTRL_getVab_out_addr(obj));

	// run the space Vector Generator (SVGEN) module
	SVGEN_run(obj->svgenHandle,CTRL_getVab_out_addr(obj),&(pPwmData->Tabc));

	obj->Tabc.value[0]=pPwmData->Tabc.value[0];
	obj->Tabc.value[1]=pPwmData->Tabc.value[1];
	obj->Tabc.value[2]=pPwmData->Tabc.value[2];

	//GPIO_setLow(halHandle->gpioHandle,GPIO_Number_32);
}


//! \brief      Runs the online user controller
//! \details    An implementation of the field oriented control.  The online user controller
//!             is executed in user's memory i.e. RAM/FLASH and can be changed in any way
//!             suited to the user.
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  pAdcData  The pointer to the ADC data
//! \param[out] pPwmData  The pointer to the PWM data
static inline void CTRL_runOpenLoop_User(CTRL_Handle handle,HAL_Handle halHandle,
                           const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData, HAL_QepData_t *pQepData)//! \Mod By Dl.K
{
	//! \Mod By Dl.K OpenLoop Control
	CTRL_Obj *obj = (CTRL_Obj *)handle;
	int16_t angle_temp_pu;
	float_t angle_pu;
	MATH_vec2 phasor;
	MATH_vec2 vdq = {0,0};

	// run Clarke transform on current
	CLARKE_run(obj->clarkeHandle_I,&pAdcData->I,CTRL_getIab_in_addr(handle));

	RMP_CNTL_SetTargetValue(obj->rmpcntl_handle,CTRL_getSpd_ref_pu(obj));
	RMP_CNTL_Run(obj->rmpcntl_handle);

	RAMPGEN_SetFreq(obj->rampgen_handle,RMP_CNTL_GetFreq(obj->rmpcntl_handle));
	RAMPGEN_Run(obj->rampgen_handle);

	// generate the motor electrical angle
	angle_pu = RAMPGEN_GetAngleOut(obj->rampgen_handle);

	// compute the sin/cos phasor
	CTRL_computePhasor(angle_pu * MATH_TWO_PI,&phasor);

	if(CTRL_doSpeedCtrl(handle))
	{
		angle_temp_pu = 32768.0f*pQepData->qep_cnt*USER_MOTOR_NUM_POLE_PAIRS;
		angle_temp_pu &= 0x7FFF;
		angle_pu = angle_temp_pu *0.0000305f;
		Speed_SetElecTheta(obj->Speed_handle, angle_pu * MATH_TWO_PI);
		Speed_Run(obj->Speed_handle);
	}

	// set
	vdq.value[0] = (0.0f);
	vdq.value[1] = ((15.0f)*RMP_CNTL_GetFreq(obj->rmpcntl_handle));//ESTUN: (10.0), PHASE: (15.0)
	if( vdq.value[1] > (0.2f) )
	{
		vdq.value[1] = (0.2f);
	}
	CTRL_setVdq_out_pu(obj,&vdq);

	// set the phasor in the inverse Park transform
	IPARK_setPhasor(obj->iparkHandle,&phasor);

	// run the inverse Park module
	IPARK_run(obj->iparkHandle,CTRL_getVdq_out_addr(obj),CTRL_getVab_out_addr(obj));

	// run the space Vector Generator (SVGEN) module
	SVGEN_run(obj->svgenHandle,CTRL_getVab_out_addr(obj),&(pPwmData->Tabc));

	obj->Tabc.value[0]=pPwmData->Tabc.value[0];
	obj->Tabc.value[1]=pPwmData->Tabc.value[1];
	obj->Tabc.value[2]=pPwmData->Tabc.value[2];

 return;
} // end of CTRL_runOnLine_User() function

//! \brief      Runs the online user controller
//! \details    An implementation of the field oriented control.  The online user controller
//!             is executed in user's memory i.e. RAM/FLASH and can be changed in any way
//!             suited to the user.
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  pAdcData  The pointer to the ADC data
//! \param[out] pPwmData  The pointer to the PWM data
static inline void CTRL_runOnLine_User(CTRL_Handle handle,HAL_Handle halHandle,
                           const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData, HAL_QepData_t *pQepData)//! \Mod By Dl.K
{
/*  CTRL_Obj *obj = (CTRL_Obj *)handle;

  float_t angle_pu;

  MATH_vec2 phasor;


 // run Clarke transform on current
 CLARKE_run(obj->clarkeHandle_I,&pAdcData->I,CTRL_getIab_in_addr(handle));


 // run Clarke transform on voltage
 CLARKE_run(obj->clarkeHandle_V,&pAdcData->V,CTRL_getVab_in_addr(handle));


 // run the estimator
 EST_run(obj->estHandle,CTRL_getIab_in_addr(handle),CTRL_getVab_in_addr(handle),
         pAdcData->dcBus,TRAJ_getIntValue(obj->trajHandle_spd));


 // generate the motor electrical angle
 angle_pu = EST_getAngle_pu(obj->estHandle);


 // compute the sin/cos phasor
 CTRL_computePhasor(angle_pu,&phasor);


 // set the phasor in the Park transform
 PARK_setPhasor(obj->parkHandle,&phasor);


 // run the Park transform
 PARK_run(obj->parkHandle,CTRL_getIab_in_addr(handle),CTRL_getIdq_in_addr(handle));


 // when appropriate, run the PID speed controller
 if(CTRL_doSpeedCtrl(handle))
   {
     float_t refValue = TRAJ_getIntValue(obj->trajHandle_spd);
     float_t fbackValue = EST_getFm_pu(obj->estHandle);
     float_t outMax = TRAJ_getIntValue(obj->trajHandle_spdMax);
     float_t outMin = -outMax;

     // reset the speed count
     CTRL_resetCounter_speed(handle);

     PID_setMinMax(obj->pidHandle_spd,outMin,outMax);

     PID_run_spd(obj->pidHandle_spd,refValue,fbackValue,CTRL_getSpd_out_addr(handle));
   }


 // when appropriate, run the PID Id and Iq controllers
 if(CTRL_doCurrentCtrl(handle))
   {
     float_t Kp_Id = CTRL_getKp(handle,CTRL_Type_PID_Id);
     float_t Kp_Iq = CTRL_getKp(handle,CTRL_Type_PID_Iq);
     float_t refValue;
     float_t fbackValue;
     float_t outMin,outMax;

     // read max voltage vector to set proper limits to current controllers
     float_t maxVsMag = CTRL_getMaxVsMag_pu(handle);


     // reset the current count
     CTRL_resetCounter_current(handle);

     // ***********************************
     // configure and run the Id controller

     // compute the Kp gain
     // Scale Kp instead of output to prevent saturation issues
     if(CTRL_getFlag_enableDcBusComp(handle))
       {
         Kp_Id = (Kp_Id*EST_getOneOverDcBus_pu(obj->estHandle));
       }

     PID_setKp(obj->pidHandle_Id,Kp_Id);

     // compute the reference value
     refValue = TRAJ_getIntValue(obj->trajHandle_Id) + CTRL_getId_ref_pu(handle);

     // update the Id reference value
     EST_updateId_ref_pu(obj->estHandle,&refValue);

     // get the feedback value
     fbackValue = CTRL_getId_in_pu(handle);

     // set minimum and maximum for Id controller output
     outMax = maxVsMag;
     outMin = -outMax;

     // set the minimum and maximum values
     PID_setMinMax(obj->pidHandle_Id,outMin,outMax);

     // run the Id PID controller
     PID_run(obj->pidHandle_Id,refValue,fbackValue,CTRL_getVd_out_addr(handle));

     // ***********************************
     // configure and run the Iq controller

     // compute the Kp gain
     // Scale Kp instead of output to prevent saturation issues
     if(CTRL_getFlag_enableDcBusComp(handle))
       {
         Kp_Iq = (Kp_Iq*EST_getOneOverDcBus_pu(obj->estHandle));
       }

     PID_setKp(obj->pidHandle_Iq,Kp_Iq);

     // get the reference value
     if(CTRL_getFlag_enableSpeedCtrl(handle))
       {
         refValue = CTRL_getSpd_out_pu(handle);
       }
     else
       {
         // get the Iq reference value
         refValue = CTRL_getIq_ref_pu(handle);
       }

     // get the feedback value
     fbackValue = CTRL_getIq_in_pu(handle);

     // set minimum and maximum for Id controller output
     outMax = sqrtf((maxVsMag*maxVsMag) - (CTRL_getVd_out_pu(handle)*CTRL_getVd_out_pu(handle)));
     outMin = -outMax;

     // set the minimum and maximum values
     PID_setMinMax(obj->pidHandle_Iq,outMin,outMax);

     // run the Iq PID controller
     PID_run(obj->pidHandle_Iq,refValue,fbackValue,CTRL_getVq_out_addr(handle));
   }

   {
     float_t angleComp_pu;


     // compensate angle delay
     angleComp_pu = CTRL_angleDelayComp(handle, angle_pu);


     // compute the sin/cos phasor
     CTRL_computePhasor(angleComp_pu,&phasor);
   }


 // set the phasor in the inverse Park transform
 IPARK_setPhasor(obj->iparkHandle,&phasor);


 // run the inverse Park module
 IPARK_run(obj->iparkHandle,CTRL_getVdq_out_addr(handle),CTRL_getVab_out_addr(handle));


 // run the space Vector Generator (SVGEN) module
 SVGEN_run(obj->svgenHandle,CTRL_getVab_out_addr(handle),&(pPwmData->Tabc));
*/

	CTRL_Obj *obj = (CTRL_Obj *)handle;

	int16_t angle_temp_pu;
	float_t angle_pu;

	MATH_vec2 phasor;


	// run Clarke transform on current
	CLARKE_run(obj->clarkeHandle_I,&pAdcData->I,CTRL_getIab_in_addr(handle));


//	// run Clarke transform on voltage
//	CLARKE_run(obj->clarkeHandle_V,&pAdcData->V,CTRL_getVab_in_addr(handle));


//	// run the estimator
//	EST_run(obj->estHandle,CTRL_getIab_in_addr(handle),CTRL_getVab_in_addr(handle),
//		 pAdcData->dcBus,TRAJ_getIntValue(obj->trajHandle_spd));


//	// generate the motor electrical angle
//	angle_pu = EST_getAngle_pu(obj->estHandle);
//	angle_pu = _IQ13toIQ(QEP_read_posn_count(halHandle->qepHandle[0]))*USER_MOTOR_NUM_POLE_PAIRS;
//	angle_pu &= 0xFFFFFF;
	angle_temp_pu = 32768.0f*pQepData->qep_cnt*USER_MOTOR_NUM_POLE_PAIRS;
	angle_temp_pu &= 0x7FFF;
	angle_pu = angle_temp_pu *0.0000305f - obj->angle_offset_pu;

//	obj->angle_pu = angle_pu;

	// compute the sin/cos phasor
	CTRL_computePhasor(angle_pu * MATH_TWO_PI,&phasor);


	// set the phasor in the Park transform
	PARK_setPhasor(obj->parkHandle,&phasor);


	// run the Park transform
	PARK_run(obj->parkHandle,CTRL_getIab_in_addr(handle),CTRL_getIdq_in_addr(handle));


	// when appropriate, run the PID speed controller
	if(CTRL_doSpeedCtrl(handle))
	{
//		float_t refValue = TRAJ_getIntValue(obj->trajHandle_spd);
//		float_t fbackValue = EST_getFm_pu(obj->estHandle);
//		float_t outMax = TRAJ_getIntValue(obj->trajHandle_spdMax);
//		float_t outMin = -outMax;
		float_t refValue = CTRL_getSpd_ref_pu(obj);
		float_t fbackValue = (0.0f);
		float_t outMax = (0.04f);
		float_t outMin = -outMax;
		float_t Kp_Spd = CTRL_getKp(handle,CTRL_Type_PID_spd);
		float_t Ki_Spd = CTRL_getKi(handle,CTRL_Type_PID_spd);

		// reset the speed count
		CTRL_resetCounter_speed(handle);

		Speed_SetElecTheta(obj->Speed_handle, angle_pu);
		Speed_Run(obj->Speed_handle);
		fbackValue = Speed_GetElecSpeed(obj->Speed_handle);

		PID_setKp(obj->pidHandle_spd,Kp_Spd);
		PID_setKi(obj->pidHandle_spd,Ki_Spd);

		PID_setMinMax(obj->pidHandle_spd,outMin,outMax);

		PID_run_spd(obj->pidHandle_spd,refValue,fbackValue,CTRL_getSpd_out_addr(handle));
	}


	// when appropriate, run the PID Id and Iq controllers
	if(CTRL_doCurrentCtrl(handle))
	{
		float_t Kp_Id = CTRL_getKp(handle,CTRL_Type_PID_Id);
		float_t Kp_Iq = CTRL_getKp(handle,CTRL_Type_PID_Iq);
		float_t Ki_Id = CTRL_getKi(handle,CTRL_Type_PID_Id);
		float_t Ki_Iq = CTRL_getKi(handle,CTRL_Type_PID_Iq);
		float_t refValue;
		float_t fbackValue;
		float_t outMin,outMax;


//		// read max voltage vector to set proper limits to current controllers
//		float_t maxVsMag = CTRL_getMaxVsMag_pu(handle);
		float_t maxVsMag = (0.2f);


		// reset the current count
		CTRL_resetCounter_current(handle);

		// ***********************************
		// configure and run the Id controller

//		// compute the Kp gain
//		// Scale Kp instead of output to prevent saturation issues
//		if(CTRL_getFlag_enableDcBusComp(handle))
//		{
//			Kp_Id = (Kp_Id*EST_getOneOverDcBus_pu(obj->estHandle));
//		}

		PID_setKp(obj->pidHandle_Id,Kp_Id);
		PID_setKi(obj->pidHandle_Id,Ki_Id);

//		// compute the reference value
//		refValue = TRAJ_getIntValue(obj->trajHandle_Id) + CTRL_getId_ref_pu(handle);
		refValue = CTRL_getId_ref_pu(handle);

//		// update the Id reference value
//		EST_updateId_ref_pu(obj->estHandle,&refValue);

		// get the feedback value
		fbackValue = CTRL_getId_in_pu(handle);

		// set minimum and maximum for Id controller output
		outMax = maxVsMag;
		outMin = -outMax;

		// set the minimum and maximum values
		PID_setMinMax(obj->pidHandle_Id,outMin,outMax);

		// run the Id PID controller
		PID_run(obj->pidHandle_Id,refValue,fbackValue,CTRL_getVd_out_addr(handle));

		// ***********************************
		// configure and run the Iq controller

//		// compute the Kp gain
//		// Scale Kp instead of output to prevent saturation issues
//		if(CTRL_getFlag_enableDcBusComp(handle))
//		{
//			Kp_Iq = (Kp_Iq*EST_getOneOverDcBus_pu(obj->estHandle));
//		}

		PID_setKp(obj->pidHandle_Iq,Kp_Iq);
		PID_setKi(obj->pidHandle_Iq,Ki_Iq);

		// get the reference value
		if(CTRL_getFlag_enableSpeedCtrl(handle))
		{
			refValue = CTRL_getSpd_out_pu(handle);
		}
		else
		{
		// get the Iq reference value
			refValue = CTRL_getIq_ref_pu(handle);
		}

		// get the feedback value
		fbackValue = CTRL_getIq_in_pu(handle);

//		// set minimum and maximum for Id controller output
//		outMax = sqrtf((maxVsMag*maxVsMag) - (CTRL_getVd_out_pu(handle)*CTRL_getVd_out_pu(handle)));
//		outMin = -outMax;
		outMax = (0.2f);
		outMin = -outMax;

		// set the minimum and maximum values
		PID_setMinMax(obj->pidHandle_Iq,outMin,outMax);

		// run the Iq PID controller
		PID_run(obj->pidHandle_Iq,refValue,fbackValue,CTRL_getVq_out_addr(handle));
	}

//	{
//		float_t angleComp_pu;
//
//
//		// compensate angle delay
//		angleComp_pu = CTRL_angleDelayComp(handle, angle_pu) ;
//
//
//		// compute the sin/cos phasor
//		CTRL_computePhasor(angleComp_pu * MATH_TWO_PI,&phasor);
//	}


	// set the phasor in the inverse Park transform
	IPARK_setPhasor(obj->iparkHandle,&phasor);


	// run the inverse Park module
	IPARK_run(obj->iparkHandle,CTRL_getVdq_out_addr(handle),CTRL_getVab_out_addr(handle));


	// run the space Vector Generator (SVGEN) module
	SVGEN_run(obj->svgenHandle,CTRL_getVab_out_addr(handle),&(pPwmData->Tabc));


	obj->Tabc.value[0]=pPwmData->Tabc.value[0];
	obj->Tabc.value[1]=pPwmData->Tabc.value[1];
	obj->Tabc.value[2]=pPwmData->Tabc.value[2];
	//	DATA_LOG_Run(obj->datalog_handle);

	obj->MntValue.value[0] = Speed_GetElecSpeed(obj->Speed_handle);
	obj->MntValue.value[1] = angle_pu;
	obj->MntValue.value[2] = CTRL_getVq_out_pu(handle);

return;
} // end of CTRL_runOnLine_User() function


//! \brief      Runs the online controller
//! \param[in]  handle    The controller (CTRL) handle
static inline void CTRL_runPiOnly(CTRL_Handle handle) //! \Mod By Dl.K//,const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

 
  // when appropriate, run the PID speed controller
  if(CTRL_getFlag_enableSpeedCtrl(handle))
  {
    if(CTRL_doSpeedCtrl(handle))
    {
      float_t refValue = CTRL_getRefValue_spd_pu(handle);
      float_t fbackValue = CTRL_getFbackValue_spd_pu(handle);
      float_t outMax = CTRL_getSpeed_outMax_pu(handle);
      float_t outMin = -outMax;

      // reset the speed count
      CTRL_resetCounter_speed(handle);

      PID_setMinMax(obj->pidHandle_spd,outMin,outMax);

      PID_run_spd(obj->pidHandle_spd,refValue,fbackValue,CTRL_getIq_ref_pu_addr(handle));
    }
  }
  else
  {
    // zero the speed command
    CTRL_setIq_ref_pu(handle,(0.0f));

    // reset the integrator
    PID_setUi(obj->pidHandle_spd,(0.0f));
  }


 // when appropriate, run the PID Id and Iq controllers
  if(CTRL_doCurrentCtrl(handle))
  {
    float_t Kp_Id = CTRL_getKp(handle,CTRL_Type_PID_Id);
    float_t Kp_Iq = CTRL_getKp(handle,CTRL_Type_PID_Iq);
    float_t refValue;
    float_t fbackValue;
    float_t outMin,outMax;

    float_t maxVsMag = CTRL_getMaxVsMag_pu(handle);

    // reset the current count
    CTRL_resetCounter_current(handle);

    // ***********************************
    // configure and run the Id controller

    // compute the Kp gain
    // Scale Kp instead of output to prevent saturation issues
    if(CTRL_getFlag_enableDcBusComp(handle))
    {
     Kp_Id = (Kp_Id*EST_getOneOverDcBus_pu(obj->estHandle));
    }

    PID_setKp(obj->pidHandle_Id,Kp_Id);

    // compute the reference value
    refValue = CTRL_getRefValue_Id_pu(handle);

    // update the Id reference value
    EST_updateId_ref_pu(obj->estHandle,&refValue);

    // get the feedback value
    fbackValue = CTRL_getId_in_pu(handle);

    // compute the Id output limits
    CTRL_computeOutputLimits_Id(handle,maxVsMag,&outMin,&outMax);

    // set the minimum and maximum values
    PID_setMinMax(obj->pidHandle_Id,outMin,outMax);

    // run the Id PID controller
    PID_run(obj->pidHandle_Id,refValue,fbackValue,CTRL_getVd_out_addr(handle));

    // set the Id reference value in the estimator
    EST_setId_ref_pu(obj->estHandle,refValue);

    // ***********************************
    // configure and run the Iq controller

    // compute the Kp gain
    // Scale Kp instead of output to prevent saturation issues
    if(CTRL_getFlag_enableDcBusComp(handle))
    {
     Kp_Iq = (Kp_Iq*EST_getOneOverDcBus_pu(obj->estHandle));
    }

    PID_setKp(obj->pidHandle_Iq,Kp_Iq);

    // get the Iq reference value
    refValue = CTRL_getRefValue_Iq_pu(handle);  //CTRL_getIq_ref_pu(handle);

    // get the feedback value
    fbackValue = CTRL_getIq_in_pu(handle);

    // compute the Id output limits
    CTRL_computeOutputLimits_Id(handle,maxVsMag,&outMin,&outMax);

    // set the minimum and maximum values
    PID_setMinMax(obj->pidHandle_Iq,outMin,outMax);

    // run the Iq PID controller
    PID_run(obj->pidHandle_Iq,refValue,fbackValue,CTRL_getVq_out_addr(handle));

    // set the Iq reference value in the estimator
    EST_setIq_ref_pu(obj->estHandle,refValue);

    // add voltage offsets
    CTRL_addVdq_offset(handle);
  }


 return;

} // end of CTRL_runPiOnly() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _CTRL_H_ definition




