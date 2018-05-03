#ifndef _HAL_H_
#define _HAL_H_
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

//! \file   solutions/instaspin_foc/boards/drv8312kit_revD/f28x/f2806xF/src/hal.h
//! \brief  Contains public interface to various functions related
//!         to the HAL object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes


// modules
#include "modules/hal/hal_obj.h"
#include "modules/user/user.h"


//!
//!
//! \defgroup HAL HAL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines
#define AD_CONV_RATIO 0.000244140625f//1/4096

//! \brief Enumeration for the sensor types
//!
typedef enum
{
  HAL_SensorType_Current=0,      //!< Enumeration for current sensor
  HAL_SensorType_Voltage         //!< Enumeration for voltage sensor
} HAL_SensorType_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief      Gets the ADC bias value
//! \details    The ADC bias contains the feedback circuit's offset and bias.
//!             Bias is the mathematical offset used when a bi-polar signal
//!             is read into a uni-polar ADC.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The ADC bias value
static inline float_t HAL_getBias(HAL_Handle handle,
                              const HAL_SensorType_e sensorType,
                              uint_least8_t sensorNumber)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;
//  _iq bias = _IQ(0.0);
  float_t bias = (0.0);//! \Mod By Dl.K
	
  if(sensorType == HAL_SensorType_Current)
    {
      bias = obj->adcBias.I.value[sensorNumber];
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      bias = obj->adcBias.V.value[sensorNumber];
    }

  return(bias);
} // end of HAL_getBias() function


//! \brief      Gets the current scale factor
//! \details    The current scale factor is defined as
//!             USER_ADC_FULL_SCALE_CURRENT_A/USER_IQ_FULL_SCALE_CURRENT_A.
//!             This scale factor is not used when converting between PU amps
//!             and real amps.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return     The current scale factor
static inline float_t HAL_getCurrentScaleFactor(HAL_Handle handle)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  return(obj->current_sf);
} // end of HAL_getCurrentScaleFactor() function


//! \brief     Gets the number of current sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of current sensors
static inline uint_least8_t HAL_getNumCurrentSensors(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  return(obj->numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function


//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint_least8_t HAL_getNumVoltageSensors(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  return(obj->numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function


//! \brief      Gets the value used to set the low pass filter pole for offset estimation
//! \details    An IIR single pole low pass filter is used to find the feedback circuit's
//!             offsets.  This function returns the value of that pole.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The value used to set the low pass filter pole, pu
static inline float_t HAL_getOffsetBeta_lp_pu(HAL_Handle handle,
                                          const HAL_SensorType_e sensorType,
                                          const uint_least8_t sensorNumber)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;

//  _iq beta_lp_pu = _IQ(0.0);
  float_t beta_lp_pu = (0.0);//! \Mod By Dl.K
  
  if(sensorType == HAL_SensorType_Current)
    {
      beta_lp_pu = OFFSET_getBeta(obj->offsetHandle_I[sensorNumber]);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      beta_lp_pu = OFFSET_getBeta(obj->offsetHandle_V[sensorNumber]);
    }

  return(beta_lp_pu);
} // end of HAL_getOffsetBeta_lp_pu() function


//! \brief      Gets the offset value
//! \details    The offsets that are calculated during the feedback circuits calibrations
//!             are returned from the IIR filter object.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The offset value
static inline float_t HAL_getOffsetValue(HAL_Handle handle,
                                     const HAL_SensorType_e sensorType,
                                     const uint_least8_t sensorNumber)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;

//  _iq offset = _IQ(0.0);
  float_t offset = (0.0);//! \Mod By Dl.K
  
  if(sensorType == HAL_SensorType_Current)
    {
      offset = OFFSET_getOffset(obj->offsetHandle_I[sensorNumber]);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      offset = OFFSET_getOffset(obj->offsetHandle_V[sensorNumber]);
    }

  return(offset);
} // end of HAL_getOffsetValue() function


//! \brief      Gets the voltage scale factor
//! \details    The voltage scale factor is defined as
//!             USER_ADC_FULL_SCALE_VOLTAGE_V/USER_IQ_FULL_SCALE_VOLTAGE_V.
//!             This scale factor is not used when converting between PU volts
//!             and real volts.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return     The voltage scale factor
static inline float_t HAL_getVoltageScaleFactor(HAL_Handle handle)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  return(obj->voltage_sf);
} // end of HAL_getVoltageScaleFactor() function


//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);


//! \brief      Reads the ADC data
//! \details    Reads in the ADC result registers, adjusts for offsets, and
//!             scales the values according to the settings in user.h.  The
//!             structure gAdcData holds three phase voltages, three line
//!             currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData  A pointer to the ADC data buffer
static inline void HAL_readAdcData(HAL_Handle handle,HAL_AdcData_t *pAdcData)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  float_t value;
  float_t current_sf = HAL_getCurrentScaleFactor(handle);
  float_t voltage_sf = HAL_getVoltageScaleFactor(handle);
	//! \Mod By Dl.K

  // convert current A
  // sample the first sample twice due to errata sprz342f, ignore the first sample
	value = pAdcData->I.value[0] * AD_CONV_RATIO;
  value = (value*current_sf) - obj->adcBias.I.value[0];      // divide by 2^numAdcBits = 2^12
  pAdcData->I.value[0] = value;//! \Mod By Dl.K Del'-'

  // convert current B
	value = pAdcData->I.value[1] * AD_CONV_RATIO;
  value = (value*current_sf) - obj->adcBias.I.value[1];      // divide by 2^numAdcBits = 2^12
  pAdcData->I.value[1] = value;//! \Mod By Dl.K Del'-'

  // convert current C
	value = pAdcData->I.value[2] * AD_CONV_RATIO;
  value = (value*current_sf) - obj->adcBias.I.value[2];      // divide by 2^numAdcBits = 2^12
  pAdcData->I.value[2] = value;//! \Mod By Dl.K Del'-'

  // convert voltage A
	value = pAdcData->V.value[0] * AD_CONV_RATIO;
  value = (value*voltage_sf) - obj->adcBias.V.value[0];      // divide by 2^numAdcBits = 2^12
  pAdcData->V.value[0] = value;

  // convert voltage B
	value = pAdcData->V.value[1] * AD_CONV_RATIO;
  value = (value*voltage_sf) - obj->adcBias.V.value[1];      // divide by 2^numAdcBits = 2^12
  pAdcData->V.value[1] = value;

  // convert voltage C
	value = pAdcData->I.value[2] * AD_CONV_RATIO;
  value = (value*voltage_sf) - obj->adcBias.V.value[2];      // divide by 2^numAdcBits = 2^12
  pAdcData->V.value[2] = value;

  // read the dcBus voltage value //! \Mod By Dl.K
	value = pAdcData->VdcBus * AD_CONV_RATIO;
  value = (value*voltage_sf) - (V_DC_offset)*USER_VOLTAGE_SF;
  pAdcData->VdcBus = value;

  // read the dcBus current value //! \Mod By Dl.K
	value = pAdcData->IdcBus * AD_CONV_RATIO;
  value = (value*current_sf) - (I_DC_offset)*USER_CURRENT_SF;
  pAdcData->IdcBus = value;

  return;
} // end of HAL_readAdcData() function


//! \brief     Sets the ADC bias value
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] bias          The ADC bias value
static inline void HAL_setBias(HAL_Handle handle,
                               const HAL_SensorType_e sensorType,
                               uint_least8_t sensorNumber,
                               const float_t bias)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  if(sensorType == HAL_SensorType_Current)
    {
      obj->adcBias.I.value[sensorNumber] = bias;
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      obj->adcBias.V.value[sensorNumber] = bias;
    }

  return;
} // end of HAL_setBias() function


//! \brief     Sets the current scale factor in the hardware abstraction layer
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] current_sf  The current scale factor
static inline void HAL_setCurrentScaleFactor(HAL_Handle handle,const float_t current_sf)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->current_sf = current_sf;

  return;
} // end of HAL_setCurrentScaleFactor() function


//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void HAL_setNumCurrentSensors(HAL_Handle handle,const uint_least8_t numCurrentSensors)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->numCurrentSensors = numCurrentSensors;

  return;
} // end of HAL_setNumCurrentSensors() function


//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void HAL_setNumVoltageSensors(HAL_Handle handle,const uint_least8_t numVoltageSensors)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->numVoltageSensors = numVoltageSensors;

  return;
} // end of HAL_setNumVoltageSensors() function


//! \brief     Sets the value used to set the low pass filter pole for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] beta_lp_pu    The value used to set the low pass filter pole, pu
static inline void HAL_setOffsetBeta_lp_pu(HAL_Handle handle,
                                           const HAL_SensorType_e sensorType,
                                           const uint_least8_t sensorNumber,
                                           const float_t beta_lp_pu)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  if(sensorType == HAL_SensorType_Current)
    {
      OFFSET_setBeta(obj->offsetHandle_I[sensorNumber],beta_lp_pu);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      OFFSET_setBeta(obj->offsetHandle_V[sensorNumber],beta_lp_pu);
    }

  return;
} // end of HAL_setOffsetBeta_lp_pu() function


//! \brief     Sets the offset initial condition value for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] initCond      The initial condition value
static inline void HAL_setOffsetInitCond(HAL_Handle handle,
                                         const HAL_SensorType_e sensorType,
                                         const uint_least8_t sensorNumber,
                                         const float_t initCond)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  if(sensorType == HAL_SensorType_Current)
    {
      OFFSET_setInitCond(obj->offsetHandle_I[sensorNumber],initCond);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      OFFSET_setInitCond(obj->offsetHandle_V[sensorNumber],initCond);
    }

  return;
} // end of HAL_setOffsetInitCond() function


//! \brief     Sets the initial offset value for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] value         The initial offset value
static inline void HAL_setOffsetValue(HAL_Handle handle,
                                      const HAL_SensorType_e sensorType,
                                      const uint_least8_t sensorNumber,
                                      const float_t value)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  if(sensorType == HAL_SensorType_Current)
    {
      OFFSET_setOffset(obj->offsetHandle_I[sensorNumber],value);
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      OFFSET_setOffset(obj->offsetHandle_V[sensorNumber],value);
    }

  return;
} // end of HAL_setOffsetValue() function


//! \brief     Sets the voltage scale factor in the hardware abstraction layer
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] voltage_sf  The voltage scale factor
static inline void HAL_setVoltageScaleFactor(HAL_Handle handle,const float_t voltage_sf)//! \Mod By Dl.K
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  
  obj->voltage_sf = voltage_sf;

  return;
} // end of HAL_setVoltageScaleFactor() function


//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
//! \param[in]  pUserParams  The pointer to the user parameters
extern void HAL_setParams(HAL_Handle handle,const USER_Params *pUserParams);


//! \brief      Updates the ADC bias values
//! \details    This function is called before the motor is started.  It sets the voltage
//!             and current measurement offsets.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_updateAdcBias(HAL_Handle handle)
{
  uint_least8_t cnt;
  HAL_Obj *obj = (HAL_Obj *)handle;
  float_t bias;//! \Mod By Dl.K


  // update the current bias
  for(cnt=0;cnt<HAL_getNumCurrentSensors(handle);cnt++)
    {
      bias = HAL_getBias(handle,HAL_SensorType_Current,cnt);
      
      bias += OFFSET_getOffset(obj->offsetHandle_I[cnt]);//! \Mod By Dl.K Very Important: bias -= to bias +=

      HAL_setBias(handle,HAL_SensorType_Current,cnt,bias);
    }


  // update the voltage bias
  for(cnt=0;cnt<HAL_getNumVoltageSensors(handle);cnt++)
    {
      bias = HAL_getBias(handle,HAL_SensorType_Voltage,cnt);

      bias += OFFSET_getOffset(obj->offsetHandle_V[cnt]);

      HAL_setBias(handle,HAL_SensorType_Voltage,cnt,bias);
    }

  return;
} // end of HAL_updateAdcBias() function



//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPwmData  The pointer to the PWM data
/*
*pwmData_neg = _IQmpy(pPwmData->Tabc.value[cnt],_IQ(-1.0));  对占空比进行反相，
*pwmData_sat = _IQsat(pwmData_neg,_IQ(1.0),_IQ(-1.0));  饱和，设置上下限 为-1~ 1之间。
*pwmData_sat_dc = _IQmpy(pwmData_sat + _IQ(1.0), _IQ(0.5));  将占空比调整为 0~ 1之间
*value = _IQmpy(pwmData_sat_dc, period);  占空比乘以周期值，就是比较器的值。
*value_sat = (uint16_t)_IQsat(value, period, _IQ(0.0)); 饱和，设置比较器的上下限
*/
/*而且老板的库中是这个：pwmData_sat_dc = _IQmpy(pwmData_sat + _IQ(1.0), _IQ(0.5));
新版的库中是这个：pwmData_sat_dc = pwmData_sat + _IQ(0.5);

在同等电压下

用pwmData_sat_dc = pwmData_sat + _IQ(0.5); 转速有提高，但是油门就没有最大，转速就已经最高了

用pwmData_sat_dc = _IQmpy(pwmData_sat + _IQ(1.0), _IQ(0.5)); 转速比上者低几百转，但是油门可以提到最高

这是什么原因呢？？

ERIC：

新版本在这里做了调整了，所以你看到user.h里面的VS最大都是比以前除以2.

新版的库中是这个：pwmData_sat_dc = pwmData_sat + _IQ(0.5);   这里还是把pwmData_sat_dc 调整到 0~ 1. 

pwmData_sat  是-0.5~0.5.

用pwmData_sat_dc = pwmData_sat + _IQ(0.5); 转速有提高，但是油门就没有最大，转速就已经最高了

用pwmData_sat_dc = _IQmpy(pwmData_sat + _IQ(1.0), _IQ(0.5)); 转速比上者低几百转，但是油门可以提到最高

ERIC: 不理解你这个内容，油门表示什么？扭矩？ 你用新的版本就好了。
*/
static inline void HAL_writePwmData(HAL_Handle handle,HAL_PwmData_t *pPwmData)
{
  uint_least8_t cnt;
  float_t pwmData_neg;
  float_t pwmData_sat;
  float_t pwmData_sat_dc;

  for(cnt=0;cnt<3;cnt++)
    {
      pwmData_neg = (pPwmData->Tabc.value[cnt]*(1.0f));//! \Attention : Dl.K
      pwmData_sat = MATH_sat(pwmData_neg,(1.0f),(-1.0f));
      pwmData_sat_dc = ( (pwmData_sat + (1.0f)) * (0.5f) );
			pPwmData->Tabc.value[cnt] = pwmData_sat_dc;
    }

  return;
} // end of HAL_writePwmData() function

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _HAL_H_ definition


