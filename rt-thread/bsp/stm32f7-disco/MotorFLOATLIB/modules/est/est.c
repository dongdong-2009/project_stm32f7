/*=====================================================================================
 File name:        Template.c  (IQ version)
                    
 Originator:	DL.K

 Description:  The ...
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/est/est.h"

//! \brief     Determines if current control should be performed during motor identification
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether (true) or not (false) to perform current control
extern bool EST_doCurrentCtrl(EST_Handle handle)
{
	return 1;
}


//! \brief     Determines if speed control should be performed during motor identification
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether (true) or not (false) to perform speed control
extern bool EST_doSpeedCtrl(EST_Handle handle)
{
	return 1;
}



//! \brief     Gets the angle value from the estimator in per unit (pu), IQ24.
//! \details   This function returns a per units value of the rotor flux angle. This value wraps around 
//!            at 1.0, so the return value is between 0x00000000 or _IQ(0.0) to 0x00FFFFFF or _IQ(1.0). 
//!            An example of using this angle is shown:
//! \code
//! float_t Rotor_Flux_Angle_pu = EST_getAngle_pu(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle value, pu, in IQ24.
extern float_t EST_getAngle_pu(EST_Handle handle)
{
	return 0.0;
}


//! \brief     Gets the DC bus value from the estimator in per unit (pu), IQ24.
//! \details   This value is originally passed as a parameter when calling function EST_run(). 
//!            A similar function can be simply reading what has been read and scaled by the ADC converter
//!            on pAdcData->dcBus. This value is used by the libraries internally to calculate one over 
//!            dcbus, which is a value used to compensate the proportional gains of the current 
//!            controllers. The following example shows how to use this function to calculate a DC bus value
//!            in kilo volts:
//! \code
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//!
//! float_t Vbus_pu = EST_getDcBus_pu(handle);
//! float_t Vbus_pu_to_kV_sf = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V / 1000.0);
//! float_t Vbus_kV = _IQmpy(Vbus_pu,Vbus_pu_to_kV_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The DC bus value, pu
extern float_t EST_getDcBus_pu(EST_Handle handle)
{
	return 0.0;
}



//! \brief     Gets the force angle delta value from the estimator in per unit (pu), IQ24.
//! \details   This function returns a valid value only after initializing the controller object
//!            by calling CTRL_setParams() function. The force angle delta represents the increments
//!            to be added to or subtracted from the forced angle. The higher this value is, the higher
//!            frequency will be generated when the angle is forced (estimated angle is bypassed when 
//!            in forced angle mode). By default the forced angle frequency is set in user.h. 
//!            The following example shows how to convert delta in per units to kilo Hertz (kHz).
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_EST_TICK  (1)
//! #define USER_PWM_FREQ_kHz         (15.0)
//! #define USER_ISR_FREQ_Hz          (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz         (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_EST_FREQ_Hz          (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_EST_TICK)
//!
//! float_t delta_pu_to_kHz_sf = _IQ((float_t)USER_EST_FREQ_Hz/1000.0);
//! float_t Force_Angle_Delta_pu = EST_getForceAngleDelta_pu(handle);
//! float_t Force_Angle_Freq_kHz = _IQmpy(Force_Angle_Delta_pu, delta_pu_to_kHz_sf);
//! \endcode
//! \note      Note that kHz is prefered to avoid overflow of IQ24 variables.
//! \param[in] handle  The estimator (EST) handle
//! \return    The force angle delta, pu. Minimum value of _IQ(0.0) and maximum of _IQ(1.0).
extern float_t EST_getForceAngleDelta_pu(EST_Handle handle)
{
	return 0.0;
}


//! \brief     Gets the krpm to pu scale factor in per unit (pu), IQ24.
//! \details   This function is needed when a user needs to scale a value of the motor speed from 
//!            kpm (kilo revolutions per minute) to a per units value. This scale factor is calculated
//!            and used as shown below:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS       (2)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz      (500.0)
//!
//! float_t scale_factor = _IQ(USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / (60.0 * USER_IQ_FULL_SCALE_FREQ_Hz));
//!
//! float_t Speed_krpm = EST_getSpeed_krpm(handle);
//! float_t Speed_krpm_to_pu_sf = EST_get_krpm_to_pu_sf(handle);
//! float_t Speed_pu = _IQmpy(Speed_krpm,Speed_krpm_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The krpm to pu scale factor. This value is in IQ24.
extern float_t EST_get_krpm_to_pu_sf(EST_Handle handle)
{
	return 0.0;
}


//! \brief     Determines if there is an estimator error
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if there is an estimator error (true) or not (false)
extern bool EST_isError(EST_Handle handle)
{
	return 1;
}


//! \brief     Determines if the estimator is idle
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is idle (true) or not (false)
extern bool EST_isIdle(EST_Handle handle)
{
	return 1;
}


//! \brief     Determines if the estimator is waiting for the rotor to be locked
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is waiting for the rotor to be locked (true) or not (false)
extern bool EST_isLockRotor(EST_Handle handle)
{
	return 1;
}


//! \brief     Determines if the motor has been identified
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the motor is identified (true) or not (false)
extern bool EST_isMotorIdentified(EST_Handle handle)
{
	return 1;
}


//! \brief     Determines if the estimator is ready for online control
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is ready for online control (true) or not (false)
extern bool EST_isOnLine(EST_Handle handle)
{
	return 1;
}

//! \brief     Runs the estimator
//! \param[in] handle         The estimator (EST) handle
//! \param[in] pIab_pu        The pointer to the phase currents in the alpha/beta coordinate system, pu IQ24
//! \param[in] pVab_pu        The pointer to the phase voltages in the alpha/beta coordinate system, pu IQ24
//! \param[in] dcBus_pu       The DC bus voltage, pu IQ24
//! \param[in] speed_ref_pu   The speed reference value to the controller, pu IQ24
extern void EST_run(EST_Handle handle,
	const MATH_vec2 *pIab_pu,
	const MATH_vec2 *pVab_pu,
	const float_t dcBus_pu,
	const float_t speed_ref_pu)
{
	return ;
}


//! \brief     Sets the estimator to idle
//! \param[in] handle  The estimator (EST) handle
extern void EST_setIdle(EST_Handle handle)
{
	return ;
}


//! \brief     Sets the estimator and all of the subordinate estimators to idle
//! \param[in] handle    The estimator (EST) handle
extern void EST_setIdle_all(EST_Handle handle)
{
	return ;
}


//! \brief     Sets the direct current (Id) reference value in the estimator in per unit (pu), IQ24.
//! \param[in] handle     The estimator (EST) handle
//! \param[in] Id_ref_pu  The Id reference value, pu
extern void EST_setId_ref_pu(EST_Handle handle, const float_t Id_ref_pu)
{
	return ;
}


//! \brief     Sets the stator resistance value used in the estimator in per unit (pu), IQ30.
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Rs_pu    The stator resistance value, pu
extern void EST_setRs_pu(EST_Handle handle, const float_t Rs_pu)
{
	return ;
}


//! \brief     Updates the Id reference value used for online stator resistance estimation in per unit (pu), IQ24.
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pId_ref_pu  The pointer to the Id reference value, pu
extern void EST_updateId_ref_pu(EST_Handle handle, float_t *pId_ref_pu)
{
	return ;
}


//! \brief      Updates the estimator state
//! \param[in]  handle        The estimator (EST) handle
//! \param[in]  Id_target_pu  The target Id current during each estimator state, pu IQ24
//! \return     A boolean value denoting if the state has changed (true) or not (false)
extern bool EST_updateState(EST_Handle handle, const float_t Id_target_pu)
{
	return 1;
}


//! \brief     Determines if a zero Iq current reference should be used in the controller
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if a zero Iq current reference should be used (true) or not (false)
extern bool EST_useZeroIq_ref(EST_Handle handle)
{
	return 1;
}


//! \brief     Gets the flux value in per unit (pu), IQ24.
//! \details   The estimator continuously calculates the flux linkage between the rotor and stator, which is the
//!            portion of the flux that produces torque. This function returns the flux linkage, ignoring the
//!            number of turns, between the rotor and stator coils, in per units.
//!            This functions returns a precise value only after the motor has been identified, which can be 
//!            checked by the following code example:
//! \code
//! if(EST_isMotorIdentified(handle))
//!   {
//!     // once the motor has been identified, get the flux
//!     float_t Flux_pu = EST_getFlux_pu(handle);
//!   }
//! \endcode
//! \details   For some applications it is important to get this value in per units, since it is much faster to
//!            process especially when the architecture of the microcontroller does not have a floating point
//!            processing unit. In order to translate this per units value into a scaled value in _iq, it is
//!            important to consider a scale factor to convert this flux in per units to the required units.
//!            The following example shows how to scale a per units value to Wb and V/Hz in IQ for faster
//!            processing:
//! \code
//! float_t FullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)USER_EST_FREQ_Hz);
//! float_t maxFlux = (USER_MOTOR_RATED_FLUX*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
//! float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));
//! float_t gFlux_pu_to_Wb_sf = _IQ(FullScaleFlux/(2.0*MATH_PI)*pow(2.0,lShift));
//! float_t gFlux_pu_to_VpHz_sf = _IQ(FullScaleFlux*pow(2.0,lShift));
//! // The value of gFlux_pu_to_Wb_sf and gFlux_pu_to_VpHz_sf can be calculated once at the beginning of the 
//! // code and stored as global variables
//!
//! float_t Flux_Wb;
//! float_t Flux_VpHz;
//! float_t Flux_pu = EST_getFlux_pu(handle);
//!
//! Flux_Wb = _IQmpy(Flux_pu, gFlux_pu_to_Wb_sf);
//! Flux_VpHz = _IQmpy(Flux_pu, gFlux_pu_to_VpHz_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The flux value, pu
extern float_t EST_getFlux_pu(EST_Handle handle)
{
	return 0.0;
}


//! \brief     Gets the direct stator inductance value in Henries (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct stator inductance value
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getLs_d_H(EST_Handle handle)
{
	return 0;
}
#else
extern float_t EST_getLs_d_H(EST_Handle handle)
{
	return 0.0;
}
#endif


//! \brief     Gets the direct stator inductance value in per unit (pu), IQ30.
//! \details   The per units value of the direct stator inductance can be used as an alternative way
//!            of calculating the direct stator inductance of a permanent magnet motor using fixed point math.
//!            An example showing how this is done is shown here:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define MATH_PI (3.1415926535897932384626433832795)
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//! #define USER_VOLTAGE_FILTER_POLE_Hz  (335.648)
//! #define USER_VOLTAGE_FILTER_POLE_rps (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)
//!
//! uint_least8_t Ls_qFmt = EST_getLs_qFmt(handle);
//! float_t fullScaleInductance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps));
//! float_t Ls_d_pu = _IQ30toIQ(EST_getLs_d_pu(handle));
//! float_t pu_to_h_sf = VarShift(fullScaleInductance, 30 - Ls_qFmt);
//! float_t Ls_d_H = _IQmpy(Ls_d_pu, pu_to_h_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct stator inductance value, pu
extern float_t EST_getLs_d_pu(EST_Handle handle)
{
	return 0.0;
}


//! \brief     Gets the stator inductance value in the quadrature coordinate direction in Henries (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator inductance value
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getLs_q_H(EST_Handle handle)
{
	return 0;
}
#else
extern float_t EST_getLs_q_H(EST_Handle handle)
{
	return 0.0;
}
#endif


//! \brief     Gets the stator inductance value in the quadrature coordinate direction in per unit (pu), IQ30.
//! \details   The per units value of the quadrature stator inductance can be used as an alternative way
//!            of calculating the quadrature stator inductance of a permanent magnet motor using fixed point math.
//!            An example showing how this is done is shown here:
//! \code
//! #define VarShift(var,nshift) (((nshift) < 0) ? ((var)>>(-(nshift))) : ((var)<<(nshift)))
//!
//! #define MATH_PI (3.1415926535897932384626433832795)
//! #define USER_IQ_FULL_SCALE_VOLTAGE_V (300.0)
//! #define USER_IQ_FULL_SCALE_CURRENT_A (10.0)
//! #define USER_VOLTAGE_FILTER_POLE_Hz  (335.648)
//! #define USER_VOLTAGE_FILTER_POLE_rps (2.0 * MATH_PI * USER_VOLTAGE_FILTER_POLE_Hz)
//!
//! uint_least8_t Ls_qFmt = EST_getLs_qFmt(handle);
//! float_t fullScaleInductance = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps));
//! float_t Ls_q_pu = _IQ30toIQ(EST_getLs_q_pu(handle));
//! float_t pu_to_h_sf = VarShift(fullScaleInductance, 30 - Ls_qFmt);
//! float_t Ls_q_H = _IQmpy(Ls_q_pu, pu_to_h_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator inductance value, pu
extern float_t EST_getLs_q_pu(EST_Handle handle)
{
	return 0.0;
}


//! \brief     Gets the stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator resistance value, Ohm
#ifdef __TMS320C28XX_FPU32__
extern int32_t EST_getRs_Ohm(EST_Handle handle)
{
	return 0;
}
#else
extern float_t EST_getRs_Ohm(EST_Handle handle)
{
	return 0.0;
}
#endif

//! \brief     Gets the mechanical frequency of the motor in per unit (pu), IQ24.
//! \details   Similar to EST_getFe_pu() function, this function returns the mechanical frequency
//!            of the motor in per units. In order to convert the mechanical frequency from 
//!            per units to kHz (to avoid saturation of IQ24), the user needs to multiply the 
//!            returned value by the following scale factor:
//! \code
//! #define USER_IQ_FULL_SCALE_FREQ_Hz  (500.0)
//!
//! float_t pu_to_khz_sf = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz/1000.0);
//! float_t khz_to_krpm_sf = _IQ(60.0/USER_MOTOR_NUM_POLE_PAIRS);
//! float_t Mechanical_Freq_kHz = _IQmpy(EST_getFm_pu(handle),pu_to_khz_sf);
//! float_t Speed_kRPM = _IQmpy(Mechanical_Freq_kHz,khz_to_krpm_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical frequency, pu
extern float_t EST_getFm_pu(EST_Handle handle)
{
	return 0.0;
}


//! \brief     Gets the inverse of the DC bus voltage in per unit (pu), IQ24.
//! \param[in] handle  The estimator (EST) handle
//! \return    The inverse of the DC bus voltage, pu
extern float_t EST_getOneOverDcBus_pu(EST_Handle handle)
{
	return 0.0;
}

//! \brief     Sets the quadrature current (Iq) reference value in the estimator in per unit (pu), IQ24.
//! \param[in] handle     The estimator (EST) handle
//! \param[in] Iq_ref_pu  The Iq reference value, pu
extern void EST_setIq_ref_pu(EST_Handle handle,const float_t Iq_ref_pu)
{
	return;
}
