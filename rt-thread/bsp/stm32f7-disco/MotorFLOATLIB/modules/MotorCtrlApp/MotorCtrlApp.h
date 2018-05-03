/*=====================================================================================
 File name:        MotorCtrlApp.h  (IQ version)

 Originator:	DL.K

 Description:  The ...
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/
#ifndef __MOTORCTRLAPP_H__
#define __MOTORCTRLAPP_H__

// **************************************************************************
// the includes
#include "modules/hal/hal_obj.h"

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

// **************************************************************************
// the typedefs

// **************************************************************************
// the functions

extern void Motor_Ctrl_Init(void);

extern uint8_t Motor_Ctrl_State(int8_t *cha_pt, int8_t *flag_pt);

extern void Motor_Ctrl_Run(HAL_AdcData_t *pAdcData, HAL_PwmData_t *pPwmData, HAL_QepData_t *pQepData);

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // __MOTORCTRLAPP_H__
