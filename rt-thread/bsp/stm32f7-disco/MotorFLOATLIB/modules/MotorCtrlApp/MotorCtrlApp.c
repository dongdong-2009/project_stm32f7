/*=====================================================================================
 File name:        Template.c  (IQ version)
                    
 Originator:	DL.K

 Description:  The ...
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/MotorCtrlApp/MotorCtrlApp.h"

#include "modules/types/types.h"
#include "modules/user/user.h"
#include "modules/hal/hal.h"
#include "modules/ctrl/ctrl.h"

#include <rtthread.h>
#include <rtdef.h>

extern rt_err_t myprintf(char *buf, uint8_t size);

// **************************************************************************
// the defines
	
USER_Params gUserParams;
	
HAL_Obj gHal;
HAL_Handle gHalHandle;
	
CTRL_Obj gCtrl;
CTRL_Handle gCtrlHandle;


void Motor_Ctrl_Init(void)
{
	gHalHandle = HAL_init(&gHal, sizeof(gHal));
	
	gCtrlHandle = CTRL_init(&gCtrl, sizeof(gCtrl));
	
	USER_setParams(&gUserParams);
	
	HAL_setParams(gHalHandle, &gUserParams);
	
	CTRL_setParams(gCtrlHandle, &gUserParams);
	
}

uint8_t Motor_Ctrl_State(int8_t *cha_pt, int8_t *flag_pt)
{
	//while(1)
	{
//		uint8_t i,value;
//		rt_device_t device = rt_device_find("uart1");
//		for(i=0;i<DATA_RXD_SIZE;i++)
//		{
//			value=0x00;
//			while(value==0x00)
//			{
//				rt_device_read (device, 0, &(value), sizeof(value));
//				if(value==0xF0)break;
//			}
//			if(value!=0x00 || value!=0xF0)
//			{
//				gCtrlHandle->data_rtxd_handle->data_rxd[i] = value;
//			}
//		}
		if(DATA_RXD_Status(gCtrlHandle->data_rtxd_handle)&&*flag_pt==1)
		{
			int8_t i;
			for(i=0;i<DATA_RXD_SIZE;i++)
			{
				DATA_RXD_Run(gCtrlHandle->data_rtxd_handle,*cha_pt);
				cha_pt++;
			}
			*flag_pt=0;
		}

		if(CTRL_isError(gCtrlHandle))
		  {
			// set the enable controller flag to false
			CTRL_setFlag_enableCtrl(gCtrlHandle,false);
			rt_kprintf("ctrl is error\r\n");
			// disable the PWM
				return false;
		  }
		else
		  {
			CTRL_State_e ctrlState = CTRL_getState(gCtrlHandle);
			CTRL_State_e ctrlprevState = gCtrlHandle->prevState;
			if(ctrlState == CTRL_State_Idle)
			{
				// disable the PWM
				if( ctrlprevState == CTRL_State_OpenLoop )
				{
					
					rt_kprintf("CTRL_State_OpenLoop\r\n");
					RMP_CNTL_InitPara( gCtrlHandle->rmpcntl_handle );
				}
				else if( ctrlprevState == CTRL_State_CurrentCloseLoop )
				{
					rt_kprintf("CTRL_State_CurrentCloseLoop\r\n");
					PID_setUi(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setUi(gCtrlHandle->pidHandle_Iq,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_Iq,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_Iq,(0.0f));
				}
				else if( ctrlprevState == CTRL_State_SpeedCloseLoop )
				{
					rt_kprintf("CTRL_State_SpeedCloseLoop\r\n");
					PID_setUi(gCtrlHandle->pidHandle_spd,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_spd,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_spd,(0.0f));
					PID_setUi(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setUi(gCtrlHandle->pidHandle_Iq,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_Iq,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_Iq,(0.0f));
				}
				return false;
			}
			else if(ctrlState == CTRL_State_OffLine)
			{
				// enable the PWM
				if( ctrlprevState == CTRL_State_OpenLoop )
				{
					RMP_CNTL_InitPara( gCtrlHandle->rmpcntl_handle );
				}
				else if( ctrlprevState == CTRL_State_CurrentCloseLoop )
				{
					PID_setUi(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setUi(gCtrlHandle->pidHandle_Iq,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_Iq,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_Iq,(0.0f));
				}
				else if( ctrlprevState == CTRL_State_SpeedCloseLoop )
				{
					PID_setUi(gCtrlHandle->pidHandle_spd,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_spd,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_spd,(0.0f));
					PID_setUi(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_Id,(0.0f));
					PID_setUi(gCtrlHandle->pidHandle_Iq,(0.0f));
					PID_setRefValue(gCtrlHandle->pidHandle_Iq,(0.0f));
					PID_setFbackValue(gCtrlHandle->pidHandle_Iq,(0.0f));
				}
			}
			else if(ctrlState == CTRL_State_OffLineOffsetComp)
			{
				// update the ADC bias values
				HAL_updateAdcBias(gHalHandle);

				CTRL_setState(gCtrlHandle, CTRL_State_OffLine);
			}
			else if( ctrlState == CTRL_State_OnLine || ctrlState == CTRL_State_CurrentCloseLoop || ctrlState == CTRL_State_SpeedCloseLoop )
			{
//				// set the speed reference
//				CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);
//
//				// set the speed acceleration
//				CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));
			}

		  }
			return true;
		}
}

void Motor_Ctrl_Run(HAL_AdcData_t *pAdcData, HAL_PwmData_t *pPwmData, HAL_QepData_t *pQepData)
{
	static uint_least16_t count_sci=50,numSciTick=50;

  // convert the ADC data
  HAL_readAdcData(gHalHandle,pAdcData);

  // run the controller
  
#if 1
  CTRL_run(gCtrlHandle,gHalHandle,pAdcData,pPwmData,pQepData);
#endif
	// test
//	pPwmData->Tabc.value[0] = -0.1f;
//	pPwmData->Tabc.value[1] = -0.0f;
//	pPwmData->Tabc.value[2] = 0.1f;
  // write the PWM compare values
  
  HAL_writePwmData(gHalHandle,pPwmData);

//  // setup the controller
//  CTRL_setup(gCtrlHandle);
#if 1
	if(count_sci>=numSciTick)
	{
		count_sci = 1;
		DATA_RTXD_Decode(gCtrlHandle,gHalHandle);
		DATA_RTXD_Run(gCtrlHandle,gHalHandle,pAdcData,pQepData);
		while( DATA_TXD_Status(gCtrlHandle->data_rtxd_handle))
		{
			//rt_kprintf("%c",DATA_TXD_Run(handle->data_rtxd_handle));
			myprintf((char*)Get_DATA_TXD_Ptr(gCtrlHandle->data_rtxd_handle), 13);
		}		
	}
	else count_sci++;//! \Mod By Dl.K
	#endif
}
