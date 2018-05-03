/*=====================================================================================
 File name:        DATA_SELECT.C  (IQ version)
                    
 Originator:	DL.K

 Description:  The DATA SELECT
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/data_select/data_select.h"

DATA_e gData_e[DATA_TXD_SIZE/2-1]={STA_DATA,IA_DATA,IA_DATA,IA_DATA,IA_DATA};

/*
*  |0  	|1   		 |2   	 |3    	|4    	|5     	|6     	|7       	|8    	|9      	|10          |11		|12		
*    0x11		0xEB   timel     timeh	DATA1l	DATA1h	DATA2l	DATA2h	DATA3l	DATA3h	DATA4l	DATA4h		CHECK
*/


void DATA_RTXD_Run(CTRL_Handle handle, HAL_Handle halHandle, const HAL_AdcData_t *pAdcData, const HAL_QepData_t *pQepData)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)(handle->data_rtxd_handle);
	uint8_t i;
	int16_t value;
	obj->data_txd[0] = 0x11;
	obj->data_txd[1] = 0xEB;

	obj->data_txd[2] = (uint8_t)( obj->time & (uint16_t)0x00FF );
	obj->data_txd[3] = (uint8_t)( ((obj->time)>>8) & (uint16_t)0x00FF );
	obj->time++;

	obj->data_txd[DATA_TXD_SIZE-1] = 0;
	obj->data_txd[DATA_TXD_SIZE-1] += obj->data_txd[0] + obj->data_txd[1] + obj->data_txd[2] + obj->data_txd[3];

	for(i=2;i<DATA_TXD_SIZE/2;i++)
	{
		value = DATA_Select(handle, halHandle, pAdcData, pQepData, gData_e[i-1]);
		obj->data_txd[i+i] = (uint8_t)( value & (uint16_t)0x00FF );
		obj->data_txd[i+i+1] = (uint8_t)( (value>>8) & (uint16_t)0x00FF );
		obj->data_txd[DATA_TXD_SIZE-1] += obj->data_txd[i+i] + obj->data_txd[i+i+1];
	}
	
//	for(i=0;i<DATA_TXD_SIZE;i++)
//	{
//		obj->data_txd[i]=i;
//	}
	
	Reset_DATA_TXD(obj);

}




/*
*  |0  	|1   		 |2   	 |3    	|4    	|5     	|6     	|7       	|8    	|9      	|10          |11		|12		
*    0x01		0x0E   DATAE      DATAE	DATAE	DATAE	STATE	CMD		VALUE	VALUE					CHECK
*/
void DATA_RTXD_Decode(CTRL_Handle handle, HAL_Handle halHandle)
{
	DATA_RTXD_Obj *obj = (DATA_RTXD_Obj *)(handle->data_rtxd_handle);
	uint8_t i,value=0;
	float_t value1=0.0f;

	if(!(DATA_RXD_Status(handle->data_rtxd_handle)))
	{
		for(i=0;i<DATA_RXD_SIZE-1;i++)
		{
//			if(obj->data_rxd[i]>=32)
//			{
//				obj->data_rxd[i] -= 32;//0x20:32
				value += obj->data_rxd[i];
//			}
		}
//		obj->data_rxd[DATA_RXD_SIZE-1] -= 32;//0x20:32
		if( obj->data_rxd[0]==0x01 && obj->data_rxd[1]==0x0E && value== obj->data_rxd[DATA_RXD_SIZE-1] )
		{
			for(i=2;i<DATA_RXD_SIZE-1;i++)
			{
				if( i<DATA_RXD_SIZE/2 && obj->data_rxd[i]>STA_DATA && obj->data_rxd[i]<END_DATA )
				{
					gData_e[i-1] = (DATA_e)obj->data_rxd[i];
				}
				else if( i<DATA_RXD_SIZE/2+1 && obj->data_rxd[i]>CTRL_State_Error && obj->data_rxd[i]<CTRL_numStates)
				{
					handle->prevState = handle->state;
					handle->state = (CTRL_State_e)obj->data_rxd[i];
				}
				else if(  i<DATA_RXD_SIZE/2+2 && obj->data_rxd[i]>USER_Motor_Cmd_Err && obj->data_rxd[i]<USER_Motor_Cmd_Num )
				{
					value1 = obj->data_rxd[i+1] + obj->data_rxd[i+2]*256.0f;
					if( value1 > 32767.0f )
					{
						value1 -= 65536.0f;
					}
					value1 *= 0.00004f;
					switch(obj->data_rxd[i])
					{
						case USER_Motor_Speed_Ref:
							handle->spd_ref = value1;
							break;
						case USER_Motor_Speed_P:
							handle->Kp_spd = value1;
							break;
						case USER_Motor_Speed_I:
							handle->Ki_spd = value1;
							break;
						case USER_Motor_Iq_Ref:
							handle->Idq_ref.value[1] = value1;
							break;
						case USER_Motor_Iq_P:
							handle->Kp_Id = value1;
							handle->Kp_Iq = value1;
							break;
						case USER_Motor_Iq_I:
							handle->Ki_Id = value1;
							handle->Ki_Iq = value1;
							break;
						default:
							break;
					}
				}
			}
		}
		Reset_DATA_RXD(obj);
	}

	return;
}


