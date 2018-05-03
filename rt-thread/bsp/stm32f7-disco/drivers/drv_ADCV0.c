/*
 * File      : drv_ADC.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-08-01     xiaonong     the first version for stm32f7xx
 */
#include <rtthread.h>
#ifdef USER_USING_MOTORLOWLEVEL
#ifdef USER_USING_MOTORBOARDV0
#include "MC_Lowlevel.h"
#include "stm32f7xx_MClib.h"
#include "MC_Globals.h"

#define NB_CONVERSIONS 16
#define ADC_RIGHT_ALIGNMENT 4
//#define DATAALIGN_LEFT
#define DATAALIGN_RIGHT

/* Definition for USART1 clock resources */
#define ADC1_CLK_ENABLE()           __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC2_CLK_ENABLE()           __HAL_RCC_ADC2_CLK_ENABLE()
#define ADC3_CLK_ENABLE()           __HAL_RCC_ADC3_CLK_ENABLE()
#define ADC_GPIOA_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC_GPIOC_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_GPIOF_CLK_ENABLE()      __HAL_RCC_GPIOF_CLK_ENABLE()

#define ADC_FORCE_RESET()           __HAL_RCC_ADC_FORCE_RESET()
#define ADC_RELEASE_RESET()        	__HAL_RCC_ADC_RELEASE_RESET()

#define ADC_SR ((u32 *)(0x0 + ADC1_BASE))
#define JEOC1 (0x00000004)
#define JEOC2 (0x00000400)
#define JEOC3 (0x00040000)


#define PHASE_C1_CURRENT_CHANNEL     ADC_CHANNEL_6  	/*ADC1  IW*/
#define PHASE_C1_CURRENT_PIN         GPIO_PIN_6
#define PHASE_C1_CURRENT_PORT       	GPIOA

#define PHASE_B1_CURRENT_CHANNEL     ADC_CHANNEL_3	 /*ADC2  IV*/
#define PHASE_B1_CURRENT_PIN         GPIO_PIN_3
#define PHASE_B1_CURRENT_PORT       	GPIOA

#define PHASE_A1_CURRENT_CHANNEL     ADC_CHANNEL_5	 /*ADC3  IU*/
#define PHASE_A1_CURRENT_PIN         GPIO_PIN_7
#define PHASE_A1_CURRENT_PORT       	GPIOF


#define PHASE_C2_CURRENT_CHANNEL     ADC_CHANNEL_9  	/*ADC1  IW*/
#define PHASE_C2_CURRENT_PIN         GPIO_PIN_1
#define PHASE_C2_CURRENT_PORT       	GPIOB

#define PHASE_B2_CURRENT_CHANNEL     ADC_CHANNEL_12	 /*ADC2  IV*/
#define PHASE_B2_CURRENT_PIN         GPIO_PIN_2
#define PHASE_B2_CURRENT_PORT       	GPIOC

#define PHASE_A2_CURRENT_CHANNEL     ADC_CHANNEL_6	 /*ADC3  IU*/
#define PHASE_A2_CURRENT_PIN         GPIO_PIN_8
#define PHASE_A2_CURRENT_PORT       	GPIOF


#define PHASE_C_VOLTAGE_CHANNEL     ADC_CHANNEL_5 	/*ADC1 UW*/
#define PHASE_C_VOLTAGE_PIN         GPIO_PIN_5
#define PHASE_C_VOLTAGE_PORT       	GPIOA

#define PHASE_B_VOLTAGE_CHANNEL     ADC_CHANNEL_4/*ADC2 UV */
#define PHASE_B_VOLTAGE_PIN         GPIO_PIN_4
#define PHASE_B_VOLTAGE_PORT       	GPIOA

#define PHASE_A_VOLTAGE_CHANNEL     ADC_CHANNEL_7/*ADC3 UU */
#define PHASE_A_VOLTAGE_PIN         GPIO_PIN_9
#define PHASE_A_VOLTAGE_PORT       	GPIOF

		 	
#define PHASE_TEMP_CHANNEL     ADC_CHANNEL_8   /*ADC1 TEMP*/
#define PHASE_TEMP_PIN         GPIO_PIN_0
#define PHASE_TEMP_PORT       	GPIOB

#define PHASE_BUS_CURRENT_CHANNEL     ADC_CHANNEL_10/*ADC2 UD*/
#define PHASE_BUS_CURRENT_PIN         GPIO_PIN_0
#define PHASE_BUS_CURRENT_PORT       	GPIOC


#define PHASE_BUS_VOLTAGE_CHANNEL     ADC_CHANNEL_4/*ADC3 ID */
#define PHASE_BUS_VOLTAGE_PIN         GPIO_PIN_6
#define PHASE_BUS_VOLTAGE_PORT       	GPIOF


QIEMW_ADC_HandleTypeDef Global_User_ADC;


void QIEMW_ADC_init(QIEMW_ADC_HandleTypeDef *QIEMW_ADC_Handle)
{
	QIEMW_ADC_Handle->PhaseA.qI1_offset= 0;
	QIEMW_ADC_Handle->PhaseA.qI2_offset= 0;
	QIEMW_ADC_Handle->PhaseA.qV_offset= 0;
	QIEMW_ADC_Handle->PhaseB.qI1_offset = 0;
	QIEMW_ADC_Handle->PhaseB.qI2_offset = 0;
	QIEMW_ADC_Handle->PhaseB.qV_offset = 0;
	QIEMW_ADC_Handle->PhaseC.qI1_offset = 0;
	QIEMW_ADC_Handle->PhaseC.qI2_offset = 0;
	QIEMW_ADC_Handle->PhaseC.qV_offset = 0;
	QIEMW_ADC_Handle->BUS.qV_offset = 0;
	QIEMW_ADC_Handle->BUS.qI_offset = 0;
	QIEMW_ADC_Handle->UR.qV_offset = 0;
	QIEMW_ADC_Handle->US.qV_offset = 0;
	QIEMW_ADC_Handle->UT.qV_offset = 0;
	QIEMW_ADC_Handle->Temp.TEMP_offset = 0;
	

	QIEMW_ADC_Handle->PhaseA.qI_value = 0;
	QIEMW_ADC_Handle->PhaseA.qI1_value = 0;
	QIEMW_ADC_Handle->PhaseA.qI2_value = 0;
	QIEMW_ADC_Handle->PhaseA.qV_value = 0;
	QIEMW_ADC_Handle->PhaseB.qI_value = 0;
	QIEMW_ADC_Handle->PhaseB.qI1_value = 0;
	QIEMW_ADC_Handle->PhaseB.qI2_value = 0;
	QIEMW_ADC_Handle->PhaseB.qV_value = 0;
	QIEMW_ADC_Handle->PhaseC.qI1_value = 0;
	QIEMW_ADC_Handle->PhaseC.qI2_value = 0;
	QIEMW_ADC_Handle->PhaseC.qI_value = 0;
	QIEMW_ADC_Handle->PhaseC.qV_value = 0;
	QIEMW_ADC_Handle->BUS.qV_value = 0;
	QIEMW_ADC_Handle->BUS.qI_value = 0;
	QIEMW_ADC_Handle->UR.qV_value = 0;
	QIEMW_ADC_Handle->US.qV_value = 0;
	QIEMW_ADC_Handle->UT.qV_value = 0;
	QIEMW_ADC_Handle->Temp.TEMP_value = 0;
}

void ADC_Calibration(ADC_HandleTypeDef* hadc,QIEMW_ADC_HandleTypeDef *QIEMW_ADC_Handle)
{
	u8 bIndex;
	
	ADC_ChannelConfTypeDef sConfig;
	HAL_NVIC_DisableIRQ(ADC_IRQn);
	HAL_ADCEx_InjectedStop_IT(hadc);
	
	if(hadc->Instance == ADC1)
	{
		
		sConfig.Channel = PHASE_C1_CURRENT_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->PhaseC.qI1_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
		sConfig.Channel = PHASE_C2_CURRENT_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->PhaseC.qI2_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
		sConfig.Channel = PHASE_C_VOLTAGE_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->PhaseC.qV_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}

		sConfig.Channel = PHASE_TEMP_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->Temp.TEMP_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
	}

	if(hadc->Instance == ADC2)
	{
		sConfig.Channel = PHASE_B1_CURRENT_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->PhaseB.qI1_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
		HAL_ADC_Stop(hadc);
		
		sConfig.Channel = PHASE_B2_CURRENT_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);

		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			
			QIEMW_ADC_Handle->PhaseB.qI2_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
		sConfig.Channel = PHASE_B_VOLTAGE_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);

		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			
			QIEMW_ADC_Handle->PhaseB.qV_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
		sConfig.Channel = PHASE_BUS_CURRENT_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);

		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->BUS.qI_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
	}

	if(hadc->Instance == ADC3)
	{
		sConfig.Channel = PHASE_A1_CURRENT_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->PhaseA.qI1_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
		sConfig.Channel = PHASE_A2_CURRENT_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->PhaseA.qI2_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
		sConfig.Channel = PHASE_A_VOLTAGE_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->PhaseA.qV_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
		sConfig.Channel = PHASE_BUS_VOLTAGE_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		HAL_ADC_ConfigChannel(hadc, &sConfig);
		for(bIndex=NB_CONVERSIONS; bIndex != 0; bIndex--)
		{
			HAL_ADC_Start(hadc);
			while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))) { }
			QIEMW_ADC_Handle->BUS.qV_offset += ((u16)HAL_ADC_GetValue(hadc)>>ADC_RIGHT_ALIGNMENT);
			
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
		}
	}
}

extern rt_err_t myprintf(char *buf, uint8_t size);


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	hadc->Instance = ADC1;
	Global_User_ADC.PhaseC.qI1_value = (s16)(HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1));
	Global_User_ADC.PhaseC.qI2_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	Global_User_ADC.PhaseC.qV_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
	Global_User_ADC.Temp.TEMP_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
	hadc->Instance = ADC2;
	Global_User_ADC.PhaseB.qI1_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
	Global_User_ADC.PhaseB.qI2_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	Global_User_ADC.PhaseB.qV_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
	Global_User_ADC.BUS.qI_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
	hadc->Instance = ADC3;
	Global_User_ADC.PhaseA.qI1_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
	Global_User_ADC.PhaseA.qI2_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	Global_User_ADC.PhaseA.qV_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
	Global_User_ADC.BUS.qV_value = (s16)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);

	Global_User_ADC.PhaseC.qI_value = (s16)(((s32)Global_User_ADC.PhaseC.qI1_value + (s32)Global_User_ADC.PhaseC.qI2_value) >> 1);
	Global_User_ADC.PhaseB.qI_value = (s16)(((s32)Global_User_ADC.PhaseB.qI1_value + (s32)Global_User_ADC.PhaseB.qI2_value) >> 1);
	Global_User_ADC.PhaseA.qI_value = (s16)(((s32)Global_User_ADC.PhaseA.qI1_value + (s32)Global_User_ADC.PhaseA.qI2_value) >> 1);
	#ifdef RT_USING_STEMWIN
	ArrayDeal();
	#endif

    if (SVPWMEOCEvent())
    {
      if ((State == START) || (State == RUN))
      {
        /**********STARTS THE VECTOR CONTROL ************************/    
        IFOC_Model();
      }
  
      #ifdef DAC_FUNCTIONALITY  
        MCDAC_Update_Value(I_A,Stat_Curr_a_b.qI_Component1);
        MCDAC_Update_Value(I_B,Stat_Curr_a_b.qI_Component2);
        MCDAC_Update_Value(I_ALPHA,Stat_Curr_alfa_beta.qI_Component1);
        MCDAC_Update_Value(I_BETA,Stat_Curr_alfa_beta.qI_Component2);
        MCDAC_Update_Value(I_Q,Stat_Curr_q_d.qI_Component1);
        MCDAC_Update_Value(I_D,Stat_Curr_q_d.qI_Component2);
        MCDAC_Update_Value(I_Q_REF,hTorque_Reference);
        MCDAC_Update_Value(I_D_REF,hFlux_Reference);
        MCDAC_Update_Value(V_Q,Stat_Volt_q_d.qV_Component1);
        MCDAC_Update_Value(V_D,Stat_Volt_q_d.qV_Component2);
        MCDAC_Update_Value(V_ALPHA,Stat_Volt_alfa_beta.qV_Component1);
        MCDAC_Update_Value(V_BETA,Stat_Volt_alfa_beta.qV_Component2); 
        MCDAC_Update_Value(SENS_SPEED,((85*hRot_Freq_Hz)-S16_MAX));
        #if (defined ENCODER)
        MCDAC_Update_Value(SENS_ANGLE,ENC_Get_Electrical_Angle());
        #elif (defined TACHO)
        MCDAC_Update_Value(SENS_ANGLE,(s16) 0 );
        #endif
        
        MCDAC_Update_Output();
      #endif
    }
#if 1
	static char i=0;
	i++;
	if(i>200)
	{
		i = 0;
		myprintf("123456789012\r\n", 15);
	}
	#endif
	//__HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
}



void ADC_IRQHandler(void)
{
	ADC_HandleTypeDef ADC_handle;

    /* enter interrupt */
    rt_interrupt_enter();
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3, GPIO_PIN_SET);

	ADC_handle.Instance = ADC1;
	HAL_ADC_IRQHandler(&ADC_handle);
	
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3, GPIO_PIN_RESET);

    /* leave interrupt */
    rt_interrupt_leave();
}
/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
	void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    if (hadc->Instance == ADC1)
    {
 		ADC1_CLK_ENABLE();
		ADC_GPIOA_CLK_ENABLE();
		ADC_GPIOC_CLK_ENABLE();

		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_C1_CURRENT_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_C1_CURRENT_PORT, &GPIO_InitStruct);		

		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_C2_CURRENT_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_C2_CURRENT_PORT, &GPIO_InitStruct);		
		
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_C_VOLTAGE_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_C_VOLTAGE_PORT, &GPIO_InitStruct);		
		
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_TEMP_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_TEMP_PORT, &GPIO_InitStruct);		
    }

	if (hadc->Instance == ADC2)
    {
 		ADC2_CLK_ENABLE();
		ADC_GPIOA_CLK_ENABLE();
		ADC_GPIOC_CLK_ENABLE();

		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_B1_CURRENT_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_B1_CURRENT_PORT, &GPIO_InitStruct);		

		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_B2_CURRENT_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_B2_CURRENT_PORT, &GPIO_InitStruct);		

		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_B_VOLTAGE_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_B_VOLTAGE_PORT, &GPIO_InitStruct);	
		
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_BUS_CURRENT_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_BUS_CURRENT_PORT, &GPIO_InitStruct);		
    }

	
	if (hadc->Instance == ADC3)
    {
 		ADC3_CLK_ENABLE();
		ADC_GPIOA_CLK_ENABLE();
		ADC_GPIOF_CLK_ENABLE();

		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_A1_CURRENT_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_A1_CURRENT_PORT, &GPIO_InitStruct);		

		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_A2_CURRENT_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_A2_CURRENT_PORT, &GPIO_InitStruct);		
		
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_A_VOLTAGE_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_A_VOLTAGE_PORT, &GPIO_InitStruct);		
		
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin = PHASE_BUS_VOLTAGE_PIN;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PHASE_BUS_VOLTAGE_PORT, &GPIO_InitStruct);		
    }

	/*************************test pin************************/
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct); 	
	
	HAL_NVIC_SetPriority(ADC_IRQn,ADC_IRQ_PREEMPT,ADC_IRQ_SUB);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
}



extern void ADC_CalibrationAll(void)
{
	ADC_HandleTypeDef ADC1_handle,ADC2_handle,ADC3_handle;
	ADC_InjectionConfTypeDef ADCConfigInjected;
	ADC_MultiModeTypeDef ADCMultiMode;

	QIEMW_ADC_init(&Global_User_ADC);
	ADC1_handle.Instance = ADC1;
	ADC2_handle.Instance = ADC2;
	ADC3_handle.Instance = ADC3;
	#if 1
	ADC_Calibration(&ADC1_handle,&Global_User_ADC);
	ADC_Calibration(&ADC2_handle,&Global_User_ADC);
	ADC_Calibration(&ADC3_handle,&Global_User_ADC);
	#endif
	
	ADCConfigInjected.AutoInjectedConv = DISABLE;
	ADCConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_TRGO2;
	ADCConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING;
	ADCConfigInjected.InjectedChannel = PHASE_C1_CURRENT_CHANNEL;
	ADCConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 1;
	ADCConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES;
	#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseC.qI1_offset>>4;
	#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseC.qI1_offset;
	ADCConfigInjected.InjectedOffset = 0;
	#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC1_handle,&ADCConfigInjected);

	ADCConfigInjected.InjectedChannel = PHASE_C2_CURRENT_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 2;
	
	#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseC.qI2_offset>>4;
	#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseC.qI2_offset;
	ADCConfigInjected.InjectedOffset = 0;
	#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC1_handle,&ADCConfigInjected);

	ADCConfigInjected.InjectedChannel = PHASE_C_VOLTAGE_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 3;
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseC.qV_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseC.qV_offset;
	ADCConfigInjected.InjectedOffset = 0;

#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC1_handle,&ADCConfigInjected);

	ADCConfigInjected.InjectedChannel = PHASE_TEMP_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 4;
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.Temp.TEMP_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.Temp.TEMP_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC1_handle,&ADCConfigInjected);
	
	
	ADCConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
	ADCConfigInjected.InjectedChannel = PHASE_B1_CURRENT_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 1;
	
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseB.qI1_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseB.qI1_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC2_handle,&ADCConfigInjected);

	ADCConfigInjected.InjectedChannel = PHASE_B2_CURRENT_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 2;
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseB.qI2_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseB.qI2_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC2_handle,&ADCConfigInjected);

	ADCConfigInjected.InjectedChannel = PHASE_B_VOLTAGE_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 3;
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseB.qV_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseB.qV_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC2_handle,&ADCConfigInjected);
	
	ADCConfigInjected.InjectedChannel = PHASE_BUS_CURRENT_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 4;
	
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.BUS.qI_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.BUS.qI_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC2_handle,&ADCConfigInjected);

	ADCConfigInjected.InjectedChannel = PHASE_A1_CURRENT_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 1;
	
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseA.qI1_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseA.qI1_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC3_handle,&ADCConfigInjected);

	ADCConfigInjected.InjectedChannel = PHASE_A2_CURRENT_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 2;
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseA.qI2_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseA.qI2_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC3_handle,&ADCConfigInjected);

	ADCConfigInjected.InjectedChannel = PHASE_A_VOLTAGE_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 3;
	
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseA.qV_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.PhaseA.qV_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC3_handle,&ADCConfigInjected);
	
	ADCConfigInjected.InjectedChannel = PHASE_BUS_VOLTAGE_CHANNEL;
	ADCConfigInjected.InjectedNbrOfConversion = 4;
	ADCConfigInjected.InjectedRank = 4;
#ifdef DATAALIGN_LEFT
	ADCConfigInjected.InjectedOffset = Global_User_ADC.BUS.qV_offset>>4;
#else
	//ADCConfigInjected.InjectedOffset = Global_User_ADC.BUS.qV_offset;
	
	ADCConfigInjected.InjectedOffset = 0;
#endif
	HAL_ADCEx_InjectedConfigChannel(&ADC3_handle,&ADCConfigInjected);

	ADCMultiMode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
	ADCMultiMode.Mode = ADC_TRIPLEMODE_INJECSIMULT;
	ADCMultiMode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
	HAL_ADCEx_MultiModeConfigChannel(&ADC1_handle, &ADCMultiMode);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_ADCEx_InjectedStart_IT(&ADC1_handle);
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
	void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
   
}

extern int stm32_hw_ADC_init(void)
{
	ADC_HandleTypeDef ADC1_handle,ADC2_handle,ADC3_handle;

	QIEMW_ADC_init(&Global_User_ADC);
	ADC1_handle.Instance = ADC1;
	HAL_ADC_DeInit(&ADC1_handle);
	ADC1_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8; /* 96M/8 */
	ADC1_handle.Init.ContinuousConvMode = DISABLE;
	#ifdef DATAALIGN_LEFT
	ADC1_handle.Init.DataAlign = ADC_DATAALIGN_LEFT;
	#else
	ADC1_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	#endif
	#if 1
	ADC1_handle.Init.DiscontinuousConvMode = DISABLE;
	ADC1_handle.Init.DMAContinuousRequests = DISABLE;
	ADC1_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	ADC1_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	ADC1_handle.Init.NbrOfConversion = 1;
	ADC1_handle.Init.NbrOfDiscConversion = 0;
	#endif
	ADC1_handle.Init.Resolution = ADC_RESOLUTION_12B;
	ADC1_handle.Init.ScanConvMode = ENABLE;
	HAL_ADC_Init(&ADC1_handle);
	
	ADC2_handle.Instance = ADC2;
	HAL_ADC_DeInit(&ADC2_handle);

	
	ADC2_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8; /* 96M/8 */
	ADC2_handle.Init.ContinuousConvMode = DISABLE;
	
#ifdef DATAALIGN_LEFT
	ADC2_handle.Init.DataAlign = ADC_DATAALIGN_LEFT;
#else
	ADC2_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
#endif
#if 1
	ADC2_handle.Init.DiscontinuousConvMode = DISABLE;
	ADC2_handle.Init.DMAContinuousRequests = DISABLE;
	ADC2_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	ADC2_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	ADC2_handle.Init.NbrOfConversion = 1;
	ADC2_handle.Init.NbrOfDiscConversion = 0;
	#endif
	ADC2_handle.Init.Resolution = ADC_RESOLUTION_12B;
	ADC2_handle.Init.ScanConvMode = ENABLE;
	HAL_ADC_Init(&ADC2_handle);
	
	ADC3_handle.Instance = ADC3;
	HAL_ADC_DeInit(&ADC3_handle);
	ADC3_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8; /* 96M/8 */
	ADC3_handle.Init.ContinuousConvMode = DISABLE;
	
#ifdef DATAALIGN_LEFT
	ADC3_handle.Init.DataAlign = ADC_DATAALIGN_LEFT;
#else
	ADC3_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
#endif
#if 1
	ADC3_handle.Init.DiscontinuousConvMode = DISABLE;
	ADC3_handle.Init.DMAContinuousRequests = DISABLE;
	ADC3_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	ADC3_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	ADC3_handle.Init.NbrOfConversion = 1;
	ADC3_handle.Init.NbrOfDiscConversion = 0;
	#endif
	ADC3_handle.Init.Resolution = ADC_RESOLUTION_12B;
	ADC3_handle.Init.ScanConvMode = ENABLE;
	HAL_ADC_Init(&ADC3_handle);

	ADC_CalibrationAll();
}

#endif
#endif
/*---------------------------------------------end------------------------------------------*/
