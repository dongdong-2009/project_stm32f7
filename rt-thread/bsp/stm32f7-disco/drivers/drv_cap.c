/*
 * File      : drv_qep.c
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
 * 2015-10-08    helong     the first version for stm32f7xx
 */
 
#include <rtthread.h>
#ifdef USER_USING_MOTORLOWLEVEL
#include "MC_Lowlevel.h"
#include "board.h"
#include "MC_encoder_param.h"
#include <stm32f7x_type.h>
#include <rtdevice.h>
#include "stm32f7xx_MClib.h"
#include "MC_Globals.h"
#define CAP
#ifdef CAP
/* Definition for USART1 clock resources */
#define TIM2_CLK_ENABLE()            __HAL_RCC_TIM2_CLK_ENABLE()
#define TIM2_GPIOH_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define TIM2_FORCE_RESET()         __HAL_RCC_TIM2_FORCE_RESET()
#define TIM2_RELEASE_RESET()       __HAL_RCC_TIM2_RELEASE_RESET()

#define CAP1_PIN                    GPIO_PIN_10  /*TIM5_CH1  */
#define CAP1_GPIO_PORT              GPIOB
#define CAP1_AF                     GPIO_AF1_TIM2




#define TIMx_PRE_EMPTION_PRIORITY 2
#define TIMx_SUB_PRIORITY 0

//static bool bIs_First_Measurement = TRUE;
//static bool bError_Speed_Measurement = FALSE;


/*******************************************************************************
* Function Name  : TIMx_IRQHandler
* Description    : This function handles TIMx Update interrupt request.
                   Encoder unit connected to TIMx (x = 2,3 or 4)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
  	unsigned int t_array[1000];
    
	 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{
		static unsigned int i = 0;
		int j;
		if(i < 1000)
		{
			t_array[i] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			i++;
		}
		else 
		{
			i = 0;
			for(j = 0; j < 999; j++)
			{
				rt_kprintf("%d\r\n",t_array[j+1]-t_array[j]);
			}
		}
	}

  
  void TIM2_IRQHandler(void)
  {
	  TIM_HandleTypeDef *htim; 
	  
	  rt_interrupt_enter();
	  htim->Instance = TIM2;

		  __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC3);
		 
			HAL_TIM_IC_CaptureCallback(htim);
	  
	  //HAL_TIM_IRQHandler(&htim);
	  rt_interrupt_leave();
  }








extern u16 GetICValue(TIM_TypeDef *TIMx)
{
	return(TIMx->CNT);
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
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Init;

    /* Enable GPIO clock */
	TIM2_GPIOH_CLK_ENABLE();
    /* Enable TIMx clock */
    TIM2_CLK_ENABLE();

    /* GPIO pin configuration  */
 	GPIO_Init.Pin = CAP1_PIN;
	GPIO_Init.Mode = GPIO_MODE_AF_PP;
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_HIGH;
	GPIO_Init.Alternate = CAP1_AF;
	HAL_GPIO_Init(CAP1_GPIO_PORT,&GPIO_Init);

}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
    /* Reset peripherals */
    TIM2_FORCE_RESET();
    TIM2_RELEASE_RESET();

    /* Disable peripherals and GPIO Clocks */
    HAL_GPIO_DeInit(CAP1_GPIO_PORT, CAP1_PIN);
	
	HAL_NVIC_SetPriority(TIM2_IRQn, TIM2_IRQ_PREEMPT,TIM2_IRQ_SUB);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
}


#define POLENUM (5)
extern int stm32_hw_CAP_init(void)
{
	TIM_HandleTypeDef htim;  
	TIM_IC_InitTypeDef sConfig;
	
	htim.Instance = TIM2;
	HAL_TIM_Base_DeInit(&htim);
	HAL_TIM_IC_DeInit(&htim);
    //TIM2_CLK_ENABLE();
	//HAL_TIM_IC_MspInit(&htim);
	#if 1
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = 0xffff;
	htim.Init.Prescaler = 0x0;
	//htim.Init.RepetitionCounter = 
	HAL_TIM_IC_Init(&htim);	
	
	sConfig.ICFilter = 0xa;
	sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	sConfig.ICPrescaler = TIM_ICPSC_DIV1;
	sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	
	HAL_TIM_IC_ConfigChannel(&htim,  &sConfig,TIM_CHANNEL_3);
	#endif
	
	HAL_NVIC_SetPriority(TIM2_IRQn, TIM2_IRQ_PREEMPT,TIM2_IRQ_SUB);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	//__HAL_TIM_ENABLE_IT(&htim, TIM_IT_CC2);
	
	HAL_TIM_IC_Start_IT(&htim, TIM_CHANNEL_3);
	
    return 0;
}
#endif 
#endif
/*------------------------end-------------------------------*/

