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

#ifdef ENCODER
/* Definition for USART1 clock resources */
#define TIM5_CLK_ENABLE()            __HAL_RCC_TIM5_CLK_ENABLE()
#define TIM5_GPIOH_CLK_ENABLE()      __HAL_RCC_GPIOH_CLK_ENABLE()

#define TIM5_FORCE_RESET()         __HAL_RCC_TIM5_FORCE_RESET()
#define TIM5_RELEASE_RESET()       __HAL_RCC_TIM5_RELEASE_RESET()

#define QEP1_PIN                    GPIO_PIN_0  /*TIM5_CH1  */
#define QEP1_GPIO_PORT              GPIOA
#define QEP1_AF                     GPIO_AF2_TIM5

#define QEP2_PIN                    GPIO_PIN_11 /*TIM5_CH2*/
#define QEP2_GPIO_PORT              GPIOH
#define QEP2_AF                     GPIO_AF2_TIM5

#define ENC_COUNTER_RESET   (u8)0
#define ICx_FILTER          (u8)8 

#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))

#define TIMx_PRE_EMPTION_PRIORITY 2
#define TIMx_SUB_PRIORITY 0

static s16 hPrevious_angle, hSpeed_Buffer[SPEED_BUFFER_SIZE], hRot_Speed;
static u8 bSpeed_Buffer_Index = 0;
static volatile u16 hEncoder_Timer_Overflow = 0; 
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
#if defined(TIMER2_HANDLES_ENCODER)
  
    
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
	  if (hEncoder_Timer_Overflow != U16_MAX)  
	  {
		 hEncoder_Timer_Overflow++;
	  }
  }
  
  void TIM2_IRQHandler(void)
  {
	  TIM_HandleTypeDef htim; 
	  
	  rt_interrupt_enter();
	  htim.Instance = TIM2;
	  HAL_TIM_IRQHandler(&htim);
	  rt_interrupt_leave();
  }


#elif defined(TIMER3_HANDLES_ENCODER)
 

  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
	  if (hEncoder_Timer_Overflow != U16_MAX)  
	  {
		 hEncoder_Timer_Overflow++;
	  }
  }
  
  void TIM3_IRQHandler(void)
  {
	  TIM_HandleTypeDef htim; 
	  
	  rt_interrupt_enter();
	  htim.Instance = TIM3;
	  HAL_TIM_IRQHandler(&htim);
	  rt_interrupt_leave();
  }


#elif defined(TIMER4_HANDLES_ENCODER)
  
    
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (hEncoder_Timer_Overflow != U16_MAX)	
	{
	   hEncoder_Timer_Overflow++;
	}
}

void TIM4_IRQHandler(void)
{
	TIM_HandleTypeDef htim; 
	
	rt_interrupt_enter();
	htim.Instance = TIM4;
	HAL_TIM_IRQHandler(&htim);
	rt_interrupt_leave();
}


#elif defined(TIMER5_HANDLES_ENCODER)

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (hEncoder_Timer_Overflow != U16_MAX)	
	{
	   hEncoder_Timer_Overflow++;
	}
}

void TIM5_IRQHandler(void)
{
	TIM_HandleTypeDef htim; 
	
	rt_interrupt_enter();
	htim.Instance = TIM5;
	HAL_TIM_IRQHandler(&htim);
	rt_interrupt_leave();
}
#endif // TIMER4_HANDLES_ENCODER




extern u16 GetCounterValue(TIM_TypeDef *TIMx)
{
	return(TIMx->CNT);
}

extern bool IsCounterUp(TIM_TypeDef *TIMx)
{
 	return((ENCODER_TIMER->CR1 & TIM_COUNTERMODE_UP) == TIM_COUNTERMODE_UP);
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
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Init;

    /* Enable GPIO clock */
	TIM5_GPIOH_CLK_ENABLE();
    /* Enable TIMx clock */
    TIM5_CLK_ENABLE();

    /* GPIO pin configuration  */
 	GPIO_Init.Pin = QEP1_PIN;
	GPIO_Init.Mode = GPIO_MODE_AF_PP;
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_HIGH;
	GPIO_Init.Alternate = QEP1_AF;
	HAL_GPIO_Init(QEP1_GPIO_PORT,&GPIO_Init);

	GPIO_Init.Pin = QEP2_PIN;
	HAL_GPIO_Init(QEP2_GPIO_PORT,&GPIO_Init);
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim)
{
    /* Reset peripherals */
    TIM5_FORCE_RESET();
    TIM5_RELEASE_RESET();

    /* Disable peripherals and GPIO Clocks */
    HAL_GPIO_DeInit(QEP1_GPIO_PORT, QEP1_PIN);
    HAL_GPIO_DeInit(QEP2_GPIO_PORT, QEP2_PIN);
	
	HAL_NVIC_SetPriority(TIM5_IRQn, TIM5_IRQ_PREEMPT,TIM5_IRQ_SUB);
	HAL_NVIC_EnableIRQ(TIM5_IRQn);
}


#define POLENUM (5)
extern int stm32_hw_QEP_init(void)
{
	TIM_HandleTypeDef htim;  
	TIM_Encoder_InitTypeDef sConfig;
	
	htim.Instance = TIM5;
	HAL_TIM_Base_DeInit(&htim);
	HAL_TIM_Encoder_DeInit(&htim);
    TIM5_CLK_ENABLE();
	HAL_TIM_Encoder_MspInit(&htim);
	#if 1
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = (POLENUM*(4*ENCODER_PPR))-1;
	htim.Init.Prescaler = 0x0;
	HAL_TIM_IC_Init(&htim);	
	
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Filter = ICx_FILTER;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Selection = TIM_CCMR1_CC1S_0;//????
	sConfig.IC2Filter = ICx_FILTER;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Selection = TIM_CCMR1_CC1S_0;
	HAL_TIM_Encoder_Init(&htim,  &sConfig);
	#endif
	__HAL_TIM_ENABLE_IT(&htim, TIM_IT_UPDATE);
	
	HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_2);
	
    return 0;
}
#endif 
#endif
/*------------------------end------------------------------------------*/
