/*
 * File      : drv_tim8.c
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
#include <rtdevice.h>
#include "stm32f7xx_MClib.h"
#include "MC_Globals.h"


#define ENC_COUNTER_RESET   (u8) 0


static void _delay_us(uint32_t us)
{
    volatile uint32_t len;
    for (; us > 0; us --)
        for (len = 0; len < 20; len++ );
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
	if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_BREAK) != RESET)
	{
		rt_kprintf(" BREAK1 is triggered\r\n");
		
		_delay_us(1000);
		HAL_GPIO_WritePin(OCDRELEASE_PORT,OCDRELEASE_PIN,GPIO_PIN_RESET);
		rt_kprintf("fault is cleared\r\n");
		_delay_us(1000);
		HAL_GPIO_WritePin(OCDRELEASE_PORT,OCDRELEASE_PIN,GPIO_PIN_SET);
		  //MCL_SetFault(OVER_CURRENT);
	}
	else if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_BREAK2) != RESET)
	{
		rt_kprintf(" BREAK2 is triggered\r\n");
		  //MCL_SetFault(OVER_CURRENT);
	}
	else 
	{
		rt_kprintf("TIM8 is triggered\r\n");
	}

}


void TIM8_BRK_TIM12_IRQHandler(void)
{
	TIM_HandleTypeDef	 TimHandle;
	
	/* Set TIMx instance */
	
	/*enter interrupt */
	rt_interrupt_enter();
	
	TimHandle.Instance = TIM8;
	HAL_TIM_IRQHandler(&TimHandle);
		
    /* leave interrupt */
    rt_interrupt_leave();

}

void TIM8_UP_TIM13_IRQHandler(void)
{
	TIM_HandleTypeDef	 TimHandle;
	rt_interrupt_enter();
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3, GPIO_PIN_SET);
	TimHandle.Instance = TIM8;
	HAL_TIM_IRQHandler(&TimHandle);
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3, GPIO_PIN_RESET);
    /* leave interrupt */
    rt_interrupt_leave();
}

extern void CounterReset(TIM_TypeDef *TIMx)
{
	TIMx->CNT = ENC_COUNTER_RESET;

}

extern void SetDutyCycles(TIM_TypeDef *TIMx, u16 hTimePhA, u16 hTimePhB, u16 hTimePhC)
{
	TIMx->CCR1 = hTimePhA;
	TIMx->CCR2 = hTimePhB;
	TIMx->CCR3 = hTimePhC;
}


extern void PWMOutputsEnable(TIM_TypeDef *TIMx)
{
	TIMx->BDTR |= TIM_BDTR_MOE;
	HAL_GPIO_WritePin(OCDRELEASE_PORT,OCDRELEASE_PIN,GPIO_PIN_RESET);
}


extern void PWMOutputsDisable(TIM_TypeDef *TIMx)
{
	TIMx->BDTR &= (~TIM_BDTR_MOE);
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
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Init;
    if (htim->Instance == TIM8)
    {
        /* Enable GPIO clock */
        TIM8_GPIOA_CLK_ENABLE();
        TIM8_GPIOB_CLK_ENABLE();
		TIM8_GPIOC_CLK_ENABLE();
        TIM8_GPIOH_CLK_ENABLE();
		TIM8_GPIOI_CLK_ENABLE();

        /* Enable TIMx clock */
        TIM8_CLK_ENABLE();

        /* GPIO pin configuration  */
	 	GPIO_Init.Pin = PWMWP_PIN | PWMVP_PIN | PWMBREAK_PIN;
		GPIO_Init.Mode = GPIO_MODE_AF_PP;
		GPIO_Init.Pull = GPIO_NOPULL;
		GPIO_Init.Speed = GPIO_SPEED_LOW;
		GPIO_Init.Alternate = PWMWP_AF;
		HAL_GPIO_Init(PWMWP_GPIO_PORT,&GPIO_Init);
		
	 	GPIO_Init.Pin = PWMUP_PIN;
		HAL_GPIO_Init(PWMUP_GPIO_PORT,&GPIO_Init);

		GPIO_Init.Pin = PWMWN_PIN | PWMVN_PIN | PWMUN_PIN;
		HAL_GPIO_Init(PWMWN_GPIO_PORT,&GPIO_Init);

		GPIO_Init.Pin = FAULT1_PIN;
		HAL_GPIO_Init(FAULT1_GPIO_PORT,&GPIO_Init);

		GPIO_Init.Pin = OCDRELEASE_PIN;
		GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_Init.Pull = GPIO_PULLUP;
		GPIO_Init.Speed = GPIO_SPEED_LOW;
		HAL_GPIO_Init(OCDRELEASE_PORT,&GPIO_Init);
        /* NVIC for BRK */
        HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, TIM8_BRK_TIM12_IRQ_PREEMPT, TIM8_BRK_TIM12_IRQ_SUB );
        HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
		
		#ifdef TIM8UPDATEITENABLE
		HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, TIM8_BRK_TIM12_IRQ_PREEMPT, TIM8_BRK_TIM12_IRQ_SUB );
        HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
		#endif
    }
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        /* Reset peripherals */
        TIM8_FORCE_RESET();
        TIM8_RELEASE_RESET();

        /* Disable peripherals and GPIO Clocks */
        HAL_GPIO_DeInit(PWMWP_GPIO_PORT, PWMWP_PIN | PWMVP_PIN | PWMUP_PIN | PWMBREAK_PIN);
        HAL_GPIO_DeInit(PWMWN_GPIO_PORT, PWMWN_PIN | PWMVN_PIN | PWMUN_PIN);
        HAL_GPIO_DeInit(FAULT1_GPIO_PORT, FAULT1_PIN | FAULT2_PIN);
		
        HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
		
		#ifdef TIM8UPDATEITENABLE
        HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
		#endif
    }
}


extern int stm32_hw_tim8_init(void)
{
	TIM_HandleTypeDef	 TimHandle;
	TIM_OC_InitTypeDef PwmConfig;
	TIM_BreakDeadTimeConfigTypeDef PwmBDTRconfig;
	TIM_MasterConfigTypeDef TIM8MasterConfig;
	char deadtime;
	/* Set TIMx instance */
	TimHandle.Instance = TIM8;

	HAL_TIM_PWM_DeInit(&TimHandle);

	/* Initialize TIMx peripheral	*/
	TimHandle.Init.Period            = PWM_PERIOD;
	TimHandle.Init.Prescaler         = PWM_PRSC;
	TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV2;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
	TimHandle.Init.RepetitionCounter = REP_RATE;
	if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
	{
	  /* Initialization Error */
	  /*Error_Handler(); */
	}
	PwmConfig.OCMode = TIM_OCMODE_PWM1;
	PwmConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	PwmConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	PwmConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
	PwmConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	PwmConfig.Pulse = PWM_PERIOD/2;
	PwmConfig.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&TimHandle,&PwmConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle,&PwmConfig, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&TimHandle,&PwmConfig, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&TimHandle,&PwmConfig, TIM_CHANNEL_3);

	TIM_OC1_SetConfig(TIM8,&PwmConfig);
	TIM_OC2_SetConfig(TIM8,&PwmConfig);
	TIM_OC3_SetConfig(TIM8,&PwmConfig);
	TIM_OC4_SetConfig(TIM8,&PwmConfig);

	if (DEADTIME1 < 128)
		deadtime = DEADTIME1;
	else if( DEADTIME2 <64)
		deadtime  = DEADTIME2 | 0x80;
	else if( DEADTIME3 < 32)
		deadtime = DEADTIME3 | 0xC0;
	else
		deadtime = DEADTIME4 | 0xE0;
	PwmBDTRconfig.DeadTime  = deadtime;
	PwmBDTRconfig.LockLevel = TIM_LOCKLEVEL_1;
	PwmBDTRconfig.OffStateRunMode = TIM_OSSR_ENABLE;
	PwmBDTRconfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
	/*Break 1 */
	PwmBDTRconfig.Break2State = TIM_BREAK2_DISABLE;/* NOT used any more */
	PwmBDTRconfig.Break2Polarity = TIM_BREAK2POLARITY_LOW;
	PwmBDTRconfig.Break2Filter = 0x0;
	/*break 0*/
	PwmBDTRconfig.BreakState = TIM_BREAK_DISABLE;
	PwmBDTRconfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	PwmBDTRconfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	PwmBDTRconfig.BreakFilter = 0x0;
	HAL_TIMEx_ConfigBreakDeadTime(&TimHandle,&PwmBDTRconfig);

	TIM8MasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	TIM8MasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
	TIM8MasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	HAL_TIMEx_MasterConfigSynchronization(&TimHandle,&TIM8MasterConfig);
	
	TIM8->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC2E | \
					TIM_CCER_CC2NE | TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC4E;
	TIM8->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR;

	
	HAL_TIM_PWM_Start(&TimHandle,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandle,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TimHandle,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TimHandle,TIM_CHANNEL_4);
	
	__HAL_TIM_ENABLE_IT(&TimHandle, TIM_IT_BREAK);
	#ifdef TIM8UPDATEITENABLE
	__HAL_TIM_ENABLE_IT(&TimHandle, TIM_IT_UPDATE);
	#endif
	PWMOutputsDisable(TIM8);
    return 0;
}
#endif
