#if 0
/*
 * File      : drv_MiscMotor.c
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
#include "stm32f7xx.h"
#include "board.h"
#include <stm32f7x_type.h>
#include <rtdevice.h>


#define FAN_PIN                    GPIO_PIN_12  /*FAN  */
#define FAN_GPIO_PORT              GPIOH
#define FAN_MODE                   GPIO_MODE_OUTPUT_PP

#define OVP_PIN                    GPIO_PIN_13 /*OVP*/
#define OVP_GPIO_PORT              GPIOB
#define OVP_MODE                   GPIO_MODE_INPUT

#define UVP_PIN                    GPIO_PIN_7 /*UVP*/
#define UVP_GPIO_PORT              GPIOG
#define UVP_MODE                   GPIO_MODE_INPUT

#define OCP_PIN                    GPIO_PIN_5 /*OCP*/
#define OCP_GPIO_PORT              GPIOI
#define OCP_MODE                   GPIO_MODE_INPUT

#define POFF_PIN                   GPIO_PIN_12 /*POFF*/
#define POFF_GPIO_PORT             GPIOB
#define POFF_MODE                  GPIO_MODE_INPUT

#define PON_PIN                    GPIO_PIN_3 /*PON*/
#define PON_GPIO_PORT              GPIOG
#define PON_MODE                   GPIO_MODE_INPUT


int stm32_hw_MiscMotor_init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/* Configure GPIO pin:  */
	GPIO_InitStruct.Pin   = FAN_PIN;
	GPIO_InitStruct.Mode  = FAN_MODE;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(FAN_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin   = OVP_PIN;
	GPIO_InitStruct.Mode  = OVP_MODE;
	HAL_GPIO_Init(OVP_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin   = UVP_PIN;
	GPIO_InitStruct.Mode  = UVP_MODE;
	HAL_GPIO_Init(UVP_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin   = OCP_PIN;
	GPIO_InitStruct.Mode  = OCP_MODE;
	HAL_GPIO_Init(OCP_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin   = POFF_PIN;
	GPIO_InitStruct.Mode  = POFF_MODE;
	HAL_GPIO_Init(POFF_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin   = PON_PIN;
	GPIO_InitStruct.Mode  = PON_MODE;
	HAL_GPIO_Init(PON_GPIO_PORT, &GPIO_InitStruct);

    return 0;
}
INIT_BOARD_EXPORT(stm32_hw_MiscMotor_init);
#endif
