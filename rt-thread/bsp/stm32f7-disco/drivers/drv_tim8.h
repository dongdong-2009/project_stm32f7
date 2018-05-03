/*
 * File      : drv_pwm.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_TIM8_H
#define __DRV_TIM8_H

#include <rthw.h>
#include <rtthread.h>
#include <stm32f7x_type.h>
#include <MC_Control_Param.h>

/* Definition for USART1 clock resources */
#define TIM8_CLK_ENABLE()            __HAL_RCC_TIM8_CLK_ENABLE()
#define TIM8_GPIOA_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define TIM8_GPIOB_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define TIM8_GPIOC_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define TIM8_GPIOH_CLK_ENABLE()      __HAL_RCC_GPIOH_CLK_ENABLE()
#define TIM8_GPIOI_CLK_ENABLE()      __HAL_RCC_GPIOI_CLK_ENABLE()

#define TIM8_FORCE_RESET()         __HAL_RCC_TIM8_FORCE_RESET()
#define TIM8_RELEASE_RESET()       __HAL_RCC_TIM8_RELEASE_RESET()

#define PWMWP_PIN                    GPIO_PIN_6
#define PWMWP_GPIO_PORT              GPIOC
#define PWMWP_AF                     GPIO_AF3_TIM8

#define PWMWN_PIN                    GPIO_PIN_13
#define PWMWN_GPIO_PORT              GPIOH
#define PWMWN_AF                     GPIO_AF3_TIM8

#define PWMVP_PIN                    GPIO_PIN_7
#define PWMVP_GPIO_PORT              GPIOC
#define PWMVP_AF                     GPIO_AF3_TIM8

#define PWMVN_PIN                    GPIO_PIN_14
#define PWMVN_GPIO_PORT              GPIOH
#define PWMVN_AF                     GPIO_AF3_TIM8

#define PWMUP_PIN                    GPIO_PIN_7
#define PWMUP_GPIO_PORT              GPIOI
#define PWMUP_AF                     GPIO_AF3_TIM8

#define PWMUN_PIN                    GPIO_PIN_15
#define PWMUN_GPIO_PORT              GPIOH
#define PWMUN_AF                     GPIO_AF3_TIM8

#define PWMBREAK_PIN                    GPIO_PIN_9
#define PWMBREAK_GPIO_PORT              GPIOC
#define PWMBREAK_AF                     GPIO_AF3_TIM8

#define FAULT1_PIN                    GPIO_PIN_4
#define FAULT1_GPIO_PORT              GPIOI
#define FAULT1_AF                     GPIO_AF3_TIM8

#define FAULT2_PIN                    GPIO_PIN_1
#define FAULT2_GPIO_PORT              GPIOI
#define FAULT2_AF                     GPIO_AF3_TIM8

#define OCDRELEASE_PIN                    GPIO_PIN_8
#define OCDRELEASE_PORT              GPIOA
//#define OCDRELEASE_AF                     GPIO_AF3_TIM8


#endif  /*__MC_PWM_ICS_PRM_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

