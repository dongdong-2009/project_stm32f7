/*
 * File      : drv_ADC.h
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

#ifndef __DRV_ADC_H__
#define __DRV_ADC_H__

#include <rthw.h>
#include <rtthread.h>
#include "stm32f7xx.h"
#include "board.h"
//#include <stm32f7xx_hal_adc.h>
#include <stm32f7xx_hal_adc_ex.h>
#include <rtdevice.h>
#include <stm32f7x_type.h>


typedef struct 
{
	s16 qI1_value;
	u16 qI1_offset;
	s16 qI2_value;
	u16 qI2_offset;
	s16 qI_value;
	s16 qV_value;
	u16 qV_offset;
} QIEMW_Phase;

typedef struct 
{
	s16 TEMP_value;
	u16 TEMP_offset;
	s16 FANTEMP_value;
	u16 FANTEMP_offset;
} QIEMW_Temp;

typedef struct 
{
	s16 qI_value;
	u16 qI_offset;
	s16 qV_value;
	u16 qV_offset;
} QIEMW_Bus;


/** 
  * @brief  ADC handle Structure definition
  */ 
typedef struct
{
	QIEMW_Phase PhaseA;
	QIEMW_Phase PhaseB;
	QIEMW_Phase PhaseC;
	QIEMW_Bus BUS;
	QIEMW_Phase UR;
	QIEMW_Phase US;
	QIEMW_Phase UT;
	QIEMW_Temp Temp;
} QIEMW_ADC_HandleTypeDef;

extern QIEMW_ADC_HandleTypeDef Global_User_ADC;
int stm32_hw_ADC_init(void);
void QIEMW_ADC_init(QIEMW_ADC_HandleTypeDef *QIEMW_ADC_Handle);
void ADC_Calibration(ADC_HandleTypeDef* hadc,QIEMW_ADC_HandleTypeDef *QIEMW_ADC_Handle);

extern int stm32_hw_ADC_init(void);


#endif
