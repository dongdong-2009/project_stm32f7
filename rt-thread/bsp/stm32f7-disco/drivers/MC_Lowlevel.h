#ifndef MC_LOWLEVEL_H
#define MC_LOWLEVEL_H

#include "stm32f7xx.h"
#include "drv_tim8.h"
#include "drv_qep.h"
#include "drv_ADC.h"


extern void CounterReset(TIM_TypeDef *TIMx);
extern void SetDutyCycles(TIM_TypeDef *TIMx, u16 hTimePhA, u16 hTimePhB, u16 hTimePhC);
extern void PWMOutputsEnable(TIM_TypeDef *TIMx);
extern void PWMOutputsDisable(TIM_TypeDef *TIMx);
extern int stm32_hw_tim8_init(void);


extern u16 GetCounterValue(TIM_TypeDef *TIMx);
extern bool IsCounterUp(TIM_TypeDef *TIMx);

extern void ADC_CalibrationAll(void);
#endif
