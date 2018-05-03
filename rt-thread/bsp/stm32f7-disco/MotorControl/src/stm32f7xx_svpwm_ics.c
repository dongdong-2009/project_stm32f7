/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32x_svpwm_ics.c
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : ICS current reading and PWM generation module 
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 09/07/08 v2.0.1
* 14/07/08 v2.0.2
* 17/07/08 v2.0.3
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

#include "STM32F7xx_MCconf.h"

#ifdef ICS_SENSORS

/* Includes-------------------------------------------------------------------*/
//#include "stm32f10x_lib.h"
#include "stm32f7x_type.h"
#include "stm32f7xx_svpwm_ics.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SQRT_3		1.732051
#define T		(PWM_PERIOD * 4)
#define T_SQRT3         (u16)(T * SQRT_3)

#define SECTOR_1	(u32)1
#define SECTOR_2	(u32)2
#define SECTOR_3	(u32)3
#define SECTOR_4	(u32)4
#define SECTOR_5	(u32)5
#define SECTOR_6	(u32)6

#define PHASE_A_MSK       (u32)((u32)(PHASE_A_ADC_CHANNEL) << 10)
#define PHASE_B_MSK       (u32)((u32)(PHASE_B_ADC_CHANNEL) << 10)

#define TEMP_FDBK_MSK     (u32)((u32)(TEMP_FDBK_CHANNEL) <<15)
#define BUS_VOLT_FDBK_MSK (u32)((u32)(BUS_VOLT_FDBK_CHANNEL) <<15)
#define SEQUENCE_LENGHT    0x00100000

#define ADC_PRE_EMPTION_PRIORITY 1
#define ADC_SUB_PRIORITY 1

#define BRK_PRE_EMPTION_PRIORITY 0
#define BRK_SUB_PRIORITY 1

#define LOW_SIDE_POLARITY  TIM_OCIdleState_Reset


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u16 hPhaseAOffset;
static u16 hPhaseBOffset;
    
/* Private function prototypes -----------------------------------------------*/

void SVPWM_IcsInjectedConvConfig(void);

/*******************************************************************************
* Function Name  : SVPWM_IcsInit
* Description    : It initializes PWM and ADC peripherals
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_IcsInit(void)
{ 
	
 	stm32_hw_tim8_init();
	stm32_hw_ADC_init();
  	SVPWM_IcsCurrentReadingCalibration();
} 

/*******************************************************************************
* Function Name  : SVPWM_IcsCurrentReadingCalibration
* Description    : Store zero current converted values for current reading 
                   network offset compensation in case of Ics 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_IcsCurrentReadingCalibration(void)
{
 	//ADC_CalibrationAll();
   
 
}



/*******************************************************************************
* Function Name  : SVPWM_IcsPhaseCurrentValues
* Description    : This function computes current values of Phase A and Phase B 
*                 in q1.15 format starting from values acquired from the A/D 
*                 Converter peripheral.
* Input          : None
* Output         : Stat_Curr_a_b
* Return         : None
*******************************************************************************/
Curr_Components SVPWM_IcsGetPhaseCurrentValues(void)
{
  Curr_Components Local_Stator_Currents;
  s32 wAux;

      
 // Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL vale)  
  wAux = Global_User_ADC.PhaseA.qI_value;          
 //Saturation of Ia 
  if (wAux < S16_MIN)
  {
    Local_Stator_Currents.qI_Component1= S16_MIN;
  }  
  else  if (wAux > S16_MAX)
        { 
          Local_Stator_Currents.qI_Component1= S16_MAX;
        }
        else
        {
          Local_Stator_Currents.qI_Component1= wAux;
        }
                     
 // Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value)
  wAux = Global_User_ADC.PhaseB.qI_value;
 // Saturation of Ib
  if (wAux < S16_MIN)
  {
    Local_Stator_Currents.qI_Component2= S16_MIN;
  }  
  else  if (wAux > S16_MAX)
        { 
          Local_Stator_Currents.qI_Component2= S16_MAX;
        }
        else
        {
          Local_Stator_Currents.qI_Component2= wAux;
        }
  
  return(Local_Stator_Currents); 
}

/*******************************************************************************
* Function Name  : SVPWM_IcsCalcDutyCycles
* Description    : Computes duty cycle values corresponding to the input value
		   and configures 
* Input          : Stat_Volt_alfa_beta
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_IcsCalcDutyCycles (Volt_Components Stat_Volt_Input)
{
   u8 bSector;
   s32 wX, wY, wZ, wUAlpha, wUBeta;
   u16  hTimePhA=0, hTimePhB=0, hTimePhC=0;
    
   wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;
   wUBeta = -(Stat_Volt_Input.qV_Component2 * T);

   wX = wUBeta;
   wY = (wUBeta + wUAlpha)/2;
   wZ = (wUBeta - wUAlpha)/2;

  // Sector calculation from wX, wY, wZ
   if (wY<0)
   {
      if (wZ<0)
      {
        bSector = SECTOR_5;
      }
      else // wZ >= 0
        if (wX<=0)
        {
          bSector = SECTOR_4;
        }
        else // wX > 0
        {
          bSector = SECTOR_3;
        }
   }
   else // wY > 0
   {
     if (wZ>=0)
     {
       bSector = SECTOR_2;
     }
     else // wZ < 0
       if (wX<=0)
       {  
         bSector = SECTOR_6;
       }
       else // wX > 0
       {
         bSector = SECTOR_1;
       }
    }
   
   /* Duty cycles computation */
  
  switch(bSector)
  {  
    case SECTOR_1:
    case SECTOR_4:
                hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
		hTimePhB = hTimePhA + wZ/131072;
		hTimePhC = hTimePhB - wX/131072;                                       
                break;
    case SECTOR_2:
    case SECTOR_5:  
                hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
        	hTimePhB = hTimePhA + wZ/131072;
		hTimePhC = hTimePhA - wY/131072;
                break;

    case SECTOR_3:
    case SECTOR_6:
                hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
		hTimePhC = hTimePhA - wY/131072;
		hTimePhB = hTimePhC + wX/131072;
                break;
    default:
		break;
   }
  
  /* Load compare registers values */
   SetDutyCycles(TIM8, hTimePhA, hTimePhB, hTimePhC);
}

/*******************************************************************************
* Function Name  : SVPWMEOCEvent
* Description    :  Routine to be performed inside the end of conversion ISR
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 SVPWMEOCEvent(void)
{
  return ((u8)(1));
}
#endif //ICS_SENSORS

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/  
