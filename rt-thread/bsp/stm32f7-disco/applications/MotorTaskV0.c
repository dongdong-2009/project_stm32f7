#include <rtthread.h>
#ifdef USER_USING_MOTORBOARDV0

#include "finsh.h"
#include "stm32f7xx.h"
#include "board.h"
#include <stm32f7x_type.h>
#include <rtdevice.h>
#include "stm32f7xx_MClib.h"
#include "MC_Globals.h"
#define SLEEPTIME(ms) (((ms)/1000)*RT_TICK_PER_SECOND)


#define USER_MOTORTASK_PRIORITY (10)

/**
*FOR FAN
*/
#define FAN_PIN                    GPIO_PIN_6
#define FAN_GPIO_PORT              GPIOE
#define FAN_MODE                   GPIO_MODE_OUTPUT_PP

#define FAN_GPIOE_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()

/**
*tempreture select
*/
#define TS0_PIN                    GPIO_PIN_3
#define TS0_GPIO_PORT              GPIOE
#define TS0_MODE                   GPIO_MODE_OUTPUT_PP
	
#define TS0_GPIOE_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()

#define TS1_PIN                    GPIO_PIN_5
#define TS1_GPIO_PORT              GPIOE
#define TS1_MODE                   GPIO_MODE_OUTPUT_PP
	
#define TS1_GPIOE_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()



extern rt_err_t myscanf(char *buf);


void USER_MotorTask_entry(void *parameter)
{
rt_device_t device;
char rx_data[15],i;

#ifdef THREE_SHUNT  
	SVPWM_3ShuntInit();
#elif defined ICS_SENSORS
	SVPWM_IcsInit();
#elif defined SINGLE_SHUNT
	SVPWM_1ShuntInit();
#endif

#if defined(TACHO)
	TAC_TachoTimerInit();
	TAC_InitTachoMeasure();  
#elif defined(ENCODER)
	ENC_Init();
#endif

	
  
  //TB_Init();
  
  
  PID_Init(&PID_Torque_InitStructure, &PID_Flux_InitStructure, &PID_Speed_InitStructure);
  
  /* TIM1 Counter Clock stopped when the core is halted */
  //DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
  
  
  // Init Bus voltage and Temperature arrays  
 
  MCL_Init_Arrays();
    
 
  State = IDLE;
  
  PWMOutputsEnable(TIM8);
  while(1)
  {
    //MCL_ChkPowerStage();    
    
    /*
    	* IDLE--INIT---START-----RUN-----BREAK-----STOP----WAITE----FAULT *
    	*/  
    if(myscanf(rx_data) == RT_EOK)
		rt_kprintf("%s\r\n", rx_data);
    #if 0
	rt_kprintf("input i\r\n");
		for (j = 0; j <13; j++)
		i[j]=0;
	device = rt_device_find("uart1");
	for(j=0;j<13;j++)
	{
		while(i[j] == 0)
		{
			rt_device_read (device, 0, &i[j], 13);
		}
	}
	rt_kprintf("i = %s\r\n", i);
	#endif
    switch (State)
    {
      case IDLE:     
        break;
        
      case INIT:
        MCL_Init();
        TB_Set_StartUp_Timeout(STARTUP_TIMEOUT);
        State = START;
        break;
         
      case START: 

#ifdef TACHO          
          if(TAC_ValidSpeedInfo(TACHO_SPEED_VAL))
          {           
            
            if ((wGlobal_Flags & CLOSED_LOOP) == CLOSED_LOOP)
            {              
              //computes hRot_Freq [1.15], i.e the rotor mechanical frequency           
              hRot_Freq_Hz = TAC_GetRotorFreqInHz();                   
              
              //It initializes Integral term of speed PID to avoid discontinuity
              //in torque reference
              PID_Speed_InitStructure.wIntegral=((s32)(hTorque_Reference*256));
            }
            State = RUN; 
          }
          else 
            if (TB_StartUp_Timeout_IsElapsed())
            {
                //shutdown power
                
                //MCL_SetFault(SPEED_FEEDBACK);

            }

            
#elif defined ENCODER          
          {
            s16 hMech_Speed_Hz;
            s16 hLocalSpeed_Reference = hSpeed_Reference;
            
            hMech_Speed_Hz = ENC_Get_Mechanical_Speed();
            if((wGlobal_Flags & CLOSED_LOOP) == CLOSED_LOOP)
            {  
              u16 Abs_Freq;
              Abs_Freq = (hMech_Speed_Hz < 0 ? -hMech_Speed_Hz : hMech_Speed_Hz);
              
              if (Abs_Freq > ENCODER_CL_ENABLE)
              {              
                //It initializes Integral term of speed PID to avoid discontinuity
                //in torque reference
                PID_Speed_InitStructure.wIntegral =
                  (((s32)(hTorque_Reference)*128)-
                   ((s32)(PID_Speed_InitStructure.hKp_Gain*8)*
                    (hLocalSpeed_Reference-hMech_Speed_Hz)));
                State = RUN; 
              }
              else if (TB_StartUp_Timeout_IsElapsed())
              {
                //shutdown power
                
                MCL_SetFault(SPEED_FEEDBACK);
                
              }
            }
            else //Torque mode loop
            { 
              if (hMech_Speed_Hz != 0)
              {
                State = RUN; 
              }
              else if (TB_StartUp_Timeout_IsElapsed())
              {
                //shutdown power
                
                MCL_SetFault(SPEED_FEEDBACK);
                
              }
            }
          }

#endif
        break;
      
      case RUN:
        
#ifdef TACHO 
            if (TAC_IsTimedOut())
             
			
#elif defined ENCODER             

            if(ENC_ErrorOnFeedback() == TRUE)
              
#endif            
            {  
              //MCL_SetFault(SPEED_FDBK_TIMED_OUT);
              
            }

            break;         
            
      case BRAKE:    
          State = STOP;
        break;  
        
      case STOP: 
          
          // shutdown power 
                    
          /* Main PWM Output Disable */
		  PWMOutputsDisable(TIM8);
          
#ifdef THREE_SHUNT          
          SVPWM_3ShuntAdvCurrentReading(DISABLE);
#endif
#ifdef SINGLE_SHUNT          
          SVPWM_1ShuntAdvCurrentReading(DISABLE);
#endif 
          
          Stat_Volt_alfa_beta.qV_Component1 = Stat_Volt_alfa_beta.qV_Component2 = 0;
          
#ifdef ICS_SENSORS
          SVPWM_IcsCalcDutyCycles(Stat_Volt_alfa_beta);
#elif defined THREE_SHUNT
          SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);
#endif        
          State = WAIT;
          
        
        break;
          
        case WAIT:    // wait state

#ifdef TACHO
           if (TAC_GetRotorFreqInHz() == 0) 
#elif defined ENCODER
           if (ENC_Get_Mechanical_Speed() == 0)  
#endif          
            {
              State = IDLE;
            }
          break; 
          
         case FAULT:   // Fault detected  
              
             if (MCL_ClearFault() == TRUE)
                  {
                    if(wGlobal_Flags & CLOSED_LOOP == CLOSED_LOOP)
                    {
                      //bMenu_index = CONTROL_MODE_MENU_1;
                    }
                    else
                    {
                      //bMenu_index = CONTROL_MODE_MENU_6;
                    }   
                    State = IDLE;
                  }
              break;
           
           
     
        default:
          break;     
    }
	
	//rt_thread_delay(SLEEPTIME(20));
  }
}


void USER_Motor_Mspinit(void)
{
	GPIO_InitTypeDef GPIO_Init;
    /* Enable GPIO clock */
	FAN_GPIOE_CLK_ENABLE();

    /* GPIO pin configuration  */
 	GPIO_Init.Pin = FAN_PIN;
	GPIO_Init.Mode = FAN_MODE;
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(FAN_GPIO_PORT,&GPIO_Init);	
	
	HAL_GPIO_WritePin(FAN_GPIO_PORT,FAN_PIN, GPIO_PIN_SET);

    /* Enable GPIO clock */
	TS0_GPIOE_CLK_ENABLE();
	TS1_GPIOE_CLK_ENABLE();
	
    /* GPIO pin configuration  */
 	GPIO_Init.Pin = TS0_PIN;
	GPIO_Init.Mode = TS0_MODE;
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(TS0_GPIO_PORT,&GPIO_Init);
	HAL_GPIO_WritePin(TS0_GPIO_PORT,TS0_PIN, GPIO_PIN_SET);
	
	/* GPIO pin configuration  */
 	GPIO_Init.Pin = TS1_PIN;
	GPIO_Init.Mode = TS1_MODE;
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(TS1_GPIO_PORT,&GPIO_Init);
	HAL_GPIO_WritePin(TS1_GPIO_PORT,TS1_PIN, GPIO_PIN_SET);
}

int USER_Motor_hwinit(void)
{
USER_Motor_Mspinit();
return 0;
}

INIT_BOARD_EXPORT(USER_Motor_hwinit);



void TempSeltest(int argc,char **argv)
{
	if(strcmp(argv[1],"0")==0){
		HAL_GPIO_WritePin(TS0_GPIO_PORT,TS0_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TS1_GPIO_PORT,TS1_PIN,GPIO_PIN_RESET);
	}else if (strcmp(argv[1],"1")==0){
		HAL_GPIO_WritePin(TS0_GPIO_PORT,TS0_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(TS1_GPIO_PORT,TS1_PIN,GPIO_PIN_RESET);
	}else if(strcmp(argv[1],"2")==0) {
		HAL_GPIO_WritePin(TS0_GPIO_PORT,TS0_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TS1_GPIO_PORT,TS1_PIN,GPIO_PIN_SET);
	}else if(strcmp(argv[1],"3")==0) {
		HAL_GPIO_WritePin(TS0_GPIO_PORT,TS0_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(TS1_GPIO_PORT,TS1_PIN,GPIO_PIN_SET);
	}else{
		rt_kprintf("unknown parameter\r\n");
	}
}
MSH_CMD_EXPORT_ALIAS(TempSeltest,TS,TS 0 or TS 1 or TS 2 or TS 3);


void FANtest(int argc,char**argv)
{
	if (strcmp(argv[1], "on") == 0)
		HAL_GPIO_WritePin(FAN_GPIO_PORT,FAN_PIN, GPIO_PIN_RESET);
	else if(strcmp(argv[1], "off") == 0)
		HAL_GPIO_WritePin(FAN_GPIO_PORT,FAN_PIN, GPIO_PIN_SET);
	else
		rt_kprintf("unknown parameter\r\n");
}
MSH_CMD_EXPORT(FANtest,FANtest on or FANtest off);

void ADCtest(int argc,char**argv)
{
	rt_kprintf("temp= 0x%x \r\n", Global_User_ADC.Temp.TEMP_value);
	rt_kprintf("BUS V= 0x%x \r\n", Global_User_ADC.BUS.qV_value);
	rt_kprintf("BUS I= 0x%x \r\n", Global_User_ADC.BUS.qI_value);
	rt_kprintf("VOLTAGE C= 0x%x \r\n", Global_User_ADC.PhaseC.qV_value);
	rt_kprintf("VOLTAGE B= 0x%x \r\n", Global_User_ADC.PhaseB.qV_value);
	rt_kprintf("VOLTAGE A= 0x%x \r\n", Global_User_ADC.PhaseA.qV_value);
	rt_kprintf("current C2= 0x%x \r\n", Global_User_ADC.PhaseC.qI2_value);
	rt_kprintf("current C1= 0x%x \r\n", Global_User_ADC.PhaseC.qI1_value);
	rt_kprintf("current B2= 0x%x \r\n", Global_User_ADC.PhaseB.qI2_value);
	rt_kprintf("current B1= 0x%x \r\n", Global_User_ADC.PhaseB.qI1_value);
	rt_kprintf("current A2= 0x%x \r\n", Global_User_ADC.PhaseA.qI2_value);
	rt_kprintf("current A1= 0x%x \r\n", Global_User_ADC.PhaseA.qI1_value);

	
	rt_kprintf("temp offset= 0x%x \r\n", Global_User_ADC.Temp.TEMP_offset);
	rt_kprintf("BUS V  offset= 0x%x \r\n", Global_User_ADC.BUS.qV_offset);
	rt_kprintf("BUS I offset= 0x%x \r\n", Global_User_ADC.BUS.qI_offset);
	rt_kprintf("VOLTAGE C offset= 0x%x \r\n", Global_User_ADC.PhaseC.qV_offset);
	rt_kprintf("VOLTAGE B offset= 0x%x \r\n", Global_User_ADC.PhaseB.qV_offset);
	rt_kprintf("VOLTAGE A offset= 0x%x \r\n", Global_User_ADC.PhaseA.qV_offset);
	rt_kprintf("current C2 offset= 0x%x \r\n", Global_User_ADC.PhaseC.qI2_offset);
	rt_kprintf("current C1 offset= 0x%x \r\n", Global_User_ADC.PhaseC.qI1_offset);
	rt_kprintf("current B2 offset= 0x%x \r\n", Global_User_ADC.PhaseB.qI2_offset);
	rt_kprintf("current B1 offset= 0x%x \r\n", Global_User_ADC.PhaseB.qI1_offset);
	rt_kprintf("current A2 offset= 0x%x \r\n", Global_User_ADC.PhaseA.qI2_offset);
	rt_kprintf("current A1 offset= 0x%x \r\n", Global_User_ADC.PhaseA.qI1_offset);
}
MSH_CMD_EXPORT(ADCtest,ADCtest print ADCValue);


void PWMset(int argc,char **argv)
{
		SetDutyCycles(TIM8, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
}
MSH_CMD_EXPORT(PWMset,PWMset T1 T2 T3);


void PWMdis(int argc, char **argv)
{
	PWMOutputsDisable(TIM8);

}
MSH_CMD_EXPORT(PWMdis,PWMdisable);

void PWMen(int argc, char **argv)
{
	PWMOutputsEnable(TIM8);

}
MSH_CMD_EXPORT(PWMen,PWMenable);


void MCctrl(int argc,char**argv)
{
	if (strcmp(argv[1], "on") == 0)
	{
		State = INIT;
		rt_kprintf("MotorV0 is started\r\n");
	}
	else if(strcmp(argv[1], "off") == 0)
	{
		State = STOP;
		rt_kprintf("MotorV1 is stopped\r\n");
	}
}
MSH_CMD_EXPORT(MCctrl,MCctrl on or MCctrl off);

#ifdef ENCODER
void QEPtest(int argc,char**argv)
{
	rt_kprintf("TIM5 CNT =%x\r\n",GetCounterValue(TIM5) );
}
MSH_CMD_EXPORT(QEPtest,QEPtest);
#endif

int USER_MotorTask_init()
{
    rt_thread_t tid;
	
	tid = rt_thread_create("Motor",
                           USER_MotorTask_entry, RT_NULL,
                           1024, USER_MOTORTASK_PRIORITY, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(USER_MotorTask_init);
#endif
