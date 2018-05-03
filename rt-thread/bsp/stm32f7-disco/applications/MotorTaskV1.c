
#include <rtthread.h>
#ifdef USER_USING_MOTORBOARDV1

#include "finsh.h"
#include "stm32f7xx.h"
#include "board.h"
#include <stm32f7x_type.h>
#include <rtdevice.h>
#include "stm32f7xx_MClib.h"
#include "MC_Globals.h"


//! \Add By Dl.K
#include "modules/hal/hal_obj.h"
#include "modules/MotorCtrlApp/MotorCtrlApp.h"
#include "N25Q128.h"

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

#define NUM 15
int8_t a[NUM]={0};
int8_t gFlag=0;
//void Set(int32_t argc,int8_t **argv)
//{
//	int8_t i,j;
//	for(i = 0;i<NUM;i++)
//	{
//		a[i]=argv[1][i];
//	}
//	gFlag=1;
//	//rt_kprintf("a=%s\r\n", a);
//}
//MSH_CMD_EXPORT(Set,Set parameter);

void stm32_hw_Emergency_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Enable GPIO TX/RX clock */
    __GPIOI_CLK_ENABLE();

    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);	

}



void USER_MotorTask_entry(void *parameter)
{



	//! \Add By Dl.K
	Motor_Ctrl_Init();
 	stm32_hw_tim8_init();
	stm32_hw_ADC_init();
	stm32_hw_QEP_init();
	stm32_hw_CAP_init();
	stm32_hw_Emergency_init();

	//PWMOutputsEnable(TIM8);
	while(1)
	{
		#if 0
		if(Motor_Ctrl_State(a,&gFlag))
		{
			PWMOutputsEnable(TIM8);
		}
		else
		{
			PWMOutputsDisable(TIM8);
		}
		if(myscanf((char *)a) == RT_EOK)
		{
			gFlag = 1;
		}
		#endif
		if (HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6) == GPIO_PIN_RESET)
		{
			usart_rst();


		}
			/* idle 30usmax; online 40us max 20160822*/
		//rt_thread_delay(SLEEPTIME(20));
//		rt_kprintf("temp= 0x%x \r\n", Global_User_ADC.Temp.TEMP_value);
//		rt_kprintf("Ia= 0x%x \r\n", Global_User_ADC.PhaseA.qI_value);
//		rt_kprintf("Ib= 0x%x \r\n", Global_User_ADC.PhaseB.qI_value);
//		rt_kprintf("Ic= 0x%x \r\n", Global_User_ADC.PhaseC.qI_value);
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



void IOtest(int argc,char**argv)
{

	GPIO_InitTypeDef GPIO_Init;
	int i;
	__HAL_RCC_GPIOK_CLK_ENABLE();

	GPIO_Init.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_LOW;
	//GPIO_Init.Alternate = PWMWP_AF;
	HAL_GPIO_Init(GPIOK,&GPIO_Init);

	i = 10;
	while(i -- )
	{
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5, GPIO_PIN_RESET);
		rt_thread_delay(100);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOK, GPIO_PIN_4, GPIO_PIN_SET);
		rt_thread_delay(100);
	}

	
}
MSH_CMD_EXPORT(IOtest,IOtest);



void ADCtest(int argc,char**argv)
{
	rt_kprintf("motor temp= 0x%x \r\n", Global_User_ADC.Temp.TEMP_value);
	rt_kprintf("BUS V= 0x%x \r\n", Global_User_ADC.BUS.qV_value);
	rt_kprintf("temp= 0x%x \r\n", Global_User_ADC.BUS.qI_value);
	rt_kprintf("VOLTAGE C= 0x%x \r\n", Global_User_ADC.PhaseC.qV_value);
	rt_kprintf("VOLTAGE B= 0x%x \r\n", Global_User_ADC.PhaseB.qV_value);
	rt_kprintf("VOLTAGE A= 0x%x \r\n", Global_User_ADC.PhaseA.qV_value);
	rt_kprintf("current C2= 0x%x \r\n", Global_User_ADC.PhaseC.qI2_value);
	rt_kprintf("current C1= 0x%x \r\n", Global_User_ADC.PhaseC.qI1_value);
	rt_kprintf("current B2= 0x%x \r\n", Global_User_ADC.PhaseB.qI2_value);
	rt_kprintf("current B1= 0x%x \r\n", Global_User_ADC.PhaseB.qI1_value);
	rt_kprintf("current A2= 0x%x \r\n", Global_User_ADC.PhaseA.qI2_value);
	rt_kprintf("current A1= 0x%x \r\n", Global_User_ADC.PhaseA.qI1_value);

	
	rt_kprintf("motor temp offset= 0x%x \r\n", Global_User_ADC.Temp.TEMP_offset);
	rt_kprintf("BUS V  offset= 0x%x \r\n", Global_User_ADC.BUS.qV_offset);
	rt_kprintf("temp offset= 0x%x \r\n", Global_User_ADC.BUS.qI_offset);
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

#include<stdlib.h>
#include<stdio.h>


void out(double d)
{

}

void floatmul(int argc,char**argv)
{
	double d0,d1,result;
	d0 = atof(argv[1]);
	d1 = atof(argv[2]);
	result = mymul(d0,d1);
	result = d0*d1;
	out(d0);
	rt_kprintf("d0:\r\n%x = %x\r\n%x = %x\r\n",&d0,*(&d0),d0,d0);
	rt_kprintf("d1:\r\n%x = %x\r\n%x = %x\r\n",&d1,*(&d1),d1,d1);
}
MSH_CMD_EXPORT(floatmul,floatmul d1 d2);


void MCctrl(int argc,char**argv)
{
	if (strcmp(argv[1], "on") == 0)
	{
//		State = INIT;
		PWMOutputsEnable(TIM8);

		rt_kprintf("Motor is started\r\n");
	}
	else if(strcmp(argv[1], "off") == 0)
	{
//		State = STOP;
		PWMOutputsDisable(TIM8);
		rt_kprintf("Motor is stopped\r\n");
	}
}
MSH_CMD_EXPORT(MCctrl,MCctrl on or MCctrl off);

unsigned char sindata[13];
void in(int argc,char**argv)
{
	unsigned char i=0;
	rt_kprintf("get %d datas\r\n",argc-1);
	for (i=0;i<argc-1;i++)
	{
		
		sindata[i] = atoi(argv[i+1]);
		rt_kprintf("%d ",sindata[i]);
	}
	
	rt_kprintf("\r\n");
}
MSH_CMD_EXPORT(in,sin 13 byte data);


void flashtest(int argc,char**argv)
{
	int i;
	int addr = 0x80ffef0;
	int j;
	int z;
	if(strcmp(argv[1], "w") == 0 && strcmp(argv[2],"") != 0)
	{

		//HAL_FLASH_Unlock();
		
		//HAL_FLASH_Unlock();
		 HAL_FLASH_Unlock();
		FLASH_EraseInitTypeDef pEraseInit;
		pEraseInit.NbSectors = 1;
		pEraseInit.Sector = FLASH_SECTOR_7;
		pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		uint32_t SectorError;
		HAL_FLASHEx_Erase(&pEraseInit, &SectorError);

		i = atoi(argv[2]);
		rt_kprintf("write %x\r\n",i);
		while(addr < 0x80ffffc)
			{
				//z = 100;
				//while(*(int *)addr != i && z>1)
				//{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, i);
				//	z--;
				//	for(j = 1000;j>0;j--);
				//}

		
				rt_kprintf("addr:%x,flash: %x\r\n",addr,*(int *)addr);
				i++;
				addr = addr+4;
			}
		HAL_FLASH_Lock();
	}
	else if(strcmp(argv[1], "r") == 0)
	{

		while(addr < 0x80ffffc)
		{
		
				rt_kprintf("addr:%x,flash: %x\r\n",addr,*(int *)addr);
				addr = addr+4;
		}

	}
}
MSH_CMD_EXPORT(flashtest,flashtest wr word);




void qspitest(int argc,char**argv)
{

	//BSP_QSPI_Init();
	char i[1000] = {0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0x11,0x25};
	int addr = 0x0;
	static int result;
	static char j[1000];
	char k;
	static QSPI_Info pInfo;

	
			BSP_QSPI_Init();
			
			rt_kprintf("init\r\n");

			
	rt_kprintf("size:%x;sectorsize:%x;sectornum:%x\r\n",pInfo.FlashSize,pInfo.EraseSectorSize,pInfo.EraseSectorsNumber);
	for(k = 0;k<10;k++)
	{

		rt_kprintf("addr:%x,flash: %x\r\n",addr,j[k]);
	}
	rt_kprintf("result = %x", result);
	//if(strcmp(argv[1], "w"))
	{

		//HAL_FLASH_Unlock();
		
		BSP_QSPI_Erase_Chip();
		
		rt_kprintf("erase\r\n");
		
		result = BSP_QSPI_GetInfo(&pInfo);

			
		
		rt_kprintf("size:%x;sectorsize:%x;sectornum:%x\r\n",pInfo.FlashSize,pInfo.EraseSectorSize,pInfo.EraseSectorsNumber);
		
		rt_kprintf("write\r\n");
		
		result = BSP_QSPI_Write(&i[0], addr,10);
		
		

	  
	}
	//else if(strcmp(argv[1], "r") == 0)
	{
		
		//result = BSP_QSPI_Read(j,addr,1000);
		result = BSP_QSPI_Read(&j[0],addr,10);
	}
}
MSH_CMD_EXPORT(qspitest,qspitest);



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

