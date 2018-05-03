/*=====================================================================================
 File name:        Template.c  (IQ version)
                    
 Originator:	DL.K

 Description:  The ...
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/Han_Endat/Han_Endat.h"


//	{
//		static uint32_t value,gpio_value;
//		static uint16_t sStartCount=0;
//
//		if(sStartCount<=8900)
//		{
//			// CLK
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			sStartCount++;
//		}
//		else if(sStartCount<=9000)
//		{
//			// CLK
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			sStartCount++;
//		}
//		else if(sStartCount<=13000)
//		{
//			// CLK
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			sStartCount++;
//		}
//		else if(sStartCount<=13050)
//		{
//			// CLK
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_31);
//			sStartCount++;
//		}
//		else
//		{
//			// CLK
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//
//			// After two Clk Pluse
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//
//			// Mod Cmd
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_31);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_31);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_31);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_31);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_31);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_31);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_31);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_31);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//	//		usDelay(5);???
//
//			// Read Pos
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value=0;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<0;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<1;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<2;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<3;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<4;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<5;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<6;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<7;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<8;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<9;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<10;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<11;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<12;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<13;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<14;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			gpio_value=GPIO_read(halHandle->gpioHandle,GPIO_Number_30);
//
//			GPIO_setLow(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//			value+=gpio_value<<15;
//			GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_18);
//			usDelay(1);
//		}
//
//	}
