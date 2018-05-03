/*
*
*/
#include <rtthread.h>
#include "stm32f7xx.h"
#include "Drv_Can.h"
#include "finsh.h"
#include <rtdevice.h>

#define CAN1_CLK_ENABLE()           __HAL_RCC_CAN1_CLK_ENABLE()
#define CAN1_GPIOB_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define CAN1_TX_PORT GPIOB
#define CAN1_RX_PORT GPIOB
#define CAN1_TX_PIN GPIO_PIN_9
#define CAN1_RX_PIN GPIO_PIN_8

struct rt_can_device canDevice1;
stm32_canTypeDef can1;

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	//rt_kprintf("tx complete\r\n");
}

void CAN1_TX_IRQHandler(void)
{
	CAN_HandleTypeDef hcan;
	/* enter interrupt */
    rt_interrupt_enter();

	hcan.Instance = CAN1;
	HAL_CAN_IRQHandler(&hcan);

    /* leave interrupt */
    rt_interrupt_leave();

}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	char i;
	#if 0
	rt_kprintf("rx data: ");
	for(i = 0; i < hcan->pRxMsg->DLC; i++)
	{
		rt_kprintf("0x%x ", hcan->pRxMsg->Data[i]);
	}
	rt_kprintf("\r\n");
	#endif
}

void CAN1_RX0_IRQHandler(void)
{
	
	CAN_HandleTypeDef hcan;
	/* enter interrupt */
	
    rt_interrupt_enter();
	hcan.Instance = CAN1;
	
	HAL_CAN_IRQHandler(&hcan);

    /* leave interrupt */
    rt_interrupt_leave();

}

void CAN1_RX1_IRQHandler(void)
{
	
	CAN_HandleTypeDef hcan;
	/* enter interrupt */
	rt_interrupt_enter();

	hcan.Instance = CAN1;
	HAL_CAN_IRQHandler(&hcan);

	/* leave interrupt */
	rt_interrupt_leave();

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	
	if((hcan->ErrorCode & HAL_CAN_ERROR_EWG) == HAL_CAN_ERROR_EWG)
		rt_kprintf("ewg error\r\n");
	if ((hcan->ErrorCode & HAL_CAN_ERROR_EPV) == HAL_CAN_ERROR_EPV)
		rt_kprintf("epv error\r\n");
	if ((hcan->ErrorCode & HAL_CAN_ERROR_BOF) == HAL_CAN_ERROR_BOF)
		rt_kprintf("bof error\r\n");
	if ((hcan->ErrorCode & HAL_CAN_ERROR_STF) == HAL_CAN_ERROR_STF)
		rt_kprintf("stf error\r\n");
	if ((hcan->ErrorCode & HAL_CAN_ERROR_FOR) == HAL_CAN_ERROR_FOR)
		rt_kprintf("for error\r\n");
	if ((hcan->ErrorCode & HAL_CAN_ERROR_ACK) == HAL_CAN_ERROR_ACK)
		rt_kprintf("ack error\r\n");
	if ((hcan->ErrorCode & HAL_CAN_ERROR_BR) == HAL_CAN_ERROR_BR)
		rt_kprintf("br error\r\n");
	if ((hcan->ErrorCode & HAL_CAN_ERROR_BD) == HAL_CAN_ERROR_BD)
		rt_kprintf("bd error\r\n");
	if ((hcan->ErrorCode & HAL_CAN_ERROR_CRC) == HAL_CAN_ERROR_CRC)
		rt_kprintf("crc error\r\n");
	
	//HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
}

void CAN1_SCE_IRQHandler(void)
{
	CAN_HandleTypeDef hcan;

	/* enter interrupt */
	rt_interrupt_enter();

	HAL_CAN_IRQHandler(&hcan);

	/* leave interrupt */
	rt_interrupt_leave();

}



void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
	
	CAN1_CLK_ENABLE();
	CAN1_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pin = CAN1_TX_PIN;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(CAN1_TX_PORT, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Pin = CAN1_RX_PIN;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(CAN1_RX_PORT, &GPIO_InitStruct);	
	
	HAL_NVIC_SetPriority(CAN1_TX_IRQn,CAN1_TX_IRQ_PREEMPT,CAN1_TX_TRQ_SUB);
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
	
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn,CAN1_RX0_IRQ_PREEMPT,CAN1_RX0_TRQ_SUB);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

	HAL_NVIC_SetPriority(CAN1_RX1_IRQn,CAN1_RX1_IRQ_PREEMPT,CAN1_RX1_TRQ_SUB);
	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

	HAL_NVIC_SetPriority(CAN1_SCE_IRQn,CAN1_SCE_IRQ_PREEMPT,CAN1_SCE_IRQ_SUB);
	HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}




rt_err_t stm32_CANcontrol(struct rt_can_device *can, int cmd, void *arg)
{
	stm32_canTypeDef *stcan;
	stcan = (stm32_canTypeDef *)can->parent.user_data;
	rt_err_t result = RT_EOK;
	switch(cmd)
	{
		case (RT_DEVICE_CTRL_CLR_INT):
			__HAL_CAN_DISABLE_IT(&(stcan->hcan), CAN_IT_ERR);
			break;
		case (RT_DEVICE_CTRL_SET_INT):
			__HAL_CAN_ENABLE_IT(&(stcan->hcan), CAN_IT_ERR);
			break;
		default:
			break;
	}	
	return result;
}

int stm32_CANsendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
	stm32_canTypeDef *stcan;
	struct rt_can_msg *tmpmsg = (struct rt_can_msg *)buf;
	stcan = (stm32_canTypeDef *)can->parent.user_data;
	rt_err_t result = RT_EOK;
	unsigned char i;
	unsigned char ecode;
	if((stcan->hcan.pTxMsg != NULL))
	{
		//rt_kprintf(" TX malloc success\r\n");
		stcan->hcan.pTxMsg->ExtId =  tmpmsg->id;
		if(tmpmsg->ide)
			stcan->hcan.pTxMsg->IDE = CAN_ID_EXT;
		else
			stcan->hcan.pTxMsg->IDE = CAN_ID_STD;
		if(tmpmsg->rtr)
			stcan->hcan.pTxMsg->RTR = CAN_RTR_REMOTE;
		else 
			stcan->hcan.pTxMsg->RTR = CAN_RTR_DATA;
		stcan->hcan.pTxMsg->DLC = tmpmsg->len;
		for(i=0;i<8;i++)
		{
			stcan->hcan.pTxMsg->Data[i] = tmpmsg->data[i];
		}
		if ((ecode = HAL_CAN_Transmit(&(stcan->hcan),100)) == HAL_OK)
		
		//if (HAL_CAN_Transmit_IT(&hcan) == HAL_OK)1600958092213
		{
			//rt_kprintf("CAN tx success %x\r\n", stcan->hcan.Instance->ESR);
		}
		else
		{
			rt_kprintf("CAN tx fail! %x\r\n", ecode);
		}
		//rt_free(hcan.pTxMsg);
	}
	return result;
}
int stm32_CANrecvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno)
{
	stm32_canTypeDef *stcan;
	stcan = (stm32_canTypeDef *)can->parent.user_data;
	rt_err_t result = RT_EOK;

	while((uint8_t)(stcan->hcan.Instance->RF0R&(uint32_t)0x03))
	{
		if (HAL_CAN_Receive(&(stcan->hcan),CAN_FIFO0,0) == HAL_OK)
		{
			rt_kprintf("receive can\r\n");
			struct rt_can_msg tmpmsg;
			struct rt_can_rx_fifo *rx_fifo;
			struct rt_can_msg_list *listmsg = RT_NULL;
			rt_base_t level;
			#if 0
			{
				char i;
				rt_kprintf("rx data: ");
				for(i = 0; i < stcan->hcan.pRxMsg->DLC; i++)
				{
					rt_kprintf("0x%x ", stcan->hcan.pRxMsg->Data[i]);
				}
				rt_kprintf("\r\n");
			}
			#endif
			rx_fifo = (struct rt_can_rx_fifo *)can->can_rx;
			RT_ASSERT(rx_fifo != RT_NULL);
			/* interrupt mode receive */
			//RT_ASSERT(can->parent.open_flag & RT_DEVICE_FLAG_INT_RX);
	
			//no = event >> 8;
			//ch = can->ops->recvmsg(can, &tmpmsg, no);
			//if (ch == -1) break;
	
			tmpmsg.id = stcan->hcan.pRxMsg->ExtId;
			tmpmsg.ide = stcan->hcan.pRxMsg->IDE;
			tmpmsg.rtr = stcan->hcan.pRxMsg->RTR;
			tmpmsg.len = stcan->hcan.pRxMsg->DLC;
			tmpmsg.data[0] = stcan->hcan.pRxMsg->Data[0];
			tmpmsg.data[1] = stcan->hcan.pRxMsg->Data[1];
			tmpmsg.data[2] = stcan->hcan.pRxMsg->Data[2];
			tmpmsg.data[3] = stcan->hcan.pRxMsg->Data[3];
			tmpmsg.data[4] = stcan->hcan.pRxMsg->Data[4];
			tmpmsg.data[5] = stcan->hcan.pRxMsg->Data[5];
			tmpmsg.data[6] = stcan->hcan.pRxMsg->Data[6];
			tmpmsg.data[7] = stcan->hcan.pRxMsg->Data[7];

			/* disable interrupt */
			level = rt_hw_interrupt_disable();
			can->status.rcvpkg++;
			can->status.rcvchange = 1;
			if (!rt_list_isempty(&rx_fifo->freelist))
			{
				listmsg = rt_list_entry(rx_fifo->freelist.next, struct rt_can_msg_list, list);
				rt_list_remove(&listmsg->list);
				RT_ASSERT(rx_fifo->freenumbers > 0);
				rx_fifo->freenumbers--;
			}
			else if (!rt_list_isempty(&rx_fifo->uselist))
			{
				listmsg = rt_list_entry(rx_fifo->uselist.next, struct rt_can_msg_list, list);
				can->status.dropedrcvpkg++;
				rt_list_remove(&listmsg->list);
			}
			/* enable interrupt */
			rt_hw_interrupt_enable(level);
	
			if (listmsg != RT_NULL)
			{
				rt_memcpy(&listmsg->data, &tmpmsg, sizeof(struct rt_can_msg));
				level = rt_hw_interrupt_disable();
				rt_list_insert_before(&rx_fifo->uselist, &listmsg->list);
				rt_hw_interrupt_enable(level);
			}
			
			if (can->parent.rx_indicate != RT_NULL)
            {
                rt_size_t rx_length;

                level = rt_hw_interrupt_disable();
                /* get rx length */
                rx_length = rx_fifo->freenumbers * sizeof(struct rt_can_msg);
                rt_hw_interrupt_enable(level);

                can->parent.rx_indicate(&can->parent, rx_length);
            }

		}
	}
	return result;
}

static void timerCanrxfunc(void* parameter)
{
	stm32_CANrecvmsg(&canDevice1,NULL,0);
}



rt_err_t stm32_CANconfigure(struct rt_can_device *can, struct can_configure *cfg)
{
	stm32_canTypeDef *stcan;
	stcan = (stm32_canTypeDef *)can->parent.user_data;
	rt_err_t result = RT_EOK;
	static rt_timer_t timerCanrx;
	
	switch (cfg->baud_rate)
	{
		case (250) :
			if(HAL_CAN_DeInit(&(stcan->hcan))!= HAL_OK)
			{
				rt_kprintf("error deinit can\r\n");
				return RT_ERROR;
			}
			stcan->hcan.Init.ABOM = DISABLE;
			stcan->hcan.Init.AWUM = DISABLE;
			#if 1
			stcan->hcan.Init.BS1 = CAN_BS1_8TQ;
			stcan->hcan.Init.BS2 = CAN_BS2_3TQ;
			stcan->hcan.Init.Prescaler = 16;
			#endif
			stcan->hcan.Init.Mode = CAN_MODE_NORMAL;
			stcan->hcan.Init.NART = DISABLE;
			stcan->hcan.Init.RFLM = DISABLE;
			stcan->hcan.Init.SJW = CAN_SJW_2TQ;
			stcan->hcan.Init.TTCM = DISABLE;
			stcan->hcan.Init.TXFP = DISABLE;
			
			if(HAL_CAN_Init(&(stcan->hcan))!= HAL_OK)
			{
				rt_kprintf("error init can\r\n");
				return RT_ERROR;
			}	
			
			CAN_FilterConfTypeDef sFilterConfig;
			sFilterConfig.BankNumber = 0;
			sFilterConfig.FilterActivation = ENABLE;
			sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
			sFilterConfig.FilterIdHigh = 0x0000;
			sFilterConfig.FilterIdLow = 0x0000;
			sFilterConfig.FilterMaskIdHigh = 0x0000;
			sFilterConfig.FilterMaskIdLow = 0x0000;
			sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
			sFilterConfig.FilterNumber = 0;
			sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
			HAL_CAN_ConfigFilter(&(stcan->hcan),&sFilterConfig);
			stcan->hcan.pTxMsg = (CanTxMsgTypeDef*)rt_malloc(sizeof(CanTxMsgTypeDef));
			stcan->hcan.pRxMsg = (CanRxMsgTypeDef*)rt_malloc(sizeof(CanRxMsgTypeDef));
			if(stcan->hcan.pTxMsg == NULL || stcan->hcan.pRxMsg == NULL)
			{
				rt_kprintf("malloc fail\r\n");
				return RT_ERROR;
			}
			#if 1
			timerCanrx = rt_timer_create("timerCanrx",timerCanrxfunc,RT_NULL,20,RT_TIMER_FLAG_PERIODIC);
			if (timerCanrx != RT_NULL) rt_timer_start(timerCanrx);
			#endif
			#if 0
			__HAL_CAN_ENABLE_IT(&stcan->hcan, CAN_FLAG_FF0);
			__HAL_CAN_ENABLE_IT(&stcan->hcan, CAN_FLAG_FF1);
			
			//stcan->hcan.pRxMsg = &canrxbuf;
			HAL_CAN_Receive_IT(&stcan->hcan,CAN_FIFO0);
			HAL_CAN_Receive_IT(&stcan->hcan,CAN_FIFO1);
			#endif
			/* Enable Error Interrupt */
			break;
		default:
			return RT_ERROR;
			break;
	}
	return result;
}

static const struct rt_can_ops stm32_can_ops =
{    
	stm32_CANconfigure,    
	stm32_CANcontrol,    
	stm32_CANsendmsg,    
	stm32_CANrecvmsg,    
};

int Can_hw_init(void)
{
	stm32_canTypeDef *can;
    struct can_configure config = CAN1_CONFIG;
	
	canDevice1.config = config;

	can = &can1;
	can->hcan.Instance = CAN1;

	rt_hw_can_register(&canDevice1,"can1",&stm32_can_ops,can);
	#if 0
	CAN_HandleTypeDef hcan;
	hcan.Instance = CAN1;
	if(HAL_CAN_DeInit(&hcan)!= HAL_OK)
	{
		rt_kprintf("error deinit can\r\n");
		return 0;
	}
	hcan.Init.ABOM = DISABLE;
	hcan.Init.AWUM = DISABLE;
	hcan.Init.BS1 = CAN_BS1_8TQ;
	hcan.Init.BS2 = CAN_BS2_3TQ;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.NART = DISABLE;
	hcan.Init.Prescaler = 16;
	hcan.Init.RFLM = ENABLE;
	hcan.Init.SJW = CAN_SJW_2TQ;
	hcan.Init.TTCM = DISABLE;
	hcan.Init.TXFP = DISABLE;
	
	if(HAL_CAN_Init(&hcan)!= HAL_OK)
	{
		rt_kprintf("error init can\r\n");
		return;
	}	

	CAN_FilterConfTypeDef sFilterConfig;
	sFilterConfig.BankNumber = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan,&sFilterConfig);

	
	//__HAL_CAN_ENABLE_IT(&hcan, CAN_FLAG_FF0);
	//__HAL_CAN_ENABLE_IT(&hcan, CAN_FLAG_FF1);
	
	//hcan.pRxMsg = &canrxbuf;
	//HAL_CAN_Receive_IT(&hcan,CAN_FIFO0);
	//HAL_CAN_Receive_IT(&hcan,CAN_FIFO1);
	
	/* Enable Error Interrupt */
	__HAL_CAN_ENABLE_IT(&hcan, CAN_IT_ERR);
	#endif
	return 0;
	
}
INIT_BOARD_EXPORT(Can_hw_init);
