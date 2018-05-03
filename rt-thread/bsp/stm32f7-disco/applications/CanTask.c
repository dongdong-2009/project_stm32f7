#include <rtthread.h>
#ifdef RT_USING_J1939
#include "finsh.h"
#include "stm32f7xx.h"
#include "board.h"
#include <stm32f7x_type.h>
#include <rtdevice.h>
#include "SAEJ1939.h"
#include "stdlib.h"

#define USER_CANTASK_PRIORITY (9)
#define J1939_RCVTASK_PRIORITY (10)
#define J1939_TXDTASK_PRIORITY (10)


#define DEFAULTADDRCA1 (0x23);
#define SENDSIZE (13)
#define CA1 {		\
		0x47,		\
		0x45,		\
		0x2,		\
		0x15,		\
		0x46,		\
		0x0,		\
		0x17,		\
		0x5,		\
		0x3,		\
		0x0		\
}

//static CanTxMsgTypeDef cantxbuf;
//static CanRxMsgTypeDef canrxbuf;
CA_Msg_t ca_msg;
unsigned char *data;

#if 1

void J1939_TxdTask_entry(void *parameter)
{
    rt_thread_t tid;
	
	//ca_msg.CAname = &caname;
	//ca_msg.msg.SourceAddress = DEFAULTADDRCA1;
	
	if(ca_msg != NULL)
	{
		if (sendMessage(ca_msg,data)== RC_ERROR)
			rt_kprintf("rc_error\r\n");
	}
	tid = rt_thread_self();
	rt_thread_delete(tid);

}

void CANtx(int argc,char**argv)
{
	#if 0
	CAN_HandleTypeDef hcan;
	char i;
	hcan.Instance  = CAN1;
	hcan.pTxMsg = (CanTxMsgTypeDef *)rt_malloc(sizeof(CanTxMsgTypeDef));
	//hcan.pTxMsg = &cantxbuf;
	
	if((hcan.pTxMsg != NULL))
	{
		rt_kprintf(" TX malloc success\r\n");
		hcan.pTxMsg->ExtId =  0x15555555;
		hcan.pTxMsg->IDE = CAN_ID_EXT;
		hcan.pTxMsg->RTR = CAN_RTR_DATA;
		hcan.pTxMsg->DLC = 8;
		for(i=0;i<8;i++)
		{
			hcan.pTxMsg->Data[i] = 0x55;
		}
		if (HAL_CAN_Transmit(&hcan,10) == HAL_OK)
		
		//if (HAL_CAN_Transmit_IT(&hcan) == HAL_OK)1600958092213
		{
			rt_kprintf("CAN tx success %x\r\n", hcan.Instance->ESR);
		}
		else
		{
			rt_kprintf("CAN tx fail!\r\n");
		}
		//rt_free(hcan.pTxMsg);
	}
	#endif
	#if 0
	rt_device_t device;
	struct rt_can_msg tmpmsg;
	
	tmpmsg.data[7] = 0x55;
	tmpmsg.data[6] = 0x55;
	tmpmsg.data[5] = 0x55;
	tmpmsg.data[4] = 0x55;
	tmpmsg.data[3] = 0x55;
	tmpmsg.data[2] = 0x55;
	tmpmsg.data[1] = 0x55;
	tmpmsg.data[0] = 0x55;
	tmpmsg.id = 0x15555555;
	tmpmsg.ide = 1;
	tmpmsg.len = 8;
	tmpmsg.rtr = 0;
	
	device = rt_device_find("can1");
	if(device != RT_NULL)
		rt_device_write(device, 0, &tmpmsg,1);
	#endif
	#if 0
	unsigned char data[8];
	unsigned char i;
	struct CA_Msg CA_msg;
	CA_msg.sendbytes = 8;
	CA_msg.msg.DataPage = 0;
	CA_msg.msg.edp = 0;
	CA_msg.msg.PDUFormat = 0x35;
	CA_msg.msg.PDUSpecific = 0x46;
	CA_msg.msg.Priority = 0x05;
	for(i=0;i<8;i++)
	{
		data[i] = 0x55;
	}
	sendMessage(&CA_msg,data);
	#endif
    rt_thread_t tid;
	unsigned int sendsize = atoi(argv[1]);
	unsigned int i;
	if(data != NULL)
		data = (unsigned char *)malloc(sendsize * sizeof(char));

	ca_msg->sendbytes = sendsize;
	ca_msg->msg.DataPage = 0;
	ca_msg->msg.edp =0;
	ca_msg->msg.PDUFormat = atoi(argv[2]);
	ca_msg->msg.PDUSpecific = atoi(argv[3]);
	ca_msg->msg.Priority = 6;
	
	for(i=0;i<sendsize;i++)
		data[i] = i+1;

	tid = rt_thread_create("J1939txd",						
                          J1939_TxdTask_entry, RT_NULL,
                           1024, J1939_TXDTASK_PRIORITY, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);
	if(data != NULL)
	{
		free(data);
		data = NULL;
	}
}
MSH_CMD_EXPORT(CANtx,CANtx sendsize pf ps);
#endif



void USER_CanTask_entry(void *parameter)
{
	#if 0
	rt_device_t device;
	struct rt_can_msg tmpmsg;
	rt_size_t size;
	
	device = rt_device_find("can1");
	
	if(device != RT_NULL)
		if (rt_device_open (device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
		{
			rt_kprintf("open error\r\n");
		}
	while(1)
	{
		size = rt_device_read(device,0,&tmpmsg,sizeof(struct rt_can_msg));
		if (size != 0)
		{
		
			char i;
			rt_kprintf("rx data: ");
			for(i = 0; i < tmpmsg.len; i++)
			{
				rt_kprintf("0x%x ", tmpmsg.data[i]);
			}
			rt_kprintf("\r\n");
		}
		rt_thread_delay(20);
	}
	#endif
	struct J1939_CAname caname = CA1;
	
	rt_device_t device;
	rt_size_t size;
	
    osEvent event;
	struct rx_Message *rxmsg;
	
	device = rt_device_find("can1");
	
	if(device != RT_NULL)
		if (rt_device_open (device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
		{
			rt_kprintf("open error\r\n");
		}
	
	initJ1939();
	ca_msg = (CA_Msg_t)malloc(sizeof(struct CA_Msg));
	if(ca_msg != NULL)
	{
		
		memset(ca_msg,0,sizeof(struct CA_Msg));
		ca_msg->CAname = &caname;
		ca_msg->msg.SourceAddress = DEFAULTADDRCA1;

		J1939_CA_register(ca_msg);
	}
	while(1)
	{
		unsigned char i;
		#if 0
		for(i=0;i<SENDSIZE;i++)
			data[i] = i;
		ca_msg->sendbytes = SENDSIZE;
		ca_msg->msg.DataPage = 0;
		ca_msg->msg.edp =0;
		ca_msg->msg.PDUFormat = 220;
		ca_msg->msg.PDUSpecific = 0x23;
		ca_msg->msg.Priority = 6;
		if (sendMessage(ca_msg,data)== RC_ERROR)
			rt_kprintf("rc_error\r\n");
		#endif
		event = osMessageGet(ca_msg->rxq, 20);
		if(event.status == osEventMessage)
		{
			unsigned short i;

			rxmsg = event.value.p;
			if(rxmsg == NULL)
			{
				rt_kprintf("val null\r\n");
			}
			else
			{
				rt_kprintf(" dp: %x\r\n", rxmsg->dp);
				rt_kprintf("pf: %x \r\n", rxmsg->PF);
				rt_kprintf("ps: %x \r\n", rxmsg->PS);
				rt_kprintf("rcv data: ");
				for(i = 0;i<rxmsg->size;i++)
				{
					rt_kprintf("%x ", rxmsg->data[i]);
				}
				rt_kprintf("\r\n");
			}
		}
		//rt_thread_delay(20);
	}
}


int USER_CanTask_init()
{
    rt_thread_t tid;
	
	tid = rt_thread_create("Can",						
                           USER_CanTask_entry, RT_NULL,
                           2048, USER_CANTASK_PRIORITY, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(USER_CanTask_init);

static rt_sem_t can_rx_sem;

static rt_err_t can_input(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(can_rx_sem);
	return RT_EOK;
}


void J1939_RcvTask_entry(void *parameter)
{
	rt_device_t device;
	rt_err_t result;
	device = rt_device_find("can1");
	
	if ( device != RT_NULL )
	{
		rt_device_set_rx_indicate(device, can_input);
		#if 0
		if (rt_device_open (device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX ) != RT_EOK)
		{
			rt_kprintf("open error\r\n");
		}
		#endif
	}

	while(1)
	{
		result = rt_sem_take(can_rx_sem, RT_WAITING_FOREVER);
		if (result == RT_EOK)
		{
			receiveMessage();
		}
	}
}

int J1939_RcvTask_init()
{
    rt_thread_t tid;

	
	if ((can_rx_sem = rt_sem_create("J1939rcv", 0, RT_IPC_FLAG_FIFO)) == RT_NULL)
	{
		rt_kprintf("can_rx_sem create error\r\n");
	}
	
	tid = rt_thread_create("J1939Rcv",						
                          J1939_RcvTask_entry, RT_NULL,
                           1024, J1939_RCVTASK_PRIORITY, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(J1939_RcvTask_init);

#endif
