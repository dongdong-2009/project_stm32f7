

#include <rtthread.h>
#include "finsh.h"
#include "stm32f7xx.h"
#include "board.h"
#include <stm32f7x_type.h>
#include <rtdevice.h>

#define UART3TXFIFOLENGTH (3)
#define USER_Usart3rxTask_PRIORITY (9)
#define USER_Usart3txTask_PRIORITY (9)
#define RCVLENTH (14)
#define RXMQSIZE (3)

struct tx_msg
{
	char *data;
	rt_size_t size;
};
struct Txfifo
{
	struct tx_msg *txmsg;
	uint8_t index;
};
struct Txfifo *txfifo;


static rt_mq_t tx_mq;
static char ch;
static rt_sem_t rx_sem;

struct rx_msg
{
	char *data;
	char size;
};

static rt_mq_t rx_mq;
rt_err_t myprintf(char *buf, uint8_t size)
{
	struct tx_msg txmsg;
	rt_err_t result = RT_EOK;
	
	if(tx_mq == RT_NULL || txfifo == RT_NULL || buf == RT_NULL)
		return (result = RT_ERROR);
	txmsg.size = size;
	txmsg.data = buf;
	txfifo->txmsg[txfifo->index] = txmsg;
	result = rt_mq_send(tx_mq,&(txfifo->txmsg[txfifo->index]),sizeof(struct tx_msg));
	txfifo->index++;
	if(txfifo->index > UART3TXFIFOLENGTH)
	{
		txfifo->index = 0;
	}
	return result;
}

void USER_Usart3txTask_entry(void *parameter)
{
	
	rt_device_t device;
	device = rt_device_find("uart3");
	rt_err_t result = RT_EOK;
	
	struct tx_msg txmsg;
	if ( device != RT_NULL )
	{
		
		
		if (rt_device_open (device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
		{
			rt_kprintf("open error\r\n");
		}
	}
	
	while(1)
	{
		rt_mq_recv(tx_mq,&txmsg,sizeof(struct tx_msg),50);
		
		if (result == RT_EOK)
		{
			rt_device_write(device, 0,txmsg.data,txmsg.size);
		}
	}
	
}


int USER_Usart3txTask_init()
{
    rt_thread_t tid;

	
	if ((tx_mq = rt_mq_create("usart3tx", sizeof(struct tx_msg),UART3TXFIFOLENGTH,RT_IPC_FLAG_FIFO)) == RT_NULL)
	{
		rt_kprintf("sem create error\r\n");
	}
	
	txfifo = (struct Txfifo *)rt_malloc(sizeof(struct Txfifo)+UART3TXFIFOLENGTH*sizeof(struct tx_msg));
	if(txfifo != RT_NULL)
	{
		txfifo->txmsg = (struct tx_msg *)(txfifo+1);
		rt_memset(txfifo->txmsg,0,UART3TXFIFOLENGTH*sizeof(struct tx_msg));
		txfifo->index = 0;
	}
	
	tid = rt_thread_create("usart3tx",						
                           USER_Usart3txTask_entry, RT_NULL,
                           1024, USER_Usart3txTask_PRIORITY, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(USER_Usart3txTask_init);


rt_err_t myscanf(char *buf)
{
	rt_err_t result = RT_EOK;
	char i;
	struct rx_msg rx_msg;
	
	if(rx_mq == RT_NULL || buf ==  RT_NULL)
		return (result = RT_ERROR);
	result = rt_mq_recv(rx_mq,&rx_msg,sizeof(struct rx_msg),50);
	if(result == RT_EOK)
	{
		for(i = 0;i<rx_msg.size;i++)
			buf[i] = rx_msg.data[i];
		buf[i] = '\0';
	}
	return result;
}
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(rx_sem);
	return RT_EOK;
}

void USER_Usart3rxTask_entry(void *parameter)
{

	rt_device_t device;
	rt_err_t result = RT_EOK;
	static char i = 0;
	char rcvdata[RCVLENTH];
	struct rx_msg rx_msg;
	
	
	device = rt_device_find("uart3");
	
	if ( device != RT_NULL )
	{
		rt_device_set_rx_indicate(device, uart_input);
		if (rt_device_open (device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX ) != RT_EOK)
		{
			rt_kprintf("open error\r\n");
		}
	}
	while(1)
	{
		//struct rx_msg msg;
		//rt_device_write(device, 0,buf,130);
		result = rt_sem_take(rx_sem, RT_WAITING_FOREVER);
		if (result == RT_EOK)
		{
			rt_uint32_t rx_length;
			rx_length = rt_device_read(device, 0, &ch,1);
			if(rx_length != 0)
			{
				rcvdata[i++] = ch;
				if(ch == 0x0d)
				{
					rx_msg.size = i;
					i = 0;
					rx_msg.data = rcvdata;
					rt_mq_send(rx_mq,&rx_msg,sizeof(struct rx_msg));
				}
				
				if(i >= RCVLENTH)
					i = 0;
			}
		}
	}

}





int USER_Usart3rxTask_init()
{
    rt_thread_t tid;
	rt_device_t device;

	if ((rx_sem = rt_sem_create("usart3rcv", 0, RT_IPC_FLAG_FIFO)) == RT_NULL)
	{
		rt_kprintf("sem create error\r\n");
	}
	if ((rx_mq = rt_mq_create("usart3rxmq", sizeof(struct rx_msg),RXMQSIZE,RT_IPC_FLAG_FIFO)) == RT_NULL)
	{
		rt_kprintf("rx_mq create error\r\n");
	}
	
	tid = rt_thread_create("usart3rx",						
                           USER_Usart3rxTask_entry, RT_NULL,
                           1024, USER_Usart3rxTask_PRIORITY, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(USER_Usart3rxTask_init);

