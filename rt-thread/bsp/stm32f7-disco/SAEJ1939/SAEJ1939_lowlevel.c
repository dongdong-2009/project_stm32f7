
#include <rtthread.h>
#include "finsh.h"
#include <rtdevice.h>
#include "SAEJ1939.h"

unsigned char writeCan(J1939_msg_t j1939_msg)
{
	rt_device_t device;
	device = rt_device_find("can1");
	struct rt_can_msg tmpmsg;
	unsigned long tmp;
	
	if(device != NULL)
	{
		tmpmsg.data[7] = j1939_msg->Data[7];
		tmpmsg.data[6] = j1939_msg->Data[6];
		tmpmsg.data[5] = j1939_msg->Data[5];
		tmpmsg.data[4] = j1939_msg->Data[4];
		tmpmsg.data[3] = j1939_msg->Data[3];
		tmpmsg.data[2] = j1939_msg->Data[2];
		tmpmsg.data[1] = j1939_msg->Data[1];
		tmpmsg.data[0] = j1939_msg->Data[0];
		tmp = (unsigned long)(j1939_msg->Priority)<<26 			| 	\
				(unsigned long)(j1939_msg->edp)<<25				|  	\
				(unsigned long)(j1939_msg->DataPage)<<24				|  	\
				(unsigned long)(j1939_msg->PDUFormat)<<16		|  	\
				(unsigned long)(j1939_msg->PDUSpecific)<<8		|	\
				(unsigned long)(j1939_msg->SourceAddress);		
		tmpmsg.id = tmp;
		tmpmsg.ide = 1;
		tmpmsg.len = j1939_msg->DataLength;
		tmpmsg.rtr = 0;
		
		return rt_device_write(device, 0, &tmpmsg,1);
	}
	return RC_ERROR;
}

unsigned char readCan(J1939_msg_t j1939_msg)
{
	rt_device_t device;
	device = rt_device_find("can1");
	struct rt_can_msg tmpmsg;
	unsigned char i,size;
	unsigned char result;
	
	if(device != NULL)
	{
		size = rt_device_read(device,0,&tmpmsg,sizeof(struct rt_can_msg));
		if (size != 0)
		{
			j1939_msg->Priority = tmpmsg.id >> 26;
			j1939_msg->edp = tmpmsg.id >> 25;
			j1939_msg->DataPage = tmpmsg.id >> 24;
			j1939_msg->PDUFormat = tmpmsg.id >> 16;
			j1939_msg->PDUSpecific = tmpmsg.id >> 8;
			j1939_msg->SourceAddress = tmpmsg.id;
			j1939_msg->DataLength = tmpmsg.len;
			for(i=0;i<j1939_msg->DataLength;i++)
			{
				j1939_msg->Data[i] = tmpmsg.data[i];
			}
			return result = RC_SUCCESS;
		}
		else
			return result = RC_QUEUEEMPTY;
	}
	return RC_ERROR;
}

