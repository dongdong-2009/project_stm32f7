/*
** only support one ca
*/
#include "SAEJ1939.h"
//#include "jsutils.h"



struct CA_list calist;


#define Q_MAX_MSG 10;


char inttoch(int val) {
  if (val<10) return (char)('0'+val);
  return (char)('A'+val-10);
}

void inttostring(int vals,char *str,unsigned int base)
{
	unsigned int val;
	if (vals<0) {
	  *(str++)='-';
	  val = (unsigned int)(-vals);
	} else {
	  val = (unsigned int)vals;
	}
	unsigned int d = 1;
	while (d*base <= val) d*=base;
	while (d > 1) {
	  unsigned int v = (unsigned int)(val / d);
	  val -= v*d;
	  *(str++) = inttoch((int)v);
	  d /= base;
	}
	*(str++)=inttoch((int)val);
	*(str++)=0;

}

unsigned char J1939_CA_register(CA_Msg_t ca_msg)
{
	unsigned char result = RC_SUCCESS;
	struct CA_node * canode;
	struct os_messageQ_def queue_def;
	char str1[5];
	
	if((CA_FindAddress(&calist,ca_msg)) != 254)
		return RC_ERROR;
	
	inttostring(ca_msg->CAname->FunInstance,str1,10);
	strcat(str1,"tx");
	queue_def.name = str1;
	rt_kprintf("%s\r\n",queue_def.name );
	queue_def.msg_size = sizeof(struct q_Message);
	queue_def.max_msgs = Q_MAX_MSG;
	queue_def.flag = IPC_FLAG_FIFO;
	ca_msg->txq = osMessageCreate(&queue_def,NULL);
	
	inttostring(ca_msg->CAname->FunInstance,str1,10);
	strcat(str1,"rx");
	queue_def.name = str1;
	queue_def.msg_size = sizeof(struct rx_Message);
	queue_def.max_msgs = Q_MAX_MSG;
	queue_def.flag = IPC_FLAG_FIFO;
	ca_msg->rxq = osMessageCreate(&queue_def,NULL);

	canode = (struct CA_node * )malloc(sizeof(struct CA_node));
	canode->ca_msg = ca_msg;
	j1939_list_insert_before(&(calist.head),&(canode->node));

	initAddr(ca_msg);
	return result;
}

unsigned char sendData(CA_Msg_t CA_msg, unsigned char *data)
{
	unsigned char result = RC_SUCCESS;
	osEvent event;
	struct J1939_msg j1939_msg;
	unsigned short i;
	q_Message_t qmsg;

	j1939_msg.DataPage = 0;
	j1939_msg.edp = 0;
	j1939_msg.PDUFormat = J1939_PF_DT;
	
	#if 0
	if(CA_msg->msg.PDUFormat < 240)
		j1939_msg.PDUSpecific = CA_msg->msg.PDUSpecific;
	else
		j1939_msg.PDUSpecific = J1939_GLOBAL_ADDRESS;
	#endif
	
	j1939_msg.PDUSpecific = CA_msg->msg.PDUSpecific;
	j1939_msg.DataLength = 8;
	j1939_msg.Priority = 7;
	j1939_msg.SourceAddress = CA_msg->msg.SourceAddress;
sendmsg:
	for(i = CA_msg->cm.nextnum; i < (CA_msg->cm.sendnum + CA_msg->cm.nextnum);i++)
	{
		unsigned short j;
		j1939_msg.Data[0] = i;
		for(j = 1; j<8;j++)
		{
			unsigned short k;
			k = (i-1)*7+j-1;
			if(k < (CA_msg->sendbytes+3))
				j1939_msg.Data[j] = data[k];
			else
				j1939_msg.Data[j] = 0xff;
		}
		
		if(writeCan(&j1939_msg)!= RC_SUCCESS)
		{
			return result = RC_CANNOTTRANSMIT;
		}
	}
	if(i > CA_msg->cm.sendpackets)
	{
		CA_msg->cm.sendcomp = 1;
		//rt_kprintf("sendData success\r\n");
		return RC_SUCCESS;
	}
	CA_msg->cm.t3.value = T3TIMEOUT;
	CA_msg->cm.t3.en = 1;
	CA_msg->cm.t3.release = 0;
	event = osMessageGet(CA_msg->txq,WAITFOREVER);
	qmsg = event.value.p;
	if(qmsg->timeout == 0)
	{
	
		CA_msg->cm.nextnum = qmsg->nextnum;
		CA_msg->cm.sendnum = qmsg->sendnum;
		if(CA_msg->cm.nextnum != 0)
			goto sendmsg;
		else
			return result = RC_SUCCESS;
	}
	else
		return result = RC_TIMEOUT;
	
}


unsigned char sendBAM(CA_Msg_t CA_msg, unsigned char *data)
{
	
	unsigned char result = RC_SUCCESS;
	struct J1939_msg j1939_msg;
	unsigned char i;

	j1939_msg.Data[0] = J1939_BAM_CONTROL_BYTE;
	j1939_msg.Data[1] = (CA_msg->sendbytes+3+CA_msg->cm.sendpackets)%256;
	j1939_msg.Data[2] = (CA_msg->sendbytes+3+CA_msg->cm.sendpackets)/256;
	j1939_msg.Data[3] = CA_msg->cm.sendpackets;
	j1939_msg.Data[4] = 0xff;     /*don't support maxium number of packets*/
	j1939_msg.Data[5] = CA_msg->msg.PDUSpecific;
	j1939_msg.Data[6] = CA_msg->msg.PDUFormat;
	j1939_msg.Data[7] = CA_msg->msg.DataPage;
	j1939_msg.DataLength = 8;
	j1939_msg.DataPage = 0;
	j1939_msg.edp = 0;
	j1939_msg.PDUFormat = J1939_PF_TP_CM;
	j1939_msg.PDUSpecific = J1939_GLOBAL_ADDRESS;
	j1939_msg.Priority = 7;
	j1939_msg.SourceAddress = CA_msg->msg.SourceAddress;
	if(writeCan(&j1939_msg)!= RC_SUCCESS)
		return RC_ERROR;
	
	j1939_msg.DataPage = 0;
	j1939_msg.edp = 0;
	j1939_msg.PDUFormat = J1939_PF_DT;
	#if 0
	if(CA_msg->msg.PDUFormat < 240)
		j1939_msg.PDUSpecific = CA_msg->msg.PDUSpecific;
	else
		j1939_msg.PDUSpecific = J1939_GLOBAL_ADDRESS;
	#endif
	
	j1939_msg.PDUSpecific = J1939_GLOBAL_ADDRESS;
	j1939_msg.DataLength = 8;
	j1939_msg.Priority = 7;
	j1939_msg.SourceAddress = CA_msg->msg.SourceAddress;
	
	for(i = 1; (i <= CA_msg->cm.sendpackets) && (i != 0);i++)
	{
		unsigned short j;
		j1939_msg.Data[0] = i;
		for(j = 1; j<8;j++)
		{
			unsigned short k;
			k = (i-1)*7+j-1;
			if(k < (CA_msg->sendbytes+3))
				j1939_msg.Data[j] = data[k];
			else
				j1939_msg.Data[j] = 0xff;
		}
		osDelay(BAMDTTIME);
		if(writeCan(&j1939_msg)!= RC_SUCCESS)
		{


			return result = RC_CANNOTTRANSMIT;
		}
	}
	
	return result = RC_SUCCESS;
}

unsigned char sendRTS(CA_Msg_t CA_msg)
{
	q_Message_t qmsg;
	unsigned char result = RC_SUCCESS;
	osEvent event;
	struct J1939_msg j1939_msg;
	j1939_msg.Data[0] = J1939_RTS_CONTROL_BYTE;
	j1939_msg.Data[1] = (CA_msg->sendbytes+3+CA_msg->cm.sendpackets)%256;
	j1939_msg.Data[2] = (CA_msg->sendbytes+3+CA_msg->cm.sendpackets)/256;
	j1939_msg.Data[3] = CA_msg->cm.sendpackets;
	j1939_msg.Data[4] = 0xff;     /*don't support maxium number of packets*/
	j1939_msg.Data[5] = CA_msg->msg.PDUSpecific;
	j1939_msg.Data[6] = CA_msg->msg.PDUFormat;
	j1939_msg.Data[7] = CA_msg->msg.DataPage;
	j1939_msg.DataLength = 8;
	j1939_msg.DataPage = 0;
	j1939_msg.edp = 0;
	j1939_msg.PDUFormat = J1939_PF_TP_CM;
	j1939_msg.PDUSpecific = CA_msg->msg.PDUSpecific;
	j1939_msg.Priority = 7;
	j1939_msg.SourceAddress = CA_msg->msg.SourceAddress;
	if(writeCan(&j1939_msg)== RC_SUCCESS)
	{

		CA_msg->cm.t3.value = T3TIMEOUT;
		CA_msg->cm.t3.en = 1;
		CA_msg->cm.t3.release = 0;
		event = osMessageGet(CA_msg->txq,WAITFOREVER);
		qmsg = (q_Message_t)event.value.p;
		if(qmsg->timeout == 0)
		{
			CA_msg->cm.nextnum = qmsg->nextnum;
			CA_msg->cm.sendnum = qmsg->sendnum;
			CA_msg->cm.cm_lock = 1;
			return result = RC_SUCCESS;
		}
		else
		{
			return result = RC_TIMEOUT;
		}
	}
	else
		return result = RC_CANNOTTRANSMIT;
}

unsigned char sendAbort(CA_Msg_t CA_msg,unsigned char reason)
{
	unsigned char result = RC_SUCCESS;
	struct J1939_msg j1939_msg;
	struct J1939_CM *tempCm;

	
	j1939_msg.Data[0] = J1939_CONNABORT_CONTROL_BYTE;
	j1939_msg.Data[1] = reason;
	j1939_msg.Data[2] = 0xff;
	j1939_msg.Data[3] = 0xff;
	j1939_msg.Data[4] = 0xff;     /*don't support maxium number of packets*/
	j1939_msg.Data[5] = CA_msg->msg.PDUSpecific;
	j1939_msg.Data[6] = CA_msg->msg.PDUFormat;
	j1939_msg.Data[7] = CA_msg->msg.DataPage;
	j1939_msg.DataLength = 8;
	j1939_msg.DataPage = 0;
	j1939_msg.edp = 0;
	j1939_msg.PDUFormat = J1939_PF_TP_CM;
	j1939_msg.PDUSpecific = CA_msg->msg.PDUSpecific;
	j1939_msg.Priority = 7;
	j1939_msg.SourceAddress = CA_msg->msg.SourceAddress;
	CA_msg->cm.cm_lock = 0;
	
	if(writeCan(&j1939_msg)!= RC_SUCCESS)
	{
		
		if(CA_msg->rcvdata != NULL)
		{
			free(CA_msg->rcvdata);
			CA_msg -> rcvdata = NULL;
		}
		tempCm = &(CA_msg->cm);
		memset(tempCm,0,sizeof(struct J1939_CM));
		return result = RC_CANNOTTRANSMIT;
	}
	else 
	{
		if(CA_msg->rcvdata != NULL)
		{
			free(CA_msg->rcvdata);
			CA_msg->rcvdata = NULL;
		}
		tempCm = &(CA_msg->cm);
		memset(tempCm,0,sizeof(struct J1939_CM));

		return result = RC_SUCCESS;
	}
}

unsigned char sendMessage(CA_Msg_t CA_msg, unsigned char *data)
{
	unsigned char i;
	struct J1939_msg j1939_msg;
	unsigned char sa;
	unsigned char result;
	if((sa = CA_FindAddress(&calist,CA_msg)) == 254)
		return RC_ERROR;
	if (CA_msg->sendbytes <= 8)
	{
		j1939_msg.DataLength = CA_msg->sendbytes;
		j1939_msg.DataPage = CA_msg->msg.DataPage;
		j1939_msg.edp = CA_msg->msg.edp;
		j1939_msg.PDUFormat = CA_msg->msg.PDUFormat;
		j1939_msg.PDUSpecific = CA_msg->msg.PDUSpecific;
		j1939_msg.Priority = CA_msg->msg.Priority;
		j1939_msg.SourceAddress = sa;
		for (i = 0;i < CA_msg->sendbytes;i++)
			j1939_msg.Data[i] = data[i];
		return writeCan(&j1939_msg);
	}
	else
	{
		unsigned char *tmp;
		unsigned short j,k;
		
		j = ceil(((double)CA_msg->sendbytes + 3)/7);
		CA_msg->cm.sendpackets = j;
		tmp = (unsigned char *)malloc((CA_msg->sendbytes+3) * sizeof(char));
		for(k=0;k < (CA_msg->sendbytes);k++)
		{
				tmp[k] = data[k];
		}
		tmp[k] = CA_msg->msg.PDUSpecific;
		tmp[k+1] = CA_msg->msg.PDUFormat;
		tmp[k+2] = CA_msg->msg.DataPage;
		CA_msg->msg.SourceAddress = sa;
		if(CA_msg->msg.PDUSpecific== J1939_GLOBAL_ADDRESS)
			result = sendBAM(CA_msg,tmp);
		else
		{
			
			result = sendRTS(CA_msg);
			if(result == RC_SUCCESS)
			{
				result = sendData(CA_msg, tmp);
				if(result != RC_SUCCESS )
					sendAbort(CA_msg, 0);
				if(tmp != NULL)
				{
					free(tmp);
					tmp = NULL;
				}
				return result;
			}
			else if(result == RC_TIMEOUT)
			{
				if(tmp != NULL)
				{
					free(tmp);
					tmp = NULL;
				}
				sendAbort(CA_msg,0);
				return result;
			}
			else if(result == RC_CANNOTTRANSMIT)
			{
				if(tmp != NULL)
				{
					free(tmp);
					tmp = NULL;
				}
				return result;
			}
		}

		if(tmp != NULL)
		{
			free(tmp);
			tmp = NULL;
		}
	}
	return result;
}


unsigned char checkRes()
{
	return RC_SUCCESS;
}

unsigned char sendCTS(J1939_msg_t j1939_msg,CA_Msg_t CA_msg,unsigned int startnum,unsigned int num)
{
	struct J1939_msg temp_msg;
	unsigned char result;
	temp_msg.DataLength = j1939_msg->DataLength;
	temp_msg.DataPage = j1939_msg->DataPage;
	temp_msg.edp = j1939_msg->edp;
	temp_msg.PDUFormat = j1939_msg->PDUFormat;
	temp_msg.PDUSpecific = j1939_msg->SourceAddress;
	temp_msg.Priority = j1939_msg->Priority;
	temp_msg.SourceAddress = j1939_msg->PDUSpecific;
	temp_msg.Data[0] = J1939_CTS_CONTROL_BYTE;
	#if 0
	if(j1939_msg->Data[4] < j1939_msg->Data[3])
		temp_msg.Data[1] = j1939_msg->Data[4];
	else
		temp_msg.Data[1] = j1939_msg->Data[3];
#endif
	temp_msg.Data[1] = num;
	temp_msg.Data[2] = startnum;
	temp_msg.Data[3] = 0xff;
	temp_msg.Data[4] = 0xff;
	temp_msg.Data[5] = CA_msg->cm.DTPS;
	temp_msg.Data[6] = CA_msg->cm.DTPF;
	temp_msg.Data[7] = CA_msg->cm.DTDP;
	
	if(writeCan(&temp_msg) == RC_SUCCESS)
	{
		CA_msg->cm.t2.en = 1;
		CA_msg->cm.t2.release = 0;
		CA_msg->cm.t2.value = T2TIMEOUT;
		return result = RC_SUCCESS;
	}
	else
		return result = RC_CANNOTTRANSMIT;
}

unsigned char response_RTS(J1939_msg_t j1939_msg ,CA_Msg_t CA_msg)
{
	unsigned char result = RC_SUCCESS;
	
	if(j1939_msg->PDUSpecific == J1939_GLOBAL_ADDRESS)
		return result = RC_ERROR;
	if(checkRes() == RC_SUCCESS && CA_msg->cm.cm_lock == 0)
	{
		rt_kprintf("receive rts packets = %d\r\n", j1939_msg->Data[3]);
		CA_msg->cm.rcvpackets = j1939_msg->Data[3];
		CA_msg->cm.rcvbytes = j1939_msg->Data[2]*256+j1939_msg->Data[1];
		CA_msg->rcvdata = (unsigned char *)malloc(7 * CA_msg->cm.rcvpackets);
		CA_msg->cm.DTDP = j1939_msg->DataPage;
		CA_msg->cm.DTPF = j1939_msg->PDUFormat;
		CA_msg->cm.DTPS = j1939_msg->PDUSpecific;
		#if 0
		if(j1939_msg->Data[4] < j1939_msg->Data[3])
			if(j1939_msg->Data[4] < 8)
				sendCTS(j1939_msg,CA_msg,1,j1939_msg->Data[4] );
			else 
				sendCTS(j1939_msg,CA_msg,1,8);
		else
			if(j1939_msg->Data[3] < 8)
				sendCTS(j1939_msg,CA_msg,1,j1939_msg->Data[3] );
			else 
				sendCTS(j1939_msg,CA_msg,1,8);
		#endif 
		
		sendCTS(j1939_msg,CA_msg,1,1);
		CA_msg->cm.cm_lock = 1;
		return result = RC_SUCCESS;
	}
	return result = RC_ERROR;
}


unsigned char response_CTS(J1939_msg_t j1939_msg ,CA_Msg_t CA_msg)
{
	unsigned char result = RC_SUCCESS;
	static struct q_Message q_msg;
	if(j1939_msg->PDUSpecific == J1939_GLOBAL_ADDRESS)
		return result = RC_ERROR;
	if(CA_msg->cm.t3.en == 0 && CA_msg->cm.t4.en == 0)
		return result = RC_ERROR;
	CA_msg->cm.t3.release = 1;
	CA_msg->cm.t4.release = 1;
	if(j1939_msg->Data[1] == 0 && j1939_msg->Data[2] == 255)
	{
		CA_msg->cm.t4.en = 1;
		CA_msg->cm.t4.release = 0;
		CA_msg->cm.t4.value = 1050;
		return result = RC_ERROR;
	}
	else
	{
		q_msg.nextnum = j1939_msg->Data[2];
		q_msg.sendnum = j1939_msg->Data[1];
		q_msg.timeout = 0;
		osMessagePut(CA_msg->txq,(unsigned long)&q_msg, 0);
		return result = RC_SUCCESS;
	}
}
unsigned char sendEndofMsgACK(J1939_msg_t j1939_msg ,CA_Msg_t CA_msg)
{
	struct J1939_msg temp_j1939;
	struct J1939_CM *tempCm;
	
	temp_j1939.DataLength = 8;
	temp_j1939.DataPage = 0;
	temp_j1939.edp = 0;
	temp_j1939.PDUFormat = J1939_PF_TP_CM;
	temp_j1939.PDUSpecific = j1939_msg->SourceAddress;
	temp_j1939.Priority = 7;
	temp_j1939.SourceAddress = j1939_msg->PDUSpecific;
	temp_j1939.Data[0] = J1939_EOMACK_CONTROL_BYTE;
	temp_j1939.Data[1] = CA_msg->cm.rcvbytes % 256;
	temp_j1939.Data[2] = CA_msg->cm.rcvbytes / 256;
	temp_j1939.Data[3] = CA_msg->cm.rcvpackets;
	temp_j1939.Data[4] = 0xff;
	temp_j1939.Data[5] = j1939_msg->Data[5];
	temp_j1939.Data[6] = j1939_msg->Data[6];
	temp_j1939.Data[7] = j1939_msg->Data[7];
	CA_msg->cm.cm_lock = 0;
	if(CA_msg->rcvdata != NULL)
	{
		free(CA_msg->rcvdata);
		CA_msg->rcvdata = NULL;
	}
	tempCm = &(CA_msg->cm);
	memset(tempCm,0,sizeof(struct J1939_CM));
	return writeCan(&temp_j1939);
}

void receiveData(J1939_msg_t j1939_msg ,CA_Msg_t CA_msg)
{
	unsigned char i;
	static struct rx_Message rxmsg,bamrxmsg;
	unsigned short j;
	if((j1939_msg->PDUSpecific == J1939_GLOBAL_ADDRESS) )
	{
		if((CA_msg->bam.cm_lock == 1) && (CA_msg->bamrcvdata != NULL))
		{
			CA_msg->bam.t1.en = 1;
			CA_msg->bam.t1.release = 1;
			CA_msg->bam.t1.value = 0;
				
			for(i=1;i<j1939_msg->DataLength;i++)
			{
				CA_msg->bamrcvdata[7*(j1939_msg->Data[0]-1)+i-1] = j1939_msg->Data[i];
			}
			
			if(j1939_msg->Data[0] >= CA_msg->bam.rcvpackets)
			{
				bamrxmsg.data = CA_msg->bamrcvdata;
				j = 7 * CA_msg->bam.rcvpackets - 1;
				while(bamrxmsg.data[j] == 0xff)
					j--;
				bamrxmsg.dp = bamrxmsg.data[j];
				bamrxmsg.PF = bamrxmsg.data[j-1];
				bamrxmsg.PS = bamrxmsg.data[j-2];
				bamrxmsg.size = j-2;
				rt_kprintf("\nbam receive %d data:\r\n", bamrxmsg.size);
				bamAbort(CA_msg);
				osMessagePut(CA_msg->rxq, (unsigned long)&bamrxmsg, 0);
				/*
				if(j1939_msg->PDUSpecific != J1939_GLOBAL_ADDRESS)
				{
					sendEndofMsgACK(j1939_msg ,CA_msg);
				}
				*/

			}
			else
			{
					CA_msg->bam.t1.en = 1;
					CA_msg->bam.t1.release = 0;
					CA_msg->bam.t1.value = T1TIMEOUT;
			}
		}

	}
	else
	{
		if((CA_msg->cm.cm_lock == 1) &&  (CA_msg->rcvdata != NULL))
		{
		
			CA_msg->cm.t1.en = 1;
			CA_msg->cm.t1.release = 1;
			CA_msg->cm.t1.value = 0;

			CA_msg->cm.t2.en = 1;
			CA_msg->cm.t2.release = 1;
			CA_msg->cm.t2.value = 0;


			for(i=1;i<j1939_msg->DataLength;i++)
			{
				CA_msg->rcvdata[7*(j1939_msg->Data[0]-1)+i-1] = j1939_msg->Data[i];
			}

			rt_kprintf("Data[0] = %d, rcvpackets = %d\r\n", j1939_msg->Data[0], CA_msg->cm.rcvpackets);


			if(j1939_msg->Data[0] >= CA_msg->cm.rcvpackets)
			{
				rxmsg.data = CA_msg->rcvdata;
				j = 7 * CA_msg->cm.rcvpackets - 1;
				while(rxmsg.data[j] == 0xff)
					j--;
				rxmsg.dp = rxmsg.data[j];
				rxmsg.PF = rxmsg.data[j-1];
				rxmsg.PS = rxmsg.data[j-2];
				rxmsg.size = j-2;
				rt_kprintf("\nDT receive %d data:\r\n", rxmsg.size);
				osMessagePut(CA_msg->rxq, (unsigned long)&rxmsg, 0);
				
				//if(j1939_msg->PDUSpecific != J1939_GLOBAL_ADDRESS)
				//{
				sendEndofMsgACK(j1939_msg ,CA_msg);
				//}
			}
			else 
			{
				j1939_msg->PDUFormat = J1939_PF_TP_CM;
				j1939_msg->Priority = 7;
				j1939_msg->DataPage = 0;
				j1939_msg->edp = 0;
				j1939_msg->DataLength = 8;
		
				sendCTS(j1939_msg,CA_msg,j1939_msg->Data[0]+1,1);
			}
		
		#if 0
		if((CA_msg->cm.cm_lock == 1) &&  (CA_msg->rcvdata != NULL))
		{
			static unsigned char curnum = 0;
			static unsigned char errnum = 0;
			static int lastpacket = 0; 
			unsigned char tmp;
			
			CA_msg->cm.t1.en = 1;
			CA_msg->cm.t1.release = 1;
			CA_msg->cm.t1.value = 0;

			CA_msg->cm.t2.en = 1;
			CA_msg->cm.t2.release = 1;
			CA_msg->cm.t2.value = 0;

			tmp = (j1939_msg->Data[0] % 8);
			if(tmp == 0)
				tmp = 8;
			if((curnum + 1) != tmp)
			{
				while(curnum != tmp -1)
				{
					errnum |= (unsigned int)0x01<<curnum;
					curnum++;
				}
				//curnum = tmp - 1;
			}
			curnum++;
			for(i=1;i<j1939_msg->DataLength;i++)
			{
				CA_msg->rcvdata[7*(j1939_msg->Data[0]-1)+i-1] = j1939_msg->Data[i];
			}
			if(j1939_msg->Data[0] >= CA_msg->cm.rcvpackets || tmp == 8)
			{
				lastpacket = 1;
			}
			rt_kprintf("Data[0] = %d, rcvpackets = %d\r\n", j1939_msg->Data[0], CA_msg->cm.rcvpackets);

			if(errnum == 0)
			{
				if(j1939_msg->Data[0] >= ((CA_msg->cm.rcvpackets / 8)*8) && lastpacket == 1)
				{
					lastpacket = 0;
					rxmsg.data = CA_msg->rcvdata;
					j = 7 * CA_msg->cm.rcvpackets - 1;
					while(rxmsg.data[j] == 0xff)
						j--;
					rxmsg.dp = rxmsg.data[j];
					rxmsg.PF = rxmsg.data[j-1];
					rxmsg.PS = rxmsg.data[j-2];
					rxmsg.size = j-2;
					rt_kprintf("\ndt receive %d data:\r\n", rxmsg.size);
					osMessagePut(CA_msg->rxq, (unsigned long)&rxmsg, 0);
					curnum = 0;
					errnum = 0;
					
					//if(j1939_msg->PDUSpecific != J1939_GLOBAL_ADDRESS)
					//{
						sendEndofMsgACK(j1939_msg ,CA_msg);
					//}
					

				}
				else if(lastpacket == 1)
				{
					//CA_msg->cm.t1.en = 1;
					//CA_msg->cm.t1.release = 0;
					//CA_msg->cm.t1.value = T1TIMEOUT;
					int nextpacket;
					lastpacket = 0;
					j1939_msg->PDUFormat = J1939_PF_TP_CM;
					j1939_msg->Priority = 7;
					j1939_msg->DataPage = 0;
					j1939_msg->edp = 0;
					j1939_msg->DataLength = 8;
					nextpacket = ((j1939_msg->Data[0]/8) + 1)*8 + 1;
					curnum = nextpacket - 1;
					if(CA_msg->cm.rcvpackets - nextpacket > 7)
						sendCTS(j1939_msg,CA_msg,nextpacket,8);
					else
						sendCTS(j1939_msg,CA_msg,nextpacket,CA_msg->cm.rcvpackets - nextpacket + 1);
				}
				else 
				{
					CA_msg->cm.t1.en = 1;
					CA_msg->cm.t1.release = 0;
					CA_msg->cm.t1.value = T1TIMEOUT;
				}
			}
			else
			{
				if((lastpacket == 1))
				{
					unsigned int startnum,num;

					j1939_msg->PDUFormat = J1939_PF_TP_CM;
					j1939_msg->Priority = 7;
					j1939_msg->DataPage = 0;
					j1939_msg->edp = 0;
					j1939_msg->DataLength = 8;
					i = 0;
					rt_kprintf("errnum = %x\n", errnum);
					while((errnum & (0x01 << i)) == 0)
						i++;
					errnum &= 0xfe<<i;
					
					rt_kprintf("errnum = %x\n", errnum);
					if(tmp != 8)
						startnum = ((j1939_msg->Data[0]/8) * 8) + i + 1;
					else
						startnum = (((j1939_msg->Data[0]/8)-1) * 8) + i + 1;
					curnum = startnum - 1;
					sendCTS(j1939_msg,CA_msg,startnum,1);
				}
				else
				{
					CA_msg->cm.t1.en = 1;
					CA_msg->cm.t1.release = 0;
					CA_msg->cm.t1.value = T1TIMEOUT;
				}	
			}
		}
		#endif
		}
	}
}

void responseEndofMsgACK(J1939_msg_t j1939_msg ,CA_Msg_t CA_msg)
{
	//if(CA_msg->cm.sendcomp == 1)
	//{
		rt_kprintf("receive endOfMsgAck\r\n");
		//osEnter_critical();
		if(CA_msg->rcvdata != NULL)
		{
			free(CA_msg->rcvdata);
			CA_msg->rcvdata = NULL;
		}
		//osExit_critical();
		CA_msg->cm.cm_lock = 0;
		CA_msg->cm.nextnum = 0;
		CA_msg->cm.sendcomp = 0;
		CA_msg->cm.sendpackets = 0;
		CA_msg->sendbytes = 0;
	//}
}
void responseBAM(J1939_msg_t j1939_msg ,CA_Msg_t CA_msg)
{
	if(j1939_msg->PDUSpecific == J1939_GLOBAL_ADDRESS)
	{
		rt_kprintf("receive bam\r\n");
		if(checkRes() == RC_SUCCESS && CA_msg->bam.cm_lock == 0)
		{
			CA_msg->bam.rcvpackets = j1939_msg->Data[3];
			CA_msg->bam.rcvbytes = j1939_msg->Data[2]*256+j1939_msg->Data[1];
			CA_msg->bamrcvdata = (unsigned char *)malloc(7 * CA_msg->bam.rcvpackets );
			CA_msg->bam.cm_lock = 1;
			CA_msg->bam.t1.en = 1;
			CA_msg->bam.t1.release = 0;
			CA_msg->bam.t1.value = T1TIMEOUT;
		}
	}
}

unsigned char responseAbort(J1939_msg_t j1939_msg ,CA_Msg_t CA_msg)
{
	struct J1939_CM *tempCm;
	
	if(CA_msg->rcvdata != NULL)
	{
		free(CA_msg->rcvdata);
		CA_msg->rcvdata = NULL;
	}
	tempCm = &(CA_msg->cm);
	memset(tempCm,0,sizeof(struct J1939_CM));
	
	return RC_SUCCESS;
}
void receiveMessage()
{
	static struct J1939_msg j1939_msg;
	CA_Msg_t CA_msg;
	static struct rx_Message rxmsg;
	struct CA_node *canode;
	list_node_t node;
	int i;

	while(readCan(&j1939_msg) == RC_SUCCESS)
	{
		if(j1939_msg.PDUFormat <240)
		{
			if (j1939_msg.PDUSpecific == J1939_GLOBAL_ADDRESS) 
			{
				node = calist.head.next;
				if(node != &(calist.head))
				{
					canode = rt_list_entry(node, struct CA_node, node);
					CA_msg = canode->ca_msg;
				}
				else 
					break;
			}
			else if ((CA_msg = Addr_FindCA(&calist,j1939_msg.PDUSpecific)) != NULL)
			{
			}
			else
				continue;

			switch (j1939_msg.PDUFormat)
			{
				case J1939_PF_TP_CM:
					switch (j1939_msg.Data[0])
					{
						case J1939_RTS_CONTROL_BYTE:
							response_RTS(&j1939_msg,CA_msg);
							break;
						case J1939_CTS_CONTROL_BYTE:
							response_CTS(&j1939_msg,CA_msg);
							break;
						case J1939_EOMACK_CONTROL_BYTE:
							responseEndofMsgACK(&j1939_msg,CA_msg);
							break;
						case J1939_BAM_CONTROL_BYTE:
							responseBAM(&j1939_msg,CA_msg);
							break;
						case J1939_CONNABORT_CONTROL_BYTE:
							responseAbort(&j1939_msg,CA_msg);
							break;
						default:
							goto sendmsg;
					}
					break;
				case J1939_PF_DT:
					receiveData(&j1939_msg,CA_msg);
					break;
				case J1939_PF_REQUEST:
					if ((j1939_msg.Data[0] == J1939_PGN0_REQ_ADDRESS_CLAIM) &&
						(j1939_msg.Data[1] == J1939_PGN1_REQ_ADDRESS_CLAIM) &&
						(j1939_msg.Data[2] == J1939_PGN2_REQ_ADDRESS_CLAIM))
						J1939_RequestForAddressClaimHandling(&j1939_msg,CA_msg);
					else 
						goto sendmsg;
					break;
				case J1939_PF_ADDRESS_CLAIMED:
					J1939_AddressClaimHandling( &j1939_msg,CA_msg );
					break;

				default:
	sendmsg:
				
				rxmsg.data = &j1939_msg.Data;
				rxmsg.size = j1939_msg.DataLength;
				rxmsg.dp = j1939_msg.DataPage;
				rxmsg.PF = j1939_msg.PDUFormat;
				rxmsg.PS = j1939_msg.PDUSpecific;
				
				osMessagePut(CA_msg->rxq, (unsigned long)&rxmsg, 0);
				break;
			}
		}
		else 
		{
			
			node = calist.head.next;
			if(node != &(calist.head))
			{
				canode = rt_list_entry(node, struct CA_node, node);
				CA_msg = canode->ca_msg;
			}
			else 
				break;
			
			goto sendmsg;
		}
	}
}

void bamAbort(CA_Msg_t CA_msg)
{
	struct J1939_BAM *tempbam;
	
	if(CA_msg->bamrcvdata != NULL)
	{
		free(CA_msg->bamrcvdata);
		CA_msg->bamrcvdata = NULL;
	}
	tempbam = &(CA_msg->bam);
	memset(tempbam,0,sizeof(struct J1939_BAM));
}

static void timerJ1939func(void* parameter)
{
	CA_Msg_t tmp_ca_msg;
	struct list_node *node;
	static struct q_Message q_msg;
	struct CA_node *canode;
	for(node = calist.head.next; node != &(calist.head); node = node->next)
	{
		canode = rt_list_entry(node, struct CA_node, node);
		tmp_ca_msg = canode->ca_msg;

		if((tmp_ca_msg->bam.t1.en) == 1)
		{
			if((tmp_ca_msg->bam.t1.release) == 1)
			{
				tmp_ca_msg->bam.t1.en = 0;
				tmp_ca_msg->bam.t1.value = 0;
				tmp_ca_msg->bam.t1.release = 0;
			}
			else
			{
				if((tmp_ca_msg->bam.t1.value)-- == 0)
				{
					tmp_ca_msg->bam.t1.en = 0;
					tmp_ca_msg->bam.t1.value = 0;
					tmp_ca_msg->bam.t1.release = 0;
				//	osMessagePut(tmp_ca_msg->cm.txq, &q_msg, NULL);
					//sendAbort(tmp_ca_msg,0);

			bamAbort(tmp_ca_msg);

				}
			}
		}

		if((tmp_ca_msg->cm.t1.en) == 1)
		{
			if((tmp_ca_msg->cm.t1.release) == 1)
			{
				tmp_ca_msg->cm.t1.en = 0;
				tmp_ca_msg->cm.t1.value = 0;
				tmp_ca_msg->cm.t1.release = 0;
			}
			else
			{
				if((tmp_ca_msg->cm.t1.value)-- == 0)
				{
					tmp_ca_msg->cm.t1.en = 0;
					tmp_ca_msg->cm.t1.value = 0;
					tmp_ca_msg->cm.t1.release = 0;
				//	osMessagePut(tmp_ca_msg->cm.txq, &q_msg, NULL);
					sendAbort(tmp_ca_msg,0);
				}
			}
		}
		if((tmp_ca_msg->cm.t2.en) == 1)
		{
			if((tmp_ca_msg->cm.t2.release) == 1)
			{
				tmp_ca_msg->cm.t2.en = 0;
				tmp_ca_msg->cm.t2.value = 0;
				tmp_ca_msg->cm.t2.release = 0;
			}
			else
			{
				if((tmp_ca_msg->cm.t2.value)-- == 0)
				{
					q_msg.timeout = 1;
					q_msg.nextnum = 0;
					q_msg.sendnum = 0;
					tmp_ca_msg->cm.t2.en = 0;
					tmp_ca_msg->cm.t2.value = 0;
					tmp_ca_msg->cm.t2.release = 0;
				//	osMessagePut(tmp_ca_msg->cm.txq, &q_msg, NULL);
					sendAbort(tmp_ca_msg,0);
				}
			}
		}
		if((tmp_ca_msg->cm.t3.en) == 1)
		{
			if((tmp_ca_msg->cm.t3.release) == 1)
			{
				tmp_ca_msg->cm.t3.en = 0;
				tmp_ca_msg->cm.t3.value = 0;
				tmp_ca_msg->cm.t3.release = 0;
			}
			else
			{
				if((tmp_ca_msg->cm.t3.value)-- == 0)
				{
					q_msg.timeout = 1;
					q_msg.nextnum = 0;
					q_msg.sendnum = 0;
					tmp_ca_msg->cm.t3.en = 0;
					tmp_ca_msg->cm.t3.value = 0;
					tmp_ca_msg->cm.t3.release = 0;
					osMessagePut(tmp_ca_msg->txq, (unsigned long)&q_msg, 0);
				}
			}
		}
		if((tmp_ca_msg->cm.t4.en) == 1)
		{
			if((tmp_ca_msg->cm.t4.release) == 1)
			{
				tmp_ca_msg->cm.t4.en = 0;
				tmp_ca_msg->cm.t4.value = 0;
				tmp_ca_msg->cm.t4.release = 0;
			}
			else
			{
				if((tmp_ca_msg->cm.t4.value)-- == 0)
				{
					q_msg.timeout = 1;
					q_msg.nextnum = 0;
					q_msg.sendnum = 0;
					tmp_ca_msg->cm.t4.en = 0;
					tmp_ca_msg->cm.t4.value = 0;
					tmp_ca_msg->cm.t4.release = 0;
					osMessagePut(tmp_ca_msg->txq, (unsigned long)&q_msg, 0);
				}
			}
		}
		if((tmp_ca_msg->cm.tr.en) == 1)
		{
			if((tmp_ca_msg->cm.tr.release) == 1)
			{
				tmp_ca_msg->cm.tr.en = 0;
				tmp_ca_msg->cm.tr.value = 0;
				tmp_ca_msg->cm.tr.release = 0;
			}
			else
			{
				if((tmp_ca_msg->cm.tr.value)-- == 0)
				{
					q_msg.timeout = 1;
					q_msg.nextnum = 0;
					q_msg.sendnum = 0;
					tmp_ca_msg->cm.tr.en = 0;
					tmp_ca_msg->cm.tr.value = 0;
					tmp_ca_msg->cm.tr.release = 0;
					osMessagePut(tmp_ca_msg->txq, (unsigned long)&q_msg,0);
				}
			}
		}
		if((tmp_ca_msg->cm.th.en) == 1)
		{
			if((tmp_ca_msg->cm.th.release) == 1)
			{
				tmp_ca_msg->cm.th.en = 0;
				tmp_ca_msg->cm.th.value = 0;
				tmp_ca_msg->cm.th.release = 0;
			}
			else
			{
				if((tmp_ca_msg->cm.th.value)-- == 0)
				{
					q_msg.timeout = 1;
					q_msg.nextnum = 0;
					q_msg.sendnum = 0;
					tmp_ca_msg->cm.th.en = 0;
					tmp_ca_msg->cm.th.value = 0;
					tmp_ca_msg->cm.th.release = 0;
					osMessagePut(tmp_ca_msg->txq, (unsigned long)&q_msg, 0);
				}
			}
		}
			#if 1
		if(tmp_ca_msg->nm.t250.en == 1)
		{
			if(tmp_ca_msg->nm.t250.release == 1)
			{
				tmp_ca_msg->nm.cannotClaimAddr = 1;
				tmp_ca_msg->nm.WaitingForAddressClaimContention = 0;
				tmp_ca_msg->nm.t250.en = 0;
				tmp_ca_msg->nm.t250.value = 0;
				tmp_ca_msg->nm.t250.release = 0;
			}
			else
			{
				if(tmp_ca_msg->nm.t250.value-- == 0)
				{
					tmp_ca_msg->nm.cannotClaimAddr = 0;
					tmp_ca_msg->nm.WaitingForAddressClaimContention = 0;
					tmp_ca_msg->nm.t250.en = 0;
					tmp_ca_msg->nm.t250.value = 0;
					tmp_ca_msg->nm.t250.release = 0;
				}
			}
		}
			#endif
	}
}


 unsigned char initJ1939()
{
	unsigned char result = RC_SUCCESS;
	osTimerId timerid;
	struct os_timer_def timer_def;
	
	list_init(&(calist.head));

	timer_def.flag = osTimerPeriodic;
	timer_def.name = "J1939timer";
	timer_def.parameter = NULL;
	timer_def.time = 1;
	timer_def.timeout = timerJ1939func;
	timerid = osTimerCreate(&timer_def, osTimerPeriodic, NULL);
	if(timerid != NULL) 
		osTimerStart(timerid,timer_def.time);
	return result;
}


