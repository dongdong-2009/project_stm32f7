
#include "SAEJ1939.h"

static signed char j1939strncmp(const char *cs, const char *ct, unsigned long count)
{
    signed char __res = 0;
	
	if((cs == NULL) || (ct == NULL))
	{
		return 1;
	}
    while (count)
    {
        if ((__res = *cs++ - *ct++) != 0 )
            break;
        count --;
    }

    return __res;
}

unsigned char CA_FindAddress(CA_list_t calist,CA_Msg_t ca_msg)
{
	list_node_t node;
	struct CA_node *canode;
	J1939_CAname_t CAname = ca_msg->CAname;
	CA_Msg_t tmp_ca_msg;
	
	for(node = calist->head.next; node != &(calist->head); node = node->next)
	{
		canode = rt_list_entry(node, struct CA_node, node);
		tmp_ca_msg = canode->ca_msg;
		if(j1939strncmp((char *)CAname,(char *)tmp_ca_msg->CAname,sizeof(struct J1939_CAname)) == 0)
		{
			return (tmp_ca_msg->msg.SourceAddress);
		}
	}
	
	return 254;
}
CA_Msg_t Addr_FindCA(CA_list_t calist, unsigned char addr)
{
	list_node_t node;
	struct CA_node *canode;
	CA_Msg_t tmp_ca_msg;

	for(node = calist->head.next; node != &(calist->head); node = node->next)
	{
		unsigned char i=0;
		canode = rt_list_entry(node, struct CA_node, node);
		tmp_ca_msg = canode->ca_msg;
		if(tmp_ca_msg->msg.SourceAddress == addr)
			break;
	}
	if(node == &(calist->head) )
		return NULL;
	else
		return tmp_ca_msg;

}

unsigned char J1939_RequestForAddressClaimHandling( J1939_msg_t j1939_msg ,CA_Msg_t CA_msg )
{
	unsigned char i;
	unsigned char *CA_Name = (unsigned char *)(CA_msg->CAname);

	if (CA_msg->nm.cannotClaimAddr)
		j1939_msg->SourceAddress = J1939_NULL_ADDRESS;	// Send Cannot Claim Address message
	else
		j1939_msg->SourceAddress = CA_msg->msg.SourceAddress;		// Send Address Claim for current address

	j1939_msg->Priority = 6;
	j1939_msg->PDUFormat = J1939_PF_ADDRESS_CLAIMED;	// Same as J1939_PF_CANNOT_CLAIM_ADDRESS
	j1939_msg->PDUSpecific = J1939_GLOBAL_ADDRESS;
	j1939_msg->DataLength = J1939_DATA_LENGTH;

	for (i=0; i<J1939_DATA_LENGTH; i++)
		j1939_msg->Data[i] = CA_Name[i];
	
	return writeCan(j1939_msg);
}

signed char CompareName( unsigned char *OtherName,CA_Msg_t CA_msg  )
{
	unsigned char	i;
	unsigned char *CA_Name = (unsigned char *)(CA_msg->CAname);
	
	for (i = 0; (i<J1939_DATA_LENGTH) && (OtherName[i] == CA_Name[i]); i++);

	if (i == J1939_DATA_LENGTH)
		return 0;
	else if (CA_Name[i] < OtherName[i] )
		return -1;
	else
		return 1;
}

unsigned char J1939_AddressClaimHandling(  J1939_msg_t j1939_msg ,CA_Msg_t CA_msg  )
{

	unsigned char i;
	unsigned char *CA_Name = (unsigned char *)(CA_msg->CAname);
	
	j1939_msg->Priority = 6;
	j1939_msg->PDUFormat = J1939_PF_ADDRESS_CLAIMED;
	j1939_msg->PDUSpecific = J1939_GLOBAL_ADDRESS;
	j1939_msg->DataLength = J1939_DATA_LENGTH;


	//if (j1939_msg->SourceAddress != CA_msg->msg.SourceAddress)
	//	return RC_ERROR;

	if (CompareName( j1939_msg->Data,CA_msg ) != -1) // Our CA_Name is not less
	{

		for (i=0; i<J1939_DATA_LENGTH; i++)
			j1939_msg->Data[i] = CA_Name[i];
		
		j1939_msg->SourceAddress = J1939_NULL_ADDRESS;
		if(writeCan(j1939_msg)== RC_SUCCESS)
		{
			CA_msg->nm.t250.release = 1;
			return RC_SUCCESS;
		}
		else 
			return RC_CANNOTTRANSMIT;
	}
	
	for (i=0; i<J1939_DATA_LENGTH; i++)
		j1939_msg->Data[i] = CA_Name[i];

	j1939_msg->SourceAddress = CA_msg->msg.SourceAddress;
	
	if (writeCan(j1939_msg ) == RC_SUCCESS)
	{
		if ((( CA_msg->msg.SourceAddress & 0x80) == 0) ||			// Addresses 0-127
			(( CA_msg->msg.SourceAddress & 0xF8) == 0xF8))		// Addresses 248-253 (254,255 illegal)
		{
			CA_msg->nm.cannotClaimAddr = 0;
		}
		else
		{
			// We don't have a proprietary address, so we need to wait.
	 		CA_msg->nm.WaitingForAddressClaimContention = 1;
			//osDelay(250);
	 		//if (CA_msg->nm.WaitingForAddressClaimContention == 0)
	 		//{
			//	CA_msg->nm.cannotClaimAddr = 0;
	 		//}
			#if 1
			CA_msg->nm.t250.value = 250;
			CA_msg->nm.t250.en = 1;
			CA_msg->nm.t250.release = 0;
			#endif
		}
	}
	else 
		return RC_CANNOTTRANSMIT;
	return RC_SUCCESS;
}

unsigned char sendAddrClaimed(CA_Msg_t CA_msg)
{
	struct J1939_msg j1939_msg;
	unsigned char *CA_Name = (unsigned char *)(CA_msg->CAname);
	unsigned char i;
	
	CA_msg->nm.cannotClaimAddr = 1;
	
	j1939_msg.SourceAddress = CA_msg->msg.SourceAddress;		// Send Address Claim for current address
	j1939_msg.Priority = 6;
	j1939_msg.edp = 0;
	j1939_msg.DataPage = 0;
	j1939_msg.PDUFormat = J1939_PF_ADDRESS_CLAIMED;	// Same as J1939_PF_CANNOT_CLAIM_ADDRESS
	j1939_msg.PDUSpecific = J1939_GLOBAL_ADDRESS;
	j1939_msg.DataLength = J1939_DATA_LENGTH;

	for (i=0; i<J1939_DATA_LENGTH; i++)
		j1939_msg.Data[i] = CA_Name[i];
	
	if(writeCan(&j1939_msg)==RC_SUCCESS)
	{
		// We don't have a proprietary address, so we need to wait.
		CA_msg->nm.WaitingForAddressClaimContention = 1;
		//osDelay(250);
		//if (CA_msg->nm.WaitingForAddressClaimContention == 0)
		//{
		//	CA_msg->nm.cannotClaimAddr = 0;
		//}
		#if 1
		CA_msg->nm.t250.value = 250;
		CA_msg->nm.t250.en = 1;
		CA_msg->nm.t250.release = 0;
		#endif

		return RC_SUCCESS;
	}
	return RC_CANNOTTRANSMIT;
}


unsigned char initAddr(CA_Msg_t ca_msg)
{
	if(ca_msg->msg.SourceAddress != 254)
	{
		return sendAddrClaimed(ca_msg);
	}
	return RC_ERROR;
}



