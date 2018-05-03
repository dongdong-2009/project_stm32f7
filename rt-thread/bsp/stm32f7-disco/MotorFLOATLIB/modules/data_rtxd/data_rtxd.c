/*=====================================================================================
 File name:        DATA_RTXD.C  (IQ version)
                    
 Originator:	DL.K

 Description:  The DATA RTXD
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/data_rtxd/data_rtxd.h"

DATA_RTXD_Handle DATA_RTXD_Init(void *pMemory, const size_t numBytes)
{
	DATA_RTXD_Handle handle;
	uint8_t i;

	if (numBytes < sizeof(DATA_RTXD_Obj))
		return((DATA_RTXD_Handle)NULL);

	// assign the handle
	handle = (DATA_RTXD_Handle)pMemory;

	handle->data_rxd_num = 0;
	for (i = 0; i<DATA_RXD_SIZE; i++)
	{
		handle->data_rxd[i] = 0;
	}

	handle->data_txd_num = DATA_TXD_SIZE;
	for (i = 0; i<DATA_TXD_SIZE; i++)
	{
		handle->data_txd[i] = 0;
	}

	handle->time = 0;

	return(handle);

}


