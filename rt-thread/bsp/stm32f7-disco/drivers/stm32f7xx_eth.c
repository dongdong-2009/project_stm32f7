/**
  ******************************************************************************
  * @file    stm32f7xx_eth.c
  * @author  MCD Application Team
  * @version V0.0.1
  * @date    01/21/2016
  * @brief   
  *          
  *         
  ******************************************************************************
  * @copy
  *
  */

/*
 * Change Logs:
 * Date           Author       Notes

*/

#include <rtthread.h>
#ifdef RT_USING_LWIP
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_eth.h"
#include "stm32f7xx_eth.h"
#include "ethernetif.h"
#include "lwipopts.h"
/* RT-Thread Device Interface */
//#include <netif/ethernetif.h>
#include <netif/etharp.h>
#include <lwip/icmp.h>

/* debug option */
//#define ETH_DEBUG
//#define ETH_RX_DUMP
//#define ETH_TX_DUMP

#ifdef ETH_DEBUG
#define STM32_ETH_PRINTF          rt_kprintf
#else
#define STM32_ETH_PRINTF(...)
#endif
#define LAN8742A_PHY_ADDRESS          0x00



#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x2000E000
__no_init ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
#pragma location=0x2000E100
__no_init ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
#elif defined ( __CC_ARM   )
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __attribute__((at(0x2000E000)));/* Ethernet Rx MA Descriptor */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __attribute__((at(0x2000E100)));/* Ethernet Tx DMA Descriptor */
#elif defined ( __GNUC__   )
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __attribute__((section(".RxDescripSection")));/* Ethernet Rx MA Descriptor */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __attribute__((section(".TxDescripSection")));/* Ethernet Tx DMA Descriptor */
#endif

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2000E200
__no_init uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; /* Ethernet Receive Buffer */
#pragma location=0x2000FFC4
__no_init uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; /* Ethernet Transmit Buffer */
#elif defined ( __CC_ARM   )
uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((at(0x2000E200)));  /* Ethernet Receive Buffer */
uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]  __attribute__((at(0x2000FFC4))); /* Ethernet Transmit Buffer */
#elif defined ( __GNUC__   )
uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__((section(".RxBUF")));/* Ethernet Receive Buffer */
uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__((section(".TxBUF")));/* Ethernet Transmit Buffer */
#endif


#define MAX_ADDR_LEN 6
struct rt_stm32_eth
{
	/* inherit from ethernet device */
	struct eth_device parent;

	/* interface address info. */
	rt_uint8_t  dev_addr[MAX_ADDR_LEN];			/* hw address	*/

	uint32_t    ETH_Speed; /*!< @ref ETH_Speed */
	uint32_t    ETH_Mode;  /*!< @ref ETH_Duplex_Mode */
};
static struct rt_stm32_eth stm32_eth_device;
static struct rt_semaphore tx_wait;
static rt_bool_t tx_is_waiting = RT_FALSE;
ETH_HandleTypeDef EthHandle;
/** @defgroup ETH_Private_Defines
  * @{
  */
/* Global pointers on Tx and Rx descriptor used to track transmit and receive descriptors */
ETH_DMADescTypeDef  *DMATxDescToSet;
ETH_DMADescTypeDef  *DMARxDescToGet;

ETH_DMADescTypeDef  *DMAPTPTxDescToSet;
ETH_DMADescTypeDef  *DMAPTPRxDescToGet;

/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  heth: ETH handle
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
	#ifdef ETH_DEBUG
	static int i=0;
	#endif
	STM32_ETH_PRINTF("----------------------------------------\r\n");
	STM32_ETH_PRINTF("ETH_DMA_IT_R  %d\r\n",i++);
	/* a frame has been received */
	eth_device_ready(&(stm32_eth_device.parent));
}
extern int txmbcounter;

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
	STM32_ETH_PRINTF("ETH_DMA_IT_T\r\n");
	if (tx_is_waiting == RT_TRUE)
	{
		tx_is_waiting = RT_FALSE;
		rt_sem_release(&tx_wait);
	}	
}


HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
	STM32_ETH_PRINTF("ETH_DMA_IT_AIS\r\n");
	rt_kprintf("ETH_DMA_IT_AIS\r\n");
	#if 1
	/* [1]:Transmit Process Stopped. */
	if(__HAL_ETH_DMA_GET_FLAG(heth,ETH_DMA_IT_TPS))
	{
		STM32_ETH_PRINTF("AIS ETH_DMA_IT_TPS\r\n");
		rt_kprintf("AIS ETH_DMA_IT_TPS\r\n");
	}
	
	/* [3]:Transmit Jabber Timeout. */
	if(__HAL_ETH_DMA_GET_FLAG(heth,ETH_DMA_IT_TJT))
	{
		STM32_ETH_PRINTF("AIS ETH_DMA_IT_TJT\r\n");
		rt_kprintf("AIS ETH_DMA_IT_TJT\r\n");
	}
	
	/* [4]: Receive FIFO Overflow. */
	if(__HAL_ETH_DMA_GET_FLAG(heth,ETH_DMA_IT_RO))
	{
		STM32_ETH_PRINTF("AIS ETH_DMA_IT_RO\r\n");
		rt_kprintf("AIS ETH_DMA_IT_RO\r\n");
	}
	
	/* [5]: Transmit Underflow. */
	if(__HAL_ETH_DMA_GET_FLAG(heth,ETH_DMA_IT_TU))
	{
		STM32_ETH_PRINTF("AIS ETH_DMA_IT_TU\r\n");
		rt_kprintf("AIS ETH_DMA_IT_TU\r\n");
	}
	
	/* [7]: Receive Buffer Unavailable. */
	if(__HAL_ETH_DMA_GET_FLAG(heth,ETH_DMA_IT_RBU))
	{
		STM32_ETH_PRINTF("AIS ETH_DMA_IT_RBU\r\n");
		rt_kprintf("AIS ETH_DMA_IT_RBU\r\n");
	}
	
	/* [8]: Receive Process Stopped. */
	if(__HAL_ETH_DMA_GET_FLAG(heth,ETH_DMA_IT_RPS))
	{
		STM32_ETH_PRINTF("AIS ETH_DMA_IT_RPS\r\n");
		rt_kprintf("AIS ETH_DMA_IT_RPS\r\n");
	}
	
	/* [9]: Receive Watchdog Timeout. */
	if(__HAL_ETH_DMA_GET_FLAG(heth,ETH_DMA_IT_RWT))
	{
		STM32_ETH_PRINTF("AIS ETH_DMA_IT_RWT\r\n");
		rt_kprintf("AIS ETH_DMA_IT_RWT\r\n");
	}
	
	/* [10]: Early Transmit Interrupt. */
	
	/* [13]: Fatal Bus Error. */
	if(__HAL_ETH_DMA_GET_FLAG(heth,ETH_DMA_IT_FBE))
	{
		STM32_ETH_PRINTF("AIS ETH_DMA_IT_FBE\r\n");
		rt_kprintf("AIS ETH_DMA_IT_FBE\r\n");
	}
	#endif
}



/**
  * @brief  Ethernet IRQ Handler
  * @param  None
  * @retval None
  */


void ETH_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

  	HAL_ETH_IRQHandler(&EthHandle);
      /* leave interrupt */
    rt_interrupt_leave();
}



/* initialize the interface */
static rt_err_t rt_stm32_eth_init(rt_device_t dev)
{
    struct rt_stm32_eth * stm32_eth = (struct rt_stm32_eth *)dev;
	ETH_DMAInitTypeDef dmaconf;
	ETH_MACInitTypeDef macconf;
	
  	uint8_t macaddress[6]= { MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5 };
	
	EthHandle.Instance = ETH;  
	EthHandle.Init.MACAddr = macaddress;
	EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
	EthHandle.Init.Speed = ETH_SPEED_100M;
	EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
	EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
	EthHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
	EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
	EthHandle.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
		
	/* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
	HAL_ETH_Init(&EthHandle);
	
	/* Initialize Tx Descriptors list: Chain Mode */
	HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
	DMATxDescToSet = DMATxDscrTab;
	/* Initialize Rx Descriptors list: Chain Mode  */
	HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
	DMARxDescToGet = DMARxDscrTab;
	/* Enable MAC and DMA transmission and reception */
	HAL_ETH_Start(&EthHandle);

    return RT_EOK;
}

static rt_err_t rt_stm32_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t rt_stm32_eth_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_size_t rt_stm32_eth_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	rt_set_errno(-RT_ENOSYS);
	return 0;
}

static rt_size_t rt_stm32_eth_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	rt_set_errno(-RT_ENOSYS);
	return 0;
}

static rt_err_t rt_stm32_eth_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch(cmd)
	{
	case NIOCTL_GADDR:
		/* get mac address */
		if(args) rt_memcpy(args, stm32_eth_device.dev_addr, 6);
		else return -RT_ERROR;
		break;

	default :
		break;
	}

	return RT_EOK;
}


	rt_err_t rt_stm32_eth_tx(rt_device_t dev, struct pbuf *p)
	{
	  err_t errval;
	  struct pbuf *q;
	  uint8_t *buffer = (uint8_t *)(EthHandle.TxDesc->Buffer1Addr);
	  __IO ETH_DMADescTypeDef *DmaTxDesc;
	  uint32_t framelength = 0;
	  uint32_t bufferoffset = 0;
	  uint32_t byteslefttocopy = 0;
	  uint32_t payloadoffset = 0;
	
	  DmaTxDesc = EthHandle.TxDesc;
	  bufferoffset = 0;
	  
	  /* copy frame from pbufs to driver buffers */
	  for(q = p; q != NULL; q = q->next)
	  {
		#if 1
		/* Is this buffer available? If not, goto error */
		if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
		{
		  errval = ERR_USE;
		  goto error;
		}
		#endif

		#if 0
		while ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
		{
		    rt_err_t result;
		    rt_uint32_t level;

		    level = rt_hw_interrupt_disable();
		    tx_is_waiting = RT_TRUE;
		    rt_hw_interrupt_enable(level);

		    /* it's own bit set, wait it */
		    result = rt_sem_take(&tx_wait, RT_WAITING_FOREVER);
		    if (result == RT_EOK) 
		    {
				break;
		    }		
		    if (result == -RT_ERROR) 
		    {
				return -RT_ERROR;
		    }
		}
		#endif	
		/* Get bytes in current lwIP buffer */
		byteslefttocopy = q->len;
		payloadoffset = 0;
		
		/* Check if the length of data to copy is bigger than Tx buffer size*/
		while( (byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE )
		{
		  /* Copy data to Tx buffer*/
		  memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)q->payload + payloadoffset), (ETH_TX_BUF_SIZE - bufferoffset) );
		  
		  /* Point to next descriptor */
		  DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);

		  #if 1
		  /* Check if the buffer is available */
		  if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
		  {
			errval = ERR_USE;
			goto error;
		  }
		  #endif
		  
		  buffer = (uint8_t *)(DmaTxDesc->Buffer1Addr);
		  
		  byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
		  payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
		  framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
		  bufferoffset = 0;
		}
		
		/* Copy the remaining bytes */
		memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)q->payload + payloadoffset), byteslefttocopy );
		bufferoffset = bufferoffset + byteslefttocopy;
		framelength = framelength + byteslefttocopy;
	  }
	  STM32_ETH_PRINTF("transmit to DMA\r\n");
	  /* Prepare transmit descriptors to give to DMA */ 
	  HAL_ETH_TransmitFrame(&EthHandle, framelength);
	  
	  errval = ERR_OK;
	 #if 1
	error:
	  
	  /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
	  //rt_kprintf("tx error\r\n");
	  if ((EthHandle.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
	  {
		/* Clear TUS ETHERNET DMA flag */
		EthHandle.Instance->DMASR = ETH_DMASR_TUS;
		
		/* Resume DMA transmission*/
		EthHandle.Instance->DMATPDR = 0;
	  }
	  #endif
	  return errval;
	}




struct pbuf * rt_stm32_eth_rx(rt_device_t dev)
{
  struct pbuf *p = NULL, *q = NULL;
  uint16_t len = 0;
  uint8_t *buffer;
  __IO ETH_DMADescTypeDef *dmarxdesc;
  uint32_t bufferoffset = 0;
  uint32_t payloadoffset = 0;
  uint32_t byteslefttocopy = 0;
  uint32_t i=0;
  
  /* get received frame */
 // STM32_ETH_PRINTF("enter rx\r\n");
  if(HAL_ETH_GetReceivedFrame_IT(&EthHandle) != HAL_OK)
    return NULL;
  //STM32_ETH_PRINTF("received data in the low level\r\n");
  /* Obtain the size of the packet and put it into the "len" variable. */
  len = EthHandle.RxFrameInfos.length;
  buffer = (uint8_t *)EthHandle.RxFrameInfos.buffer;
  
  if (len > 0)
  {
    /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  }
  
  if (p != NULL)
  {
    dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
    bufferoffset = 0;
    
    for(q = p; q != NULL; q = q->next)
    {
      byteslefttocopy = q->len;
      payloadoffset = 0;
      
      /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size */
      while( (byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE )
      {
        /* Copy data to pbuf */
        memcpy( (uint8_t*)((uint8_t*)q->payload + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));
        
        /* Point to next descriptor */
        dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
        buffer = (uint8_t *)(dmarxdesc->Buffer1Addr);
        
        byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
        payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
        bufferoffset = 0;
      }
      
      /* Copy remaining data in pbuf */
      memcpy( (uint8_t*)((uint8_t*)q->payload + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), byteslefttocopy);
      bufferoffset = bufferoffset + byteslefttocopy;
    }
  }
    
  /* Release descriptors to DMA */
  /* Point to first descriptor */
  dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
  /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
  for (i=0; i< EthHandle.RxFrameInfos.SegCount; i++)
  {  
    dmarxdesc->Status |= ETH_DMARXDESC_OWN;
    dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
  }
    
  /* Clear Segment_Count */
  EthHandle.RxFrameInfos.SegCount =0;
  
  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((EthHandle.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)  
  {
    /* Clear RBUS ETHERNET DMA flag */
    EthHandle.Instance->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    EthHandle.Instance->DMARPDR = 0;
  }
  return p;
}



/*
 * GPIO Configuration for ETH
 */
static void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOs clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/* Ethernet pins configuration ************************************************/
	/*
	RMII_REF_CLK ----------------------> PA1
	RMII_MDIO -------------------------> PA2
	RMII_MDC --------------------------> PC1
	RMII_MII_CRS_DV -------------------> PA7
	RMII_MII_RXD0 ---------------------> PC4
	RMII_MII_RXD1 ---------------------> PC5
	RMII_MII_RXER ---------------------> PG2
	RMII_MII_TX_EN --------------------> PG11
	RMII_MII_TXD0 ---------------------> PG13
	RMII_MII_TXD1 ---------------------> PG14
	*/

	/* Configure PA1, PA2 and PA7 */
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL; 
	GPIO_InitStructure.Alternate = GPIO_AF11_ETH;
	GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PC1, PC4 and PC5 */
	GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PG2, PG11, PG13 and PG14 */
	GPIO_InitStructure.Pin =	GPIO_PIN_2 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

	/* Enable the Ethernet global Interrupt */
	__HAL_ETH_DMA_ENABLE_IT(&EthHandle, ETH_DMA_IT_AIS | ETH_DMA_IT_R);
	HAL_NVIC_SetPriority(ETH_IRQn, ETH_IRQ_PREEMPT, ETH_IRQ_SUB);
	HAL_NVIC_EnableIRQ(ETH_IRQn);

	/* Enable ETHERNET clock	*/
	__HAL_RCC_ETH_CLK_ENABLE();

}

/* PHY: LAN8720 */
static uint8_t phy_speed = 0;
#define PHY_LINK_MASK       (1<<0)
#define PHY_100M_MASK       (1<<1)
#define PHY_DUPLEX_MASK     (1<<2)
static void phy_monitor_thread_entry(void *parameter)
{
    uint8_t phy_addr = 0xFF;
    uint8_t phy_speed_new = 0;

    /* phy search */
    {
        rt_uint32_t i;
        rt_uint16_t temp;

        for(i=0; i<=0x1F; i++)
        {
            temp = HAL_ETH_ReadPHYRegister(&EthHandle,i, 0x02);

            if( temp != 0xFFFF )
            {
                phy_addr = i;
                break;
            }
        }
    } /* phy search */

    if(phy_addr == 0xFF)
    {
        STM32_ETH_PRINTF("phy not probe!\r\n");
        return;
    }
    else
    {
        STM32_ETH_PRINTF("found a phy, address:0x%02X\r\n", phy_addr);
    }

    /* RESET PHY */
    STM32_ETH_PRINTF("RESET PHY!\r\n");
    HAL_ETH_WritePHYRegister(&EthHandle, PHY_BCR, PHY_RESET);
    rt_thread_delay(RT_TICK_PER_SECOND * 2);
    HAL_ETH_WritePHYRegister(&EthHandle, PHY_BCR, PHY_AUTONEGOTIATION);

    while(1)
    {
        uint16_t status  = HAL_ETH_ReadPHYRegister(&EthHandle,phy_addr, PHY_BSR);
        //STM32_ETH_PRINTF("LAN8720 status:0x%04X\r\n", status);

        phy_speed_new = 0;

        if(status & (PHY_AUTONEGO_COMPLETE | PHY_LINKED_STATUS))
        {
            uint16_t SR;

            SR = HAL_ETH_ReadPHYRegister(&EthHandle,phy_addr, 31);
            STM32_ETH_PRINTF("LAN8720 REG 31:0x%04X\r\n", SR);

            SR = (SR >> 2) & 0x07; /* LAN8720, REG31[4:2], Speed Indication. */
            phy_speed_new = PHY_LINK_MASK;

            if((SR & 0x03) == 2)
            {
                phy_speed_new |= PHY_100M_MASK;
            }

            if(SR & 0x04)
            {
                phy_speed_new |= PHY_DUPLEX_MASK;
            }
        }

        /* linkchange */
        if(phy_speed_new != phy_speed)
        {
            if(phy_speed_new & PHY_LINK_MASK)
            {
                STM32_ETH_PRINTF("link up ");

                if(phy_speed_new & PHY_100M_MASK)
                {
                    STM32_ETH_PRINTF("100Mbps");
                    stm32_eth_device.ETH_Speed = ETH_SPEED_100M;
                }
                else
                {
                    stm32_eth_device.ETH_Speed = ETH_SPEED_10M;
                    STM32_ETH_PRINTF("10Mbps");
                }

                if(phy_speed_new & PHY_DUPLEX_MASK)
                {
                    STM32_ETH_PRINTF(" full-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_MODE_FULLDUPLEX;
                }
                else
                {
                    STM32_ETH_PRINTF(" half-duplex\r\n");
                    stm32_eth_device.ETH_Mode = ETH_MODE_HALFDUPLEX;
                }
                rt_stm32_eth_init((rt_device_t)&stm32_eth_device);

                /* send link up. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
            } /* link up. */
            else
            {
                STM32_ETH_PRINTF("link down\r\n");
                /* send link down. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
            } /* link down. */

            phy_speed = phy_speed_new;
        } /* linkchange */

        rt_thread_delay(RT_TICK_PER_SECOND);
    } /* while(1) */
}

void rt_hw_stm32_eth_init(void)
{
    GPIO_Configuration();

    stm32_eth_device.ETH_Speed = ETH_SPEED_100M;
    stm32_eth_device.ETH_Mode  = ETH_MODE_FULLDUPLEX;

    /* OUI 00-80-E1 STMICROELECTRONICS. */
    stm32_eth_device.dev_addr[0] = MAC_ADDR0;
    stm32_eth_device.dev_addr[1] = MAC_ADDR1;
    stm32_eth_device.dev_addr[2] = MAC_ADDR2;
    /* generate MAC addr from 96bit unique ID (only for test). */
    stm32_eth_device.dev_addr[3] = MAC_ADDR3;
    stm32_eth_device.dev_addr[4] = MAC_ADDR4;
    stm32_eth_device.dev_addr[5] = MAC_ADDR5;

    stm32_eth_device.parent.parent.init       = rt_stm32_eth_init;
    stm32_eth_device.parent.parent.open       = rt_stm32_eth_open;
    stm32_eth_device.parent.parent.close      = rt_stm32_eth_close;
    stm32_eth_device.parent.parent.read       = rt_stm32_eth_read;
    stm32_eth_device.parent.parent.write      = rt_stm32_eth_write;
    stm32_eth_device.parent.parent.control    = rt_stm32_eth_control;
    stm32_eth_device.parent.parent.user_data  = RT_NULL;

    stm32_eth_device.parent.eth_rx     = rt_stm32_eth_rx;
    stm32_eth_device.parent.eth_tx     = rt_stm32_eth_tx;

    /* init tx semaphore */
    rt_sem_init(&tx_wait, "tx_wait", 0, RT_IPC_FLAG_FIFO);

    /* register eth device */
    eth_device_init(&(stm32_eth_device.parent), "e0");

    /* start phy monitor */
    {
        rt_thread_t tid;
        tid = rt_thread_create("phy",
                               phy_monitor_thread_entry,
                               RT_NULL,
                               512,
                               RT_THREAD_PRIORITY_MAX - 3,
                               2);
        if (tid != RT_NULL)
            rt_thread_startup(tid);
    }
}
INIT_APP_EXPORT(rt_hw_stm32_eth_init);
#endif

