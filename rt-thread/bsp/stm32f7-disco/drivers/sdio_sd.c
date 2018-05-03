
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
#if 1
/*
 * RT-Thread SD Card Driver
 * 20100715 Bernard support SDHC card great than 4G.
 * 20110905 JoyChen support to STM32F2xx
 */
#include <rtthread.h>
#ifdef RT_USING_DFS_ELMFAT
#include <dfs_fs.h>
#include "sdio_sd.h"
#include "stm32f7xx_hal_sd.h"


/* set sector size to 512 */
#define SECTOR_SIZE		512

static struct rt_device sdcard_device;
static struct dfs_partition part;
static struct rt_semaphore sd_lock;
static SD_HandleTypeDef hsd;
static uint32_t CardType =  SDIO_STD_CAPACITY_SD_CARD_V1_1;
__IO uint32_t StopCondition = 0;
__IO HAL_SD_ErrorTypedef TransferError = SD_OK;
__IO uint32_t TransferEnd = 0, DMAEndOfTransfer = 0;
static SD_CardInfo SDCardInfo;
static SD_CardInfo      uSdCardInfo;



/* RT-Thread Device Driver Interface */
static rt_err_t rt_sdcard_init(rt_device_t dev)
{
/*	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	   */

	if (rt_sem_init(&sd_lock, "sdlock", 1, RT_IPC_FLAG_FIFO) != RT_EOK)
	{
		rt_kprintf("init sd lock semaphore failed\n");
	}
	else
		rt_kprintf("SD Card init OK\n");

	return RT_EOK;
}

static rt_err_t rt_sdcard_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t rt_sdcard_close(rt_device_t dev)
{
	return RT_EOK;
}

static uint32_t dma_buffer[512/sizeof(uint32_t)];
static rt_size_t rt_sdcard_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	HAL_SD_ErrorTypedef status;
	//rt_kprintf("sd: read 0x%X, sector 0x%X, 0x%X\n", (uint32_t)buffer ,pos, size);
	rt_sem_take(&sd_lock, RT_WAITING_FOREVER);

    if(((uint32_t)buffer & 0x03) != 0)
    {
        /* non-aligned. */
        uint32_t i;
        rt_size_t sector_adr;
        uint8_t* copy_buffer;

        sector_adr = pos*SECTOR_SIZE;
        copy_buffer = (uint8_t*)buffer;

        for(i=0; i<size; i++)
        {
            status = HAL_SD_ReadBlocks_DMA(&hsd,(uint32_t*)dma_buffer,sector_adr,SECTOR_SIZE,1);

			/* Wait until transfer is complete */
			if(status == SD_OK)
			{
			if(HAL_SD_CheckReadOperation(&hsd, (uint32_t)SD_DATATIMEOUT) != SD_OK)
				{
				status = SD_ERROR;
				}
			else
				{
				status = SD_OK;
				}
			}
			
            memcpy(copy_buffer, dma_buffer, SECTOR_SIZE);
            sector_adr += SECTOR_SIZE;
            copy_buffer += SECTOR_SIZE;
        }
    }
    else 
    {
        status = HAL_SD_ReadBlocks_DMA(&hsd,(uint32_t*)buffer,pos*SECTOR_SIZE,SECTOR_SIZE,size);
		/* Wait until transfer is complete */
		if(status == SD_OK)
		{
		if((status = HAL_SD_CheckReadOperation(&hsd, (uint32_t)SD_DATATIMEOUT)) != SD_OK)
			{
				status = SD_ERROR;
			}
		else
			{
				status = SD_OK;
			}
		}
    }

	rt_sem_release(&sd_lock);
	if (status == SD_OK) return size;

	rt_kprintf("read failed: %d, buffer 0x%08x\n", status, buffer);
	return 0;
}


static rt_size_t rt_sdcard_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	HAL_SD_ErrorTypedef status;

	//rt_kprintf("sd: write 0x%X, sector 0x%X, 0x%X\n", (uint32_t)buffer , pos, size);
	rt_sem_take(&sd_lock, RT_WAITING_FOREVER);

    if(((uint32_t)buffer & 0x03) != 0)
    {
        /* non-aligned. */
        uint32_t i;
        rt_size_t sector_adr;
        uint8_t* copy_buffer;

        sector_adr = pos*SECTOR_SIZE;
        copy_buffer = (uint8_t*)buffer;

        for(i=0; i<size; i++)
        {
            memcpy(dma_buffer, copy_buffer, SECTOR_SIZE);
            status = HAL_SD_WriteBlocks_DMA(&hsd, (uint32_t*)dma_buffer,sector_adr, SECTOR_SIZE, 1);

			  /* Wait until transfer is complete */
			  if(status == SD_OK)
			  {
			    if(HAL_SD_CheckWriteOperation(&hsd, (uint32_t)SD_DATATIMEOUT) != SD_OK)
			    {
			      status = SD_ERROR;
			    }
			    else
			    {
			      status = SD_OK;
			    }
			  }

            sector_adr += SECTOR_SIZE;
            copy_buffer += SECTOR_SIZE;
        }
    }
    else  
    {
		status = HAL_SD_WriteBlocks_DMA(&hsd, (uint32_t*)buffer, pos*SECTOR_SIZE, SECTOR_SIZE, size);

		/* Wait until transfer is complete */
		if(status == SD_OK)
		{
		if(HAL_SD_CheckWriteOperation(&hsd, (uint32_t)SD_DATATIMEOUT) != SD_OK)
			{
			  status = SD_ERROR;
			}
		else
			{
			  status = SD_OK;
			}
		}
    }
	// }
	rt_sem_release(&sd_lock);

	if (status == SD_OK) return size;

	rt_kprintf("write failed: %d, buffer 0x%08x\n", status, buffer);
	return 0;
}




static rt_err_t rt_sdcard_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = 512;
        geometry->block_size = SDCardInfo.CardBlockSize;
		if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
			geometry->sector_count = (SDCardInfo.SD_csd.DeviceSize + 1)  * 1024;
		else
        	geometry->sector_count = SDCardInfo.CardCapacity/SDCardInfo.CardBlockSize;
    }

	return RT_EOK;
}


/** @defgroup STM32_EVAL_SDIO_SD_Private_Functions
  * @{
  */
/**
  * @brief	DeInitializes the SD MSP.
  * @param	hsd: SD handle
  * @param	Params
  * @retval None
  */
__weak void BSP_SD_MspDeInit(SD_HandleTypeDef *hsd, void *Params)
{
  static DMA_HandleTypeDef dma_rx_handle;
  static DMA_HandleTypeDef dma_tx_handle;

  /* Disable NVIC for DMA transfer complete interrupts */
  HAL_NVIC_DisableIRQ(SD_DMAx_Rx_IRQn);
  HAL_NVIC_DisableIRQ(SD_DMAx_Tx_IRQn);

  /* Deinitialize the stream for new transfer */
  dma_rx_handle.Instance = SD_DMAx_Rx_STREAM;
  HAL_DMA_DeInit(&dma_rx_handle);

  /* Deinitialize the stream for new transfer */
  dma_tx_handle.Instance = SD_DMAx_Tx_STREAM;
  HAL_DMA_DeInit(&dma_tx_handle);

  /* Disable NVIC for SDIO interrupts */
  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

  /* DeInit GPIO pins can be done in the application
	 (by surcharging this __weak function) */

  /* Disable SDMMC1 clock */
  __HAL_RCC_SDMMC1_CLK_DISABLE();

  /* GPIO pins clock and DMA clocks can be shut down in the application
	 by surcharging this __weak function */
}


/**
  * @brief  DeInitializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_DeInit(void)
{ 
  uint8_t sd_state = MSD_OK;
 
  hsd.Instance = SDMMC1;
  
  /* HAL SD deinitialization */
  if(HAL_SD_DeInit(&hsd) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  /* Msp SD deinitialization */
  hsd.Instance = SDMMC1;
  BSP_SD_MspDeInit(&hsd, NULL);
  
  return  sd_state;
}


/**
  * @brief  Initializes the SD MSP.
  * @param  hsd: SD handle
  * @param  Params
  * @retval None
  */
__weak void BSP_SD_MspInit(SD_HandleTypeDef *hsd, void *Params)
{
  static DMA_HandleTypeDef dma_rx_handle;
  static DMA_HandleTypeDef dma_tx_handle;
  GPIO_InitTypeDef gpio_init_structure;

  /* Enable SDIO clock */
  __HAL_RCC_SDMMC1_CLK_ENABLE();
  
  /* Enable DMA2 clocks */
  __DMAx_TxRx_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Common GPIO configuration */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_LOW;
  gpio_init_structure.Alternate = GPIO_AF12_SDMMC1;
  
  /* GPIOC configuration */
  #if 0
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  #endif
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &gpio_init_structure);

  /* GPIOD configuration */
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);
#if 1
  /* NVIC configuration for SDIO interrupts */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, SDMMC1_IRQ_PREEMPT, SDMMC1_IRQ_SUB);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
#endif

  /* Configure DMA Rx parameters */
  dma_rx_handle.Init.Channel             = SD_DMAx_Rx_CHANNEL;
  dma_rx_handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  dma_rx_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dma_rx_handle.Init.MemInc              = DMA_MINC_ENABLE;
  dma_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma_rx_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dma_rx_handle.Init.Mode                = DMA_PFCTRL;
  dma_rx_handle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dma_rx_handle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dma_rx_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma_rx_handle.Init.MemBurst            = DMA_MBURST_INC4;
  dma_rx_handle.Init.PeriphBurst         = DMA_PBURST_INC4;
  
  dma_rx_handle.Instance = SD_DMAx_Rx_STREAM;
  
  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmarx, dma_rx_handle);
  
  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dma_rx_handle);
  
  /* Configure the DMA stream */
  HAL_DMA_Init(&dma_rx_handle);
  
  /* Configure DMA Tx parameters */
  dma_tx_handle.Init.Channel             = SD_DMAx_Tx_CHANNEL;
  dma_tx_handle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  dma_tx_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dma_tx_handle.Init.MemInc              = DMA_MINC_ENABLE;
  dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma_tx_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dma_tx_handle.Init.Mode                = DMA_PFCTRL;
  dma_tx_handle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dma_tx_handle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dma_tx_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma_tx_handle.Init.MemBurst            = DMA_MBURST_INC4;
  dma_tx_handle.Init.PeriphBurst         = DMA_PBURST_INC4;
  
  dma_tx_handle.Instance = SD_DMAx_Tx_STREAM;
  
  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmatx, dma_tx_handle);
  
  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dma_tx_handle);
  
  /* Configure the DMA stream */
  HAL_DMA_Init(&dma_tx_handle); 
  #if 1
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SD_DMAx_Rx_IRQn, SD_DMAX_RX_IRQ_PREEMPT, SD_DMAX_RX_IRQ_SUB);
  HAL_NVIC_EnableIRQ(SD_DMAx_Rx_IRQn);
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SD_DMAx_Tx_IRQn, SD_DMAX_TX_IRQ_PREEMPT,SD_DMAX_TX_IRQ_SUB);
  HAL_NVIC_EnableIRQ(SD_DMAx_Tx_IRQn);
  #endif
}


#if 1
/**
  * @brief  Handles SD card interrupt request.
  * @retval None
  */
void BSP_SD_IRQHandler(void)
{
  HAL_SD_IRQHandler(&hsd);
}

/**
  * @brief  Handles SD DMA Tx transfer interrupt request.
  * @retval None
  */
void BSP_SD_DMA_Tx_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hsd.hdmatx); 
}

/**
  * @brief  Handles SD DMA Rx transfer interrupt request.
  * @retval None
  */
void BSP_SD_DMA_Rx_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hsd.hdmarx);
}

#endif

/**
  * @brief  This function handles DMA2 Stream 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void)
{
  BSP_SD_DMA_Rx_IRQHandler();
}

/**
  * @brief  This function handles DMA2 Stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream6_IRQHandler(void)
{
  BSP_SD_DMA_Tx_IRQHandler(); 
}

/**
  * @brief  This function handles SDIO interrupt request.
  * @param  None
  * @retval None
  */
void SDMMC1_IRQHandler(void)
{
  BSP_SD_IRQHandler();
}



/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_Init(void)
{ 
  uint8_t sd_state = MSD_OK;
  
  /* uSD device interface configuration */
  hsd.Instance = SDMMC1;

  hsd.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass         = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide             = SDMMC_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv            = SDMMC_TRANSFER_CLK_DIV;

  #if 0
  /* Msp SD Detect pin initialization */
  BSP_SD_Detect_MspInit(&uSdHandle, NULL);
  if(BSP_SD_IsDetected() != SD_PRESENT)   /* Check if SD card is present */
  {
    return MSD_ERROR_SD_NOT_PRESENT;
  }
  #endif
  
  /* Msp SD initialization */
  BSP_SD_MspInit(&hsd, NULL);

  /* HAL SD initialization */
  if(HAL_SD_Init(&hsd, &uSdCardInfo) != SD_OK)
  {
    sd_state = MSD_ERROR;
  }
  
  /* Configure SD Bus width */
  if(sd_state == MSD_OK)
  {
    /* Enable wide operation */
    if(HAL_SD_WideBusOperation_Config(&hsd, SDMMC_BUS_WIDE_1B) != SD_OK)
    {
      sd_state = MSD_ERROR;
    }
    else
    {
      sd_state = MSD_OK;
    }
  }
  
  return  sd_state;
}


/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for data
  *         transfer).
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
HAL_SD_ErrorTypedef SD_Init(void)
{
  __IO HAL_SD_ErrorTypedef errorstatus = SD_OK;

  BSP_SD_DeInit();
  if (BSP_SD_Init() != MSD_OK)
  	errorstatus = SD_ERROR;

  return(errorstatus);
}

void rt_hw_sdcard_init(void)
{
	if (SD_Init() == SD_OK)
	{
		//HAL_SD_ErrorTypedef status;
		//rt_uint8_t *sector;

		/*status = SD_GetCardInfo(&SDCardInfo);
		if (status != SD_OK) goto __return;

		status = SD_SelectDeselect((u32) (SDCardInfo.RCA << 16));
		if (status != SD_OK) goto __return;

		SD_EnableWideBusOperation(SDIO_BusWide_4b);
		SD_SetDeviceMode(SD_DMA_MODE); */



		// /* get the first sector to read partition table */
		// sector = (rt_uint8_t*) rt_malloc (512);
		// if (sector == RT_NULL)
		// {
			// rt_kprintf("allocate partition sector buffer failed\n");
			// return;
		// }
		// status = SD_ReadBlock(0, (uint8_t*)sector, 512);
		// status = SD_WaitReadOperation();
        // while(SD_GetStatus() != SD_TRANSFER_OK);

		// if (status == SD_OK)
		// {
			// /* get the first partition */
			// if (dfs_filesystem_get_partition(&part, sector, 0) != 0)
            // {
                // /* there is no partition */
                // part.offset = 0;
                // part.size   = 0;
            // }
		// }
		// else
		{
			/* there is no partition table */
			part.offset = 0;
			part.size   = 0;
		}

		// /* release sector buffer */
		// rt_free(sector);

		/* register sdcard device */
		sdcard_device.type  = RT_Device_Class_Block;
		sdcard_device.init 	= rt_sdcard_init;
		sdcard_device.open 	= rt_sdcard_open;
		sdcard_device.close = rt_sdcard_close;
		sdcard_device.read 	= rt_sdcard_read;
		sdcard_device.write = rt_sdcard_write;
		sdcard_device.control = rt_sdcard_control;

		/* no private */
		sdcard_device.user_data = &SDCardInfo;

		rt_device_register(&sdcard_device, "sd0",
			RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);

		rt_kprintf("sdcard init success\r\n");

		return;
	}

//__return:
	rt_kprintf("sdcard init failed\r\n");
}
INIT_FS_EXPORT(rt_hw_sdcard_init);

#endif
#if 0
/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* Process All SDIO Interrupt Sources */
    if( SD_ProcessIRQSrc() == 2)
		rt_kprintf("SD Error\n");

    /* leave interrupt */
    rt_interrupt_leave();
}

void SD_SDIO_DMA_IRQHANDLER(void)
{
  SD_ProcessDMAIRQ();
}
#endif
#endif
