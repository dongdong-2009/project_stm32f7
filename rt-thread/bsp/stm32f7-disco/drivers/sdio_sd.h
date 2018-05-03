/**
  ******************************************************************************
  * @file    stm32746g_discovery_sd.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32746g_discovery_sd.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SDIO_SD
#define SDIO_SD

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32746g_discovery.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32746G_DISCOVERY
  * @{
  */
    
/** @addtogroup STM32746G_DISCOVERY_SD
  * @{
  */    

/** @defgroup STM32746G_DISCOVERY_SD_Exported_Types STM32746G_DISCOVERY_SD Exported Types
  * @{
  */

/** 
  * @brief SD Card information structure 
  */
#define SD_CardInfo HAL_SD_CardInfoTypedef
/**
  * @}
  */
/**  
* @brief Supported SD Memory Cards 
*/
#define SDIO_STD_CAPACITY_SD_CARD_V1_1			  ((uint32_t)0x00000000)
#define SDIO_STD_CAPACITY_SD_CARD_V2_0			   ((uint32_t)0x00000001)
#define SDIO_HIGH_CAPACITY_SD_CARD 				((uint32_t)0x00000002)
#define SDIO_MULTIMEDIA_CARD						 ((uint32_t)0x00000003)
#define SDIO_SECURE_DIGITAL_IO_CARD				  ((uint32_t)0x00000004)
#define SDIO_HIGH_SPEED_MULTIMEDIA_CARD 		   ((uint32_t)0x00000005)
#define SDIO_SECURE_DIGITAL_IO_COMBO_CARD			((uint32_t)0x00000006)
#define SDIO_HIGH_CAPACITY_MMC_CARD				 ((uint32_t)0x00000007)

/** 
  * @brief  SD status structure definition  
  */     
#define MSD_OK                        ((uint8_t)0x00)
#define MSD_ERROR                     ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)
   
/** @defgroup STM32746G_DISCOVERY_SD_Exported_Constants STM32746G_DISCOVERY_SD Exported Constants
  * @{
  */ 
#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

#define SD_DATATIMEOUT           ((uint32_t)100000000)
    
/* DMA definitions for SD DMA transfer */
#define __DMAx_TxRx_CLK_ENABLE            __HAL_RCC_DMA2_CLK_ENABLE
#define SD_DMAx_Tx_CHANNEL                DMA_CHANNEL_4
#define SD_DMAx_Rx_CHANNEL                DMA_CHANNEL_4
#define SD_DMAx_Tx_STREAM                 DMA2_Stream6  
#define SD_DMAx_Rx_STREAM                 DMA2_Stream3  
#define SD_DMAx_Tx_IRQn                   DMA2_Stream6_IRQn
#define SD_DMAx_Rx_IRQn                   DMA2_Stream3_IRQn
#define SD_DMAx_Tx_IRQHandler             DMA2_Stream6_IRQHandler   
#define SD_DMAx_Rx_IRQHandler             DMA2_Stream3_IRQHandler 
#define SD_DetectIRQHandler()             HAL_GPIO_EXTI_IRQHandler(SD_DETECT_PIN)
/**
  * @}
  */
  
/** @defgroup STM32746G_DISCOVERY_SD_Exported_Macro STM32746G_DISCOVERY_SD Exported Macro
  * @{
  */ 
/** @defgroup STM32_EVAL_SDIO_SD_Exported_Types
  * @{
  */
  #if 0
typedef enum
{
/**
  * @brief	SDIO specific error defines
  */
  SD_CMD_CRC_FAIL					 = (1), /*!< Command response received (but CRC check failed) */
  SD_DATA_CRC_FAIL					 = (2), /*!< Data bock sent/received (CRC check Failed) */
  SD_CMD_RSP_TIMEOUT				 = (3), /*!< Command response timeout */
  SD_DATA_TIMEOUT					 = (4), /*!< Data time out */
  SD_TX_UNDERRUN					 = (5), /*!< Transmit FIFO under-run */
  SD_RX_OVERRUN 					 = (6), /*!< Receive FIFO over-run */
  SD_START_BIT_ERR					 = (7), /*!< Start bit not detected on all data signals in widE bus mode */
  SD_CMD_OUT_OF_RANGE				 = (8), /*!< CMD's argument was out of range.*/
  SD_ADDR_MISALIGNED				 = (9), /*!< Misaligned address */
  SD_BLOCK_LEN_ERR					 = (10), /*!< Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
  SD_ERASE_SEQ_ERR					 = (11), /*!< An error in the sequence of erase command occurs.*/
  SD_BAD_ERASE_PARAM				 = (12), /*!< An Invalid selection for erase groups */
  SD_WRITE_PROT_VIOLATION			 = (13), /*!< Attempt to program a write protect block */
  SD_LOCK_UNLOCK_FAILED 			 = (14), /*!< Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card */
  SD_COM_CRC_FAILED 				 = (15), /*!< CRC check of the previous command failed */
  SD_ILLEGAL_CMD					 = (16), /*!< Command is not legal for the card state */
  SD_CARD_ECC_FAILED				 = (17), /*!< Card internal ECC was applied but failed to correct the data */
  SD_CC_ERROR						 = (18), /*!< Internal card controller error */
  SD_GENERAL_UNKNOWN_ERROR			 = (19), /*!< General or Unknown error */
  SD_STREAM_READ_UNDERRUN			 = (20), /*!< The card could not sustain data transfer in stream read operation. */
  SD_STREAM_WRITE_OVERRUN			 = (21), /*!< The card could not sustain data programming in stream mode */
  SD_CID_CSD_OVERWRITE				 = (22), /*!< CID/CSD overwrite error */
  SD_WP_ERASE_SKIP					 = (23), /*!< only partial address space was erased */
  SD_CARD_ECC_DISABLED				 = (24), /*!< Command has been executed without using internal ECC */
  SD_ERASE_RESET					 = (25), /*!< Erase sequence was cleared before executing because an out of erase sequence command was received */
  SD_AKE_SEQ_ERROR					 = (26), /*!< Error in sequence of authentication. */
  SD_INVALID_VOLTRANGE				 = (27),
  SD_ADDR_OUT_OF_RANGE				 = (28),
  SD_SWITCH_ERROR					 = (29),
  SD_SDIO_DISABLED					 = (30),
  SD_SDIO_FUNCTION_BUSY 			 = (31),
  SD_SDIO_FUNCTION_FAILED			 = (32),
  SD_SDIO_UNKNOWN_FUNCTION			 = (33),

/**
  * @brief	Standard error defines
  */
  SD_INTERNAL_ERROR,
  SD_NOT_CONFIGURED,
  SD_REQUEST_PENDING,
  SD_REQUEST_NOT_APPLICABLE,
  SD_INVALID_PARAMETER,
  SD_UNSUPPORTED_FEATURE,
  SD_UNSUPPORTED_HW,
  SD_ERROR,
  SD_OK = 0
} SD_Error;
#endif
/**
  * @}
  */
   
/** @addtogroup STM32746G_DISCOVERY_SD_Exported_Functions
  * @{
  */   
uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_DeInit(void);
uint8_t BSP_SD_ITConfig(void);
void    BSP_SD_DetectIT(void);
void    BSP_SD_DetectCallback(void);
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumOfBlocks);
uint8_t BSP_SD_Erase(uint64_t StartAddr, uint64_t EndAddr);
void    BSP_SD_IRQHandler(void);
void    BSP_SD_DMA_Tx_IRQHandler(void);
void    BSP_SD_DMA_Rx_IRQHandler(void);
HAL_SD_TransferStateTypedef BSP_SD_GetStatus(void);
void    BSP_SD_GetCardInfo(HAL_SD_CardInfoTypedef *CardInfo);
uint8_t BSP_SD_IsDetected(void);

/* These functions can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
void    BSP_SD_MspInit(SD_HandleTypeDef *hsd, void *Params);
void    BSP_SD_Detect_MspInit(SD_HandleTypeDef *hsd, void *Params);
void    BSP_SD_MspDeInit(SD_HandleTypeDef *hsd, void *Params);

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __STM32746G_DISCOVERY_SD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
