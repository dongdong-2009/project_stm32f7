/*
 * File      : drv_usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-08-01     xiaonong     the first version for stm32f7xx
 */

#include "stm32f7xx.h"
#include "drv_usart.h"
#include "board.h"
#include <rtdevice.h>

/* Definition for USART1 clock resources */
#define USART1_CLK_ENABLE()              __USART1_CLK_ENABLE()
#define USART1_RX_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USART1_FORCE_RESET()             __USART1_FORCE_RESET()
#define USART1_RELEASE_RESET()           __USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USART1_TX_PIN                    GPIO_PIN_9
#define USART1_TX_GPIO_PORT              GPIOA
#define USART1_TX_AF                     GPIO_AF7_USART1
#define USART1_RX_PIN                    GPIO_PIN_10
#define USART1_RX_GPIO_PORT              GPIOA
#define USART1_RX_AF                     GPIO_AF7_USART1


/* Definition for USART1 clock resources */
#define USART3_CLK_ENABLE()              __USART3_CLK_ENABLE()
#define USART3_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define USART3_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()

#define USART3_FORCE_RESET()             __USART3_FORCE_RESET()
#define USART3_RELEASE_RESET()           __USART3_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USART3_TX_PIN                    GPIO_PIN_10
#define USART3_TX_GPIO_PORT              GPIOC
#define USART3_TX_AF                     GPIO_AF7_USART3
#define USART3_RX_PIN                    GPIO_PIN_11
#define USART3_RX_GPIO_PORT              GPIOC
#define USART3_RX_AF                     GPIO_AF7_USART3



/* STM32 uart driver */
struct stm32_uart
{
    UART_HandleTypeDef UartHandle;
    IRQn_Type irq;
};

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;
	if(uart->UartHandle.Instance == USART1)
	{
	    uart->UartHandle.Init.BaudRate   = cfg->baud_rate;
	    uart->UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	    uart->UartHandle.Init.Mode       = UART_MODE_TX_RX;
	    uart->UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	    switch (cfg->data_bits)
	    {
	    case DATA_BITS_7:
	        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_7B;
	        break;
	    case DATA_BITS_8:
	        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	        break;
	    case DATA_BITS_9:
	        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
	        break;
	    default:
	        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	        break;
	    }
	    switch (cfg->stop_bits)
	    {
	    case STOP_BITS_1:
	        uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
	        break;
	    case STOP_BITS_2:
	        uart->UartHandle.Init.StopBits   = UART_STOPBITS_2;
	        break;
	    default:
	        uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
	        break;
	    }
	    switch (cfg->parity)
	    {
	    case PARITY_NONE:
	        uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
	        break;
	    case PARITY_ODD:
	        uart->UartHandle.Init.Parity     = UART_PARITY_ODD;
	        break;
	    case PARITY_EVEN:
	        uart->UartHandle.Init.Parity     = UART_PARITY_EVEN;
	        break;
	    default:
	        uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
	        break;
	    }
	    if (HAL_UART_DeInit(&uart->UartHandle) != HAL_OK)
	    {
	        return RT_ERROR;
	    }
	    if (HAL_UART_Init(&uart->UartHandle) != HAL_OK)
	    {
	        return RT_ERROR;
	    }
	}

	if(uart->UartHandle.Instance == USART3)
	{
	    uart->UartHandle.Init.BaudRate   = cfg->baud_rate;
	    uart->UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	    uart->UartHandle.Init.Mode       = UART_MODE_TX_RX;
	    uart->UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	    switch (cfg->data_bits)
	    {
	    case DATA_BITS_7:
	        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_7B;
	        break;
	    case DATA_BITS_8:
	        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	        break;
	    case DATA_BITS_9:
	        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
	        break;
	    default:
	        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	        break;
	    }
	    switch (cfg->stop_bits)
	    {
	    case STOP_BITS_1:
	        uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
	        break;
	    case STOP_BITS_2:
	        uart->UartHandle.Init.StopBits   = UART_STOPBITS_2;
	        break;
	    default:
	        uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
	        break;
	    }
	    switch (cfg->parity)
	    {
	    case PARITY_NONE:
	        uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
	        break;
	    case PARITY_ODD:
	        uart->UartHandle.Init.Parity     = UART_PARITY_ODD;
	        break;
	    case PARITY_EVEN:
	        uart->UartHandle.Init.Parity     = UART_PARITY_EVEN;
	        break;
	    default:
	        uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
	        break;
	    }
	    if (HAL_UART_DeInit(&uart->UartHandle) != HAL_OK)
	    {
	        return RT_ERROR;
	    }
	    if (HAL_UART_Init(&uart->UartHandle) != HAL_OK)
	    {
	        return RT_ERROR;
	    }

	}
    return RT_EOK;
}

static struct stm32_uart uart1;
struct rt_serial_device serial1;
static struct stm32_uart uart3;
struct rt_serial_device serial3;

void UART_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
	rt_hw_serial_isr(&serial3, RT_SERIAL_EVENT_RX_DMADONE | 0x100);
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart *uart;
	static DMA_HandleTypeDef hdma_rx;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        __HAL_UART_DISABLE_IT(&uart->UartHandle, UART_IT_RXNE);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        __HAL_UART_ENABLE_IT(&uart->UartHandle, UART_IT_RXNE);
        break;
    case RT_DEVICE_CTRL_SET_DMARX:

#if 1
		rt_kprintf("in the control\r\n");
		__HAL_RCC_DMA1_CLK_ENABLE();
		
		hdma_rx.Instance = DMA1_Stream1;
		uart->UartHandle.RxXferSize = 1;
		uart->UartHandle.Instance->CR3 |= USART_CR3_DMAR;
		
		hdma_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
		hdma_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.Mode = DMA_NORMAL;
		hdma_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
		
		
		if (HAL_DMA_DeInit(&hdma_rx) != HAL_OK)
		{
			return RT_ERROR;
		}
		if (HAL_DMA_Init(&hdma_rx) != HAL_OK)
		{
			return RT_ERROR;
		}
		__HAL_LINKDMA(&uart->UartHandle, hdmarx, hdma_rx);
		uart->UartHandle.pRxBuffPtr = (uint8_t *)rt_malloc((uart->UartHandle.RxXferSize)*sizeof(uint8_t));
		  /* NVIC configuration for DMA transfer complete interrupt (USART6_RX) */
 		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, DMA1STREAM1_IRQ_PREEMPT, DMA1STREAM1_IRQ_SUB);
 		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
		HAL_UART_Receive_DMA(&uart->UartHandle,(uint8_t *)uart->UartHandle.pRxBuffPtr, uart->UartHandle.RxXferSize);
#endif
		
	    break;
    }

    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    while (!(uart->UartHandle.Instance->ISR & UART_FLAG_TXE));
    uart->UartHandle.Instance->TDR = c;

    return 1;
}
/*test code by he*/


static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    ch = -1;
    if (uart->UartHandle.Instance->ISR & UART_FLAG_RXNE)
    {
        ch = uart->UartHandle.Instance->RDR & 0xff;
    }
	/*test code by he*/
    return ch;
}

static rt_size_t stm32_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
	rt_uint8_t                  *pRxPtr, *data;
	struct stm32_uart *uart;
	
    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;
	if (direction == RT_SERIAL_DMA_RX)
	{
		pRxPtr = uart->UartHandle.pRxBuffPtr;
		data = buf;
		*data = 0;
		
		#if 0
		pRxPtr = pRxPtr + i;
		while(size)
		{
			*data = *(pRxPtr);
			if ()
			data++;
			i++;
			pRxPtr += i;
			size--;
			if (i>=uart->UartHandle.RxXferSize)
			i = 0;
		}
		#endif
		*data = *pRxPtr;
	}
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
    stm32_dma_transmit,
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */




void USART1_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart1;

    /* enter interrupt */
    rt_interrupt_enter();

    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(&uart->UartHandle, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
        rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_SEND_REQ(&uart->UartHandle, UART_RXDATA_FLUSH_REQUEST);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART3)
/* UART1 device driver structure */


void USART3_IRQHandler(void)
{
    struct stm32_uart *uart;
    uart = &uart3;

    /* enter interrupt */
    rt_interrupt_enter();
	
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(&uart->UartHandle, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
        rt_hw_serial_isr(&serial3, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_SEND_REQ(&uart->UartHandle, UART_RXDATA_FLUSH_REQUEST);
    }
	
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART3 */

void DMA1_Stream1_IRQHandler(void)
{
	
    struct stm32_uart *uart;
	
	/* enter interrupt */
    rt_interrupt_enter();
    uart = &uart3;
	
	HAL_DMA_IRQHandler((uart->UartHandle.hdmarx));
		
	/* leave interrupt */
	rt_interrupt_leave();
}

/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
	
    if (huart->Instance == USART1)
    {
		/* Enable GPIO TX/RX clock */
        USART1_TX_GPIO_CLK_ENABLE();
        USART1_RX_GPIO_CLK_ENABLE();
        /* Enable USARTx clock */
        USART1_CLK_ENABLE();

        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USART1_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = USART1_TX_AF;
        HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USART1_RX_PIN;
        GPIO_InitStruct.Alternate = USART1_RX_AF;
        HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);

        /* NVIC for USART */
		HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
        HAL_NVIC_SetPriority(USART1_IRQn, USART1_IRQ_PREEMPT ,USART1_TRQ_SUB);
       HAL_NVIC_EnableIRQ(USART1_IRQn);
	   //HAL_UART3_MspInit(huart);
    }
	
    if (huart->Instance == USART3)
    {
		/* Enable GPIO TX/RX clock */
        USART3_TX_GPIO_CLK_ENABLE();
        USART3_RX_GPIO_CLK_ENABLE();
        /* Enable USARTx clock */
        USART3_CLK_ENABLE();

        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USART3_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = USART3_TX_AF;
        HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USART3_RX_PIN;
        GPIO_InitStruct.Alternate = USART3_RX_AF;
        HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);

		HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
        HAL_NVIC_SetPriority(USART3_IRQn, USART3_IRQ_PREEMPT ,USART3_TRQ_SUB);
      	HAL_NVIC_EnableIRQ(USART3_IRQn);
		
        /* NVIC for USART */
		        UART_ENABLE_IRQ(USART3_IRQn);
        /* enable interrupt */
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    }
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* Disable the NVIC for UART */
		HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
        HAL_NVIC_DisableIRQ(USART1_IRQn);

		/* Reset peripherals */
        USART1_FORCE_RESET();
        USART1_RELEASE_RESET();

        /* Disable peripherals and GPIO Clocks */
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USART1_TX_GPIO_PORT, USART1_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USART1_RX_GPIO_PORT, USART1_RX_PIN);

    }
	//HAL_UART3_MspDeInit(huart);
	
    if (huart->Instance == USART3)
    {
        /* Disable the NVIC for UART */
		
		HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        UART_DISABLE_IRQ(USART3_IRQn);
        /* disable interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);


		/* Reset peripherals */
        USART3_FORCE_RESET();
        USART3_RELEASE_RESET();

        /* Disable peripherals and GPIO Clocks */
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USART3_TX_GPIO_PORT, USART3_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USART3_RX_GPIO_PORT, USART3_RX_PIN);

    }
}
void usart_rst()
{

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	stm32_configure(&serial1, &config);
	struct serial_configure config_uart3 = RT_SERIAL_CONFIG_UART3;
	stm32_configure(&serial3, &config);
}

int stm32_hw_usart_init(void)
{
    struct stm32_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef RT_USING_UART1
    uart = &uart1;
    uart->UartHandle.Instance = USART1;

    serial1.ops    = &stm32_uart_ops;
    serial1.config = config;

    /* register UART1 device */
    rt_hw_serial_register(&serial1, "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART3
		uart = &uart3;
		uart->UartHandle.Instance = USART3;
		struct serial_configure config_uart3 = RT_SERIAL_CONFIG_UART3;
		
		serial3.ops    = &stm32_uart_ops;
		serial3.config = config_uart3;
	
		/* register UART1 device */
		rt_hw_serial_register(&serial3, "uart3",
							  RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
							  uart);
#endif /* RT_USING_UART3 */


    return 0;
}
INIT_BOARD_EXPORT(stm32_hw_usart_init);
