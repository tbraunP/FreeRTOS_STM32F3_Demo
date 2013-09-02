#include "uart.h"
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_dma.h"
#include "ringbuf.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include "FreeRTOS.h"
#include "semphr.h"

#define RX_SIZE  256
#define TX_SIZE  256

struct uart_state {
	ringbuf_t rx_buf;
	ringbuf_t tx_buf;

	volatile struct uart_stats {
		uint32_t rx_overrun;
		uint32_t rx_bytes;
		uint32_t tx_bytes;
		uint32_t tx_overrun;
	} uart_stats;

	DMA_InitTypeDef uartTXDMA;
	volatile uint8_t dmaRunning;
} uart_state;

void uart_base_init(void) {
	rb_alloc(&uart_state.rx_buf, RX_SIZE);
	rb_alloc(&uart_state.tx_buf, TX_SIZE);
	uart_state.dmaRunning = 0;
}

int uart_chars_avail(void) {
	return uart_state.rx_buf.len;
}

void startDMA();

ssize_t uart_write_r(struct _reent *r, int fd, const void *ptr, size_t len) {
	int ln = 0;

    vPortEnterCritical();
    {
		ln = rb_write(&uart_state.tx_buf, (const uint8_t*) ptr, len);
		if (ln != len) {
			++uart_state.uart_stats.tx_overrun;
		}

		if (!uart_state.dmaRunning && ln > 0) {
            startDMA();
		}
    }
	vPortExitCritical();

	return ln;
}

ssize_t uart_read_r(struct _reent *r, int fd, void *ptr, size_t len) {
//	while (!rx_buf.len)
//		;
//
//	if (len > rx_buf.len)
//		len = rx_buf.len;
//
//	char *c = (char*) ptr;
//	for (int i = 0; i < len; i++)
//		rb_getc(&rx_buf, c++);
//
//	return len;
	return 0;
}

void uart_poll_send(const char *ch) {
	while (*ch) {
		USART_SendData(UART4, *ch);
		ch++;
	}
}

/**
 * Initialize UART.
 *
 * \param  baudrate  Baudrate
 *
 *  PC10   USART1_TXD
 *  PC11   USART1_RXD
 *
 */
void uart_init(int baudrate) {
	uart_base_init();

	/* Enable Clock and configure Pinconfig */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_5 );
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_5 );

	GPIO_InitTypeDef GPIO_InitStructure =
			{ .GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11, .GPIO_Speed =
					GPIO_Speed_50MHz, .GPIO_Mode = GPIO_Mode_AF, .GPIO_OType =
					GPIO_OType_PP, .GPIO_PuPd = GPIO_PuPd_UP };

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Initialize Serial Port
	USART_DeInit(UART4 );

	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = baudrate;

	USART_Init(UART4, &USART_InitStructure);

	// configure TX DMA
	DMA_DeInit(DMA2_Channel5 );

	// DMA Configuration
	uart_state.uartTXDMA.DMA_PeripheralBaseAddr = (uint32_t) & UART4 ->TDR;
	uart_state.uartTXDMA.DMA_MemoryBaseAddr = (uint32_t) NULL;
    uart_state.uartTXDMA.DMA_DIR = DMA_DIR_PeripheralDST;
	uart_state.uartTXDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	uart_state.uartTXDMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
	uart_state.uartTXDMA.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	uart_state.uartTXDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	uart_state.uartTXDMA.DMA_Mode = DMA_Mode_Normal;
	uart_state.uartTXDMA.DMA_Priority = DMA_Priority_Low;
	uart_state.uartTXDMA.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA2_Channel5, &uart_state.uartTXDMA);

	// Enable DMA Finished Interrupt
	DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);
	DMA_ClearITPendingBit(DMA2_IT_TC5 );

	// enable DMA requests
	USART_DMACmd(UART4, (USART_DMAReq_Tx | USART_DMAReq_Rx ), ENABLE);

	USART_Cmd(UART4, ENABLE);

	// Enable Interrupts
	NVIC_Init(&(NVIC_InitTypeDef ) { .NVIC_IRQChannel = DMA2_Channel5_IRQn,
					.NVIC_IRQChannelPreemptionPriority = 14,
					.NVIC_IRQChannelSubPriority = 0, .NVIC_IRQChannelCmd =
							ENABLE });

	// transmit channel (clear tc (must be 1 if transmission has been completed!))
	//USART_ClearITPendingBit(UART4, USART_IT_TC );
	//DMA_Cmd(DMA2_Channel5, ENABLE);
}

void DMA2_Channel5_IRQHandler() {
	DMA_Cmd(DMA2_Channel5, DISABLE);
	DMA_ClearITPendingBit(DMA2_IT_TC5 );

	// ready for next transmission
	USART_ClearITPendingBit(UART4, USART_IT_TC );

	// free current memory
    vPortEnterCritical();
	free((uint8_t*) uart_state.uartTXDMA.DMA_MemoryBaseAddr);
    uart_state.uartTXDMA.DMA_MemoryBaseAddr = (uint32_t) NULL;

	// check for new transmission
	if (uart_state.tx_buf.len > 0) {
        startDMA();
	}else{
       uart_state.dmaRunning = 0;
    }

	vPortExitCritical();
}


void startDMA(){
	// copy message
    int len = uart_state.tx_buf.len;
	uint8_t* str = (uint8_t*) malloc(len * sizeof(uint8_t));
	len = rb_read(&uart_state.tx_buf, str, len);
	uart_state.uartTXDMA.DMA_MemoryBaseAddr = (uint32_t) str;
	uart_state.uartTXDMA.DMA_BufferSize = len;

	// start transfer
	// DMA Request: Always call DMA_Init and DMA_CMD
	DMA_Init(DMA2_Channel5, &uart_state.uartTXDMA);
	DMA_Cmd(DMA2_Channel5, ENABLE);
	uart_state.dmaRunning = 1;
}


/*
 // -------------------- Shell commands --------------------
 //
 static void cmd_baudrate(int argc, char *argv[])
 {
 if (argc != 2) {
 printf("usage: %s <baudrate>\n", argv[0]);
 return;
 }

 uart_init(atoi(argv[1]));
 }

 static void cmd_uart_stats(void)
 {
 printf("rx_bytes:   %8lu\n", uart_stats.rx_bytes);
 printf("rx_overrun: %8lu\n", uart_stats.rx_overrun);
 printf("tx_bytes:   %8lu\n", uart_stats.tx_bytes);
 }


 SHELL_CMD(baudrate,   (cmdfunc_t)cmd_baudrate,   "set baudrate")
 SHELL_CMD(uart_stats, (cmdfunc_t)cmd_uart_stats, "show UART statistics")
 */
