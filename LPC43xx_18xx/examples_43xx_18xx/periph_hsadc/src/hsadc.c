/*
 * @brief High speed ADC example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include <stdio.h>
#include "chip.h"
#include "string.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* The default example uses DMA. To use the example in an interrutp based
   configuration, enable the following definition. */
#define USE_INTERRUPT_MODE

/* HSADC clock rate used for sampling */
#define HSADC_CLOCK_RATE (80 * 1000000)

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 128	/* Send */
#define UART_RRB_SIZE 32	/* Receive */

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

#if defined(BOARD_NXP_LPCLINK2_4370)
#define LPC_UARTX       LPC_USART2
#define UARTx_IRQn      USART2_IRQn
#define UARTx_IRQHandler UART2_IRQHandler
#endif

/* Indicate USART2 data trans */
bool isTrans = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Last saved ADC sample for each input */
volatile uint32_t lastSample[8];

/* DMA variables */
static uint8_t dmaChannelNum;
/* Buffer locate in AHB SRAM */
static uint32_t adchsBuffer[128] __attribute__((at(0x20000000)));


/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Setting up the HSADC clock to 80MHz */
static void HSADC_ClockSetup(void)
{
	Chip_USB0_Init(); /* Initialize the USB0 PLL to 480 MHz */
    Chip_Clock_SetDivider(CLK_IDIV_A, CLKIN_USBPLL, 2); /* Source DIV_A from USB0PLL, and set divider to 2 (Max div value supported is 4) [IN 480 MHz; OUT 240 MHz */
    Chip_Clock_SetDivider(CLK_IDIV_B, CLKIN_IDIVA, 3); /* Source DIV_B from DIV_A, [IN 240 MHz; OUT 80 MHz */
    Chip_Clock_SetBaseClock(CLK_BASE_ADCHS, CLKIN_IDIVB, true, false); /* Source ADHCS base clock from DIV_B */
}

/* HSADC Initialization */
static void HSADC_Init(void)
{
	uint32_t freqHSADC = 0;
	
	/************************ HSADC时钟初始化 ************************/
	/*Setting up the HSADC clock to 80MHz*/
	HSADC_ClockSetup();
	
	/* Initialize HSADC */
	Chip_HSADC_Init(LPC_ADCHS);

	/* Show the actual HSADC clock rate */
//	freqHSADC = Chip_HSADC_GetBaseClockRate(LPC_ADCHS);
//	DEBUGOUT("HSADC sampling rate = %dKHz\r\n", freqHSADC / 1000);

	
	/************************ HSADC参数配置 ************************/
	/* Setup FIFO trip points for interrupt/DMA to 8 samples, packing */
	Chip_HSADC_SetupFIFO(LPC_ADCHS, 8, true);

	/* Software trigger only, 0x90 recovery clocks, add channel ID to FIFO entry */
	Chip_HSADC_ConfigureTrigger(LPC_ADCHS, HSADC_CONFIG_TRIGGER_SW,
								HSADC_CONFIG_TRIGGER_RISEEXT, HSADC_CONFIG_TRIGGER_NOEXTSYNC,
								HSADC_CHANNEL_ID_EN_ADD, 0x90);

	/* Select both positive and negative DC biasing for input 0 */
	Chip_HSADC_SetACDCBias(LPC_ADCHS, 0, HSADC_CHANNEL_DCBIAS, HSADC_CHANNEL_NODCBIAS);

	/* Setup data format for 2's complement and update clock settings. This function
	   should be called whenever a clock change is made to the HSADC */
	Chip_HSADC_SetPowerSpeed(LPC_ADCHS, true);

	/*  Select Table 0 desccriptor 0 */
	Chip_HSADC_SetActiveDescriptor(LPC_ADCHS, 0, 0);


	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 0, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_SWAP | HSADC_DESC_MATCH(0x95) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 0, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 1, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 2, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 3, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 4, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 5, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 6, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 7, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_FIRST | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_UPDATE_TABLE));


	/* Update descriptor tables - needed after updating any descriptors */
	Chip_HSADC_UpdateDescTable(LPC_ADCHS, 0);
	Chip_HSADC_UpdateDescTable(LPC_ADCHS, 1);


	/*Flush FIFO*/
	Chip_HSADC_FlushFIFO(LPC_ADCHS);

	/* Enable HSADC power */
	Chip_HSADC_EnablePower(LPC_ADCHS);
}

static void HSADC_TransOnce(void)
{
	/* Flush FIFO */
	Chip_HSADC_FlushFIFO(LPC_ADCHS);
	
	/* Trig Trans */
	Chip_HSADC_SWTrigger(LPC_ADCHS);
}

/* GPDMA Initialization */
static void GPDMA_Init(void)
{
	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);
	
	dmaChannelNum = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_ADCHS_Read);
	Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
						GPDMA_CONN_ADCHS_Read,
					  (uint32_t) &adchsBuffer[0],
					  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
					  sizeof(adchsBuffer)/sizeof(adchsBuffer[0]));
}

/* USART2 Initialization */
static void USART2_Init(void)
{
	Board_UART_Init(LPC_UARTX);

	/* Setup UART for 9.6K8N1 */
	Chip_UART_Init(LPC_UARTX);
	Chip_UART_SetBaud(LPC_UARTX, 9600);
	Chip_UART_ConfigData(LPC_UARTX, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_UARTX, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_UARTX);

	/* Before using the ring buffers, initialize them using the ring
	   buffer init function */
	RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

	/* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
	Chip_UART_SetupFIFOS(LPC_UARTX, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |
							UART_FCR_TX_RS | UART_FCR_TRG_LEV2));

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(LPC_UARTX, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(UARTx_IRQn, 1);
	NVIC_EnableIRQ(UARTx_IRQn);
}

static void delay_ms(uint16_t time)
{
	int i,j;
	
	for(i=0; i<time; i++)
	{
		j = 204000;
		while(j--);
	}
	
}

static void Lora_Trans(char *data, uint16_t size)
{
	char head[3] = {0x00, 0x01, 0x17};			//LoRa address, channel
	int i,j;
	
	for(i=0; i<size; i+=256)
	{
		for(j=0; j<3; j++)
		{
			while ((Chip_UART_ReadLineStatus(LPC_UARTX) & UART_LSR_THRE) == 0);
			Chip_UART_SendByte(LPC_UARTX, head[j]);
		}
		for(j=i; j<i+256; j++)
		{
			while ((Chip_UART_ReadLineStatus(LPC_UARTX) & UART_LSR_THRE) == 0);
			Chip_UART_SendByte(LPC_UARTX, data[j]);
		}
		
		delay_ms(1000);
	}
	
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
void DMA_IRQHandler(void)
{
	/*Change Disciptor to stop the ADCHS*/
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 1, 0, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_NONE |
												HSADC_DESC_RESET_TIMER | HSADC_DESC_HALT | HSADC_DESC_UPDATE_TABLE));
	
	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChannelNum) == SUCCESS) {
//		DEBUGOUT("DMA Transformation Success!\r\n");
	}
	else{
		DEBUGOUT("DMA Transformation Fail!\r\n");
	}
	
	/* Shutdown HSADC when done */
	Chip_HSADC_DeInit(LPC_ADCHS);
	
	isTrans = 1;
}

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void UARTx_IRQHandler(void)
{
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */
	Chip_UART_IRQRBHandler(LPC_UARTX, &rxring, &txring);
}



/**
 * @brief	main routine for HSADC example
 * @return	Function should not exit
 */
int main(void)
{
	int i;
	uint8_t key;
	int bytes;

	/************************ 板级初始化（时钟更新、GPIO、UART初始化） ************************/
	SystemCoreClockUpdate();
	Board_Init();
	
	/* Initialize USART2 */
	USART2_Init();
	

	/************************ 应用部分 ************************/
	/*Clearing the adchsBuffer*/
	for(i=0; i<sizeof(adchsBuffer)/sizeof(adchsBuffer[0]); i++)
	{
		adchsBuffer[i] = 0;
	}
	

	/* Poll the receive ring buffer for the ESC (ASCII 27) key */
	key = 0;
	while (1) {
		__WFI();
		bytes = Chip_UART_ReadRB(LPC_UARTX, &rxring, &key, 1);
		if (bytes > 0) {
			/* Wrap value back around */
			Chip_UART_SendRB(LPC_UARTX, &txring, (const uint8_t *) &key, 1);
			if('a' == key)
			{
				/* Initialize HSADC */
				HSADC_Init();
				/* Initialize GPDMA */
				GPDMA_Init();
				HSADC_TransOnce();		
			}
		}
		
		/* Trans sample data */
		if(1 == isTrans)
		{
//			Lora_Trans((char *)adchsBuffer, sizeof(adchsBuffer));
			Lora_Trans((char *)adchsBuffer, 128*4);			
			isTrans = 0;
		}
	}
	
}






