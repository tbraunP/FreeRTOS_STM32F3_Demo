/*
 * gyroTask.c
 *
 *  Created on: 10.05.2013
 *      Author: pyro
 */
#include "FreeRTOS.h"
#include "task.h"
#include "gyroTask.h"
#include "hw/stm32f3_discovery_l3gd20.h"

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */

void decodeGyroRead(uint8_t* inputBuffer, float* pfData);

struct GyroState {
	L3GD20_InitTypeDef L3GD20_InitStructure;
	int waterMark;
	volatile int dataRead;

	// SPI DMA
	DMA_InitTypeDef spiRXDMA;
	DMA_InitTypeDef spiTXDMA;

	// result
	volatile uint8_t GyroIn[8];

	// State for DMA GyroReadOut
	volatile enum GyroReadOutState {
		SetInt2ToEmpty, FirstReadOut, ReadOut, SetInt2ToWatermark
	} GyroReadOutState;
} GyroState;

uint8_t Int2FifoEmpty[] = { L3GD20_CTRL_REG3_ADDR, 0x00 };
uint8_t RequestGyro[] = { (L3GD20_OUT_X_L_ADDR | READWRITE_CMD
		| MULTIPLEBYTE_CMD ), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void configureDMA() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	// configure TX DMA
	DMA_DeInit(DMA1_Channel3 );

	// DMA Configuration
	GyroState.spiTXDMA.DMA_BufferSize = 7;
	GyroState.spiTXDMA.DMA_MemoryBaseAddr = (uint32_t) & RequestGyro;
	GyroState.spiTXDMA.DMA_PeripheralBaseAddr = (uint32_t) & SPI1 ->DR;
	GyroState.spiTXDMA.DMA_DIR = DMA_DIR_PeripheralDST;
	GyroState.spiTXDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	GyroState.spiTXDMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
	GyroState.spiTXDMA.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	GyroState.spiTXDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	GyroState.spiTXDMA.DMA_Mode = DMA_Mode_Normal;
	GyroState.spiTXDMA.DMA_Priority = DMA_Priority_Low;
	GyroState.spiTXDMA.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel3, &GyroState.spiTXDMA);

	// DMA Configuration for receiver
	DMA_DeInit(DMA1_Channel2 );
	GyroState.spiRXDMA.DMA_BufferSize = 7;
	GyroState.spiRXDMA.DMA_MemoryBaseAddr = (uint32_t) & GyroState.GyroIn;
	//uart_state.uartTXDMA.DMA_MemoryBaseAddr = (uint32_t) value;
	GyroState.spiRXDMA.DMA_PeripheralBaseAddr = (uint32_t) & SPI1 ->DR;
	GyroState.spiRXDMA.DMA_DIR = DMA_DIR_PeripheralSRC;
	GyroState.spiRXDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	GyroState.spiRXDMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
	GyroState.spiRXDMA.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	GyroState.spiRXDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	GyroState.spiRXDMA.DMA_Mode = DMA_Mode_Normal;
	GyroState.spiRXDMA.DMA_Priority = DMA_Priority_Low;
	GyroState.spiRXDMA.DMA_M2M = DMA_M2M_Disable;

	// Enable DMA Finished Interrupt
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	DMA_ClearITPendingBit(DMA1_IT_TC2);
	DMA_Init(DMA1_Channel2, &GyroState.spiRXDMA);

	// Enable DMA Requests
	SPI_I2S_DMACmd(L3GD20_SPI, (SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx), ENABLE);

	// Enable DMA Finished Interrupt for reception
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x12;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configure the Mems to gyroscope application.
 * @param  None
 * @retval None
 */
void GyroConfig(void) {
	GyroState.waterMark = 10;

	/* Configure Mems L3GD20 */
	GyroState.L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	//L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	GyroState.L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_4;
	GyroState.L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	GyroState.L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	GyroState.L3GD20_InitStructure.BlockData_Update =
			L3GD20_BlockDataUpdate_Continous;
	GyroState.L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	GyroState.L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;
	GyroState.L3GD20_InitStructure.FifoMode = L3GD20_FIFO_STREAM;
	GyroState.L3GD20_InitStructure.FifoThreshold = GyroState.waterMark;
	L3GD20_Init(&GyroState.L3GD20_InitStructure);

	// Configure filter
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
	L3GD20_FilterStructure.HighPassFilter_Mode_Selection =
			L3GD20_HPM_AUTORESET_INT;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure);
	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE );

	// Set up and enable DMA Requests
	configureDMA();

	// Enable Interrupt for Int2 Watermark Threshold (data ready for readout)
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Connect EXTI0 Line to PA0 pin */
	SYSCFG_EXTILineConfig(L3GD20_SPI_INT2_EXTI_PORT_SOURCE,
			L3GD20_SPI_INT2_EXTI_PIN_SOURCE);

	/* Configure EXTI0 line */
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = L3GD20_SPI_INT2_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = L3GD20_SPI_INT2_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable Watermark Interrupt
	L3GD20_INT2InterruptCmd(L3GD20_INT2INTERRUPT_ENABLE_WTM );

	// Initiate Reading
	if (L3GD20_SPI_INT2_GPIO_PORT ->IDR & L3GD20_SPI_INT2_PIN) {
		NVIC_SetPendingIRQ(L3GD20_SPI_INT2_EXTI_IRQn);
	}
}

void EXTI1_IRQHandler() {
	L3GD20_CS_HIGH();
	//printf("Interupt\n");

	// Disable Interrupt
	NVIC_DisableIRQ(L3GD20_SPI_INT2_EXTI_IRQn);

	/* Clear the EXTI line 0 pending bit */
	EXTI_ClearITPendingBit(L3GD20_SPI_INT2_EXTI_LINE);

	L3GD20_CS_LOW();

	// start reading and writing
	GyroState.dataRead = GyroState.waterMark;

	// DMA Request: Always call DMA_Init and DMA_CMD
	DMA_Init(DMA1_Channel3, &GyroState.spiTXDMA);
	DMA_Init(DMA1_Channel2, &GyroState.spiRXDMA);

	// gogogo
	DMA_Cmd(DMA1_Channel2, ENABLE);
	DMA_Cmd(DMA1_Channel3, ENABLE);
}

void handleData() {
	static int i = 0;
	float measure[6];
	decodeGyroRead(&GyroState.GyroIn[1], measure);
	if (i == 150) {
		printf("Gyro %f, %f, %f\n", measure[0], measure[1], measure[2]);
		i = 0;
	}
	++i;
}

void DMA1_Channel2_IRQHandler(void) {
	L3GD20_CS_HIGH();

	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	DMA_ClearITPendingBit(DMA1_IT_TC2);

	// handle data
	handleData();

	if (GyroState.dataRead > 1) {
		--GyroState.dataRead;
		L3GD20_CS_LOW();
		// gogogo
		DMA_Init(DMA1_Channel3, &GyroState.spiTXDMA);
		DMA_Init(DMA1_Channel2, &GyroState.spiRXDMA);

		DMA_Cmd(DMA1_Channel2, ENABLE);
		DMA_Cmd(DMA1_Channel3, ENABLE);

		return;
	}

	// new request received, necessary since rising edge may have been skipped (since buffer was never that empty)?
	if (L3GD20_SPI_INT2_GPIO_PORT ->IDR & L3GD20_SPI_INT2_PIN) {
		//EXTI1_IRQHandler();
		NVIC_SetPendingIRQ(L3GD20_SPI_INT2_EXTI_IRQn);
	}
	NVIC_EnableIRQ(L3GD20_SPI_INT2_EXTI_IRQn);
}

void gyroTask(void *pvParameters) {
	printf("GyroTaks started\n");
	GyroConfig();

	int i = 0;
	float measure[3];

	for (;;) {
		// check for watermark
		if (L3GD20_SPI_INT2_GPIO_PORT ->IDR & L3GD20_SPI_INT2_PIN) {
//			Demo_GyroReadAngRate(measure);
			printf("DataReady\n");
//			++i;
		} else {
//			printf("%d - Gyro %f, %f, %f\n", i, measure[0], measure[1],
//					measure[2]);
//			i = 0;
			printf("Not ready");
		}
		vTaskDelay(1000);
	}
}

/**
 * @brief  Calculate the angular Data rate Gyroscope.
 * @param  pfData : Data out pointer
 * @retval None
 */
void Demo_GyroReadAngRate(float* pfData) {
	uint8_t tmpbuffer[6] = { 0 };

	L3GD20_Read(tmpbuffer, L3GD20_OUT_X_L_ADDR, 6);
	decodeGyroRead(tmpbuffer, pfData);
}

/**
 * @brief  Calculate the angular Data rate Gyroscope.
 * @param inputBuffer: RawData Input from Gyro
 * @param  pfData : Data out pointer
 * @retval None
 */
void decodeGyroRead(uint8_t* inputBuffer, float* pfData) {
	int16_t RawData[3] = { 0 };
	uint8_t tmpreg = 0;
	float sensitivity = 0;
	int i = 0;

	/* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
	if (!(tmpreg & 0x40)) {
		for (i = 0; i < 3; i++) {
			RawData[i] = (int16_t)(
					((uint16_t) inputBuffer[2 * i + 1] << 8)
							+ inputBuffer[2 * i]);
		}
	} else {
		for (i = 0; i < 3; i++) {
			RawData[i] = (int16_t)(
					((uint16_t) inputBuffer[2 * i] << 8)
							+ inputBuffer[2 * i + 1]);
		}
	}

	switch (GyroState.L3GD20_InitStructure.Full_Scale) {
	case L3GD20_FULLSCALE_250 :
		sensitivity = L3G_Sensitivity_250dps;
		break;
	case L3GD20_FULLSCALE_500 :
		sensitivity = L3G_Sensitivity_500dps;
		break;
	case L3GD20_FULLSCALE_2000 :
		sensitivity = L3G_Sensitivity_2000dps;
		break;
	}

	/* divide by sensitivity */
	for (i = 0; i < 3; i++) {
		pfData[i] = (float) RawData[i] / sensitivity;
	}
}

uint32_t L3GD20_TIMEOUT_UserCallback(void) {
	printf("L3DG20 Interrupt Timeout\n");
	return 0;
}
