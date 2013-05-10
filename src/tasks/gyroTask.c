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

static L3GD20_InitTypeDef L3GD20_InitStructure;

/**
 * @brief  Configure the Mems to gyroscope application.
 * @param  None
 * @retval None
 */
void GyroConfig(void) {
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

	/* Configure Mems L3GD20 */
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	//L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_4;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;
	L3GD20_InitStructure.FifoMode = L3GD20_FIFO_STREAM;
	L3GD20_InitStructure.FifoThreshold = 0xA;
	L3GD20_Init(&L3GD20_InitStructure);

	L3GD20_FilterStructure.HighPassFilter_Mode_Selection =
			L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure);

	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE );
	L3GD20_INT2InterruptCmd(L3GD20_INT2INTERRUPT_ENABLE_WTM );
}

void gyroTask(void *pvParameters) {
	printf("GyroTaks started\n");
	GyroConfig();

	int i=0;
	float measure[3];

	for (;;) {
		// check for watermark
		//if(1==1){
		if (L3GD20_SPI_INT2_GPIO_PORT->IDR & L3GD20_SPI_INT2_PIN) {
			Demo_GyroReadAngRate(measure);
			++i;
		} else {
			printf("%d - Gyro %f, %f, %f\n", i, measure[0], measure[1], measure[2]);
			i = 0;
			vTaskDelay(10);
		}
	}
}

/**
 * @brief  Calculate the angular Data rate Gyroscope.
 * @param  pfData : Data out pointer
 * @retval None
 */
void Demo_GyroReadAngRate(float* pfData) {
	uint8_t tmpbuffer[6] = { 0 };
	int16_t RawData[3] = { 0 };
	uint8_t tmpreg = 0;
	float sensitivity = 0;
	int i = 0;

	L3GD20_Read(tmpbuffer, L3GD20_OUT_X_L_ADDR, 6);

	/* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
	if (!(tmpreg & 0x40)) {
		for (i = 0; i < 3; i++) {
			RawData[i] = (int16_t)(
					((uint16_t) tmpbuffer[2 * i + 1] << 8) + tmpbuffer[2 * i]);
		}
	} else {
		for (i = 0; i < 3; i++) {
			RawData[i] = (int16_t)(
					((uint16_t) tmpbuffer[2 * i] << 8) + tmpbuffer[2 * i + 1]);
		}
	}

	switch (L3GD20_InitStructure.Full_Scale) {
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
