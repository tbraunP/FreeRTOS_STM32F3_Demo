/*
 * accTask.c
 *
 *  Created on: 11.05.2013
 *      Author: pyro
 */
#include "accTask.h"
#include "hw/stm32f3_discovery_lsm303dlhc.h"

void Demo_CompassConfig(void);
void Demo_CompassReadMag(float* pfData);
void Demo_CompassReadAcc(float* pfData);

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

struct ACCState {
	LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
	LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
	int watermark;
} ACCState;

void accTask(void *pvParameters) {
	printf("ACCTask started\n");
//	Demo_CompassConfig();

	int i = 0;
	float measure[3];

	for (;;) {
		printf("ACCTask working\n");
		// check for watermark
//		Demo_CompassReadAcc(measure);

//		if (L3GD20_SPI_INT2_GPIO_PORT->IDR & L3GD20_SPI_INT2_PIN) {
//			Demo_GyroReadAngRate(measure);
//			printf("DataReady\n");
//			++i;
//		} else {
//		printf("ACC %f, %f, %f\n", i, measure[0], measure[1], measure[2]);
//			i = 0;
//		printf("Not ready");
//		}
		vTaskDelay(5000);
	}
}

/**
 * @brief  Configure the Mems to compass application.
 * @param  None
 * @retval None
 */
void Demo_CompassConfig(void) {

	ACCState.watermark = 10;

	/* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
	ACCState.LSM303DLHC_InitStructure.Temperature_Sensor =
			LSM303DLHC_TEMPSENSOR_DISABLE;
	ACCState.LSM303DLHC_InitStructure.MagOutput_DataRate =
			LSM303DLHC_ODR_220_HZ ;
	ACCState.LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
	ACCState.LSM303DLHC_InitStructure.Working_Mode =
			LSM303DLHC_CONTINUOS_CONVERSION;
	LSM303DLHC_MagInit(&ACCState.LSM303DLHC_InitStructure);

	/* Fill the accelerometer structure */
	ACCState.LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
	ACCState.LSM303DLHCAcc_InitStructure.AccOutput_DataRate =
			LSM303DLHC_ODR_400_HZ;
	ACCState.LSM303DLHCAcc_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
	ACCState.LSM303DLHCAcc_InitStructure.AccFull_Scale =
			LSM303DLHC_FULLSCALE_8G;
	ACCState.LSM303DLHCAcc_InitStructure.BlockData_Update =
			LSM303DLHC_BlockUpdate_Continous;
	ACCState.LSM303DLHCAcc_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
	ACCState.LSM303DLHCAcc_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;
	ACCState.LSM303DLHCAcc_InitStructure.FifoMode = LSM303DLHC_FIFO_STREAM;
	ACCState.LSM303DLHCAcc_InitStructure.FifoThreshold = ACCState.watermark;

	/* Configure the accelerometer main parameters */
	LSM303DLHC_AccInit(&ACCState.LSM303DLHCAcc_InitStructure);

	/* Fill the accelerometer LPF structure */
	LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;
	LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =
			LSM303DLHC_HPM_AUTORESET_INT;
	LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency =
			LSM303DLHC_HPFCF_16;
	LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 =
			LSM303DLHC_HPF_AOI1_DISABLE;
	LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 =
			LSM303DLHC_HPF_AOI2_DISABLE;

	/* Configure the accelerometer LPF main parameters */
	LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);

	// Enable Watermark Interrupt
	LSM303DLHC_AccIT1Config(LSM303DLHC_IT1_WTM, ENABLE);
}

/**
 * @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit reppresentation)
 * @param pnData: pointer to float buffer where to store data
 * @retval None
 */
void Demo_CompassReadAcc(float* pfData) {
	int16_t pnRawData[3];
	uint8_t ctrlx[2];
	uint8_t buffer[6], cDivider;

	float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;

	switch (ACCState.LSM303DLHCAcc_InitStructure.AccFull_Scale) {
	case LSM303DLHC_FULLSCALE_8G :
		LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
		break;
	case LSM303DLHC_FULLSCALE_16G :
		LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
		break;
	case LSM303DLHC_FULLSCALE_4G :
		LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
		break;
	case LSM303DLHC_FULLSCALE_2G :
		LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
		break;
	}

	/* Read the register content */
	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

	if (ACCState.LSM303DLHCAcc_InitStructure.FifoMode
			!= LSM303DLHC_FIFO_BYPASS_MODE ) {
		cDivider = 16;
	} else {
		cDivider = 64;
	}

	/* check in the control register4 the data alignment*/
	/* Little Endian Mode or FIFO mode */
	if (ACCState.LSM303DLHCAcc_InitStructure.FifoMode
			!= LSM303DLHC_FIFO_BYPASS_MODE
			|| ACCState.LSM303DLHCAcc_InitStructure.Endianness
					== LSM303DLHC_BLE_LSB ) {
		for (int i = 0; i < 3; i++) {
			pnRawData[i] = ((int16_t)((uint16_t) buffer[2 * i + 1] << 8)
					+ buffer[2 * i]) / cDivider;
		}
	} else /* Big Endian Mode */
	{
		for (int i = 0; i < 3; i++)
			pnRawData[i] = ((int16_t)((uint16_t) buffer[2 * i] << 8)
					+ buffer[2 * i + 1]) / cDivider;
	}
//	/* Read the register content */
//	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx, 2);
//
//	if (ctrlx[1] & 0x40) {
//		/* FIFO mode */
//		LSM_Acc_Sensitivity = 0.25;
//	} else {
//		/* normal mode */
//		/* switch the sensitivity value set in the CRTL4*/
//		switch (ctrlx[0] & 0x30) {
//		case LSM303DLHC_FULLSCALE_2G :
//			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
//			break;
//		case LSM303DLHC_FULLSCALE_4G :
//			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
//			break;
//		case LSM303DLHC_FULLSCALE_8G :
//			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
//			break;
//		case LSM303DLHC_FULLSCALE_16G :
//			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
//			break;
//		}
//	}

	/* Obtain the mg value for the three axis */
	for (int i = 0; i < 3; i++) {
		pfData[i] = (float) pnRawData[i] / LSM_Acc_Sensitivity;
	}

}

/**
 * @brief  calculate the magnetic field Magn.
 * @param  pfData: pointer to the data out
 * @retval None
 */
void Demo_CompassReadMag(float* pfData) {
	static uint8_t buffer[6] = { 0 };
	uint8_t CTRLB = 0;
	uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
	uint8_t i = 0;
	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer + 1, 1);
	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer + 2, 1);
	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer + 3, 1);
	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer + 4, 1);
	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer + 5, 1);
	/* Switch the sensitivity set in the CRTLB*/
	switch (CTRLB & 0xE0) {
	case LSM303DLHC_FS_1_3_GA :
		Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
		Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
		break;
	case LSM303DLHC_FS_1_9_GA :
		Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
		Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
		break;
	case LSM303DLHC_FS_2_5_GA :
		Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
		Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
		break;
	case LSM303DLHC_FS_4_0_GA :
		Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
		Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
		break;
	case LSM303DLHC_FS_4_7_GA :
		Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
		Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
		break;
	case LSM303DLHC_FS_5_6_GA :
		Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
		Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
		break;
	case LSM303DLHC_FS_8_1_GA :
		Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
		Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
		break;
	}

	for (i = 0; i < 2; i++) {
		pfData[i] = (float) ((int16_t)(
				((uint16_t) buffer[2 * i] << 8) + buffer[2 * i + 1]) * 1000)
				/ Magn_Sensitivity_XY;
	}
	pfData[2] = (float) ((int16_t)(((uint16_t) buffer[4] << 8) + buffer[5])
			* 1000) / Magn_Sensitivity_Z;
}

uint32_t LSM303DLHC_TIMEOUT_UserCallback() {
	printf("LSM303 Timeout\n");
}
