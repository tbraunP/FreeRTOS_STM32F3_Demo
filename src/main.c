#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "hw/uart.h"
#include "ansi.h"
#include "ustime.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "tasks/gyroTask.h"
//#include "tasks/accTask.h"
#include "tasks/accTaskOrig.h"

/**
 *
 */
#include "hw/stm32f3_discovery_lsm303dlhc.h"
#include "hw/stm32f3_discovery_l3gd20.h"

#define ABS(x)         (x < 0) ? (-x) : x

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

struct task_param {
	char *name;
	int interval;
	uint8_t led;
};

static void fpu_task(void *pvParameters) {
	//LED_On(LED_BLUE);
	// Force x to stay in a FPU reg.
	//
	//register float x = 0;
	struct task_param *p = pvParameters;

	for (;;) {
		STM_EVAL_LEDToggle(p->led);
		vTaskDelay(p->interval);
		//x*=2.1;
	}

}

static void init_tasks(void *pvParameters) {
	for (int i = 0; i < 8; i++)
		STM_EVAL_LEDInit(i);

	//uart_init(115200);
	//printf(ANSI_FG_LTRED "STM32F407" ANSI_NORMAL " FreeRTOS Test\n" );

	vTaskDelay(100);

	for (int i = 0; i < 5; i++) {
		//printf("Starting FPU task %d..\n", i);

		struct task_param *p;

		p = malloc(sizeof(struct task_param));
		p->name = malloc(16);
		p->interval = (i + 1) * 500;
		p->led = i;
		sprintf(p->name, "FPU_%d", i);
		STM_EVAL_LEDToggle(LED10);

		xTaskCreate(fpu_task, (int8_t* )p->name, 1024, p, tskIDLE_PRIORITY+1,
				NULL);
		vTaskDelay(1500);
	}

	for (;;)
		;
}

static void init_task0(void *pvParameters) {
	for (;;) {
		STM_EVAL_LEDToggle(LED8);
		vTaskDelay(500);
		//printf("Hallo Welt\n");
	}
}

static void init_task1(void *pvParameters) {
	for (;;) {
		STM_EVAL_LEDToggle(LED7);
		vTaskDelay(500);
	}
}

static void init_task2(void *pvParameters) {
	for (;;) {
		STM_EVAL_LEDToggle(LED10);
		vTaskDelay(500);
	}
}

static void init_task(void *pvParameters) {
	for (int i = 0; i < 8; i++)
		STM_EVAL_LEDInit(i);

	xTaskCreate(init_task0, (signed char* )"init0", 128, NULL, 2, NULL);
	//xTaskCreate(init_task1, (signed char* )"init1", 512, NULL, 2, NULL);
	//xTaskCreate(init_task2, (signed char* )"init2", 512, NULL, 2, NULL);
	//xTaskCreate(accTask, (signed char* )"ACCTask", 1024, NULL, 3, NULL);
	xTaskCreate(gyroTask, (signed char* )"GyroTask", 1024, NULL, 3, NULL);

	for (;;) {
		STM_EVAL_LEDToggle(LED9);
		vTaskDelay(500);
	}
}

//int main(void) {
//	// FreeRTOS assumes 4 preemption- and 0 subpriority-bits
//	//
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//
//	uart_init(115200);
//
//	for (int i = 0; i < 8; i++)
//		STM_EVAL_LEDInit(i);
//
//	// Create init task and start the scheduler
//	//
//	//xTaskCreate(init_task, (signed char* )"init", 1024, NULL, 2, NULL);
//	//xTaskCreate(init_task3, (signed char*)"init3", 1024, NULL, 3, NULL);
//	//xTaskCreate(init_task2, (signed char*)"init2", 1024, NULL, tskIDLE_PRIORITY, NULL);
//	xTaskCreate(init_task0, (signed char* )"init0", 128, NULL, 2, NULL);
//	//xTaskCreate(accTask, (signed char* )"ACCTask", 1024, NULL, 3, NULL);
//	xTaskCreate(gyroTask, (signed char* )"GyroTask", 1024, NULL, 3, NULL);
//	xTaskCreate(accTaskOrig, (signed char* )"ACCTaskOrig", 1024, NULL, 3, NULL);
//
//	vTaskStartScheduler();
//}

/**
 * @brief  Configure the Mems to compass application.
 * @param  None
 * @retval None
 */
void Demo_CompassConfig(void) {
	LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
	LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
	LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

	/* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
	LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
	LSM303DLHC_InitStructure.MagOutput_DataRate = LSM303DLHC_ODR_30_HZ;
	LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
	LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
	LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

	/* Fill the accelerometer structure */
	LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
	LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
	LSM303DLHCAcc_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
	LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
	LSM303DLHCAcc_InitStructure.BlockData_Update =
			LSM303DLHC_BlockUpdate_Continous;
	LSM303DLHCAcc_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
	LSM303DLHCAcc_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;
	/* Configure the accelerometer main parameters */
	LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

	/* Fill the accelerometer LPF structure */
	LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =
			LSM303DLHC_HPM_NORMAL_MODE;
	LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency =
			LSM303DLHC_HPFCF_16;
	LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 =
			LSM303DLHC_HPF_AOI1_DISABLE;
	LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 =
			LSM303DLHC_HPF_AOI2_DISABLE;

	/* Configure the accelerometer LPF main parameters */
	LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

/**
 * @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
 * @param pnData: pointer to float buffer where to store data
 * @retval None
 */
void Demo_CompassReadAcc(float* pfData) {
	int16_t pnRawData[3];
	uint8_t ctrlx[2];
	uint8_t buffer[6], cDivider;
	uint8_t i = 0;
	float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

	/* Read the register content */
	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx, 2);
	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

	if (ctrlx[1] & 0x40)
		cDivider = 64;
	else
		cDivider = 16;

	/* check in the control register4 the data alignment*/
	if (!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
	{
		for (i = 0; i < 3; i++) {
			pnRawData[i] = ((int16_t)((uint16_t) buffer[2 * i + 1] << 8)
					+ buffer[2 * i]) / cDivider;
		}
	} else /* Big Endian Mode */
	{
		for (i = 0; i < 3; i++)
			pnRawData[i] = ((int16_t)((uint16_t) buffer[2 * i] << 8)
					+ buffer[2 * i + 1]) / cDivider;
	}
	/* Read the register content */
	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx, 2);

	if (ctrlx[1] & 0x40) {
		/* FIFO mode */
		LSM_Acc_Sensitivity = 0.25;
	} else {
		/* normal mode */
		/* switch the sensitivity value set in the CRTL4*/
		switch (ctrlx[0] & 0x30) {
		case LSM303DLHC_FULLSCALE_2G :
			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
			break;
		case LSM303DLHC_FULLSCALE_4G :
			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
			break;
		case LSM303DLHC_FULLSCALE_8G :
			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
			break;
		case LSM303DLHC_FULLSCALE_16G :
			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
			break;
		}
	}

	/* Obtain the mg value for the three axis */
	for (i = 0; i < 3; i++) {
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

RCC_ClocksTypeDef RCC_Clocks;
float MagBuffer[3] = { 0.0f }, AccBuffer[3] = { 0.0f }, Buffer[3] = { 0.0f };
float fNormAcc, fSinRoll, fCosRoll, fSinPitch, fCosPitch = 0.0f, RollAng = 0.0f,
		PitchAng = 0.0f;
float fTiltedX, fTiltedY = 0.0f, HeadingValue = 0.0f;

void accTask(void *pvParameters) {
	while (1) {
//		/* Wait for data ready */
//		printf("Starting wait..");
//
//		// delay
//		STM_EVAL_LEDToggle(LED9);
//		vTaskDelay(0);
//
//		printf("..done\n");

		/* Read Compass data */
		Demo_CompassReadMag(MagBuffer);
		Demo_CompassReadAcc(AccBuffer);

		for (int i = 0; i < 3; i++)
			AccBuffer[i] /= 100.0f;

		fNormAcc = sqrt(
				(AccBuffer[0] * AccBuffer[0]) + (AccBuffer[1] * AccBuffer[1])
						+ (AccBuffer[2] * AccBuffer[2]));

		fSinRoll = -AccBuffer[1] / fNormAcc;
		fCosRoll = sqrt(1.0 - (fSinRoll * fSinRoll));
		fSinPitch = AccBuffer[0] / fNormAcc;
		fCosPitch = sqrt(1.0 - (fSinPitch * fSinPitch));
		if (fSinRoll > 0) {
			if (fCosRoll > 0) {
				RollAng = acos(fCosRoll) * 180 / PI;
			} else {
				RollAng = acos(fCosRoll) * 180 / PI + 180;
			}
		} else {
			if (fCosRoll > 0) {
				RollAng = acos(fCosRoll) * 180 / PI + 360;
			} else {
				RollAng = acos(fCosRoll) * 180 / PI + 180;
			}
		}

		if (fSinPitch > 0) {
			if (fCosPitch > 0) {
				PitchAng = acos(fCosPitch) * 180 / PI;
			} else {
				PitchAng = acos(fCosPitch) * 180 / PI + 180;
			}
		} else {
			if (fCosPitch > 0) {
				PitchAng = acos(fCosPitch) * 180 / PI + 360;
			} else {
				PitchAng = acos(fCosPitch) * 180 / PI + 180;
			}
		}

		if (RollAng >= 360) {
			RollAng = RollAng - 360;
		}

		if (PitchAng >= 360) {
			PitchAng = PitchAng - 360;
		}

		fTiltedX = MagBuffer[0] * fCosPitch + MagBuffer[2] * fSinPitch;
		fTiltedY = MagBuffer[0] * fSinRoll * fSinPitch + MagBuffer[1] * fCosRoll
				- MagBuffer[1] * fSinRoll * fCosPitch;

		HeadingValue = (float) ((atan2f((float) fTiltedY, (float) fTiltedX))
				* 180) / PI;

		if (HeadingValue < 0) {
			HeadingValue = HeadingValue + 360;
		}

		printf("ACC: Roll: %f, Pitch: %f, Yaw: %f\n", RollAng, PitchAng, HeadingValue);
		vTaskDelay(500);

	}
}

int main(void) {
	// FreeRTOS assumes 4 preemption- and 0 subpriority-bits
	//
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	uart_init(115200);

	for (int i = 0; i < 8; i++)
		STM_EVAL_LEDInit(i);

	Demo_CompassConfig();

	// start freeRTOS
	xTaskCreate(accTask, (signed char* )"accTask", 1024, NULL, 3, NULL);
	vTaskStartScheduler();
}

/**
 * @brief  Basic management of the timeout situation.
 * @param  None.
 * @retval None.
 */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void) {
	return 0;
}

void assert_failed(uint8_t* file, uint32_t line) {
}
