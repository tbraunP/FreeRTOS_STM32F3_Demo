/*
 * gyroTask.h
 *
 *  Created on: 10.05.2013
 *      Author: pyro
 */

#ifndef GYROTASK_H_
#define GYROTASK_H_
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

void Demo_GyroReadAngRate(float* pfData);

void gyroTask(void *pvParameters);

void EXTI1_IRQHandler();
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* GYROTASK_H_ */
