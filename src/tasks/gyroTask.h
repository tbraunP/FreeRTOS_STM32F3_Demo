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

void Demo_GyroReadAngRate(float* pfData);

void gyroTask(void *pvParameters);

#endif /* GYROTASK_H_ */
