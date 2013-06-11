/*
 * motorControlTask.c
 *
 *  Created on: 11.06.2013
 *      Author: braun
 */
#include "motorControlTask.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hw/pwm.h"

#include <stdio.h>
#include <stdint.h>

void initMotorControl() {
	PWM_Init();
}

void motorControlTask(void *pvParameters) {
	initMotorControl();

	printf("PWM running\n");

	// set timer values
	//TIM2->CCR1 = 0;
	//TIM2->CCR2 = 0;
	//TIM2->CCR3 = 0;
	//TIM2->CCR4 = 0;

	while (1) {
		vTaskDelay(5000);
	}
}

