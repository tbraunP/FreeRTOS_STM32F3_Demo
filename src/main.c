#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "uart.h"
#include "ansi.h"
#include "ustime.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>


struct task_param {
    char *name;
    int   interval;
    uint8_t led;
};

static void fpu_task(void *pvParameters)
{
    //LED_On(LED_BLUE);
    // Force x to stay in a FPU reg.
    //
    register float x = 0;
    struct task_param *p = pvParameters;

    for(;;) {
       STM_EVAL_LEDToggle(p->led);
        vTaskDelay(p->interval);
	x*=2.1;
    }
   
}

static void init_task(void *pvParameters)
{
    for(int i=0; i < 8; i++)
      STM_EVAL_LEDInit(i);

    uart_init(115200);
    printf(ANSI_FG_LTRED "STM32F407" ANSI_NORMAL " FreeRTOS Test\n" );

    vTaskDelay(100);

    for (int i=0; i<5; i++) {
        printf("Starting FPU task %d..\n", i);

        struct task_param *p;

        p = malloc(sizeof(struct task_param));
        p->name     = malloc(16);
        p->interval = (i+1) * 500;
	p->led = i;
        sprintf(p->name, "FPU_%d", i);
        STM_EVAL_LEDOn(LED10);

        xTaskCreate(fpu_task, (int8_t*)p->name, 1024, p, tskIDLE_PRIORITY, NULL);
    }

    for(;;);
}


int main(void)
{
    // FreeRTOS assumes 4 preemption- and 0 subpriority-bits
    //
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // Create init task and start the scheduler
    //
    xTaskCreate(init_task, (signed char*)"init", 1024, NULL, tskIDLE_PRIORITY, NULL);
    vTaskStartScheduler();
}


void assert_failed(uint8_t* file, uint32_t line){
}
