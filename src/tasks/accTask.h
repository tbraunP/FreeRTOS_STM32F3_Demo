#ifndef __ACCTASK_H_
#define __ACCTASK_H_
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

void accTask(void *pvParameters);

void EXTI4_IRQHandler();

void I2C1_EV_IRQHandler();

void DMA1_Channel7_IRQHandler();

uint16_t LSM303DLHC_ReadDMA(uint8_t DeviceAddr, uint8_t RegAddr,
		uint8_t* pBuffer, uint16_t NumByteToRead);

#ifdef __cplusplus
 }
#endif

#endif
