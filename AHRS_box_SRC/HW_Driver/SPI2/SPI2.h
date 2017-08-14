#ifndef __SPI2_H
#define __SPI2_H

#include "stm32f4xx.h"

void SPI2_Configuration(void);
void SPI2_SetSpeed(uint16_t SpeedSet);
uint8_t SPI2_ReadWrite_Byte(uint8_t byte);

#endif

//------------------End of File----------------------------
