#ifndef __SPI1_H
#define __SPI1_H

#include "stm32f4xx.h"

void SPI1_Configuration(void);
uint8_t SPI1_ReadWrite_Byte(uint8_t byte);
void SPI1_SetSpeed(uint16_t SpeedSet);
void SPI1_writeReg(u8 reg ,u8 data);
u8 SPI1_readReg(u8 reg);
void SPI1_readRegs(u8 reg, u8 length, u8 *data);

#endif

//------------------End of File----------------------------
