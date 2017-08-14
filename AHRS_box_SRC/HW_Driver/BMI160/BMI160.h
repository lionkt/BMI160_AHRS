#ifndef __BMI160_H
#define __BMI160_H

#include "SPI1.h"
#include "eeprom.h"
#include "delay.h"

//BMI160片选信号控制
#define BMI160_CSH()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define BMI160_CSL()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)

#define NV_CONF  0x70
#define ACC_CONF 0x40 
#define ACC_FS	 0x41
#define GRY_CONF 0x42
#define GRY_FS	 0x43
#define PMU_TRIG 0x6c
#define CMD  	 0x7e
#define PMU_STATUS  0x03

extern int16_t lastGx,lastGy,lastGz;//最新的三轴角速度ADC值。
extern int16_t  lastAx,lastAy,lastAz;//最新的加速度ADC值

uint8_t BMI160_Read_ID(void);
void BMI160_init(void);
void BMI160_readAccGyro(int16_t *data) ;
void BMI160_InitGyro_Offset(void);
int16_t BMI160_get_ACCMAX(unsigned char ais);
void BMI160_Reset_ACC_Offset(void);
void ACC_Save_cal(void);
void ACC_Cal(unsigned char ch);

#endif

//------------------End of File----------------------------

