#ifndef __UART2_H
#define __UART2_H

#include <stdio.h> 
#include "stm32f4xx.h"
#include "eeprom.h"
#include "Time3_Driver.h"
#include "UART1.h"

#define Gyro_init  0xE0
#define HMC_calib  0xE1
#define High_init  0xE2
#define HMC_calib_begin  0xE3
#define Read_Offset 0xb5


#define Mode_IMU       0x00
#define Mode_IMU_Move  0x01
#define Mode_Move      0x02

#define Atuo_Send  0x01
#define Modbus_Send  0x02



extern volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
extern u8 file_buf[52];

void UART2_Send_ACC(int16_t offset,unsigned char ch);
void Initial_UART2(u32 baudrate);
void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,uint32_t IMUpersec);
void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
unsigned char UART2_CommandRoute(void);
void UART2_ReportHMC(int16_t maxx,int16_t maxy,int16_t maxz
,int16_t minx,int16_t miny,int16_t minz,int16_t IMUpersec);
void UART2_Send_ACC(int16_t offset,unsigned char ch);
#endif

//------------------End of File----------------------------

