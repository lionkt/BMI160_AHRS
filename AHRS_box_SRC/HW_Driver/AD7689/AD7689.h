#ifndef __AD7689_H
#define __AD7689_H

#include "stm32f4xx.h"
#include "delay.h"
#include "SPI2.h"

extern int16_t 
	  Accel_Xint,  //ADC值
	  Accel_Yint,	
	  Accel_Zint,
	  Gyro_Xint,   
	  Gyro_Yint,	
	  Gyro_Zint ;

extern float Accel_X,  //加速度X轴, 单位g [9.8m/S^2]
			  Accel_Y,	//加速度Y轴, 单位g [9.8m/S^2]
			  Accel_Z,	//加速度Z轴, 单位g [9.8m/S^2]
			  Gyro_X,   //角速度X轴, 单位dps [度每秒]
			  Gyro_Y,	//角速度Y轴, 单位dps [度每秒]
			  Gyro_Z	//角速度Z轴, 单位dps [度每秒]
			  ;
extern int16_t lastAx,lastAy,lastAz,
		lastGx,lastGy,lastGz;
extern int16_t acc_vector_tran;
extern float Transe_ax,Transe_ay,Transe_az,
		Transe_gx,Transe_gy,Transe_gz,transe_Data[6],transe_gyro,transe_acc;

void Gyro_Initial_Offset(void);
void Dof6_Update(void);	//更新6轴的传感器数据
void Reset_ACC_Offset(void);
int16_t get_ACCMAX(unsigned char aisx);
void Gyro_update_config(void);
#endif

//------------------End of File----------------------------


