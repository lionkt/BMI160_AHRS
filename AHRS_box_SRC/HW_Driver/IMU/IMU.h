#ifndef __IMU_H
#define __IMU_H

#include "delay.h"
#include "LED.h"
#include "QMC5883.h"
#include "BMI160.h"
 #include "MS5611.h"


#include <math.h>

#define M_PI  (float)3.1415926535

#define Gyro_Resolution	 16.4f	   //16.4LSB/dps
#define Acc_Resolution	 8192.0f   //8192LSB/g
#define Mag_Resolution	 1090.0f   //1090LSB/Gauss

extern float  pitch ,roll ,yaw;
// AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(float * angles); //更新姿态

#endif

//------------------End of File----------------------------
