#ifndef __HMC5983L_H
#define __HMC5983L_H

#include "stm32f4xx.h"
#include "delay.h"
#include "SPI1.h"

#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC5983 register map. For details see HMC5983 datasheet
#define HMC5983_R_CONFA 0
#define HMC5983_R_CONFB 1
#define HMC5983_R_MODE 2
#define HMC5983_R_XM 3
#define HMC5983_R_XL 4

#define HMC5983_R_YM (7)  //!< Register address for YM.
#define HMC5983_R_YL (8)  //!< Register address for YL.
#define HMC5983_R_ZM (5)  //!< Register address for ZM.
#define HMC5983_R_ZL (6)  //!< Register address for ZL.

#define HMC5983_R_STATUS 9
#define HMC5983_R_IDA 10
#define HMC5983_R_IDB 11
#define HMC5983_R_IDC 12

extern unsigned char HMC5983_calib;
extern int16_t  
		 		lastMx,lastMy,lastMz;
//当前磁场的最大值和最小值
extern int16_t  HMC5983_maxx,HMC5983_maxy,HMC5983_maxz,
		 HMC5983_minx,HMC5983_miny,HMC5983_minz;
extern unsigned char Mag_calib; //初始化完成标志
extern float  magic_GRAVITY;

void HMC5983L_SetUp(void);	//初始化
void HMC5983_getID(char id[3]);	//读芯片ID
void HMC5983_getRaw(int16_t *x,int16_t *y,int16_t *z); //读ADC
void HMC5983_mgetValues(float *arry); //IMU 专用的读取磁力计值
void HMC5983_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void HMC5983L_Save_Calib(void);
void HMC5983L_Start_Calib(void);
void HMC5983L_update_config(void);
#endif

//------------------End of File----------------------------



