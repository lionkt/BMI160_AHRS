#ifndef __ATM32ADC_H
#define __ATM32ADC_H

#include "stm32f4xx.h"


extern int16_t lastVOL;
// 模数转换引出 的API 程序
extern void ADC_Voltage_initial(void); //初始化，在上电的时候调用一次。之后 ADC会自动采集更新
extern int16_t Get_Bat_Vol(void);  //读取当前的电池电压值， 单位 0.01V
extern uint8_t Is_BAT_LOW(void);
#endif

//------------------End of File----------------------------
