#ifndef __Time4_Dr_H
#define __Time4_Dr_H

#include "stm32f4xx.h"
/*
使用 STM32F 定时器Tim4驱动程序
引出的API 子程序
*/

void Time3_Inttrup_init(void);
void Tim3_Set_Speed(u16 Speed);
void Tim3_Stop(void);
void Tim3_Restart(void);

#endif /* __Time4_Dr_H */

//------------------End of File----------------------------
