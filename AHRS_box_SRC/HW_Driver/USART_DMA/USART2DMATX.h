/*******************************************************************************
* 描述	         : USART的驱动函数
* 移植步骤		 : 中间层函数
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
#ifndef  _USART2DMA_H
#define  _USART2DMA_H

#include "stm32f4xx.h"

#define		DMA_MODE 		1 		//定义是采用DMA模式，还是普通的中断模式

void USART2DMAUpdate(void);
unsigned char USART2WriteDataToBuffer(unsigned char *buffer,unsigned char count);
unsigned char USART1DispFun(unsigned char *buffer);

#endif

//------------------End of File----------------------------
