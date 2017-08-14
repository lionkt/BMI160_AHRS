#ifndef __UART1_H
#define __UART1_H

#include <stdio.h> 
#include "stm32f4xx.h"


//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40

// USART Receiver buffer
#define RX_BUFFER_SIZE 100

	 //浮点 联合体
typedef union {
	float  value;
	unsigned char byte[4];
} f_bytes;

//整数 联合体
typedef union {
	int16_t  value;
	unsigned char byte[2];
} i_bytes;

void Initial_UART1(u32 baudrate);
void UART1_Put_Char(unsigned char DataToSend);
void UART1_Put_String(unsigned char *Str);
void UART1_Putw_Dec(uint32_t w);
unsigned char UART1_CommandRoute(void);
void Send_Pos(float px,float py,float pz,
			float spx,float spy,float spz);

#endif

//------------------End of File----------------------------

