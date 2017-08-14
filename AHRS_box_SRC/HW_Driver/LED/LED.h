#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
#include "delay.h"

#define ON  0xff
#define OFF 0x00

//LEDs 定义LED 操作宏.
//(输出低电平,灯亮;输出高电平灯灭)

#define LED_OFF() TIM8->CCR1 = 0;  //GPIO_ResetBits(GPIOC, GPIO_Pin_6)
#define LED_ON()  TIM8->CCR1 = 200;//GPIO_SetBits(GPIOC, GPIO_Pin_6)

#define Check_POW() (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)==0x00)

#define RS485_TX() //GPIOA->ODR |= GPIO_Pin_0
#define RS485_RX() // GPIOA->ODR  &= ~GPIO_Pin_0

#define LED_EER_OFF()  
#define LED_EER_ON() 

#define BEEP_ON()   GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define BEEP_OFF()  GPIO_SetBits(GPIOC, GPIO_Pin_1)

#define USB_Enable() GPIO_SetBits(GPIOB, GPIO_Pin_9) 
#define USB_Disable()  GPIO_ResetBits(GPIOB, GPIO_Pin_9)

//JTAG模式设置定义
#define JTAG_SWD_DISABLE   0x02
#define SWD_ENABLE         0x01
#define JTAG_SWD_ENABLE    0x00	

extern uint8_t  Key_IS_Press;

void Initial_LED_GPIO(void);
void LED_Reverse(void);
void Initial_Timer1(void);
double Get_Timer1_p(void);
void SET_POWER_Down(void);
void LED_Change(void);
void Scan_Key_Routing(void);
void Set_BeepON(uint16_t ON_ms);

#endif


//------------------End of File----------------------------
