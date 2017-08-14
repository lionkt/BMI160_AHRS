/* Time3_Driver.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
定时器产生中断的配置。和初始化程序
------------------------------------
 */

#include "Time3_Driver.h"


/**************************实现函数********************************************
*函数原型:		void Tim3_NVIC_Init(void)
*功　　能:		 初始化开启定时器4 	中断 配置为最高优先级别的中断
*******************************************************************************/
void Tim3_NVIC_Init(void) 
{ 
 	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
 	NVIC_Init(&NVIC_InitStructure); 
}

/**************************实现函数********************************************
*函数原型:		void Time3_Inttrup_init(void)
*功　　能:		 初始化定时器3  配置定时器的时钟频率为 10K Hz
				并开启Tim3的溢出中断，在中断程序中更新数据
*******************************************************************************/
void Time3_Inttrup_init(void)
{
	//系统是84Mhz  84M/（8399+1）=10000，
	//计数器装的是500，*10=5000/10000=0.05s
	//声明一个定时器的结构体变量
 	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//开外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
 	//计数器装载的值
 	TIM_TimeBaseStructure.TIM_Period = 10000;  
 	//预分频的值，设置值减1
 	TIM_TimeBaseStructure.TIM_Prescaler = 8399; 
 	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
 	//计数器计数方向
 	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//开中断，更新中断。
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 

	TIM3->CR1 &=~0x01;    //关定时器4 
	Tim3_NVIC_Init();  //配置中断优先级
}							

/**************************实现函数********************************************
*函数原型:		void Tim4_Set_Speed(u16 Speed)
*功　　能:		设置数据输出频率，更改Tim4中断的频率
输入   要更新的速度，单位 Hz
*******************************************************************************/
void Tim3_Set_Speed(u16 Speed)
{
	u16 New_arr;
	TIM3->CR1 &=~0x01;    //关定时器4
	New_arr	= 10000/Speed - 1;
	TIM3->ARR = New_arr;  //写入计数器自动重装值
	//TIM4->CR1 |= 0x01;    //使能定时器4
}

/**************************实现函数********************************************
*函数原型:		void Tim4_Stop(void)
*功　　能:		关闭定时器4的时钟
*******************************************************************************/
void Tim3_Stop(void)
{
	TIM3->CR1 &=~0x01;    //关定时器4
}

/**************************实现函数********************************************
*函数原型:		void Tim4_Restart(void)
*功　　能:		开启定时器4的时钟
*******************************************************************************/
void Tim3_Restart(void)
{
	TIM3->CNT = 0;
	TIM3->CR1 |= 0x01;    //使能定时器4
}

//------------------End of File----------------------------
