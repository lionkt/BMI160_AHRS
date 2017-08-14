/* KEY.C file
STM32-SDK 开发板相关例程
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-02-28
测试： 本程序已在第七实验室的STM32-SDK上完成测试
功能：实现	Captian 的LED初始化和操作 API

---------硬件上的引脚连接:----------

------------------------------------
 */

#include "LED.h"


//LED 亮度级别表
static int LightLevel[40]={0,0,0,0,0,1,1,2,4,8,16,32,50,64,80,100,100,120,140,180,180,140,120,100,100,80,64,50,32,16,8,4,2,1,1,0,0,0,0,0};
u8 lightc=0;

void Initial_PWM_LED(u32 arr,u32 psc);
/**************************实现函数********************************************
*函数原型:		void Initial_LED_GPIO(void)
*功　　能:		配置 LED 对应的端口为输出
*******************************************************************************/
void Initial_LED_GPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //使能GPIOA 的时钟,
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC , ENABLE);
  //配置PA8 为推挽输出  刷新频率为2Mhz  
  GPIOB->AFR[0] &= 0xff000fff;	//开启SWD
  GPIOB->AFR[0] |= 0x00033000;  //禁止JTAG接口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; // USB上拉电阻	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //应用配置到GPIOB 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  USB_Disable();

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	   //BEEP
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	   //PC3 电源检测。低电平关机。
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	   //PA0 蓝牙连接状态引脚
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	   //PB8 SD 检测
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	   //PB10 按钮 检测
  GPIO_Init(GPIOB, &GPIO_InitStructure);

   //设置LED 端口输出高电平, 关灯.
   Initial_PWM_LED(250,5);
   GPIO_SetBits(GPIOC, GPIO_Pin_6);
   BEEP_OFF();
}

uint32_t key_pressT;
uint8_t  key_staut = 0;
uint8_t  Key_IS_Press = 0;
uint32_t Beep_StartT,Beep_OnT = 0;

void Scan_Key_Routing(void){  //按键扫描
	uint32_t temp;	
   if((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)==0x00)){ //低电平
   		if(key_staut == 0){
		key_staut = 1;
		key_pressT = micros(); //记录高电平的时间
		}
   }else{  //高电平。
	 if(key_staut != 0){
	 	key_staut = 0;
		temp = micros();
		//按键时间在200ms  - 1000ms之间有效
		if((1000000>(temp - key_pressT))&&((temp - key_pressT)>200000)){
			Key_IS_Press = 1;
		}
	 }

   }
//------------beep 
	if(Beep_OnT){
	temp = micros();	
	if((temp - Beep_StartT) > Beep_OnT){
		Beep_OnT = 0;
		BEEP_OFF();	
	}
	}
}

void Set_BeepON(uint16_t ON_ms){
	if(ON_ms == 0){
		Beep_OnT = 0;
		BEEP_OFF();
		return;
	}
	Beep_StartT = micros();
	Beep_OnT = (uint32_t)ON_ms * 1000;  // ms -> us
	BEEP_ON();
	lightc=0;
}



void SET_POWER_Down(void){
   GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //应用配置到GPIOC 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOC, GPIO_Pin_3);	
}

/**************************实现函数********************************************
*函数原型:		void LED_Reverse(void)
*功　　能:		LED 灯取反, 即,当亮时设置端口使之转成灭状态,
								当灭时设置端口使之转成亮状态.
*******************************************************************************/
u8 LED_ST = 0;
void LED_Reverse(void)
{
/*
	if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_6))	
		GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		else
		GPIO_SetBits(GPIOC, GPIO_Pin_6);  */
		if(LED_ST){
		   TIM8->CCR1 = 200;
		   LED_ST = 0;
		} else{
			TIM8->CCR1 = 0;
			LED_ST = 1;	
		} 
}

/**************************实现函数********************************************
*函数原型:		void LED_Change(void)
*功　　能:		改变LED的亮度，从	LightLevel 数据
*******************************************************************************/
void LED_Change(void)
{
	TIM8->CCR1=LightLevel[lightc]; //更新通道1的比较值
	if(++lightc==40)lightc=0;
}

/**************************实现函数********************************************
*函数原型:		void initial_Timer1(void)
*功　　能:		Timer1 初始化 
*******************************************************************************/
void Initial_Timer1(void){
    
    //系统是84Mhz  84M/（5+1）=14000 000，
	//计数器装的是65536，
	//声明一个定时器的结构体变量
 	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	//开外设时钟
 	//计数器装载的值
 	TIM_TimeBaseStructure.TIM_Period = 0xffff;  
 	//预分频的值，设置值减1
 	TIM_TimeBaseStructure.TIM_Prescaler = 5; //14M 的时钟	最大计时4ms
 	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
 	//计数器计数方向
 	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	//开中断，更新中断。
    //TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);     
	//使能定时器
   	TIM_Cmd(TIM1, ENABLE); 
}

double Get_Timer1_p(void){
	uint32_t temp=0 ;
	double tempf;
  	temp = TIM1->CNT;
	TIM1->CNT = 0;  //重启定时器
  	tempf = ( (double)temp ) / 28000000.0f;
	return tempf;
}


void Initial_PWM_LED(u32 arr,u32 psc){

	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8时钟使能   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOC6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PC6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); //GPIOC6复用为定时器8
  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV4; 
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器8
	
	//初始化TIM8 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性
	TIM_OCInitStructure.TIM_Pulse = 20;	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	//禁止OC2 OC3输出
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM8在CCR1上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	TIM_CtrlPWMOutputs(TIM8, ENABLE); 
}


//------------------End of File----------------------------
