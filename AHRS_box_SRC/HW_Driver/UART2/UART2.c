/* UART1.C file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-06-28
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

功能：实现	[Captain 飞控板] 的 UART2 接口操作

---------硬件上的引脚连接:----------
UART2接口：
UART2TXD  -->  PA2  (UART2-TXD)
UART2RXD  -->  PA3 (UART2-RXD)
------------------------------------
 */

#include "UART2.h"
#include "UART1.h"
#include "USART2DMATX.h"
#include "OSQMem.h"	
#include "delay.h"
 #include "FAT_driver.h"

extern u8 OSUSART2MemQ[OS_MEM_USART2_MAX];  			//空白内存块
OSMEMTcb* OSQUSART2Index;

//接收缓冲区
volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_wr_index;
volatile unsigned char RC_Flag;
u8 U2TxBuffer[258];
volatile u8 buf_lock =0;

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA2_Stream7 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x08;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化STM32-SDK开发板上的RS232接口
输入参数：
		u32 baudrate   设置RS232串口的波特率
输出参数：没有	
*******************************************************************************/
void Initial_UART2(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef     DMA_InitStructure;
	u8 err;

	/* 使能 UART1 模块的时钟  使能 UART1对应的引脚端口PA的时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  	 /* 配置UART1 的发送引脚
	 配置PA9 为复用输出  刷新频率50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	/* 
	  配置UART1 的接收引脚
	  配置PA10为浮地输入 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	USART_DeInit(USART2);  
	/* 
	  UART2的配置:
	  1.波特率为调用程序指定的输入 baudrate;
	  2. 8位数据			  USART_WordLength_8b;
	  3.一个停止位			  USART_StopBits_1;
	  4. 无奇偶效验			  USART_Parity_No ;
	  5.不使用硬件流控制	  USART_HardwareFlowControl_None;
	  6.使能发送和接收功能	  USART_Mode_Rx | USART_Mode_Tx;
	 */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//应用配置到UART2
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//使能接收中断
	//启动UART2
  	USART_Cmd(USART2, ENABLE);

	//DMA 请求
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Stream6);
	//USART1 TX DMA Configure
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART2->DR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&OSUSART2MemQ;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//这里是byte
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);	 // 使能发送DMA
	NVIC_Configuration(); //配置中断优先级
	//申请内存管理管理快
	OSQUSART2Index=(OSMEMTcb *)OSMemCreate(OSUSART2MemQ,OS_MEM_USART2_BLK,OS_MEM_USART2_MAX/OS_MEM_USART2_BLK,&err);
	
}

/*******************************************************************************
* 文件名	  	 : DMA1_Stream6_IRQHandler
* 描述	         : DMA1_Stream6_IRQHandler（USART2发送）DMA函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void DMA1_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
	{
	  DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);	
	  USART2DMAUpdate();
	}
}

/**************************实现函数********************************************
*函数原型:		void UART2_Put_String(unsigned char *buffer)
*功　　能:		RS232发送字符串
输入参数：
		unsigned char *buffer   要发送的字符串
输出参数：没有	
*******************************************************************************/
void UART2_Put_String(unsigned char *buffer)
{
	unsigned long count=0;
	while(buffer[count]!='\0')count++;
	(USART2WriteDataToBuffer(buffer,count));	
}

u16 Buff_WR_Index=0;

void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,uint32_t IMUpersec)
{
 	unsigned int temp=0xaF+2+4;
	char ctemp;
	Buff_WR_Index=0;
	U2TxBuffer[Buff_WR_Index++]=(0xa5);
	U2TxBuffer[Buff_WR_Index++]=(0x5a);
	U2TxBuffer[Buff_WR_Index++]=(14+6);
	U2TxBuffer[Buff_WR_Index++]=(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=roll;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=alt;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=press;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>24;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec>>16;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec>>0;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	
	U2TxBuffer[Buff_WR_Index++]=(temp%256);
	U2TxBuffer[Buff_WR_Index++]=(0xaa);
	USART2WriteDataToBuffer(&U2TxBuffer[0],Buff_WR_Index);
//	USART1WriteDataToBuffer(&U2TxBuffer[0],Buff_WR_Index);
}

void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	Buff_WR_Index=0;
	U2TxBuffer[Buff_WR_Index++]=(0xa5);
	U2TxBuffer[Buff_WR_Index++]=(0x5a);
	U2TxBuffer[Buff_WR_Index++]=(14+8);
	U2TxBuffer[Buff_WR_Index++]=(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=ax;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=ay;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=az;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=gx;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=gy;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=gz;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=hx;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=hy;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=hz;
	U2TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	U2TxBuffer[Buff_WR_Index++]=(temp%256);
	U2TxBuffer[Buff_WR_Index++]=(0xaa);
	USART2WriteDataToBuffer(&U2TxBuffer[0],24);
//	USART1WriteDataToBuffer(&U2TxBuffer[0],24);
}

void UART2_ReportHMC(int16_t maxx,int16_t maxy,int16_t maxz
,int16_t minx,int16_t miny,int16_t minz,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2+3;
	char ctemp;
	U2TxBuffer[0]=(0xa5);
	U2TxBuffer[1]=(0x5a);
	U2TxBuffer[2]=(14+4);
	U2TxBuffer[3]=(0xA4);

	if(maxx<0)maxx=32768-maxx;
	ctemp=maxx>>8;
	U2TxBuffer[4]=(ctemp);
	temp+=ctemp;
	ctemp=maxx;
	U2TxBuffer[5]=(ctemp);
	temp+=ctemp;

	if(maxy<0)maxy=32768-maxy;
	ctemp=maxy>>8;
	U2TxBuffer[6]=(ctemp);
	temp+=ctemp;
	ctemp=maxy;
	U2TxBuffer[7]=(ctemp);
	temp+=ctemp;

	if(maxz<0)maxz=32768-maxz;
	ctemp=maxz>>8;
	U2TxBuffer[8]=(ctemp);
	temp+=ctemp;
	ctemp=maxz;
	U2TxBuffer[9]=(ctemp);
	temp+=ctemp;

	if(minx<0)minx=32768-minx;
	ctemp=minx>>8;
	U2TxBuffer[10]=(ctemp);
	temp+=ctemp;
	ctemp=minx;
	U2TxBuffer[11]=(ctemp);
	temp+=ctemp;

	if(miny<0)miny=32768-miny;
	ctemp=miny>>8;
	U2TxBuffer[12]=(ctemp);
	temp+=ctemp;
	ctemp=miny;
	U2TxBuffer[13]=(ctemp);
	temp+=ctemp;

	if(minz<0)minz=32768-minz;
	ctemp=minz>>8;
	U2TxBuffer[14]=(ctemp);
	temp+=ctemp;
	ctemp=minz;
	U2TxBuffer[15]=(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	U2TxBuffer[16]=(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	U2TxBuffer[17]=(ctemp);
	temp+=ctemp;

	U2TxBuffer[18]=(temp%256);
	U2TxBuffer[19]=(0xaa);
	USART2WriteDataToBuffer(&U2TxBuffer[0],20);
}

void UART2_Send_ACC(int16_t offset,unsigned char ch){
	unsigned char buf[9];
	unsigned int temp=0xA5+7;
	char ctemp;

	buf[0]=(0xa5);
	buf[1]=(0x5a);
	buf[2]=(7);
	buf[3]=(0xA5);


	if(offset<0)offset=32768-offset;
	ctemp=offset>>8;
	buf[4]=(ctemp);
	temp+=ctemp;
	ctemp=offset;
	buf[5]=(ctemp);
	temp+=ctemp;

	buf[6]=ch;
	temp+=ch;

	buf[7]=(temp%256);
	buf[8]=(0xaa);
	USART2WriteDataToBuffer(buf,9);
}


//------------------------------------------------------
volatile unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_wr_index;
volatile unsigned char RC_Flag;
void USART2_IRQHandler(void)
{
  unsigned char data;
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
  data=USART_ReceiveData(USART2);
  if(data==0xa5)
  { 
	RC_Flag|=b_uart_head;
    rx_buffer[rx_wr_index++]=data;
  }
  else if(data==0x5a)
       { if(RC_Flag&b_uart_head)
	     { rx_wr_index=0;
		   RC_Flag&=~b_rx_over;
         }
         else
		  rx_buffer[rx_wr_index++]=data;
         RC_Flag&=~b_uart_head;
       }
	   else
	   { rx_buffer[rx_wr_index++]=data;
		 RC_Flag&=~b_uart_head;
		 if(rx_wr_index==rx_buffer[0])
	     {  
			RC_Flag|=b_rx_over;
          }
	   }
  if(rx_wr_index==RX_BUFFER_SIZE)
  rx_wr_index--;
  /* Clear the USART1 RX interrupt */
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

unsigned char Sum_check(void)
{ 
  unsigned char i;
  unsigned int checksum=0; 
  for(i=0;i<rx_buffer[0]-2;i++)
   checksum+=rx_buffer[i];
  if((checksum%256)==rx_buffer[rx_buffer[0]-2])
   return(0x01); //Checksum successful
  else
   return(0x00); //Checksum error
}





unsigned char UART2_CommandRoute(void)
{
 if(RC_Flag&b_rx_over){
		RC_Flag&=~b_rx_over;
		if(Sum_check()){
		return rx_buffer[1];
		}
	}
return 0xff; //没有收到上位机的命令，或者是命令效验没有通过
}

//------------------End of File----------------------------
