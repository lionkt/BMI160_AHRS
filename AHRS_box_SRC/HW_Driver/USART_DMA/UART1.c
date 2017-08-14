/* UART1.C file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-06-28
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

功能：实现	[Captain 飞控板] 的 UART1 接口操作

---------硬件上的引脚连接:----------
UART1接口：
UART1TXD  -->  PA9  (UART1-TXD)
UART1RXD  -->  PA10 (UART1-RXD)
------------------------------------
 */

#include "UART1.h"
#include "USART1DMATX.h"
#include "OSQMem.h"	

extern u8 OSUSART1MemQ[OS_MEM_USART1_MAX];  			//空白内存块
OSMEMTcb* OSQUSART1Index;

//接收缓冲区
static unsigned char rx_buffer[RX_BUFFER_SIZE];
static unsigned char rx_wr_index;
static unsigned char RC_Flag;
unsigned char U1TxBuffer[100];

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA2_Stream7 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
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
void Initial_UART1(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef     DMA_InitStructure;
	u8 err;

	/* 使能 UART1 模块的时钟  使能 UART1对应的引脚端口PA的时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  	 /* 配置UART1 的发送引脚
	 配置PA9 为复用输出  刷新频率50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	/* 
	  配置UART1 的接收引脚
	  配置PA10为浮地输入 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	USART_DeInit(USART1);  
	/* 
	  UART1的配置:
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
	//应用配置到UART1
	USART_Init(USART1, &USART_InitStructure); 
	//USART_ITConfig(USART1, USART_IT_TXE, DISABLE);        
    //USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	//使能接收中断
	//启动UART1
  	USART_Cmd(USART1, ENABLE);

	//DMA 请求
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream7);
	//USART1 TX DMA Configure
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&OSUSART1MemQ;
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
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	 // 使能发送DMA
	NVIC_Configuration(); //配置中断优先级
	//申请内存管理管理快
	OSQUSART1Index=(OSMEMTcb *)OSMemCreate(OSUSART1MemQ,OS_MEM_USART1_BLK,OS_MEM_USART1_MAX/OS_MEM_USART1_BLK,&err);
	//DMA_Cmd(DMA2_Stream7, ENABLE);
}

/*******************************************************************************
* 文件名	  	 : DMA2_Stream7_IRQHandler
* 描述	         : DMA2_Stream7_IRQHandler（USART1发送）DMA函数通道
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
	{
	    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);	
		USART1DMAUpdate();
	}
}

/**************************实现函数********************************************
*函数原型:		void UART1_Put_String(unsigned char *buffer)
*功　　能:		RS232发送字符串
输入参数：
		unsigned char *buffer   要发送的字符串
输出参数：没有	
*******************************************************************************/
void UART1_Put_String(unsigned char *buffer)
{
	unsigned long count=0;
	while(buffer[count]!='\0')count++;
	(USART1WriteDataToBuffer(buffer,count));	
}

void UART1_Putc_Hex(uint8_t b)
{
	unsigned char buf[3];
    if((b >> 4) < 0x0a)
        buf[0] = ((b >> 4) + '0'); 
    else
        buf[0] = ((b >> 4) - 0x0a + 'A'); 

    if((b & 0x0f) < 0x0a)
        buf[1] = ((b & 0x0f) + '0');
    else
        buf[1] = ((b & 0x0f) - 0x0a + 'A');
   buf[2] = (' ');
   USART1WriteDataToBuffer(buf,3);
}

void UART1_Putw_Dec(uint32_t w)
{
	unsigned char buf[20],index = 0;
    uint32_t num = 1000000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
			buf[index++] = '0' + b;
            //UART1_Put_Char('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }

   USART1WriteDataToBuffer(buf,index) ;
}


void UART1_ReportMotion2(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	char Buff_WR_Index=0;
	U1TxBuffer[Buff_WR_Index++]=(0xa5);
	U1TxBuffer[Buff_WR_Index++]=(0x5a);
	U1TxBuffer[Buff_WR_Index++]=(14+8);
	U1TxBuffer[Buff_WR_Index++]=(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=ax;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=ay;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=az;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=gx;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=gy;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=gz;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=hx;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=hy;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;
	ctemp=hz;
	U1TxBuffer[Buff_WR_Index++]=(ctemp);
	temp+=ctemp;

	U1TxBuffer[Buff_WR_Index++]=(temp%256);
	U1TxBuffer[Buff_WR_Index++]=(0xaa);
	USART1WriteDataToBuffer(&U1TxBuffer[0],24);
}

void Send_Pos(float px,float py,float pz,
			float spx,float spy,float spz){
  	f_bytes data;
	unsigned int sum,i;
	char Buff_WR_Index=0;
	U1TxBuffer[Buff_WR_Index++]=(0xa5);
	U1TxBuffer[Buff_WR_Index++]=(0x5a);
	U1TxBuffer[Buff_WR_Index++]=(28);
	U1TxBuffer[Buff_WR_Index++]=(0xA0);
	data.value = px;
	U1TxBuffer[Buff_WR_Index++]=data.byte[0];
	U1TxBuffer[Buff_WR_Index++]=data.byte[1];
	U1TxBuffer[Buff_WR_Index++]=data.byte[2];
	U1TxBuffer[Buff_WR_Index++]=data.byte[3];
	data.value = py;
	U1TxBuffer[Buff_WR_Index++]=data.byte[0];
	U1TxBuffer[Buff_WR_Index++]=data.byte[1];
	U1TxBuffer[Buff_WR_Index++]=data.byte[2];
	U1TxBuffer[Buff_WR_Index++]=data.byte[3];
	data.value = pz;
	U1TxBuffer[Buff_WR_Index++]=data.byte[0];
	U1TxBuffer[Buff_WR_Index++]=data.byte[1];
	U1TxBuffer[Buff_WR_Index++]=data.byte[2];
	U1TxBuffer[Buff_WR_Index++]=data.byte[3];

	data.value = spx;
	U1TxBuffer[Buff_WR_Index++]=data.byte[0];
	U1TxBuffer[Buff_WR_Index++]=data.byte[1];
	U1TxBuffer[Buff_WR_Index++]=data.byte[2];
	U1TxBuffer[Buff_WR_Index++]=data.byte[3];
	data.value = spy;
	U1TxBuffer[Buff_WR_Index++]=data.byte[0];
	U1TxBuffer[Buff_WR_Index++]=data.byte[1];
	U1TxBuffer[Buff_WR_Index++]=data.byte[2];
	U1TxBuffer[Buff_WR_Index++]=data.byte[3];
	data.value = spz;
	U1TxBuffer[Buff_WR_Index++]=data.byte[0];
	U1TxBuffer[Buff_WR_Index++]=data.byte[1];
	U1TxBuffer[Buff_WR_Index++]=data.byte[2];
	U1TxBuffer[Buff_WR_Index++]=data.byte[3];
	sum = 0;
	for(i=2;i<Buff_WR_Index;i++){
			sum += U1TxBuffer[i];
		}
	U1TxBuffer[Buff_WR_Index++] = sum%256;
	U1TxBuffer[Buff_WR_Index++] = 0xAA;
	USART1WriteDataToBuffer(&U1TxBuffer[0],Buff_WR_Index);
}


//------------------------------------------------------
void USART1_IRQHandler(void)
{
  unsigned char data;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
  data=USART_ReceiveData(USART1);
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

//校验接收到的数据
static unsigned char Sum_check(void)
{ 
  unsigned char i;
  unsigned int checksum=0; 
  for(i=0;i<rx_buffer[0]-2;i++)
   checksum+=rx_buffer[i];  // 累加合校验
  if((checksum%256)==rx_buffer[rx_buffer[0]-2])
   return(0x01); //Checksum successful
  else
   return(0x00); //Checksum error
}

//查询是否接收数据
unsigned char UART1_CommandRoute(void)
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
