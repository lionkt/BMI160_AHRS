

#include "SPI2.h"

static uint8_t SPI2_ready = 0;

void SPI2_Configuration(void){

	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	if(SPI2_ready >0)return;
	SPI2_ready++;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/* SCK, MISO and MOSI  PB13=CLK,PB14=MISO,PB15=MOSI*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //开启上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);

	 
	/*  PB12 作片选*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);//预置为高

	/*  PB10 作卡检测*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	/* SPI2 configuration  */
	SPI_Cmd(SPI2, DISABLE);        
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //两线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //主
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;      //8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;       //CPOL=0 时钟悬空低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       //CPHA=0 数据捕获第1个
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        //软件NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //4分频

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      //高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;        //CRC7
    
	SPI_Init(SPI2, &SPI_InitStructure);	 //应用配置到 SPI2
	SPI_Cmd(SPI2, ENABLE); 
}

void SPI2_SetSpeed(uint16_t SpeedSet){
	SPI2->CR1 &= 0XFF87; 
	SPI2->CR1 |= SpeedSet;	//设置SPI2速度  
	SPI2->CR1 |= 1<<6; 		//SPI设备使能
}

/************************************************************************
** 函数名称:static u8 SPI_ReadWrite_Byte(u8 byte)
** 功能描述:  发送或者接收1个字节
** 输　入:    byte    发送时候,byte传递为发送的数据字节， 接收的时候，则固定为0xff
** 输　出:    SPI1->DR  发送时候，可以忽略, 接收的时候，则为接收数据
***********************************************************************/
uint8_t SPI2_ReadWrite_Byte(uint8_t byte)
{
	/*等待发送寄存器空*/
	while((SPI2->SR & SPI_I2S_FLAG_TXE)==RESET);
	/*发送一个字节*/
	SPI2->DR = byte;
	/* 等待接收寄存器有效*/
	while((SPI2->SR & SPI_I2S_FLAG_RXNE)==RESET);
	return(SPI2->DR);
}

	
//------------------End of File----------------------------		
