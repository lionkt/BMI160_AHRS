
#include "SPI1.h"

static uint8_t SPI1_ready = 0;

/**************************实现函数********************************************
*函数原型:		void SPI1_Configuration(void)
*功　　能:	    初始化 SPI1 接口
*******************************************************************************/
void SPI1_Configuration(void){

	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	if(SPI1_ready >1)return;
	SPI1_ready++;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* SCK, MISO and MOSI  PA5=CLK,PA6=MISO,PA7=MOSI*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);

	/*  PC2 PA4 作片选*/

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC, GPIO_Pin_2);//预置为高
	GPIO_SetBits(GPIOA, GPIO_Pin_4);//预置为高
	
	/* SPI1 configuration  */
	SPI_Cmd(SPI1, DISABLE);        
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //两线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //主
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;      //8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;        //CPOL=0 时钟悬空低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;       //CPHA=0 数据捕获第1个
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        //软件NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;  //32分频

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      //高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;        //CRC7
    
	SPI_Init(SPI1, &SPI_InitStructure);	 //应用配置到 SPI1
	SPI_Cmd(SPI1, ENABLE); 
					   
}

void SPI1_SetSpeed(uint16_t SpeedSet){
	SPI1->CR1 &= 0XFF87; 
	SPI1->CR1 |= SpeedSet;	//设置SPI1速度  
	SPI1->CR1 |= 1<<6; 		//SPI设备使能
}

/************************************************************************
** 函数名称:uint8_t SPI1_ReadWrite_Byte(uint8_t byte)
** 功能描述:  发送或者接收1个字节
** 输　入:    byte    发送时候,byte传递为发送的数据字节， 接收的时候，则固定为0xff
** 输　出:    SPI1->DR  发送时候，可以忽略, 接收的时候，则为接收数据
***********************************************************************/
uint8_t SPI1_ReadWrite_Byte(uint8_t byte)
{	
	//等待发送寄存器空
	while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
	SPI1->DR = byte;  //发送一个字节
	// 等待接收寄存器有效
	while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);
	return(SPI1->DR);	
}


//写寄存器
void SPI1_writeReg(u8 reg ,u8 data){
	SPI1_ReadWrite_Byte(reg);
	SPI1_ReadWrite_Byte(data);
}

//读寄存器
u8 SPI1_readReg(u8 reg){
	SPI1_ReadWrite_Byte(reg|0x80);
	return SPI1_ReadWrite_Byte(0xff);
}

//从寄存器读出多个字节[寄存器地址要自动增加]
void SPI1_readRegs(u8 reg, u8 length, u8 *data){
	u8 count = 0;
	SPI1_ReadWrite_Byte(reg|0x80);
	for(count=0;count<length;count++){
		data[count] = SPI1_ReadWrite_Byte(0xff);
	}
}

//------------------End of File----------------------------
