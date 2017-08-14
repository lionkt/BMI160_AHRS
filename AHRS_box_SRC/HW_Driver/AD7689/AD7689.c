


#include "LED.h"
#include "math.h"
#include "AD7689.h"
#include "eeprom.h"


//基准电压+5.000V  16位ADC ,对应 [13.107  LBS/mV]
#define Sensitive_Accel  13107.0f        //加速度灵敏度[1000mV/g]
#define Sensitive_Gyro   78.642f         //陀螺仪灵敏度[6mV/°/sec] 
#define Gyro_To_32_8     (32.8f / Sensitive_Gyro) //将	陀螺仪 分 辨 率转成32.8,以便和上位机对应。

#define Buf_SIZE    10

int16_t 
	  Accel_Xint,  //ADC值
	  Accel_Yint,	
	  Accel_Zint,
	  Gyro_Xint,   
	  Gyro_Yint,	
	  Gyro_Zint ;

float Accel_X,  //加速度X轴, 单位g [9.8m/S^2]
	  Accel_Y,	//加速度Y轴, 单位g [9.8m/S^2]
	  Accel_Z,	//加速度Z轴, 单位g [9.8m/S^2]
	  Gyro_X,   //角速度X轴, 单位dps [度每秒]
	  Gyro_Y,	//角速度Y轴, 单位dps [度每秒]
	  Gyro_Z	//角速度Z轴, 单位dps [度每秒]
	  ;
	  
//-------------------------------------------------------------------------
#define	CSH			GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define	CSL			GPIO_ResetBits(GPIOA, GPIO_Pin_8)

/*
 输入通道为单极性，INx与GND做为参考
 使用外部基准电压源 [5.0V]
*/
static uint16_t  AD7689_Config_buf[8]={
							(0x3c39|(0x0005<<7)), //通道5的CFG配置值
							(0x3c39|(0x0006<<7)), //通道6的CFG配置值
							(0x3c39|(0x0007<<7)),  //通道7的CFG配置值
							(0x3c39|(0x0000<<7)), //通道0的CFG配置值
							(0x3c39|(0x0001<<7)), //通道1的CFG配置值
							(0x3c39|(0x0002<<7)), //通道2的CFG配置值
							(0x3c39|(0x0003<<7)), //通道3的CFG配置值
							(0x3c39|(0x0004<<7))  //通道4的CFG配置值
							};
static uint16_t  AD7689_Result[8],AD7689_Config_index = 0;
static uint8_t   Result_Point[8] =  //当前读取到的数据，对应的ADC通道。
							{3,4,5,6,7,0,1,2};
uint8_t Gyro_Off_started = 0;
int16_t lastAx,lastAy,lastAz,
		lastGx,lastGy,lastGz;
static uint16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;

//写入CFG配置,并读取上上次ADC转换的结果	[结合AD7689的数据手册]
//输入 uint16_t Config   14位的配置信息  第n个配置信息
//返回 ADC转换结果                       第n-2个转换结果
static uint16_t Read_AD7689(uint16_t Config){
    uint16_t  ADC_Value = 0x00;
	Config = Config << 2;
	Config |= 0x8000;
	CSL;//选择芯片
	ADC_Value = SPI2_ReadWrite_Byte(Config >> 8);
	ADC_Value = ADC_Value << 8;	//ADC结果高字节
	ADC_Value |= SPI2_ReadWrite_Byte(Config & 0x00ff);
	CSH;
	delay_us(1);
	return ADC_Value;
}

//扫描8个通通的ADC转换结果
static void AD7689_Update_Result(void){
	uint8_t i;
	uint16_t temp;
	AD7689_Config_index = 0;
	for(i=0; i<8; i++){	//扫8个通道
	temp = Read_AD7689(AD7689_Config_buf[AD7689_Config_index]);
	AD7689_Result[Result_Point[AD7689_Config_index]] = temp;
	AD7689_Config_index++;
	AD7689_Config_index = AD7689_Config_index%8;
	}

}

//读取缓冲区中的ADC结果
static uint16_t Read_AD7689_Result(uint8_t ch){
	if(ch > 7)return 0;
	return AD7689_Result[ch];
}

void Gyro_Initial_Offset(void){
	uint16_t i;
	uint32_t offset_sumx = 0,
			offset_sumy = 0,
			offset_sumz = 0;
	for(i = 0;i < 50 ; i++)AD7689_Update_Result();
	for(i = 0;i < 600 ; i++){
	AD7689_Update_Result();
	delay_ms(1);
	offset_sumx += Read_AD7689_Result(2); //gyro x 
	offset_sumy += Read_AD7689_Result(1); //gyro y
	offset_sumz += Read_AD7689_Result(5); //gyro z
	}
	Config.dGx_offset = (uint16_t)(offset_sumx / 600);
	Config.dGy_offset = (uint16_t)(offset_sumy / 600);
	Config.dGz_offset = (uint16_t)(offset_sumz / 600);
	Write_config();  //写入flash保存
}

int16_t filter_buf[6][Buf_SIZE]; 
int Wr_Index = 0 ;
int16_t Fliter_AVG(int16_t* buf) {
  	int32_t sum = 0;
	int i;
	for(i=0;i<Buf_SIZE;i++)
		sum += buf[i];
	return sum/Buf_SIZE;
}

//更新6轴的传感器数据。
void Dof6_Update(void){
	float temp , mid;
	AD7689_Update_Result();
	mid = (float)32767.0; // [0xffff/2]  
	

	temp = (float)(AD7689_Result[2]);  //Gyro X
	temp -= Config.dGx_offset;
	temp = temp;   //坐标变换
	Gyro_X = temp / Sensitive_Gyro;	//转成度每秒
	lastGx = Gyro_Xint = temp;//*Gyro_To_32_8;
	
	temp = (float)(AD7689_Result[1]);  //Gyro Y
	temp -= Config.dGy_offset;
	temp = -temp;   //坐标变换
	Gyro_Y = temp / Sensitive_Gyro;
	lastGy = Gyro_Yint = temp;//*Gyro_To_32_8;
	
	temp = (float)(AD7689_Result[5]);  //Gyro Z
	temp -= Config.dGz_offset;
	temp = -temp;   //坐标变换
	Gyro_Z = temp / Sensitive_Gyro;
	lastGz = Gyro_Zint = temp;//*Gyro_To_32_8;
	
	temp = (float)((float)AD7689_Result[7]-mid);  //ACC x
	temp = -temp;   //坐标变换
	lastAx = Accel_Xint = temp;
	Accel_X = temp / Sensitive_Accel; //转成单位为g  
		
	temp = (float)((float)AD7689_Result[6]-mid); //ACC y
	temp = -temp;   //坐标变换
	lastAy =Accel_Yint = temp;
	Accel_Y = temp / Sensitive_Accel;
	
	temp = (float)((float)AD7689_Result[3]-mid); //ACC z
	temp = -temp;   //坐标变换	
	lastAz = Accel_Zint = temp;
	Accel_Z = temp / Sensitive_Accel;

	filter_buf[0][Wr_Index] = Gyro_Xint;
	filter_buf[1][Wr_Index] = Gyro_Yint;
	filter_buf[2][Wr_Index] = Gyro_Zint;
	filter_buf[3][Wr_Index] = Accel_Xint;
	filter_buf[4][Wr_Index] = Accel_Yint;
	filter_buf[5][Wr_Index] = Accel_Zint;
	Wr_Index = (Wr_Index + 1) % Buf_SIZE;

	lastGx = Fliter_AVG(&filter_buf[0][0]);
	lastGy = Fliter_AVG(&filter_buf[1][0]);
	lastGz = Fliter_AVG(&filter_buf[2][0]);
	lastAx = Fliter_AVG(&filter_buf[3][0]);
	lastAy = Fliter_AVG(&filter_buf[4][0]);
	lastAz = Fliter_AVG(&filter_buf[5][0]);

}


//------------------End of File----------------------------
