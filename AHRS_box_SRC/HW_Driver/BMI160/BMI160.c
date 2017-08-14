
#include "BMI160.h"
#include "LED.h"
#include "UART2.h"
#include "eeprom.h"
#include "IMU.h"

#define Buf_SIZE  10	   //保存最近 10组数据 做平均滤波

uint8_t Gyro_Off_started = 0; //陀螺仪零偏采集标志，1表示正在采集零偏
int16_t lastGx,lastGy,lastGz; //最近的角速度ADC值
int16_t  lastAx,lastAy,lastAz;//最新的加速度ADC值
int16_t  BMI160_FIFO[6][Buf_SIZE];  //角速度ADC值缓冲数组
static uint8_t	 Wr_Index = 0;
static int16_t Ax_offset=0,Ay_offset=0,Az_offset=0;
static int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;	 //三个轴的零偏值
static float ax_scale,ay_scale,az_scale;
uint32_t SensorTime = 0;

//添加一个新的值到 温度队列 进行滤波
static void BMI160_NewVal(int16_t* buf,int16_t val) {
  	buf[Wr_Index] = val;
}

//读取一个ADC数组的平均值。
static int16_t BMI160_GetAvg(int16_t* buf){
    int i;
	int32_t	sum = 0;
	for(i=0;i<Buf_SIZE;i++)
		sum += buf[i];
	sum = sum / Buf_SIZE;
	return (int16_t)sum;
}

uint8_t BMI160_Read_ID(void){
	uint8_t id;
	BMI160_CSL();
	SPI1_ReadWrite_Byte(0x00|0x80);
	id = SPI1_ReadWrite_Byte(0xff);
	BMI160_CSH();
	return id;
}

static uint8_t BMI160_Read_Reg(u8 reg){
	 uint8_t temp;
	 BMI160_CSL();
	 SPI1_ReadWrite_Byte(0x00|0x80);
	 temp = SPI1_ReadWrite_Byte(0xff);
	 BMI160_CSH();
	 return temp;
}

//设置BMI160的寄存器，reg寄存器地址， data为要写入的值
static void mwriteReg(u8 reg, u8 data){
	BMI160_CSL();
	SPI1_writeReg(reg,data);
	BMI160_CSH();	
}

void BMI160_init(void)
{
		BMI160_Read_ID();
		mwriteReg(NV_CONF,0x01);               //SPI en	
		// mwriteReg(ACC_CONF,0x08);				//odr:100 Hz  
		mwriteReg(ACC_CONF,0x29);				//odr:200 Hz, crown change 17-7-20   
		// mwriteReg(ACC_FS,0x05);					//FS:+-4g
		mwriteReg(ACC_FS,0x03);					//FS:+-2g, crown change 17-7-20
		
		// mwriteReg(GRY_CONF,0x08);				//odr:100 Hz
		mwriteReg(GRY_CONF,0x29);				//odr:200 Hz, crown change 17-7-20 
		// mwriteReg(GRY_FS,0x00);					//FS:+-2000 dps
		mwriteReg(GRY_FS,0x03);					//FS:+-250 dps, crown change 17-7-20
		
		mwriteReg(PMU_TRIG,0x00);	
		mwriteReg(CMD,0x11);
		delay_ms(10);
		mwriteReg(CMD,0x15);
		delay_ms(10);
		mwriteReg(CMD,0x18);
		delay_ms(10);		
		BMI160_Read_Reg(PMU_STATUS);	//acc gyro:normal    mac:suspend

		Gx_offset = Config.dGx_offset ;//读取三个轴陀螺仪的零偏值。
		Gy_offset = Config.dGy_offset ;
		Gz_offset = Config.dGz_offset ;

		Ax_offset = Config.dAx_offset;
		Ay_offset = Config.dAy_offset;
		Az_offset = Config.dAz_offset;

		ax_scale = Config.dAx_scale;
		ay_scale = Config.dAy_scale;
		az_scale = Config.dAz_scale;
}


void BMI160_readAccGyro(int16_t *data){
	u8 buf[20];
	int16_t gx,gy,gz ,ax,ay ,az ;
    BMI160_CSL();
	SPI1_readRegs(0x0C,15,buf);
    BMI160_CSH();
	SensorTime = (uint32_t)((((uint32_t)buf[14]) << 16)|(((uint32_t)buf[13]) << 8) | buf[12]);
	BMI160_NewVal(&BMI160_FIFO[0][0],(int16_t)(((int16_t)buf[1]) << 8 | buf[0]));  //gx
	BMI160_NewVal(&BMI160_FIFO[1][0],(int16_t)(((int16_t)buf[3]) << 8 | buf[2]));  //gy
	BMI160_NewVal(&BMI160_FIFO[2][0],(int16_t)(((int16_t)buf[5]) << 8 | buf[4]));
	BMI160_NewVal(&BMI160_FIFO[3][0],(int16_t)(((int16_t)buf[7]) << 8 | buf[6]));  //ax
	BMI160_NewVal(&BMI160_FIFO[4][0],(int16_t)(((int16_t)buf[9]) << 8 | buf[8]));
	BMI160_NewVal(&BMI160_FIFO[5][0],(int16_t)(((int16_t)buf[11]) << 8 | buf[10]));	//az
	Wr_Index = (Wr_Index + 1) % Buf_SIZE;

	gx = BMI160_GetAvg(&BMI160_FIFO[0][0]);	//Gx
	gy = BMI160_GetAvg(&BMI160_FIFO[1][0]); //Gy
	gz = BMI160_GetAvg(&BMI160_FIFO[2][0]);	//Gz

	data[3] = gx - Gx_offset;  //减去偏置
	data[4] = gy - Gy_offset;
	data[5] = gz - Gz_offset;
	lastGx = data[3] ;	//更新数据。
	lastGy = data[4] ;
	lastGz = data[5] ;

	ax = BMI160_GetAvg(&BMI160_FIFO[3][0]);  //Ax
	ay = BMI160_GetAvg(&BMI160_FIFO[4][0]);  //Ay
	az = BMI160_GetAvg(&BMI160_FIFO[5][0]);  //Az
	ax = (int16_t)((float)(ax-Ax_offset) * ax_scale);
	ay = (int16_t)((float)(ay-Ay_offset) * ay_scale);
	az = (int16_t)((float)(az-Az_offset) * az_scale);
	data[0] = ax;
	data[1] = ay;
	data[2] = az;
	lastAx = data[0];
	lastAy = data[1];
	lastAz = data[2];
}

void BMI160_InitGyro_Offset(void){
	unsigned int i;
	int16_t temp[6];
	int32_t	tempgx=0,tempgy=0,tempgz=0;
	Gx_offset=0;	//
	Gy_offset=0;
	Gz_offset=0;
	Gyro_Off_started = 1;  

	LED_ON(); //LED常亮，表示正在采集静止时的角速度输出值
	for(i=0;i<Buf_SIZE;i++){ //刷新fifo数组。
  		delay_us(100);
		BMI160_readAccGyro(temp);
	}
 	for(i=0;i<10000;i++){  //连接采集10000个样本
		delay_us(200);
		BMI160_readAccGyro(temp);
		tempgx+= temp[3];
		tempgy+= temp[4];
		tempgz+= temp[5];
	}
	Gz_offset = tempgz/10000;
	Gx_offset = tempgx/10000;	 //取平均
	Gy_offset = tempgy/10000;		
	Config.dGx_offset = Gx_offset;
	Config.dGy_offset = Gy_offset;
	Config.dGz_offset = Gz_offset;
	Write_config();	  //将新的偏置值写入flash中保存
	Gyro_Off_started = 0;
	LED_OFF();
}

//取当前的加速度值。该子程序用于加速度标定
int16_t BMI160_get_ACCMAX(unsigned char ais){
	int32_t	tempax=0,tempay=0,tempaz=0;
	uint16_t i;
	int16_t temp[6];

	LED_ON();
	for(i=0;i<2000;i++){
		delay_us(500);
		BMI160_readAccGyro(temp);
		tempax+= temp[0];
		tempay+= temp[1];
		tempaz+= temp[2];
	}

	if(ais == 0){	//X轴
		return (int16_t)(tempax /2000);
	}else if(ais == 1){ //Y轴 
		return (int16_t)(tempay /2000);
	}else if(ais == 2){ //Z轴
		return (int16_t)(tempaz /2000);
	}
   LED_OFF();
   return 0;
}

//复位加速度标定
void BMI160_Reset_ACC_Offset(void){
    Ax_offset = 0;
	Ay_offset = 0;
	Az_offset = 0;
	ax_scale = 1.0;
	ay_scale = 1.0;
	az_scale = 1.0;
}

int16_t  AX_min,AX_max,AY_min,AY_max,AZ_min,AZ_max;
//计算并保存加速度计的偏置
void ACC_Save_cal(void){

	Config.dAx_offset = (AX_min + AX_max)/2;
	Config.dAy_offset = (AY_min + AY_max)/2;
	Config.dAz_offset = (AZ_min + AZ_max)/2;

	Config.dAx_scale = (Acc_Resolution/((float)AX_max - (float)AX_min))*2.0f;
	Config.dAy_scale = (Acc_Resolution/((float)AY_max - (float)AY_min))*2.0f;
	Config.dAz_scale = (Acc_Resolution/((float)AZ_max - (float)AZ_min))*2.0f;
	
	Ax_offset = Config.dAx_offset;
	Ay_offset = Config.dAy_offset;
	Az_offset = Config.dAz_offset;

	ax_scale = Config.dAx_scale;
	ay_scale = Config.dAy_scale;
	az_scale = Config.dAz_scale;

	Write_config();
}

//提取加速度计各个轴的最大 最小值
void ACC_Cal(unsigned char ch){

	switch(ch){
	case 0:
		AX_min = BMI160_get_ACCMAX(0);
		UART2_Send_ACC(AX_min,ch);	
		break;
	case 1:
		AX_max = BMI160_get_ACCMAX(0);	
		UART2_Send_ACC(AX_max,ch);
		break;
	case 2:
		AY_min = BMI160_get_ACCMAX(1);
		UART2_Send_ACC(AY_min,ch);	
		break;
	case 3:
		AY_max = BMI160_get_ACCMAX(1);	
		UART2_Send_ACC(AY_max,ch);
		break;
	case 4:
		AZ_min = BMI160_get_ACCMAX(2);	
		UART2_Send_ACC(AZ_min,ch);
		break;
	case 5:
		AZ_max = BMI160_get_ACCMAX(2);
		UART2_Send_ACC(AZ_max,ch);	
		break;
	default : break;
	}

}



//------------------End of File----------------------------
