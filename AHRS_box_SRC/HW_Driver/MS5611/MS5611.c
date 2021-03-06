/* MS5611.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-06-25
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

占用资源：
1. I2C 接口访问MS561101BA
2. 读取 当前系统的时间  以确定是否完成了转换

功能：
提供MS5611 初始化 控制 读取温度 气压 API
------------------------------------
 */
#include "MS5611.h"
#include <math.h>

#define MS5611Press_OSR  MS561101BA_OSR_4096  //气压采样精度
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //温度采样精度

// 气压计状态机
#define SCTemperature  0x01	  //开始 温度转换
#define CTemperatureing  0x02  //正在转换温度
#define SCPressure  0x03	  //开始转换 气压
#define SCPressureing  0x04	  //正在转换气压值

#define MOVAVG_SIZE  10	   //保存最近 10组数据

const float  MS5611_Lowpass = 7.9577e-3f;  //10hz

static uint8_t  Now_doing = SCTemperature;	//当前转换状态
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT]; //标定值存放
static uint32_t Current_delay;	   //转换延时时间 us 
static uint32_t Start_Convert_Time;   //启动转换时的 时间 us 
static int32_t  tempCache;
uint8_t ALT_Updated = 0; //气压计高度更新完成标志。
static float Alt_Offset_cm = 0;
static float avg_Pressure;
float ALT_Update_Interval = 0.0; //两次高度测量，之间的时间间隔

//units (Celsius degrees*100, mbar*100  ).
//单位 [温度 0.01度] [气压 帕]  [高度0.01米] 
float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude;

// 延时表  单位 us 	  不同的采样精度对应不同的延时值
uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	11000,//MS561101BA_OSR_4096 9.1ms 0x08
};

// FIFO 队列					
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0; //队列指针
static uint8_t  pass_init=0,temp_init=0;

//添加一个新的值到 温度队列 进行滤波
void MS561101BA_NewTemp(float val) {
  int i;
  if(temp_init==0){	//第一个数据。用于初始化fifo
	 for(i=0;i<MOVAVG_SIZE;i++)
	 	Temp_buffer[i] = val;
	 temp_index = (temp_index + 1) % MOVAVG_SIZE;
	 temp_init = 1;
	 return;
  }
  Temp_buffer[temp_index] = val;
  temp_index = (temp_index + 1) % MOVAVG_SIZE;
}

//添加一个新的值到 气压队列 进行滤波
void MS561101BA_NewPress(float val) {
  int i;
  if(pass_init==0){	//第一个数据。用于初始化fifo
	 for(i=0;i<MOVAVG_SIZE;i++)
	 	Press_buffer[i] = val;
	 press_index = (press_index + 1) % MOVAVG_SIZE;
	 pass_init = 1;
	 return;
  }
  Press_buffer[press_index] = val;
  press_index = (press_index + 1) % MOVAVG_SIZE;
}

//添加一个新的值到 高度队列 进行滤波
void MS561101BA_NewAlt(float val) {
  int16_t i;
  static uint32_t alt_lastupdate , temp;
  temp = micros();
  ALT_Update_Interval = ((float)(temp - alt_lastupdate))/1000000.0f;
  alt_lastupdate = temp;
  for(i=1;i<MOVAVG_SIZE;i++)
  Alt_buffer[i-1] = Alt_buffer[i];
  Alt_buffer[MOVAVG_SIZE-1] = val;
}

//取气压计的D变化率
float MS5611BA_Get_D(void){
	float new=0,old=0;
	int16_t i;
	for(i=0;i<MOVAVG_SIZE/2;i++)
		old += Alt_buffer[i];
	old /= (MOVAVG_SIZE/2);

	for(i=MOVAVG_SIZE/2;i<MOVAVG_SIZE;i++)
	    new += Alt_buffer[i];
	new /= (MOVAVG_SIZE/2);

	return new - old;
}

//读取队列 的平均值
float MS561101BA_getAvg(float * buff, int size) {
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_readPROM(void)
*功　　能:	    读取 MS561101B 的工厂标定值
读取 气压计的标定值  用于修正温度和气压的读数
*******************************************************************************/
void MS561101BA_readPROM(void) {
  u8  inth,intl;
  int i;
  for (i=0;i<MS561101BA_PROM_REG_COUNT;i++) {
		IIC_Start();
    	IIC_Send_Byte(MS5611_ADDR);
		IIC_Wait_Ack();
    	IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
		IIC_Wait_Ack();	
    	IIC_Stop();
		delay_us(5);
   		IIC_Start();
		IIC_Send_Byte(MS5611_ADDR+1);  //进入接收模式	
		delay_us(1);
		IIC_Wait_Ack();
		inth = IIC_Read_Byte(1);  //带ACK的读数据
		delay_us(1);
		intl = IIC_Read_Byte(0);	 //最后一个字节NACK
		IIC_Stop();
    PROM_C[i] = (((uint16_t)inth << 8) | intl);
  }
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_reset(void)
*功　　能:	    发送复位命令到 MS561101B 
*******************************************************************************/
void MS561101BA_reset(void) {
	IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); //写地址
	IIC_Wait_Ack();
    IIC_Send_Byte(MS561101BA_RESET);//发送复位命令
	IIC_Wait_Ack();	
    IIC_Stop();
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_startConversion(uint8_t command)
*功　　能:	    发送启动转换命令到 MS561101B
可选的 转换命令为 MS561101BA_D1  转换气压
				  MS561101BA_D2  转换温度	 
*******************************************************************************/
void MS561101BA_startConversion(uint8_t command) {
  // initialize pressure conversion
  IIC_Start();
  IIC_Send_Byte(MS5611_ADDR); //写地址
  IIC_Wait_Ack();
  IIC_Send_Byte(command); //写转换命令
  IIC_Wait_Ack();	
  IIC_Stop();
}

/**************************实现函数********************************************
*函数原型:		unsigned long MS561101BA_getConversion(void)
*功　　能:	    读取 MS561101B 的转换结果	 
*******************************************************************************/
unsigned long MS561101BA_getConversion(void) {
		unsigned long conversion = 0;
		u8 temp[3];
		IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); //写地址
		IIC_Wait_Ack();
    IIC_Send_Byte(0);// start read sequence
		IIC_Wait_Ack();	
    IIC_Stop();
		
		IIC_Start();
		IIC_Send_Byte(MS5611_ADDR+1);  //进入接收模式	
		IIC_Wait_Ack();
		temp[0] = IIC_Read_Byte(1);  //带ACK的读数据  bit 23-16
		temp[1] = IIC_Read_Byte(1);  //带ACK的读数据  bit 8-15
		temp[2] = IIC_Read_Byte(0);  //带NACK的读数据 bit 0-7
		IIC_Stop();
		conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
		return conversion;
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_init(void)
*功　　能:	    初始化 MS561101B 
*******************************************************************************/
void MS561101BA_init(void) {  
  MS561101BA_reset(); // 复位 MS561101B 
  delay_ms(100); // 延时 
  MS561101BA_readPROM(); // 读取EEPROM 中的标定值 待用	
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_GetTemperature(void)
*功　　能:	    读取 温度转换结果	 
*******************************************************************************/
void MS561101BA_GetTemperature(void){	
	tempCache = MS561101BA_getConversion();	
}

float Alt_offset_Pa=0; //存放着0米时 对应的气压值  这个值存放上电时的气压值 
uint8_t  Covert_count=0;
/**************************实现函数********************************************
*函数原型:		float MS561101BA_get_altitude(void)
*功　　能:	    将当前的气压值转成 高度。	 
*******************************************************************************/
float MS561101BA_get_altitude(void){

	static float Altitude;
	if(Alt_offset_Pa==0){ // 是否初始化过0米气压值？
		if(Covert_count++<50);  //等待气压稳定 后 再取零米时的气压值
		else Alt_offset_Pa = MS5611_Pressure; //把 当前气压值保存成 0 米时的气压
		avg_Pressure = MS5611_Pressure;
		Altitude = 0; //高度 为 0
		return Altitude;
	}
	//计算相对于 上电时的位置的 高度值 。
	Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903));
	Altitude = Altitude + Alt_Offset_cm ;  //加偏置
	MS561101BA_NewAlt(Altitude);
	Altitude = MS561101BA_getAvg(Alt_buffer,MOVAVG_SIZE);
	return (Altitude);
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_ResetAlt(void)
*功　　能:	    将当前的气压做为0米时的气压。	 
*******************************************************************************/
void MS561101BA_ResetAlt(void){
	Alt_offset_Pa = MS5611_Pressure; //把 当前气压值保存成 0 米时的气压	
	Alt_Offset_cm = 0;
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_SetAlt(void)
*功　　能:	    将当前的气压做为 Current 米时的气压。	 
*******************************************************************************/
void MS561101BA_SetAlt(float Current){
	Alt_offset_Pa = (avg_Pressure); //把 当前气压值保存成 0 米时的气压	
	Alt_Offset_cm = Current*100.0f; //米转成 CM
	MS561101BA_NewAlt(Current*100.0f);	 //新的高度值
	ALT_Updated = 1; //高度更新 完成。
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_getPressure(void)
*功　　能:	    读取 气压转换结果 并做补偿修正	 
*******************************************************************************/
void MS561101BA_getPressure(void) {
	int64_t off,sens;
	int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
	int32_t rawPress = MS561101BA_getConversion();
	int64_t dT  = tempCache - (((int32_t)PROM_C[4]) << 8);
	float temp;
	TEMP = 2000 + (dT * (int64_t)PROM_C[5])/8388608;
	off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
	sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);
	
	if (TEMP < 2000){   // second order temperature compensation
		T2 = (((int64_t)dT)*dT) >> 31;
		Aux_64 = (TEMP-2000)*(TEMP-2000);
		OFF2 = (5*Aux_64)>>1;
		SENS2 = (5*Aux_64)>>2;
		TEMP = TEMP - T2;
		off = off - OFF2;
		sens = sens - SENS2;
	}

	MS561101BA_NewPress((((((int64_t)rawPress) * sens) >> 21) - off) / 32768);
    MS5611_Pressure = MS561101BA_getAvg(Press_buffer,MOVAVG_SIZE); //0.01mbar
	
	avg_Pressure = avg_Pressure + (MS5611_Pressure - avg_Pressure)*0.1f;

	MS561101BA_NewTemp(TEMP);
	MS5611_Temperature = MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE); //0.01c
	
	temp = MS561101BA_get_altitude(); // 0.01meter
							  
	MS5611_Altitude = MS5611_Altitude +	  //低通滤波   20hz
	 (ALT_Update_Interval/(ALT_Update_Interval + MS5611_Lowpass))*(temp - MS5611_Altitude);
	
}


/**************************实现函数********************************************
*函数原型:		void MS5611BA_Routing(void)
*功　　能:	    MS5611BA 的运行程序 ，需要定期调用 以更新气压值和温度值 	 
*******************************************************************************/
void MS5611BA_Routing(void) {

	switch(Now_doing){ //查询状态 看看我们现在 该做些什么？
		case SCTemperature:  //启动温度转换
			MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
			Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//转换时间
			Start_Convert_Time = micros(); //计时开始
			Now_doing = CTemperatureing;//下一个状态
			break;
		case CTemperatureing:  //正在转换中 
			if((micros()-Start_Convert_Time) > Current_delay){ //延时时间到了吗？
			MS561101BA_GetTemperature(); //取温度	
			Now_doing = SCPressure;	
			}
			break;
		case SCPressure: //启动气压转换
			MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
			Current_delay = MS5611_Delay_us[MS5611Press_OSR];//转换时间
			Start_Convert_Time = micros();//计时开始
			Now_doing = SCPressureing;//下一个状态
			break;
		case SCPressureing:	 //正在转换气压值
			if((micros()-Start_Convert_Time) > Current_delay){ //延时时间到了吗？
			MS561101BA_getPressure();  //更新 	
			ALT_Updated = 1; //高度更新 完成。
			Now_doing = SCTemperature; //从头再来	
			}
			break;
		default: 
			Now_doing = SCTemperature;
			break;
	}
}

//------------------End of File----------------------------
