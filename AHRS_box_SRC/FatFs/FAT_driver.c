#include "FAT_driver.h"
#include "UART1.h"
#include  "IMU.h"

#define FILE_BUF_LENGTH 3	//crown add，定义文件缓冲区的长度

FATFS fs;  		//逻辑磁盘工作区.	 
FIL file;	  		//文件1
FIL ftemp;	  		//文件2.
UINT br,bw;			//读写变量
FILINFO fileinfo;	//文件信息
DIR dir;  			//目录

u8 fatbuf[512];			//SD卡数据缓存区
u8 res=0;
u32 FileSave_DelayC = 2000;

void FAT_Initial(void){
	u16 try = 0;
	while(SD_Init()){
		if(++try > 100)return;
	}
	res = 1;try = 0;
	while((res != 0)&&(try++ < 100)){
		res = f_mount(&fs,"0:",1);
		}
}

//打开路径下的文件
//path:路径+文件名
//mode:打开模式
//返回值:执行结果
u8 mf_open(u8 *path,u8 mode)
{
	u16 try = 0;
	res = 1;
	while((res != 0)&&(try++ < 100)){
		res = f_open(&file,(TCHAR*)path,mode);//打开文件
		}
	return res;
} 
//关闭文件
//返回值:执行结果
u8 mf_close(void)
{
	f_close(&file);
	return 0;
}
//读出数据
//len:读出的长度
//返回值:执行结果
u8 mf_read(u16 len)
{
	u16 i;
	u16 tlen=0;
	for(i=0;i<len/512;i++)
	{
		res=f_read(&file,fatbuf,512,&br);
		if(res)
		{
			break;
		}else
		{
			tlen+=br;
		}
	}
	if(len%512)
	{
		res=f_read(&file,fatbuf,len%512,&br);
		if(res)	//读数据出错了
		{  
		}else
		{
			tlen+=br; 
		}	 
	} 
	return res;
}
//写入数据
//dat:数据缓存区
//len:写入长度
//返回值:执行结果
u8 mf_write(u8*dat,u16 len)
{			    				   	 
	res = f_write(&file,dat,len,&bw);
	//if(res != 0){UART1_Putw_Dec(res);UART1_Put_String("f_write err\r\n");}
	return res;
}

//文件读写指针偏移
//offset:相对首地址的偏移量
//返回值:执行结果.
u8 mf_lseek(u32 offset)
{
	return f_lseek(&file,offset);
}

//设置数据保存的频率
void FileSave_TimerSet(u16 Speed)
{
	//系统是84Mhz  84M/（8399+1）=10000，
	//计数器装的是500，*10=5000/10000=0.05s
	//声明一个定时器的结构体变量
	NVIC_InitTypeDef NVIC_InitStructure;
 	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//开外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
 	//计数器装载的值
 	TIM_TimeBaseStructure.TIM_Period = 10000/Speed - 1;  
 	//预分频的值，设置值减1
 	TIM_TimeBaseStructure.TIM_Prescaler = 8399; 
 	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
 	//计数器计数方向
 	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	//开中断，更新中断。
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
	
 	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; 
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
 	NVIC_Init(&NVIC_InitStructure); 

	TIM4->CNT = 0;
	TIM4->CR1 |= 0x01;    //使能定时器
}

void FileSave_Stop(void){
   TIM4->CNT = 0;
   TIM4->CR1 &= 0xFFFE;    //停止定时器
}

extern float  pitch ,roll ,yaw;
//  Temperature in 0.01C
//  Pressure    in 0.01mbar = Pa
//  Altitude    in meter  / cm
extern float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude;
extern int16_t lastGx,lastGy,lastGz;//最新的三轴角速度ADC值。
extern int16_t  lastAx,lastAy,lastAz,  //最新的加速度ADC值
		 		lastMx,lastMy,lastMz;  //最新的磁力计ADC值
extern int16_t lastVOL;  //读取当前的电池电压值， 单位 0.01V
uint32_t dtime;
int16_t fdata_t;
uint32_t fdata_32t;
unsigned char File_Data[45];
unsigned char File_Data_buf[FILE_BUF_LENGTH*45];	//crown change，扩展到FILE_BUF_LENGTH个File_Data
unsigned char Data_Ready = 0;
unsigned char toFile_ptr = 0;					//crown add,文件缓冲区的指针，指向缓冲区包含内容的起始位置
unsigned char toBuf_ptr = 0;					//crown add,文件缓冲区的指针，指向缓冲区空余的起始位置
extern void Updata_PC_Route(void);
void TIM4_IRQHandler(void)					//数据发送定时器。在发送的同时，将要保存的数据加载到缓冲区，准备写入SD卡
{
static int i,sum;
if (TIM4->SR&0X0001)//溢出中断
	{
        if(FileSave_DelayC != 0){
            FileSave_DelayC --;
            TIM4->SR&=~(1<<0);
            return;
        }
        
	dtime = micros();
	File_Data[0] = 0xA5;
	File_Data[1] = 0x5A;
	File_Data[2] = sizeof(File_Data)-2;
	File_Data[3] = 0xDF;  //数据识别字节
	fdata_t = (int16_t)(yaw*10.0f);
	File_Data[4] = fdata_t>>8;
	File_Data[5] = fdata_t;

	fdata_t = (int16_t)(pitch*10.0f);
	File_Data[6] = fdata_t>>8;
	File_Data[7] = fdata_t;

	fdata_t = (int16_t)(roll*10.0f);
	File_Data[8] = fdata_t>>8;
	File_Data[9] = fdata_t;
	//0.01C
	fdata_t = (int16_t)MS5611_Temperature;
	File_Data[10] = fdata_t>>8;
	File_Data[11] = fdata_t;
	//Pa
	fdata_32t = (int32_t)MS5611_Pressure;
	File_Data[12] = fdata_32t>>24;
	File_Data[13] = fdata_32t>>16;
	File_Data[14] = fdata_32t>>8;
	File_Data[15] = fdata_32t;
	//0.01m
	fdata_32t = (int32_t)MS5611_Altitude;
	File_Data[16] = fdata_32t>>24;
	File_Data[17] = fdata_32t>>16;
	File_Data[18] = fdata_32t>>8;
	File_Data[19] = fdata_32t;
	//0.01V
	fdata_t = (int16_t)lastVOL;
	File_Data[20] = fdata_t>>8;
	File_Data[21] = fdata_t;

	fdata_t = (int16_t)lastAx;
	File_Data[22] = fdata_t>>8;
	File_Data[23] = fdata_t;

	fdata_t = (int16_t)lastAy;
	File_Data[24] = fdata_t>>8;
	File_Data[25] = fdata_t;

	fdata_t = (int16_t)lastAz;
	File_Data[26] = fdata_t>>8;
	File_Data[27] = fdata_t;

	fdata_t = (int16_t)lastGx;
	File_Data[28] = fdata_t>>8;
	File_Data[29] = fdata_t;

	fdata_t = (int16_t)lastGy;
	File_Data[30] = fdata_t>>8;
	File_Data[31] = fdata_t;

	fdata_t = (int16_t)lastGz;
	File_Data[32] = fdata_t>>8;
	File_Data[33] = fdata_t;

	fdata_t = (int16_t)lastMx;
	File_Data[34] = fdata_t>>8;
	File_Data[35] = fdata_t;

	fdata_t = (int16_t)lastMy;
	File_Data[36] = fdata_t>>8;
	File_Data[37] = fdata_t;

	fdata_t = (int16_t)lastMz;
	File_Data[38] = fdata_t>>8;
	File_Data[39] = fdata_t;
	//Time  us
	fdata_32t = dtime;
	File_Data[40] = fdata_32t>>24;
	File_Data[41] = fdata_32t>>16;
	File_Data[42] = fdata_32t>>8;
	File_Data[43] = fdata_32t;

	sum = 0;
	for(i=2;i<sizeof(File_Data)-1;i++)
		sum += File_Data[i]; 
		
	File_Data[44] = sum;

	if(Data_Ready == 0x0){ //确认数据已保存
	for(i=0;i<sizeof(File_Data);i++) //复制数据到缓冲区
		File_Data_buf[toBuf_ptr*45 + i] = File_Data[i];		//crown change
	}
	toBuf_ptr = (toBuf_ptr+1)%FILE_BUF_LENGTH;	//crown add
	Updata_PC_Route();	
	Data_Ready = 1;
	//LED_Reverse();
	}  
	TIM4->SR&=~(1<<0);//清除中断标志位  	
}

void File_head(void){
	uint16_t  temp,sum=0,i;
	temp = Gyro_Resolution;
	File_Data[0] = temp>>8;
	File_Data[1] = temp;

	temp = Acc_Resolution;
	File_Data[2] = temp>>8;
	File_Data[3] = temp;

	temp = Mag_Resolution;
	File_Data[4] = temp>>8;
	File_Data[5] = temp;
	sum = 0;
	for(i=0;i<6;i++)sum += File_Data[i];
	File_Data[6] = sum;
	mf_write(File_Data,7);
	Data_Ready = 0;
    
    if(FileSave_DelayC == 0)
        FileSave_DelayC = 100;
}

//定时调用，以确定是否有数据需要写到文件
void File_Save_Routing(void){
     
	 if(Data_Ready){
		// crown change
		if(toBuf_ptr <= toFile_ptr)
		{
			int start_index = toFile_ptr*45;
			if(mf_write(File_Data_buf + start_index,sizeof(File_Data*(FILE_BUF_LENGTH-toFile_ptr))==0);		//先写一段
			if(mf_write(File_Data_buf,sizeof(File_Data*(toBuf_ptr))==0);		//再写令一段

		}
		else
		{
			int start_index = toFile_ptr*45;
			int temp_write_length = toBuf_ptr - toFile_ptr;
			if(mf_write(File_Data_buf+start_index,sizeof(File_Data*temp_write_length))==0);				//这种情况下可以一次写完
		}
		toFile_ptr = toBuf_ptr;		//crown add,更新toFile_ptr指针的位置
		Data_Ready = 0;
	}

}
