
#include "stm32f4xx.h"
#include "LED.h"
#include "SPI2.h"
#include "IOI2C.h"
#include "UART2.h"
#include "UART1.h"
#include "MS5611.h"
#include "IMU.h"
#include "eeprom.h"
#include "Time3_Driver.h"
#include "STM32ADC.h"
#include "FAT_driver.h"
#include "usbd_usr.h"
#include "BMI160.h"

//上传数据的状态机
#define REIMU 0x01 //上传解算的姿态数据
#define REMOV 0x02 //上传传感器的输出
#define REHMC 0x03 //上传磁力计的标定值

#define Upload_Speed 100 //数据上传速率，hz

void STM32_Reset(void);
void ACC_Save_cal(void);
void ACC_Cal(unsigned char ch);
void USB_Check(void);

extern volatile u8 buf_lock;
extern uint8_t USB_Staut;
int16_t offset_yaw, offset_pitch, offset_roll;
int16_t upload_Hz = 10;
u16 UART_Upload_Mode;

u32 Cbaudrate, Upload_time;
volatile int16_t Math_hz = 0;
u8 state = REIMU, Start = 0, file_wr = 0;
;
float ypr[3] = {0.0, 0.0, 0.0}; // yaw pitch roll
extern u8 Divece_STA;
int main(void)
{
	float ypr[3];
	uint8_t flie_name[24];
	static u16 cly_count = 0;
	unsigned char PC_comm, SD_Ready = 0; //PC 命令关键字节
	u16 try
		= 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	Initial_LED_GPIO();
	LED_OFF();
	ADC_Voltage_initial();
	IIC_Init();
	delay_init(168);
	SPI1_Configuration();
	SPI2_Configuration();
	Initial_UART2(115200L);
	Initial_UART1(115200L);
	usbd_CloseMassStorage();
	delay_ms(200); //等待器件上电
	MS561101BA_init();
	load_config();
	while (SD_Init() != 0)
	{ // 初始化TF卡
		if (++try > 100)
			break;
	}

	if (try > 100)
	{
		SD_Ready = 0; //卡没有检测到
	}
	else
	{
		//有卡，是否进入U盘模式。
		usbd_OpenMassStorage();
		SD_Ready = 1;
		try
			= 0;
		/*
		while((USB_Staut != USB_OTG_CONFIGURED)&&(try++<2000)){
	   		delay_ms(1); //等待USB连接。2S
	   		MS5611BA_Routing();
			}
		if(try < 2000)
				USB_Check(); //U盘状态
		else  usbd_CloseMassStorage();	*/
	}

	if (SD_Ready == 1)
	{
		FAT_Initial();
		Get_file_name(flie_name);
		mf_open(flie_name, FA_WRITE | FA_OPEN_ALWAYS);
		File_head();

		Set_BeepON(200);
	}
	else
	{
		//beep响
		Set_BeepON(1500); //提示没有TF卡
	}
	// FileSave_TimerSet(100); //设置保存数据的频率，100hz（原始的频率为100Hz）
	FileSave_TimerSet(100); //设置保存数据的频率，crown change to 200hz
							//注意要和ACC_CONF以及GYR_CONF的ODR相匹配
	LED_OFF();
	IMU_init();
	Time3_Inttrup_init();
	//Tim3_Set_Speed(Upload_Speed);
	Tim3_Set_Speed(10);
	Tim3_Restart();
	Tim3_Stop();
	while (1)
	{
		if ((USB_Staut == USB_OTG_CONFIGURED))
		{
			mf_close();
			USB_Check(); //U盘状态
		}

		Scan_Key_Routing();
		if (Key_IS_Press)
		{
			Key_IS_Press = 0;
			mf_close(); // 重新创建新文件
			Get_file_name(flie_name);
			mf_open(flie_name, FA_WRITE | FA_OPEN_ALWAYS);
			File_head();
			Set_BeepON(100);
		}
		else
			File_Save_Routing();

		MS5611BA_Routing();
		IMU_getYawPitchRoll(ypr);		
		Math_hz++;
		cly_count++;
		if ((cly_count > 200) && (SD_Ready != 0))
		{
			//LED_Reverse();
			LED_Change();
			cly_count = 0;
		}
		else if (cly_count > 1000)
		{
			LED_Reverse();
			cly_count = 0;
		}

		//处理PC发送来的命令  UART2
		if ((PC_comm = UART2_CommandRoute()) != 0xff)
		{
			switch (PC_comm)
			{ //检查命令标识
			case Gyro_init:
				BMI160_InitGyro_Offset();
				break; //读取陀螺仪零偏
			case HMC_calib:
				QMC5883L_Save_Calib();
				break; //保存磁力计标定
			case High_init:
				MS561101BA_ResetAlt();
				break; //气压高度 清零
			case HMC_calib_begin:
				QMC5883L_Start_Calib();
				break; //开始磁力计标定
			case 0xA0:
				ACC_Cal(rx_buffer[2]);
				break;
			case 0xE4:
				ACC_Save_cal();
				break;
			case 0xE6:
				BMI160_Reset_ACC_Offset();
				break;
			}
		} // 处理PC 发送的命令

		//处理PC发送来的命令  UART1
		if ((PC_comm = UART1_CommandRoute()) != 0xff)
		{
			switch (PC_comm)
			{ //检查命令标识
			case Gyro_init:
				BMI160_InitGyro_Offset();
				break; //读取陀螺仪零偏
			case HMC_calib:
				QMC5883L_Save_Calib();
				break; //保存磁力计标定
			case High_init:
				MS561101BA_ResetAlt();
				break; //气压高度 清零
			case HMC_calib_begin:
				QMC5883L_Start_Calib();
				break; //开始磁力计标定
			case 0xA0:
				ACC_Cal(rx_buffer[2]);
				break;
			case 0xE4:
				ACC_Save_cal();
				break;
			case 0xE6:
				BMI160_Reset_ACC_Offset();
				break;
			}
		} // 处理PC 发送的命令

		//------------电源处理-----------
		while (Check_POW())
		{ //从机要求关机
			mf_close();
			while (1)
				LED_OFF(); //检测到关机信号 PB9 低电平
		}
		if (Is_BAT_LOW())
		{ //电池电压低于3.6V 提示关机
			LED_OFF();
			SET_POWER_Down();
			mf_close();
			while (1)
				;
		}
	}
}

void USB_Check(void)
{
	//	 LED闪烁时，正在读写SD卡

	if ((USB_Staut == USB_OTG_DEFAULT) || (USB_Staut == USB_OTG_SUSPENDED))
		return;
	BEEP_ON();
	delay_ms(100);
	BEEP_OFF();
	while (1)
	{
		if (Divece_STA == 0x01)
		{ //正在读写SD卡
			LED_OFF();
			delay_ms(50);
			Divece_STA = 0;
		}
		if (USB_Staut == USB_OTG_SUSPENDED)
		{
			USB_Disable();
			LED_OFF(); //USB挂起，LED灭，用户可以拔掉USB
			delay_ms(100);
			SET_POWER_Down(); //关机
			while (1)
				;
		}
		else
		{
			LED_ON();
		}
	}
}

/**************************实现函数********************************************
*函数原型:	   void Updata_PC_Route(void)
*功　　能:	   上传数据，将当然的姿态角度 和传感器的输出 发送到串口
				这个程序将由定时器4的溢出中断调用，以保证固定频率的数据输出
*******************************************************************************/
extern volatile int16_t Pdata[12];
extern uint32_t fdata_32t;
void Updata_PC_Route(void)
{

	switch (state)
	{
	case REIMU:
		UART2_ReportIMU((int16_t)(yaw * 10.0f), (int16_t)(pitch * 10.0f),
						(int16_t)(roll * 10.0f),
						MS5611_Altitude / 10,
						MS5611_Temperature / 10,
						MS5611_Pressure / 10,
						fdata_32t);
		UART2_ReportMotion(lastAx, lastAy, lastAz,
						   lastGx, lastGy, lastGz,
						   lastMx, lastMy, lastMz);
		state = REIMU;
		Math_hz = 0;
		file_wr = 1;
		if (QMC5883_calib)
		{
			UART2_ReportHMC(QMC5883_maxx, QMC5883_maxy, QMC5883_maxz,
							QMC5883_minx, QMC5883_miny, QMC5883_minz, 0); //发送标定值
		}
		break;

	default:

		state = REIMU;
		break;
	} //switch(state)
	Math_hz = 0;
}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles TIM3 global interrupt request.
定时器4的溢出中断程序，在程序中发送姿态数据
*******************************************************************************/
void TIM3_IRQHandler(void)
{
	if (TIM3->SR & 0X0001) //溢出中断
	{
		buf_lock = 1;
		Updata_PC_Route(); //发送数据
		buf_lock = 0;
	}
	TIM3->SR &= ~(1 << 0); //清除中断标志位
}

#ifdef USE_FULL_ASSERT

static void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/*****END OF FILE****/
