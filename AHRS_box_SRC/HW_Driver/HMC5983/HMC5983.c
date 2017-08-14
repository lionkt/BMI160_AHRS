
#include "math.h"
#include "HMC5983.h"
#include "eeprom.h"

//-------------------------------------------------------------------------
#define	HMC_CSH			GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define	HMC_CSL			GPIO_ResetBits(GPIOC, GPIO_Pin_2)

//磁力计标定值
int16_t  HMC5983_maxx=0,HMC5983_maxy=0,HMC5983_maxz=0,
		 HMC5983_minx=-0,HMC5983_miny=-0,HMC5983_minz=-0;
unsigned char HMC5983_calib = 0; //初始化完成标志

//磁力计标定值
static int16_t  Mag_Offset_X = 0, //偏置
				Mag_Offset_Y = 0,
				Mag_Offset_Z = 0;

static float  Mag_Scale_X = 1.0f, //灵敏度
	   		  Mag_Scale_Y = 1.0f,
	   		  Mag_Scale_Z = 1.0f;
int16_t  
		 lastMx,lastMy,lastMz; //最新的磁力计ADC值

int16_t  HMC5983_FIFO[3][11]; //磁力计滤波
void HMC5983_getRaw(int16_t *x,int16_t *y,int16_t *z);


/**************************实现函数********************************************
*函数原型:	   void HMC5983_FIFO_init(void)
*功　　能:	   连续读取100次数据，以初始化FIFO数组
输入参数：  无
输出参数：  无
*******************************************************************************/
void HMC5983_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC5983_getRaw(&temp[0],&temp[1],&temp[2]);
  delay_us(200);  //延时再读取数据
  }
}

/**************************实现函数********************************************
*函数原型:	   void  HMC5983_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  HMC5983_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5983_FIFO[0][i-1]=HMC5983_FIFO[0][i];
		HMC5983_FIFO[1][i-1]=HMC5983_FIFO[1][i];
		HMC5983_FIFO[2][i-1]=HMC5983_FIFO[2][i];
	}

	HMC5983_FIFO[0][9]=y;	//x
	HMC5983_FIFO[1][9]=-x;	  //轴对换
	HMC5983_FIFO[2][9]=z;	  //z

	sum=0;
	for(i=0;i<10;i++){	//取数组内的值进行求和再取平均
   		sum+=HMC5983_FIFO[0][i];
	}
	HMC5983_FIFO[0][10]=sum/10;	//将平均值更新

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5983_FIFO[1][i];
	}
	HMC5983_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5983_FIFO[2][i];
	}
	HMC5983_FIFO[2][10]=sum/10;

	if(HMC5983_calib){//校正有效的话 采集最大值最小值
		if(HMC5983_minx>HMC5983_FIFO[0][10])HMC5983_minx=(int16_t)HMC5983_FIFO[0][10];
		if(HMC5983_miny>HMC5983_FIFO[1][10])HMC5983_miny=(int16_t)HMC5983_FIFO[1][10];
		if(HMC5983_minz>HMC5983_FIFO[2][10])HMC5983_minz=(int16_t)HMC5983_FIFO[2][10];

		if(HMC5983_maxx<HMC5983_FIFO[0][10])HMC5983_maxx=(int16_t)HMC5983_FIFO[0][10];
		if(HMC5983_maxy<HMC5983_FIFO[1][10])HMC5983_maxy=(int16_t)HMC5983_FIFO[1][10];
		if(HMC5983_maxz<HMC5983_FIFO[2][10])HMC5983_maxz=(int16_t)HMC5983_FIFO[2][10];
	}

} //HMC5983_newValues

/**************************实现函数********************************************
*函数原型:	   void HMC5983_writeReg(unsigned char reg, unsigned char val)
*功　　能:	   写HMC5983L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC5983_writeReg(unsigned char reg, unsigned char val) {
  HMC_CSL;
  SPI1_ReadWrite_Byte(reg);
  SPI1_ReadWrite_Byte(val);
  HMC_CSH;
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5983L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC5983_getRaw(int16_t *x,int16_t *y,int16_t *z) {
  unsigned char vbuff[6] , i=100;
  vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
  HMC_CSL;
  SPI1_ReadWrite_Byte(HMC5983_R_XM | 0xC0);
  for(i=0;i<6;i++)
  vbuff[i] = SPI1_ReadWrite_Byte(0xFF);
  HMC_CSH;
  HMC5983_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)vbuff[4] << 8) | vbuff[5],((int16_t)vbuff[2] << 8) | vbuff[3]); 
   *x = HMC5983_FIFO[0][10];
   *y = HMC5983_FIFO[1][10];
   *z = HMC5983_FIFO[2][10];
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针	
输出参数：  无
*******************************************************************************/
void HMC5983_getlastValues(int16_t *x,int16_t *y,int16_t *z) {
  *x = HMC5983_FIFO[0][10];
  *y = HMC5983_FIFO[1][10]; 
  *z = HMC5983_FIFO[2][10]; 
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983_mgetValues(float *arry)
*功　　能:	   读取 校正后的 磁力计ADC值
输入参数：    输出数组指针	
输出参数：  无
*******************************************************************************/
uint32_t HMC5883_Last_Update = 0;
void HMC5983_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  uint32_t mtime = micros();  //读取时间
  //读取频率 220hz
  if((HMC5883_Last_Update==0)||((HMC5883_Last_Update+4500)<mtime)||(HMC5883_Last_Update > mtime)){
  HMC5983_getRaw(&xr, &yr, &zr);
  HMC5883_Last_Update = mtime;
  }else {
  	HMC5983_getlastValues(&xr, &yr, &zr);
  }

  arry[0] = (xr-Mag_Offset_X)*Mag_Scale_X;  //Mx
  arry[1] = (yr-Mag_Offset_Y)*Mag_Scale_Y;  //My
  arry[2] = (zr-Mag_Offset_Z)*Mag_Scale_Z;  //Mz
  lastMx = arry[0];
  lastMy = arry[1];
  lastMz = arry[2];
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983_setGain(unsigned char gain)
*功　　能:	   设置 5883L的增益
输入参数：     目标增益 0-7
输出参数：  无
*******************************************************************************/
void HMC5983_setGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  HMC5983_writeReg(HMC5983_R_CONFB, gain << 5);
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983_setMode(unsigned char mode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void HMC5983_setMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  HMC5983_writeReg(HMC5983_R_MODE, mode);
  delay_us(100);
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983_init(u8 setmode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void HMC5983_init(u8 setmode) {
  if (setmode) {
    HMC5983_setMode(0);
  }

  HMC5983_writeReg(HMC5983_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  HMC5983_writeReg(HMC5983_R_CONFB, 0x20); // -+1.3Ga	  1090LSB/Gauss
  HMC5983_writeReg(HMC5983_R_MODE, 0x00);
  HMC5983L_update_config();
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983_setDOR(unsigned char DOR)
*功　　能:	   设置 5983L的 数据输出速率
输入参数：     速率值
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 220Hz  
输出参数：  无
*******************************************************************************/
void HMC5983_setDOR(unsigned char DOR) {
	  if (DOR>6) return;
	  HMC5983_writeReg(HMC5983_R_CONFA,DOR<<2);
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983_getID(char id[3])
*功　　能:	   读取芯片的ID
输入参数：     	ID存放的数组
输出参数：  无
*******************************************************************************/
void HMC5983_getID(char id[3]) 
{
	HMC_CSL;
  	SPI1_ReadWrite_Byte(HMC5983_R_IDA | 0xC0);
  	id[0] = SPI1_ReadWrite_Byte(0xff);
	id[1] = SPI1_ReadWrite_Byte(0xff);
	id[2] = SPI1_ReadWrite_Byte(0xff);
  	HMC_CSH;
}   // getID().

/**************************实现函数********************************************
*函数原型:	  void HMC5983L_SetUp(void)
*功　　能:	   初始化 HMC5983L 使之进入可用状态
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5983L_SetUp(void)
{ 
	  char id[3];
	  HMC5983_getID(id);
	  HMC5983_init(0); //  -+1.3Ga
	  HMC5983_setMode(0);
	  HMC5983_setDOR(6);  //220hz 更新率
	  HMC5983_FIFO_init();
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983L_Start_Calib(void)
*功　　能:	   进入磁力计标定
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5983L_Start_Calib(void)
{

	HMC5983_calib = 1;//开始标定
	HMC5983_maxx = -4096;	//将原来的标定值清除
	HMC5983_maxy = -4096;
	HMC5983_maxz = -4096;
	HMC5983_minx = 4096;
	HMC5983_miny = 4096;
	HMC5983_minz = 4096;
	Mag_Scale_X = 1.0f;
	Mag_Scale_Y = 1.0f;
	Mag_Scale_Z = 1.0f;
	Mag_Offset_X = 0;
	Mag_Offset_Y = 0;
	Mag_Offset_Z = 0;
}

void HMC5983L_update_config(void){
	Mag_Offset_X = Config.dMx_offset;
	Mag_Offset_Y = Config.dMy_offset;
	Mag_Offset_Z = Config.dMz_offset;

	Mag_Scale_X = Config.dMx_scale ; 
    Mag_Scale_Y = Config.dMy_scale ;  
    Mag_Scale_Z = Config.dMz_scale ;
}

/**************************实现函数********************************************
*函数原型:	  void HMC5983L_Save_Calib(void)
*功　　能:	  保存磁力计标定值 到Flash
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5983L_Save_Calib(void){

	if(HMC5983_maxx == HMC5983_minx)return ;
	if(HMC5983_maxy == HMC5983_miny)return ;
	if(HMC5983_maxz == HMC5983_minz)return ;  //磁力计有问题，或者根本就没有转过。
	//将磁力计标定值写入 Flash 保存
	Config.dMx_offset = (HMC5983_maxx+HMC5983_minx)/2;
	Config.dMy_offset = (HMC5983_maxy+HMC5983_miny)/2;
	Config.dMz_offset = (HMC5983_maxz+HMC5983_minz)/2;

	Config.dMx_scale = Mag_Scale_X = 1.0f;
	Config.dMy_scale = Mag_Scale_Y = (float)(HMC5983_maxx-HMC5983_minx)/(float)(HMC5983_maxy-HMC5983_miny);
	Config.dMz_scale = Mag_Scale_Z = (float)(HMC5983_maxx-HMC5983_minx)/(float)(HMC5983_maxz-HMC5983_minz); 

	Write_config();	 //将当前配置写入
	HMC5983_calib=0; //结束标定
}	//HMC5983L_Save_Calib()

//------------------End of File----------------------------
