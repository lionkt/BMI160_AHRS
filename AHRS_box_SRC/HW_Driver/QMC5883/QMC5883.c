
#include "math.h"
#include "QMC5883.h"
#include "eeprom.h"
#include "IMU.h"

//-------------------------------------------------------------------------

//磁力计标定值
int16_t  QMC5883_maxx=0,QMC5883_maxy=0,QMC5883_maxz=0,
		 QMC5883_minx=-0,QMC5883_miny=-0,QMC5883_minz=-0;
unsigned char QMC5883_calib = 0; //初始化完成标志

//磁力计标定值
static int16_t  Mag_Offset_X = 0, //偏置
				Mag_Offset_Y = 0,
				Mag_Offset_Z = 0;

static float  Mag_Scale_X = 1.0f, //灵敏度
	   		  Mag_Scale_Y = 1.0f,
	   		  Mag_Scale_Z = 1.0f;
int16_t  
		 lastMx,lastMy,lastMz; //最新的磁力计ADC值
float  magic_GRAVITY;

int16_t  QMC5883_FIFO[3][11]; //磁力计滤波
void QMC5883_getRaw(int16_t *x,int16_t *y,int16_t *z);


/**************************实现函数********************************************
*函数原型:	   void QMC5883_FIFO_init(void)
*功　　能:	   连续读取100次数据，以初始化FIFO数组
输入参数：  无
输出参数：  无
*******************************************************************************/
void QMC5883_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  QMC5883_getRaw(&temp[0],&temp[1],&temp[2]);
  delay_us(200);  //延时再读取数据
  }
}

/**************************实现函数********************************************
*函数原型:	   void  QMC5883_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  QMC5883_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		QMC5883_FIFO[0][i-1]=QMC5883_FIFO[0][i];
		QMC5883_FIFO[1][i-1]=QMC5883_FIFO[1][i];
		QMC5883_FIFO[2][i-1]=QMC5883_FIFO[2][i];
	}

	QMC5883_FIFO[0][9]=y;	//x
	QMC5883_FIFO[1][9]=-x;	  //轴对换
	QMC5883_FIFO[2][9]=z;	  //z

	sum=0;
	for(i=0;i<10;i++){	//取数组内的值进行求和再取平均
   		sum+=QMC5883_FIFO[0][i];
	}
	QMC5883_FIFO[0][10]=sum/10;	//将平均值更新

	sum=0;
	for(i=0;i<10;i++){
   		sum+=QMC5883_FIFO[1][i];
	}
	QMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=QMC5883_FIFO[2][i];
	}
	QMC5883_FIFO[2][10]=sum/10;

	if(QMC5883_calib){//校正有效的话 采集最大值最小值
		if(QMC5883_minx>QMC5883_FIFO[0][10])QMC5883_minx=(int16_t)QMC5883_FIFO[0][10];
		if(QMC5883_miny>QMC5883_FIFO[1][10])QMC5883_miny=(int16_t)QMC5883_FIFO[1][10];
		if(QMC5883_minz>QMC5883_FIFO[2][10])QMC5883_minz=(int16_t)QMC5883_FIFO[2][10];

		if(QMC5883_maxx<QMC5883_FIFO[0][10])QMC5883_maxx=(int16_t)QMC5883_FIFO[0][10];
		if(QMC5883_maxy<QMC5883_FIFO[1][10])QMC5883_maxy=(int16_t)QMC5883_FIFO[1][10];
		if(QMC5883_maxz<QMC5883_FIFO[2][10])QMC5883_maxz=(int16_t)QMC5883_FIFO[2][10];
	}

} //QMC5883_newValues

/**************************实现函数********************************************
*函数原型:	   void QMC5883_writeReg(unsigned char reg, unsigned char val)
*功　　能:	   写QMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void QMC5883_writeReg(unsigned char reg, unsigned char val) {
  IICwriteByte(QMC5883L_ADDR,reg,val);
}

/**************************实现函数********************************************
*函数原型:	  void QMC5883_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写QMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void QMC5883_getRaw(int16_t *x,int16_t *y,int16_t *z) {
  unsigned char vbuff[6] , i=100;
  vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
  if((I2C_ReadOneByte(QMC5883L_ADDR,0x06)&0x01) != 0){
     IICreadBytes(QMC5883L_ADDR,0x00,6,vbuff);
  }
  QMC5883_newValues(((int16_t)vbuff[1] << 8) | vbuff[0],((int16_t)vbuff[3] << 8) | vbuff[2],((int16_t)vbuff[5] << 8) | vbuff[4]); 
   *x = QMC5883_FIFO[0][10];
   *y = QMC5883_FIFO[1][10];
   *z = QMC5883_FIFO[2][10];
}

/**************************实现函数********************************************
*函数原型:	  void QMC5883_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针	
输出参数：  无
*******************************************************************************/
void QMC5883_getlastValues(int16_t *x,int16_t *y,int16_t *z) {
  *x = QMC5883_FIFO[0][10];
  *y = QMC5883_FIFO[1][10]; 
  *z = QMC5883_FIFO[2][10]; 
}

/**************************实现函数********************************************
*函数原型:	  void QMC5883_mgetValues(float *arry)
*功　　能:	   读取 校正后的 磁力计ADC值
输入参数：    输出数组指针	
输出参数：  无
*******************************************************************************/
uint32_t HMC5883_Last_Update = 0;
void QMC5883_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  uint32_t mtime = micros();  //读取时间
  //读取频率 220hz
  if((HMC5883_Last_Update==0)||((HMC5883_Last_Update+4500)<mtime)||(HMC5883_Last_Update > mtime)){
  QMC5883_getRaw(&xr, &yr, &zr);
  HMC5883_Last_Update = mtime;
  }else {
  	QMC5883_getlastValues(&xr, &yr, &zr);
  }

  arry[0] = (xr-Mag_Offset_X)*Mag_Scale_X;  //Mx
  arry[1] = (yr-Mag_Offset_Y)*Mag_Scale_Y;  //My
  arry[2] = (zr-Mag_Offset_Z)*Mag_Scale_Z;  //Mz
  lastMx = arry[0];
  lastMy = arry[1];
  lastMz = arry[2];
}

/**************************实现函数********************************************
*函数原型:	  void QMC5883_init(u8 setmode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void QMC5883_init(u8 setmode) {
  
  QMC5883_writeReg(0x09, 0xD9); // 8 gauss,
  QMC5883_writeReg(0x0A, 0x00); // -+1.3Ga	  1090LSB/Gauss
  QMC5883_writeReg(0x0B, 0x01);
  QMC5883_writeReg(0x20,0x40);
  QMC5883_writeReg(0x21,0x01);
  
  QMC5883L_update_config();
}

/**************************实现函数********************************************
*函数原型:	  void QMC5883L_SetUp(void)
*功　　能:	   初始化 QMC5883L 使之进入可用状态
输入参数：     	
输出参数：  无
*******************************************************************************/
void QMC5883L_SetUp(void)
{ 
	  char id[3];
	  QMC5883_init(0); //  -+8Ga
	  QMC5883_FIFO_init();
}

/**************************实现函数********************************************
*函数原型:	  void QMC5883L_Start_Calib(void)
*功　　能:	   进入磁力计标定
输入参数：     	
输出参数：  无
*******************************************************************************/
void QMC5883L_Start_Calib(void)
{
	QMC5883_calib = 1;//开始标定
	QMC5883_maxx = -32400;	//将原来的标定值清除
	QMC5883_maxy = -32400;
	QMC5883_maxz = -32400;
	QMC5883_minx = 32400;
	QMC5883_miny = 32400;
	QMC5883_minz = 32400;
	Mag_Scale_X = 1.0f;
	Mag_Scale_Y = 1.0f;
	Mag_Scale_Z = 1.0f;
	Mag_Offset_X = 0;
	Mag_Offset_Y = 0;
	Mag_Offset_Z = 0;
}

void QMC5883L_update_config(void){
  
	Mag_Offset_X = Config.dMx_offset;
	Mag_Offset_Y = Config.dMy_offset;
	Mag_Offset_Z = Config.dMz_offset;

	Mag_Scale_X = Config.dMx_scale ; 
  Mag_Scale_Y = Config.dMy_scale ;  
  Mag_Scale_Z = Config.dMz_scale ;
}

/**************************实现函数********************************************
*函数原型:	  void QMC5883L_Save_Calib(void)
*功　　能:	  保存磁力计标定值 到Flash
输入参数：     	
输出参数：  无
*******************************************************************************/
void QMC5883L_Save_Calib(void){

	if(QMC5883_maxx == QMC5883_minx)return ;
	if(QMC5883_maxy == QMC5883_miny)return ;
	if(QMC5883_maxz == QMC5883_minz)return ;  //磁力计有问题，或者根本就没有转过。
	//将磁力计标定值写入 Flash 保存
	Config.dMx_offset = (QMC5883_maxx+QMC5883_minx)/2;
	Config.dMy_offset = (QMC5883_maxy+QMC5883_miny)/2;
	Config.dMz_offset = (QMC5883_maxz+QMC5883_minz)/2;

	Config.dMx_scale = Mag_Scale_X = 1.0f;
	Config.dMy_scale = Mag_Scale_Y = (float)(QMC5883_maxx-QMC5883_minx)/(float)(QMC5883_maxy-QMC5883_miny);
	Config.dMz_scale = Mag_Scale_Z = (float)(QMC5883_maxx-QMC5883_minx)/(float)(QMC5883_maxz-QMC5883_minz); 
    
	Write_config();	 //将当前配置写入
	QMC5883_calib=0; //结束标定
}	//QMC5883L_Save_Calib()

//------------------End of File----------------------------
