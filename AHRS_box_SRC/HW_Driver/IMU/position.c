
#include "position.h"

#define bufSize  100
#define ADC_to_mss  (9.8f/Acc_Resolution)

#define filter_acc_err   0.1f  // g
#define filter_gyro      0.4f  // dps

float pos_accx_mss,pos_accy_mss,pos_accz_mss;
float pos_accx_offset ,pos_accy_offset ,pos_accz_offset ;
float now_accx,now_accy,now_accz;
float Speed_x,Speed_y,Speed_z;
float pos_x,pos_y,pos_z;

float accg,gyrodps;
uint8_t  Moving = 0;

static int16_t eax[bufSize],eay[bufSize],eaz[bufSize];
static int16_t egx[bufSize],egy[bufSize],egz[bufSize];
static float cal_Accx[bufSize],cal_Accy[bufSize],cal_Accz[bufSize];
static int16_t index = -1;


extern void UART1_ReportMotion2(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz) ;

//初始化
void Initial_Pos(void){
	pos_x = 0;
	pos_y = 0;
	pos_z = 0;
	Speed_x = 0;
	Speed_y = 0;
	Speed_z = 0;
	pos_accx_offset = 0;
	pos_accy_offset = 0;
	pos_accz_offset = 0;
}

//在模块静止时调用。
void Reset_speed(void){
	int i;
	float sum;
	Speed_x = 0;  //复位速度
	Speed_y = 0;
	Speed_z = 0;
	sum = 0;		   //将当前加速度做为偏置
	for(i=0;i<bufSize;i++)sum += cal_Accx[i];
	pos_accx_offset = sum/bufSize;

	sum = 0;
	for(i=0;i<bufSize;i++)sum += cal_Accy[i];
	pos_accy_offset = sum/bufSize;

	sum = 0;
	for(i=0;i<bufSize;i++)sum += cal_Accz[i];
	pos_accz_offset = sum/bufSize;
}

void Estimate_Posi(float dt){

	//积分位置
    pos_x += Speed_x * dt; 
	pos_y += Speed_y * dt;
	pos_z += Speed_z * dt;
	now_accx = (pos_accx_mss - pos_accx_offset);
	now_accy = (pos_accy_mss - pos_accy_offset);
	now_accz = (pos_accz_mss - pos_accz_offset);

	// 积分速度
	Speed_x += now_accx * dt;
	Speed_y += now_accy * dt;
	Speed_z += now_accz * dt;	
}


uint16_t  cytel = 0;
extern void UART2_ReportMotion2(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
extern void Send_Pos(float px,float py,float pz,
			float spx,float spy,float spz);

void Get_acc_Vector(float * q,float dt){
		float Matrix[3][3]; //四元数对应的旋转矩阵
		float ax=0,ay=0,az=1*Acc_Resolution;
		float acc_Vectorx,acc_Vectory,acc_Vectorz;
		float acc_Vectorx1,acc_Vectory1,acc_Vectorz1;
		float aSq = q[0] * q[0];
		float bSq = q[1] * q[1];
		float cSq = q[2] * q[2];
		float dSq = q[3] * q[3];
		Matrix[0][0] = aSq + bSq - cSq - dSq;
		Matrix[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
		Matrix[0][2] = 2.0f * (q[0] * q[2] + q[1] * q[3]);
		Matrix[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
		Matrix[1][1] = aSq - bSq + cSq - dSq;
		Matrix[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
		Matrix[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
		Matrix[2][1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
		Matrix[2][2] = aSq - bSq - cSq + dSq;
		//通过旋转矩阵，计算重力加速度的分步
		acc_Vectorx = Matrix[0][0]*ax + Matrix[1][0]*ay + Matrix[2][0]*az;
		acc_Vectory = Matrix[0][1]*ax + Matrix[1][1]*ay + Matrix[2][1]*az;
		acc_Vectorz = Matrix[0][2]*ax + Matrix[1][2]*ay + Matrix[2][2]*az;
		acc_Vectorx1 = lastAx - acc_Vectorx;
		acc_Vectory1 = lastAy - acc_Vectory;
		acc_Vectorz1 = lastAz - acc_Vectorz;
		pos_accx_mss = (lastAx - acc_Vectorx) * ADC_to_mss;
		pos_accy_mss = (lastAy - acc_Vectory) * ADC_to_mss;
		pos_accz_mss = (lastAz - acc_Vectorz) * ADC_to_mss;
		Estimate_Posi(dt);

		if(cytel++>500){
		//UART1_ReportMotion2(lastAx,lastAy,lastAz,
		//					acc_Vectorx,acc_Vectory,acc_Vectorz,
		//					acc_Vectorx1,acc_Vectory1,acc_Vectorz1
		//					);
		//UART1_ReportMotion2(//pos_x*1000,pos_y*1000,pos_z*1000,
		//					accg*1000,0,gyrodps*100,
		////					Speed_x*1000,Speed_y*1000,Speed_z*1000,
		//					now_accx*1000,now_accy*1000,now_accz*1000
		//					);

	//	Send_Pos(pos_x,pos_y,pos_z,
		//		 Speed_x,Speed_y,Speed_z);
							cytel = 0;
							}
}

//判断模块此时是否是静止的。
void Estimate_Motion(int16_t ax,int16_t ay,int16_t az,
					int16_t gx,int16_t gy,int16_t gz){
	  int16_t i;
	  int16_t acc,gyro;
	  int32_t sum,fax,fay,faz,fgx,fgy,fgz;
	  if(index == -1){	// 第一次初始化
		 for(i=0;i<bufSize;i++)	{
		 eax[i] = ax;
		 eay[i] = ay;
		 eaz[i] = az;
		 egx[i] = gx;
		 egy[i] = gy;
		 egz[i] = gz;
		 cal_Accx[i] = 0;
		 cal_Accy[i] = 0;
		 cal_Accz[i] = 0;
		 }
		 index = 0;	
		 return;
	  	}
	  	 eax[index] = ax;
		 eay[index] = ay;
		 eaz[index] = az;
		 egx[index] = gx;
		 egy[index] = gy;
		 egz[index] = gz;
		 cal_Accx[index] = pos_accx_mss;
		 cal_Accy[index] = pos_accy_mss;
		 cal_Accz[index] = pos_accz_mss;
		 index = (index+1)%bufSize;
		 sum = 0;
		 for(i=0;i<bufSize;i++)
		 	sum += eax[i];
		 fax = (sum / bufSize);

		 sum = 0;
		 for(i=0;i<bufSize;i++)
		 	sum += eay[i];
		 fay = (sum / bufSize);

		 sum = 0;
		 for(i=0;i<bufSize;i++)
		 	sum += eaz[i];
		 faz = (sum / bufSize);

		 sum = 0;
		 for(i=0;i<bufSize;i++)
		 	sum += egx[i];
		 fgx = (sum / bufSize);

		 sum = 0;
		 for(i=0;i<bufSize;i++)
		 	sum += egy[i];
		 fgy = (sum / bufSize);

		 sum = 0;
		 for(i=0;i<bufSize;i++)
		 	sum += egz[i];
		 fgz = (sum / bufSize);

		 acc = sqrt(fax*fax+fay*fay+faz*faz);
		 gyro = sqrt(fgx*fgx+fgy*fgy+fgz*fgz);
		 accg = acc/Acc_Resolution;
		 gyrodps = gyro/Gyro_Resolution;
		 Moving = 1;
		if(accg>(1.0f+filter_acc_err))return; //加速度合超过一个重力还多
		if(accg<(1.0f-filter_acc_err))return;
		if(gyrodps > filter_gyro)return;
		Moving = 0;
		Reset_speed();
}




//------------------End of File----------------------------
