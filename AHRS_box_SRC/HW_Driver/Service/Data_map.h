#ifndef __DATAMAP_H
#define __DATAMAP_H

#define  miniAHRS_MPU6050  0xA1
#define  AHRS_PRO_MPU6500  0xA2
#define  I9DOF_ADI_V1      0xA3
#define  I9DOF_ADI_V2      0xA4

#define SW_version   21  // 软件版本号V2.1

#define HW_version  I9DOF_ADI_V2  //硬件板子，


struct data_map{
int16_t is_good;   //数据是否有效
uint16_t dGx_offset;
uint16_t dGy_offset;
uint16_t dGz_offset;


int16_t dMx_offset;
int16_t dMy_offset;
int16_t dMz_offset;
};

extern struct data_map Config;


#endif

//------------------End of File----------------------------
