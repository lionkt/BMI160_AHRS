#ifndef __POSITION_H
#define __POSITION_H

#include "IMU.h"

extern uint8_t  Moving;
void Get_acc_Vector(float * q,float dt);
void Initial_Pos(void);
void Estimate_Motion(int16_t ax,int16_t ay,int16_t az,
					int16_t gx,int16_t gy,int16_t gz);


#endif

//------------------End of File----------------------------
