#ifndef __EEPROM_H
#define __EEPROM_H

#include "stm32f4xx.h"

struct data_map{
uint8_t demmy;
int16_t is_good;   //数据是否有效
uint16_t dGx_offset;
uint16_t dGy_offset;
uint16_t dGz_offset;

float	 dMx_scale;
float	 dMy_scale;
float	 dMz_scale;

int16_t dMx_offset;
int16_t dMy_offset;
int16_t dMz_offset;
uint16_t File_index;

int16_t dAx_offset;
int16_t dAy_offset;
int16_t dAz_offset;
float  dAx_scale;
float  dAy_scale;
float  dAz_scale;

};

extern struct data_map Config;

void Write_config(void);
void load_config(void);
void Get_file_name(uint8_t *name);

#endif /* __EEPROM_H */

//------------------End of File----------------------------
