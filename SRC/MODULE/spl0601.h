#ifndef __SPL0601_H
#define __SPL0601_H
#include "stm32f4xx.h"

typedef struct{
	int16_t c0;
	int16_t c1;
	int32_t c00;
	int32_t c10;
	int16_t c01;
	int16_t c11;
	int16_t c20;
	int16_t c21;
	int16_t c30;
}_spl0601_calib_param_t;

typedef struct{
	_spl0601_calib_param_t calib_param;	//calibration data
	u8 chip_id;
	int32_t i32rawPressure;
	int32_t i32rawTemperature;
	int32_t i32kP;
	int32_t i32kT;
}_spl0601_t;


void SPL0601CSPin_Init(void);
u8 SPL0601_Init(void);

void SPL0601_Update(void);
void SPL0601_ReadPres(float *pres);
void SPL0601_ReadTemp(float *temp);

#endif
