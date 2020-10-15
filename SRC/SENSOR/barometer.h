#ifndef __BAROMETER_H
#define	__BAROMETER_H
#include "stm32f4xx.h"

typedef struct{
	float alt;
	float lastAlt;
	float pressure;
	float velocity;
	float alt_offset;
	float temperature;
}BAROMETER_t;

void BaroDataPreTreat(void);
float BaroGetAlt(void);
float BaroGetTemp(void);
float BaroGetVelocity(void);

#endif

