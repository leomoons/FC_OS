#ifndef __ADAPTIVE_H
#define __ADAPTIVE_H
#include "mathConfig.h"
#include "disturbanceEst.h"

typedef struct
{
	// coefficient
	float gamax[9];
	float gamaR[9];
	float Wx[9];
	float Wx_T[9];
	float WR[9];
	float WR_T[9];
	float cax[9];
	float caR[9];
	float Bx;
	float BR;
	
	estimate_set_t _est;
}adaptive_t;
extern adaptive_t _ada;

void adaptiveInit(void);

void adaptiveUpdate(void);

#endif
