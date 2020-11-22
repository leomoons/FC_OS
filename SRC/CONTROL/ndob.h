#ifndef __NDOB_H
#define __NDOB_H
#include "mathConfig.h"
#include "disturbanceEst.h"

typedef struct
{
	// Coefficient
	float Kox[9];
	float KoR[9];
	
	estimate_set_t _est;
}nDOB_t;
extern nDOB_t _dob;

void ndobInit(void);

void ndobUpdate(void);

#endif
