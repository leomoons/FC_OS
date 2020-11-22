#ifndef __DISTURBANCE_EST_H
#define __DISTURBANCE_EST_H
#include "mathConfig.h"

typedef struct 
{
	Vector3f_t F_b;
	Vector3f_t M_b;
}estimate_set_t;
extern estimate_set_t _est;


void estimatorInit(void);

void estimatorUpdate(void);


#endif
