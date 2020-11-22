#ifndef __CONTROLLER_H
#define __CONTROLLER_H
#include "mathConfig.h"

typedef struct
{
	float J[9];				//inertia matrix
	float J_inv[9];		//inverse of inertia matrix
	
	float mass;
	
	float Binv[6][6];			//inverse of control allocation matrix

	float T;				//thrust coefficient
//	float TT;				//反扭矩和推力的比值
}vehicle_para;
extern vehicle_para _veh;

typedef struct 
{
	Vector3f_t F_b;
	Vector3f_t M_b;
	float wrench[6];
}control_set_t;
extern control_set_t _ctrl;

void ControllerInit(void);
void CtrlTask(void);


#endif
