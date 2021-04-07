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
extern control_set_t _ctrl_only;
extern control_set_t _ctrl;

typedef struct
{
	Vector3f_t pos_des;
	Vector3f_t pos_fb;
	Vector3f_t vel_des;
	Vector3f_t vel_fb;
	Vector3f_t acc_des;
	
	float R_des[9];
	float R_fb[9];
	Vector3f_t W_des;
	Vector3f_t W_fb;
	Vector3f_t W_dot_des;
	
	// 误差向量
	Vector3f_t pos_err;
	Vector3f_t vel_err;
	Vector3f_t R_err;
	Vector3f_t W_err;
}droneState_t;
extern droneState_t _state;

void ControllerInit(void);
void CtrlTask(void);


#endif
