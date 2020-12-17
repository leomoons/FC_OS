#ifndef __GEOCTRL_H
#define __GEOCTRL_H

#include "mathConfig.h" 
#include "controller.h"

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
	
	//误差向量
	Vector3f_t pos_err;
	Vector3f_t vel_err;
	Vector3f_t R_err;
	Vector3f_t W_err;
	
	// Gain coefficient
	float Kp[9];
	float Kv[9];
	float KR[9];
	float KW[9];
	
	control_set_t ctrl;
}GeoControl_t;
extern GeoControl_t _geo;

void GeoControllerInit(void);
void GeoCtrlUpdateParam(uint16_t param, float data);
void GeoCtrlUpdate(void);


#endif
