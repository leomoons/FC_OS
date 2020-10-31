#ifndef __GEOCTRL_H
#define __GEOCTRL_H

#include "mathConfig.h" 


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
	
	// Gain coefficient
	float Kp[9];
	float Kv[9];
	float KR[9];
	float KW[9];
	
	float CtrlSet[6];
}GeoControl_t;
extern GeoControl_t _geo;

void GeoParamInit(void);
void GeoCtrlTask(Vector3f_t W_fb);


#endif
