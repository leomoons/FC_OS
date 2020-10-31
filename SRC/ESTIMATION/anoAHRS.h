#ifndef __ANOAHRS_H
#define __AHOAHRS_H

#include "mathConfig.h"

typedef struct{
	float w;	//q0
	float x; 	//q1
	float y;	//q2
	float z;	//q3
	
	Vector3f_t hori_vec;
	
	Vector3f_t acc_b;
	Vector3f_t acc_w;
	Vector3f_t acc_h;		//水平面上的加速度
	
	Vector3f_t mag_w;		
	
	Vector3f_t obs_acc_w;
	Vector3f_t obs_acc_b;
	Vector3f_t gra_acc;
	
	Vector3f_t eulerAngle;
	float dcMat[9];
	
	
}_imu_data_st;

typedef struct{
	float akp;
	float aki;
	
	float mkp;
	float frag_p;
	
	u8 G_reset;
	u8 M_reset;
	u8 G_fix_en;
	u8 M_fix_en;
	
	u8 obs_en;			//是否使能机体坐标系下的状态量观测（observe）
}_imu_state_st;

void AnoAHRSinit(void);
void AnoAHRSupdate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag);

float* AnoGetDCM(void);


#endif