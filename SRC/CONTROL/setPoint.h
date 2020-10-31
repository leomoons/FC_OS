#ifndef __SETPOINT_H
#define __SETPOINT_H
#include "mathConfig.h"

typedef struct
{
	Vector3f_t pos;
	Vector3f_t vel;
	Vector3f_t acc;
	
	float att[9];
	Vector3f_t ang_vel;
	Vector3f_t ang_acc;
}Setpoint_t;

void SetPointUpdate(void);

Vector3f_t GetDesiredPos(void);
Vector3f_t GetDesiredVel(void);
Vector3f_t GetDesiredAcc(void);

void GetDesiredAtt(float *dcm);
Vector3f_t GetDesiredAngVel(void);
Vector3f_t GetDesiredAngAcc(void);

#endif
