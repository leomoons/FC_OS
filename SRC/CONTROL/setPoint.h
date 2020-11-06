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

void GetDesiredPos(Vector3f_t *pos);
void GetDesiredVel(Vector3f_t *vel);
void GetDesiredAcc(Vector3f_t *acc);

void GetDesiredAtt(float *dcm);
void GetDesiredAngVel(Vector3f_t *ang_vel);
void GetDesiredAngAcc(Vector3f_t *ang_acc);

#endif
