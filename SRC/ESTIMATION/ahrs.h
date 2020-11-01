#ifndef __AHRS_H
#define __AHRS_H
#include "mathConfig.h"

typedef struct
{
	float quat[4];
	float dcm[9];
	Vector3f_t euler;
}AHRS_t;

void AHRSinit(void);
void AHRSupdate(Vector3f_t *gyro, Vector3f_t *acc, Vector3f_t *mag);

void GetDCM(float *dcm);
void GetEuler(Vector3f_t *euler);

#endif
