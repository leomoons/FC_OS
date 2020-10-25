#ifndef __MAHONYAHRS_H
#define __MAHONYAHRS_H
#include "mathConfig.h"

void MahonyAHRSinit(void);
void MahonyAHRSupdate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag);
void MahonyAHRSupdateIMU(Vector3f_t gyro, Vector3f_t acc);

float* GetDCM(void);
Vector3f_t GetEuler(void);

#endif
