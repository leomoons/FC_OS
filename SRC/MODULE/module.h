#ifndef __MODULE_H
#define __MODULE_H
#include "mathConfig.h"


void IMUSensorInit(void);
void GyroDataUpdate(void);
void AccDataUpdate(void);
void IMUtempUpdate(void);
void GyroDataRead(Vector3f_t *gyro);
void AccDataRead(Vector3f_t *acc);
void IMUtempRead(float* temp);

void MagSensorInit(void);
void MagDataUpdate(void);
void MagDataRead(Vector3f_t* mag);

void BaroSensorInit(void);
void BaroDataUpdate(void);
void BaroPresRead(float *pres);
void BaroTempRead(float *temp);

void TempCtrlSetTemp(int16_t temp);

#endif
