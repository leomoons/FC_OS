#ifndef __MODULE_H
#define __MODULE_H
#include "mathConfig.h"


//IMU中的数据通过队列传递，其他传感器定义好读取函数接口 (*DataRead函数)

void IMUSensorInit(void);
void GyroDataUpdate(Vector3f_t *gyro);
void AccDataUpdate(Vector3f_t *acc);
void IMUTempUpdate(float* temp);

void MagSensorInit(void);
void MagDataUpdate(void);
void MagDataRead(Vector3f_t* mag);

void BaroSensorInit(void);
void BaroDataUpdate(void);
void BaroPresRead(float *pres);
void BaroTempRead(float *temp);

void TempCtrlSetTemp(int16_t temp);

#endif
