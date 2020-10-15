#ifndef __MAGNETOMETER_H
#define	__MAGNETOMETER_H
#include "vector3.h"
#include "sensor.h"

typedef struct {
    Vector3f_t data;
    float mag;
    SENSOR_CALI_t cali;
    float earthMag;

} MAGNETOMETER_t;

void MagPreTreatInit(void);
void MagDataPreTreat(void);
void MagCalibration(void);
Vector3f_t MagGetData(void);
void MagCalibrateEnable(void);

Vector3f_t GetMagOffsetCaliData(void);
Vector3f_t GetMagScaleCaliData(void);

#endif
