#ifndef __AK8975_H
#define __AK8975_H
#include "stm32f4xx.h"
#include "vector3.h"

void AK8975CSPin_Init(void);

void AK8975_Update(void);

void AK8975_ReadMag(Vector3f_t* mag);

#endif
