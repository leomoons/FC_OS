#ifndef __ICM20602_H
#define __ICM20602_H
#include "stm32f4xx.h"
#include "mathTool.h"

void ICM20602CSPin_Init(void);
void ICM20602_Init(void);

//void ICM20602_SingleWrite(uint8_t reg, uint8_t value);
//void ICM20602_MultiRead(uint8_t reg, uint8_t *data, uint8_t len);

void ICM20602_UpdateAcc(Vector3f_t* acc);
void ICM20602_UpdateGyro(Vector3f_t* gyro);
void ICM20602_UpdateTemp(float* temp);

#endif 
