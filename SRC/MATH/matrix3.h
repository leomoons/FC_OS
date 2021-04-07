#ifndef __MATRIX3_H
#define __MATRIX3_H
#include "stm32f4xx.h"

void Scalar_Matrix3(float s, float* a);
void Matrix3_Add(float* a,float* b,float* c);
void Matrix3_Sub(float* a,float* b,float* c);
void Matrix3_Mul(float* a,float* b,float* c);
void Matrix3_Copy(float* a, float* b);
void Matrix3_Tran(float* a, float* b);
void Matrix3_Inv(float* a,float* b);
void Matrix3_Eye(float* a);
float Matrix3_Trace(float* a);
uint8_t isDCM(float* dcm);


#endif
