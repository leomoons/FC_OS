#ifndef __VECTOR3_H
#define __VECTOR3_H
#include "stm32f4xx.h"


typedef struct {
	float x;
	float y;
	float z;
}Vector3f_t;

typedef struct {
	double x;
	double y;
	double z;
}Vector3d_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}Vector3i_t;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} Vector3l_t;


void Vector3f_Normalize(Vector3f_t* vector);

Vector3f_t Vector3iTo3f(Vector3i_t vector);
Vector3i_t Vector3fTo3i(Vector3f_t vector);
Vector3f_t Vector3f_Add(Vector3f_t v1, Vector3f_t v2);
Vector3f_t Vector3f_Sub(Vector3f_t v1, Vector3f_t v2);

void Hat_Map(float* mat, Vector3f_t vec);

Vector3f_t VectorCrossProduct(Vector3f_t v1, Vector3f_t v2);
Vector3f_t Matrix3MulVector3(float* m, Vector3f_t vector);
Vector3f_t VectorRotateToBodyFrame(Vector3f_t vector, Vector3f_t deltaAngle);
Vector3f_t VectorRotateToEarthFrame(Vector3f_t vector, Vector3f_t deltaAngle);

void AccVectorToRollPitchAngle(Vector3f_t* angle, Vector3f_t vector);
void MagVectorToYawAngle(Vector3f_t* angle, Vector3f_t vector);

#endif
