#ifndef __TRANSITION_H
#define __TRANSITION_H
#include "mathConfig.h" 

void Quater_to_DCM(float* dcm, float* quat);
void DCM_to_Quater(float* quat, float* dcm);
void Euler_to_DCM(float* dcm, Vector3f_t euler);
void Euler_to_DCM_T(float* dcm, Vector3f_t euler);
Vector3f_t DCM_to_Euler(float* dcm);
void Euler_to_Quater(float* quat, Vector3f_t euler);
Vector3f_t Quater_to_Euler(float* quat);


#endif
