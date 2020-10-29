#ifndef __OPTITRACK_H
#define __OPTITRACK_H
#include "stdint.h"
#include "mathConfig.h"

typedef struct 
{
	uint8_t online;			//超过一秒没更新位姿信息就将online置为0
	
	Vector3f_t pos;
	Vector3f_t vel;
	Vector3f_t acc;
	
	float quat[4];		//使用四元数传递
	Vector3f_t euler;	//欧拉角表示
	float dcm[9];			//方向余弦矩阵
	Vector3f_t W;			//角速度
	Vector3f_t W_dot;		//角加速度
}Optitrack_t;

void OptitrackInit(void);
void Opti_Get_Byte(uint8_t byte);
void Opti_Get_Data_Task(void);

Vector3f_t GetOptiPos(void);
Vector3f_t GetOptiVel(void);
Vector3f_t GetOptiAcc(void);

void GetOptiAttQuat(float* quat);
void GetOptiAttDCM(float* dcm);
Vector3f_t GetOptiAttEuler(void);
Vector3f_t GetOptiAngVel(void);
Vector3f_t GetOptiAngAcc(void);

#endif
