 /**********************************************************************************************************
 * @文件    ahrsConfig.h
 * @说明     姿态&航向估计配置头文件
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#ifndef __AHRSCONFIG_H
#define __AHRSCONFIG_H
#include "mathConfig.h"

typedef struct
{
	float quat[4];
	
	float _dcm[9];
	
	Vector3f_t euler;
}AHRS_t;

#endif
