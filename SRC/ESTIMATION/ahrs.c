 /**********************************************************************************************************
 * @文件     ahrs.c
 * @说明     姿态&航向估计总接口文件，选择调用不同的姿态解算算法
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "ahrs.h"
#include "anoAHRS.h"
#include "mahonyAHRS.h"

enum 
{
	ANO,
	MAHONY
};

#define AHRSalgorithm ANO 



/**********************************************************************************************************
*函 数 名: AHRSinit
*功能说明: 姿态解算初始化函数
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void AHRSinit(void)
{
	if(AHRSalgorithm == ANO)
	{
		AnoAHRSinit();
	}
	else if(AHRSalgorithm == MAHONY)
	{
		MahonyAHRSinit();
	}
}

/**********************************************************************************************************
*函 数 名: AHRSupdate
*功能说明: 姿态解算主函数
*形    参: 角速度向量指针 加速度向量指针 磁场向量指针
*返 回 值: 无
**********************************************************************************************************/
void AHRSupdate(Vector3f_t *gyro, Vector3f_t *acc, Vector3f_t *mag)
{
	if(AHRSalgorithm == ANO)
	{
		AnoAHRSupdate(*gyro, *acc, *mag);
	}
	else if(AHRSalgorithm == MAHONY)
	{
		MahonyAHRSupdate(*gyro, *acc, *mag);
	}
}

/**********************************************************************************************************
*函 数 名: GetDCM
*功能说明: 获取方向余弦矩阵
*形    参: 旋转矩阵数组头
*返 回 值: 无
**********************************************************************************************************/
void GetDCM(float* dcm)
{
	if(AHRSalgorithm == ANO)
	{
		AnoGetDCM(dcm);
	}
	else if(AHRSalgorithm == MAHONY)
	{
		MahonyGetDCM(dcm);
	}
}

/**********************************************************************************************************
*函 数 名: GetEuler
*功能说明: 获取欧拉角
*形    参: 无
*返 回 值: ZYX旋转顺序欧拉角(rad)
**********************************************************************************************************/
void GetEuler(Vector3f_t *euler)
{
	if(AHRSalgorithm == ANO)
	{
		AnoGetEuler(euler);
	}
	else if(AHRSalgorithm == MAHONY)
	{
		MahonyGetEuler(euler);
	}
}
