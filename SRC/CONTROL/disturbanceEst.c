/**********************************************************************************************************
 * @文件     disturbanceEst.c
 * @说明     扰动估计的总和
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "disturbanceEst.h"
#include "ndob.h"
#include "adaptive.h"

estimate_set_t _est;

enum
{
	NO_ESTIMATOR,
	NDOB,
	ADAPTIVE
};

#define ESTIMATOR NO_ESTIMATOR


/**********************************************************************************************************
*函 数 名: estimatorInit
*功能说明: 扰动观测模块初始化初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void estimatorInit(void)
{
	if(ESTIMATOR == NO_ESTIMATOR)
	{
		;
	}
	else if(ESTIMATOR == NDOB)
	{
		ndobInit();
	}
	else if(ESTIMATOR == ADAPTIVE)
	{
		adaptiveInit();
	}
	
}


/**********************************************************************************************************
*函 数 名: estimatorUpdate
*功能说明: 扰动观测模块主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void estimatorUpdate(void)
{
	if(ESTIMATOR == NO_ESTIMATOR)
	{
		_est.F_b.x = _est.F_b.y = _est.F_b.z = 0.0f;
		_est.M_b.x = _est.M_b.y = _est.M_b.z = 0.0f;
	}
	else if(ESTIMATOR == NDOB)
	{
		ndobUpdate();
		_est = _dob._est;
	}
	else if(ESTIMATOR == ADAPTIVE)
	{
		adaptiveUpdate();
		_est = _dob._est;
	}
	
}



