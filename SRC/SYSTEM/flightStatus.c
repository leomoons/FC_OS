/**********************************************************************************************************
 * @文件     flightStatus.c
 * @说明     飞行状态分类与检测
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "flightStatus.h"
#include "mathTool.h"

FLIGHT_STATUS_t flyStatus;



/**********************************************************************************************************
*函 数 名: PlaceStausCheck
*功能说明: 飞行器放置状态检测：静止或运动
*形    参: 角速度
*返 回 值: 无
**********************************************************************************************************/
void PlaceStausCheck(Vector3f_t gyro)
{
	Vector3f_t gyroDiff;
	static Vector3f_t lastGyro;
	static float threshold = 1.0f;
	static uint16_t checkNum = 0;
	static int16_t count = 0;
	
	gyroDiff.x = gyro.x - lastGyro.x;
    gyroDiff.y = gyro.y - lastGyro.y;
    gyroDiff.z = gyro.z - lastGyro.z;
    lastGyro = gyro;
	
	//30次采样数据中超过10次数值变化大于阈值就表明飞机不处于静止状态
	if(count < 30)
	{
		count++;
		//陀螺仪数值变化大于阈值
		if(abs(gyroDiff.x) > threshold || abs(gyroDiff.y) > threshold || abs(gyroDiff.z) > threshold)
		{
			checkNum++;
		}
	}
	else
	{
		//陀螺仪数据抖动次数大于一定值时认为飞机不处于静止状态
		if(checkNum > 10)
			flyStatus.placement = MOTIONAL;
		else
			flyStatus.placement = STATIC;
		
		checkNum = 0;
		count = 0;
	}
}

/**********************************************************************************************************
*函 数 名: GetPlaceStatus
*功能说明: 获取飞行器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetPlaceStatus(void)
{
    return flyStatus.placement;
}


/**********************************************************************************************************
*函 数 名: SetCaliStatus
*功能说明: 设置传感器校准的状态
*形    参: 状态
*返 回 值: 无
**********************************************************************************************************/
void SetCaliStatus(uint8_t status)
{
	flyStatus.caliSt = status;
}

/**********************************************************************************************************
*函 数 名: GetCaliStatus
*功能说明: 获取传感器校准状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetCaliStatus(void)
{
    return flyStatus.caliSt;
}
