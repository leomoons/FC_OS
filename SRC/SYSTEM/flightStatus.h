#ifndef __FLIGHTSTATUS_H
#define __FLIGHTSTATUS_H
#include "stm32f4xx.h"
#include "vector3.h"


//放置状态
enum
{
	STATIC,					//静止
	MOTIONAL				//运动
};

//传感器校准状态
enum
{
	NoCali,					//不校准状态
	ImuLevelCali,			//IMU水平补偿
	AccCaliDataCollecting,	//加速度计采集数据中
	AccCaliOneDataReady,	//加速度计一个方向的数据采集完毕，提示切换到下一个方向
	GyroCali,				//陀螺仪校准
	MagCaliHorizontal,		//磁罗盘校准中的水平旋转
	MagCaliVertical,		//磁罗盘校准中的竖直旋转
};

//飞行模式
enum
{
	MANUAL = 0,			//手动
	MISSION,			//按设定轨迹飞行
	FAILSAFE			//紧急停止
};

typedef struct
{
	uint8_t placement;		//放置状态
	uint8_t caliSt;			//传感器所处校准状态
	uint8_t mode;			//飞行模式
}FLIGHT_STATUS_t;

extern FLIGHT_STATUS_t flyStatus;

void PlaceStatusCheck(Vector3f_t gyro);
uint8_t GetPlaceStatus(void);

void SetCaliStatus(uint8_t status);
uint8_t GetCaliStatus(void);

void SetFlightMode(uint8_t mode);
uint8_t GetFlightMode(void);

#endif
