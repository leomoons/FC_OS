/**********************************************************************************************************
 * @文件     sensor.c
 * @说明     传感器数据预处理，传感器恒温控制，判断飞行器摆放姿态（六个之一）
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "sensor.h"
#include "math.h"

#include "accelerometer.h"
#include "gyroscope.h"


enum ORIENTATION_STATUS orientationStatus;

/**********************************************************************************************************
*函 数 名: ImuOrientationDetect
*功能说明: 检测传感器放置方向
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ImuOrientationDetect(void)
{
	const float CONSTANTS_ONE_G = 1.0;
	const float accel_err_thr = 0.5;
	
	Vector3f_t acc;
	
	//读取加速度数据
	acc = AccGetData();
	
	// [ g, 0, 0 ]
	if(fabsf(acc.x-CONSTANTS_ONE_G) < accel_err_thr &&
			fabsf(acc.y) < accel_err_thr &&
			fabsf(acc.z) < accel_err_thr)
	{
		orientationStatus = ORIENTATION_FRONT;
	}
	// [ -g, 0, 0 ]
    if (fabsf(acc.x + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_BACK;
    }
    // [ 0, g, 0 ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y - CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_LEFT;
    }
    // [ 0, -g, 0 ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_RIGHT;
    }
    // [ 0, 0, g ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z - CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_UP;
    }
    // [ 0, 0, -g ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z + CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_DOWN;
    }
}

/**********************************************************************************************************
*函 数 名: GetImuOrientation
*功能说明: 获取传感器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
enum ORIENTATION_STATUS GetImuOrientation(void)
{
    return orientationStatus;
}
