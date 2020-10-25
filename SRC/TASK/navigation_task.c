/**********************************************************************************************************
 * @文件     navigation_task.c
 * @说明     导航相关任务，包括姿态估计、速度估计和位置估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "navigation_task.h"
#include "TaskConfig.h"
#include "mathConfig.h"
#include "boardConfig.h"
#include "flightStatus.h"
#include "gyroscope.h"

#include "anoAHRS.h"
#include "mahonyAHRS.h"
#include "messageQueue.h"
#include "magnetometer.h"

xTaskHandle navigationHandle;
xTaskHandle flightStatusHandle;

/**********************************************************************************************************
*函 数 名: vNavigationTask
*功能说明: 导航计算相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vNavigationTask, pvParameters)
{
	Vector3f_t* gyro;
	Vector3f_t* acc;
	portTickType xLastWakeTime;
	
	vTaskDelay(500);
	
	//挂起调度器
	vTaskSuspendAll();
	
	//姿态估计参数初始化
	MahonyAHRSinit();
	//AnoAHRSinit();
	//唤醒调度器
	xTaskResumeAll();
	
	float *dcmat;
	Vector3f_t eulers;
	
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//从消息队列中获取数据
		xQueueReceive(messageQueue[GYRO_PRETREAT], &gyro, (3/portTICK_RATE_MS));
		xQueueReceive(messageQueue[ACC_PRETREAT], &acc, (3/portTICK_RATE_MS));
		
		//姿态解算更新
		//Vector3f_t vecTmp; vecTmp.x=0.0f; vecTmp.y=0.0f; vecTmp.z=0.0f;
		MahonyAHRSupdate(*gyro, *acc, MagGetData());

		dcmat = GetDCM();
		eulers = GetEuler();
		
		//AnoAHRSupdate(*gyro, *acc, MagGetData());
		
		//阻塞2ms
		vTaskDelayUntil(&xLastWakeTime, (1/portTICK_RATE_MS));
	}
	
}

/**********************************************************************************************************
*函 数 名: vFlightStatusTask
*功能说明: 飞行状态检测相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vFlightStatusTask, pvParameters)
{
	portTickType xLastWakeTime;
	
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		
		//飞行器放置状态检测（静止或移动）
		PlaceStatusCheck(GyroLpfGetData());
		
		//传感器方向检测（用于校准时的判断）
		ImuOrientationDetect();

		//睡眠10ms
		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_RATE_MS));
	}
}

 /**********************************************************************************************************
*函 数 名: NavigationTaskCreate
*功能说明: 导航相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationTaskCreate(void)
{
    xTaskCreate(vNavigationTask, "navigation", NAVIGATION_TASK_STACK, NULL, NAVIGATION_TASK_PRIORITY, &navigationHandle);
    xTaskCreate(vFlightStatusTask, "flightStatus", FLIGHT_STATUS_TASK_STACK, NULL, FLIGHT_STATUS_TASK_PRIORITY, &flightStatusHandle);
}

