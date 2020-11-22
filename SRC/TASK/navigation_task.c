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

#include "ahrs.h"
#include "messageQueue.h"
#include "magnetometer.h"

xTaskHandle ahrsHandle;
xTaskHandle flightStatusHandle;

/**********************************************************************************************************
*函 数 名: vAhrsTask
*功能说明: 导航计算相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
#ifdef _DEBUG_
Vector3f_t gyroN, accN;
#endif
Vector3f_t magN;
float dcmat[9];
Vector3f_t eulers;
portTASK_FUNCTION(vAhrsTask, pvParameters)
{
	portTickType xLastWakeTime;
	uint32_t cnt = 0;
		
	//消息队列传递的数据	
	Vector3f_t *gyro;
	Vector3f_t *acc;
	Vector3f_t *mag;
	
	//挂起调度器
	vTaskSuspendAll();
	
	//姿态估计参数初始化
	AHRSinit();

	//唤醒调度器
	xTaskResumeAll();
	
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		cnt++;
		cnt %= 20;
		//从消息队列中获取数据
		xQueuePeek(messageQueue[GYRO_LPF], &gyro, (3/portTICK_RATE_MS));
		xQueueReceive(messageQueue[ACC_PRETREAT], &acc, (3/portTICK_RATE_MS));
#ifdef _DEBUG_
		accN = *acc;
		gyroN = *gyro;
#endif
		if(cnt%10 == 0)
		{
			xQueueReceive(messageQueue[MAG_PRETREAT], &mag, 0);
			magN = *mag;
		}
		
		//姿态解算更新
		//Vector3f_t vecTmp; vecTmp.x=0.0f; vecTmp.y=0.0f; vecTmp.z=0.0f;
		AHRSupdate(gyro, acc, &magN);		//使用magN防止内存错误访问

		GetDCM(dcmat);
		GetEuler(&eulers);
		
		//阻塞1ms
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
    xTaskCreate(vAhrsTask, "ahrs", AHRS_TASK_STACK, NULL, AHRS_TASK_PRIORITY, &ahrsHandle);
    xTaskCreate(vFlightStatusTask, "flightStatus", FLIGHT_STATUS_TASK_STACK, NULL, FLIGHT_STATUS_TASK_PRIORITY, &flightStatusHandle);
}

