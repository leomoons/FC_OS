/**********************************************************************************************************
 * @文件     control_task.c
 * @说明     飞行控制相关任务
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "control_task.h"
#include "TaskConfig.h"
#include "mathConfig.h"
#include "messageQueue.h"
#include "controller.h"
#include "motor.h"
#include "setPoint.h"

xTaskHandle flightControlHandle;

/**********************************************************************************************************
*函 数 名: vFlightControlTask
*功能说明: 飞行控制相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vFlightControlTask, pvParameters)
{
	portTickType xLastWakeTime;

	ControllerInit();
	MotorInit();
	
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		//主控算法
		CtrlTask();
		//电机控制
		MotorCtrlTask();
		//轨迹生成
		SetPointUpdate();
		
		//阻塞1ms
		vTaskDelayUntil(&xLastWakeTime, (2/portTICK_RATE_MS));
	}
}

/**********************************************************************************************************
*函 数 名: ControlTaskCreate
*功能说明: 控制相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ControlTaskCreate(void)
{
    xTaskCreate(vFlightControlTask, "flightControl", FLIGHTCONTROL_TASK_STACK, NULL, FLIGHTCONTROL_TASK_PRIORITY, &flightControlHandle);
}


