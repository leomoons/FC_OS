/**********************************************************************************************************
 * @文件     TaskConfig.h
 * @说明     RTOS的任务配置文件，任务调度模式为抢占式，共有15个优先级（最多可设置为32）
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.8
**********************************************************************************************************/
#ifndef __TASKCONFIG_H
#define __TASKCONFIG_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "module_task.h"
#include "sensor_task.h"
#include "message_task.h"
#include "navigation_task.h"

//任务堆栈大小
#define IMU_SENSOR_READ_TASK_STACK			256
#define OTHER_SENSOR_READ_TASK_STACK		256
#define IMU_SENSOR_PRETREAT_STACK			256
#define OTHER_SENSOR_PRETREAT_STACK			256
#define MESSAGE_TASK_STACK					512
#define NAVIGATION_TASK_STACK 				512
#define FLIGHT_STATUS_TASK_STACK			256


//任务优先级
#define IMU_SENSOR_READ_TASK_PRIORITY		13
#define OTHER_SENSOR_READ_TASK_PRIORITY		8
#define IMU_SENSOR_PRETREAT_PRIORITY		12
#define OTHER_SENSOR_PRETREAT_PRIORITY		7
#define MESSAGE_TASK_PRIORITY				6
#define NAVIGATION_TASK_PRIORITY			11
#define FLIGHT_STATUS_TASK_PRIORITY			5


//消息队列
enum
{
	GYRO_SENSOR_READ,
	ACC_SENSOR_READ,
	TEMP_SENSOR_READ,
	
	GYRO_PRETREAT,
	ACC_PRETREAT,
	GYRO_LPF,
	QUEUE_NUM			//消息队列数目
};


#endif
