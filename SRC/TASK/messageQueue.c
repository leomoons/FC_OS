/**********************************************************************************************************
 * @文件     messageQueue.c
 * @说明     消息队列，主要用于对数据传递实时性要求较高的任务间通信
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.12
**********************************************************************************************************/
#include "messageQueue.h"
#include "mathConfig.h"



//定义消息队列句柄
QueueHandle_t messageQueue[QUEUE_NUM];

/**********************************************************************************************************
*函 数 名: MessageQueueCreate
*功能说明: 消息队列创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageQueueCreate(void)
{
	//IMU原始数据队列
	messageQueue[GYRO_SENSOR_READ] = xQueueCreate(2, sizeof(Vector3f_t *));
	messageQueue[ACC_SENSOR_READ] = xQueueCreate(2, sizeof(Vector3f_t *));
	messageQueue[TEMP_SENSOR_READ] = xQueueCreate(2, sizeof(float *));
	
	//磁罗盘原始数据队列
	messageQueue[MAG_SENSOR_READ] = xQueueCreate(2, sizeof(Vector3f_t *));
	
	//气压计原始数据队列
	messageQueue[BARO_PRES_READ] = xQueueCreate(2, sizeof(float *));
	messageQueue[BARO_TEMP_READ] = xQueueCreate(2, sizeof(float *));
	
	
	messageQueue[GYRO_PRETREAT] = xQueueCreate(2, sizeof(Vector3f_t *));
	messageQueue[ACC_PRETREAT] = xQueueCreate(2, sizeof(Vector3f_t *));
	messageQueue[GYRO_LPF] = xQueueCreate(1, sizeof(Vector3f_t *));
	messageQueue[MAG_PRETREAT] = xQueueCreate(1, sizeof(Vector3f_t *));
}


