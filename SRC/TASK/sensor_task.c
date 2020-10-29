/**********************************************************************************************************
 * @文件     sensor_task.c
 * @说明     传感器校准及数据预处理相关任务
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "sensor_task.h"
#include "TaskConfig.h"
#include "mathConfig.h"
#include "messageQueue.h"

#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "barometer.h"

xTaskHandle imuSensorPretreatHandle;
xTaskHandle otherSensorPretreatHandle;


/**********************************************************************************************************
*函 数 名: vImuDataPreTreatTask
*功能说明: IMU传感器数据预处理任务，任务优先级仅次于IMU传感器读取
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
Vector3f_t ACCPRE, GYROPRE, GYROLPF;

portTASK_FUNCTION(vImuPretreatTask, pvParameters)
{
	portTickType xLastWakeTime;
	
	Vector3f_t* gyroRaw;
	Vector3f_t* accRaw;
	float* tempRaw;
	Vector3f_t* accPre = pvPortMalloc(sizeof(Vector3f_t));
	Vector3f_t* gyroPre = pvPortMalloc(sizeof(Vector3f_t));
	Vector3f_t* gyroLpf = pvPortMalloc(sizeof(Vector3f_t));
	
	//挂起调度器
	vTaskSuspendAll();
	
	//陀螺仪预处理初始化
	GyroPreTreatInit();
	//加速度计预处理初始化
	AccPreTreatInit();
	//TODO: IMU传感器恒温参数初始化
	
	
	//唤醒调度器
	xTaskResumeAll();
	
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		//从消息队列中获取数据
		xQueueReceive(messageQueue[GYRO_SENSOR_READ], &gyroRaw, (3/portTICK_RATE_MS));
		xQueueReceive(messageQueue[ACC_SENSOR_READ], &accRaw, (3/portTICK_RATE_MS));
		xQueueReceive(messageQueue[TEMP_SENSOR_READ], &tempRaw, (3/portTICK_RATE_MS));
		
		
		//陀螺仪校准
		GyroCalibration(*gyroRaw);
		//加速度校准
		AccCalibration(*accRaw);
		//IMU安装误差校准
		ImuLevelCalibration();
		
		//加速度数据预处理
		AccDataPreTreat(*accRaw, accPre);
		//陀螺仪数据预处理
		GyroDataPreTreat(*gyroRaw, *tempRaw, gyroPre, gyroLpf);
		
		ACCPRE = *accPre;
		GYROPRE = *gyroPre;
		GYROLPF = *gyroLpf;
		
		//往下一级消息队列中填充数据
		xQueueSendToBack(messageQueue[ACC_PRETREAT], (void*)&accPre, 0);
		xQueueSendToBack(messageQueue[GYRO_PRETREAT], (void*)&gyroPre, 0);
		xQueueSendToBack(messageQueue[GYRO_LPF], (void*)&gyroLpf, 0);
		
		//阻塞1ms
		vTaskDelayUntil(&xLastWakeTime, (1/portTICK_RATE_MS));
		
	}
	
}

/**********************************************************************************************************
*函 数 名: vOtherSensorTask
*功能说明: 其它传感器数据预处理任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vOtherPretreatTask, pvParameters)
{
	portTickType xLastWakeTime;
	static uint16_t cnt = 0;
	
	//挂起调度器
	vTaskSuspendAll();
	
	//磁力计校准参数初始化
	MagPreTreatInit();
	
	//唤醒调度器
	xTaskResumeAll();
	
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		//100Hz
		if(cnt%2 == 0)
		{
			//磁力计校准
			MagCalibration();
			
			//磁力计数据预处理
			MagDataPreTreat();
		}
		
		//25Hz
		if(cnt%8 == 0)
		{
			//气压高度数据预处理
			BaroDataPreTreat();
		}
		
		//睡眠5ms
		vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));
	}
}


/**********************************************************************************************************
*函 数 名: SensorTaskCreate
*功能说明: 传感器数据预处理相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SensorTaskCreate(void)
{
    xTaskCreate(vImuPretreatTask, "imuDataPreTreat", IMU_SENSOR_PRETREAT_STACK, NULL, IMU_SENSOR_PRETREAT_PRIORITY, &imuSensorPretreatHandle);
    xTaskCreate(vOtherPretreatTask, "otherSensor", OTHER_SENSOR_PRETREAT_STACK, NULL, OTHER_SENSOR_PRETREAT_PRIORITY, &otherSensorPretreatHandle);
}
