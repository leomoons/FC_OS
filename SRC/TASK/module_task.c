/**********************************************************************************************************
 * @文件     module_task.c
 * @说明     传感器及外设等相关任务,负责数据更新
 * @版本  	 V1.0
 * @作者     Leomoon	
 * @日期     2020.8
**********************************************************************************************************/
#include "module_task.h"
#include "TaskConfig.h"
#include "mathConfig.h"
#include "boardConfig.h"
#include "module.h"
#include "messageQueue.h"
#include "parameter.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h" 


//声明任务句柄
xTaskHandle imuSensorReadHandle;
xTaskHandle otherSensorReadHandle;


/**********************************************************************************************************
*函 数 名: vImuSensorReadTask
*功能说明: IMU传感器数据读取任务，此任务具有最高优先级，运行频率为1KHz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
#ifdef _DEBUG_
Vector3f_t accM, gyroM;
float tempM;
#endif
portTASK_FUNCTION(vImuSensorReadTask, pvParameters)
{
	portTickType xLastWakeTime;
	
	Vector3f_t *accRaw = pvPortMalloc(sizeof(Vector3f_t));
	Vector3f_t *gyroRaw = pvPortMalloc(sizeof(Vector3f_t));
	float *tempRaw = pvPortMalloc(sizeof(float));
	
	//挂起调度器
	vTaskSuspendAll();
	
	//IMU传感器初始化，由于调度器被挂起，OS系统时钟终止，无法在这里使用DelayMs(因为包含vTaskDelay)
	IMUSensorInit();
	
	//唤醒调度器
	xTaskResumeAll();
	
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		//更新传感器原始数据
		AccDataUpdate();
		GyroDataUpdate();
		IMUtempUpdate();
		//读取传感器数据
		AccDataRead(accRaw);
		GyroDataRead(gyroRaw);
		IMUtempRead(tempRaw);
#ifdef _DEBUG_
		accM = *accRaw;
		gyroM = *gyroRaw;
		tempM = *tempRaw;
#endif
		
		
		//更新消息队列，通知数据预处理任务对IMU数据进行预处理
		xQueueSendToBack(messageQueue[ACC_SENSOR_READ], (void *)&accRaw, 0);
		xQueueSendToBack(messageQueue[GYRO_SENSOR_READ], (void *)&gyroRaw, 0);
		xQueueSendToBack(messageQueue[TEMP_SENSOR_READ], (void *)&tempRaw, 0);

		
		//阻塞1ms，对于当前系统就是一个时间片
		vTaskDelayUntil(&xLastWakeTime, (1/portTICK_RATE_MS));
	}
}

/**********************************************************************************************************
*函 数 名: vSensorUpdateTask
*功能说明: IMU之外的传感器数据更新任务(200Hz)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
#ifdef _DEBUG_
Vector3f_t magM;
#endif
portTASK_FUNCTION(vOtherSensorReadTask, pvParameters)
{
	portTickType xLastWakeTime;
	uint32_t cnt = 0;	//static型不能全局同变量名
	
	Vector3f_t *magRaw = pvPortMalloc(sizeof(Vector3f_t));
	float *baroPres = pvPortMalloc(sizeof(float));
	float *baroTemp = pvPortMalloc(sizeof(float));
	
	//挂起调度器
	vTaskSuspendAll();
	
	//地磁传感器初始化
	MagSensorInit();
	//气压传感器初始化
	BaroSensorInit();
	
	//唤醒调度器
	xTaskResumeAll();
	
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		cnt++;
		cnt %= 20;
		//地磁传感器数据更新(50Hz)
		if(cnt%2 == 0)
		{
			vTaskSuspendAll();
			MagDataUpdate();
			MagDataRead(magRaw);
			xTaskResumeAll();
#ifdef _DEBUG_
			magM = *magRaw;
#endif
			xQueueSendToBack(messageQueue[MAG_SENSOR_READ], (void *)&magRaw, 0);
		}
			
		//气压传感器数据更新(50Hz)
		if(cnt%2 == 0)
		{
			//读取气压计数据时挂起调度器，防止SPI总线冲突
			vTaskSuspendAll();
			BaroDataUpdate();
			xTaskResumeAll();
			BaroPresRead(baroPres);
			BaroTempRead(baroTemp);
			xQueueSendToBack(messageQueue[BARO_PRES_READ], (void *)&baroPres, 0);
			xQueueSendToBack(messageQueue[BARO_TEMP_READ], (void *)&baroTemp, 0);
		}
		
		//飞控参数保存(参数有更新才会写入)20Hz
		if(cnt%5 == 0)
		{
			ParamSaveToFlash();
		}
		
		//RGB闪烁
		RGB_Flash();
		
		//睡眠10ms
		vTaskDelayUntil(&xLastWakeTime, (10/portTICK_RATE_MS));
	}
}

/**********************************************************************************************************
*函 数 名: ModuleTaskCreate
*功能说明: 传感器组件相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ModuleTaskCreate(void)
{
	xTaskCreate(vImuSensorReadTask, "imuSensorRead", IMU_SENSOR_READ_TASK_STACK, NULL,  IMU_SENSOR_READ_TASK_PRIORITY, &imuSensorReadHandle);
	xTaskCreate(vOtherSensorReadTask, "sensorUpdate", OTHER_SENSOR_READ_TASK_STACK, NULL, OTHER_SENSOR_READ_TASK_PRIORITY, &otherSensorReadHandle);
}

