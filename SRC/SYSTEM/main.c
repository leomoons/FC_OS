/**********************************************************************************************************
 * @编译：   Keil ARM MDK 5.25

 * 1缩进等于4空格!
 * 文件编码格式为UTF-8

 * @文件     main.c
 * @说明     程序入口文件
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.08
**********************************************************************************************************/
#include "FreeRTOS.h"
#include "task.h"

#include "boardConfig.h"
#include "parameter.h"
#include "messageQueue.h"


xTaskHandle startHandle;


/**********************************************************************************************************
*函 数 名: vStartTask
*功能说明: 系统启动任务，调用各类初始化函数，并创建消息队列和要运行的用户任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vStartTask, pvParameters)
{
	//外设驱动初始化
 	BoardInit();
	configASSERT(1+1);
	//参数初始化
	ParamInit();
	//消息队列创建
	MessageQueueCreate();
	/*******************用户任务创建***************************/
	//传感器模块数据读取任务创建
	ModuleTaskCreate();
	
	//传感器数据预处理任务创建
	SensorTaskCreate();
	
	//数据通信任务创建
	MessageTaskCreate();
	
	//姿态解算任务创建
	NavigationTaskCreate();
	
	while(1)
	{
		
		vTaskDelay(5000);	//阻塞5000个tick
	}
	
}

/**********************************************************************************************************
*函 数 名: main
*功能说明: 系统程序入口
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int main()
{
	//创建启动任务
	xTaskCreate(vStartTask, "startTask", 128, NULL, 0, &startHandle);
	//OS调度器启动
	vTaskStartScheduler();
	
	while(1)
	{
	}
}
