/**********************************************************************************************************
 * @文件     message_task.c
 * @说明     飞控数据通信相关任务
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "message_task.h"
#include "TaskConfig.h"

#include "LYHdecode.h"
#include "remote.h"
#include "message.h"
#include "optitrack.h"

xTaskHandle messageHandle;

/**********************************************************************************************************
*函 数 名: vMessageTask
*功能说明: 数据通信任务(1000Hz)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vMessageTask, pvParameters)
{
    portTickType xLastWakeTime;

	uint32_t cnt = 0;
	Remote_Control_Init();
	
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
		//发送飞控数据
		MessageSendLoop();
		
		//解码接收信息
		LYH_Receive_Loop();
		
		//解码optitrack信息
		Opti_Get_Data_Task();
			
		//100Hz遥控接收
		if(cnt%10==0)
		{
			RC_Duty_Task(10);
		}
		vTaskDelayUntil(&xLastWakeTime, (1 / portTICK_RATE_MS));
    }
}

/**********************************************************************************************************
*函 数 名: MessageTaskCreate
*功能说明: 数据通信任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageTaskCreate(void)
{
    xTaskCreate(vMessageTask, "message", MESSAGE_TASK_STACK, NULL, MESSAGE_TASK_PRIORITY, &messageHandle);
}

