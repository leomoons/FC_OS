#include "stm32f4xx.h"
#include "drv_usart.h"
#include "message.h"
#include "delay.h"
#include "LYHDecode.h"
#include "rgb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "ANOmessage.h"

xTaskHandle messageHandler;

/**********************************************************************************************************
*函 数 名: vMesageTest
*功能说明: 时间管理测试任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vMessageTest, pvParameters)
{
	
	uint32_t i=0;
	
	while(1)
	{
		//teat1: usart2
		//printf("printf function test\r\n");
	
		//test2: message(usart2&usart3)
		//MessageSendString("you dian sao\r");
	
		//test3: LYHDecode
		LYH_Receive_Loop();
		
		//test4: send message to ano ground station
		ANO_Send_Status(i%20, i%30, i%20, 10.0, 0,0);
		i++;
		
		
		OsDelayTick(50);
	}
}

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	RGB_Init();		//定时器中断中会调用red led闪烁功能
	
	Usart2_Init(500000);
	Usart3_Init(115200);
	
	//OS调度器启动
	xTaskCreate(vMessageTest, "message_test", (uint16_t)128, (void*)NULL, 4, &messageHandler);
	vTaskStartScheduler();

	while(1)
	{

	}
}

