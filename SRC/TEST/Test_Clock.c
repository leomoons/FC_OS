#include "stm32f4xx.h"
#include "boardConfig.h"

#include "FreeRTOS.h"
#include "task.h"

uint64_t us;
uint32_t ms;

xTaskHandle clockHandler;

/**********************************************************************************************************
*函 数 名: vClockTest
*功能说明: 时间管理测试任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vClockTest, pvParameters)
{
	
	while(1)
	{
		DelayUs(500);
		DelayXms(1000);
		
		//OS调度下的时间功能函数
		DelayMs(10);
		us = GetSysTimeUs();
		ms = GetSysTimeMs();
		
		OsDelayTick(2);
	}
}

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//获取当前时钟频率，进入调试可以查看数值
	RCC_ClocksTypeDef get_rcc_clock;
	RCC_GetClocksFreq(&get_rcc_clock);
	
	//延时功能初始化
	DelayInit(168);
	RGB_Init();		//定时器中断中会调用red led闪烁功能
	
	
	//OS调度器启动
	xTaskCreate(vClockTest, "clock_test", (uint16_t)128, (void*)NULL, 4, &clockHandler);
	vTaskStartScheduler();

	while(1)
	{

	}
}
