#include "stm32f4xx.h"
#include "drv_sbus.h"
#include "message.h"
#include "delay.h"
#include "remote.h"


int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	
	Sbus_Init();
	
	while(1)
	{
		RC_Duty_Task(2);
		
		DelayMs(2);
	}
}
