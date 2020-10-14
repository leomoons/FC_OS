#include "stm32f4xx.h"
#include "delay.h"
#include "rgb.h"
#include "flightStatus.h"

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	RGB_Init();
	
	uint32_t count = 0;
	while(1)
	{
		count++;
		if(count%473 == 0)
		{
			SetCaliStatus(count%7);
		}
		
		RGB_Flash();
		DelayMs(10);
	}
}
