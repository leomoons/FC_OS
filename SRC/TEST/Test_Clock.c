#include "stm32f4xx.h"
#include "delay.h"

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//获取当前时钟频率，进入调试可以查看数值
	RCC_ClocksTypeDef get_rcc_clock;
	RCC_GetClocksFreq(&get_rcc_clock);
	
	//测试延时函数的效果
	DelayInit(168);
	while(1)
	{
		DelayUs(500);
	}
}
