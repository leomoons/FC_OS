#include "stm32f4xx.h"

int main()
{
	//获取当前时钟频率
	RCC_ClocksTypeDef get_rcc_clock;
	RCC_GetClocksFreq(&get_rcc_clock);

	return 1;
}
