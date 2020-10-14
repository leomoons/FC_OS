#include "stm32f4xx.h"
#include "drv_usart.h"
#include "message.h"
#include "delay.h"
#include "LYHDecode.h"
#include "rgb.h"

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	
	Usart2_Init(500000);
	RGB_Init();
	
	while(1)
	{
		//teat1: usart
		printf("asshole");
	
		//test2: message
		MessageSendString("liyuanhao niubi");
	
		//test4: LYHDecode
		LYH_Receive_Loop();
		DelayMs(2000);
	}
	
}
