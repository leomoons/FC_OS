#include "stm32f4xx.h"
#include "w25qxx.h"
#include "delay.h"
#include "drv_spi.h"

const u8 TEXT_Buffer[] = {"FCos SPI1 TEST"};
#define SIZE sizeof(TEXT_Buffer)

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	Spi1_Init();
	W25QXX_Init();
	DelayInit(168);
	
	u16 ID = 0;
	while(ID != 0xEF15)
	{
		ID = W25QXX_ReadID();
	}
	
	int count = 0;
	
	//Test1: W25qxx 
//	u8 datatemp[SIZE] = {0};
//	while (1)
//	{
//		count++;
//		if(count==10)
//			W25QXX_PageWrite((u8*)TEXT_Buffer, 0x000000, SIZE);
//		if(count==20)
//			W25QXX_PageRead(datatemp, 0x000000, SIZE);
//		
//		DelayUs(10);
//	}
	
	//Test2: Parameter read and write
	while(1)
	{
		
	}
	
}
