#include "stm32f4xx.h"
#include "w25qxx.h"
#include "delay.h"
#include "drv_spi.h"
#include "parameter.h"
#include "drv_usart.h"
#include "string.h"
#include "message.h"
#include "LYHdecode.h"

const u8 TEXT_Buffer[] = {"FCos SPI1 TESTliyunahoasdoalnvoanosnfoansvlnaslikghvoliasnvc olanscvolbnasofgbaolsvnolasnfglabsvlbnaslfvhnaslighnolpsdfnbpsdvsd"};
#define SIZE sizeof(TEXT_Buffer)


uint8_t datatemp[SIZE];

float offset_x;

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	Usart2_Init(500000);
	Spi1_Init();
	W25QXX_Init();
	
	//测试芯片ID读取功能
	u16 ID = 0;
	while(ID != 0xEF15)
	{
		ID = W25QXX_ReadID();
	}
	
	int count=0;
	//float dataupdate = 3.2;
	
	
	while (1)
	{
		if(LYH_data_ok)
		{
			count++;
			LYH_data_ok = 0;
			
			/**********test1 for flash basic read&write***************/
			//命令选择： 'w'：写入flash， 'r'：读flash，
			switch(LYH_RxBuffer[4])
			{
				case 'w':
					W25QXX_SectorErase(PARAM_START_ADDR);
					W25QXX_PageRead(datatemp, PARAM_START_ADDR, SIZE);
					W25QXX_PageWrite((u8*)TEXT_Buffer, 0x000000, SIZE);
				break;
				
				case 'r':
					memset(datatemp, 0, SIZE);
					W25QXX_PageRead(datatemp, PARAM_START_ADDR, SIZE);
				break;
				
				default: 
				break;
			}
			
			/**********test2 for parameter read&write***************/
			//命令选择： 'w'：参数写入， 'r'：参数读取， 'i':参数初始化， 'u':参数更新， 'G':获得参数
//			switch(LYH_RxBuffer[4])
//			{
//				case 'w':
//					Param_save_cnt_tox(1);
//					ParamSaveToFlash();
//				break;
//				
//				case 'r':
//					memset(Param.buffer, 0, PARAM_NUM*4);
//					memset(datatemp, 0, SIZE);
//					W25QXX_PageRead(datatemp, PARAM_START_ADDR, SIZE);
//					ParamReadFromFlash();
//				break;
//				
//				case 'i':
//					ParamInit();
//				break;
//				
//				case 'u':
//					count %= 20;
//					ParamUpdateData(PARAM_GYRO_OFFSET_X+count, &dataupdate);
//				break;
//				
//				case 'G':
//					ParamGetData(PARAM_GYRO_OFFSET_Z, &offset_x, 4);
//				break;
//				
//				default: 
//				break;
//			}
			
		}
		DelayXms(10);
	}
	
	
}
