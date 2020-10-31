#include "stm32f4xx.h"
#include "delay.h"
#include "rgb.h"
#include "drv_spi.h"
#include "icm20602.h"
#include "vector3.h"
#include "module.h"
#include "drv_usart.h"
#include "message.h"
#include "LYHdecode.h"
#include "parameter.h"
#include "w25qxx.h"
#include "ak8975.h"
#include "spl0601.h"

#include "sensor.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "barometer.h"

float presR, tempR;


int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	RGB_Init();
	Spi2_Init();

	BaroSensorInit();
	
	uint32_t cnt = 0;
	while(1)
	{	
		cnt++;	
		
		//test1: 直接读取传感器的原始数据
//		if(cnt%20 == 0)		//50Hz
//		{
//			SPL0601_Update();
//			SPL0601_ReadPres(&presR);
//			SPL0601_ReadTemp(&tempR);
//		}
		
		//test2: 通过module文件这个接口来更新数据
		if(cnt%20 == 0)		//50Hz
		{
			BaroDataUpdate();
			BaroPresRead(&presR);
			BaroTempRead(&tempR);
		}
		
		
		//test3: 气压计数据预处理
		if(cnt%40 == 0)
		{
			BaroDataPreTreat(presR, tempR);
		}
		
		
		DelayXms(1);
	}
}
