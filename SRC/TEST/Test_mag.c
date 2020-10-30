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

#include "sensor.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"

Vector3f_t gyroR;
Vector3f_t accR;
Vector3f_t magR;

int main()
{
 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	RGB_Init();
	Usart2_Init(500000);
	Spi1_Init();
	W25QXX_Init();
	ParamInit();
	
	//初始化icm20602
	Spi2_Init();
	
	IMUSensorInit();
	//DelayUs(10);
	//AccPreTreatInit();
	//GyroPreTreatInit();
	
	MagSensorInit();
	
	uint32_t cnt = 0;
	while(1)
	{	
		cnt++;
		
		//更新和读取IMU数据用于磁罗盘校准
		AccDataUpdate();
		GyroDataUpdate();
		//DelayUs(1000);
		AccDataRead(&accR);
		GyroDataRead(&gyroR);
		
		
		// test1: 通过ak8975文件来更新和读取磁场数据，向量方向没有旋转修整
		if(cnt%10 == 0)		//100Hz
		{
			AK8975_Update();
			AK8975_ReadMag(&magR);
		}
		
		// test2: 通过module接口来更新数据
//		if(cnt%10 == 0)		//100Hz
//		{
//			MagDataUpdate();
//			MagDataRead(&magR);
//		}
		
		DelayXms(1);
	}
}
