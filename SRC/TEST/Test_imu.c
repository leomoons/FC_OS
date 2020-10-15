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

#include "sensor.h"
#include "accelerometer.h"
#include "gyroscope.h"


Vector3f_t gyro, gyroTreat, gyroLpf;
Vector3f_t acc, accTreat;
float temp;

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
	
//	ICM20602CSPin_Init();
//	ICM20602_Init();
	IMUSensorInit();
	
	AccPreTreatInit();
	GyroPreTreatInit();
	
	uint32_t cnt = 0;
	while(1)
	{	
		cnt++;
		//test1: icm20602芯片原始数据更新
//		ICM20602_UpdateGyro(&gyro);
//		ICM20602_UpdateAcc(&acc);
//		ICM20602_UpdateTemp(&temp);
		
		
		//test2: 通过module文件这个接口来更新数据
		GyroDataUpdate(&gyro);
		AccDataUpdate(&acc);
		IMUTempUpdate(&temp);
		
		
		//test3: 加速度计和陀螺仪数据处理：校准，低通滤波
		if(cnt%10 == 0)
		{
			LYH_Receive_Loop();
			RGB_Flash();
			ImuOrientationDetect();
		}
		if(cnt%50==0)
		{
			ParamSaveToFlash();
		}
		//加速度计数据处理
		AccCalibration(acc);
		AccDataPreTreat(acc, &accTreat);
		ImuLevelCalibration();
		//陀螺仪数据处理
		GyroCalibration(gyro);
		GyroDataPreTreat(gyro, temp, &gyroTreat, &gyroLpf);
		
		
		DelayUs(999);
	}
}
