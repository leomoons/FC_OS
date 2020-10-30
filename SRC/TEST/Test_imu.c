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

Vector3f_t gyroR, gyroM , gyroTreat, gyroLpf;
Vector3f_t accR, accM, accTreat;
float tempR, tempM;

int main()
{
 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	RGB_Init();
	Usart2_Init(500000);
	Spi1_Init();
	W25QXX_Init();
	ParamInit();
	Spi2_Init();
	
	//初始化icm20602
//	ICM20602CSPin_Init();
//	ICM20602_Init();
	IMUSensorInit();
	
	AccPreTreatInit();
	GyroPreTreatInit();

	
	
	uint32_t cnt = 0;
	while(1)
	{	
		cnt++;
		//test1: icm20602芯片原始数据更新，向量方向没有旋转修正
//		ICM20602_UpdateGyro();
//		ICM20602_UpdateAcc();
//		ICM20602_UpdateTemp();
//		ICM20602_ReadAcc(&accR);
//		ICM20602_ReadGyro(&gyroR);
//		ICM20602_ReadTemp(&tempR);
		
		//test2: 通过module文件这个接口来更新数据
		AccDataUpdate();
		GyroDataUpdate();
		IMUtempUpdate();
		AccDataRead(&accR);
		GyroDataRead(&gyroR);
		IMUtempRead(&tempR);
		
		
		//test3: 加速度计和陀螺仪数据处理：校准，低通滤波
//		if(cnt%10 == 0)
//		{
//			LYH_Receive_Loop();
//			RGB_Flash();
//			ImuOrientationDetect();
//		}
//		if(cnt%50==0)
//		{
//			ParamSaveToFlash();
//		}
//		//加速度计数据处理
//		AccCalibration(accR);
//		AccDataPreTreat(accR, &accTreat);
//		ImuLevelCalibration();
//		//陀螺仪数据处理
//		GyroCalibration(gyroR);
//		GyroDataPreTreat(gyroR, tempR, &gyroTreat, &gyroLpf);
		
		
		DelayXms(1);
	}
}
