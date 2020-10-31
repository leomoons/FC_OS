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

Vector3f_t gyroR, gyroP, gyroLpf;
Vector3f_t accR, accP;
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
		
		//串口发送指令开启校准，飞控朝向判断
		if(cnt%10 == 0)		//100Hz
		{
			LYH_Receive_Loop();
			ImuOrientationDetect();
		}
		
		//参数写入flash
		if(cnt%1000 == 0)
		{
			//Param_save_cnt_tox(1);
			ParamSaveToFlash();
		}
			
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
		//加速度计校准和数据预处理
		ImuLevelCalibration();
		AccCalibration(accR);
		AccDataPreTreat(accR, &accP);
		
		//陀螺仪校准和数据预处理
		GyroCalibration(gyroR);
		GyroDataPreTreat(gyroR, tempR, &gyroP, &gyroLpf);
		
		
		if(cnt%5 == 0)
			RGB_Flash();		//200Hz
		DelayXms(1);
	}
}
