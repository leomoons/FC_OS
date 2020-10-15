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

Vector3f_t gyro, gyroTreat, gyroLpf;
Vector3f_t acc, accTreat;
Vector3f_t magR;
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
	Spi2_Init();
	
	IMUSensorInit();
	AccPreTreatInit();
	GyroPreTreatInit();
	
	MagSensorInit();
	MagPreTreatInit();
	
	uint32_t cnt = 0;
	while(1)
	{	
		cnt++;
		
		//串口发送命令开启校准
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
		
		
		
		//test1: 直接读取传感器的原始数据
//		if(cnt%10 == 0)
//		{
//			AK8975_Update();
//			AK8975_ReadMag(&magR);
//		}
//		
		//test2: 通过module文件这个接口来更新数据
		if(cnt%10 == 0)
		{
			MagDataUpdate();
			MagDataRead(&magR);
		}
		
		
		//test3: 加速度计和陀螺仪数据处理：校准等
		if(cnt%10 == 0)
		{
			MagCalibration();
			MagDataPreTreat();
		}
		
		
		GyroDataUpdate(&gyro);
		AccDataUpdate(&acc);
		IMUTempUpdate(&temp);
		//加速度计数据处理
		AccCalibration(acc); 
		AccDataPreTreat(acc, &accTreat);
		ImuLevelCalibration();
		//陀螺仪数据处理
		GyroCalibration(gyro);
		GyroDataPreTreat(gyro, temp, &gyroTreat, &gyroLpf);
//		
		
		
		DelayUs(999);
	}
}
