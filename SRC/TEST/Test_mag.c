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

Vector3f_t gyroR1, gyroP, gyroLpf;
Vector3f_t accR1, accP;
Vector3f_t magR1, magP;
float tempR;

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
		
		
		
		// test1: 通过ak8975文件来更新和读取磁场数据，向量方向没有旋转修整
//		if(cnt%10 == 0)		//100Hz
//		{
//			AK8975_Update();
//			AK8975_ReadMag(&magR);
//		}
		
		// test2: 通过module接口来更新数据
		if(cnt%10 == 0)		//100Hz
		{
			MagDataUpdate();
			MagDataRead(&magR1);
		}
		
		/****test3: 测试校准和预处理功能******/
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
		AccDataUpdate();
		DelayUs(10);
		GyroDataUpdate();
		DelayUs(10);
		IMUtempUpdate();
		DelayUs(10);
		AccDataRead(&accR1);
		GyroDataRead(&gyroR1);
		IMUtempRead(&tempR);
		ImuLevelCalibration();
		AccCalibration(&accR1);
		AccDataPreTreat(&accR1, &accP);
		GyroCalibration(&gyroR1);
		GyroDataPreTreat(&gyroR1, tempR, &gyroP, &gyroLpf);
		
		if(cnt%10 == 0)
		{
			MagCalibration(&magR1, &gyroLpf);
			MagDataPreTreat(&magR1, &magP);
		}
		
		
		if(cnt%5 == 0)
			RGB_Flash();		//200Hz
		
		DelayXms(1);
	}
}
