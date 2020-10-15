/**********************************************************************************************************
 * @文件     LYHDecode.c
 * @说明     简单的信息解码并实现相应控制
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "LYHDecode.h"  
#include "stdlib.h"
#include "message.h"

#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"

u8 LYH_RxBuffer[10], LYH_data_ok=0;

/**********************************************************************************************************
*函 数 名: LYH_Data_Receive_Prepare
*功能说明: 解码信息，被usart中断调用每次接收一个字节
*形    参: data，一个字节
*返 回 值: 无
**********************************************************************************************************/
void LYH_Data_Receive_Prepare(u8 data)
{
	static u8 state = 0;
	
	if(state==0&&data==0x46)	//帧头0x46('F')
	{
		state=1;
		LYH_RxBuffer[0]=data;
	}
	else if(state==1&&data==0x75)	//第二帧0x75('u')
	{
		state=2;
		LYH_RxBuffer[1]=data;
	}
	else if(state==2 && data==0x63)		//第三帧0x63('c')
	{
		state=3;
		LYH_RxBuffer[2]=data;
	}
	else if(state==3 && data==0x6B)		//第四帧0x6B('k')
	{
		state=4;
		LYH_RxBuffer[3]=data;
	}
	else if(state==4)		//命令选择，
	{
		state = 5;
		LYH_RxBuffer[4]=data;
		LYH_data_ok = 1;
	}
	else
		state = 0;
}


/**********************************************************************************************************
*函 数 名: LYH_Receive_Loop
*功能说明: 解码信息并完成相应操作，调用频率1000Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void LYH_Receive_Loop(void)
{
	if(LYH_data_ok)
	{
		LYH_data_ok = 0;
		char *str;
		//命令选择: 'a'：加速度计校准， 'g'：陀螺仪校准， 'm': 磁力计校准， 'l'：水平校准
		switch(LYH_RxBuffer[4])			
		{
			case 'a':
				AccCalibrateEnable();
				str = "accelerator calibration";
				MessageSendString(str);
			break;
			
			case 'l':
				LevelCalibrateEnable();
				str = "IMU Level calibration";
				MessageSendString(str);
			break;
			
			case 'g':
				GyroCalibrateEnable();
				str = "gyroscope calibreation";
				MessageSendString(str);
			break;
			
			case 'm':
				MagCalibrateEnable();
			
				str = "Magnerometer calibration";
				MessageSendString(str);
			break;
			
			default:
				break;	
		}
	}
}
