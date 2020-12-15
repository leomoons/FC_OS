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
#include "parameter.h"

#include "accelerometer.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "geoCtrl.h"

u8 LYH_RxBuffer[10], LYH_cali_ok=0;
u8 LYH_controller_ok=0;

/**********************************************************************************************************
*函 数 名: LYH_Data_Receive_Prepare
*功能说明: 解码信息，被usart中断调用每次接收一个字节
*形    参: data，一个字节
*返 回 值: 无
**********************************************************************************************************/
void LYH_Data_Receive_Prepare(u8 data)
{
	static u8 state1 = 0, state2 = 0;
	
	//校准指令解析
	if(state1==0 && data==0x46)	//帧头0x46('F')
	{
		state1=1;
		LYH_RxBuffer[0]=data;
	}
	else if(state1==1 && data==0x75)	//第二帧0x75('u')
	{
		state1=2;
		LYH_RxBuffer[1]=data;
	}
	else if(state1==2 && data==0x63)		//第三帧0x63('c')
	{
		state1=3;
		LYH_RxBuffer[2]=data;
	}
	else if(state1==3 && data==0x6B)		//第四帧0x6B('k')
	{
		state1=4;
		LYH_RxBuffer[3]=data;
	}
	else if(state1==4)		//命令选择，
	{
		state1 = 5;
		LYH_RxBuffer[4]=data;
		LYH_cali_ok = 1;
	}
	else
		state1 = 0;
	
	// 修改控制器参数指令解析
	if(state2==0 && data==0x50)	//帧头0x50('P')
	{
		state2=1;
		LYH_RxBuffer[0] = data;
	}
	//选择p:0x70 v:0x76 R:0x52 W:0x57
	else if(state2==1 && (data==0x70 || data==0x76 || data==0x52 || data==0x57))	
	{
		state2=2;
		LYH_RxBuffer[1] = data;
	}	
	//选择三个轴中的一个
	else if(state2==2 && (data==0x78 || data==0x79 || data==0x7A))
	{
		state2=3;
		LYH_RxBuffer[2] = data;
	}
	//参数载入buffer
	else if(state2>=3 && LYH_RxBuffer[0] == 0x50)
	{
		LYH_RxBuffer[state2++] = data;
		if(state2 == 7)
		{
			state2=0;
			LYH_controller_ok = 1;
		}
	}
	else 
		state2=0;
}



/**********************************************************************************************************
*函 数 名: LYH_Receive_Loop
*功能说明: 解码信息并完成相应操作，调用频率1000Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void LYH_Receive_Loop(void)
{
	char *str;
	if(LYH_cali_ok)
	{
		LYH_cali_ok = 0;
		
		//命令选择: 'a'：加速度计校准， 'g'：陀螺仪校准， 'm': 磁力计校准， 'l'：水平校准, 'r':参数reset
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
			
			case 'r':
				ParamBufferReset();
				GeoControllerInit();
			break;
			
			default:
				break;	
		}
	}
	if(LYH_controller_ok)
	{
		LYH_controller_ok = 0;
		
		float paramData;
		uint8_t* pp = (uint8_t*)&paramData;
		pp[0] = LYH_RxBuffer[6];
		pp[1] = LYH_RxBuffer[5];
		pp[2] = LYH_RxBuffer[4];
		pp[3] = LYH_RxBuffer[3];
		
		int paramSelect = 0;
		
		// 控制器参数修改选择：
		// 'p':位置环节参数， 'v'：速度环节参数， 'R'：姿态环节参数， 'W'：角速度环节参数
		switch(LYH_RxBuffer[1])
		{
			case 'p':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_PD_Kp_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_PD_Kp_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_PD_Kp_Z;
					break;
				}
			break;
			
			case 'v':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_PD_Kv_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_PD_Kv_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_PD_Kv_Z;
					break;
				}
			break;	
				
			case 'R':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_PD_KR_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_PD_KR_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_PD_KR_Z;
					break;
				}
			break;
						
			case 'W':
				switch(LYH_RxBuffer[2])
				{
					case 'x':
						paramSelect = CONTROLLER_PD_KW_X;
					break;
					case 'y':
						paramSelect = CONTROLLER_PD_KW_Y;
					break;
					case 'z':
						paramSelect = CONTROLLER_PD_KW_Z;
					break;
				}
			break;						
		}
		
		GeoCtrlUpdateParam(paramSelect, paramData);
	}
}
