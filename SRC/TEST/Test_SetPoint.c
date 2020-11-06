#include "stm32f4xx.h"
#include "delay.h"
#include "rgb.h"
#include "mathConfig.h"
#include "boardConfig.h"
#include "LYHdecode.h"
#include "flightStatus.h"

#include "setPoint.h"

#include "FreeRTOS.h"
#include "task.h"

xTaskHandle trajectoryHandler;


Vector3f_t pos, vel, acc, ang_vel, ang_acc;
float att_TEST[9];




/**********************************************************************************************************
*函 数 名: vTrajectoryTest
*功能说明: 时间管理测试任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
portTASK_FUNCTION(vTrajectoryTest, pvParameters)
{
	
	while(1)
	{
		if(LYH_data_ok)
		{
			LYH_data_ok = 0;
			
			/**********test1 for flash basic read&write***************/
			//命令选择： 's':开始轨线迭代， 't'：停止轨线迭代
			switch(LYH_RxBuffer[4])
			{
				case 's':
					SetFlightMode(MISSION);
				break;
				
				case 't':
					SetFlightMode(MANUAL);
				break;
				
				default: 
				break;
			}
		}
			
		SetPointUpdate();
			
			
		//读取轨线数据
		GetDesiredPos(&pos);
		GetDesiredVel(&vel);
		GetDesiredAcc(&acc);
		GetDesiredAtt(att_TEST);
		GetDesiredAngVel(&ang_vel);
		GetDesiredAngAcc(&ang_acc);
		
		
		
		OsDelayTick(2);
	}
}





int main()
{   
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	RGB_Init();
	Usart2_Init(500000);
	
	//OS调度器启动
	xTaskCreate(vTrajectoryTest, "clock_test", (uint16_t)128, (void*)NULL, 4, &trajectoryHandler);
	vTaskStartScheduler();
	
	
	while(1)
	{
		
		
	}	
}
