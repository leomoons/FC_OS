 /**********************************************************************************************************
 * @文件     delay.c
 * @说明     时间延迟
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

static u8  fac_us = 0;						//us延时倍乘数
static u16 fac_ms = 0;						//ma延时倍乘数，在OS下，代表每个节拍的ms数

	
/**********************************************************************************************************
*函 数 名: DelayInit
*功能说明: 延时函数初始化
*形    参: SYSCLK  系统时钟频率(Mhz)
*返 回 值: 无
*说    明： SYSTICK的时钟固定为AHB时钟，方便兼容FreeRTOS
**********************************************************************************************************/
void DelayInit(u8 SYSCLK)
{
	u32 reload;
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); 
	fac_us=SYSCLK;							//不论是否使用OS,fac_us都需要使用
	reload=SYSCLK;							//每秒钟的计数次数 单位为M	   
	reload*=1000000/configTICK_RATE_HZ;		//根据configTICK_RATE_HZ设定溢出时间
											//reload为24位寄存器,最大值:16777216,在168M下,约合0.0998s左右	
	fac_ms=1000/configTICK_RATE_HZ;			//代表OS可以延时的最少单位	   
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
	SysTick->LOAD=reload; 					//每1/configTICK_RATE_HZ断一次	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启SYSTICK    
}

/**********************************************************************************************************
*函 数 名: DelayUs
*功能说明: us级延时
*形    参: nus 要延时的us数
*返 回 值: 无
*说    明： nus:0~204522252(最大值即2^32/fac_us@fac_us=168)
**********************************************************************************************************/
void DelayUs(u32 nus)
{
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	};	
}

/**********************************************************************************************************
*函 数 名: DelayMs
*功能说明: ms级延时
*形    参: nms 要延时的ms数
*返 回 值: 无
**********************************************************************************************************/
void DelayMs(u32 nms)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
	{		
		if(nms>=fac_ms)						//延时的时间大于OS的最少时间周期 
		{ 
   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS延时
		}
		nms%=fac_ms;						//OS已经无法提供这么小的延时了,采用普通方式延时    
	}
	DelayUs((u32)(nms*1000));				//普通方式延时
}

/**********************************************************************************************************
*函 数 名: DelayXms
*功能说明: ms级延时，不会引起任务调度
*形    参: nms 要延时的ms数
*返 回 值: 无
**********************************************************************************************************/
void DelayXms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) DelayUs(1000);
}
