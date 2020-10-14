/**********************************************************************************************************
 * @文件     rgb.c
 * @说明     RGB驱动
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.8
**********************************************************************************************************/
#include "rgb.h"
#include "flightStatus.h"

/*
RGB Green - PE0
RGB Red   - PE1
RGB Blue  - PE2
LED Red   - PE7
*/

#define LED_GPIO    		GPIOE
#define RGB_GREEN_PIN      	GPIO_Pin_0
#define RGB_RED_PIN     	GPIO_Pin_1
#define RGB_BLUE_PIN       	GPIO_Pin_2
#define LED_RED_PIN			GPIO_Pin_7

static void RGB_Green_Off(void)
{
    GPIO_ResetBits(LED_GPIO, RGB_GREEN_PIN);
}
static void RGB_Green_On(void)
{
    GPIO_SetBits(LED_GPIO, RGB_GREEN_PIN);
}
static void RGB_Red_Off(void)
{
    GPIO_ResetBits(LED_GPIO, RGB_RED_PIN);
}
static void RGB_Red_On(void)
{
    GPIO_SetBits(LED_GPIO, RGB_RED_PIN);
}
static void RGB_Blue_Off(void)
{
    GPIO_ResetBits(LED_GPIO, RGB_BLUE_PIN);
}
static void RGB_Blue_On(void)
{
    GPIO_SetBits(LED_GPIO, RGB_BLUE_PIN);
}
static void LED_Red_On(void)
{
	GPIO_ResetBits(LED_GPIO, LED_RED_PIN);
}
static void LED_Red_Off(void)
{
	GPIO_SetBits(LED_GPIO, LED_RED_PIN);
}



/**********************************************************************************************************
*函 数 名: RGB_Init
*功能说明: RGB初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RGB_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	//RGB的GPIO口配置
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = RGB_GREEN_PIN | RGB_RED_PIN | RGB_BLUE_PIN;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	
	//单独red的GPIO口配置
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin   = LED_RED_PIN;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	
	RGB_Green_Off();
    RGB_Red_Off();
    RGB_Blue_Off();
	LED_Red_On();
	LED_Red_Off();
}

/**********************************************************************************************************
*函 数 名: RGB_Flash
*功能说明: RGB闪烁 运行频率100Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RGB_Flash(void)
{
	static uint32_t cnt = 0;
	
//	switch(GetInitStatus())
//	{
//		//加热中, 红灯常亮
//		case HEATING:
//			if(cnt%3 == 0)
//			{
//				RGB_Red_Off();
//				RGB_Green_Off();
//				RGB_Blue_Off();
//			}	
//			else
//			{
//				RGB_Red_Off();
//				RGB_Green_Off();
//				RGB_Blue_Off();
//			}
//		break;
//		//加热完成，绿灯常亮
//		case HEAT_FINISH:
//			if(cnt%3 == 0)
//			{
//				RGB_Red_Off();
//				RGB_Green_Off();
//				RGB_Blue_Off();
//			}
//			else
//			{
//				RGB_Red_Off();
//				RGB_Green_Off();
//				RGB_Blue_Off();
//			}
//		break;
//		//初始化完成，可以进行传感器校准选项
//		case INIT_FINISH:
//			switch(GetCaliStatus())
//			{	
//				//正常状态，白光常亮
//				case NoCali:
//					if(cnt%3 == 0)
//					{
//						RGB_Red_Off();
//						RGB_Green_Off();
//						RGB_Blue_Off();
//					}
//					else
//					{
//						RGB_Red_Off();
//						RGB_Green_Off();
//						RGB_Blue_Off();
//					}
//				break;
//				//水平校准，红灯快闪
//				case ImuLevelCali:
//					if(cnt % 5 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_On();
//						RGB_Blue_Off(); 
//					}
//					if(cnt % 10 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_Off();
//						RGB_Blue_Off(); 
//					}
//				break;
//				//加速度计采集数据中，绿灯快闪
//				case AccCaliDataCollecting:
//					if(cnt % 5 == 0)
//					{
//						RGB_Green_On();
//						RGB_Red_Off();
//						RGB_Blue_Off(); 
//					}
//					if(cnt % 10 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_Off();
//						RGB_Blue_Off(); 
//					}
//				break;
//				//加速度计一个方向的数据采集完毕，绿灯快闪
//				case AccCaliOneDataReady:
//					if(cnt % 50 == 0)
//					{
//						RGB_Green_On();
//						RGB_Red_Off();
//						RGB_Blue_Off(); 
//					}
//					if(cnt % 100 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_Off();
//						RGB_Blue_Off(); 
//					}
//				break;	
//				//陀螺仪校准，蓝灯快闪
//				case GyroCali:
//					if(cnt % 5 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_Off();
//						RGB_Blue_On(); 
//					}
//					if(cnt % 10 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_Off();
//						RGB_Blue_Off(); 
//					}
//				break;
//				//磁罗盘校准，水平旋转,紫光快闪
//				case MagCaliHorizontal:
//					if(cnt % 5 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_On();
//						RGB_Blue_On(); 
//					}
//					if(cnt % 10 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_Off();
//						RGB_Blue_Off(); 
//					}
//				break;
//				//磁罗盘校准，水平旋转,橙光快闪
//				case MagCaliVertical:
//					if(cnt % 5 == 0)
//					{
//						RGB_Green_On();
//						RGB_Red_On();
//						RGB_Blue_Off(); 
//					}
//					if(cnt % 10 == 0)
//					{
//						RGB_Green_Off();
//						RGB_Red_Off();
//						RGB_Blue_Off(); 
//					}
//				break;
//				
//				default:
//					break;
//			}	
//		break;
//			
//		default:
//			break;
//	}
	//根据传感器校准状态来点灯
	switch(GetCaliStatus())
	{	
		//正常状态，白光常亮
		case NoCali:
			if(cnt%3 == 0)
			{
				RGB_Red_Off();
				RGB_Green_Off();
				RGB_Blue_Off();
			}
			else
			{
				RGB_Red_Off();
				RGB_Green_Off();
				RGB_Blue_Off();
			}
		break;
		//水平校准，红灯快闪
		case ImuLevelCali:
			if(cnt % 5 == 0)
			{
				RGB_Green_Off();
				RGB_Red_On();
				RGB_Blue_Off(); 
			}
			if(cnt % 10 == 0)
			{
				RGB_Green_Off();
				RGB_Red_Off();
				RGB_Blue_Off(); 
			}
		break;
		//加速度计采集数据中，绿灯快闪
		case AccCaliDataCollecting:
			if(cnt % 5 == 0)
			{
				RGB_Green_On();
				RGB_Red_Off();
				RGB_Blue_Off(); 
			}
			if(cnt % 10 == 0)
			{
				RGB_Green_Off();
				RGB_Red_Off();
				RGB_Blue_Off(); 
			}
		break;
		//加速度计一个方向的数据采集完毕，绿灯快闪
		case AccCaliOneDataReady:
			if(cnt % 50 == 0)
			{
				RGB_Green_On();
				RGB_Red_Off();
				RGB_Blue_Off(); 
			}
			if(cnt % 100 == 0)
			{
				RGB_Green_Off();
				RGB_Red_Off();
				RGB_Blue_Off(); 
			}
		break;	
		//陀螺仪校准，蓝灯快闪
		case GyroCali:
			if(cnt % 5 == 0)
			{
				RGB_Green_Off();
				RGB_Red_Off();
				RGB_Blue_On(); 
			}
			if(cnt % 10 == 0)
			{
				RGB_Green_Off();
				RGB_Red_Off();
				RGB_Blue_Off(); 
			}
		break;
		//磁罗盘校准，水平旋转,紫光快闪
		case MagCaliHorizontal:
			if(cnt % 5 == 0)
			{
				RGB_Green_Off();
				RGB_Red_On();
				RGB_Blue_On(); 
			}
			if(cnt % 10 == 0)
			{
				RGB_Green_Off();
				RGB_Red_Off();
				RGB_Blue_Off(); 
			}
		break;
		//磁罗盘校准，水平旋转,橙光快闪
		case MagCaliVertical:
			if(cnt % 5 == 0)
			{
				RGB_Green_On();
				RGB_Red_On();
				RGB_Blue_Off(); 
			}
			if(cnt % 10 == 0)
			{
				RGB_Green_Off();
				RGB_Red_Off();
				RGB_Blue_Off(); 
			}
		break;
				
		default:
			break;
	}
	
	cnt++;
}
