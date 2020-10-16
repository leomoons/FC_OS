/**********************************************************************************************************
 * @文件     boardConfig.h
 * @说明     飞控硬件配置文件(引脚，元器件)
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.08
**********************************************************************************************************/
#ifndef __BOARDCONFIG_H
#define __BOARDCONFIG_H

#include "board.h"
#include "delay.h"
#include "drv_spi.h"
#include "drv_usart.h"
#include "drv_sbus.h"
#include "drv_pwm.h"
#include "rgb.h"

enum 
{
	ICM20602,
	AK8975,
	SPL0601,
	W25QXX
};

/**********************************************************************************************************
*信息发送通道
**********************************************************************************************************/
#define DT_USE_USART2 				//开启串口2数传功能
//#define DT_USE_USB_HID				//开启飞控USBHID连接上位机功能

/**********************************************************************************************************
*传感器安装方向
**********************************************************************************************************/
#define GYRO_ROTATION       ROTATION_YAW_270
#define ACC_ROTATION        ROTATION_YAW_90
#define MAG_ROTATION        ROTATION_NONE

/**********************************************************************************************************
*传感器配置
**********************************************************************************************************/
#define IMU_TYPE             ICM20602      	//IMU型号
#define BARO_TYPE            SPL0601       	//气压计型号
#define MAG_TYPE             AK8975        	//罗盘型号
#define FLASH_TYPE			 W25QXX			//Flash型号


/*板载各个传感器的使能引脚配置*/
#define W25QXX_CS_RCC		RCC_AHB1Periph_GPIOA
#define W25QXX_CS_GPIO		GPIOA
#define W25QXX_CS_PIN		GPIO_Pin_4
#define ICM20602_CS_RCC		RCC_AHB1Periph_GPIOD
#define ICM20602_CS_GPIO	GPIOD
#define ICM20602_CS_PIN		GPIO_Pin_0
#define AK8975_CS_RCC		RCC_AHB1Periph_GPIOC
#define AK8975_CS_GPIO		GPIOC
#define AK8975_CS_PIN		GPIO_Pin_10
#define SPL06_CS_RCC		RCC_AHB1Periph_GPIOC
#define SPL06_CS_GPIO		GPIOC
#define SPL06_CS_PIN		GPIO_Pin_11


#endif
