/**********************************************************************************************************
 * @文件     boardConfig.h
 * @说明     飞控硬件配置文件(引脚，元器件)
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.08
**********************************************************************************************************/
#ifndef __BOARDCONFIG_H
#define __BOARDCONFIG_H

/*板载各个传感器的使能引脚配置*/
#define W25QXX_CS_RCC		RCC_AHB1Periph_GPIOA
#define W25QXX_CS_GPIO		GPIOA
#define W25QXX_CS_PIN		GPIO_Pin_4

#endif
