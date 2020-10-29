#ifndef __DRV_USART_H
#define __DRV_USART_H
#include "stm32f4xx.h"
#include <stdio.h>		//关于printf()函数的声明

int fputc(int ch, FILE *f);

void Usart2_Init(uint32_t baudrate);
void Usart2_IRQ(void);
void Usart2_Send(char *DataToSend, u8 data_num );

void Usart3_Init(uint32_t baudrate);
void Usart3_IRQ(void);
void Usart3_Send(char *DataToSend, u8 data_num);
	
#endif
