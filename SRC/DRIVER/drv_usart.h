#ifndef __DRV_USART_H
#define __DRV_USART_H
#include "stm32f4xx.h"
#include <stdio.h>

int fputc(int ch, FILE *f);

void Usart2_Init(uint32_t baudrate);

void Usart2_IRQ(void);

void Usart2_Send (char *DataToSend , u8 data_num );

//void Usart3_Init(uint32_t baudrate);

//void Usart3_IRA(void);
#endif