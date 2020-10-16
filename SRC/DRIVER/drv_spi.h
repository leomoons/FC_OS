#ifndef __DRV_SPI_H
#define __DRV_SPI_H
#include "stm32f4xx.h"

void Spi1_Init(void);
uint8_t Spi1_ReadWriteByte(uint8_t TxData);
void Spi1_Receive(uint8_t *pData, uint16_t len);


void Spi2_Init(void);
uint8_t Spi2_ReadWriteByte(uint8_t TxData);
void Spi2_Receive(uint8_t *pData, uint16_t len);

#endif
