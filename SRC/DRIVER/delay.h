#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f4xx.h"

void DelayInit(uint8_t SYSCLK);
void DelayUs(uint32_t nus);
void DelayMs(uint32_t nms);
void DelayXms(uint32_t nms);
void OsDelayTick(uint32_t tick);

uint64_t GetSysTimeUs(void);
uint32_t GetSysTimeMs(void);

#endif
