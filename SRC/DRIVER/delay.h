#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void DelayInit(u8 SYSCLK);
void DelayUs(u32 nus);
void DelayMs(u32 nms);
void DelayXms(u32 nms);
void OsDelayMs(uint32_t ms);

uint64_t GetSysTimeUs(void);
uint32_t GetSysTimeMs(void);

#endif
