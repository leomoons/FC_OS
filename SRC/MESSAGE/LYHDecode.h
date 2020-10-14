#ifndef __LYHDECODE_H
#define __LYHDECODE_H
#include "stm32f4xx.h"

extern u8 LYH_RxBuffer[10], LYH_data_ok;

void LYH_Data_Receive_Prepare(u8 data);

void LYH_Receive_Loop(void);

#endif
