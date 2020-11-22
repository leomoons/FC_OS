#ifndef __DRV_SBUS_H
#define __DRV_SBUS_H
#include "stm32f4xx.h" 
 
extern uint16_t Rc_Sbus_In[16]; 
 
void Sbus_Init(void);
void Sbus_IRQ(void);
 
#endif

