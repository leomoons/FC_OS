#ifndef __RGB_H
#define __RGB_H
#include "stm32f4xx.h"

void RGB_Init(void);
void RGB_Flash(void);
void Red_Flash(void);

void RGB_Green_Off(void);
void RGB_Green_On(void);
void RGB_Red_Off(void);
void RGB_Red_On(void);
void RGB_Blue_Off(void);
void RGB_Blue_On(void);
void LED_Red_On(void);
void LED_Red_Off(void);

#endif
