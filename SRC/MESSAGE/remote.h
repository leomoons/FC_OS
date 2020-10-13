#ifndef __REMOTE_H
#define __REMOTE_H
#include "stm32f4xx.h"

typedef struct
{
	u16 s_cnt;
	u8 s_now_times;
	u8 s_state;
}_stick_f_c_st;


enum
{
	CH1 = 0,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6,
	CH7,
	CH8,
	CH_NUM
};


void Remote_Control_Init(void);
void RC_Duty_Task(u8 dT_ms);
void ch_watch_dog_feed(u8 ch_n);

#endif
