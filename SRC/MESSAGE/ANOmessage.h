#ifndef __ANOSEND_H
#define __ANOSEND_H
#include "stm32f4xx.h"

typedef struct
{
    u8 send_status;
    u8 send_sensor;
    u8 send_sensor2;
    u8 send_rcdata;
	u8 send_power;
    u8 send_motopwm;
	u8 send_string;
} dt_flag_t;

extern dt_flag_t ano_flag;


void ANO_Send_Loop(void);
void ANO_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_Send_Senser2(s32 bar_alt,s32 csb_alt, s16 sensertmp);
void ANO_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_Send_Power(u16 votage, u16 current);
void ANO_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_Send_String(const char *str);


#endif
