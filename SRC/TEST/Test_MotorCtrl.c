#include "stm32f4xx.h"
#include "delay.h"
#include "drv_pwm.h"
#include "delay.h"

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	DelayInit(168);
	
	PWM_Out_Init();
	
	int16_t i= 0;
	u8 direction = 0;	
	
	int16_t pwm[6] = {5940,5940,5940,5940,5940,5940};
	Set_PWM(pwm);
	DelayMs(200);
	
	while(1)
	{
		for(int num=0; num<6; num++)
		{
			pwm[num] = 20*i+5940;
		}
		Set_PWM(pwm);
				
		if(direction == 0)
		{
			i=i+1;
			if(i>=80)	direction=1;
		}
		else if(direction == 1)
		{
			i=i-1;
			if(i<=-80) direction=0;
		}
			
		DelayMs(30);
	}
	
}
