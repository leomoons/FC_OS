/**********************************************************************************************************
 * @文件     drv_pwm.c
 * @说明     PWM信号输出
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "drv_pwm.h"
#include "mathConfig.h"

//21分频到 84000000/21 = 4M		0.25us

/*初始化高电平时间1000us(4000份)*/
#define INIT_DUTY 4000 	//u16(1000/0.25)
/*频率400Hz*/
#define HZ 400
/*分辨率10000份，每份0.25us*/
#define ACCURACY 10000 //u16(2500/0.25)
/*设置飞控控制信号转换比例为4*/
#define PWM_RADIO 4		//(8000 - 4000)/1000.0

/**********************************************************************************************************
*函 数 名: PWM_Out_Init
*功能说明: PWM信号输出初始化(400Hz)
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void PWM_Out_Init(void)
{
    uint16_t PrescalerValue = 0;
    u32 hz_set = ACCURACY * HZ;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit ( &TIM_TimeBaseStructure );
    TIM_OCStructInit ( &TIM_OCInitStructure );

    hz_set = LIMIT ( hz_set, 1, 84000000 );

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM5, ENABLE );
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM8, ENABLE );
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM1, ENABLE );
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE );

/********************************TIM5初始化*************************************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIO_Pin_0 | GPIO_Pin_1 |
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );

//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource2, GPIO_AF_TIM5 );
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource3, GPIO_AF_TIM5 );

    /* Compute the prescaler value */
    PrescalerValue = ( uint16_t ) ( ( SystemCoreClock / 2 ) / hz_set ) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit ( TIM5, &TIM_TimeBaseStructure );


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

//  /* PWM1 Mode configuration: Channel1 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel2 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC3Init ( TIM5, &TIM_OCInitStructure );
    TIM_OC3PreloadConfig ( TIM5, TIM_OCPreload_Enable );

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init ( TIM5, &TIM_OCInitStructure );
    TIM_OC4PreloadConfig ( TIM5, TIM_OCPreload_Enable );

    TIM_ARRPreloadConfig ( TIM5, ENABLE );
    TIM_Cmd ( TIM5, ENABLE );
	
/***************************TIM1初始化*****************************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOE, &GPIO_InitStructure );

    GPIO_PinAFConfig ( GPIOE, GPIO_PinSource9, GPIO_AF_TIM1 );
    GPIO_PinAFConfig ( GPIOE, GPIO_PinSource11, GPIO_AF_TIM1 );
    GPIO_PinAFConfig ( GPIOE, GPIO_PinSource13, GPIO_AF_TIM1 );
    GPIO_PinAFConfig ( GPIOE, GPIO_PinSource14, GPIO_AF_TIM1 );

    /* Compute the prescaler value */
    PrescalerValue = ( uint16_t ) ( ( SystemCoreClock ) / hz_set ) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit ( TIM1, &TIM_TimeBaseStructure );


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC1Init ( TIM1, &TIM_OCInitStructure );
    //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC2Init ( TIM1, &TIM_OCInitStructure );
    //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC3Init ( TIM1, &TIM_OCInitStructure );
    //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init ( TIM1, &TIM_OCInitStructure );
    //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs ( TIM1, ENABLE );
    TIM_ARRPreloadConfig ( TIM1, ENABLE );
    TIM_Cmd ( TIM1, ENABLE );
	
/*************************TIM8初始化******************************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );

    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource8, GPIO_AF_TIM8 );
    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource9, GPIO_AF_TIM8 );

    /* Compute the prescaler value */
    PrescalerValue = ( uint16_t ) ( ( SystemCoreClock ) / hz_set ) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = ACCURACY;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit ( TIM8, &TIM_TimeBaseStructure );


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    /* PWM1 Mode configuration: Channel3 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC3Init ( TIM8, &TIM_OCInitStructure );
    //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
    TIM_OC4Init ( TIM8, &TIM_OCInitStructure );
    //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs ( TIM8, ENABLE );
    TIM_ARRPreloadConfig ( TIM8, ENABLE );
    TIM_Cmd ( TIM8, ENABLE );
}


/**********************************************************************************************************
*函 数 名: Set_PWM
*功能说明: 电机PWM输出值设置
*形    参: PWM值数组
*返 回 值: void
**********************************************************************************************************/
void Set_PWM(int16_t pwm[])
{
	TIM1->CCR4 = pwm[0];
	TIM1->CCR3 = pwm[1];
	TIM1->CCR2 = pwm[2];
	TIM1->CCR1 = pwm[3];
	TIM5->CCR4 = pwm[4];
	TIM5->CCR3 = pwm[5];
}
