/**********************************************************************************************************
 * @文件     drv_usart.c
 * @说明     串口驱动，usart2用于和串口助手通信校准传感器，发送字符串。 usart3用于接收opti_track位姿信号
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "drv_usart.h"
#include "LYHDecode.h"



//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif

/**********************************************************************************************************
*函 数 名: Usart2_Init
*功能说明: 串口2初始化
*形    参: baudrate:波特率设置
*返 回 值: 无
**********************************************************************************************************/
void Usart2_Init(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART2, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource5, GPIO_AF_USART2 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource6, GPIO_AF_USART2 );

    //配置PD5作为USART2　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );
    //配置PD6作为USART2　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );

    //配置USART2
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = baudrate;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART2时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART2, &USART_InitStructure );
    USART_ClockInit ( USART2, &USART_ClockInitStruct );

    //使能USART2接收中断
    USART_ITConfig ( USART2, USART_IT_RXNE, ENABLE );
    //使能USART2
    USART_Cmd ( USART2, ENABLE );
}



u8 TxBuffer[256];
u8 TxCounter = 0;
u8 count = 0;

u8 RxBuffer[256];


/**********************************************************************************************************
*函 数 名: Usart2_IRQ
*功能说明: USART2中断函数，包括ORE(串口溢出中断），接收到信号中断，发送信号中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Usart2_IRQ(void)
{
	u8 com_data;
	
	if ( USART2->SR & USART_SR_ORE ) //ORE中断
	{
		com_data = USART2->DR;
	}

	//接收中断
	if ( USART_GetITStatus ( USART2, USART_IT_RXNE ) )
	{
		USART_ClearITPendingBit ( USART2, USART_IT_RXNE ); //清除中断标志

		com_data = USART2->DR;
		LYH_Data_Receive_Prepare(com_data);
	}
	//发送（进入移位）中断
	if ( USART_GetITStatus ( USART2, USART_IT_TXE ) )
	{

		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
		if ( TxCounter == count )
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}
	
}


/**********************************************************************************************************
*函 数 名: Usart2_Send
*功能说明: 将数据存入到TxBuffer缓存，并使能发射中断 
*形    参: DataToSend: 要发送的字符串指针
		   data_num: 字符串长度
*返 回 值: 无
**********************************************************************************************************/
void Usart2_Send (char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        TxBuffer[count++] = * ( DataToSend + i );
    }

    if ( ! ( USART2->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( USART2, USART_IT_TXE, ENABLE ); //打开发送中断
    }

}


