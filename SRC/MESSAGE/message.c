/**********************************************************************************************************
 * @文件     message.c
 * @说明     飞控数据通信，通过USART串口发送接收
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2018.07 
**********************************************************************************************************/
#include "message.h"
#include "drv_usart.h"
#include "ANOmessage.h"

//通信协议类型（发送）
enum MESSAGE_TYPE
{
	UNKNOWN = 0,
	ANO,			//发送的信心能被匿名地面站解析
	MAVLINK
};

/*
 *mavlink协议参考
 *http://mavlink.org/messages/common
 *https://mavlink.io/en/messages/common.html
**/

//定义当前所使用的的通信协议类型
enum MESSAGE_TYPE messageType = ANO; 

/**********************************************************************************************************
*函 数 名: MessageSendLoop
*功能说明: 检测是否有需要发送的数据帧
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageSendLoop(void)
{
	/*匿名地面站发送消息*/
	if(messageType == ANO)
	{
		ANO_Send_Loop();
		//MessageSendString("Fuck you");
	}
	else if(messageType == MAVLINK)	//mavlink发送循环
	{
		
	}
}


/**********************************************************************************************************
*函 数 名: MessageSendString
*功能说明: 通过Usart2发送字符串
*形    参: 字符指针
*返 回 值: 无
**********************************************************************************************************/
void MessageSendString(char *str)
{
	u8 cnt = 0;
	while(*(str+cnt) != '\0')
	{
		cnt++;
		if(cnt>50)	break;
	}
	Usart2_Send(str, cnt);
	//Usart3_Send(str, cnt);	//测试usart3信号发送时调用
}

