/**********************************************************************************************************
 * @文件     anoSend.c
 * @说明     匿名地面站数据帧发送
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "ANOmessage.h"
#include "drv_usart.h"
#include "boardConfig.h"

/*数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，
  需要把数据拆分成单独字节进行发送*/
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

//通信协议中用于校验的字节
#define MYHWADDR	0x05
#define SWJADDR		0xAF


char data_to_send[50];		//数据发送的缓存
dt_flag_t ano_flag;			//需要发送数据的标志

/**********************************************************************************************************
*函 数 名: ANO_Send_Data
*功能说明: Send_Data函数是协议中所有发送数据功能使用到的发送函数
*          移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
*形    参: 发送字节长度
*返 回 值: 无
**********************************************************************************************************/
static void ANO_Send_Data(u8 length)
{
#ifdef DT_USE_USB_HID
	Usb_Hid_Adddata(data_to_send,length);
#endif
#ifdef DT_USE_USART2
	Usart2_Send(data_to_send, length);
#endif
}

/**********************************************************************************************************
*函 数 名: ANO_Send_Loop
*功能说明: 处理各种数据发送请求，调用频率1000Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ANO_Send_Loop(void)
{
	static u16 cnt = 0;
	const u16 status_cnt	= 15;
	const u16 sensor_cnt 	= 10;
	const u16 sensor2_cnt	= 50;
	const u16 rcdata_cnt	= 20;
	const u16 power_cnt		= 50;
	const u16 motopwm_cnt	= 20;
	
	/*信息发送flag经过固定时间后置1*/
	if((cnt%status_cnt) == (status_cnt-1))
		ano_flag.send_status = 1;
	if((cnt%sensor_cnt) == (sensor_cnt-1))
		ano_flag.send_sensor = 1;
	if((cnt%sensor2_cnt) == (sensor2_cnt-1))
		ano_flag.send_sensor2 = 1;
	if((cnt%rcdata_cnt) == (rcdata_cnt-1))
		ano_flag.send_rcdata = 1;
	if((cnt%power_cnt) == (power_cnt-1))
		ano_flag.send_power = 1;
	if((cnt%motopwm_cnt) == (motopwm_cnt-1))
		ano_flag.send_motopwm = 1;
	
	/*调用各类型数据发送函数，一次只能调用一个*/
	if(ano_flag.send_status)
	{
		ano_flag.send_status = 0;
		
	}
	else if(ano_flag.send_sensor)
	{
		ano_flag.send_sensor = 0;
		
	}
	else if(ano_flag.send_sensor2)
	{
		ano_flag.send_sensor2 = 0;
	}
	else if(ano_flag.send_rcdata)
	{
		ano_flag.send_rcdata = 0;
	}
	else if(ano_flag.send_power)
	{
		ano_flag.send_power = 0;
	}
	else if(ano_flag.send_rcdata)
	{
		ano_flag.send_motopwm = 0;
	}
}

/**********************************************************************************************************
*函 数 名: ANO_Send_Status
*功能说明: 向匿名地面站发送基本飞行状态信息
*形    参: roll, pitch, yaw, altitude, fly_mode, arm status
*返 回 值: 无
**********************************************************************************************************/
void ANO_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_Send_Data(_cnt);
}	
	
/**********************************************************************************************************
*函 数 名: ANO_Send_Sensor
*功能说明: 向匿名地面站发送陀螺仪，加速度计，磁罗盘测得数据
*形    参: 加速度计三轴数据(a_x,a_y,a_z)，陀螺仪三轴数据(g_x,g_y,g_z)，磁罗盘三轴数据(m_x,m_y,m_z)，
*返 回 值: 无
**********************************************************************************************************/
void ANO_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_Send_Data(_cnt);
}

/**********************************************************************************************************
*函 数 名: ANO_Send_Sensor2
*功能说明: 向匿名地面站发送 气压计海拔高度，相对高度，传感器温度
*形    参: bar_alt, csb_alt, sensortmp
*返 回 值: 无
**********************************************************************************************************/
void ANO_Send_Senser2(s32 bar_alt,s32 csb_alt, s16 sensertmp)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE3(csb_alt);
	data_to_send[_cnt++]=BYTE2(csb_alt);
	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[_cnt++]=BYTE1(sensertmp);
	data_to_send[_cnt++]=BYTE0(sensertmp);
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_Send_Data(_cnt);
}

/**********************************************************************************************************
*函 数 名: ANO_Send_RCData
*功能说明: 向匿名地面站发送遥控器各通道值
*形    参: throttle, yaw, roll, pitch, aux1-6
*返 回 值: 无
**********************************************************************************************************/
void ANO_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_Send_Data(_cnt);
}

/**********************************************************************************************************
*函 数 名: ANO_Send_Power
*功能说明: 向匿名地面站发送外接电源电压，电流
*形    参: volatge, current
*返 回 值: 无
**********************************************************************************************************/
void ANO_Send_Power(u16 voltage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = voltage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_Send_Data(_cnt);
}

/**********************************************************************************************************
*函 数 名: ANO_Send_MotoPWM
*功能说明: 向匿名地面站发送八个电机的控制PWM占空比
*形    参: m_1, m_2, m_3, m_4, m_5, m_6, m_7, m_8 
*返 回 值: 无
**********************************************************************************************************/
void ANO_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_Send_Data(_cnt);
}

/**********************************************************************************************************
*函 数 名: ANO_Send_String
*功能说明: 向匿名地面站发送其能解析的字符串
*形    参: 字符指针 
*返 回 值: 无
**********************************************************************************************************/
void ANO_Send_String(const char *str)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xA0;
	data_to_send[_cnt++]=0;
	u8 i = 0;
	while(*(str+i) != '\0')
	{
		data_to_send[_cnt++] = *(str+i++);
		if(_cnt > 50)
			break;
	}
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_Send_Data(_cnt);
}
