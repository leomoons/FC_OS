/**********************************************************************************************************
 * @文件     spl0601.c
 * @说明     spl0601气压传感器驱动
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "spl0601.h"
#include "boardConfig.h"
#include "drv_spi.h"

#define PRESSURE_SENSOR			0
#define TEMPERATURE_SENSOR		1

#define SPL_RA_PSR_B2   	0x00
#define SPL_RA_PSR_B1		0x01
#define SPL_RA_PSR_B0   	0x02
#define SPL_RA_TMP_B2		0x03
#define SPL_RA_TMP_B1   	0x04
#define SPL_RA_TMP_B0		0x05
#define SPL_RA_PRS_CFG   	0x06
#define SPL_RA_TMP_CFG		0x07
#define SPL_RA_MEAS_CFG		0x08
#define SPL_RA_CFG_REG		0x09
#define SPL_RA_INT_STS		0x0A
#define SPL_RA_FIFO_STS		0x0B
#define SPL_RA_RESET		0x0C
#define SPL_RA_ID			0x0D

_spl0601_t spl0601;

/**********************************************************************************************************
*函 数 名: SPL0601_CS
*功能说明: SPL0601传感器CS脚使能或失能
*形    参: 1代表使能，0代表失能
*返 回 值: 无
**********************************************************************************************************/
static void SPL0601_CS(u8 ena)
{
	if ( ena )
        GPIO_ResetBits ( SPL06_CS_GPIO, SPL06_CS_PIN );
    else
        GPIO_SetBits ( SPL06_CS_GPIO, SPL06_CS_PIN );
}

/**********************************************************************************************************
*函 数 名: SPL0601CSPin_Init
*功能说明: SPL06 CS使能引脚初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SPL0601CSPin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(SPL06_CS_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = SPL06_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init ( SPL06_CS_GPIO, &GPIO_InitStructure );

    GPIO_SetBits ( SPL06_CS_GPIO, SPL06_CS_PIN );
}

/**********************************************************************************************************
*函 数 名: SPL0601_SingleWrite
*功能说明: SPL06传感器单个寄存器写入
*形    参: 寄存器地址 写入值
*返 回 值: 无
**********************************************************************************************************/
static void SPL0601_SingleWrite(uint8_t reg, uint8_t value)
{
	SPL0601_CS(1);
	Spi2_ReadWriteByte(reg);
	Spi2_ReadWriteByte(value);
	SPL0601_CS(0);
}

/**********************************************************************************************************
*函 数 名: SPL0601_MultiRead
*功能说明: SPL06多个寄存器读出
*形    参: 寄存器地址 读出缓冲区 读出长度
*返 回 值: 无
**********************************************************************************************************/
static void SPL0601_MultiRead(uint8_t reg, uint8_t *data, uint8_t length)
{
	SPL0601_CS(1);
	Spi2_ReadWriteByte(reg | 0x80);
	Spi2_Receive(data, length);
	SPL0601_CS(0);
}

/**********************************************************************************************************
*函 数 名: SPL0601_RateSet
*功能说明: 设置气压传感器或温度传感器的采样频率和过采样率
*形    参: uint8 iSensor	  0:Pressure    1:Temperature
		   uint8 u8SmplRate   采样频率(Hz)	Maximal	= 128	
		   uint8 u80overSmpl  过采样率		Maximal = 128
*返 回 值: 无
**********************************************************************************************************/
static void SPL0601_RateSet(u8 iSensor, u8 u8SmplRate, u8 u8OverSmpl)
{
	u8 val = 0;
	int32_t i32kPkT = 0;
	switch (u8SmplRate)
	{
		case 2:
			val |= ( 1 << 4 );
        break;
		case 4:
			val |= ( 2 << 4 );
        break;
		case 8:
			val |= ( 3 << 4 );
        break;
		case 16:
			val |= ( 4 << 4 );
        break;
		case 32:
			val |= ( 5 << 4 );
        break;
		case 64:
			val |= ( 6 << 4 );
        break;
		case 128:
			val |= ( 7 << 4 );
        break;
		case 1:
		default:
        break;
	}
	switch (u8OverSmpl)
	{
		case 2:
			val |= 1;
			i32kPkT = 1572864;
        break;
		case 4:
			val |= 2;
			i32kPkT = 3670016;
        break;
		case 8:
			val |= 3;
			i32kPkT = 7864320;
        break;
		case 16:
			i32kPkT = 253952;
			val |= 4;
        break;
		case 32:
			i32kPkT = 516096;
			val |= 5;
        break;
		case 64:
			i32kPkT = 1040384;
			val |= 6;
        break;
		case 128:
			i32kPkT = 2088960;
			val |= 7;
        break;
		case 1:
		default:
			i32kPkT = 524288;
        break;
	}
	
	if(iSensor == 0)
	{
		spl0601.i32kP = i32kPkT;
		SPL0601_SingleWrite(SPL_RA_PRS_CFG, val);
		if(u8OverSmpl>8)
		{
			SPL0601_MultiRead(SPL_RA_CFG_REG, &val, 1);
			SPL0601_SingleWrite(SPL_RA_CFG_REG, val | 0x04);
		}
	}
	if(iSensor == 1)
	{
		spl0601.i32kT = i32kPkT;
		SPL0601_SingleWrite(SPL_RA_TMP_CFG, val | 0x80);
		if(u8OverSmpl>8)
		{
			SPL0601_MultiRead(SPL_RA_CFG_REG, &val, 1);
			SPL0601_SingleWrite(SPL_RA_CFG_REG, val | 0x08);
		}
	}
}

/**********************************************************************************************************
*函 数 名: SPL0601_GetCalibParam
*功能说明: 从寄存器中读取校准参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void SPL0601_GetCalibParam(void)
{
	uint8_t buffer[18];	
	SPL0601_MultiRead(0x10, buffer, 18);
	
	spl0601.calib_param.c0 = (int16_t)buffer[0]<<4 | buffer[1]>>4;
	spl0601.calib_param.c0 = (spl0601.calib_param.c0 & 0x0800) ? (0xf000 | spl0601.calib_param.c0) : spl0601.calib_param.c0;
	
	spl0601.calib_param.c1 = (int16_t)(buffer[1]&0x0f) << 8 | buffer[2];
	spl0601.calib_param.c1 = ( spl0601.calib_param.c1 & 0x0800 ) ? ( 0xF000 | spl0601.calib_param.c1 ) : spl0601.calib_param.c1;
	
	uint8_t h,m,l;
	SPL0601_MultiRead(0x13, &h, 1);
	SPL0601_MultiRead(0x14, &m, 1);
	SPL0601_MultiRead(0x15, &l, 1);
	spl0601.calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
	spl0601.calib_param.c00 = (int32_t)buffer[3]<<12 | (int32_t)buffer[4]<<4 | (int32_t)buffer[5]>>4;
	spl0601.calib_param.c00 = ( spl0601.calib_param.c00 & 0x080000 ) ? ( 0xFFF00000 | spl0601.calib_param.c00 ) : spl0601.calib_param.c00;

	spl0601.calib_param.c10 = (int32_t)buffer[5]<<16 | (int32_t)buffer[6]<<8 | buffer[7];
	spl0601.calib_param.c10 = ( spl0601.calib_param.c10 & 0x080000 ) ? ( 0xFFF00000 | spl0601.calib_param.c10 ) : spl0601.calib_param.c10;
	
	spl0601.calib_param.c01 = (int16_t)buffer[8]<<8 | buffer[9];
	spl0601.calib_param.c11 = (int16_t)buffer[10]<<8 | buffer[11];
	spl0601.calib_param.c20 = (int16_t)buffer[12]<<8 | buffer[13];
	spl0601.calib_param.c21 = (int16_t)buffer[14]<<8 | buffer[15];
	spl0601.calib_param.c30 = (int16_t)buffer[16]<<8 | buffer[17];
}

/**********************************************************************************************************
*函 数 名: SPL0601_Init
*功能说明: SPL0601配置初始化
*形    参: 无
*返 回 值: 是否初始化成功
**********************************************************************************************************/
u8 SPL0601_Init(void)
{
	spl0601.i32rawPressure = 0;
	spl0601.i32rawTemperature = 0;
	
	u8 tmp;
	SPL0601_MultiRead(SPL_RA_ID, &tmp, 1);
	spl0601.chip_id = tmp;
	
	SPL0601_GetCalibParam();
	
	SPL0601_RateSet(PRESSURE_SENSOR, 128, 16);
	
	SPL0601_RateSet(TEMPERATURE_SENSOR, 8, 8);
	
	SPL0601_SingleWrite(SPL_RA_MEAS_CFG, 7);	//Continuous pressure and temperature measurement mode
	
	if(spl0601.chip_id == 0x10)
		return 1;
	else 
		return 0;
}

/**********************************************************************************************************
*函 数 名: SPL0601_UpdateTemp
*功能说明: 获取温度的原始值，并转化为32Bits整型
*形    参: void
*返 回 值: 无
**********************************************************************************************************/
static void SPL0601_UpdateTemp(void)
{
	u8 buffer[3] = {0};
	SPL0601_MultiRead(SPL_RA_TMP_B2, buffer, 3);
	
	spl0601.i32rawTemperature = ( int32_t )buffer[0]<<16 | (int32_t)buffer[1]<<8 | (int32_t)buffer[2];
	spl0601.i32rawTemperature = ( spl0601.i32rawTemperature & 0x800000 ) ? ( 0xFF000000 | spl0601.i32rawTemperature ) : spl0601.i32rawTemperature;
}

/**********************************************************************************************************
*函 数 名: SPL0601_UpdatePres
*功能说明: 获取压力的原始值，并转化为32Bits整型
*形    参: void
*返 回 值: 无
**********************************************************************************************************/
static void SPL0601_UpdatePres(void)
{
	u8 buffer[3] = {0};
	SPL0601_MultiRead(SPL_RA_PSR_B2, buffer, 3);
	
	spl0601.i32rawPressure = (int32_t)buffer[0]<<16 | (int32_t)buffer[1]<<8 | (int32_t)buffer[2];
	spl0601.i32rawPressure = ( spl0601.i32rawPressure & 0x800000 ) ? ( 0xFF000000 | spl0601.i32rawPressure ) : spl0601.i32rawPressure;
}

/**********************************************************************************************************
*函 数 名: SPL0601_Update
*功能说明: SPL0601更新原始气压数据和温度数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SPL0601_Update(void)
{
	SPL0601_UpdateTemp();
	SPL0601_UpdatePres();
}

/**********************************************************************************************************
*函 数 名: SPL0601_ReadPres
*功能说明: 在获得原始压力数据的基础上，结合温度数据返回浮点校准后的压力值
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void SPL0601_ReadPres(float *pres)
{
	float fTsc, fPsc;
	float qua2, qua3;
	
	fTsc = spl0601.i32rawTemperature / (float)spl0601.i32kT;
	fPsc = spl0601.i32rawPressure / (float)spl0601.i32kP;
	qua2 = spl0601.calib_param.c10 + fPsc * (spl0601.calib_param.c20+fPsc*spl0601.calib_param.c30);
	qua3 = fTsc*fPsc*(spl0601.calib_param.c11 + fPsc*spl0601.calib_param.c21);
	
	*pres = spl0601.calib_param.c00 + fPsc*qua2 + fTsc*spl0601.calib_param.c01 +qua3;
}

/**********************************************************************************************************
*函 数 名: SPL0601_ReadTemp
*功能说明: 在原始温度数据的基础上，返回浮点校准后的温度值
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void SPL0601_ReadTemp(float *temp)
{
	float fTsc;
	
	fTsc = spl0601.i32rawTemperature / (float)spl0601.i32kT;
	*temp = spl0601.calib_param.c0*0.5 + spl0601.calib_param.c1*fTsc;
}
