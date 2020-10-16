/**********************************************************************************************************
 * @文件     ak8975.c
 * @说明     电子罗盘驱动，只需要配置CS引脚，传感器内部寄存器的配置可有可无
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.9
**********************************************************************************************************/
#include "ak8975.h"
#include "boardConfig.h"


#define AK8975_WIA_REG          0X00 
#define AK8975_INFO_REG         0X01 
#define AK8975_ST1_REG          0X02 
#define AK8975_HXL_REG          0X03 
#define AK8975_HXH_REG          0X04
#define AK8975_HYL_REG          0X05
#define AK8975_HYH_REG          0X06
#define AK8975_HZL_REG          0X07
#define AK8975_HZH_REG          0X08
#define AK8975_ST2_REG          0X09 
#define AK8975_CNTL_REG         0X0A 
#define AK8975_RSV_REG          0X0B
#define AK8975_ASTC_REG         0X0C 
#define AK8975_TS1_REG          0X0D
#define AK8975_TS2_REG          0X0E
#define AK8975_I2CDIS_REG       0X0F 
#define AK8975_ASAX_REG         0X10 
#define AK8975_ASAY_REG         0X11
#define AK8975_ASAZ_REG         0X12

#define AK8975_MAG_TO_UTESLA 	0.3f	//从采样值到磁感应强度（单位：uT，微特斯拉）

Vector3i_t magRaw;

/**********************************************************************************************************
*函 数 名: AK8975_CS
*功能说明: AK9075传感器CS脚使能或失能
*形    参: 1代表使能，0代表失能
*返 回 值: 无
**********************************************************************************************************/
static void AK8975_CS(u8 ena)
{
	if(ena)
		GPIO_ResetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
	else
		GPIO_SetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: AK8975CSPin_Init
*功能说明: AK8975 CS使能引脚初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AK8975CSPin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(AK8975_CS_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = AK8975_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(AK8975_CS_GPIO, &GPIO_InitStructure);
	
	GPIO_SetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: AK8975_Trig
*功能说明: AK8975 Operation modes transition from Power-down mode to Single measurement modes
磁罗盘在一次采样后会自动进入Power down模式，需要主动进入采样模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void AK8975_Trig(void)
{
	AK8975_CS(1);
	Spi2_ReadWriteByte(AK8975_CNTL_REG);
	Spi2_ReadWriteByte(0x01);
	AK8975_CS(0);
}

/**********************************************************************************************************
*函 数 名: AK8975_Update
*功能说明: AK8975读取磁罗盘数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AK8975_Update(void)
{
	uint8_t MAGbuffer[6];
	AK8975_CS(1);
	Spi2_ReadWriteByte(AK8975_HXL_REG | 0x80);
	for(u8 i=0; i<6; i++)
		MAGbuffer[i] = Spi2_ReadWriteByte(0xff);
	AK8975_CS(0);
	
	//数据类型转换以及转换到WNU（东北天）
	magRaw.x = ((((int16_t)MAGbuffer[1]) << 8) | MAGbuffer[0]);
	magRaw.y = ((((int16_t)MAGbuffer[3]) << 8) | MAGbuffer[2]);
	magRaw.z = ((((int16_t)MAGbuffer[5]) << 8) | MAGbuffer[4]);

	AK8975_Trig();	//转换到采样模式
}

/**********************************************************************************************************
*函 数 名: AK8975_ReadMag
*功能说明: 其他文件获得磁感应强度的接口
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
//Vector3i_t magRAW2;
void AK8975_ReadMag(Vector3f_t* mag)
{
	mag->x = (float)magRaw.x * AK8975_MAG_TO_UTESLA;
	mag->y = (float)magRaw.y * AK8975_MAG_TO_UTESLA;
	mag->z = (float)magRaw.z * AK8975_MAG_TO_UTESLA;
	
//	magRAW2.x = magRaw.x;
//	magRAW2.y = magRaw.y;
//	magRAW2.z = magRaw.z;
}






