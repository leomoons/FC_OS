/**********************************************************************************************************
 * @文件     gyroscope.c
 * @说明     陀螺仪校准及数据预处理
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.08
**********************************************************************************************************/
#include "icm20602.h"
#include "boardConfig.h"


// Register address
#define MPU_RA_XG_OFFS_TC       	0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       	0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       	0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      	0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      	0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      	0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        	0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     	0x07
#define MPU_RA_YA_OFFS_H        	0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     	0x09
#define MPU_RA_ZA_OFFS_H       	 	0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     	0x0B
#define MPU_RA_PRODUCT_ID      		0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH    	 	0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     	0x14
#define MPU_RA_YG_OFFS_USRH     	0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     	0x16
#define MPU_RA_ZG_OFFS_USRH     	0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     	0x18
#define MPU_RA_SMPLRT_DIV       	0x19
#define MPU_RA_CONFIG           	0x1A
#define MPU_RA_GYRO_CONFIG      	0x1B
#define MPU_RA_ACCEL_CONFIG     	0x1C
#define MPU_RA_ACCEL_CONFIG2    	0x1D
#define MPU_RA_FF_DUR           	0x1E
#define MPU_RA_MOT_THR          	0x1F
#define MPU_RA_MOT_DUR          	0x20
#define MPU_RA_ZRMOT_THR        	0x21
#define MPU_RA_ZRMOT_DUR        	0x22
#define MPU_RA_FIFO_EN          	0x23
#define MPU_RA_I2C_MST_CTRL     	0x24
#define MPU_RA_I2C_SLV0_ADDR    	0x25
#define MPU_RA_I2C_SLV0_REG     	0x26
#define MPU_RA_I2C_SLV0_CTRL    	0x27
#define MPU_RA_I2C_SLV1_ADDR    	0x28
#define MPU_RA_I2C_SLV1_REG     	0x29
#define MPU_RA_I2C_SLV1_CTRL    	0x2A
#define MPU_RA_I2C_SLV2_ADDR    	0x2B
#define MPU_RA_I2C_SLV2_REG     	0x2C
#define MPU_RA_I2C_SLV2_CTRL    	0x2D
#define MPU_RA_I2C_SLV3_ADDR    	0x2E
#define MPU_RA_I2C_SLV3_REG     	0x2F
#define MPU_RA_I2C_SLV3_CTRL    	0x30
#define MPU_RA_I2C_SLV4_ADDR    	0x31
#define MPU_RA_I2C_SLV4_REG     	0x32
#define MPU_RA_I2C_SLV4_DO      	0x33
#define MPU_RA_I2C_SLV4_CTRL    	0x34
#define MPU_RA_I2C_SLV4_DI      	0x35
#define MPU_RA_I2C_MST_STATUS   	0x36
#define MPU_RA_INT_PIN_CFG      	0x37
#define MPU_RA_INT_ENABLE       	0x38
#define MPU_RA_DMP_INT_STATUS   	0x39
#define MPU_RA_INT_STATUS       	0x3A
#define MPU_RA_ACCEL_XOUT_H     	0x3B
#define MPU_RA_ACCEL_XOUT_L     	0x3C
#define MPU_RA_ACCEL_YOUT_H     	0x3D
#define MPU_RA_ACCEL_YOUT_L     	0x3E
#define MPU_RA_ACCEL_ZOUT_H     	0x3F
#define MPU_RA_ACCEL_ZOUT_L     	0x40
#define MPU_RA_TEMP_OUT_H       	0x41
#define MPU_RA_TEMP_OUT_L       	0x42
#define MPU_RA_GYRO_XOUT_H      	0x43
#define MPU_RA_GYRO_XOUT_L      	0x44
#define MPU_RA_GYRO_YOUT_H      	0x45
#define MPU_RA_GYRO_YOUT_L      	0x46
#define MPU_RA_GYRO_ZOUT_H      	0x47
#define MPU_RA_GYRO_ZOUT_L      	0x48
#define MPU_RA_EXT_SENS_DATA_00 	0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      	0x63
#define MPU_RA_I2C_SLV1_DO      	0x64
#define MPU_RA_I2C_SLV2_DO      	0x65
#define MPU_RA_I2C_SLV3_DO      	0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        	0x6A
#define MPU_RA_PWR_MGMT_1       	0x6B
#define MPU_RA_PWR_MGMT_2       	0x6C
#define MPU_RA_BANK_SEL         	0x6D
#define MPU_RA_MEM_START_ADDR   	0x6E
#define MPU_RA_MEM_R_W          	0x6F
#define MPU_RA_DMP_CFG_1        	0x70
#define MPU_RA_DMP_CFG_2        	0x71
#define MPU_RA_FIFO_COUNTH      	0x72
#define MPU_RA_FIFO_COUNTL      	0x73
#define MPU_RA_FIFO_R_W         	0x74
#define MPU_RA_WHO_AM_I         	0x75

// 低通滤波频率选择宏
#define ICM20602_LPF_250HZ       0
#define ICM20602_LPF_176HZ       1
#define ICM20602_LPF_92HZ        2
#define ICM20602_LPF_41HZ        3
#define ICM20602_LPF_20HZ        4
#define ICM20602_LPF_10HZ        5
#define ICM20602_LPF_5HZ         6
#define ICM20602_LPF_3281HZ      7

//不同测量范围的最小有效位(一共16位有效表示)
#define MPU_A_2mg                ((float)0.00006103f)  //g/LSB
#define MPU_A_4mg                ((float)0.00012207f)  //g/LSB
#define MPU_A_8mg                ((float)0.00024414f)  //g/LSB
#define MPU_A_16mg				 ((float)0.00048828f)  //g/LSB	

#define MPU_G_s250dps            ((float)0.0076296f)  //dps/LSB		degree per second
#define MPU_G_s500dps            ((float)0.0152592f)  //dps/LSB
#define MPU_G_s1000dps           ((float)0.0305185f)  //dps/LSB
#define MPU_G_s2000dps           ((float)0.0610370f)  //dps/LSB


Vector3i_t accRaw;
Vector3i_t gyroRaw;
int16_t tempRaw;

/**********************************************************************************************************
*函 数 名: ICM20602_CS
*功能说明: ICM20602传感器CS脚使能或失能
*形    参: 1代表使能，0代表失能
*返 回 值: 无
**********************************************************************************************************/
static void ICM20602_CS(u8 ena)
{
	if(ena)
		GPIO_ResetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
	else
		GPIO_SetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: ICM20602CSPin_Init
*功能说明: ICM20602 CS使能引脚初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ICM20602CSPin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(ICM20602_CS_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = ICM20602_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ICM20602_CS_GPIO, &GPIO_InitStructure);
	
	GPIO_SetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: ICM20602_SingleWrite
*功能说明: ICM传感器单个寄存器写入
*形    参: 寄存器地址 写入值
*返 回 值: 无
**********************************************************************************************************/
static void ICM20602_SingleWrite(uint8_t reg, uint8_t value)
{
	ICM20602_CS(1);
	Spi2_ReadWriteByte(reg);
	Spi2_ReadWriteByte(value);
	ICM20602_CS(0);
}

/**********************************************************************************************************
*函 数 名: ICM20602_MultiRead
*功能说明: ICM20602多个寄存器读出
*形    参: 寄存器地址 读出缓冲区 读出长度
*返 回 值: 无
**********************************************************************************************************/
static void ICM20602_MultiRead(uint8_t reg, uint8_t *data, uint8_t length)
{
	ICM20602_CS(1);
	Spi2_ReadWriteByte(reg | 0x80);
	Spi2_Receive(data, length);
	ICM20602_CS(0);
}

/**********************************************************************************************************
*函 数 名: ICM20602_Init
*功能说明: ICM20602 寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ICM20602_Init(void)
{
	// Device reset
	ICM20602_SingleWrite(MPU_RA_PWR_MGMT_1, 0x80);		
	DelayXms(10);
	// Auto select the best available clock source-PLL if ready
	ICM20602_SingleWrite(MPU_RA_PWR_MGMT_1, 0x01);
	DelayXms(10);
	
	// Verify the identity of the device
//	u8 who_am_i;
//	ICM20602_MultiRead(MPU_RA_WHO_AM_I, &who_am_i, 1);
//	if(who_am_i != 0x12)		//ICM20602的默认设备号为0x12
//		return 0;

	// reset accelerator and temperaturer digital signal path
	ICM20602_SingleWrite(MPU_RA_SIGNAL_PATH_RESET, 0x03);
	DelayXms(10);
	// reset all gyro digital signal path, accel digital signal path, and temp digital signal path
	ICM20602_SingleWrite(MPU_RA_USER_CTRL, 0x01);
	DelayXms(10);
	
	// Disable I2C slave module and put the serial interface in SPI mode only
	ICM20602_SingleWrite(MPU_RA_DMP_CFG_1, 0x40);
	DelayXms(10);
	
	// Turn on X,Y,Z accelerometer, X,Y,Z gyroscope
	ICM20602_SingleWrite(MPU_RA_PWR_MGMT_2, 0x00);
	DelayXms(10);
	
	// sampling frequency 0x00(1000H), SAMPLE_RATE = INTERNAL_SAMPLE_RATE/(1+SMPLRT_DIV)
	ICM20602_SingleWrite(MPU_RA_SMPLRT_DIV, (1000/1000-1));
	DelayXms(10);
	
	//低通滤波频率
	ICM20602_SingleWrite(MPU_RA_CONFIG, ICM20602_LPF_20HZ);
	DelayXms(10);
	
	//陀螺仪自检及测量范围，典型值0x18(不自检，2000dps) (0x10 1000dps) (0x08 500dps)
	ICM20602_SingleWrite(MPU_RA_GYRO_CONFIG, (3 << 3));
	DelayXms(10);
	
	//加速度计自检及测量范围典型值0x18(不自检， +-16G)
	ICM20602_SingleWrite(MPU_RA_ACCEL_CONFIG, (3 << 3));
	DelayXms(10);
	
	//加速度计LPF 20Hz
	ICM20602_SingleWrite(MPU_RA_ACCEL_CONFIG2, ICM20602_LPF_20HZ);
	DelayXms(10);
	
	//关闭低功耗
	ICM20602_SingleWrite(MPU_RA_FF_DUR, 0x00);
	DelayXms(10);
	
	//关闭FIFO
	ICM20602_SingleWrite(MPU_RA_FIFO_EN, 0x00);
	DelayXms(10);

}

/**********************************************************************************************************
*函 数 名: ICM20602_UpdateAcc
*功能说明: ICM20602更新加速度传感器原始数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//uint8_t ACCbuffer[6];
void ICM20602_UpdateAcc(void)
{
	uint8_t ACCbuffer[6];
	
	ICM20602_MultiRead(MPU_RA_ACCEL_XOUT_H, ACCbuffer, 6);//连续接受6个ICM20602寄存器的数据
	accRaw.x = (s16)((((u16)ACCbuffer[0]) << 8) | ACCbuffer[1]);
	accRaw.y = (s16)((((u16)ACCbuffer[2]) << 8) | ACCbuffer[3]);
	accRaw.z = (s16)((((u16)ACCbuffer[4]) << 8) | ACCbuffer[5]);
	
	DelayUs(1);
}

/**********************************************************************************************************
*函 数 名: ICM20602_UpdateGyro
*功能说明: ICM20602更新陀螺仪传感器原始数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//uint8_t GYRObuffer[6];
void ICM20602_UpdateGyro(void)
{
	uint8_t GYRObuffer[6];
	
	
	ICM20602_MultiRead(MPU_RA_GYRO_XOUT_H, GYRObuffer, 6);
	gyroRaw.x = (s16)((((u16)GYRObuffer[0]) << 8) | GYRObuffer[1]);
    gyroRaw.y = (s16)((((u16)GYRObuffer[2]) << 8) | GYRObuffer[3]);
    gyroRaw.z = (s16)((((u16)GYRObuffer[4]) << 8) | GYRObuffer[5]);
	
	DelayUs(1);
}

/**********************************************************************************************************
*函 数 名: ICM20602_UpdateTemp
*功能说明: ICM20602更新温度传感器原始数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//uint8_t TEMPbuffer[2];
void ICM20602_UpdateTemp(void)
{
	uint8_t TEMPbuffer[2];
    
	ICM20602_MultiRead(MPU_RA_TEMP_OUT_H, TEMPbuffer, 2);
	tempRaw = ((((int16_t)TEMPbuffer[0]) << 8) | TEMPbuffer[1]);
	DelayUs(1);
}

/**********************************************************************************************************
*函 数 名: ICM20602_ReadAcc
*功能说明: 读取ICM20602加速度原始数据，并转化为单位重力加速度g（归一化）
*形    参: 加速度向量指针
*返 回 值: 无
**********************************************************************************************************/
void ICM20602_ReadAcc(Vector3f_t* acc)
{
	//转化为右手系，单位转化为单位重力加速度
    acc->x = (float)accRaw.x * MPU_A_16mg;
    acc->y = (float)accRaw.y * MPU_A_16mg;
    acc->z =-(float)accRaw.z * MPU_A_16mg;
//	acc->x = (float)accRaw.x;
//  acc->y = (float)accRaw.y;
//  acc->z = (float)accRaw.z;
	
}

/**********************************************************************************************************
*函 数 名: ICM20602_ReadGyro
*功能说明: 读取ICM20602陀螺仪原始数据，并转化单位为rad/s
*形    参: 角速度向量指针
*返 回 值: 无
**********************************************************************************************************/
void ICM20602_ReadGyro(Vector3f_t* gyro)
{	
	//转化为右手系，单位转化为rad/s
    gyro->x = (float)gyroRaw.x * MPU_G_s2000dps * DEG_TO_RAD;
    gyro->y = (float)gyroRaw.y * MPU_G_s2000dps * DEG_TO_RAD;
    gyro->z = (float)gyroRaw.z * MPU_G_s2000dps * DEG_TO_RAD;
//	gyro->x = (float)gyroRaw.x;
//  gyro->y = (float)gyroRaw.y;
//  gyro->z = (float)gyroRaw.z;

}

/**********************************************************************************************************
*函 数 名: ICM20602_ReadTemp
*功能说明: ICM20602更新温度传感器数据，转化单位为摄氏度
*形    参: 温度变量指针
*返 回 值: 无
**********************************************************************************************************/
void ICM20602_ReadTemp(float* temp)
{
	*temp = 25.0f + (float)tempRaw / 326.8f;
}
