/**********************************************************************************************************
 * @文件     parameter.c
 * @说明     飞控参数
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.8
**********************************************************************************************************/
#include "parameter.h"
#include "w25qxx.h"
#include "drv_usart.h"
#include "mathTool.h"
#include "string.h"



union Parameter_u Param;

//参数写入计数器
static uint16_t param_save_cnt = 0;

/**********************************************************************************************************
*函 数 名: ParamInit
*功能说明: 参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamInit(void)
{
    ParamReadFromFlash();
	
	//TODO: 设置参数读取后的相应状态
}

/**********************************************************************************************************
*函 数 名: ParamDataReset
*功能说明: 参数恢复默认
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ParamDataReset(void)
{
	//陀螺仪校准参数
	Param.data[PARAM_GYRO_OFFSET_X] = 0.0;
	Param.data[PARAM_GYRO_OFFSET_Y] = 0.0;
	Param.data[PARAM_GYRO_OFFSET_Z] = 0.0;
	Param.data[PARAM_GYRO_SCALE_X] = 1.0;
	Param.data[PARAM_GYRO_SCALE_Y] = 1.0;
	Param.data[PARAM_GYRO_SCALE_Z] = 1.0;
	//加速度校准参数
	Param.data[PARAM_ACC_OFFSET_X] = 0.0;
	Param.data[PARAM_ACC_OFFSET_Y] = 0.0;
	Param.data[PARAM_ACC_OFFSET_Z] = 0.0;
	Param.data[PARAM_ACC_SCALE_X] = 1.0;
	Param.data[PARAM_ACC_SCALE_Y] = 1.0;
	Param.data[PARAM_ACC_SCALE_Z] = 1.0;
	//磁力计校准参数
	Param.data[PARAM_MAG_OFFSET_X] = 0.0;
	Param.data[PARAM_MAG_OFFSET_Y] = 0.0;
	Param.data[PARAM_MAG_OFFSET_Z] = 0.0;
	Param.data[PARAM_MAG_SCALE_X] = 1.0;
	Param.data[PARAM_MAG_SCALE_Y] = 1.0;
	Param.data[PARAM_MAG_SCALE_Z] = 1.0;
	Param.data[PARAM_MAG_EARTH_MAG] = 0.4;
	//水平校准参数
	Param.data[PARAM_IMU_LEVEL_X] = 0.0;
	Param.data[PARAM_IMU_LEVEL_Y] = 0.0;
	Param.data[PARAM_IMU_LEVEL_Z] = 0.0;
}

/**********************************************************************************************************
*函 数 名: ParamBufferReset
*功能说明: 参数存储buffer恢复默认
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamBufferReset(void)
{	
	//参数恢复默认值
	ParamDataReset();
	
	param_save_cnt = 1;
}

/**********************************************************************************************************
*函 数 名: ParamReadFromFlash
*功能说明: 把飞控参数存储区的内容读取出来
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamReadFromFlash(void)
{
	float dataSum=0, checkNum=0, checkSum=0;
	
	W25QXX_PageRead(Param.buffer, PARAM_START_ADDR, PARAM_NUM*4);	//读取指定长度的字节
	
	ParamGetData(PARAM_CHECK_NUM, &checkNum, 4);
	checkNum = ConstrainInt32(checkNum, 0, PARAM_NUM);
	ParamGetData(PARAM_CHECK_SUM, &checkSum, 4);
	
	//计算参数和
	for(u8 i=8; i<(u8)(checkNum*4); i++)
    {
        dataSum += (float)Param.buffer[i];
    }
	
	//和保存的校验和进行对比，如果不符合则重置所有参数
    if(checkSum != dataSum)
    {
        ParamBufferReset();
    }	
	
	printf("Parameters readed from flash");
}

/**********************************************************************************************************
*函 数 名: ParamSaveToFlash
*功能说明: 把飞控参数写入存储区 运行频率20Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
u8 data[PARAM_NUM*4];
void ParamSaveToFlash(void)
{
	uint32_t i=0;
	float dataSum = 0.0;
	float dataNum = 0.0;
	
	//TODO:解锁后不得保存参数
	
	if(param_save_cnt == 1)
	{
		//保存参数数量
		dataNum = PARAM_NUM;
		memcpy(Param.buffer+PARAM_CHECK_NUM*4, &dataNum, 4);
//		u8 tmpByte = Param.buffer[0];
//		Param.buffer[0] = Param.buffer[3];
//		Param.buffer[3] = tmpByte;
//		tmpByte = Param.buffer[1];
//		Param.buffer[1] = Param.buffer[2];
//		Param.buffer[2] = tmpByte;
		
		//计算参数和并保存
		for(i=8; i<PARAM_NUM*4; i++)
		{
			dataSum += (float)Param.buffer[i];
		}
		memcpy(Param.buffer+PARAM_CHECK_SUM*4, &dataSum, 4);
		W25QXX_SectorErase(PARAM_START_ADDR);
		W25QXX_PageRead(data, PARAM_START_ADDR, PARAM_NUM*4);
		W25QXX_PageWrite(Param.buffer, PARAM_START_ADDR, PARAM_NUM*4);
		W25QXX_PageRead(data, PARAM_START_ADDR, PARAM_NUM*4);
		printf("Parameters write to flash");
	}
	
	if(param_save_cnt > 0)
		param_save_cnt--;
}

/**********************************************************************************************************
*函 数 名: ParamUpdateData
*功能说明: 更新飞控参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamUpdateData(uint16_t dataNum, const void *data)
{
    memcpy(Param.buffer+dataNum*4, data, 4);
    //参数更新的3秒后刷新一次Flash
    param_save_cnt = 60;
}

/**********************************************************************************************************
*函 数 名: ParamGetData
*功能说明: 获取飞控参数
*形    参: dataNum: 参数列表里的第几个参数
*          void data: pointer to the buffer that receives the data，使用void可以避免类型冲突
*		   length: 该类数据所占据的字节数目
*返 回 值: 无
**********************************************************************************************************/
void ParamGetData(uint16_t dataNum, void *data, uint8_t length)
{
	memcpy(data, Param.buffer+dataNum*4, length);
}

