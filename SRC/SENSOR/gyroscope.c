/**********************************************************************************************************
 * @文件     gyroscope.c
 * @说明     陀螺仪校准及数据预处理
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "gyroscope.h"
#include "parameter.h"
#include "accelerometer.h"
#include "flightStatus.h"
#include "drv_usart.h"

#include "mathConfig.h"

GYROSCOPE_t _gyro;


/**********************************************************************************************************
*函 数 名: GyroPreTreatInit
*功能说明: 陀螺仪预处理初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GyroPreTreatInit(void)
{
	ParamGetData(PARAM_GYRO_OFFSET_X, &_gyro.cali.offset.x, 4);
	ParamGetData(PARAM_GYRO_OFFSET_Y, &_gyro.cali.offset.y, 4);
	ParamGetData(PARAM_GYRO_OFFSET_Z, &_gyro.cali.offset.z, 4);
	ParamGetData(PARAM_GYRO_SCALE_X, &_gyro.cali.scale.x, 4);
    ParamGetData(PARAM_GYRO_SCALE_Y, &_gyro.cali.scale.y, 4);
    ParamGetData(PARAM_GYRO_SCALE_Z, &_gyro.cali.scale.z, 4);
	
	if(isnan(_gyro.cali.offset.x) || isnan(_gyro.cali.offset.y) || isnan(_gyro.cali.offset.z))
	{
		_gyro.cali.offset.x = 0;
		_gyro.cali.offset.y = 0;
		_gyro.cali.offset.z = 0;
	}
	
	if(abs(_gyro.cali.scale.x - 1) > 0.1f || abs(_gyro.cali.scale.y - 1) > 0.1f || abs(_gyro.cali.scale.z - 1) > 0.1f)
    {
        _gyro.cali.scale.x = 1;
        _gyro.cali.scale.y = 1;
        _gyro.cali.scale.z = 1;
    }
	
	//陀螺仪低通滤波系数计算
	LowPassFilter2ndFactorCal(0.001, GYRO_LPF_CUT, &_gyro.lpf_2nd);
}

/**********************************************************************************************************
*函 数 名: GyroDataPreTreat
*功能说明: 陀螺仪数据预处理
*形    参: 陀螺仪原始数据 陀螺仪预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void GyroDataPreTreat(Vector3f_t gyroRaw, float temperature, Vector3f_t* gyroData, Vector3f_t* gyroLpfData)
{
	_gyro.data = gyroRaw;
	
	//获取温度值
	_gyro.temperature = temperature;
	
	//零偏误差校准
	_gyro.data.x = (_gyro.data.x - _gyro.cali.offset.x) * _gyro.cali.scale.x;
	_gyro.data.y = (_gyro.data.y - _gyro.cali.offset.y) * _gyro.cali.scale.y;
	_gyro.data.z = (_gyro.data.z - _gyro.cali.offset.z) * _gyro.cali.scale.z;
	
	//安装误差校准
	_gyro.data = VectorRotateToBodyFrame(_gyro.data, GetLevelCalibraData());
	
	//低通滤波
	_gyro.dataLpf = LowPassFilter2nd(&_gyro.lpf_2nd, _gyro.data);
	
	*gyroData = _gyro.data;
	*gyroLpfData = _gyro.dataLpf;
}

/**********************************************************************************************************
*函 数 名: GyroCalibration
*功能说明: 陀螺仪校准
*形    参: 陀螺仪原始数据
*返 回 值: 无
**********************************************************************************************************/
void GyroCalibration(Vector3f_t gyroRaw)
{
	const int16_t CALIBRATING_GYRO_CYCLES = 1000;
	static float gyro_sum[3] = {0, 0, 0};
	Vector3f_t gyro_cali_temp, gyro_raw_temp;
	static int16_t count = 0;
	static int8_t staticFlag;
	
	if(!_gyro.cali.should_cali)
		return;
	
	gyro_raw_temp = gyroRaw;
	
	gyro_sum[0] += gyro_raw_temp.x;
	gyro_sum[1] += gyro_raw_temp.y;
	gyro_sum[2] += gyro_raw_temp.z;
	count++;
	
	//mavlink发送校准进度
	
	_gyro.cali.step = 1;
	
	//陀螺仪校准过程中如果检测到飞机不是静止状态则认为校准失败
	if(GetPlaceStatus() != STATIC)
	{
		staticFlag = 1;
	}
	
	if(count == CALIBRATING_GYRO_CYCLES)
	{
		count = 0;
		_gyro.cali.step = 2;
		
		gyro_cali_temp.x = gyro_sum[0] / CALIBRATING_GYRO_CYCLES;
		gyro_cali_temp.y = gyro_sum[1] / CALIBRATING_GYRO_CYCLES;
		gyro_cali_temp.z = gyro_sum[2] / CALIBRATING_GYRO_CYCLES;
		gyro_sum[0] = 0;
		gyro_sum[1] = 0;
		gyro_sum[2] = 0;
		
		//检测校准数据是否有效
		if((abs(gyro_raw_temp.x - gyro_cali_temp.x) + abs(gyro_raw_temp.x - gyro_cali_temp.x)
                + abs(gyro_raw_temp.x - gyro_cali_temp.x)) < 0.6f && !staticFlag)
		{
			_gyro.cali.success = 1;
		}
		else
		{
			_gyro.cali.success = 0;
		}
		
		if(_gyro.cali.success)
		{
			_gyro.cali.offset.x = gyro_cali_temp.x;
			_gyro.cali.offset.y = gyro_cali_temp.y;
			_gyro.cali.offset.z = gyro_cali_temp.z;
			
			//保存陀螺仪校准参数
            ParamUpdateData(PARAM_GYRO_OFFSET_X, &_gyro.cali.offset.x);
            ParamUpdateData(PARAM_GYRO_OFFSET_Y, &_gyro.cali.offset.y);
            ParamUpdateData(PARAM_GYRO_OFFSET_Z, &_gyro.cali.offset.z);
            ParamUpdateData(PARAM_GYRO_SCALE_X, &_gyro.cali.scale.x);
            ParamUpdateData(PARAM_GYRO_SCALE_Y, &_gyro.cali.scale.y);
            ParamUpdateData(PARAM_GYRO_SCALE_Z, &_gyro.cali.scale.z);
			//TODO:更新mavlink参数
			//TODO:mavlink发送校准结果
			
			printf("Gyroscope calibrated successful.");
		}
		else
		{
			//mavlink发送校准结果
			
			printf("Gyroscope calibreated failed.");
		}
		
		staticFlag = 0;
		
		//bstlink发送校准结果
		
		_gyro.cali.should_cali = 0;
		_gyro.cali.step = 0;
		
		SetCaliStatus(NoCali);
	}
}

/**********************************************************************************************************
*函 数 名: GetGyroCaliStatus
*功能说明: 陀螺仪校准状态
*形    参: 无
*返 回 值: 状态 0：不在校准中 非0：校准中
**********************************************************************************************************/
uint8_t GetGyroCaliStatus(void)
{
    return _gyro.cali.step;
}

/**********************************************************************************************************
*函 数 名: GyroCalibrateEnable
*功能说明: 陀螺仪校准使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GyroCalibrateEnable(void)
{
    _gyro.cali.should_cali = 1;
	SetCaliStatus(GyroCali);
}

/**********************************************************************************************************
*函 数 名: GyroGetData
*功能说明: 获取经过处理后的陀螺仪数据
*形    参: 无
*返 回 值: 角速度
**********************************************************************************************************/
Vector3f_t GyroGetData(void)
{
    return _gyro.data;
}

/**********************************************************************************************************
*函 数 名: GyroLpfGetData
*功能说明: 获取低通滤波后的陀螺仪数据
*形    参: 无
*返 回 值: 角速度
**********************************************************************************************************/
Vector3f_t GyroLpfGetData(void)
{
    return _gyro.dataLpf;
}

/**********************************************************************************************************
*函 数 名: GyroGetTemp
*功能说明: 获取陀螺仪温度
*形    参: 无
*返 回 值: 温度值
**********************************************************************************************************/
float GyroGetTemp(void)
{
    return _gyro.temperature;
}

/**********************************************************************************************************
*函 数 名: GetGyroOffsetCaliData
*功能说明: 获取陀螺仪零偏校准数据
*形    参: 无
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetGyroOffsetCaliData(void)
{
    return _gyro.cali.offset;
}
