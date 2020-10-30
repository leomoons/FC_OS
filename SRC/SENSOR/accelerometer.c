/**********************************************************************************************************
 * @文件     accelerometer.c
 * @说明     加速度校准及数据预处理
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "accelerometer.h"
#include "parameter.h"
#include "drv_usart.h"
#include "flightStatus.h"
#include "mathConfig.h"

ACCELEROMETER_t _acc;


/**********************************************************************************************************
*函 数 名: AccPreTreatInit
*功能说明: 加速度预处理初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AccPreTreatInit(void)
{
	ParamGetData(PARAM_ACC_OFFSET_X, &_acc.cali.offset.x, 4);
    ParamGetData(PARAM_ACC_OFFSET_Y, &_acc.cali.offset.y, 4);
    ParamGetData(PARAM_ACC_OFFSET_Z, &_acc.cali.offset.z, 4);
    ParamGetData(PARAM_ACC_SCALE_X, &_acc.cali.scale.x, 4);
    ParamGetData(PARAM_ACC_SCALE_Y, &_acc.cali.scale.y, 4);
    ParamGetData(PARAM_ACC_SCALE_Z, &_acc.cali.scale.z, 4);

    ParamGetData(PARAM_IMU_LEVEL_X, &_acc.levelCali.scale.x, 4);
    ParamGetData(PARAM_IMU_LEVEL_Y, &_acc.levelCali.scale.y, 4);
    ParamGetData(PARAM_IMU_LEVEL_Z, &_acc.levelCali.scale.z, 4);
	
	if(isnan(_acc.cali.offset.x) || isnan(_acc.cali.offset.y) || isnan(_acc.cali.offset.z) || \
            isnan(_acc.cali.scale.x) || isnan(_acc.cali.scale.y) || isnan(_acc.cali.scale.z) ||    \
            _acc.cali.scale.x == 0 || _acc.cali.scale.y == 0 || _acc.cali.scale.z == 0)
    {
        _acc.cali.offset.x = 0;
        _acc.cali.offset.y = 0;
        _acc.cali.offset.z = 0;
        _acc.cali.scale.x = 1;
        _acc.cali.scale.y = 1;
        _acc.cali.scale.z = 1;
    }
	
	if(abs(_acc.cali.offset.x) > 1 || abs(_acc.cali.offset.y) > 1 || abs(_acc.cali.offset.z) > 1 ||
            abs(_acc.cali.scale.x) > 2 || abs(_acc.cali.scale.y) > 2 || abs(_acc.cali.scale.z) > 2 ||
            abs(_acc.cali.scale.x) < 0.3f || abs(_acc.cali.scale.y) < 0.3f || abs(_acc.cali.scale.z) < 0.3f)
    {
        _acc.cali.offset.x = 0;
        _acc.cali.offset.y = 0;
        _acc.cali.offset.z = 0;
        _acc.cali.scale.x = 1;
        _acc.cali.scale.y = 1;
        _acc.cali.scale.z = 1;
    }
	
	if(isnan(_acc.levelCali.scale.x) || isnan(_acc.levelCali.scale.y) || isnan(_acc.levelCali.scale.z) ||
            abs(_acc.levelCali.scale.x) > 0.2f || abs(_acc.levelCali.scale.y) > 0.2f || abs(_acc.levelCali.scale.z) > 0.2f)
    {
        _acc.levelCali.scale.x = 0;
        _acc.levelCali.scale.y = 0;
        _acc.levelCali.scale.z = 0;
    }
	
	//加速度低通滤波系数计算
	LowPassFilter2ndFactorCal(0.001, ACC_LPF_CUT, &_acc.lpf_2nd);
}

/**********************************************************************************************************
*函 数 名: AccDataPreTreat
*功能说明: 加速度数据预处理
*形    参: 加速度原始数据 加速度预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
float ACCDATAX, ACCDATAY, ACCDATAZ, ACCLPFX, ACCLPFY, ACCLPFZ;
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accPre)
{
	static float lastAccMag, accMagderi;
	const float deltaT = 0.001f;
	
	_acc.data = accRaw;
	
	//加速度数据校准
	_acc.data.x = (_acc.data.x - _acc.cali.offset.x) * _acc.cali.scale.x;
    _acc.data.y = (_acc.data.y - _acc.cali.offset.y) * _acc.cali.scale.y;
    _acc.data.z = (_acc.data.z - _acc.cali.offset.z) * _acc.cali.scale.z;
	
	//水平误差校准
	_acc.data = VectorRotateToBodyFrame(_acc.data, _acc.levelCali.scale);
	ACCDATAX = _acc.data.x;
	ACCDATAY = _acc.data.y;
	ACCDATAZ = _acc.data.z;
	
	//低通滤波
	_acc.dataLpf = LowPassFilter2nd(&_acc.lpf_2nd, _acc.data);
	ACCLPFX = _acc.dataLpf.x;
	ACCLPFY = _acc.dataLpf.y;
	ACCLPFZ = _acc.dataLpf.z;
	
	//计算加速度模值
	_acc.mag = Pythagorous3(_acc.dataLpf.x, _acc.dataLpf.y, _acc.dataLpf.z);
	
	//震动系数计算
	accMagderi = (_acc.mag - lastAccMag) / deltaT;
	lastAccMag = _acc.mag;
	_acc.vibraCoef = _acc.vibraCoef * 0.9995f + abs(accMagderi) * 0.0005f;
	
	*accPre = _acc.data;
}

/**********************************************************************************************************
*函 数 名: AccCalibration
*功能说明: 加速度校准
*形    参: 加速度原始数据
*返 回 值: 无
**********************************************************************************************************/
void AccCalibration(Vector3f_t accRaw)
{
	static uint16_t samples_count = 0;
	static uint8_t orientationCaliFlag[6];
	//static uint8_t currentOrientation;
	static Vector3f_t new_offset;
	static Vector3f_t new_scale;
	static Vector3f_t samples[6];
	static uint8_t caliFlag = 0;
	static uint32_t caliCnt = 0;
	
	if(!_acc.cali.should_cali)
		return;
	
	/**********************检测IMU放置方向***************************/
	if(GetImuOrientation() == ORIENTATION_UP && !orientationCaliFlag[ORIENTATION_UP])
	{
		//判断IMU是否处于静止状态
		if(GetPlaceStatus() == STATIC)
			caliCnt++;
		else
			caliCnt = 0;
		SetCaliStatus(AccCaliDataCollecting);	//绿灯快闪
		
		if(caliCnt > 1000)
		{
			caliFlag = 1;
			orientationCaliFlag[ORIENTATION_UP] = 1;
			samples_count = 0;
			_acc.cali.step++;
			//currentOrientation = ORIENTATION_UP;
			//mavlink发送检测提示
			
			SetCaliStatus(AccCaliOneDataReady);	//绿灯慢闪
		}
	}
	if(GetImuOrientation() == ORIENTATION_DOWN && !orientationCaliFlag[ORIENTATION_DOWN])
	{
		//判断IMU是否处于静止状态
		if(GetPlaceStatus() == STATIC)
			caliCnt++;
		else
			caliCnt = 0;
		SetCaliStatus(AccCaliDataCollecting);	//绿灯快闪
		
		if(caliCnt > 1000)
		{
			caliFlag = 1;
			orientationCaliFlag[ORIENTATION_DOWN] = 1;
			samples_count = 0;
			_acc.cali.step++;
			//currentOrientation = ORIENTATION_DOWN;
			//mavlink发送检测提示
			
			SetCaliStatus(AccCaliOneDataReady);	//绿灯慢闪
		}
	}
	if(GetImuOrientation() == ORIENTATION_FRONT && !orientationCaliFlag[ORIENTATION_FRONT])
	{
		//判断IMU是否处于静止状态
		if(GetPlaceStatus() == STATIC)
			caliCnt++;
		else
			caliCnt = 0;
		SetCaliStatus(AccCaliDataCollecting);	//绿灯快闪
		
		if(caliCnt > 1000)
		{
			caliFlag = 1;
			orientationCaliFlag[ORIENTATION_FRONT] = 1;
			samples_count = 0;
			_acc.cali.step++;
			//currentOrientation = ORIENTATION_FRONT;
			//mavlink发送检测提示
			
			SetCaliStatus(AccCaliOneDataReady);	//绿灯慢闪
		}
	}
	if(GetImuOrientation() == ORIENTATION_BACK && !orientationCaliFlag[ORIENTATION_BACK])
	{
		//判断IMU是否处于静止状态
		if(GetPlaceStatus() == STATIC)
			caliCnt++;
		else
			caliCnt = 0;
		SetCaliStatus(AccCaliDataCollecting);	//绿灯快闪
		
		if(caliCnt > 1000)
		{
			caliFlag = 1;
			orientationCaliFlag[ORIENTATION_BACK] = 1;
			samples_count = 0;
			_acc.cali.step++;
			//currentOrientation = ORIENTATION_BACK;
			//mavlink发送检测提示
			
			SetCaliStatus(AccCaliOneDataReady);	//绿灯慢闪
		}
	}
	if(GetImuOrientation() == ORIENTATION_LEFT && !orientationCaliFlag[ORIENTATION_LEFT])
	{
		//判断IMU是否处于静止状态
		if(GetPlaceStatus() == STATIC)
			caliCnt++;
		else
			caliCnt = 0;
		SetCaliStatus(AccCaliDataCollecting);	//绿灯快闪
		
		if(caliCnt > 1000)
		{
			caliFlag = 1;
			orientationCaliFlag[ORIENTATION_LEFT] = 1;
			samples_count = 0;
			_acc.cali.step++;
			//currentOrientation = ORIENTATION_LEFT;
			//mavlink发送检测提示
			
			SetCaliStatus(AccCaliOneDataReady);	//绿灯慢闪
		}
	}
	if(GetImuOrientation() == ORIENTATION_RIGHT && !orientationCaliFlag[ORIENTATION_RIGHT])
	{
		//判断IMU是否处于静止状态
		if(GetPlaceStatus() == STATIC)
			caliCnt++;
		else
			caliCnt = 0;
		SetCaliStatus(AccCaliDataCollecting);	//绿灯快闪
		
		if(caliCnt > 1000)
		{
			caliFlag = 1;
			orientationCaliFlag[ORIENTATION_RIGHT] = 1;
			samples_count = 0;
			_acc.cali.step++;
			//currentOrientation = ORIENTATION_RIGHT;
			//mavlink发送检测提示
			
			SetCaliStatus(AccCaliOneDataReady);	//绿灯慢闪
		}
	}
	/****************************************************************************************/
	
	//分别采集加速度计六个方向的数据，顺序随意，每个方向取500个样本求平均值
	if(caliFlag)
	{
		if(samples_count < 1000)
		{
			samples[_acc.cali.step-1].x += accRaw.x;
			samples[_acc.cali.step-1].y += accRaw.y;
			samples[_acc.cali.step-1].z += accRaw.z;
			samples_count++;
		}
		else if(samples_count == 1000)
		{
			samples[_acc.cali.step-1].x /= 1000;
			samples[_acc.cali.step-1].y /= 1000;
			samples[_acc.cali.step-1].z /= 1000;
			samples_count++;
			
			caliFlag = 0;
			caliCnt =0;
			
			//bsklink发送当前校准步骤
			
			//mavlink发送当前校准步骤
		}
	}
	
	if(_acc.cali.step == 6 && samples_count ==1001)
	{
		//计算方程解初值
		float initBeta[6];
		initBeta[0] = 0;
		initBeta[1] = 0;
		initBeta[2] = 0;
		initBeta[3] = 1;
		initBeta[4] = 1;
		initBeta[5] = 1;
		
		//LM法求解传感器误差方程最优解
		LevenbergMarquardt(samples, &new_offset, &new_scale, initBeta, 1);
		
		//判断校准参数是否正常
		if(fabsf(new_scale.x-1.0f)>0.1f || fabsf(new_scale.y-1.0f)>0.1f || fabsf(new_scale.z-1.0f)>0.1f)
		{
			_acc.cali.success = false;
		}
		else if(fabsf(new_offset.x) > (1 * 0.35f) || fabsf(new_offset.y) > (1 * 0.35f) || fabsf(new_offset.z) > (1 * 0.6f))
		{
			_acc.cali.success = false;
		}
		else
		{
			_acc.cali.success = true;
		}
		
		//数据采集样本清零
		for(u8 i=0; i<6; i++)
		{
			samples[i].x = 0;
			samples[i].y = 0;
			samples[i].z = 0;
		}
		
		if(_acc.cali.success)
		{
			_acc.cali.offset = new_offset;
			_acc.cali.scale = new_scale;
			
			//保存加速度校准参数
			ParamUpdateData(PARAM_ACC_OFFSET_X, &_acc.cali.offset.x);
			ParamUpdateData(PARAM_ACC_OFFSET_Y, &_acc.cali.offset.y);
			ParamUpdateData(PARAM_ACC_OFFSET_Z, &_acc.cali.offset.z);
            ParamUpdateData(PARAM_ACC_SCALE_X, &_acc.cali.scale.x);
            ParamUpdateData(PARAM_ACC_SCALE_Y, &_acc.cali.scale.y);
            ParamUpdateData(PARAM_ACC_SCALE_Z, &_acc.cali.scale.z);
			//更新mavlink参数
			
			//mavlink发送校准结果
			
			
		}
		else
		{
			//mavlink发送校准失败结果
			
		}
		
		//发送校准结果
		printf("Accelerometer calibration result: %d", _acc.cali.success);
		SetCaliStatus(NoCali);
		
		//加速度传感器校准完成，相关flag reset
		_acc.cali.should_cali = 0;
		_acc.cali.step = 0;
		for(uint8_t i=0; i<6; i++)
			orientationCaliFlag[i] = 0;
	}
}	


/**********************************************************************************************************
*函 数 名: ImuLevelCalibration
*功能说明: IMU传感器的水平校准（安装误差），主要读取静止时的加速度数据并求平均值，得到校准角度值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ImuLevelCalibration(void)
{
	const int16_t CALIBRATING_ACC_LEVEL_CYCLES = 3000;
	static float acc_sum[3] = {0, 0, 0};
	Vector3f_t accAverage;
	Vector3f_t caliTemp;
	static int16_t count = 0;
	
	if(!_acc.levelCali.should_cali)
		return;
	
	if(count == 0)
	{
		_acc.levelCali.scale.x = 0;
		_acc.levelCali.scale.y = 0;
		_acc.levelCali.scale.z = 0;
	}
	else
	{
		acc_sum[0] += _acc.data.x;
		acc_sum[1] += _acc.data.y;
		acc_sum[2] += _acc.data.z;
	}
	count++;
	
	//mavlink发送校准进度
	
	_acc.levelCali.step = 1;
	
	if(count == CALIBRATING_ACC_LEVEL_CYCLES)
	{
		accAverage.x = acc_sum[0] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		accAverage.y = acc_sum[1] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		accAverage.z = acc_sum[2] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		acc_sum[0] = 0;
		acc_sum[1] = 0;
		acc_sum[2] = 0;
		count = 0;
		_acc.levelCali.should_cali = 0;
		_acc.levelCali.step = 2;
		
		//加速度向量转化为姿态角
		AccVectorToRollPitchAngle(&caliTemp, accAverage);
		
		float tmp = SafeArcsin(1.0);
		tmp = atan2f(1.0,1.0);
		
		if(abs(Degrees(caliTemp.x))<10 && abs(Degrees(caliTemp.y))<10)
		{
			_acc.levelCali.success = 1;
			
			_acc.levelCali.scale.x = -caliTemp.x;
			_acc.levelCali.scale.y = -caliTemp.y;
			_acc.levelCali.scale.z = 0;
			
			//保存IMU安装误差校准参数
			ParamUpdateData(PARAM_IMU_LEVEL_X, &_acc.levelCali.scale.x);
			ParamUpdateData(PARAM_IMU_LEVEL_Y, &_acc.levelCali.scale.y);
            ParamUpdateData(PARAM_IMU_LEVEL_Z, &_acc.levelCali.scale.z);
			//更新mavlink参数
			
			//amvlink发送校准结果
			
		}
		else
		{
			_acc.levelCali.success = 0;
			
			//bsklink发送校准结果
			
		}
		
		//发送校准结果
		printf("Accelerometer level calibration result: %d", _acc.levelCali.success);
		
		_acc.levelCali.step = 0;
		SetCaliStatus(NoCali);
	}
}

/**********************************************************************************************************
*函 数 名: AccCalibrateEnable
*功能说明: 加速度校准使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AccCalibrateEnable(void)
{
    _acc.cali.should_cali = 1;
}

/**********************************************************************************************************
*函 数 名: LevelCalibrateEnable
*功能说明: 水平校准使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void LevelCalibrateEnable(void)
{
  _acc.levelCali.should_cali = 1;
	SetCaliStatus(ImuLevelCali);
}

/**********************************************************************************************************
*函 数 名: GetAccOffsetCaliData
*功能说明: 获取加速度零偏校准数据
*形    参: 无
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetAccOffsetCaliData(void)
{
    return _acc.cali.offset;
}

/**********************************************************************************************************
*函 数 名: GetAccScaleCaliData
*功能说明: 获取加速度比例校准数据
*形    参: 无
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetAccScaleCaliData(void)
{
    return _acc.cali.scale;
}

/**********************************************************************************************************
*函 数 名: GetLevelCalibraData
*功能说明: 获取IMU安装误差校准参数
*形    参: 无
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetLevelCalibraData(void)
{
    return _acc.levelCali.scale;
}

/**********************************************************************************************************
*函 数 名: GetAccMag
*功能说明: 获取加速度数据模值
*形    参: 无
*返 回 值: 模值
**********************************************************************************************************/
float GetAccMag(void)
{
    return _acc.mag;
}

/**********************************************************************************************************
*函 数 名: AccGetData
*功能说明: 获取经过处理后的加速度数据
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t AccGetData(void)
{
    return _acc.data;
}

/**********************************************************************************************************
*函 数 名: AccLpfGetData
*功能说明: 获取经过滤波后的加速度数据
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t AccLpfGetData(void)
{
    return _acc.dataLpf;
}



