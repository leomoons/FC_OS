/**********************************************************************************************************
 * @文件     magnetometer.c
 * @说明     磁力计校准及数据预处理
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "magnetometer.h"
#include "parameter.h"
#include "mathConfig.h"
#include "LevenbergMarquardt.h"
#include "ak8975.h"
#include "delay.h"
#include "drv_usart.h"
#include "gyroscope.h"
#include "flightStatus.h"
#include "module.h"

//#define MAGCALI_LM 

enum
{
	MaxX,
	MinX,
	MaxY,
	MinY,
	MaxZ,
	MinZ
};

//地球表面赤道上的磁场强度在0.29～0.40高斯之间,两极处的强度略大,地磁北极约0.61高斯,南极约0.68高斯
//不同地方磁场强度有所区别，所以每次校准磁力计时要把当地磁场强度的大概值保存下来

MAGNETOMETER_t _mag;

/**********************************************************************************************************
*函 数 名: MagCaliDataInit
*功能说明: 磁力计校准参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MagPreTreatInit(void)
{
	ParamGetData(PARAM_MAG_OFFSET_X, &_mag.cali.offset.x, 4);
	ParamGetData(PARAM_MAG_OFFSET_Y, &_mag.cali.offset.y, 4);
	ParamGetData(PARAM_MAG_OFFSET_Z, &_mag.cali.offset.z, 4);
	ParamGetData(PARAM_MAG_SCALE_X, &_mag.cali.scale.x, 4);
	ParamGetData(PARAM_MAG_SCALE_Y, &_mag.cali.scale.y, 4);
	ParamGetData(PARAM_MAG_SCALE_Z, &_mag.cali.scale.z, 4);
	ParamGetData(PARAM_MAG_EARTH_MAG, &_mag.earthMag, 4);
	
	// confirm the validity 
	if(isnan(_mag.cali.offset.x) || isnan(_mag.cali.offset.y) || isnan(_mag.cali.offset.z) || \
		isnan(_mag.cali.scale.x) || isnan(_mag.cali.scale.y) || isnan(_mag.cali.scale.z) || \
		abs(_mag.cali.offset.x)<0.3f || abs(_mag.cali.scale.x)>2 || \
		abs(_mag.cali.offset.y)<0.3f || abs(_mag.cali.scale.y)>2 || \
		abs(_mag.cali.offset.z)<0.3f || abs(_mag.cali.scale.z)>2 || \
		_mag.cali.scale.x==0 || _mag.cali.scale.y==0 || _mag.cali.scale.z==0)
	{
		_mag.cali.offset.x = 0;
		_mag.cali.offset.y = 0;
		_mag.cali.offset.z = 0;
		_mag.cali.scale.x = 1;
		_mag.cali.scale.y = 1;
		_mag.cali.scale.z = 1;
		_mag.earthMag = 0.4;
	}
}


/**********************************************************************************************************
*函 数 名: MagDataPreTreat
*功能说明: 磁力计数据预处理
*形    参: 磁力计原始数据 磁力计预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
Vector3f_t MAGRAW1;
void MagDataPreTreat(Vector3f_t *magRaw, Vector3f_t* magPre)
{
	MAGRAW1 = *magRaw;

	
	//磁力计数据校准
	_mag.data.x = (magRaw->x - _mag.cali.offset.x) * _mag.cali.scale.x;
    _mag.data.y = (magRaw->y - _mag.cali.offset.y) * _mag.cali.scale.y;
    _mag.data.z = (magRaw->z - _mag.cali.offset.z) * _mag.cali.scale.z;
	
	//低通滤波
	LowPassFilter1st(magPre, _mag.data, 0.5);
	
	//计算磁场强度模值，用于判断周边是否存在磁场干扰（正常值为1）
	_mag.mag = _mag.mag * 0.9f + Pythagorous3(_mag.data.x, _mag.data.y, _mag.data.z) / _mag.earthMag * 0.01f;
	
}

/**********************************************************************************************************
*函 数 名: MagCalibration（可选择LM取最优校准参数法 或 匿名飞控校准法）
*功能说明: 磁力计校准
*形    参: 磁力计原始数据向量指针  角速度向量指针
*返 回 值: 无
**********************************************************************************************************/
#ifdef MAGCALI_LM
void MagCalibration(Vector3f_t *magRaw, Vector3f_t *gyroLpf)
{
	static Vector3f_t samples[6];
    static uint32_t cnt_m=0;
    static float cali_rotate_angle = 0.0f;
    static Vector3f_t new_offset;
    static Vector3f_t new_scale;
    static float earthMag = 0;
	
	//计算时间间隔，用于积分
	static uint64_t previousT;
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;
	previousT = GetSysTimeUs();
	
	if(_mag.cali.should_cali)
	{
		//校准分两个阶段：1.水平旋转 2.机头朝上或朝下然后水平旋转
		//两个阶段分别对飞机的z轴和x轴陀螺仪数据进行积分，记录旋转过的角度
		if(_mag.cali.step == 1)
		{
			cali_rotate_angle += gyroLpf->z * deltaT;
			
		}
		else if(_mag.cali.step == 2)
		{
			cali_rotate_angle += gyroLpf->x * deltaT;
		}
		
		if(cnt_m == 0)
		{
			_mag.cali.step =1;
			cali_rotate_angle = 0;
			cnt_m++;
			
			//发送当前校准步骤
			
			printf("magnerometer calibration step 1.");
		}
		else if(cnt_m == 1)
		{
			//初始化磁场强度模值
			earthMag = Pythagorous3(magRaw->x, magRaw->y, magRaw->z);
			//初始化采样点
			samples[MaxX] = samples[MinX] = *magRaw;
			samples[MaxY] = samples[MinY] = *magRaw;
			samples[MaxZ] = samples[MinZ] = *magRaw;
			cnt_m++;
		}
		else
		{
			//实时计算磁场强度模值
			earthMag = earthMag * 0.998f + Pythagorous3(magRaw->x, magRaw->y, magRaw->z)*0.002f;
			
			//找到每个轴的最大最小值，并对采样值进行一阶低通滤波
			if(Pythagorous3(magRaw->x, magRaw->y, magRaw->z) < earthMag*1.5f)
			{
				//找到每个轴的最大最小值，并对采样值进行一阶低通滤波
				if(magRaw->x > samples[MaxX].x)
				{
					LowPassFilter1st(&samples[MaxX], *magRaw, 0.3);
				}
				if(magRaw->x < samples[MinX].x)
                {
                    LowPassFilter1st(&samples[MinX], *magRaw, 0.3);
                }
                if(magRaw->y > samples[MaxY].y)
                {
                    LowPassFilter1st(&samples[MaxY], *magRaw, 0.3);
                }
                if(magRaw->y < samples[MinY].y)
                {
                    LowPassFilter1st(&samples[MinY], *magRaw, 0.3);
                }
                if(magRaw->z > samples[MaxZ].z)
                {
                    LowPassFilter1st(&samples[MaxZ], *magRaw, 0.3);
                }
                if(magRaw->z < samples[MinZ].z)
                {
                    LowPassFilter1st(&samples[MinZ], *magRaw, 0.3);
                }
			}
			else
			{
				earthMag = earthMag;
			}
			
			//TODO:mavlink发送当前校准进度
			
			
			//水平旋转一周
			if(_mag.cali.step == 1 && abs(cali_rotate_angle) > 2*M_PI)
			{
				_mag.cali.step = 2;
				cali_rotate_angle = 0;
				
				SetCaliStatus(MagCaliVertical);
				//TODO:bsklink发送当前校准步骤
				//TODO:mavlink发送当前校准步骤
			}
			//竖直旋转一周
			if(_mag.cali.step == 2 && abs(cali_rotate_angle) > 2*M_PI)
			{
				cnt_m = 0;
				_mag.cali.should_cali = 0;
				_mag.cali.step = 3;
				cali_rotate_angle = 0;
				earthMag = 0;
				
				//计算当地地磁场强度模值均值
				for(u8 i=0; i<3; i++)
				{
					earthMag += Pythagorous3((samples[i*2].x - samples[i*2+1].x) * 0.5f,
                                             (samples[i*2].y - samples[i*2+1].y) * 0.5f,
                                             (samples[i*2].z - samples[i*2+1].z) * 0.5f);
				}
				earthMag /= 3;
				
				//计算方程解初值
				float initBeta[6];
				initBeta[0] = (samples[MaxX].x + samples[MaxX].x) * 0.5f;
				initBeta[1] = (samples[MaxY].y + samples[MinY].y) * 0.5f;
                initBeta[2] = (samples[MaxZ].z + samples[MinZ].z) * 0.5f;
                initBeta[3] = 1 / earthMag;
                initBeta[4] = 1 / earthMag;
                initBeta[5] = 1 / earthMag;
				
				//LM法求解传感器误差方程最优解
				LevenbergMarquardt(samples, &new_offset, &new_scale, initBeta, earthMag);
				
				//判断校准参数是否正常
				if(isnan(new_scale.x) || isnan(new_scale.y) || isnan(new_scale.z))
				{
					_mag.cali.success = false;
				}
				else if(fabsf(new_scale.x-1.0f) > 0.8f || fabsf(new_scale.y-1.0f) > 0.8f || fabsf(new_scale.z-1.0f) > 0.8f)
				{
					_mag.cali.success = false;
				}
				else if(fabsf(new_offset.x) > (earthMag * 2) || fabsf(new_offset.y) > (earthMag * 2) || fabsf(new_offset.z) > (earthMag * 2))
				{
					_mag.cali.success = false;
				}
				else
				{
					_mag.cali.success = true;
				}
				
				if(_mag.cali.success)
				{
					_mag.cali.offset = new_offset;
					_mag.cali.scale = new_scale;
					_mag.earthMag = earthMag;
					
					//保存校准参数
                    ParamUpdateData(PARAM_MAG_OFFSET_X, &_mag.cali.offset.x);
                    ParamUpdateData(PARAM_MAG_OFFSET_Y, &_mag.cali.offset.y);
                    ParamUpdateData(PARAM_MAG_OFFSET_Z, &_mag.cali.offset.z);
                    ParamUpdateData(PARAM_MAG_SCALE_X, &_mag.cali.scale.x);
                    ParamUpdateData(PARAM_MAG_SCALE_Y, &_mag.cali.scale.y);
                    ParamUpdateData(PARAM_MAG_SCALE_Z, &_mag.cali.scale.z);
                    ParamUpdateData(PARAM_MAG_EARTH_MAG, &_mag.earthMag);
					//TODO:更新mavlink参数
					//TODO:mavlink发送校准结果
					
					printf("Magnetometer calibrated successful.");
				}
				else
				{
					//TODO: mavlink发送校准结果
					
					printf("Magnetometer calibrated failed.");
				}
				
				//TODO: bsklink发送校准结果
				
				_mag.cali.step = 0;
				earthMag = 0;
				
				SetCaliStatus(NoCali);
			}
		}
	}
}
#else
Vector3f_t magMax;
Vector3f_t magMin;
void MagCalibration(Vector3f_t *magRaw, Vector3f_t *gyroLpf)
{
	static float mag_cali_angle = 0.0f;
	
	//计算时间间隔，用于积分
	static uint64_t previousT;
	float dT_s = (GetSysTimeUs() - previousT) * 1e-6f;
	previousT = GetSysTimeUs();
	
	
	if(_mag.cali.should_cali)
	{
		switch(_mag.cali.step)
		{
			case 0:		//第一步，水平旋转
				magMax.x = max(magMax.x, magRaw->x);
				magMax.y = max(magMax.y, magRaw->y);
				magMin.x = min(magMin.x, magRaw->x);
				magMin.y = min(magMin.y, magRaw->y);
			
				if(GetImuOrientation() != ORIENTATION_UP)		//校准过程中不够水平，则累计数据清零
				{
					mag_cali_angle = 0.0;
					magMax.x = magMax.y = magMin.x = magMin.y = 0.0f;
				}
				else
				{
					mag_cali_angle += dT_s * gyroLpf->z;	//角度积分
					if(abs(mag_cali_angle) > 2*M_PI)
					{
						_mag.cali.step = 1;
						mag_cali_angle = 0.0f;
						
						SetCaliStatus(MagCaliVertical);
					}
				}
			break;
			case 1:		//第二步，竖直旋转,机头朝上
				magMax.z = max(magMax.z, magRaw->z);
				magMin.z = min(magMin.z, magRaw->z);
			
				if(GetImuOrientation() != ORIENTATION_FRONT)	//校准过程中不够竖直，则数据清零
				{
					mag_cali_angle = 0.0f;
					magMax.z = magMin.z = 0.0f;
				}
				else
				{
					mag_cali_angle += dT_s * gyroLpf->x;	//角度积分
					if(abs(mag_cali_angle) > 2*M_PI)
					{
						_mag.cali.step = 2;
						mag_cali_angle = 0.0f;
						
						SetCaliStatus(NoCali);
					}
				}
			break;
				
			case 2:		//第三步，根据数据计算校准参数并保存
				_mag.cali.offset.x = 0.5f * (magMax.x + magMin.x);
				_mag.cali.offset.y = 0.5f * (magMax.y + magMin.y);
				_mag.cali.offset.z = 0.5f * (magMax.z + magMin.z);
				_mag.cali.scale.x = safe_div(60, (0.5f*(magMax.x-magMin.x)), 0);
				_mag.cali.scale.y = safe_div(60, (0.5f*(magMax.y-magMin.y)), 0);
				_mag.cali.scale.z = safe_div(60, (0.5f*(magMax.z-magMin.z)), 0);
			
				//保存校准参数
                ParamUpdateData(PARAM_MAG_OFFSET_X, &_mag.cali.offset.x);
                ParamUpdateData(PARAM_MAG_OFFSET_Y, &_mag.cali.offset.y);
                ParamUpdateData(PARAM_MAG_OFFSET_Z, &_mag.cali.offset.z);
                ParamUpdateData(PARAM_MAG_SCALE_X, &_mag.cali.scale.x);
                ParamUpdateData(PARAM_MAG_SCALE_Y, &_mag.cali.scale.y);
                ParamUpdateData(PARAM_MAG_SCALE_Z, &_mag.cali.scale.z);
                //ParamUpdateData(PARAM_MAG_EARTH_MAG, &_mag.earthMag);
			
				_mag.cali.step = 0;
				_mag.cali.should_cali = 0;
			
				printf("Magnetometer calibrated completed.");
				
			break;
		}
	}
	
}
#endif



/**********************************************************************************************************
*函 数 名: MagGetData
*功能说明: 获取经过处理后的磁力计数据
*形    参: 磁场向量指针
*返 回 值: 无
**********************************************************************************************************/
void MagGetData(Vector3f_t *mag)
{
    *mag = _mag.data;
}

/**********************************************************************************************************
*函 数 名: MagCalibrateEnable
*功能说明: 磁力计校准使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MagCalibrateEnable(void)
{
    _mag.cali.should_cali = 1;
	SetCaliStatus(MagCaliHorizontal);
}

/**********************************************************************************************************
*函 数 名: GetMagOffsetCaliData
*功能说明: 获取磁力计零偏校准数据
*形    参: 无
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetMagOffsetCaliData(void)
{
    return _mag.cali.offset;
}

/**********************************************************************************************************
*函 数 名: GetAccScaleCaliData
*功能说明: 获取磁力计比例校准数据
*形    参: 无
*返 回 值: 校准参数
**********************************************************************************************************/
Vector3f_t GetMagScaleCaliData(void)
{
    return _mag.cali.scale;
}
