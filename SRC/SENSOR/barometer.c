/**********************************************************************************************************
 * @文件     barometer.c
 * @说明     气压计数据预处理
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2018.05
**********************************************************************************************************/
#include "barometer.h"
#include "delay.h"
#include "module.h"

BAROMETER_t _baro;


/**********************************************************************************************************
*函 数 名: BaroDataPreTreat
*功能说明: 气压高度数据预处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BaroDataPreTreat(float baroPresRaw, float baroTempRaw)
{
	static uint64_t lastTime = 0;
	
	float deltaT = (GetSysTimeUs() - lastTime) * 1e-6;
	lastTime = GetSysTimeUs();
	
	_baro.pressure = baroPresRaw;
	_baro.temperature = baroTempRaw;
	
	
	//计算气压计高度
	float alt_3, baroAltTemp;
	alt_3 = ( 101400 - _baro.pressure ) / 1000.0f;
	baroAltTemp = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101400 - _baro.pressure ) * 100.0f ;
	
//	if(GetInitStatus() == HEAT_FINISH)
//	{
//		_baro.alt_offset += baroAltTemp;
//		_baro.alt_offset *= 0.5f;
//	}
	_baro.alt_offset += baroAltTemp;
	_baro.alt_offset *= 0.5f;
	
	baroAltTemp -= _baro.alt_offset;
	
	//TODO:飞行中的气压高度补偿
	
	
	/**********借鉴自BlueSkyFlightControl***************/
	//气压高度低通滤波
	_baro.alt = _baro.alt * 0.5f + baroAltTemp * 0.5f;
	
	//计算气压变化速度，并进行低通滤波
	_baro.velocity = _baro.velocity * 0.65f + ((_baro.alt - _baro.lastAlt) / deltaT) * 0.35f;
	_baro.lastAlt = _baro.alt;

}

/**********************************************************************************************************
*函 数 名: BaroGetAlt
*功能说明: 获取气压高度数据
*形    参: 无
*返 回 值: 气压高度
**********************************************************************************************************/
float BaroGetAlt(void)
{
    return _baro.alt;
}

/**********************************************************************************************************
*函 数 名: BaroGetTemp
*功能说明: 获取气压温度数据
*形    参: 无
*返 回 值: 气压温度
**********************************************************************************************************/
float BaroGetTemp(void)
{
    return _baro.temperature;
}

/**********************************************************************************************************
*函 数 名: BaroGetVelocity
*功能说明: 获取气压高度变化速度
*形    参: 无
*返 回 值: 气压高度变化速度
**********************************************************************************************************/
float BaroGetVelocity(void)
{
    return _baro.velocity;
}



