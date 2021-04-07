#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "stm32f4xx.h"

#define PARAM_START_ADDR 0x00000000


enum PARAM_TYPE
{
	PARAM_CHECK_NUM,
	PARAM_CHECK_SUM,
	/*******陀螺仪校准参数*********/
    PARAM_GYRO_OFFSET_X,
    PARAM_GYRO_OFFSET_Y,
    PARAM_GYRO_OFFSET_Z,
    PARAM_GYRO_SCALE_X,
    PARAM_GYRO_SCALE_Y,
    PARAM_GYRO_SCALE_Z,
	/*******加速度校准参数*********/
    PARAM_ACC_OFFSET_X,
    PARAM_ACC_OFFSET_Y,
    PARAM_ACC_OFFSET_Z,
    PARAM_ACC_SCALE_X,
    PARAM_ACC_SCALE_Y,
    PARAM_ACC_SCALE_Z,
	/*******磁力计校准参数*********/
    PARAM_MAG_OFFSET_X,
    PARAM_MAG_OFFSET_Y,
    PARAM_MAG_OFFSET_Z,
    PARAM_MAG_SCALE_X,
    PARAM_MAG_SCALE_Y,
    PARAM_MAG_SCALE_Z,
    PARAM_MAG_EARTH_MAG,
	/*********水平校准参数*********/
    PARAM_IMU_LEVEL_X,
    PARAM_IMU_LEVEL_Y,
    PARAM_IMU_LEVEL_Z,
		
	/*******PD控制器增益系数********/
		CONTROLLER_PD_Kp_X,
		CONTROLLER_PD_Kp_Y,
		CONTROLLER_PD_Kp_Z,
		CONTROLLER_PD_Kv_X,
		CONTROLLER_PD_Kv_Y,
		CONTROLLER_PD_Kv_Z,
		CONTROLLER_PD_KR_X,
		CONTROLLER_PD_KR_Y,
		CONTROLLER_PD_KR_Z,
		CONTROLLER_PD_KW_X,
		CONTROLLER_PD_KW_Y,
		CONTROLLER_PD_KW_Z,
		
	/*******SMC控制器增益系数********/
		CONTROLLER_SMC_csP_X,
		CONTROLLER_SMC_csP_Y,
		CONTROLLER_SMC_csP_Z,
		CONTROLLER_SMC_HsP_X,
		CONTROLLER_SMC_HsP_Y,
		CONTROLLER_SMC_HsP_Z,
		CONTROLLER_SMC_ksP_X,
		CONTROLLER_SMC_ksP_Y,
		CONTROLLER_SMC_ksP_Z,
		CONTROLLER_SMC_csR_X,
		CONTROLLER_SMC_csR_Y,
		CONTROLLER_SMC_csR_Z,
		CONTROLLER_SMC_HsR_X,
		CONTROLLER_SMC_HsR_Y,
		CONTROLLER_SMC_HsR_Z,
		CONTROLLER_SMC_ksR_X,
		CONTROLLER_SMC_ksR_Y,
		CONTROLLER_SMC_ksR_Z,
		
	/******************************/
    PARAM_NUM
};


union Parameter_u
{
	//参数读出缓冲区，乘4是因为float占4个字节
	uint8_t buffer[PARAM_NUM*4];
	float data[PARAM_NUM];
};

extern union Parameter_u Param;



void ParamInit(void);
void ParamReadFromFlash(void);
void ParamBufferReset(void);
void ParamSaveToFlash(void);
void ParamUpdateData(uint16_t dataNum, const void * data);
void ParamGetData(uint16_t dataNum, void *data, uint8_t length);
//const char* ParamGetString(uint8_t paramNum);
void Param_save_cnt_tox(uint8_t cnt);

#endif
