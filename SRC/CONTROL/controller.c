/**********************************************************************************************************
 * @文件     controller.c
 * @说明     控制器总和
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "controller.h"
#include "flightStatus.h"
#include "mathConfig.h"
#include "remote.h"

#include "disturbanceEst.h"
#include "setPoint.h"
#include "ahrs.h"
#include "optitrack.h"
#include "gyroscope.h"
#include "pd.h"
#include "smc.h"


vehicle_para _veh;
control_set_t _ctrl_only;		// 只有控制器计算生成的控制量
control_set_t _ctrl;				// 结合了扰动估计的控制量
droneState_t _state;

#define MAX_POS_DES	2	//手动飞行中摇杆所能对应的最大位置误差（单位：m）
#define MAX_ATT_DES	1.57	//手动飞行中摇杆所能对应的最大角度误差（单位:rad）
#define VEL_DEFAULT 0.5	//手动飞行中默认的线速度大小(m/s)
#define ANG_VEL_DEFAULT 0.1	//手动飞行中默认的角速度大小(rad/s)


enum
{
	IMU_EST,
	OPTITRACK_IN
};

enum
{
	PD,
	SMC
};


#define FEEDBACK_SRC	IMU_EST
#define CONTROLLER SMC


/**********************************************************************************************************
*函 数 名: ControllerInit
*功能说明: 控制器参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ControllerInit(void)
{
	/***********机身物理参数初始化***********************/
	//inerita matrix
	_veh.J[0]=0.0444f;	_veh.J[1]=  0.0f;	 	_veh.J[2]=   0.0f;
	_veh.J[3]=   0.0f;  _veh.J[4]=0.434f;		_veh.J[5]=   0.0f;
	_veh.J[6]=   0.0f;	_veh.J[7]=  0.0f;		_veh.J[8]=0.0756f;
	
	_veh.J_inv[0]=22.5087f;	_veh.J_inv[1]=    0.0f; _veh.J_inv[2]=    0.0f;
	_veh.J_inv[3]=    0.0f; _veh.J_inv[4]=23.0661f;	_veh.J_inv[5]=    0.0f;
	_veh.J_inv[6]=    0.0f;	_veh.J_inv[7]=    0.0f;	_veh.J_inv[8]=13.2188f;
	
	
	_veh.mass = 0.8846;
	// inverse of control allocation matrix
//	_veh.Binv[0][0]=      0; _veh.Binv[0][1]=-0.4507; _veh.Binv[0][2]=-0.2461; _veh.Binv[0][3]=      0; _veh.Binv[0][4]= 1.3279; _veh.Binv[0][5]=-0.6640;
//	_veh.Binv[1][0]= 0.4262; _veh.Binv[1][1]=-0.2461; _veh.Binv[1][2]= 0.2253; _veh.Binv[1][3]= 1.1500; _veh.Binv[1][4]=-0.6640; _veh.Binv[1][5]=-0.6640;
//	_veh.Binv[2][0]= 0.3903; _veh.Binv[2][1]= 0.2253; _veh.Binv[2][2]=-0.2461; _veh.Binv[2][3]=-1.1500; _veh.Binv[2][4]=-0.6640; _veh.Binv[2][5]=-0.6640;
//	_veh.Binv[3][0]=      0; _veh.Binv[3][1]= 0.4921; _veh.Binv[3][2]= 0.2253; _veh.Binv[3][3]=      0; _veh.Binv[3][4]= 1.3279; _veh.Binv[3][5]=-0.6640;
//	_veh.Binv[4][0]=-0.3903; _veh.Binv[4][1]= 0.2253; _veh.Binv[4][2]=-0.2461; _veh.Binv[4][3]= 1.1500; _veh.Binv[4][4]=-0.6640; _veh.Binv[4][5]=-0.6640;
//	_veh.Binv[5][0]=-0.4262; _veh.Binv[5][1]=-0.2461; _veh.Binv[5][2]= 0.2253; _veh.Binv[5][3]=-1.1500; _veh.Binv[5][4]=-0.6640; _veh.Binv[5][5]=-0.6640;
	
	_veh.Binv[0][0]=      0; _veh.Binv[0][1]= 0.4714; _veh.Binv[0][2]= 0.2357; _veh.Binv[0][3]=      0; _veh.Binv[0][4]=-1.3889; _veh.Binv[0][5]= 0.6360;
	_veh.Binv[1][0]= 0.4082; _veh.Binv[1][1]=-0.2357; _veh.Binv[1][2]= 0.2357; _veh.Binv[1][3]= 1.2029; _veh.Binv[1][4]=-0.6945; _veh.Binv[1][5]=-0.6360;
	_veh.Binv[2][0]=-0.4082; _veh.Binv[2][1]=-0.2357; _veh.Binv[2][2]= 0.2357; _veh.Binv[2][3]= 1.2029; _veh.Binv[2][4]= 0.6945; _veh.Binv[2][5]= 0.6360;
	_veh.Binv[3][0]=      0; _veh.Binv[3][1]= 0.4714; _veh.Binv[3][2]= 0.2357; _veh.Binv[3][3]=      0; _veh.Binv[3][4]= 1.3889; _veh.Binv[3][5]=-0.6360;
	_veh.Binv[4][0]= 0.4082; _veh.Binv[4][1]=-0.2357; _veh.Binv[4][2]= 0.2357; _veh.Binv[4][3]=-1.2029; _veh.Binv[4][4]= 0.6945; _veh.Binv[4][5]= 0.6360;
	_veh.Binv[5][0]=-0.4082; _veh.Binv[5][1]=-0.2357; _veh.Binv[5][2]= 0.2357; _veh.Binv[5][3]=-1.2029; _veh.Binv[5][4]=-0.6945; _veh.Binv[5][5]=-0.6360;

	_veh.T = 0.004f;
	
	//选择控制器
	if(CONTROLLER == PD)
	{
		pdCtrlInit();
	}
	else if(CONTROLLER == SMC)
	{
		smcCtrlInit();
	}
	//扰动估计模块初始化
	estimatorInit();
}


/**********************************************************************************************************
*函 数 名: CtrlTask
*功能说明: 控制器主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void CtrlTask(void)
{
	/********不同的飞行模式选择不同的轨线期望值和反馈值来源**********/
	// 轨线期望值生成， 来源： 1.遥控器 2.按时间轴生成
	if(GetFlightMode() == MANUAL)
	{
		/*******************摇杆值到期望值的映射***********************/
		float max_pos_des = MAX_POS_DES;
		float max_att_des = MAX_ATT_DES;
		// float vel_default = VEL_DEFAULT;
		float W_default = ANG_VEL_DEFAULT;
		int16_t _deadzone = 50;
		
		Vector3f_t att_des;
		
		// 遥控通道值归一化
		_state.pos_des.x = ApplyDeadbandFloat(CH_N[CH7], _deadzone)*0.0023f*max_pos_des;
		_state.pos_des.y = ApplyDeadbandFloat(CH_N[CH8], _deadzone)*0.0023f*max_pos_des;
		_state.pos_des.z = ApplyDeadbandFloat(CH_N[CH3], _deadzone)*0.0023f*max_pos_des;
		att_des.x = ApplyDeadbandFloat(CH_N[CH1], _deadzone)*0.0023f*max_att_des;
		att_des.y = ApplyDeadbandFloat(CH_N[CH2], _deadzone)*0.0023f*max_att_des;
		att_des.z = ApplyDeadbandFloat(CH_N[CH4], _deadzone)*0.0023f*max_att_des;
		
		/*********期望位置向量及各阶导数*****************/
		_state.vel_des.x = 0.0f; _state.vel_des.y = 0.0f; _state.vel_des.z = 0.0f;
		_state.acc_des.x = 0.0f; _state.acc_des.y = 0.0f; _state.acc_des.z = 0.0f;
		
		/*******根据期望的欧拉角度来生成相应的DCM，角速度和角加速度********/
		if(att_des.x<0)		_state.W_des.x = -W_default;
		else if(att_des.x==0)	_state.W_des.x = 0.0f;
		else							_state.W_des.x = W_default;
		if(att_des.y<0)		_state.W_des.y = -W_default;
		else if(att_des.y==0)	_state.W_des.y = 0.0f;
		else							_state.W_des.y = W_default;
		if(att_des.z<0)		_state.W_des.z = -W_default;
		else if(att_des.z==0)	_state.W_des.z = 0.0f;
		else							_state.W_des.z = W_default;
		
		_state.W_dot_des.x = 0.0f; _state.W_dot_des.y = 0.0f; _state.W_dot_des.z = 0.0f;
		
		Euler_to_DCM(_state.R_des, att_des);
		/****************获取当前角度和角速度，*************************/
	}
	else if(GetFlightMode() == MISSION)	// 沿既定轨线飞行
	{
		//期望轨迹由SetPoint模块生成
		GetDesiredPos(&_state.pos_des);
		GetDesiredVel(&_state.vel_des);
		GetDesiredAcc(&_state.acc_des);
		
		GetDesiredAtt(_state.R_des);
		GetDesiredAngVel(&_state.W_des);
		GetDesiredAngAcc(&_state.W_dot_des);
	}
	
	/******飞行器姿态反馈， 来源：1.IMU解算 2.optitrack **********/
	if(FEEDBACK_SRC == IMU_EST)
	{
		/*******反馈的位置向量及各阶导数************/
		_state.pos_fb.x = 0.0f; _state.pos_fb.y = 0.0f; _state.pos_fb.z = 0.0f;
		_state.vel_fb.x = 0.0f; _state.vel_fb.y = 0.0f; _state.vel_fb.z = 0.0f;
		
		/*********当前姿态和角速度，来自IMU解算，角度单位(rad)*********/
		GetDCM(_state.R_fb);
		_state.W_fb = GyroLpfGetData();
	}
	else if(FEEDBACK_SRC == OPTITRACK_IN)
	{
		/*********使用Optitrack提供的真是反馈**********/
		_state.pos_fb = GetOptiPos();
		_state.vel_fb = GetOptiVel();
		
		GetOptiAttDCM(_state.R_fb);
		_state.W_fb = GetOptiAngVel();
	}
	
	/************控制器选择:  1.PD  2.SMC****************/
	if(CONTROLLER == PD)
	{
		pdCtrlUpdate();
	}
	else if(CONTROLLER == SMC)
	{
		smcCtrlUpdate();
	}
	
	/****扰动估计***/
	estimatorUpdate();
	
	
	// 加和控制器和扰动估计器的控制量
	_ctrl.F_b = Vector3f_Add(_ctrl_only.F_b, _est.F_b);
	_ctrl.M_b = Vector3f_Add(_ctrl_only.M_b, _est.M_b);
	_ctrl.wrench[0] = _ctrl.F_b.x;
	_ctrl.wrench[1] = _ctrl.F_b.y;
	_ctrl.wrench[2] = _ctrl.F_b.z;
	_ctrl.wrench[3] = _ctrl.M_b.x;
	_ctrl.wrench[4] = _ctrl.M_b.y;
	_ctrl.wrench[5] = _ctrl.M_b.z;
}
