/**********************************************************************************************************
 * @文件     geoCtrl.c
 * @说明     几何控制器
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "geoCtrl.h"
#include "flightStatus.h"
#include "remote.h"
#include "mathConfig.h"
#include "controller.h"


#include "ahrs.h"
#include "setPoint.h"
#include "optitrack.h"
#include "gyroscope.h"
#define MAX_POS_DES	2	//手动飞行中摇杆所能对应的最大位置误差（单位：m）
#define MAX_ATT_DES	2	//手动飞行中摇杆所能对应的最大角度误差（单位:rad）
#define VEL_DEFAULT 0.5	//手动飞行中默认的线速度大小(m/s)
#define ANG_VEL_DEFAULT 0.1	//手动飞行中默认的角速度大小(rad/s)


GeoControl_t _geo;

/**********************************************************************************************************
*函 数 名: GeoControllerInit
*功能说明: 几何控制器参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GeoControllerInit(void)
{
	_geo.Kp[0]=-3; _geo.Kp[1]= 0; _geo.Kp[2]= 0;
	_geo.Kp[3]= 0; _geo.Kp[4]=-3; _geo.Kp[5]= 0;
	_geo.Kp[6]= 0; _geo.Kp[7]= 0; _geo.Kp[8]=-3;
	
	_geo.Kv[0]=-1; _geo.Kv[1]= 0; _geo.Kv[2]= 0;
	_geo.Kv[3]= 0; _geo.Kv[4]=-1; _geo.Kv[5]= 0;
	_geo.Kv[6]= 0; _geo.Kv[7]= 0; _geo.Kv[8]=-1;
	
	_geo.KR[0]=-2; _geo.KR[1]= 0; _geo.KR[2]= 0;
	_geo.KR[3]= 0; _geo.KR[4]=-1; _geo.KR[5]= 0;
	_geo.KR[6]= 0; _geo.KR[7]= 0; _geo.KR[8]=-1;
	
	_geo.KW[0]=-0.5; _geo.KW[1]= 0; _geo.KW[2]= 0;
	_geo.KW[3]= 0; _geo.KW[4]=-0.3; _geo.KW[5]= 0;
	_geo.KW[6]= 0; _geo.KW[7]= 0; _geo.KW[8]=-0.2;
}


/**********************************************************************************************************
*函 数 名: ForceCal
*功能说明: 几何控制器中的作用力计算
*形    参: 无
*返 回 值: 作用力向量 
**********************************************************************************************************/
static Vector3f_t ForceCal(void)
{
	/***********定义计算中间量***************/
	Vector3f_t tmp1, tmp2, tmp3, tmp4;
	Vector3f_t sum1, sum2, sum3;
	float R_T[9];
	/***********计算位置和速度误差***********/
	Vector3f_t pos_err = Vector3f_Sub(_geo.pos_fb, _geo.pos_des);
	Vector3f_t vel_err = Vector3f_Sub(_geo.vel_fb, _geo.vel_des);
	
	// each element of Force equation 
	tmp1 = Matrix3MulVector3(_geo.Kp, pos_err);		// first element	
	tmp2 = Matrix3MulVector3(_geo.Kv, vel_err); 	// second element
	tmp3.x=0.0f; tmp3.y=0.0f; tmp3.z=_veh.mass*GRAVITY_ACCEL;	//masss*g*[0;0;1]
	tmp4.x = _veh.mass * _geo.acc_des.x;
	tmp4.y = _veh.mass * _geo.acc_des.y;
	tmp4.z = _veh.mass * _geo.acc_des.z;			// mass.*desired_acceleration
	
	// add all the elements
	sum1 = Vector3f_Add(tmp1, tmp2);
	sum2 = Vector3f_Add(sum1, tmp3);
	sum3 = Vector3f_Add(sum2, tmp4);
	
	// Transpose of attitude matrix
	Matrix3_Tran(_geo.R_fb, R_T);
	
	return Matrix3MulVector3(R_T, sum3);
}

/**********************************************************************************************************
*函 数 名: Attitude_Error
*功能说明: 几何控制中的姿态误差计算
*形    参: void
*返 回 值: 误差向量
**********************************************************************************************************/
static Vector3f_t Attitude_Error(void)
{
	float R_des_T[9], R_fb_T[9], tmp1[9], tmp2[9], tmp3[9];
	
	Matrix3_Tran(_geo.R_des, R_des_T);
	Matrix3_Tran(_geo.R_fb, R_fb_T);
	
	Matrix3_Mul(R_des_T, _geo.R_fb, tmp1);
	Matrix3_Mul(R_fb_T, _geo.R_des, tmp2);
	Matrix3_Sub(tmp1, tmp2, tmp3);
	
	Vector3f_t R_err;
	// vee_map
	R_err.x = tmp3[7]/2;
	R_err.y = tmp3[2]/2;
	R_err.z = tmp3[3]/2;
	
	return R_err;
}

/**********************************************************************************************************
*函 数 名: Angular_Rate_Error
*功能说明: 几何控制中的角速度误差计算
*形    参: 期望姿态旋转矩阵数组头， 反馈姿态局旋转矩阵数组头
*返 回 值: 误差向量
**********************************************************************************************************/
static Vector3f_t Angular_Rate_Error(void)
{
	float R_fb_T[9], tmp1[9];
	
	Matrix3_Tran(_geo.R_fb, R_fb_T);
	
	Matrix3_Mul(R_fb_T, _geo.R_des, tmp1);
	Vector3f_t v1 = Matrix3MulVector3(tmp1, _geo.W_des);
	
	return Vector3f_Sub(_geo.W_fb, v1);
}

/**********************************************************************************************************
*函 数 名: MomentCal
*功能说明: 几何控制器中的力矩计算
*形    参: 无
*返 回 值: 力矩向量 
**********************************************************************************************************/
static Vector3f_t MomentCal(void)
{
	/***********定义计算中间量***************/
	Vector3f_t R_err, W_err;
	Vector3f_t v1, v2, v3, v4, v5, v6, v7, v8, v9, v10;
	float m1[9], m2[9], m3[9];
	float R_fb_T[9], W_fb_hat[9];
	
	//中间量计算
	Hat_Map(W_fb_hat, _geo.W_fb);
	Matrix3_Tran(_geo.R_fb, R_fb_T);
	R_err = Attitude_Error();
	W_err = Angular_Rate_Error();
	
	//each element of moment function
	v1 = Matrix3MulVector3(_geo.KR, R_err);		//first element
	v2 = Matrix3MulVector3(_geo.KW, W_err);		//second element
	v3 = Matrix3MulVector3(_veh.J, _geo.W_fb);
	v4 = VectorCrossProduct(_geo.W_fb, v3);		//third element
	//将括号内作为最后一项
	Matrix3_Mul(W_fb_hat, R_fb_T, m1);
	Matrix3_Mul(m1, _geo.R_des, m2);
	v5 = Matrix3MulVector3(m2, _geo.W_des);		//括号内的第一项
	Matrix3_Mul(R_fb_T, _geo.R_des, m3);
	v6 = Matrix3MulVector3(m3, _geo.W_dot_des);	//括号内的第二项
	v7 = Vector3f_Sub(v5, v6);
	v8 = Matrix3MulVector3(_veh.J, v7);
	
	// add all the elements
	v9 = Vector3f_Add(v1, v2);
	v10 = Vector3f_Add(v9, v4);
	
	return Vector3f_Sub(v10, v8);
}



/**********************************************************************************************************
*函 数 名: GeoCtrlTask
*功能说明: 几何控制器主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GeoCtrlTask(void)
{
	//不同的飞行模式选择不同的轨线期望值和反馈值来源
	if(GetFlightMode() == MANUAL)
	{
		/********************摇杆值到期望值的映射************************/
		float max_pos_des = MAX_POS_DES;
		float max_att_des = MAX_ATT_DES;
		//float vel_default = VEL_DEFAULT;
		float W_default = ANG_VEL_DEFAULT;
		int16_t _deadzone = 50;

		Vector3f_t att_des;
		
		//遥控通道值归一化
		_geo.pos_des.x = ApplyDeadbandFloat(CH_N[CH7], _deadzone)*0.0023f*max_pos_des;
		_geo.pos_des.y = ApplyDeadbandFloat(CH_N[CH8], _deadzone)*0.0023f*max_pos_des;
		_geo.pos_des.z = ApplyDeadbandFloat(CH_N[CH3], _deadzone)*0.0023f*max_pos_des;
		att_des.x = ApplyDeadbandFloat(CH_N[CH1], _deadzone)*0.0023f*max_att_des;
		att_des.y = ApplyDeadbandFloat(CH_N[CH2], _deadzone)*0.0023f*max_att_des;
		att_des.z = ApplyDeadbandFloat(CH_N[CH4], _deadzone)*0.0023f*max_att_des;
		
		/*************位置向量及各阶导数(setpoint&feedback)**************/
		_geo.vel_des.x = 0.0f; _geo.vel_des.y = 0.0f; _geo.vel_des.z = 0.0f;
		_geo.acc_des.x = 0.0f; _geo.acc_des.y = 0.0f; _geo.acc_des.z = 0.0f;
		_geo.pos_fb.x = 0.0f; _geo.pos_fb.y = 0.0f; _geo.pos_fb.z = 0.0f;
		_geo.vel_fb.x = 0.0f; _geo.vel_fb.y = 0.0f; _geo.vel_fb.z = 0.0f;
		
		/*******根据期望的欧拉角度来生成相应的DCM,角速度和角加速度*********/
		if(att_des.x<0)		_geo.W_des.x = -W_default;
		else if(att_des.x==0)	_geo.W_des.x = 0.0f;
		else						_geo.W_des.x = W_default;
		if(att_des.y<0)		_geo.W_des.y = -W_default;
		else if(att_des.y==0)	_geo.W_des.y = 0.0f;
		else						_geo.W_des.y = W_default;
		if(att_des.z<0)		_geo.W_des.z = -W_default;
		else if(att_des.z==0)	_geo.W_des.z = 0.0f;
		else						_geo.W_des.z = W_default;
		
		_geo.W_dot_des.x = 0.0f; _geo.W_dot_des.y = 0.0f; _geo.W_dot_des.z = 0.0f;
		
		Euler_to_DCM(_geo.R_des, att_des);	
		/*************获取当前角度和角速度，来自IMU，采用rad***************/
		GetDCM(_geo.R_fb);
		_geo.W_fb = GyroLpfGetData();
	}
	else if(GetFlightMode() == MISSION)		//沿既定轨线飞行
	{
		//期望轨迹由setPoint模块生成
		GetDesiredPos(&_geo.pos_des);
		GetDesiredVel(&_geo.vel_des);
		GetDesiredAcc(&_geo.acc_des);
		
		GetDesiredAtt(_geo.R_des);
		GetDesiredAngVel(&_geo.W_des);
		GetDesiredAngAcc(&_geo.W_dot_des);
		
		//使用OptiTrack提供的真实反馈
		_geo.pos_fb = GetOptiPos();
		_geo.vel_fb = GetOptiVel();
		
		GetOptiAttDCM(_geo.R_fb);
		_geo.W_fb = GetOptiAngVel();
		
		//暂时没有optitrack数据的情况
		_geo.pos_fb.x = 1.0f;  _geo.pos_fb.y = -1.0f;  _geo.pos_fb.z = 0.0f;
		_geo.vel_fb.x = 1.0f;  _geo.vel_fb.y = -1.0f;  _geo.vel_fb.z = 0.0f;
		_geo.W_fb.x = 1.0f;  _geo.W_fb.y = -1.0f; _geo.W_fb.z = 0.0f;
		_geo.R_fb[0] = 1.0f;  _geo.R_fb[1] = 0.0f;  _geo.R_fb[0] = 0.0f;
		_geo.R_fb[0] = 0.0f;  _geo.R_fb[1] = 1.0f;  _geo.R_fb[0] = 0.0f;
		_geo.R_fb[0] = 0.0f;  _geo.R_fb[1] = 0.0f;  _geo.R_fb[0] = 1.0f;
	}
	// calculate desired set given desired trajectory
	Vector3f_t force = ForceCal();
	Vector3f_t moment = MomentCal();
	
	_geo.CtrlSet[0] = force.x;
	_geo.CtrlSet[1] = force.y;
	_geo.CtrlSet[2] = force.z;
	_geo.CtrlSet[3] = moment.x;
	_geo.CtrlSet[4] = moment.y;
	_geo.CtrlSet[5] = moment.z;
	
}


