/**********************************************************************************************************
 * @文件     setPoint.c
 * @说明     飞行器轨迹期望值生成
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "setPoint.h"
#include "remote.h"
#include "mathConfig.h"
#include "boardConfig.h"

#include "flightStatus.h"

Setpoint_t _set;


/**********************************************************************************************************
*函 数 名: PosUpdate
*功能说明: 位置期望值更新
*形    参: 时间增量， 轨迹生成当前时间
*返 回 值: void
**********************************************************************************************************/
void PosUpdate(float dT_s, float T)
{
	float v = 1.0;
	float a = 1.0;
	
	_set.pos.x = v*T; _set.pos.y = 1.0f*sin(a*T);      _set.pos.z = 1.5f*cos(a*T);
	_set.vel.x = v;   _set.vel.y = 1.0f*a*cos(a*T);    _set.vel.z =-1.5f*a*sin(a*T);
	_set.acc.x = 0;   _set.acc.y =-1.0f*a*a*sin(a*T);  _set.acc.x =-1.5f*a*a*cos(a*T);
}

/**********************************************************************************************************
*函 数 名: AttUpdate
*功能说明: 姿态期望值更新
*形    参: 时间增量， 轨迹生成当前时间
*返 回 值: void
**********************************************************************************************************/
void AttUpdate(float dT_s, float T)
{
	static float R_former[9] = {1,0,0, 0,1,0, 0,0,1};
	static Vector3f_t W_former;
	W_former.x = 0.0f; W_former.y = 0.0f; W_former.z = 0.0f; 
	Vector3f_t W_dot;
	float R_dot[9];
	
	if(T>=0 && T<=10)
	{
		W_dot.x = 3.14159/10;
		W_dot.y = 0.0;
		W_dot.z = 0.0;
	}
	else
	{
		W_dot.x = 0.0;
		W_dot.y = 0.0;
		W_dot.z = 0.0;
	}
	
	//姿态的积分迭代
	_set.ang_vel.x = dT_s*W_dot.x + W_former.x;
	_set.ang_vel.y = dT_s*W_dot.y + W_former.y;
	_set.ang_vel.z = dT_s*W_dot.z + W_former.z;
		
	R_dot[0] = R_former[1]*_set.ang_vel.z 		  + R_former[2]*_set.ang_vel.y*(-1.0f);
	R_dot[1] = R_former[0]*_set.ang_vel.z*(-1.0f) + R_former[2]*_set.ang_vel.x        ;
	R_dot[2] = R_former[0]*_set.ang_vel.y 	      + R_former[1]*_set.ang_vel.x*(-1.0f);
	R_dot[3] = R_former[4]*_set.ang_vel.z 		  + R_former[5]*_set.ang_vel.y*(-1.0f);
	R_dot[4] = R_former[3]*_set.ang_vel.z*(-1.0f) + R_former[5]*_set.ang_vel.x        ;
	R_dot[5] = R_former[3]*_set.ang_vel.y 		  + R_former[4]*_set.ang_vel.x*(-1.0f);
	R_dot[6] = R_former[7]*_set.ang_vel.z 		  + R_former[8]*_set.ang_vel.y*(-1.0f);
	R_dot[7] = R_former[6]*_set.ang_vel.z*(-1.0f) + R_former[8]*_set.ang_vel.x        ;
	R_dot[8] = R_former[6]*_set.ang_vel.y 		  + R_former[7]*_set.ang_vel.x*(-1.0f); 
	
	_set.att[0]=dT_s*R_dot[0]+R_former[0]; _set.att[1]=dT_s*R_dot[1]+R_former[1]; _set.att[2]=dT_s*R_dot[2]+R_former[2];
	_set.att[3]=dT_s*R_dot[3]+R_former[3]; _set.att[4]=dT_s*R_dot[4]+R_former[4]; _set.att[5]=dT_s*R_dot[5]+R_former[5];
	_set.att[6]=dT_s*R_dot[6]+R_former[6]; _set.att[7]=dT_s*R_dot[7]+R_former[7]; _set.att[8]=dT_s*R_dot[8]+R_former[8];
	
	R_former[0]=_set.att[0]; R_former[1] = _set.att[1]; R_former[2] = _set.att[2];
	R_former[3]=_set.att[3]; R_former[4] = _set.att[4]; R_former[5] = _set.att[5];
	R_former[6]=_set.att[6]; R_former[7] = _set.att[7]; R_former[8] = _set.att[8];
	
	W_former.x = _set.ang_vel.x; W_former.y = _set.ang_vel.y; W_former.z = _set.ang_vel.z;
	_set.ang_acc.x = W_dot.x; _set.ang_acc.y = W_dot.y; _set.ang_acc.z = W_dot.z;
}


/**********************************************************************************************************
*函 数 名: SetPointUpdate
*功能说明: 轨迹期望值更新
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void SetPointUpdate(void)
{
	static uint64_t T_now=0, T_former=0, T_start=0;
	float dT_s;
	float T;
	
	if(GetFlightMode() == MISSION)
	{
		T_now = GetSysTimeUs();
		T = (T_now-T_start)*1e-6f;
		dT_s = (T_now - T_former)*1e-6f;
		PosUpdate(dT_s, T);
		AttUpdate(dT_s, T);
		T_former = GetSysTimeUs();
	}
	else
	{
		T_start = GetSysTimeUs();
		T_former = T_start;
		
		_set.pos.x=0.0f; _set.pos.y=0.0f; _set.pos.z=0.0f;
		_set.vel.x=0.0f; _set.vel.y=0.0f; _set.vel.z=0.0f;
		_set.acc.x=0.0f; _set.acc.y=0.0f; _set.acc.z=0.0f;
		
		_set.att[0]=1; _set.att[1]=0; _set.att[2]=0;
		_set.att[3]=0; _set.att[4]=1; _set.att[5]=0;
		_set.att[6]=0; _set.att[7]=0; _set.att[8]=1;
		_set.ang_vel.x=0.0f; _set.ang_vel.y=0.0f; _set.ang_vel.z=0.0f;
		_set.ang_acc.x=0.0f; _set.ang_acc.y=0.0f; _set.ang_acc.z=0.0f;
	}
}

/**********************************************************************************************************
*函 数 名: GetDesiredPos
*功能说明: 获取期望轨迹中的位置信息
*形    参: 位置向量指针
*返 回 值: void
**********************************************************************************************************/
void GetDesiredPos(Vector3f_t *pos)
{
	*pos = _set.pos;
}

/**********************************************************************************************************
*函 数 名: GetDesiredVel
*功能说明: 获取期望轨迹中的速度信息
*形    参: 速度向量指针
*返 回 值: void
**********************************************************************************************************/
void GetDesiredVel(Vector3f_t *vel)
{
	*vel = _set.vel;
}

/**********************************************************************************************************
*函 数 名: GetDesiredAcc
*功能说明: 获取期望轨迹中的加速度信息
*形    参: 加速度向量指针
*返 回 值: void
**********************************************************************************************************/
void GetDesiredAcc(Vector3f_t *acc)
{
	*acc = _set.acc;
}

/**********************************************************************************************************
*函 数 名: GetDesiredAtt
*功能说明: 获取期望轨迹中的姿态信息
*形    参: 方向余弦矩阵数组头
*返 回 值: void
**********************************************************************************************************/
void GetDesiredAtt(float *dcm)
{
	Matrix3_Copy(_set.att, dcm);
}

/**********************************************************************************************************
*函 数 名: GetDesiredAngVel
*功能说明: 获取期望轨迹中的角速度信息
*形    参: 角速度向量指针
*返 回 值: void
**********************************************************************************************************/
void GetDesiredAngVel(Vector3f_t *ang_vel)
{
	*ang_vel = _set.ang_vel;
}

/**********************************************************************************************************
*函 数 名: GetDesiredAngAcc
*功能说明: 获取期望轨迹中的角加速度信息
*形    参: v角加速度向量指针
*返 回 值: void
**********************************************************************************************************/
void GetDesiredAngAcc(Vector3f_t *ang_acc)
{
	*ang_acc = _set.ang_acc;
}
