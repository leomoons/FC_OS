/**********************************************************************************************************
 * @文件     adaptive.c
 * @说明     外界扰动的自适应算法估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.11
**********************************************************************************************************/
#include "adaptive.h"
#include "boardConfig.h"
#include "ahrs.h"
#include "controller.h"

adaptive_t _ada;


/**********************************************************************************************************
*函 数 名: adaptiveInit
*功能说明: 扰动的自适应算法估计初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void adaptiveInit(void)
{
	_ada.gamax[0]=1.0f; _ada.gamax[1]=0.0f; _ada.gamax[2]=0.0f;
	_ada.gamax[3]=0.0f; _ada.gamax[4]=1.0f; _ada.gamax[5]=0.0f;
	_ada.gamax[6]=0.0f; _ada.gamax[7]=0.0f; _ada.gamax[8]=1.0f;
	
	_ada.gamaR[0]=5.0f; _ada.gamaR[1]=0.0f; _ada.gamaR[2]=0.0f;
	_ada.gamaR[3]=0.0f; _ada.gamaR[4]=5.0f; _ada.gamaR[5]=0.0f;
	_ada.gamaR[6]=0.0f; _ada.gamaR[7]=0.0f; _ada.gamaR[8]=5.0f;
	
	_ada.Wx[0]=1.0f; _ada.Wx[1]=0.0f; _ada.Wx[2]=0.0f;
	_ada.Wx[3]=0.0f; _ada.Wx[4]=1.0f; _ada.Wx[5]=0.0f;
	_ada.Wx[6]=0.0f; _ada.Wx[7]=0.0f; _ada.Wx[8]=1.0f;
	
	_ada.Wx_T[0]=1.0f; _ada.Wx_T[1]=0.0f; _ada.Wx_T[2]=0.0f;
	_ada.Wx_T[3]=0.0f; _ada.Wx_T[4]=1.0f; _ada.Wx_T[5]=0.0f;
	_ada.Wx_T[6]=0.0f; _ada.Wx_T[7]=0.0f; _ada.Wx_T[8]=1.0f;
	
	_ada.WR[0]=1.0f; _ada.WR[1]=0.0f; _ada.WR[2]=0.0f;
	_ada.WR[3]=0.0f; _ada.WR[4]=1.0f; _ada.WR[5]=0.0f;
	_ada.WR[6]=0.0f; _ada.WR[7]=0.0f; _ada.WR[8]=1.0f;
	
	_ada.WR_T[0]=1.0f; _ada.WR_T[1]=0.0f; _ada.WR_T[2]=0.0f;
	_ada.WR_T[3]=0.0f; _ada.WR_T[4]=1.0f; _ada.WR_T[5]=0.0f;
	_ada.WR_T[6]=0.0f; _ada.WR_T[7]=0.0f; _ada.WR_T[8]=1.0f;
	
	_ada.cax[0]=1.0f; _ada.cax[1]=0.0f; _ada.cax[2]=0.0f;
	_ada.cax[3]=0.0f; _ada.cax[4]=1.0f; _ada.cax[5]=0.0f;
	_ada.cax[6]=0.0f; _ada.cax[7]=0.0f; _ada.cax[8]=1.0f;
	
	_ada.caR[0]=1.0f; _ada.caR[1]=0.0f; _ada.caR[2]=0.0f;
	_ada.caR[3]=0.0f; _ada.caR[4]=1.0f; _ada.caR[5]=0.0f;
	_ada.caR[6]=0.0f; _ada.caR[7]=0.0f; _ada.caR[8]=1.0f;
	
	_ada.Bx = 2.0f;
	_ada.BR = 5.0f;
}

/**********************************************************************************************************
*函 数 名: adaptiveForceUpdate
*功能说明: 自适应算法 扰动力观测更迭代
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void adaptiveForceUpdate(void)
{
	static uint64_t pT8;
	float dT_s = (GetSysTimeUs() - pT8) * 1e-6;
	dT_s = ConstrainFloat(dT_s , 0.0005, 0.002);
	pT8 = GetSysTimeUs();
	
	float R_fb[9], R_fb_T[9];
	GetDCM(R_fb);
	Matrix3_Tran(R_fb, R_fb_T);
	
	/*********定义中间变量**********/
	Vector3f_t tmp1, tmp2, tmp3, tmp4, tmp5;
	
	tmp1 = Matrix3MulVector3(_ada.cax, _state.pos_err);
	tmp2 = Vector3f_Add(_state.vel_err, tmp1);
	
	tmp3 = Matrix3MulVector3(_ada.Wx_T, tmp2);
	tmp4 = Matrix3MulVector3(_ada.gamax, tmp3);
	
	tmp5 = Matrix3MulVector3(_ada.Wx, tmp4);
	
	_ada._est.F_b = Matrix3MulVector3(R_fb_T, tmp5);
}

/**********************************************************************************************************
*函 数 名: adaptiveMomentUpdate
*功能说明: 自适应算法 扰动力矩观测更迭代
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void adaptiveMomentUpdate(void)
{
	static uint64_t pT9;
	float dT_s = (GetSysTimeUs() - pT9) * 1e-6;
	dT_s = ConstrainFloat(dT_s , 0.0005, 0.002);
	pT9 = GetSysTimeUs();
	
	/*********定义中间变量**********/
	Vector3f_t tmp1, tmp2, tmp3, tmp4;
	
	tmp1 = Matrix3MulVector3(_ada.caR, _state.R_err);
	tmp2 = Vector3f_Add(_state.W_err, tmp1);
	
	tmp3 = Matrix3MulVector3(_ada.WR_T, tmp2);
	tmp4 = Matrix3MulVector3(_ada.gamaR, tmp3);
	
	_ada._est.M_b = Matrix3MulVector3(_ada.WR, tmp4);
}

/**********************************************************************************************************
*函 数 名: adaptiveUpdate
*功能说明: 扰动的自适应算法估计主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void adaptiveUpdate(void)
{
	adaptiveForceUpdate();
	adaptiveMomentUpdate();
	
}

