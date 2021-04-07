/**********************************************************************************************************
 * @文件     ndob.c
 * @说明     外界扰动的非线性扰动观测器算法估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "ndob.h"
#include "ahrs.h"
#include "gyroscope.h"
#include "boardConfig.h"
#include "controller.h"
#include "optitrack.h"

nDOB_t _dob;

/**********************************************************************************************************
*函 数 名: ndobInit
*功能说明: 非线性扰动观测器初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ndobInit(void)
{
	_dob.Kox[0] =-15.0f; _dob.Kox[1] =     0; _dob.Kox[2] =      0;
	_dob.Kox[3] =     0; _dob.Kox[4] =-15.0f; _dob.Kox[5] =      0;
	_dob.Kox[6] =     0; _dob.Kox[7] =     0; _dob.Kox[8] = -15.0f;
	
	_dob.KoR[0] =-15.0f; _dob.KoR[1] =     0; _dob.KoR[2] =     0;
	_dob.KoR[3] =     0; _dob.KoR[4] =-15.0f; _dob.KoR[5] =     0;
	_dob.KoR[6] =     0; _dob.KoR[7] =     0; _dob.KoR[8] =-15.0f;
	
	
}

/**********************************************************************************************************
*函 数 名: ndobForceUpdate
*功能说明: 非线性扰动力观测更迭代
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ndobForceUpdate(void)
{
	static uint64_t pT6;
	float dT_s = (GetSysTimeUs() - pT6) * 1e-6;
	dT_s = ConstrainFloat(dT_s , 0.0005, 0.002);
	pT6 = GetSysTimeUs();
	
	static Vector3f_t zx_dot, zx, pv_nega;
	
	//获得惯性坐标下的速度反馈和姿态转换矩阵
	Vector3f_t v_fb = GetOptiVel();
	float R_fb[9], R_fb_T[9];
	GetDCM(R_fb);
	Matrix3_Tran(R_fb, R_fb_T);
	
	/************定义中间变量***************/
	Vector3f_t tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
	
	pv_nega = Matrix3MulVector3(_dob.Kox, v_fb);		//second element in the brackets
	tmp1.x=0.0f; tmp1.y=0.0f; tmp1.z=GRAVITY_ACCEL;	//third element in the brackets
	tmp2 = Matrix3MulVector3(R_fb, _ctrl.F_b);				//fourth element in the brackets
	
	tmp3 = Vector3f_Sub(zx, pv_nega);
	tmp4 = Vector3f_Sub(tmp3, tmp1);
	tmp5 = Vector3f_Add(tmp4, tmp2);
	
	tmp6.x = tmp5.x/_veh.mass; tmp6.y = tmp5.y/_veh.mass; tmp6.z = tmp5.z/_veh.mass; 
	zx_dot = Matrix3MulVector3(_dob.Kox, tmp6);
	
	zx.x += (zx_dot.x * dT_s); zx.y += (zx_dot.y * dT_s); zx.z += (zx_dot.z * dT_s); 	//integration
	tmp7 = Vector3f_Sub(zx, pv_nega);	//Force estimation in inerita frame
	
	_dob._est.F_b = Matrix3MulVector3(R_fb_T, tmp7);
}

/**********************************************************************************************************
*函 数 名: ndobMomentUpdate
*功能说明: 非线性姿态扰动力矩观测更迭代
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ndobMomentUpdate(void)
{
	static uint64_t pT7;
	float dT_s = (GetSysTimeUs() - pT7) * 1e-6;
	dT_s = ConstrainFloat(dT_s , 0.0005, 0.002);
	pT7 = GetSysTimeUs();
	
	static Vector3f_t zR_dot, zR, pW_nega;
	
	Vector3f_t W_fb = GyroLpfGetData();
	
	/********定义中间变量***********/
	Vector3f_t tmp1, tmp2, tmp3, tmp4, tmp5, tmp6;
	
	pW_nega = Matrix3MulVector3(_dob.KoR, W_fb);		//second element in the brackets
	tmp1 = Matrix3MulVector3(_veh.J, W_fb);
	tmp2 = VectorCrossProduct(W_fb, tmp1);			//third element in the brackets
	
	
	tmp3 = Vector3f_Sub(zR, pW_nega);
	tmp4 = Vector3f_Sub(tmp3, tmp2);
	tmp5 = Vector3f_Add(tmp4, _ctrl.M_b);

	tmp6 = Matrix3MulVector3(_veh.J_inv, tmp5);
	zR_dot = Matrix3MulVector3(_dob.KoR, tmp6);
	
	zR.x += (zR_dot.x * dT_s); zR.y += (zR_dot.y * dT_s); zR.z += (zR_dot.z * dT_s); 	//integration
	_dob._est.M_b = Vector3f_Sub(zR, pW_nega);
}

/**********************************************************************************************************
*函 数 名: ndobUpdate
*功能说明: 非线性扰动观测器主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ndobUpdate(void)
{	
	ndobForceUpdate();
	ndobMomentUpdate();
}
