/**********************************************************************************************************
 * @文件     disturbanceEst.c
 * @说明     扰动估计的总和
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "pd.h"
#include "parameter.h"

#include "controller.h"


_PD_CTRL_t _pd;

/**********************************************************************************************************
*函 数 名: pdCtrlInit
*功能说明: PD形式的几何控制器参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void pdCtrlInit(void)
{
	_pd.Kp[1]= _pd.Kp[2]= _pd.Kp[3]= _pd.Kp[5]= _pd.Kp[6]= _pd.Kp[7]= 0; 
	ParamGetData(CONTROLLER_PD_Kp_X, &(_pd.Kp[0]), 4);
	ParamGetData(CONTROLLER_PD_Kp_Y, &(_pd.Kp[4]), 4);
	ParamGetData(CONTROLLER_PD_Kp_Z, &(_pd.Kp[8]), 4);
	
	_pd.Kv[1]= _pd.Kv[2]= _pd.Kv[3]= _pd.Kv[5]= _pd.Kv[6]= _pd.Kv[7]= 0; 
	ParamGetData(CONTROLLER_PD_Kv_X, &(_pd.Kv[0]), 4);
	ParamGetData(CONTROLLER_PD_Kv_Y, &(_pd.Kv[4]), 4);
	ParamGetData(CONTROLLER_PD_Kv_Z, &(_pd.Kv[8]), 4);
	
	_pd.KR[1]= _pd.KR[2]= _pd.KR[3]= _pd.KR[5]= _pd.KR[6]= _pd.KR[7]= 0;
	ParamGetData(CONTROLLER_PD_KR_X, &(_pd.KR[0]), 4);
	ParamGetData(CONTROLLER_PD_KR_Y, &(_pd.KR[4]), 4);
	ParamGetData(CONTROLLER_PD_KR_Z, &(_pd.KR[8]), 4);
	
	_pd.KW[1]= _pd.KW[2]= _pd.KW[3]= _pd.KW[5]= _pd.KW[6]= _pd.KW[7]= 0;
	ParamGetData(CONTROLLER_PD_KW_X, &(_pd.KW[0]), 4);
	ParamGetData(CONTROLLER_PD_KW_Y, &(_pd.KW[4]), 4);
	ParamGetData(CONTROLLER_PD_KW_Z, &(_pd.KW[8]), 4);
}

/**********************************************************************************************************
*函 数 名: pdCtrlUpdateParam
*功能说明: PD形式的几何控制器参数修改
*形    参: 参数选择param, 参数修改值data
*返 回 值: 无
**********************************************************************************************************/
void pdCtrlUpdateParam(uint16_t param, float data)
{
	switch(param)
	{
		case CONTROLLER_PD_Kp_X:
			_pd.Kp[0] = data;
		break;
		case CONTROLLER_PD_Kp_Y:
			_pd.Kp[4] = data;
		break;
		case CONTROLLER_PD_Kp_Z:
			_pd.Kp[8] = data;
		break;
		case CONTROLLER_PD_Kv_X:
			_pd.Kv[0] = data;
		break;
		case CONTROLLER_PD_Kv_Y:
			_pd.Kv[4] = data;
		break;
		case CONTROLLER_PD_Kv_Z:
			_pd.Kv[8] = data;
		break;
		case CONTROLLER_PD_KR_X:
			_pd.KR[0] = data;
		break;
		case CONTROLLER_PD_KR_Y:
			_pd.KR[4] = data;
		break;
		case CONTROLLER_PD_KR_Z:
			_pd.KR[8] = data;
		break;
		case CONTROLLER_PD_KW_X:
			_pd.KW[0] = data;
		break;
		case CONTROLLER_PD_KW_Y:
			_pd.KW[4] = data;
		break;
		case CONTROLLER_PD_KW_Z:
			_pd.KW[8] = data;
		break;
	}
	
	//在flash中修改相应参数值
	ParamUpdateData(param, &data);
}	



/**********************************************************************************************************
*函 数 名: Attitude_Error
*功能说明: 几何控制中的姿态误差计算
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void Attitude_Error(void)
{
	float R_des_T[9], R_fb_T[9], tmp1[9], tmp2[9], tmp3[9];
	
	Matrix3_Tran(_state.R_des, R_des_T);
	Matrix3_Tran(_state.R_fb, R_fb_T);
	
	Matrix3_Mul(R_des_T, _state.R_fb, tmp1);
	Matrix3_Mul(R_fb_T, _state.R_des, tmp2);
	Matrix3_Sub(tmp1, tmp2, tmp3);
	
	// vee_map
	_state.R_err.x = tmp3[7]/2;
	_state.R_err.y = tmp3[2]/2;
	_state.R_err.z = tmp3[3]/2;
}

/**********************************************************************************************************
*函 数 名: Angular_Rate_Error
*功能说明: 几何控制中的角速度误差计算
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void Angular_Rate_Error(void)
{
	float R_fb_T[9], tmp1[9];
	
	Matrix3_Tran(_state.R_fb, R_fb_T);
	
	Matrix3_Mul(R_fb_T, _state.R_des, tmp1);
	Vector3f_t v1 = Matrix3MulVector3(tmp1, _state.W_des);
	
	_state.W_err = Vector3f_Sub(_state.W_fb, v1);
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
	Vector3f_t v1, v2, v3, v4, v5, v6, v7;
	float R_T[9];
	/***********计算位置和速度误差***********/
	_state.pos_err = Vector3f_Sub(_state.pos_fb, _state.pos_des);
	_state.vel_err = Vector3f_Sub(_state.vel_fb, _state.vel_des);
	
	// each element of Force equation 
	v1 = Matrix3MulVector3(_pd.Kp, _state.pos_err);		// first element
	v1.x=-v1.x; v1.y=-v1.y; v1.z=-v1.z;
	v2 = Matrix3MulVector3(_pd.Kv, _state.vel_err); 	// second element
	v3.x=0.0f; v3.y=0.0f; v3.z=_veh.mass*GRAVITY_ACCEL;	//masss*g*[0;0;1]
	v4.x = _veh.mass * _state.acc_des.x;
	v4.y = _veh.mass * _state.acc_des.y;
	v4.z = _veh.mass * _state.acc_des.z;			// mass.*desired_acceleration
	
	// add all the elements
	v5 = Vector3f_Sub(v1, v2);
	v6 = Vector3f_Add(v5, v3);
	v7 = Vector3f_Add(v6, v4);
	
	// Transpose of attitude matrix
	Matrix3_Tran(_state.R_fb, R_T);
	
	return Matrix3MulVector3(R_T, v7);
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
	Vector3f_t v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
	Vector3f_t alpha_d;
	float R_fb_T[9], W_fb_hat[9];
	
	//中间量计算
	Hat_Map(W_fb_hat, _state.W_fb);
	Matrix3_Tran(_state.R_fb, R_fb_T);
	Attitude_Error();
	Angular_Rate_Error();
	
	//each element of moment function
	v1 = Matrix3MulVector3(_pd.KR, _state.R_err);		//first element
	v1.x=-v1.x; v1.y=-v1.y; v1.z=-v1.z;
	v2 = Matrix3MulVector3(_pd.KW, _state.W_err);		//second element
	v3 = Matrix3MulVector3(_veh.J, _state.W_fb);	//third element
	v4 = Matrix3MulVector3(W_fb_hat, v3);		
	// 计算中间量alpha_d
	v5 = Matrix3MulVector3(_state.R_des, _state.W_des);	// First element
	v6 = Matrix3MulVector3(R_fb_T, v5);
	v7 = Matrix3MulVector3(W_fb_hat, v6);
	v8 = Matrix3MulVector3(_state.R_des, _state.W_dot_des); //Second element
	v9 = Matrix3MulVector3(R_fb_T, v8);
	alpha_d = Vector3f_Sub(v9, v7);
	
	// add all the elements
	v10 = Vector3f_Sub(v1, v2);
	v11 = Vector3f_Add(v10, v4);
	
	// J * alpha_d
	v12 = Matrix3MulVector3(_veh.J, alpha_d);
	
	return Vector3f_Add(v11, v12);
}

/**********************************************************************************************************
*函 数 名: pdCtrlUpdate
*功能说明: PD形式的几何控制器主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void pdCtrlUpdate(void)
{
	_ctrl_only.F_b = ForceCal();
	_ctrl_only.M_b = MomentCal();
	
	_ctrl_only.wrench[0] = _ctrl_only.F_b.x;
	_ctrl_only.wrench[1] = _ctrl_only.F_b.y;
	_ctrl_only.wrench[2] = _ctrl_only.F_b.z;
	_ctrl_only.wrench[3] = _ctrl_only.M_b.x;
	_ctrl_only.wrench[4] = _ctrl_only.M_b.y;
	_ctrl_only.wrench[5] = _ctrl_only.M_b.z;
}



