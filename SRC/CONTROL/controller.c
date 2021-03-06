/**********************************************************************************************************
 * @文件     controller.c
 * @说明     控制器总和
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "controller.h"
#include "disturbanceEst.h"
#include "geoCtrl.h"

vehicle_para _veh;
control_set_t _ctrl;



enum
{
	PD,
	PDObserver
};

#define CONTROLLER PD


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
		GeoControllerInit();
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
	if(CONTROLLER == PD)
	{
		GeoCtrlUpdate();
	}
	//estimatorUpdate();
	
//	_ctrl.F_b = Vector3f_Add(_geo.ctrl.F_b, _est.F_b);
//	_ctrl.M_b = Vector3f_Add(_geo.ctrl.M_b, _est.M_b);
	
	_ctrl.F_b = _geo.ctrl.F_b;
	_ctrl.M_b = _geo.ctrl.M_b;
	
	_ctrl.wrench[0] = _ctrl.F_b.x;
	_ctrl.wrench[1] = _ctrl.F_b.y;
	_ctrl.wrench[2] = _ctrl.F_b.z;
	_ctrl.wrench[3] = _ctrl.M_b.x;
	_ctrl.wrench[4] = _ctrl.M_b.y;
	_ctrl.wrench[5] = _ctrl.M_b.z;
}
