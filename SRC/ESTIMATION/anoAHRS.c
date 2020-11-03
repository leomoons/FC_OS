/**********************************************************************************************************
 * @文件     anoAHRS.c
 * @说明     姿态估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "anoAHRS.h"
#include "boardConfig.h"

#define USE_MAG


/*参考坐标，定义为ANO坐标*

俯视，机头方向为x正方向
     +x
     |
 +y--|--
     |
		 
*/	
const float grav_const = 9.81;
ANO_ahrs_t _ano;
ANO_state_st ano_state;


/**********************************************************************************************************
*函 数 名: AnoAHRSinit
*功能说明: ANO姿态解算初始化函数
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void AnoAHRSinit(void)
{
	ano_state.akp = 0.2f;
	ano_state.aki = 0.01f;
	ano_state.mkp = 1.0f;	//0.1f
	
	_ano.quat[0] = 1;
	_ano.quat[1] = 0;
	_ano.quat[2] = 0;
	_ano.quat[3] = 0;
}


/**********************************************************************************************************
*函 数 名: AttitudeEstimate
*功能说明: 姿态估计主函数
*形    参: 角速度测量值 加速度测量值 磁场强度测量值
*返 回 值: 无
**********************************************************************************************************/
float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3; 
float acc_norm_l, q_norm_l;
Vector3f_t acc_norm, x_vec, y_vec, z_vec;
Vector3f_t vec_err, vec_err_i;
float mag_2d_vec_w[2][2] = {{1,0},{1,0}};	//地理坐标中，水平面磁场方向恒为南北(1,0)
float mag_yaw_err, mag_err_dot_product;
Vector3f_t d_angle;
void AnoAHRSupdate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag)
{
	
	
	static uint64_t previousT;
	float dT_s = (GetSysTimeUs() - previousT) * 1e-6;
	dT_s = ConstrainFloat(dT_s, 0.0005, 0.002);
	previousT = GetSysTimeUs();
	
	q0q1 = _ano.quat[0] * _ano.quat[1];
	q0q2 = _ano.quat[0] * _ano.quat[2];
	q1q1 = _ano.quat[1] * _ano.quat[1];
	q1q3 = _ano.quat[1] * _ano.quat[3];
	q2q2 = _ano.quat[2] * _ano.quat[2];
	q2q3 = _ano.quat[2] * _ano.quat[3];
	q3q3 = _ano.quat[3] * _ano.quat[3];
	q1q2 = _ano.quat[1] * _ano.quat[2];
	q0q3 = _ano.quat[0] * _ano.quat[3];
	
	_ano.gra_acc.x = -acc.x;
	_ano.gra_acc.y = -acc.y;
	_ano.gra_acc.z = -acc.z;
	
	//加速度归一化
	acc_norm_l = Pythagorous3(_ano.gra_acc.x, _ano.gra_acc.y, _ano.gra_acc.z);
	acc_norm.x = _ano.gra_acc.x / acc_norm_l;
	acc_norm.y = _ano.gra_acc.y / acc_norm_l;
	acc_norm.z = _ano.gra_acc.z / acc_norm_l;
	
	
	//机体坐标下的x方向向量，单位化
	_ano.dcm[0] = x_vec.x = 1 - (2*q2q2 + 2*q3q3);
	_ano.dcm[1] = x_vec.y = 2*q1q2 - 2*q0q3;
	_ano.dcm[2] = x_vec.z = 2*q1q3 + 2*q0q2;
		
	_ano.dcm[3] = y_vec.x = 2*q1q2 + 2*q0q3;
	_ano.dcm[4] = y_vec.y = 1 - (2*q1q1 + 2*q3q3);
	_ano.dcm[5] = y_vec.z = 2*q2q3 - 2*q0q1;
		
	_ano.dcm[6] = z_vec.x = 2*q1q3 - 2*q0q2;
	_ano.dcm[7] = z_vec.y = 2*q2q3 + 2*q0q1;
	_ano.dcm[8] = z_vec.z = 1 - (2*q1q1 + 2*q2q2);
	
	
	//测量值与等效重力向量的叉积（计算向量误差）
	vec_err.x =  (acc_norm.y * z_vec.z - z_vec.y * acc_norm.z);
	vec_err.y = -(acc_norm.x * z_vec.z - z_vec.x * acc_norm.z);
	vec_err.z = -(acc_norm.y * z_vec.x - z_vec.y * acc_norm.x);
	
	
#ifdef USE_MAG
	//计算磁场向量和测量值的误差
	if(!(mag.x==0 && mag.y==0 && mag.z==0))
	{
		//把机体坐标下的罗盘数据转换到地理坐标下
		_ano.mag_w = Matrix3MulVector3(_ano.dcm, mag);
		//计算磁场方向向量归一化的模
		float mag_norm = Pythagorous2(_ano.mag_w.x, _ano.mag_w.y);
		//计算南北朝向向量
		mag_2d_vec_w[1][0] = _ano.mag_w.x / mag_norm;
		mag_2d_vec_w[1][1] = _ano.mag_w.y / mag_norm;
		//计算南北朝向误差(2维叉乘)，地理坐标中，水平面磁场方向向量应恒为南北(1,0)
		mag_yaw_err = mag_2d_vec_w[1][0]*mag_2d_vec_w[0][1] - mag_2d_vec_w[1][1]*mag_2d_vec_w[0][0];
		//计算南北朝向向量点乘，判断同向或反向
		mag_err_dot_product = mag_2d_vec_w[1][0]*mag_2d_vec_w[0][0] + mag_2d_vec_w[1][1]*mag_2d_vec_w[0][1];
		//若反向，直接给最大误差
		if(mag_err_dot_product<0)
		{
			mag_yaw_err = sign(mag_yaw_err) * 1.0f;
		}
	}
#endif
	
	//误差积分
	vec_err_i.x = LIMIT(vec_err.x, -0.1f, 0.1f) * dT_s * ano_state.aki;
	vec_err_i.y = LIMIT(vec_err.y, -0.1f, 0.1f) * dT_s * ano_state.aki;
	vec_err_i.z = LIMIT(vec_err.z, -0.1f, 0.1f) * dT_s * ano_state.aki;

	//构造增量旋转（含融合纠正）
#ifdef USE_MAG	
	d_angle.x = (gyro.x+(vec_err.x+vec_err_i.x)*ano_state.akp + mag_yaw_err*_ano.dcm[6]*ano_state.mkp) * dT_s/2;
	d_angle.y = (gyro.y+(vec_err.y+vec_err_i.y)*ano_state.akp + mag_yaw_err*_ano.dcm[7]*ano_state.mkp) * dT_s/2;
	d_angle.z = (gyro.z+(vec_err.z+vec_err_i.z)*ano_state.akp + mag_yaw_err*_ano.dcm[8]*ano_state.mkp) * dT_s/2;
#else
	d_angle.x = (gyro.x+(vec_err.x+vec_err_i.x)*ano_state.akp) * dT_s/2;
	d_angle.y = (gyro.y+(vec_err.y+vec_err_i.y)*ano_state.akp) * dT_s/2;
	d_angle.z = (gyro.z+(vec_err.z+vec_err_i.z)*ano_state.akp) * dT_s/2;
#endif
	//更新姿态
	_ano.quat[0] = _ano.quat[0]			  - _ano.quat[1]*d_angle.x - _ano.quat[2]*d_angle.y - _ano.quat[3]*d_angle.z;
	_ano.quat[1] = _ano.quat[0]*d_angle.x + _ano.quat[1]           + _ano.quat[2]*d_angle.z - _ano.quat[3]*d_angle.y;
	_ano.quat[2] = _ano.quat[0]*d_angle.y - _ano.quat[1]*d_angle.z + _ano.quat[2]           + _ano.quat[3]*d_angle.x;
	_ano.quat[3] = _ano.quat[0]*d_angle.z + _ano.quat[1]*d_angle.y - _ano.quat[2]*d_angle.x + _ano.quat[3];
	
	q_norm_l = Pythagorous4(_ano.quat[0], _ano.quat[1], _ano.quat[2], _ano.quat[3]);
	_ano.quat[0] /= q_norm_l;
	_ano.quat[1] /= q_norm_l;
	_ano.quat[2] /= q_norm_l;
	_ano.quat[3] /= q_norm_l;
	
	/*******************修正开关，根据飞行状态来调整迭代系数*********************/
	//TODO:
}

/**********************************************************************************************************
*函 数 名: AnoGetDCM
*功能说明: 获取方向余弦矩阵的值
*形    参: dcm数组头
*返 回 值: void
**********************************************************************************************************/
void AnoGetDCM(float* dcm)
{
	Matrix3_Copy(_ano.dcm, dcm);
}

/**********************************************************************************************************
*函 数 名: AnoGetEuler
*功能说明: 获取欧拉角
*形    参: 无
*返 回 值: ZYX旋转顺序欧拉角(rad)
**********************************************************************************************************/
void AnoGetEuler(Vector3f_t *euler)
{
	float quat[4] = {_ano.quat[0], _ano.quat[1], _ano.quat[2], _ano.quat[3]};
	_ano.euler = *euler = Quater_to_Euler(quat);
	euler->z = -euler->z;
	euler->y = -euler->y;
	
//	float t_temp = LIMIT(1 - Sq(_ano.dcm[6]), 0, 1);
//	
//	if(abs(_ano.dcm[8])>0.05f)//避免奇点的运算
//	{
//			_ano.euler.x =  fast_atan2(_ano.dcm[6],Sqrt(t_temp));
//			_ano.euler.y =  fast_atan2(_ano.dcm[7], _ano.dcm[8]); 
//		    _ano.euler.z = -fast_atan2(_ano.dcm[3], _ano.dcm[0]); 
//	}
//	*euler = _ano.euler;
}


