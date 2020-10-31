/**********************************************************************************************************
 * @文件     anoAHRS.c
 * @说明     姿态估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "anoAHRS.h"
#include "boardConfig.h"

//#define USE_MAG


/*参考坐标，定义为ANO坐标*

俯视，机头方向为x正方向
     +x
     |
 +y--|--
     |
		 
*/	
const float grav_const = 9.81;
_imu_data_st imu;
_imu_state_st imu_state;


/**********************************************************************************************************
*函 数 名: AnoAHRSinit
*功能说明: 姿态估计初始化函数
*形    参: void
*返 回 值: void
**********************************************************************************************************/
void AnoAHRSinit(void)
{
	imu_state.akp = 0.2f;
	imu_state.aki = 0.01f;
	imu_state.mkp = 0.1f;
	
	imu.w = 1;
	imu.x = 0;
	imu.y = 0;
	imu.z = 0;
}

/**********************************************************************************************************
*函 数 名: CalculateEuler
*功能说明: 转换成欧拉角
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void CalculateEuler(void)
{
	float tmp = LIMIT(1-Sq(imu.dcMat[6]), 0, 1);
	
	if(abs(imu.dcMat[8]) > 0.0f)	//避免奇点的运算
	{
		imu.eulerAngle.x = fast_atan2(imu.dcMat[6], Sqrt(tmp));
		imu.eulerAngle.y = fast_atan2(imu.dcMat[7], imu.dcMat[8]);
		imu.eulerAngle.z =-fast_atan2(imu.dcMat[3], imu.dcMat[0]);
	}

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
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;
	deltaT = ConstrainFloat(deltaT, 0.0005, 0.002);
	previousT = GetSysTimeUs();
	
	q0q1 = imu.w * imu.x;
	q0q2 = imu.w * imu.y;
	q1q1 = imu.x * imu.x;
	q1q3 = imu.x * imu.z;
	q2q2 = imu.y * imu.y;
	q2q3 = imu.y * imu.z;
	q3q3 = imu.z * imu.z;
	q1q2 = imu.x * imu.y;
	q0q3 = imu.w * imu.z;
	
	//加速度归一化
	acc_norm_l = Pythagorous3(acc.x, acc.y, acc.z);
	acc_norm.x = acc.x / acc_norm_l;
	acc_norm.y = acc.y / acc_norm_l;
	acc_norm.z = acc.z / acc_norm_l;
	
	
	//机体坐标下的x方向向量，单位化
	imu.dcMat[0] = x_vec.x = 1 - (2*q2q2 + 2*q3q3);
	imu.dcMat[1] = x_vec.y = 2*q1q2 - 2*q0q3;
	imu.dcMat[2] = x_vec.z = 2*q1q3 + 2*q0q2;
		
	imu.dcMat[3] = y_vec.x = 2*q1q2 + 2*q0q3;
	imu.dcMat[4] = y_vec.y = 1 - (2*q1q1 + 2*q3q3);
	imu.dcMat[5] = y_vec.z = 2*q2q3 - 2*q0q1;
		
	imu.dcMat[6] = z_vec.x = 2*q1q3 - 2*q0q2;
	imu.dcMat[7] = z_vec.y = 2*q2q3 + 2*q0q1;
	imu.dcMat[8] = z_vec.z = 1 - (2*q1q1 + 2*q2q2);
	
	
	//转换出欧拉角
	CalculateEuler();
	
	//计算机体坐标系下的运动加速度。（与姿态解算无关）
	imu.acc_b.x = acc_norm.x - imu.dcMat[2];
	imu.acc_b.y = acc_norm.y - imu.dcMat[5];
	imu.acc_b.z = acc_norm.z - imu.dcMat[8];
	
	//计算世界坐标系下的运动加速度。坐标系为北西天
	imu.acc_w = Matrix3MulVector3(imu.dcMat, imu.acc_b);
	
	
	//测量值与等效重力向量的叉积（计算向量误差）
	vec_err.x =  (acc_norm.y * imu.dcMat[8] - imu.dcMat[7] * acc_norm.z);
	vec_err.y = -(acc_norm.x * imu.dcMat[8] - imu.dcMat[6] * acc_norm.z);
	vec_err.z = -(acc_norm.y * imu.dcMat[6] - imu.dcMat[7] * acc_norm.x);
	
	
#ifdef USE_MAG
	//计算磁场向量和测量值的误差
	if(!(mag.x==0 && mag.y==0 && mag.z==0))
	{
		//把机体坐标下的罗盘数据转换到地理坐标下
		imu.mag_w = Matrix3MulVector3(imu.dcMat, mag);
		//计算磁场方向向量归一化的模
		float mag_norm = Pythagorous2(imu.mag_w.x, imu.mag_w.y);
		//计算南北朝向向量
		mag_2d_vec_w[1][0] = imu.mag_w.x / mag_norm;
		mag_2d_vec_w[1][1] = imu.mag_w.y / mag_norm;
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
	
	//误差值如果太大，则清零
	if(acc_norm_l>1.06f || acc_norm_l<0.9f)
	{
		vec_err.x = vec_err.y = vec_err.z = 0;
	}
	
	//误差积分
	vec_err_i.x = LIMIT(vec_err.x, -0.1f, 0.1f) * deltaT * imu_state.aki;
	vec_err_i.y = LIMIT(vec_err.y, -0.1f, 0.1f) * deltaT * imu_state.aki;
	vec_err_i.z = LIMIT(vec_err.z, -0.1f, 0.1f) * deltaT * imu_state.aki;

	//构造增量旋转（含融合纠正）
#ifdef USE_MAG	
	d_angle.x = (gyro.x+(vec_err.x+vec_err_i.x)*imu_state.akp + mag_yaw_err*imu.dcMat[6]*imu_state.mkp) * deltaT/2;
	d_angle.y = (gyro.y+(vec_err.y+vec_err_i.y)*imu_state.akp + mag_yaw_err*imu.dcMat[7]*imu_state.mkp) * deltaT/2;
	d_angle.z = (gyro.z+(vec_err.z+vec_err_i.z)*imu_state.akp + mag_yaw_err*imu.dcMat[8]*imu_state.mkp) * deltaT/2;
#else
	d_angle.x = (gyro.x+(vec_err.x+vec_err_i.x)*imu_state.akp) * deltaT/2;
	d_angle.y = (gyro.y+(vec_err.y+vec_err_i.y)*imu_state.akp) * deltaT/2;
	d_angle.z = (gyro.z+(vec_err.z+vec_err_i.z)*imu_state.akp) * deltaT/2;
#endif
	//更新姿态
	imu.w = imu.w			      - imu.x*d_angle.x - imu.y*d_angle.y - imu.z*d_angle.z;
	imu.x = imu.w*d_angle.x + imu.x           + imu.y*d_angle.z - imu.z*d_angle.y;
	imu.y = imu.w*d_angle.y + imu.x*d_angle.z + imu.y           - imu.z*d_angle.x;
	imu.z = imu.w*d_angle.z + imu.x*d_angle.y + imu.y*d_angle.x - imu.z;
	
	q_norm_l = Pythagorous4(imu.w, imu.x, imu.y, imu.z);
	imu.w /= q_norm_l;
	imu.x /= q_norm_l;
	imu.y /= q_norm_l;
	imu.z /= q_norm_l;
	
	/*******************修正开关，根据飞行状态来调整迭代系数*********************/
	//TODO:
}

/**********************************************************************************************************
*函 数 名: AnoGetDCM
*功能说明: 获取方向余弦矩阵的值
*形    参: void
*返 回 值: 数组指针
**********************************************************************************************************/
float* AnoGetDCM(void)
{
	return imu.dcMat;
}

