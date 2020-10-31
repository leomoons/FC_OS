/**********************************************************************************************************
 * @文件     transition.c
 * @说明     姿态表示方法互相转换
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "transition.h"


/**********************************************************************************************************
*函 数 名: Quater_to_DCM
*功能说明: 四元数转换成方向余弦矩阵
*形    参: dcm数组头， quaternion数组头
*返 回 值: void
**********************************************************************************************************/
void Quater_to_DCM(float* dcm, float* quat)
{
	dcm[0] = 1 - 2*(Sq(quat[2])+Sq(quat[3]));
	dcm[1] = 2 * (quat[1]*quat[2] - quat[0]*quat[3]);
	dcm[2] = 2 * (quat[0]*quat[2] + quat[1]*quat[3]);
	dcm[3] = 2 * (quat[1]*quat[2] + quat[0]*quat[3]);
	dcm[4] = 1 - 2*(Sq(quat[1])+Sq(quat[3]));
	dcm[5] = 2 * (quat[2]*quat[3] - quat[0]*quat[1]);
	dcm[6] = 2 * (quat[1]*quat[3] - quat[0]*quat[2]);
	dcm[7] = 2 * (quat[0]*quat[1] + quat[2]*quat[3]);
	dcm[8] = 1 - 2*(Sq(quat[1])+Sq(quat[2]));
}

/**********************************************************************************************************
*函 数 名: DCM_to_Quater
*功能说明: 方向余弦矩阵转换成四元数  
*参    考： http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
*形    参: quaternion数组头， dcm数组头
*返 回 值: 无
**********************************************************************************************************/
void DCM_to_Quater(float* quat, float* dcm)
{
	float trace = dcm[0] + dcm[4] +dcm[8];
	if(trace > 0.0f)
	{
		float s = 0.5f / Sqrt(trace + 1.0f);
		quat[0] = 0.25f / s;
		quat[1] = (dcm[7] - dcm[5]) * s;
		quat[2] = (dcm[2] - dcm[6]) * s;
		quat[3] = (dcm[3] - dcm[1]) * s;
	}
	else
	{
		if(dcm[0]>dcm[4] && dcm[0]>dcm[8])
		{
			float s = 2.0f * Sqrt(1.0f + dcm[0] - dcm[4] - dcm[8]);
			quat[0] = (dcm[7] - dcm[5]) / s;
			quat[1] = 0.25f * s;
			quat[2] = (dcm[1] + dcm[3]) / s;
			quat[3] = (dcm[2] + dcm[6]) / s;
		}
		else if(dcm[4] > dcm[8])
		{
			float s = 2.0f * Sqrt(1.0f + dcm[4] - dcm[0] - dcm[8]);
			quat[0] = (dcm[2] - dcm[6]) / s;
			quat[1] = (dcm[1] + dcm[3]) / s;
			quat[2] = 0.25f * s;
			quat[3] = (dcm[5] + dcm[7]) / s;
		}
		else
		{
			float s = 2.0f * Sqrt(1.0f + dcm[8] - dcm[0] - dcm[4]);
			quat[0] = (dcm[3] - dcm[1]) / s;
			quat[1] = (dcm[2] + dcm[6]) / s;
			quat[2] = (dcm[5] + dcm[7]) / s;
			quat[3] = 0.25f * s;
		}
	}	
}


/**********************************************************************************************************
*函 数 名: Euler_to_DCM
*功能说明: 欧拉角转换成方向余弦矩阵(机体系到参考系)
*形    参: dcm数组头， 欧拉角向量
*返 回 值: void
**********************************************************************************************************/
void Euler_to_DCM(float* dcm, Vector3f_t euler)
{
	Vector3f_t sin, cos;
	sin.x = sinf(euler.x);
	sin.y = sinf(euler.y);
	sin.z = sinf(euler.z);
	cos.x = cosf(euler.x);
	cos.y = cosf(euler.y);
	cos.z = cosf(euler.z);
	
	dcm[0] = cos.y * cos.z;
	dcm[1] = sin.x * sin.y * cos.z - cos.x * sin.z;
	dcm[2] = cos.x * sin.y * cos.z + sin.x * sin.z;
	dcm[3] = cos.y * sin.z;
	dcm[4] = sin.x * sin.y * sin.z + cos.x * cos.z;
	dcm[5] = cos.x * sin.y * sin.z - sin.x * cos.z;
	dcm[6] =-sin.y;
	dcm[7] = sin.x * cos.y; 
	dcm[8] = cos.x * cos.y;
}

/**********************************************************************************************************
*函 数 名: Euler_to_DCM_T
*功能说明: 欧拉角转换成方向余弦矩阵(参考系到机体系)
*形    参: dcm数组头， 欧拉角向量
*返 回 值: void
**********************************************************************************************************/
void Euler_to_DCM_T(float* dcm, Vector3f_t euler)
{
	Vector3f_t sin, cos;
	sin.x = sinf(euler.x);
	sin.y = sinf(euler.y);
	sin.z = sinf(euler.z);
	cos.x = cosf(euler.x);
	cos.y = cosf(euler.y);
	cos.z = cosf(euler.z);
	
	dcm[0] = cos.y * cos.z;
	dcm[1] = cos.y * sin.z;
	dcm[2] = -sin.y;
	dcm[3] = sin.x * sin.y * cos.z - cos.x * sin.z;
	dcm[4] = sin.x * sin.y * sin.z + cos.x * cos.z;
	dcm[5] = sin.x * cos.y; 
	dcm[6] = cos.x * sin.y * cos.z + sin.x * sin.z;
	dcm[7] = cos.x * sin.y * sin.z - sin.x * cos.z;
	dcm[8] = cos.x * cos.y;
}

/**********************************************************************************************************
*函 数 名: DCM_to_Euler
*功能说明: 方向余弦矩阵(机体系到参考系)转换成欧拉角
*参   考： https://www.learnopencv.com/rotation-matrix-to-euler-angles/
*形    参: dcm数组头
*返 回 值: 欧拉角向量
**********************************************************************************************************/
Vector3f_t DCM_to_Euler(float* dcm)
{
	Vector3f_t euler;
	euler.x = 0.0f; euler.y = 0.0f; euler.z = 0.0f;
	

	float sy = Sqrt(Sq(dcm[0]) + Sq(dcm[3]));
	if(sy < 1e-6f)
	{
		euler.x = fast_atan2(-dcm[5], dcm[4]);
		euler.y = fast_atan2(-dcm[6], sy);
		euler.z = 0;
	}
	else
	{
		euler.x = fast_atan2(dcm[7], dcm[8]);
		euler.y = fast_atan2(-dcm[6], sy);
		euler.z = fast_atan2(dcm[3], dcm[0]);
	}
	return euler;
}

/**********************************************************************************************************
*函 数 名: Euler_to_Quater
*功能说明: 欧拉角转换成四元数
*形    参: quaternion数组头， 欧拉角向量
*返 回 值: void
**********************************************************************************************************/
void Euler_to_Quater(float* quat, Vector3f_t euler)
{
	Vector3f_t sin, cos;
	sin.x = sinf(euler.x * 0.5f);
	sin.y = sinf(euler.y * 0.5f);
	sin.z = sinf(euler.z * 0.5f);
	cos.x = cosf(euler.x * 0.5f);
	cos.y = cosf(euler.y * 0.5f);
	cos.z = cosf(euler.z * 0.5f);
	
	quat[0] = cos.x * cos.y * cos.z + sin.x * sin.y * sin.z;
	quat[1] = sin.x * cos.y * cos.z - cos.x * sin.y * sin.z;
	quat[2] = cos.x * sin.y * cos.z + sin.x * cos.y * sin.z;
	quat[3] = cos.x * cos.y * sin.z - sin.x * sin.y * cos.z;
}

/**********************************************************************************************************
*函 数 名: Quater_to_Euler
*功能说明: 四元数转换成欧拉角
*参    考： https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
*形    参: quaternion数组头
*返 回 值: 欧拉角向量
**********************************************************************************************************/
Vector3f_t Quater_to_Euler(float* quat)
{
	Vector3f_t euler;
	
	//roll(x-axis rotation)
	float sinr_cosp = 2 * (quat[0]*quat[1] + quat[2]*quat[3]);
	float cosr_cosp = 1 - 2*(Sq(quat[1]) + Sq(quat[2]));
	euler.x = fast_atan2(sinr_cosp, cosr_cosp);
	
	//pitch (y-axis rotation)
	float sinp = 2 * (quat[0]*quat[2] - quat[3]*quat[1]);
	if(abs(sinp) >= 1)
		euler.y = M_PI / 2 * abs(sinp);
	else
		euler.y = SafeArcsin(sinp);
	
	//yaw(z-axis rotation)
	float siny_cosp = 2 * (quat[0]*quat[3] + quat[1]*quat[2]);
	float cosy_cosp = 1 - 2*(Sq(quat[2]) + Sq(quat[3]));
	euler.z = fast_atan2(siny_cosp, cosy_cosp);
	
	return euler;
} 
