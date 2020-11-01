 /**********************************************************************************************************
 * @文件     mahonyAHRS.c
 * @说明     Mahony的基于四元数的姿态估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "mahonyAHRS.h"
#include "boardConfig.h"
#include "ahrs.h"

#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

volatile float twoKp;											// 2 * proportional gain (Kp)
volatile float twoKi;											// 2 * integral gain (Ki)					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx,  integralFBy, integralFBz;			// integral error terms scaled by Ki

AHRS_t ahrs;


/**********************************************************************************************************
*函 数 名: MahonyAHRSinit
*功能说明: 初始化Mahony滤波器
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MahonyAHRSinit(void)
{
	ahrs.quat[0] = 1.0f;
	ahrs.quat[1] = 0.0f;
	ahrs.quat[2] = 0.0f;
	ahrs.quat[3] = 0.0f;
	
	twoKp = twoKpDef;
	twoKi = twoKiDef;
	
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
}

/**********************************************************************************************************
*函 数 名: MahonyAHRSupdate
*功能说明: 姿态朝向状态更新
*形    参: 陀螺仪数据向量，加速度计数据向量，磁力计数据向量
*返 回 值: 无
**********************************************************************************************************/
void MahonyAHRSupdate(Vector3f_t gyro, Vector3f_t acc, Vector3f_t mag)
{
	float norm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	
	static uint64_t previousT = 0.0f;
	float dT_s = (GetSysTimeUs() - previousT) * 1e-6;
	dT_s = ConstrainFloat(dT_s, 5e-4, 2e-3);
	previousT = GetSysTimeUs();
	
	//Use IMU algorithm if magnetometer mesaurement invvalid (avoids NaN in magnetometer normalisation)
	if(mag.x==0.0f && mag.y==0.0f && mag.z==0.0f)
	{
		MahonyAHRSupdateIMU(gyro, acc);
		return;
	}
	
	//Compute feedback only if accelerometer measurement valid(avoids NaN in accelerometer normalisation)
	if(!(acc.x==0.0f && acc.y==0.0f && acc.z==0.0f))
	{
		
		//Normalise accelerometer measurement
		norm = Pythagorous3(acc.x, acc.y, acc.z);
		acc.x /= norm;
		acc.y /= norm;
		acc.z /= norm;
		
		//Normalise magnetometer measurement 
		norm = Pythagorous3(mag.x, mag.y, mag.z);
		mag.x /= norm;
		mag.y /= norm;
		mag.z /= norm;
		
		//Auxiliary variables to avoid repeated arithmetic 
		q0q0 = ahrs.quat[0] * ahrs.quat[0];
        q0q1 = ahrs.quat[0] * ahrs.quat[1];
        q0q2 = ahrs.quat[0] * ahrs.quat[2];
        q0q3 = ahrs.quat[0] * ahrs.quat[3];
        q1q1 = ahrs.quat[1] * ahrs.quat[1];
        q1q2 = ahrs.quat[1] * ahrs.quat[2];
        q1q3 = ahrs.quat[1] * ahrs.quat[3];
        q2q2 = ahrs.quat[2] * ahrs.quat[2];
        q2q3 = ahrs.quat[2] * ahrs.quat[3];
        q3q3 = ahrs.quat[3] * ahrs.quat[3]; 
		
		//Reference direction of Earth's magnetic field
		hx = 2.0f * (mag.x * (0.5f - q2q2 - q3q3) + mag.y * (q1q2 - q0q3) + mag.z * (q1q3 + q0q2));
        hy = 2.0f * (mag.x * (q1q2 + q0q3) + mag.y * (0.5f - q1q1 - q3q3) + mag.z * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mag.x * (q1q3 - q0q2) + mag.y * (q2q3 + q0q1) + mag.z * (0.5f - q1q1 - q2q2));
		
		//Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2); 
		
		//Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (acc.y * halfvz - acc.z * halfvy) + (mag.y * halfwz - mag.z * halfwy);
		halfey = (acc.z * halfvx - acc.x * halfvz) + (mag.z * halfwx - mag.x * halfwz);
		halfez = (acc.x * halfvy - acc.y * halfvx) + (mag.x * halfwy - mag.y * halfwx);
		
		//Compute and apply integral feedback if enabled
		if(twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * dT_s;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dT_s;
			integralFBz += twoKi * halfez * dT_s;
			gyro.x += integralFBx;	// apply integral feedback
			gyro.y += integralFBy;
			gyro.z += integralFBz;
		}	
		else
		{
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
		
		// Apply proportional feedback
		gyro.x += twoKp * halfex;
		gyro.y += twoKp * halfey;
		gyro.z += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gyro.x *= (0.5f * dT_s);		// pre-multiply common factors
	gyro.y *= (0.5f * dT_s);
	gyro.z *= (0.5f * dT_s);
	qa = ahrs.quat[0];
	qb = ahrs.quat[1];
	qc = ahrs.quat[2];
	ahrs.quat[0] += (-qb * gyro.x - qc * gyro.y - ahrs.quat[3] * gyro.z);
	ahrs.quat[1] += (qa * gyro.x + qc * gyro.z - ahrs.quat[3] * gyro.y);
	ahrs.quat[2] += (qa * gyro.y - qb * gyro.z + ahrs.quat[3] * gyro.x);
	ahrs.quat[3] += (qa * gyro.z + qb * gyro.y - qc * gyro.x); 
	
	// Normalise quaternion
	norm = Pythagorous4(ahrs.quat[0], ahrs.quat[1], ahrs.quat[2], ahrs.quat[3]);
	ahrs.quat[0] /= norm;
	ahrs.quat[1] /= norm;
	ahrs.quat[2] /= norm;
	ahrs.quat[3] /= norm;
}

/**********************************************************************************************************
*函 数 名: MahonyAHRSupdateIMU
*功能说明: 姿态状态更新（只融合加速度计和陀螺仪）
*形    参: 陀螺仪数据向量，加速度计数据向量
*返 回 值: 无
**********************************************************************************************************/
void MahonyAHRSupdateIMU(Vector3f_t gyro, Vector3f_t acc)
{
	float norm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	
	static uint64_t previousT;
	float dT_s = (GetSysTimeUs() - previousT) * 1e-6;
	dT_s = ConstrainFloat(dT_s, 5e-4, 2e-3);
	previousT = GetSysTimeUs();

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((acc.x == 0.0f) && (acc.y == 0.0f) && (acc.z == 0.0f)))
	{

		// Normalise accelerometer measurement
		norm = Pythagorous3(acc.x, acc.y, acc.z);
		acc.x /= norm;
		acc.y /= norm;
		acc.z /= norm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = ahrs.quat[1] * ahrs.quat[3] - ahrs.quat[0] * ahrs.quat[2];
		halfvy = ahrs.quat[0] * ahrs.quat[1] + ahrs.quat[2] * ahrs.quat[3];
		halfvz = ahrs.quat[0] * ahrs.quat[0] - 0.5f + ahrs.quat[3] * ahrs.quat[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (acc.y * halfvz - acc.z * halfvy);
		halfey = (acc.z * halfvx - acc.x * halfvz);
		halfez = (acc.x * halfvy - acc.y * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dT_s;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dT_s;
			integralFBz += twoKi * halfez * dT_s;
			gyro.x += integralFBx;	// apply integral feedback
			gyro.y += integralFBy;
			gyro.z += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gyro.x += twoKp * halfex;
		gyro.y += twoKp * halfey;
		gyro.z += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gyro.x *= (0.5f * dT_s);		// pre-multiply common factors
	gyro.y *= (0.5f * dT_s);
	gyro.z *= (0.5f * dT_s);
	qa = ahrs.quat[0];
	qb = ahrs.quat[1];
	qc = ahrs.quat[2];
	ahrs.quat[0] += (-qb * gyro.x - qc * gyro.y - ahrs.quat[3] * gyro.z);
	ahrs.quat[1] += (qa * gyro.x + qc * gyro.z - ahrs.quat[3] * gyro.y);
	ahrs.quat[2] += (qa * gyro.y - qb * gyro.z + ahrs.quat[3] * gyro.x);
	ahrs.quat[3] += (qa * gyro.z + qb * gyro.y - qc * gyro.x); 
	
	// Normalise quaternion
	norm = Pythagorous4(ahrs.quat[0], ahrs.quat[1], ahrs.quat[2], ahrs.quat[3]);
	ahrs.quat[0] /= norm;
	ahrs.quat[1] /= norm;
	ahrs.quat[2] /= norm;
	ahrs.quat[3] /= norm;
}


/**********************************************************************************************************
*函 数 名: MahonyGetDCM
*功能说明: 获取方向余弦矩阵
*形    参: 旋转矩阵数组头
*返 回 值: 无
**********************************************************************************************************/
void MahonyGetDCM(float* dcm)
{
	
	Quater_to_DCM(ahrs.dcm, ahrs.quat);
	
	Matrix3_Copy(ahrs.dcm, dcm);
}


/**********************************************************************************************************
*函 数 名: MahonyGetEuler
*功能说明: 获取欧拉角
*形    参: 无
*返 回 值: ZYX旋转顺序欧拉角(rad)
**********************************************************************************************************/
void MahonyGetEuler(Vector3f_t *euler)
{
	ahrs.euler = Quater_to_Euler(ahrs.quat);
	
	*euler = ahrs.euler;
}

