 /**********************************************************************************************************
 * @文件     mahonyAHRS.c
 * @说明     Mahony的基于四元数的姿态估计
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.10
**********************************************************************************************************/
#include "mahonyAHRS.h"
#include "ahrsConfig.h"
#include "boardConfig.h"


#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

volatile float twoKp;											// 2 * proportional gain (Kp)
volatile float twoKi;											// 2 * integral gain (Ki)					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx,  integralFBy, integralFBz;	// integral error terms scaled by Ki

AHRS_t ahrs;


/**********************************************************************************************************
*函 数 名: MahonyAHRSinit
*功能说明: 初始化Mahony滤波器
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MahonyAHRSinit(void)
{
	ahrs.q0 = 1.0f;
	ahrs.q1 = 0.0f;
	ahrs.q2 = 0.0f;
	ahrs.q3 = 0.0f;
	
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
	
	static uint64_t previousT;
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;
	deltaT = ConstrainFloat(deltaT, 5e-4, 2e-3);
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
		q0q0 = ahrs.q0 * ahrs.q0;
        q0q1 = ahrs.q0 * ahrs.q1;
        q0q2 = ahrs.q0 * ahrs.q2;
        q0q3 = ahrs.q0 * ahrs.q3;
        q1q1 = ahrs.q1 * ahrs.q1;
        q1q2 = ahrs.q1 * ahrs.q2;
        q1q3 = ahrs.q1 * ahrs.q3;
        q2q2 = ahrs.q2 * ahrs.q2;
        q2q3 = ahrs.q2 * ahrs.q3;
        q3q3 = ahrs.q3 * ahrs.q3; 
		
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
//		halfex = (acc.y * halfvz + acc.z * halfvy);
//		halfey =(-acc.z * halfvx - acc.x * halfvz);
		halfez = (acc.x * halfvy - acc.y * halfvx) + (mag.x * halfwy - mag.y * halfwx);
		
		//Compute and apply integral feedback if enabled
		if(twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * deltaT;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * deltaT;
			integralFBz += twoKi * halfez * deltaT;
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
	gyro.x *= (0.5f * deltaT);		// pre-multiply common factors
	gyro.y *= (0.5f * deltaT);
	gyro.z *= (0.5f * deltaT);
	qa = ahrs.q0;
	qb = ahrs.q1;
	qc = ahrs.q2;
	ahrs.q0 += (-qb * gyro.x - qc * gyro.y - ahrs.q3 * gyro.z);
	ahrs.q1 += (qa * gyro.x + qc * gyro.z - ahrs.q3 * gyro.y);
	ahrs.q2 += (qa * gyro.y - qb * gyro.z + ahrs.q3 * gyro.x);
	ahrs.q3 += (qa * gyro.z + qb * gyro.y - qc * gyro.x); 
	
	// Normalise quaternion
	norm = Pythagorous4(ahrs.q0, ahrs.q1, ahrs.q2, ahrs.q3);
	ahrs.q0 /= norm;
	ahrs.q1 /= norm;
	ahrs.q2 /= norm;
	ahrs.q3 /= norm;
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
	float deltaT = (GetSysTimeUs() - previousT) * 1e-6;
	deltaT = ConstrainFloat(deltaT, 5e-4, 2e-3);
	previousT = GetSysTimeUs();

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((acc.x == 0.0f) && (acc.y == 0.0f) && (acc.z == 0.0f))) {

		// Normalise accelerometer measurement
//		norm = Pythagorous3(acc.x, acc.y, acc.z);
//		acc.x /= norm;
//		acc.y /= norm;
//		acc.z /= norm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = ahrs.q1 * ahrs.q3 - ahrs.q0 * ahrs.q2;
		halfvy = ahrs.q0 * ahrs.q1 + ahrs.q2 * ahrs.q3;
		halfvz = ahrs.q0 * ahrs.q0 - 0.5f + ahrs.q3 * ahrs.q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (acc.y * halfvz - acc.z * halfvy);
		halfey = (acc.z * halfvx - acc.x * halfvz);
		halfez = (acc.x * halfvy - acc.y * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * deltaT;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * deltaT;
			integralFBz += twoKi * halfez * deltaT;
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
	gyro.x *= (0.5f * deltaT);		// pre-multiply common factors
	gyro.y *= (0.5f * deltaT);
	gyro.z *= (0.5f * deltaT);
	qa = ahrs.q0;
	qb = ahrs.q1;
	qc = ahrs.q2;
	ahrs.q0 += (-qb * gyro.x - qc * gyro.y - ahrs.q3 * gyro.z);
	ahrs.q1 += (qa * gyro.x + qc * gyro.z - ahrs.q3 * gyro.y);
	ahrs.q2 += (qa * gyro.y - qb * gyro.z + ahrs.q3 * gyro.x);
	ahrs.q3 += (qa * gyro.z + qb * gyro.y - qc * gyro.x); 
	
	// Normalise quaternion
	norm = Pythagorous4(ahrs.q0, ahrs.q1, ahrs.q2, ahrs.q3);
	ahrs.q0 /= norm;
	ahrs.q1 /= norm;
	ahrs.q2 /= norm;
	ahrs.q3 /= norm;
}


/**********************************************************************************************************
*函 数 名: GetDCM
*功能说明: 获取方向余弦矩阵
*形    参: 无
*返 回 值: 旋转矩阵的数值向量
**********************************************************************************************************/
float* GetDCM(void)
{
	//Auxiliary variables to avoid repeated arithmetic 
	float q0q0 = ahrs.q0 * ahrs.q0;
    float q0q1 = ahrs.q0 * ahrs.q1;
    float q0q2 = ahrs.q0 * ahrs.q2;
    float q0q3 = ahrs.q0 * ahrs.q3;
    float q1q1 = ahrs.q1 * ahrs.q1;
    float q1q2 = ahrs.q1 * ahrs.q2;
    float q1q3 = ahrs.q1 * ahrs.q3;
	float q2q2 = ahrs.q2 * ahrs.q2;
    float q2q3 = ahrs.q2 * ahrs.q3;
    float q3q3 = ahrs.q3 * ahrs.q3; 
	
    ahrs._dcm[0] = 1 - (2*q2q2 + 2*q3q3);
    ahrs._dcm[1] = 2*q1q2 - 2*q0q3;
    ahrs._dcm[2] = 2*q1q3 + 2*q0q2;
		
    ahrs._dcm[3] = 2*q1q2 + 2*q0q3;
    ahrs._dcm[4] = 1 - (2*q1q1 + 2*q3q3);
    ahrs._dcm[5] = 2*q2q3 - 2*q0q1;
		
    ahrs._dcm[6] = 2*q1q3 - 2*q0q2;
    ahrs._dcm[7] = 2*q2q3 + 2*q0q1;
    ahrs._dcm[8] = 1 - (2*q1q1 + 2*q2q2);
	
	return ahrs._dcm;
}


/**********************************************************************************************************
*函 数 名: GetEuler
*功能说明: 获取欧拉角
*形    参: 无
*返 回 值: ZYX旋转顺序欧拉角(rad)
**********************************************************************************************************/
Vector3f_t GetEuler(void)
{
	float tmp = LIMIT(1-Sq(ahrs._dcm[6]), 0, 1);
	
	if(abs(ahrs._dcm[8]) > 0.0f)	//避免奇点的运算
	{
		ahrs.euler.x = fast_atan2(ahrs._dcm[6], Sqrt(tmp));
		ahrs.euler.y = fast_atan2(ahrs._dcm[7], ahrs._dcm[8]);
		ahrs.euler.z =-fast_atan2(ahrs._dcm[3], ahrs._dcm[0]);
	}
	return ahrs.euler;
}

