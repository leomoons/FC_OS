#include "stm32f4xx.h"
#include "mathConfig.h"


float F0;
int16_t i16;
uint16_t ui16;
int32_t i32;
uint8_t isdcm_;


Vector3f_t v1, v2, v3;
Vector3i_t v4;



float quat[4];
float dcm[9];
Vector3f_t euler;


int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/********test for mathTool.c************/
	float a, b;
	a = 1.023f;
	b = 2.3123f;
	
	F0 = a+b;		//FPU test, 汇编调参窗口中会调用VADD, VLDR等汇编指令
	F0 = min(a, b);
	F0 = max(a, a);
	F0 = abs(-a);
	F0 = sign(1e-7f);
	F0 = LIMIT(a, -1.0f, 1.0f);

	F0 = SafeArcsin(a);
	F0 = SafeArcsin(HALF_SQRT_2);
	
	F0 = fast_atan2(1.0f, 2.0f);
	
	F0 = ConstrainFloat(F0, a, b);
	i16 = ConstrainInt16((int16_t)10, (int16_t)a, (int16_t)b);
	ui16 = ConstrainUint16((uint16_t)4, (uint16_t)a, (uint16_t)b);
	i32 =  ConstrainInt32((int32_t)-100, (int32_t)-99, (int32_t)0);
	
	F0 = Radians(89.0f);
	F0 = Degrees(M_PI);
	
	F0 = Sq(-9.9923f);
	F0 = Pythagorous2(a,b);
	F0 = Pythagorous3(a,b, -1.023f);
	F0 = Pythagorous4(a,5.123f, 11.1f, -32.1f);
	
	
	
	/*************test for vector3*************************/
	v1.x = 0.3f; v1.y = 1.23f; v1.z = -0.12f;
	v2.x = 0.3345f; v2.y = 13.23f; v2.z = -1.12f;
	
	Vector3f_Normalize(&v1);
	
	v1.x = 0.3f; v1.y = 1.23f; v1.z = -0.12f;
	
	v4 = Vector3fTo3i(v2);
	v3 = Vector3f_Add(v1, v2);
	v3 = Vector3f_Sub(v1, v2);
	v3 = Vector3iTo3f(v4);
	v3 = VectorCrossProduct(v1, v2);
	float m3[9] = {1,4,2, 3,1,9, 1.2,3.2,1.25};
	v3 = Matrix3MulVector3(m3, v1);
	
	//TODO: 根据 重力加速度向量 和 磁场向量 获得姿态参考
	//AccVectorToRollPitchAngle();
	//MagVectorToYawAngle();
	
	
	
	/**************test for matrix3**************************/
	float m1[9] = {3,12,20.0, 1.23,-0.1,-13, 4.2,5.2,0.02};
	float m2[9] = {3,1,10.0, 3.2,1,-1, 0,-3.2,2.12};
	Matrix3_Add(m1, m2, m3);
	Matrix3_Sub(m1, m2, m3);
	Matrix3_Mul(m1, m2, m3);
	Matrix3_Copy(m2, m3);
	Matrix3_Tran(m2, m3);
	Matrix3_Inv(m1, m3);
	Matrix3_Eye(m3);
	
	/***************test for attitude representation transition*********************/
	quat[0] = 0.8202f; quat[1] = 0.4044f; quat[2] = 0.3996f; quat[3] = -0.06382f;
	
	Quater_to_DCM(dcm, quat);
	euler = Quater_to_Euler(quat);
	
	euler.x = M_PI/3; euler.y = M_PI/4; euler.z = M_PI/10;
	Euler_to_DCM(dcm, euler);
	Euler_to_DCM_T(dcm, euler);
	Euler_to_Quater(quat, euler);
	
	dcm[0] = 0.6725f; dcm[1] = 0.4279f; dcm[2] = 0.6039f;
	dcm[3] = 0.2185f; dcm[4] = 0.6648f; dcm[5] =-0.7144f;
	dcm[6] =-0.7071f; dcm[7] = 0.6124f; dcm[8] = 0.3536f;
	DCM_to_Quater(quat, dcm);
	euler = DCM_to_Euler(dcm);
	
	
	
}
