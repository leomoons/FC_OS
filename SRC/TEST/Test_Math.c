#include "stm32f4xx.h"
#include "mathTool.h"


float F0, F1,F2,F3,F4,F5,F6,F7,F8,F9;
int16_t i16;
uint16_t ui16;
int32_t i32;

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/********test for mathTool.c************/
	float a, b;
	a = 1.023f;
	b = 2.3123f;
	
	F0 = a+b;
	F1 = min(a, b);
	F2 = max(a, a);
	F3 = abs(-a);
	F4 = sign(1e-7);
	F5 = LIMIT(a, -1.0, 1.0f);
	
	F6 = SafeArcsin(a);
	F7 = SafeArcsin(HALF_SQRT_2);
	
	F8 = fast_atan2(1.0, 2.0);
	
	F9 = ConstrainFloat(F8, a, b);
	i16 = ConstrainInt16(F8, a, b);
	ui16 = ConstrainUint16(F8, a, b);
	i32 =  ConstrainInt16(F8, a, b);
	
	F9 = Radians(89.0);
	F9 = Degrees(M_PI);
	
	F9 = Sq(F9);
	F9 = Pythagorous2(a,b);
	F9 = Pythagorous3(a,b, F8);
	F9 = Pythagorous4(a,a,a,b);
	
	/*************test for vector3*************************/
	Vector3f_t v1, v2, v3;
	Vector3i_t v4;
	v1.x = 0.3; v1.y = 1.23; v1.z = -0.12;
	v2.x = 0.3345; v2.y = 13.23; v2.z = -1.12;
	
	Vector3f_Normalize(&v1);
	
	v1.x = 0.3; v1.y = 1.23; v1.z = -0.12;
	float m3[9] = {1,4,2, 3,1,9, 1.2,3.2,1.25};
	v4 = Vector3fTo3i(v2);
	v3 = Vector3f_Add(v1, v2);
	v3 = Vector3f_Sub(v1, v2);
	
	v3 = VectorCrossProduct(v1, v2);
	v3 = Matrix3MulVector3(m3, v1);
	
	Vector3f_t euler;
	euler.x = M_PI/6; euler.y = M_PI/40; euler.z = M_PI/10;
	F9 = 10/180*M_PI;
	EulerAngleToDCM(euler, m3);
	EulerAngleToDCM_T(euler, m3);
	
	//TODO: 根据重力加速度向量 和 磁场向量获得姿态参考
	
	/**************test for matrix3**************************/
	float m1[9] = {3,12,20.0, 1.23,-0.1,-13, 4.2,5.2,0.02};
	float m2[9] = {3,1,10.0, 3.2,1,-1, 0,-3.2,2.12};
	Matrix3_Add(m1, m2, m3);
	Matrix3_Sub(m1, m2, m3);
	Matrix3_Mul(m1, m2, m3);
	Matrix3_Copy(m2, m3);
	Matrix3_Tran(m2, m3);
	Matrix3_Inv(m1, m3);
	
	
	
}
