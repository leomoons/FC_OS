#ifndef __CONTROLLER_H
#define __CONTROLLER_H

typedef struct
{
	float J[9];				//inertia matrix
	float mass;
	
	float Binv[6][6];			//inverse of control allocation matrix
	
//	float B[36];			//control allocation matrix	
//	float L;				//force leg of motor;
//	
//	//Three angles of vehicle structure	
//	float Tilt[3][6];
//		
	float T;				//thrust coefficient
//	float TT;				//反扭矩和推力的比值
}_vehicle_para;

extern _vehicle_para _veh;
void ControllerInit(void);
void CtrlTask(void);


#endif
