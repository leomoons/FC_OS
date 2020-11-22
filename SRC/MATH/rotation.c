/**********************************************************************************************************
 * @文件     rotation.c
 * @说明     方向旋转
 * @版本  	 V1.0
 * @作者     Leomoon
 * @日期     2020.09
**********************************************************************************************************/
#include "rotation.h"
#include "mathTool.h"

/**********************************************************************************************************
*函 数 名: RotateVector3f
*功能说明: 向量方向变换
*形    参: 旋转方式 原始向量指针
*返 回 值: 无
**********************************************************************************************************/
void RotateVector3f(enum Rotation rot, Vector3f_t* v)
{
	float tmp;
	
	switch (rot)
	{
		case ROTATION_NONE:
		case ROTATION_MAX:
			break;
		
		case ROTATION_YAW_45: {
			tmp  = HALF_SQRT_2 * (v->x - v->y);
			v->y = HALF_SQRT_2 * (v->x + v->y);
			v->x = tmp;
			break;
		}

		case ROTATION_YAW_90: {
			tmp = v->x;
			v->x = -v->y;
			v->y = tmp;
			break;
		}

		case ROTATION_YAW_135: {
			tmp  = -HALF_SQRT_2 * (v->x + v->y);
			v->y =  HALF_SQRT_2 * (v->x - v->y);
			v->x = tmp;
			break;
		}

		case ROTATION_YAW_180: {
			v->x = -v->x;
			v->y = -v->y;
			break;
		}

		case ROTATION_YAW_225: {
			tmp  = HALF_SQRT_2 * (v->y - v->x);
			v->y = -HALF_SQRT_2 * (v->x + v->y);
			v->x = tmp;
			break;
		}

		case ROTATION_YAW_270: {
			tmp = v->x;
			v->x = v->y;
			v->y = -tmp;
			break;
		}

		case ROTATION_YAW_315: {
			tmp  = HALF_SQRT_2 * (v->x + v->y);
			v->y = HALF_SQRT_2 * (v->y - v->x);
			v->x = tmp;
			break;
		}
		
		case ROTATION_ROLL_180: {
			v->y = -v->y;
			v->z = -v->z;
			break;
		}

		case ROTATION_ROLL_180_YAW_45: {
			tmp  = HALF_SQRT_2 * (v->x + v->y);
			v->y = HALF_SQRT_2 * (v->x - v->y);
			v->x = tmp;
			v->z = -v->z;
			break;
		}

		case ROTATION_ROLL_180_YAW_90: {
			tmp = v->x;
			v->x = v->y;
			v->y = tmp;
			v->z = -v->z;
			break;
		}

		case ROTATION_ROLL_180_YAW_135: {
			tmp = HALF_SQRT_2 * (v->y - v->x);
			v->y   = HALF_SQRT_2 * (v->y + v->x);
			v->x = tmp;
			v->z = -v->z;
			break;
		}
		
		case ROTATION_PITCH_180: {
			v->x = -v->x;
			v->z = -v->z;
			break;
		}
		
		case ROTATION_ROLL_180_YAW_225: {
			tmp = -HALF_SQRT_2 * (v->x + v->y);
			v->y   =  HALF_SQRT_2 * (v->y - v->x);
			v->x = tmp;
			v->z = -v->z;
			break;
		}

		case ROTATION_ROLL_180_YAW_270: {
			tmp = v->x;
			v->x = v->y;
			v->y = tmp;
			v->z = -v->z;
			break;
		}

		case ROTATION_ROLL_180_YAW_315: {
			tmp  =  HALF_SQRT_2 * (v->x - v->y);
			v->y = -HALF_SQRT_2 * (v->x + v->y);
			v->x = tmp;
			v->z = -v->z;
			break;
		}
		
		case ROTATION_ROLL_90: {
			tmp = v->z;
			v->z = v->y;
			v->y = -tmp;
			break;
		}

		case ROTATION_ROLL_90_YAW_45: {
			tmp  = v->z;
			v->z = v->y;
			v->y = -tmp;
			tmp  = HALF_SQRT_2 * (v->x - v->y);
			v->y = HALF_SQRT_2 * (v->x + v->y);
			v->x = tmp;
			break;
		}

		case ROTATION_ROLL_90_YAW_90: {
			tmp = v->z;
			v->z = v->y;
			v->y = -tmp;
			tmp = v->x;
			v->x = -v->y;
			v->y = tmp;
			break;
		}

		case ROTATION_ROLL_90_YAW_135: {
			tmp  = v->z;
			v->z = v->y;
			v->y = -tmp;
			tmp  = -HALF_SQRT_2 * (v->x + v->y);
			v->y =  HALF_SQRT_2 * (v->x - v->y);
			v->x = tmp;
			break;
		}

		case ROTATION_ROLL_270: {
			tmp = v->z;
			v->z = -v->y;
			v->y = tmp;
			break;
		}

		case ROTATION_ROLL_270_YAW_45: {
			tmp  = v->z;
			v->z = -v->y;
			v->y = tmp;
			tmp  = HALF_SQRT_2 * (v->x - v->y);
			v->y = HALF_SQRT_2 * (v->x + v->y);
			v->x = tmp;
			break;
		}

		case ROTATION_ROLL_270_YAW_90: {
			tmp = v->z;
			v->z = -v->y;
			v->y = tmp;
			tmp = v->x;
			v->x = -v->y;
			v->y = tmp;
			break;
		}

		case ROTATION_ROLL_270_YAW_135: {
			tmp  = v->z;
			v->z = -v->y;
			v->y = tmp;
			tmp  = -HALF_SQRT_2 * (v->x + v->y);
			v->y =  HALF_SQRT_2 * (v->x - v->y);
			v->x = tmp;
			break;
		}

		case ROTATION_ROLL_270_YAW_270: {
			tmp = v->z;
			v->z = -v->y;
			v->y = tmp;
			tmp = v->x;
			v->x = v->y;
			v->y = -tmp;
			break;
		}

		case ROTATION_PITCH_90: {
			tmp = v->z;
			v->z = -v->x;
			v->x = tmp;
			break;
		}

		case ROTATION_PITCH_270: {
			tmp = v->z;
			v->z = v->x;
			v->x = -tmp;
			break;
		}

		case ROTATION_ROLL_180_PITCH_270: {
			tmp = v->z;
			v->z = v->x;
			v->x = tmp;
			v->y = -v->y;
			break;
		}

		case ROTATION_PITCH_90_YAW_180: {
			tmp = v->x;
			v->x = v->z;
			v->z = tmp;
			v->y = -v->y;
			break;
		}

		case ROTATION_PITCH_90_ROLL_90: {
			tmp = v->x;
			v->x = v->y;
			v->y = -v->z;
			v->z = -tmp;
			break;
		}

		case ROTATION_YAW_293_PITCH_68_ROLL_90: {
			float tmpx = v->x;
			float tmpy = v->y;
			float tmpz = v->z;
			v->x =  0.143039f * tmpx +  0.368776f * tmpy + -0.918446f * tmpz;
			v->y = -0.332133f * tmpx + -0.856289f * tmpy + -0.395546f * tmpz;
			v->z = -0.932324f * tmpx +  0.361625f * tmpy +  0.000000f * tmpz;
			break;
		}

		case ROTATION_PITCH_90_ROLL_270: {
			tmp = v->x;
			v->x = -v->y;
			v->y = v->z;
			v->z = -tmp;
			break;
		}
		
		case ROTATION_ACC_DOWN: {
			float tmp = v->x;
			v->x = -v->y;
			v->y = -tmp;
			v->z = v->z;
			break;
		}
		
		case ROTATION_GYRO_DOWN: {
			float tmp = v->x;
			v->x = v->y;
			v->y = tmp;
			v->z = -v->z;
			break;
		}
		
		case ROTATION_ACC_UP: {
			float tmp = v->x;
			v->x = -v->y;
			v->y = tmp;
			v->z = -v->z;
			break;
		}
		
		case ROTATION_GYRO_UP: {
			float tmp = v->x;
			v->x = v->y;
			v->y = -tmp;
			break;
		}
		
		
		default:
			break;
	}
}
