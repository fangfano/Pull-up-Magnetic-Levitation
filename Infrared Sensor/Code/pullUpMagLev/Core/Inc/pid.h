/*
 * pid.h
 *
 *  Created on: Jul 28, 2021
 *      Author: han
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f1xx_hal.h"


typedef struct PID
{
	float P;
	float I;
	float D;
	float Error;    //比例项
	float Integral;   //积分项
	float Differ;    //微分项
	float PreError;  //上一次误差
	float Ilimit;  //积分分离
	float Irang;  //积分限幅
	uint8_t Ilimit_flag;  //积分分离标志
	float Pout;  //比例项输出
	float Iout;  //积分项输出
	float Dout;  //微分项输出
	float Output;  //总输出
}PID_TYPE;


void PID_Position_Cal(PID_TYPE *PID, float target, float messure);


#endif /* INC_PID_H_ */
