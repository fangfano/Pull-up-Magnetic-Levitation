/*
 * pid.c
 *
 *  Created on: Jul 28, 2021
 *      Author: han
 */

#include "pid.h"


//PID计算
void PID_Position_Cal(PID_TYPE *PID, float target, float measure) {

	PID->Error = target - measure;
	PID->Differ = PID->Error - PID->PreError;
	PID->Integral += PID->Error;


	if(measure < PID->Ilimit)      //如果磁铁距离线圈太远,不可能吸上,便不再输出
	{
		PID->Iout = 0;
		PID->Dout = 0;

		PID->Output = 0;

		PID->PreError = PID->Error;
		PID->Ilimit_flag = 0;
		return;
	}
	PID->Ilimit_flag = 1;


	//积分限幅
	if (PID->Integral > PID->Irang)
		PID->Integral = PID->Irang;
	else if (PID->Integral < 0 - PID->Irang)
		PID->Integral = 0 - PID->Irang;


	//计算输出
	PID->Pout = PID->P * PID->Error;
	PID->Iout = PID->I * PID->Integral;
	PID->Dout = PID->D * PID->Differ;

	PID->Output = PID->Pout + PID->Iout + PID->Dout;

	PID->PreError = PID->Error;
}

