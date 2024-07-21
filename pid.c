#include "pid.h"


//PID初始化
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->error=0;
	pid->integral=0;
	pid->lastError=0;
	pid->output=0;
}

//单级PID计算
void PID_SingleCalc(PID *pid,float target,float feedback)
{
	pid->error = targrt-feedback;
	if(pid->error < 0.3 && pid->error > -0.3) pid->error = 0;//pid死区
	pid->integral += pid->error;
	if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
	else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

	if(target == 0) pid->integral = 0; // 刹车时清空i


	pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * (pid->error - pid->lastError));//全量式PID

	//输出限幅
	if(target >= 0)//正转时
	{
			if(pid->output < 0) pid->output = 0;
			else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
	}
	else if(target < 0)//反转时
		{
			if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
			else if(pid->output > 0) pid->output = 0;
	}

	pid->lastError = pid->error;
	if(target == 0) pid->output = 0; // 刹车时直接输出0
	return pid->output;
}

//串级PID计算
/*void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	PID_SingleCalc(&pid->outer,angleRef,angleFdb);            //外环角度
	PID_SingleCalc(&pid->inner,pid->outer.output,speedFdb);   //内环速度
	pid->output=pid->inner.output;
}*/

//清理历史数据
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}
//重设输出限幅
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	PID_Clear(pid);
	pid->maxOutput=maxOut;
}