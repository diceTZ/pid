#include "pid.h"

#define ABS(x) ((x > 0) ? x : -x)

//@作者			：tou_zi
//@编写时间	：2019年4月6日
//@修改时间	：2019年4月6日
//@函数名		：initPID_Position_Module
//@函数功能	：初始化位置式pid模块
//@输入1		：*pid	位置式pid模块结构体指针
//@返回值		：无
void initPID_Position_Module(PID_Position_Module *pid)
{
	pid->flag.run = 0;
	pid->flag.integral_way = INTEGRAL_NORMAL;
	pid->flag.differential_way = DIFFERENTIAL_COMPLETE;
	pid->flag.use_predict = 0;
	
	pid->interface.data.target = 0;
	pid->interface.data.present = 0;
	pid->interface.data.present = 0;
	
	pid->parameter.kp = 0;
	pid->parameter.ki = 0;
	pid->parameter.kd = 0;
	pid->parameter.kf = 0;
	pid->parameter.kd_lpf = 0;
	pid->parameter.kd_pre = 0;
	pid->parameter.k_pre = 0;
	
	pid->parameter.target_limit = -1;
	pid->parameter.bias_limit	= -1;
	pid->parameter.bias_dead_zone = -1;
	pid->parameter.bias_for_integral = -1;
	pid->parameter.integral_limit = -1;
	pid->parameter.out_limit = -1;
	
	pid->data.out = 0;
	
	pid->process.bias = 0;
	pid->process.differential_bias = 0;
	pid->process.lpf_differential_bias = 0;
	pid->process.feedforward = 0;
	pid->process.integral_bias = 0;
	pid->process.last_bias = 0;
	pid->process.last_target = 0;
}

//@作者			：tou_zi
//@编写时间	：2019年4月6日
//@修改时间	：2019年4月6日
//@函数名		：calculatePID_Position_Module
//@函数功能	：计算位置式pid模块
//@输入1		：*pid	位置式pid模块结构体指针
//@输入2		：cycle 调用周期
//@返回值		：无
void calculatePID_Position_Module(PID_Position_Module *pid, float cycle)
{	
	if (pid->flag.run == 0)
		return;
////////////////////////////////期望值限幅
	if (pid->parameter.target_limit >= 0)
	{
		if (*(pid->interface.data.target) > pid->parameter.target_limit)
			*(pid->interface.data.target) = pid->parameter.target_limit;
		else if (*(pid->interface.data.target) < -pid->parameter.target_limit)
			*(pid->interface.data.target) = -pid->parameter.target_limit;
	}

////////////////////////////////前馈操作--前馈值直接加入输出
	pid->process.feedforward = pid->parameter.kf * *(pid->interface.data.target);
/////////////////////////////////////////////////////////////////////////////////
	
////////////////////////////////预测操作--预测值直接加入偏差	
	if (pid->flag.use_predict == 0 || pid->interface.data.predict == 0)
		pid->process.predict = 0;
	else
		pid->process.predict = pid->parameter.k_pre * *pid->interface.data.predict * ABS(*pid->interface.data.predict);
/////////////////////////////////////////////////////////////////////////////////
	
////////////////////////////////偏差操作	
	//第一部分为期望
	//第二部分为反馈
	//第三部分为预测（一般为内环反馈,可不设置）
	float temp_bias = 	*(pid->interface.data.target) 	-  		
											*(pid->interface.data.present) 	- 
											pid->process.predict;
	
	if (pid->parameter.bias_dead_zone >= 0)
		temp_bias = (temp_bias < pid->parameter.bias_dead_zone && temp_bias > -pid->parameter.bias_dead_zone) ? 0 : temp_bias;	//误差死区判断--不用该功能时可以将其置-1
	
	if (pid->parameter.bias_limit >= 0)
	{
		temp_bias = (temp_bias > pid->parameter.bias_limit) ? pid->parameter.bias_limit : temp_bias;														//误差限幅			--不用该功能时可以将其置-1
		temp_bias = (temp_bias < -pid->parameter.bias_limit) ? -pid->parameter.bias_limit : temp_bias;
	}
	pid->process.bias = temp_bias;																																														//计算误差
/////////////////////////////////////////////////////////////////////////////////	
	
////////////////////////////////积分操作
	if (pid->parameter.ki == 0)
		pid->process.integral_bias = 0;
	
	else
		switch (pid->flag.integral_way)
		{
			case INTEGRAL_NORMAL: 
				//普通积分
				pid->process.integral_bias += pid->process.bias * cycle;	
			break;
			
			case INTEGRAL_SEPARATION:
				//积分分离
				if (pid->process.bias > pid->parameter.bias_for_integral || pid->process.bias < -pid->parameter.bias_for_integral)
					break;
				
				pid->process.integral_bias += pid->process.bias * cycle;	
			break;

			case INTEGRAL_SATURATION:
				//抗积分饱和
				if (pid->process.integral_bias * pid->parameter.ki > pid->parameter.integral_limit)
					pid->process.integral_bias = pid->parameter.integral_limit / pid->parameter.ki; 
				
				else if (pid->process.integral_bias * pid->parameter.ki < -pid->parameter.integral_limit)
					pid->process.integral_bias = -pid->parameter.integral_limit / pid->parameter.ki; 
				
				else
					pid->process.integral_bias += pid->process.bias * cycle;
			break;
			
			case INTEGRAL_SPEED:
				//变速积分 -- 可自行添加函数或处理算法
				pid->process.integral_bias += (pid->process.bias + pid->process.last_bias) / (2.0f * cycle);	
			break;
			
			default:
				//默认为普通积分
				pid->process.integral_bias += pid->process.bias * cycle;	
			break;
		}																													
/////////////////////////////////////////////////////////////////////////////////	
	
////////////////////////////////微分操作	
	switch	(pid->flag.differential_way)
	{
		case DIFFERENTIAL_COMPLETE:
			//直接求微分
			pid->process.lpf_differential_bias 
				= pid->process.differential_bias 
				= pid->process.bias - pid->process.last_bias;
		break;

		case DIFFERENTIAL_PART:
			//求微分，再低通滤波
			pid->process.differential_bias = pid->process.bias - pid->process.last_bias;
			pid->process.lpf_differential_bias += pid->parameter.kd_lpf * 3.14f * cycle 
																						* (pid->process.differential_bias - pid->process.lpf_differential_bias);
		break;
		
		case DIFFERENTIAL_PREVIOUS:
			//微分先行
			pid->process.lpf_differential_bias 
				= pid->process.differential_bias 
				= (pid->process.bias - pid->process.last_bias) - pid->parameter.kd_pre * (*(pid->interface.data.target) - pid->process.last_target);
		break;
		
		default : 
			//直接求微分
			pid->process.lpf_differential_bias 
				= pid->process.differential_bias 
				= pid->process.bias - pid->process.last_bias;
		break;
	}
/////////////////////////////////////////////////////////////////////////////////	
	
////////////////////////////////输出合成
	//计算输出
	pid->data.out = pid->parameter.kp * pid->process.bias +
									pid->parameter.ki * pid->process.integral_bias +
									pid->parameter.kd * pid->process.lpf_differential_bias / cycle + 	//可以将cycle注释
									pid->process.feedforward;
	
	//输出限幅
	if (pid->parameter.out_limit >= 0)
	{
		if (pid->data.out > pid->parameter.out_limit)
			pid->data.out = pid->parameter.out_limit;
		
		else if (pid->data.out < -pid->parameter.out_limit)
			pid->data.out = -pid->parameter.out_limit;
	}
	
	//存储过去值
	pid->process.last_target = *(pid->interface.data.target);
	pid->process.last_bias = pid->process.bias;
}

