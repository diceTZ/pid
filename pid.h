//@作者			：tou_zi
//@编写时间	：2019年4月6日
//@修改时间	：2019年4月6日
//@文件名		：pid.h
//@描述			：pid模块库，可实现pid的计算
#ifndef _PID_H
#define _PID_H

#ifndef u8
#define u8 unsigned char
#endif

#define INTEGRAL_NORMAL				0			//普通积分
#define INTEGRAL_SEPARATION		1			//积分分离
#define INTEGRAL_SATURATION 	2			//抗饱和积分
#define INTEGRAL_SPEED				3			//变速积分	

#define DIFFERENTIAL_COMPLETE	0			//完全微分
#define DIFFERENTIAL_PART			1			//不完全微分
#define DIFFERENTIAL_PREVIOUS 2			//微分先行

typedef struct PID_Position_Flag
{
	u8 run;
	u8 integral_way;
	u8 differential_way;
	u8 use_predict;
}PID_Position_Flag;

typedef struct PID_Position_DInterface
{
	float *target;
	float *present;
	float *predict;
}PID_Position_DInterface;

typedef struct PID_Position_Interface
{
	PID_Position_DInterface data;
}PID_Position_Interface;

typedef struct PID_Position_Parameter
{
	float kp;								//比例系数
	float ki;								//积分系数
	float kd;								//微分系数
	float kf;								//前馈系数
	float kd_lpf;						//不完全微分系数
	float kd_pre;						//微分先行系数
	float k_pre;						//预测系数
	
	float target_limit;				//目标值限幅
	float bias_limit;					//误差限幅
	float bias_dead_zone;			//小于这个值将不进行PID操作
	float bias_for_integral;	//开始积分的误差	--	用于积分分离
	float integral_limit;			//积分限幅				--	用于抗积分饱和
	float out_limit;					//输出限幅
}PID_Position_Parameter;

typedef struct PID_Position_Data
{	
	float out;
}PID_Position_Data;

typedef struct PID_Position_Process
{
	float bias;
	float integral_bias;
	float differential_bias;
	float lpf_differential_bias;
	
	float feedforward;
	float predict;
	
	float last_target;
	float last_bias;
}PID_Position_Process;

typedef struct PID_Position_Module
{
	PID_Position_Flag		flag;
	PID_Position_Interface 	interface;
	PID_Position_Parameter	parameter;
	PID_Position_Data 		data;
	PID_Position_Process 	process;
}PID_Position_Module;

void initPID_Position_Module(PID_Position_Module *pid);
void calculatePID_Position_Module(PID_Position_Module *pid, float cycle);
#endif


