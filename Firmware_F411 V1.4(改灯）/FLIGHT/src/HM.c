#include "HM.h"
#include <stdbool.h>
#include <math.h>
#include "maths.h"
float Kp_f=1,Kp_u=3.9;
extern s16 m1,m2,m3,m4;
int Wr=0;
float HM_Update1(HM* hm, const float error,float actualAnglePITCH,float actualAngleYAW,float actualAngleROW)
{
	hm->error = error;   
	hm->actulangle_pitch=actualAnglePITCH;
	hm->actulangle_yaw =actualAngleYAW;
	hm->actulangle_roll=actualAngleROW;
	
	hm->E_deriv = (hm->error - hm->prevError) / hm->dt; 
	hm->P_deriv=hm->actulangle_pitch-hm->pre_actulangle_pitch;//俯仰误差导数
	hm->Y_deriv=hm->actulangle_yaw-hm->pre_actulangle_yaw;    //偏航误差导数
	hm->R_deriv=hm->actulangle_roll-hm->pre_actulangle_roll;    //翻滚误差导数
	
	hm->s=hm->kp*hm->error+hm->kd*hm->E_deriv;											//滑模面
	if(m1!=0 &&m2!=0 &&m3!=0 &&m4!=0 )
	{Wr=(abs(m1)+abs(m3)-abs(m2)-abs(m4))/1000;}  //求Wr
	else Wr=0;
		
	if(Wr>50)Wr=50;																//Wr限幅
	if(Wr<-50) Wr=-50;
	
	if(hm->s>500){hm->sign_s=1;}
	else if(hm->s<-500){hm->sign_s=-1;}
	else {hm->sign_s=hm->s/500;} 	//饱和函数
	
	hm->f = Kp_f*( hm->P_deriv * hm->Y_deriv * 0.8266f - hm->P_deriv*Wr*0.08f);//f函数
	hm->u=0.0075f*( 60*hm->f - 500*hm->R_deriv - hm->s - hm->sign_s);					//控制输入
	
	hm->prevError=hm->error;
	hm->pre_actulangle_pitch=hm->actulangle_pitch;
	hm->pre_actulangle_yaw  =hm->actulangle_yaw;
	hm->pre_actulangle_roll =hm->actulangle_roll;            //保存上一次误差
	return Kp_u*hm->u;																//返回值
}

void HMInit_roll(HM* hm, const float desired, const float dt)
{
	hm->actulangle_pitch =0;
	hm->actulangle_roll  =0;
	hm->actulangle_yaw   =0;
	hm->error=0;
	hm->E_deriv=0;
	hm->E_desired =0.0f;
	hm->f      =0.0f;
	hm->kd    =60.0f;
	hm->kp    =500.0f;
	hm->out =0;
	hm->prevError=0;
	hm->pre_actulangle_pitch=0;
	hm->pre_actulangle_roll=0;
	hm->pre_actulangle_yaw=0;
	hm->P_deriv=0;
	hm->R_deriv=0;
	hm->Y_deriv=0;
	hm->s=0;
	hm->sign_s=0;
	hm->u=0;
	hm->dt = dt;
}

