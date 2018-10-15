#include	"altitude1.h"
#include	"state_estimation.h"

vehicle_attitude_s      vehicle_att;

volatile	static  __SPEED_STRUCTF gyroll_calspeed={0,0,0,0};
volatile	static  __SPEED_STRUCTF gypitch_calspeed={0,0,0,0};

//NED
float acc_ef[3] = {0.0f,0.0f,0.0f};
float accz_comalt=0;
//body frame
//float acc_bias_bf[3] = {0.0f,0.0f,0.0f};
float corr_baro = 0.0f;

//float x_esta[2] = {0.0f,0.0f};	//pos,vel
//float y_esta[2] = {0.0f,0.0f};	//pos,vel
//float z_esta[2] = {0.0f,0.0f};	//pos,vel
float coefficient_acc=1;
struct vertical_information  baro_acc_alt;

volatile float h_target=0,h_targetf,up_down_add=0;

static int estimate_altp,estimate_altd,estimate_aerrorf;
static int estimate_velp,estimate_veld,estimate_verrorf;
static int position_error,velocity_error;
static float estimate_alti=0,estimate_veli=0;

static  float abs_pitch=0,abs_pitchrollf=0;
static  float abs_roll=0;
static  uint16 angle_timer=0;


/*
*********************************************************************************************************
*                                            ahrs_update_R_bf_to_ef()
*
* Description : 得到从机体坐标到NED坐标的旋转矩阵
*
* Argument(s) : none.
*
* Return(s)   : 加速度向量(m/s^2)
*
* Note(s)     : none.
*********************************************************************************************************
*/



#define fblr_com_limit 10
void param_modify(float angle_pitch,float angle_roll)
{
	static  uint8 angle_com_count=0;	
	static  int16 fblr_comf=0;
	
	abs_pitch=FL_ABS(angle_pitch);
	abs_roll=FL_ABS(angle_roll);
	
	if(angle_timer==0)
	{
		if(abs_pitch+abs_roll<3)
			initialize_system_parameter(0.004f,110,10);
		else if(abs_pitch+abs_roll<6)
			initialize_system_parameter(0.004f,110,45);
		else if(abs_pitch+abs_roll<10)
			initialize_system_parameter(0.004f,110,60);
		else
			initialize_system_parameter(0.004f,110,100);
	}
	if(abs_pitch+abs_roll>5)
		angle_timer=0;
	else
	{
		angle_timer++;
		if(angle_timer>300)
		{
			angle_timer=1000;
			initialize_system_parameter(0.004f,100,10);
		}
	}
}


//说明
//accz 200 参数时会有空中掉落情况
void check_stop(int16 accz_velocity,int16 accz)
{
	
	static int32 H_I_STOP;
	static uint8 slow_down_count=0;
	static uint16 down_time=0;
	static uint8 Stop_Motor=0;
//	if(hold_high_statue==HOLD_HIGH_STOP)
//		H_I_STOP=H_I_average;
	down_time++;
	if((MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK)||MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_THRBOTTON)) && MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT))
	{
		if(down_time>=300)
		{
			down_time=2000;
			//增加pitch和roll在下拉油门杆状态下频繁打方向杆误触发的情况处理  ZFM 20180922
			if(((accz_velocity>=0&&accz<50&&accz>-50) || (accz>350 || accz<-350))&& (FL_ABS(gvst_ThisBird.TelDat[3]-0x80)<=30) &&(FL_ABS(gvst_ThisBird.TelDat[4]-0x80)<=30)&&MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)&&MF_IF_NOFLAG(FLAGA,FLAGA_FLIP_FINISH)&&MF_IF_NOFLAG(FLAGA,FLAGA_FLIP_END_UP)/*&& (H_I_STOP-H_I_average<300)*/)
				{
				slow_down_count++;
				if(slow_down_count>1)
				{
					hold_high_statue=HOLD_HIGH_STOP;
					MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_STOPF);
					MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_LINKDOWN);
					MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN);
					MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEY_STOP);   //一键下降停止
					MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THRBOTTON);
				}
			}
			else if(slow_down_count>0)
			{
				if(accz_velocity<0)
					slow_down_count=0;
				else
					slow_down_count--;
			}
		}
	}
	else
	{
		down_time=0;
		slow_down_count=0;
	}
		/*加入李伟停转处理动作*/
	if(fabs(accz_velocity)<=5&&throttle_stick<10&&MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT)&&MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN)&&hold_high_statue!=HOLD_HIGH_STOP){
		if(++Stop_Motor>=200){
				hold_high_statue=HOLD_HIGH_STOP;
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_STOPF);
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_LINKDOWN);
    }
  }
	else
		Stop_Motor=0;
}

static	__SingleKlmPar	accelKlmParInUpdateAhrsData[3]={{0.1f,1.0f,0.005f,6.0f},{0.1f,1.0f,0.005f,6.0f},{0.1f,1.0f,0.005f,6.0f}};


void accel_baro_mag_gps_calculator(int16 accel_x,int16 accel_y,int16 accel_z,int16 zero_accelz,int32 baro_alt)
{
	
//	static uint8 usart_timer=0;
	static int16 acc_xyz[3],real_acczz;
	
	if(zero_accelz==0)
		return;
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
		return;
	
	acc_xyz[0]=accel_x*980/MV_ACC_RP;
	acc_xyz[1]=accel_y*980/MV_ACC_RP;
	acc_xyz[2]=accel_z*980/MV_ACC_RP;
	
	real_acczz=-acc_xyz[1]*vehicle_att.R[2][0]-acc_xyz[0]*vehicle_att.R[2][1]+acc_xyz[2]*vehicle_att.R[2][2]-980;
	
	if(hold_high_statue==HOLD_HIGH_STOP)
	{
		if(real_acczz>200) real_acczz=0;
		if(real_acczz<-200) real_acczz=0;
	}
	if(abs_pitch>80 || abs_roll>80)   //翻角度高度不做计算
		real_acczz=0;
	
	TimeUpdate(&baro_acc_alt,real_acczz);
	
	check_stop(baro_acc_alt.velocity_hat,real_acczz);    //检测停止
	accel_outhand=(int16)(baro_acc_alt.velocity_hat*4);
	accel_out=altitude_out();
//	usart_timer++;
//	if(usart_timer>=50)
//	{
//		usart_timer=0;
////		SendDatShow((int16)(gvst_ThisBird.EulerAngle.roll_y*10),(int16)(gvst_ThisBird.EulerAngle.pitch_x*10));
//		
//	}
}

/*
*********************************************************************************************************
*                                            ahrs_update_R_bf_to_ef()
*
* Description : 得到从机体坐标到NED坐标的旋转矩阵
*
* Argument(s) : none.
*
* Return(s)   : 加速度向量(m/s^2)
*
* Note(s)     : none.
*********************************************************************************************************
*/
void ahrs_update_R_bf_to_ef(float angle_pitch,float angle_roll,float angle_yaw)
{
	float sin_pitch = sin(angle_pitch * M_DEG_TO_RAD);
	float cos_pitch = cos(angle_pitch * M_DEG_TO_RAD);
	
	float sin_roll = sin(angle_roll * M_DEG_TO_RAD);
	float cos_roll = cos(angle_roll * M_DEG_TO_RAD);
	
	float sin_yaw = sin(angle_yaw * M_DEG_TO_RAD);
	float cos_yaw = cos(angle_yaw * M_DEG_TO_RAD);
	
	param_modify(angle_pitch,angle_roll);
	
//	vehicle_att.R[0][0] = cos_pitch * cos_yaw;
//	
//	vehicle_att.R[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
//	
//	vehicle_att.R[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
//	
//	vehicle_att.R[1][0] = cos_pitch * sin_yaw;
//	
//	vehicle_att.R[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
//	
//	vehicle_att.R[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
	
	vehicle_att.R[2][0] = -sin_pitch;
	
	vehicle_att.R[2][1] = sin_roll * cos_pitch;
	 
	vehicle_att.R[2][2] = cos_roll * cos_pitch;	
}

//向上为正
//float ahrs_get_accel_ef_z_mss(void)
//{
//    return -(vehicle_att.R[2][0] * gvst_ThisBird.AccDatNow.pitch + vehicle_att.R[2][1] * gvst_ThisBird.AccDatNow.roll + 
//			vehicle_att.R[2][2] * (gvst_ThisBird.AccDatNow.yaw-gvst_ThisBird.AccDatReZero.yaw));
//}

//float ahrs_get_yaw_degree(void)
//{
//    return vehicle_att.yaw;
//}


static void alt_target_dis(void)
{
	static int8 up_down_add=0;
	static int16 up_down_count=0,onekeyup_surecount=0,stable_timer=300,fblr_recover=0,fblr_toup=0;
	if(hold_high_statue==HOLD_HIGH_STOP)
	{
		up_down_count=0;
		h_target=baro_acc_alt.altitude_hat;
		h_targetf=h_target;//表示静止时候的高度
	} 
	else if(hold_high_statue==HOLD_HIGH_UP)
	{
		up_down_add=10;
		up_down_count=0;
		h_target=baro_acc_alt.altitude_hat+dg_updown;
		
		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_UPDOWNFLAG);   //有上升标志
		stable_timer=300;
		if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP))
		{ 
			stable_timer=5;//10;
			up_down_add=10;
			if(baro_acc_alt.altitude_hat-h_targetf>=180) //是否需要修改
			{
				onekeyup_surecount++;
				if(onekeyup_surecount>=10)
				{
					onekeyup_surecount=20;
					hold_high_statue=HOLD_HIGH_KEEP;
				  MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP);
				}
			}
			else
				onekeyup_surecount=0;
		}
//		else {
//			up_down_add=10;
//			stable_timer=50;
//		}
		
	}
	else if(hold_high_statue==HOLD_HIGH_DOWN)
	{
		up_down_add=-10;
		up_down_count=0;
		h_target=baro_acc_alt.altitude_hat-dg_updown;
		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_UPDOWNFLAG);  //有下降标志
		stable_timer=300;
	}
	
	//定高打舵处理
	if(fblr_state!=fblr_nomode && (abs_pitch+abs_roll>8))
	{
		if(hold_high_statue==HOLD_HIGH_KEEP)
		{
			if(baro_acc_alt.altitude_hat-h_targetf>0 && baro_acc_alt.velocity_hat<-10)  //50不会过升
			{
				up_down_count=0;
				up_down_add=0;
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FBLRFLAG);
				stable_timer=300;
			}
		}
	}
	else
	{
		if(abs_pitch+abs_roll>abs_pitchrollf)
		{
			if(fblr_state!=fblr_nomode)
		  fblr_toup=100;
		}
		abs_pitchrollf=abs_pitch+abs_roll;
	}
	if(hold_high_statue==HOLD_HIGH_KEEP)
	{
		if(fblr_state!=fblr_nomode && (abs_pitch+abs_roll>6))//pitch.roll方向打舵
		{
			fblr_recover=0;
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_ONLYFBLR);
		}
		else if(fblr_state==fblr_nomode)//没打舵或者打舵完成后会中
		{
			fblr_recover++;
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONLYFBLR) && baro_acc_alt.altitude_hat-h_targetf>0 && baro_acc_alt.velocity_hat<0)
			{
				up_down_count=0;
				up_down_add=0;
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FBLRFLAG);
				stable_timer=150+305-fblr_recover;
				fblr_recover=301;
			}
			if(fblr_recover>=300)
			{
				fblr_recover=301;
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONLYFBLR);
			}
		}
	}
//	if(baro_acc_alt.altitude_hat-h_targetf>20 && baro_acc_alt.velocity_hat>50)
//	{
//		fblr_toup=0;
//		up_down_count=stable_timer-1;
//		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FBLRFLAG);
//	}
	fblr_toup++;
	if(fblr_toup<250)
	{
		up_down_count=stable_timer-1;
		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FBLRFLAG);
	}
	else
		fblr_toup=1001;
	//

	if(hold_high_statue==HOLD_HIGH_KEEP)
	{
		if(up_down_count>=0)
		  up_down_count++;
		if(up_down_count<=stable_timer && up_down_count>=0)
		{
			if(up_down_count==stable_timer)
			{
				h_target=baro_acc_alt.altitude_hat+up_down_add;
				if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FBLRFLAG) && MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_UPDOWNFLAG))//定高方向舵，不打油门杆
					h_target=h_targetf;
				h_targetf=h_target;
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FBLRFLAG);
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_UPDOWNFLAG);
			}
			else if(up_down_count<stable_timer)//计数时间未到，则进行上升下降油门杆控制处理
				h_target=baro_acc_alt.altitude_hat+up_down_add;
		}
		else
		{
			up_down_count=-1;
		}
	}
}

#define INER_ALTP 4
#define INER_ALTI 0.002f
#define INER_ALTD 6
#define OUTER_ALTP 0.5f
#define OUTER_ALTI 0.0003f
#define OUTER_ALTD 3

#define OUTER_ILIMIT 100
#define INER_ILIMIT 500

int16 altitude_out(void)
{
	
	int16 out,out_alt;
	//移植李伟翻滚完后状态更新的问题   20180922  
		if(MF_IF_FLAG(FLAGA,FLAGA_FLIP_END_UP))
	{
		MF_CLR_FLAG(FLAGA,FLAGA_FLIP_END_UP);
		
		velocity_error  =0;
		position_error  =0;
		estimate_alti   =0;
		estimate_veli   =0;
		estimate_verrorf=0;
		estimate_aerrorf=0;
		out=0;
		out_alt=0;	
		if(h_target<baro_acc_alt.altitude_hat){
			hold_high_statue=HOLD_HIGH_UP;
			dg_updown=(baro_acc_alt.altitude_hat-h_target)*1.0;
		}
		return 0;
	}	
	alt_target_dis();      //去目标点
	position_error=-baro_acc_alt.altitude_hat+(int)h_target;//外环偏差
	
	estimate_altp=position_error*OUTER_ALTP;
	estimate_altd=(position_error-estimate_aerrorf)*OUTER_ALTD;
	if(fblr_state==fblr_nomode && ak<0.3f && spin_rate_rad_s<0.5f && abs_pitch<5 && abs_roll<5)
	{
		if(position_error<300 && position_error>-300)
		  estimate_alti+=position_error*OUTER_ALTI;//累计历史误差
	}
	if(estimate_alti>=OUTER_ILIMIT) estimate_alti=OUTER_ILIMIT;
	if(estimate_alti<=-OUTER_ILIMIT) estimate_alti=-OUTER_ILIMIT;
	estimate_aerrorf=position_error;
	out_alt=(int16)(estimate_altp+(int)estimate_alti+estimate_altd);
/*******************************************************************/	
	velocity_error=-baro_acc_alt.velocity_hat+(int)out_alt;//内环偏差
	estimate_velp=velocity_error*INER_ALTP;
	estimate_veld=(velocity_error-estimate_verrorf)*INER_ALTD;

	if(hold_high_statue!=HOLD_HIGH_STOP && ak<0.3f && spin_rate_rad_s<0.5f && abs_pitch<15 && abs_roll<15)
	{
		if(position_error<300 && position_error>-300 && fblr_state==fblr_nomode)
		  estimate_veli+=velocity_error*INER_ALTI;//累计历史误差
	}
	if(estimate_veli>=INER_ILIMIT) estimate_veli=INER_ILIMIT;
	if(estimate_veli<=-INER_ILIMIT) estimate_veli=-INER_ILIMIT;
	estimate_verrorf=velocity_error;
	
	//一键下降防止飞机吸顶下不来
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN))
	{
		estimate_alti=0;
		estimate_veli=0;
	}
	
	if(hold_high_statue==HOLD_HIGH_STOP)
	{
		estimate_alti=0;
		estimate_veli=0;
	}
	
	out=(int16)(estimate_velp+(int)estimate_veli+estimate_veld);
	return out;
}
