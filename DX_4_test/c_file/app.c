#include	"config.h"
#include "hr8p506.h"
#include	"flash.h"
#include "uart.h"
#include "app.h"
/* Frank 18.7.19 AM
//               3.61v
//ÉÁµÆ¾¯Ê¾       3.55v
//ÉÁµÆ¾¯Ê¾       3.53v
//30sÍ£Ö¹¶¯×÷    3.23v
*/

// #define	MV_VOL_POWER_LOW_STEP3	3.61//                
// #define	MV_VOL_POWER_LOW_STEP4	3.55//ÉÁµÆ¾¯Ê¾       
// #define	MV_VOL_POWER_LOW_STEP1	3.53//ÉÁµÆ¾¯Ê¾       
// #define	MV_VOL_POWER_LOW_STEP2	3.23//30sÍ£Ö¹¶¯×÷  

#define	MV_VOL_POWER_LOW_TOP	1640
#define	MV_VOL_POWER_LOW_STEP5	1600 
#define	MV_VOL_POWER_LOW_STEP6	1580//Ôö¼ÓµçÑ¹²¹³¥ ZFM 20181009
#define	MV_VOL_POWER_LOW_STEP3	1560 //Ò»¼¶µçÑ¹²¹³¥

#define	MV_VOL_POWER_LOW_STEP4	1540//2390//°´ÕÕÐÇ¸çÒªÇóÐÞ¸Ä   ¶þ¼¶µçÑ¹²¹³¥

#define	MV_VOL_POWER_LOW_STEP1	1520//ÒòÎªµÍÑ¹»º½µºóµçÑ¹»¹ÓÐ3.73×óÓÒ£¬ËùÒÔ½«1530¸ÄÎª1520
#define	MV_VOL_POWER_LOW_STEP2	1345//1355

#define THRO_START  500

//Íâ»·PID²ÎÊý
//Íâ»·±ÈÀýfor normal type
#define	KP_p_1	20.0//12.0f//±ÈÀý-yaw-zÖá5
#define	KP_r_1	20.0//12.0f//±ÈÀý-roll-yÖá5

#define	KP_y_1	1.0f//±ÈÀý-pitch-xÖá5
//Íâ»·»ý·Ö
#define	KI_p_1	0.04f//»ý·Ö-yaw-zÖá0.08
#define	KI_r_1	0.04f//»ý·Ö-roll-yÖá0.08
#define	KI_y_1	0.005f//»ý·Ö-pitch-xÖá
//Íâ»·Î¢·Ö
#define	KD_p_1	0.0f//Î¢·Ö-yaw-zÖá
#define	KD_r_1	0.0f//Î¢·Ö-roll-yÖá
#define	KD_y_1	0.0f//Î¢·Ö-pitch-xÖá

//ÄÚ»·PID²ÎÊý
//ÄÚ»·±ÈÀý
#define	KP_p_2	1.2f//1.2f//±ÈÀý-pitch-xÖá
#define	KP_r_2	1.2f//1.2f//±ÈÀý-roll-yÖá
#define	KP_y_2	12.0//10.1f//±ÈÀý-yaw-zÖá
//#define	KP_y_2	8.1f//±ÈÀý-yaw-zÖá

//ÄÚ»·»ý·Ö
#define	KI_p_2	0.003f//15f//»ý·Ö-pitch-xÖá
#define	KI_r_2	0.003f//15f//»ý·Ö-roll-yÖá
#define	KI_y_2	0.0f//»ý·Ö-yaw-zÖá

//ÄÚ»·Î¢·Ö
#define	KD_p_2	20.0f//Î¢·Ö-pitch-xÖá
#define	KD_r_2	20.0f//Î¢·Ö-roll-yÖá
//#define	KD_y_2	28.0//10.0f//Î¢·Ö-yaw-zÖá
#define	KD_y_2	15.0//10.0f//Î¢·Ö-yaw-zÖá

static	__TheZAxis	lsvst_ThisZAxis={1,0.0f};//Êµ¼ÊµÄzÖá½Ç¶È£¬Ïû³ýËÄÔªÊýµ¼ÖÂµÄZÖáÒç³ö

//#define twoKpDef	(2.0f * 0.1f)	// 2 * proportional gain
//#define twoKiDef	0//(2.0f * 0.000015f)	// 2 * integral gain


static float	MV_RATE_TEL=0.4250f;//Ò£¿ØÆ÷ÁéÃô¶Èdef 0.6
//static float	MV_RATE_TELP=1;
//static float	MV_RATE_TELR=1;
#define	LIMIT_OF_TEL		150.0f//Ò£¿ØÆ÷×ªÍäÏÞ·ù
#define	LIMIT_OF_FALL_RATE	16.0f//·­×ªÏÞ·ù

volatile static	__FP_ACC twoKp =(2.0f * 0.1f); //twoKpDef;// 2 * proportional gain (Kp)=1.0f
volatile static	__FP_ACC twoKi = 0;//twoKiDef;// 2 * integral gain (Ki)=0.0f
volatile static	__FP_ACC integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

volatile static	__FP_ACC	halfT=((__FP_ACC)(MV_FREQ_READ_IMU+1)/(__FP_ACC)MV_FREQ_SYSTEM_TICK)/(__FP_ACC)2;//»¥²¹ÂË²¨Ê±¼ä³£Êì1/2T
volatile static	__FP_ACC	sampleFreq=((__FP_ACC)MV_FREQ_SYSTEM_TICK/(__FP_ACC)(MV_FREQ_READ_IMU+1));//»¥²¹ÂË²¨Ê±¼ä³£ÊìT
volatile	static	__FP_ACC	exInt=0.0f,eyInt=0.0f,ezInt=0.0f;
volatile	static	__FP_ACC	q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
volatile	static	__FP_ACC	ax=0.0f,ay=0.0f,az=0.0f,gx=0.0f,gy=0.0f,gz=0.0f;
volatile	static	__FP_ACC	gyro_x=0.0f,gyro_y=0.0f,gyro_z=0.0f,acc_x=0.0f,acc_y=0.0f,acc_z=0.0f,gyro_x_pid=0.0f,gyro_y_pid=0.0f,gyro_z_pid=0.0f;
volatile	static	__PidPar	birdpitch_1st_link={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},birdroll_1st_link={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},birdyaw_1st_link={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},birdheight_1st_link={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
//µÚÒ»»·½ÚÊä³ö½ÇÔöÁ¿
volatile	static	__PidPar	birdpitch_2nd_link={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},birdroll_2nd_link={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},birdyaw_2nd_link={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},birdheight_2nd_link={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
volatile	static	__FP_ACC	telthrottle=0.0f,telyaw=0.0f,telroll=0.0f,telpitch=0.0f,realthrottle=0.0f,realthrottleFALL=0;
//volatile	static	__FP_ACC  consult_pitch=0.0f,consult_roll=0.0f,consult_yaw=0.0f,consult_pitchF=0.0f,consult_rollF=0.0f;
volatile	static	int			initialThrottleHold=400,standbycnt=0;
volatile	static	__FP_ACC	real_acc_z=0.0f;
volatile	static	int			off_angle_delay=0;
volatile	static  int16   thr_com=0;
volatile   int	   throttle_stick=0,pitch_stick=0,roll_stick=0,yaw_stick=0;	
//static __AVERAGE_DATA throttle_average={0,0,500};


//Íâ»·PID²ÎÊý
volatile	static	__FP_ACC	lvsfp_1st_pitch_kp=KP_p_1;//Íâ»·xÖá¸©ÑöP»·½Ú£¬È±Ê¡Îª1.0
volatile	static	__FP_ACC	lvsfp_1st_pitch_ki=KI_p_1;//Íâ»·xÖá¸©ÑöI»·½Ú£¬È±Ê¡Îª0.0
volatile	static	__FP_ACC	lvsfp_1st_pitch_kd=KD_p_1;//Íâ»·xÖá¸©ÑöD»·½Ú£¬È±Ê¡Îª0.0

volatile	static	__FP_ACC	lvsfp_1st_roll_kp=KP_r_1;//Íâ»·yÖá·­¹öP»·½Ú£¬È±Ê¡Îª1.0
volatile	static	__FP_ACC	lvsfp_1st_roll_ki=KI_r_1;//Íâ»·yÖá·­¹öI»·½Ú£¬È±Ê¡Îª0.0
volatile	static	__FP_ACC	lvsfp_1st_roll_kd=KD_r_1;//Íâ»·yÖá·­¹öD»·½Ú£¬È±Ê¡Îª0.0

volatile	static	__FP_ACC	lvsfp_1st_yaw_kp=KP_y_1;//Íâ»·zÖá×ªÏòP»·½Ú£¬È±Ê¡Îª1.0
volatile	static	__FP_ACC	lvsfp_1st_yaw_ki=KI_y_1;//Íâ»·zÖá×ªÏòI»·½Ú£¬È±Ê¡Îª0.0
volatile	static	__FP_ACC	lvsfp_1st_yaw_kd=KD_y_1;//Íâ»·zÖá×ªÏòD»·½Ú£¬È±Ê¡Îª0.0

//ÄÚ»·PID²ÎÊý
volatile	static	__FP_ACC	lvsfp_2nd_pitch_kp=KP_p_2;//ÄÚ»·xÖá¸©ÑöP»·½Ú£¬È±Ê¡Îª1.5
volatile	static	__FP_ACC	lvsfp_2nd_pitch_ki=KI_p_2;//ÄÚ»·xÖá¸©ÑöI»·½Ú£¬È±Ê¡Îª0.01
volatile	static	__FP_ACC	lvsfp_2nd_pitch_kd=KD_p_2;//ÄÚ»·xÖá¸©ÑöD»·½Ú£¬È±Ê¡Îª5.0

volatile	static	__FP_ACC	lvsfp_2nd_roll_kp=KP_r_2;//ÄÚ»·yÖá·­¹öP»·½Ú£¬È±Ê¡Îª1.5
volatile	static	__FP_ACC	lvsfp_2nd_roll_ki=KI_r_2;//ÄÚ»·yÖá·­¹öI»·½Ú£¬È±Ê¡Îª0.01
volatile	static	__FP_ACC	lvsfp_2nd_roll_kd=KD_r_2;//ÄÚ»·yÖá·­¹öD»·½Ú£¬È±Ê¡Îª5.0

volatile	static	__FP_ACC	lvsfp_2nd_yaw_kp=KP_y_2;//ÄÚ»·zÖá×ªÏòP»·½Ú£¬È±Ê¡Îª1.5
volatile	static	__FP_ACC	lvsfp_2nd_yaw_ki=KI_y_2;//ÄÚ»·zÖá×ªÏòI»·½Ú£¬È±Ê¡Îª0.01
volatile	static	__FP_ACC	lvsfp_2nd_yaw_kd=KD_y_2;//ÄÚ»·zÖá×ªÏòD»·½Ú£¬È±Ê¡Îª5.0

volatile	static	__FP_ACC	gvfpacc_TelMovRangLim=LIMIT_OF_TEL;//Ò£¿ØÆ÷¶¯×÷·ù¶ÈÏÞ·ùÖµ

volatile	static	__FP_ACC	lsvfp_RealYawRate=0.0f;//yaw×Ô×ª½Ç£¬±ÜÃâyawÒª¸É¹éÎ»ºózÖá¸´Î»
volatile	static	__FP_ACC	yaw_dvalue=0.0f;

//·­¹öÊ±ÑÓÊ±¼ÆÊ±Æ÷¼ä¸ôÁ½´Î·­¹ö×îÐ¡Ê±¼ä2s
volatile	static	int		lsv32_FallTwiceTimeCnt=0;
#define		MV_TWICE_FALL_MIN_TIME	2000//Á½´Î·­¹ö×îÐ¡ÑÓÊ±£¬µ¥Î»ºÁÃë
//ÓÍÃÅ»ý·ÖÇåÁã¼ÆÊ±Æ÷£¬ÓÍÃÅ³ÖÐø10´ÎÎª0£¬¼´200msÎª0£¬»ý·ÖÇåÁã
volatile	static	int		lsv32_GetZeroTimeCnt=0;    //ÍÓÂÝÒÇÊý¾ÝÐ£×¼¼ì²âÊ±¼ä
volatile	static	int		lsv32_ClearPidLinkTimeCnt=200;
volatile	static	int		lsv32_ImuCalcTimeCnt=0;
volatile	static	int		lsv32_GetErrTimeCnt=0;
volatile static uint16 flip_finish_timer=0;//Ôö¼Ó·­¹öÍêÖ®ºó¼ÆÊý  ZFM  20180922
uint16_t thr_curve[]={
                       0,0,0,100,128,154,178,200,220,238,
                       255,271,286,300,313,325,336,346,356,366,
											 376,386,396,405,413,420,426,431,437,442,
											 450,460,469,477,484,490,495,499,502,504,
											 510,515,519,522,524,530,535,539,542,544,
											 550,555,559,562,564,570,575,581,588,596,
											 601,607,614,622,627,633,640,648,653,659,
	                     666,674,683,693,704,712,721,731,742,750,
	                     759,769,780,788,797,807,810,820,830,840,
                       };
//
uint16_t thr_gears=0;
uint16_t  motor_start_timer=0;
volatile	static	int		fblr_back_timer=1300;
volatile	static  uint16  motor_zero_timer=0;
volatile	static  uint16	tmp_thr=0;
volatile  float Power_average=0xb07;
volatile  static  float fall_d_com=0.0f;
volatile  static  int  fall_com_timer=0;
volatile  static  uint8  pid_timer=0;											 

#define	MV_FALL_P1	(-LIMIT_OF_FALL_RATE<gvst_ThisBird.EulerAngle.pitch_x&&gvst_ThisBird.EulerAngle.pitch_x<LIMIT_OF_FALL_RATE&&-LIMIT_OF_FALL_RATE<gvst_ThisBird.EulerAngle.roll_y&&gvst_ThisBird.EulerAngle.roll_y<LIMIT_OF_FALL_RATE)
#define	MV_FALL_P2	(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L)&&MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R)&&MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_D)&&MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_F))

volatile	static	int	ls32_FalloffGoUpCnt=0;
volatile	static	__FP_ACC	fall_pitch_off=0.0f,fall_roll_off=0.0f,fall_yaw_off=0.0f;//,fallgyro_pitch_off=0.0f,fallgyro_roll_off=0.0f;
volatile  static  int16 cal_over_timer=0;    //¸ÕÐ£×¼½áÊø½Ç¶È¼ÆËã»áÓÐÒ»»áµÄÆ«²î  Õâ¸öÊÇËãÐ£×¼½áÊøµÄÊ±¼ä
											 
static  float abs_pitch1=0;
static  float abs_roll1=0;
											 
volatile	static	__FP_ACC  no_head_fb=0.0f,no_head_lr=0.0f,com_yawangle=0;
static Breaking Breaking_Car={0,0,0,0,0,0,0,0,0,0,0,0};//É²³µÏà¹Ø±äÁ¿ ZFM 20180920
static  uint8  fall_flag_clear_timer=0,SPPED_GEARS=1;

/************/
//u16 Power_Difference_value[20];
/************/


static	void	ClearPidLink(void)
{//¿ÕÓÍÃÅÊ±Çåpid»·µÄ»ý·Ö
	memset((void*)&birdpitch_1st_link,0,sizeof(__PidPar));      //ÄÚ´æÇåÁã
	memset((void*)&birdroll_1st_link,0,sizeof(__PidPar));
	memset((void*)&birdyaw_1st_link,0,sizeof(__PidPar));
	memset((void*)&birdheight_1st_link,0,sizeof(__PidPar));
	
	memset((void*)&birdpitch_2nd_link,0,sizeof(__PidPar));
	memset((void*)&birdroll_2nd_link,0,sizeof(__PidPar));
	memset((void*)&birdyaw_2nd_link,0,sizeof(__PidPar));
	memset((void*)&birdheight_2nd_link,0,sizeof(__PidPar));
	birdpitch_1st_link.i_err=0;
//	birdroll_1st_link.i_err=-20;
	
//	strike_revise();
}

static	void	EulerSetZero(void)
{
	if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD)){
		q0=1.0f;q1=0.0f;q2=0.0f;q3=0.0f;

		gvst_ThisBird.EulerAngle.pitch_x=0.0f;
		gvst_ThisBird.EulerAngle.roll_y=0.0f;
		gvst_ThisBird.EulerAngle.yaw_z=0.0f;
		lsvfp_RealYawRate=0.0f;
		lsvst_ThisZAxis.quadrant=1;
		lsvst_ThisZAxis.Z_axisAngle=0.0f;
	}
	//motor_zero_timer=0;
}

#define Threshold_Max_Gyro  	 (-32768L)
#define Threshold_Min_Gyro  	 ( 32767L)
#define MAX_MIN_Gyro           (500L  )

#define Threshold_Max_Acc      ( 500L )
#define Threshold_Min_ACC   	 (-500L )
#define Threshold_Max_Acc_Yaw  ( 3000L)
#define Threshold_Min_ACC_Yaw  (-3000L)
#define MIN_Yaw                (-200L )

static	void	IMUSetZero(void)      //ÍÓÂÝÒÇÐ£×¼ 
{
	int	i,pitch=0,roll=0,yaw=0,pitchf,rollf,yawf;
	
	int16 pitch_max=(int16)Threshold_Max_Gyro,roll_max=(int16)Threshold_Max_Gyro,yaw_max=(int16)Threshold_Max_Gyro,
	      pitch_min=(int16)Threshold_Min_Gyro,roll_min=(int16)Threshold_Min_Gyro,yaw_min=(int16)Threshold_Min_Gyro;

	__GyroDat	tmpgyro;
	__AccDat	tmpacc;

	gven_LedFlashKind=EV_LED_1;
	i=0;
	while(i<MV_GET_SENSOR_ZERO_TIM)
	{
		gvst_ThisHal.ReadAcc((void*)&tmpacc);
		if(tmpacc.pitch<Threshold_Max_Acc && (tmpacc.roll<Threshold_Max_Acc) && (tmpacc.pitch>Threshold_Min_ACC) && (tmpacc.roll>Threshold_Min_ACC) && ((tmpacc.yaw>Threshold_Max_Acc_Yaw)||(tmpacc.yaw<Threshold_Min_ACC_Yaw)))
		{
			pitch+=(int16)tmpacc.pitch;
		  roll+=(int16)tmpacc.roll;
		  yaw+=(int16)tmpacc.yaw;
			i++;
			pitchf=(int16)tmpacc.pitch;
		  rollf=(int16)tmpacc.roll;
		  yawf=(int16)tmpacc.yaw;
		}
		else
		{
			pitch-=(int16)pitchf;
		  roll-=(int16)rollf;
		  yaw-=(int16)yawf;
			i--;
			DelayMs(100);
			i=0;
			if(i<=0)
			{
				pitch=0;roll=0;yaw=0;
				i=0;
			}
		}
		if(START_MODE!=1 && tmpacc.yaw<(int16)MIN_Yaw)
		{
			pitch=0;roll=0;yaw=0;
			i=0;
		}
	}
	gvst_ThisBird.AccDatReZero.pitch=pitch/MV_GET_SENSOR_ZERO_TIM;
	gvst_ThisBird.AccDatReZero.roll=roll/MV_GET_SENSOR_ZERO_TIM;
	gvst_ThisBird.AccDatReZero.yaw=yaw/MV_GET_SENSOR_ZERO_TIM;
	
	pitch=0;roll=0;yaw=0;
	i=0;
	while(i<MV_GET_SENSOR_ZERO_TIM)
	{
		gvst_ThisHal.ReadGyro((void*)&tmpgyro);
		if(1)
		{
			if(tmpgyro.pitch>=pitch_max)
				pitch_max=tmpgyro.pitch;
			if(tmpgyro.pitch<=pitch_min)
				pitch_min=tmpgyro.pitch;
			if(tmpgyro.roll>=roll_max)
				roll_max=tmpgyro.roll;
			if(tmpgyro.roll<=roll_min)
				roll_min=tmpgyro.roll;
			if(tmpgyro.yaw>=yaw_max)
				yaw_max=tmpgyro.yaw;
			if(tmpgyro.yaw<=yaw_min)
				yaw_min=tmpgyro.yaw;
			
			pitch+=(int16)tmpgyro.pitch;
		  roll+=(int16)tmpgyro.roll;
		  yaw+=(int16)tmpgyro.yaw;
			i++;
			if(pitch_max-pitch_min>=MAX_MIN_Gyro || (roll_max-roll_min>=MAX_MIN_Gyro) || (yaw_max-yaw_min>=MAX_MIN_Gyro))  //10ok 5ng
			{
				i=0;
				pitch=0;roll=0;yaw=0;
        pitch_max=(int16)Threshold_Max_Gyro,roll_max=(int16)Threshold_Max_Gyro,yaw_max=(int16)Threshold_Max_Gyro,
	      pitch_min=(int16)Threshold_Min_Gyro,roll_min=(int16)Threshold_Min_Gyro,yaw_min=(int16)Threshold_Min_Gyro;
			}
		}
	}
	gvst_ThisBird.GyroDatReZero.pitch=pitch/MV_GET_SENSOR_ZERO_TIM;
	gvst_ThisBird.GyroDatReZero.roll=roll/MV_GET_SENSOR_ZERO_TIM;
	gvst_ThisBird.GyroDatReZero.yaw=yaw/MV_GET_SENSOR_ZERO_TIM;

	//MF_GET_ZERO_HEIGHT();//Ð£×¼¸ß¶È

	MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_ZERO);
	gven_LedFlashKind=EV_LED_3;
	MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_6881OK);
	
	cal_over_timer=0;   //Ð£×¼½áÊø
	EulerSetZero();
	
	dataTX[7]|=0xaa;
	if(START_MODE!=1)
		gyro_acc_datasave();  //Ð£×¼Íê±Ï±£´æÊý¾Ý
}
static	void	BridClear(void)
{
	IMUSetZero();
	EulerSetZero();
	ClearPidLink();
}

void  no_derection_fly(int roll_data,int pitch_data,float nose_now_angle)  //·ÉÐÐÄ£Ê½£¬ÎÞÍ·£¬Õý³£
{
	static double rad_angle;
//	if(hold_high_statue==HOLD_HIGH_STOP)
//		com_yawangle=0;
//	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FH) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
//	  rad_angle=(nose_now_angle)*3.1415926f/180;
//	else
		rad_angle=(nose_now_angle-com_yawangle)*3.1415926f/180;
	
	no_head_lr=(pitch_data*sin(rad_angle)+roll_data*cos(rad_angle));
	no_head_fb=(roll_data*sin(-rad_angle)+pitch_data*cos(rad_angle));
}
static	int	UpdateZ_axis(__FP_ACC		z_yaw)
{//lsvfp_RealYawRate
	static uint8 delay_yawtimer=0,delay_yawtimer1=0;
	yaw_dvalue=-lsvfp_RealYawRate+lsvst_ThisZAxis.Z_axisAngle;
	if(yaw_dvalue>=300)      //
		yaw_dvalue=yaw_dvalue-360;
	if(yaw_dvalue<=-300)
		yaw_dvalue=yaw_dvalue+360;
	lsvst_ThisZAxis.Z_axisAngle=z_yaw;
	if(telyaw!=0)// || realthrottle==0)//ÊµÊ±¸üÐÂlsvfp_RealYawRate
	{
		delay_yawtimer1++;
		if(delay_yawtimer1<100)
			lvsfp_2nd_yaw_kp=1.6f+delay_yawtimer1/20;
		else
			delay_yawtimer1=100;
//		lvsfp_2nd_yaw_kp=1.6f;
		delay_yawtimer=0;
		lsvfp_RealYawRate=lsvst_ThisZAxis.Z_axisAngle;
	}
	else
		delay_yawtimer1=0;
	delay_yawtimer++;
	if(delay_yawtimer<=35)
	{
		if(telyaw==0)
		  lvsfp_2nd_yaw_kp=4.6f;
		lsvfp_RealYawRate=lsvst_ThisZAxis.Z_axisAngle;
	}
	else
	{
		if(delay_yawtimer<70)
		{
			lvsfp_2nd_yaw_kp=4.6f+(delay_yawtimer-35)/7;
		}
		else
		{
			lvsfp_2nd_yaw_kp=KP_y_2;
		  delay_yawtimer=250;
		}
	}
	
	return	lsvfp_RealYawRate;
}
//volatile  uint8 FALL_com=0;
volatile static   int16 FALL_com_down=0;
volatile static  int8 FALL_com_up=0;
volatile static  uint8 angle_up=0;
volatile static  uint8 throttle_up=0;
#define	MV_FALL_ANGLE_OFF	410
#define	MV_ADDOIL_TIME	80//100(N/333)/SEC   //ÐÞ¸Ä·­¹öÉÏÉýÏÂ½µµÄÊ±¼ä
#define	MV_ADDOIL_TIME1	150//(N/333)/SEC
#define	MV_FALL_STEP1_ANGLE	110//add until n angle

//static	__FP_ACC	GetRealRollAngleR(__FP_ACC	rollangle)
//{//ÓÒ·­
//	if(rollangle>=0)
//		return	90+rollangle +0;//90+

//	return	-180-rollangle;
//}
//static	__FP_ACC	GetRealRollAngleL(__FP_ACC	rollangle)
//{//×ó·­
//	if(rollangle<0)
//		return	-90+rollangle -0;//-90+

//	return	180-rollangle;
//}
#define	MV_TELLPRATE	10
#define	RATIO_LIMIT	12

#define FB_NO 0
#define FB_FORWARD 1
#define FB_BACK 2
#define LR_RIGHT 1
#define LR_LEFT 2
//#define FBLR_RIGHT 3
//#define FBLR_LEFT 4
#define Dtaa_Size 14
#define Data_APP_Head 0xa1
#define Data_APP_Tail 0x52
#define Reference_Voltage     2.8
#define Divider_Resistance_1  100//330
#define Divider_Resistance_2  47
#define Resistance            (Divider_Resistance_1+Divider_Resistance_2)
#define Sampling_precision    4096
#define Collect_Number        20  
#define Coefficient           (Reference_Voltage*Resistance/Sampling_precision/Divider_Resistance_2)
void Data_To_APP(void)
{
	static uint8 Special_register1=0X00,Special_register2=0X00,Special_register3=0X00;
   uint8 Send_Data[Dtaa_Size],i;
	uint16 Data_sum=0;
	/**********Special Byte1**************/	 
	/*·­¹ö*/
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
		Special_register1|=0x01;
	else
		Special_register1&=(~0x01);
	/*ÅÄÕÕ*/
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO))
		Special_register1|=0x02;
	else
		Special_register1&=(~0x02);	
	/*Â¼Ïñ*/
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_DV))
		Special_register1|=0x04;
	else
		Special_register1&=(~0x04);	
	/*Æð·É½µÂä*/
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP))
		Special_register1|=0x08;
	else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN))
		Special_register1&=(~0x08);	
	/*·µº½*/
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP))
		Special_register1|=0x10;
	else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN))
		Special_register1&=(~0x10);	
	/*ÎÞÍ·*/
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP))
		Special_register1|=0x20;
	else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN))
		Special_register1&=(~0x20);	
	/*ËÙ¶ÈµµÎ»*/
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP))
		Special_register1|=0xc0;
	else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN))
		Special_register1&=(~0xc0);	
	/**********Special Byte2**************/	 
	/*¶ÔÂë*/
	if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_LOCK_LINK))
		Special_register2|=0x01;
	else
		Special_register2&=(~0x01);
	/*Æ÷¼þ¼ì²â*/
//	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_GET_PHOTO1))
//		Special_register1|=0x02;
//	else
//		Special_register1&=(~0x02);	
	/*µç»ú×´Ì¬*/
//	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_DDV))
//		Special_register2|=0x10;
//	else
//		Special_register2&=(~0x10);	
	/*·É»ú´¹Ö±Î»ÖÃ*/
	if(hold_high_statue!=HOLD_HIGH_STOP)
		Special_register2|=0x20;
	else
		Special_register2&=(~0x20);	
	/*Ð£×¼*/
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_GET_ZERO))
		Special_register2|=0x40;
	else 
		Special_register2&=(~0x40);	
	/*½âËø*/
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK))
		Special_register2|=0x80;
	else 
		Special_register2&=(~0x80);	
	/**********Special Byte3**************/	 
	/*¹âÁ÷Æô¶¯*/
//	if(Optical_Flag==1)
//		Special_register3|=0x01;
//	else
//		Special_register3&=(~0x01);	
	/******************************************/
	Send_Data[0]=Data_APP_Head;
	Send_Data[1]=throttle_stick;
	Send_Data[2]=yaw_stick+0x80;
	Send_Data[3]=pitch_stick+0x80;
	Send_Data[4]=roll_stick+0x80;
	Send_Data[5]=0;
	Send_Data[6]=0;
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1))
	{
		Send_Data[7]=1;//Coefficient*Power_average*10;
	}
else
{
	Send_Data[7]=Coefficient*Power_average*10;
}
	Send_Data[8]=11;
	Send_Data[9]=Special_register1;
	Send_Data[10]=Special_register2;
	Send_Data[11]=Special_register3;
	Send_Data[12]=0;
	for(i=1;i<12;i++)
	{
	 Send_Data[12]+=Send_Data[i];
	}
	Send_Data[Dtaa_Size-1]=Data_APP_Tail;
	hw2181_uart_tx(Send_Data,14);
//	for(i=0;i<14;i++)
//	{
//		UxSend(Send_Data[i]);
//	}
}


//void UxSend(uint8 data)
//{
//	
//	UART0->TBW.Byte[0] =*data;
//	while (!(UART0->IF.TB));
//}
 /*¼ÓÈëÀîÎ°É²³µ¹¦ÄÜ */
		
static void 	Airplan_Breankingcar(Breaking* _beaking,uint8* value)
{
	if((value[Tel_Dat_RollMain]!=0x80||value[Tel_Dat_PitchMain]!=0x80)&&_beaking->Step_1==1) {  //´ò¸Ë
		_beaking->Step_2=1;
		 /*ÀÛ¼Ó*/
		_beaking->Pitch_Sum += pitch_stick;
		_beaking->Roll_Sum  += roll_stick;	
		_beaking->Sum_Times++;
		 /*·´·½Ïò´ò¸ËÇåÁã*/
		if((fabs(_beaking->Pitch_Sum)<fabs(_beaking->Last_Pitch_Sum))||(fabs(_beaking->Roll_Sum)<fabs(_beaking->Last_Roll_Sum))) {
			_beaking->Pitch_Sum = 0;
			_beaking->Roll_Sum  = 0;
			_beaking->Sum_Times = 0;
		}
		/*±£´æ±¾´ÎÀÛ¼ÓÖµ*/
		_beaking->Last_Pitch_Sum = _beaking->Pitch_Sum;
		_beaking->Last_Roll_Sum  = _beaking->Roll_Sum;
		_beaking->Times=2;
	}
	else if(value[Tel_Dat_RollMain]==0x80&&value[Tel_Dat_PitchMain]==0x80&&_beaking->Times>0&&_beaking->Step_2==1){ //»ØÖÐ
		if(_beaking->Step_1==1) { /*Êý¾Ý´¦Àí*/
			_beaking->Step_1=0;
			/*ÇóÆ½¾ù*/
			_beaking->Pitch_Averge= -(_beaking->Pitch_Sum/(_beaking->Sum_Times-1));
			_beaking->Roll_Averge = -(_beaking->Roll_Sum/(_beaking->Sum_Times-1));	
			/*ÏÞ·ù*/
			if(_beaking->Pitch_Averge>120.0)
				_beaking->Pitch_Averge=120.0;
			else if(_beaking->Pitch_Averge<-120.0)
				_beaking->Pitch_Averge=-120.0;	 
			if(_beaking->Roll_Averge>120.0)
				_beaking->Roll_Averge=120.0;		
			else if(_beaking->Roll_Averge<-120.0)
				_beaking->Roll_Averge=-120.0;
			/*¼ÆËãÉ²³µ±¶ÂÊ (ÓÉ´ò¸ËÊ±¼äºÍ´ò¸Ë´óÐ¡Í¬Ê±È·¶¨)*/
			if(_beaking->Sum_Times>200) 
				_beaking->Sum_Times=200;
			_beaking->Multiple_Pitch = (_beaking->Sum_Times*0.01)+(fabs(_beaking->Pitch_Averge)*0.01);
			_beaking->Multiple_Roll  = (_beaking->Sum_Times*0.01)+(fabs(_beaking->Roll_Averge) *0.01);
//			if(_beaking->Multiple_Pitch<=0.8)
//				_beaking->Multiple_Pitch=0.8;
//			if(_beaking->Multiple_Roll<=0.8)
//				_beaking->Multiple_Roll=0.8;
			if(_beaking->Sum_Times<=25)
			{ 
				if(SPPED_GEARS==0||SPPED_GEARS==1)
				{
					_beaking->Multiple_Pitch=0.3;
					_beaking->Multiple_Roll=0.3;
					_beaking->Times=3;
				}
				else
				{
						_beaking->Multiple_Pitch=0.1;
						_beaking->Multiple_Roll=0.1;
						_beaking->Times=3;
				}
			}
			if(_beaking->Sum_Times<=50)
			{
				_beaking->Multiple_Pitch=0.5;
				_beaking->Multiple_Roll=0.5;
				_beaking->Times=7;
			}
			if(_beaking->Sum_Times<=75)
			{
				_beaking->Multiple_Pitch=0.7;
				_beaking->Multiple_Roll=0.7;
				_beaking->Times=15;
			}
			else
			{
				_beaking->Multiple_Pitch=0.8;
				_beaking->Multiple_Roll=0.8;
				_beaking->Times=20;
			}
			
			
			/*Á½¸ö·½ÏòÉÏ¶¼ÓÐÔË¶¯Ê±¼õÉÙ·½´ó±¶Êý*/
			if(fabs(_beaking->Pitch_Averge)>60&&fabs(_beaking->Roll_Averge)>60){
				_beaking->Multiple_Pitch = 0.8;	 
				_beaking->Multiple_Roll  = 0.8;
			}
			/*É²³µÊ±¼ä*/
//			_beaking->Times=((_beaking->Sum_Times>>2)&0x00ff);
//			if(_beaking->Times>20)
//				_beaking->Times=20;
//			else if (_beaking->Times<5)
//				_beaking->Times=5;
		}
		pitch_stick = _beaking->Pitch_Averge * _beaking->Multiple_Pitch;
		roll_stick  = _beaking->Roll_Averge  * _beaking->Multiple_Roll;
		
		if(pitch_stick>125)
			pitch_stick=125;
		else if(pitch_stick<-125)
			pitch_stick=-125;
		
		if(roll_stick>125)
			roll_stick=125;
		else if(roll_stick<-125)
			roll_stick=-125;		
		
		if(_beaking->Times>0){
			_beaking->Times--;
		}
	}
	else {
		/*±¾´Î¸©Ñö¡¢ºá¹öÀÛ¼ÓÖµ*/
		_beaking->Pitch_Sum=0.f;
		_beaking->Last_Pitch_Sum=0.f;	
		/*Ç°Ò»´Î¸©Ñö¡¢ºá¹öÀÛ¼ÓÖµ*/
		_beaking->Roll_Sum=0.f;
		_beaking->Last_Roll_Sum=0.f;	
		/*¸©Ñö¡¢ºá¹öÀÛ¼ÓÆ½¾ù*/
		_beaking->Pitch_Averge=0.f;
		_beaking->Roll_Averge=0.f;
		/*ÀÛ¼Ó´ÎÊý*/
		_beaking->Sum_Times=0;
		/*ÀÛ¼Ó¹ý³Ì*/
		_beaking->Step_1=1;
		/*ÇóÆ½¾ù¹ý³Ì*/
		_beaking->Step_2=0;
		/*É²³µ±¶Êý*/
		_beaking->Multiple_Pitch=0.f;
		/*É²³µ±¶Êý*/
		_beaking->Multiple_Roll=0.f;
		/*É²³µÊ±¼ä*/
		_beaking->Times=0;
  }
}
// É²³µº¯Êý½áÊø
static	void	UpdateTelDat(uint8* dat)
{  
	        uint8	  dat_p[10],itel;
	static  float  yaw_fall_nohead;
	static  int16  data_fallfb,data_falllr;
	static  uint8  slow_down_timer=0,slow_down_detect=2,down_stop_count=0;
	static  uint8  fall_flag_clear_timer=0/*,SPPED_GEARS=1*/;
	
	static  uint8  alt_auto_mode=0;
	static  uint8  check_cal=0,onekeyup_flagclear=0;
	#if Break_Car 
		static uint32 Num=0,num_i=0;
		static float  Roll_Sum=0.0,Pitch_Sum=0.0,Last_Roll_Sum=0.0,Last_Pitch_Sum=0.0,Roll_Num=0.0,Pitch_Num=0.0;
	  float Frank_num=0;
	#endif
	if(dat==NULL)
	{
		return;
	}
//ÓÍÃÅÏÞ·ù765£¬×ËÌ¬ÏÞ·ù+-gvfpacc_TelResponseRate
	for(itel=0;itel<=8;itel++)
	{
		dat_p[itel]=gvst_ThisBird.TelDat[itel];
	}
//	dat_p=(uint8*)dat;
	//Ë«ÖØÐ£Ñé
	check_cal=0xff-(dat_p[0]+dat_p[1]+dat_p[2]+dat_p[3]+dat_p[4]+dat_p[5]+dat_p[6]+dat_p[7]);
	if(check_cal!=dat_p[8] && MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
	{
		return;
	}
	//Ë«ÖØÐ£Ñé
	
	if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
		throttle_stick	=dat_p[Tel_Dat_Throttle];
	if(dat_p[Tel_Dat_PitchMain]<0x8B && dat_p[Tel_Dat_PitchMain]>0x75)//ÐÞ¸ÄÖÐÎ»Öµ  ZFM  20181010

		dat_p[Tel_Dat_PitchMain]=0x80;
	if(dat_p[Tel_Dat_RollMain]<0x8B && dat_p[Tel_Dat_RollMain]>0x75)

		dat_p[Tel_Dat_RollMain]=0x80;

	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1))
	{
		pitch_stick		=dat_p[Tel_Dat_PitchSub]-0xc0+0x80;//*3/5
	  roll_stick		=dat_p[Tel_Dat_RollSub]-0xc0+0x80;
	}
	else
	{
		pitch_stick		=dat_p[Tel_Dat_PitchMain]+(dat_p[Tel_Dat_PitchSub]-0x40)*0.25f/MV_RATE_TEL-0x80;//*3/5
	  roll_stick		=dat_p[Tel_Dat_RollMain]+(dat_p[Tel_Dat_RollSub]-0x40)*0.25f/MV_RATE_TEL-0x80;
		
if(dat_p[Tel_Dat_PitchMain]<0x8B && dat_p[Tel_Dat_PitchMain]>0x75 && dat_p[Tel_Dat_RollMain]<0x8B && dat_p[Tel_Dat_RollMain]>0x75)

			fblr_state=fblr_nomode;
		else
			fblr_state=fblr_yesmode;
			//¼ÓÈëÀîÎ°É²³µº¯Êý
//	if((hold_high_statue==HOLD_HIGH_KEEP)&&(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))&&MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK)&&MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FOF))
//	{Airplan_Breankingcar((Breaking*) &Breaking_Car,dat_p);}
//		pitch_stickf=pitch_stick;
//		roll_stickf=roll_stick;
	}
// 	static int8  Save_Roll[10],Save_Pitch[10];
// 	static uint8  Num=0;
	yaw_stick		=(dat_p[Tel_Dat_Yaw]-0x80);
	
	if(yaw_stick>100) yaw_stick=100;
	if(yaw_stick<-100) yaw_stick=-100;
	

	if(yaw_stick>-30 && (yaw_stick<30))
	{
		yaw_stick=0;
//		pitch_trim=0;
//		roll_trim=0;
	}
	else  //20170531 modify
	{	
		if(yaw_stick<=-30)
		{
//			yaw_stick+=15;

			if(yaw_stick>-50)
				yaw_stick=yaw_stick/2;
			else if(yaw_stick>-80)
				yaw_stick=yaw_stick/1.5f;
		}
		else if(yaw_stick>=30)
		{
//			yaw_stick-=15;
			if(yaw_stick<50)
				yaw_stick=yaw_stick/2;
			else if(yaw_stick<80)
				yaw_stick=yaw_stick/1.5f;
		}
	  Breaking_Car.Step_1=0;
		Breaking_Car.Step_2=0;
	}
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
	{
		pitch_stick=0;
		roll_stick=0;
		yaw_stick=0;
	}

	if(hold_high_statue==HOLD_HIGH_UP || (hold_high_statue==HOLD_HIGH_KEEP) || (hold_high_statue==HOLD_HIGH_DOWN) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
	{//telthrottle	= (__FP_ACC)throttle_stick*4.0f;//step 1.½ö¸üÐÂÓÍÃÅ
		if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
		  lsv32_ClearPidLinkTimeCnt=0;
		if((MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2) && MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT)) || (MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK)&& MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT)))
		{
			slow_down_timer++;
			if(thr_gears<=25)
				slow_down_detect=3;
			else if(thr_gears<=45)
				slow_down_detect=2;
			else
				slow_down_detect=1;
			if(slow_down_timer>=slow_down_detect)
			{
				slow_down_timer=0;
				if(thr_gears>=1)
				{
					if(thr_gears>=32)
					  thr_gears-=2;
					else
						thr_gears-=1;
				}
			}
			tmp_thr=thr_curve[thr_gears];
			telthrottle=tmp_thr;
		}
		else
		{
			thr_gears=(uint16_t)throttle_stick/3;
		  tmp_thr=thr_curve[thr_gears];
			telthrottle=tmp_thr;//+fblr_com+fblr_com1;
			//¶¨¸ß
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT) || MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_PWM_ZERO);
				telthrottle=realthrottlesave;//+(int16)(one_secon/4)*0+thr_com;//THRO_START+thr_com;
				tmp_thr=500;
			}
		}
		motor_zero_timer=0;
	}
	else	if(hold_high_statue==HOLD_HIGH_STOP)
	{
		telthrottle=0;
		lsv32_ClearPidLinkTimeCnt++;
		//Î¢µ÷±£´æ
//  	pitch_trim1=gvst_ThisBird.TelDat[5]-0x40;
//		roll_trim1=gvst_ThisBird.TelDat[6]-0x40;
//		if(pitch_trim1>10) pitch_trim1=10;
//		if(pitch_trim1<-10) pitch_trim1=-10;
//		if(roll_trim1>10) roll_trim1=10;
//		if(roll_trim1<-10) roll_trim1=-10;
//		if(pitch_trim1f!=pitch_trim1 || roll_trim1f!=roll_trim1)
//			gyro_acc_datasave();
//		pitch_trim1f=pitch_trim1;
//		roll_trim1f=roll_trim1;
		//Çå·­¹ö×´Ì¬
//		MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_PWM_ZERO);
//		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1);
//		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST2);
//		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST3);
//		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST4);
//		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_D);
//		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_F);
//		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R);
//		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L);
//		MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE);
//		lvsfp_2nd_pitch_kd=KD_p_2;
//		lvsfp_2nd_roll_kd=KD_p_2;
//		lvsfp_2nd_pitch_kp=KP_p_2;
//		lvsfp_2nd_roll_kp=KP_r_2;
//		lvsfp_1st_pitch_kp=KP_p_1;
//		lvsfp_1st_roll_kp=KP_r_1;
//		lvsfp_2nd_yaw_kp=KP_y_2;
//		fall_pitch_off=0.0f;fall_roll_off=0.0f;fall_yaw_off=0.0f;
		ls32_FalloffGoUpCnt=0;
		motor_start_timer=0;
		Breaking_Car.Step_1=0;
		Breaking_Car.Step_2=0;
	}
	#if	0
	telpitch	=-(__FP_ACC)pitch_stick*MV_RATE_TEL;//Êä³ö[-109,+109]
	telroll		= (__FP_ACC)roll_stick*MV_RATE_TEL;//Êä³ö[-109,+109]
	telyaw		=-(__FP_ACC)yaw_stick*MV_RATE_TEL;//Êä³ö[-109,+109]
	#else

	SPPED_GEARS=dat_p[Tel_Dat_Misc]&0xc0;
	SPPED_GEARS=SPPED_GEARS>>6;
	SPPED_GEARS=SPPED_GEARS&0x03;
	if(SPPED_GEARS==0)
		MV_RATE_TEL=0.12f;
	else if(SPPED_GEARS==1)//ÐÞ¸Ä¶þÈýµµpitchºÍroll·½ÏòËÙ¶È£¬±ÜÃâÇãÐ±½Ç¹ý´óµ¼ÖÂºóÆÚµçÑ¹¹ýµÍµÄÊ±ºò»Ø¸´ÄÜÁ¦½µµÍµ¼ÖÂ·É»úÕðµ´  ZFM  20180922
    MV_RATE_TEL=0.15f;
	//		MV_RATE_TEL=0.18f;
	else if(SPPED_GEARS==2)
    MV_RATE_TEL=0.17f;
	//		MV_RATE_TEL=0.22f;
//	else if(SPPED_GEARS==3)
//		MV_RATE_TEL=0.25f;
	
//		if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT))
		{
			telpitch	=-(__FP_ACC)roll_stick*MV_RATE_TEL;//Êä³ö[-109,+109]
		  telroll		=-(__FP_ACC)pitch_stick*MV_RATE_TEL;//Êä³ö[-109,+109]
		}
	if(SPPED_GEARS==0)
	{
        telyaw		=-(__FP_ACC)yaw_stick*1.5f;//Êä³ö[-109,+109]
    }
	else if(SPPED_GEARS==1)
    {
        telyaw		=-(__FP_ACC)yaw_stick*1.65f;//Êä³ö[-109,+109]
    }
	else if(SPPED_GEARS==2)
	{
        telyaw		=-(__FP_ACC)yaw_stick*1.8f;//Êä³ö[-109,+109]
    }
//	else if(SPPED_GEARS==3)
//	  telyaw		=-(__FP_ACC)yaw_stick*1.9f;//Êä³ö[-109,+109]
	#endif
	//×ªÏòÏÞ·ù
	if(telpitch>gvfpacc_TelMovRangLim)	telpitch=gvfpacc_TelMovRangLim;
	if(telpitch<-gvfpacc_TelMovRangLim)	telpitch=-gvfpacc_TelMovRangLim;
	if(telroll>gvfpacc_TelMovRangLim)	telroll=gvfpacc_TelMovRangLim;
	if(telroll<-gvfpacc_TelMovRangLim)	telroll=-gvfpacc_TelMovRangLim;
//	if(telyaw>gvfpacc_TelMovRangLim)  telyaw=gvfpacc_TelMovRangLim;
//	if(telyaw<-gvfpacc_TelMovRangLim)  telyaw=-gvfpacc_TelMovRangLim;

	//PHOTO&DV
if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1))//ÕâÀï´¦Àí³Æ·ÇµÍÑ¹Çé¿öÏÂ£¬Íê³ÉÅÄÕÕÉãÏñ£¬µÍÑ¹ºóÔò½ûÖ¹µôÅÄÕÕÉãÏñ¹¦ÄÜ  ZFM 20181005  ÕâÑù´¦ÀíµÄÔ­ÒòÊÇ±ÜÃâÔÚµÍÑ¹³öÏÖÊ±£¬Ò£¿Ø°´ÅÄÕÕ±êÖ¾Ò»Ö±Î´Çå³öÏÖÁ¬ÅÄµÄ¹¦ÄÜ
{
	if(dat_p[Tel_Dat_Misc]&0x02)
	{	MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO1);}
	else
	{
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO1))
		{
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO);
			MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_PHOTO1);
		}
	}
	if(dat_p[Tel_Dat_Misc]&0x04)
	{	MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_DV);}
	else
	{	MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_DV);}
}
	
	//´ò¸ËÈ¡Ïû·µº½
	if(dat_p[Tel_Dat_PitchMain]-0x80>10 || dat_p[Tel_Dat_PitchMain]-0x80<-10 || dat_p[Tel_Dat_RollMain]-0x80>10 || dat_p[Tel_Dat_RollMain]-0x80<-10 || telyaw!=0)
		MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FH);
	//ÎÞÍ·Ä£Ê½  ·µº½
	if(hold_high_statue!=HOLD_HIGH_STOP)
	{
		if(dat_p[Tel_Dat_Misc]&0x10)
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FH1);
		else
		{
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FH1))
			{
				if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FH))
					MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FH);
				else
					MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FH);
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FH1);
			}
		}
	}
	
	if(dat_p[Tel_Dat_Misc]&0x20)
	{
		yaw_fall_nohead=lsvst_ThisZAxis.Z_axisAngle-com_yawangle;
		if(yaw_fall_nohead>300) yaw_fall_nohead=yaw_fall_nohead-360;
		if(yaw_fall_nohead<-300) yaw_fall_nohead=yaw_fall_nohead+360;
		if(yaw_fall_nohead>=-45&&yaw_fall_nohead<=45)
		{
			data_fallfb=dat_p[Tel_Dat_PitchMain]-0x80;
		  data_falllr=dat_p[Tel_Dat_RollMain]-0x80;
		}
		else if(yaw_fall_nohead>=-135&&yaw_fall_nohead<=-45)
		{
			data_fallfb=dat_p[Tel_Dat_RollMain]-0x80;
		  data_falllr=-dat_p[Tel_Dat_PitchMain]+0x80;
		}
		else if(yaw_fall_nohead>=45&&yaw_fall_nohead<=135)
		{
			data_fallfb=-dat_p[Tel_Dat_RollMain]+0x80;
		  data_falllr=dat_p[Tel_Dat_PitchMain]-0x80;
		}
		else if(yaw_fall_nohead>=135||yaw_fall_nohead<=-135)
		{
			data_fallfb=-dat_p[Tel_Dat_PitchMain]+0x80;
		  data_falllr=-dat_p[Tel_Dat_RollMain]+0x80;
		}
//		if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_DISTINGUISH_NOHEAD))
//		{
//			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD))
//				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD);
//			else
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD);
//		}
//		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_DISTINGUISH_NOHEAD);
	}
		
	else
	{
		
//		if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_DISTINGUISH_NOHEAD))
//		{
//			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD))
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD);
		
		data_fallfb=dat_p[Tel_Dat_PitchMain]-0x80;
		data_falllr=dat_p[Tel_Dat_RollMain]-0x80;
//			else
//				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD);
//		}
//		MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_DISTINGUISH_NOHEAD);
	}
	if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD))
		com_yawangle=lsvst_ThisZAxis.Z_axisAngle;
		
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD))
	{
		no_derection_fly(roll_stick,pitch_stick,lsvst_ThisZAxis.Z_axisAngle);
		telpitch=-no_head_lr*MV_RATE_TEL;
		telroll=-no_head_fb*MV_RATE_TEL;
	}

	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FH))// || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
	{
		if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD)) {  //ÎÞÍ·Ä£Ê½ÏÂ·­¹ö
			if(SPPED_GEARS==0)
				telroll=7;
			else if(SPPED_GEARS==1)
				telroll=7;
			else if(SPPED_GEARS==2)
				telroll=13;
//			else if(SPPED_GEARS==3)
//				telroll=13;				
		}
		else
		{
			if(SPPED_GEARS==0)
				no_derection_fly(roll_stick,-50,lsvst_ThisZAxis.Z_axisAngle);
			else if(SPPED_GEARS==1)
				no_derection_fly(roll_stick,-50,lsvst_ThisZAxis.Z_axisAngle);
			else if(SPPED_GEARS==2)
				no_derection_fly(roll_stick,-100,lsvst_ThisZAxis.Z_axisAngle);
//			else if(SPPED_GEARS==3)
//				no_derection_fly(roll_stick,-100,lsvst_ThisZAxis.Z_axisAngle);						
			telpitch=-no_head_lr*MV_RATE_TEL;
			telroll=-no_head_fb*MV_RATE_TEL;
		}
//		no_derection_fly(roll_stick,-40,lsvst_ThisZAxis.Z_axisAngle);
//		telpitch=-no_head_fb*MV_RATE_TEL;
//		telroll=-no_head_lr*MV_RATE_TEL;
	}

	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FOF))
	{
		telroll=-(dat_p[Tel_Dat_PitchSub]-0x40)*0.25f;
		telpitch=-(dat_p[Tel_Dat_RollSub]-0x40)*0.25f;
	}
	
	if(hold_high_statue!=HOLD_HIGH_STOP)
	  down_stop_count=0;
	else
	{
		if(ak>1.5f)
			down_stop_count=0;
		else
			down_stop_count++;
		if(down_stop_count>200) down_stop_count=200;
	}
	if(dat_p[Tel_Dat_Misc]&0x08 && MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT))
	{
		if(hold_high_statue==HOLD_HIGH_STOP)
		{
			if(down_stop_count>=50 && onekeyup_flagclear>80)
			{
				down_stop_count=200;
				if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN) && MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2) && MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK))   //Ã»ÓÐ½øÈë¶þ¼¶µÍÑ¹
				{
					onekeyup_flagclear=0;
					MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP);
					MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN);
				}
			}
		}
		else
		{
			if(onekeyup_flagclear>95)
			{
				onekeyup_flagclear=0;
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN);
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP);
			}
		}	
	}
	onekeyup_flagclear++;//Ò»¼üÏÂ½µ½âËø
	if(onekeyup_flagclear>200) onekeyup_flagclear=200;
	//¶¨¸ß£¬ÊÖ¶¯ÇÐ»»
//	if(!(dat_p[Tel_Dat_Misc]&0x20))
	if(1)
	{
		alt_auto_mode++;
		if(alt_auto_mode>=1)
		{
			alt_auto_mode=2;
			if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT) && MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))// && hold_high_statue==HOLD_HIGH_STOP)
			{
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT);
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT1);
			}
		}
	}
	else
	{
		alt_auto_mode--;
		if(alt_auto_mode==0)
		{
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT);
		  MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT1);
		  alt_auto_mode=0;
		}
	}
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT))
	  check_flymode((uint8)throttle_stick,1);
	else
		check_flymode((uint8)throttle_stick,0);
	
	if(((dat_p[Tel_Dat_Misc]&0x01)==0x01))
	{
		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FOF);
		fall_flag_clear_timer=0;
	}
	fall_flag_clear_timer++;
	if(fall_flag_clear_timer>=5)
	{
		fall_flag_clear_timer=11;
		MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FOF);
	}
	//¸úÍ·
	//Ìõ¼þ1Å·À­½Ç¹ýÁã2ÓÍÃÅãÐÖµÒÔÉÏ3ÉÏ´Î·­¹öÆ½ºâºó4Õý¸º16¶ÈÒÔÄÚ
	if((MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FOF))&&telthrottle>0 &&(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S4))/* (MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1))*/)//ÔÚS4ÖÃÆðÖ®ºó½ûÖ¹µô·­¹ö   ZFM  20180928
	{
		//off_angle_delay=25;
        if((data_falllr<-32) && (MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)))//you    ¸ÄÎª   ZFM ×ó
//		if(dat_p[Tel_Dat_RollMain]<0x60 && (MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)))//you
		{
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_D);
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1);
            ls32_FalloffGoUpCnt=MV_ADDOIL_TIME+FALL_com_up;
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_BEHIDE);
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE);
		}
        else if((data_falllr>32)&& (MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)))//zuo   ¸ÄÎª  ZFM ÓÒ
//		else if(dat_p[Tel_Dat_RollMain]>0xa0 && (MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)))//zuo
		{
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_F);
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1);
			ls32_FalloffGoUpCnt=MV_ADDOIL_TIME+FALL_com_up;
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE);
		}
        else if((data_fallfb<-32) && (MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)))//qian  ¸ÄÎª ZFM ºó
//		else if(dat_p[Tel_Dat_PitchMain]<0x60 && (MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)))//qian
		{
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R);
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1);
			ls32_FalloffGoUpCnt=MV_ADDOIL_TIME+FALL_com_up+10;
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE);
		}
        else if((data_fallfb>32) && (MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)))//hou   ¸ÄÎª ZFM   Ç°
//		else if(dat_p[Tel_Dat_PitchMain]>0xa0 && (MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE)))//hou
		{
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L);
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1);
			ls32_FalloffGoUpCnt=MV_ADDOIL_TIME+FALL_com_up+10;
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE);
		}
	}
}

#define PITCH_FCOM 50
#define ROLL_BCOM 20
static	void	PowerValueUpdateForFall(const	__Euler *euler)
{
	static uint16 FALL_timer=0;
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
		FALL_timer++;
	else
		FALL_timer=0;
	if(FALL_timer>=1000 || hold_high_statue==HOLD_HIGH_STOP)
	{
	//µôÏßÇå·­¹ö±êÖ¾Î»
		MF_CLR_FLAG(FLAGA,FLAGA_FLIP_FINISH);
		MF_CLR_FLAG(FLAGA,FLAGA_FLIP_END_UP);
		MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE);
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1);
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST2);
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST3);
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST4);
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_D);
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_F);
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R);
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L);
		MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FOF);
        MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FFLAG);   //HAVE FALL  //Õâ¸ö±êÖ¾Î»ÓÃÓÚÔÚ·­¹öµÄÊ±ºòÏû³ýyaw·½ÏòµÄÐ§Ó¦  chen-yunzhong  teach ZFM 20180429
		
//    lvsfp_2nd_pitch_kd=KD_p_2;
//	  lvsfp_2nd_roll_kd=KD_p_2;
		lvsfp_2nd_pitch_kd=KD_p_2;
		lvsfp_2nd_roll_kd=KD_p_2;
		
		lvsfp_2nd_pitch_kp=KP_p_2;
		lvsfp_2nd_roll_kp=KP_r_2;
		
		lvsfp_1st_pitch_kp=KP_p_1;
		lvsfp_1st_roll_kp=KP_r_1;
		
		lvsfp_2nd_yaw_kp=KP_y_2;
		lvsfp_2nd_yaw_kd=KD_y_2;
		
		fall_pitch_off=0;
		fall_roll_off=0;
		fall_yaw_off=0.0f;
	}
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1))
		{//step 1st.¼ÓËÙ½×¶Î
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT);
//			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S3)||MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S5))//¼Ó´óÁ¦¶È
//			{
//				realthrottleFALL=900;
//			}
//			else
			
			{ realthrottleFALL=850;}//tmp_thr+300;    ZFM
//		telthrottle=800;//tmp_thr+300;
		if(ls32_FalloffGoUpCnt--<=0)
		{
			MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST1);
			MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST2);
			ls32_FalloffGoUpCnt=0;
		}
	}

	else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST2))
	{//step 2nd.¿ªÊ¼·­×ª-·­×ªµÚÒ»±êÖ¾½×¶Î£¬¼´2/4ÏóÏÞÄÚ
		MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_FFLAG);   //HAVE FALL
        realthrottleFALL=0;//ZFM  20180428  ÖØÐÂÉèÖÃ·­¹ö²ßÂÔ
		lvsfp_1st_pitch_kp=15;
		lvsfp_1st_roll_kp=15;
		lvsfp_2nd_pitch_kp=1.0f;
		lvsfp_2nd_roll_kp=1.0f;

        lvsfp_2nd_pitch_kd=30;
		lvsfp_2nd_roll_kd=30;
//		lvsfp_2nd_yaw_kp=8;
//		lvsfp_2nd_yaw_kd=35;
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//¸ÄÎªZFM  Ç°ºó roll_y
		{
			  lvsfp_1st_roll_kp=17;
		    lvsfp_1st_pitch_kp=15;
				lvsfp_2nd_roll_kp=1.0f;
		    lvsfp_2nd_pitch_kp=8.0f;

        lvsfp_2nd_roll_kd=30;
		    lvsfp_2nd_pitch_kd=10;
//			lvsfp_2nd_pitch_kd=20;
//			lvsfp_2nd_roll_kd=9;
		}
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_D) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_F))//ZFM ×óÓÒ pitch_x
		{
				lvsfp_1st_pitch_kp=17;
		    lvsfp_1st_roll_kp=15;
				lvsfp_2nd_pitch_kp=1.0f;
		    lvsfp_2nd_roll_kp=2.5f;

        lvsfp_2nd_pitch_kd=30;
		    lvsfp_2nd_roll_kd=20;
//			lvsfp_2nd_pitch_kd=5;
//			lvsfp_2nd_roll_kd=20;
		}
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_D))//ZFM ×ó
		{//ºó·­£¬±êÖ¾½×¶ÎÎª-135¶È
			motor_start_timer=0;
			fall_pitch_off=-MV_FALL_ANGLE_OFF-30-angle_up;//¼Ó´óÒÔºó¿ÉÒÔ·ÀÖ¹·­¹ýÍ·
			fall_roll_off=-gvst_ThisBird.EulerAngle.roll_y;
			fall_yaw_off=-yaw_dvalue;
			if(gvst_ThisBird.EulerAngle.pitch_x<-MV_FALL_STEP1_ANGLE)
			{//Ö±µ½·­Ãæ
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST2);
//				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST3);
				MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST4);
			}
		}
		else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_F))//ZFM ÓÒ
		{//Ç°·­£¬±êÖ¾½×¶ÎÎª135¶È
			motor_start_timer=0;
			fall_pitch_off=MV_FALL_ANGLE_OFF+30+angle_up;
			fall_roll_off=-gvst_ThisBird.EulerAngle.roll_y;
			fall_yaw_off=-yaw_dvalue;
			if(gvst_ThisBird.EulerAngle.pitch_x>MV_FALL_STEP1_ANGLE)
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST2);
//				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST3);
				MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST4);
			}
		}
		else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//QIAN ¸ÄÎªZFM ºó
		{//ÓÒ·­£¬±êÖ¾½×¶ÎÎª¹ýÁãµã
			motor_start_timer=0;
			fall_pitch_off=-gvst_ThisBird.EulerAngle.pitch_x;
			fall_roll_off=-MV_FALL_ANGLE_OFF-30-angle_up;
			fall_yaw_off=-yaw_dvalue;
			if(gvst_ThisBird.EulerAngle.roll_y<-MV_FALL_STEP1_ANGLE)
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST2);
//				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST3);
				MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST4);
			}
		}
		else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L))//HOU  ¸ÄÎªZFM  Ç°
		{//×ó·­£¬±êÖ¾½×¶ÎÎª¹ýÁãµã
			motor_start_timer=0;
			fall_pitch_off=-gvst_ThisBird.EulerAngle.pitch_x;
			fall_roll_off=MV_FALL_ANGLE_OFF+30+angle_up;
			fall_yaw_off=-yaw_dvalue;
			if(gvst_ThisBird.EulerAngle.roll_y>MV_FALL_STEP1_ANGLE)
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST2);
//				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST3);
				MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST4);
			}
		}
	}
	else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST4))
	{//step 4th.·­×ªÉ²³µ½×¶Î£¬¼´4/4ÏóÏÞ
//		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))
//		{
//			lvsfp_2nd_pitch_kd=KD_p_2-5;
//		}
//		if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S4)||MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S5))//ÔÚS4µçÑ¹ÒÔºó½ûÖ¹µô·­¹ö  20181007
//		{
//			lvsfp_2nd_pitch_kd=35;
//		}
//		else
		
		{   lvsfp_2nd_pitch_kd=30;}
//	    lvsfp_2nd_roll_kd=KD_p_2+5;//¶ÔÓ¦µÄÊÇ×óÓÒµÄPID¿ØÖÆ  Ä¿Ç°Õâ¸ö²ÎÊýÏÂ£¬×óÓÒOK  20180531  ÐÞ¸ÄÎªÇ°ºóµÄPID¿ØÖÆ²ÎÊýPID  20180607
        realthrottleFALL=650;//ZFM   20180428
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_F))////ZFM ÓÒ
		{//Ç°·­
			motor_start_timer=0;
//			telthrottle=tmp_thr;
			fall_pitch_off=0;
			fall_roll_off=-gvst_ThisBird.EulerAngle.roll_y;
			fall_yaw_off=-yaw_dvalue;
			if(85>gvst_ThisBird.EulerAngle.pitch_x && 0<gvst_ThisBird.EulerAngle.pitch_x)
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_F);
				ls32_FalloffGoUpCnt=MV_ADDOIL_TIME1+FALL_com_down;
				fall_roll_off=0.0f;fall_pitch_off=0.0f;fall_yaw_off=0.0f;
			}
		}
		else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_D))//ZFM ×ó
		{//ºó·­
			motor_start_timer=0;
//			telthrottle=tmp_thr;
			fall_pitch_off=0;
			fall_roll_off=-gvst_ThisBird.EulerAngle.roll_y;
			fall_yaw_off=-yaw_dvalue;
			if(-85<gvst_ThisBird.EulerAngle.pitch_x && gvst_ThisBird.EulerAngle.pitch_x<0)
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_D);
				ls32_FalloffGoUpCnt=MV_ADDOIL_TIME1+FALL_com_down;
				fall_roll_off=0.0f;fall_pitch_off=0.0f;fall_yaw_off=0.0f;
			}
		}
		else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//QIAN¸ÄÎªZFM ºó
		{//ÓÒ·­
            lvsfp_2nd_roll_kd=35;
			motor_start_timer=0;
//			telthrottle=tmp_thr;
			fall_pitch_off=-gvst_ThisBird.EulerAngle.pitch_x;
			fall_roll_off=0;
			fall_yaw_off=-yaw_dvalue;
			if(-85<gvst_ThisBird.EulerAngle.roll_y && gvst_ThisBird.EulerAngle.roll_y<0)
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R);
				ls32_FalloffGoUpCnt=MV_ADDOIL_TIME1+FALL_com_down;
				fall_roll_off=0.0f;fall_pitch_off=0.0f;fall_yaw_off=0.0f;
			}
		}
		else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L))//HOU  ¸ÄÎªZFM  Ç°
		{//×ó·­
            lvsfp_2nd_roll_kd=40;
			motor_start_timer=0;
//			telthrottle=tmp_thr;//+200;
			fall_pitch_off=-gvst_ThisBird.EulerAngle.pitch_x;
			fall_roll_off=0;
			fall_yaw_off=-yaw_dvalue;
			if(85>gvst_ThisBird.EulerAngle.roll_y && gvst_ThisBird.EulerAngle.roll_y>0)
			{
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L);
				ls32_FalloffGoUpCnt=MV_ADDOIL_TIME1+FALL_com_down;
				fall_roll_off=0.0f;fall_pitch_off=0.0f;fall_yaw_off=0.0f;
			}
		}
		else
		{//step 1st.¼ÓËÙ½×¶Î
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FFLAG);   //HAVE END
//        if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))
//		{
//		   realthrottleFALL=700;
//		}
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S3)||MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S5)||(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S6)) )
			{
				realthrottleFALL=850;
			}
			else
      {      realthrottleFALL=830;}
//			telthrottle=800;//tmp_thr+200;
			
			lvsfp_2nd_pitch_kp=2.5f;
			lvsfp_2nd_roll_kp=2.5f;
			lvsfp_1st_pitch_kp=15;
			lvsfp_1st_roll_kp=15;

			if(ls32_FalloffGoUpCnt--<=0)
			{
          lvsfp_2nd_pitch_kp=KP_p_2;
			    lvsfp_2nd_roll_kp=KP_r_2;
                
				  lvsfp_2nd_pitch_kd=KD_p_2;
			    lvsfp_2nd_roll_kd=KD_p_2;
				
          lvsfp_1st_pitch_kp=KP_p_1;
			    lvsfp_1st_roll_kp=KP_r_1;
				lvsfp_2nd_yaw_kp=KP_y_2;
				lvsfp_2nd_yaw_kd=KD_y_2;
				
//				if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S4)||MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S5))
//				{
//					flip_finish_timer=250;//350;//400;
//				}
//				else
				MF_SET_FLAG(FLAGA,FLAGA_FLIP_FINISH);//Ôö¼Ó·­¹öÍêÒÔºóÑÓÊ±´¦Àí¶¯×
					if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S3))
				{
//					MF_SET_FLAG(FLAGA,FLAGA_FLIP_FINISH);//Ôö¼Ó·­¹öÍêÒÔºóÑÓÊ±´¦Àí¶¯×
					flip_finish_timer=140;//250;//zfm 20180922
					
				}
				else if (MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S6))
				{
					flip_finish_timer=130;
				}
				else if (MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S5))
				{
//					MF_SET_FLAG(FLAGA,FLAGA_FLIP_FINISH);//Ôö¼Ó·­¹öÍêÒÔºóÑÓÊ±´¦Àí¶¯×
					flip_finish_timer=110;//250;//zfm 20180922
				}
				else if(MF_IF_FLAG(FLAGA,FLAGA_POWER_LOW_TOP))
				{
					flip_finish_timer=90;
				}
				else
				{
					flip_finish_timer=100;
				}
				
				if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT1)/*&&MF_IF_NOFLAG(FLAGA,FLAGA_FLIP_FINISH)*/)
					MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT);
				MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_ST4);
				ls32_FalloffGoUpCnt=0;
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE);

			}
		}
	}
}
//ÄÚ»·PID-2nd
//Íâ»·PIDÏÞ·ù
#define	INER_ILIMIT	5
#define	OUT_ILIMIT	100
#define OUTER 1
#define INER 0
static	void	UpdatePidParLoop(__PidPar* par,__FP_ACC tar,__FP_ACC cur,__FP_ACC ratio_p,__FP_ACC ratio_i,__FP_ACC ratio_d,int dt,uint8 iner_outer)
{
	__FP_ACC	err=0.0f,i_err=0.0f,d_err=0.0f,dd_err=0.0f,results=0.0f;
	
	err=tar-cur;
	i_err=par->i_err+ratio_i*err*(float)dt/8000;
	d_err=err-par->p_err;
	dd_err=d_err-par->d_err;

	par->tar=tar;
	par->cur=cur;
	par->p_err=err;
	if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))   //·­¹ö²»»ý·Ö
	{
		if(fblr_state!=fblr_nomode)   //´ò¶æ»ý·Ö
		{
			if(telyaw==0)
				par->i_err=i_err;
		}
		else if(abs_pitch1<5 && abs_roll1<5)  //²»´ò¶æÇÒ½Ç¶ÈÐ¡²Å»ý·Ö
		{
			if(telyaw==0)
				par->i_err=i_err;
		}
	}
	d_err=d_err*ratio_d*8000/(float)dt;
//	d_err=0.6f*d_err+0.4f*par->d_err;
	par->d_err=d_err;
	par->dd_err=dd_err;
	
	err=err*ratio_p;
//	err=err*0.8f+d_err*0.2f;
//	d_err=d_err*0.8f+dd_err*0.2f;
	if(iner_outer==OUTER)
	{
		if(par->i_err>OUT_ILIMIT)	par->i_err=OUT_ILIMIT;
	  if(par->i_err<-OUT_ILIMIT)	par->i_err=-OUT_ILIMIT;
	}
	else
	{
		if(par->i_err>INER_ILIMIT)	par->i_err=INER_ILIMIT;
	  if(par->i_err<-INER_ILIMIT)	par->i_err=-INER_ILIMIT;
	}

	results=err+par->i_err+d_err;
	par->results=results;
}

float	MV_GYRO_LPR=0,MV_GYRO_LPR1=0;
#define	MV_ACC_LPR	0.55f
#define LIMIT_16INT 32767
static	void	UpdateAhrsData(void)  //ÍÓÂÝÒÇ ¼ÓËÙ¶È¼ÆÊý¾Ý¸üÐÂ  ZFM
{//MV_RAD_DEG
	//static	__FP_ACC	tmp_gyro_x=0,tmp_gyro_y=0,tmp_gyro_z=0;
	static	int16	tmp_acc_x=0,tmp_acc_y=0,tmp_acc_z=0;
	__AccDat	tmp_acc={0,0,0};
	int read_6050_dat[3]={0,0,0};
  static int read_6050_datf[3]={0,0,0},read_6050_datf1[3]={0,0,0};
	static	__SingleKlmPar	gyroKlmParInUpdateAhrsData[3]={{0.0f,1.0f,0.00001f,0.002f},{0.0f,1.0f,0.00001f,0.002f},{0.0f,1.0f,0.00001f,0.002f}};//{}
	
	gvst_ThisHal.ReadGyro((void*)&gvst_ThisBird.GyroDatNow);
	read_6050_dat[0]=gvst_ThisBird.GyroDatNow.pitch;
	read_6050_dat[1]=gvst_ThisBird.GyroDatNow.roll;
	read_6050_dat[2]=gvst_ThisBird.GyroDatNow.yaw;
	read_6050_dat[0]-=gvst_ThisBird.GyroDatReZero.pitch;
	read_6050_dat[1]-=gvst_ThisBird.GyroDatReZero.roll;
	read_6050_dat[2]-=gvst_ThisBird.GyroDatReZero.yaw;
	read_6050_dat[0]=Fylib_Constrain(read_6050_dat[0],-LIMIT_16INT,LIMIT_16INT);
	read_6050_dat[1]=Fylib_Constrain(read_6050_dat[1],-LIMIT_16INT,LIMIT_16INT);
	read_6050_dat[2]=Fylib_Constrain(read_6050_dat[2],-LIMIT_16INT,LIMIT_16INT);
	
//	if(read_6050_dat[0]>-9 && read_6050_dat[0]<9) read_6050_dat[0]=0;
//	if(read_6050_dat[1]>-9 && read_6050_dat[1]<9) read_6050_dat[1]=0;
//	if(read_6050_dat[2]>-5 && read_6050_dat[2]<5) read_6050_dat[2]=0;
	
if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE) && spin_rate_rad_s<0.3f)
	{
		MV_GYRO_LPR1=0.5f;
		MV_GYRO_LPR=0.6f;
//		MV_GYRO_LPR1=0.00f;
//		MV_GYRO_LPR=0.0f;
	}
	else
	{
				MV_GYRO_LPR=0.6f;
		MV_GYRO_LPR1=0;
//		MV_GYRO_LPR=0;
//		MV_GYRO_LPR1=0;
	}
		
	read_6050_datf[0]=read_6050_datf[0]*MV_GYRO_LPR+read_6050_dat[0]*(1-MV_GYRO_LPR);
	read_6050_datf[1]=read_6050_datf[1]*MV_GYRO_LPR+read_6050_dat[1]*(1-MV_GYRO_LPR);
//	read_6050_datf[2]=read_6050_datf[2]*MV_GYRO_LPR+read_6050_dat[2]*(1-MV_GYRO_LPR);
	read_6050_datf[2]=read_6050_datf[2]*0+read_6050_dat[2]*(1-0);//ÐÞ¸Äº½ÏòÂË²¨  ZFM 20181007
	
	gyro_x_pid=(__FP_ACC)(read_6050_datf[0])/(__FP_ACC)MV_GYRO_ANG_RP;
	gyro_y_pid=(__FP_ACC)(read_6050_datf[1])/(__FP_ACC)MV_GYRO_ANG_RP;
	gyro_z_pid=(__FP_ACC)(read_6050_datf[2])/(__FP_ACC)MV_GYRO_ANG_RP;
	
	read_6050_datf1[0]=read_6050_datf1[0]*MV_GYRO_LPR1+read_6050_dat[0]*(1-MV_GYRO_LPR1);
	read_6050_datf1[1]=read_6050_datf1[1]*MV_GYRO_LPR1+read_6050_dat[1]*(1-MV_GYRO_LPR1);
//	read_6050_datf1[2]=read_6050_datf1[2]*MV_GYRO_LPR1+read_6050_dat[2]*(1-MV_GYRO_LPR1);
	read_6050_datf1[2]=read_6050_datf1[2]*0+read_6050_dat[2]*(1-0);//ÐÞ¸Äº½ÏòÂË²¨ ZFM 20181007
    //ZFM   ÐÞ¸ÄÊýÖµÓÃÓÚ·­¹ö
//    gyro_x=(__FP_ACC)(read_6050_datf1[0])/(__FP_ACC)(MV_GYRO_ANG_RP-15);
//	gyro_y=(__FP_ACC)(read_6050_datf1[1])/(__FP_ACC)(MV_GYRO_ANG_RP-15);
	gyro_x=(__FP_ACC)(read_6050_datf1[0])/(__FP_ACC)MV_GYRO_ANG_RP;
	gyro_y=(__FP_ACC)(read_6050_datf1[1])/(__FP_ACC)MV_GYRO_ANG_RP;
	gyro_z=(__FP_ACC)(read_6050_datf1[2])/(__FP_ACC)MV_GYRO_ANG_RP;
	
	gvst_ThisBird.GyroDatNow.pitch=(int16)read_6050_datf1[0];  //Ç°¸ººóÕý
	gvst_ThisBird.GyroDatNow.roll=(int16)read_6050_datf1[1];   //ÓÒÕý×ó¸º
	gvst_ThisBird.GyroDatNow.yaw=(int16)read_6050_datf1[2];     //ÄæÕýË³¸º
	
	gvst_ThisBird.GyroDatNow.pitch=(int16)read_6050_datf[0];  //Ç°¸ººóÕý
	gvst_ThisBird.GyroDatNow.roll=(int16)read_6050_datf[1];   //ÓÒÕý×ó¸º
	gvst_ThisBird.GyroDatNow.yaw=(int16)read_6050_datf[2];     //ÄæÕýË³¸º

	gvst_ThisHal.ReadAcc((void*)&tmp_acc);
	read_6050_dat[0]=tmp_acc.pitch;
	read_6050_dat[1]=tmp_acc.roll;
	read_6050_dat[2]=tmp_acc.yaw;
	read_6050_dat[0]-=gvst_ThisBird.AccDatReZero.pitch;
	read_6050_dat[1]-=gvst_ThisBird.AccDatReZero.roll;
	read_6050_dat[2]-=gvst_ThisBird.AccDatReZero.yaw;
	read_6050_dat[0]=Fylib_Constrain(read_6050_dat[0],-LIMIT_16INT,LIMIT_16INT);
	read_6050_dat[1]=Fylib_Constrain(read_6050_dat[1],-LIMIT_16INT,LIMIT_16INT);
	read_6050_dat[2]=Fylib_Constrain(read_6050_dat[2],-LIMIT_16INT,LIMIT_16INT);
	tmp_acc.pitch=(int16)read_6050_dat[0];
	tmp_acc.roll=(int16)read_6050_dat[1];
	tmp_acc.yaw-=gvst_ThisBird.AccDatReZero.yaw;
	tmp_acc.yaw+=MV_ACC_RP;
	
	real_accz=tmp_acc.yaw-gvst_ThisBird.AccDatReZero.yaw;
	if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))  //ÎÞ·­¹öÂË²¨
	{
		tmp_acc_x=(int16)(tmp_acc_x*MV_ACC_LPR+tmp_acc.pitch*(1-MV_ACC_LPR));
		tmp_acc_y=(int16)(tmp_acc_y*MV_ACC_LPR+tmp_acc.roll*(1-MV_ACC_LPR));
		tmp_acc_z=(int16)(tmp_acc_z*MV_ACC_LPR+tmp_acc.yaw*(1-MV_ACC_LPR));
//		Fylib_LpFilter(MV_ACC_LPR,(float)tmp_acc.pitch,(float*)&tmp_acc_x);
//		Fylib_LpFilter(MV_ACC_LPR,(float)tmp_acc.roll,(float*)&tmp_acc_y);
//		Fylib_LpFilter(MV_ACC_LPR,(float)tmp_acc.yaw,(float*)&tmp_acc_z);
	}
	else
	{
		tmp_acc_x=0;//(int16)tmp_acc.pitch;
		tmp_acc_y=0;//(int16)tmp_acc.roll;
		tmp_acc_z=MV_ACC_RP;//(int16)tmp_acc.yaw;
	}
//			DT_Send_Senser((int16) (tmp_acc.pitch),(int16) (tmp_acc_x),(int16) (tmp_acc.roll),
//										(int16) (tmp_acc_y),(int16) (tmp_acc.yaw),(int16) (tmp_acc_z),
//										0,0,0,
//										0);	
	ak=(1/Fylib_Qrsqrt(tmp_acc_x*tmp_acc_x + tmp_acc_y*tmp_acc_y+tmp_acc_z*tmp_acc_z)-MV_ACC_RP)/MV_ACC_RP;
		if(ak<0) ak=-ak;
	gvst_ThisBird.AccDatNow.pitch=(int16)tmp_acc_x;
	gvst_ThisBird.AccDatNow.roll=(int16)tmp_acc_y;
	gvst_ThisBird.AccDatNow.yaw=(int16)tmp_acc_z;
	
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_ZERO))
	  accel_baro_mag_gps_calculator(gvst_ThisBird.AccDatNow.pitch,gvst_ThisBird.AccDatNow.roll,gvst_ThisBird.AccDatNow.yaw,gvst_ThisBird.AccDatReZero.yaw,H_I_average);

	acc_x=(__FP_ACC)(gvst_ThisBird.AccDatNow.pitch)/(__FP_ACC)MV_ACC_RP;//gvst_ThisBird.AccDatReZero.yaw;ÓÐ¿ÉÄÜËÀ»ú
	acc_y=(__FP_ACC)(gvst_ThisBird.AccDatNow.roll)/(__FP_ACC)MV_ACC_RP;//gvst_ThisBird.AccDatReZero.yaw;
	acc_z=(__FP_ACC)(gvst_ThisBird.AccDatNow.yaw)/(__FP_ACC)MV_ACC_RP;//gvst_ThisBird.AccDatReZero.yaw;

}

static float VariableParameterg(float error)
{
	float back_param=1.2f;
	back_param=0.8f;
	if(ak>0.15f)
		back_param=0.06f;
	else if(ak>0.1f)
		back_param=0.2f;
	else if(ak>0.05f)
		back_param=0.6f;
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
		back_param=0;
	return back_param;
}

static	void	IMUupdate(const __FP_ACC gxi,const __FP_ACC gyi,const __FP_ACC gzi,const __FP_ACC axi,const __FP_ACC ayi,const __FP_ACC azi,__Euler *euler)
{

	__FP_ACC recipNorm=0.0f;
	__FP_ACC halfvx=0.0f, halfvy=0.0f, halfvz=0.0f;
	__FP_ACC halfex=0.0f, halfey=0.0f, halfez=0.0f;
	__FP_ACC qa=0.0f, qb=0.0f, qc=0.0f;
//	float acc_val;
	static	int	last_time=0;int	now_time=0;
	
	if(euler==NULL)
		return;

	now_time=gvst_ThisHal.SystemGetMirco();
	if(last_time!=0)
		{//µÚÒ»´Î²»±Ø¸üÐÂsampleFreq
		sampleFreq=1000000.0f/(__FP_ACC)(now_time-last_time);
	}
//	SendDatShow((int16)(gxi*1000),(int16)(gvst_ThisBird.EulerAngle.pitch_x*10),(int16)(spin_rate_rad_s*100),(int16)(gvst_ThisBird.EulerAngle.roll_y*10));

	last_time=now_time;
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
	{
			twoKp=	0;//(2.0f * 0.1f);	// 2 * proportional gain
			twoKi=	0;//(2.0f * 0.000015f);	// 2 * integral gain
	}
	else
	{
			twoKp=	(2.0f * 0.1f);	// 2 * proportional gain
			twoKi=	0;//(2.0f * 0.000015f);	// 2 * integral gain
	}

	ax=axi;ay=ayi;az=azi;gx=gxi;gy=gyi;gz=gzi;
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		recipNorm = Fylib_Qrsqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   


		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = 2*(q1 * q3 - q0 * q2);
		halfvy = 2*(q0 * q1 + q2 * q3);
		halfvz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
		
		if(Fylib_Qrsqrt(gx*gx + gy*gy + gz*gz)!=0)
		  spin_rate_rad_s = 1/Fylib_Qrsqrt(gx*gx + gy*gy + gz*gz);
//		if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOT_RUN) && spin_rate_rad_s < 0.00988f)
//			twoKp=4;
//		else
		{
			twoKp=VariableParameterg(spin_rate_rad_s);
		}
		//test
		if(gvst_ThisBird.TelDat[Tel_Dat_RollMain]>0xa0 || gvst_ThisBird.TelDat[Tel_Dat_RollMain]<0x60 || gvst_ThisBird.TelDat[Tel_Dat_PitchMain]>0xa0 || gvst_ThisBird.TelDat[Tel_Dat_PitchMain]<0x60)
		{
			if(halfex>0.01f || halfex<-0.01f || halfey>0.01f || halfey<-0.01f)
				twoKp=twoKp*0.5f;
		}
		
		if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f)
		{
			if(spin_rate_rad_s < 0.0988f)
			{
				integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
				integralFBy += twoKi * halfey * (1.0f / sampleFreq);
				integralFBz += twoKi * halfez * (1.0f / sampleFreq);
				gx += integralFBx;	// apply integral feedback
				gy += integralFBy;
				gz += integralFBz;
			}
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 

	// Normalise quaternion
	recipNorm = Fylib_Qrsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	euler->pitch_x= atan2(2*q0*q1 + 2*q2*q3, 1 -2*q1*q1 - 2*q2*q2)* MV_RAD_DEG;//pitch
	euler->roll_y= asin(2*q0*q2 - 2*q1*q3)*MV_RAD_DEG;//roll
	euler->yaw_z= atan2(2*q0*q3+ 2*q1*q2 , 1 -2*q2*q2 - 2*q3*q3)*MV_RAD_DEG;//yaw
	if(euler->pitch_x>=90||(euler->pitch_x<=-90))
	{
		if(euler->roll_y>0)
			euler->roll_y=180-euler->roll_y;
		if(euler->roll_y<0)
			euler->roll_y=-(euler->roll_y+180);
	}
	
	abs_pitch1=FL_ABS(euler->pitch_x);
	abs_roll1=FL_ABS(euler->roll_y);
	
	halfvx = 2*(q1 * q3 - q0 * q2);
	halfvy = 2*(q0 * q1 + q2 * q3);
	halfvz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	ahrs_update_R_bf_to_ef(euler->pitch_x,euler->roll_y,euler->yaw_z);
}

#define PID_FILTER 0.2f
#define PID_FILTER1 0.9f
static	void	PowerValueUpdateForAttitude(const	__Euler *euler)
{
//	static	__SingleKlmPar pid_result_filter[3]={{0.1f,1.0f,0.88f,2.0f},{0.1f,1.0f,0.88f,2.0f},{0.1f,1.0f,0.88f,2.0f}};
	//if(euler==NULL )
	//	return;
	static	int	last_Dtime=0;int	now_Dtime=0;
	static  int pid_dtimer;
//	static  uint8 fblr_back_time=0;

	now_Dtime=gvst_ThisHal.SystemGetMirco();
	if(last_Dtime!=0)
	{//µÚÒ»´Î²»±Ø¸üÐÂ
		pid_dtimer=now_Dtime-last_Dtime;
	}
	last_Dtime=now_Dtime;
	
	
	//µÚÒ»¼¶PID£¬Íâ»·ÊäÈë½Ç¶È£¬Êä³ö½ÇËÙ¶È
	UpdatePidParLoop((__PidPar*)&birdpitch_1st_link,telpitch,Fylib_ConstrainF(euler->pitch_x+fall_pitch_off,-660,660),
			lvsfp_1st_pitch_kp,lvsfp_1st_pitch_ki,lvsfp_1st_pitch_kd,pid_dtimer,OUTER);
	UpdatePidParLoop((__PidPar*)&birdroll_1st_link,telroll,Fylib_ConstrainF(euler->roll_y+fall_roll_off,-660,660),
			lvsfp_1st_roll_kp,lvsfp_1st_roll_ki,lvsfp_1st_roll_kd,pid_dtimer,OUTER);
	UpdatePidParLoop((__PidPar*)&birdyaw_1st_link,telyaw,Fylib_ConstrainF(yaw_dvalue+fall_yaw_off,-85,85),
			lvsfp_1st_yaw_kp,lvsfp_1st_yaw_ki,lvsfp_1st_yaw_kd,pid_dtimer,OUTER);

	//µÚ¶þ¼¶PID£¬ÄÚ»·ÊäÈë½ÇËÙ¶È£¬Êä³öPWMÔöÒæ
	UpdatePidParLoop((__PidPar*)&birdpitch_2nd_link,birdpitch_1st_link.results,gyro_x_pid*MV_RAD_DEG,
			lvsfp_2nd_pitch_kp,lvsfp_2nd_pitch_ki,lvsfp_2nd_pitch_kd,pid_dtimer,INER);
	UpdatePidParLoop((__PidPar*)&birdroll_2nd_link,birdroll_1st_link.results,gyro_y_pid*MV_RAD_DEG,
			lvsfp_2nd_roll_kp,lvsfp_2nd_roll_ki,lvsfp_2nd_roll_kd,pid_dtimer,INER);
	UpdatePidParLoop((__PidPar*)&birdyaw_2nd_link,birdyaw_1st_link.results,gyro_z_pid*MV_RAD_DEG,
			lvsfp_2nd_yaw_kp,lvsfp_2nd_yaw_ki,lvsfp_2nd_yaw_kd,pid_dtimer,INER);
	
    if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FFLAG))//  chen-yunzhong teach ZFM  20180429   Õâ¸ö±êÖ¾ÓÃÓÚÏû³ýyawÔÚ·­¹öÊ±ºòµÄÐ§Ó¦
	{
        birdyaw_2nd_link.results=0;//0.5*Fylib_ConstrainF(birdyaw_2nd_link.results,-600,600);
    }
	else
	{
        birdyaw_2nd_link.results=Fylib_ConstrainF(birdyaw_2nd_link.results,-600,600);
    }


}


static	void	PowerValueUpdate(void)
{
//	static int32 realthrottlef=0,realthrottleaf=500;
	static uint16 thr_max=810;
//	static uint16 up_downto_keep_count=0;
//	static float accz_velfcom=0;
		static  uint8 _time=0;
	static uint8 flip_flag=0;
	PowerValueUpdateForFall((__Euler*)&gvst_ThisBird.EulerAngle);     //·­×ª
	
	pid_timer++;
	if(pid_timer>=2)
	{
		pid_timer=0;
		PowerValueUpdateForAttitude((__Euler*)&gvst_ThisBird.EulerAngle);  //³£¹æ×ËÌ¬¼ÆËã
	}
//	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
//	{
//		flip_flag=1;
//	}
//	else
//	{
//		flip_flag=0;
//	}
//			if(_time++>15)
//	{
//		_time=0;
// DT_Send_Senser((int32_t) (100*telpitch),(int32_t) (Fylib_ConstrainF(gvst_ThisBird.EulerAngle.pitch_x+fall_pitch_off,-660,660)),(int32_t) (birdpitch_1st_link.results/20),
//										(int32_t) gyro_x_pid*MV_RAD_DEG,(int32_t) birdpitch_2nd_link.results,(int32_t) birdyaw_2nd_link.d_err,
//										(int16_t) 0,(int16_t) 0,(int16_t) 0,
//										(int32_t) 0);
//		 DT_Send_Senser((int32_t) (Fylib_ConstrainF(gvst_ThisBird.EulerAngle.pitch_x+fall_pitch_off,-660,660)	),(int32_t) (gvst_ThisBird.EulerAngle.pitch_x	),(int32_t) (gyro_x_pid*MV_RAD_DEG),
//										 (int32_t) (birdpitch_1st_link.results/100)	,(int32_t) (0),(int32_t)1000*flip_flag,
//										(int32_t) realthrottle,(int32_t)  (0),(int32_t)  (0),
//										(int32_t)  0,0);
//				 DT_Send_Senser((int32_t) (Fylib_ConstrainF(gvst_ThisBird.EulerAngle.roll_y+fall_roll_off,-660,660)	),(int32_t) (gvst_ThisBird.EulerAngle.roll_y	),(int32_t) (gyro_y_pid*MV_RAD_DEG),
//										 (int32_t) (birdroll_1st_link.results/100)	,(int32_t) (0),(int32_t)1000*flip_flag,
//										(int32_t) realthrottle,(int32_t)  (0),(int32_t)  (0),
//										(int32_t)  0,0);
//	}

	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOTOR_BK))
	{
		realthrottle=0;
		hold_high_statue=HOLD_HIGH_STOP;
	}
	else	realthrottle=telthrottle;
	
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_6881OK) && (START_MODE==1))    //²âÊÔÄ£Ê½
	{
		START_TEST_TIMER++;
		if(START_TEST_TIMER<=400)
		{
			realthrottle=100;
			if(START_TEST_TIMER<100)
			{
				gvst_ThisBird.MotorValue[0]=800;
				gvst_ThisBird.MotorValue[1]=0;
				gvst_ThisBird.MotorValue[2]=0;
				gvst_ThisBird.MotorValue[3]=0;
			}
			else if(START_TEST_TIMER<200)
			{
				gvst_ThisBird.MotorValue[0]=0;
				gvst_ThisBird.MotorValue[1]=800;
				gvst_ThisBird.MotorValue[2]=0;
				gvst_ThisBird.MotorValue[3]=0;
			}
			else if(START_TEST_TIMER<300)
			{
				gvst_ThisBird.MotorValue[0]=0;
				gvst_ThisBird.MotorValue[1]=0;
				gvst_ThisBird.MotorValue[2]=800;
				gvst_ThisBird.MotorValue[3]=0;
			}
			else if(START_TEST_TIMER<400)
			{
				gvst_ThisBird.MotorValue[0]=0;
				gvst_ThisBird.MotorValue[1]=0;
				gvst_ThisBird.MotorValue[2]=0;
				gvst_ThisBird.MotorValue[3]=800;
			}
			return;
		}
		else
		{
			START_TEST_TIMER=3001;
			realthrottle=0;
		}
	}
	if(hold_high_statue!=HOLD_HIGH_STOP)
		sleep_timer=0;
	
	if(cal_over_timer<300)
	{
		cal_over_timer++;
	}
	if(realthrottle<100.0f)// || cal_over_timer<300)
	{
		gvst_ThisBird.MotorValue[0]=0;
		gvst_ThisBird.MotorValue[1]=0;
		gvst_ThisBird.MotorValue[2]=0;
		gvst_ThisBird.MotorValue[3]=0;
	}
	else
	{
//xÄ£Ê½
//		accel_outhand=accel_out;
		if(accel_outhand>50) accel_outhand=50;
		if(accel_outhand<-50) accel_outhand=-50;
		if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT))
		  realthrottle=realthrottle+accel_out;
		else
			realthrottle=realthrottle-accel_outhand;
		if(realthrottle>=thr_max)
			  realthrottle=thr_max;
		else if(realthrottle<=0)
			realthrottle=0;
		
		if(realthrottle>=thr_max)
			  realthrottle=thr_max;
		
		if(realthrottle>=900 || MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))//ZFM 
			  realthrottle=900;
		if(realthrottle<=100)
		{
            realthrottle=100;
        }
        if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))//ÓÃÓÚ·­¹öµÄÉèÖÃ
		{
            realthrottle=realthrottleFALL;
        }
		
		//test
		if(MF_IF_FLAG(FLAGA,FLAGA_FLIP_FINISH))
		{

			flip_finish_timer--;
			if(flip_finish_timer!=0)
			{
				if(realthrottle<=540)
				{
					realthrottle=540;//realthrottlesave;	
//					MF_SET_FLAG(FLAGA,FLAGA_FLIP_END_UP);
				}
				if(flip_finish_timer==50)
				{
					MF_SET_FLAG(FLAGA,FLAGA_FLIP_END_UP);
				}
			}
			else
			{
				flip_finish_timer=0;
				MF_CLR_FLAG(FLAGA,FLAGA_FLIP_FINISH);
			}
			
		}
        gvst_ThisBird.MotorValue[0]=Fylib_Constrain(realthrottle+birdroll_2nd_link.results-birdpitch_2nd_link.results+birdyaw_2nd_link.results,10,960);
		gvst_ThisBird.MotorValue[1]=Fylib_Constrain(realthrottle-birdroll_2nd_link.results-birdpitch_2nd_link.results-birdyaw_2nd_link.results,10,960);
		gvst_ThisBird.MotorValue[2]=Fylib_Constrain(realthrottle-birdroll_2nd_link.results+birdpitch_2nd_link.results+birdyaw_2nd_link.results,10,960);
		gvst_ThisBird.MotorValue[3]=Fylib_Constrain(realthrottle+birdroll_2nd_link.results+birdpitch_2nd_link.results-birdyaw_2nd_link.results,10,960);
		

	if(hold_high_statue==HOLD_HIGH_STOP)
		{
				gvst_ThisBird.MotorValue[0]=0;
				gvst_ThisBird.MotorValue[1]=0;
				gvst_ThisBird.MotorValue[2]=0;
				gvst_ThisBird.MotorValue[3]=0;
		}
//+Ä£Ê½
	}
}

static	void	SystemUpdateMotorPower(void)
{
	int	i;
  if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_6881OK))
	{
		if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))
		  {MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOT_RUN);close_pwm();return;}

	  if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP))
		  {MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOT_RUN);close_pwm();return;}

	  if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_ZERO))
		  {MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOT_RUN);close_pwm();return;}
	}
	
	if(realthrottle<100.0f)
		  MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOT_RUN);
	  else
		  MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOT_RUN);
	
	for(i=0;i<MV_HOWMANY_MOTOR;i++)
	{
		if(gvst_ThisBird.MotorValue[i]<MV_MIN_MOTOR_VALUE)	gvst_ThisBird.MotorValue[i]=MV_MIN_MOTOR_VALUE;
		if(gvst_ThisBird.MotorValue[i]>MV_MAX_MOTOR_VALUE)	gvst_ThisBird.MotorValue[i]=MV_MAX_MOTOR_VALUE;
	}
	for(i=0;i<=3;i++)
	{
		gvst_ThisBird.MotorValue_pwm[i]=gvst_ThisBird.MotorValue[i];
	}
	//if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_PWM_OC))
		open_pwm();
}

static	void	LedSetState(void)
{
  if(gven_LedFlashKind==EV_LED_4)
		return;

	if((MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DROP)) && (START_MODE==0))
		{gven_LedFlashKind=EV_LED_2;}//Î´¶ÔÂë

	else if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_ZERO))
		{gven_LedFlashKind=EV_LED_1;}//Î´Ð£×¼IMU

	else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOTOR_BK))
		{gven_LedFlashKind=EV_LED_6;}//µç»ú¿¨ËÀ

	else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2))//
		{gven_LedFlashKind=EV_LED_5;}//ÏµÍ³µÍÑ¹
	else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))
	{gven_LedFlashKind=EV_LED_2;}
			
//	else	if(MF_IF_FLAG(FLAGA,FLAGA_FLIP_FINISH))
//		{
//			gven_LedFlashKind=EV_LED_1;
//		}
   
//    else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S4))//·­¹ö²¹³¥ 2
//    {
//        gven_LedFlashKind=EV_LED_7;
//    }
////	else	if(MF_IF_FLAG(FLAGA,FLAGA_FLIP_FINISH))
//    else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S3))//·­¹ö²¹³¥ 1
//    {
//        gven_LedFlashKind=EV_LED_1;
//    }
//		else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S6))  
//    {
//        gven_LedFlashKind=EV_LED_7;
//    }
//			else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S5))  
//    {
//        gven_LedFlashKind=EV_LED_6;
//    } 
//    else if(MF_IF_FLAG(FLAGA,FLAGA_POWER_LOW_TOP))
//		{
//			gven_LedFlashKind=EV_LED_5;
//		}
	
	else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD))
	  {gven_LedFlashKind=EV_LED_7;}
	else
		gven_LedFlashKind=EV_LED_3;//Õý³£´ý»ú
}
/*
void	AskSystemState(char *dat)
{
	if(strncmp(dat,"ASK",strlen("ASK"))==0)
	{
		printf("height,tar= %f,cur= %f\r\n",gvfpacc_ZeroHeight,gvfpacc_CurHeight);
		printf("1st,rel= %f,err= %f,i_err= %f,drr= %f\r\n",birdheight_1st_link.results,birdheight_1st_link.p_err,birdheight_1st_link.i_err,birdheight_1st_link.d_err);
		printf("acc,cur= %f\r\n",real_acc_z);
		printf("2nd,rel= %f,err= %f,i_err= %f,drr= %f\r\n",birdheight_2nd_link.results,birdheight_2nd_link.p_err,birdheight_2nd_link.i_err,birdheight_2nd_link.d_err);	
	}
}*/
#define	MV_LEVEL	400//500  //ZFM   ÓÉÓÚ¿¨ËÀÈÝÒ×ÔÚ»§Íâ±»Òì³£¹ÒÆð£¬ËùÒÔÐèÒª½«¿¨ËÀµ÷ÕûµÄÐ¡Ò»µã

// #define Reference_Voltage     2.8
// #define Divider_Resistance_1  100
// #define Divider_Resistance_2  47
// #define Resistance            (Divider_Resistance_1+Divider_Resistance_2)
// #define Sampling_precision    4096
// #define Collect_Number        10  
// #define Coefficient           (Reference_Voltage*Resistance/Sampling_precision/Divider_Resistance_2/Collect_Number)


extern u16 Power_value;

void	WatchVol(void)
{
	static  int16  MV_BKVOL_OFF1,MV_BKVOL_OFF2,MV_BKVOL_OFF3,MV_BKVOL_OFF4;
	static	uint32  lu32_mbk1cnt=0,lu32_mbk2cnt=0,lu32_mbk3cnt=0,lu32_mbk4cnt=0;
	static  uint16_t Power_average_count=0;
	static  uint32_t Power_sum=0;
	static  uint8  power_low1=0,power_low2=0,power_low3=0,power_low4=0,power_low5=0,power_low6=0,power_low_top=0;//,fall_power_count1=0,fall_power_count2=0;
	
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_50MSPWM))
	{
		MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_50MSPWM);
		  close_pwm();
//		  GPIO->PA_DIRBCR = 0x000000C0;
//		  GPIO->PB_DIRBCR = 0x000020A0;
		  delayus(10);
		  gvst_ThisBird.system_vol[System_Vol_PowerLow]	=hr8p506_adc_sample(0);   //ADC¿Ú¶¨Òå
        		//¸ü¸ÄË³Ðò  ZFM 20180418
		  gvst_ThisBird.system_vol[System_Vol_T1]			=hr8p506_adc_sample(2);    //T1¶ÔÓ¦MV_BKVOL_OFF1
		  gvst_ThisBird.system_vol[System_Vol_T2]			=hr8p506_adc_sample(5);     //T2¶ÔÓ¦MV_BKVOL_OFF2
		  gvst_ThisBird.system_vol[System_Vol_T3]			=hr8p506_adc_sample(8);    //T3¶ÔÓ¦MV_BKVOL_OFF3
	    gvst_ThisBird.system_vol[System_Vol_T4]			=hr8p506_adc_sample(13);      //T4¶ÔÓ¦MV_BKVOL_OFF4
			open_pwm();
//		  GPIO->PA_INEB = 0x000000C0;    //PA7  PA6¹Ø±ÕÊý×ÖÊäÈë¿Ú  Ê¹ÄÜÄ£ÄâÍ¨µÀ¶Ë¿Ú
//	    GPIO->PA_DIRBSR = 0x000000C0;
//	    GPIO->PB_INEB = 0x000020A0;   //PB5,6,7,8¹Ø±ÕÊý×ÖÊäÈë¿Ú  Ê¹ÄÜÄ£ÄâÍ¨µÀ¶Ë¿Ú
//	    GPIO->PB_DIRBSR = 0x000020A0;
		
		
		if(gvst_ThisBird.system_vol[System_Vol_PowerLow]<0xd00 && (gvst_ThisBird.system_vol[System_Vol_PowerLow]>0x500))
		{
			Power_sum+=gvst_ThisBird.system_vol[System_Vol_PowerLow];
	    Power_average_count++;
		}
	if(Power_average_count>=10)
	{
		Power_average=Power_sum/10;
//      if(Power_average>=2650)
//			{
//				fall_com_timer=0;
//			}
//			else if(Power_average>=2600)
//			{
//				fall_com_timer=50;
//			}
//			else if(Power_average>=2550)
//			{
//				fall_com_timer=100;
//			}
//			else if(Power_average>=2500)
//			{
//				fall_com_timer=150;
//			}
//			else
//				fall_com_timer=200;
		
		Power_average_count=0;
		Power_sum=0;
		if(Power_average<MV_VOL_POWER_LOW_STEP1)
	  {
//			MV_BKVOL_OFF1=MV_BKVOL_OFF1-80;
//			MV_BKVOL_OFF2=MV_BKVOL_OFF2-80;
		  power_low1++;
		  if(power_low1>=10)
			{
				MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S1);
			}
	  }
	  else
      {
          power_low1=0;
      }
			
//	  if(Power_average<MV_VOL_POWER_LOW_STEP2)//¸ÄÎªÔÚµÍÑ¹S1ÖÃÆðºó£¬ÔÚÖÐ¶Ïº¯ÊýÖÐ¼ÆÊ±50sÒÔºóÖÃÆðS2  20180920
//	  {
//		  power_low2++;
//		  if(power_low2>=5)
//			  MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2);
//	  }
//	  else
//      {
//          power_low2=0;
//      }

	  if(Power_average<MV_VOL_POWER_LOW_TOP)
		{
			 power_low_top++;
		  if(power_low_top>=5)
			{
				MF_SET_FLAG(FLAGA,FLAGA_POWER_LOW_TOP);
			}	
		} 
		else
		{
			power_low_top=0;
		}
		
		if(Power_average<MV_VOL_POWER_LOW_STEP5)
	  {
		  power_low5++;
		  if(power_low5>=5)
			{
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S5);
			}
	  }
	  else
    {
          power_low5=0;
    }
		
		if(Power_average<MV_VOL_POWER_LOW_STEP6)
	  {
		  power_low6++;
		  if(power_low6>=5)
			{
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S6);
			}
	  }
	  else
    {
          power_low6=0;
    }	
		
		if(Power_average<MV_VOL_POWER_LOW_STEP3)
	  {
		  power_low3++;
		  if(power_low3>=5)
			{
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S3);
			}
	  }
	  else
    {
          power_low3=0;
    }
		
		if(Power_average<MV_VOL_POWER_LOW_STEP4)
	  {
		  power_low4++;
		  if(power_low4>=5)
			{
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S4);
			}
	  }
	  else
    {
          power_low4=0;
    }


//		if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S4))
//        {
//           FALL_com_up=50;//65;
//           FALL_com_down=90; 
//					if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//¸ÄÎªZFM  Ç°ºó roll_y
//          { angle_up=30; }
//          else
//					{
//						angle_up=15;
//					}						
//        }
//		else
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S3))      
       {
          	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//¸ÄÎªZFM  Ç°ºó roll_y
						{
							FALL_com_down=50;
						}
					else
					{
						 FALL_com_down=40;
					}						
				 FALL_com_up=45;//30; 
         angle_up=15;   
        }
			 
			else if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S6))      
       {
          	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//¸ÄÎªZFM  Ç°ºó roll_y
						{
							FALL_com_down=20;
						}
					else
					{
						 FALL_com_down=15;
					}						
				 FALL_com_up=25;//30;       
          angle_up=15;   
      }
	  
			else  if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_POWER_LOW_S5))//Ôö¼Ó²¹³¥   ZFM 20181007
        {      
					if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//¸ÄÎªZFM  Ç°ºó roll_y
					{
						FALL_com_down=20;
					}
					else
					{
						FALL_com_down=10;
					}
            FALL_com_up=20;  
            angle_up=0;
        }
				
		else if(MF_IF_FLAG(FLAGA,FLAGA_POWER_LOW_TOP))
		{
					if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//¸ÄÎªZFM  Ç°ºó roll_y
					{
						FALL_com_down=5;//20;
					}
					else
					{
						FALL_com_down=-10;//0;
					}
            FALL_com_up=10;//20;   
            angle_up=0;
    }
		
		else //Âúµç
		{
        if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_L) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_FALL_R))//¸ÄÎªZFM  Ç°ºó roll_y
				{
					FALL_com_down=-10;
				}
				else
				{	
					FALL_com_down=-20;
				}
            FALL_com_up=-20;  
            angle_up=0;	
		}

            
	
 	}
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOT_RUN))//¿¨ËÀ±£»¤ÔÚµç»úÔËÐÐÊ±²ÅÆô¶¯
	{
		motor_start_timer++;
		if(motor_start_timer>=4)
			motor_start_timer=5;
		else
		{
			lu32_mbk1cnt=0;
		  lu32_mbk2cnt=0;
		  lu32_mbk3cnt=0;
		  lu32_mbk4cnt=0;
		}
		
		if(gvst_ThisBird.MotorValue[0]>600)
		  MV_BKVOL_OFF1=-MV_LEVEL;   //-500     ÊýÖµÔ½´ó¿¨ËÀÔ½ÁéÃô
		else if(gvst_ThisBird.MotorValue[0]>350)
			MV_BKVOL_OFF1=-MV_LEVEL+100;    //-600
		else
			MV_BKVOL_OFF1=-MV_LEVEL+200;    //-700
    if(gvst_ThisBird.MotorValue[1]>600)
		  MV_BKVOL_OFF2=-MV_LEVEL;
		else if(gvst_ThisBird.MotorValue[1]>350)
			MV_BKVOL_OFF2=-MV_LEVEL+100;
		else
			MV_BKVOL_OFF2=-MV_LEVEL+200;
		if(gvst_ThisBird.MotorValue[2]>600)
		  MV_BKVOL_OFF3=-MV_LEVEL;
		else if(gvst_ThisBird.MotorValue[2]>350)
			MV_BKVOL_OFF3=-MV_LEVEL+100;
		else
			MV_BKVOL_OFF3=-MV_LEVEL+200;
		if(gvst_ThisBird.MotorValue[3]>600)
		  MV_BKVOL_OFF4=-MV_LEVEL;
		else if(gvst_ThisBird.MotorValue[3]>350)
			MV_BKVOL_OFF4=-MV_LEVEL+100;
		else
			MV_BKVOL_OFF4=-MV_LEVEL+200;
		
		if(Power_average+MV_BKVOL_OFF1<(gvst_ThisBird.system_vol[System_Vol_T1]) && (3600>(gvst_ThisBird.system_vol[System_Vol_T1])) && gvst_ThisBird.MotorValue[0]>235)
			lu32_mbk1cnt+=1;
		else
		{
			if(gvst_ThisBird.system_vol[System_Vol_T1]>400)
			{
				if(lu32_mbk1cnt>2)
				  lu32_mbk1cnt-=2;
			}
		}
		if(Power_average+MV_BKVOL_OFF2<(gvst_ThisBird.system_vol[System_Vol_T2]) && (3600>(gvst_ThisBird.system_vol[System_Vol_T2])) && gvst_ThisBird.MotorValue[1]>235)
			lu32_mbk2cnt+=1;
		else
		{
			if(gvst_ThisBird.system_vol[System_Vol_T2]>400)
			{
				if(lu32_mbk2cnt>2)
				  lu32_mbk2cnt-=2;
			}
		}
		if(Power_average+MV_BKVOL_OFF3<(gvst_ThisBird.system_vol[System_Vol_T3]) && (3600>(gvst_ThisBird.system_vol[System_Vol_T3])) && gvst_ThisBird.MotorValue[2]>235)
			lu32_mbk3cnt+=1;
		else
		{
			if(gvst_ThisBird.system_vol[System_Vol_T3]>400)
			{
				if(lu32_mbk3cnt>2)
				  lu32_mbk3cnt-=2;
			}
		}
		if(Power_average+MV_BKVOL_OFF4<(gvst_ThisBird.system_vol[System_Vol_T4]) && (3600>(gvst_ThisBird.system_vol[System_Vol_T4])) && gvst_ThisBird.MotorValue[3]>235)
			lu32_mbk4cnt+=1;
		else
		{
			if(gvst_ThisBird.system_vol[System_Vol_T4]>400)
			{
				if(lu32_mbk4cnt>2)
				  lu32_mbk4cnt-=2;
			}
		}
		if(lu32_mbk1cnt>=10 || (lu32_mbk2cnt>10) || (lu32_mbk3cnt>10) || (lu32_mbk4cnt>10))
		{
			if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2))
				MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOTOR_BK);
		}
	}
	else
	{
		lu32_mbk1cnt=0;
		lu32_mbk2cnt=0;
		lu32_mbk3cnt=0;
		lu32_mbk4cnt=0;
	}
	}
}    

//u16 Difference_value(void)
//{
// static u16 Last_Vol=0;
// static u8 times,number;

//	
// if(hold_high_statue!=HOLD_HIGH_STOP)
// {
//	 times++; //1sÀÛ¼ÓÒ»´Î
//	 
//	 if(Last_Vol==0&&Power_average>MV_VOL_POWER_LOW_STEP2) //¸üÐÂÉÏ´ÎµçÑ¹(Ö»¸üÐÂÒ»´Î)
//		 Last_Vol=Power_average;
//	 
//	 if(times>=30)//30s¸üÐÂÒ»´Î
//	 {
//		 times=0;
//		 number++;
//		 if(number<10)
//			Power_Difference_value[number]=Last_Vol-Power_average; //¼ÆËãÁ½´ÎµçÑ¹²îÖµ
//		 else
//			number=0;
//		 Last_Vol=Power_average;
//	 }
//	 return Power_Difference_value[number];
// }
// else
//	  return 0;

//}



void	MissionInit(void)
{
	EulerSetZero();
}

void	MissionLoop(void)
{
	static uint8 usart_send_timer=0;
	// read tel
	
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_READ_TEL))   //20ms
	//if(0)
	{// 20ms
		static uint8 timer=0;
		//WatchVol();
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_READ_TEL);
		
		//if(gvst_ThisHal.ReadTel()==TRUE)//Õý³£
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK) || (START_MODE==1 && MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_TESTF)))
		{
			if(hold_high_statue==HOLD_HIGH_STOP && MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK))  //½âËø²»ÄÜÐ£Ë®Æ½
			{
				if(	(gvst_ThisBird.TelDat[0]<=60 && gvst_ThisBird.TelDat[3]<0x70 && gvst_ThisBird.TelDat[4]>0x90 && gvst_ThisBird.TelDat[1]>0x90) || MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_TESTF) || gyro_acc_data[6]!=11 || MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_START_UP))
				{//ÍÓÂÝÒÇÐ£×¼
					if(lsv32_GetZeroTimeCnt++>35 || MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_TESTF) || gyro_acc_data[6]!=11  || MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_START_UP))
					{
//						MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_NOHEAD);
						MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_TESTF);
						MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_START_UP);
						lsv32_GetZeroTimeCnt=0;
						BridClear();
					}
				}
				else
				{
					if(lsv32_GetZeroTimeCnt>=2)
						lsv32_GetZeroTimeCnt-=2;
				}
			}
			//tel¶ÁÈ¡´íÎó¼ÆÊý
			UpdateTelDat((uint8*)gvst_ThisBird.TelDat);
		}
		else if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_TEL_DOCK))//Ò£¿ØÆ÷Î´Á¬½Ó
		{UpdateTelDat(NULL);}
//		else if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK))//Ò£¿ØÆ÷È·ÈÏµôÏß
//		{UpdateTelDat(NULL);}
		if(++timer>=2) {
				Data_To_APP();
        timer=0;		
			}	
		else
		{
			lsv32_GetErrTimeCnt++;
		}
		
	}

	// read imu
	//if(0)
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_READ_IMU))    //2ms½øÈë  imu½âËã
	{
//		usart_send_timer++;
//		if(usart_send_timer>=2)   //3   100ms
//		{
//			usart_send_timer=0;
//			usart_data_send();
//		}
		//DT_Data_Exchange();
		gvst_ThisHal.ReadTel();
//		lsv32_ImuCalcTimeCnt++;
		MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_READ_IMU);
		
		UpdateAhrsData();    //¶ÁÈ¡ÍÓÂÝÒÇ ¼ÓËÙ¶È¼Æ²¢ÂË²¨
		PowerValueUpdate();
		IMUupdate(gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,(__Euler*)&gvst_ThisBird.EulerAngle);   //¸üÐÂ×ËÌ¬½Ç
		UpdateZ_axis(gvst_ThisBird.EulerAngle.yaw_z);
//		if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_ZERO))
//		{
//			gvst_ThisBird.AccDatReZero.pitch=0;
//			gvst_ThisBird.AccDatReZero.roll=0;
//			gvst_ThisBird.AccDatReZero.yaw=4000;
//			gvst_ThisBird.GyroDatReZero.pitch=0;
//			gvst_ThisBird.GyroDatReZero.roll=0;
//			gvst_ThisBird.GyroDatReZero.yaw=0;
//			lsvfp_RealYawRate=gvst_ThisBird.EulerAngle.yaw_z;
//			EulerSetZero();
//			ClearPidLink();
//		}
		if(lsv32_ClearPidLinkTimeCnt>20 )
		{
//			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_FIRST_H);
			if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOT_RUN))
			{
//				if(gvst_ThisBird.AccDatNow.roll>-100 && gvst_ThisBird.AccDatNow.roll<100 && gvst_ThisBird.AccDatNow.pitch>-100 && gvst_ThisBird.AccDatNow.pitch<100 && gvst_ThisBird.AccDatNow.yaw>3000)
//					EulerSetZero();
				ClearPidLink();//ÓÍÃÅÎª0Ê±Çåpid
			}
			//Æð·ÉÇå³ýÎ¢µ÷
//			pitch_trim=gvst_ThisBird.TelDat[5]-0x40;
//			roll_trim=gvst_ThisBird.TelDat[6]-0x40;
			
			lsvfp_RealYawRate=gvst_ThisBird.EulerAngle.yaw_z;
			lsv32_ClearPidLinkTimeCnt=21;
		}
//		PowerValueUpdate();
		SystemUpdateMotorPower();
		if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
		  ReadBaro();
	}
	WatchVol();
	LedSetState();
}

struct ACC
{
  uint8 x;
	uint8 y;
	uint8  z;

};

typedef struct ACC acc_w;



