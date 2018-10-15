#ifndef	__CONFIG_H_
#define	__CONFIG_H_

#include <stdint.h>

typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable	*/
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable*/
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 	*/
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable*/
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 	*/
typedef float          fp32;                    /* single precision floating point variable (32bits) */
typedef double         fp64;                    /* double precision floating point variable (64bits) */

/*变量类型重新命名*/
typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
/*
**************************************************************************
extern macro<VALUE>
**************************************************************************
*/
#ifndef	FALSE
#define	FALSE	0
#endif
#ifndef	TRUE
#define	TRUE	1
#endif
#ifndef	false
#define	false	0
#endif
#ifndef	true
#define	true	0
#endif				
/***************************************/
//2.4通信协议 APEX 格瑞达 Frank 18.7.4 AM
#define APEX 1

#if APEX
 #define NO_Data_TIME 4
#else
 #define NO_Data_TIME 16
#endif
/***************************************/
/*
/////////////////////////////////////
滤波设置区域
/////////////////////////////////////
*/
#define	__FP_ACC	fp32//滤波计算精度
#define	MV_ACC_RP	4096//16bit for +-8g
#define	MV_GYRO_ANG_RP	934.90f//16bit for +-34.9angle/s   34.9弧度/S  32768/938.9=34.9
#define	MV_GYRO_RP	16.38f//16bit for +-2000der/s
#define	MV_RAD_DEG	57.30f//1rad for deg
#define	MV_STAND_PRESS	101325.0f//标准大气压

#define	MV_GYRO_OLDDAT_WEIGHT	0.95f//陀螺仪数据老数据权重
#define	MV_GYRO_NEWDAT_WEIGHT	0.05f//陀螺仪数据新数据权重
#define	MV_ACC_OLDDAT_WEIGHT	0.95f//加速度计老数据权重
#define	MV_ACC_NEWDAT_WEIGHT	0.05f//加速度计新数据权重


#define	CARRIER_TIME	16
#define	CHANNEL_MAX1		5

#define HOLD_HIGH_UP    0 //定高上升阶段
#define HOLD_HIGH_KEEP  1 //定高稳定
#define HOLD_HIGH_DOWN  2 //定高下降阶段
#define HOLD_HIGH_STOP  3 //定高停止阶段

#if APEX
	#define MATCH_FRE 79
#else
	#define MATCH_FRE 60
#endif

#define LINK_NOLINK 0
#define LINK_USART 1
#define LINK_2P4G 2

//前后左右打舵
#define STATE_F 1

#define fblr_nomode       0
#define fblr_yesmode       1


/*
/////////////////////////////////////
滤波设置区域
/////////////////////////////////////
*/

#define	MV_USE_HAL_KIND		1//适用电路板，0小板1中板2大板
#define	MV_SYSTEM_CLOCK		48000000	//系统时钟36000000 & 48000000时
#define	MV_CMD_PREFIX		""		//命令行前缀，用以标示
#define	MV_U1_BUF			11		//串口1缓存
#define	MV_U1_TIMEOUT		20		//串口1接收超时计时器，单位毫秒
#define	MV_GET_SENSOR_ZERO_TIM		1000	//获取传感器零点平均次数
#define	MV_FREQ_SYSTEM_TICK	1000	//系统中断时钟频率
#define	MV_FREQ_READ_TEL	20	//读取遥控器频率单位ms
#define	MV_FREQ_READ_IMU	3	//读取并解算IMU单元数据频率单位ms
#define	MV_FREQ_READ_GPS	4	//读取GPS频率单位ms
#define	MV_FREQ_READ_BARO	1	//读取气压计频率单位ms
#define	MV_TEL_DROP_DELAY	100//(5*MV_FREQ_SYSTEM_TICK/MV_FREQ_READ_TEL)//遥控器掉线延时时间
#define	MV_PHOTO_TIMEOUT	200
#define	MV_DV_TIMEOUT		1000

#define NVDS_PAGE_START_ADDR	0x00008c00

/*
四组电机
序号一为左上角PIN32/PB8，顺时针旋转
序号二为左下角PIN9/PA3，逆时针旋转
序号三为右下角PIN18/PA8，顺时针旋转
序号四为右上角PIN21/PA11，逆时针旋转
*/
#define	MV_HOWMANY_MOTOR	4	//记录本机有多少个电机
#define	MV_MAX_MOTOR_VALUE	960	//最高动力值
#define	MV_MIN_MOTOR_VALUE	0	//最低动力值
#define	MV_MOTOR_NO1	0	//电机编号一
#define	MV_MOTOR_NO2	1	//电机编号二
#define	MV_MOTOR_NO3	2	//电机编号三
#define	MV_MOTOR_NO4	3	//电机编号四

#define	MV_ADC_SYSTEM_VCC	ADC_Channel_0//系统电压检测通道
#define	MV_ADC_M3M4_VCC		ADC_Channel_1//电机3、4检测通道
#define	MV_ADC_M1M2_VCC		ADC_Channel_2//电机1、2检测通道

#define	MV_SYSTEM_FLG_U1BUSY	0x00000001//串口1忙
#define	MV_SYSTEM_FLG_U1DONE	0x00000002//串口1完成
#define	MV_SYSTEM_FLG_GET_PHOTO	0x00000004//拍照
#define	MV_SYSTEM_FLG_GET_DV	0x00000008//录像

#define	MV_SYSTEM_FLG_TEL_DOCK	0x00000010//遥控对码成功，一次通电仅对码一次
#define	MV_SYSTEM_FLG_TEL_DROP	0x00000020//遥控掉线或跳频中
#define	MV_SYSTEM_FLG_GET_ZERO	0x00000040//陀螺仪已经校准
#define	MV_SYSTEM_FLG_MOTOR_BK	0x00000080//卡死保护启动

#define	MV_SYSTEM_FLG_READ_TEL	0x00000100//读取遥控数据
#define	MV_SYSTEM_FLG_READ_IMU	0x00000200//读取位置传感单元数据
#define	MV_SYSTEM_FLG_MOT_RUN	0x00000400//电机运行
#define	MV_SYSTEM_FLG_READ_BARO	0x00000800//读取气压计信息

#define	MV_SYSTEM_FLG_FALL_L	0x00001000//左翻跟头
#define	MV_SYSTEM_FLG_FALL_R	0x00002000//右翻跟头
#define	MV_SYSTEM_FLG_FALL_F	0x00004000//前翻跟头
#define	MV_SYSTEM_FLG_FALL_D	0x00008000//后翻跟头

#define	MV_SYSTEM_FLG_FALL_ST1		0x00010000//第一阶段
#define	MV_SYSTEM_FLG_FALL_ST2		0x00020000//第一阶段
#define	MV_SYSTEM_FLG_FALL_ST3		0x00040000//第一阶段
#define	MV_SYSTEM_FLG_FALL_ST4		0x00080000//第一阶段

#define	MV_SYSTEM_FLG_ALT_CHANGE	0x00100000//定高模式下飞机更新高度
#define	MV_SYSTEM_FLG_ALT_RUN		0x00200000//定高模式下飞机启动
#define	MV_SYSTEM_FLG_POWER_LOW_S1	0x00400000
#define	MV_SYSTEM_FLG_POWER_LOW_S2	0x00800000

#define MV_SYSTEM_FLG_PWM_OC        0x01000000 //开关pwm标志位 1为开 0为关
#define MV_SYSTEM_FLG_START_UP        0x02000000
#define MV_SYSTEM_FLG_LOCK_LINK        0x04000000  //遥控器掉线
#define MV_SYSTEM_FLG_6881OK        0x08000000  //test 
#define	MV_SYSTEM_FLG_GET_PHOTO1	    0x10000000//拍照
#define	MV_SYSTEM_FLG_FALL_BEHIDE	    0x20000000//
#define	MV_SYSTEM_FLG_PWM_ZERO      	0x40000000//


//#define	MV_SYSTEM_FLG_
//#define	MV_SYSTEM_FLG_OUTCTRL	0x80000000//测试状态，使飞机不受IMU控制

//FLAGA_FLAG
#define	MV_SYSTEM_FLG_50MSPWM	0x00000001  //PWM 50MS OPEN AND THEN CLOSE DETECT ADC
#define	MV_SYSTEM_FLG_FFLAG	0x00000002    //FALL FLAG
#define	MV_SYSTEM_FLG_FOF	0x00000004   //翻滚0f标志位
#define	MV_SYSTEM_FLG_FSTAGE	0x00000008
#define	MV_SYSTEM_FLG_TESTF	0x00000010
#define MV_SYSTEM_FLG_AUTO_ALT1 0x00000020
#define MV_SYSTEM_FLG_AUTO_ALT 0x00000040
#define MV_SYSTEM_FLG_POWER_LOW_S6 0x00000080  //增加补偿 ZFM 20181009
#define MV_SYSTEM_FLG_ONEKEYUP 0x00000100   //一键起飞缓降
#define MV_SYSTEM_FLG_ONEKEYDOWN 0x00000200   //一键起飞缓降
#define MV_SYSTEM_FLG_START_FLAG 0x00000400   //开机标志
#define MV_SYSTEM_FLG_POWER_LOW_S4  0x00000800 //定高检测停止
#define MV_SYSTEM_FLG_STOPF  0x00001000 //定高停止前
#define MV_SYSTEM_FLG_FALLSTAGE  0x00002000  //开机第一次定高
#define FLAGA_FLIP_END_UP  0x00004000  //翻滚完成以后增加相关处理，避免在从翻滚完成以后直接切定高时油门出现问题  ZFM 20180922
#define FLAGA_FLIP_FINISH  0x00008000  //翻滚完成后的标志  ZFM 20180922
#define MV_SYSTEM_FLG_ONEKEY_STOP  0x00010000  //一键下降停止

#define MV_SYSTEM_FLG_THR_DEBLOCK  0x00020000  //穿越模式解锁
#define FLAGA_POWER_LOW_TOP  0x00040000  //开机只检测一次穿越模式

#define MV_SYSTEM_FLG_LINKDOWN  0x00080000

#define MV_SYSTEM_FLG_SENDTX  0x00100000   //需要发送对码数据
#define	MV_SYSTEM_FLG_POWER_LOW_S3	0x00200000
#define MV_SYSTEM_FLG_FBLRFLAG  0x00400000  //定高打舵
#define MV_SYSTEM_FLG_UPDOWNFLAG  0x00800000  //定高上升下降标志

#define MV_SYSTEM_FLG_ONLYFBLR  0x01000000  //定高打舵标志
#define MV_SYSTEM_FLG_DMMODE  0x02000000 //对码模式
#define MV_SYSTEM_FLG_RX  0x04000000

#define MV_SYSTEM_FLG_FH  0x08000000 //返航
#define MV_SYSTEM_FLG_NOHEAD  0x10000000  //无头
#define MV_SYSTEM_FLG_FH1  0x20000000
#define MV_SYSTEM_FLG_THRBOTTON  0x40000000  //油门拉到底下降
#define MV_SYSTEM_FLG_POWER_LOW_S5 0x80000000


//记录LED灯状态
enum	LED_FLASH_KIND
{
	EV_LED_M=0,	//手动模式
	EV_LED_1,	//1.未校准陀螺仪=两灯交替闪烁
	EV_LED_2,	//2.遥控器未对码或丢失=两灯同时闪烁，频率1S/次。
	EV_LED_3,	//3.已对码=两灯全亮。
	EV_LED_4,	//4.飞行器校零=两灯同时闪烁6次，频率0.5S/次
	EV_LED_5,	//5.飞行器低压=两灯同时闪烁，频率0.2S/次
	EV_LED_6,	//6.飞行器卡死=两灯同时双闪，频率0.5S/次
	EV_LED_7, //无头
//	EV_LED_8,//陀螺仪初始化

	EV_LED_NULL,//无效指示灯状态，用于自动回位
};
typedef	enum	Fly_Kind_Mode
{
	Fly_Kind_Auto=0,	//自动模式
	Fly_Kind_Althold,	//定高模式
	
	Fly_Kind_Undefine	//未定义模式
}__Fly_Kind_Mode;
typedef	enum	Tel_Dat_Mode
{
	Tel_Dat_Throttle=0,	//油门0
	Tel_Dat_Yaw,		//偏航1
	Tel_Dat_Null,		//未定义2
	Tel_Dat_PitchMain,	//主pitch3
	Tel_Dat_RollMain,	//主roll4
	Tel_Dat_PitchSub,	//次pitch5
	Tel_Dat_RollSub,	//次roll6
	Tel_Dat_Misc,		//杂项7
}__Tel_Dat_Mode;

typedef	enum	System_Vol_Mode
{
	System_Vol_T1=0,	//电机卡死检测T1
	System_Vol_T2=1,	//电机卡死检测T2
	System_Vol_T3=2,	//电机卡死检测T3
	System_Vol_T4=3,	//电机卡死检测T4
	System_Vol_PowerLow=4	//低压检测
}__System_Vol_Mode;
typedef struct	SingleKlmPar{
	volatile	__FP_ACC	x_pre;
	volatile	__FP_ACC	p_pre;
	volatile	__FP_ACC	Q;
	volatile	__FP_ACC	R;
}__SingleKlmPar;
typedef	enum	Axis
{
	__x=0,	//低压检测
	__y,	//电机卡死检测T1
	__z,	//电机卡死检测T2
	__h,
}__Axis;
typedef	enum	PidParameterType
{
	__normal=0,	//正常模式
	__falloff,	//翻转模式
	
}__PidParameterType;

/*
**************************************************************************
extern macro<FUNCTION>
**************************************************************************
*/
#define	MF_SET_FLAG(flag,flagtype)	(flag|=flagtype)
#define	MF_CLR_FLAG(flag,flagtype)	(flag&=~flagtype)
#define	MF_IF_FLAG(flag,flagtype)	(flag&flagtype)
#define	MF_IF_NOFLAG(flag,flagtype)	((flag&flagtype)==0)

/*
**************************************************************************
define date type
**************************************************************************
*/
typedef struct	ThisHal{
//	void	(*SystemClockSupplyInit)(void);/*系统时钟供应初始化*/
//	void	(*SystemMemoryRemapInit)(void);/*系统内存重映射初始化*/
	void	(*SystemPeripheralEnableInit)(void);/*系统外设使能初始化*/
	void	(*SystemMsicPeripheralInit)(void);/*系统外设初始化*/
	void	(*SystemInterruputInit)(void);/*系统中断初始化*/
	void	(*SystemGPIOInit)(void);/*系统管脚初始化*/
	void	(*Iwdg_Init)();
	void	(*Iwdg_Feed)();
	void	(*SystemRestart)(void);/*系统重启*/
	//void	(*SystemVersionPrint)(void);/*系统打印信息*/
	//uint32	(*ReadADValue)(uint32	ch);/*读取AD通道*/
	void	(*AircraftInit)(void);/*飞行功能模块初始化*/
	void	(*SystemUpdateMotorPower)(int* dat);/*更新电机动力*/
	void	(*SystemWriteDisk)(uint32 address,uint16* dat,uint32 len);/*系统存储信息*/
	void	(*SystemReadDisk)(uint32 address,uint16* dat,uint32 len);/*系统读取信息*/
	//void	(*SetIO)(int gr,int ch,int va);/*设置IO数值*/
	//int		(*ReadIO)(int gr,int ch);/*读取IO数值*/
	void	(*CheckoutTel)(void);/*遥控器对码*/
	int		(*ReadTel)(void);/*读取遥控器数值*/
	void	(*ReadAcc)(void* dat);/*读取加速度计数值*/
	void	(*ReadGyro)(void* dat);/*读取陀螺仪数值*/
//	void	(*ReadMag)(void* dat);/*读取磁力计数值*/
	//void	(*ReadBaro)(void);/*读取气压计数值*/
//	void	(*ReadGps)(void* dat);/*读取GPS数值*/
	uint32	(*SystemGetMirco)(void);
}__ThisHal;



typedef struct	GyroDat{
	//航空欧拉次序z-x-y/yaw-pitch-roll/天东北
	int16	yaw;//z轴，天
	int16	pitch;//x轴，东
	int16	roll;//y轴，北
}__GyroDat;
typedef struct	AccDat{
	//航空欧拉次序z-x-y/yaw-pitch-roll/天东北
	int16	yaw;//z轴，天
	int16	pitch;//x轴，东
	int16	roll;//y轴，北
}__AccDat;
typedef struct	qu{
	//四元数
	__FP_ACC	q0;
	__FP_ACC	q1;
	__FP_ACC	q2;
	__FP_ACC	q3;
}__qu;
typedef	struct	Euler{
	__FP_ACC	yaw_z;//z-axis,1st;
	__FP_ACC	pitch_x;//x-axis,2nd;
	__FP_ACC	roll_y;//y-axis,3rd;
}__Euler;
typedef	struct	PidPar{
	__FP_ACC	tar;//目标值
	__FP_ACC	cur;//当前值
	__FP_ACC	p_err;//p环节
	__FP_ACC	i_err;//i环节
	__FP_ACC	d_err;//d环节
	__FP_ACC	dd_err;//二重d环节

	__FP_ACC	results;//结果
	__FP_ACC	resultsf;//结果
} __PidPar;
typedef struct	BaroDat{
	__FP_ACC	pressure;//气压运算结果
	__FP_ACC	temp;//高度运算结果
}__BaroDat;
typedef struct	TheVelocity{
	volatile	__FP_ACC	x;
	volatile	__FP_ACC	y;
	volatile	__FP_ACC	z;
}__TheVelocity;
typedef struct	TheTrack {
	volatile	__FP_ACC	x;
	volatile	__FP_ACC	y;
	volatile	__FP_ACC	z;
}__TheTrack;
typedef struct	NowTime {
	int	hour;
	int	minute;
	int	second;
	int	miro_sec;
}__NowTime;
typedef struct	ThisBird{
	__GyroDat	GyroDatReZero;//当前陀螺仪零点
	__AccDat	AccDatReZero;//当前加速度计零点
	__GyroDat	GyroDatNow;//当前陀螺仪测量值
	__AccDat	AccDatNow;//当前加速度计测量值
	int			MotorValue[8];//当前电机动力值
	int			MotorValue_pwm[4];//电机pwm值
	uint8		TelDat[10];//当前遥控数值
	__Euler		EulerAngle;//当前欧拉角yaw_z->pitch_x->roll_y
	int			Calc_coeff[13];//气压计calc coefficient
	__BaroDat	barodat;
	uint16		system_vol[8];//系统电压，用于低压检测
	__NowTime	time;
}__ThisBird;
typedef struct	TheZAxis{
	volatile	int	quadrant;//[-12,12]
	volatile	__FP_ACC	Z_axisAngle;//当前z轴角度

}__TheZAxis;
typedef enum
{
	IDLE = 0,
	TX = 1,
	RX = 2,
} radio_mode_t;
//typedef struct
//{
//	uint32_t quad_id;
//	uint8_t  quad_unlock;
//	uint8_t  channel[4];
//	uint8_t  channel_num;
//	
//	uint16_t tick;
//	uint16_t life;
//} radio_object_t;
//李伟刹车
typedef struct
{
	/*本次俯仰、横滚累加值*/
	volatile float Pitch_Sum;
	volatile float Last_Pitch_Sum;	
	/*前一次俯仰、横滚累加值*/
	volatile float Roll_Sum;
	volatile float Last_Roll_Sum;	
	/*俯仰、横滚累加平均*/
	volatile float Pitch_Averge;
	volatile float Roll_Averge;
	/*累加次数*/
	volatile uint16 Sum_Times;
	/*累加过程*/
	volatile uint8  Step_1;
	/*求平均过程*/
	volatile uint8  Step_2;
	/*刹车倍数*/
	volatile float  Multiple_Pitch;
	/*刹车倍数*/
	volatile float  Multiple_Roll;
	/*刹车时间*/
	volatile uint8  Times;
}Breaking;
/*
**************************************************************************
extern global variable
**************************************************************************
*/
extern	const	char	gcu8_ThisVersion[8];
extern	volatile	__ThisHal	gvst_ThisHal;
extern	volatile	__ThisBird	gvst_ThisBird;
extern	volatile	uint32	gvu32_SystemRunTime;
extern	volatile	uint32	gvu32_SystemFlag;
extern	volatile	uint32	gvu32_ImuSamFeqCnt;
extern	volatile	uint8	gvu8_U1ReceiveBuf[MV_U1_BUF];
extern	volatile	uint8	gvu8_TelTxAddr[6];
extern	volatile	uint8	gvu8_TelRxAddr[5];
extern	volatile	uint8	gvu8_U1ReceiveBufCnt;
extern	volatile	uint8	gvu8_U1ReceiveTimeCnt;
//extern	volatile	uint8 mpu6050_acc[6];
//extern  volatile  uint8 mpu6050_gyro[6];
extern	volatile	__FP_ACC	gvfpacc_CurHeight;//当前高度
extern	volatile	__FP_ACC	gvfpacc_ZeroHeight;//初始高度
extern	volatile	enum	LED_FLASH_KIND	gven_LedFlashKind;

extern  volatile uint8 _hw2000_irq_request;
extern  volatile uint8 indexcount;
extern  volatile uint32_t quad_id;
extern  volatile uint8_t  quad_unlock;
extern  volatile uint8_t  channel[5];
extern  volatile uint8_t  channel_num;
	
extern  volatile uint16_t tick;
extern  volatile uint16_t life;

extern  volatile uint32_t quad_id;
extern  volatile uint32 quad_idf;
extern  volatile uint32_t FLAGA;
extern  volatile int ms_count;

extern	volatile	uint8  START_MODE;
extern	volatile	uint16 START_TEST_TIMER;
extern	volatile	uint16 sleep_timer;
extern	uint32 gyro_acc_data[16];
extern	volatile  uint32 vol_pwm[8];
extern  volatile  int32  realthrottlea;
extern  volatile  int32	realthrottleb;
extern  volatile  int32	realthrottlesave;

extern  volatile  uint8 hold_high_statue;
extern	volatile	int32	H_I;//当前高度
extern	volatile	int32	H_I_initialvalue;//初始高度
extern	volatile	int32 H_I_average;     //求出气压平均值
extern	volatile	int32 H_I_zero;     //油门为零时气压
extern  volatile	int32 H_I_start;
//extern	volatile  int16 baro_out;
extern	volatile  int16  dg_updown;

extern  volatile  int16 baro_speed1;
extern  volatile  int32 fbm320_error1;

extern  volatile  uint8 fblr_state;
extern  volatile  uint8 fblr_stateall;

extern  volatile  uint16 tx_rx_delayms;

extern  volatile  int16 accel_out;
extern  volatile  int16 baro_dis;

extern  volatile  float check_accz_vel;
extern  volatile  int16 fblr_com;
extern  volatile  int16 fblr_com1;
extern  volatile  uint16 up_to_keep;
extern  volatile  int16 up_down_speed;

extern  volatile  float log_lz;
extern  volatile  int16 one_secon;
extern  volatile  float acc_velf;
//extern  volatile  float coefficient_acc;
//extern  volatile  float gxgygz;

//穿越模式加锁
//extern  volatile  uint16 thr_clock_count;
extern  volatile  int16  real_accz;

extern  volatile  float ak;
extern  volatile  float spin_rate_rad_s;
//2.4G跳频
extern  volatile  uint8 carrier_time_count;
extern  uint8 dataTel[20];
extern  volatile uint8 signal_strength;
extern  volatile uint8 signal_display;
extern  volatile uint8 signal_display1;
extern  volatile uint8 throttle_stop_time;

extern  volatile int16 accel_outhand;
//extern  volatile int8 pitch_trim,roll_trim,pitch_trim1,roll_trim1;
extern  volatile uint16 data1a,data2a;
extern  uint8 dataTX[10];
extern  volatile uint8 link_select;
extern  volatile int baro_home;
extern volatile   int	   throttle_stick;	


/*
**************************************************************************
std C library
**************************************************************************
*/
#include	<stdio.h>
#include	<ctype.h>
#include	<string.h>
#include	<stdlib.h>
#include	<math.h>
#include	<rt_misc.h>

/*
**************************************************************************
stm32 library
**************************************************************************
*/
//#include "stm32f0xx_conf.h"
//#include "arm_math.h"


/*
**************************************************************************
project library
**************************************************************************
*/
//#include	"stm32f0xx_it.h"
#include	"fylib.h"
#include	"hal.h"
#include	"app.h"
#include	"err.h"


/*
**************************************************************************
system define
**************************************************************************
*/




#endif
