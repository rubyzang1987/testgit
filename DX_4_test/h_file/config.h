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

/*����������������*/
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
//2.4ͨ��Э�� APEX ����� Frank 18.7.4 AM
#define APEX 1

#if APEX
 #define NO_Data_TIME 4
#else
 #define NO_Data_TIME 16
#endif
/***************************************/
/*
/////////////////////////////////////
�˲���������
/////////////////////////////////////
*/
#define	__FP_ACC	fp32//�˲����㾫��
#define	MV_ACC_RP	4096//16bit for +-8g
#define	MV_GYRO_ANG_RP	934.90f//16bit for +-34.9angle/s   34.9����/S  32768/938.9=34.9
#define	MV_GYRO_RP	16.38f//16bit for +-2000der/s
#define	MV_RAD_DEG	57.30f//1rad for deg
#define	MV_STAND_PRESS	101325.0f//��׼����ѹ

#define	MV_GYRO_OLDDAT_WEIGHT	0.95f//����������������Ȩ��
#define	MV_GYRO_NEWDAT_WEIGHT	0.05f//����������������Ȩ��
#define	MV_ACC_OLDDAT_WEIGHT	0.95f//���ٶȼ�������Ȩ��
#define	MV_ACC_NEWDAT_WEIGHT	0.05f//���ٶȼ�������Ȩ��


#define	CARRIER_TIME	16
#define	CHANNEL_MAX1		5

#define HOLD_HIGH_UP    0 //���������׶�
#define HOLD_HIGH_KEEP  1 //�����ȶ�
#define HOLD_HIGH_DOWN  2 //�����½��׶�
#define HOLD_HIGH_STOP  3 //����ֹͣ�׶�

#if APEX
	#define MATCH_FRE 79
#else
	#define MATCH_FRE 60
#endif

#define LINK_NOLINK 0
#define LINK_USART 1
#define LINK_2P4G 2

//ǰ�����Ҵ��
#define STATE_F 1

#define fblr_nomode       0
#define fblr_yesmode       1


/*
/////////////////////////////////////
�˲���������
/////////////////////////////////////
*/

#define	MV_USE_HAL_KIND		1//���õ�·�壬0С��1�а�2���
#define	MV_SYSTEM_CLOCK		48000000	//ϵͳʱ��36000000 & 48000000ʱ
#define	MV_CMD_PREFIX		""		//������ǰ׺�����Ա�ʾ
#define	MV_U1_BUF			11		//����1����
#define	MV_U1_TIMEOUT		20		//����1���ճ�ʱ��ʱ������λ����
#define	MV_GET_SENSOR_ZERO_TIM		1000	//��ȡ���������ƽ������
#define	MV_FREQ_SYSTEM_TICK	1000	//ϵͳ�ж�ʱ��Ƶ��
#define	MV_FREQ_READ_TEL	20	//��ȡң����Ƶ�ʵ�λms
#define	MV_FREQ_READ_IMU	3	//��ȡ������IMU��Ԫ����Ƶ�ʵ�λms
#define	MV_FREQ_READ_GPS	4	//��ȡGPSƵ�ʵ�λms
#define	MV_FREQ_READ_BARO	1	//��ȡ��ѹ��Ƶ�ʵ�λms
#define	MV_TEL_DROP_DELAY	100//(5*MV_FREQ_SYSTEM_TICK/MV_FREQ_READ_TEL)//ң����������ʱʱ��
#define	MV_PHOTO_TIMEOUT	200
#define	MV_DV_TIMEOUT		1000

#define NVDS_PAGE_START_ADDR	0x00008c00

/*
������
���һΪ���Ͻ�PIN32/PB8��˳ʱ����ת
��Ŷ�Ϊ���½�PIN9/PA3����ʱ����ת
�����Ϊ���½�PIN18/PA8��˳ʱ����ת
�����Ϊ���Ͻ�PIN21/PA11����ʱ����ת
*/
#define	MV_HOWMANY_MOTOR	4	//��¼�����ж��ٸ����
#define	MV_MAX_MOTOR_VALUE	960	//��߶���ֵ
#define	MV_MIN_MOTOR_VALUE	0	//��Ͷ���ֵ
#define	MV_MOTOR_NO1	0	//������һ
#define	MV_MOTOR_NO2	1	//�����Ŷ�
#define	MV_MOTOR_NO3	2	//��������
#define	MV_MOTOR_NO4	3	//��������

#define	MV_ADC_SYSTEM_VCC	ADC_Channel_0//ϵͳ��ѹ���ͨ��
#define	MV_ADC_M3M4_VCC		ADC_Channel_1//���3��4���ͨ��
#define	MV_ADC_M1M2_VCC		ADC_Channel_2//���1��2���ͨ��

#define	MV_SYSTEM_FLG_U1BUSY	0x00000001//����1æ
#define	MV_SYSTEM_FLG_U1DONE	0x00000002//����1���
#define	MV_SYSTEM_FLG_GET_PHOTO	0x00000004//����
#define	MV_SYSTEM_FLG_GET_DV	0x00000008//¼��

#define	MV_SYSTEM_FLG_TEL_DOCK	0x00000010//ң�ض���ɹ���һ��ͨ�������һ��
#define	MV_SYSTEM_FLG_TEL_DROP	0x00000020//ң�ص��߻���Ƶ��
#define	MV_SYSTEM_FLG_GET_ZERO	0x00000040//�������Ѿ�У׼
#define	MV_SYSTEM_FLG_MOTOR_BK	0x00000080//������������

#define	MV_SYSTEM_FLG_READ_TEL	0x00000100//��ȡң������
#define	MV_SYSTEM_FLG_READ_IMU	0x00000200//��ȡλ�ô��е�Ԫ����
#define	MV_SYSTEM_FLG_MOT_RUN	0x00000400//�������
#define	MV_SYSTEM_FLG_READ_BARO	0x00000800//��ȡ��ѹ����Ϣ

#define	MV_SYSTEM_FLG_FALL_L	0x00001000//�󷭸�ͷ
#define	MV_SYSTEM_FLG_FALL_R	0x00002000//�ҷ���ͷ
#define	MV_SYSTEM_FLG_FALL_F	0x00004000//ǰ����ͷ
#define	MV_SYSTEM_FLG_FALL_D	0x00008000//�󷭸�ͷ

#define	MV_SYSTEM_FLG_FALL_ST1		0x00010000//��һ�׶�
#define	MV_SYSTEM_FLG_FALL_ST2		0x00020000//��һ�׶�
#define	MV_SYSTEM_FLG_FALL_ST3		0x00040000//��һ�׶�
#define	MV_SYSTEM_FLG_FALL_ST4		0x00080000//��һ�׶�

#define	MV_SYSTEM_FLG_ALT_CHANGE	0x00100000//����ģʽ�·ɻ����¸߶�
#define	MV_SYSTEM_FLG_ALT_RUN		0x00200000//����ģʽ�·ɻ�����
#define	MV_SYSTEM_FLG_POWER_LOW_S1	0x00400000
#define	MV_SYSTEM_FLG_POWER_LOW_S2	0x00800000

#define MV_SYSTEM_FLG_PWM_OC        0x01000000 //����pwm��־λ 1Ϊ�� 0Ϊ��
#define MV_SYSTEM_FLG_START_UP        0x02000000
#define MV_SYSTEM_FLG_LOCK_LINK        0x04000000  //ң��������
#define MV_SYSTEM_FLG_6881OK        0x08000000  //test 
#define	MV_SYSTEM_FLG_GET_PHOTO1	    0x10000000//����
#define	MV_SYSTEM_FLG_FALL_BEHIDE	    0x20000000//
#define	MV_SYSTEM_FLG_PWM_ZERO      	0x40000000//


//#define	MV_SYSTEM_FLG_
//#define	MV_SYSTEM_FLG_OUTCTRL	0x80000000//����״̬��ʹ�ɻ�����IMU����

//FLAGA_FLAG
#define	MV_SYSTEM_FLG_50MSPWM	0x00000001  //PWM 50MS OPEN AND THEN CLOSE DETECT ADC
#define	MV_SYSTEM_FLG_FFLAG	0x00000002    //FALL FLAG
#define	MV_SYSTEM_FLG_FOF	0x00000004   //����0f��־λ
#define	MV_SYSTEM_FLG_FSTAGE	0x00000008
#define	MV_SYSTEM_FLG_TESTF	0x00000010
#define MV_SYSTEM_FLG_AUTO_ALT1 0x00000020
#define MV_SYSTEM_FLG_AUTO_ALT 0x00000040
#define MV_SYSTEM_FLG_POWER_LOW_S6 0x00000080  //���Ӳ��� ZFM 20181009
#define MV_SYSTEM_FLG_ONEKEYUP 0x00000100   //һ����ɻ���
#define MV_SYSTEM_FLG_ONEKEYDOWN 0x00000200   //һ����ɻ���
#define MV_SYSTEM_FLG_START_FLAG 0x00000400   //������־
#define MV_SYSTEM_FLG_POWER_LOW_S4  0x00000800 //���߼��ֹͣ
#define MV_SYSTEM_FLG_STOPF  0x00001000 //����ֹͣǰ
#define MV_SYSTEM_FLG_FALLSTAGE  0x00002000  //������һ�ζ���
#define FLAGA_FLIP_END_UP  0x00004000  //��������Ժ�������ش��������ڴӷ�������Ժ�ֱ���ж���ʱ���ų�������  ZFM 20180922
#define FLAGA_FLIP_FINISH  0x00008000  //������ɺ�ı�־  ZFM 20180922
#define MV_SYSTEM_FLG_ONEKEY_STOP  0x00010000  //һ���½�ֹͣ

#define MV_SYSTEM_FLG_THR_DEBLOCK  0x00020000  //��Խģʽ����
#define FLAGA_POWER_LOW_TOP  0x00040000  //����ֻ���һ�δ�Խģʽ

#define MV_SYSTEM_FLG_LINKDOWN  0x00080000

#define MV_SYSTEM_FLG_SENDTX  0x00100000   //��Ҫ���Ͷ�������
#define	MV_SYSTEM_FLG_POWER_LOW_S3	0x00200000
#define MV_SYSTEM_FLG_FBLRFLAG  0x00400000  //���ߴ��
#define MV_SYSTEM_FLG_UPDOWNFLAG  0x00800000  //���������½���־

#define MV_SYSTEM_FLG_ONLYFBLR  0x01000000  //���ߴ���־
#define MV_SYSTEM_FLG_DMMODE  0x02000000 //����ģʽ
#define MV_SYSTEM_FLG_RX  0x04000000

#define MV_SYSTEM_FLG_FH  0x08000000 //����
#define MV_SYSTEM_FLG_NOHEAD  0x10000000  //��ͷ
#define MV_SYSTEM_FLG_FH1  0x20000000
#define MV_SYSTEM_FLG_THRBOTTON  0x40000000  //�����������½�
#define MV_SYSTEM_FLG_POWER_LOW_S5 0x80000000


//��¼LED��״̬
enum	LED_FLASH_KIND
{
	EV_LED_M=0,	//�ֶ�ģʽ
	EV_LED_1,	//1.δУ׼������=���ƽ�����˸
	EV_LED_2,	//2.ң����δ�����ʧ=����ͬʱ��˸��Ƶ��1S/�Ρ�
	EV_LED_3,	//3.�Ѷ���=����ȫ����
	EV_LED_4,	//4.������У��=����ͬʱ��˸6�Σ�Ƶ��0.5S/��
	EV_LED_5,	//5.��������ѹ=����ͬʱ��˸��Ƶ��0.2S/��
	EV_LED_6,	//6.����������=����ͬʱ˫����Ƶ��0.5S/��
	EV_LED_7, //��ͷ
//	EV_LED_8,//�����ǳ�ʼ��

	EV_LED_NULL,//��Чָʾ��״̬�������Զ���λ
};
typedef	enum	Fly_Kind_Mode
{
	Fly_Kind_Auto=0,	//�Զ�ģʽ
	Fly_Kind_Althold,	//����ģʽ
	
	Fly_Kind_Undefine	//δ����ģʽ
}__Fly_Kind_Mode;
typedef	enum	Tel_Dat_Mode
{
	Tel_Dat_Throttle=0,	//����0
	Tel_Dat_Yaw,		//ƫ��1
	Tel_Dat_Null,		//δ����2
	Tel_Dat_PitchMain,	//��pitch3
	Tel_Dat_RollMain,	//��roll4
	Tel_Dat_PitchSub,	//��pitch5
	Tel_Dat_RollSub,	//��roll6
	Tel_Dat_Misc,		//����7
}__Tel_Dat_Mode;

typedef	enum	System_Vol_Mode
{
	System_Vol_T1=0,	//����������T1
	System_Vol_T2=1,	//����������T2
	System_Vol_T3=2,	//����������T3
	System_Vol_T4=3,	//����������T4
	System_Vol_PowerLow=4	//��ѹ���
}__System_Vol_Mode;
typedef struct	SingleKlmPar{
	volatile	__FP_ACC	x_pre;
	volatile	__FP_ACC	p_pre;
	volatile	__FP_ACC	Q;
	volatile	__FP_ACC	R;
}__SingleKlmPar;
typedef	enum	Axis
{
	__x=0,	//��ѹ���
	__y,	//����������T1
	__z,	//����������T2
	__h,
}__Axis;
typedef	enum	PidParameterType
{
	__normal=0,	//����ģʽ
	__falloff,	//��תģʽ
	
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
//	void	(*SystemClockSupplyInit)(void);/*ϵͳʱ�ӹ�Ӧ��ʼ��*/
//	void	(*SystemMemoryRemapInit)(void);/*ϵͳ�ڴ���ӳ���ʼ��*/
	void	(*SystemPeripheralEnableInit)(void);/*ϵͳ����ʹ�ܳ�ʼ��*/
	void	(*SystemMsicPeripheralInit)(void);/*ϵͳ�����ʼ��*/
	void	(*SystemInterruputInit)(void);/*ϵͳ�жϳ�ʼ��*/
	void	(*SystemGPIOInit)(void);/*ϵͳ�ܽų�ʼ��*/
	void	(*Iwdg_Init)();
	void	(*Iwdg_Feed)();
	void	(*SystemRestart)(void);/*ϵͳ����*/
	//void	(*SystemVersionPrint)(void);/*ϵͳ��ӡ��Ϣ*/
	//uint32	(*ReadADValue)(uint32	ch);/*��ȡADͨ��*/
	void	(*AircraftInit)(void);/*���й���ģ���ʼ��*/
	void	(*SystemUpdateMotorPower)(int* dat);/*���µ������*/
	void	(*SystemWriteDisk)(uint32 address,uint16* dat,uint32 len);/*ϵͳ�洢��Ϣ*/
	void	(*SystemReadDisk)(uint32 address,uint16* dat,uint32 len);/*ϵͳ��ȡ��Ϣ*/
	//void	(*SetIO)(int gr,int ch,int va);/*����IO��ֵ*/
	//int		(*ReadIO)(int gr,int ch);/*��ȡIO��ֵ*/
	void	(*CheckoutTel)(void);/*ң��������*/
	int		(*ReadTel)(void);/*��ȡң������ֵ*/
	void	(*ReadAcc)(void* dat);/*��ȡ���ٶȼ���ֵ*/
	void	(*ReadGyro)(void* dat);/*��ȡ��������ֵ*/
//	void	(*ReadMag)(void* dat);/*��ȡ��������ֵ*/
	//void	(*ReadBaro)(void);/*��ȡ��ѹ����ֵ*/
//	void	(*ReadGps)(void* dat);/*��ȡGPS��ֵ*/
	uint32	(*SystemGetMirco)(void);
}__ThisHal;



typedef struct	GyroDat{
	//����ŷ������z-x-y/yaw-pitch-roll/�춫��
	int16	yaw;//z�ᣬ��
	int16	pitch;//x�ᣬ��
	int16	roll;//y�ᣬ��
}__GyroDat;
typedef struct	AccDat{
	//����ŷ������z-x-y/yaw-pitch-roll/�춫��
	int16	yaw;//z�ᣬ��
	int16	pitch;//x�ᣬ��
	int16	roll;//y�ᣬ��
}__AccDat;
typedef struct	qu{
	//��Ԫ��
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
	__FP_ACC	tar;//Ŀ��ֵ
	__FP_ACC	cur;//��ǰֵ
	__FP_ACC	p_err;//p����
	__FP_ACC	i_err;//i����
	__FP_ACC	d_err;//d����
	__FP_ACC	dd_err;//����d����

	__FP_ACC	results;//���
	__FP_ACC	resultsf;//���
} __PidPar;
typedef struct	BaroDat{
	__FP_ACC	pressure;//��ѹ������
	__FP_ACC	temp;//�߶�������
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
	__GyroDat	GyroDatReZero;//��ǰ���������
	__AccDat	AccDatReZero;//��ǰ���ٶȼ����
	__GyroDat	GyroDatNow;//��ǰ�����ǲ���ֵ
	__AccDat	AccDatNow;//��ǰ���ٶȼƲ���ֵ
	int			MotorValue[8];//��ǰ�������ֵ
	int			MotorValue_pwm[4];//���pwmֵ
	uint8		TelDat[10];//��ǰң����ֵ
	__Euler		EulerAngle;//��ǰŷ����yaw_z->pitch_x->roll_y
	int			Calc_coeff[13];//��ѹ��calc coefficient
	__BaroDat	barodat;
	uint16		system_vol[8];//ϵͳ��ѹ�����ڵ�ѹ���
	__NowTime	time;
}__ThisBird;
typedef struct	TheZAxis{
	volatile	int	quadrant;//[-12,12]
	volatile	__FP_ACC	Z_axisAngle;//��ǰz��Ƕ�

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
//��ΰɲ��
typedef struct
{
	/*���θ���������ۼ�ֵ*/
	volatile float Pitch_Sum;
	volatile float Last_Pitch_Sum;	
	/*ǰһ�θ���������ۼ�ֵ*/
	volatile float Roll_Sum;
	volatile float Last_Roll_Sum;	
	/*����������ۼ�ƽ��*/
	volatile float Pitch_Averge;
	volatile float Roll_Averge;
	/*�ۼӴ���*/
	volatile uint16 Sum_Times;
	/*�ۼӹ���*/
	volatile uint8  Step_1;
	/*��ƽ������*/
	volatile uint8  Step_2;
	/*ɲ������*/
	volatile float  Multiple_Pitch;
	/*ɲ������*/
	volatile float  Multiple_Roll;
	/*ɲ��ʱ��*/
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
extern	volatile	__FP_ACC	gvfpacc_CurHeight;//��ǰ�߶�
extern	volatile	__FP_ACC	gvfpacc_ZeroHeight;//��ʼ�߶�
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
extern	volatile	int32	H_I;//��ǰ�߶�
extern	volatile	int32	H_I_initialvalue;//��ʼ�߶�
extern	volatile	int32 H_I_average;     //�����ѹƽ��ֵ
extern	volatile	int32 H_I_zero;     //����Ϊ��ʱ��ѹ
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

//��Խģʽ����
//extern  volatile  uint16 thr_clock_count;
extern  volatile  int16  real_accz;

extern  volatile  float ak;
extern  volatile  float spin_rate_rad_s;
//2.4G��Ƶ
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
