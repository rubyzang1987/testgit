#include	"config.h"
#ifndef	__FYLIB_H__
#define	__FYLIB_H__

typedef struct  SPEED_STRUCT{
	int32 distance_my;
	int32 errorf_my;
	int32 distance_myout;
	int16 KT_COUNT;
}__SPEED_STRUCT;

typedef struct  SPEED_STRUCTF{
	float distance_my;
	float errorf_my;
	float distance_myout;
	int16 KT_COUNT;
}__SPEED_STRUCTF;

typedef struct  AVERAGE_DATA{
	int32 data_sum;
	int16 in_count;
	int32 data_average;
}__AVERAGE_DATA;

//extern	uint32	GetCommand(void);
extern	void	DelayMs(int ms);
extern	int	TelDatCheck(uint8* dat,int cnt);
//extern	void	SendDatShow(int dat);
extern	void	Fylib_DelayMs(volatile int *time,int ms);
extern	void	Fylib_Delays(int dlys);
extern	float 	Fylib_Qrsqrt( float number );
extern	void	Fylib_AmplitudeLimiting1(const float limit,float *output);
extern	float	Fylib_KalmanFilter(void* par,float input);
extern	void	Fylib_LpFilter(const float ratio,const float input,float *output);
extern	float	Fylib_ConstrainF(float dat,float min,float max);
extern	int		Fylib_Constrain(int dat,int min,int max);
extern  float my_cos(float xx);
extern  float my_sin(float yy);
extern  float FL_ABS(float x);

extern  void inertial_filter_predict(double x[4],float acczerror,int32 baro_alt,uint16 count);
extern  int32 speed_calculate1(int32 error_my,uint16 KT_NUM,__SPEED_STRUCT* speed_par);
extern  float speed_calculatef(float error_my,uint16 KT_NUM,__SPEED_STRUCTF* speed_parf);
extern  int32 average_cal(int32 in_data,int16 KT_NUM,__AVERAGE_DATA* average_par);

#endif


