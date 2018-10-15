#include "altitude.h"


volatile	static	__SingleKlmPar Kalman_accz[4]={0,0,0.02f,6.00f};

extern struct vertical_information  baro_acc_alt;   //�߶��ں�

//static  double z_est[6]={0,0,0,0,0,0};
volatile	static int16 angle_com=0;

//����Z����ٶȼ�PID
#define H_KP      1.1f
#define H_KI1     0.06f
#define H_KI2     0.25f
#define H_KD      40.0f

#define angle_com_limit 40


#define Fly_Auto    0
#define Fly_Althold  1
#define ONEKEY_UP   1
#define NAMOL_UP   0
void	check_flymode(uint8 throttle_advalue,uint8 mode_sure)   //���ߺ���   //20ms
{
	static   uint8 thr_deblock_count=0,thr_block_count=0;
//	static   uint16  start_up_timer=1001;   // ���ο�ʼ�����׶ηɻ�ֱ������һ��Ž�����������ģʽ
//	static   int32 H_I_average_stop;
	static   uint8 up_flag=NAMOL_UP,up_flag1=0,down_flag1=0;
	static   uint8 throttle_stop_advalue=0;//,throttle_max_min=0x80;
	static   uint8 fall_count=0;
	static   uint16 block_timer=0;
	uint8 thr_type;
	/***/
	static uint8 Times=0;
	static u8 Deblock_Flag=0;
	
	/*ң�ؽ���˺� ��Ҫң��ȫ������ ������ȷ���� Frank 18.8.31 PM*/
//	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK)&&gvst_ThisBird.TelDat[0]<=0x85 && gvst_ThisBird.TelDat[0]>=0x75
//																						&&gvst_ThisBird.TelDat[1]<=0x85 && gvst_ThisBird.TelDat[1]>=0x75
//																						&&gvst_ThisBird.TelDat[3]<=0x85 && gvst_ThisBird.TelDat[3]>=0x75
//																						&&gvst_ThisBird.TelDat[4]<=0x85 && gvst_ThisBird.TelDat[4]>=0x75)
//	{
//		Deblock_Flag=1;
//	}
//	else if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK))
//	{
//		Deblock_Flag=0;
//	}	
	
	if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_STOPF))   //����ֹͣǰ
	{
		if(hold_high_statue==HOLD_HIGH_UP)
		  dg_updown=(throttle_advalue-0x80)/0.5f;
		else if(hold_high_statue==HOLD_HIGH_DOWN)
			dg_updown=(throttle_advalue-0x80)/0.5f;
		if(dg_updown<0)
			dg_updown=-dg_updown;
		else if(dg_updown>0 && dg_updown<30)
			dg_updown=30;
	}
	if(throttle_advalue<0x90)
	  MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEY_STOP);   //һ���½��������Ÿ�����
	
	if(throttle_advalue>0x0a)
	  throttle_stop_time=0;
	else
	{
		if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOTOR_BK))
		  MF_CLR_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_MOTOR_BK);
	}
  //����
	if(gvst_ThisBird.TelDat[0]<=60 && gvst_ThisBird.TelDat[3]<0x70 && gvst_ThisBird.TelDat[4]>0x90 && gvst_ThisBird.TelDat[1]<0x70 && hold_high_statue==HOLD_HIGH_STOP)
	{
		thr_block_count=0;
		thr_deblock_count++;
		if(thr_deblock_count>=30)
		{
			thr_deblock_count=0;
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK);
		}
	}
	else
		thr_deblock_count=0;
	
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK) || MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2))
		MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK);
//����
  if(gvst_ThisBird.TelDat[0]<=32)// && gvst_ThisBird.TelDat[3]<0x70 && gvst_ThisBird.TelDat[4]<0x70 && gvst_ThisBird.TelDat[1]>0x90 && hold_high_statue==HOLD_HIGH_STOP)
	{
		thr_block_count++;
		if(thr_block_count>=30)
		{
			thr_block_count=0;
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK);
		}
	}
	else
		thr_block_count=0;
	
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK))
	{
		block_timer++;
		if(block_timer>=700)
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK);
	}
	else
		block_timer=0;
	
		
	
	if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_AUTO_ALT1))
		thr_type=Fly_Althold;
	else
		thr_type=Fly_Auto;
	
	if((FL_ABS(gvst_ThisBird.EulerAngle.pitch_x)>80 || FL_ABS(gvst_ThisBird.EulerAngle.roll_y>80)) && MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
	{
		fall_count++;
		if(fall_count>=20)
		{
			fall_count=0;
			hold_high_statue=HOLD_HIGH_STOP;
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_STOPF);
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_LINKDOWN);
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN);
			MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEY_STOP);   //һ���½�ֹͣ
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP);
			MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN);
		}
		return;
	}
	else
		fall_count=0;
	switch(thr_type)
	{
		case	Fly_Auto:
		{

			if(throttle_advalue>=0x0a)
	    {
				if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2))
					return;
				  hold_high_statue=HOLD_HIGH_UP;
	    }
			else
				hold_high_statue=HOLD_HIGH_STOP;
			break;
		}
		case	Fly_Althold:     //����
		{
			
			if(hold_high_statue==HOLD_HIGH_STOP)
			{
				down_flag1=0;    //����һ���½����Ÿ�
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THRBOTTON);
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_STOPF);
				throttle_stop_advalue=throttle_advalue;   //��סֹͣ���Ÿ�
//				if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK)||Deblock_Flag==0)
//					return;
				if(MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK))
					return;
				if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK)&&(FL_ABS(gvst_ThisBird.TelDat[3]-0x80)>30 || FL_ABS(gvst_ThisBird.TelDat[4]-0x80)>30))//ZFM  ���ӽ�����δ�ɿ�pitch��roll �����  20180830
				{
					return;
				}
				if(throttle_advalue>=0xa0 && MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_ONEKEY_STOP))
	      {
					if(MF_IF_NOFLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2))
					{
//						Times++;
//						if(Times>5||throttle_advalue>0xc0) {
//							Times=0;
							hold_high_statue=HOLD_HIGH_UP;
							MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK);
//						}
//						else
//							return;
					}
					else
						return;
	      }
				else
				{
					if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP))
					{
						dg_updown=70;
						hold_high_statue=HOLD_HIGH_UP;
						MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THR_DEBLOCK);
					}
					return;
				}
			}
			else
			{
				if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_POWER_LOW_S2) || (MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_LOCK_LINK) && MF_IF_NOFLAG(FLAGA,MV_SYSTEM_FLG_LINKDOWN)))
			  {
					dg_updown=120;
			  	hold_high_statue=HOLD_HIGH_DOWN;    //��ѹ����
			  	return;
			  }
			}
			

			if(throttle_advalue-throttle_stop_advalue>15 || throttle_advalue-throttle_stop_advalue<-15)
			{
				up_flag=NAMOL_UP;//���������Ÿ�����
				if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP))
				{
					hold_high_statue=HOLD_HIGH_KEEP;
					MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP);
				}
			}
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP))
			{
				up_flag1=0;
				up_flag=ONEKEY_UP;//һ����������
				dg_updown=100;
				hold_high_statue=HOLD_HIGH_UP;
				  return;
			}
			
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN))
			{
				up_flag1=1;
				up_flag=NAMOL_UP;
			}
			if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN))
			{
				dg_updown=200;
				hold_high_statue=HOLD_HIGH_DOWN;
				if(throttle_advalue-throttle_stop_advalue>15 || throttle_advalue-throttle_stop_advalue<-15)
					down_flag1=1;
//				if(down_flag1==1)
//				{
//					if(throttle_stop_time==0)
//						throttle_stop_time=1;
//					if(throttle_stop_time>=30)
//					{
//						MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP);
//						MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN);
//						hold_high_statue=HOLD_HIGH_STOP;
//					}
//				}
				return;
			}
			if(up_flag==ONEKEY_UP)//һ��������ת������һ����ɶ������������
				return;
			else
			{
				up_flag=NAMOL_UP;
				if(throttle_advalue-throttle_stop_advalue>15 || throttle_advalue-throttle_stop_advalue<-15)  //ҡ���Ѷ�
				{
					if(throttle_advalue-throttle_stop_advalue>15 && throttle_advalue>=0x6e)
					  up_flag1=1;
					else if(throttle_advalue-throttle_stop_advalue>15)
						throttle_stop_advalue=throttle_advalue;
					if(throttle_advalue-throttle_stop_advalue<-15)
						up_flag1=1;
				}
				if(up_flag1!=1)
					return;         //���Ÿ�δ�Ӵ�����
//				if(up_flag1==1 && throttle_advalue==0x00)
//				{
//					if(throttle_stop_time==0)
//						throttle_stop_time=1;
//					if(throttle_stop_time>=30)
//					{
//						MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP);
//						MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN);
//						hold_high_statue=HOLD_HIGH_STOP;
//					}
//				}
			}
			

			
	    if(throttle_advalue>=187)
	    {
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THRBOTTON);
				up_flag=NAMOL_UP;
				hold_high_statue=HOLD_HIGH_UP;
	    }
	    else if(throttle_advalue>=67)
	    {
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THRBOTTON);
		    hold_high_statue=HOLD_HIGH_KEEP;
	    }
	    else if(throttle_advalue>=0x01)
	    {
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_THRBOTTON);
				hold_high_statue=HOLD_HIGH_DOWN;
	    }
			if(throttle_advalue<=0x0a)
			{
				if(MF_IF_FLAG(FLAGA,FLAGA_FLIP_FINISH)||MF_IF_FLAG(FLAGA,FLAGA_FLIP_END_UP))
				{
					MF_CLR_FLAG(FLAGA,FLAGA_FLIP_FINISH);
					MF_CLR_FLAG(FLAGA,FLAGA_FLIP_END_UP);
					
				}
				hold_high_statue=HOLD_HIGH_DOWN;
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYUP);
				MF_CLR_FLAG(FLAGA,MV_SYSTEM_FLG_ONEKEYDOWN);
				MF_SET_FLAG(FLAGA,MV_SYSTEM_FLG_THRBOTTON);
//				if(MF_IF_FLAG(FLAGA,MV_SYSTEM_FLG_FSTAGE))
//					hold_high_statue=HOLD_HIGH_STOP;
			}
			break;
		}
	}
	return;
}

