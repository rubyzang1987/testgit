#include "baro.h"
#include "fylib.h"
#include	"state_estimation.h"


//volatile	static  __SPEED_STRUCT baro_calspeed1={0,0,0,0};
//volatile	static  __SPEED_STRUCT baro_calspeed2={0,0,0,0};
#ifdef SPL06_01


static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

extern struct vertical_information  baro_acc_alt;   //�߶��ں�

void spl0601_get_calib_param(void);



/*****************************************************************************
 �� �� ��  : spl0601_init
 ��������  : SPL06-01 ��ʼ������
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
void spl0601_init(void)
{
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = 0x34;
    spl0601_get_calib_param();
    // sampling rate = 1Hz; Pressure oversample = 2;
    spl0601_rateset(PRESSURE_SENSOR,32, 8);   
    // sampling rate = 1Hz; Temperature oversample = 1; 
    spl0601_rateset(TEMPERATURE_SENSOR,32, 8);
    //Start background measurement
	spl0601_start_continuous(CONTINUOUS_P_AND_T);
    
}

/*****************************************************************************
 �� �� ��  : spl0601_rateset
 ��������  :  �����¶ȴ�������ÿ����������Լ���������
 �������  : uint8 u8OverSmpl  ��������         Maximal = 128
             uint8 u8SmplRate  ÿ���������(Hz) Maximal = 128
             uint8 iSensor     0: Pressure; 1: Temperature
 �������  : ��
 �� �� ֵ  : ��
 ���ú���  :
 ��������  :

 �޸���ʷ      :
  1.��    ��   : 2015��11��24��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
void spl0601_rateset(uint8 iSensor, uint8 u8SmplRate, uint8 u8OverSmpl)
{
    uint8 reg = 0;
    int32 i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<5);
            break;
        case 4:
            reg |= (2<<5);
            break;
        case 8:
            reg |= (3<<5);
            break;
        case 16:
            reg |= (4<<5);
            break;
        case 32:
            reg |= (5<<5);
            break;
        case 64:
            reg |= (6<<5);
            break;
        case 128:
            reg |= (7<<5);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3; 
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == 0)
    {
        p_spl0601->i32kP = i32kPkT;
        hr8p506_i2c_write(HW_ADR, 0x06, 1, reg);
        if(u8OverSmpl > 8)
        {
            hr8p506_i2c_read(HW_ADR, 0x09, 1, &reg);
            hr8p506_i2c_write(HW_ADR, 0x09, 1, reg | 0x04);
        }
    }
    if(iSensor == 1)
    {
        p_spl0601->i32kT = i32kPkT;
        hr8p506_i2c_write(HW_ADR, 0x07, 1, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            hr8p506_i2c_read(HW_ADR, 0x09, 1, &reg);
            hr8p506_i2c_write(HW_ADR, 0x09, 1, reg | 0x08);
        }
    }

}

/*****************************************************************************
 �� �� ��  : spl0601_get_calib_param
 ��������  : ��ȡУ׼����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
void spl0601_get_calib_param(void)
{
	  uint8 reg;
    uint32 h=0;
    uint32 m=0;
    uint32 l=0;
	  hr8p506_i2c_read(HW_ADR, 0x10, 1, &reg);
	  h=reg;
	  hr8p506_i2c_read(HW_ADR, 0x11, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c0 = (int16)h<<4 | l>>4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
    hr8p506_i2c_read(HW_ADR, 0x11, 1, &reg);
	  h=reg;
    hr8p506_i2c_read(HW_ADR, 0x12, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c1 = (int16)(h&0x0F)<<8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
    hr8p506_i2c_read(HW_ADR, 0x13, 1, &reg);
	  h=reg;
    hr8p506_i2c_read(HW_ADR, 0x14, 1, &reg);
	  m=reg;
    hr8p506_i2c_read(HW_ADR, 0x15, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c00 = (int32)h<<12 | (int32)m<<4 | (int32)l>>4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
    hr8p506_i2c_read(HW_ADR, 0x15, 1, &reg);
	  h=reg;
    hr8p506_i2c_read(HW_ADR, 0x16, 1, &reg);
	  m=reg;
    hr8p506_i2c_read(HW_ADR, 0x17, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c10 = (int32)h<<16 | (int32)m<<8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
    hr8p506_i2c_read(HW_ADR, 0x18, 1, &reg);
	  h=reg;
    hr8p506_i2c_read(HW_ADR, 0x19, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c01 = (int16)h<<8 | l;
    hr8p506_i2c_read(HW_ADR, 0x1A, 1, &reg);
	  h=reg;
    hr8p506_i2c_read(HW_ADR, 0x1B, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c11 = (int16)h<<8 | l;
    hr8p506_i2c_read(HW_ADR, 0x1C, 1, &reg);
	  h=reg;
    hr8p506_i2c_read(HW_ADR, 0x1D, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c20 = (int16)h<<8 | l;
    hr8p506_i2c_read(HW_ADR, 0x1E, 1, &reg);
	  h=reg;
    hr8p506_i2c_read(HW_ADR, 0x1F, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c21 = (int16)h<<8 | l;
    hr8p506_i2c_read(HW_ADR, 0x20, 1, &reg);
	  h=reg;
    hr8p506_i2c_read(HW_ADR, 0x21, 1, &reg);
	  l=reg;
    p_spl0601->calib_param.c30 = (int16)h<<8 | l;
}


/*****************************************************************************
 �� �� ��  : spl0601_start_temperature
 ��������  : ����һ���¶Ȳ���
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
void spl0601_start_temperature(void)
{
    hr8p506_i2c_write(HW_ADR, 0x08, 1, 0x02);
}

/*****************************************************************************
 �� �� ��  : spl0601_start_pressure
 ��������  : ����һ��ѹ��ֵ����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
void spl0601_start_pressure(void)
{
    hr8p506_i2c_write(HW_ADR, 0x08, 1, 0x01);
}

/*****************************************************************************
 �� �� ��  : spl0601_start_continuous
 ��������  : Select node for the continuously measurement
 �������  : uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��25��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
void spl0601_start_continuous(uint8 mode)
{
    hr8p506_i2c_write(HW_ADR, 0x08, 1, mode+4);
}

//void spl0601_stop(void)
//{
//    hr8p506_i2c_write(HW_ADR, 0x08, 1, 0);
//}


/*****************************************************************************
 �� �� ��  : spl0601_get_raw_temp
 ��������  : ��ȡ�¶ȵ�ԭʼֵ����ת����32Bits����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
uint8 spl0601_get_raw_temp(void)
{
    uint8 h,m,l;
	uint8 reg[3];

	hr8p506_i2c_read(HW_ADR, 0x03, 3, reg);
	h=reg[0];
	m=reg[1];
	l=reg[2];
    p_spl0601->i32rawTemperature = (int32)h<<16 | (int32)m<<8 | (int32)l;
    p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
	return 1;
}

/*****************************************************************************
 �� �� ��  : spl0601_get_raw_pressure
 ��������  : ��ȡѹ��ԭʼֵ����ת����32bits����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
uint8 spl0601_get_raw_pressure(void)
{
    uint8 h,m,l; 
    uint8 reg[3];

		hr8p506_i2c_read(HW_ADR, 0x00, 3, reg);
		h=reg[0];
		m=reg[1];
		l=reg[2];
    p_spl0601->i32rawPressure = (int32)h<<16 | (int32)m<<8 | (int32)l;
    p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
	return 1;
}


/*****************************************************************************
 �� �� ��  : spl0601_get_temperature
 ��������  : �ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼����¶�ֵ
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 �� �� ��  : spl0601_get_pressure
 ��������  : �ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼���ѹ��ֵ
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��11��30��
    ��    ��   : WL
    �޸�����   : �����ɺ���

*****************************************************************************/
float spl0601_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}

#endif
#ifdef SPL06_01//weifang

void	ReadBaro(void)//��ȡ��ѹ����ֵ
{
	float fSplTemp;
	float fSplPressure;
	static	int	readbarocnt=0;

//	if(readbarocnt==1)
//	{
//		// sampling rate = 32Hz; Pressure oversample = 8;
//	    spl0601_rateset(PRESSURE_SENSOR,32, 8);   
//		// sampling rate = 32Hz; Temperature oversample = 8; 
//	    spl0601_rateset(TEMPERATURE_SENSOR,32, 8);
//	}
  if(readbarocnt==7)
	{
		while(!spl0601_get_raw_temp());
		while(!spl0601_get_raw_pressure());
		fSplTemp = spl0601_get_temperature();
		fSplPressure = spl0601_get_pressure();
		H_I=(int32)(fSplPressure*10);
    baro_average_pid();
		
		readbarocnt=0;
	}
	readbarocnt++;
}
#endif

void    baro_average_pid(void)    //��ѹ��ֵ��ƽ����pid����  15ms
{
	static int baro_homef;
	static int8 baro_home_count=0;
	
	H_I_average=H_I;
	
	if(baro_home_count>=0)
	{
		baro_home_count++;
		if(baro_home_count==H_I_average_time*2)
		{
			baro_home_count=-1;
			H_I_start=H_I_average;
		}
		if(baro_home_count<H_I_average_time*2)
			H_I_start=H_I_average;
	}
	baro_home=(int)((-H_I_average+H_I_start));
	//�����׶θ߶Ȳ�����
	if(hold_high_statue==HOLD_HIGH_UP)
	{
		if(baro_home<baro_homef)
			baro_home=baro_homef;
	}
	
	baro_homef=baro_home;
	if(MF_IF_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_ZERO))
	  ObservationUpdate(&baro_acc_alt,baro_home);

	//�������½��ٶ��Զ�����
	//up_down_speed=(int16)speed_calculate1(baro_acc_alt.altitude_hat,7,(__SPEED_STRUCT*)&baro_calspeed2);
	//baro_dis=(int16)speed_calculate1(H_I_average,5,(__SPEED_STRUCT*)&baro_calspeed1);
	
}
