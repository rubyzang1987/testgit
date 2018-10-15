#include	"flash.h"

void hw2181_flash_read(uint32 *ram_addr, uint32 flash_addr, uint32 length)
{
    uint32_t  i;
    uint32_t *ram_addr32;
    const uint32_t  *flash_addr32;

    ram_addr32 = (uint32_t *)ram_addr;
    flash_addr32 = (const uint32_t *)flash_addr;
	
    for (i = 0; i < length; i++) {
        *ram_addr32 = *flash_addr32 ;
        ram_addr32++;
        flash_addr32++;
    }
}

__asm uint8_t hw2181_iapop_words_program(uint32 address, uint32 data[], uint32 length, uint32 erase)
{
	PUSH	{R6,R7,LR}
	LDR		R6,=0x10000000
	LDR		R7,[R6]
	BLX		R7
	POP		{R6,R7,PC}
	ALIGN
}
void  erase_iappa(void)
{
	iap_enable();
	IAP->ADDR.PA = 23;
	IAP->TR=0x00005EA1;
	while(IAP->STAT.BSY);
	while(!IAP->STAT.ERASE_END);
}
void  program_iappa(uint32 data[],uint32 length)
{
//	uint8 i;
//	iap_operation_enable();
//	//erase_iappa();
	
	disable_irq();
	hw2181_iapop_words_program(NVDS_PAGE_START_ADDR,gyro_acc_data,length,1);
	enable_irq();
//	IAP->ADDR.PA = 23;
//	for(i=0;i<length;i++)
//	{
//		IAP->DATA=data[i];
//		IAP->TR=0x00005DA2;
//		while(IAP->STAT.BSY);
//		while(!IAP->STAT.PROG_END);
//	}
//	iap_operation_disable();
}
//void  asdff(void)
//{
//	gyro_acc_data[0]=0x00123456;
//}
void  gyro_acc_dataread(void)
{
	hw2181_flash_read(gyro_acc_data, NVDS_PAGE_START_ADDR, 16);
	gvst_ThisBird.AccDatReZero.pitch=gyro_acc_data[0];
	gvst_ThisBird.AccDatReZero.roll=gyro_acc_data[1];
	gvst_ThisBird.AccDatReZero.yaw=gyro_acc_data[2];
	gvst_ThisBird.GyroDatReZero.pitch=gyro_acc_data[3];
	gvst_ThisBird.GyroDatReZero.roll=gyro_acc_data[4];
	gvst_ThisBird.GyroDatReZero.yaw=gyro_acc_data[5];
	if(gyro_acc_data[6]==11)
	{
		gvst_ThisBird.AccDatReZero.pitch=gyro_acc_data[7];
		gvst_ThisBird.AccDatReZero.roll=gyro_acc_data[8];
		gvst_ThisBird.AccDatReZero.yaw=gyro_acc_data[9];
		gvst_ThisBird.GyroDatReZero.pitch=gyro_acc_data[10];
		gvst_ThisBird.GyroDatReZero.roll=gyro_acc_data[11];
		gvst_ThisBird.GyroDatReZero.yaw=gyro_acc_data[12];
//		pitch_trim=(int8)gyro_acc_data[14];
//		roll_trim=(int8)gyro_acc_data[15];
	}
	if(gyro_acc_data[13]==12)
		MF_SET_FLAG(gvu32_SystemFlag,MV_SYSTEM_FLG_GET_ZERO);
}
void  gyro_acc_datasave(void)
{
	gyro_acc_data[0]=gvst_ThisBird.AccDatReZero.pitch;
	gyro_acc_data[1]=gvst_ThisBird.AccDatReZero.roll;
	gyro_acc_data[2]=gvst_ThisBird.AccDatReZero.yaw;
	gyro_acc_data[3]=gvst_ThisBird.GyroDatReZero.pitch;
	gyro_acc_data[4]=gvst_ThisBird.GyroDatReZero.roll;
	gyro_acc_data[5]=gvst_ThisBird.GyroDatReZero.yaw;
	gyro_acc_data[6]=11;
	gyro_acc_data[7]=gvst_ThisBird.AccDatReZero.pitch;
	gyro_acc_data[8]=gvst_ThisBird.AccDatReZero.roll;
	gyro_acc_data[9]=gvst_ThisBird.AccDatReZero.yaw;
	gyro_acc_data[10]=gvst_ThisBird.GyroDatReZero.pitch;
	gyro_acc_data[11]=gvst_ThisBird.GyroDatReZero.roll;
	gyro_acc_data[12]=gvst_ThisBird.GyroDatReZero.yaw;
	gyro_acc_data[13]=12;
//	gyro_acc_data[14]=pitch_trim1+pitch_trim;
//	gyro_acc_data[15]=roll_trim1+roll_trim;
	program_iappa(gyro_acc_data,64);
}


void  iap_disable(void)
{
	IAP->UL=0x00000000;
}
void  iap_enable(void)
{
	IAP->UL=0x000000a5;
}

void   iap_operation_enable(void)
{
	IAP->UL=0x000000a5;
	IAP->CON.EN=1;
	IAP->CON.FLASH_REQ=1;
	while(!IAP->CON.FLASH_ACK);
}
void   iap_operation_disable(void)
{
	IAP->UL=0x000000a5;
	IAP->CON.FLASH_REQ=0;
	while(IAP->CON.FLASH_ACK);
	IAP->CON.EN=0;
	IAP->UL=0x00000000;
}

void  disable_irq(void)
{
	SYSTICK->CSR=0X00000000;
	NVIC->ICER = 0xffffffff;
}
void  enable_irq(void)
{
	hr8p506_interrupt_init();
}

