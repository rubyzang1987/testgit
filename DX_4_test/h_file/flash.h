#include "hr8p506.h"
#include	"config.h"

void hw2181_flash_read(uint32 *ram_addr, uint32 flash_addr, uint32 length);
void  iap_disable(void);
void  iap_enable(void);
void  gyro_acc_dataread(void);
void  gyro_acc_datasave(void);
void  erase_iappa(void);
void  program_iappa(uint32 data[],uint32 length);
__asm uint8_t hw2181_iapop_words_program(uint32 address, uint32 data[], uint32 length, uint32 erase);
void   iap_operation_enable(void);
void   iap_operation_disable(void);

