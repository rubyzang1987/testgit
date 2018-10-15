#ifndef	__HAL_H__
#define	__HAL_H__

#include	"config.h"

extern	void	SystemHalInit(__ThisHal *hal);
extern	void	SystemHardwareInit(void);
extern	void	SystemSoftwareInit(void);
extern	void	SetLed(int no,int set);
//extern	void	UxSend(uint8 dat);
extern	void	LedFlash(void);


#endif

