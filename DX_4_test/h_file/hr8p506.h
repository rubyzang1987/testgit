/******************************************************************************
 * @file     hr8p506.h
 * @brief    This file defines all structures and symbols for HR8P506:
 *			 - Registers and bitfields
 *			 - peripheral base address
 *			 - peripheral ID
 *			 - PIO definitions
 *           
 * @version  1.0
 * @date     2015-12-01
 *
 * @author   
 *
 * @note
 * @Copyright (C) 2015 Shanghai Eastsoft Microelectronics C0., Ltd.
 ******************************************************************************/
#ifndef __HR8P506_REG_H
#define __HR8P506_REG_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* IO definitions (access restrictions to peripheral registers) */
#ifdef __cplusplus
	#define	__I	volatile		/*!< defines 'read only' permissions*/
#else
	#define __I	volatile const 	/*!< defines 'read only' permissions*/
#endif
#define __O	volatile  			/*!< defines 'write only' permissions*/
#define __IO volatile        	/*!< defines 'read / write' permissions*/

#include <stdint.h>

/*Device Specific Peripheral Registers structures*/

//#if defined ( __CC_ARM   )
#pragma anon_unions
//#endif

/*------------- System Control Unit (SCU) ------------------------------------*/
/* System Config Protect Register */
typedef union {
	struct {
		uint32_t SCU_PROT :1;
		uint32_t          :31;
	};

	uint32_t Word;
} SCU_PROT_TypeDef;

/* NMI Select Register */
typedef union {
	struct {
		uint32_t NMIEN :1;
		uint32_t NMICS :5;
		uint32_t       :26;
	};

	uint32_t Word;
} SCU_NMICON_TypeDef;

/* Reset Register */
typedef union {
	struct {
		uint32_t PORF      :1;
		uint32_t PORRCF    :1;
		uint32_t PORRSTF   :1;
		uint32_t BORF      :1;
		uint32_t WDTRSTF   :1;
		uint32_t MRSTF     :1;
		uint32_t SOFT_RSTF :1;
		uint32_t POR_LOST  :1;
		uint32_t CFG_RST   :1;
		uint32_t           :23;
	};

	uint32_t Word;
} SCU_PWRC_TypeDef;

/* Fault Flag Register */
typedef union {
	struct {
		uint32_t FT_FLAG0 :1;
		uint32_t FT_FLAG1 :1;
		uint32_t FT_FLAG2 :1;
		uint32_t          :2;
	};

	uint32_t Word;
} SCU_FAULTFLAG_TypeDef;

/* Flash Wait Time Config Register */
typedef union {
	struct {
		uint32_t FLASH_ACCT :4;
		uint32_t            :28;
	};

	uint32_t Word;
} SCU_FLASH_WAIT_TypeDef;

/* Software Config Register */
typedef union {
	struct {
		uint32_t SOFT_BORV :4;
		uint32_t           :28;
	};

	uint32_t Word;
} SCU_SOFT_CFG_TypeDef;

/* LVD Control Register */
typedef union {
	struct {
		uint32_t EN    :1;
		uint32_t FLTEN :1;
		uint32_t       :2;
		uint32_t VS    :4;
		uint32_t IF    :1;
		uint32_t IE    :1;
		uint32_t IFS   :3;
		uint32_t       :2;
		uint32_t OUT   :1;
		uint32_t       :16;
	};

	uint32_t Word;
} SCU_LVDCON_TypeDef;

/* External Clock Check&Control Register */
typedef union {
	struct {
		uint32_t EN   :1;
		uint32_t      :3;
		uint32_t IE   :1;
		uint32_t IFS  :3;
		uint32_t IF   :1;
		uint32_t      :7;
		uint32_t FLAG :1;
		uint32_t      :15;
	};

	uint32_t Word;
} SCU_CCM_TypeDef;

/* PLL Lock Control Register */
typedef union {
	struct {
		uint32_t LK_IE   :1;
		uint32_t         :3;
		uint32_t LK_IFS  :3;
		uint32_t         :1;
		uint32_t IF      :1;
		uint32_t         :7;
		uint32_t LK_FLAG :1;
		uint32_t         :15;
	};

	uint32_t Word;
} SCU_PLL_LK_CON_TypeDef;

/* System clk Config Register */
typedef union {
	struct {
		uint32_t CLK_SEL     :2;
		uint32_t XTAL_LP     :1;
		uint32_t             :5;
		uint32_t PLL_MUX     :1;
		uint32_t             :3;
		uint32_t SYSCLK_DIV  :3;
		uint32_t             :1;
		uint32_t CLKFLT_BY   :8;
		uint32_t CLKOUT0_SEL :2;
		uint32_t CLKOUT1_SEL :2;
		uint32_t             :4;
	};

	uint32_t Word;
} SCU_CLKEN_SYS0_TypeDef;

/* System clk Config Register */
typedef union {
	struct {
		uint32_t XTAL_EN     :1;
		uint32_t HRC_EN      :1;
		uint32_t             :6;
		uint32_t PLL_PEF_SEL :3;
		uint32_t PLL_48M_SEL :1;
		uint32_t PLL_EN      :1;
		uint32_t PLL_BYLOCK  :1;
		uint32_t             :2;
		uint32_t XTAL_RDY    :1;
		uint32_t HRC_RDY     :1;
		uint32_t PLL_RDY     :1;
		uint32_t             :13;
	};

	uint32_t Word;
} SCU_CLKEN_SYS1_TypeDef;

/* Peripharal clk Config Register */
typedef union {
	struct {
		uint32_t CLKEN_SCU    :1;
		uint32_t CLKEN_GPIO   :1;
		uint32_t CLKEN_IAP    :1;
		uint32_t        	  :1;
		uint32_t CLKEN_ADC    :1;
		uint32_t CLKEN_RTC    :1;
		uint32_t CLKEN_LCD    :1;
		uint32_t CLKEN_WDT    :1;
		uint32_t CLKEN_T16N0  :1;
		uint32_t CLKEN_T16N1  :1;
		uint32_t CLKEN_T16N2  :1;
		uint32_t CLKEN_T16N3  :1;
		uint32_t CLKEN_T32N0  :1;
		uint32_t              :3;
		uint32_t CLKEN_UATR0  :1;
		uint32_t CLKEN_UATR1  :1;
		uint32_t              :2;
		uint32_t CLKEN_EUATR0 :1;
		uint32_t              :3;
		uint32_t CLKEN_SPI0   :1;
		uint32_t CLKEN_SPI1   :1;
		uint32_t              :2;
		uint32_t CLKEN_I2C    :1;
		uint32_t              :3;
	};

	uint32_t Word;
} SCU_CLKEN_PERI_TypeDef;

/* Wakeup Time Control Register */
typedef union {
	struct {
		uint32_t WAKEUP_TIME :12;
		uint32_t MOSC_PD     :1;
		uint32_t CLKFLT_EN   :1;
		uint32_t             :2;
		uint32_t FLASHPW_PD  :1;
		uint32_t             :15;
	};

	uint32_t Word;
} SCU_WAKEUP_TIME_TypeDef;	

/* Int Vector remap enable Register */
typedef union {
	struct {
		uint32_t TBLREMAPEN :1;
		uint32_t            :31;
	};

	uint32_t Word;
} SCU_TBLREMAPEN_TypeDef;

/* Int Vector Offset Register */
typedef union {
	struct {
		uint32_t RSV    :8;
		uint32_t TBLOFF :24;
	};

	uint32_t Word;
} SCU_TBLOFF_TypeDef;

/* System Calculation Protect Register */
typedef union {
	struct {
		uint32_t CAL_PROT :1;
		uint32_t          :31;
	};

	uint32_t Word;
} SCU_CAL_PROT_TypeDef;

/* Test Control Register */
typedef union {
	struct {
		uint32_t TEST_SEL      :3;
		uint32_t               :1;
		uint32_t PLL_TEST      :1;
		uint32_t ADC_TBUF_EN   :1;
		uint32_t ADC_TBUF_PAD  :1;
		uint32_t ADC_TBUF_FLIP :1;
		uint32_t ADC_TVP_SEL   :1;
		uint32_t               :3;
		uint32_t TMUX_SEL      :4;
		uint32_t TO_PAD_EN     :1;
		uint32_t TO_PAD2_EN    :1;
		uint32_t LDO_TEST_EN   :1;
		uint32_t VR_TEST_EN    :1;
		uint32_t PREC_INTOSC   :2;
		uint32_t IOSC_CAL_DONE :1;
		uint32_t IOSC_FIN_DONE :1;
		uint32_t PREC_WDT      :2;
		uint32_t WDT_CAL_DONE  :1;
		uint32_t WDT_FIN_DONE  :1;
		uint32_t               :4;
	};

	uint32_t Word;
} SCU_TESTCON_TypeDef;

/* VR5 Calculation Register */
typedef union {
	struct {
		uint32_t VR05CAI :9;
		uint32_t         :3;
		uint32_t VBECAL  :3;
		uint32_t         :17;
	};

	uint32_t Word;
} SCU_VR5CAL_TypeDef;

/* LDO Calculation Register */
typedef union {
	struct {
		uint32_t LDO_LS_CAL :4;
		uint32_t LDO_HS_CAL :4;
		uint32_t IBIAS_CAL  :5;
		uint32_t            :19;
	};

	uint32_t Word;
} SCU_LDOCAL_TypeDef;

/* LDO Bias Register */
typedef union {
	struct {
		uint32_t LDOLP_VBIAS_SEL   :3;
		uint32_t                   :1;
		uint32_t LDOLP_IBIAS_SEL   :2;
		uint32_t LDOLP_COMPCAP_SEL :2;
		uint32_t LDOHP_VBIAS_SEL   :3;
		uint32_t                   :1;
		uint32_t LDOHP_IBIAS_SEL   :2;
		uint32_t LDOHP_COMPCAP_SEL :2;
		uint32_t                   :16;
	};

	uint32_t Word;
} SCU_LDOBIAS_TypeDef;

/* Internal Osc Calculation Register */
typedef union {
	struct {
		uint32_t INOSCCAL  :10;
		uint32_t INOSCCAL1 :2;
		uint32_t           :20;
	};

	uint32_t Word;
} SCU_INOSCCAL_TypeDef;

/* External Osc Calculation Register */
typedef union {
	struct {
		uint32_t OSC_VSELCAL :4;
		uint32_t             :28;
	};

	uint32_t Word;
} SCU_MOSCCAL_TypeDef;

/* WDT Clock Calculation Register */
typedef union {
	struct {
		uint32_t WDTCAL :8;
		uint32_t        :24;
	};

	uint32_t Word;
} SCU_WDTCAL_TypeDef;

/* PLL Calculation Register */
typedef union {
	struct {
		uint32_t PLL_CPI      :4;
		uint32_t PLL_KVCOI    :3;
		uint32_t              :1;
		uint32_t PLL_VCOI     :3;
		uint32_t              :1;
		uint32_t CAL_PLL_I    :3;
		uint32_t              :1;
		uint32_t PLL_LOCKSEL  :2;
		uint32_t PLL_VCOD2SHP :1;
		uint32_t              :13;
	};

	uint32_t Word;
} SCU_PLLCAL_TypeDef;

/* VBG Reference Voltage Calculation Register */
typedef union {
	struct {
		uint32_t VREF_BG_CAL :10;
		uint32_t             :22;
	};

	uint32_t Word;
} SCU_VREF_BG_CAL_TypeDef;

/* VR26 Reference Voltage Calculation Register */
typedef union {
	struct {
		uint32_t VR26_CAL  :9;
		uint32_t           :3;
		uint32_t VTEMP_CAL :5;
		uint32_t           :15;
	};

	uint32_t Word;
} SCU_VREF_VR26_CAL_TypeDef;

typedef struct {
	__IO SCU_PROT_TypeDef          SCU_PROT;  				/*!< Offset: 0x000 (R/W)  System Config Protect Register */
	__IO SCU_NMICON_TypeDef        NMICON;                 	/*!< Offset: 0x004 (R/W)  NMI Select Register */
	__IO SCU_PWRC_TypeDef          PWRC;                   	/*!< Offset: 0x008 (R/W)  Reset Register */
	__IO SCU_FAULTFLAG_TypeDef     FAULTFLAG;              	/*!< Offset: 0x00C (R/W)  Fault Flag Register */
    uint32_t                       RSV0;
	uint32_t                       RSV1;
    uint32_t                       RSV2;
    uint32_t                       RSV3;
  	__IO SCU_FLASH_WAIT_TypeDef    FLASH_WAIT;             	/*!< Offset: 0x020 (R/W)  Flash Wait Time Config Register */
  	__IO SCU_SOFT_CFG_TypeDef      SOFT_CFG;               	/*!< Offset: 0x024 (R/W)  Software Config Register */
  	__IO SCU_LVDCON_TypeDef        LVDCON;                 	/*!< Offset: 0x028 (R/W)  LVD Control Register */
  	__IO SCU_CCM_TypeDef           CCM;                    	/*!< Offset: 0x02C (R/W)  External Clock Check&Control Register */
  	__IO SCU_PLL_LK_CON_TypeDef    PLL_LK_CON;             	/*!< Offset: 0x030 (R/W)  PLL Lock Control Register */
  	__IO uint32_t                  TIMER_EN;               	/*!< Offset: 0x034 (R/W)  Timer enable Register */
  	__IO uint32_t                  TIMER_DIS;              	/*!< Offset: 0x038 (R/W)  Timer disable Register */
    uint32_t                       RSV4;
  	__IO SCU_CLKEN_SYS0_TypeDef    CLKEN_SYS0;             	/*!< Offset: 0x040 (R/W)  System clk Config Register0 */
  	__IO SCU_CLKEN_SYS1_TypeDef    CLKEN_SYS1;             	/*!< Offset: 0x044 (R/W)  System clk Config Register1 */
  	__IO SCU_CLKEN_PERI_TypeDef    CLKEN_PERI;             	/*!< Offset: 0x048 (R/W)  Peripharal clk Config Register */
  	__IO SCU_WAKEUP_TIME_TypeDef   WAKEUP_TIME;            	/*!< Offset: 0x04C (R/W)  Wakeup Time Control Register */
    uint32_t                       RSV5;
    uint32_t                       RSV6;
    uint32_t                       RSV7;
    uint32_t                       RSV8;
  	__IO SCU_TBLREMAPEN_TypeDef    TBLREMAPEN;             	/*!< Offset: 0x060 (R/W)  Int Vector remap enable Register */
  	__IO SCU_TBLOFF_TypeDef        TBLOFF;                 	/*!< Offset: 0x064 (R/W)  Int Vector Offset Register */
    uint32_t                       RSV9;
    uint32_t                       RSV10;
    uint32_t                       RSV11;
    uint32_t                       RSV12;
    uint32_t                       RSV13;
    uint32_t                       RSV14;
	__IO SCU_CAL_PROT_TypeDef      CAL_PROT;               	/*!< Offset: 0x080 (R/W)  System Calculation Protect Register */
	__IO SCU_TESTCON_TypeDef       TESTCON;                	/*!< Offset: 0x084 (R/W)  Test Control Register */
	__IO SCU_VR5CAL_TypeDef        VR5CAL;                 	/*!< Offset: 0x088 (R/W)  VR5 Calculation Register */
	__IO SCU_LDOCAL_TypeDef        LDOCAL;                 	/*!< Offset: 0x08C (R/W)  LDO Calculation Register */
	__IO SCU_LDOBIAS_TypeDef       LDOBIAS;                	/*!< Offset: 0x090 (R/W)  LDO Bias Register */
	__IO SCU_INOSCCAL_TypeDef      INOSCCAL;               	/*!< Offset: 0x094 (R/W)  Internal Osc Calculation Register */
	__IO SCU_MOSCCAL_TypeDef       MOSCCAL;                	/*!< Offset: 0x098 (R/W)  External Osc Calculation Register */
	__IO SCU_WDTCAL_TypeDef        WDTCAL;                 	/*!< Offset: 0x09C (R/W)  WDT Clock Calculation Register */
	__IO SCU_PLLCAL_TypeDef        PLLCAL;                 	/*!< Offset: 0x0A0 (R/W)  PLL Calculation Register */
	__IO SCU_VREF_BG_CAL_TypeDef   VREF_BG_CAL;            	/*!< Offset: 0x0A4 (R/W)  VBG Reference Voltage Calculation Register */
	__IO SCU_VREF_VR26_CAL_TypeDef VREF_VR26_CAL;           /*!< Offset: 0x0A8 (R/W)  VR26 Reference Voltage Calculation Register */
} HR8P506_SCU_TypeDef;
/*@}*/ /* end of group HR8P506_SCU */


/*------------- System Control Block (SCB) --------------------------------------------------*/
/** @addtogroup HR8P506_SCB System Control Block */
typedef struct {
	__I  uint32_t CPUID;               	/*!< Offset: 0x000 (R/ )  CPUID Register */
	__IO uint32_t ICSR;                 /*!< Offset: 0x004 (R/W)  Interrupt Control and Status Register */
	uint32_t      RESERVED0;
	__IO uint32_t AIRCR;                /*!< Offset: 0x00C (R/W)  Applicational Interrupt and Reset Control Register */
	__I  uint32_t SCR;                 	/*!< Offset: 0x010 (R/ )  System Control Register */
	__I  uint32_t CCR;              	/*!< Offset: 0x014 (R/ )  Config and Control Register */
	uint32_t      RESERVED1;
	__I  uint32_t SHPR2;	            /*!< Offset: 0x01C (R/ )  System Handler Priority Register 2*/
	__I  uint32_t SHPR3;	            /*!< Offset: 0x020 (R/ )  System Handler Priority Register 3*/
} HR8P506_SCB_TypeDef;
/*@}*/ /* end of group HR8P506_SCB */


/*------------- General Purpose Input and Output (GPIO) --------------------------*/
/** @addtogroup HR8P506_GPIO HR8P506 GPIO Unit*/
typedef struct {
	__IO uint32_t PA_PORT;                   	/*!< Offset: 0x000 (R/W)  GPIO Port Status Register */
	__IO uint32_t PA_DATA;                   	/*!< Offset: 0x004 (R/W)  GPIO Data Register */
	__IO uint32_t PA_DATABSR;                	/*!< Offset: 0x008 (R/W)  GPIO Output Bit-wise Set Register */
	__IO uint32_t PA_DATABCR;                	/*!< Offset: 0x00C (R/W)  GPIO Output Bit-wise Clear Register */
	__IO uint32_t PA_DATABRR;                	/*!< Offset: 0x010 (R/W)  GPIO Output Bit-wise Reverse Register */
	__IO uint32_t PA_DIR;                    	/*!< Offset: 0x014 (R/W)  GPIO I/O Direction Control Register */
	__IO uint32_t PA_DIRBSR;                 	/*!< Offset: 0x018 (R/W)  GPIO I/O Direction Bit-wise Set Register */
	__IO uint32_t PA_DIRBCR;                 	/*!< Offset: 0x01C (R/W)  GPIO I/O Direction Bit-wise Clear Register */
	__IO uint32_t PA_DIRBRR;                 	/*!< Offset: 0x020 (R/W)  GPIO I/O Direction Bit-wise Reverse Register */
	__IO uint32_t PA_FUNC0;                  	/*!< Offset: 0x024 (R/W)  GPIO Function Select Register 0 */
	__IO uint32_t PA_FUNC1;                  	/*!< Offset: 0x028 (R/W)  GPIO Function Select Register 1 */
	__IO uint32_t PA_FUNC2;                  	/*!< Offset: 0x02C (R/W)  GPIO Function Select Register 2 */
	__IO uint32_t PA_FUNC3;                  	/*!< Offset: 0x030 (R/W)  GPIO Function Select Register 3 */
	__IO uint32_t PA_INEB;               		/*!< Offset: 0x034 (R/W)  GPIO Input Control Register */
	__IO uint32_t PA_ODE;                		/*!< Offset: 0x038 (R/W)  GPIO Open-Drain Control Register */
	__IO uint32_t PA_PUE;                		/*!< Offset: 0x03C (R/W)  GPIO Pull-Up Control Register */
	__IO uint32_t PA_PDE;                		/*!< Offset: 0x040 (R/W)  GPIO Pull-Down Control Register */
	__IO uint32_t PA_DS;                 		/*!< Offset: 0x044 (R/W)  GPIO Drive Strength Control Register */
	uint32_t 	  RESERVED0[14];
	__IO uint32_t PB_PORT;                   	/*!< Offset: 0x000 (R/W)  GPIO Port Status Register */
	__IO uint32_t PB_DATA;                   	/*!< Offset: 0x004 (R/W)  GPIO Data Register */
	__IO uint32_t PB_DATABSR;                	/*!< Offset: 0x008 (R/W)  GPIO Output Bit-wise Set Register */
	__IO uint32_t PB_DATABCR;                	/*!< Offset: 0x00C (R/W)  GPIO Output Bit-wise Clear Register */
	__IO uint32_t PB_DATABRR;                	/*!< Offset: 0x010 (R/W)  GPIO Output Bit-wise Reverse Register */
	__IO uint32_t PB_DIR;                    	/*!< Offset: 0x014 (R/W)  GPIO I/O Direction Control Register */
	__IO uint32_t PB_DIRBSR;                 	/*!< Offset: 0x018 (R/W)  GPIO I/O Direction Bit-wise Set Register */
	__IO uint32_t PB_DIRBCR;                 	/*!< Offset: 0x01C (R/W)  GPIO I/O Direction Bit-wise Clear Register */
	__IO uint32_t PB_DIRBRR;                 	/*!< Offset: 0x020 (R/W)  GPIO I/O Direction Bit-wise Reverse Register */
	__IO uint32_t PB_FUNC0;                  	/*!< Offset: 0x024 (R/W)  GPIO Function Select Register 0 */
	__IO uint32_t PB_FUNC1;                  	/*!< Offset: 0x028 (R/W)  GPIO Function Select Register 1 */
	uint32_t 	  RESERVED1[2];
	__IO uint32_t PB_INEB;               		/*!< Offset: 0x034 (R/W)  GPIO Input Control Register */
	__IO uint32_t PB_ODE;                		/*!< Offset: 0x038 (R/W)  GPIO Open-Drain Control Register */
	__IO uint32_t PB_PUE;                		/*!< Offset: 0x03C (R/W)  GPIO Pull-Up Control Register */
	__IO uint32_t PB_PDE;                		/*!< Offset: 0x040 (R/W)  GPIO Pull-Down Control Register */
	__IO uint32_t PB_DS;                 		/*!< Offset: 0x044 (R/W)  GPIO Drive Strength Control Register */
} HR8P506_GPIO_TypeDef;
/*@}*/ /* end of group HR8P506_GPIO */

/*------------- General Purpose Input and Output Extensions (GPIOE) --------------------------*/
/** @addtogroup HR8P506_GPIO HR8P506 GPIO Unit Extension Registers*/
typedef struct {
	__IO uint32_t PINTIE;                 	/*!< Offset: 0x000 (R/W)  PINT Interrupt Enable Register */
	__IO uint32_t PINTIF;                 	/*!< Offset: 0x004 (R/W)  PINT Interrupt Flag Register */
	__IO uint32_t PINTSEL;                	/*!< Offset: 0x008 (R/W)  PINT Interrupt Source Select Register */
	__IO uint32_t PINTCFG;                	/*!< Offset: 0x00C (R/W)  PINT Interrupt Config Register */
	__IO uint32_t KINTIE;                 	/*!< Offset: 0x010 (R/W)  KINT Interrupt Enable Register */
	__IO uint32_t KINTIF;                 	/*!< Offset: 0x014 (R/W)  KINT Interrupt Flag Register */
	__IO uint32_t KINTSEL;                	/*!< Offset: 0x018 (R/W)  KINT Interrupt Source Select Register */
	__IO uint32_t KINTCFG;                	/*!< Offset: 0x01C (R/W)  KINT Interrupt Config Register */
	uint32_t      RESERVED0[4];
	__IO uint32_t IOINT_FLT_S;            	/*!< Offset: 0x030 (R/W)  20ns Filter Config Register */
	uint32_t      RESERVED1[3];
	__IO uint32_t TMR_FLT_SEL;            	/*!< Offset: 0x040 (R/W)  Timer 20ns Filter Config Register */
	__IO uint32_t SPI_FLT_SEL;            	/*!< Offset: 0x044 (R/W)  SPI 20ns Filter Config Register */
	uint32_t      RESERVED2[14];
	__IO uint32_t GPIO_TXPWM;             	/*!< Offset: 0x080 (R/W)  PWM Register */
	__IO uint32_t GPIO_BUZC;              	/*!< Offset: 0x084 (R/W)  Buzzer Control Register */
} HR8P506_GPIOE_TypeDef;
/*@}*/ /* end of group HR8P506_GPIOE */

/*------------- In Application Programmer (IAP) --------------------------------------------------*/
/** @addtogroup HR8P506_IAP In Application Programmer*/
/* IAP Control Register */
typedef union {
	struct {
		uint32_t EN         :1;
		uint32_t RST        :1;
		uint32_t            :2;
		uint32_t FLASH_REQ  :1;
		uint32_t FLASH_ACK  :1;
		uint32_t            :1;
		uint32_t FLASH_FAIL :1;
		uint32_t            :24;
	};

	uint32_t Word;
} IAP_CON_TypeDef;

/* IAP Address Register */
typedef union {
	struct {
		uint32_t       :2;
		uint32_t CA    :8;
		uint32_t PA    :6;
		uint32_t       :4;
		uint32_t IFREN :1;
		uint32_t       :11;
	};

	uint32_t Word;
} IAP_ADDR_TypeDef;

/* IAP Status Register */
typedef union {
	struct {
		uint32_t BSY         :1;
		uint32_t ERASE_END   :1;
		uint32_t PROG_END    :1;
		uint32_t TIMEOUT_ERR :1;
		uint32_t             :28;
	};

	uint32_t Word;
} IAP_STAT_TypeDef;

typedef struct {
	__IO IAP_CON_TypeDef CON;       	/*!< Offset: 0x000 (R/W)  IAP Control Register */
	__IO IAP_ADDR_TypeDef ADDR;      	/*!< Offset: 0x004 (R/W)  IAP Address Register */
	__IO uint32_t         DATA;      	/*!< Offset: 0x008 (R/W)  IAP Data Register */
	__IO uint32_t         TR;        	/*!< Offset: 0x00C (R/W)  IAP Trigger Register */
	__IO uint32_t         UL;           /*!< Offset: 0x010 (R/W)  IAP Unlock Register */
	__IO IAP_STAT_TypeDef STAT;         /*!< Offset: 0x014 (R/W)  IAP Status Register */
} HR8P506_IAP_TypeDef;
/*@}*/ /* end of group HR8P506_IAP */

/*------------- Analog Digital Converter (ADC) --------------------------------------------------*/
/** @addtogroup HR8P506_ADC Analog Digital Converter*/

/* Conversion Data Register */
typedef union {
	struct {
		uint32_t DR :16;
		uint32_t    :16;
	};

	uint32_t Word;
} ADC_DR_TypeDef;

/* Control Register 0 */
typedef union {
	struct {
		uint32_t EN     :1;
		uint32_t TRG    :1;
		uint32_t ACP_EN :1;
		uint32_t        :29;
	};

	uint32_t Word;
} ADC_CON0_TypeDef;

/* Control Register 1 */
typedef union {
	struct {
		uint32_t CLKDIV    :3;
		uint32_t CLKS      :1;
		uint32_t           :4;
		uint32_t VREFP     :2;
		uint32_t VREFN     :1;
		uint32_t VRBUF_EN  :1;
		uint32_t SMPS      :1;
		uint32_t SMPON     :1;
		uint32_t HSEN      :1;
		uint32_t           :1;
		uint32_t ST        :5;
		uint32_t           :3;
		uint32_t VCMBUF_EN :1;
		uint32_t VCMBUF_HS :1;
		uint32_t           :6;
	};

	uint32_t Word;
} ADC_CON1_TypeDef;

/* Channel Select Register */
typedef union {
	struct {
		uint32_t CHS          :5;
		uint32_t              :3;
		uint32_t VDD5_FLAG_EN :1;
		uint32_t              :23;
	};

	uint32_t Word;
} ADC_CHS_TypeDef;

/* Interrupt Enable Register */
typedef union {
	struct {
		uint32_t IE       :1;
		uint32_t ACPMINIE :1;
		uint32_t ACPMAXIE :1;
		uint32_t ACPOVIE  :1;
		uint32_t          :28;
	};

	uint32_t Word;
} ADC_IE_TypeDef;

/* interrupt Flag Register */
typedef union {
	struct {
		uint32_t IF       :1;
		uint32_t ACPMINIF :1;
		uint32_t ACPMAXIF :1;
		uint32_t ACPOVIF  :1;
		uint32_t          :28;
	};

	uint32_t Word;
} ADC_IF_TypeDef;

/* Auto-Conversion Control Register */
typedef union {
	struct {
		uint32_t ACP_OVFL_TIME :12;
		uint32_t               :4;
		uint32_t ACP_TIMES     :2;
		uint32_t               :2;
		uint32_t ACP_CLKS      :1;
		uint32_t               :11;
	
	};

	uint32_t Word;
} ADC_ACPC0_TypeDef;

/* Auto-Conversion Comparison Register */
typedef union {
	struct {
		uint32_t ACP_CMP_MIN :12;
		uint32_t             :4;
		uint32_t ACP_CMP_MAX :12;
		uint32_t             :4;
	};

	uint32_t Word;
} ADC_ACPCMP_TypeDef;

/* Auto-Conversion Mean Value Register */
typedef union {
	struct {
		uint32_t ACP_MEAN_DATA :12;
		uint32_t               :20;
	};

	uint32_t Word;
} ADC_ACPMEAN_TypeDef;

/* Reference Voltage Control Register */
typedef union {
	struct {
		uint32_t VREF_EN       :1;
		uint32_t VREF_SEL      :1;
		uint32_t IREF_EN       :1;
		uint32_t CHOP_EN       :1;
		uint32_t CHOP_EN1      :1;
		uint32_t               :3;
		uint32_t CHOP_CLK_SEL  :3;
		uint32_t               :1;
		uint32_t FILTERCLK_SEL :2;
		uint32_t               :2;
		uint32_t CHOP_CLK_DIV  :3;
		uint32_t               :13;
	};

	uint32_t Word;
} ADC_VREF_CON_TypeDef;

typedef struct {
	__IO ADC_DR_TypeDef       DR;                     /*!< Offset: 0x000 (R/W)  ADC Conversion Data Register */
	__IO ADC_CON0_TypeDef     CON0;                   /*!< Offset: 0x004 (R/W)  ADC Control Register 0 */
	__IO ADC_CON1_TypeDef     CON1;                   /*!< Offset: 0x008 (R/W)  ADC Control Register 1 */
	__IO ADC_CHS_TypeDef      CHS;                    /*!< Offset: 0x00C (R/W)  ADC Channel Select Register */
	__IO ADC_IE_TypeDef       IE;                     /*!< Offset: 0x010 (R/W)  ADC Interrupt Enable Register */
	__IO ADC_IF_TypeDef       IF;                     /*!< Offset: 0x014 (R/W)  ADC Interrupt Flag Register */
	uint32_t                  RSV0[4];
	__IO ADC_ACPC0_TypeDef    ACPC0;                  /*!< Offset: 0x028 (R/W)  ADC Auto-Conversion Control Register 0 */
	uint32_t                  RSV1;
	__IO ADC_ACPCMP_TypeDef   ACPCMP;                 /*!< Offset: 0x030 (R/W)  ADC Auto-Conversion Comparison Register */
	__IO ADC_ACPMEAN_TypeDef  ACPMEAN;                /*!< Offset: 0x034 (R/W)  ADC Auto-Conversion Mean Value Register */
	uint32_t                  RSV2[2];
	__IO ADC_VREF_CON_TypeDef VREF_CON;               /*!< Offset: 0x040 (R/W)  ADC Reference Voltage Control Register */
} HR8P506_ADC_TypeDef;
/*@}*/ /* end of group HR8P506_ADC */

/*------------- Real Time Clock (RTC) --------------------------------------------------*/
/** @addtogroup HR8P506_RTC Real Time Clock*/
typedef struct {
	__IO uint32_t CON;                   	/*!< Offset: 0x000 (R/W)  RTC Control Register */
	uint32_t      RESERVED0[3];
	__IO uint32_t CAL;                    	/*!< Offset: 0x010 (R/W)  RTC Calculation Register */
	uint32_t      RESERVED1[3];
	__IO uint32_t WA;                     	/*!< Offset: 0x020 (R/W)  RTC Weekly Alarm Register */
	uint32_t      RESERVED2[3];
	__IO uint32_t DA;                     	/*!< Offset: 0x030 (R/W)  RTC Daily Alarm Register */
	uint32_t      RESERVED3[3];
	__IO uint32_t HMS;                    	/*!< Offset: 0x040 (R/W)  RTC Hour Minute Second Register */
	uint32_t      RESERVED4[3];
	__IO uint32_t YMDW;                   	/*!< Offset: 0x050 (R/W)  RTC Year Month Day Week Register */
	uint32_t      RESERVED5[3];
	__IO uint32_t IE;                     	/*!< Offset: 0x060 (R/W)  RTC Interrupt Enable Register */
	uint32_t      RESERVED6[3];
	__IO uint32_t IF;                     	/*!< Offset: 0x070 (R/W)  RTC Interrupt Enable Register */
	uint32_t      RESERVED7[3];
	__IO uint32_t WP;                     	/*!< Offset: 0x080 (R/W)  RTC Write Protect Register */
} HR8P506_RTC_TypeDef;
/*@}*/ /* end of group HR8P506_RTC */

/*------------- LCD/LED --------------------------------------------------*/
/** @addtogroup HR8P506_LCD LCD/LED*/
typedef struct {
	__IO uint32_t CON;                    	/*!< Offset: 0x000 (R/W)  LCD Control Register */
	uint32_t      RESERVED0[3];
	__IO uint32_t TIME;                   	/*!< Offset: 0x010 (R/W)  LCD Flash Time Register */
	uint32_t      RESERVED1[3];
	__IO uint32_t SEL;                    	/*!< Offset: 0x020 (R/W)  LCD Segment Select Register */
	uint32_t      RESERVED2[7];
	__IO uint32_t SYSCFG;                 	/*!< Offset: 0x040 (R/W)  LCD System Config Register */
	uint32_t      RESERVED3[15];
	__IO uint32_t D0;                     	/*!< Offset: 0x080 (R/W)  LCD Pixel Register 0 */
	uint32_t      RESERVED4[3];
	__IO uint32_t D1;                     	/*!< Offset: 0x090 (R/W)  LCD Pixel Register 1 */
	uint32_t      RESERVED5[3];
	__IO uint32_t D2;                     	/*!< Offset: 0x0A0 (R/W)  LCD Pixel Register 2 */
	uint32_t      RESERVED6[3];
	__IO uint32_t D3;                     	/*!< Offset: 0x0B0 (R/W)  LCD Pixel Register 3 */
	uint32_t      RESERVED7[3];
	__IO uint32_t D4;                     	/*!< Offset: 0x0C0 (R/W)  LCD Pixel Register 4 */
	uint32_t      RESERVED8[3];
	__IO uint32_t D5;                     	/*!< Offset: 0x0D0 (R/W)  LCD Pixel Register 5 */
	uint32_t      RESERVED9[3];
	__IO uint32_t D6;                     	/*!< Offset: 0x0E0 (R/W)  LCD Pixel Register 6 */
} HR8P506_LCD_TypeDef;
/*@}*/ /* end of group HR8P506_LCD */

/*------------- Watchdog Timer (WDT) --------------------------------------------------*/
/** @addtogroup HR8P506_WDT Watchdog Timer*/
typedef struct {
	__IO uint32_t LOAD;                   	/*!< Offset: 0x000 (R/W)  WDT Counter Load Value Register */
	__IO uint32_t VALUE;                  	/*!< Offset: 0x004 (R/W)  WDT Counter Current Value Register */
	__IO uint32_t CON;                    	/*!< Offset: 0x008 (R/W)  WDT Control Register */
	__O  uint32_t INTCLR;                 	/*!< Offset: 0x00C ( /W)  WDT Interrupt Flag Clear Register */
	__I  uint32_t RIS;                    	/*!< Offset: 0x010 (R/ )  WDT Interrupt Flag Register */
	uint32_t      RESERVED0[55];
	__IO uint32_t LOCK;                   	/*!< Offset: 0x100 (R/W)  WDT Access Enable Register */
	uint32_t      RESERVED1[127];
	__IO uint32_t ITCR;                   	/*!< Offset: 0x300 (R/W)  WDT Test Control Register */
	__O  uint32_t ITOP;                   	/*!< Offset: 0x304 ( /W)  WDT Test Control Register */
} HR8P506_WDT_TypeDef;
/*@}*/ /* end of group HR8P506_WDT */


/*------------- Timer (TMR) --------------------------------------------------*/
/** @addtogroup HR8P506_T16N 16-bit Counter/Timer*/
typedef struct {
	__IO uint32_t CNT0;                   	/*!< Offset: 0x000 (R/W)  Timer Counter Register 0 */
	__IO uint32_t CNT1;                   	/*!< Offset: 0x004 (R/W)  Timer Counter Register 0 */
	__IO uint32_t PRECNT;                 	/*!< Offset: 0x008 (R/W)  Prescale Counter Register */
	__IO uint32_t PREMAT;                 	/*!< Offset: 0x00C (R/W)  Prescale Counter Match Register */
	__IO uint32_t CON0;                   	/*!< Offset: 0x010 (R/W)  Timer Control Register 0 */
	__IO uint32_t CON1;                   	/*!< Offset: 0x014 (R/W)  Timer Control Register 1 */
	__IO uint32_t CON2;                   	/*!< Offset: 0x018 (R/W)  Timer Control Register 2 */
	uint32_t      RESERVED0;
	__IO uint32_t IE;                     	/*!< Offset: 0x020 (R/W)  Interrupt Enable Register */
	__IO uint32_t IF;                     	/*!< Offset: 0x024 (R/W)  Interrupt Flag Register */
	__IO uint32_t PDZ;                    	/*!< Offset: 0x028 (R/W)  PWM Mode Dead Zone Config Register */
	__IO uint32_t PTR;                    	/*!< Offset: 0x02C (R/W)  PWM Mode ADC Trigger Register */
	__IO uint32_t MAT0;                   	/*!< Offset: 0x030 (R/W)  Counter Match Register 0 */
	__IO uint32_t MAT1;                   	/*!< Offset: 0x034 (R/W)  Counter Match Register 1 */
	__IO uint32_t MAT2;                   	/*!< Offset: 0x038 (R/W)  Counter Match Register 2 */
	__IO uint32_t MAT3;                   	/*!< Offset: 0x03C (R/W)  Counter Match Register 3 */
	__IO uint32_t TOP0;                   	/*!< Offset: 0x040 (R/W)  Peak Value Register 0 */
	__IO uint32_t TOP1;                   	/*!< Offset: 0x044 (R/W)  Peak Value Register 1 */
} HR8P506_T16N_TypeDef;
/*@}*/ /* end of group HR8P506_T16N */


/*------------- Timer (TMR) --------------------------------------------------*/
/** @addtogroup HR8P506_T32N 32-bit Counter/Timer*/
typedef struct {
	__IO uint32_t CNT;                    	/*!< Offset: 0x000 (R/W)  Timer Counter Register */
	__IO uint32_t CON0;                   	/*!< Offset: 0x004 (R/W)  Timer Control Register 0 */
	__IO uint32_t CON1;                   	/*!< Offset: 0x008 (R/W)  Timer Control Register 1 */
	uint32_t      RESERVED0;
	__IO uint32_t PRECNT;                 	/*!< Offset: 0x010 (R/W)  Prescale Counter Register */
	__IO uint32_t PREMAT;                 	/*!< Offset: 0x014 (R/W)  Prescale Counter Match Register */
	__IO uint32_t IE;                     	/*!< Offset: 0x018 (R/W)  Interrupt Enable Register */
	__IO uint32_t IF;                     	/*!< Offset: 0x01C (R/W)  Interrupt Flag Register */
	__IO uint32_t MAT0;                   	/*!< Offset: 0x020 (R/W)  Counter Match Register 0 */
	__IO uint32_t MAT1;                   	/*!< Offset: 0x024 (R/W)  Counter Match Register 1 */
	__IO uint32_t MAT2;                   	/*!< Offset: 0x028 (R/W)  Counter Match Register 2 */
	__IO uint32_t MAT3;                   	/*!< Offset: 0x02C (R/W)  Counter Match Register 3 */
} HR8P506_T32N_TypeDef;
/*@}*/ /* end of group HR8P506_T32N */


/*------------- Universal Asynchronous Receiver Transmitter (UART) -----------*/
/** @addtogroup HR8P506_UART HR8P506 Universal Asynchronous Receiver/Transmitter*/
/* Control Register 0 */
typedef union {
	struct {
		uint32_t TXEN  :1;
		uint32_t TRST  :1;
		uint32_t TBCLR :1;
		uint32_t TXI   :1;
		uint32_t       :4;
		uint32_t TXMOD :4;
		uint32_t TXP   :1;
		uint32_t TXFS  :1;
		uint32_t       :2;
		uint32_t RXEN  :1;
		uint32_t RRST  :1;
		uint32_t RBCLR :1;
		uint32_t RXI   :1;
		uint32_t BDEN  :1;
		uint32_t IDEN  :1;
		uint32_t       :2;
		uint32_t RXMOD :4;
		uint32_t RXP   :1;
		uint32_t       :3;
	};

	uint32_t Word;
} UART_CON0_TypeDef;

/* Control Register 1 */
typedef union {
	struct {
		uint32_t TBIM :2;
		uint32_t      :2;
		uint32_t RBIM :2;
		uint32_t      :2;
		uint32_t BCS  :3;
		uint32_t      :1;
		uint32_t BDM  :2;
		uint32_t      :2;
		uint32_t IDM  :2;
		uint32_t      :14;
	};

	uint32_t Word;
} UART_CON1_TypeDef;

/* Baudrate Config Register */
typedef union {
	struct {
		uint32_t BRFRA :4;
		uint32_t BRINT :12;
		uint32_t       :16;
	};

	uint32_t Word;
} UART_BRR_TypeDef;

/* Status Register */
typedef union {
	struct {
		uint32_t TBPTR  :4;
		uint32_t TBOV   :1;
		uint32_t TXBUSY :1;
		uint32_t        :2;
		uint32_t RBPTR  :4;
		uint32_t RBOV   :1;
		uint32_t RXBUSY :1;
		uint32_t        :2;
		uint32_t FER0   :1;
		uint32_t PER0   :1;
		uint32_t FER1   :1;
		uint32_t PER1   :1;
		uint32_t FER2   :1;
		uint32_t PER2   :1;
		uint32_t FER3   :1;
		uint32_t PER3   :1;
		uint32_t        :8;
	};

	uint32_t Word;
} UART_STA_TypeDef;

/* Interrupt Enable Register */
typedef union {
	struct {
		uint32_t TB   :1;
		uint32_t TC   :1;
		uint32_t      :6;
		uint32_t TBWE :1;
		uint32_t TBWO :1;
		uint32_t      :6;
		uint32_t RB   :1;
		uint32_t ID   :1;
		uint32_t      :6;
		uint32_t RO   :1;
		uint32_t FE   :1;
		uint32_t PE   :1;
		uint32_t BDE  :1;
		uint32_t RBRE :1;
		uint32_t RBRO :1;
		uint32_t      :2;
	};

	uint32_t Word;
} UART_IE_TypeDef;

/* Interrupt Flag Register */
typedef union {
	struct {
		uint32_t TB   :1;
		uint32_t TC   :1;
		uint32_t      :6;
		uint32_t TBWE :1;
		uint32_t TBWO :1;
		uint32_t      :6;
		uint32_t RB   :1;
		uint32_t ID   :1;
		uint32_t      :6;
		uint32_t RO   :1;
		uint32_t FE   :1;
		uint32_t PE   :1;
		uint32_t BDE  :1;
		uint32_t RBRE :1;
		uint32_t RBRO :1;
		uint32_t      :2;
	};

	uint32_t Word;
} UART_IF_TypeDef;

/* Transmit Buffer Write Register */
typedef union {
	uint8_t Byte[4];
	uint16_t HalfWord[2];
	uint32_t Word;
} UART_TBW_TypeDef;

/* Receive Buffer Read Register */
typedef union {
	uint8_t Byte[4];
	uint16_t HalfWord[2];
	uint32_t Word;
} UART_RBR_TypeDef;

/* Transmit Buffer Register */
typedef union {
	struct {
		uint32_t TB   :9;
		uint32_t      :3;
		uint32_t TP   :1;
		uint32_t TBFF :1;
		uint32_t      :18;
	};

	uint32_t Word;
} UART_TB_TypeDef;

/* Receive Buffer Register */
typedef union {
	struct {
		uint32_t RB   :9;
		uint32_t      :3;
		uint32_t RP   :1;
		uint32_t RBFF :1;
		uint32_t FE   :1;
		uint32_t PE   :1;
		uint32_t      :16;
	};

	uint32_t Word;
} UART_RB_TypeDef;

typedef struct {
	__IO UART_CON0_TypeDef CON0;      	/*!< Offset: 0x000 (R/W)  Control Register 0 */
	__IO UART_CON1_TypeDef CON1;      	/*!< Offset: 0x004 (R/W)  Control Register 1 */
	uint32_t               RSV0;
	uint32_t               RSV1;
	__IO UART_BRR_TypeDef  BRR;         	/*!< Offset: 0x010 (R/W)  Baudrate Config Register */
	__IO UART_STA_TypeDef  STA;          	/*!< Offset: 0x014 (R/W)  Status Register */
	__IO UART_IE_TypeDef   IE;        	/*!< Offset: 0x018 (R/W)  Interrupt Enable Register */
	__IO UART_IF_TypeDef   IF;           	/*!< Offset: 0x01C (R/W)  Interrupt Flag Register */
	__O  UART_TBW_TypeDef  TBW;         	/*!< Offset: 0x020 ( /W)  Transmit Buffer Write Register */
	__I  UART_RBR_TypeDef  RBR;		/*!< Offset: 0x024 (R/ )  Receive Buffer Read Register */
	uint32_t               RSV[6];
	__I  UART_TB_TypeDef   TB0;		/*!< Offset: 0x040 (R/ )  Transmit Buffer Register 0 */
	__I  UART_TB_TypeDef   TB1;		/*!< Offset: 0x044 (R/ )  Transmit Buffer Register 1 */
	__I  UART_TB_TypeDef   TB2;		/*!< Offset: 0x048 (R/ )  Transmit Buffer Register 2 */
	__I  UART_TB_TypeDef   TB3;		/*!< Offset: 0x04C (R/ )  Transmit Buffer Register 3 */
	__I  UART_TB_TypeDef   TB4;		/*!< Offset: 0x050 (R/ )  Transmit Buffer Register 4 */
	__I  UART_TB_TypeDef   TB5;		/*!< Offset: 0x054 (R/ )  Transmit Buffer Register 5 */
	__I  UART_TB_TypeDef   TB6;		/*!< Offset: 0x058 (R/ )  Transmit Buffer Register 6 */
	__I  UART_TB_TypeDef   TB7;		/*!< Offset: 0x05C (R/ )  Transmit Buffer Register 7 */
	__I  UART_RB_TypeDef   RB0;		/*!< Offset: 0x060 (R/ )  Receive Buffer Register 0 */
	__I  UART_RB_TypeDef   RB1;		/*!< Offset: 0x064 (R/ )  Receive Buffer Register 1 */
	__I  UART_RB_TypeDef   RB2;		/*!< Offset: 0x068 (R/ )  Receive Buffer Register 2 */
	__I  UART_RB_TypeDef   RB3;		/*!< Offset: 0x06C (R/ )  Receive Buffer Register 3 */
	__I  UART_RB_TypeDef   RB4;		/*!< Offset: 0x070 (R/ )  Receive Buffer Register 4 */
	__I  UART_RB_TypeDef   RB5;		/*!< Offset: 0x074 (R/ )  Receive Buffer Register 5 */
	__I  UART_RB_TypeDef   RB6;		/*!< Offset: 0x078 (R/ )  Receive Buffer Register 6 */
	__I  UART_RB_TypeDef   RB7;		/*!< Offset: 0x07C (R/ )  Receive Buffer Register 7 */
} HR8P506_UART_TypeDef;
/*@}*/ /* end of group HR8P506_UART */

/*------------- Enhanced Universal Asynchronous Receiver Transmitter (EUART) -----------*/
/** @addtogroup HR8P506_UART HR8P506 Universal Asynchronous Receiver/Transmitter*/
/* Control Register 0 */
typedef union {
	struct {
		uint32_t TXEN  :1;
		uint32_t TRST  :1;
		uint32_t TBCLR :1;
		uint32_t       :5;
		uint32_t TXMOD :4;
		uint32_t TXP   :1;
		uint32_t TXFS  :1;
		uint32_t       :2;
		uint32_t RXEN  :1;
		uint32_t RRST  :1;
		uint32_t RBCLR :1;
		uint32_t       :5;
		uint32_t RXMOD :4;
		uint32_t RXR   :1;
		uint32_t       :3;
	};

	uint32_t Word;
} EUART_CON0_TypeDef;

/* Control Register 1 */
typedef union {
	struct {
		uint32_t TBIM   :2;
		uint32_t        :2;
		uint32_t RBIM   :2;
		uint32_t        :2;
		uint32_t BCS    :3;
		uint32_t        :5;
		uint32_t TXBUSY :1;
		uint32_t RXBUSY :1;
		uint32_t        :14;
	};

	uint32_t Word;
} EUART_CON1_TypeDef;

/* Control Register 2 */
typedef union {
	struct {
		uint32_t MOD    :1;
		uint32_t ERST   :1;
		uint32_t CK0E   :1;
		uint32_t CK1E   :1;
		uint32_t CHS    :1;
		uint32_t IOC    :1;
		uint32_t DAS    :1;
		uint32_t PS     :1;
		uint32_t ARTE   :1;
		uint32_t ARRE   :1;
		uint32_t TNAS   :2;
		uint32_t ARTS   :2;
		uint32_t CKS    :2;
		uint32_t ETUS   :8;
		uint32_t BGTE   :1;
		uint32_t        :3;
		uint32_t RNACK  :1;
		uint32_t TXFEND :1;
		uint32_t        :2;
	};

	uint32_t Word;
} EUART_CON2_TypeDef;

/* Baudrate Config Register */
typedef union {
	struct {
		uint32_t BRR :11;
		uint32_t     :21;
	};

	uint32_t Word;
} EUART_BRR_TypeDef;

/* Interrupt Enable Register */
typedef union {
	struct {
		uint32_t TB   :1;
		uint32_t TC   :1;
		uint32_t      :6;
		uint32_t TBWE :1;
		uint32_t      :3;
		uint32_t ARTE :1;
		uint32_t RNA  :1;
		uint32_t      :2;
		uint32_t RB   :1;
		uint32_t      :7;
		uint32_t RO   :1;
		uint32_t FE   :1;
		uint32_t PE   :1;
		uint32_t      :1;
		uint32_t RBRE :1;
		uint32_t      :3;
	};

	uint32_t Word;
} EUART_IE_TypeDef;

/* Interrupt Flag Register */
typedef union {
	struct {
		uint32_t TB   :1;
		uint32_t TC   :1;
		uint32_t      :6;
		uint32_t TBWE :1;
		uint32_t      :3;
		uint32_t ARTE :1;
		uint32_t RNA  :1;
		uint32_t      :2;
		uint32_t RB   :1;
		uint32_t      :7;
		uint32_t RO   :1;
		uint32_t FE   :1;
		uint32_t PE   :1;
		uint32_t      :1;
		uint32_t RBRE :1;
		uint32_t      :3;
	};

	uint32_t Word;
} EUART_IF_TypeDef;

/* Transmit Buffer Write Register */
typedef union {
	uint8_t byte[4];
	uint16_t halfword[2];
	uint32_t word;
} EUART_TBW_TypeDef;

/* Receive Buffer Read Register */
typedef union {
	uint8_t byte[4];
	uint16_t halfword[2];
	uint32_t word;
} EUART_RBR_TypeDef;

/* Transmit Buffer Register 0/1 */
typedef union {
	struct {
		uint32_t TB0   :9;
		uint32_t       :3;
		uint32_t TP0   :1;
		uint32_t TBEF0 :1;
		uint32_t       :2;
		uint32_t TB1   :9;
		uint32_t       :3;
		uint32_t TP1   :1;
		uint32_t TBEF1 :1;
		uint32_t       :2;
	};

	uint32_t Word;
} EUART_TB01_TypeDef;

/* Transmit Buffer Register 2/3 */
typedef union {
	struct {
		uint32_t TB2   :9;
		uint32_t       :3;
		uint32_t TP2   :1;
		uint32_t TBEF2 :1;
		uint32_t       :2;
		uint32_t TB3   :9;
		uint32_t       :3;
		uint32_t TP3   :1;
		uint32_t TBEF3 :1;
		uint32_t       :2;
	};

	uint32_t Word;
} EUART_TB23_TypeDef;

/* Receive Buffer Register 0/1 */
typedef union {
	struct {
		uint32_t RB0   :9;
		uint32_t       :3;
		uint32_t RP0   :1;
		uint32_t RBEF0 :1;
		uint32_t FE0   :1;
		uint32_t PE0   :1;
		uint32_t RB1   :9;
		uint32_t       :3;
		uint32_t RP1   :1;
		uint32_t RBEF1 :1;
		uint32_t FE1   :1;
		uint32_t PE1   :1;
	};

	uint32_t Word;
} EUART_RB01_TypeDef;

/* Receive Buffer Register 2/3 */
typedef union {
	struct {
		uint32_t RB2   :9;
		uint32_t       :3;
		uint32_t RP2   :1;
		uint32_t RBEF2 :1;
		uint32_t FE2   :1;
		uint32_t PE2   :1;
		uint32_t RB3   :9;
		uint32_t       :3;
		uint32_t RP3   :1;
		uint32_t RBEF3 :1;
		uint32_t FE3   :1;
		uint32_t PE3   :1;
	};

	uint32_t Word;
} EUART_RB23_TypeDef;

typedef struct {
	__IO EUART_CON0_TypeDef CON0;                  	/*!< Offset: 0x000 (R/W)  Control Register 0 */
	__IO EUART_CON1_TypeDef CON1;                  	/*!< Offset: 0x004 (R/W)  Control Register 1 */
	__IO EUART_CON2_TypeDef CON2;                  	/*!< Offset: 0x008 (R/W)  Control Register 2 */
	uint32_t                RSV0;
	__IO EUART_BRR_TypeDef  BRR;                   	/*!< Offset: 0x010 (R/W)  Baudrate Config Register */
	uint32_t                RSV1;
	__IO EUART_IE_TypeDef   IE;                    	/*!< Offset: 0x018 (R/W)  Interrupt Enable Register */
	__IO EUART_IF_TypeDef   IF;                    	/*!< Offset: 0x01C (R/W)  Interrupt Flag Register */
	__O  EUART_TBW_TypeDef  TBW;                  	/*!< Offset: 0x020 ( /W)  Transmit Buffer Write Register */
	__I  EUART_RBR_TypeDef  RBR;                 	/*!< Offset: 0x024 (R/ )  Receive Buffer Read Register */
	uint32_t                RSV[6];
	__I  EUART_TB01_TypeDef TB01;					/*!< Offset: 0x040 (R/ )  Transmit Buffer Register 0/1*/
	__I  EUART_TB23_TypeDef TB23;					/*!< Offset: 0x044 (R/ )  Transmit Buffer Register 2/3*/
	__I  EUART_RB01_TypeDef RB01;					/*!< Offset: 0x048 (R/ )  Receive Buffer Register 0/1*/
	__I  EUART_RB23_TypeDef RB23;					/*!< Offset: 0x04C (R/ )  Receive Buffer Register 2/3*/
} HR8P506_EUART_TypeDef;
/*@}*/ /* end of group HR8P506_UART */

/*------------- Serial Periphral Interface (SPI) -----------*/
/** @addtogroup HR8P506_SPI HR8P506  Serial Periphral Interface*/

/* Control Register */
typedef union {
	struct {
		uint32_t EN    :1;
		uint32_t RST   :1;
		uint32_t MS    :1;
		uint32_t REN   :1;
		uint32_t       :1;
		uint32_t DRE   :1;
		uint32_t DFS   :2;
		uint32_t       :8;
		uint32_t TME   :1;
		uint32_t TMS   :1;
		uint32_t TMP   :6;
		uint32_t DW    :3;
		uint32_t       :3;
		uint32_t TXCLR :1;
		uint32_t RXCLR :1;
	};

	uint32_t Word;
} SPI_CON_TypeDef;

/* Transmit Buffer Write Register */
typedef union {
	uint8_t  Byte[4];
	uint16_t HalfWord[2];
	uint32_t Word;
} SPI_TBW_TypeDef;

/* Receive Buffer Read Register */
typedef union {
	uint8_t  Byte[4];
	uint16_t HalfWord[2];
	uint32_t Word;
} SPI_RBR_TypeDef;

/* Interrupt Enable Register */
typedef union {
	struct {
		uint32_t TBIE   :1;
		uint32_t RBIE   :1;
		uint32_t TEIE   :1;
		uint32_t ROIE   :1;
		uint32_t IDIE   :1;
		uint32_t NSSIE  :1;
		uint32_t TBWEIE :1;
		uint32_t        :1;
		uint32_t TBIM   :2;
		uint32_t RBIM   :2;
		uint32_t        :20;
	};

	uint32_t Word;
} SPI_IE_TypeDef;

/* Interrupt Flag Register */
typedef union {
	struct {
		uint32_t TBIF   :1;
		uint32_t RBIF   :1;
		uint32_t TEIF   :1;
		uint32_t ROIF   :1;
		uint32_t IDIF   :1;
		uint32_t NSSIF  :1;
		uint32_t TBWEIF :1;
		uint32_t NSS    :1;
		uint32_t TBEF0  :1;
		uint32_t TBEF1  :1;
		uint32_t TBEF2  :1;
		uint32_t TBEF3  :1;
		uint32_t RBFF0  :1;
		uint32_t RBFF1  :1;
		uint32_t RBFF2  :1;
		uint32_t RBFF3  :1;
		uint32_t IDLE   :1;
		uint32_t        :15;
	};

	uint32_t Word;
} SPI_IF_TypeDef;

/* Transmit Buffer Register */
typedef union {
	struct {
		uint32_t TB0 :8;
		uint32_t TB1 :8;
		uint32_t TB2 :8;
		uint32_t TB3 :8;
	};

	uint32_t Word;
} SPI_TB_TypeDef;

/* Reveive Buffer Register */
typedef union {
	struct {
		uint32_t RB0 :8;
		uint32_t RB1 :8;
		uint32_t RB2 :8;
		uint32_t RB3 :8;
	};

	uint32_t Word;
} SPI_RB_TypeDef;

/* Status Register */
typedef union {
	struct {
		uint32_t       :7;
		uint32_t NSS   :1;
		uint32_t TBEF0 :1;
		uint32_t TBEF1 :1;
		uint32_t TBEF2 :1;
		uint32_t TBEF3 :1;
		uint32_t RBFF0 :1;
		uint32_t RBFF1 :1;
		uint32_t RBFF2 :1;
		uint32_t RBFF3 :1;
		uint32_t IDLE  :1;
		uint32_t       :15;
	};

	uint32_t Word;
} SPI_STA_TypeDef;

/* Baudrate Config Register */
typedef union {
	struct {
		uint32_t CKS :10;
		uint32_t     :22;
	};

	uint32_t Word;
} SPI_CKS_TypeDef;

typedef struct {
	__IO SPI_CON_TypeDef CON;       /*!< Offset: 0x000 (R/W)  Control Register */
 	uint32_t             RSV;
	__O  SPI_TBW_TypeDef TBW;		/*!< Offset: 0x008 ( /W)  Transmit Buffer Write Register */
	__I  SPI_RBR_TypeDef RBR;		/*!< Offset: 0x00C (R/ )  Receive Buffer Read Register */
	__IO SPI_IE_TypeDef  IE;		/*!< Offset: 0x010 (R/W)  Interrupt Enable Register */
	__IO SPI_IF_TypeDef  IF;		/*!< Offset: 0x014 (R/W)  Interrupt Flag Register */
	__I  SPI_TB_TypeDef  TB;		/*!< Offset: 0x018 (R/ )  Transmit Buffer Register */
	__I  SPI_RB_TypeDef  RB;		/*!< Offset: 0x01C (R/ )  Receive Buffer Register */
	__I  SPI_STA_TypeDef STA;		/*!< Offset: 0x020 (R/ )  Status Register */
	__IO SPI_CKS_TypeDef CKS;		/*!< Offset: 0x024 (R/W)  Baudrate Config Register */
} HR8P506_SPI_TypeDef;
/*@}*/ /* end of group HR8P506_SPI */


/*------------- Inter-Integrated Circuit (IIC) -----------*/
/** @addtogroup HR8P506_I2C HR8P506  Inter-Integrated Circuit*/
/* Control Register */
typedef union {
	struct {
		uint32_t EN    :1;
		uint32_t RST   :1;
		uint32_t SCKOD :1;
		uint32_t SDAOD :1;
		uint32_t SCKSE :1;
		uint32_t SDASE :1;
		uint32_t       :1;
		uint32_t TJE   :1;
		uint32_t TJP   :8;
		uint32_t RW    :1;
		uint32_t SA    :7;
		uint32_t       :8;
	};

	uint32_t Word;
} I2C_CON_TypeDef;

/* Mode Select Register */

typedef union {
	struct {
		uint32_t MS   :1;
		uint32_t RDM  :3;
		uint32_t CSE  :1;
		uint32_t ANAE :1;
		uint32_t SRAE :1;
		uint32_t SPAE :1;
		uint32_t ADLY :3;
		uint32_t ADE  :1;
		uint32_t TIS  :4;
		uint32_t SRT  :1;
		uint32_t SPT  :1;
		uint32_t RDT  :1;
		uint32_t BLD  :1;
		uint32_t      :4;
		uint32_t TAS  :1;
		uint32_t      :7;
	};

	uint32_t Word;
} I2C_MODE_TypeDef;

/* Interrupt Enable Register */
typedef union {
	struct {
		uint32_t SR    :1;
		uint32_t SP    :1;
		uint32_t TB    :1;
		uint32_t RB    :1;
		uint32_t TE    :1;
		uint32_t RO    :1;
		uint32_t NA    :1;
		uint32_t TBWE  :1;
		uint32_t TBIM  :2;
		uint32_t RBIM  :2;
		uint32_t TIDLE :1;
		uint32_t       :19;
	};

	uint32_t Word;
} I2C_IE_TypeDef;

/* Interrupt Flag Register */
typedef union {
	struct {
		uint32_t SR    :1;
		uint32_t SP    :1;
		uint32_t TB    :1;
		uint32_t RB    :1;
		uint32_t TE    :1;
		uint32_t RO    :1;
		uint32_t NA    :1;
		uint32_t TBWE  :1;
		uint32_t       :4;
		uint32_t TIDLE :1;
		uint32_t       :19;
	};

	uint32_t Word;
} I2C_IF_TypeDef;

/* Transmit Date Buffer Register */
typedef union {
	uint8_t Byte[4];
	uint16_t HalfWord[2];
	uint32_t Word;
} I2C_TBW_TypeDef;

/* Receive Data Buffer Register */
typedef union {
	uint8_t Byte[4];
	uint16_t HalfWord[2];
	uint32_t Word;
} I2C_RBR_TypeDef;

/* Transmit Buffer Register */
typedef union {
	struct {
		uint32_t TB0 :8;
		uint32_t TB1 :8;
		uint32_t TB2 :8;
		uint32_t TB3 :8;
	};

	uint32_t Word;
} I2C_TB_TypeDef;

/* Receive Buffer Register */
typedef union {
	struct {
		uint32_t RB0 :8;
		uint32_t RB1 :8;
		uint32_t RB2 :8;
		uint32_t RB3 :8;
	};

	uint32_t Word;
} I2C_RB_TypeDef;

/* Status Register */
typedef union {
	struct {
		uint32_t       :8;
		uint32_t TBEF0 :1;
		uint32_t TBEF1 :1;
		uint32_t TBEF2 :1;
		uint32_t TBEF3 :1;
		uint32_t RBFF0 :1;
		uint32_t RBFF1 :1;
		uint32_t RBFF2 :1;
		uint32_t RBFF3 :1;
		uint32_t ACK   :1;
		uint32_t IDLE  :1;
		uint32_t       :14;
	};

	uint32_t Word;
} I2C_STA_TypeDef;

typedef struct {
	__IO I2C_CON_TypeDef  CON; 		/*!< Offset: 0x000 (R/W)  Control Register */
	__IO I2C_MODE_TypeDef MODE;     /*!< Offset: 0x004 (R/W)  Mode Select Register */
	__IO I2C_IE_TypeDef   IE;       /*!< Offset: 0x008 (R/W)  Interrupt Enable Register */
	__IO I2C_IF_TypeDef   IF;       /*!< Offset: 0x00C (R/W)  Status Register */
	__O  I2C_TBW_TypeDef  TBW;
	__I  I2C_RBR_TypeDef  RBR;		/*!< Offset: 0x014 (R/ )  Receive Data Buffer Register */
	__I  I2C_TB_TypeDef   TB;		/*!< Offset: 0x018 (R/ )  Transmit Buffer Register */
	__I  I2C_RB_TypeDef   RB;		/*!< Offset: 0x01C (R/ )  Receive Buffer Register */
	__I  I2C_STA_TypeDef  STA;		/*!< Offset: 0x020 (R/ )  Status Register */
} HR8P506_I2C_TypeDef;
/*@}*/ /* end of group HR8P506_SPI */


/*------------- System Tick (SYSTICK) -------------------------------*/
/** @addtogroup HR8P506_SYSTICK HR8P506 system tick Interface 
  @{
*/
typedef struct {
	__IO uint32_t CSR;                 	/*!< Offset: 0x000 (R/W)  SYST Control/Status Register */
	__IO uint32_t RVR;                 	/*!< Offset: 0x004 (R/W)  SYST Reload Value Register */
	__IO uint32_t CVR;                  /*!< Offset: 0x008 (R/W)  SYST Current Value Register */
	__I  uint32_t CALIB;                /*!< Offset: 0x00C (R/ )  SYST Calibration Register */
} HR8P506_SYSTICK_TypeDef;
/*@}*/ /* end of group HR8P506_SYSTICK */


/*------------- Nested Vectored Interrupt Controller -------------------------------*/
/** @addtogroup HR8P506_NVIC HR8P506 Nested Vectored Interrupt Controller*/
typedef struct {
  	__IO uint32_t ISER;       	/*!< Offset: 0x000 (R/W)  Interrupt Enable Register */
	uint32_t RESERVED0[31];
	__IO uint32_t ICER;        	/*!< Offset: 0x080 (R/W)  Interrupt Enable Clear Register */
	uint32_t RESERVED1[63];
	__IO uint32_t ISPR;        	/*!< Offset: 0x200 (R/W)  Interrupt Pending Set Register */
	uint32_t RESERVED2[31];
	__IO uint32_t ICPR;        	/*!< Offset: 0x280 (R/W)  Interrupt Pending Clear Register */
	uint32_t RESERVED3[127];
	__IO uint32_t PR0;			/*!< Offset: 0x400 (R/W)  IRQ0~3   Priority Register */
	__IO uint32_t PR1;			/*!< Offset: 0x404 (R/W)  IRQ4~7   Priority Register */
	__IO uint32_t PR2;			/*!< Offset: 0x408 (R/W)  IRQ8~11  Priority Register */
	__IO uint32_t PR3;			/*!< Offset: 0x40C (R/W)  IRQ12~15 Priority Register */
	__IO uint32_t PR4;			/*!< Offset: 0x410 (R/W)  IRQ16~19 Priority Register */
	__IO uint32_t PR5;			/*!< Offset: 0x414 (R/W)  IRQ20~23 Priority Register */
	__IO uint32_t PR6;			/*!< Offset: 0x418 (R/W)  IRQ24~27 Priority Register */
	__IO uint32_t PR7;			/*!< Offset: 0x41C (R/W)  IRQ28~31 Priority Register */
} HR8P506_NVIC_TypeDef;
/*@}*/ /* end of group HR8P506_NVIC */


//#if defined ( __CC_ARM   )
#pragma no_anon_unions
//#endif

/* Peripheral memory map */

/* Base addresses */
#define HR8P506_FLASH_BASE        	(0x00000000UL)
#define HR8P506_SRAM_BASE         	(0x20000000UL)
#define HR8P506_APB_BASE          	(0x40000000UL)
#define HR8P506_RAM_BASE          	(0x60000000UL)
#define HR8P506_SYS_BASE          	(0xE0000000UL)

/* APB peripherals */
#define HR8P506_SCU_BASE          	(HR8P506_APB_BASE + 0x00000)
#define HR8P506_GPIO_BASE         	(HR8P506_APB_BASE + 0x00400)
#define HR8P506_GPIOE_BASE        	(HR8P506_APB_BASE + 0x00700)
#define HR8P506_IAP_BASE          	(HR8P506_APB_BASE + 0x00800)
#define HR8P506_ADC_BASE          	(HR8P506_APB_BASE + 0x01000)
#define HR8P506_RTC_BASE          	(HR8P506_APB_BASE + 0x01400)
#define HR8P506_LCD_BASE          	(HR8P506_APB_BASE + 0x01800)
#define HR8P506_WDT_BASE          	(HR8P506_APB_BASE + 0x01C00)
#define HR8P506_T16N0_BASE        	(HR8P506_APB_BASE + 0x02000)
#define HR8P506_T16N1_BASE        	(HR8P506_APB_BASE + 0x02400)
#define HR8P506_T16N2_BASE        	(HR8P506_APB_BASE + 0x02800)
#define HR8P506_T16N3_BASE        	(HR8P506_APB_BASE + 0x02C00)
#define HR8P506_T32N0_BASE        	(HR8P506_APB_BASE + 0x04000)
#define HR8P506_UART0_BASE        	(HR8P506_APB_BASE + 0x06000)
#define HR8P506_UART1_BASE        	(HR8P506_APB_BASE + 0x06400)
#define HR8P506_EUART_BASE        	(HR8P506_APB_BASE + 0x07000)
#define HR8P506_SPI0_BASE         	(HR8P506_APB_BASE + 0x08000)
#define HR8P506_SPI1_BASE         	(HR8P506_APB_BASE + 0x08400)
#define HR8P506_I2C_BASE            (HR8P506_APB_BASE + 0x09000)
#define HR8P506_SYSTICK_BASE      	(HR8P506_SYS_BASE + 0x0E010)
#define HR8P506_NVIC_BASE           (HR8P506_SYS_BASE + 0x0E100)
#define HR8P506_SCB_BASE          	(HR8P506_SYS_BASE + 0x0ED00)


#define HR8P506_CAL_BASE            (HR8P506_APB_BASE + 0x0094)
#define HR8P506_PORTCAL_BASE        (HR8P506_APB_BASE + 0x0080)

/* Peripheral declaration */
#define SCU               	((HR8P506_SCU_TypeDef    *) HR8P506_SCU_BASE    )
#define GPIO              	((HR8P506_GPIO_TypeDef   *) HR8P506_GPIO_BASE   )
#define GPIOE             	((HR8P506_GPIOE_TypeDef  *) HR8P506_GPIOE_BASE  )
#define IAP               	((HR8P506_IAP_TypeDef    *) HR8P506_IAP_BASE    )
#define ADC               	((HR8P506_ADC_TypeDef    *) HR8P506_ADC_BASE    )
#define RTC               	((HR8P506_RTC_TypeDef    *) HR8P506_RTC_BASE    )
#define LCD               	((HR8P506_LCD_TypeDef    *) HR8P506_LCD_BASE    )
#define WDT               	((HR8P506_WDT_TypeDef    *) HR8P506_WDT_BASE    )
#define T16N0             	((HR8P506_T16N_TypeDef   *) HR8P506_T16N0_BASE  )
#define T16N1             	((HR8P506_T16N_TypeDef   *) HR8P506_T16N1_BASE  )
#define T16N2             	((HR8P506_T16N_TypeDef   *) HR8P506_T16N2_BASE  )
#define T16N3             	((HR8P506_T16N_TypeDef   *) HR8P506_T16N3_BASE  )
#define T32N0             	((HR8P506_T32N_TypeDef   *) HR8P506_T32N0_BASE  )
#define UART0             	((HR8P506_UART_TypeDef   *) HR8P506_UART0_BASE  )
#define UART1             	((HR8P506_UART_TypeDef   *) HR8P506_UART1_BASE  )
#define EUART             	((HR8P506_EUART_TypeDef  *) HR8P506_EUART_BASE  )
#define SPI0              	((HR8P506_SPI_TypeDef    *) HR8P506_SPI0_BASE   )
#define SPI1              	((HR8P506_SPI_TypeDef    *) HR8P506_SPI1_BASE   )
#define I2C               	((HR8P506_I2C_TypeDef    *) HR8P506_I2C_BASE   )
#define SYSTICK           	((HR8P506_SYSTICK_TypeDef*) HR8P506_SYSTICK_BASE)

#define CALR           		((uint32_t*) HR8P506_CAL_BASE )
#define PORTCALR           	((uint32_t*) HR8P506_PORTCAL_BASE)

#define NVIC              	((HR8P506_NVIC_TypeDef   *) HR8P506_NVIC_BASE   )
#define SCB               	((HR8P506_SCB_TypeDef    *) HR8P506_SCB_BASE    )

#ifdef __cplusplus
}
#endif

#endif  /* __HR8P506_REG_H */
