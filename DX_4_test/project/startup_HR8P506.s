;*******************************************************************************
; * @file     startup.s
; * @brief    hr8p506 start up file
; *           
; *
; * @version  1.0
; * @date     2015-12-01
; *
; * @author   
; *
; * @note
; * @Copyright (C) 2015 Shanghai Eastsoft Microelectronics C0., Ltd.
;******************************************************************************/
 
;Stack Configuration------------------------------------------------------------
Stack_Size      EQU     0x00000800
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp
;-------------------------------------------------------------------------------

;Heap Configuration-------------------------------------------------------------
Heap_Size       EQU     0x00000000
                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit
;-------------------------------------------------------------------------------
                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset-------------------------------------
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors
                DCD     __initial_sp    				;0,  load top of stack
                DCD     Reset_Handler           		;1,  reset handler
                DCD     NMI_HANDLER            		 	;2,  nmi handler
                DCD     HARDFAULT_HANDLER       		;3,  hard fault handler
                DCD     0                       		;4,  Reserved
                DCD     0                       		;5,  Reserved
                DCD     0                       		;6,  Reserved
                DCD     0                       		;7,  Reserved
                DCD     0                       		;8,  Reserved
                DCD     0                       		;9,  Reserved
                DCD     0                       		;10, Reserved
                DCD     SVC_HANDLER            		 	;11, svcall handler
                DCD     0                       		;12, Reserved
                DCD     0                       		;13, Reserved
                DCD     PENDSV_HANDLER          		;14, pendsv handler
                DCD     SYSTICK_HANDLER         		;15, systick handler
                DCD     PINT0_HANDLER           		;16, pint0 handler
                DCD     PINT1_HANDLER           		;17, pint1 handler
                DCD     PINT2_HANDLER           		;18, pint2 handler
                DCD     PINT3_HANDLER           		;19, pint3 handler
                DCD     PINT4_HANDLER           		;20, pint4 handler
                DCD     PINT5_HANDLER           		;21, pint5 handler
                DCD     PINT6_HANDLER           		;22, pint6 handler
                DCD     PINT7_HANDLER           		;23, pint7 handler
                DCD     T16N0_HANDLER           		;24, t16n0 handler
                DCD     T16N1_HANDLER           		;25, t16n1 handler
                DCD     T16N2_HANDLER           		;26, t16n2 handler
                DCD     T16N3_HANDLER           		;27, Reserved
                DCD     T32N0_HANDLER           		;28, t32n0 handler
                DCD     T32N1_HANDLER           		;29, t32n1 handler
                DCD     0                       		;30, Reserved
                DCD     0                       		;31, Reserved
                DCD     TWDT_HANDLER            		;32, wdt  handler
                DCD     TRTC_HANDLER            		;33, rtc  handler
                DCD     KINT_HANDLER           	 		;34, kint handler
                DCD     ADC_HANDLER             		;35, adc  handler
                DCD     LCD_HANDLER             		;36, lcd handler
                DCD     0                       		;37, Reserved
                DCD     0                       		;38, Reserved
                DCD     UART0_HANDLER           		;39, uart0 handler
                DCD     UART1_HANDLER           		;40, Reserved
                DCD     EUART0_HANDLER          		;41, Reserved
                DCD     0                      	 		;42, Reserved
                DCD     SPI0_HANDLER            		;43, euart0 handler
                DCD     SPI1_HANDLER            		;44, Reserved
                DCD     I2C0_HANDLER            		;45  spi handler
                DCD     I2C1_HANDLER           			;46, i2c0 handler
                DCD     CCM_HANDLER             		;47, ccm handler

;-------------------------------------------------------------------------------
                AREA    INT, CODE, READONLY       ;int code begin

;Reset Handler------------------------------------------------------------------
Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
                LDR     R0, =__main
                BX      R0
                NOP
                ALIGN
                ENDP

;system int---------------------------------------------------------------------
NMI_HANDLER     PROC                                   ;int 2
                EXPORT  NMI_HANDLER               [WEAK]
                IMPORT  INT_NMI                   [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_NMI
                POP     {PC}                           ;int exit
                ENDP

HARDFAULT_HANDLER\
                PROC                                   ;int3
                EXPORT  HARDFAULT_HANDLER         [WEAK]
                IMPORT  INT_HARDFAULT             [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_HARDFAULT
                POP     {PC}                           ;int exit
                ENDP

SVC_HANDLER     PROC                                   ;int11
                EXPORT  SVC_HANDLER               [WEAK]
                IMPORT  INT_SVC                   [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_SVC
                POP     {PC}                           ;int exit
                ENDP

PENDSV_HANDLER  PROC                                   ;int14
                EXPORT  PENDSV_HANDLER            [WEAK]
                IMPORT  INT_PENDSV                [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_PENDSV
                POP     {PC}                           ;int exit
                ENDP

SYSTICK_HANDLER PROC                                   ;int15
                EXPORT  SYSTICK_HANDLER           [WEAK]
                IMPORT  INT_SYSTICK               [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_SYSTICK
                POP     {PC}                           ;int exit
                ENDP

;pint int &peripheral module int -----------------------------------------------

PINT0_HANDLER   PROC                                   ;int16
                EXPORT  PINT0_HANDLER             [WEAK]
                IMPORT  INT_PINT0                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_PINT0
                POP     {PC}                           ;int exit
                ENDP

PINT1_HANDLER   PROC                                   ;int17
                EXPORT  PINT1_HANDLER             [WEAK]
                IMPORT  INT_PINT1                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_PINT1
                POP     {PC}                           ;int exit
                ENDP

PINT2_HANDLER   PROC                                   ;int18
                EXPORT  PINT2_HANDLER             [WEAK]
                IMPORT  INT_PINT2                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_PINT2
                POP     {PC}                           ;int exit
                ENDP

PINT3_HANDLER   PROC                                   ;int19
                EXPORT  PINT3_HANDLER             [WEAK]
                IMPORT  pint3_handler             [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      pint3_handler
                POP     {PC}                           ;int exit
                ENDP

PINT4_HANDLER   PROC                                   ;int20
                EXPORT  PINT4_HANDLER             [WEAK]
                IMPORT  pint4_handler                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      pint4_handler
                POP     {PC}                           ;int exit
                ENDP

PINT5_HANDLER   PROC                                   ;int21
                EXPORT  PINT5_HANDLER             [WEAK]
                IMPORT  pint5_handler                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      pint5_handler
                POP     {PC}                           ;int exit
                ENDP

PINT6_HANDLER   PROC                                   ;int22
                EXPORT  PINT6_HANDLER             [WEAK]
                IMPORT  pint6_handler              [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      pint6_handler
                POP     {PC}                           ;int exit
                ENDP

PINT7_HANDLER   PROC                                   ;int23
                EXPORT  PINT7_HANDLER             [WEAK]
                IMPORT  INT_PINT7                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_PINT7
                POP     {PC}                           ;int exit
                ENDP

T16N0_HANDLER   PROC                                   ;int24
                EXPORT  T16N0_HANDLER             [WEAK]
                IMPORT  INT_T16N0                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_T16N0
                POP     {PC}                           ;int exit
                ENDP

T16N1_HANDLER   PROC                                   ;int25
                EXPORT  T16N1_HANDLER             [WEAK]
                IMPORT  INT_T16N1                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_T16N1
                POP     {PC}                           ;int exit
                ENDP

T16N2_HANDLER   PROC                                   ;int26
                EXPORT  T16N2_HANDLER             [WEAK]
                IMPORT  INT_T16N2                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_T16N2
                POP     {PC}                           ;int exit
                ENDP

T16N3_HANDLER   PROC                                   ;int27
                EXPORT  T16N3_HANDLER             [WEAK]
                IMPORT  INT_T16N3                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_T16N3
                POP     {PC}                           ;int exit
                ENDP

T32N0_HANDLER   PROC                                   ;int28
                EXPORT  T32N0_HANDLER             [WEAK]
                IMPORT  INT_T32N0                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_T32N0
                POP     {PC}                           ;int exit
                ENDP

T32N1_HANDLER   PROC                                   ;int29
                EXPORT  T32N1_HANDLER             [WEAK]
                IMPORT  INT_T32N1                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_T32N1
                POP     {PC}                           ;int exit
                ENDP

TWDT_HANDLER    PROC                                   ;int32
                EXPORT  TWDT_HANDLER              [WEAK]
                IMPORT  INT_TWDT                  [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_TWDT
                POP     {PC}                           ;int exit
                ENDP

TRTC_HANDLER    PROC                                   ;int33
                EXPORT  TRTC_HANDLER              [WEAK]
                IMPORT  INT_RTC                  [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_RTC
                POP     {PC}                           ;int exit
                ENDP

KINT_HANDLER    PROC                                   ;int34
                EXPORT  KINT_HANDLER              [WEAK]
                IMPORT  INT_KINT                  [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_KINT
                POP     {PC}                           ;int exit
                ENDP

ADC_HANDLER     PROC                                   ;int35
                EXPORT  ADC_HANDLER               [WEAK]
                IMPORT  INT_ADC                   [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_ADC
                POP     {PC}                           ;int exit
                ENDP

LCD_HANDLER     PROC                                   ;int36
                EXPORT  LCD_HANDLER               [WEAK]
                IMPORT  INT_LCD                   [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_LCD
                POP     {PC}                           ;int exit
                ENDP

UART0_HANDLER\
                PROC                                   ;int39
                EXPORT  UART0_HANDLER             [WEAK]
                IMPORT  INT_UART0                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_UART0
                POP     {PC}                           ;int exit
                ENDP

UART1_HANDLER\
                PROC                                   ;int40
                EXPORT  UART1_HANDLER             [WEAK]
                IMPORT  INT_UART1                 [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_UART1
                POP     {PC}                           ;int exit
                ENDP

EUART0_HANDLER\
                PROC                                   ;int41
                EXPORT  EUART0_HANDLER            [WEAK]
                IMPORT  INT_EUART0                [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_EUART0
                POP     {PC}                           ;int exit
                ENDP

SPI0_HANDLER\
                PROC                                   ;int43
                EXPORT  SPI0_HANDLER              [WEAK]
                IMPORT  INT_SPI0                  [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_SPI0
                POP     {PC}                           ;int exit
                ENDP

SPI1_HANDLER\
                PROC                                   ;int44
                EXPORT  SPI1_HANDLER              [WEAK]
                IMPORT  INT_SPI1                  [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_SPI1
                POP     {PC}                           ;int exit
                ENDP

I2C0_HANDLER\
                PROC                                   ;int45
                EXPORT  I2C0_HANDLER              [WEAK]
                IMPORT  INT_I2C0                  [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_I2C0
                POP     {PC}                           ;int exit
                ENDP

I2C1_HANDLER\
                PROC                                   ;int46
                EXPORT  I2C1_HANDLER              [WEAK]
                IMPORT  INT_I2C1                  [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_I2C1
                POP     {PC}                           ;int exit
                ENDP

CCM_HANDLER\
                PROC                                   ;int47
                EXPORT  CCM_HANDLER               [WEAK]
                IMPORT  INT_CCM                   [WEAK]
                PUSH    {LR}                           ;protect exc_return
                BL      INT_CCM
                POP     {PC}                           ;int exit
                ENDP

; User Initial Stack & Heap-----------------------------------------------------
                ALIGN
                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap
                LDR     R0, =  Heap_Mem
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN
                ENDIF
					
				ALIGN
				END	
					