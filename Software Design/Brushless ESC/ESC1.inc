/*
 * ESC2.inc
 *
 *  Created: 19/10/2012 3:40:08 PM
 *   Author: SEA-ROBOT
 */ 
 .equ	F_CPU		= 8000000
.equ	USE_INT0	= 1
.equ	USE_I2C		= 0
.equ	USE_UART	= 0
.equ	USE_ICP		= 0

;*********************
; PORT D definitions *
;*********************
;.equ	mux_c		= 7	;i <plus> comparator input (AIN0)
;.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	ApFET		= 5	;o
.equ	BpFET		= 4	;o
.equ	CpFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input

.equ	INIT_PD		= 0
.equ	DIR_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)

.equ	ApFET_port	= PORTD
.equ	BpFET_port	= PORTD
.equ	CpFET_port	= PORTD

.MACRO rcp_int_enable
	ldi	@0, (1<<INT0)	; enable ext0int
	out	GICR, @0
.ENDMACRO
.MACRO rcp_int_disable
	out	GICR, ZH	; disable ext0int
.ENDMACRO
.MACRO rcp_int_rising_edge
	ldi	@0, (1<<ISC01)+(1<<ISC00)
	out	MCUCR, @0	; set next int0 to rising edge
.ENDMACRO
.MACRO rcp_int_falling_edge
	ldi	@0, (1<<ISC01)
	out	MCUCR, @0	; set next int0 to falling edge
.ENDMACRO

.MACRO ApFET_on
	sbi	PORTD, ApFET
.ENDMACRO
.MACRO ApFET_off
	cbi	PORTD, ApFET
.ENDMACRO
.MACRO BpFET_on
	sbi	PORTD, BpFET
.ENDMACRO
.MACRO BpFET_off
	cbi	PORTD, BpFET
.ENDMACRO
.MACRO CpFET_on
	sbi	PORTD, CpFET
.ENDMACRO
.MACRO CpFET_off
	cbi	PORTD, CpFET
.ENDMACRO

.MACRO ApFET_on_reg
	sbr	@0, 1<<ApFET
.ENDMACRO
.MACRO ApFET_off_reg
	cbr	@0, 1<<ApFET
.ENDMACRO
.MACRO BpFET_on_reg
	sbr	@0, 1<<BpFET
.ENDMACRO
.MACRO BpFET_off_reg
	cbr	@0, 1<<BpFET
.ENDMACRO
.MACRO CpFET_on_reg
	sbr	@0, 1<<CpFET
.ENDMACRO
.MACRO CpFET_off_reg
	cbr	@0, 1<<CpFET
.ENDMACRO

.MACRO all_pFETs_off
	in	@0, PORTD
	cbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
	out	PORTD, @0
.ENDMACRO


;*********************
; PORT C definitions *
;*********************
;.equ			= 7	; ADC7
;.equ			= 6	; ADC6
.equ	mux_a		= 2	; ADC2 phase input
.equ	mux_b		= 4	; ADC4 phase input
;.equ			= 3	; ADC3
.equ	mux_c		= 5	; ADC5 phase input
.equ	temp_adc	= 1	; ADC1 temperature control input
.equ	accu_adc	= 0	; ADC0 voltage control input

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0

.MACRO comp_init
	in	@0, SFIOR
	sbr	@0, (1<<ACME)	; set Analog Comparator Multiplexer Enable
	out	SFIOR, @0
	cbi	ADCSRA, ADEN	; disable ADC
.ENDMACRO
.MACRO set_comp_phase_a
	ldi	@0, mux_a	; set comparator multiplexer to phase A
	out	ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_b
	ldi	@0, mux_b	; set comparator multiplexer to phase B
	out	ADMUX, @0
.ENDMACRO
.MACRO set_comp_phase_c	
	ldi	@0, mux_c	; set comparator multiplexer to phase C
	out	ADMUX, @0
.ENDMACRO


;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck stk200 interface)
;.equ			= 4	(miso stk200 interface)
;.equ			= 3	(mosi stk200 interface)
.equ	AnFET		= 2
.equ	BnFET		= 1
.equ	CnFET		= 0

.equ	INIT_PB		= 0
.equ	DIR_PB		= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

.equ	AnFET_port	= PORTB
.equ	BnFET_port	= PORTB
.equ	CnFET_port	= PORTB

.MACRO AnFET_on
	sbi	PORTB, AnFET
.ENDMACRO
.MACRO AnFET_off
	cbi	PORTB, AnFET
.ENDMACRO
.MACRO BnFET_on
	sbi	PORTB, BnFET
.ENDMACRO
.MACRO BnFET_off
	cbi	PORTB, BnFET
.ENDMACRO
.MACRO CnFET_on
	sbi	PORTB, CnFET
.ENDMACRO
.MACRO CnFET_off
	cbi	PORTB, CnFET
.ENDMACRO

.MACRO AnFET_on_reg
	sbr	@0, 1<<AnFET
.ENDMACRO
.MACRO AnFET_off_reg
	cbr	@0, 1<<AnFET
.ENDMACRO
.MACRO BnFET_on_reg
	sbr	@0, 1<<BnFET
.ENDMACRO
.MACRO BnFET_off_reg
	cbr	@0, 1<<BnFET
.ENDMACRO
.MACRO CnFET_on_reg
	sbr	@0, 1<<CnFET
.ENDMACRO
.MACRO CnFET_off_reg
	cbr	@0, 1<<CnFET
.ENDMACRO

.MACRO nFET_brake
	in	@0, PORTB
	sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTB, @0
.ENDMACRO
.MACRO all_nFETs_off
	in	@0, PORTB
	cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	out	PORTB, @0
.ENDMACRO

.MACRO RED_on
.ENDMACRO
.MACRO RED_off
.ENDMACRO
.MACRO GRN_on
.ENDMACRO
.MACRO GRN_off
.ENDMACRO

 
 .equ	BOOT_LOADER	= 0	; Enable or disable boot loader

.equ	I2C_ADDR	= 0x50	; MK-style I2C address
.equ	MOTOR_ID	= 2	; MK-style I2C motor ID, or UART motor number

.equ	COMP_PWM	= 0	; During PWM off, switch high side on (unsafe on some boards!)
.equ	MOTOR_ADVANCE	= 25	; Degrees of timing advance (0 - 30, 30 meaning no delay) (18)
.equ	MOTOR_BRAKE	= 0	; Enable brake
.equ	MOTOR_REVERSE	= 0	; Reverse normal commutation direction
.equ	RC_PULS_REVERSE	= 0	; Enable RC-car style forward/reverse throttle
.equ	SLOW_THROTTLE	= 0	; Limit maximum throttle jump to try to prevent overcurrent
.equ	BEACON		= 1	; Beep periodically when RC signal is lost
.equ	MOTOR_DEBUG	= 0

.equ	RCP_TOT		= 16	; Number of 65536us periods before considering rc pulse lost
.equ	CPU_MHZ		= F_CPU / 1000000

 ; Minimum PWM on-time (too low and FETs won't turn on, hard starting)
.if !defined(MIN_DUTY)
.equ	MIN_DUTY	= 58 ;(58) 
.endif

; Number of PWM steps (too high and PWM frequency drops into audible range)
.if !defined(POWER_RANGE)
.equ	POWER_RANGE	= 400 + MIN_DUTY; (800)
.endif

.equ	MAX_POWER	= (POWER_RANGE-1)
.equ	PWR_MIN_START	= (POWER_RANGE/6) ; Power limit while starting (to start)
.equ	PWR_MAX_START	= (POWER_RANGE/4) ; Power limit while starting (if still not running)
.equ	PWR_MAX_RPM1	= (POWER_RANGE/4) ; Power limit when running slower than TIMING_RANGE1
.equ	PWR_MAX_RPM2	= (POWER_RANGE/2) ; Power limit when running slower than TIMING_RANGE2

.equ	BRAKE_POWER	= MAX_POWER*2/3	; Brake force is exponential, so start fairly high
.equ	BRAKE_SPEED	= 3		; Speed to reach MAX_POWER, 0 (slowest) - 8 (fastest)

/*.equ	TIMING_MIN	= 0x8000 ; 8192us per commutation
.equ	TIMING_RANGE1	= 0x4000 ; 4096us per commutation
.equ	TIMING_RANGE2	= 0x2000 ; 2048us per commutation
.equ	TIMING_MAX	= 0x00e0 ; 56us per commutation*/

.equ	TIMING_MIN	= 1024*2 ; 1024us per commutation (1024x4)
.equ	TIMING_RANGE1	= 4096*1 ; 4096us per commutation (4096x4)
.equ	TIMING_RANGE2	= 2048*1 ; 2048us per commutation (2048x4)
.equ	TIMING_MAX	= 56*2 ; 56us per commutation (56x4)

/*.equ	TIMING_MIN	= 1000*4 ; 8192us per commutation
.equ	TIMING_RANGE1	= 5096*4 ; 4096us per commutation
.equ	TIMING_RANGE2	= 248*4 ; 2048us per commutation
.equ	TIMING_MAX	= 100 ; 56us per commutation*/


.equ	timeoutSTART = 15000 ; 48ms per commutation (15000
.equ	timeoutMIN	 = 100	; 36ms per commutation (36000)