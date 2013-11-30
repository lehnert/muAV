#ifndef _GLOBAL_H
#define _GLOBAL_H

//UMPL configuration flags

#define INV_FEATURE_GYROTC_UTC
#define DMPDEFAULT_UNSORTEDKEYLOOKUP
#define INV_CACHE_DMP 0
#define CONFIG_MPU_SENSORS_MPU3050
#define CONFIG_MPU_SENSORS_ADXL345
#define UMPL_TARGET_AT328
#define BIG_ENDIAN
#define UMPL_DISABLE_STORE_CAL
#define UMPL_DISABLE_LOAD_CAL

#include <avr/io.h>

// global AVRLIB types definitions
#include "avrlibtypes.h"

// global AVRLIB defines
#include "avrlibdefs.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif



#define CYCLES_PER_US ((F_CPU+500000)/1000000)

#define FALSE	0
#define TRUE	-1

#define SET_DDR_OUTPUT(ddr) ((ddr) = 0xFF)
#define SET_DDR_INPUT(ddr) ((ddr) = 0x00)

#define SET_DDR_PIN_OUTPUT(ddr, pin) ((ddr) |= (1 << (pin)))
#define SET_DDR_PIN_INPUT(ddr, pin) ((ddr) &= ~(1 << (pin)))

#define SET_PORT(port) ((port) = 0xFF)
#define CLEAR_PORT(port) ((port) = 0x00)

#define SET_PORT_PIN(port, pin) ((port) |= (1 << (pin)))
#define CLEAR_PORT_PIN(port, pin) ((port) &= ~(1 << (pin)))
#define TOGGLE_PORT_PIN(port,pin) ((port) ^= (1 << (pin)))

#define ENABLE_LED1 SET_DDR_PIN_OUTPUT(DDRB, PIN3)
#define ENABLE_LED2 SET_DDR_PIN_OUTPUT(DDRB, PIN4)

#define LED1_ON		SET_PORT_PIN(PORTB, PIN3)
#define LED1_OFF	CLEAR_PORT_PIN(PORTB, PIN3)
#define LED1_TOGGLE TOGGLE_PORT_PIN(PORTB, PIN3)

#define LED2_ON		SET_PORT_PIN(PORTB, PIN4)
#define LED2_OFF	CLEAR_PORT_PIN(PORTB, PIN4)
#define LED2_TOGGLE TOGGLE_PORT_PIN(PORTB, PIN4)

#define MOTORS_OFF CLEAR_PORT_PIN(PORTB, PIN0)
#define MOTORS_ON	SET_PORT_PIN(PORTB, PIN0)

#define DISABLE_CONTROL_LOOP cbi(TIMSK2,OCIE2A);
#define ENABLE_CONTROL_LOOP sbi(TIMSK2,OCIE2A);

#define DISABLE_MOTOR_CONTROL cbi(TIMSK1, OCIE1A);
#define ENABLE_MOTOR_CONTROL sbi(TIMSK1, OCIE1A);

#define MIN(a,b)			((a<b)?(a):(b))
#define MAX(a,b)			((a>b)?(a):(b))

typedef enum {
	STOP = 0, 	   //stop motors
	START, //no motors running, system initialized
	STARTUP,   //motor open loop and startup sequence
	TAKEOFF,   //takeoff sequence using sensor stabilisation
	HOVER,	   //stabilise system dynamics while maintaining position
	MANEUVER,  //perform a movement command received
	LAND,	   //perform landing sequence
	IDLE,	   //continue running motors

} state_t;



//COMMANDS
#define START_CONTROL 'b'
#define SPIN_UP 'm'
#define PAUSE 'n'
#define CALIBRATE 'c'
#define RESET_MOTORS 'y'

// definitions for remote access
#define ROLL_LEFT 'a'
#define ROLL_RIGHT 'd'
#define PITCH_UP 's'
#define PITCH_DOWN 'w'
#define YAW_CW 'q'
#define YAW_CCW 'e'

#define MOTOR_UP_1 'p'
#define MOTOR_DOWN_1 'l'
	
#define MOTOR_UP_2 'o'
#define MOTOR_DOWN_2 'k'
	
#define MOTOR_UP_3 'i'
#define MOTOR_DOWN_3 'j'
	
#define MOTOR_UP_4 'u'
#define MOTOR_DOWN_4 'h'


#endif