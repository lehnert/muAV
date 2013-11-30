
#include "motorControl.h"
#include "global.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>



void init_motors()
{
	
	//setup reset line
	SET_DDR_PIN_OUTPUT(DDRB, PIN0);
	//set motor output pins
	DDRB |= (1<<PINB1)|(1<<PINB2);
	DDRD |= (1<<PIND5)|(1<<PIND6);
		
	configure_motor(&(motors[0]),0,PORTB,PINB1,M1_IDLE_SPEED, M1_OFFSET);
	configure_motor(&(motors[1]),1,PORTB,PINB2,M2_IDLE_SPEED, M2_OFFSET);
	configure_motor(&(motors[2]),2,PORTD,PIND5,M3_IDLE_SPEED, M3_OFFSET);
	configure_motor(&(motors[3]),3,PORTD,PIND6,M4_IDLE_SPEED, M4_OFFSET);
		
	initPWM();	
		
	calibrate_motors();	
		//reset motors
	//MOTORS_OFF;
	
	//Reset the RC pulse to zero to simulate throttle low position
	/*for(int i = 0; i < nMOTORS; i++) {
		//writeVelocity(0,&(motors[i]));
		writePWMduty(0,&(motors[i]));
	}*/
	//writePWMduty(0,&(motors[1]));
	//_delay_ms(50);
	//MOTORS_ON;
	
	
}

void configure_motor(volatile BLMOTOR_t *motor, uint8_t channel, uint8_t outputPort, uint8_t outputPin, uint8_t idle_speed, uint8_t pwmOffset)
{
	motor->channel = channel;
	motor->Port = outputPort;
	motor->Pin = outputPin;
	motor->idle_speed = idle_speed;
	motor->trim = pwmOffset;
	motor->duty = 0;
}	

// The motors can only be calibrated directly after the ESC's have been
// reset otherwise it will put full power to the motors
void calibrate_motors(void){
	
	//turn of control loop interrupt
	DISABLE_CONTROL_LOOP;
	
	//reset motors
	MOTORS_OFF;
	
	//Hold the RC pulse high to simulate high throttle position
	for(int i = 0; i < nMOTORS; i++) {
		//writeVelocity(255,&(motors[i]));
		writePWMduty(MAX_DUTY,&(motors[i]));
	}
	
	_delay_ms(50);
	//turn on motors while high throttle
	MOTORS_ON;
	//wait till the ESC's register high throttle calibration
	_delay_ms(1000);
	
	//Reset the RC pulse to zero to simulate throttle low position
	for(int i = 0; i < nMOTORS; i++) {
		//writeVelocity(0,&(motors[i]));
		writePWMduty(0,&(motors[i]));
	}
	//wait till the ESC's register low throttle calibration
	_delay_ms(1000);
	
	//turn of control loop interrupt
	ENABLE_CONTROL_LOOP;
	
}

void reset_motors(void){
	MOTORS_OFF;
	_delay_ms(100);
	MOTORS_ON;	
}

void start_motors(void)
{
	for(int i = 0; i < nMOTORS; i++) {
		writePWMduty(motors[i].idle_speed,&(motors[i]));
	}
}

void initPWM(void){
	
	//setup PWM timers 0 and 1
	TCNT1 = 0;

	
	//init PWM Duty to 0 (1ms)
	for(int i = 0; i < nMOTORS; i++) {
		writePWMduty(0, &(motors[i]));
	}		
	
	//setup PWM timers 0 and 1
	TCNT0 = 0;
	TCNT1 = 0;
	
	//Timer 0 mode = PWM Phase Correct 8bit mode (non inverted) 32 prescaler
	TCCR0A |= (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0)  | (0 << WGM01) | (1 << WGM00);
	TCCR0B |= (0 << WGM02) | (0 << CS02) | (1 << CS01) | (1 << CS00);
	
	//Timer 1 mode = PWM Phase Correct 8bit mode (non inverted) 32 prescaler
	TCCR1A |= (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0)  | (0 << WGM11) | (1 << WGM10);
	TCCR1B |= (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (1 << CS11) | (01 << CS10);
	
	//init PWM output
	for(int i = 0; i < nMOTORS; i++) writePWMduty(0, &(motors[i]));
	sei();

}

void writePWMduty(uint8_t duty, volatile BLMOTOR_t *motor){
	
	motor->duty = duty;
	
	//scale duty from 0-255 to PWM ticks
	duty = (uint8_t)((uint16_t)duty*(MAX_DUTY_TICKS-MIN_DUTY_TICKS)/MAX_DUTY)+MIN_DUTY_TICKS;
	
	//bind limits
	duty = MAX(duty, MIN_DUTY_TICKS);
	duty = MIN(duty, MAX_DUTY_TICKS);
	
	switch (motor->channel)
	{
		case 0: //PINB2
			OCR1B = duty;
			break;
		case 1: //PINB1
			OCR1A = duty;
			break;
		case 2: //PIND5
			OCR0B = duty;
			break;
		case 3: //PIND6
			OCR0A = duty;
			break;
	}
}
