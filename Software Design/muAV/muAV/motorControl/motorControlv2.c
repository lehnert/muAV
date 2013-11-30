
#include "motorControl.h"
#include "global.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>



void init_motors()
{
	OCR1A1 = 0;
	OCR1B1 = 0;
	OCR1A2 = 0;
	OCR1B2 = 0;
	
	//setup reset line
	SET_DDR_PIN_OUTPUT(DDRB, PIN0);
	//set motor output pins
	DDRB |= (1<<PINB1)|(1<<PINB2);
	DDRD |= (1<<PIND5)|(1<<PIND6);
		
	configure_motor(&(motors[0]),0,PORTB,PINB1,M1_STARTUP_OFFSET);
	configure_motor(&(motors[1]),1,PORTB,PINB2,M2_STARTUP_OFFSET);
	configure_motor(&(motors[2]),2,PORTD,PIND5,M3_STARTUP_OFFSET);
	configure_motor(&(motors[3]),3,PORTD,PIND6,M4_STARTUP_OFFSET);
		
	initPWM();	
		
	calibrate_motors();	
		
}

void configure_motor(volatile BLMOTOR_t *motor, uint8_t channel, uint8_t outputPort, uint8_t outputPin, uint8_t pwmOffset)
{
	motor->channel = channel;
	motor->Port = outputPort;
	motor->Pin = outputPin;
	motor->offset = pwmOffset;
	motor->velocity = 0;
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
		writePWMduty(MAX_DUTY_TICKS,&(motors[i]));
	}
	
	_delay_ms(10);
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
		writePWMduty(motors[i].offset,&(motors[i]));
	}
}

void initPWM(void){
	
	//setup PWM timers 0 and 1
	TCNT1 = 0;
	TOGGLE_CHANNEL = TRUE;
	
	//init PWM Duty to 0 (1ms)
	for(int i = 0; i < nMOTORS; i++) {
		writePWMduty(0, &(motors[i]));
	}		
	
	//Clear PWM lines
	PORTD &= ~((1 << PIND5) | (1 << PIND6));
	PORTB &= ~((1 << PINB1) | (1 << PINB2));
	
	//Timer 1 mode = CTC ICR top 14bit mode (non inverted) 8 prescaler
	TCCR1A |= (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0)  | (0 << WGM11) | (0 << WGM10);
	TCCR1B |= (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10); //starts timer
	TIMSK1 |= (1 << ICIE1) | (1 << OCIE1B) | (1 << OCIE1A);
	//1ms period = 16MHz/1KHz = 16000
	ICR1 = MAX_DUTY_TICKS;
	sei();

}

void writePWMduty(uint16_t duty, volatile BLMOTOR_t *motor){
	
	motor->velocity = duty;
	
	//map 0-100 input to 132-232 (1ms - 1.86ms) pulse
	//steps of 8us per tick
	/*if(duty > 100) {
		duty = 232;
	} else if(duty < 0) { 
		duty = 132;
	}else{
		duty += 132;
	}*/
	
	if(duty >= MAX_DUTY_TICKS - 100) {
		duty = MAX_DUTY_TICKS - 100;
	} else if(duty < 275) {
		duty = 275;
	}	
	
		
	switch (motor->channel)
	{
		case 0: //PINB2
			OCR1B1 = duty;
			break;
		case 1: //PINB1
			OCR1A1 = duty;
			break;
		case 2: //PIND5
			OCR1B2 = duty;
			break;
		case 3: //PIND6
			OCR1A2 = duty;
			break;
	}
}

//Alternate between PWM channels 1&2 <-> 3&4
ISR(TIMER1_CAPT_vect){
	if(TOGGLE_CHANNEL){
		PORTD |= (1 << PIND5) | (1 << PIND6);	
		
		//Compare match for motors 1 and 2
		OCR1A = OCR1A1;
		OCR1B = OCR1B1;
		TOGGLE_CHANNEL = FALSE;
	}else{
		PORTB |= (1 << PINB1) | (1 << PINB2);
		//Compare match for motors 3 and 4
		OCR1A = OCR1A2;
		OCR1B = OCR1B2;
		TOGGLE_CHANNEL = TRUE;
	}
}

ISR(TIMER1_COMPA_vect){
	//Clear PWM A Line for current channel
	if(TOGGLE_CHANNEL){
		PORTD &= ~(1 << PIND5);
	}else{
		PORTB &= ~(1 << PINB1);
	}	
}

ISR(TIMER1_COMPB_vect){
	//Clear PWM B Line for current channel
	if(TOGGLE_CHANNEL){
		PORTD &= ~(1 << PIND6);
	}else{
		PORTB &= ~(1 << PINB2);
	}
}



/*ISR(TIMER0_OVF_vect){
	//handle start and end conditions (12 bit) (4096 steps or 16*256 overflows)
	if(TIMER0_OVF_COUNT == 16) {
		PORTD |= (1 << PIND5) | (1 << PIND6); // SET PWM line high for start of period
		TIMER0_OVF_COUNT = 0;
	}else{
		TIMER0_OVF_COUNT++;
	}				
	
	//check if within correct overflow cycle and enable compare match pin else disable
	if (OCR0A_H == TIMER0_OVF_COUNT) { 
		(TCCR0A |= (1 << COM0A1) | (0 << COM0A0));
	} else {
		(TCCR0A |= (0 << COM0A1) | (0 << COM0A0));
	}
    if (OCR0B_H == TIMER0_OVF_COUNT) {
		 (TCCR0B |= (1 << COM0B1) | (0 << COM0B0));
	} else {
		(TCCR0B |= (0 << COM0B1) | (0 << COM0B0));
	}	
}*/