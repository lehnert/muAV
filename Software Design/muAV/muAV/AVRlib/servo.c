/*! \file servo.c \brief Interrupt-driven RC Servo function library. */
//*****************************************************************************
//
// File Name	: 'servo.c'
// Title		: Interrupt-driven RC Servo function library
// Author		: Pascal Stang - Copyright (C) 2002
// Created		: 7/31/2002
// Revised		: 8/02/2002
// Version		: 1.0
// Target MCU	: Atmel AVR Series
// Editor Tabs	: 4
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************
	
#ifndef WIN32
	#include <avr/io.h>
#endif

#include "global.h"
#include "servo.h"
#include <avr/interrupt.h>

// Program ROM constants

// Global variables
// servo channel registers
uint16_t ServoPosTics;
uint8_t ServoChannel;
ServoChannelType ServoChannels[SERVO_NUM_CHANNELS];

// functions

//! initializes software PWM system
void servoInit(void)
{
	uint8_t channel;
	// disble the timer1 output compare A interrupt
	cbi(TIMSK1, OCIE1A);
	// set the prescaler for timer1
	timer1SetPrescaler(TIMER_CLK_DIV256);
	// attach the software PWM service routine to timer1 output compare A
	//timerAttach(TIMER1OUTCOMPAREA_INT, servoService);
	// enable and clear channels
	for(channel=0; channel<SERVO_NUM_CHANNELS; channel++)
	{
		// set minimum position as default
		ServoChannels[channel].duty = SERVO_MIN;
	}
	// set PosTics
	ServoPosTics = 0;
	// set PeriodTics
	#define ServoPeriodTics (SERVO_MAX*9)
	// set initial interrupt time
	uint16_t OCValue;
	// read in current value of output compare register OCR1A
	OCValue =  OCR1A;
	
	// increment OCR1A value by nextTics
	OCValue += ServoPeriodTics; 
	
	// set future output compare time to this new value
	OCR1A = OCValue;
	
	// enable the timer1 output compare A interrupt
	sbi(TIMSK1, OCIE1A);
}

//! turns off software PWM system
void servoOff(void)
{
	// disable the timer1 output compare A interrupt
	cbi(TIMSK1, OCIE1A);
	// detach the service routine
	timerDetach(TIMER1OUTCOMPAREA_INT);
}

//! set port and I/O pin for channel
void servoSetChannelIO(uint8_t channel, uint8_t port, uint8_t pin)
{
	ServoChannels[channel].port = port;
	ServoChannels[channel].pin = (1<<(pin&0x07));
}

//! set servo position on channel
void servoSetPosition(uint8_t channel, uint8_t position)
{
	// input should be between 0 and SERVO_POSITION_MAX
	uint16_t pos_scaled;
	// calculate scaled position
	pos_scaled = ((uint16_t)position*(SERVO_MAX-SERVO_MIN)/SERVO_POSITION_MAX)+SERVO_MIN;
	// set position
	servoSetPositionRaw(channel, pos_scaled);
}

//! get servo position on channel
uint8_t servoGetPosition(uint8_t channel)
{
	return (uint8_t)( ((servoGetPositionRaw(channel)-SERVO_MIN)*SERVO_POSITION_MAX)/(SERVO_MAX-SERVO_MIN) );
}

//! set servo position on channel (raw unscaled format)
void servoSetPositionRaw(uint8_t channel, uint16_t position)
{
	// bind to limits
	position = MAX(position, SERVO_MIN);
	position = MIN(position, SERVO_MAX);
	// set position
	ServoChannels[channel].duty = position;
}

//! get servo position on channel (raw unscaled format)
uint16_t servoGetPositionRaw(uint8_t channel)
{
	return ServoChannels[channel].duty;
}

ISR(TIMER1_COMPA_vect){
//void inline servoService(void)
//{
	uint16_t nextTics;

	if(ServoChannel < SERVO_NUM_CHANNELS)
	{
		// turn off current channel
		outb(_SFR_IO8(ServoChannels[ServoChannel].port), inb(_SFR_IO8(ServoChannels[ServoChannel].port)) & ~(ServoChannels[ServoChannel].pin));
	}
	
	// next channel
	ServoChannel++;

	if(ServoChannel != SERVO_NUM_CHANNELS)
	{
		// loop to channel 0 if needed
		if(ServoChannel > SERVO_NUM_CHANNELS)	ServoChannel = 0;
		// turn on new channel
		outb(_SFR_IO8(ServoChannels[ServoChannel].port), inb(_SFR_IO8(ServoChannels[ServoChannel].port)) | (ServoChannels[ServoChannel].pin));
		// schedule turn off time
		nextTics = ServoChannels[ServoChannel].duty;
	}
	else //(Channel == SERVO_NUM_CHANNELS)
	{
		// ***we could save time by precalculating this
		// schedule end-of-period
		nextTics = ServoPeriodTics-ServoPosTics;
	}
	// read in current value of output compare register OCR1A
	// schedule next interrupt
	uint16_t OCValue = OCR1A;
	// increment OCR1A value by nextTics
	OCValue += nextTics;
	// set future output compare time to this new value
	OCR1A = OCValue;
	// set our new tic position
	ServoPosTics += nextTics;
	if(ServoPosTics >= ServoPeriodTics) ServoPosTics = 0;
}

