#ifndef _ADXL345_H_
#define _ADXL345_H_

#include <avr/io.h>	

#define ADXL345_W			(0xA6)
#define ADXL345_R			(0xA7)

#define ADXL345_ADDR		(0x53)

#define ADXL345_SCALE_SHIFT		(8)

void read_adxl345(int16_t *x, int16_t* y, int16_t *z);
void init_adxl345(void);


typedef enum {
		ADXL345_DEVID = 0,		/* 00 0x00 Device ID*/
		ADXL345_01_RSVD,        /* 01 0x01 */
		ADXL345_02_RSVD,        /* 02 0x02 */
		ADXL345_03_RSVD,        /* 03 0x03 */
		ADXL345_04_RSVD,        /* 04 0x04 */
		ADXL345_05_RSVD,        /* 05 0x05 */
		ADXL345_06_RSVD,        /* 06 0x06 */
		ADXL345_07_RSVD,        /* 07 0x07 */
		ADXL345_08_RSVD,        /* 08 0x08 */
		ADXL345_09_RSVD,        /* 09 0x09 */
		ADXL345_0A_RSVD,        /* 10 0x0a */
		ADXL345_0B_RSVD,        /* 11 0x0b */
		ADXL345_0C_RSVD,        /* 12 0x0c */
		ADXL345_0D_RSVD,        /* 13 0x0d */
		ADXL345_0E_RSVD,        /* 14 0x0e */
		ADXL345_0F_RSVD,        /* 15 0x0f */
		ADXL345_10_RSVD,        /* 16 0x10 */
		ADXL345_11_RSVD,        /* 17 0x11 */
		ADXL345_12_RSVD,        /* 18 0x12 */
		ADXL345_13_RSVD,        /* 19 0x13 */
		ADXL345_14_RSVD,        /* 20 0x14 */
		ADXL345_15_RSVD,        /* 21 0x15 */
		ADXL345_16_RSVD,        /* 22 0x16 */
		ADXL345_17_RSVD,        /* 23 0x17 */
		ADXL345_18_RSVD,        /* 24 0x18 */
		ADXL345_19_RSVD,        /* 25 0x19 */
		ADXL345_1A_RSVD,        /* 26 0x1a */
		ADXL345_1B_RSVD,        /* 27 0x1b */
		ADXL345_1C_RSVD,        /* 28 0x1c */
		ADXL345_THRESH_TAP,		/* 29 0x1d Tap threshold*/
		ADXL345_OFSX,			/* 30 0x1e X-axis offset*/
		ADXL345_OFSY,			/* 31 0x1f Y-axis offset*/
		ADXL345_OFSZ,			/* 32 0x20 Z-axis offset*/
		ADXL345_DUR,			/* 33 0x21 Tap duration*/
		ADXL345_vLatent,		/* 34 0x22 Tap latency*/
		ADXL345_Window,			/* 35 0x23 Tap window*/
		ADXL345_THRESH_ACT,		/* 36 0x24 Activity threshold*/
		ADXL345_THRESH_INACT,	/* 37 0x25 Inactivity threshold*/
		ADXL345_TIME_INACT,		/* 38 0x26 Inactivity time*/
		ADXL345_ACT_INACT_CTL,	/* 39 0x27 Axis enable control for activity and inactivity detection*/
		ADXL345_THRESH_FF,		/* 40 0x28 Free-fall threshold*/
		ADXL345_TIME_FF,		/* 41 0x29 Free-fall time*/
		ADXL345_TAP_AXES,		/* 42 0x2a Axis control for single tap/double tap*/ 
		ADXL345_ACT_TAP_STATUS,	/* 43 0x2b Source of single tap/double tap*/
		ADXL345_BW_RATE,		/* 44 0x2c Data rate and power mode control*/
		ADXL345_POWER_CTL,		/* 45 0x2d Power-saving features control*/
		ADXL345_INT_ENABLE,		/* 46 0x2e Interrupt enable control*/
		ADXL345_INT_MAP,		/* 47 0x2f Interrupt mapping control*/
		ADXL345_INT_SOURCE,		/* 48 0x30 Source of interrupts*/
		ADXL345_DATA_FORMAT,	/* 49 0x31 Data format control*/
		ADXL345_DATAX0,			/* 50 0x32 X-Axis Data 0*/
		ADXL345_DATAX1,			/* 51 0x33 X-Axis Data 1*/
		ADXL345_DATAY0,			/* 52 0x34 Y-Axis Data 0*/
		ADXL345_DATAY1,			/* 53 0x35 Y-Axis Data 1*/
		ADXL345_DATAZ0,			/* 54 0x36 Z-Axis Data 0*/
		ADXL345_DATAZ1,			/* 55 0x37 Z-Axis Data 1*/
		ADXL345_FIFO_CTL,		/* 56 0x38 FIFO control*/
		ADXL345_FIFO_STATUS		/* 57 0x39 FIFO status*/
}adxl345_register_t;


/* ADXL_POWER_CTRL (0x2d) */
//#define WAKEUP		(0x02)
//#define SLEEP		(0x04)
//#define MEASURE		(0x08)
//#define AUTO_SLEEP	(0x10)
//#define LINK		(0x20)

//Power Control Register Bits
#define WU_0		(1<<0)	//Wake Up Mode - Bit 0
#define	WU_1		(1<<1)	//Wake Up mode - Bit 1
#define SLEEP		(1<<2)	//Sleep Mode
#define	MEASURE		(1<<3)	//Measurement Mode
#define AUTO_SLP	(1<<4)	//Auto Sleep Mode bit
#define LINK		(1<<5)	//Link bit

//Interrupt Enable/Interrupt Map/Interrupt Source Register Bits
#define	OVERRUN		(1<<0)
#define	WATERMARK	(1<<1)
#define FREE_FALL	(1<<2)
#define	INACTIVITY	(1<<3)
#define	ACTIVITY	(1<<4)
#define DOUBLE_TAP	(1<<5)
#define	SINGLE_TAP	(1<<6)
#define	DATA_READY	(1<<7)

//Data Format Bits
#define RANGE_0		(1<<0)
#define	RANGE_1		(1<<1)
#define JUSTIFY		(1<<2)
#define	FULL_RES	(1<<3)

#define	INT_INVERT	(1<<5)
#define	SPI			(1<<6)
#define	SELF_TEST	(1<<7)

//sampling rate (bandwitdh = rate/2)
#define ACCEL_3200HZ	0x0F
#define ACCEL_1600HZ	0x0E
#define ACCEL_800HZ		0x0D
#define ACCEL_400HZ		0x0C
#define ACCEL_200HZ		0x0B
#define ACCEL_100HZ		0x0A
#define ACCEL_50HZ		0x09

#endif