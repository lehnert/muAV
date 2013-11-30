  
#ifndef _IMU3000_H_
#define _IMU3000_H_
	 
  
#include <avr/io.h>	 
#include "adxl345.h"
	  
/* TWI/I2C address (write @ 0xd0 on bus, read @ 0xd1 on bus)
*
* The low bit may be modified by a pin 9 input value or by
* writing to the IMU3000_WHOAMI register.
*/
#define IMU3000_ADDR        (0x68)
#define IMU3000_ADDR_W      (0xD0)
#define IMU3000_ADDR_R      (0xD1)

#define IMU3000_SCALE_x2		(131)

#define DELTA_T_MS				(3)	  
	  
#define TEMP_COUNTS_PER_DEG_C   (280)       /* counts per degree C */
#define TEMP_OFFSET             (-13200)    /* temperature sensor offset */
#define TEMP_REF_DEG            (35)        /* reference temp (degrees C) */

#define ROTATE_X(X,Y)		(7071*(X-Y)/10000)
#define ROTATE_Y(X,Y)		(7071*(X+Y)/10000)

#define BITS_PER_DEG 16380L //45*364
#define ACCEL_SCALE 45 //90 // = 65.5*(Freq = 250)/364
	  
typedef enum {
		IMU3000_WHO_AM_I = 0,   /* 00 0x00 chip ID - I2C address */
		IMU3000_01_RSVD,     /* 01 0x01 */
		IMU3000_02_RSVD,        /* 02 0x02 */
		IMU3000_03_RSVD,        /* 03 0x03 */
		IMU3000_04_RSVD,        /* 04 0x04 */
		IMU3000_05_RSVD,        /* 05 0x05 */
		IMU3000_06_RSVD,        /* 06 0x06 */
		IMU3000_07_RSVD,        /* 07 0x07 */
		IMU3000_08_RSVD,        /* 08 0x08 */
		IMU3000_09_RSVD,        /* 09 0x09 */
		IMU3000_0A_RSVD,        /* 10 0x0a */
		IMU3000_0B_RSVD,        /* 11 0x0b */
		IMU3000_X_OFFS_USRH,    /* 12 0x0c X gyro DC bias offset high byte */
		IMU3000_X_OFFS_USRL,    /* 13 0x0d X gyro DC bias offset high byte */
		IMU3000_Y_OFFS_USRH,    /* 14 0x0e Y gyro DC bias offset high byte */
		IMU3000_Y_OFFS_USRL,    /* 15 0x0f Y gyro DC bias offset low byte */
		IMU3000_Z_OFFS_USRH,    /* 16 0x10 Z gyro DC bias offset high byte */
		IMU3000_Z_OFFS_USRL,    /* 17 0x11 Z gyro DC bias offset low byte */
		IMU3000_FIFO_EN,        /* 18 0x12 FIFO configure & enable */
		IMU3000_AUX_VDDIO,      /* 19 0x13 accelerometer I/O logic levels */
		IMU3000_AUX_SLV_ADDR,   /* 20 0x14 accelerometer I2C address */
		IMU3000_SMPLRT_DIV,     /* 21 0x15 sample rate divider */
		IMU3000_DLPF_FS,        /* 22 0x16 full scale & dig low pass */
		IMU3000_INT_CFG,        /* 23 0x17 interrupt config */
		IMU3000_AUX_BURST_ADDR, /* 24 0x18 accelerometer burst-mode-read */
		IMU3000_19_RSVD,        /* 25 0x19 */
		IMU3000_INT_STATUS,     /* 26 0x1a interrupt status */
		IMU3000_TEMP_OUT_H,     /* 27 0x1b temperature out - MSB */
		IMU3000_TEMP_OUT_L,     /* 28 0x1c temperature out - LSB */
		IMU3000_GYRO_XOUT_H,    /* 29 0x1d gyro X out - MSB */
		IMU3000_GYRO_XOUT_L,    /* 30 0x1e gyro X out - LSB */
		IMU3000_GYRO_YOUT_H,    /* 31 0x1f gyro Y out - MSB */
		IMU3000_GYRO_YOUT_L,    /* 32 0x20 gyro Y out - LSB */
		IMU3000_GYRO_ZOUT_H,    /* 33 0x21 gyro Z out - MSB */
		IMU3000_GYRO_ZOUT_L,    /* 34 0x22 gyro Z out - LSB */
		IMU3000_AUX_XOUT_H,		/* 35 0x23 */
		IMU3000_AUX_XOUT_L,		/* 36 0x24 */
		IMU3000_AUX_YOUT_H,		/* 37 0x25 */
		IMU3000_AUX_YOUT_L,		/* 38 0x26 */
		IMU3000_AUX_ZOUT_H,		/* 39 0x27 */
		IMU3000_AUX_ZOUT_L,		/* 40 0x28 */
		IMU3000_29_RSVD,        /* 41 0x29 */
		IMU3000_2A_RSVD,        /* 42 0x2a */
		IMU3000_2B_RSVD,        /* 43 0x2b */
		IMU3000_2C_RSVD,        /* 44 0x2c */
		IMU3000_2D_RSVD,        /* 45 0x2d */
		IMU3000_2E_RSVD,        /* 46 0x2e */
		IMU3000_2F_RSVD,        /* 47 0x2f */
		IMU3000_30_RSVD,        /* 48 0x30 */
		IMU3000_31_RSVD,        /* 49 0x31 */
		IMU3000_32_RSVD,        /* 50 0x32 */
		IMU3000_33_RSVD,        /* 51 0x33 */
		IMU3000_34_RSVD,        /* 52 0x34 */
		IMU3000_DMP_CFG_1,      /* 53 0x35 */
		IMU3000_DMP_CFG_2,      /* 54 0x36 */
		IMU3000_BANK_SEL,       /* 55 0x37 */
		IMU3000_MEM_START_ADDR, /* 56 0x38 */
		IMU3000_MEM_R_W,        /* 57 0x39 */
		IMU3000_FIFO_COUNTH,    /* 58 0x3a FIFO data count high bits */
		IMU3000_FIFO_COUNTL,    /* 59 0x3b FIFO data count low byte */
		IMU3000_FIFO_R_W,       /* 60 0x3c FIFO data address (head) */
		IMU3000_USER_CTRL,      /* 61 0x3d reset / enable device functions */
		IMU3000_PWR_MGM,        /* 62 0x3e power management */
		IMU3000_3F_RSVD,        /* 63 0x3f */
		IMU3000_REGISTER_COUNT  /* 64 0x40 */
} IMU3000_register_t;
	  
/* IMU3000_WHOAMI (0x00), defaults to the IMU3000 I2C bus address */
	  
#define IMU3000_ID_VAL          (IMU3000_TWI_ADDR)
	  
/* IMU3000_FIFO_EN (0x12) */

typedef enum {	  
	FIFO_EN_TEMP_OUT        =(0x80),      /* insert temperature data in FIFO */
	FIFO_EN_GYRO_XOUT       =(0x40),      /* insert X gyro data in FIFO */
	FIFO_EN_GYRO_YOUT       =(0x20),      /* insert Y gyro data in FIFO */
	FIFO_EN_GYRO_ZOUT       =(0x10),      /* insert Z gyro data in FIFO */
	FIFO_EN_AUX_XOUT        =(0x08),      /* insert X accel data in FIFO */
	FIFO_EN_AUX_YOUT        =(0x04),      /* insert Y accel data in FIFO */
	FIFO_EN_AUX_ZOUT        =(0x02),      /* insert Z accel data in FIFO */
	FIFO_EN_FOOTER          =(0x01)      /* insert last word for FIFO read */
} FIFO_EN_t;	 
	  
/* IMU3000_AUX_VDDIO (0x13) */
#define AUX_VDDIO_VDD           (0x04)      /* VDD I/O logic for aux. I2C bus */
#define AUX_VDDIO_VLOGIC        (0x00)      /* VLOGIC I/O logic for aux. I2C bus */
	  
/* IMU3000_AUX_ADDR (0x14) */
#define AUX_ADDR_CLKOUTEN       (0x80)      /* enable reference clock at CLKOUT */
	  
/* IMU3000_DLPF_FS (0x16) */
typedef enum {		  
	 DLPF_CFG_FIELD          =(0x07),      /* low pass bandwidth (3 bits) */
	 DLPF_CFG_256HZ          =(0x00),      /* 256Hz  low pass b/w */
	 DLPF_CFG_188HZ          =(0x01),      /* 188Hz  low pass b/w */
	 DLPF_CFG_98HZ           =(0x02),      /* 98Hz   low pass b/w */
	 DLPF_CFG_42HZ           =(0x03),      /* 42Hz   low pass b/w */
	 DLPF_CFG_20HZ           =(0x04),      /* 20Hz   low pass b/w */
	 DLPF_CFG_10HZ           =(0x05),      /* 10Hz   low pass b/w */
	 DLPF_CFG_5HZ            =(0x06),      /* 5Hz    low pass b/w */
	 DLPF_CFG_2100HZ         =(0x07),      /* 2.1KHz low pass b/w */
	 FS_SEL_FIELD            =(0x18),      /* full scale range (2 bits) */
	 FS_SEL_250              =(0x00),      /* +/-250  deg/sec full scale */
	 FS_SEL_500              =(0x08),      /* +/-500  deg/sec full scale */
	 FS_SEL_1000             =(0x10),      /* +/-1000 deg/sec full scale */
	 FS_SEL_2000             =(0x18)      /* +/-2000 deg/sec full scale */
} DLPF_FS_t;

/* IMU3000_INT_CFG (0x17) */
typedef enum {	  
	 INT_CFG_RAW_RDY_EN      =(0x01),      /* data available interrupt */
	 INT_CFG_DMP_DONE_EN     =(0x02),      /* DMP done interrupt */
	 INT_CFG_IMU_RDY_EN      =(0x04),      /* device ready interrupt */
	 INT_CFG_I2C_MST_ERR_EN  =(0x08),      /* accel NACK on secondary I2C bus */
	 INT_CFG_ANYRD_2CLEAR    =(0x10),      /* clr latch on any reg read */
	 INT_CFG_LATCH_INT_EN    =(0x20),      /* latch until int cleared */
	 INT_CFG_OPEN            =(0x40),      /* open drain for int output */
	 INT_CFG_ACTL            =(0x80)      /* int is active low */
} INT_CFG_t;	  
	  
/* IMU3000_INT_STATUS (0x1a) */
typedef enum {	  
 INT_STATUS_RAW_DATA_RDY =(0x01),      /* raw data is ready */
 INT_STATUS_DMP_DONE     =(0x02),      /* digital motion processor done */
 INT_STATUS_IMU_RDY      =(0x04),      /* device (PLL) is ready */
 INT_STATUS_I2C_MST_ERR  =(0x08),      /* accel NACK on secondary I2C bus */
 INT_STATUS_FIFO_FULL    =(0x80)      /* FIFO has overflowed */
} INT_STATUS_t;	  

/* IMU3000_USER_CTRL (0x3d) */
typedef enum {	  
 USER_CTRL_GYRO_RST      =(0x01),      /* reset gyro analog & digital funcs */
 USER_CTRL_FIFO_RST      =(0x02),      /* reset / clear the FIFO */
 USER_CTRL_DMP_RST       =(0x04),      /* reset digital motion processor */
 USER_CTRL_AUX_IF_RST    =(0x08),      /* reset auxiliary accelerometer */
 USER_CTRL_AUX_IF_EN     =(0x20),      /* enable auxiliary accelerometer */
 USER_CTRL_FIFO_EN       =(0x40),      /* enable FIFO for sensor data */
 USER_CTRL_DMP_EN        =(0x80)      /* enable digital motion processor */
}  USER_CTRL_t;

/* IMU3000_PWR_MGM (0x3e) */
typedef enum {	  
 PWR_MGM_CLK_SEL_INT     =(0x00),      /* internal oscillator */
 PWR_MGM_CLK_SEL_X       =(0x01),      /* PLL w/ X gyro reference */
 PWR_MGM_CLK_SEL_Y       =(0x02),      /* PLL w/ Y gyro reference */
 PWR_MGM_CLK_SEL_Z       =(0x03),      /* PLL w/ Z gyro reference */
 PWR_MGM_CLK_SEL_EXT_32K =(0x04),      /* PLL w/ external 32.768 KHz */
 PWR_MGM_CLK_SEL_EXT_19M =(0x05),      /* PLL w/ external 19.2 MHz */
 PWR_MGM_STBY_ZG         =(0x08),      /* put Z gyro in standby mode */
 PWR_MGM_STBY_YG         =(0x10),      /* put Y gyro in standby mode */
 PWR_MGM_STBY_XG         =(0x20),      /* put X gyro in standby mode */
 PWR_MGM_SLEEP           =(0x40),      /* enable low power sleep mode */
 PWR_MGM_H_RESET         =(0x80)      /* reset device */
}  PWR_MGM_t;

typedef struct
{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	
	int16_t gyro_x_offset;
	int16_t gyro_y_offset;
	int16_t gyro_z_offset;
	
	/*int16_t raw_accel_x;
	int16_t raw_accel_y;
	int16_t raw_accel_z;
	
	int16_t raw_gyro_x;
	int16_t raw_gyro_y;
	int16_t raw_gyro_z;*/
	
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	
	int32_t accel_roll;
	int32_t accel_pitch;
	
	int32_t roll;
	int32_t pitch;
	int32_t yaw;

} IMU_t;



void read_imu(volatile IMU_t *imu);
//void read_imu(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);
void init_imu();

void calibrate_imu(void);

void high_pass_filter(volatile IMU_t *imu);

void low_pass_filter(volatile IMU_t *imu);  
  
void complimentary_filter(volatile IMU_t *imu);



#endif