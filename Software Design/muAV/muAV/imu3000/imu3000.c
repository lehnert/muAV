#include "imu3000.h"
#include "i2c.h"
#include "global.h"
#include "global_parameters.h"
#include "atan2LUT.h"
#include <math.h>

volatile IMU_t imu;

inline void read_imu(volatile IMU_t *imu)
{
	unsigned char buffer[12];
	
	int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
	int16_t raw_accel_x, raw_accel_y, raw_accel_z;
	int32_t accel_roll, accel_pitch;
	static int16_t raw_accel_x_old = 0, raw_accel_y_old = 0, raw_accel_z_old = 0;
	static int16_t raw_gyro_x_old = 0, raw_gyro_y_old = 0, raw_gyro_z_old = 0;

	
	
	//global_counter = TCNT2;
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W); i2cWaitForComplete();
	i2cSendByte(IMU3000_GYRO_XOUT_H); i2cWaitForComplete();
		
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_R); i2cWaitForComplete();
	
	for(int i = 0; i<11; i++) 
	{
		i2cReceiveByte(TRUE); i2cWaitForComplete();
		buffer[i] =  i2cGetReceivedByte();
	}	
	i2cReceiveByte(FALSE); i2cWaitForComplete();
	buffer[11] = i2cGetReceivedByte();
	i2cSendStop();
	
	raw_gyro_x = ((buffer[0]<<8) | buffer[1]) - imu->gyro_x_offset;
	raw_gyro_y = ((buffer[2]<<8) | buffer[3]) - imu->gyro_y_offset;
	raw_gyro_z = ((buffer[4]<<8) | buffer[5]) - imu->gyro_z_offset;
	
	raw_accel_x = (buffer[7]<<8) | buffer[6];
	raw_accel_y = (buffer[9]<<8) | buffer[8];
	raw_accel_z = (buffer[11]<<8) | buffer[10];
	
	/*imu->raw_accel_x = raw_accel_x;
	imu->raw_accel_y = raw_accel_y;
	imu->raw_accel_z = raw_accel_z;
	
	imu->raw_gyro_x = raw_gyro_x;
	imu->raw_gyro_y = raw_gyro_y;
	imu->raw_gyro_z = raw_gyro_z;*/
	
	//low pass filter over accel and gyro data for filtering vibration from propellers
	raw_accel_x = raw_accel_x_old + ((raw_accel_x  - raw_accel_x_old)/8);
	raw_accel_y = raw_accel_y_old + ((raw_accel_y  - raw_accel_y_old)/8);
	raw_accel_z = raw_accel_z_old + ((raw_accel_z  - raw_accel_z_old)/8);
	
	imu->accel_x = raw_accel_x;
	imu->accel_y = raw_accel_y;
	imu->accel_z = raw_accel_z;
	
	raw_gyro_x = raw_gyro_x_old + ((raw_gyro_x  - raw_gyro_x_old)/4);
	raw_gyro_y = raw_gyro_y_old + ((raw_gyro_y  - raw_gyro_y_old)/4);
	//raw_gyro_z = raw_gyro_z_old + ((raw_gyro_z  - raw_gyro_z_old)/50);
	
	raw_accel_x_old = raw_accel_x;
	raw_accel_y_old = raw_accel_y;
	raw_accel_z_old = raw_accel_z;
		
	raw_gyro_x_old = raw_gyro_x;
	raw_gyro_y_old = raw_gyro_y;
	raw_gyro_z_old = raw_gyro_z;
	
	
	//transformations from xframe to +frame
	imu->gyro_x = (int16_t)ROTATE_X((int32_t)raw_gyro_x,(int32_t)-raw_gyro_y);
	imu->gyro_y = (int16_t)ROTATE_Y((int32_t)raw_gyro_x,(int32_t)-raw_gyro_y);
	imu->gyro_z = raw_gyro_z;
	
	
	accel_roll = atan2Lookup((int32_t)-1*imu->accel_y,(int32_t)imu->accel_z);
	accel_pitch = atan2Lookup((int32_t)imu->accel_x,(int32_t)hypot((double)imu->accel_y,(double)imu->accel_z));

	//transformations from xframe to +frame
	imu->accel_roll = ROTATE_X(accel_roll,accel_pitch);
	imu->accel_pitch = ROTATE_Y(accel_roll,accel_pitch);
	
	
	complimentary_filter(imu);
	
	//global_counter = TCNT2 - global_counter; 
	//store previous value for low pass filter

}



inline void complimentary_filter(volatile IMU_t *imu)
{
	#define alpha 100

	imu->roll += imu->gyro_x + (ACCEL_SCALE*imu->accel_roll - imu->roll)/alpha; 	
	imu->pitch += imu->gyro_y + (ACCEL_SCALE*imu->accel_pitch - imu->pitch)/alpha; 
	imu->yaw += imu->gyro_z;
}

inline void low_pass_filter(volatile IMU_t *imu){
	static int32_t data_roll_old = 0, data_pitch_old = 0;

	//low pass filter the accel data
	//imu->accel_roll_filt = data_roll_old + ((imu->accel_roll - data_roll_old)/20);
	//imu->accel_pitch_filt = data_pitch_old + ((imu->accel_pitch  - data_pitch_old)/20);
	
	//data_roll_old = imu->accel_roll_filt;
	//data_pitch_old = imu->accel_pitch_filt;
}

inline void high_pass_filter(volatile IMU_t *imu)
{
	static int16_t gyro_x_old = 0, gyro_y_old = 0, gyro_z_old = 0;
	static int16_t gyro_x_filt_old = 0, gyro_y_filt_old = 0, gyro_z_filt_old = 0;
	int16_t gyro_data_x, gyro_data_y, gyro_data_z;
	
	gyro_data_x = imu->gyro_x;
	gyro_data_y = imu->gyro_y;
	gyro_data_z = imu->gyro_z;
	
	imu->gyro_x = ((gyro_x_filt_old + imu->gyro_x - gyro_x_old)/10);
	imu->gyro_y = ((gyro_y_filt_old + imu->gyro_y - gyro_y_old)/10);
	imu->gyro_z = ((gyro_z_filt_old + imu->gyro_z - gyro_z_old)/10);
	
	gyro_x_old = gyro_data_x;	gyro_y_old = gyro_data_y;	gyro_z_old = gyro_data_z;
	gyro_x_filt_old = imu->gyro_x;	gyro_y_filt_old = imu->gyro_y;	gyro_z_filt_old = imu->gyro_z;
	
	
}


void init_imu(void)
{

	imu.gyro_x = 0;	imu.gyro_y = 0;	imu.gyro_z = 0;
	imu.accel_x = 0;	imu.accel_y = 0;	imu.accel_z = 0;
	imu.accel_roll = 0;	imu.accel_pitch = 0;
	imu.roll= 0;imu.pitch = 0;imu.yaw = 0;
	
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W);i2cWaitForComplete();
	i2cSendByte(IMU3000_DLPF_FS);i2cWaitForComplete();
	i2cSendByte(DLPF_CFG_256HZ|FS_SEL_500);i2cWaitForComplete();
	i2cSendStop();

	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W);i2cWaitForComplete();
	i2cSendByte(IMU3000_AUX_BURST_ADDR);i2cWaitForComplete();
	i2cSendByte(ADXL345_DATAX0);i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W); i2cWaitForComplete();
	i2cSendByte(IMU3000_AUX_SLV_ADDR); i2cWaitForComplete();
	i2cSendByte(ADXL345_ADDR); i2cWaitForComplete();
	i2cSendStop();

	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W); i2cWaitForComplete();
	i2cSendByte(IMU3000_USER_CTRL); i2cWaitForComplete();
	i2cSendByte(USER_CTRL_AUX_IF_RST); i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(ADXL345_W); i2cWaitForComplete();
	i2cSendByte(ADXL345_POWER_CTL); i2cWaitForComplete();
	i2cSendByte(MEASURE); i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(ADXL345_W); i2cWaitForComplete();
	i2cSendByte(ADXL345_BW_RATE); i2cWaitForComplete();
	i2cSendByte(ACCEL_800HZ); i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W); i2cWaitForComplete();
	i2cSendByte(IMU3000_USER_CTRL); i2cWaitForComplete();
	i2cSendByte(USER_CTRL_AUX_IF_RST|USER_CTRL_AUX_IF_EN); i2cWaitForComplete();				
	i2cSendStop();
	
	calibrate_imu();
}

void calibrate_imu(void){
	
	int32_t raw_gyro_x_offset = 0, raw_gyro_y_offset = 0, raw_gyro_z_offset = 0;
	unsigned char buffer[6];
	
	//calibrate gyro sensor offset
	for(int i = 0; i< 100; i++)
	{
		
		i2cSendStart();	i2cWaitForComplete();
		i2cSendByte(IMU3000_ADDR_W); i2cWaitForComplete();
		i2cSendByte(IMU3000_GYRO_XOUT_H); i2cWaitForComplete();
		
		i2cSendStart();	i2cWaitForComplete();
		i2cSendByte(IMU3000_ADDR_R); i2cWaitForComplete();
		
		for(int j = 0; j<5; j++)
		{
			i2cReceiveByte(TRUE); i2cWaitForComplete();
			buffer[j] =  i2cGetReceivedByte();
		}
		
		i2cReceiveByte(FALSE); i2cWaitForComplete();
		buffer[5] = i2cGetReceivedByte();
		i2cSendStop();
		
		raw_gyro_x_offset += (buffer[0]<<8) | buffer[1];
		raw_gyro_y_offset += (buffer[2]<<8) | buffer[3];
		raw_gyro_z_offset += (buffer[4]<<8) | buffer[5];
	}
	
	imu.gyro_x_offset = (int16_t)(raw_gyro_x_offset/100);
	imu.gyro_y_offset = (int16_t)(raw_gyro_y_offset/100);
	imu.gyro_z_offset = (int16_t)(raw_gyro_z_offset/100);
	
	/*i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W);i2cWaitForComplete();
	i2cSendByte(IMU3000_X_OFFS_USRH); i2cWaitForComplete();
	i2cSendByte(raw_gyro_x_offset>>8); i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W);i2cWaitForComplete();
	i2cSendByte(IMU3000_X_OFFS_USRL);i2cWaitForComplete();
	i2cSendByte(raw_gyro_x_offset);i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W);i2cWaitForComplete();
	i2cSendByte(IMU3000_Y_OFFS_USRH);i2cWaitForComplete();
	i2cSendByte(raw_gyro_y_offset>>8);i2cWaitForComplete();
	i2cSendStop();
	
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W);i2cWaitForComplete();
	i2cSendByte(IMU3000_Y_OFFS_USRL);i2cWaitForComplete();
	i2cSendByte(raw_gyro_y_offset);i2cWaitForComplete();
	i2cSendStop();
		
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W);i2cWaitForComplete();
	i2cSendByte(IMU3000_Z_OFFS_USRH);i2cWaitForComplete();
	i2cSendByte(raw_gyro_z_offset>>8);i2cWaitForComplete();
	i2cSendStop();
			
	i2cSendStart();	i2cWaitForComplete();
	i2cSendByte(IMU3000_ADDR_W);i2cWaitForComplete();
	i2cSendByte(IMU3000_Z_OFFS_USRL);i2cWaitForComplete();
	i2cSendByte(raw_gyro_z_offset);i2cWaitForComplete();
	i2cSendStop();*/
}
