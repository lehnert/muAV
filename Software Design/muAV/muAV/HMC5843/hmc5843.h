#ifdef __cplusplus
extern "C" {
	#endif
		
	#ifndef _HMC5843_H
	#define _HMC5843_H
	
	#include <avr/io.h>

	#define HMC5843_W	0x3C
	#define HMC5843_R	0x3D

	//#define FALSE	0
	//#define TRUE	-1


	//Function definitions

	int16_t read_hmc5843(char reg_adr);
    int16_t read_hmc5883(char reg_adr, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);
	void init_hmc5843(void);

	#endif
	
#ifdef __cplusplus
}
#endif
