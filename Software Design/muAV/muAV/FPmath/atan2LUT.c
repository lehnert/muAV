
#include <avr/io.h>
#include "atan2LUT.h"
#include "atanLUT.h"


// QDIV stands for the fixed-point division method most appropriate for 
// your system. Modify where appropriate. 
// This would be for NDS. 

/*static inline int32_t QDIV(int32_t num, int32_t den, const int32_t bits) 
{     
	while(REG_DIVCNT & DIV_BUSY);     
	REG_DIVCNT = DIV_64_32;     
	REG_DIV_NUMER = ((int64)num)<bits;
	REG_DIV_DENOM = den;
	
	while(REG_DIVCNT & DIV_BUSY);
	
	return (REG_DIV_RESULT_L);
}*/
// Arctangens LUT. Interval: [0, 1] (one=128); PI=0x20000
// Some constants for dealing with atanLUT.
#define FP_PI_SHIFT (16)
//#define FP_PI (1 << FP_PI_SHIFT)
//#define FP_PI_2 (1 << (FP_PI_SHIFT-1))

#define ATAN_ONE (4096)
#define ATAN_FP (12)

//static const uint32_t ATANLUT_INDEX = ATAN_ONE/256, ATANLUT_STRIDE_SHIFT= 5;
static const int32_t FP_PI = 65536, FP_PI_2 = 32768;


// Get the octant a coordinate pair is in.
#define OCTANTIFY(_x, _y, _o)   do {\
int32_t _t; _o= 0;\
	if(_y<  0)  {            _x= -_x;   _y= -_y; _o += 4; }\
	if(_x<= 0)  { _t= _x;    _x=  _y;   _y= -_t; _o += 2; }\
	if(_x<=_y)  { _t= _y-_x; _x= _x+_y; _y=  _t; _o += 1; }\
} while(0);



// Basic lookup for atan2. Returns [0,2pi), where pi ~ 0x4000. 

int32_t atan2Lookup(int32_t y, int32_t x) 
{     
	
	//standard atan2 cases
	if(x==0){
		if(y==0) return 0;
		
		if(y>0) {
			return FP_PI_2;
		}else{
			return -FP_PI_2;
		}
	}
	
	//return (y*256)/x;
	
	//octants 1-4 
	if(y > 0) {
		//octants 1 & 2
		if(x > 0) {
			if(y > x){
				return (FP_PI_2 -(int32_t)pgm_read_word(&atanLUT[GETINDEX(x,y)])); 
			}else{
				return ((int32_t)pgm_read_word(&atanLUT[GETINDEX(y,x)]));
			}				
		}else{
		//octants 3 & 4
			if(y > -x){
				return (FP_PI_2 +(int32_t)pgm_read_word(&atanLUT[GETINDEX(-x,y)]));
			}else{
				return (FP_PI -(int32_t)pgm_read_word(&atanLUT[GETINDEX(y,-x)]));
			}
		}
	//octants 5-8			
	}else{
		//octants 7 & 8
		if(x > 0) {
			if(-y > x){ //7
				return ((int32_t)pgm_read_word(&atanLUT[GETINDEX(x,-y)]) - FP_PI_2);
			}else{  //8
				return (-1*(int32_t)pgm_read_word(&atanLUT[GETINDEX(-y,x)]));
			}
		//octants 5 & 6	
		}else{
			if(y < x){
				return (-1*(int32_t)pgm_read_word(&atanLUT[GETINDEX(-x,-y)]) - FP_PI_2);
			}else{
				return ((int32_t)pgm_read_word(&atanLUT[GETINDEX(-y,-x)]) - FP_PI);
			}
			
		}
	}

}

// Basic lookup+linear interpolation for atan2. 
// Returns [0,2pi), where pi ~ 0x4000. 
/*uint32_t atan2Lerp(int32_t x, int32_t y) {    
	 
	 if(y==0)    return (x>=0 ? 0 : BRAD_PI);     
	 int32_t phi;     
	 uint32_t t, fa, fb, h;  
	    
	 OCTANTIFY(x, y, phi);    
	  phi *= BRAD_PI/4;     
	  t= fix16_div(y, x);    
	  h= t % ATANLUT_STRIDE;     
	  fa= atanLUT[t/ATANLUT_STRIDE  ];     
	  fb= atanLUT[t/ATANLUT_STRIDE+1];     
	  return phi + ( fa + ((fb-fa)*h >> ATANLUT_STRIDE_SHIFT) )/8;
} */

