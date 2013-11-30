#ifndef _atan2LUT_H
#define _atan2LUT_H

// atan LUT defined from 0 to 1.0 for angles (0 to 1/4*pi) 


// Basic lookup for atan2. Returns [0,2pi), where pi ~ 0x4000.
int32_t atan2Lookup(int32_t x, int32_t y);


#endif