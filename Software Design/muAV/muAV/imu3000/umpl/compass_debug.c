/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

#include "compass_debug.h"
#include "packet.h"

static int debug_long(char type, long val);

int debug_rawmag(long * raw)
{
    int result;
    result = debug_long(PACKET_TYPE_RAWMAG_X,raw[0]);
    if (result) 
        return result;
    result = debug_long(PACKET_TYPE_RAWMAG_Y,raw[1]);
    if (result) 
        return result;
    result = debug_long(PACKET_TYPE_RAWMAG_Z,raw[2]);
    return result;
}

int debug_bias(long * bias)
{
    int result;
    result = debug_long(PACKET_TYPE_BIAS_X,bias[0]);
    if (result) 
        return result;
    result = debug_long(PACKET_TYPE_BIAS_Y,bias[1]);
    if (result) 
        return result;
    result = debug_long(PACKET_TYPE_BIAS_Z,bias[2]);
    return result;
}

static int debug_long(char type, long val)
{
    int i;
    unsigned char payload[10];
    payload[0] = (unsigned char) ((val >> 0) & 0xff);
    payload[1] = (unsigned char) ((val >> 8) & 0xff);
    payload[2] = (unsigned char) ((val >> 16) & 0xff);
    payload[3] = (unsigned char) ((val >> 24) & 0xff);
    for (i = 4; i < 10; i++)
        payload[i] = 0;
    return sendPacket(type,payload); 
}



