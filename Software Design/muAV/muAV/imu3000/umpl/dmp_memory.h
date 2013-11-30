/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

#ifndef __INV_DMP_MEMORY_H__
#define __INV_DMP_MEMORY_H__

#ifdef CONFIG_MPU_SENSORS_MPU3050
#define DMP_CODE_SIZE (1024)
#else
#define DMP_CODE_SIZE (1929)
#endif

extern const unsigned char dmpMemory[];

#endif // __INV_DMP_MEMORY_H__

