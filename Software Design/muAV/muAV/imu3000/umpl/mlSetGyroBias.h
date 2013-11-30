/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id$
 *
 *****************************************************************************/

#ifndef INV_SET_GYRO_BIAS__H__
#define INV_SET_GYRO_BIAS__H__

#include "mltypes.h"
#include "ml.h"

#define INV_SGB_NO_MOTION 4
#define INV_SGB_FAST_NO_MOTION 5
#define INV_SGB_TEMP_COMP 6

    inv_error_t inv_enable_set_bias(void);
    inv_error_t inv_disable_set_bias(void);
    inv_error_t inv_set_gyro_bias_in_hw_unit(const short *bias, int mode);
    inv_error_t inv_set_gyro_bias_in_dps(const long *bias, int mode);
    inv_error_t inv_set_gyro_bias_in_dps_float(const float *bias, int mode);
    void inv_convert_bias(const unsigned char *regs, short *bias);
    void inv_set_motion_state(int motion);
	inv_error_t MLSetGyroBiasCB(struct inv_obj_t * inv_obj);

#endif                          // INV_SET_GYRO_BIAS__H__
