/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id:$
 *
 *****************************************************************************/

#ifndef __INV_TEMP_COMP_H__
#define __INV_TEMP_COMP_H__

#include "mltypes.h"

/* APIs */
inv_error_t inv_enable_temp_comp(void);
inv_error_t inv_disable_temp_comp(void);
/* Formerly declared in ml.h: */
inv_error_t inv_get_gyro_temp_slope(long *data);
inv_error_t inv_get_gyro_temp_slope_float(float *data);
inv_error_t inv_set_gyro_temp_slope(long *data);
inv_error_t inv_set_gyro_temp_slope_float(float *data);

/* Private APIs */
int   inv_temp_comp_enabled(void);
int   inv_temp_comp_has_slope(void);
int   inv_temp_comp_find_bin(float temp);
void  inv_temp_comp_reset(void);
float inv_get_calibration_temp_difference(void);

#endif // __INV_TEMP_COMP_H__
