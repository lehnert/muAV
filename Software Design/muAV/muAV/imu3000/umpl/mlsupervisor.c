/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

/******************************************************************************
 *
 * $Id: mlsupervisor.c 5708 2011-06-28 21:15:31Z kkeal $
 *
 *****************************************************************************/

/**
 *  @defgroup   ML_SUPERVISOR
 *  @brief      Basic sensor fusion supervisor functionalities.
 *
 *  @{
 *      @file   mlsupervisor.c
 *      @brief  Basic sensor fusion supervisor functionalities.
 */

#include "ml.h"
#include "mldl.h"
#include "mldl_cfg.h"
#include "mltypes.h"
#include "mlinclude.h"
//#include "compass.h"
//#include "pressure.h"
#include "dmpKey.h"
#include "dmpDefault.h"
#include "mlstates.h"
#include "mlFIFO.h"
#include "mlFIFOHW.h"
#include "mlMathFunc.h"
#include "mlsupervisor.h"
#include "mlmath.h"
//#include "compass_supervisor.h"
#include "mlsl.h"
#include "mlos.h"

static unsigned long lastCompassTime = 0;
static unsigned long polltime = 0;
static int compassCalStableCount = 0;
static int compassCalCount = 0;

#define SUPERVISOR_DEBUG 0

struct inv_supervisor_cb_obj ml_supervisor_cb = { 0 };

/**
 *  @brief  This initializes all variables that should be reset on
 */
void inv_init_sensor_fusion_supervisor(void)
{
    lastCompassTime = 0;
    polltime = 0;
    inv_obj.lite_fusion->acc_state = SF_STARTUP_SETTLE;
    compassCalStableCount = 0;
    compassCalCount = 0;

    if (ml_supervisor_cb.supervisor_reset_func != NULL) {
        ml_supervisor_cb.supervisor_reset_func();
    }
}

/**
 *  @}
 */
