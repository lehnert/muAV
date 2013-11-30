/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 *
 * $Id: umpl-states.c 2011-04-15  phickey $
 *
 *****************************************************************************/

/** 
 *  @defgroup UMPL 
 *  @brief  uMPL Layer.
 *          umpl-states.c provides a mechanism to control the uMPL state machine.
 *          It controls the flow from one state to the next.
 *
 *  @{
 *      @file   umpl-states.c
 *      @brief  Controls the UMPL state machine.
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */
#include "umplstates.h"
#include "umplsetup.h"

#include "ml.h"
#include "mlFIFO.h"
#include "mldl.h"

#include "ustore_manager.h"
#include "ustore_lite_fusion_delegate.h"

/*
 * UMPL State Interface
 *
 */

static int umplState = UMPL_UNINIT;
static unsigned short fifo_rate;

/* --------------- */
/* -  Functions. - */
/* --------------- */

/**
 *  @brief  umplGetState returns the current state of UMPL.
 *
 *
 *  @return The current state of UMPL
 */
int umplGetState(void)
{
    return umplState;
}

/**
 *  @brief  umplSetState is called to set the umpl state to the required state.
 *
 *
 *  @param  data
 *              the state to which umpl must be set.
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
static inv_error_t umplSetState(int s)
{
    /* illegal to set state to UMPL_UNINT */
    if ( s != UMPL_UNINIT )
    {
        umplState = s;
        return INV_SUCCESS;
    }
    return INV_ERROR;
}


/**
 *  @brief  umplNotifyInterrupt is called to handle a recently occurred interrupt.
 *
 *
 *
 *  @pre    umplInit() and  umplStartAccelOnly() or umplStartMPU()
 *          must have been called.
 *
 *  @param  data        
 *              the occurred interrupt which must be handled
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t umplNotifyInterrupt(unsigned char inv_interrupt)
{
    if (  umplState == UMPL_ACCEL_ONLY
       || umplState == UMPL_RUN
       || umplState == UMPL_LPACCEL_ONLY)
    {
        return inv_interrupt_handler(inv_interrupt);
    }
    return INV_ERROR_SM_IMPROPER_STATE;
}

/**
 *  @brief  umplOnTimer calls MLUpdataData which updates all the realtime 
 *			data from the motion algorithms.
 *
 *
 *
 *  @pre    umplInit() and  umplStartAccelOnly() or umplStartMPU()
 *          must have been called.
 *
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t umplOnTimer(void)
{
    if (  umplState == UMPL_ACCEL_ONLY
       || umplState == UMPL_RUN
       || umplState == UMPL_LPACCEL_ONLY)
    {
        inv_error_t result;
        result = inv_update_data();

        return result;
    }
    return INV_ERROR_SM_IMPROPER_STATE;
}



/**
 *  @brief  umplInit is the entry point to uMPL and must be called before
 *          all other functions. It initializes the platform specific data.
 *          It also initializes the MPL state.
 *
 *
 *  @param  data
 *              A dummy port number. Could be NULL.
 *
 */
inv_error_t umplInit(unsigned char *port)
{
    if (umplState == UMPL_UNINIT)
    {
        inv_error_t result;

        result = inv_ustore_enable();
        if (result != INV_SUCCESS) return result;

        result = inv_init_ustore_lite_fusion();
        if (result != INV_SUCCESS) return result;

        result = umplPlatformSetup();
        if (result != INV_SUCCESS) return result;
        
        result = inv_serial_start((char const *)port);
        if (result != INV_SUCCESS) return result;

        umplSetState(UMPL_STOP);
        return INV_SUCCESS;
    }
    return INV_ERROR_SM_IMPROPER_STATE;
}

/**
 *  @brief  umplStartMPU kicks starts the uMPL state machine. This function
 *          in turn calls the MPL functions like inv_dmp_open() and inv_dmp_start() which starts 
 *          the motion processing algorithms. This function also enables the required features
 *          such as turning on the bias trackers and temperature compensation.
 *          This function enables the required type of data to be put in FIFO.
 *            
 *
 *
 *  @pre    umplInit() must have been called.
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t umplStartMPU(void)
{
	inv_error_t result;

    if (umplState == UMPL_STOP)
    {
        
        result = inv_dmp_open();
        if (result != INV_SUCCESS) return result;

        result = umplDmpSetup();
        if (result != INV_SUCCESS) return result;

        result = inv_dmp_start();
        if (result != INV_SUCCESS) return result;

#ifndef UMPL_DISABLE_LOAD_CAL
        result = inv_uload_calibration();
        if (result != INV_SUCCESS)
#endif
        umplSetState(UMPL_RUN);
        return INV_SUCCESS;
    }
    else if( umplState == UMPL_ACCEL_ONLY )
    {
        struct mldl_cfg * mldl_cfg = inv_get_dl_config();
        
        if (mldl_cfg->slave[EXT_SLAVE_TYPE_COMPASS]) {
            inv_set_mpu_sensors( INV_NINE_AXIS );
        } else {
            inv_set_mpu_sensors( INV_SIX_AXIS_GYRO_ACCEL );
        }
        inv_set_fifo_rate(fifo_rate);
        umplSetState(UMPL_RUN);
        return INV_SUCCESS;
    }
    else if( umplState == UMPL_LPACCEL_ONLY )
    {
        umplStartAccelOnly(0.0);
        umplStartMPU();
    }
    return INV_ERROR_SM_IMPROPER_STATE;
}

/**
 *  @brief  umplStartAccelOnly starts the accelerometer.
 *
 *  @pre    umplStartMPU() must have been called.
 *  @param  freq
 *          The update rate can be 18 Hz, 50 Hz and 100 Hz.  
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t umplStartAccelOnly(float freq) 
{
    int result;
    if(umplState == UMPL_STOP)
    {
        umplStartMPU();
    }
    if (umplState == UMPL_RUN)
    {
        fifo_rate = inv_get_fifo_rate();
    }
    if (umplState == UMPL_RUN || umplState == UMPL_LPACCEL_ONLY)
    {
        if( freq <= 18.0 )
            inv_set_fifo_rate(10); //fifo rate = 18 hz
        else if( freq <= 50.0 && freq > 18.0 )
            inv_set_fifo_rate(3); //fifo rate = 50 hz
        else
            inv_set_fifo_rate(1); //fifo rate = 100 hz

        inv_set_mpu_sensors(INV_THREE_AXIS_ACCEL);
        umplSetState(UMPL_ACCEL_ONLY);

        return INV_SUCCESS;
    }
    return INV_ERROR_SM_IMPROPER_STATE;
}

/**
 *  @brief  umplStartAccelOnlyLowPower starts the accelerometer 
 *          in low power mode.
 *
 *  @pre    umplStartMPU() must have been called.
 *  @param  freq
 *          The update rate can be 40 Hz, 10 Hz, 2.5 Hz, 1.25 Hz.
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t umplStartAccelOnlyLowPower(float freq)
{

    int result;
    if(umplState == UMPL_STOP)
    {
        
        umplStartMPU();
    }
    if (umplState == UMPL_RUN)
    {
        fifo_rate = inv_get_fifo_rate();
    }
    if (umplState == UMPL_RUN || umplState == UMPL_ACCEL_ONLY)
    {    
        if( freq <= 1.25 )
            inv_set_fifo_rate(159); //fifo rate = 1.25 hz
        else if( freq <= 2.5 && freq > 1.25 )
            inv_set_fifo_rate(79); //fifo rate = 2.5 hz
        else if( freq <= 10.0 && freq > 2.5 )
            inv_set_fifo_rate(19); //fifo rate = 10 hz
        else
            inv_set_fifo_rate(4); //fifo rate = 40 hz

        inv_set_mpu_sensors(INV_THREE_AXIS_ACCEL);
        umplSetState(UMPL_LPACCEL_ONLY);

        return INV_SUCCESS;
    }
    return INV_ERROR_SM_IMPROPER_STATE;
}

/**
 *  @brief  umplStop stops the uMPL state machine. This function
 *          in turn calls the necessary MPL functions like inv_dmp_stop()
 *          and inv_dmp_close() to stop the motion processing algorithms.
 *
 *
 *  @pre    umplInit() and umplStartMPU() must have been called.
 *
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t umplStop(void)
{
    inv_error_t result;
    if (umplState == UMPL_RUN)
    {
#ifndef UMPL_DISABLE_STORE_CAL
    result = inv_ustore_calibration();
    if (result != INV_SUCCESS) return result;
#endif
    }
    if (umplState == UMPL_RUN ||  umplState == UMPL_ACCEL_ONLY ||
     umplState == UMPL_LPACCEL_ONLY)
    {
        result = inv_dmp_stop();
        if (result != INV_SUCCESS) return result;

        result = inv_dmp_close();
        if (result != INV_SUCCESS) return result;

        umplSetState(UMPL_STOP);
        return INV_SUCCESS;
    }
    return INV_ERROR_SM_IMPROPER_STATE;
}

