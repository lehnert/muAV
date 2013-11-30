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

#include <stdio.h>
#include <string.h>

#include "temp_comp.h"

#include "ml.h"
#include "mlSetGyroBias.h"
#include "mlmath.h"
#include "mlMathFunc.h"
#include "mlFIFO.h"
#include "mlos.h"
#include "dmpKey.h"
#include "mldl.h"

#include "ustore_manager.h"
#include "ustore_delegate_io.h"

#define TC_DEBUG 0

/*
    Data Structures
*/
struct _TC_NONVOL {
    float eig[3];
    
    float temp_lag;
    float gyro_lag[3];
    
    bool first_run;
};
struct _TC_VOL {
    short sum_num;
    float temp_sum;
    bool recompute;

    unsigned long last_time;
    unsigned long timer;
};
struct _TC {
    struct _TC_NONVOL nv;
    struct _TC_VOL    v;
};

/*
    Prototypes
*/
inv_error_t utc_supervisor(struct inv_obj_t *inv_obj);

static inv_error_t inv_load_tc_data(void);
static inv_error_t inv_store_tc_data(void);
/*
    Globals
*/
extern struct inv_supervisor_cb_obj ml_supervisor_cb;

static struct _TC tc_data;

static const struct uloadstoredelegate tc_ustore_delegate = {
    /* .store = */   inv_store_tc_data
    /* .load  = */ , inv_load_tc_data
    /* .len   = */ , sizeof(struct _TC_NONVOL)
    /* .tag   = */ , INV_USTORE_ID_TC
};

/*
    Functions
*/
void init_tc_data()
{
    int ii;

    for (ii = 0; ii < 3; ii++) {
        tc_data.nv.eig[ii] = 0;
    }
    tc_data.nv.first_run = 1;
    tc_data.v.recompute = 0;
}

inv_error_t inv_enable_temp_comp(void)
{
    inv_error_t result;

    

    result = inv_register_fifo_rate_process(utc_supervisor,
                                            INV_PRIORITY_TEMP_COMP);
    inv_params_obj.bias_mode |= INV_LEARN_BIAS_FROM_TEMPERATURE;
    
    init_tc_data();

    inv_ustore_register_handler(&tc_ustore_delegate);

    return result;
}

inv_error_t inv_disable_temp_comp(void)
{
    inv_error_t result;

    

    inv_params_obj.bias_mode &= ~INV_LEARN_BIAS_FROM_TEMPERATURE;
    result = inv_unregister_fifo_rate_process(utc_supervisor);

    return result;
}


/**
 *  @brief  Say if the temperature compensation algorithm is fully active.
 *  @return 1 if the temp co is enabled. 0 if it is not.
 */
int inv_temp_comp_enabled(void)
{
    return (
        (inv_params_obj.bias_mode & INV_LEARN_BIAS_FROM_TEMPERATURE)
            &&
        inv_get_gyro_present()
    );
}

void inv_temp_comp_reset(void)
{
    
    
    memset(&tc_data.nv, 0, sizeof(tc_data.nv));
    memset(&tc_data.v, 0, sizeof(tc_data.v));
    init_tc_data();
}

float inv_get_calibration_temp_difference(void)
{
    float newTemp;
    inv_get_temperature_float(&newTemp);
    return (newTemp - tc_data.nv.temp_lag);
}

static inv_error_t inv_set_dmp_intercept(void)
{
    inv_error_t result;
    struct mldl_cfg *mldl_cfg = inv_get_dl_config();
    float intercept[3];
    extern struct inv_obj_t inv_obj;
    signed char *orient;
    unsigned char regs[12];
    orient = mldl_cfg->pdata->orientation;
    intercept[0] = orient[0] * tc_data.nv.gyro_lag[0] +
        orient[1] * tc_data.nv.gyro_lag[1] +
        orient[2] * tc_data.nv.gyro_lag[2];
    intercept[1] = orient[3] * tc_data.nv.gyro_lag[0] +
        orient[4] * tc_data.nv.gyro_lag[1] +
        orient[5] * tc_data.nv.gyro_lag[2];
    intercept[2] = orient[6] * tc_data.nv.gyro_lag[0] +
        orient[7] * tc_data.nv.gyro_lag[1] +
        orient[8] * tc_data.nv.gyro_lag[2];
    
    // We're in units of DPS, convert it back to chip units
    intercept[0] = 
        intercept[0] * (1L<<16) * inv_obj.gyro->sf / inv_obj.gyro->sens;
    intercept[1] = 
        intercept[1] * (1L<<16) * inv_obj.gyro->sf / inv_obj.gyro->sens;
    intercept[2] = 
        intercept[2] * (1L<<16) * inv_obj.gyro->sf / inv_obj.gyro->sens;

    inv_int32_to_big8((long)intercept[0],regs);
    inv_int32_to_big8((long)intercept[1],&regs[4]);
    inv_int32_to_big8((long)intercept[2],&regs[8]);

    result = inv_set_mpu_memory(KEY_D_2_96, 12, regs);
    return result;
}

#if defined CONFIG_MPU_SENSORS_MPU6050A2 || \
    defined CONFIG_MPU_SENSORS_MPU6050B1
static inv_error_t inv_init_dmp_tempcomp(void)
{
    short tR;
    long tL;
    long tR32;
    unsigned char regs[4];
    inv_error_t result;

    result = inv_get_mpu_memory(KEY_D_2_108, 4, regs);
    tR32 = inv_big8_to_int32(regs);
    if (tR32 == 0)
    {
        // We need to initialize the temperature and bias for temp comp
        // These will get overwritten once a no motion event occurs on the DMP

        // Set the current temperature
        tL = (long) (tc_data.nv.temp_lag * 65536.0f) - 2462307L;
        tR = (short) (inv_q30_div(tL, 2977653L) >> 16);
        if (result) {
            
            return result;
        }
        tR32 = (long)tR<<15;
        result = 
            inv_set_mpu_memory(KEY_D_2_108, 4, inv_int32_to_big8(tR32,regs));
        if (result) {
            
            return result;
        }

        // Set the bias
        result = inv_set_dmp_intercept();
    }
    return result;
}

static inv_error_t inv_set_dmp_slope(void)
{
    inv_error_t result;
    struct mldl_cfg *mldl_cfg = inv_get_dl_config();
    unsigned char regs[4];
    float scale;
    long sf;
    long slp;
    float slope[3];
    extern struct inv_obj_t inv_obj;
    signed char *orient;
    long temperatureRange;

    orient = mldl_cfg->pdata->orientation;
    slope[0] = orient[0] * tc_data.nv.eig[0] +
        orient[1] * tc_data.nv.eig[1] +
        orient[2] * tc_data.nv.eig[2];
    slope[1] = orient[3] * tc_data.nv.eig[0] +
        orient[4] * tc_data.nv.eig[1] +
        orient[5] * tc_data.nv.eig[2];
    slope[2] = orient[6] * tc_data.nv.eig[0] +
        orient[7] * tc_data.nv.eig[1] +
        orient[8] * tc_data.nv.eig[2];

    if (mldl_cfg->mpu_chip_info->gyro_sens_trim != 0) {
        sf = 2000 * 131 / mldl_cfg->mpu_chip_info->gyro_sens_trim;
    } else {
        sf = 2000;
    }

    temperatureRange = 
        inv_decode_temperature(32767) - inv_decode_temperature(-32768);
    // inv_obj.gyro->sf converts from chip*2^16 to gyro used in DMP
    scale = (float)inv_obj.gyro->sf * temperatureRange / (1L<<16) / sf;
    slp = (long)(scale * slope[0]);
    result = inv_set_mpu_memory(KEY_D_2_244, 4, inv_int32_to_big8(slp,regs));
    if (result) {
            
            return result;
        }
    slp = (long)(scale * slope[1]);
    result = inv_set_mpu_memory(KEY_D_2_248, 4, inv_int32_to_big8(slp,regs));
    if (result) {
            
            return result;
        }
    slp = (long)(scale * slope[2]);
    result = inv_set_mpu_memory(KEY_D_2_252, 4, inv_int32_to_big8(slp,regs));
    return result;
}
#endif /* CONFIG_MPU_SENSORS_MPU6050A2 || CONFIG_MPU_SENSORS_MPU6050B1 */

inv_error_t utc_apply(void)
{
    float new_biases[3];
    float newTemp;
    float delta_temp;

    inv_get_temperature_float(&newTemp);
    delta_temp = newTemp - tc_data.nv.temp_lag;

    new_biases[0] = tc_data.nv.gyro_lag[0] + tc_data.nv.eig[0] * delta_temp;
    new_biases[1] = tc_data.nv.gyro_lag[1] + tc_data.nv.eig[1] * delta_temp;
    new_biases[2] = tc_data.nv.gyro_lag[2] + tc_data.nv.eig[2] * delta_temp;

    inv_set_gyro_bias_float(new_biases);

    return INV_SUCCESS;
}

static inv_error_t utc_clear_data(void)
{
    tc_data.v.temp_sum = 0;

    tc_data.v.sum_num = 0;
    return INV_SUCCESS;
}
static inv_error_t utc_add_data(void)
{
    float newTemp;
    inv_get_temperature_float(&newTemp);
    tc_data.v.temp_sum += newTemp;
    
    tc_data.v.sum_num++;
    return INV_SUCCESS;
}

static inv_error_t utc_recompute(void)
{
    inv_error_t result;
    float gyro_avg_new[3];
    float temp_avg_new;
    
    float temp_lag_diff;
    float gyro_lag_diff[3];

    float var_temp_new;
    float cov_vect_new[2][3];
    float eig_new[3];
    int ii;
    float alpha = 0.3f;       // LPF factor (gyro and temp)
    float eig_alpha = 0.5f;   // LPF factor (slope)
    float min_lag_tail = 0.8; // minimum diff needed (after applying LPF)
    float temp0;

    inv_get_gyro_bias_float(gyro_avg_new);
    temp_avg_new = tc_data.v.temp_sum / tc_data.v.sum_num;

    if (tc_data.nv.first_run) {
        tc_data.nv.first_run = 0;
        
        tc_data.nv.gyro_lag[0] = gyro_avg_new[0];
        tc_data.nv.gyro_lag[1] = gyro_avg_new[1];
        tc_data.nv.gyro_lag[2] = gyro_avg_new[2];
        tc_data.nv.temp_lag = temp_avg_new;

#if defined CONFIG_MPU_SENSORS_MPU6050A2 || defined CONFIG_MPU_SENSORS_MPU6050B1
    {
        inv_error_t result;
        result = inv_init_dmp_tempcomp();
        if (result) {
            
            return result;
        }
    }
#endif
    } else {
        // LPF (also creates tail/lag)
        tc_data.nv.gyro_lag[0] = tc_data.nv.gyro_lag[0] + 
            alpha * (gyro_avg_new[0] - tc_data.nv.gyro_lag[0]);
        tc_data.nv.gyro_lag[1] = tc_data.nv.gyro_lag[1] + 
            alpha * (gyro_avg_new[1] - tc_data.nv.gyro_lag[1]);
        tc_data.nv.gyro_lag[2] = tc_data.nv.gyro_lag[2] + 
            alpha * (gyro_avg_new[2] - tc_data.nv.gyro_lag[2]);
        tc_data.nv.temp_lag = tc_data.nv.temp_lag + 
            alpha * (temp_avg_new - tc_data.nv.temp_lag);
        
        // length of tail (projected onto temperature axis)
        temp_lag_diff = temp_avg_new - tc_data.nv.temp_lag;
        if (fabs(temp_avg_new - tc_data.nv.temp_lag) > min_lag_tail) {
            // pre-calculate temperature variance
            var_temp_new = temp_lag_diff * temp_lag_diff;
            for (ii = 0; ii < 3; ii++) {
                gyro_lag_diff[ii] = gyro_avg_new[ii] - tc_data.nv.gyro_lag[ii];
                
                /* covariance matrix calculated
                * | var_temp_new            cov_vect_new[0][ii] |
                * | cov_vect_new[0][ii]     cov_vect_new[1][ii] |
                */
                cov_vect_new[0][ii] = 
                    gyro_lag_diff[ii] / temp_lag_diff;
                cov_vect_new[1][ii] = 
                    gyro_lag_diff[ii] * gyro_lag_diff[ii] / var_temp_new;

                // covariance matrix applied using vector iteration
                temp0 = 1 + tc_data.nv.eig[ii] * cov_vect_new[0][ii];
                eig_new[ii] = cov_vect_new[0][ii] + 
                    tc_data.nv.eig[ii] * cov_vect_new[1][ii];
                eig_new[ii] /= temp0;

                /* LPF (could add temperature weighting here...
                have eig_alpha depend on temp_lag_diff)
                */
                tc_data.nv.eig[ii] = tc_data.nv.eig[ii] + 
                    eig_alpha * (eig_new[ii] - tc_data.nv.eig[ii]);
            }
        }
    }
    if (TC_DEBUG) {
        MPL_LOGD("Slope: %4.2f %4.2f %4.2f\n",
            tc_data.nv.eig[0], tc_data.nv.eig[1], tc_data.nv.eig[2]);
        
        
    }

#if defined CONFIG_MPU_SENSORS_MPU6050A2 || defined CONFIG_MPU_SENSORS_MPU6050B1
    {
        inv_error_t result;
        result = inv_set_dmp_slope();
        if (result) {
            
            return result;
        }
    }
#endif

    return INV_SUCCESS;
}

inv_error_t utc_supervisor(struct inv_obj_t *inv_obj)
{
    unsigned long delta_time, time;

    time = inv_get_tick_count();
    delta_time = time - tc_data.v.last_time;
    tc_data.v.last_time = time;
    tc_data.v.timer += delta_time;

    if (inv_get_motion_state() == INV_MOTION) {
#if defined CONFIG_MPU_SENSORS_MPU3050
        utc_apply();
#endif
        tc_data.v.recompute = 0;
        tc_data.v.timer = 0;
    }

    else if (tc_data.v.recompute == 1) {
        utc_add_data();
        if(tc_data.v.timer > 2000) { /* time constant */
            utc_recompute();
            utc_clear_data();
            tc_data.v.timer = 0;
        }
    }

    else if (tc_data.v.recompute == 0) {
        utc_clear_data();
        tc_data.v.recompute = 1;
        tc_data.v.timer = 0;
    }

    return INV_SUCCESS;
}

/**
 *  @brief  inv_get_gyro_temp_slope is used to get the temperature
 *          compensation algorithm's estimate of the gyroscope bias temperature
 *          coefficient.
 *          The argument array elements are ordered X,Y,Z.
 *          Values are in units of dps per deg C (degrees per second per degree
 *          Celcius). Values are scaled so that 1 dps per deg C = 2^16 LSBs.
 *          Please refer to the provided "9-Axis Sensor Fusion
 *          Application Note" document.
 *
 *  @pre    MLDmpOpen() \ifnot UMPL or MLDmpPedometerStandAloneOpen() \endif
 *          must have been called.
 *
 *  @param  data
 *              A pointer to an array to be passed back to the user.
 *              <b>Must be 3 cells long </b>.
 *
 *  @return Zero if the command is successful; an ML error code otherwise.
 */
inv_error_t inv_get_gyro_temp_slope(long *data)
{
    if (NULL == data) {
        return INV_ERROR_INVALID_PARAMETER;
    }
    if (inv_params_obj.bias_mode & INV_LEARN_BIAS_FROM_TEMPERATURE) {
        data[0] = (long)(tc_data.nv.eig[0] * 65536.0f);
        data[1] = (long)(tc_data.nv.eig[1] * 65536.0f);
        data[2] = (long)(tc_data.nv.eig[2] * 65536.0f);
        return INV_SUCCESS;
    } else {
        return INV_ERROR_FEATURE_NOT_ENABLED;
    }
}

/**
 *  @brief  inv_get_gyro_temp_slope_float is used to get the temperature
 *          compensation algorithm's estimate of the gyroscope bias temperature
 *          coefficient.
 *          The argument array elements are ordered X,Y,Z.
 *          Values are in units of dps per deg C (degrees per second per degree
 *          Celcius)
 *          Please refer to the provided "9-Axis Sensor Fusion
 *          Application Note" document.
 *
 *  @pre    MLDmpOpen() \ifnot UMPL or MLDmpPedometerStandAloneOpen() \endif
 *          must have been called.
 *
 *  @param  data
 *              A pointer to an array to be passed back to the user.
 *              <b>Must be 3 cells long </b>.
 *
 *  @return INV_SUCCESS if the command is successful; an error code otherwise.
 */
inv_error_t inv_get_gyro_temp_slope_float(float *data)
{
    if (NULL == data) {
        return INV_ERROR_INVALID_PARAMETER;
    }
    if (inv_params_obj.bias_mode & INV_LEARN_BIAS_FROM_TEMPERATURE) {
        data[0] = tc_data.nv.eig[0];
        data[1] = tc_data.nv.eig[1];
        data[2] = tc_data.nv.eig[2];
        return INV_SUCCESS;
    } else {
        return INV_ERROR_FEATURE_NOT_ENABLED;
    }
}

/**
 *  @brief  inv_set_gyro_temp_slope is used to set the temperature
 *          compensation algorithm's estimate of the gyroscope bias temperature
 *          coefficient.
 *          The argument array elements are ordered X,Y,Z.
 *          Values are in units of dps per deg C (degrees per second per degree
 *          Celcius), and scaled such that 1 dps per deg C = 2^16 LSBs.
 *          Please refer to the provided "9-Axis Sensor Fusion
 *          Application Note" document.
 *
 *  @brief  inv_set_gyro_temp_slope is used to set Gyro temperature slope
 *
 *
 *  @pre    MLDmpOpen() \ifnot UMPL or
 *          MLDmpPedometerStandAloneOpen() \endif
 *
 *  @param  data        A pointer to an array to be copied from the user.
 *
 *  @return INV_SUCCESS if successful; a non-zero error code otherwise.
 */
inv_error_t inv_set_gyro_temp_slope(long *data)
{
    if (NULL == data) {
        return INV_ERROR_INVALID_PARAMETER;
    }
    if (inv_params_obj.bias_mode & INV_LEARN_BIAS_FROM_TEMPERATURE) {
        tc_data.nv.eig[0] = ((float) data[0]) / 65536.0f;
        tc_data.nv.eig[1] = ((float) data[1]) / 65536.0f;
        tc_data.nv.eig[2] = ((float) data[2]) / 65536.0f;
        return INV_SUCCESS;
    } else {
        return INV_ERROR_FEATURE_NOT_ENABLED;
    }
}

/**
 *  @brief  inv_set_gyro_temp_slope_float is used to get the temperature
 *          compensation algorithm's estimate of the gyroscope bias temperature
 *          coefficient.
 *          The argument array elements are ordered X,Y,Z.
 *          Values are in units of dps per deg C (degrees per second per degree
 *          Celcius)

 *          Please refer to the provided "9-Axis Sensor Fusion
 *          Application Note" document provided.
 *
 *  @pre    MLDmpOpen() \ifnot UMPL or
 *          MLDmpPedometerStandAloneOpen() \endif
 *
 *  @param  data        A pointer to an array to be copied from the user.
 *
 *  @return INV_SUCCESS if successful; a non-zero error code otherwise.
 */
inv_error_t inv_set_gyro_temp_slope_float(float *data)
{
   if (NULL == data) {
        return INV_ERROR_INVALID_PARAMETER;
    }
    if (inv_params_obj.bias_mode & INV_LEARN_BIAS_FROM_TEMPERATURE) {
        tc_data.nv.eig[0] = data[0];
        tc_data.nv.eig[1] = data[1];
        tc_data.nv.eig[2] = data[2];
        return INV_SUCCESS;
    } else {
        return INV_ERROR_FEATURE_NOT_ENABLED;
    }
}

static inv_error_t inv_store_tc_data(void)
{
    inv_error_t result;
    result = inv_ustore_mem(&tc_data.nv, sizeof(tc_data.nv));
    return result;
}

static inv_error_t inv_load_tc_data(void)
{
    inv_error_t result;

    result = inv_uload_mem(&tc_data.nv, sizeof(tc_data.nv));
    if (result != INV_SUCCESS){
        
        return result;
    }

     utc_apply();

#if defined CONFIG_MPU_SENSORS_MPU6050A2 || \
    defined CONFIG_MPU_SENSORS_MPU6050B1
    {
        inv_init_dmp_tempcomp();
        inv_set_dmp_slope();
    }
#endif
    return result;
}
/**
 *  @}
 */
