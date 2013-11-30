/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */

/**
 *  @addtogroup MLDL
 */

#include <string.h>
#include "mltypes.h"
#include "mpu.h"
#include "mldl_cfg.h"
#include "mlos.h"
#include "mldl_cfg_init.h"

#include "dmp_memory.h"

#define GYRO_ORIENT_00 1
#define GYRO_ORIENT_01 0
#define GYRO_ORIENT_02 0
#define GYRO_ORIENT_10 0
#define GYRO_ORIENT_11 1
#define GYRO_ORIENT_12 0
#define GYRO_ORIENT_20 0
#define GYRO_ORIENT_21 0
#define GYRO_ORIENT_22 1

#define ACCEL_GET_SLAVE_DESCR kxtf9_get_slave_descr
struct ext_slave_descr * ACCEL_GET_SLAVE_DESCR(void);

#define ACCEL_SLAVE_ADDRESS 0x0F
#define ACCEL_ORIENT_00 1
#define ACCEL_ORIENT_01 0
#define ACCEL_ORIENT_02 0
#define ACCEL_ORIENT_10 0
#define ACCEL_ORIENT_11 1
#define ACCEL_ORIENT_12 0
#define ACCEL_ORIENT_20 0
#define ACCEL_ORIENT_21 0
#define ACCEL_ORIENT_22 1

/*---- structure containing control variables used by MLDL ----*/
static struct mpu_ram s_mpu_ram = {
    /* __uint16_t length = */   DMP_CODE_SIZE
    /* __u8  *ram   = */ , &dmpMemory 
};

static struct mpu_gyro_cfg    s_mpu_gyro_cfg;
static struct mpu_offsets     s_mpu_offsets;
static struct mpu_chip_info   s_mpu_chip_info;
static struct inv_mpu_cfg     s_inv_mpu_cfg;
static struct inv_mpu_state   s_inv_mpu_state;

/* Platform Data */
struct mpu_platform_data s_pdata = {
    /* __u8 int_config     = */   0x30 /* 0x30: latch, clear on any read */
    /* __u8 level_shifter  = */ , 0
    /* __s8 orientation[9] = */ , 
        { GYRO_ORIENT_00 , GYRO_ORIENT_01 , GYRO_ORIENT_02
        , GYRO_ORIENT_10 , GYRO_ORIENT_11 , GYRO_ORIENT_12
        , GYRO_ORIENT_20 , GYRO_ORIENT_21 , GYRO_ORIENT_22 }
};
struct ext_slave_platform_data s_pdata_slave_accel = {
    /* __u8 type            = */   EXT_SLAVE_TYPE_ACCEL
    /* __uint32_t irq            = */ , 0
    /* __uint32_t adapt_num      = */ , 0
    /* __uint32_t bus            = */ , EXT_SLAVE_BUS_SECONDARY
    /* __u8 address         = */ , ACCEL_SLAVE_ADDRESS
    /* __s8 orientation[9]  = */ , 
        { ACCEL_ORIENT_00 , ACCEL_ORIENT_01 , ACCEL_ORIENT_02
        , ACCEL_ORIENT_10 , ACCEL_ORIENT_11 , ACCEL_ORIENT_12
        , ACCEL_ORIENT_20 , ACCEL_ORIENT_21 , ACCEL_ORIENT_22 }
    /* void *irq_data       = */ , NULL
    /* void *private_data   = */ , NULL
};

static struct mldl_cfg s_mldl_cfg = {
    /* struct mpu_ram       *mpu_ram       = */   &s_mpu_ram
    /* struct mpu_gyro_cfg  *mpu_gyro_cfg  = */ , &s_mpu_gyro_cfg
    /* struct mpu_offsets   *mpu_offsets   = */ , &s_mpu_offsets
    /* struct mpu_chip_info *mpu_chip_info = */ , &s_mpu_chip_info
    /* struct inv_mpu_cfg   *inv_mpu_cfg   = */ , &s_inv_mpu_cfg
    /* struct inv_mpu_state *inv_mpu_state = */ , &s_inv_mpu_state
    /* struct ext_slave_descr *slave[EXT_SLAVE_NUM_TYPES] = */
                                                , { NULL , NULL , NULL , NULL }
    /* struct mpu_platform_data *pdata     = */ , &s_pdata
    /* struct ext_slave_platform_data *pdata_slave[EXT_SLAVE_NUM_TYPES] = */
                                                , { NULL
                                                  , &s_pdata_slave_accel
                                                  , NULL
                                                  , NULL }
};

/**
 *  @brief  Open the driver layer and resets the internal
 *          gyroscope, accelerometer, and compass data
 *          structures.
 *  @param  mlslHandle
 *              the serial handle.
 *  @return INV_SUCCESS if successful, a non-zero error code otherwise.
 */
inv_error_t inv_mldl_cfg_init(struct mldl_cfg **p_g_mldl_cfg)
{
    if (!p_g_mldl_cfg)
        return INV_ERROR_INVALID_PARAMETER;

    s_mldl_cfg.slave[EXT_SLAVE_TYPE_ACCEL]    = ACCEL_GET_SLAVE_DESCR();
    s_mldl_cfg.mpu_chip_info->addr  = 0x68;
    
    *p_g_mldl_cfg = &s_mldl_cfg;
    return INV_SUCCESS;
}

inv_error_t inv_mldl_cfg_exit(struct mldl_cfg **p_g_mldl_cfg)
{
    if (p_g_mldl_cfg)
        *p_g_mldl_cfg = NULL;

    return INV_SUCCESS;
}
