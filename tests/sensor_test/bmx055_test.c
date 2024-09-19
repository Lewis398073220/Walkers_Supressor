/***************************************************************************
 *
 * Copyright 2015-2020 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 * Application layer function of bmx055  9-axis sensor module
 *  2020-12-07     tanwenchen    Initial version
 ****************************************************************************/
#ifdef SENSOR_TEST
#include "hal_trace.h"
#include "hal_i2c.h"
#include "string.h"
#include "bma2x2.h"

#ifdef RTOS
#include "cmsis_os.h"
#ifdef KERNEL_RTX
#include "rt_Time.h"
#endif
#endif

#define     i2c_number_select  0   //0:mcu  i2c0
                                   //1:mcu  i2c1
                                   //2:sens i2c0
                                   //3:sens i2c1
                                   //4:sens i2c2

#define     i2c_spi_select     0   //0:i2c,1:spi

#define     bmx055_address (BMA2x2_I2C_ADDR1<<1)
#define     sensor_printf   TRACE

#define BUFFER_SIZE_MAX 128
static uint8_t bmx055_buffer[BUFFER_SIZE_MAX] = {0};

static uint8_t i2c_id = HAL_I2C_ID_0;

static int8_t bmx055_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t length)
{
#if (i2c_spi_select == 0)

    uint16_t i;
    uint32_t ret = 0;

    bmx055_buffer[0] = reg_addr;

    if(length > BUFFER_SIZE_MAX)
    {
        length = BUFFER_SIZE_MAX;
    }

    for(i = 0; i < length; i++)
    {
        bmx055_buffer[i + 1] = reg_data[i];
    }
    ret = hal_i2c_send(i2c_id, i2c_addr, bmx055_buffer, 1, length, 0, NULL);
#else
    // TODO:No need to implement it for the time being,2020-11-14
#endif
    return (int8_t)ret;
}

static int8_t bmx055_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t length)
{
#if (i2c_spi_select == 0)

    uint16_t i;
    uint32_t ret = 0;

    bmx055_buffer[0] = reg_addr;

    if(length > BUFFER_SIZE_MAX)
    {
        length = BUFFER_SIZE_MAX;
    }

    ret = hal_i2c_recv(i2c_id, i2c_addr, bmx055_buffer, 1, length, HAL_I2C_RESTART_AFTER_WRITE, 0, 0);

    for(i= 0; i < length; i++)
    {
         reg_data[i] = bmx055_buffer[i + 1];
    }
#else
    // TODO:No need to implement it for the time being,2020-11-14
#endif
    return (int8_t)ret;
}

static void bmx055_platform_init(void)
{
    struct HAL_I2C_CONFIG_T cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = HAL_I2C_API_MODE_TASK;//HAL_I2C_API_MODE_SIMPLE;
    cfg.use_dma = 0;
    cfg.use_sync = 1;
    cfg.speed = 100*1000;
    cfg.as_master = 1;

    switch(i2c_number_select)
    {
        case 0:
        {
            i2c_id = HAL_I2C_ID_0;
            hal_iomux_set_i2c0();
            break;
        }
        case 1:
        {
            i2c_id = HAL_I2C_ID_1;
            hal_iomux_set_i2c1();
            break;
        }
#ifdef CHIP_SUBSYS_SENS
        case 2:
        {
            i2c_id = HAL_I2C_ID_0;
            hal_iomux_set_i2c0();
            break;
        }
        case 3:
        {
            i2c_id = HAL_I2C_ID_1;
            hal_iomux_set_i2c1();
            break;
        }
        case 4:
        {
            i2c_id = HAL_I2C_ID_2;
            hal_iomux_set_i2c2();
            break;
        }
#endif
        default:
        {
            break;
        }
    }
    hal_i2c_open(i2c_id, &cfg);
}

static void bmx055_delay(uint32_t ms)
{
    osDelay(ms);
}

void bmx055_test(void)
{
    /*Local variables for reading accel x, y and z data*/
    s16 accel_x_s16, accel_y_s16, accel_z_s16 = BMA2x2_INIT_VALUE;

    /* bma2x2acc_data structure used to read accel xyz data*/
    struct bma2x2_accel_data sample_xyz;
    /* bma2x2acc_data_temp structure used to read
        accel xyz and temperature data*/
    struct bma2x2_accel_data_temp sample_xyzt;
    /* Local variable used to assign the bandwidth value*/
    u8 bw_value_u8 = BMA2x2_INIT_VALUE;
    /* Local variable used to set the bandwidth value*/
    u8 banwid = BMA2x2_INIT_VALUE;
    /* status of communication*/
    s32 com_rslt = ERROR;
    struct bma2x2_t bma2x2;

    bma2x2.bus_write = bmx055_write;
    bma2x2.bus_read = bmx055_read;
    bma2x2.delay_msec = bmx055_delay;
    bma2x2.dev_addr = bmx055_address;

    bmx055_platform_init();
 /*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *  the following parameters
 *  I2C address
 *  Bus Write
 *  Bus read
 *  Chip id
 *-------------------------------------------------------------------------*/
    com_rslt = bma2x2_init(&bma2x2);

/*  For initialization it is required to set the mode of
 *  the sensor as "NORMAL"
 *  NORMAL mode is set from the register 0x11 and 0x12
 *  0x11 -> bit 5,6,7 -> set value as 0
 *  0x12 -> bit 5,6 -> set value as 0
 *  data acquisition/read/write is possible in this mode
 *  by using the below API able to set the power mode as NORMAL
 *  For the Normal/standby/Low power 2 mode Idle time
        of at least 2us(micro seconds)
 *  required for read/write operations*/
    /* Set the power mode as NORMAL*/
    com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
/*  Note:
    * For the Suspend/Low power1 mode Idle time of
        at least 450us(micro seconds)
    * required for read/write operations*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
    /* This API used to Write the bandwidth of the sensor input
    value have to be given
    bandwidth is set from the register 0x10 bits from 1 to 4*/
    bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
    com_rslt += bma2x2_set_bw(bw_value_u8);

    /* This API used to read back the written value of bandwidth*/
    com_rslt += bma2x2_get_bw(&banwid);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*-------------------------------------------------------------------*/
/*------------------------------------------------------------------*
************************* START READ SENSOR DATA(X,Y and Z axis) ********
*---------------------------------------------------------------------*/
    while(1)
    {
        /* Read the accel X data*/
        com_rslt += bma2x2_read_accel_x(&accel_x_s16);
        /* Read the accel Y data*/
        com_rslt += bma2x2_read_accel_y(&accel_y_s16);
        /* Read the accel Z data*/
        com_rslt += bma2x2_read_accel_z(&accel_z_s16);

        //sensor_printf(3,"bmx055 read data,xyzs16=%d %d %d\r\n"  ,accel_x_s16
        //                                                    ,accel_y_s16
        //                                                    ,accel_y_s16);

        /* accessing the bma2x2acc_data parameter by using sample_xyz*/
        /* Read the accel XYZ data*/
        com_rslt += bma2x2_read_accel_xyz(&sample_xyz);

        /* accessing the bma2x2acc_data_temp parameter by using sample_xyzt*/
        /* Read the accel XYZT data*/
        com_rslt += bma2x2_read_accel_xyzt(&sample_xyzt);

        sensor_printf(7,"bmx055 read data,xyzt=%d %d %d %d %d %d %d\r\n"
                                                            ,sample_xyz.x
                                                            ,sample_xyz.y
                                                            ,sample_xyz.z
                                                            ,sample_xyzt.x
                                                            ,sample_xyzt.y
                                                            ,sample_xyzt.z
                                                            ,sample_xyzt.temp);
        bma2x2.delay_msec(1000);
    }
/*--------------------------------------------------------------------*
************************* END READ SENSOR DATA(X,Y and Z axis) ************
*-------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
/*  For de-initialization it is required to set the mode of
 *  the sensor as "DEEP SUSPEND"
 *  DEEP SUSPEND mode is set from the register 0x11
 *  0x11 -> bit 5 -> set value as 1
 *  the device reaches the lowest power consumption only
 *  interface selection is kept alive
 *  No data acquisition is performed
 *  by using the below API able to set the power mode as DEEPSUSPEND*/
 /* Set the power mode as DEEPSUSPEND*/
    //com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
    //return com_rslt;
}

#endif
