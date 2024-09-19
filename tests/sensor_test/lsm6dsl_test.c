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
 * Application layer function of lsm6dsl 3D accelerometer and 3D gyroscope sensor
 *  2020-10-22     tanwenchen    Initial version
 ****************************************************************************/
#ifdef SENSOR_TEST
#include "hal_trace.h"
#include "hal_i2c.h"
#include "hal_spi.h"
#include "string.h"
#include "lsm6dsl_reg.h"
#include "stdlib.h"

#ifdef RTOS
#include "cmsis_os.h"
#ifdef KERNEL_RTX
#include "rt_Time.h"
#endif
#endif

#define     i2c_number_select  2   //0:mcu  i2c0
                                   //1:mcu  i2c1
                                   //2:sens i2c0
                                   //3:sens i2c1
                                   //4:sens i2c2
                                   //5:sens i2c0 ,gpio simulation,2020-11-10

#define     i2c_spi_select     0   //0:i2c,1:spi

typedef union{
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
    int16_t i16bit;
    uint8_t u8bit[2];
} axis1bit16_t;


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME       1000//ms
#define    WAIT_TIME_A     200 //ms
#define    WAIT_TIME_G_01  150 //ms
#define    WAIT_TIME_G_02   50 //ms

/* Self test limits. */
#define    MIN_ST_LIMIT_mg         90.0f
#define    MAX_ST_LIMIT_mg       1700.0f
#define    MIN_ST_LIMIT_mdps   150000.0f
#define    MAX_ST_LIMIT_mdps   700000.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

#define     lsm6dsl_address ((LSM6DSL_I2C_ADD_H & 0xFE)>>1)
#define     sensor_printf   TRACE

#define BUFFER_SIZE_MAX 128
static uint8_t lsm6dsl_buffer[BUFFER_SIZE_MAX] = {0};

#if (i2c_spi_select == 0)
#if (i2c_number_select != 5)
static uint8_t i2c_id = HAL_I2C_ID_0;
#else
struct HAL_GPIO_I2C_CONFIG_T cfg;
#endif
#endif

static int32_t lsm6dsl_write(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
    uint16_t i;
    uint32_t ret = 0;

    lsm6dsl_buffer[0] = reg;

    if(len > BUFFER_SIZE_MAX)
    {
        len = BUFFER_SIZE_MAX;
    }

    for(i = 0; i < len; i++)
    {
        lsm6dsl_buffer[i + 1] = bufp[i];
    }
#if (i2c_spi_select == 0)
#if (i2c_number_select == 5)
    ret = hal_gpio_i2c_send(&cfg,lsm6dsl_address, lsm6dsl_buffer, 1, len);
#else
    ret = hal_i2c_send(i2c_id, lsm6dsl_address, lsm6dsl_buffer, 1, len, 0, NULL);
#endif  //#if (i2c_number_select == 5)

#else
    ret =hal_spi_send(lsm6dsl_buffer, len+1);
#endif
    return (int32_t)ret;
}

static int32_t lsm6dsl_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
#if (i2c_spi_select == 0)

    uint16_t i;
    uint32_t ret = 0;

    lsm6dsl_buffer[0] = reg;

    if(len > BUFFER_SIZE_MAX)
    {
        len = BUFFER_SIZE_MAX;
    }

#if (i2c_number_select == 5)
    ret = hal_gpio_i2c_recv(&cfg,lsm6dsl_address, lsm6dsl_buffer, 1, len, 0);
#else
    ret = hal_i2c_recv(i2c_id, lsm6dsl_address, lsm6dsl_buffer, 1, len, HAL_I2C_RESTART_AFTER_WRITE, 0, 0);
#endif

    for(i= 0; i < len; i++)
    {
         bufp[i] = lsm6dsl_buffer[i + 1];
    }

#else
    int ret = 0;
    //uint8_t *str;
    uint8_t outBuf[len + 1];

    //str = (uint8_t *) malloc(len+1);

    /* Read command */
    reg |= 0x80;
    ret=hal_spi_recv(&reg, outBuf, len+1);
    memcpy(bufp,outBuf+1,len);//remove head

    //free(str);
#endif
    return (int32_t)ret;
}

static void lsm6dsl_platform_init(void)
{
#if (i2c_spi_select == 0)
#if (i2c_number_select == 5)

    cfg.scl=HAL_GPIO_PIN_P0_4;
    cfg.sda=HAL_GPIO_PIN_P0_5;
    cfg.speed=100*1000;
    hal_gpio_i2c_open(&cfg);
#else

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
#endif

#else

    struct HAL_SPI_CFG_T spi_cfg;

    memset(&spi_cfg, 0, sizeof(spi_cfg));
    spi_cfg.clk_delay_half = true;
    spi_cfg.clk_polarity = true;
    spi_cfg.slave = false;
    spi_cfg.dma_rx = false;
    spi_cfg.dma_tx = false;
    spi_cfg.rx_sep_line = false;
    spi_cfg.cs = 0;
    spi_cfg.rate = 6*1000*1000;
    spi_cfg.rx_bits = 8;
    spi_cfg.tx_bits = 8;
    spi_cfg.rx_frame_bits = 0;

    hal_iomux_set_spi();
    hal_spi_open(&spi_cfg);
#endif
}

static void lsm6dsl_delay(uint32_t ms)
{
    osDelay(ms);
}

void lsm6dsl_self_test(void)
{
    /* Because the stack is small, the following variables are defined as static ,tanwenchen,2020-10-23*/
    static axis3bit16_t data_raw;
    static stmdev_ctx_t dev_ctx;
    static float val_st_off[3];
    static float val_st_on[3];
    static float test_val[3];
    static uint8_t st_result;
    static uint8_t whoamI;
    static uint8_t drdy;
    static uint8_t rst;
    static uint8_t i;
    static uint8_t j;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = lsm6dsl_write;
    dev_ctx.read_reg = lsm6dsl_read;
    dev_ctx.handle = NULL;

    /* Initialize platform specific hardware */
    lsm6dsl_platform_init();

    /* Wait sensor boot time */
    lsm6dsl_delay(BOOT_TIME);

    /* Check device ID */
    lsm6dsl_device_id_get(&dev_ctx, &whoamI);
    while (whoamI != LSM6DSL_ID)
    {
        sensor_printf(1, "lsm6dsl id(0x6A)=0x%X", whoamI);
        lsm6dsl_device_id_get(&dev_ctx, &whoamI);
        lsm6dsl_delay(WAIT_TIME_A);
    }
    sensor_printf(1, "lsm6dsl id(0x6A)=0x%X", whoamI);

    /* Restore default configuration */
    lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do
    {
        sensor_printf(1, "-----------------waitting......");
        lsm6dsl_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Enable Block Data Update */
    lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /*
    * Accelerometer Self Test
    */
    /* Set Output Data Rate */
    lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_52Hz);

    /* Set full scale */
    lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_4g);

    /* Wait stable output */
    lsm6dsl_delay(WAIT_TIME_A);

    /* Check if new value available */
    do
    {
        lsm6dsl_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);
    /* Read dummy data and discard it */
    lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw.u8bit);

    /* Read 5 sample and get the average vale for each axis */
    memset(val_st_off, 0x00, 3*sizeof(float));
    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            sensor_printf(1, "-----------------waitting......");
            lsm6dsl_xl_flag_data_ready_get(&dev_ctx, &drdy);
        } while(!drdy);
        /* Read data and accumulate the mg value */
        lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw.u8bit);
        for (j = 0; j < 3; j++)
        {
            val_st_off[j] += lsm6dsl_from_fs4g_to_mg(data_raw.i16bit[j]);
        }
    }

    /* Calculate the mg average values */
    for (i = 0; i < 3; i++)
    {
        val_st_off[i] /= 5.0f;
    }

    /* Enable Self Test positive (or negative) */
    lsm6dsl_xl_self_test_set(&dev_ctx, LSM6DSL_XL_ST_NEGATIVE);
    //lsm6dsl_xl_self_test_set(&dev_ctx, LSM6DSL_XL_ST_POSITIVE);

    /* Wait stable output */
    lsm6dsl_delay(WAIT_TIME_A);

    /* Check if new value available */
    do
    {
        lsm6dsl_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);
    /* Read dummy data and discard it */
    lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw.u8bit);

    /* Read 5 sample and get the average vale for each axis */
    memset(val_st_on, 0x00, 3*sizeof(float));
    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            sensor_printf(1, "-----------------waitting......");
            lsm6dsl_xl_flag_data_ready_get(&dev_ctx, &drdy);
        } while(!drdy);
        /* Read data and accumulate the mg value */
        lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw.u8bit);
        for (j = 0; j < 3; j++)
        {
            val_st_on[j] += lsm6dsl_from_fs4g_to_mg(data_raw.i16bit[j]);
        }
    }

    /* Calculate the mg average values */
    for (i = 0; i < 3; i++)
    {
        val_st_on[i] /= 5.0f;
    }

    /* Calculate the mg values for self test */
    for (i = 0; i < 3; i++)
    {
        test_val[i] = (float)fabs((val_st_on[i] - val_st_off[i]));
    }

    /* Check self test limit */
    st_result = ST_PASS;
    for (i = 0; i < 3; i++)
    {
        if (( MIN_ST_LIMIT_mg > test_val[i] ) || ( test_val[i] > MAX_ST_LIMIT_mg))
        {
            st_result = ST_FAIL;
        }
    }

    /* Disable Self Test */
    lsm6dsl_xl_self_test_set(&dev_ctx, LSM6DSL_XL_ST_DISABLE);
    lsm6dsl_delay(BOOT_TIME);
    /* Disable sensor. */
    lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_OFF);
    lsm6dsl_delay(BOOT_TIME);
    /*
    * Gyroscope Self Test
    */

    /* Set Output Data Rate */
    lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_208Hz);
    lsm6dsl_delay(BOOT_TIME);
    /* Set full scale */
    lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);

    /* Wait stable output */
    lsm6dsl_delay(BOOT_TIME);

    /* Check if new value available */
    do
    {
        lsm6dsl_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);
    /* Read dummy data and discard it */
    lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw.u8bit);

    /* Read 5 sample and get the average vale for each axis */
    memset(val_st_off, 0x00, 3*sizeof(float));
    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            sensor_printf(1, "-----------------waitting......");
            lsm6dsl_xl_flag_data_ready_get(&dev_ctx, &drdy);
        } while(!drdy);
        /* Read data and accumulate the mg value */
        lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw.u8bit);
        for (j = 0; j < 3; j++)
        {
            val_st_off[j] += lsm6dsl_from_fs2000dps_to_mdps(data_raw.i16bit[j]);
        }
    }
    /* Calculate the mg average values */
    for (i = 0; i < 3; i++)
    {
        val_st_off[i] /= 5.0f;
    }

    /* Enable Self Test positive (or negative) */
    lsm6dsl_gy_self_test_set(&dev_ctx, LSM6DSL_GY_ST_POSITIVE);
    //lsm6dsl_gy_self_test_set(&dev_ctx, LIS2DH12_GY_ST_NEGATIVE);

    /* Wait stable output */
    lsm6dsl_delay(WAIT_TIME_G_02);

    /* Read 5 sample and get the average vale for each axis */
    memset(val_st_on, 0x00, 3*sizeof(float));
    for (i = 0; i < 5; i++)
    {
        /* Check if new value available */
        do
        {
            sensor_printf(1, "-----------------waitting......");
            lsm6dsl_xl_flag_data_ready_get(&dev_ctx, &drdy);
        } while(!drdy);
        /* Read data and accumulate the mg value */
        lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw.u8bit);
        for (j = 0; j < 3; j++)
        {
            val_st_on[j] += lsm6dsl_from_fs2000dps_to_mdps(data_raw.i16bit[j]);
        }
    }

    /* Calculate the mg average values */
    for (i = 0; i < 3; i++)
    {
        val_st_on[i] /= 5.0f;
    }

    /* Calculate the mg values for self test */
    for (i = 0; i < 3; i++)
    {
        test_val[i] = (float)fabs((val_st_on[i] - val_st_off[i]));
    }
    /* Check self test limit */
    for (i = 0; i < 3; i++)
    {
        if (( MIN_ST_LIMIT_mdps > test_val[i] ) || ( test_val[i] > MAX_ST_LIMIT_mdps))
        {
            st_result = ST_FAIL;
        }
    }

    /* Disable Self Test */
    lsm6dsl_gy_self_test_set(&dev_ctx, LSM6DSL_GY_ST_DISABLE);
    /* Disable sensor. */
    lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_OFF);

    if (st_result == ST_PASS)
    {
        sensor_printf(0,"lsm6dsl Self Test - PASS");
    }
    else
    {
        sensor_printf(0,"lsm6dsl Self Test - FAIL");
    }
}

void lsm6dsl_read_data_polling_test(void)
{
    /* Because the stack is small, the following variables are defined as static ,tanwenchen,2020-10-23*/
    static axis3bit16_t data_raw_acceleration;
    static axis3bit16_t data_raw_angular_rate;
    static axis1bit16_t data_raw_temperature;
    static float acceleration_mg[3]={0.0};
    static float angular_rate_mdps[3]={0.0};
    static float temperature_degC=0.0;
    static uint8_t whoamI, rst;
    static stmdev_ctx_t dev_ctx;

    /*
    *  Initialize mems driver interface
    */
    dev_ctx.write_reg = lsm6dsl_write;
    dev_ctx.read_reg = lsm6dsl_read;
    dev_ctx.handle = NULL;

    /* Initialize platform specific hardware */
    lsm6dsl_platform_init();

    /*
    *  Check device ID
    */
    whoamI = 0;
    lsm6dsl_device_id_get(&dev_ctx, &whoamI);
    while (whoamI != LSM6DSL_ID)
    {
        sensor_printf(1, "lsm6dsl id(0x6A)=0x%X", whoamI);
        lsm6dsl_device_id_get(&dev_ctx, &whoamI);
        lsm6dsl_delay(WAIT_TIME_A);
    }
    sensor_printf(1, "lsm6dsl id(0x6A)=0x%X", whoamI);

    /*
    *  Restore default configuration
    */
    lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do
    {
        lsm6dsl_reset_get(&dev_ctx, &rst);
    } while (rst);
    /*
    *  Enable Block Data Update
    */
    lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /*
    * Set Output Data Rate
    */
    lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_12Hz5);
    lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_12Hz5);
    /*
    * Set full scale
    */
    lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
    lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);

    /*
    * Configure filtering chain(No aux interface)
    */
    /* Accelerometer - analog filter */
    lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);

    /* Accelerometer - LPF1 path ( LPF2 not used )*/
    //lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);

    /* Accelerometer - LPF1 + LPF2 path */
    lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);

    /* Accelerometer - High Pass / Slope path */
    //lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    //lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);

    /* Gyroscope - filtering chain */
    lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);

    /*
    * Read samples in polling mode (no int)
    */
    while(1)
    {
        /*
        * Read output only if new value is available
        */
        lsm6dsl_reg_t reg;
        lsm6dsl_status_reg_get(&dev_ctx, &reg.status_reg);

        if (reg.status_reg.xlda)
        {
            /* Read magnetic field data */
            memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
            lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
            acceleration_mg[0] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[0]);
            acceleration_mg[1] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[1]);
            acceleration_mg[2] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[2]);
        }
        if (reg.status_reg.gda)
        {
            /* Read magnetic field data */
            memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
            lsm6dsl_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
            angular_rate_mdps[0] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            angular_rate_mdps[1] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            angular_rate_mdps[2] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
        }
        if (reg.status_reg.tda)
        {
            /* Read temperature data */
            memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
            lsm6dsl_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
            temperature_degC = lsm6dsl_from_lsb_to_celsius( data_raw_temperature.i16bit );
        }

        //The display needs to be divided into two times, otherwise there will be garbled code,2020-10-22
        sensor_printf(TR_ATTR_NO_LF,"lsm6dsl Acc/Ang/Tem[mg mdps degC]:%08.2f\t%08.2f\t%08.2f\t",
                                                                        (double)acceleration_mg[0],
                                                                        (double)acceleration_mg[1],
                                                                        (double)acceleration_mg[2]);
        sensor_printf(TR_ATTR_NO_TS|TR_ATTR_NO_ID,"%08.2f\t%08.2f\t%08.2f\t%08.2f",
                                                                        (double)angular_rate_mdps[0],
                                                                        (double)angular_rate_mdps[1],
                                                                        (double)angular_rate_mdps[2],
                                                                        (double)temperature_degC);
        lsm6dsl_delay(500);
    }
}

#endif

