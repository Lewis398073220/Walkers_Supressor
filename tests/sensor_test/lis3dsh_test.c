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
 * Application layer function of li3dsh three-axis accelerometer sensor
 *  2020-10-22     tanwenchen    Initial version
 ****************************************************************************/
#ifdef SENSOR_TEST
#include "hal_trace.h"
#include "hal_i2c.h"
#include "hal_spi.h"
#include "string.h"
#include "lis3dsh_reg.h"
#include "stdlib.h"

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

#define     i2c_spi_select     1   //0:i2c,1:spi
#define     spi_id_select      0   //0:spi,1:spilcd,2021-02-09


/* Private typedef -----------------------------------------------------------*/
typedef union {
    uint8_t u8[6];
    int16_t i16[3];
} fifo_data_t;

/* Private macro -------------------------------------------------------------*/
#define     BOOT_TIME       1000//ms
#define     WAIT_TIME       200 //ms

#define     lis3dsh_address ((LIS3DSH_I2C_ADD_L & 0xFE)>>1)
#define     sensor_printf   TRACE

#define BUFFER_SIZE_MAX 128
static uint8_t lis3dsh_buffer[BUFFER_SIZE_MAX] = {0};

#if (i2c_spi_select == 0)
static uint8_t i2c_id = HAL_I2C_ID_0;
#endif

static int32_t lis3dsh_write(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
    uint16_t i;
    uint32_t ret = 0;

    /* Write command */
    lis3dsh_buffer[0] = reg;

    if(len > BUFFER_SIZE_MAX)
    {
        len = BUFFER_SIZE_MAX;
    }

    for(i = 0; i < len; i++)
    {
        lis3dsh_buffer[i + 1] = bufp[i];
    }
#if (i2c_spi_select == 0)
    ret = hal_i2c_send(i2c_id, lis3dsh_address, lis3dsh_buffer, 1, len, 0, NULL);
#else
#if (spi_id_select == 0)
    ret =hal_spi_send(lis3dsh_buffer, len+1);
#elif (spi_id_select == 1)
#ifdef CHIP_HAS_SPILCD
    ret =hal_spilcd_send(lis3dsh_buffer, len+1);
#else
    #error "The chip does not have this peripheral!"
#endif
#endif
#endif
    return (int32_t)ret;
}

static int32_t lis3dsh_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
#if (i2c_spi_select == 0)

    uint16_t i;
    uint32_t ret = 0;

    /* read command */
    lis3dsh_buffer[0] = reg;

    if(len > BUFFER_SIZE_MAX)
    {
        len = BUFFER_SIZE_MAX;
    }

    ret = hal_i2c_recv(i2c_id, lis3dsh_address, lis3dsh_buffer, 1, len, HAL_I2C_RESTART_AFTER_WRITE, 0, 0);

    for(i= 0; i < len; i++)
    {
         bufp[i] = lis3dsh_buffer[i + 1];
    }
#else
    int ret = 0;
    //uint8_t *str;
    uint8_t outBuf[len + 1];

    //str = (uint8_t *) malloc(len+1);

    /* Read command */
    reg |= 0x80;
#if (spi_id_select == 0)
    ret=hal_spi_recv(&reg, outBuf, len+1);
#elif (spi_id_select == 1)
#ifdef CHIP_HAS_SPILCD
    ret=hal_spilcd_recv(&reg, outBuf, len+1);
#else
    #error "The chip does not have this peripheral!"
#endif
#endif
    memcpy(bufp,outBuf+1,len);//remove head

    //free(str);
#endif
    return (int32_t)ret;
}

static void lis3dsh_platform_init(void)
{
#if (i2c_spi_select == 0)
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
#if (spi_id_select == 0)
    hal_iomux_set_spi();
    hal_spi_open(&spi_cfg);
#elif (spi_id_select == 1)
#ifdef CHIP_HAS_SPILCD
    hal_iomux_set_spilcd();
    hal_spilcd_open(&spi_cfg);
#else
    #error "The chip does not have this peripheral!"
#endif
#endif
#endif
}

static void lis3dsh_delay(uint32_t ms)
{
    osDelay(ms);
}

void lis3dsh_self_test(void)
{
    /* Because the stack is small, the following variables are defined as static ,tanwenchen,2020-10-23*/

    /* Self test results. */
    #define    ST_PASS     1U
    #define    ST_FAIL     0U
    #define    SAMPLES     5 //number of samples

    /* Self test limits typical values: x, y ~ 140mg / z ~590mg @ 2.5V */
    static const float min_st_limit[] = {120.0f, 120.0f, 140.0f};
    static const float max_st_limit[] = {550.0f, 550.0f, 750.0f};
    static lis3dsh_all_sources_t all_sources;
    static lis3dsh_bus_mode_t bus_mode;
    static lis3dsh_status_var_t status;
    static stmdev_ctx_t dev_ctx;
    static float maes_st_off[3];
    static float maes_st_on[3];
    static lis3dsh_data_t data;
    static float test_val[3];
    static uint8_t st_result;
    static lis3dsh_id_t id;
    static lis3dsh_md_t md;
    static uint8_t i, j;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = lis3dsh_write;
    dev_ctx.read_reg = lis3dsh_read;
    dev_ctx.handle = NULL;

    /* Initialize platform specific hardware */
    lis3dsh_platform_init();

    /* Initialize self test results */
    st_result = ST_PASS;

    /* Wait sensor boot time */
    lis3dsh_delay(BOOT_TIME);

    /* Check device ID */
    lis3dsh_id_get(&dev_ctx, &id);
    while (id.whoami != LIS3DSH_ID)
    {
        sensor_printf(3, "lis3dsh id(0x3F)=0x%X 0x%X 0x%X", id.info1, id.info2, id.whoami);
        lis3dsh_id_get(&dev_ctx, &id);
        lis3dsh_delay(WAIT_TIME);
    }
    sensor_printf(3, "lis3dsh id(0x3F)=0x%X 0x%X 0x%X", id.info1, id.info2, id.whoami);

    /* Restore default configuration */
    lis3dsh_init_set(&dev_ctx, LIS3DSH_RESET);
    do
    {
        lis3dsh_status_get(&dev_ctx, &status);
    } while (status.sw_reset);

    /* Set bdu and if_inc recommended for driver usage */
    lis3dsh_init_set(&dev_ctx, LIS3DSH_DRV_RDY);

    /* Select bus interface */
    bus_mode = LIS3DSH_SEL_BY_HW;
    lis3dsh_bus_mode_set(&dev_ctx, &bus_mode);

    /* Set Output Data Rate */
    lis3dsh_mode_get(&dev_ctx, &md);
    md.fs =  LIS3DSH_2g;
    md.odr = LIS3DSH_50Hz;
    lis3dsh_mode_set(&dev_ctx, &md);

    /* Wait stable output */
    lis3dsh_delay(WAIT_TIME);

    /* Check if new value available */
    do
    {
        lis3dsh_all_sources_get(&dev_ctx, &all_sources);
    } while(!all_sources.drdy_xl);
    /* Read dummy data and discard it */
    lis3dsh_data_get(&dev_ctx, &md, &data);

    /* Read samples and get the average vale for each axis */
    for (i = 0; i < SAMPLES; i++)
    {
        /* Check if new value available */
        do
        {
            lis3dsh_all_sources_get(&dev_ctx, &all_sources);
        } while(!all_sources.drdy_xl);
        /* Read data and accumulate the mg value */
        lis3dsh_data_get(&dev_ctx, &md, &data);
        for (j=0; j<3; j++)
        {
            maes_st_off[j] += data.xl.mg[j];
        }
    }
    /* Calculate the mg average values */
    for (i=0; i<3; i++)
    {
        maes_st_off[i] /= SAMPLES;
    }

    /* Enable Self Test positive (or negative) */
    lis3dsh_self_test_set(&dev_ctx, LIS3DSH_ST_POSITIVE);
    //lis3dsh_self_test_set(&dev_ctx, LIS3DSH_ST_NEGATIVE);

    /* Wait stable output */
    lis3dsh_delay(WAIT_TIME);

    /* Check if new value available */
    do
    {
        lis3dsh_all_sources_get(&dev_ctx, &all_sources);
    } while(!all_sources.drdy_xl);
    /* Read dummy data and discard it */
    lis3dsh_data_get(&dev_ctx, &md, &data);

    /* Read samples and get the average vale for each axis */
    for (i = 0; i < SAMPLES; i++)
    {
        /* Check if new value available */
        do
        {
            lis3dsh_all_sources_get(&dev_ctx, &all_sources);
        } while(!all_sources.drdy_xl);
        /* Read data and accumulate the mg value */
        lis3dsh_data_get(&dev_ctx, &md, &data);
        for (j=0; j<3; j++)
        {
            maes_st_on[j] += data.xl.mg[j];
        }
    }
    /* Calculate the mg average values */
    for (i=0; i<3; i++)
    {
        maes_st_on[i] /= SAMPLES;
    }

    /* Calculate the mg values for self test */
    for (i=0; i<3; i++)
    {
        test_val[i] = (float)fabs((maes_st_on[i] - maes_st_off[i]));
    }

    /* Check self test limit */
    for (i=0; i<3; i++)
    {
        if (( min_st_limit[i] > test_val[i] ) || ( test_val[i] > max_st_limit[i]))
        {
            st_result = ST_FAIL;
        }
    }

    /* Disable Self Test */
    lis3dsh_self_test_set(&dev_ctx, LIS3DSH_ST_DISABLE);
    lis3dsh_delay(WAIT_TIME);

    md.odr = LIS3DSH_OFF;
    lis3dsh_mode_set(&dev_ctx, &md);

    /* Print self test result */
    if (st_result == ST_PASS)
    {
        sensor_printf(0,"lis3dsh Self Test - PASS");
    }
    else
    {
        sensor_printf(0,"lis3dsh Self Test - FAIL");
    }
}

void lis3dsh_fifo_stream_test(void)
{
    /* Because the stack is small, the following variables are defined as static ,tanwenchen,2020-10-23*/
    static lis3dsh_pin_int1_route_t int1_route;
    static lis3dsh_all_sources_t all_sources;
    static lis3dsh_bus_mode_t bus_mode;
    static lis3dsh_status_var_t status;
    static lis3dsh_int_mode_t int_mode;
    static fifo_data_t fifo_data[32];
    static stmdev_ctx_t dev_ctx;
    static lis3dsh_reg_t reg;
    static lis3dsh_id_t id;
    static lis3dsh_md_t md;
    static uint8_t i;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = lis3dsh_write;
    dev_ctx.read_reg = lis3dsh_read;
    dev_ctx.handle = NULL;

    /* Initialize platform specific hardware */
    lis3dsh_platform_init();

    /* Wait sensor boot time */
    lis3dsh_delay(BOOT_TIME);

    /* Check device ID */
    lis3dsh_id_get(&dev_ctx, &id);
    while (id.whoami != LIS3DSH_ID)
    {
        sensor_printf(3, "lis3dsh id(0x3F)=0x%X 0x%X 0x%X", id.info1, id.info2, id.whoami);
        lis3dsh_id_get(&dev_ctx, &id);
        lis3dsh_delay(WAIT_TIME);
    }
    sensor_printf(3, "lis3dsh id(0x3F)=0x%X 0x%X 0x%X", id.info1, id.info2, id.whoami);

    /* Restore default configuration */
    lis3dsh_init_set(&dev_ctx, LIS3DSH_RESET);
    do
    {
        lis3dsh_status_get(&dev_ctx, &status);
    } while (status.sw_reset);

    /* Set bdu and if_inc recommended for driver usage */
    lis3dsh_init_set(&dev_ctx, LIS3DSH_DRV_RDY);

    /* Select bus interface */
    bus_mode = LIS3DSH_SEL_BY_HW;
    lis3dsh_bus_mode_set(&dev_ctx, &bus_mode);

    /* FIFO configuration */
    lis3dsh_read_reg(&dev_ctx, LIS3DSH_FIFO_CTRL, &reg.byte,1);
    reg.fifo_ctrl.fmode = 0x02; /* FIFO stream mode */
    lis3dsh_write_reg(&dev_ctx, LIS3DSH_FIFO_CTRL, &reg.byte,1);
    lis3dsh_read_reg(&dev_ctx, LIS3DSH_CTRL_REG6, &reg.byte,1);
    reg.ctrl_reg6.fifo_en = PROPERTY_ENABLE;
    lis3dsh_write_reg(&dev_ctx, LIS3DSH_CTRL_REG6, &reg.byte,1);

    /* Configure interrupt pins */
    lis3dsh_interrupt_mode_get(&dev_ctx, &int_mode);
    int_mode.latched = PROPERTY_DISABLE;
    lis3dsh_interrupt_mode_set(&dev_ctx, &int_mode);
    lis3dsh_pin_int1_route_get(&dev_ctx, &int1_route);
    int1_route.fifo_full   = PROPERTY_ENABLE; /* Enable hardware notification */
    lis3dsh_pin_int1_route_set(&dev_ctx, &int1_route);

    /* Set Output Data Rate */
    lis3dsh_mode_get(&dev_ctx, &md);
    md.fs =  LIS3DSH_16g;
    md.odr = LIS3DSH_1kHz6;
    lis3dsh_mode_set(&dev_ctx, &md);

    /* Read samples in polling mode (no int). */
    while(1)
    {
        /* Read output only if new values are available */
        lis3dsh_all_sources_get(&dev_ctx, &all_sources);
        if ( all_sources.fifo_full )
        {
            lis3dsh_read_reg(&dev_ctx, LIS3DSH_FIFO_SRC, &reg.byte,1);
            lis3dsh_read_reg(&dev_ctx, LIS3DSH_OUT_X_L, &fifo_data[0].u8[0],reg.fifo_src.fss*6);

            /* print sensor data  */
            for (i = 0; i < reg.fifo_src.fss; i++)
            {
                sensor_printf(3,"lis3dsh Acceleration [mg]:%08.2f\t%08.2f\t%08.2f,i=%d",    (double)lis3dsh_from_fs4_to_mg(fifo_data[i].i16[0]),
                                                                                            (double)lis3dsh_from_fs4_to_mg(fifo_data[i].i16[1]),
                                                                                            (double)lis3dsh_from_fs4_to_mg(fifo_data[i].i16[2]),i);
            }
        }
        lis3dsh_delay(500);
    }
}

void lis3dsh_read_data_polling_test(void)
{
    /* Because the stack is small, the following variables are defined as static ,tanwenchen,2020-10-23*/
    static lis3dsh_data_t data;
    static lis3dsh_pin_int1_route_t int1_route;
    static lis3dsh_all_sources_t all_sources;
    static lis3dsh_bus_mode_t bus_mode;
    static lis3dsh_status_var_t status;
    static stmdev_ctx_t dev_ctx;
    static lis3dsh_id_t id;
    static lis3dsh_md_t md;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = lis3dsh_write;
    dev_ctx.read_reg = lis3dsh_read;
    dev_ctx.handle = NULL;

    /* Initialize platform specific hardware */
    lis3dsh_platform_init();

    /* Wait sensor boot time */
    lis3dsh_delay(BOOT_TIME);

    /* Check device ID */
    lis3dsh_id_get(&dev_ctx, &id);
    while (id.whoami != LIS3DSH_ID)
    {
        sensor_printf(3, "lis3dsh id(0x3F)=0x%X 0x%X 0x%X", id.info1, id.info2, id.whoami);
        lis3dsh_id_get(&dev_ctx, &id);
        lis3dsh_delay(WAIT_TIME);
    }
    sensor_printf(3, "lis3dsh id(0x3F)=0x%X 0x%X 0x%X", id.info1, id.info2, id.whoami);

    /* Restore default configuration */
    lis3dsh_init_set(&dev_ctx, LIS3DSH_RESET);
    do
    {
        lis3dsh_status_get(&dev_ctx, &status);
    } while (status.sw_reset);

    /* Set bdu and if_inc recommended for driver usage */
    lis3dsh_init_set(&dev_ctx, LIS3DSH_DRV_RDY);

    /* Select bus interface */
    bus_mode = LIS3DSH_SEL_BY_HW;
    lis3dsh_bus_mode_set(&dev_ctx, &bus_mode);

    /* Set Output Data Rate */
    lis3dsh_mode_get(&dev_ctx, &md);
    md.fs =  LIS3DSH_4g;
    md.odr = LIS3DSH_25Hz;
    lis3dsh_mode_set(&dev_ctx, &md);

    /* Configure inerrupt pins */
    lis3dsh_pin_int1_route_get(&dev_ctx, &int1_route);
    int1_route.drdy_xl   = PROPERTY_DISABLE;
    lis3dsh_pin_int1_route_get(&dev_ctx, &int1_route);

    /* Read samples in polling mode (no int). */
    while(1)
    {
        /* Read output only if new values are available */
        lis3dsh_all_sources_get(&dev_ctx, &all_sources);
        if ( all_sources.drdy_xl )
        {
            lis3dsh_data_get(&dev_ctx, &md, &data);
            sensor_printf(3,"lis3dsh Acceleration [mg]:%08.2f\t%08.2f\t%08.2f", (double)data.xl.mg[0],
                                                                                (double)data.xl.mg[1],
                                                                                (double)data.xl.mg[2]);
        }
        lis3dsh_delay(500);
    }
}

#endif
