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
 * Application layer function of bmp280 digital pressure and temperature sensor
 *  2020-10-22     tanwenchen    Initial version
 ****************************************************************************/
#ifdef SENSOR_TEST
#include "hal_trace.h"
#include "hal_i2c.h"
#include "hal_spi.h"
#include "string.h"
#include "bmp280.h"

#ifdef RTOS
#include "cmsis_os.h"
#ifdef KERNEL_RTX
#include "rt_Time.h"
#endif
#endif

#define     I2C_MODE_TASK
#define     i2c_number_select  14  //0:mcu  i2c0
                                   //1:mcu  i2c1
                                   //2:mcu  i2c2
                                   //3:mcu  i2c3
                                   //4:mcu  i2c4
                                   //5:mcu  i2c5
                                   //10:sens i2c0
                                   //11:sens i2c1
                                   //12:sens i2c2
                                   //13:sens i2c3
                                   //14:sens i2c4
#define     spi_number_select  0   //0:spi0
                                   //1:spilcd

#define     i2c_spi_select     0   //0:i2c,1:spi

#define     bmp280_address (BMP280_I2C_ADDR_PRIM)
#define     sensor_printf   TRACE

#define BUFFER_SIZE_MAX 128
static uint8_t bmp280_buffer[BUFFER_SIZE_MAX] = {0};

#if (i2c_spi_select == 0)
static uint8_t i2c_id = HAL_I2C_ID_0;
#endif

static int8_t bmp280_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    uint32_t ret = 0;
    uint16_t i;

    bmp280_buffer[0] = reg_addr;

    if(length > BUFFER_SIZE_MAX)
    {
        length = BUFFER_SIZE_MAX;
    }

    for(i = 0; i < length; i++)
    {
        bmp280_buffer[i + 1] = reg_data[i];
    }
#if (i2c_spi_select == 0)
#ifdef I2C_MODE_TASK
    ret = hal_i2c_send(i2c_id, i2c_addr, bmp280_buffer, 1, length, 0, NULL);
#else
    ret = hal_i2c_simple_send(i2c_id, i2c_addr, bmp280_buffer, 1 + length);
#endif
#else
    ret = hal_spi_send(bmp280_buffer, length + 1);
#endif
    return (int8_t)ret;
}

static int8_t bmp280_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
#if (i2c_spi_select == 0)
    uint32_t ret = 0;
    uint16_t i;

    bmp280_buffer[0] = reg_addr;

    if(length > BUFFER_SIZE_MAX)
    {
        length = BUFFER_SIZE_MAX;
    }

#ifdef I2C_MODE_TASK
    ret = hal_i2c_recv(i2c_id, i2c_addr, bmp280_buffer, 1, length, HAL_I2C_RESTART_AFTER_WRITE, 0, 0);
#else
    ret = hal_i2c_simple_recv(i2c_id, i2c_addr, bmp280_buffer, 1, bmp280_buffer + 1, length);
#endif
    for(i= 0; i < length; i++)
    {
        reg_data[i] = bmp280_buffer[i + 1];
    }
#else
    int ret = 0;
    uint8_t outBuf[length + 1];

    ret = hal_spi_recv(&reg_addr, outBuf, length + 1);
    memcpy(reg_data, outBuf + 1, length);//remove head
#endif
    return (int8_t)ret;
}

static void bmp280_platform_init(void)
{
#if (i2c_spi_select == 0)
    struct HAL_I2C_CONFIG_T cfg;

    memset(&cfg, 0, sizeof(cfg));
#ifdef I2C_MODE_TASK
    cfg.mode = HAL_I2C_API_MODE_TASK;
#else
    cfg.mode = HAL_I2C_API_MODE_SIMPLE;
#endif
    cfg.use_dma = 0;
    cfg.use_sync = 1;
    cfg.speed = 100*1000;
    cfg.as_master = 1;

#ifdef I2C_MODE_TASK
    TRACE(0, "i2c cfg.mode      : task mode");
#else
    TRACE(0, "i2c cfg.mode      : simple mode");
#endif
    TRACE(0, "i2c cfg.use_dma   : %d", cfg.use_dma);
    TRACE(0, "i2c cfg.use_sync  : %d", cfg.use_sync);
    TRACE(0, "i2c cfg.speed     : %d", cfg.speed);
    TRACE(0, "i2c cfg.as_master : %d", cfg.as_master);
    TRACE(0, "  ");

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
#ifdef I2C2_BASE
        case 2:
        {
            i2c_id = HAL_I2C_ID_2;
            hal_iomux_set_i2c2();
            break;
        }
#endif
#ifdef I2C3_BASE
        case 3:
        {
            i2c_id = HAL_I2C_ID_3;
            hal_iomux_set_i2c3();
            break;
        }
#endif
#ifdef I2C4_BASE
        case 4:
        {
            i2c_id = HAL_I2C_ID_4;
            hal_iomux_set_i2c4();
            break;
        }
#endif
#ifdef I2C5_BASE
        case 5:
        {
            i2c_id = HAL_I2C_ID_5;
            hal_iomux_set_i2c5();
            break;
        }
#endif
#ifdef CHIP_SUBSYS_SENS
        case 10:
        {
            i2c_id = HAL_I2C_ID_0;
            hal_iomux_set_i2c0();
            break;
        }
#ifdef I2C1_BASE
        case 11:
        {
            i2c_id = HAL_I2C_ID_1;
            hal_iomux_set_i2c1();
            break;
        }
#endif
#ifdef I2C2_BASE
        case 12:
        {
            i2c_id = HAL_I2C_ID_2;
            hal_iomux_set_i2c2();
            break;
        }
#endif
#ifdef I2C3_BASE
        case 13:
        {
            i2c_id = HAL_I2C_ID_3;
            hal_iomux_set_i2c3();
            break;
        }
#endif
#ifdef I2C4_BASE
        case 14:
        {
            i2c_id = HAL_I2C_ID_4;
            hal_iomux_set_i2c4();
            break;
        }
#endif
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
#if (spi_number_select == 0)
    hal_iomux_set_spi();
#elif (spi_number_select == 1)
    hal_iomux_set_spilcd();
#endif
    hal_spi_open(&spi_cfg);
#endif
}

static void bmp280_delay(uint32_t ms)
{
    osDelay(ms);
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
static void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        sensor_printf(1,"%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
        {
            sensor_printf(1,"Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            sensor_printf(1,"Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            sensor_printf(1,"Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            sensor_printf(1,"Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            sensor_printf(1,"Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

void bmp280_cfg(uint8_t enable_init_interface,struct bmp280_dev *bmp,uint8_t enable_close_interface)
{
    /* Because the stack is small, the following variables are defined as static ,tanwenchen,2020-10-23*/
    int8_t rslt;
    struct bmp280_config conf;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp->delay_ms = bmp280_delay;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp->dev_id = bmp280_address;

    /* Select the interface mode as I2C */
    bmp->intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp->read = bmp280_read;
    bmp->write = bmp280_write;

    if(enable_init_interface)
    {
        bmp280_platform_init();
    }
    /*
     * bmp.dev_id = 0;
     * bmp.read = spi_reg_read;
     * bmp.write = spi_reg_write;
     * bmp.intf = BMP280_SPI_INTF;
     */
    rslt = bmp280_init(bmp);
    print_rslt(" bmp280_init status", rslt);

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    rslt = bmp280_get_config(&conf, bmp);
    print_rslt(" bmp280_get_config status", rslt);

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Pressure oversampling set at 16x */
    conf.os_pres = BMP280_OS_16X;
    conf.os_temp = BMP280_OS_16X;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, bmp);
    print_rslt(" bmp280_set_config status", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);

    if(enable_close_interface)
    {
#if (i2c_spi_select == 0)
        hal_i2c_close(i2c_id);
#else
        hal_spi_close(0);
#endif
    }
}

void bmp280_outputdata(struct bmp280_dev *bmp)
{
    /* Because the stack is small, the following variables are defined as static ,tanwenchen,2020-10-23*/
    int8_t rslt;
    struct bmp280_uncomp_data ucomp_data;
    uint32_t pres32, pres64;
    double pres;
    int32_t temp32;
    double temp;

    while (1)
    {
        /*--------------------pres read---------------------*/
        /* Reading the raw data from sensor */
        rslt = bmp280_get_uncomp_data(&ucomp_data, bmp);

        /* Getting the compensated pressure using 32 bit precision */
        rslt = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, bmp);

        /* Getting the compensated pressure using 64 bit precision */
        rslt = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, bmp);

        /* Getting the compensated pressure as floating point value */
        rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, bmp);

        /*--------------------temp read---------------------*/
        /* Getting the 32 bit compensated temperature */
        rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, bmp);

        /* Getting the compensated temperature as floating point value */
        rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, bmp);
        //printf("UT: %ld, T32: %ld, T: %f \r\n", ucomp_data.uncomp_temp, temp32, temp);

        //The display needs to be divided into two times, otherwise there will be garbled code,2020-10-22
        sensor_printf(TR_ATTR_NO_LF,"UP: %d, P32: %d, P64: %d, P64N: %d ",
                                                                       ucomp_data.uncomp_press,
                                                                       pres32,
                                                                       pres64,
                                                                       pres64 / 256);
        sensor_printf(TR_ATTR_NO_TS|TR_ATTR_NO_ID,"P: %f UT: %d, T32: %d, T: %3.1f¡æ,rslt:%d",
                                                                       (double)pres,
                                                                       ucomp_data.uncomp_temp, temp32, (double)temp, rslt);

        bmp->delay_ms(1000); /* Sleep time between measurements = BMP280_ODR_1000_MS */
    }
}
void bmp280_test(void)
{
    /* Because the stack is small, the following variables are defined as static ,tanwenchen,2020-10-23*/
    static int8_t rslt;
    static struct bmp280_dev bmp;
    static struct bmp280_config conf;
    static struct bmp280_uncomp_data ucomp_data;
    static uint32_t pres32, pres64;
    static double pres;
    static int32_t temp32;
    static double temp;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = bmp280_delay;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = bmp280_address;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = bmp280_read;
    bmp.write = bmp280_write;

    bmp280_platform_init();

    /*
     * bmp.dev_id = 0;
     * bmp.read = spi_reg_read;
     * bmp.write = spi_reg_write;
     * bmp.intf = BMP280_SPI_INTF;
     */
    rslt = bmp280_init(&bmp);
    print_rslt(" bmp280_init status", rslt);

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    rslt = bmp280_get_config(&conf, &bmp);
    print_rslt(" bmp280_get_config status", rslt);

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Pressure oversampling set at 16x */
    conf.os_pres = BMP280_OS_16X;
    conf.os_temp = BMP280_OS_16X;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);
    while (1)
    {
        /*--------------------pres read---------------------*/
        /* Reading the raw data from sensor */
        rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

        /* Getting the compensated pressure using 32 bit precision */
        rslt = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);

        /* Getting the compensated pressure using 64 bit precision */
        rslt = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);

        /* Getting the compensated pressure as floating point value */
        rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);

        /*--------------------temp read---------------------*/
        /* Getting the 32 bit compensated temperature */
        rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);

        /* Getting the compensated temperature as floating point value */
        rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
        //printf("UT: %ld, T32: %ld, T: %f \r\n", ucomp_data.uncomp_temp, temp32, temp);

        //The display needs to be divided into two times, otherwise there will be garbled code,2020-10-22
        sensor_printf(TR_ATTR_NO_LF,"UP: %d, P32: %d, P64: %d, P64N: %d ",
                                                                       ucomp_data.uncomp_press,
                                                                       pres32,
                                                                       pres64,
                                                                       pres64 / 256);
        sensor_printf(TR_ATTR_NO_TS|TR_ATTR_NO_ID,"P: %f UT: %d, T32: %d, T: %3.1f¡æ",
                                                                       (double)pres,
                                                                       ucomp_data.uncomp_temp, temp32, (double)temp);

        bmp.delay_ms(1000); /* Sleep time between measurements = BMP280_ODR_1000_MS */
    }
}

#endif
