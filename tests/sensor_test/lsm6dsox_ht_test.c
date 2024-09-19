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
 * Application layer function of lsm6dsox 3D accelerometer and 3D gyroscope sensor
 * (Sensor data polling trigered by timer)
 ****************************************************************************/
#ifdef SENSOR_TEST
#include "hal_trace.h"
#include "hal_i2c.h"
#include "hal_timer.h"
#include "hal_spi.h"
#include "string.h"
#include "lsm6dsox_reg.h"
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
#define     spi_wire_mode      4   //3:3 wire mode,4:4 wire mode
#define     enable_spi_dma_rx  1   //0:disable,1:enable
#define     enable_spi_dma_tx  1   //0:disable,1:enable


typedef union{
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
    int16_t i16bit;
    uint8_t u8bit[2];
} axis1bit16_t;

#define IMU_DATA_FETCH_SIGNAL          (0x01)
#define IMU_DATA_FETCH_TIMER_INTERVAL   (10)

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME       1000//ms
#define    WAIT_TIME_A     200 //ms
#define    WAIT_TIME_G_01  150 //ms
#define    WAIT_TIME_G_02   50 //ms

/* Self test limits. */
#define    MIN_ST_LIMIT_mg         50.0f
#define    MAX_ST_LIMIT_mg       1700.0f
#define    MIN_ST_LIMIT_mdps   150000.0f
#define    MAX_ST_LIMIT_mdps   700000.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

#define     lsm6dsox_address    ((LSM6DSOX_I2C_ADD_H & 0xFE)>>1)
#define     sensor_printf       TRACE

#define BUFFER_SIZE_MAX 128
static uint8_t lsm6dsox_buffer[BUFFER_SIZE_MAX] = {0};

#if (i2c_spi_select == 0)
static enum HAL_I2C_ID_T i2c_id = HAL_I2C_ID_0;
#endif

static stmdev_ctx_t dev_ctx;
static int imu_data_fetch_task_inited = 0;


static int32_t lsm6dsox_write(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
    uint16_t i;
    uint32_t ret = 0;

    /* Write command */
    lsm6dsox_buffer[0] = reg;

    if(len > BUFFER_SIZE_MAX)
    {
        len = BUFFER_SIZE_MAX;
    }

    for(i = 0; i < len; i++)
    {
        lsm6dsox_buffer[i + 1] = bufp[i];
    }

#if (i2c_spi_select == 0)
    ret = hal_i2c_send(i2c_id, lsm6dsox_address, lsm6dsox_buffer, 1, len, 0, NULL);
#else

#if (enable_spi_dma_tx == 1)
    ret =hal_spilcd_dma_send(lsm6dsox_buffer, len+1, NULL);
#else
    ret =hal_spilcd_send(lsm6dsox_buffer, len+1);
#endif  //#if (enable_spi_dma_tx == 1)

#endif  //#if (i2c_spi_select == 0)
    return (int32_t)ret;
}

static int32_t lsm6dsox_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
#if (i2c_spi_select == 0)

    uint16_t i;
    uint32_t ret = 0;

    /* read command */
    lsm6dsox_buffer[0] = reg;

    if(len > BUFFER_SIZE_MAX)
    {
        len = BUFFER_SIZE_MAX;
    }

    ret = hal_i2c_recv(i2c_id, lsm6dsox_address, lsm6dsox_buffer, 1, len, HAL_I2C_RESTART_AFTER_WRITE, 0, 0);

    for(i= 0; i < len; i++)
    {
         bufp[i] = lsm6dsox_buffer[i + 1];
    }
#else
    int ret = 0;
    //uint8_t *str;
    uint8_t outBuf[len + 1];

    //str = (uint8_t *) malloc(len+1);

    /* Read command */
    reg |= 0x80;

#if (enable_spi_dma_rx == 1)
    ret =hal_spilcd_dma_recv(&reg, outBuf, len+1, NULL);
#else
    ret=hal_spilcd_recv(&reg, outBuf, len+1);
#endif  //#if (enable_spi_dma_rx == 1)

    memcpy(bufp,outBuf+1,len);//remove head

    //free(str);
#endif  //#if (i2c_spi_select == 0)
    return (int32_t)ret;
}

static void lsm6dsox_platform_init(void)
{
#if (i2c_spi_select == 0)
    struct HAL_I2C_CONFIG_T cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = HAL_I2C_API_MODE_TASK;//HAL_I2C_API_MODE_SIMPLE;
    cfg.use_dma = 0;    /*When DMA is enabled, the macro:I2C_USE_DMA=1 needs to be added ,tanwenchen,2020-11-27*/
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
#if (enable_spi_dma_rx == 1)
    spi_cfg.dma_rx = true;
#else
    spi_cfg.dma_rx = false;
#endif
#if (enable_spi_dma_tx == 1)
    spi_cfg.dma_tx = true;
#else
    spi_cfg.dma_tx = false;
#endif
    spi_cfg.rx_sep_line = false;
    spi_cfg.cs = 0;
    spi_cfg.rate = 6*1000*1000;
    spi_cfg.rx_bits = 8;
    spi_cfg.tx_bits = 8;
    spi_cfg.rx_frame_bits = 0;

    hal_iomux_set_spilcd();
    hal_spilcd_open(&spi_cfg);
#endif
}

static void lsm6dsox_delay(uint32_t ms)
{
    osDelay(ms);
}

static void lsm6dsox_boot_init(void)
{
    static uint8_t whoamI, rst;

    /* Wait sensor boot time */
    lsm6dsox_delay(WAIT_TIME_A);
#if (spi_wire_mode==3)
    uint8_t ctrl3_c=0x0C;
    dev_ctx.write_reg(NULL, LSM6DSOX_CTRL3_C, &ctrl3_c,1);
    lsm6dsox_delay(WAIT_TIME_A);
#endif
    /* Check device ID */
    lsm6dsox_device_id_get(&dev_ctx, &whoamI);
    while (whoamI != LSM6DSOX_ID)
    {
        sensor_printf(1, "lsm6dsox id(0x6C)=0x%X",whoamI);
        lsm6dsox_device_id_get(&dev_ctx, &whoamI);
        lsm6dsox_delay(WAIT_TIME_A);
    }
    sensor_printf(1, "lsm6dsox id(0x6C)=0x%X",whoamI);

    /* Restore default configuration */
    lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do
    {
        lsm6dsox_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Disable I3C interface */
    lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
#if (spi_wire_mode==3)
    dev_ctx.write_reg(NULL, LSM6DSOX_CTRL3_C, &ctrl3_c,1);//Because the sensor has been reset in the front, so I need to write it again here,tanwenchen,2020-11-17
    lsm6dsox_delay(WAIT_TIME_A);
#endif
    /* Enable Block Data Update */
    lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate */
    lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_1667Hz);
    lsm6dsox_delay(WAIT_TIME_A);
    lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_1667Hz);
    lsm6dsox_delay(WAIT_TIME_A);

    /* Set full scale */
    lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
    lsm6dsox_delay(WAIT_TIME_A);
    lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);
    lsm6dsox_delay(WAIT_TIME_A);

    /*
    * Configure filtering chain(No aux interface)
    *
    * Accelerometer - LPF1 + LPF2 path
    */
    lsm6dsox_xl_hp_path_on_out_set(&dev_ctx, LSM6DSOX_LP_ODR_DIV_100);
    lsm6dsox_delay(WAIT_TIME_A);
    lsm6dsox_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsox_delay(WAIT_TIME_A);
}

#ifdef HEAD_TRACK_ENABLE
    #include "sensor_hub_imu.h"
    void imu_data_set(float ax, float ay, float az, float gx, float gy, float gz, unsigned int timeStamp);
#endif

static void imu_data_fetcher()
{
    static axis3bit16_t data_raw_acceleration;
    static axis3bit16_t data_raw_angular_rate;
    static float acceleration_mg[3];
    static float angular_rate_mdps[3];
    uint8_t reg;

    /* Read output only if new xl value is available */
    lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &reg);
    if (reg)
    {
        /* Read acceleration field data */
        memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
        acceleration_mg[0] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);
    }

    lsm6dsox_gy_flag_data_ready_get(&dev_ctx, &reg);
    if (reg)
    {
        /* Read angular rate field data */
        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
        angular_rate_mdps[0] = lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] = lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] = lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);
    }
#ifdef HEAD_TRACK_ENABLE
    imu_data_set(acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
                    angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
                    FAST_TICKS_TO_US(hal_fast_sys_timer_get()));
#endif


    TRACE(TR_ATTR_NO_LF,"sensor_test ##########:%08d\t%08d\t%08d\t%08d\t",
                                                                    (int)TICKS_TO_MS(hal_sys_timer_get()),
                                                                    (int)(acceleration_mg[0]),
                                                                    (int)(acceleration_mg[1]),
                                                                    (int)(acceleration_mg[2]));
    TRACE(TR_ATTR_NO_TS|TR_ATTR_NO_ID,"%08d\t%08d\t%08d\t%08d##########:",
                                                                    (int)(angular_rate_mdps[0]),
                                                                    (int)(angular_rate_mdps[1]),
                                                                    (int)(angular_rate_mdps[2]),
                                                                    (int)0);

}

#define IMU_DATA_FETCH_THREAD_STACK_SIZE    1024
static void imu_data_fetch_task_thread(void const* argument);

static osThreadId imu_data_fetch_thread_id;
static osThreadDef(imu_data_fetch_task_thread, osPriorityNormal, 1, IMU_DATA_FETCH_THREAD_STACK_SIZE, "imu_data_fetch_thread");

static void imu_data_fetch_task_thread(void const*argument)
{
    osEvent evt;
    uint32_t signals = 0;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = lsm6dsox_write;
    dev_ctx.read_reg = lsm6dsox_read;
    dev_ctx.handle = NULL;
    /* Init test platform */
    lsm6dsox_platform_init();
    /* lsm6dsox init */
    lsm6dsox_boot_init();
    /* uint32_t start_time = FAST_TICKS_TO_US(hal_fast_sys_timer_get()); */

    while (1)
    {
        evt = osSignalWait(0x0, osWaitForever);
        signals = evt.value.signals;
        // TRACE(0, "[%s]:  hello signals=%x", __func__, signals);

        if (evt.status == osEventSignal) {
            if (signals & IMU_DATA_FETCH_SIGNAL) {

                imu_data_fetcher();

            }
        }
    }
}


static int imu_data_fetch_task_init(void)
{
    TRACE(0, "%s(true),%d \n\r", __FUNCTION__, __LINE__);

    if(!imu_data_fetch_task_inited)
    {
        imu_data_fetch_thread_id = osThreadCreate(osThread(imu_data_fetch_task_thread), NULL);
        if (imu_data_fetch_thread_id == NULL) {
            TRACE(0, "create imu_data_fetch_thread failed");
            return -1;
        }

        imu_data_fetch_task_inited = 1;
    }

    return 0;
}


// static int imu_data_fetch_task_deinit(void)
// {
//     // TODO: thread exit
//     TRACE(0, "%s,%d head_track_thread exit.\n\r", __FUNCTION__, __LINE__);

//     return 0;
// }

void imu_data_fetch_timer_handler(void const *param);
osTimerDef (imu_timer, imu_data_fetch_timer_handler);
osTimerId imu_data_fetch_timer_id = NULL;

void imu_data_fetch_timer_handler(void const *param)
{
    osSignalSet(imu_data_fetch_thread_id, IMU_DATA_FETCH_SIGNAL);
}

void lsm6dsox_read_data_sending(void)
{
    /* task inited here */
    imu_data_fetch_task_init();

    /* timer inited here */
    imu_data_fetch_timer_id = osTimerCreate(osTimer(imu_timer), osTimerPeriodic, NULL);

    /* start imu data fetch timer to release signals */
    if (imu_data_fetch_timer_id != NULL) {
        osTimerStart(imu_data_fetch_timer_id, IMU_DATA_FETCH_TIMER_INTERVAL);
    }
    return;
}

#endif