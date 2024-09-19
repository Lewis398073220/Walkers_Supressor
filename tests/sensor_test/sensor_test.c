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
 ****************************************************************************/
#ifdef SENSOR_TEST

#include "cmsis.h"
#ifdef RTOS
    #include "cmsis_os.h"
#endif
#include "sensor_test.h"
#include "hal_trace.h"
#include "lis3dsh_test.h"
#include "lsm6dsl_test.h"
#include "lsm6dsox_test.h"
#include "bmp280_test.h"
#include "bmx055_test.h"

#ifdef HEAD_TRACK_ENABLE
    #include "lsm6dsox_ht_test.h"
#endif

void sensor_test(void)
{
#ifndef CHIP_SUBSYS_SENS    /* host mcu test */
    TRACE(0, "host mcu sensor test start......");
    TRACE(0, "  ");

    /*********************************************************************
        Only one of the following 5 sensors can be tested at the same time
    **********************************************************************/
    bmp280_test();

    lsm6dsox_self_test();
    lsm6dsox_read_data_polling();

    lsm6dsl_self_test();
    lsm6dsl_read_data_polling_test();

    lis3dsh_self_test();
    lis3dsh_fifo_stream_test();
    lis3dsh_read_data_polling_test();

    bmx055_test();

#else   /* sens mcu test */
    TRACE(0, "Sensor mcu sensor test start......");
    TRACE(0, "  ");

    /*********************************************************************
        Only one of the following 5 sensors can be tested at the same time
    **********************************************************************/
#ifdef HEAD_TRACK_ENABLE
    lsm6dsox_read_data_sending();
#else
    lsm6dsox_self_test();
    lsm6dsox_read_data_polling();

    bmp280_test();

    lsm6dsl_self_test();
    lsm6dsl_read_data_polling_test();

    lis3dsh_self_test();
    lis3dsh_fifo_stream_test();
    lis3dsh_read_data_polling_test();

    bmx055_test();
#endif
#endif
}

#endif

