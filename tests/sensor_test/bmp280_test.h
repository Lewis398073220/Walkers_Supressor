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
#ifndef __BMP280_TEST_H
#define __BMP280_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "bmp280.h"

void bmp280_cfg(uint8_t enable_init_interface,struct bmp280_dev *bmp,uint8_t enable_close_interface);
void bmp280_outputdata(struct bmp280_dev *bmp);
void bmp280_test(void);

#ifdef __cplusplus
}
#endif

#endif