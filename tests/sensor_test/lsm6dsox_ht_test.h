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
#ifndef __LSM6DSOX_HT_TEST_H
#define __LSM6DSOX_HT_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif

void lsm6dsox_read_data_sending(void);

#ifdef __cplusplus
}
#endif

#endif