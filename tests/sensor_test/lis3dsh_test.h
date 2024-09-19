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
#ifndef __LIS3DSH_TEST_H
#define __LIS3DSH_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif

void lis3dsh_self_test(void);
void lis3dsh_fifo_stream_test(void);
void lis3dsh_read_data_polling_test(void);

#ifdef __cplusplus
}
#endif

#endif