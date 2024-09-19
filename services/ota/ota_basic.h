/***************************************************************************
 *
 * Copyright 2015-2019 BES.
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
#ifndef __APP_OTA_RX_HANDLER_H__
#define __APP_OTA_RX_HANDLER_H__

#include "rwapp_config.h"

#ifdef __cplusplus
extern "C" {
#endif  // #ifdef __cplusplus

enum ble_rx_data_handler_num {
    BLE_OTA_RX_DATA,
    SPP_OTA_RX_DATA,
    BLE_OTA_OVER_TOTA_RX_DATA,
    SPP_OTA_OVER_TOTA_RX_DATA,
};

typedef struct {
    uint8_t  flag;
    uint8_t  conidx;
    uint16_t len;
} OTA_RX_EVENT_T;

void ota_rx_handler_init(void);

void ota_push_rx_data(uint8_t flag, uint8_t conidx, uint8_t *ptr, uint16_t len);
bool get_init_state(void);

#ifdef __cplusplus
}
#endif  // #ifdef __cplusplus

#endif  // #ifndef __APP_OTA_RX_HANDLER_H__
