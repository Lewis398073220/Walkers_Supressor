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


#ifndef APP_ANCC_H_
#define APP_ANCC_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 *
 * @brief ANCC Application entry point.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_ANC_CLIENT
#include "ancc.h"

#include "app.h"                     // Application Definitions
#include "app_task.h"                // application task definitions
#include "co_bt.h"
#include "prf_types.h"
#include "prf_utils.h"
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "string.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define ANCC_MAX_LEN    (128)
#define ANCC_UID_LEN    (sizeof(uint32_t))
#define ANCC_CMD_ID_LEN (sizeof(enum anc_cmd_id))

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
typedef struct {
    uint8_t other;
    uint8_t incomingcall;
    uint8_t missedcall;
    uint8_t voicemail;
    uint8_t social;
    uint8_t schedual;
    uint8_t email;
    uint8_t news;
    uint8_t healthfitness;
    uint8_t businessfinance;
    uint8_t location;
    uint8_t entertrainment;
}anc_category_count;

typedef struct {
    uint8_t conidx;
    anc_category_count count;
}app_anc_notification_count;

typedef struct {
    /// Connection index
    uint8_t conidx;
    /// Attribute handle
    uint16_t hdl;
    /// Value length
    uint16_t len;
    /// Value to transmit
    uint8_t *data;
}app_anc_srv_param;

typedef struct {
    enum anc_ntf_attr_id attId;
    uint16_t attLen;
}app_anc_attr_param;

typedef struct {
    bool appId;
    bool title;
    bool subTitle;
    bool msg;
    bool msgSize;
    bool date;
    uint16_t titleLen;
    uint16_t subtitleLen;
    uint16_t msgLen;
}app_anc_get_msg_detail;

typedef struct {
    uint32_t ntfId;
    uint8_t *att;
    uint32_t attLen;
}app_anc_get_ntf_attr_param;

typedef struct {
    uint32_t appIdLen;
    char *appId;
    uint8_t *att;
    uint32_t attLen;
}app_anc_get_app_attr_param;

typedef struct {
    uint32_t ntfUid;
    enum anc_action_id actionId;
}app_anc_perform_ntf_act_param;

typedef struct {
    enum anc_cmd_id cmdId;
    uint8_t param[ANCC_MAX_LEN];
}app_anc_info_param;

typedef struct {
    uint8_t  connectionIndex;
    uint8_t  ControlPoint;
}app_ancc_env_info;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *
 * ANCC Application Functions
 *
 ****************************************************************************************
 */

void app_ancc_add_ancc(void);

/**
 ****************************************************************************************
 * @brief Inialize application and enable ANCC profile.
 *
 ****************************************************************************************
 */
void app_ancc_enable(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Read ANC attribute from the server with the given connection index and the 
 * attribute handle.
 *
 ****************************************************************************************
 */
void app_ancc_read(uint8_t conidx, uint16_t hdl);

/**
 ****************************************************************************************
 * @brief Write ANC attribute to the server with the given connection index and the 
 * attribute handle.
 *
 ****************************************************************************************
 */
void app_ancc_write(uint8_t conidx, uint16_t hdl, uint16_t value_length, uint8_t *value);

void app_ancc_parse_notification_info(app_anc_srv_param *param);

void app_ancc_disconnect_handle(uint8_t conidx);

void app_ancc_rx_queue_init(void);
int app_ancc_rx_queue_length(void);
int app_ancc_rx_queue_push(uint8_t *buff, uint16_t len);
int app_ancc_rx_queue_pop(uint8_t *buff, uint16_t len);
void app_ancc_rx_queue_deinit(void);
void app_ancc_need_to_get_message_detail(uint8_t conidx, uint32_t ntfuid);
#endif //BLE_ANC_CLIENT

/// @} APP

#endif // APP_ANCC_H_
