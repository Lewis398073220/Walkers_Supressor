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


/**
 ****************************************************************************************
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"              // SW configuration

#if (BLE_APP_PRESENT)

#if (BLE_ANC_CLIENT)

#include "app_ancc.h"
#include "app_ancc_task.h"
#include "app_task.h"                  // Application Task API
#include "app_sec.h"

extern app_ancc_env_info ble_ancc_env;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles ANCS Enable response
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int ancc_anc_enable_rsp_handler(ke_msg_id_t const msgid,
                                 struct ancc_enable_rsp const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles ANC client write complete event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int _write_cmp_handler(ke_msg_id_t const msgid,
                              struct anc_cli_wr_cmp const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles ANC client received indication or notification event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int _ind_evt_handler(ke_msg_id_t const msgid,
                            struct anc_cli_ind_evt const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles ANC client received attribute value
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int _att_val_handler(ke_msg_id_t const msgid,
                            struct anc_cli_att_val const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id);

/// Default State handlers definition
const struct ke_msg_handler app_ancc_msg_handler_list[] =
{
    {ANCC_ENABLE_RSP,       (ke_msg_func_t)ancc_anc_enable_rsp_handler},
    {ANCC_WRITE_CMP,        (ke_msg_func_t)_write_cmp_handler},
    {ANCC_IND_EVT,          (ke_msg_func_t)_ind_evt_handler},
    {ANCC_ATT_VAL,          (ke_msg_func_t)_att_val_handler},
};

const struct ke_state_handler app_ancc_table_handler =
    {&app_ancc_msg_handler_list[0], (sizeof(app_ancc_msg_handler_list)/sizeof(struct ke_msg_handler))};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/
static int _write_cmp_handler(ke_msg_id_t const msgid,
                               struct anc_cli_wr_cmp const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_INSUFF_AUTHEN)
    {
        app_sec_send_security_req(param->conidx, BLE_AUTHENTICATION_LEVEL);
    }

    if (param->status == GAP_ERR_NO_ERROR && param->conidx == ble_ancc_env.connectionIndex)
    {
        ble_ancc_env.ControlPoint = false;
        if (app_ancc_rx_queue_length() > ANCC_UID_LEN)
        {
            uint32_t ntfuid = 0;
            app_ancc_rx_queue_pop((uint8_t *)&ntfuid, ANCC_UID_LEN);
            app_ancc_need_to_get_message_detail(ble_ancc_env.connectionIndex, ntfuid);
        }
    }
    return (KE_MSG_CONSUMED);
}

static int _ind_evt_handler(ke_msg_id_t const msgid,
                            struct anc_cli_ind_evt const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    TRACE(0, "Received ANC notification.W");

    uint8_t conidx = KE_IDX_GET(dest_id);
    app_anc_srv_param srvparam = 
    {
     .conidx = conidx,
     .hdl = param->hdl,
     .len = param->value_length,
     .data = (uint8_t*)(param->value),
    };

    DUMP8("0x%02x ", param->value, param->value_length);
    app_ancc_parse_notification_info(&srvparam);
    return (KE_MSG_CONSUMED);
}

static int _att_val_handler(ke_msg_id_t const msgid,
                            struct anc_cli_att_val const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // TODO: implement the attribute value received handling
    return (KE_MSG_CONSUMED);
}

#endif //BLE_ANC_CLIENT

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
