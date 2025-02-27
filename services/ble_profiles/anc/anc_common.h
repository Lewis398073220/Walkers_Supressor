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


#ifndef _ANC_COMMON_H_
#define _ANC_COMMON_H_

/**
 ****************************************************************************************
 * @addtogroup ANCS Profile
 * @ingroup ANCS
 * @brief ANCS Profile
 *****************************************************************************************
 */

#include "rwip_config.h"
#include "compiler.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_ANC_CLIENT || BLE_ANC_SERVER)

#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */

/// Error Codes
// Command not recognized by the NP - Protocol
#define ANC_CMD_UNKNOWN               (0xA0)
// Command improperly formatted - Proprietary
#define ANC_CMD_INVALID               (0xA1)
// One of the parameters does not refer to an existing object on the NP - Proprietary
#define ANC_PARAM_INVALID             (0xA2)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Category ID Values
enum
{
    /// Other
    CAT_ID_OTHER             = 0,
    /// Incoming Call
    CAT_ID_CALL              = 1,
    /// Missed Call
    CAT_ID_MISSED_CALL       = 2,
    /// Voice Mail
    CAT_ID_VOICE_MAIL        = 3,
    /// Social network
    CAT_ID_SOCIAL            = 4,
    /// Schedule
    CAT_ID_SCHEDULE          = 5,
    /// Email
    CAT_ID_EMAIL             = 6,
    /// News Feed
    CAT_ID_NEWS              = 7,
    /// Health and Fitness
    CAT_ID_HEALTH_FITNESS    = 8,
    /// Business and Finance
    CAT_ID_BUSINESS_FINANCE  = 9,
    /// Location
    CAT_ID_LOCATION          = 10,
    /// Entertainment
    CAT_ID_ENTERTAINMENT     = 11,

    CAT_ID_NB,

    /// All supported categories
    CAT_ID_ALL_SUPPORTED_CAT = 255,
};

/// Event ID Values
enum
{
    /// Notification Added
    EVT_ID_NTF_ADDED        = 0,
    /// Notification Modified
    EVT_ID_NTF_MODIFIED     = 1,
    /// Notification Removed
    EVT_ID_NTF_REMOVED      = 2,

    EVT_ID_NB,

    EVT_ID_ALL_SUPPORTED_EVT = 255,
};

/// Event Flags
enum
{
    /// Silent
    EVT_FLAG_SILENT                 = (1 << 0),
    /// Important
    EVT_FLAG_IMPORTANT              = (1 << 1),

    EVT_FLAG_NB,

    EVT_FLAG_ALL_SUPPORTED_EVT_FLAG = (1 << 7),
};

/// Command ID Values
enum
{
    /// Get Notification Attributes
    CMD_ID_GET_NTF_ATTS      = 0,
    /// Get App Attributes
    CMD_ID_GET_APP_ATTS      = 1,


    CMD_ID_NB,

    /// All supported commands
    CMD_ID_ALL_SUPPORTED_CMD = 255,
};

/// Notification Attribute ID Values
enum
{
    /// App Identifier
    NTF_ATT_ID_APP_ID        = 0,
    /// Title (Needs to be followed by a 2-bytes max length parameter)
    NTF_ATT_ID_TITLE         = 1,
    /// Subtitle (Needs to be followed by a 2-bytes max length parameter)
    NTF_ATT_ID_SUBTITLE      = 2,
    /// Message (Needs to be followed by a 2-bytes max length parameter)
    NTF_ATT_ID_MSG           = 3,
    /// Message Size
    NTF_ATT_ID_MSG_SIZE      = 4,
    /// Date
    NTF_ATT_ID_DATE          = 5,


    NTF_ATT_ID_NB,

    /// All supported Notification Attributes
    NTF_ATT_ID_ALL_SUPPORTED_NTF_ATT = 255,
};

/// Notification Attribute Bit Mask Flags
enum ancc_ntf_attr_present
{
    /// App Identifier present
    NTF_ATT_ID_APP_ID_PRESENT        = 0x01,
    /// Title present
    NTF_ATT_ID_TITLE_PRESENT         = 0x02,
    /// Subtitle present
    NTF_ATT_ID_SUBTITLE_PRESENT      = 0x04,
    /// Message present
    NTF_ATT_ID_MSG_PRESENT           = 0x08,
    /// Message Size present
    NTF_ATT_ID_MSG_SIZE_PRESENT      = 0x10,
    /// Date present
    NTF_ATT_ID_DATE_PRESENT          = 0x20,
};


/// App Attribute ID Values
enum
{
    /// Display Name present
    APP_ATT_ID_DISPLAY_NAME  = 0,
    
    APP_ADD_ID_NB,

    /// All supported App Attributes
    APP_ATT_ID_ALL_SUPPORTED_APP_ATT = 255,
};

// App Attribute Bit Mask Flags
enum
{
    /// App Identifier
    APP_ATT_ID_DISPLAY_NAME_PRESENT  = 0x01,
    
};


/*
 * STRUCTURES
 ****************************************************************************************
 */

/// Notification Source Characteristic Value Structure
struct anc_ntf_src
{
    /// Event ID
    uint8_t event_id;

    /// Event Flags
    uint8_t event_flags;

    /// Category ID
    uint8_t cat_id;

    /// Category Count: Number of active notifications in the given category
    uint8_t cat_cnt;

    /// Notification UID
    uint32_t ntf_uid;
};

typedef struct anc_srv_rd_evt
{
    /// Token provided by GATT module that must be used into the GAT_CFM message
    uint16_t token;
    /// GATT user local identifier
    uint8_t user_lid;
    /// Connection index
    uint8_t conidx;
    /// Attribute handle
    uint16_t hdl;
} anc_srv_rd_evt_t;

typedef struct anc_cli_rd_cmd
{
    /// Token provided by GATT module that must be used into the GAT_CFM message
    uint16_t token;
    /// GATT user local identifier
    uint8_t user_lid;
    /// Connection index
    uint8_t conidx;
    /// Attribute handle
    uint16_t hdl;
} anc_cli_rd_cmd_t;

typedef struct anc_srv_wr_evt
{
    /// Token provided by GATT module that must be used into the GAT_CFM message
    uint16_t token;
    /// GATT user local identifier
    uint8_t user_lid;
    /// Connection index
    uint8_t conidx;
    /// Attribute handle of client
    uint16_t hdl;
    /// Value offset, valid only for GATT_WRITE
    uint16_t offset;
    /// Value length to write
    uint16_t value_length;
    /// Attribute value
    uint8_t value[__ARRAY_EMPTY];
} anc_srv_wr_evt_t;

typedef struct anc_cli_wr_cmd
{
    /// Command code (@see enum gatt_cmd_code)
    ///  - GATT_CLI_WRITE
    uint16_t        cmd_code;
    /// Dummy parameter whose meaning is upper layer dependent and which is returned in command complete event and
    /// indications sent during command handling. It can be used as a sequence number for instance.
    uint16_t        dummy;
    /// GATT user local identifier
    uint8_t         user_lid;
    /// Connection index
    uint8_t         conidx;
    /// GATT write type (@see enum gatt_write_type)
    uint8_t         write_type;
    /// Attribute handle
    uint16_t        hdl;
    /// Value offset, valid only for GATT_WRITE
    uint16_t        offset;
    /// Value length to write
    uint16_t        value_length;
    /// Attribute value
    uint8_t         value[__ARRAY_EMPTY];
} anc_cli_wr_cmd_t;

typedef struct defer_cfm_info
{
    uint8_t conidx;
    uint8_t user_lid;
    uint16_t token;
    uint8_t status;
} defer_cfm_info_t;

typedef struct anc_cli_wr_cmp
{
    /// Connection index
    uint8_t conidx;
    /// GATT user local identifier
    uint8_t user_lid;
    /// write complete status
    uint16_t status;
} anc_cli_wr_cmp_t;

typedef struct anc_cli_ind_evt
{
    /// Command code (@see enum gatt_cmd_code)
    ///  - GATT_SRV_EVENT_SEND
    uint16_t        cmd_code;
    /// Dummy parameter whose meaning is upper layer dependent and which is returned in command complete event and
    /// indications sent during command handling. It can be used as a sequence number for instance.
    uint16_t        dummy;
    /// GATT user local identifier
    uint8_t         user_lid;
    /// Connection index
    uint8_t         conidx;
    /// Event type to trigger (@see enum gatt_evt_type)
    uint8_t         evt_type;
    /// Attribute handle
    uint16_t        hdl;
    /// Value length
    uint16_t        value_length;
    /// Value to transmit
    uint8_t         value[__ARRAY_EMPTY];
} anc_cli_ind_evt_t;

typedef struct anc_cli_att_val
{
    /// Connection index
    uint8_t         conidx;
    /// GATT user local identifier
    uint8_t         user_lid;
    /// Dummy parameter whose meaning is upper layer
    uint16_t        dummy;
    /// Attribute handle
    uint16_t        hdl;
    /// Value offset
    uint16_t        offset;
    /// Value length
    uint16_t        value_length;
    /// Value to transmit
    uint8_t         value[__ARRAY_EMPTY];
} anc_cli_att_val_t;


#endif //(BLE_ANC_CLIENT || BLE_ANC_SERVER)

/// @} anc_common

#endif //(_ANC_COMMON_H_)
