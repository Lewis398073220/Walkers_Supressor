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
#ifndef APP_GFPS_H_
#define APP_GFPS_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 *
 * @brief Device Information Application Module Entry point.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW Configuration
#include "gap.h"
#include "app_key.h"

#if (BLE_APP_GFPS)

#include <stdint.h>
/*
 * DEFINES
 ****************************************************************************************
 */
// enable IS_USE_CUSTOM_FP_INFO if wanna use custom fastpair tx power, model id and anti-proof key
#define IS_USE_CUSTOM_FP_INFOx

/// Manufacturer Name Value
#define APP_GFPS_MANUFACTURER_NAME       ("RivieraWaves SAS")
#define APP_GFPS_MANUFACTURER_NAME_LEN   (16)

/// Model Number String Value
#define APP_GFPS_MODEL_NB_STR            ("RW-BLE-1.0")
#define APP_GFPS_MODEL_NB_STR_LEN        (10)

/// Serial Number
#define APP_GFPS_SERIAL_NB_STR           ("1.0.0.0-LE")
#define APP_GFPS_SERIAL_NB_STR_LEN       (10)

/// Firmware Revision
#define APP_GFPS_FIRM_REV_STR            ("6.1.2")
#define APP_GFPS_FIRM_REV_STR_LEN        (5)

/// System ID Value - LSB -> MSB
#define APP_GFPS_SYSTEM_ID               ("\x12\x34\x56\xFF\xFE\x9A\xBC\xDE")
#define APP_GFPS_SYSTEM_ID_LEN           (8)

/// Hardware Revision String
#define APP_GFPS_HARD_REV_STR           ("1.0.0")
#define APP_GFPS_HARD_REV_STR_LEN       (5)

/// Software Revision String
#define APP_GFPS_SW_REV_STR              ("6.3.0")
#define APP_GFPS_SW_REV_STR_LEN          (5)

/// IEEE
#define APP_GFPS_IEEE                    ("\xFF\xEE\xDD\xCC\xBB\xAA")
#define APP_GFPS_IEEE_LEN                (6)

/**
 * PNP ID Value - LSB -> MSB
 *      Vendor ID Source : 0x02 (USB Implementer’s Forum assigned Vendor ID value)
 *      Vendor ID : 0x045E      (Microsoft Corp)
 *      Product ID : 0x0040
 *      Product Version : 0x0300
 */
#define APP_GFPS_PNP_ID               ("\x02\x5E\x04\x40\x00\x00\x03")
#define APP_GFPS_PNP_ID_LEN           (7)

#define APP_GFPS_ADV_POWER_UUID             "\x02\x0a\xf5"
#define APP_GFPS_ADV_POWER_UUID_LEN         (3)
#define APP_GFPS_ADV_APPEARANCE_UUID        "\x03\x19\xda\x96"
#define APP_GFPS_ADV_APPEARANCE_UUID_LEN    (4)
#define APP_GFPS_ADV_MANU_SPE_UUID_TEST     "\x07\xFF\xe0\x00\x01\x5B\x32\x01"
#define APP_GFPS_ADV_MANU_SPE_UUID_LEN      (8)

#define IN_USE_ACCOUNT_KEY_HEADER           (0x06)
#define NONE_IN_USET_ACCOUNT_KEY_HEADER     (0x04)
#define MOST_RECENT_USED_ACCOUNT_KEY_HEADER (0x05)

#define APP_GFPS_RANDOM_RESOLVABLE_DATA_TYPE    (0x06)

#define BLE_FASTPAIR_NORMAL_ADVERTISING_INTERVAL (160)
#define BLE_FASTPAIR_FAST_ADVERTISING_INTERVAL (48)
#define BLE_FASTPAIR_SPOT_ADVERTISING_INTERVAL (1000)


#if (BLE_APP_HID)
#define APP_GFPS_FEATURES             (GFPSP_MANUFACTURER_NAME_CHAR_SUP |\
                                      GFPSP_MODEL_NB_STR_CHAR_SUP      |\
                                      GFPSP_SYSTEM_ID_CHAR_SUP         |\
                                      GFPSP_PNP_ID_CHAR_SUP)
#else
#define APP_GFPS_FEATURES             (GFPSP_ALL_FEAT_SUP)
#endif //(BLE_APP_HID)
typedef void (*gfps_enter_pairing_mode)(void);
typedef uint8_t (*gfps_bt_io_cap_set)(uint8_t mode);
typedef uint8_t (*gfps_bt_io_authrequirements_set)(uint8_t authrequirements);

typedef void (*gfps_get_battery_info_handler)(uint8_t* batteryValueCount,
    uint8_t* batteryValue);

typedef enum
{
    HIDE_SUBSEQUENT_INDICATION = 2,
    SHOW_UI_INDICATION = 3,
    HIDE_UI_INDICATION = 4
} GFPS_BATTERY_DATA_TYPE_E;

typedef enum
{
    BATTERY_NOT_CHARGING = 0,
    BATTERY_CHARGING = 1,    
} GFPS_BATTERY_STATUS_E;

typedef enum {
    FP_SPEC_INIT,
    FP_SPEC_GFPS,
    FP_SPEC_SPOT,
    FP_SPEC_COUNT,
} FP_SPEC_TYPE_E;


#define GFPS_BATTERY_VALUE_MAX_COUNT    3
#ifdef SPOT_ENABLED
#define GFPS_NONCE_SIZE                 8
#define GFPS_AUTH_KEY_SIZE              8
#define GFPS_RECOVERY_KEY_SIZE          8
#define GFPS_RING_KEY_SIZE              8

#define GFPS_BEACON_READ_BEACON_PARAM                 0x00
#define GFPS_BEACON_READ_PROVISION_STATE              0x01
#define GFPS_BEACON_SET_EPHEMERAL_IDENTITY_KEY        0x02
#define GFPS_BEACON_CLEAR_EPHEMERAL_IDENTITY_KEY      0x03

#define GFPS_BEACON_READ_EPHEMERAL_IDENTITY_KEY       0x04
#define GFPS_BEACON_RING                              0x05
#define GFPS_BEACON_READ_RING_STATE                   0x06
#define GFPS_BEACON_ACTIVATE_UNWANTED_TRACK_MODE      0x07
#define GFPS_BEACON_DEACTIVATE_UNWANTED_TRACK_MODE    0x08

#define GFPS_BEACON_RINGING_STATE_STATED              0x00
#define GFPS_BEACON_RINGING_STATE_FAILED              0x01
#define GFPS_BEACON_RINGING_STATE_STOPPED_TIMEOUT     0x02
#define GFPS_BEACON_RINGING_STATE_STOPPED_PRESS       0x03
#define GFPS_BEACON_RINGING_STATE_STOPPED_REQUEST     0x04

#define GFPS_BEACON_RINGING_NONE                      0x00
#define GFPS_BEACON_RINGING_RIGHT                     0x01
#define GFPS_BEACON_RINGING_LEFT                      0x02
#define GFPS_BEACON_RINGING_RIGHT_AND_LEFT            0x03
#define GFPS_BEACON_RINGING_BOX                       0x04
#define GFPS_BEACON_RINGING_ALL                       0xFF

#define GFPS_BEACON_INCAPABLE_OF_RING                 0x00
#define GFPS_BEACON_ONE_CAPABLE_OF_RING               0x01
#define GFPS_BEACON_TWO_CAPABLE_OF_RING               0x02
#define GFPS_BEACON_THREE_CAPABLE_OF_RING             0x03
#define GFPS_BEACON_SECP_160R1_METHOD                 0x00
#define GFPS_BEACON_SECP_256R1_METHOD                 0x01

#define GFPS_BEACON_RINGING_VOLUME_NOT_AVAILABLE      0x00
#define GFPS_BEACON_RINGING_VOLUME_AVAILABLE          0x01

#define GFPS_BEACON_PROTOCOL_VERSION                  0x01
#define GFPS_BEACON_CONTROL_FLAG_SKIP_RING_AUT        0x01
#endif


struct app_gfps_env_tag
{
    uint8_t connectionIndex;
    uint8_t isKeyBasedPairingNotificationEnabled;
    uint8_t isPassKeyNotificationEnabled;
    uint8_t isInitialPairing;
    bd_addr_t seeker_bt_addr;
    bd_addr_t local_le_addr;
    bd_addr_t local_bt_addr;
    uint8_t keybase_pair_key[16];
    uint8_t aesKeyFromECDH[16];
    gfps_enter_pairing_mode enter_pairing_mode;
    gfps_bt_io_cap_set     bt_set_iocap;
    gfps_bt_io_authrequirements_set bt_set_authrequirements;
    uint8_t bt_iocap;
    uint8_t bt_authrequirements;
    uint8_t pendingLastResponse[16];
    uint8_t isLastResponsePending;
    uint8_t isWaitingForFpToConnect;
    uint8_t isPendingForWritingNameReq;	
    uint8_t isBatteryInfoIncluded;
    gfps_get_battery_info_handler get_battery_info_handler;  
    GFPS_BATTERY_DATA_TYPE_E batteryDataType;
    uint8_t advRandSalt;
    uint8_t isInFastPairing;
#ifdef SPOT_ENABLED
    uint8_t protocol_version;
    uint8_t nonce[8];
    uint8_t adv_identifer[20];
    uint8_t beacon_time[4];
    uint16_t remaining_ring_time;
    uint8_t control_flag;
    bool enable_unwanted_tracking_mode;
    uint8_t orignal_flag;
    uint8_t hashed_flag;
    uint8_t Ecc_private_key[20];
    bool ring_state;
#endif
};

typedef struct {
  uint32_t model_id;
  uint8_t public_anti_spoofing_key[64];
  uint8_t private_anti_spoofing_key[32];
} FastPairInfo;

/*
 * GLOBAL VARIABLES DECLARATION
 ****************************************************************************************
 */

/// Table of message handlers
extern const struct ke_state_handler app_gfps_table_handler;

/*
 * GLOBAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 ****************************************************************************************
 * @brief Initialize Device Information Service Application
 ****************************************************************************************
 */
void app_gfps_init(void);

/**
 ****************************************************************************************
 * @brief Add a Device Information Service instance in the DB
 ****************************************************************************************
 */
void app_gfps_add_gfps(void);

/**
 ****************************************************************************************
 * @brief Enable the Device Information Service
 ****************************************************************************************
 */
void app_gfps_enable_prf(uint16_t conhdl);

void app_gfps_connected_evt_handler(uint8_t conidx);
void app_gfps_disconnected_evt_handler(uint8_t conidx);

void app_gfps_set_bt_access_mode(gfps_enter_pairing_mode cb);
void app_gfps_set_io_cap(gfps_bt_io_cap_set cb);
void app_gfps_set_authrequirements(gfps_bt_io_authrequirements_set cb);

void app_gfps_generate_accountkey_filter(uint8_t* accountKeyFilter, uint8_t* filterSize);

uint8_t app_gfps_generate_accountkey_data(uint8_t* outputData);

void app_gfps_set_battery_info_acquire_handler(gfps_get_battery_info_handler cb);
void app_gfps_set_battery_datatype(GFPS_BATTERY_DATA_TYPE_E batteryDataType);
GFPS_BATTERY_DATA_TYPE_E app_gfps_get_battery_datatype(void);

void app_enter_fastpairing_mode(void);

void app_exit_fastpairing_mode(void);

bool app_is_in_fastpairing_mode(void);

void app_set_in_fastpairing_mode_flag(bool isEnabled);

uint8_t* app_gfps_get_last_response(void);

void app_gfps_enter_connectable_mode_req_handler(uint8_t* response);

bool app_gfps_is_last_response_pending(void);

void app_gfps_enable_battery_info(bool isEnable);

void app_fast_pair_timeout_handler(void);

bool app_is_mobile_connected_via_fastpair(void);

void app_gfps_get_battery_levels(uint8_t* pCount, uint8_t* pBatteryLevel);

void app_gfps_update_random_salt(void);

void app_gfps_tws_sync_init(void);

void app_bt_set_fast_pair_info(FastPairInfo fast_pair_info);

uint32_t app_bt_get_model_id(void);

const uint8_t* app_bt_get_fast_pair_public_key(void);

const uint8_t* app_bt_get_fast_pair_private_key(void);

void app_bt_get_fast_pair_info(void);

void app_gfps_set_intial_fast_pair_flag();
bool app_gfps_get_intial_fast_flag();
void big_little_switch(const uint8_t *in, uint8_t *out, uint8_t len);

#ifdef SPOT_ENABLED
void app_gfps_generate_nonce(void);
uint8_t* app_gfps_get_nonce(void);
void app_spot_press_stop_ring_handle(APP_KEY_STATUS *status, void *param);
void app_gfps_set_protocol_version(void);
uint8_t app_gfps_get_protocol_version(void);
#endif




#ifdef __cplusplus
}
#endif

#endif //BLE_APP_GFPS

/// @} APP

#endif //APP_GFPS_H_
