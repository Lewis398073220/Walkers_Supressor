#ifdef __GATT_OVER_BR_EDR__
#ifndef __BTGATT_API_H__
#define __BTGATT_API_H__

//#include "cqueue.h"
#include "bluetooth.h"
#include "me_api.h"
#include "btgatt.h"

#define BTIF_BTGATT_EVENT_CONTROL_CONNECTED            0x21
#define BTIF_BTGATT_EVENT_CONTROL_DISCONNECTED         0x22
#define BTIF_BTGATT_EVENT_CONTROL_DATA_IND             0x23
#define BTIF_BTGATT_EVENT_CONTROL_DATA_SENT            0x24
#define BTIF_BTGATT_EVENT_CONTROL_SET_IDLE             0x30

#define L2CAP_HEADER_LEN    4
#define BTGATT_DATA_BUF_SIZE (L2CAP_CFG_MTU)
#define BD_ADDR_LEN 6

typedef uint16_t btif_btgatt_event;

typedef void (*btgatt_event_callback)(btif_btgatt_event event, uint8_t reason);

typedef void (*btif_btgatt_data_received_callback)(uint8_t*, uint16_t);

typedef struct
{
    uint16_t tx_buf_allocated_status;
    uint8_t data[BTGATT_PACKET_COUNT][BTGATT_DATA_BUF_SIZE];
}__attribute__((packed)) BTGATT_DATA_PACKET_T;

typedef struct
{
    BtgattChannel btgatt_chann;
    btgatt_event_callback btgatt_event_cb;
}btgatt_info_t;

#ifdef __cplusplus
extern "C" {
#endif
uint8_t btif_btgatt_is_connected(void);
void btif_btgatt_client_create(btif_remote_device_t *dev);
void btif_btgatt_server_create(void);
void btif_btgatt_data_received_register(btif_btgatt_data_received_callback callback);
void btif_btgatt_send_cmd(uint8_t* cmd, uint16_t len);
bt_status_t btif_btgatt_send_cmd_no_wait(uint8_t* cmd, uint16_t len);
void btif_btgatt_init(void);
uint8_t btif_btgatt_get_connection_index(void);
int32_t btif_btgatt_get_l2cap_buffer_size(void);
void btif_btgatt_get_device_address(uint8_t *addr);

void btif_btgatt_callback(BtgattChannel *chnl, BtgattCallbackParms *Info, uint8_t reason);
uint32_t btif_btgatt_get_l2cap_handle(void);
uint16_t btif_btgatt_get_conn_handle(void);
void btif_btgatt_addsdp(uint16_t pServiceUUID, uint16_t startHandle, uint16_t endHandle);
void btif_btgatt_register_user_callback(btgatt_event_callback cb);
uint8_t btif_btgatt_force_disconnect_btgatt_profile();

#if defined(IBRT)
uint32_t btif_btgatt_profile_save_ctx(btif_remote_device_t *rem_dev, uint8_t *buf, uint32_t buf_len);
uint32_t btif_btgatt_profile_restore_ctx(uint8_t *buf, uint32_t buf_len);
#endif

#ifdef __cplusplus
}
#endif

#endif
#endif
