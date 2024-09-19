#include "cmsis_os.h"
#include "cmsis.h"
#include "ble_app_dbg.h"
#include "ota_basic.h"
#include "cqueue.h"
#include "app_bt_func.h"

#define OTA_RX_EVENT_MAX_MAILBOX    16
#define OTA_RX_BUF_SIZE             (2048)

extern void ota_handle_data(uint8_t *otaBuf, bool isViaBle,uint16_t dataLenth);

#if (BLE_APP_OTA)
extern uint8_t app_ota_get_conidx(void);
extern void app_ota_send_rx_cfm(uint8_t conidx);
#endif

static uint8_t ota_rx_buf[OTA_RX_BUF_SIZE];
static CQueue ota_rx_cqueue;

static osThreadId ota_rx_thread = NULL;
static bool ble_rx_thread_init_done = false;

static void ota_rx_handler_thread(const void *arg);
osThreadDef(ota_rx_handler_thread, osPriorityNormal, 1, 2048, "ble_rx");

osMailQDef (ble_rx_event_mailbox, OTA_RX_EVENT_MAX_MAILBOX, OTA_RX_EVENT_T);
static osMailQId ota_rx_event_mailbox = NULL;
static int32_t ota_rx_event_mailbox_init(void)
{
    ota_rx_event_mailbox = osMailCreate(osMailQ(ble_rx_event_mailbox), NULL);
    if (ota_rx_event_mailbox == NULL) {
        LOG_I("Failed to Create ota_rx_event_mailbox");
        return -1;
    }
    return 0;
}

static void update_init_state(bool state)
{
    ble_rx_thread_init_done = state;
}

bool get_init_state(void)
{
    return ble_rx_thread_init_done;
}

static void ota_rx_mailbox_free(OTA_RX_EVENT_T* rx_event)
{
    osStatus status;

    status = osMailFree(ota_rx_event_mailbox, rx_event);
    ASSERT(osOK == status, "Free ble rx event mailbox failed!");
}

void ota_rx_handler_init(void)
{
    if (!get_init_state())
    {
        InitCQueue(&ota_rx_cqueue, OTA_RX_BUF_SIZE, ( CQItemType * )ota_rx_buf);
        ota_rx_event_mailbox_init();

        ota_rx_thread = osThreadCreate(osThread(ota_rx_handler_thread), NULL);
        update_init_state(true);
    }
    else
    {
        LOG_I("rx already initialized");
    }
}

void ota_push_rx_data(uint8_t flag, uint8_t conidx, uint8_t* ptr, uint16_t len)
{
    uint32_t lock = int_lock();
    int32_t ret = EnCQueue(&ota_rx_cqueue, ptr, len);
    int_unlock(lock);
    ASSERT(CQ_OK == ret, "BLE rx buffer overflow! %d,%d",AvailableOfCQueue(&ota_rx_cqueue),len);

    OTA_RX_EVENT_T* event = (OTA_RX_EVENT_T*)osMailAlloc(ota_rx_event_mailbox, 0);
    event->flag = flag;
    event->conidx = conidx;
    event->len = len;

    #if (BLE_APP_OTA)
    if (BLE_OTA_RX_DATA == flag) {
        app_ota_send_rx_cfm(app_ota_get_conidx());
    }
    #endif
    osMailPut(ota_rx_event_mailbox, event);
}

static int32_t ota_rx_mailbox_get(OTA_RX_EVENT_T** rx_event)
{
    osEvent evt;
    evt = osMailGet(ota_rx_event_mailbox, osWaitForever);
    if (evt.status == osEventMail) {
        *rx_event = (OTA_RX_EVENT_T *)evt.value.p;
        LOG_I("flag %d len %d", (*rx_event)->flag, (*rx_event)->len);
        return 0;
    }
    return -1;
}

static void ota_rx_handler_thread(void const *argument)
{
    while (true)
    {
        OTA_RX_EVENT_T* rx_event = NULL;
        if (!ota_rx_mailbox_get(&rx_event))
        {
            bool isViaBle = true;
            uint8_t tmpData[672];
            uint32_t lock = int_lock();
            DeCQueue(&ota_rx_cqueue, tmpData, rx_event->len);
            int_unlock(lock);

            switch (rx_event->flag)
            {
            case SPP_OTA_RX_DATA:
            case SPP_OTA_OVER_TOTA_RX_DATA: {
                isViaBle = false;
                break;
            }
            case BLE_OTA_RX_DATA:
            case BLE_OTA_OVER_TOTA_RX_DATA: {
#if defined(__GATT_OVER_BR_EDR__)
                isViaBle = false;
#endif
                break;
            }
            default:
                break;
            }
            ota_handle_data(tmpData, isViaBle, rx_event->len);
            ota_rx_mailbox_free(rx_event);
        }
    }
}

