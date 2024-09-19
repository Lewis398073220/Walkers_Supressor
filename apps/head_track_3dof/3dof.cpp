/***************************************************************************
 *
 * Copyright 2015-2023 BES.
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
 * Application layer function of head tracking 3dof with lsm6dsl sensor
 ****************************************************************************/
#ifdef HEAD_TRACK_ENABLE
#include "hal_trace.h"
#include "hal_i2c.h"
#include "hal_timer.h"
#include "hal_spi.h"
#include "string.h"
#include "lsm6dsl_reg.h"
#include "stdlib.h"

#ifdef RTOS
#include "cmsis_os.h"
#ifdef KERNEL_RTX
#include "rt_Time.h"
#endif
#endif

#include "3dof.h"
#include "window_node.h"
#include "head_track_3dof.h"
#include "calibration_section.h"

#if defined(__VIRTUAL_SURROUND__) || defined(__VIRTUAL_SURROUND___) || defined(__VIRTUAL_SURROUND_STEREO__)
#include "app_bt_stream.h"
extern "C" int32_t audio_process_stereo_set_yaw(float yaw);
extern "C" int32_t audio_process_stereo_set_pitch(float pitch);
#endif

#ifdef HID_HEAD_TRACK
#include "bt_hid_service.h"
#include "app_bt_hid.h"
#include "app_bt.h"
#endif

#define HEAD_TRACK_SIGNAL_ALGO          (0x01)
#define HEAD_TRACK_TIMER_INTERVAL       (10)
#define HEAD_TRACK_TIMER_FREQ           (100)
#define SLIDE_WINDOW_SIZE               (3)
#define HID_DATA_RANGE                  (65536)
#define HID_ROTATE_BOUND                (3.14159265)
#define HID_ANG_VEL_BOUND               (32)

static int head_track_inited = 0;
static int head_track_algo_inited = 0;
static int head_track_send_inited = 0;
static bool is_head_angle_data_printing_enabled = false;
static bool is_ht_data_sending_enabled = true;
static int calibration_on = 0;

static osMutexId head_track_sensor_mutex_id = NULL;
osMutexDef(head_track_sensor_mutex);

static osMutexId head_track_pose_mutex_id = NULL;
osMutexDef(head_track_pose_mutex);

static osSemaphoreId sensor_signal = NULL;
osSemaphoreDef(sensor_signal);

static struct WindowNode* head = NULL;

void head_track_window_link_init(void)
{
    for (int i = 0; i < SLIDE_WINDOW_SIZE; i++) {
        insertNode(&head, 0.0);
    }
}

void head_track_window_link_release(void)
{
    struct WindowNode* current = head;
    while (current != NULL) {
        WindowNode* temp = current;
        current = current->next;
        free(temp);
    }
}

static struct SENSOR_IMU preImu;
void imu_data_set(float ax, float ay, float az, float gx, float gy, float gz, unsigned int timeStamp)
{
    // Accelerometer, unit mg
    preImu.a_x = ax;
    preImu.a_y = ay;
    preImu.a_z = az;

    // Gyroscope , unit mdeg/s
    preImu.g_x = gx;
    preImu.g_y = gy;
    preImu.g_z = gz;

    // timestamp unit us;  1s = 1000ms = 1000000us
    preImu.frameTimeStamp = timeStamp;
    preImu.rflag = 1;
}

void head_track_resouce_init(void)
{
    if(head_track_sensor_mutex_id == NULL) {
        head_track_sensor_mutex_id = osMutexCreate(osMutex(head_track_sensor_mutex));
        if (head_track_sensor_mutex_id == NULL) {
            TRACE(1, "Failed to Create head_track_sensor_mutex_id");
        }
    }

    if (head_track_pose_mutex_id == NULL) {
        head_track_pose_mutex_id = osMutexCreate(osMutex(head_track_pose_mutex));
        if (head_track_pose_mutex_id == NULL) {
            TRACE(1, "Failed to Create head_track_pose_mutex_id");
        }
    }

    if (sensor_signal == NULL) {
        sensor_signal = osSemaphoreCreate(osSemaphore(sensor_signal), 1);
        if (sensor_signal == NULL) {
            TRACE(1, "Failed to Create sensor_signal");
        }
    }

    if(head == NULL) {
        head_track_window_link_init();
        if(head == NULL) {
            TRACE(1, "Failed to Create head track window LinkList");
        }
    }
}

#define HEAD_TRACK_THREAD_STACK_SIZE    1024
static void head_track_task_thread(void const* argument);
static osThreadId head_track_thread_id;
static osThreadDef(head_track_task_thread, osPriorityNormal, 1, HEAD_TRACK_THREAD_STACK_SIZE, "head_track_thread");

#define POSE_SIZE (sizeof(struct POSE_S))
#define SENSOR_SIZE (sizeof(struct SENSOR_IMU))
static struct POSE_S pose_s;
static struct SENSOR_IMU sensor;
static void head_track_task_thread(void const*argument)
{
    osEvent evt;
    uint32_t signals = 0;

    while (1) {
        evt = osSignalWait(0x0, osWaitForever);
        signals = evt.value.signals;

        if (evt.status == osEventSignal) {
            if (signals & HEAD_TRACK_SIGNAL_ALGO) {
                osMutexWait(head_track_sensor_mutex_id, osWaitForever);
                memcpy(&sensor, &preImu, SENSOR_SIZE);
                osSemaphoreRelease(sensor_signal);
                osMutexRelease(head_track_sensor_mutex_id);
            }
        }
    }
}

#define HEAD_TRACK_ALGO_THREAD_STACK_SIZE    4096
static void head_track_algo_task_thread(void const* argument);
static osThreadId head_track_algo_thread_id;
static osThreadDef(head_track_algo_task_thread, osPriorityNormal, 1, HEAD_TRACK_ALGO_THREAD_STACK_SIZE, "head_track_algo_thread");

static void head_track_algo_task_thread(void const*argument)
{
    struct POSE_S algoPose;
    struct SENSOR_IMU localSensor;
    struct GYR_BIAS localBias = {0, 0, 0, 0};

    while (1)
    {
        osSemaphoreWait(sensor_signal, osWaitForever);
        osMutexWait(head_track_sensor_mutex_id, osWaitForever);
        memcpy(&localSensor, &sensor, SENSOR_SIZE);
        sensor.rflag = 0;
        osMutexRelease(head_track_sensor_mutex_id);

        if(calibration_on == 1) {
            TRACE(0, "%s: Calibration in progress. Please keep the sensor still...", __func__);
        }

        if(localSensor.rflag == 1) {
            head_track_3dof_algo_with_fix_param(&algoPose, &localSensor, &localBias);
        }

        osMutexWait(head_track_pose_mutex_id, osWaitForever);
        memcpy(&pose_s, &algoPose, POSE_SIZE);
        osMutexRelease(head_track_pose_mutex_id);

        if(localBias.v4 == 1) {
            calibration_on = 0;
            localBias.v4 = 0;

            float bArray[3];
            bArray[0] = localBias.v1;
            bArray[1] = localBias.v2;
            bArray[2] = localBias.v3;

            calibration_values_write(CALIBRATION_SECTION_DEVICE_HEAD_TRACK, (uint8_t *)bArray, sizeof(bArray));
        }
    }
}

#ifdef HID_HEAD_TRACK
void hid_head_track_data_formatter(struct bt_hid_sensor_report_t *report, POSE_S *pose_data)
{
    // map value to a range of -32767 to 32767
    report->vx = (int16_t)(pose_data->vx * HID_DATA_RANGE / HID_ANG_VEL_BOUND);
    report->vy = (int16_t)(pose_data->vy * HID_DATA_RANGE / HID_ANG_VEL_BOUND);
    report->vz = (int16_t)(pose_data->vz * HID_DATA_RANGE / HID_ANG_VEL_BOUND);
    report->rx = (int16_t)(pose_data->pitch * HID_DATA_RANGE / HID_ROTATE_BOUND);
    report->ry = (int16_t)(pose_data->yaw   * HID_DATA_RANGE / HID_ROTATE_BOUND);
    report->rz = (int16_t)(pose_data->roll  * HID_DATA_RANGE / HID_ROTATE_BOUND);
}
#endif


#define HEAD_TRACK_SEND_THREAD_STACK_SIZE    1024
static void head_track_send_task_thread(void const* argument);
static osThreadId head_track_send_thread_id;
static osThreadDef(head_track_send_task_thread, osPriorityNormal, 1, HEAD_TRACK_SEND_THREAD_STACK_SIZE, "head_track_send_thread");

#ifdef HID_HEAD_TRACK
static bool is_hid_connected = false;
static bool is_btaddr_written = false;
static bt_bdaddr_t bt_bdaddr;
#endif

void enable_head_angle_data_printing(void)
{
    is_head_angle_data_printing_enabled = true;
}

void disable_head_angle_data_printing(void)
{
    is_head_angle_data_printing_enabled = false;
}

static void head_track_send_task_thread(void const*argument)
{
    struct POSE_S send_pose;
    while (1)
    {
        if (!is_ht_data_sending_enabled)
        {
            osDelay(10);
            continue;
        }

        if (calibration_on == 1) {
            osDelay(10);
            continue;
        }

        osMutexWait(head_track_pose_mutex_id, osWaitForever);
        memcpy(&send_pose, &pose_s, POSE_SIZE);
        osMutexRelease(head_track_pose_mutex_id);
        /*
            set spatial audio with 3dof angle if necessary,
            the spatial audio APIs might be different,
            here is just a demo usage.

            also, the pose data can be sent to cell-phone or whatever, according to customer's wishes
        */

        if (is_head_angle_data_printing_enabled)
        {
            TRACE(0, "%s: pose.pitch=%06d pose.yaw=%06d", __func__, (int)(send_pose.pitch*100), (int)(send_pose.yaw*100));
        }

#if defined(__VIRTUAL_SURROUND__) || defined(__VIRTUAL_SURROUND___) || defined(__VIRTUAL_SURROUND_STEREO__)
        audio_process_stereo_set_yaw((float)pose_s.yaw);
        audio_process_stereo_set_pitch((float)pose_s.pitch);
#endif

#ifdef HID_HEAD_TRACK
        struct bt_hid_sensor_report_t head_track_ble_report;
        bt_status_t bt_status;
        uint8_t deviceId = 0;

        hid_head_track_data_formatter(&head_track_ble_report, &send_pose);

        if (!is_btaddr_written)
        {
            bool ret = app_bt_get_device_bdaddr(deviceId, bt_bdaddr.address);

            if (ret)
            {
                is_btaddr_written = true;
            }
        }
        else
        {
            is_hid_connected = app_bt_hid_is_hid_connected(&bt_bdaddr);

            if (!is_hid_connected)
            {
                bt_status = app_bt_hid_connect(&bt_bdaddr);
                if (BT_STS_SUCCESS == bt_status)
                {
                    is_hid_connected = true;
                }
            }
        }

        if (is_hid_connected)
        {
            bt_status = app_bt_hid_send_sensor_report(&bt_bdaddr, &head_track_ble_report);
        }
#endif

        osDelay(10);
    }
}

static int  head_track_task_init(void)
{
    TRACE(0, "%s(true),%d \n\r", __FUNCTION__, __LINE__);

    if (!head_track_inited) {
        head_track_thread_id = osThreadCreate(osThread(head_track_task_thread), NULL);
        if (head_track_thread_id == NULL) {
            TRACE(0, "create head_track_thread failed");
            return -1;
        }
        head_track_inited = 1;
    }
    return 0;
}

static int head_track_algo_task_init(void)
{
    TRACE(0, "%s(true),%d \n\r", __FUNCTION__, __LINE__);

    if(!head_track_algo_inited)
    {
        head_track_algo_thread_id = osThreadCreate(osThread(head_track_algo_task_thread), NULL);
        if (head_track_algo_thread_id == NULL) {
            TRACE(0, "create head_track_algo_thread failed");
            return -1;
        }

        head_track_algo_inited = 1;
    }
    return 0;
}

static int head_track_send_task_init(void)
{
    TRACE(0, "%s(true),%d \n\r", __FUNCTION__, __LINE__);

    if(!head_track_send_inited)
    {
        head_track_send_thread_id = osThreadCreate(osThread(head_track_send_task_thread), NULL);
        if (head_track_send_thread_id == NULL) {
            TRACE(0, "create head_track_send_thread failed");
            return -1;
        }

        head_track_send_inited = 1;
    }
    return 0;
}

void imu_timer_handler(void const *param);
osTimerDef (imu_timer, imu_timer_handler);
osTimerId imu_timer_id = NULL;

void imu_timer_handler(void const *param)
{
    osSignalSet(head_track_thread_id, HEAD_TRACK_SIGNAL_ALGO);
}

void head_track_bias_flash_read_set()
{
    TRACE(0, "entered %s", __func__);

    float param_bias[3] = {0.0};
    calibration_values_read(CALIBRATION_SECTION_DEVICE_HEAD_TRACK, (uint8_t *)param_bias, sizeof(param_bias));

    if (*((unsigned int*)&param_bias[0]) == 0xFFFFFFFF &&
        *((unsigned int*)&param_bias[1]) == 0xFFFFFFFF &&
        *((unsigned int*)&param_bias[2]) == 0xFFFFFFFF) {

        calibration_on = 1;
        TRACE(0, "%s, need calibration, calibrantion on", __func__);
    }else{
        head_track_set_gyro_bias(param_bias);
    }
}

void head_angle_reset(void)
{
    TRACE(0, ">>>> %s: enter...", __func__);
    head_track_3dof_reset();
}


void start_head_track_timer(void)
{
    osStatus_t status = osTimerStart(imu_timer_id, HEAD_TRACK_TIMER_INTERVAL);
    if (status != osOK) {
        TRACE(0, "start head_track algo timer failed!");
    }
}

void pause_head_track_timer(void)
{
    osStatus_t status = osTimerStop(imu_timer_id);
    if (status != osOK) {
        TRACE(0, "pause head_track algo timer failed!");
    }
}

void imusensor_init(void)
{
    /* Init head track task  */
    head_track_resouce_init();

    /* Init head track task  */
    head_track_task_init();
    head_track_algo_task_init();
    head_track_send_task_init();

    head_track_bias_flash_read_set();

    /* Create imu timer*/
    imu_timer_id = osTimerCreate(osTimer(imu_timer), osTimerPeriodic, NULL);

    /* start head track timer to release signals */
    if (imu_timer_id != NULL) {
        start_head_track_timer();
    }

    return;
}

static void head_track_send_task_deinit(void)
{
    TRACE(0, "%s(true),%d \n\r", __FUNCTION__, __LINE__);
    osThreadTerminate(head_track_send_thread_id);
    head_track_send_inited = 0;
}

static void head_track_algo_task_deinit(void)
{
    TRACE(0, "%s(true),%d \n\r", __FUNCTION__, __LINE__);
    osThreadTerminate(head_track_algo_thread_id);
    head_track_algo_inited = 0;
}

static void head_track_task_deinit(void)
{
    TRACE(0, "%s(true),%d \n\r", __FUNCTION__, __LINE__);
    osThreadTerminate(head_track_thread_id);
    head_track_inited = 0;
}

void head_track_resouce_deinit(void)
{
    head_track_sensor_mutex_id = NULL;
    head_track_pose_mutex_id = NULL;
    sensor_signal = NULL;
}

void imusensor_deinit(void)
{
    TRACE(0, "%s(true),%d \n\r", __FUNCTION__, __LINE__);
    if(imu_timer_id != NULL) {
        osTimerDelete(imu_timer_id);
    }

    head_track_send_task_deinit();
    head_track_algo_task_deinit();
    head_track_task_deinit();

    head_track_resouce_deinit();
    return;
}

void imusensor_reset(void)
{
    head_track_3dof_reset();
}

void head_track_algo_resume(void)
{
    pause_head_track_timer();
    imusensor_reset();
    start_head_track_timer();
    is_ht_data_sending_enabled = true;
}

void head_track_algo_pause(void)
{
    imusensor_reset();
    pause_head_track_timer();
    is_ht_data_sending_enabled = false;
}


void head_track_imu_calibration(void)
{

    uint16_t cnt = 0;
    float gyrBias[3] = {0.0, 0.0, 0.0};

    imusensor_deinit();

    while (1) {
        if(cnt >= 500) {
            gyrBias[0] = gyrBias[0] / (cnt * 1000);
            gyrBias[1] = gyrBias[1] / (cnt * 1000);
            gyrBias[2] = gyrBias[2] / (cnt * 1000);

            calibration_values_write(CALIBRATION_SECTION_DEVICE_HEAD_TRACK, (uint8_t *)gyrBias, sizeof(gyrBias));
            break;
        }
        gyrBias[0] += preImu.g_x;
        gyrBias[1] += preImu.g_y;
        gyrBias[2] += preImu.g_z;
        cnt++;
        osDelay(10);
    }

    imusensor_init();
}

#endif