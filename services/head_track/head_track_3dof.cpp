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
 ****************************************************************************/
#include <string.h>
#include "hal_timer.h"
#include "hal_trace.h"
#include "head_track_3dof.h"
#include "orientation_math.h"

static float alphaImuFilter = 0.8;
static float alphaVelFilt = 0.8;
static float systemLatency = 0.1;

static int nReads;
static int nSilentCounts;

static uint32_t currTimeImu = 0;
static uint32_t prevTimeImu = 0;
static uint16_t sensor_read_cnt = 0;
static float deltaT;
static float sumAcc[3] = {0,0,0};
static float sum_gyro[3] = { 0.0, 0.0, 0.0 };
static float gyro_bias[3] = {0.380, 0.338, 0.401};
static float acc[3], gyro[3];
static float phi;
static float instAngVelGyr;

static struct QUATERNIONC qTracker;
static struct QUATERNIONC qInit;
static struct QUATERNIONC qAdj;

// static struct QUATERNIONC quaternionPredict;
static struct ADJ_PARAM fix_adj_param;

void gyroBiasCollect(struct SENSOR_IMU *imu)
{
    sum_gyro[0] += imu->g_x;
    sum_gyro[1] += imu->g_y;
    sum_gyro[2] += imu->g_z;
    sensor_read_cnt ++;
}
void gyroBiasUpdate(struct GYR_BIAS *b)
{
    gyro_bias[0] = sum_gyro[0] / (float)(sensor_read_cnt * (float)1000);
    gyro_bias[1] = sum_gyro[1] / (float)(sensor_read_cnt * (float)1000);
    gyro_bias[2] = sum_gyro[2] / (float)(sensor_read_cnt * (float)1000);

    memset(sum_gyro, 0, sizeof(sum_gyro));
    sensor_read_cnt = 0;

    b->v1 = gyro_bias[0];
    b->v2 = gyro_bias[1];
    b->v3 = gyro_bias[2];
    b->v4 = 1;

    TRACE(0, "gyro bias updated,  %08d\t%08d\t%08d", (int)(b->v1*1000),
                                                    (int)(b->v2*1000),
                                                    (int)(b->v3*1000));
}

void gyroBiasClear(struct GYR_BIAS *bias)
{
    sensor_read_cnt = 0;
    memset(sum_gyro, 0, sizeof(sum_gyro));
    memset(bias, 0, sizeof(struct GYR_BIAS));
}

static void quaternion2EularAnglesXYZ(struct POSE_S *pose, struct QUATERNIONC q)
{
    float test = q.q[2] * q.q[0] + q.q[1] * q.q[3];

    if(test > 0.4999f) {
        pose->roll = (float)2.0 * atan2(q.q[2], q.q[0]);
        pose->pitch = PI/2;
        pose->yaw = 0.0;
    }
    if(test < -0.4999f) {
        pose->roll = (float)2.0 * atan2(q.q[2], q.q[0]);
        pose->pitch = -PI/2;
        pose->yaw = 0.0;
    }

    float sqx = q.q[1] * q.q[1];
    float sqy = q.q[2] * q.q[2];
    float sqz = q.q[3] * q.q[3];

    pose->yaw = atan2((float)2.0 * q.q[1] * q.q[0] - (float)2.0 * q.q[3] * q.q[2],
                        (float)1.0 - (float)2.0 * sqy - (float)2.0 * sqx);
    pose->roll = -asin((float)2.0 * test);
    pose->pitch = -atan2((float)2.0 * q.q[3] * q.q[0] - (float)2.0 * q.q[2] * q.q[1],
                        (float)1.0 - (float)2.0 * sqy - (float)2.0 * sqz);
}

void head_track_3dof_algo(struct POSE_S *pose, struct SENSOR_IMU *sensorImu, struct ADJ_PARAM *adjParam)
{
    /* convert from mg to unit of g */
    acc[0] = sensorImu->a_x / (float)1000.0;
    acc[1] = sensorImu->a_y / (float)1000.0;
    acc[2] = sensorImu->a_z / (float)1000.0;

    /* convert from mdeg/sec to unit of deg/sec */
    gyro[0] = (sensorImu->g_x / (float)1000.0) - gyro_bias[0];
    gyro[1] = (sensorImu->g_y / (float)1000.0) - gyro_bias[1];
    gyro[2] = (sensorImu->g_z / (float)1000.0) - gyro_bias[2];

    // currTimeImu = TICKS_TO_MS(hal_sys_timer_get());
    currTimeImu = sensorImu->frameTimeStamp;
    deltaT = (currTimeImu > prevTimeImu) ? (currTimeImu - prevTimeImu)/(float)1000000 : deltaT;
    prevTimeImu = currTimeImu;

    if ((nReads >= 0) && (nReads < adjParam->v5)) {
        sumAcc[0] = sumAcc[0] + acc[0];
        sumAcc[1] = sumAcc[1] + acc[1];
        sumAcc[2] = sumAcc[2] + acc[2];
        if (nReads == (adjParam->v5 - 1)){
            sumAcc[0] = sumAcc[0] / adjParam->v5;
            sumAcc[1] = sumAcc[1] / adjParam->v5;
            sumAcc[2] = sumAcc[2] / adjParam->v5;

            struct QUATERNIONC qAcc;
            quaternionCInitWithVal(&qAcc, 0, sumAcc[0], sumAcc[1], sumAcc[2]);
            normalizeC(&qAcc);

            phi = acos(-qAcc.q[1]) * 180 / (float)PI;
            float n = sqrt(pow(-qAcc.q[3],2.0) + pow(qAcc.q[2],2.0));
            float ny = -qAcc.q[3] / n;
            float nz = qAcc.q[2] / n;

            /* initial orientation */
            setFromAngleAxisC(&qTracker, phi, 0.0, ny, nz);
            inverseC(&qInit, &qTracker);
            nSilentCounts = FAST_TICKS_TO_MS(hal_fast_sys_timer_get());
        }
    } else {
        /* motion model:tracking */
        updatequaternionCompC(&qTracker, &instAngVelGyr, gyro, acc,
                                        deltaT, adjParam->v1, &phi);
        multiplyC(&qAdj, &qInit, &qTracker);
        if (instAngVelGyr > adjParam->v6) {
            nSilentCounts = FAST_TICKS_TO_MS(hal_fast_sys_timer_get());
        } else {
            int current_time_tmp = FAST_TICKS_TO_MS(hal_fast_sys_timer_get());
            if(current_time_tmp - nSilentCounts > adjParam->v7) {
                nSilentCounts = current_time_tmp;
                head_track_3dof_reset();
            }
        }
    }

    if(adjParam->v4 == 1){
        /*
         * Base Coordinate System
         * Imu x-downwards y-backwards z-leftwards
         */
        quaternion2EularAnglesXYZ(pose, qAdj);
    }else{
        /*
         * custorm Crdinate System
         * needs adjustment case by case
         */
        quaternion2EularAnglesXYZ(pose, qAdj);

    }

    pose->vx = sensorImu->g_x;
    pose->vy = sensorImu->g_y;
    pose->vz = sensorImu->g_z;
    nReads++;
}

void head_track_3dof_algo_with_fix_param(struct POSE_S *pose, struct SENSOR_IMU *sensorImu, struct GYR_BIAS *bias)
{
    fix_adj_param.v1 = alphaImuFilter;
    fix_adj_param.v2 = alphaVelFilt;
    fix_adj_param.v3 = systemLatency;
    /*
     * v4: coordinate flag
     * 1:  base coordinate, means IMU x-up2down y-front2back z-right2left
     * 0:  other coordinate, shoule rotate the base coordinate according to customer's coordinate
     */
    fix_adj_param.v4 = 1;
    fix_adj_param.v5 = 10; /* init watch window width */
    fix_adj_param.v6 = 5.0;
    fix_adj_param.v7 = 5 * 1000; /* ms, v7 seconds head stay still, pose reset */

    /* convert from mg to unit of g, if necessary*/
    /* make sure acc is g here */
    acc[0] = sensorImu->a_x / (float)1000.0;
    acc[1] = sensorImu->a_y / (float)1000.0;
    acc[2] = sensorImu->a_z / (float)1000.0;

    /* convert from mdeg/sec to unit of deg/sec, if necessary*/
    /* make sure gyro is deg/sec here */
    gyro[0] = (sensorImu->g_x / (float)1000.0) - gyro_bias[0];
    gyro[1] = (sensorImu->g_y / (float)1000.0) - gyro_bias[1];
    gyro[2] = (sensorImu->g_z / (float)1000.0) - gyro_bias[2];

    /* currTimeImu is in us */
    currTimeImu = sensorImu->frameTimeStamp;
    /* make sure deltaT here is in sec */
    deltaT = (currTimeImu > prevTimeImu) ? (currTimeImu - prevTimeImu)/(float)1000000 : deltaT;
    prevTimeImu = currTimeImu;

    if ((nReads >= 0) && (nReads < fix_adj_param.v5)) {
        sumAcc[0] = sumAcc[0] + acc[0];
        sumAcc[1] = sumAcc[1] + acc[1];
        sumAcc[2] = sumAcc[2] + acc[2];
        if (nReads == (fix_adj_param.v5 - 1)) {
            sumAcc[0] = sumAcc[0] / fix_adj_param.v5;
            sumAcc[1] = sumAcc[1] / fix_adj_param.v5;
            sumAcc[2] = sumAcc[2] / fix_adj_param.v5;

            struct QUATERNIONC qAcc;
            quaternionCInitWithVal(&qAcc, 0, sumAcc[0], sumAcc[1], sumAcc[2]);
            normalizeC(&qAcc);

            phi = acos(-qAcc.q[1]) * 180 / (float)PI;
            float n = sqrt(pow(-qAcc.q[3],2.0) + pow(qAcc.q[2],2.0));
            float ny = -qAcc.q[3] / n;
            float nz = qAcc.q[2] / n;

            /*initial orientation*/
            setFromAngleAxisC(&qTracker, phi, 0.0, ny, nz);
            inverseC(&qInit, &qTracker);
            nSilentCounts = FAST_TICKS_TO_MS(hal_fast_sys_timer_get());
        }
    } else {
        /* tracking */
        updatequaternionCompC(&qTracker, &instAngVelGyr, gyro, acc,
                                        deltaT, fix_adj_param.v1, &phi);
        multiplyC(&qAdj, &qInit, &qTracker);
        if (instAngVelGyr > fix_adj_param.v6) {
            nSilentCounts = FAST_TICKS_TO_MS(hal_fast_sys_timer_get());
            gyroBiasClear(bias);
        } else {
            int current_time_tmp = FAST_TICKS_TO_MS(hal_fast_sys_timer_get());
            gyroBiasCollect(sensorImu);
            if(current_time_tmp - nSilentCounts > fix_adj_param.v7) {
                nSilentCounts = current_time_tmp;
                gyroBiasUpdate(bias);
                head_track_3dof_reset();
            }
        }
    }

    if(fix_adj_param.v4 == 1){
        /*
         * Base Coordinate System
         * Imu x-up2down y-front2back z-right2left
         */
        quaternion2EularAnglesXYZ(pose, qAdj);
    }else{
        /*
         * Custormer Coordinate System
         * need adjustment case by case
         */
        quaternion2EularAnglesXYZ(pose, qAdj);
    }

    pose->vx = sensorImu->g_x;
    pose->vy = sensorImu->g_y;
    pose->vz = sensorImu->g_z;
    nReads++;

}

void head_track_3dof_reset(void)
{
    TRACE(0, "############# %s", __func__);
    nReads = 0;
    memset(sumAcc, 0, sizeof(sumAcc));
    memset(acc, 0, sizeof(acc));
    memset(gyro, 0, sizeof(gyro));
}

void head_track_set_gyro_bias(float values[3])
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        gyro_bias[i] = values[i];
    }
}