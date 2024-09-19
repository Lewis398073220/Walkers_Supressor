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
#ifndef __HEAD_TRACK_3DOF_H__
#define __HEAD_TRACK_3DOF_H__

struct SENSOR_IMU {
    float a_x;
    float a_y;
    float a_z;
    float g_x;
    float g_y;
    float g_z;
    float temp;
    unsigned int frameTimeStamp;
    int rflag;
};

struct ADJ_PARAM {
    float v1;   //alphaImuFilter  0.8
    float v2;   //alphaVelFilt 0.8 needs tune
    float v3;   //systemLatency 0.1s needs tune
    float v4;   //base coordinate system 1 (IMU x-downward y-backward z-leftward); customer coordinate system 0
    float v5;   //Observation window size, default 10
    float v6;   //human slience threshold
    float v7;   //slience duration
    float v8;   //gyrBiasSet x
    float v9;   //gyrBiasSet y
    float v10;  //gyrBiasSet z
};

struct POSE_S {
    float pitch;
    float yaw;
    float roll;
    float vx;
    float vy;
    float vz;
};

struct GYR_BIAS {
    float v1;   //gx_bias, dps
    float v2;   //gy_bias, dps
    float v3;   //gz_bias, dps
    uint8_t v4; //new calibration value available flag, 1: yes 0:no
};


void head_track_3dof_algo(struct POSE_S *pose, struct SENSOR_IMU *sensorImu, struct ADJ_PARAM *adjParam);

void head_track_3dof_algo_with_fix_param(struct POSE_S *pose, struct SENSOR_IMU *sensorImu, struct GYR_BIAS *g_bias);

void head_track_3dof_reset(void);

void head_track_set_gyro_bias(float values[3]);

#endif