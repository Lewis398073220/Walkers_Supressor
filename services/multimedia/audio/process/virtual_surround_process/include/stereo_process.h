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
#ifndef SPEECH_STEREO_SURROUND_H
#define SPEECH_STEREO_SURROUND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    VIRTUAL_SURROUND_FIR_MIDDLE_LEFT = 0,
    VIRTUAL_SURROUND_FIR_MIDDLE_RIGHT,
    VIRTUAL_SURROUND_FIR_DIR_LEFT_RIGHT,
    VIRTUAL_SURROUND_FIR_DIR_RIGHT_RIGHT,
    VIRTUAL_SURROUND_FIR_DIR_LEFT_LEFT,
    VIRTUAL_SURROUND_FIR_DIR_RIGHT_LEFT,
    VIRTUAL_SURROUND_FIR_SUR_LEFT_RIGHT,
    VIRTUAL_SURROUND_FIR_SUR_RIGHT_RIGHT,
    VIRTUAL_SURROUND_FIR_SUR_LEFT_LEFT,
    VIRTUAL_SURROUND_FIR_SUR_RIGHT_LEFT,
    VIRTUAL_SURROUND_FIR_QTY
} stereo_surround_fir_index_t;

struct StereoSurroundState_;

typedef struct StereoSurroundState_ StereoSurroundState;

struct StereoSurroundStateCp_;

typedef struct StereoSurroundStateCp_ StereoSurroundStateCp;

int32_t stereo_surround_get_onoff_status(StereoSurroundState *st);

StereoSurroundState *stereo_surround_init(int32_t sample_rate, int32_t frame_size, int32_t sample_bits,int32_t nv_role, int32_t is_ap_cp,int32_t sw_ch_num);

int32_t stereo_surround_destroy(StereoSurroundState *st);

int32_t stereo_surround_preprocess(StereoSurroundState *st,uint8_t *buf, int32_t len);

int32_t stereo_surround_process(StereoSurroundState *st,uint8_t *buf, int32_t len);

int32_t stereo_surround_onoff(StereoSurroundState *st, int32_t onoff);

int32_t stereo_surround_fir_onoff(StereoSurroundState *st, int32_t onoff);

int32_t stereo_surround_preprocess_cp(StereoSurroundState *st,uint8_t *buf, int32_t len);

int32_t stereo_surround_preprocess_ap(StereoSurroundState *st,uint8_t *buf, int32_t len);

int32_t stereo_surround_set_yaw_angle(StereoSurroundState *st, float yaw);

int32_t stereo_surround_set_pitch_angle(StereoSurroundState *st, float pitch);

#ifdef __cplusplus
}
#endif

#endif