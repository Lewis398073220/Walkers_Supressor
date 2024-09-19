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

#ifndef SPEECH_NS9_H
#define SPEECH_NS9_H

#include <stdint.h>
#include <stdbool.h>
#include "custom_allocator.h"

typedef struct {
    int32_t     bypass;
    float       denoise_dB;
    bool        echo_supp_enable;
    int32_t     ref_delay;
    float       gamma;
    int32_t     echo_band_start;
    int32_t     echo_band_end;
    float       min_ovrd;
    float       target_supp;
    float       ga_thr;
    float       en_thr;
} SpeechNs9Config;

struct SpeechNs9State_;

typedef struct SpeechNs9State_ SpeechNs9State;

#ifdef __cplusplus
extern "C" {
#endif

int32_t speech_ns9_process(SpeechNs9State *st, int16_t *pcm_buf, int16_t *ref_buf, uint32_t pcm_len);
void speech_ns9_destory(SpeechNs9State *st);
SpeechNs9State* speech_ns9_create(uint32_t sample_rate, uint32_t frame_size,  const SpeechNs9Config *cfg, custom_allocator *allocator);
int32_t speech_ns9_set_config(SpeechNs9State *st, const SpeechNs9Config *cfg);

uint32_t speech_ns9_in_2mic_needed_size(void);
SpeechNs9State* speech_ns9_in_2mic_create(uint32_t sample_rate, uint32_t frame_size, const SpeechNs9Config *cfg, uint8_t *work_buf, uint32_t work_buf_len);
int32_t speech_ns9_in_2mic_process(SpeechNs9State *st, float *mag, uint32_t pcm_len, int32_t pf_enable, float win_rms);
void speech_ns9_in_2mic_destory(SpeechNs9State *st);

#ifdef __cplusplus
}
#endif

#endif
