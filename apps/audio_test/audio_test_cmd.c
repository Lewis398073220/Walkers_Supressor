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
#include "stdio.h"
#include "hal_trace.h"
#include "hal_aud.h"
#include "plat_types.h"
#include "string.h"
#include "audio_test_defs.h"
#include "audio_test_cmd.h"
#include "app_trace_rx.h"
#include "app_key.h"
// #include "bluetooth_bt_api.h"
#include "app_media_player.h"
#if defined(ANC_APP)
#include "app_anc.h"
#include "app_anc_utils.h"
#endif

#define AUDIO_TEST_LOG_I(str, ...)      TR_INFO(TR_MOD(TEST), "[AUDIO_TEST]" str, ##__VA_ARGS__)

/**
 * Usage:
 *  1. CMD Format: e.g. [audio_test,anc_switch]
 **/

typedef void (*func_handler_t)(const char *cmd);

typedef struct {
    const char *name;
    func_handler_t handler;
} audio_test_func_t;

WEAK void app_debug_tool_printf(const char *fmt, ...) { };

static void audio_test_anc_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] cmd len: %d", __func__, strlen(cmd) - 2);

#if defined(ANC_APP)
    if (strlen(cmd) > 2) {
        int mode = 0;
        sscanf(cmd, "%d", &mode);

        AUDIO_TEST_LOG_I("[%s] mode: %d", __func__, mode);
        ASSERT(mode < APP_ANC_MODE_QTY, "[%s] mode is invalid: %d", __func__, mode);
        app_anc_switch(mode);
    } else {
        app_anc_loop_switch();
    }
#endif
}

#if defined(__VIRTUAL_SURROUND__) || defined(__VIRTUAL_SURROUND_CP__) || defined(__VIRTUAL_SURROUND_STEREO__)
extern int32_t audio_process_stereo_surround_onoff(int32_t onoff);
extern int32_t audio_process_stereo_set_yaw(float yaw);
extern int32_t audio_process_stereo_set_pitch(float pitch);
static int32_t stereo_surround_status = 0;
static float pitch_angle = 0;
static float yaw_angle = 0;
#endif

static void audio_test_virtual_surround_on(const char *cmd){
#if defined(__VIRTUAL_SURROUND__) || defined(__VIRTUAL_SURROUND_CP__) || defined(__VIRTUAL_SURROUND_STEREO__)
    stereo_surround_status = 1;
    audio_process_stereo_surround_onoff(stereo_surround_status);
#endif
}

static void audio_test_virtual_surround_off(const char *cmd){
#if defined(__VIRTUAL_SURROUND__) || defined(__VIRTUAL_SURROUND_CP__) || defined(__VIRTUAL_SURROUND_STEREO__)
    stereo_surround_status = 0;
    audio_process_stereo_surround_onoff(stereo_surround_status);
#endif
}

static void audio_test_virtual_surround_test_yaw(const char *cmd){
#if defined(__VIRTUAL_SURROUND__) || defined(__VIRTUAL_SURROUND_CP__) || defined(__VIRTUAL_SURROUND_STEREO__)
    yaw_angle += 10.0;
    if(yaw_angle > 180){
        yaw_angle = -180;
    }
    audio_process_stereo_set_yaw(yaw_angle);
#endif
}

static void audio_test_virtual_surround_test_pitch(const char *cmd){
#if defined(__VIRTUAL_SURROUND__) || defined(__VIRTUAL_SURROUND_CP__) || defined(__VIRTUAL_SURROUND_STEREO__)
    pitch_angle += 10.0;
    if(pitch_angle > 80){
        pitch_angle = -80;
    }
    audio_process_stereo_set_pitch(pitch_angle);
#endif
}

static void audio_test_anc_gain_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);

#if defined(ANC_APP)
    static bool flag = false;
    if (flag) {
        app_anc_set_gain_f32(ANC_GAIN_USER_TUNING, PSAP_FEEDFORWARD, 1.0, 1.0);
    } else {
        app_anc_set_gain_f32(ANC_GAIN_USER_TUNING, PSAP_FEEDFORWARD, 0.0, 0.0);
    }

    flag = !flag;
#endif
}

extern int32_t audio_test_stream(void);
static void audio_test_stream_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);

#if defined(AUDIO_TEST_STREAM)
    audio_test_stream();
#endif
}

static void audio_test_karaoke_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);

#ifdef A2DP_KARAOKE
    static bool stream_status = true;
    extern int32_t app_karaoke_start(bool on);
    app_karaoke_start(stream_status);
    stream_status = !stream_status;
#endif
}

#if defined(ANC_ASSIST_ENABLED)
#include "app_anc_assist.h"
#endif
static void audio_test_anc_assist_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);
#if defined(ANC_ASSIST_ENABLED)
    static bool flag = true;
    if (flag) {
        app_anc_assist_open(ANC_ASSIST_USER_ANC);
    } else {
        app_anc_assist_close(ANC_ASSIST_USER_ANC);
    }

    flag = !flag;
#endif
}

static void audio_test_assist_noise_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);
#if defined(ANC_ASSIST_ENABLED) && defined(VOICE_ASSIST_NOISE)
    static bool flag = true;
    if (flag) {
        app_anc_assist_open(ANC_ASSIST_USER_NOISE_ADAPT_ANC);
    } else {
        app_anc_assist_close(ANC_ASSIST_USER_NOISE_ADAPT_ANC);
    }

    flag = !flag;
#endif
}

static void audio_test_assist_noise_classify_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);
#if defined(ANC_ASSIST_ENABLED) && defined(VOICE_ASSIST_NOISE_CLASSIFY)
    static bool flag = true;
    if (flag) {
        app_anc_assist_open(ANC_ASSIST_USER_NOISE_CLASSIFY_ADAPT_ANC);
    } else {
        app_anc_assist_close(ANC_ASSIST_USER_NOISE_CLASSIFY_ADAPT_ANC);
    }

    flag = !flag;
#endif
}

#if defined(ANC_ASSIST_ENABLED)
#include "app_voice_assist_ai_voice.h"
#include "audioflinger.h"
#endif

static void audio_test_kws_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);
#if defined(ANC_ASSIST_ENABLED)

    struct AF_STREAM_CONFIG_T kws_stream_cfg;
    kws_stream_cfg.sample_rate    = AUD_SAMPRATE_16000;
    kws_stream_cfg.channel_num    = AUD_CHANNEL_NUM_1;
    kws_stream_cfg.bits           = AUD_BITS_16;
    kws_stream_cfg.device         = AUD_STREAM_USE_INT_CODEC;
    kws_stream_cfg.io_path        = AUD_INPUT_PATH_ASRMIC;
    kws_stream_cfg.vol            = 12;
    kws_stream_cfg.handler        = NULL;
    kws_stream_cfg.data_ptr       = NULL;
    kws_stream_cfg.data_size      = 0;
    TRACE(0,"111");
    static bool flag = true;
    if (flag) {
        // FIXME: Create a stream_cfg
        app_voice_assist_ai_voice_open(ASSIST_AI_VOICE_MODE_KWS, &kws_stream_cfg, NULL);
    } else {
        app_voice_assist_ai_voice_close();
    }

    flag = !flag;
#endif
}

#if defined(ANC_ASSIST_ENABLED)
int32_t app_voice_assist_wd_open(void);
int32_t app_voice_assist_wd_close(void);
#endif
static void audio_test_wd_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);
#if defined(ANC_ASSIST_ENABLED)
    static bool flag = true;
    if (flag) {
        app_voice_assist_wd_open();
    } else {
        app_voice_assist_wd_close();
    }

    flag = !flag;
#endif
}

#if defined(SPEECH_BONE_SENSOR)
#include "speech_bone_sensor.h"
#endif
static void audio_test_bone_sensor_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);
#if defined(SPEECH_BONE_SENSOR)
    static bool flag = true;
    if (flag) {
        speech_bone_sensor_open(16000, 120);
        speech_bone_sensor_start();
    } else {
        speech_bone_sensor_stop();
        speech_bone_sensor_close();
    }

    flag = !flag;
#endif
}

static void audio_test_prompt_switch(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);
#if defined(VOICE_ASSIST_CUSTOM_LEAK_DETECT) && defined(ANC_ASSIST_ENABLED)
    int prompt_buf[6] = {AUD_ID_ANC_PROMPT, AUD_ID_CUSTOM_LEAK_DETECT, 16, 17, 18, 19};
    static int prompt_cnt = 0;

    AUD_ID_ENUM id_tmp = (AUD_ID_ENUM)prompt_buf[prompt_cnt];
    TRACE(1,"[%s],id = 0x%x",__func__,(int)id_tmp);
    media_PlayAudio(id_tmp, 0);

    prompt_cnt++;

    if(6<= prompt_cnt)
        prompt_cnt = 0;

#endif
}

static void audio_test_input_param(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] %s", __func__, cmd);

    int val1 = 0;
    int val2 = 0;
    sscanf(cmd, "val1=%d val2=%d", &val1, &val2);

    AUDIO_TEST_LOG_I("[%s] val1: %d, val2: %d", __func__, val1, val2);
}

static void audio_test_call_algo_cfg(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] %s", __func__, cmd);

}

static void audio_test_hifi(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] %s", __func__, cmd);

#if defined(AUDIO_TEST_HIFI)
    extern int32_t audio_test_hifi_tx_data(void);
    audio_test_hifi_tx_data();
#endif
}

static void audio_test_switch_speech_algo_dsp(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] %s", __func__, cmd);

#if defined(AUDIO_TEST_SPEECH_ALGO_DSP)
    extern int32_t audio_test_speech_algo_dsp(void);
    audio_test_speech_algo_dsp();
#endif
}

#ifdef __AUDIO_DYNAMIC_BOOST__
extern int switch_cfg(void);
#endif

static void audio_test_dynamic_boost(const char *cmd)
{
    AUDIO_TEST_LOG_I("[%s] ...", __func__);

#ifdef __AUDIO_DYNAMIC_BOOST__
    switch_cfg();
#endif
}

extern void audio_virtual_bass_switch(int val);
static void audio_test_virtual_bass_switch(const char *cmd)
{
#ifdef AUDIO_BASS_ENHANCER
    if (!strcmp(cmd, "on")) {
        audio_virtual_bass_switch(1);
        AUDIO_TEST_LOG_I("[%s]:open", __FUNCTION__);
    } else if (!strcmp(cmd, "off")) {
        audio_virtual_bass_switch(0);
        AUDIO_TEST_LOG_I("[%s]:close", __FUNCTION__);
    } else {
        AUDIO_TEST_LOG_I("invalid cmd: %s", cmd);
        AUDIO_TEST_LOG_I("cmd usage: audio_test:virtual_bass on");
        AUDIO_TEST_LOG_I("cmd usage: audio_test:virtual_bass off");
    }
#endif
}

extern void audio_virtual_bass_ctrl(float *data);
static void audio_test_virtual_bass_ctrl(const char *cmd)
{
    AUDIO_TEST_LOG_I(" %s...", __func__);
#ifdef AUDIO_BASS_ENHANCER
    int val[5];
    float data[5];
    sscanf(cmd, "%d, %d, %d", &val[0], &val[1], &val[2]);
    data[0] = (float)val[0];
    data[1] = (float)val[1];
    data[2] = (float)val[2];

    audio_virtual_bass_ctrl(data);
#endif
}

const audio_test_func_t audio_test_func[]= {
    {"anc_switch",          audio_test_anc_switch},
    {"stream_switch",       audio_test_stream_switch},
    {"karaoke_switch",      audio_test_karaoke_switch},
    {"anc_assist_switch",   audio_test_anc_assist_switch},
    {"kws_switch",          audio_test_kws_switch},
    {"wd_switch",           audio_test_wd_switch},
    {"anc_gain_switch",     audio_test_anc_gain_switch},
    {"bone_sensor_switch",  audio_test_bone_sensor_switch},
    {"prompt_switch",       audio_test_prompt_switch},
    {"input_param",         audio_test_input_param},
    {"call_algo_cfg",       audio_test_call_algo_cfg},
    {"hifi",                audio_test_hifi},
    {"speech_algo_dsp",     audio_test_switch_speech_algo_dsp},
	{"virtual_surround_on",     audio_test_virtual_surround_on},
    {"virtual_surround_off",    audio_test_virtual_surround_off},
    {"virtual_surround_test_yaw",      audio_test_virtual_surround_test_yaw},
    {"virtual_surround_test_pitch",    audio_test_virtual_surround_test_pitch},
    {"dynamic_bass_boost",  audio_test_dynamic_boost},
    {"virtual_bass",        audio_test_virtual_bass_switch},
    {"virtual_bass_ctrl",   audio_test_virtual_bass_ctrl},
    {"voice_assist_noise_switch",   audio_test_assist_noise_switch},
    {"voice_assist_noise_classify_switch",   audio_test_assist_noise_classify_switch},
};

static unsigned int audio_test_cmd_callback(unsigned char *buf, unsigned int len)
{
    uint32_t index = 0;
    char *cmd = (char *)buf;
    func_handler_t func_handler = NULL;

    // filter ' ' before function
    for (uint32_t i=0; i<strlen(cmd); i++) {
        if (cmd[i] != ' ') {
            cmd += i;
            break;
        }
    }

    // Separate function and value with ' '
    for (index=0; index<strlen(cmd); index++) {
        if (cmd[index] == ' ' || cmd[index] == '\r' || cmd[index] == '\n') {
            break;
        }
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(audio_test_func); i++) {
        if (strncmp((char *)cmd, audio_test_func[i].name, index) == 0) {
            func_handler = audio_test_func[i].handler;
            break;
        }
    }

    // filter ' ' before value
    for (; index<strlen(cmd); index++) {
        if (cmd[index] != ' ') {
            break;
        }
    }

    if (strncmp(cmd + index, "bin ", strlen("bin ")) == 0) {
        index += strlen("bin ");
    }

    if (func_handler) {
        func_handler(cmd + index);
        AUDIO_TEST_LOG_I("[audio_test] cmd: OK!");
        TRACE(0, "[CMD] res : 0;");
        app_debug_tool_printf("cmd, Process %s", cmd);
    } else {
        AUDIO_TEST_LOG_I("[audio_test] cmd: Can not found cmd: %s", cmd);
        TRACE(0, "[CMD] res : 1; info : Can not found cmd(%s);", cmd);
        app_debug_tool_printf("cmd, Invalid %s", cmd);
    }

    return 0;
}

int32_t audio_test_cmd_init(void)
{
    app_trace_rx_register("audio_test", audio_test_cmd_callback);

#if defined(AUDIO_TEST_HIFI)
    extern int32_t audio_test_hifi_init(void);
    audio_test_hifi_init();
#endif

    return 0;
}
