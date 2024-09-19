#ifndef SPEECH_NS7_H
#define SPEECH_NS7_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t     bypass;
    float       denoise_dB;
    float       dnn_denoise_dB;
    bool        echo_supp_enable;
    int32_t     ref_delay;
    float       gamma;
    int32_t     echo_band_start;
    int32_t     echo_band_end;
    float       min_ovrd;
    float       target_supp;
} SpeechNs7Config;

struct SpeechNs7State_;

typedef struct SpeechNs7State_ SpeechNs7State;

#ifdef __cplusplus
extern "C" {
#endif


int32_t speech_ns7_process(SpeechNs7State *st, int16_t *pcm_buf, int16_t *ref_buf, uint32_t pcm_len);
SpeechNs7State* speech_ns7_create(uint32_t sample_rate, uint32_t frame_size,  const SpeechNs7Config *cfg);

int32_t speech_ns7_in_2mic_process(SpeechNs7State *st, float *mag, uint32_t pcm_len, int32_t pf_enable, float win_rms);
SpeechNs7State* speech_ns7_in_2mic_create(uint32_t sample_rate, uint32_t frame_size, const SpeechNs7Config *cfg, uint8_t *work_buf, uint32_t work_buf_len);


void speech_ns7_destory(SpeechNs7State *st);
void speech_ns7_in_2mic_destory(SpeechNs7State *st);

uint32_t speech_ns7_in_2mic_needed_size(void);

#ifdef __cplusplus
}
#endif

#endif
