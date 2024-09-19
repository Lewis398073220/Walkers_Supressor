#ifndef BASS_ENHANCER_H
#define BASS_ENHANCER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int low_cut_freq;
    int high_cut_freq;
    float gain0;
    float gain1;
} BassEnhancerConfig;

struct BassEnhancerState_;

typedef struct BassEnhancerState_ BassEnhancerState;

#ifdef __cplusplus
extern "C" {
#endif

BassEnhancerState *bass_enhancer_init(int sample_rate, int frame_size, int sample_bit, int ch_num, const BassEnhancerConfig *config);

void bass_enhancer_destroy(BassEnhancerState *st);

void bass_enhancer_process(BassEnhancerState *st, uint8_t *buf, int len);

void virtrul_bass_switch(BassEnhancerState *st, bool val);

void bass_enhancer_set_vol_level(BassEnhancerState *st, int vol_level);

void virtrul_bass_cfg(BassEnhancerState *st, float * data);
#ifdef __cplusplus
}
#endif

#endif