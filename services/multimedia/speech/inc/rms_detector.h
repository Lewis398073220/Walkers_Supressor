#ifndef RMS_DETECTOR_H
#define RMS_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "custom_allocator.h"

typedef struct
{
    float threshold;
    float tav;
    float hold_time;
} RmsDetectorConfig;

typedef struct
{
    void (*action)(bool onoff);
} RmsDetectorCallback;

struct RmsDetectorState_;

typedef struct RmsDetectorState_ RmsDetectorState;

#ifdef __cplusplus
extern "C" {
#endif

RmsDetectorState *rms_detector_create(int32_t sample_rate, int32_t sample_bit, int32_t ch_num, RmsDetectorCallback *callback, const RmsDetectorConfig *config, custom_allocator *allocator);

void rms_detector_destroy(RmsDetectorState *st);

float rms_detector_process(RmsDetectorState *st, uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif
