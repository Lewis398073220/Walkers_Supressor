#ifndef __CALIBRATION_SECTION_H__
#define __CALIBRATION_SECTION_H__

#include "plat_types.h"
#include "section_def.h"

enum CALIBRATION_SECTION_DEVICE {
    CALIBRATION_SECTION_DEVICE_HEAD_TRACK,
    CALIBRATION_SECTION_DEVICE_NUM,
};

#define HEAD_TRACK_CALIBRATION_SECTOR_SIZE             0x1000

typedef struct {
    section_head_t      head;
    uint32_t            device;
    uint32_t            cfg_len;
} calibration_section_t;

#define CALIBRATION_SECTION_CFG_RESERVED_LEN      (sizeof(calibration_section_t))

#if defined(__cplusplus)
extern "C" {
#endif

void calibration_section_flash_init(void);
void calibration_values_write(enum CALIBRATION_SECTION_DEVICE device, uint8_t* cfg, uint32_t len);
void calibration_values_read(enum CALIBRATION_SECTION_DEVICE device, uint8_t* cfg, uint32_t len);

#if defined(__cplusplus)
}
#endif

#endif