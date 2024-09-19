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
#include <stdio.h>
#include <string.h>
#include <string.h>
#include "cmsis.h"
#include "calibration_section.h"
#include "hal_timer.h"
#include "hal_trace.h"
#include "hal_norflash.h"
#include "norflash_api.h"

extern uint32_t __calibration_value_start[];
extern uint32_t __calibration_value_end[];

static uint32_t section_device_length[CALIBRATION_SECTION_DEVICE_NUM] = {
    HEAD_TRACK_CALIBRATION_SECTOR_SIZE,
};

static bool calibration_section_inited = false;

static uint32_t calibration_section_get_device_addr_offset(uint32_t device)
{
    ASSERT(device < CALIBRATION_SECTION_DEVICE_NUM, "[%s] device(%d) >= CALIBRATION_SECTION_DEVICE_NUM", __func__, device);

    uint32_t addr_offset = 0;

    for (uint32_t i=0; i<device; i++)
    {
        addr_offset += section_device_length[i];
    }

    return addr_offset;
}

void calibration_section_flash_init(void)
{
    if(calibration_section_inited)
    {
        return;
    }

    enum NORFLASH_API_RET_T result;
    enum HAL_FLASH_ID_T flash_id;
    uint32_t sector_size = 0;
    uint32_t block_size = 0;
    uint32_t page_size = 0;

    flash_id = norflash_api_get_dev_id_by_addr((uint32_t)__calibration_value_start);
    hal_norflash_get_size(flash_id,
                          NULL,
                          &block_size,
                          &sector_size,
                          &page_size);
    result = norflash_api_register(NORFLASH_API_MODULE_ID_CALIBRATION,
                                   flash_id,
                                   ((uint32_t)__calibration_value_start),
                                   ((uint32_t)__calibration_value_end - (uint32_t)__calibration_value_start),
                                   block_size,
                                   sector_size,
                                   page_size,
                                   ((uint32_t)__calibration_value_end - (uint32_t)__calibration_value_start),
                                   NULL);
    ASSERT(result == NORFLASH_API_OK, "head_track_section_init: module register failed! result = %d.", result);

    calibration_section_inited = true;
}

enum NORFLASH_API_RET_T calibration_erase_section(uint32_t start_addr, uint32_t len, bool is_async)
{
    //erase audio device[x] section
    enum NORFLASH_API_RET_T ret = 0;
    uint32_t t_size = 0;
    uint32_t b_size = 0;
    uint32_t s_size = 0;
    uint32_t p_size = 0;
    enum HAL_FLASH_ID_T flash_id;

    norflash_api_get_dev_id(NORFLASH_API_MODULE_ID_CALIBRATION, &flash_id);
    hal_norflash_get_size(flash_id, &t_size, &b_size, &s_size, &p_size);

    // judge start addr and len sector size alignment
    ASSERT(((start_addr & (s_size - 1)) == 0 &&
            (len & (s_size - 1)) == 0),
           "%s: No sec size alignment! start_addr = 0x%x, len = 0x%x",
           __func__,
           start_addr,
           len);

    do
    {
        hal_trace_pause();
        ret = norflash_api_erase(NORFLASH_API_MODULE_ID_CALIBRATION, start_addr,
                                 s_size, is_async);
        hal_trace_continue();
        if(ret != NORFLASH_API_OK)
        {
            TRACE(3, "%s:offset = 0x%x,ret = %d.", __func__, start_addr, ret);
            return ret;
        }
        start_addr += s_size;
        len -= s_size;
        if (len == 0)
        {
            break;
        }
    } while (1);

    return ret;
}

enum NORFLASH_API_RET_T calibration_write(uint32_t start_addr, uint8_t* ptr, uint32_t len, bool is_async)
{
    enum NORFLASH_API_RET_T ret;

    TRACE(1,"%s: write: 0x%x,0x%x", __func__, start_addr, len);

    ret = norflash_api_write(NORFLASH_API_MODULE_ID_CALIBRATION,
                start_addr,
                ptr,
                len,
                is_async);
    if(ret != NORFLASH_API_OK)
    {
        TRACE(4,"%s: addr = 0x%x ret = %d ch =%02x",
                        __func__, start_addr, ret, *(ptr-1));
    }

    return ret;
}

enum NORFLASH_API_RET_T calibration_read(uint32_t start_addr, uint8_t* ptr, uint32_t len, bool is_async)
{
    enum NORFLASH_API_RET_T ret;

    TRACE(1,"%s: read: 0x%x,0x%x", __func__, start_addr, len);

    ret = norflash_api_read(NORFLASH_API_MODULE_ID_CALIBRATION,
                start_addr,
                ptr,
                len);
    if(ret != NORFLASH_API_OK)
    {
        TRACE(4,"%s: offset = 0x%x ret = %d ch =%02x",
                        __func__, start_addr, ret, *(ptr-1));
    }

    return ret;
}

int calibration_section_store_cfg(uint32_t device, uint8_t *cfg, uint32_t len)
{
    uint32_t addr_start = 0;
    enum NORFLASH_API_RET_T flash_opt_res;
    bool is_async = false;
    uint32_t t_size = 0;
    uint32_t b_size = 0;
    uint32_t s_size = 0;
    uint32_t p_size = 0;
    uint32_t flashOffset;
    enum HAL_FLASH_ID_T flash_id;

    calibration_section_flash_init();

    norflash_api_get_dev_id(NORFLASH_API_MODULE_ID_CALIBRATION, &flash_id);
    hal_norflash_get_size(flash_id, &t_size, &b_size, &s_size, &p_size);

    // get flash offset
    flashOffset = calibration_section_get_device_addr_offset(device);
    ASSERT((flashOffset & (s_size - 1)) == 0,
            "%s: No sec size alignment! offset = 0x%x",
            __func__,
            flashOffset);

    /// get logic address to write
    flash_opt_res = norflash_api_get_base_addr(NORFLASH_API_MODULE_ID_CALIBRATION, &addr_start);
    addr_start = addr_start + flashOffset;

    TRACE(2,"[%s] len = %d", __func__, len);
    TRACE(2,"[%s] addr_start = 0x%x", __func__, addr_start);
    TRACE(2,"[%s] block length = 0x%x", __func__, section_device_length[device]);

    flash_opt_res = calibration_erase_section(addr_start, section_device_length[device], is_async);
    if (flash_opt_res)
    {
        TRACE(2,"[%s] ERROR: erase flash res = %d", __func__, flash_opt_res);
        return flash_opt_res;
    }

    flash_opt_res = calibration_write(addr_start, (uint8_t *)cfg, len, is_async);
    if (flash_opt_res)
    {
        TRACE(2,"[%s] ERROR: write flash res = %d", __func__, flash_opt_res);
        return flash_opt_res;
    }

    // audio_section_t *section_read_ptr = audio_section_get_device_ptr(device);
    // check

    return 0;
}

int calibration_section_load_cfg(uint32_t device, uint8_t *cfg, uint32_t len)
{
    uint32_t addr_start = 0;
    enum NORFLASH_API_RET_T flash_opt_res;
    bool is_async = false;
    uint32_t t_size = 0;
    uint32_t b_size = 0;
    uint32_t s_size = 0;
    uint32_t p_size = 0;
    uint32_t flashOffset;
    enum HAL_FLASH_ID_T flash_id;

    calibration_section_flash_init();

    norflash_api_get_dev_id(NORFLASH_API_MODULE_ID_CALIBRATION, &flash_id);
    hal_norflash_get_size(flash_id, &t_size, &b_size, &s_size, &p_size);

    // get flash offset
    flashOffset = calibration_section_get_device_addr_offset(device);
    ASSERT((flashOffset & (s_size - 1)) == 0,
            "%s: No sec size alignment! offset = 0x%x",
            __func__,
            flashOffset);

    flash_opt_res = norflash_api_get_base_addr(NORFLASH_API_MODULE_ID_CALIBRATION, &addr_start);
    addr_start = addr_start + flashOffset;

    TRACE(2,"[%s] len = %d", __func__, len);
    TRACE(2,"[%s] addr_start = 0x%x", __func__, addr_start);
    TRACE(2,"[%s] block length = 0x%x", __func__, section_device_length[device]);

    flash_opt_res = calibration_read(addr_start, (uint8_t *)cfg, len, is_async);
    if (flash_opt_res)
    {
        TRACE(2,"[%s] ERROR: read flash res = %d", __func__, flash_opt_res);
        return flash_opt_res;
    }

    return 0;
}

void calibration_values_write(enum CALIBRATION_SECTION_DEVICE device, uint8_t* cfg, uint32_t len)
{
    calibration_section_store_cfg(device, cfg, len);
}

void calibration_values_read(enum CALIBRATION_SECTION_DEVICE device, uint8_t* cfg, uint32_t len)
{
    calibration_section_load_cfg(device, cfg, len);
}