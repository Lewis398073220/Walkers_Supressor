/***************************************************************************
 *
 * Copyright 2015-2020 BES.
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
#ifndef __PMU_BEST1305_H__
#define __PMU_BEST1305_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_cmu.h"

#define PMU_REG(r)                          (((r) & 0xFFF) | 0x0000)
#define ANA_REG(r)                          (((r) & 0xFFF) | 0x1000)
#define RF_REG(r)                           (((r) & 0xFFF) | 0x2000)

#define MAX_VMIC_CH_NUM                     2

#ifndef BBPLL_FREQ_HZ
#define BBPLL_FREQ_HZ                       (393000000)
#endif

#if (BBPLL_FREQ_HZ <= 350000000)
#define BBPLL_FREF_SEL                      1
#endif

enum PMU_EFUSE_PAGE_T {
    PMU_EFUSE_PAGE_SECURITY         = 0,
    PMU_EFUSE_PAGE_BOOT             = 1,
    PMU_EFUSE_PAGE_FEATURE          = 2,
    PMU_EFUSE_PAGE_BATTER_LV        = 3,

    PMU_EFUSE_PAGE_BATTER_HV        = 4,
    PMU_EFUSE_PAGE_SW_CFG           = 5,
    PMU_EFUSE_PAGE_PROD_TEST        = 6,
    PMU_EFUSE_PAGE_RESERVED_7       = 7,

    PMU_EFUSE_PAGE_BT_POWER         = 8,
    PMU_EFUSE_PAGE_DCCALIB2_L       = 9,
    PMU_EFUSE_PAGE_DCCALIB2_L_LP    = 10,
    PMU_EFUSE_PAGE_DCCALIB_L        = 11,

    PMU_EFUSE_PAGE_DCCALIB_L_LP     = 12,
    PMU_EFUSE_PAGE_RESERVED_13      = 13,
    PMU_EFUSE_PAGE_MODEL            = 14,
    PMU_EFUSE_PAGE_RESERVED_15      = 15,

    PMU_EFUSE_PAGE_QTY,
};

enum PMU_IRQ_TYPE_T {
    PMU_IRQ_TYPE_GPADC,
    PMU_IRQ_TYPE_RTC,
    PMU_IRQ_TYPE_CHARGER,
    PMU_IRQ_TYPE_GPIO,
    PMU_IRQ_TYPE_WDT,

    PMU_IRQ_TYPE_QTY
};

enum PMU_PLL_DIV_TYPE_T {
    PMU_PLL_DIV_DIG,
    PMU_PLL_DIV_CODEC,
};

enum PMU_BIG_BANDGAP_USER_T {
    PMU_BIG_BANDGAP_USER_GPADC          = (1 << 0),
};

uint8_t pmu_gpio_setup_irq(enum HAL_GPIO_PIN_T pin, const struct HAL_GPIO_IRQ_CFG_T *cfg);

void pmu_codec_hppa_enable(int enable);

void pmu_codec_mic_bias_enable(uint32_t map);

void pmu_codec_mic_bias_lowpower_mode(uint32_t map, int enable);

int pmu_codec_volt_ramp_up(void);

int pmu_codec_volt_ramp_down(void);

void pmu_pll_div_reset_set(enum HAL_CMU_PLL_T pll);

void pmu_pll_div_reset_clear(enum HAL_CMU_PLL_T pll);

void pmu_pll_div_set(enum HAL_CMU_PLL_T pll, enum PMU_PLL_DIV_TYPE_T type, uint32_t div);

void pmu_pll_freq_reg_set(uint16_t low, uint16_t high, uint16_t high2);

void bbpll_freq_pll_config(uint32_t freq);

void bbpll_auto_calib_spd(uint32_t codec_freq, uint32_t div);

void pmu_pll_codec_clock_enable(bool en);

void pmu_led_set_hiz(enum HAL_GPIO_PIN_T pin);

void pmu_led_uart_enable(enum HAL_IOMUX_PIN_T pin);

void pmu_led_uart_disable(enum HAL_IOMUX_PIN_T pin);

void pmu_big_bandgap_enable(enum PMU_BIG_BANDGAP_USER_T user, int enable);

void pmu_rf_ana_init(void);

void pmu_bt_reconn(bool en);

void check_efuse_for_different_chip(void);

void pmu_get_vbat_calib_value(unsigned short *val_lv, unsigned short *val_hv);

bool pmu_force_lp_bg_valid_check(void);

#ifdef __cplusplus
}
#endif

#endif

