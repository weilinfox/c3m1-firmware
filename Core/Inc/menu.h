/**
 * @file        menu.h
 * @brief       Pure-English ASCII menu UI for C3M1 (dial navigation, SHOT confirm, ASA back).
 */

#ifndef C3M1_MENU_H_
#define C3M1_MENU_H_

#include <stdint.h>
#include "c3m1_cfg.h"
#include "c3m1_clock.h"
#include "fatfs.h"

typedef struct
{
    c3m1_cfg_t *cfg;
    FATFS      *fs;
    uint8_t    *work_buf;           /* Shared 4096-byte buffer for f_mkfs() */
    uint16_t    work_buf_size;
    volatile uint16_t *shutter_count;
    uint8_t   (*remount_sd_fn)(void);
    uint8_t   (*is_low_bat_fn)(void);
    uint8_t   (*is_sd_present_fn)(void);
    uint8_t     sd_removed_in_menu; /* Set to 1 if card was hot-removed while menu open */
} c3m1_menu_ctx_t;

void c3m1_menu_run(c3m1_menu_ctx_t *ctx);
void c3m1_menu_set_clock_ui(c3m1_datetime_t *dt);

#endif /* C3M1_MENU_H_ */
