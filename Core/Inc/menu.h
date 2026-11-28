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

/* Volatile runtime-only drive / shooting mode (never persisted to c3m1.cfg, resets to SINGLE on boot) */
#define C3M1_DRIVE_SINGLE     0U /* Standard single shot */
#define C3M1_DRIVE_BURST      1U /* Continuous shooting while SHOT is held */
#define C3M1_DRIVE_AEB        2U /* 3-frame AEB (0/-2/+2 EV) without HDR merge */
#define C3M1_DRIVE_MODE_COUNT 3U

/* Volatile runtime-only HDR merge mode (never persisted to c3m1.cfg, resets to OFF on boot) */
#define C3M1_HDR_OFF        0U
#define C3M1_HDR_12B_2F     1U  /* 2-frame (-1/+1 EV) in-camera merge -> 12-bit linear DNG */
#define C3M1_HDR_14B_3F     2U  /* 3-frame (-2/0/+2 EV) in-camera merge -> 14-bit linear DNG */
#define C3M1_HDR_16B_4F     3U  /* 4-frame (-3/-1/+1/+3 EV) in-camera merge -> 16-bit linear DNG */
#define C3M1_HDR_MODE_COUNT 4U

/* Volatile runtime-only JSR super-resolution burst mode (never persisted to c3m1.cfg) */
#define C3M1_JSR_OFF        0U
#define C3M1_JSR_7F         1U  /* 7 light frames + 1 lens-covered dark frame */
#define C3M1_JSR_14F        2U  /* 14 light frames + 1 lens-covered dark frame */
#define C3M1_JSR_30F        3U  /* 30 light frames + 1 lens-covered dark frame */
#define C3M1_JSR_MODE_COUNT 4U

typedef struct
{
    c3m1_cfg_t *cfg;
    uint8_t    *drive_mode;         /* Volatile runtime drive mode (NULL if unused) */
    uint8_t    *hdr_mode;           /* Volatile runtime HDR mode (NULL if unused) */
    uint8_t    *jsr_mode;           /* Volatile runtime JSR burst mode (NULL if unused) */
    FATFS      *fs;
    uint8_t    *work_buf;           /* Shared 4096-byte buffer for f_mkfs() */
    uint16_t    work_buf_size;
    volatile uint16_t *shutter_count;
    uint8_t   (*prepare_format_fn)(void);       /* Ensures SDIO init before f_mkfs on unmounted card */
    uint8_t   (*remount_sd_fn)(void);
    uint8_t   (*is_low_bat_fn)(void);
    uint8_t   (*is_sd_present_fn)(void);
    uint8_t   (*get_cnt_mode_fn)(void);         /* Returns 0=STOCK, 1=WEAR-LVL */
    uint16_t  (*get_cnt_slots_fn)(void);        /* Returns used slots in counter Flash page */
    uint8_t   (*set_cnt_mode_fn)(uint8_t mode); /* Converts Flash counter format; 1=OK, 0=ERR */
    uint8_t     sd_ready;                       /* Input/Output: 1 if FatFs mounted OK, 0 if no/unformatted SD */
    uint8_t     sd_removed_in_menu;             /* Output: 1 if an inserted card was hot-removed while menu open */
    uint8_t     woke_from_sleep;                /* Output: 1 if modal exited due to low-bat or auto-sleep wake */
} c3m1_menu_ctx_t;

void c3m1_menu_run(c3m1_menu_ctx_t *ctx);
uint8_t c3m1_menu_set_clock_ui(c3m1_datetime_t *dt);

#endif /* C3M1_MENU_H_ */
