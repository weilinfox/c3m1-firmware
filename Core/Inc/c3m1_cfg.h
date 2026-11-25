/**
 * @file        c3m1_cfg.h
 * @brief       Persistent binary configuration stored on SD card at 0:CAYE_CFG/c3m1.cfg
 *              with magic, schema, firmware-version, CRC-32, and field-range validation.
 */

#ifndef C3M1_CFG_H_
#define C3M1_CFG_H_

#include <stdint.h>

#define C3M1_CFG_DIR            "0:CAYE_CFG"
#define C3M1_CFG_PATH           "0:CAYE_CFG/c3m1.cfg"
#define C3M1_CFG_MAGIC          0x314D3343UL    /* "C3M1" little-endian */
#define C3M1_CFG_SCHEMA         2U

/* Field option counts */
#define C3M1_MAX_ISO_COUNT      3U  /* 0:400, 1:800, 2:1600 */
#define C3M1_MIN_SHT_COUNT      5U  /* 0:1/100, 1:1/50, 2:1/25, 3:1/15, 4:1/8 */
#define C3M1_FLICKER_COUNT      3U  /* 0:50HZ, 1:60HZ, 2:OFF */
#define C3M1_BAT_DISP_COUNT     2U  /* 0:VOLT, 1:PCT */
#define C3M1_FILM_LEVER_COUNT   2U  /* 0:OFF, 1:ON */
#define C3M1_HIST_COUNT         2U  /* 0:OFF, 1:ON */
#define C3M1_PEAK_COLOR_COUNT   4U  /* 0:RED, 1:YELLOW, 2:WHITE, 3:BLUE */
#define C3M1_GRID_COUNT         2U  /* 0:OFF, 1:ON */

typedef enum
{
    CFG_LOAD_OK = 0,
    CFG_LOAD_MISSING,
    CFG_LOAD_BAD_SIZE,
    CFG_LOAD_BAD_MAGIC,
    CFG_LOAD_BAD_FW_VER,
    CFG_LOAD_BAD_CRC,
    CFG_LOAD_BAD_RANGE,
} c3m1_cfg_status_t;

typedef struct
{
    uint32_t magic;             /* C3M1_CFG_MAGIC */
    uint16_t schema_ver;        /* C3M1_CFG_SCHEMA */
    uint16_t struct_size;       /* sizeof(c3m1_cfg_t) == 32 */
    uint8_t  fw_major;          /* FW_VER_MAJOR */
    uint8_t  fw_minor;          /* FW_VER_MINOR */
    uint8_t  fw_patch;          /* FW_VER_PATCH */
    uint8_t  reserved0;

    /* Persisted camera settings */
    uint8_t  iso_idx;           /* Last ISO mode: 0:AUTO, 1:100, 2:200, 3:400, 4:800, 5:1600 */
    uint8_t  max_auto_iso;      /* 0:400, 1:800, 2:1600 (default 2) */
    uint8_t  min_auto_sht;      /* 0:1/100, 1:1/50 (def), 2:1/25, 3:1/15, 4:1/8 */
    uint8_t  flicker_mode;      /* 0:50HZ (def), 1:60HZ, 2:OFF */
    uint8_t  bat_disp;          /* 0:VOLT (def), 1:PCT */
    uint8_t  film_lever;        /* 1:ON (def), 0:OFF */
    uint8_t  hist_en;           /* 1:ON (def), 0:OFF */
    uint8_t  peak_color;        /* 0:RED (def), 1:YELLOW, 2:WHITE, 3:BLUE */
    uint8_t  grid_en;           /* 0:OFF (def), 1:ON */
    uint8_t  reserved1[7];      /* Reserved for future menu options (zero-filled) */

    uint32_t crc32;             /* IEEE 802.3 CRC-32 of preceding 28 bytes */
} c3m1_cfg_t;

extern c3m1_cfg_t g_cfg;

void              c3m1_cfg_set_defaults(c3m1_cfg_t *cfg);
c3m1_cfg_status_t c3m1_cfg_load(c3m1_cfg_t *cfg);
uint8_t           c3m1_cfg_save(const c3m1_cfg_t *cfg);

/* Helpers translating menu options to live AE/P-curve & viewfinder parameters */
uint16_t          c3m1_cfg_max_auto_gain_q4(const c3m1_cfg_t *cfg);
uint16_t          c3m1_cfg_min_auto_aec(const c3m1_cfg_t *cfg);
uint16_t          c3m1_cfg_peak_rgb565(const c3m1_cfg_t *cfg);

#endif /* C3M1_CFG_H_ */
