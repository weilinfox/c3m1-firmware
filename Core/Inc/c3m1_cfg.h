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
#define C3M1_CFG_SCHEMA         16U

/* Save format enum */
#define C3M1_SAVE_DNG           0U
#define C3M1_SAVE_DNG_JPG       1U
#define C3M1_SAVE_JPG           2U

/* White balance mode enum (sorted ascending by Kelvin) */
#define C3M1_WB_AUTO            0U  /* Live Gray-World AWB */
#define C3M1_WB_3200K           1U  /* Tungsten film (3200K) */
#define C3M1_WB_5500K           2U  /* Standard film (5500K) */
#define C3M1_WB_6500K           3U  /* Daylight film (6500K) */

/* Metering mode enum */
#define C3M1_METER_STD          0U  /* Multi-zone average 18% gray */
#define C3M1_METER_CENTER       1U  /* Center-weighted (3x inner 50% box) */
#define C3M1_METER_ETTR         2U  /* Highlight-priority Expose To The Right */

/* Field option counts */
#define C3M1_MAX_ISO_COUNT      4U  /* 0:400, 1:800, 2:1600, 3:3200 */
#define C3M1_MIN_SHT_COUNT      5U  /* 0:1/100, 1:1/50, 2:1/25, 3:1/15, 4:1/8 */
#define C3M1_FLICKER_COUNT      3U  /* 0:50HZ, 1:60HZ, 2:OFF */
#define C3M1_BAT_DISP_COUNT     2U  /* 0:VOLT, 1:PCT */
#define C3M1_FILM_LEVER_COUNT   2U  /* 0:OFF, 1:ON */
#define C3M1_HIST_COUNT         2U  /* 0:OFF, 1:ON */
#define C3M1_PEAK_COLOR_COUNT   4U  /* 0:RED, 1:YELLOW, 2:WHITE, 3:BLUE */
#define C3M1_GRID_COUNT         2U  /* 0:OFF, 1:ON */
#define C3M1_SAVE_MODE_COUNT    3U  /* 0:DNG, 1:DNG+JPG, 2:JPG */
#define C3M1_WB_MODE_COUNT      4U  /* 0:AUTO, 1:3200K, 2:5500K, 3:6500K */
#define C3M1_DEBUG_INFO_COUNT   2U  /* 0:OFF, 1:ON */
#define C3M1_EV_COMP_COUNT      13U /* 0:-2.0EV .. 6:0.0EV .. 12:+2.0EV (1/3 EV step) */
#define C3M1_EV_COMP_ZERO       6U
#define C3M1_METER_MODE_COUNT   3U  /* 0:STD, 1:CENTER, 2:ETTR */
#define C3M1_AUTO_SLEEP_COUNT   6U  /* 0:5M, 1:10M (def), 2:15M, 3:20M, 4:25M, 5:OFF */
#define C3M1_AUTO_SLEEP_DEFAULT 1U  /* 10 minutes */
#define C3M1_AUTO_SLEEP_OFF     5U
#define C3M1_BOOT_CLOCK_COUNT   2U  /* 0:OFF, 1:ON (def) */
#define C3M1_PB_SKIP_BRK_COUNT  2U  /* 0:OFF (def), 1:ON */
#define C3M1_DEL_HDR_BRK_COUNT  2U  /* 0:OFF (def), 1:ON (delete _0.._3 after merge) */
#define C3M1_LENS_FOCAL_COUNT   39U    /* 0:OFF, 1..38: nominal lens focal length 4mm..500mm */
#define C3M1_LENS_FOCAL_DEFAULT 6U     /* Default 6mm stock M12 lens (EQ 54mm @ 9x crop) */
#define C3M1_LENS_FOCAL_MAX_MM  500U   /* Raw nominal focal length (mm), 0=OFF, max 500mm */
#define C3M1_LENS_FNUM_MAX_X10  320U   /* Raw f-number * 10 (0.1 step), 0=F0, max 320 (F32.0) */
#define C3M1_LENS_DIST_MIN      (-100) /* WarpRectilinear kr1 = val * 0.002 (-0.200..+0.200, 0=OFF) */
#define C3M1_LENS_DIST_MAX      100
#define C3M1_LENS_VIG_MAX       50U    /* FixVignetteRadial k0 = val * 0.05 (0.00..+2.50, 0=OFF) */
#define C3M1_LENS_DIST_INCAM_CNT 2U    /* 0:OFF (DNG metadata only, def), 1:ON (VF+JPEG+thumb warp) */

typedef enum
{
    CFG_LOAD_OK = 0,
    CFG_LOAD_MISSING,
    CFG_LOAD_IO_ERR,
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
    uint16_t struct_size;       /* sizeof(c3m1_cfg_t) == 44 */
    uint8_t  fw_major;          /* FW_VER_MAJOR */
    uint8_t  fw_minor;          /* FW_VER_MINOR */
    uint8_t  fw_patch;          /* FW_VER_PATCH */
    uint8_t  lens_dist_incam;   /* 0:OFF (def, DNG OpcodeList3 only), 1:ON (1X VF + JPEG + thumb) */

    /* Persisted camera settings */
    uint8_t  iso_idx;           /* Last ISO mode: 0:AUTO, 1:100, 2:200, 3:400, 4:800, 5:1600 */
    uint8_t  max_auto_iso;      /* 0:400, 1:800, 2:1600 (def), 3:3200 */
    uint8_t  min_auto_sht;      /* 0:1/100, 1:1/50 (def), 2:1/25, 3:1/15, 4:1/8 */
    uint8_t  flicker_mode;      /* 0:50HZ (def), 1:60HZ, 2:OFF */
    uint8_t  bat_disp;          /* 0:VOLT (def), 1:PCT */
    uint8_t  film_lever;        /* 1:ON (def), 0:OFF */
    uint8_t  hist_en;           /* 1:ON (def), 0:OFF */
    uint8_t  peak_color;        /* 0:RED (def), 1:YELLOW, 2:WHITE, 3:BLUE */
    uint8_t  grid_en;           /* 0:OFF (def), 1:ON */
    uint8_t  save_mode;         /* 0:DNG (def), 1:DNG+JPG, 2:JPG */
    uint8_t  wb_mode;           /* 0:AUTO (def), 1:3200K, 2:5500K, 3:6500K */
    uint8_t  debug_info;        /* 0:OFF (def), 1:ON (FPS badge above histogram) */
    uint8_t  ev_comp;           /* 0:-2.0EV .. 6:0.0EV (def) .. 12:+2.0EV (1/3 EV step) */
    uint8_t  meter_mode;        /* 0:STD (def), 1:CENTER, 2:ETTR */
    uint8_t  auto_sleep;        /* 0:5M, 1:10M (def), 2:15M, 3:20M, 4:25M, 5:OFF */
    uint8_t  boot_clock;        /* 0:OFF, 1:ON (def: prompt date/time on boot) */
    uint32_t saved_epoch;       /* Last persisted clock epoch since 2000-01-01 (0=unset) */
    uint8_t  pb_skip_brk;       /* 0:OFF (def), 1:ON (skip HDR _0/_1/_2 when _hdr exists) */
    uint8_t  del_hdr_brk;       /* 0:OFF (def), 1:ON (delete HDR _0.._3 after merge finishes) */
    int8_t   lens_distort;      /* -100..+100 -> WarpRectilinear kr1 = val * 0.002 (def 0=OFF) */
    uint8_t  lens_vignette;     /* 0..50    -> FixVignetteRadial k0 = val * 0.05 (def 0=OFF) */
    uint16_t lens_focal_mm;     /* Raw nominal lens focal length (mm), 0=OFF, def 6 (6mm) */
    uint16_t lens_fnum_x10;     /* Raw lens aperture f-number * 10 (0.1 step), 0=F0, max 320 */

    uint32_t crc32;             /* IEEE 802.3 CRC-32 of preceding 40 bytes */
} c3m1_cfg_t;

extern c3m1_cfg_t g_cfg;

void              c3m1_cfg_set_defaults(c3m1_cfg_t *cfg);
c3m1_cfg_status_t c3m1_cfg_load(c3m1_cfg_t *cfg);
uint8_t           c3m1_cfg_save(const c3m1_cfg_t *cfg);

/* Helpers translating menu options to live AE/P-curve & viewfinder parameters */
uint16_t          c3m1_cfg_max_auto_gain_q4(const c3m1_cfg_t *cfg);
uint16_t          c3m1_cfg_min_auto_aec(const c3m1_cfg_t *cfg);
uint16_t          c3m1_cfg_peak_rgb565(const c3m1_cfg_t *cfg);
uint8_t           c3m1_cfg_wb_gains(const c3m1_cfg_t *cfg, uint16_t *wb_r_q8, uint16_t *wb_b_q8);
uint16_t          c3m1_cfg_ae_target_q4(const c3m1_cfg_t *cfg);
const char       *c3m1_cfg_ev_comp_label(uint8_t ev_comp_idx);
uint16_t          c3m1_cfg_lens_focal_preset_mm(uint8_t idx);
uint8_t           c3m1_cfg_lens_focal_find_idx(uint16_t focal_mm);
uint16_t          c3m1_cfg_lens_focal_eq35(uint16_t focal_mm);
void              c3m1_cfg_format_lens_focal(uint16_t focal_mm, char *out, uint8_t out_sz);
void              c3m1_cfg_format_lens_fnum(uint16_t fnum_x10, char *out, uint8_t out_sz);
void              c3m1_cfg_format_lens_dist(int8_t val, char *out, uint8_t out_sz);
void              c3m1_cfg_format_lens_vig(uint8_t val, char *out, uint8_t out_sz);
uint32_t          c3m1_cfg_auto_sleep_ms(const c3m1_cfg_t *cfg);

#endif /* C3M1_CFG_H_ */
