/**
 * @file        playback.c
 * @brief       On-camera photo playback (DNG embedded preview / legacy RAW / Baseline JPEG,
 *              1:1 zoom & pan with mini viewport map, and paired DNG+JPG deletion).
 */

#include <stdio.h>
#include <string.h>

#include "playback.h"
#include "main.h"
#include "c3m1_dng.h"
#include "c3m1_jdec.h"
#include "c3m1_jpeg.h"
#include "fatfs.h"
#include "../../BSP/KEY/key.h"
#include "../../BSP/LCD/lcd.h"
#include "../../SYSTEM/delay/delay.h"

#define PB_SCR_W            320U
#define PB_VIEW_H           240U
#define PB_BAR_Y            220U
#define PB_BAR_H            20U

/* Full-width 4:3 1X display (exact 2x scale of 160x120 / 1/2x of 640x480, zero side pillars) */
#define PB_1X_W             320U
#define PB_1X_H             240U
#define PB_1X_X0            0U

/* 1X display modes cycled by short-pressing SHOT */
#define PB_DISP_CLEAN       0U  /* 1X full-width clean photo */
#define PB_DISP_CLIP        1U  /* 1X full-width photo + highlight/shadow clipping overlay */
#define PB_DISP_INFO        2U  /* Sony-style: top-left 160x120 CLIP thumb + YRGB hist + details */

/* Sony-style INFO mode geometry (320x220 viewport above 20px bottom bar) */
#define PB_INFO_TH_X        4U
#define PB_INFO_TH_Y        6U
#define PB_INFO_TH_W        160U
#define PB_INFO_TH_H        120U

/* Extra RGB565 colors for INFO histogram & borders */
#define PB_COL_DARKGRAY     0x4208U
#define PB_COL_HIST_BG      0x0841U
#define PB_COL_HIST_GRID    0x2104U
#define PB_COL_HIST_R       0xF945U
#define PB_COL_HIST_G       0x2708U
#define PB_COL_HIST_B       0x3BDFU

/* Zoom 1:1 pan step sizes (pixels) */
#define PB_PAN_STEP_X       40U
#define PB_PAN_STEP_Y       32U

/* Photo index capacity inside shared 11.3 KB scratch buffer: 800 * 6 = 4800 B (4-byte aligned) */
#define PB_MAX_PHOTOS       800U
#define PB_INDEX_BYTES      (PB_MAX_PHOTOS * sizeof(pb_entry_t))

#define PB_FLAG_DNG         (1U << 0)
#define PB_FLAG_JPG         (1U << 1)
#define PB_FLAG_JSR_HDR     (1U << 2) /* Representative JSR frame is CDIxxxxx_hdr.dng */
#define PB_FLAG_JSR_BRK0    (1U << 3) /* Fallback representative JSR frame is CDIxxxxx_0.dng (partial HDR set) */

/* Filename suffix / group variant (sort order puts _hdr last so playback opens on merged HDR) */
#define PB_VAR_NORMAL       0U  /* CDIxxxxx.{dng,jpg} */
#define PB_VAR_HDR_0        1U  /* CDIxxxxx_0.dng     */
#define PB_VAR_HDR_1        2U  /* CDIxxxxx_1.dng     */
#define PB_VAR_HDR_2        3U  /* CDIxxxxx_2.dng     */
#define PB_VAR_HDR_3        4U  /* CDIxxxxx_3.dng     */
#define PB_VAR_HDR          5U  /* CDIxxxxx_hdr.dng   */
#define PB_VAR_JSR_7        6U  /* CDIxxxxx_7/  (JSR 7F burst dir)  */
#define PB_VAR_JSR_14       7U  /* CDIxxxxx_14/ (JSR 14F burst dir) */
#define PB_VAR_JSR_30       8U  /* CDIxxxxx_30/ (JSR 30F burst dir) */

typedef struct
{
    uint16_t num;           /* 1..65535 from CDIxxxxx */
    uint16_t date_packed;   /* 0 = legacy root CAYE_CAM/; else ((year-1980)<<9)|(mon<<5)|day */
    uint8_t  flags;         /* PB_FLAG_DNG | PB_FLAG_JPG | PB_FLAG_JSR_HDR */
    uint8_t  variant;       /* PB_VAR_NORMAL / PB_VAR_HDR_* / PB_VAR_JSR_* */
} pb_entry_t;

_Static_assert(sizeof(pb_entry_t) == 6U, "pb_entry_t must be 6 bytes");
_Static_assert((PB_INDEX_BYTES % 4U) == 0U, "PB_INDEX_BYTES must be 4-byte aligned");
#define PB_AFTER_IDX_NEEDED 27400UL
#define PB_JDEC_STRIP_OFF   (((uint32_t)sizeof(c3m1_jdec_t) + 3UL) & ~3UL)
#define PB_ZOOM_NR_CHROMA_BYTES (3UL * 5UL * (PB_SCR_W / 2U)) /* 2,400 B */
#define PB_ZOOM_NR_LUMA_BYTES   (5UL * PB_SCR_W)              /* 1,600 B */
#define PB_ZOOM_NR_TOTAL_BYTES  (PB_ZOOM_NR_CHROMA_BYTES + PB_ZOOM_NR_LUMA_BYTES) /* 4,000 B */
#define PB_THUMB_NR_BYTES       ((3UL * 3UL * C3M1_DNG_THUMB_W) + (2UL * C3M1_DNG_THUMB_W * 3UL)) /* 2,400 B */
_Static_assert((PB_VIEW_H % 2U) == 0U && PB_VIEW_H >= 6U, "PB_VIEW_H must be even and >= 6");
_Static_assert((PB_SCR_W % 2U) == 0U, "PB_SCR_W must be even");
_Static_assert((PB_INDEX_BYTES + PB_AFTER_IDX_NEEDED) <= (50U * 644U), "Playback scratch exceeds shared s_bayer_strip");
_Static_assert((PB_JDEC_STRIP_OFF + (PB_1X_W * 8UL * 2UL)) <= PB_AFTER_IDX_NEEDED, "JPEG decoder + 8-row strip exceeds scratch");
_Static_assert((PB_SCR_W * 6UL + 17UL * C3M1_DNG_THUMB_W * 3UL + PB_THUMB_NR_BYTES) <= PB_AFTER_IDX_NEEDED, "1X thumb + NR exceeds scratch");
_Static_assert((512UL + 18UL * 1300UL) <= PB_AFTER_IDX_NEEDED, "Unwarped DNG zoom batch (keep=2 + 16) exceeds scratch");
_Static_assert((512UL + 18UL * 644UL + PB_ZOOM_NR_TOTAL_BYTES) <= PB_AFTER_IDX_NEEDED, "Unwarped 8-bit DNG zoom + NR exceeds scratch");
_Static_assert((512UL + 14UL * 1300UL + PB_ZOOM_NR_TOTAL_BYTES) <= PB_AFTER_IDX_NEEDED, "Unwarped 16-bit DNG zoom + NR exceeds scratch");
_Static_assert((512UL + 35UL * 340UL + 16UL * 644UL + PB_ZOOM_NR_TOTAL_BYTES) <= PB_AFTER_IDX_NEEDED, "Warped 8-bit DNG zoom + NR exceeds scratch");
_Static_assert((35UL * 340UL * 2UL + PB_ZOOM_NR_CHROMA_BYTES + 3UL * PB_SCR_W) <= PB_AFTER_IDX_NEEDED, "Warped 16-bit DNG zoom + split NR exceeds scratch");

typedef struct
{
    uint8_t  valid;
    uint8_t  has_thumb;
    uint32_t thumb_off;
    uint16_t raw_w;
    uint16_t raw_h;
    uint16_t raw_bps;       /* 8 or 16 */
    uint32_t raw_off;
    uint16_t white_level;   /* Default 4095 (or 255/1023 for legacy 16-bit) */
    uint16_t bl4[4];        /* 12-bit GRBG BlackLevel: {G0, R, B, G1} */
    uint16_t wb_r_q8;       /* Q8 Red WB gain */
    uint16_t wb_b_q8;       /* Q8 Blue WB gain */
    int16_t  baseline_ev_x100; /* Signed BaselineExposure (0xC62A) * 100 */
    uint32_t vig_scale_q16; /* OpcodeList3 FixVignetteRadial scale (0 = none) */
    int32_t  warp_kr0_q16;  /* OpcodeList3 WarpRectilinear kr0 (Q16) */
    int32_t  warp_kr1_q16;  /* OpcodeList3 WarpRectilinear kr1 (Q16) */
    uint32_t exp_num;       /* EXIF ExposureTime numerator */
    uint32_t exp_den;       /* EXIF ExposureTime denominator */
    uint16_t fnum_x10;      /* EXIF FNumber * 10 (0 = absent) */
    uint16_t focal_x10;     /* EXIF FocalLength * 10 (0 = absent) */
    uint16_t focal_eq35;    /* EXIF FocalLengthIn35mmFilm (0 = absent) */
    uint16_t iso;           /* EXIF ISOSpeedRatings */
    int16_t  ev_bias_x10;   /* EXIF ExposureBiasValue in 0.1 EV steps */
    uint8_t  has_lin_tbl;
    uint8_t  has_wb;
    uint8_t  has_bev;
    uint8_t  has_warp;
    uint8_t  has_exp;
    uint8_t  has_fnum;
    uint8_t  has_iso;
    uint8_t  has_ev;
    uint8_t  color_mode;    /* 0:COLOR, 1:B&W, 2:B&W HI, 3:B&W RED (from IFD0 0x010E) */
    char     datetime[20];  /* "YYYY-MM-DD HH:MM:SS" if present in TIFF 0x0132 */
} dng_info_t;

typedef struct
{
    uint16_t y[64];
    uint16_t r[64];
    uint16_t g[64];
    uint16_t b[64];
    uint32_t total_px;
    uint32_t hi_px;
    uint32_t lo_px;
    uint32_t sum_y;
} pb_hist_t;

typedef struct
{
    char     datetime[20];  /* "YYYY-MM-DD HH:MM:SS" or empty */
    uint32_t dng_bytes;
    uint32_t jpg_bytes;
    uint32_t exp_num;       /* EXIF ExposureTime numerator */
    uint32_t exp_den;       /* EXIF ExposureTime denominator */
    uint16_t img_w;
    uint16_t img_h;
    uint16_t wb_kelvin;     /* >0 when DNG AsShotNeutral is available */
    uint16_t white_level;   /* DNG WhiteLevel (e.g. 4064 / 16256) */
    uint16_t fnum_x10;      /* EXIF FNumber * 10 (0 = absent) */
    uint16_t focal_x10;     /* EXIF FocalLength * 10 (0 = absent) */
    uint16_t focal_eq35;    /* EXIF FocalLengthIn35mmFilm (0 = absent) */
    uint16_t iso;           /* EXIF ISOSpeedRatings */
    int16_t  ev_bias_x10;   /* EXIF ExposureBiasValue in 0.1 EV steps */
    uint8_t  raw_bps;       /* 8 or 16 for DNG, 0 for JPG-only */
    uint8_t  has_lin_tbl;
    uint8_t  is_hdr;        /* 1 for 16-bit merged HDR DNG (BaselineExposure present) */
    uint8_t  jpg_sub;       /* 1=4:2:0, 2=4:2:2, 3=4:4:4, 4=GRAY */
    uint8_t  has_exp;
    uint8_t  has_fnum;
    uint8_t  has_iso;
    uint8_t  has_ev;
    uint8_t  jsr_lights;    /* Actual light frames found inside JSR burst dir */
    uint8_t  jsr_has_dark;  /* 1 if a *_dark.dng exists inside JSR burst dir */
    uint16_t jsr_files;     /* Total file count inside JSR burst dir */
    uint32_t jsr_bytes;     /* Total bytes across all files in JSR burst dir */
} pb_info_meta_t;

/* ======================== Little-Endian Helpers ======================== */
static inline uint16_t rd16(const uint8_t *b, uint32_t off)
{
    return (uint16_t)((uint16_t)b[off] | ((uint16_t)b[off + 1U] << 8));
}

static inline uint32_t rd32(const uint8_t *b, uint32_t off)
{
    return (uint32_t)b[off] |
           ((uint32_t)b[off + 1U] << 8) |
           ((uint32_t)b[off + 2U] << 16) |
           ((uint32_t)b[off + 3U] << 24);
}

static int8_t dial_signed_delta(uint8_t prev, uint8_t curr)
{
    int8_t d = (int8_t)curr - (int8_t)prev;
    if (d > 8)  d -= 16;
    if (d < -8) d += 16;
    return d;
}

static void pb_draw_center_box(const char *msg, uint16_t color)
{
    const uint16_t x0 = 24U, y0 = 92U, x1 = 295U, y1 = 132U;
    size_t len = strlen(msg);
    uint16_t box_w = (uint16_t)(x1 - x0 + 1U);
    uint16_t txt_w = (uint16_t)(len * 8U);
    uint16_t tx = (txt_w < box_w) ? (uint16_t)(x0 + (box_w - txt_w) / 2U) : (uint16_t)(x0 + 4U);

    g_back_color = BLACK;
    lcd_fill(x0, y0, x1, y1, BLACK);
    lcd_draw_rectangle(x0, y0, x1, y1, color);
    lcd_show_string(tx, y0 + 12U, box_w - 8U, 16, 16, msg, color);
}

/* ======================== Photo Directory Scanner ======================== */
static const char *pb_variant_suffix(uint8_t variant)
{
    switch (variant)
    {
        case PB_VAR_HDR_0: return "_0";
        case PB_VAR_HDR_1: return "_1";
        case PB_VAR_HDR_2: return "_2";
        case PB_VAR_HDR_3: return "_3";
        case PB_VAR_HDR:   return "_hdr";
        default:           return "";
    }
}

static uint8_t parse_cdi_filename(const char *name, uint16_t *out_num,
                                  uint8_t *out_flag, uint8_t *out_variant)
{
    /* Match CDIxxxxx[|_0|_1|_2|_hdr].{dng|jpg} (case-insensitive, 5-digit standard C3M1 format) */
    size_t len = strlen(name);
    if (len < 12U || len > 16U) return 0U;

    char c0 = name[0], c1 = name[1], c2 = name[2];
    if ((c0 != 'C' && c0 != 'c') || (c1 != 'D' && c1 != 'd') || (c2 != 'I' && c2 != 'i'))
    {
        return 0U;
    }

    uint32_t num = 0U;
    for (uint8_t i = 3U; i < 8U; i++)
    {
        if (name[i] < '0' || name[i] > '9') return 0U;
        num = num * 10UL + (uint32_t)(name[i] - '0');
    }
    if (num > 65535UL) return 0U;

    uint8_t variant = PB_VAR_NORMAL;
    size_t  dot_pos = 8U;
    if (name[8] == '.')
    {
        variant = PB_VAR_NORMAL;
        dot_pos = 8U;
    }
    else if (name[8] == '_')
    {
        if (len == 14U && name[10] == '.' && name[9] >= '0' && name[9] <= '3')
        {
            variant = (uint8_t)(PB_VAR_HDR_0 + (uint8_t)(name[9] - '0'));
            dot_pos = 10U;
        }
        else if (len == 16U && name[12] == '.' &&
                 (name[9]  == 'h' || name[9]  == 'H') &&
                 (name[10] == 'd' || name[10] == 'D') &&
                 (name[11] == 'r' || name[11] == 'R'))
        {
            variant = PB_VAR_HDR;
            dot_pos = 12U;
        }
        else
        {
            return 0U;
        }
    }
    else
    {
        return 0U;
    }

    if (len != dot_pos + 4U) return 0U;

    char e0 = name[dot_pos + 1U], e1 = name[dot_pos + 2U], e2 = name[dot_pos + 3U];
    if ((e0 == 'd' || e0 == 'D') && (e1 == 'n' || e1 == 'N') && (e2 == 'g' || e2 == 'G'))
    {
        *out_num     = (uint16_t)num;
        *out_flag    = PB_FLAG_DNG;
        *out_variant = variant;
        return 1U;
    }
    if ((e0 == 'j' || e0 == 'J') && (e1 == 'p' || e1 == 'P') && (e2 == 'g' || e2 == 'G'))
    {
        *out_num     = (uint16_t)num;
        *out_flag    = PB_FLAG_JPG;
        *out_variant = variant;
        return 1U;
    }
    return 0U;
}

static uint8_t parse_yyyymmdd_dir(const char *name, uint16_t *out_packed)
{
    if (strlen(name) != 8U) return 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if (name[i] < '0' || name[i] > '9') return 0U;
    }

    uint16_t year = (uint16_t)((name[0] - '0') * 1000U +
                               (name[1] - '0') * 100U +
                               (name[2] - '0') * 10U +
                               (name[3] - '0'));
    uint16_t mon  = (uint16_t)((name[4] - '0') * 10U + (name[5] - '0'));
    uint16_t day  = (uint16_t)((name[6] - '0') * 10U + (name[7] - '0'));

    if (year < 1980U || year > 2107U || mon < 1U || mon > 12U || day < 1U || day > 31U)
    {
        return 0U;
    }

    *out_packed = (uint16_t)(((year - 1980U) << 9) | (mon << 5) | day);
    return 1U;
}

static inline uint8_t pb_is_jsr(const pb_entry_t *e)
{
    return (e->variant >= PB_VAR_JSR_7 && e->variant <= PB_VAR_JSR_30) ? 1U : 0U;
}

static uint8_t pb_jsr_count(uint8_t variant)
{
    if (variant == PB_VAR_JSR_30) return 30U;
    if (variant == PB_VAR_JSR_14) return 14U;
    if (variant == PB_VAR_JSR_7)  return 7U;
    return 0U;
}

static uint8_t parse_jsr_dir(const char *name, uint16_t *out_num, uint8_t *out_variant)
{
    /* Match CDIxxxxx_7, CDIxxxxx_14, CDIxxxxx_30 (case-insensitive) */
    size_t len = strlen(name);
    if (len != 10U && len != 11U) return 0U;

    char c0 = name[0], c1 = name[1], c2 = name[2];
    if ((c0 != 'C' && c0 != 'c') || (c1 != 'D' && c1 != 'd') || (c2 != 'I' && c2 != 'i'))
    {
        return 0U;
    }

    uint32_t num = 0U;
    for (uint8_t i = 3U; i < 8U; i++)
    {
        if (name[i] < '0' || name[i] > '9') return 0U;
        num = num * 10UL + (uint32_t)(name[i] - '0');
    }
    if (num > 65535UL || name[8] != '_') return 0U;

    if (len == 10U && name[9] == '7')
    {
        *out_num     = (uint16_t)num;
        *out_variant = PB_VAR_JSR_7;
        return 1U;
    }
    if (len == 11U && name[9] == '1' && name[10] == '4')
    {
        *out_num     = (uint16_t)num;
        *out_variant = PB_VAR_JSR_14;
        return 1U;
    }
    if (len == 11U && name[9] == '3' && name[10] == '0')
    {
        *out_num     = (uint16_t)num;
        *out_variant = PB_VAR_JSR_30;
        return 1U;
    }
    return 0U;
}

static void pb_build_day_dir(const char *photo_dir, uint16_t date_packed,
                             char *out, size_t out_sz)
{
    unsigned int y = 1980U + (unsigned int)(date_packed >> 9);
    unsigned int m = (unsigned int)((date_packed >> 5) & 0x0FU);
    unsigned int d = (unsigned int)(date_packed & 0x1FU);
    snprintf(out, out_sz, "%s/%04u%02u%02u", photo_dir, y, m, d);
}

static void pb_build_jsr_dir(const char *photo_dir, const pb_entry_t *e,
                             char *out, size_t out_sz)
{
    unsigned int y = 1980U + (unsigned int)(e->date_packed >> 9);
    unsigned int m = (unsigned int)((e->date_packed >> 5) & 0x0FU);
    unsigned int d = (unsigned int)(e->date_packed & 0x1FU);
    snprintf(out, out_sz, "%s/%04u%02u%02u/CDI%05u_%u",
             photo_dir, y, m, d, (unsigned int)e->num, (unsigned int)pb_jsr_count(e->variant));
}

static void pb_build_photo_path(const char *photo_dir, const pb_entry_t *e,
                                const char *ext, char *out, size_t out_sz)
{
    if (pb_is_jsr(e))
    {
        unsigned int y = 1980U + (unsigned int)(e->date_packed >> 9);
        unsigned int m = (unsigned int)((e->date_packed >> 5) & 0x0FU);
        unsigned int d = (unsigned int)(e->date_packed & 0x1FU);
        const char *fsuf = (e->flags & PB_FLAG_JSR_HDR)  ? "_hdr"
                         : (e->flags & PB_FLAG_JSR_BRK0) ? "_0" : "";
        snprintf(out, out_sz, "%s/%04u%02u%02u/CDI%05u_%u/CDI%05u%s.%s",
                 photo_dir, y, m, d, (unsigned int)e->num, (unsigned int)pb_jsr_count(e->variant),
                 (unsigned int)e->num, fsuf, ext);
        return;
    }

    const char *suf = pb_variant_suffix(e->variant);
    if (e->date_packed == 0U)
    {
        snprintf(out, out_sz, "%s/CDI%05u%s.%s", photo_dir, (unsigned int)e->num, suf, ext);
    }
    else
    {
        unsigned int y = 1980U + (unsigned int)(e->date_packed >> 9);
        unsigned int m = (unsigned int)((e->date_packed >> 5) & 0x0FU);
        unsigned int d = (unsigned int)(e->date_packed & 0x1FU);
        snprintf(out, out_sz, "%s/%04u%02u%02u/CDI%05u%s.%s",
                 photo_dir, y, m, d, (unsigned int)e->num, suf, ext);
    }
}

static inline uint8_t pb_entry_lt(const pb_entry_t *a, const pb_entry_t *b)
{
    if (a->date_packed != b->date_packed) return (a->date_packed < b->date_packed) ? 1U : 0U;
    if (a->num != b->num) return (a->num < b->num) ? 1U : 0U;
    return (a->variant < b->variant) ? 1U : 0U;
}

static void pb_add_scanned_file(pb_entry_t *entries, uint16_t *count_ptr,
                                uint16_t max_entries, uint16_t date_packed,
                                uint16_t num, uint8_t flag, uint8_t variant,
                                uint8_t skip_hdr_brk)
{
    uint16_t count = *count_ptr;

    /* When skip_hdr_brk is enabled, avoid letting _0.._3 evict old photos past PB_MAX_PHOTOS */
    if (skip_hdr_brk)
    {
        if (variant >= PB_VAR_HDR_0 && variant <= PB_VAR_HDR_3)
        {
            for (uint16_t i = 0U; i < count; i++)
            {
                if (entries[i].num == num &&
                    entries[i].date_packed == date_packed &&
                    entries[i].variant == PB_VAR_HDR)
                {
                    return; /* Merged _hdr already present; skip bracket frame immediately */
                }
            }
        }
        else if (variant == PB_VAR_HDR)
        {
            uint16_t out = 0U;
            for (uint16_t i = 0U; i < count; i++)
            {
                if (entries[i].num == num &&
                    entries[i].date_packed == date_packed &&
                    entries[i].variant >= PB_VAR_HDR_0 &&
                    entries[i].variant <= PB_VAR_HDR_3)
                {
                    continue; /* Evict already-buffered _0.._3 frames of this HDR set */
                }
                entries[out++] = entries[i];
            }
            count      = out;
            *count_ptr = count;
        }
    }

    /* Merge paired DNG+JPG of the same shot & variant in the same folder */
    for (uint16_t i = 0U; i < count; i++)
    {
        if (entries[i].num == num &&
            entries[i].date_packed == date_packed &&
            entries[i].variant == variant)
        {
            entries[i].flags |= flag;
            return;
        }
    }

    pb_entry_t cand = {
        .num         = num,
        .date_packed = date_packed,
        .flags       = flag,
        .variant     = variant,
    };

    if (count < max_entries)
    {
        entries[count] = cand;
        *count_ptr = (uint16_t)(count + 1U);
    }
    else
    {
        /* Keep the newest max_entries photos by evicting the oldest entry */
        uint16_t min_i = 0U;
        for (uint16_t i = 1U; i < count; i++)
        {
            if (pb_entry_lt(&entries[i], &entries[min_i])) min_i = i;
        }
        if (pb_entry_lt(&entries[min_i], &cand))
        {
            entries[min_i] = cand;
        }
    }
}

static void pb_scan_day_dir(const char *dir_path, uint16_t date_packed,
                            pb_entry_t *entries, uint16_t *count_ptr,
                            uint16_t max_entries, uint8_t skip_hdr_brk,
                            FILINFO *fno)
{
    char sub_path[32];
    pb_build_day_dir(dir_path, date_packed, sub_path, sizeof(sub_path));

    DIR sub_dir;
    if (f_opendir(&sub_dir, sub_path) != FR_OK) return;

    for (;;)
    {
        if (f_readdir(&sub_dir, fno) != FR_OK || fno->fname[0] == '\0')
        {
            break;
        }
        if (fno->fattrib & AM_DIR)
        {
            uint16_t jnum = 0U;
            uint8_t  jvar = 0U;
            if (parse_jsr_dir(fno->fname, &jnum, &jvar))
            {
                /* Single-pass directory scan inside CDIxxxxx_{7,14,30}/ */
                char jsr_path[56];
                uint8_t jn = pb_jsr_count(jvar);
                snprintf(jsr_path, sizeof(jsr_path), "%s/CDI%05u_%u",
                         sub_path, (unsigned int)jnum, (unsigned int)jn);
                DIR jsr_dir;
                if (f_opendir(&jsr_dir, jsr_path) == FR_OK)
                {
                    uint8_t found_hdr  = 0U;
                    uint8_t found_norm = 0U;
                    uint8_t found_brk0 = 0U;
                    for (;;)
                    {
                        if (f_readdir(&jsr_dir, fno) != FR_OK || fno->fname[0] == '\0')
                        {
                            break;
                        }
                        if (fno->fattrib & AM_DIR) continue;
                        uint16_t fnum = 0U;
                        uint8_t  fflag = 0U;
                        uint8_t  fvar = PB_VAR_NORMAL;
                        if (!parse_cdi_filename(fno->fname, &fnum, &fflag, &fvar)) continue;
                        if (fnum != jnum || (fflag & PB_FLAG_DNG) == 0U) continue;
                        if (fvar == PB_VAR_HDR)
                        {
                            found_hdr = 1U;
                            break;
                        }
                        if (fvar == PB_VAR_NORMAL) found_norm = 1U;
                        else if (fvar == PB_VAR_HDR_0) found_brk0 = 1U;
                    }
                    f_closedir(&jsr_dir);
                    if (found_hdr)
                    {
                        pb_add_scanned_file(entries, count_ptr, max_entries, date_packed,
                                            jnum, (uint8_t)(PB_FLAG_DNG | PB_FLAG_JSR_HDR), jvar, skip_hdr_brk);
                    }
                    else if (found_norm)
                    {
                        pb_add_scanned_file(entries, count_ptr, max_entries, date_packed,
                                            jnum, PB_FLAG_DNG, jvar, skip_hdr_brk);
                    }
                    else if (found_brk0)
                    {
                        pb_add_scanned_file(entries, count_ptr, max_entries, date_packed,
                                            jnum, (uint8_t)(PB_FLAG_DNG | PB_FLAG_JSR_BRK0), jvar, skip_hdr_brk);
                    }
                }
            }
            continue;
        }

        uint16_t num = 0U;
        uint8_t  flag = 0U;
        uint8_t  variant = PB_VAR_NORMAL;
        if (!parse_cdi_filename(fno->fname, &num, &flag, &variant)) continue;

        pb_add_scanned_file(entries, count_ptr, max_entries, date_packed, num, flag, variant, skip_hdr_brk);
    }
    f_closedir(&sub_dir);
}

static uint16_t pb_scan_photos(const char *dir_path, pb_entry_t *entries,
                               uint16_t max_entries, uint8_t skip_hdr_brk,
                               uint8_t *work_buf)
{
    DIR root_dir;
    FILINFO *fno = (FILINFO *)work_buf; /* Reuses idle 4 KB work_buf (-280 B stack) */
    uint16_t *day_dirs = (uint16_t *)(work_buf + 512U); /* Up to 1,024 YYYYMMDD dirs (2 KB) */
    uint16_t ndays = 0U;
    uint16_t count = 0U;

    if (f_opendir(&root_dir, dir_path) != FR_OK)
    {
        return 0U;
    }

    /* Pass 1: Collect YYYYMMDD subdirectories & any legacy root files */
    for (;;)
    {
        if (f_readdir(&root_dir, fno) != FR_OK || fno->fname[0] == '\0')
        {
            break;
        }

        if (fno->fattrib & AM_DIR)
        {
            uint16_t date_packed = 0U;
            if (!parse_yyyymmdd_dir(fno->fname, &date_packed)) continue;
            if (ndays < 1024U)
            {
                day_dirs[ndays++] = date_packed;
            }
            else
            {
                pb_scan_day_dir(dir_path, date_packed, entries, &count, max_entries, skip_hdr_brk, fno);
            }
            continue;
        }

        /* Backward-compatible: also scan legacy photos directly under CAYE_CAM/ */
        uint16_t num = 0U;
        uint8_t  flag = 0U;
        uint8_t  variant = PB_VAR_NORMAL;
        if (!parse_cdi_filename(fno->fname, &num, &flag, &variant)) continue;

        pb_add_scanned_file(entries, &count, max_entries, 0U, num, flag, variant, skip_hdr_brk);
    }
    f_closedir(&root_dir);

    /* Sort date directories DESCENDING (newest day first) so full cards prune old days immediately */
    for (uint16_t gap = ndays / 2U; gap > 0U; gap /= 2U)
    {
        for (uint16_t i = gap; i < ndays; i++)
        {
            uint16_t tmp = day_dirs[i];
            uint16_t j = i;
            while (j >= gap && day_dirs[j - gap] < tmp)
            {
                day_dirs[j] = day_dirs[j - gap];
                j = (uint16_t)(j - gap);
            }
            day_dirs[j] = tmp;
        }
    }

    /* Pass 2: Scan newest-to-oldest; once full with newer dates, prune all strictly older day dirs */
    for (uint16_t d = 0U; d < ndays; d++)
    {
        if (count >= max_entries)
        {
            uint16_t min_date = entries[0].date_packed;
            for (uint16_t i = 1U; i < count; i++)
            {
                if (entries[i].date_packed < min_date) min_date = entries[i].date_packed;
            }
            if (day_dirs[d] < min_date)
            {
                break; /* All remaining day dirs are strictly older than every buffered photo */
            }
        }
        pb_scan_day_dir(dir_path, day_dirs[d], entries, &count, max_entries, skip_hdr_brk, fno);
    }

    /* Shell sort ascending by (date_packed, num, variant) so index (count - 1) is always newest */
    for (uint16_t gap = count / 2U; gap > 0U; gap /= 2U)
    {
        for (uint16_t i = gap; i < count; i++)
        {
            pb_entry_t tmp = entries[i];
            uint16_t j = i;
            while (j >= gap && pb_entry_lt(&tmp, &entries[j - gap]))
            {
                entries[j] = entries[j - gap];
                j = (uint16_t)(j - gap);
            }
            entries[j] = tmp;
        }
    }

    /* Optional: for any HDR group that has a merged _hdr file, skip its _0/_1/_2 frames */
    if (skip_hdr_brk && count > 1U)
    {
        uint16_t out = 0U;
        uint16_t i   = 0U;
        while (i < count)
        {
            uint16_t g_num   = entries[i].num;
            uint16_t g_date  = entries[i].date_packed;
            uint16_t j       = i;
            uint8_t  has_hdr = 0U;

            while (j < count && entries[j].num == g_num && entries[j].date_packed == g_date)
            {
                if (entries[j].variant == PB_VAR_HDR)
                {
                    has_hdr = 1U;
                }
                j++;
            }

            for (uint16_t k = i; k < j; k++)
            {
                uint8_t is_brk = (entries[k].variant >= PB_VAR_HDR_0 &&
                                  entries[k].variant <= PB_VAR_HDR_3) ? 1U : 0U;
                if (has_hdr && is_brk)
                {
                    continue;
                }
                entries[out++] = entries[k];
            }
            i = j;
        }
        count = out;
    }

    return count;
}

/* ======================== DNG Parser & Renderers ======================== */
static uint32_t tiff_read_val_u32(const uint8_t *hdr, uint32_t hdr_len, uint16_t type, uint32_t raw_val_field)
{
    (void)hdr;
    (void)hdr_len;
    if (type == 3U) return raw_val_field & 0xFFFFUL;
    return raw_val_field;
}

static inline uint8_t tiff_range_ok(uint32_t off, uint32_t need, uint32_t limit)
{
    return (off <= limit && need <= (limit - off)) ? 1U : 0U;
}

static void pb_copy_exif_datetime(const uint8_t *src, char out[20])
{
    static const uint8_t digit_pos[14] = { 0, 1, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17, 18 };
    for (uint8_t i = 0U; i < 14U; i++)
    {
        uint8_t c = src[digit_pos[i]];
        if (c < '0' || c > '9') return;
    }
    if ((src[4] != ':' && src[4] != '-') || (src[7] != ':' && src[7] != '-') ||
        src[10] != ' ' || src[13] != ':' || src[16] != ':')
    {
        return;
    }
    memcpy(out, src, 19U);
    out[4]  = '-';
    out[7]  = '-';
    out[19] = '\0';
}

static uint16_t exif_rd16(const uint8_t *b, uint32_t off, uint8_t big_endian)
{
    if (big_endian) return (uint16_t)(((uint16_t)b[off] << 8) | b[off + 1U]);
    return rd16(b, off);
}

static uint32_t exif_rd32(const uint8_t *b, uint32_t off, uint8_t big_endian)
{
    if (big_endian)
    {
        return ((uint32_t)b[off] << 24) | ((uint32_t)b[off + 1U] << 16) |
               ((uint32_t)b[off + 2U] << 8) | (uint32_t)b[off + 3U];
    }
    return rd32(b, off);
}

static void pb_parse_exif_ifd(const uint8_t *tb, uint32_t tiff_len, uint32_t ifd_off, uint8_t be,
                              char datetime[20],
                              uint32_t *exp_num, uint32_t *exp_den, uint8_t *has_exp,
                              uint16_t *iso, uint8_t *has_iso,
                              int16_t *ev_x10, uint8_t *has_ev,
                              uint16_t *fnum_x10, uint8_t *has_fnum,
                              uint16_t *focal_x10, uint16_t *focal_eq35)
{
    if (!tiff_range_ok(ifd_off, 2U, tiff_len)) return;
    uint16_t cnt = exif_rd16(tb, ifd_off, be);
    uint32_t ep  = ifd_off + 2U;

    for (uint16_t i = 0U; i < cnt; i++, ep += 12U)
    {
        if (!tiff_range_ok(ep, 12U, tiff_len)) break;
        uint16_t tag   = exif_rd16(tb, ep, be);
        uint16_t type  = exif_rd16(tb, ep + 2U, be);
        uint32_t count = exif_rd32(tb, ep + 4U, be);
        uint32_t val   = exif_rd32(tb, ep + 8U, be);

        switch (tag)
        {
            case 0x829AU: /* ExposureTime (RATIONAL) */
                if (type == 5U && count >= 1U && tiff_range_ok(val, 8U, tiff_len))
                {
                    uint32_t n = exif_rd32(tb, val, be);
                    uint32_t d = exif_rd32(tb, val + 4U, be);
                    if (n > 0UL && d > 0UL)
                    {
                        *exp_num = n;
                        *exp_den = d;
                        *has_exp = 1U;
                    }
                }
                break;

            case 0x829DU: /* FNumber (RATIONAL, including 0/1 = F0) */
                if (fnum_x10 != NULL && has_fnum != NULL && type == 5U && count >= 1U && tiff_range_ok(val, 8U, tiff_len))
                {
                    uint32_t n = exif_rd32(tb, val, be);
                    uint32_t d = exif_rd32(tb, val + 4U, be);
                    if (d > 0UL)
                    {
                        uint64_t f10 = ((uint64_t)n * 10ULL + ((uint64_t)d / 2ULL)) / (uint64_t)d;
                        if (f10 > 65535ULL) f10 = 65535ULL;
                        *fnum_x10 = (uint16_t)f10;
                        *has_fnum = 1U;
                    }
                }
                break;

            case 0x8827U: /* ISOSpeedRatings (SHORT) */
                if (type == 3U && count >= 1U)
                {
                    uint16_t v = 0U;
                    if (count <= 2U)
                    {
                        v = exif_rd16(tb, ep + 8U, be);
                    }
                    else if (tiff_range_ok(val, 2U, tiff_len))
                    {
                        v = exif_rd16(tb, val, be);
                    }
                    if (v > 0U)
                    {
                        *iso     = v;
                        *has_iso = 1U;
                    }
                }
                break;

            case 0x9003U: /* DateTimeOriginal (fallback if IFD0 DateTime missing) */
                if (datetime != NULL && datetime[0] == '\0' &&
                    type == 2U && count >= 19U && tiff_range_ok(val, 19U, tiff_len))
                {
                    pb_copy_exif_datetime(&tb[val], datetime);
                }
                break;

            case 0x9204U: /* ExposureBiasValue (SRATIONAL) */
                if (type == 10U && count >= 1U && tiff_range_ok(val, 8U, tiff_len))
                {
                    int64_t n = (int64_t)(int32_t)exif_rd32(tb, val, be);
                    int64_t d = (int64_t)(int32_t)exif_rd32(tb, val + 4U, be);
                    if (d != 0)
                    {
                        if (d < 0) { n = -n; d = -d; }
                        int64_t num10 = n * 10;
                        int64_t ev10  = (num10 >= 0) ? ((num10 + (d / 2)) / d) : ((num10 - (d / 2)) / d);
                        if (ev10 < -99) ev10 = -99;
                        if (ev10 >  99) ev10 =  99;
                        *ev_x10 = (int16_t)ev10;
                        *has_ev = 1U;
                    }
                }
                break;

            case 0x920AU: /* FocalLength (RATIONAL) */
                if (focal_x10 != NULL && type == 5U && count >= 1U && tiff_range_ok(val, 8U, tiff_len))
                {
                    uint32_t n = exif_rd32(tb, val, be);
                    uint32_t d = exif_rd32(tb, val + 4U, be);
                    if (n > 0UL && d > 0UL)
                    {
                        uint64_t f10 = ((uint64_t)n * 10ULL + ((uint64_t)d / 2ULL)) / (uint64_t)d;
                        if (f10 < 1ULL) f10 = 1ULL;
                        if (f10 > 65535ULL) f10 = 65535ULL;
                        *focal_x10 = (uint16_t)f10;
                    }
                }
                break;

            case 0xA405U: /* FocalLengthIn35mmFilm (SHORT) */
                if (focal_eq35 != NULL && type == 3U && count >= 1U)
                {
                    uint16_t v = 0U;
                    if (count <= 2U)
                    {
                        v = exif_rd16(tb, ep + 8U, be);
                    }
                    else if (tiff_range_ok(val, 2U, tiff_len))
                    {
                        v = exif_rd16(tb, val, be);
                    }
                    if (v > 0U)
                    {
                        *focal_eq35 = v;
                    }
                }
                break;

            default:
                break;
        }
    }
}

/* Pure-integer IEEE-754 big-endian binary64 -> signed Q16 fixed-point (zero soft-float) */
static int32_t pb_f64_be_to_q16(const uint8_t *p)
{
    uint32_t hi      = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
    uint32_t lo      = ((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) | ((uint32_t)p[6] << 8) | (uint32_t)p[7];
    uint32_t raw_exp = (hi >> 20) & 0x7FFUL;
    if (raw_exp == 0x7FFUL) return 0L; /* Reject IEEE-754 Inf / NaN */
    int32_t  sign    = (hi & 0x80000000UL) ? -1L : 1L;
    int32_t  exp     = (int32_t)raw_exp - 1023L;
    if (exp < -17L) return 0L;
    if (exp > 9L)   return (sign < 0L) ? -(1L << 26) : (1L << 26); /* Monotonic clamp at +-1024.0 */
    uint64_t mant  = (1ULL << 52) | ((uint64_t)(hi & 0xFFFFFUL) << 32) | (uint64_t)lo;
    uint32_t shift = (uint32_t)(36L - exp);
    uint32_t mag   = (uint32_t)((mant + (1ULL << (shift - 1U))) >> shift);
    return (sign < 0L) ? -(int32_t)mag : (int32_t)mag;
}

/* Parse DNG 1.4 OpcodeList3 (0xC74E, big-endian): FixVignetteRadial (ID=3) & WarpRectilinear (ID=1) */
static void dng_parse_opcode_list3(const uint8_t *op, uint32_t len, dng_info_t *info)
{
    if (len < 4UL) return;
    uint32_t cnt = exif_rd32(op, 0U, 1U);
    if (cnt > 8UL) cnt = 8UL;
    uint32_t pos = 4UL;
    for (uint32_t i = 0UL; i < cnt; i++)
    {
        if (!tiff_range_ok(pos, 16U, len)) break;
        uint32_t id   = exif_rd32(op, pos + 0U,  1U);
        uint32_t plen = exif_rd32(op, pos + 12U, 1U);
        pos += 16UL;
        if (!tiff_range_ok(pos, plen, len)) break;

        if (id == 3UL && plen >= 56UL)
        {
            /* FixVignetteRadial: k0 <= 4.0 (covers C3M1 vignette=0..50 max k0=2.50 with zero uint32 overflow) */
            int32_t k0_q16 = pb_f64_be_to_q16(&op[pos]);
            if (k0_q16 > 0L && k0_q16 <= (4L << 16))
            {
                info->vig_scale_q16 = ((uint32_t)k0_q16 * 2UL + 625UL) / 1250UL;
            }
        }
        else if (id == 1UL && plen >= 68UL)
        {
            /* WarpRectilinear: uint32 planes, then kr0, kr1 (|kr1| <= 0.2014 covers C3M1 distort=-100..+100) */
            uint32_t planes = exif_rd32(op, pos, 1U);
            if (planes >= 1UL)
            {
                int32_t kr0 = pb_f64_be_to_q16(&op[pos + 4U]);
                int32_t kr1 = pb_f64_be_to_q16(&op[pos + 12U]);
                if (kr0 >= 32768L && kr0 <= 98304L && kr1 != 0L && kr1 >= -13200L && kr1 <= 13200L)
                {
                    info->warp_kr0_q16 = kr0;
                    info->warp_kr1_q16 = kr1;
                    info->has_warp     = 1U;
                }
            }
        }
        pos += plen;
    }
}

static void dng_parse_ifd(const uint8_t *hdr, uint32_t hdr_len, uint32_t ifd_off,
                          uint8_t is_subifd, dng_info_t *info, uint16_t *lin_tbl,
                          uint32_t *out_subifd_off, uint32_t *out_exififd_off)
{
    if (!tiff_range_ok(ifd_off, 2U, hdr_len)) return;
    uint16_t cnt = rd16(hdr, ifd_off);
    uint32_t pos = ifd_off + 2U;

    uint16_t w = 0U, h = 0U, bps = 0U, photo = 0U, spp = 1U;
    uint32_t strip_off = 0U;

    for (uint16_t i = 0U; i < cnt; i++, pos += 12U)
    {
        if (!tiff_range_ok(pos, 12U, hdr_len)) break;
        uint16_t tag   = rd16(hdr, pos);
        uint16_t type  = rd16(hdr, pos + 2U);
        uint32_t count = rd32(hdr, pos + 4U);
        uint32_t val   = rd32(hdr, pos + 8U);

        switch (tag)
        {
            case 0x0100U: w = (uint16_t)tiff_read_val_u32(hdr, hdr_len, type, val); break;
            case 0x0101U: h = (uint16_t)tiff_read_val_u32(hdr, hdr_len, type, val); break;
            case 0x0102U:
                if (count == 1U) bps = (uint16_t)tiff_read_val_u32(hdr, hdr_len, type, val);
                else if (tiff_range_ok(val, 2U, hdr_len)) bps = rd16(hdr, val);
                break;
            case 0x0106U: photo = (uint16_t)tiff_read_val_u32(hdr, hdr_len, type, val); break;
            case 0x010EU: /* ImageDescription ("C3M1 B&W", "C3M1 B&W HI", "C3M1 B&W RED") */
                if (!is_subifd && type == 2U && count >= 8U && tiff_range_ok(val, count, hdr_len))
                {
                    const char *s = (const char *)&hdr[val];
                    if (count >= 12U && memcmp(s, "C3M1 B&W RED", 12U) == 0)      info->color_mode = 3U;
                    else if (count >= 11U && memcmp(s, "C3M1 B&W HI", 11U) == 0)  info->color_mode = 2U;
                    else if (memcmp(s, "C3M1 B&W", 8U) == 0)                      info->color_mode = 1U;
                }
                break;
            case 0x0111U: strip_off = tiff_read_val_u32(hdr, hdr_len, type, val); break;
            case 0x0115U: spp = (uint16_t)tiff_read_val_u32(hdr, hdr_len, type, val); break;
            case 0x0132U: /* DateTime ("YYYY:MM:DD HH:MM:SS") */
                if (type == 2U && count >= 19U && tiff_range_ok(val, 19U, hdr_len))
                {
                    pb_copy_exif_datetime(&hdr[val], info->datetime);
                }
                break;
            case 0x014AU:
                if (!is_subifd && out_subifd_off != NULL) *out_subifd_off = val;
                break;
            case 0x8769U: /* ExifIFD */
                if (!is_subifd && out_exififd_off != NULL) *out_exififd_off = val;
                break;
            case 0xC618U: /* LinearizationTable */
                if (type == 3U && count == 256U && tiff_range_ok(val, 512U, hdr_len) && lin_tbl != NULL)
                {
                    for (uint16_t k = 0U; k < 256U; k++)
                    {
                        lin_tbl[k] = rd16(hdr, val + (uint32_t)k * 2U);
                    }
                    info->has_lin_tbl = 1U;
                }
                break;
            case 0xC61AU: /* BlackLevel */
                if (type == 3U && count == 4U && tiff_range_ok(val, 8U, hdr_len))
                {
                    info->bl4[0] = rd16(hdr, val + 0U);
                    info->bl4[1] = rd16(hdr, val + 2U);
                    info->bl4[2] = rd16(hdr, val + 4U);
                    info->bl4[3] = rd16(hdr, val + 6U);
                }
                else if (type == 3U && count == 1U)
                {
                    uint16_t bl = (uint16_t)(val & 0xFFFFU);
                    info->bl4[0] = info->bl4[1] = info->bl4[2] = info->bl4[3] = bl;
                }
                else if (type == 4U && count == 1U)
                {
                    uint16_t bl = (uint16_t)val;
                    info->bl4[0] = info->bl4[1] = info->bl4[2] = info->bl4[3] = bl;
                }
                else if (type == 5U && count >= 1U && tiff_range_ok(val, 8U, hdr_len))
                {
                    uint32_t num = rd32(hdr, val);
                    uint32_t den = rd32(hdr, val + 4U);
                    uint16_t bl  = (den > 0U) ? (uint16_t)(num / den) : 0U;
                    info->bl4[0] = info->bl4[1] = info->bl4[2] = info->bl4[3] = bl;
                }
                break;
            case 0xC61DU: /* WhiteLevel */
                info->white_level = (uint16_t)tiff_read_val_u32(hdr, hdr_len, type, val);
                break;
            case 0xC628U: /* AsShotNeutral (3 RATIONALs) */
                if (type == 5U && count == 3U && tiff_range_ok(val, 24U, hdr_len))
                {
                    uint32_t nr = rd32(hdr, val + 0U),  dr = rd32(hdr, val + 4U);
                    uint32_t ng = rd32(hdr, val + 8U),  dg = rd32(hdr, val + 12U);
                    uint32_t nb = rd32(hdr, val + 16U), db = rd32(hdr, val + 20U);
                    if (nr > 0U && dr > 0U && ng > 0U && dg > 0U && nb > 0U && db > 0U)
                    {
                        /* wb_r_q8 = 256 * (ng/dg) / (nr/dr) */
                        uint32_t wbr = (uint32_t)(((uint64_t)256ULL * ng * dr) / ((uint64_t)dg * nr));
                        uint32_t wbb = (uint32_t)(((uint64_t)256ULL * ng * db) / ((uint64_t)dg * nb));
                        if (wbr < 64U) wbr = 64U;
                        if (wbr > 1024U) wbr = 1024U;
                        if (wbb < 64U) wbb = 64U;
                        if (wbb > 1536U) wbb = 1536U;
                        info->wb_r_q8 = (uint16_t)wbr;
                        info->wb_b_q8 = (uint16_t)wbb;
                        info->has_wb  = 1U;
                    }
                }
                break;
            case 0xC62AU: /* BaselineExposure (1 SRATIONAL) */
                if (type == 10U && count == 1U && tiff_range_ok(val, 8U, hdr_len))
                {
                    int32_t num = (int32_t)rd32(hdr, val + 0U);
                    int32_t den = (int32_t)rd32(hdr, val + 4U);
                    if (den > 0)
                    {
                        int64_t ev = ((int64_t)num * 100) / den;
                        if (ev < -700) ev = -700;
                        if (ev >  700) ev =  700;
                        info->baseline_ev_x100 = (int16_t)ev;
                        info->has_bev          = 1U;
                    }
                }
                break;
            case 0xC74EU: /* OpcodeList3 (Lens Vignette + WarpRectilinear) */
                if (type == 7U && count >= 4U && tiff_range_ok(val, count, hdr_len))
                {
                    dng_parse_opcode_list3(&hdr[val], count, info);
                }
                break;
            default:
                break;
        }
    }

    if (!is_subifd && photo == 2U && spp == 3U &&
        w == C3M1_DNG_THUMB_W && h == C3M1_DNG_THUMB_H && strip_off > 0U)
    {
        info->has_thumb = 1U;
        info->thumb_off = strip_off;
    }
    if (photo == 32803U && w >= 2U && h >= 2U && (bps == 8U || bps == 16U) && strip_off > 0U)
    {
        info->raw_w   = w;
        info->raw_h   = h;
        info->raw_bps = bps;
        info->raw_off = strip_off;
        info->valid   = 1U;
    }
}

static uint8_t dng_parse_header(FIL *fp, uint8_t *hdr_buf, uint16_t hdr_buf_sz,
                                dng_info_t *info, uint16_t *lin_tbl)
{
    memset(info, 0, sizeof(*info));
    info->white_level = 4095U;
    info->bl4[0]      = 24U;
    info->bl4[1]      = 24U;
    info->bl4[2]      = 24U;
    info->bl4[3]      = 24U;
    info->wb_r_q8     = 256U;
    info->wb_b_q8     = 406U; /* Default OV7725 D65 */

    uint32_t read_len = (hdr_buf_sz >= 2048U) ? 2048U : hdr_buf_sz;
    UINT br = 0U;
    if (f_lseek(fp, 0U) != FR_OK || f_read(fp, hdr_buf, read_len, &br) != FR_OK || br < 64U)
    {
        return 0U;
    }
    if (hdr_buf[0] != 'I' || hdr_buf[1] != 'I' || rd16(hdr_buf, 2U) != 42U)
    {
        return 0U;
    }

    uint32_t ifd0_off    = rd32(hdr_buf, 4U);
    uint32_t subifd_off  = 0U;
    uint32_t exififd_off = 0U;
    dng_parse_ifd(hdr_buf, br, ifd0_off, 0U, info, lin_tbl, &subifd_off, &exififd_off);
    if (subifd_off > 0U)
    {
        dng_parse_ifd(hdr_buf, br, subifd_off, 1U, info, lin_tbl, NULL, NULL);
    }
    if (exififd_off > 0U)
    {
        pb_parse_exif_ifd(hdr_buf, br, exififd_off, 0U, info->datetime,
                          &info->exp_num, &info->exp_den, &info->has_exp,
                          &info->iso, &info->has_iso,
                          &info->ev_bias_x10, &info->has_ev,
                          &info->fnum_x10, &info->has_fnum,
                          &info->focal_x10, &info->focal_eq35);
    }

    if (!info->has_lin_tbl && lin_tbl != NULL)
    {
        for (uint16_t i = 0U; i < 256U; i++)
        {
            lin_tbl[i] = (uint16_t)(i << 4);
        }
    }
    if (info->white_level == 0U) info->white_level = 4095U;
    return (info->valid || info->has_thumb) ? 1U : 0U;
}

static inline uint16_t dng_sample_12(const uint8_t *row, uint16_t x, uint16_t bps,
                                     const uint16_t *lin_tbl, uint16_t white_level)
{
    if (bps == 8U)
    {
        return lin_tbl[row[x]];
    }
    uint32_t v = (uint32_t)rd16(row, (uint32_t)x * 2U);
    if (white_level != 4095U && white_level > 0U)
    {
        v = (v * 4095UL) / white_level;
    }
    return (v > 4095UL) ? 4095U : (uint16_t)v;
}

static uint32_t pb_ev_x100_to_gain_q8(int16_t ev_x100)
{
    if (ev_x100 <= 0) return 256UL;
    uint32_t whole = (uint32_t)ev_x100 / 100UL;
    uint32_t frac  = (uint32_t)ev_x100 % 100UL;
    if (whole > 6UL) return 16384UL;
    /* 2^(frac/100) approx: 1 + 0.693*f + 0.307*f^2 (max err <0.5%) */
    uint32_t f_q8 = 256UL + ((frac * 177UL + 50UL) / 100UL) + ((frac * frac * 79UL) / 10000UL);
    return f_q8 << whole;
}

static inline uint16_t sub_bl12(uint32_t v, uint16_t bl)
{
    return (v > bl) ? (uint16_t)(v - bl) : 0U;
}

static inline void pb_dma_wait(void)
{
    if (DMA1_Channel1->CCR & DMA_CCR_EN)
    {
        while ((DMA1->ISR & DMA_ISR_TCIF1) == 0U)
        {
        }
        DMA1_Channel1->CCR = 0U;
    }
}

static inline void pb_dma_send(const uint8_t *src, uint16_t len)
{
    pb_dma_wait();
    DMA1->IFCR           = DMA_IFCR_CGIF1;
    DMA1_Channel1->CPAR  = (uint32_t)&LCD->LCD_RAM;
    DMA1_Channel1->CMAR  = (uint32_t)src;
    DMA1_Channel1->CNDTR = len;
    DMA1_Channel1->CCR   = DMA_CCR_MEM2MEM | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_EN;
}

static const uint8_t s_dither2x2[2][2] = { { 0U, 2U }, { 3U, 1U } };

static inline __attribute__((always_inline)) void pb_put_rgb666_dither(uint8_t *dst, uint8_t r8, uint8_t g8, uint8_t b8, uint16_t x, uint16_t y)
{
    uint8_t d = s_dither2x2[y & 1U][x & 1U];
    uint8_t r = (r8 < (uint8_t)(255U - d)) ? (uint8_t)(r8 + d) : 255U;
    uint8_t g = (g8 < (uint8_t)(255U - d)) ? (uint8_t)(g8 + d) : 255U;
    uint8_t b = (b8 < (uint8_t)(255U - d)) ? (uint8_t)(b8 + d) : 255U;
    dst[0] = (uint8_t)(r & 0xFCU);
    dst[1] = (uint8_t)(g & 0xFCU);
    dst[2] = (uint8_t)(b & 0xFCU);
}

static inline __attribute__((always_inline)) void pb_put_rgb666_1x_clip(uint8_t *dst, uint8_t r8, uint8_t g8, uint8_t b8, uint16_t x, uint16_t y)
{
    if (((x + y) & 3U) == 0U)
    {
        if (r8 >= 250U && g8 >= 250U && b8 >= 250U)
        {
            dst[0] = 252U;
            dst[1] = 32U;
            dst[2] = 32U;
            return;
        }
        if (r8 <= 4U && g8 <= 4U && b8 <= 4U)
        {
            dst[0] = 32U;
            dst[1] = 112U;
            dst[2] = 252U;
            return;
        }
    }
    pb_put_rgb666_dither(dst, r8, g8, b8, x, y);
}

static inline void pb_wr_rgb666_dither(uint8_t r8, uint8_t g8, uint8_t b8, uint16_t x, uint16_t y)
{
    uint8_t d = s_dither2x2[y & 1U][x & 1U];
    uint8_t r = (r8 < (uint8_t)(255U - d)) ? (uint8_t)(r8 + d) : 255U;
    uint8_t g = (g8 < (uint8_t)(255U - d)) ? (uint8_t)(g8 + d) : 255U;
    uint8_t b = (b8 < (uint8_t)(255U - d)) ? (uint8_t)(b8 + d) : 255U;
    lcd_wr_rgb666(r, g, b);
}

/* 1X playback clipping overlay: diagonal red hatch on blown highlights, blue hatch on crushed shadows */
static inline void pb_wr_rgb666_1x_clip(uint8_t r8, uint8_t g8, uint8_t b8, uint16_t x, uint16_t y)
{
    if (((x + y) & 3U) == 0U)
    {
        if (r8 >= 250U && g8 >= 250U && b8 >= 250U)
        {
            lcd_wr_rgb666(252U, 32U, 32U);
            return;
        }
        if (r8 <= 4U && g8 <= 4U && b8 <= 4U)
        {
            lcd_wr_rgb666(32U, 112U, 252U);
            return;
        }
    }
    pb_wr_rgb666_dither(r8, g8, b8, x, y);
}

static inline __attribute__((always_inline)) uint16_t pb_apply_rgb565_1x_clip(uint16_t px, uint16_t x, uint16_t y)
{
    if (((x + y) & 3U) == 0U)
    {
        uint8_t r5 = (uint8_t)(px >> 11);
        uint8_t g6 = (uint8_t)((px >> 5) & 0x3FU);
        uint8_t b5 = (uint8_t)(px & 0x1FU);
        if (r5 >= 30U && g6 >= 62U && b5 >= 30U)
        {
            return 0xF904U; /* Bright red highlight clipping marker */
        }
        if (r5 <= 1U && g6 <= 1U && b5 <= 1U)
        {
            return 0x239FU; /* Bright blue shadow clipping marker */
        }
    }
    return px;
}

/* Inline 50% translucent bottom HUD palette & compositors (matches live viewfinder, 0 B .bss) */
#define PB_PAL_BG           0U
#define PB_PAL_WHITE        1U
#define PB_PAL_YELLOW       2U
#define PB_PAL_CYAN         3U
#define PB_PAL_GREEN        4U
#define PB_PAL_ORANGE       5U
#define PB_PAL_MAGENTA      6U
#define PB_PAL_HINT         7U

static const uint16_t s_pb_pal[8] = {
    BLACK, WHITE, YELLOW, CYAN, GREEN, ORANGE, MAGENTA, 0xD69AU
};

static const uint8_t s_pb_pal666[8][3] = {
    { 0x00U, 0x00U, 0x00U },
    { 0xFCU, 0xFCU, 0xFCU },
    { 0xFCU, 0xFCU, 0x00U },
    { 0x00U, 0xFCU, 0xFCU },
    { 0x00U, 0xFCU, 0x00U },
    { 0xFCU, 0xA4U, 0x00U },
    { 0xFCU, 0x00U, 0xFCU },
    { 0xD4U, 0xD4U, 0xD4U },
};

static void pb_bar_put_str(uint8_t bar_buf[12][160], uint16_t x, uint16_t max_w,
                           const char *s, uint8_t col_idx)
{
    uint16_t xmax = (uint16_t)(x + max_w);
    if (xmax > PB_SCR_W) xmax = PB_SCR_W;
    while (*s != '\0' && (uint16_t)(x + 6U) <= xmax)
    {
        const uint8_t *glyph = lcd_get_font12(*s++);
        for (uint8_t cx = 0U; cx < 6U; cx++)
        {
            uint16_t px = (uint16_t)(x + cx);
            uint8_t  b0 = glyph[cx * 2U];
            uint8_t  b1 = glyph[cx * 2U + 1U];
            uint16_t bi = (uint16_t)(px >> 1);
            uint8_t  hi = (uint8_t)(px & 1U);
            for (uint8_t r = 0U; r < 8U; r++)
            {
                if (b0 & (0x80U >> r))
                {
                    uint8_t *cell = &bar_buf[r][bi];
                    if (hi) *cell = (uint8_t)((*cell & 0x0FU) | (col_idx << 4));
                    else    *cell = (uint8_t)((*cell & 0xF0U) | col_idx);
                }
            }
            for (uint8_t r = 0U; r < 4U; r++)
            {
                if (b1 & (0x80U >> r))
                {
                    uint8_t *cell = &bar_buf[8U + r][bi];
                    if (hi) *cell = (uint8_t)((*cell & 0x0FU) | (col_idx << 4));
                    else    *cell = (uint8_t)((*cell & 0xF0U) | col_idx);
                }
            }
        }
        x = (uint16_t)(x + 6U);
    }
}

static void __attribute__((optimize("O3"))) pb_overlay_bar_rgb666(uint8_t *row_rgb666, uint16_t y,
                                                                  const uint8_t (*bar_buf)[160])
{
    if (y < (PB_BAR_Y + 4U) || y >= (PB_BAR_Y + 16U))
    {
        for (uint32_t i = 0U; i < (PB_SCR_W * 3U); i++)
        {
            row_rgb666[i] = (uint8_t)((row_rgb666[i] >> 1) & 0xFCU);
        }
        return;
    }

    const uint8_t *brow = bar_buf[y - (PB_BAR_Y + 4U)];
    uint8_t       *p    = row_rgb666;
    for (uint32_t i = 0U; i < 160U; i++)
    {
        uint8_t b = brow[i];
        if (__builtin_expect(b == 0U, 1))
        {
            p[0] = (uint8_t)((p[0] >> 1) & 0xFCU);
            p[1] = (uint8_t)((p[1] >> 1) & 0xFCU);
            p[2] = (uint8_t)((p[2] >> 1) & 0xFCU);
            p[3] = (uint8_t)((p[3] >> 1) & 0xFCU);
            p[4] = (uint8_t)((p[4] >> 1) & 0xFCU);
            p[5] = (uint8_t)((p[5] >> 1) & 0xFCU);
        }
        else
        {
            uint8_t c0 = (uint8_t)(b & 0x0FU);
            uint8_t c1 = (uint8_t)(b >> 4);
            if (c0 != 0U)
            {
                p[0] = s_pb_pal666[c0][0];
                p[1] = s_pb_pal666[c0][1];
                p[2] = s_pb_pal666[c0][2];
            }
            else
            {
                p[0] = (uint8_t)((p[0] >> 1) & 0xFCU);
                p[1] = (uint8_t)((p[1] >> 1) & 0xFCU);
                p[2] = (uint8_t)((p[2] >> 1) & 0xFCU);
            }
            if (c1 != 0U)
            {
                p[3] = s_pb_pal666[c1][0];
                p[4] = s_pb_pal666[c1][1];
                p[5] = s_pb_pal666[c1][2];
            }
            else
            {
                p[3] = (uint8_t)((p[3] >> 1) & 0xFCU);
                p[4] = (uint8_t)((p[4] >> 1) & 0xFCU);
                p[5] = (uint8_t)((p[5] >> 1) & 0xFCU);
            }
        }
        p += 6U;
    }
}

static void __attribute__((optimize("O3"))) pb_overlay_bar_rgb565_be(uint8_t *line, uint16_t y,
                                                                     const uint8_t (*bar_buf)[160])
{
    uint32_t *p32 = (uint32_t *)line;
    if (y < (PB_BAR_Y + 4U) || y >= (PB_BAR_Y + 16U))
    {
        for (uint32_t i = 0U; i < 160U; i++)
        {
            uint32_t w = __REV16(*p32);
            *p32++ = __REV16((w >> 1) & 0x7BEF7BEFUL);
        }
        return;
    }

    const uint8_t *brow = bar_buf[y - (PB_BAR_Y + 4U)];
    for (uint32_t i = 0U; i < 160U; i++)
    {
        uint8_t b = brow[i];
        if (__builtin_expect(b == 0U, 1))
        {
            uint32_t w = __REV16(*p32);
            *p32++ = __REV16((w >> 1) & 0x7BEF7BEFUL);
        }
        else
        {
            uint32_t w  = __REV16(*p32);
            uint8_t  c0 = (uint8_t)(b & 0x0FU);
            uint8_t  c1 = (uint8_t)(b >> 4);
            uint32_t p0 = (c0 != 0U) ? (uint32_t)s_pb_pal[c0] : ((w >> 1) & 0x7BEFUL);
            uint32_t p1 = (c1 != 0U) ? (uint32_t)s_pb_pal[c1] : ((w >> 17) & 0x7BEFUL);
            *p32++ = __REV16(p0 | (p1 << 16));
        }
    }
}

static inline uint16_t pb_apply_bar_rgb565_px(uint16_t px, uint16_t x, uint16_t y,
                                              const uint8_t (*bar_buf)[160])
{
    if (y >= (PB_BAR_Y + 4U) && y < (PB_BAR_Y + 16U))
    {
        uint8_t b = bar_buf[y - (PB_BAR_Y + 4U)][x >> 1];
        uint8_t c = (x & 1U) ? (uint8_t)(b >> 4) : (uint8_t)(b & 0x0FU);
        if (c != 0U) return s_pb_pal[c];
    }
    return (uint16_t)((px >> 1) & 0x7BEFU);
}

static inline void pb_wr_rgb666_bar_px(uint8_t r8, uint8_t g8, uint8_t b8,
                                       uint16_t x, uint16_t y, uint8_t clip_en,
                                       const uint8_t (*bar_buf)[160])
{
    if (y >= (PB_BAR_Y + 4U) && y < (PB_BAR_Y + 16U))
    {
        uint8_t b = bar_buf[y - (PB_BAR_Y + 4U)][x >> 1];
        uint8_t c = (x & 1U) ? (uint8_t)(b >> 4) : (uint8_t)(b & 0x0FU);
        if (c != 0U)
        {
            lcd_wr_rgb666(s_pb_pal666[c][0], s_pb_pal666[c][1], s_pb_pal666[c][2]);
            return;
        }
    }
    uint8_t dst[3];
    if (clip_en) pb_put_rgb666_1x_clip(dst, r8, g8, b8, x, y);
    else         pb_put_rgb666_dither(dst, r8, g8, b8, x, y);
    lcd_wr_rgb666((uint8_t)((dst[0] >> 1) & 0xFCU),
                  (uint8_t)((dst[1] >> 1) & 0xFCU),
                  (uint8_t)((dst[2] >> 1) & 0xFCU));
}

static inline void pb_hist_add_rgb8(pb_hist_t *h, uint8_t r8, uint8_t g8, uint8_t b8)
{
    uint8_t y8 = (uint8_t)(((uint16_t)r8 * 77U + (uint16_t)g8 * 150U + (uint16_t)b8 * 29U + 128U) >> 8);
    h->y[y8 >> 2]++;
    h->r[r8 >> 2]++;
    h->g[g8 >> 2]++;
    h->b[b8 >> 2]++;
    h->total_px++;
    h->sum_y += y8;
    if (r8 >= 250U && g8 >= 250U && b8 >= 250U)
    {
        h->hi_px++;
    }
    else if (r8 <= 4U && g8 <= 4U && b8 <= 4U)
    {
        h->lo_px++;
    }
}

static inline void pb_hist_add_rgb565(pb_hist_t *h, uint16_t px)
{
    uint8_t r5 = (uint8_t)(px >> 11);
    uint8_t g6 = (uint8_t)((px >> 5) & 0x3FU);
    uint8_t b5 = (uint8_t)(px & 0x1FU);
    uint8_t r8 = (uint8_t)((r5 << 3) | (r5 >> 2));
    uint8_t g8 = (uint8_t)((g6 << 2) | (g6 >> 4));
    uint8_t b8 = (uint8_t)((b5 << 3) | (b5 >> 2));
    uint8_t y8 = (uint8_t)(((uint16_t)r8 * 77U + (uint16_t)g8 * 150U + (uint16_t)b8 * 29U + 128U) >> 8);
    h->y[y8 >> 2]++;
    h->r[r8 >> 2]++;
    h->g[g8 >> 2]++;
    h->b[b8 >> 2]++;
    h->total_px++;
    h->sum_y += y8;
    if (r5 >= 30U && g6 >= 62U && b5 >= 30U)
    {
        h->hi_px++;
    }
    else if (r5 <= 1U && g6 <= 1U && b5 <= 1U)
    {
        h->lo_px++;
    }
}

/* ======================== High-ISO DNG Playback NR (Pure-Integer, 0 B .bss) ======================== */
typedef struct
{
    uint16_t lvl;           /* 0 (ISO<=200/OFF) .. 64 (400) .. 128 (800) .. 192 (1600) .. 256 (3200) */
    uint8_t  raw_imp_thr;   /* Stage 1 8-bit Gamma-domain hot/cold pixel threshold (0 = OFF) */
    uint8_t  has_luma_nr;   /* 1 at ISO >= 400: run Stage 2B luma bilateral */
    uint8_t  w_c1[64];      /* Zoom Chroma inner tap (±1) LUT [0..14] for denom 64 (>> 6) */
    uint8_t  w_c2[64];      /* Zoom Chroma outer tap (±2) LUT [0..10] for denom 64 (>> 6) */
    uint8_t  w_y_mid[64];   /* Zoom Luma midtone tap (±1) LUT [0..4] for denom 16 (>> 4) */
    uint8_t  w_y_sh[64];    /* Zoom Luma shadow  tap (±1) LUT [0..4] for denom 16 (>> 4) */
} pb_nr_params_t;

/* Contiguous chroma row (480 B) allows 1 base pointer per row via Thumb-2 LDRSB #0/#160/#320 */
typedef struct
{
    int8_t cb[PB_SCR_W / 2U];
    int8_t cr[PB_SCR_W / 2U];
    int8_t yh[PB_SCR_W / 2U];
} pb_chroma_row_t;

_Static_assert(sizeof(pb_chroma_row_t) * 5UL == PB_ZOOM_NR_CHROMA_BYTES, "pb_chroma_row_t size mismatch");

typedef struct
{
    pb_nr_params_t   nr;
    pb_chroma_row_t *chroma; /* 5 contiguous rows = 2,400 B */
    int8_t          *y_h[5]; /* 5 x 320 B luma rows = 1,600 B */
    int16_t          cb_even[PB_SCR_W / 2U];
    int16_t          cr_even[PB_SCR_W / 2U];
    int16_t          yh_even[PB_SCR_W / 2U];
    int8_t           tmp_y0[PB_SCR_W];
    uint8_t         *dma_buf[2];
    const uint8_t  (*bar_buf)[160];
    uint8_t          buf_idx;
} pb_zoom_nr_t;

static inline __attribute__((always_inline)) int32_t pb_nr_abs(int32_t v)
{
    return (v >= 0) ? v : -v;
}

static inline __attribute__((always_inline)) int8_t pb_nr_clamp_i8(int32_t v)
{
    return (int8_t)__SSAT(v, 8);
}

static inline __attribute__((always_inline)) uint8_t pb_nr_clamp_u8(int32_t v)
{
    return (uint8_t)__USAT(v, 8);
}

/* Inline calibrated cam12 -> sRGB8 (matches c3m1_jpeg.c bit-for-bit, avoids 70,400 cross-TU calls/frame) */
static inline __attribute__((always_inline)) uint8_t pb_lin12_to_srgb8(int32_t v12, const uint8_t srgb_lut[256])
{
    if (v12 <= 0)    return 0U;
    if (v12 >= 4064) return 255U;
    uint32_t idx  = (uint32_t)v12 >> 4;
    int32_t  frac = (int32_t)((uint32_t)v12 & 15UL);
    int32_t  s0   = (int32_t)srgb_lut[idx];
    int32_t  s1   = (int32_t)srgb_lut[idx + 1U];
    return (uint8_t)__USAT(s0 + (((s1 - s0) * frac + 8) >> 4), 8);
}

static inline __attribute__((always_inline)) void pb_norm_wb_q8(
    uint16_t wb_r_q8, uint16_t wb_b_q8,
    uint16_t *out_r, uint16_t *out_g, uint16_t *out_b)
{
    uint32_t min_wb = 256UL;
    if ((uint32_t)wb_r_q8 < min_wb) min_wb = (uint32_t)wb_r_q8;
    if ((uint32_t)wb_b_q8 < min_wb) min_wb = (uint32_t)wb_b_q8;
    if (min_wb < 256UL && min_wb > 0UL)
    {
        *out_r = (uint16_t)(((uint32_t)wb_r_q8 * 256UL + (min_wb >> 1)) / min_wb);
        *out_g = (uint16_t)((65536UL + (min_wb >> 1)) / min_wb);
        *out_b = (uint16_t)(((uint32_t)wb_b_q8 * 256UL + (min_wb >> 1)) / min_wb);
    }
    else
    {
        *out_r = wb_r_q8;
        *out_g = 256U;
        *out_b = wb_b_q8;
    }
}

static inline __attribute__((always_inline)) void pb_color_cam12_to_srgb8_norm(
    uint16_t r12, uint16_t g12, uint16_t b12,
    uint16_t wbr_n, uint16_t wbg_n, uint16_t wbb_n,
    const uint8_t srgb_lut[256],
    uint8_t *out_r, uint8_t *out_g, uint8_t *out_b)
{
    int32_t rl = (int32_t)(((uint32_t)r12 * (uint32_t)wbr_n) >> 8);
    int32_t gl = (int32_t)(((uint32_t)g12 * (uint32_t)wbg_n) >> 8);
    int32_t bl = (int32_t)(((uint32_t)b12 * (uint32_t)wbb_n) >> 8);
    if (rl > 4064) rl = 4064;
    if (gl > 4064) gl = 4064;
    if (bl > 4064) bl = 4064;

    int32_t cr = (384 * rl -  92 * gl -  36 * bl + 128) >> 8;
    int32_t cg = (-64 * rl + 380 * gl -  60 * bl + 128) >> 8;
    int32_t cb = (-20 * rl - 104 * gl + 380 * bl + 128) >> 8;
    *out_r = pb_lin12_to_srgb8(cr, srgb_lut);
    *out_g = pb_lin12_to_srgb8(cg, srgb_lut);
    *out_b = pb_lin12_to_srgb8(cb, srgb_lut);
}

static inline __attribute__((always_inline)) void pb_color_cam12_to_srgb8(
    uint16_t r12, uint16_t g12, uint16_t b12,
    uint16_t wb_r_q8, uint16_t wb_b_q8,
    const uint8_t srgb_lut[256],
    uint8_t *out_r, uint8_t *out_g, uint8_t *out_b)
{
    uint16_t wbr_n, wbg_n, wbb_n;
    pb_norm_wb_q8(wb_r_q8, wb_b_q8, &wbr_n, &wbg_n, &wbb_n);
    pb_color_cam12_to_srgb8_norm(r12, g12, b12, wbr_n, wbg_n, wbb_n, srgb_lut, out_r, out_g, out_b);
}

/* Inline B&W linear-light cam12 -> pre-tone sRGB8 (skips color CCM to avoid ~1.55x noise boost) */
static inline __attribute__((always_inline)) uint8_t pb_color_bw_cam12_to_y8_norm(
    uint16_t r12, uint16_t g12, uint16_t b12,
    uint16_t wbr_n, uint16_t wbg_n, uint16_t wbb_n,
    uint8_t color_mode,
    const uint8_t srgb_lut[256])
{
    int32_t rl = (int32_t)(((uint32_t)r12 * (uint32_t)wbr_n) >> 8);
    int32_t gl = (int32_t)(((uint32_t)g12 * (uint32_t)wbg_n) >> 8);
    int32_t bl = (int32_t)(((uint32_t)b12 * (uint32_t)wbb_n) >> 8);
    if (rl > 4064) rl = 4064;
    if (gl > 4064) gl = 4064;
    if (bl > 4064) bl = 4064;

    int32_t yl = (color_mode == 3U)
                 ? ((168 * rl +  80 * gl +  8 * bl + 128) >> 8)
                 : (( 64 * rl + 174 * gl + 18 * bl + 128) >> 8);
    return pb_lin12_to_srgb8(yl, srgb_lut);
}

static inline __attribute__((always_inline)) uint8_t pb_color_bw_cam12_to_y8(
    uint16_t r12, uint16_t g12, uint16_t b12,
    uint16_t wb_r_q8, uint16_t wb_b_q8,
    uint8_t color_mode,
    const uint8_t srgb_lut[256])
{
    uint16_t wbr_n, wbg_n, wbb_n;
    pb_norm_wb_q8(wb_r_q8, wb_b_q8, &wbr_n, &wbg_n, &wbb_n);
    return pb_color_bw_cam12_to_y8_norm(r12, g12, b12, wbr_n, wbg_n, wbb_n, color_mode, srgb_lut);
}

/* Fused YCbCr -> RGB666 + 2x2 Bayer dither using single-cycle Cortex-M3 USAT */
static inline __attribute__((always_inline)) void pb_put_rgb666_ycbcr_d(
    uint8_t *dst, int32_t y8_d, int32_t dr, int32_t dg, int32_t db)
{
    dst[0] = (uint8_t)(__USAT(y8_d + dr, 8) & 0xFCU);
    dst[1] = (uint8_t)(__USAT(y8_d - dg, 8) & 0xFCU);
    dst[2] = (uint8_t)(__USAT(y8_d + db, 8) & 0xFCU);
}

/* Exact fast v % 5 for v in [0..255] (zero hardware udiv) */
static inline __attribute__((always_inline)) uint8_t pb_nr_mod5(uint16_t v)
{
    return (uint8_t)(v - (((uint32_t)v * 52429UL) >> 18) * 5UL);
}

/* Fast 8-bit Gamma-domain isolated hot/cold pixel rejection (matches c3m1_jpeg.c Stage 1) */
static inline __attribute__((always_inline)) uint8_t pb_nr_despeckle_u8(uint8_t c,
                                                                        uint8_t n0, uint8_t n1,
                                                                        uint8_t n2, uint8_t n3,
                                                                        uint8_t thr)
{
    uint8_t lo = n0, hi = n0;
    if (n1 < lo) lo = n1;
    if (n1 > hi) hi = n1;
    if (n2 < lo) lo = n2;
    if (n2 > hi) hi = n2;
    if (n3 < lo) lo = n3;
    if (n3 > hi) hi = n3;

    if (__builtin_expect((int32_t)c > ((int32_t)hi + (int32_t)thr) ||
                         (int32_t)c < ((int32_t)lo - (int32_t)thr), 0))
    {
        return (uint8_t)(((uint32_t)n0 + n1 + n2 + n3 + 2UL) >> 2);
    }
    return c;
}

static inline __attribute__((always_inline)) void pb_nr_rgb_to_ycbcr(uint8_t sr, uint8_t sg, uint8_t sb,
                                                                     int8_t *oy, int8_t *ocb, int8_t *ocr)
{
    int32_t yv  = (( 77 * (int32_t)sr + 150 * (int32_t)sg +  29 * (int32_t)sb + 128) >> 8) - 128;
    int32_t cbv = ((-43 * (int32_t)sr -  85 * (int32_t)sg + 128 * (int32_t)sb + 128) >> 8);
    int32_t crv = ((128 * (int32_t)sr - 107 * (int32_t)sg -  21 * (int32_t)sb + 128) >> 8);
    *oy  = pb_nr_clamp_i8(yv);
    *ocb = pb_nr_clamp_i8(cbv);
    *ocr = pb_nr_clamp_i8(crv);
}

static inline __attribute__((always_inline)) void pb_nr_ycbcr_to_rgb(int32_t yv, int32_t cbv, int32_t crv,
                                                                     uint8_t *or8, uint8_t *og8, uint8_t *ob8)
{
    int32_t y8 = yv + 128;
    *or8 = pb_nr_clamp_u8(y8 + ((359 * crv + 128) >> 8));
    *og8 = pb_nr_clamp_u8(y8 - (( 88 * cbv + 183 * crv + 128) >> 8));
    *ob8 = pb_nr_clamp_u8(y8 + ((454 * cbv + 128) >> 8));
}

/* ISO-adaptive NR curve matching c3m1_jpeg.c (ISO<=200 bypassed, 400-640 quality-preserving, 800-3200 ramp) */
static void pb_nr_init_params(pb_nr_params_t *nr, uint16_t iso)
{
    memset(nr, 0, sizeof(*nr));
    if (iso <= 200U)
    {
        return;
    }

    uint32_t lvl;
    if (iso <= 400U)
    {
        lvl = ((uint32_t)(iso - 200U) * 64UL) / 200UL;
    }
    else if (iso <= 800U)
    {
        lvl = 64UL + ((uint32_t)(iso - 400U) * 64UL) / 400UL;
    }
    else if (iso <= 1600U)
    {
        lvl = 128UL + ((uint32_t)(iso - 800U) * 64UL) / 800UL;
    }
    else if (iso <= 3200U)
    {
        lvl = 192UL + ((uint32_t)(iso - 1600U) * 64UL) / 1600UL;
    }
    else
    {
        lvl = 256UL;
    }
    if (lvl == 0UL)
    {
        return;
    }

    nr->lvl = (uint16_t)lvl;
    if (lvl >= 64UL)
    {
        nr->raw_imp_thr = (uint8_t)(28UL - ((lvl * 10UL) >> 8)); /* 26@400, 23@800, 21@1600, 18@3200 */
    }

    uint32_t sc = 12UL + ((lvl * 44UL) >> 8); /* 23@400, 34@800, 45@1600, 56@3200 */
    for (uint32_t e = 0UL; e < sc && e < 64UL; e++)
    {
        uint32_t rem = sc - e;
        nr->w_c1[e] = (uint8_t)((14UL * rem + (sc >> 1)) / sc);
        nr->w_c2[e] = (uint8_t)((10UL * rem + (sc >> 1)) / sc);
    }

    if (lvl >= 64UL)
    {
        nr->has_luma_nr = 1U;
        uint32_t sy    = 4UL + ((lvl * 28UL) >> 8); /* 11@400, 18@800, 25@1600, 32@3200 */
        uint32_t sy_sh = (sy * 11UL) >> 3;          /* 1.375x wider in dark shadows (Y < -48) */
        for (uint32_t d = 0UL; d < sy && d < 64UL; d++)
        {
            nr->w_y_mid[d] = (uint8_t)((4UL * (sy - d) + (sy >> 1)) / sy);
        }
        for (uint32_t d = 0UL; d < sy_sh && d < 64UL; d++)
        {
            nr->w_y_sh[d] = (uint8_t)((4UL * (sy_sh - d) + (sy_sh >> 1)) / sy_sh);
        }
    }
}

/* 1X thumbnail 3-tap H pass (160 px, zero udiv/sdiv): Chroma Joint-Bilateral + optional gentle Luma */
static void __attribute__((noinline, optimize("O2"))) pb_thumb_nr_h_row(
    const uint8_t *rgb_row,
    const uint8_t w_tc[64],
    const uint8_t w_ty_mid[64],
    const uint8_t w_ty_sh[64],
    uint8_t has_luma,
    int8_t yh_dst[C3M1_DNG_THUMB_W],
    int8_t cbh_dst[C3M1_DNG_THUMB_W],
    int8_t crh_dst[C3M1_DNG_THUMB_W])
{
    int8_t ly[C3M1_DNG_THUMB_W + 2U];
    int8_t lcb[C3M1_DNG_THUMB_W + 2U];
    int8_t lcr[C3M1_DNG_THUMB_W + 2U];

    for (uint16_t x = 0U; x < C3M1_DNG_THUMB_W; x++)
    {
        uint32_t o = (uint32_t)x * 3U;
        pb_nr_rgb_to_ycbcr(rgb_row[o], rgb_row[o + 1U], rgb_row[o + 2U],
                           &ly[x + 1U], &lcb[x + 1U], &lcr[x + 1U]);
    }
    ly[0]  = ly[1];  ly[C3M1_DNG_THUMB_W + 1U]  = ly[C3M1_DNG_THUMB_W];
    lcb[0] = lcb[1]; lcb[C3M1_DNG_THUMB_W + 1U] = lcb[C3M1_DNG_THUMB_W];
    lcr[0] = lcr[1]; lcr[C3M1_DNG_THUMB_W + 1U] = lcr[C3M1_DNG_THUMB_W];

    for (uint16_t x = 0U; x < C3M1_DNG_THUMB_W; x++)
    {
        int32_t y0 = ly[x + 1U];
        int32_t c0 = lcb[x + 1U];
        int32_t r0 = lcr[x + 1U];
        uint8_t is_sh = (y0 < -48) ? 1U : 0U;

        int32_t acc_cb = 8, acc_cr = 8;
        int32_t dcb = (int32_t)lcb[x] - c0;
        int32_t dcr = (int32_t)lcr[x] - r0;
        int32_t dy  = pb_nr_abs((int32_t)ly[x] - y0);
        int32_t dc  = (pb_nr_abs(dcb) + pb_nr_abs(dcr)) >> 1;
        if (is_sh) dc = (dc * 3) >> 2;
        uint32_t e = (uint32_t)(((dy * 3) >> 1) + dc);
        if (e < 64U)
        {
            int32_t w = (int32_t)w_tc[e];
            acc_cb += w * dcb;
            acc_cr += w * dcr;
        }

        dcb = (int32_t)lcb[x + 2U] - c0;
        dcr = (int32_t)lcr[x + 2U] - r0;
        dy  = pb_nr_abs((int32_t)ly[x + 2U] - y0);
        dc  = (pb_nr_abs(dcb) + pb_nr_abs(dcr)) >> 1;
        if (is_sh) dc = (dc * 3) >> 2;
        e = (uint32_t)(((dy * 3) >> 1) + dc);
        if (e < 64U)
        {
            int32_t w = (int32_t)w_tc[e];
            acc_cb += w * dcb;
            acc_cr += w * dcr;
        }
        cbh_dst[x] = (int8_t)(c0 + (acc_cb >> 4));
        crh_dst[x] = (int8_t)(r0 + (acc_cr >> 4));

        if (has_luma)
        {
            const uint8_t *wl = is_sh ? w_ty_sh : w_ty_mid;
            int32_t dl = (int32_t)ly[x]      - y0;
            int32_t dr = (int32_t)ly[x + 2U] - y0;
            uint32_t al = (uint32_t)pb_nr_abs(dl);
            uint32_t ar = (uint32_t)pb_nr_abs(dr);
            int32_t ay = 8;
            if (al < 64U) ay += (int32_t)wl[al] * dl;
            if (ar < 64U) ay += (int32_t)wl[ar] * dr;
            yh_dst[x] = (int8_t)(y0 + (ay >> 4));
        }
        else
        {
            yh_dst[x] = (int8_t)y0;
        }
    }
}

/* 1X thumbnail 3-tap V pass (160 px, zero udiv/sdiv) -> denoised RGB888 row */
static void __attribute__((noinline, optimize("O2"))) pb_thumb_nr_v_row(
    const uint8_t w_tc[64],
    const uint8_t w_ty_mid[64],
    const uint8_t w_ty_sh[64],
    uint8_t has_luma,
    const int8_t *yh_up,  const int8_t *yh_c,  const int8_t *yh_dn,
    const int8_t *cbh_up, const int8_t *cbh_c, const int8_t *cbh_dn,
    const int8_t *crh_up, const int8_t *crh_c, const int8_t *crh_dn,
    uint8_t *rgb_out)
{
    for (uint16_t x = 0U; x < C3M1_DNG_THUMB_W; x++)
    {
        int32_t y0 = yh_c[x];
        int32_t c0 = cbh_c[x];
        int32_t r0 = crh_c[x];
        uint8_t is_sh = (y0 < -48) ? 1U : 0U;

        int32_t acc_cb = 8, acc_cr = 8;
        int32_t dcb = (int32_t)cbh_up[x] - c0;
        int32_t dcr = (int32_t)crh_up[x] - r0;
        int32_t dy  = pb_nr_abs((int32_t)yh_up[x] - y0);
        int32_t dc  = (pb_nr_abs(dcb) + pb_nr_abs(dcr)) >> 1;
        if (is_sh) dc = (dc * 3) >> 2;
        uint32_t e = (uint32_t)(((dy * 3) >> 1) + dc);
        if (e < 64U)
        {
            int32_t w = (int32_t)w_tc[e];
            acc_cb += w * dcb;
            acc_cr += w * dcr;
        }

        dcb = (int32_t)cbh_dn[x] - c0;
        dcr = (int32_t)crh_dn[x] - r0;
        dy  = pb_nr_abs((int32_t)yh_dn[x] - y0);
        dc  = (pb_nr_abs(dcb) + pb_nr_abs(dcr)) >> 1;
        if (is_sh) dc = (dc * 3) >> 2;
        e = (uint32_t)(((dy * 3) >> 1) + dc);
        if (e < 64U)
        {
            int32_t w = (int32_t)w_tc[e];
            acc_cb += w * dcb;
            acc_cr += w * dcr;
        }

        int32_t cb_out = c0 + (acc_cb >> 4);
        int32_t cr_out = r0 + (acc_cr >> 4);
        int32_t y_out  = y0;
        if (has_luma)
        {
            const uint8_t *wl = is_sh ? w_ty_sh : w_ty_mid;
            int32_t du = (int32_t)yh_up[x] - y0;
            int32_t dd = (int32_t)yh_dn[x] - y0;
            uint32_t au = (uint32_t)pb_nr_abs(du);
            uint32_t ad = (uint32_t)pb_nr_abs(dd);
            int32_t ay = 8;
            if (au < 64U) ay += (int32_t)wl[au] * du;
            if (ad < 64U) ay += (int32_t)wl[ad] * dd;
            y_out = y0 + (ay >> 4);
        }

        uint8_t r8, g8, b8;
        pb_nr_ycbcr_to_rgb(y_out, cb_out, cr_out, &r8, &g8, &b8);
        uint32_t o = (uint32_t)x * 3U;
        rgb_out[o + 0U] = r8;
        rgb_out[o + 1U] = g8;
        rgb_out[o + 2U] = b8;
    }
}

/* 4X Zoom Luma Pass H: 3-tap padded bilateral (denom 16, >> 4) or passthrough at ISO 250-320 */
static void __attribute__((noinline, optimize("O2"))) pb_zoom_nr_luma_h(
    const pb_nr_params_t *nr,
    const int8_t y_pad[PB_SCR_W + 2U],
    int8_t dst[PB_SCR_W])
{
    if (!nr->has_luma_nr)
    {
        memcpy(dst, &y_pad[1], PB_SCR_W);
        return;
    }
    const uint8_t *const w_mid = nr->w_y_mid;
    const uint8_t *const w_sh  = nr->w_y_sh;
    for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
    {
        int32_t c = y_pad[dx + 1U];
        const uint8_t *wl = (c < -48) ? w_sh : w_mid;
        int32_t dl = (int32_t)y_pad[dx]      - c;
        int32_t dr = (int32_t)y_pad[dx + 2U] - c;
        uint32_t al = (uint32_t)pb_nr_abs(dl);
        uint32_t ar = (uint32_t)pb_nr_abs(dr);
        int32_t acc = 8;
        if (al < 64U) acc += (int32_t)wl[al] * dl;
        if (ar < 64U) acc += (int32_t)wl[ar] * dr;
        dst[dx] = (int8_t)(c + (acc >> 4));
    }
}

/* 4X Zoom Chroma Pass H (5-tap padded, >> 6, matches c3m1_jpeg.c Stage 2A) + odd-row Luma H */
static void __attribute__((noinline, optimize("O2"))) pb_zoom_nr_prep_odd(
    pb_zoom_nr_t *znr,
    uint16_t in_cy,
    const uint8_t *rgb_row,
    int8_t tmp_y1[PB_SCR_W])
{
    int8_t y_pad[PB_SCR_W + 2U];
    int8_t lcb[(PB_SCR_W / 2U) + 4U];
    int8_t lcr[(PB_SCR_W / 2U) + 4U];
    int8_t lyh[(PB_SCR_W / 2U) + 4U];

    for (uint16_t cx = 0U; cx < (PB_SCR_W / 2U); cx++)
    {
        uint16_t dx0 = (uint16_t)(cx << 1);
        uint32_t o0  = (uint32_t)dx0 * 3U;
        int8_t y0, cb0, cr0, y1, cb1, cr1;
        pb_nr_rgb_to_ycbcr(rgb_row[o0],      rgb_row[o0 + 1U], rgb_row[o0 + 2U], &y0, &cb0, &cr0);
        pb_nr_rgb_to_ycbcr(rgb_row[o0 + 3U], rgb_row[o0 + 4U], rgb_row[o0 + 5U], &y1, &cb1, &cr1);
        y_pad[dx0 + 1U] = y0;
        y_pad[dx0 + 2U] = y1;
        lcb[cx + 2U] = pb_nr_clamp_i8(((int32_t)znr->cb_even[cx] + cb0 + cb1 + 2) >> 2);
        lcr[cx + 2U] = pb_nr_clamp_i8(((int32_t)znr->cr_even[cx] + cr0 + cr1 + 2) >> 2);
        lyh[cx + 2U] = pb_nr_clamp_i8(((int32_t)znr->yh_even[cx] + y0  + y1  + 2) >> 2);
    }
    y_pad[0] = y_pad[1];
    y_pad[PB_SCR_W + 1U] = y_pad[PB_SCR_W];

    lcb[0] = lcb[2]; lcb[1] = lcb[2];
    lcr[0] = lcr[2]; lcr[1] = lcr[2];
    lyh[0] = lyh[2]; lyh[1] = lyh[2];
    lcb[(PB_SCR_W / 2U) + 2U] = lcb[(PB_SCR_W / 2U) + 1U];
    lcb[(PB_SCR_W / 2U) + 3U] = lcb[(PB_SCR_W / 2U) + 1U];
    lcr[(PB_SCR_W / 2U) + 2U] = lcr[(PB_SCR_W / 2U) + 1U];
    lcr[(PB_SCR_W / 2U) + 3U] = lcr[(PB_SCR_W / 2U) + 1U];
    lyh[(PB_SCR_W / 2U) + 2U] = lyh[(PB_SCR_W / 2U) + 1U];
    lyh[(PB_SCR_W / 2U) + 3U] = lyh[(PB_SCR_W / 2U) + 1U];

    pb_chroma_row_t *crow   = &znr->chroma[pb_nr_mod5(in_cy)];
    int8_t          *cb_dst = crow->cb;
    int8_t          *cr_dst = crow->cr;
    memcpy(crow->yh, &lyh[2], PB_SCR_W / 2U);

    const uint8_t *const wc1 = znr->nr.w_c1;
    const uint8_t *const wc2 = znr->nr.w_c2;

#define PB_CHROMA_H_TAP(idx, wlut) do { \
        int32_t _dcb = (int32_t)lcb[idx] - c0; \
        int32_t _dcr = (int32_t)lcr[idx] - r0; \
        int32_t _dy  = pb_nr_abs((int32_t)lyh[idx] - y0); \
        int32_t _dc  = (pb_nr_abs(_dcb) + pb_nr_abs(_dcr)) >> 1; \
        if (is_sh) _dc = (_dc * 3) >> 2; \
        uint32_t _e  = (uint32_t)(((_dy * 3) >> 1) + _dc); \
        if (_e < 64U) { \
            int32_t _w = (int32_t)(wlut)[_e]; \
            acc_cb += _w * _dcb; \
            acc_cr += _w * _dcr; \
        } \
    } while (0)

    for (uint16_t cx = 0U; cx < (PB_SCR_W / 2U); cx++)
    {
        int32_t c0 = lcb[cx + 2U];
        int32_t r0 = lcr[cx + 2U];
        int32_t y0 = lyh[cx + 2U];
        uint8_t is_sh = (y0 < -48) ? 1U : 0U;
        int32_t acc_cb = 32;
        int32_t acc_cr = 32;
        PB_CHROMA_H_TAP(cx,      wc2);
        PB_CHROMA_H_TAP(cx + 1U, wc1);
        PB_CHROMA_H_TAP(cx + 3U, wc1);
        PB_CHROMA_H_TAP(cx + 4U, wc2);
        cb_dst[cx] = (int8_t)(c0 + (acc_cb >> 6));
        cr_dst[cx] = (int8_t)(r0 + (acc_cr >> 6));
    }
#undef PB_CHROMA_H_TAP

    pb_zoom_nr_luma_h(&znr->nr, y_pad, tmp_y1);
}

/* 4X Zoom Chroma Pass V (5 contiguous pb_chroma_row_t pointers -> minimal spills, zero udiv/sdiv) */
static void __attribute__((noinline, optimize("O2"))) pb_zoom_nr_chroma_v(
    const pb_nr_params_t *nr,
    const pb_chroma_row_t *rm2,
    const pb_chroma_row_t *rm1,
    const pb_chroma_row_t *rc,
    const pb_chroma_row_t *rp1,
    const pb_chroma_row_t *rp2,
    int8_t cb_v[PB_SCR_W / 2U],
    int8_t cr_v[PB_SCR_W / 2U])
{
    const uint8_t *const wc1 = nr->w_c1;
    const uint8_t *const wc2 = nr->w_c2;

#define PB_CHROMA_V_TAP(rowp, wlut) do { \
        int32_t _dcb = (int32_t)(rowp)->cb[cx] - c0; \
        int32_t _dcr = (int32_t)(rowp)->cr[cx] - r0; \
        int32_t _dy  = pb_nr_abs((int32_t)(rowp)->yh[cx] - y0); \
        int32_t _dc  = (pb_nr_abs(_dcb) + pb_nr_abs(_dcr)) >> 1; \
        if (is_sh) _dc = (_dc * 3) >> 2; \
        uint32_t _e  = (uint32_t)(((_dy * 3) >> 1) + _dc); \
        if (_e < 64U) { \
            int32_t _w = (int32_t)(wlut)[_e]; \
            acc_cb += _w * _dcb; \
            acc_cr += _w * _dcr; \
        } \
    } while (0)

    for (uint16_t cx = 0U; cx < (PB_SCR_W / 2U); cx++)
    {
        int32_t c0 = rc->cb[cx];
        int32_t r0 = rc->cr[cx];
        int32_t y0 = rc->yh[cx];
        uint8_t is_sh = (y0 < -48) ? 1U : 0U;
        int32_t acc_cb = 32;
        int32_t acc_cr = 32;
        PB_CHROMA_V_TAP(rm2, wc2);
        PB_CHROMA_V_TAP(rm1, wc1);
        PB_CHROMA_V_TAP(rp1, wc1);
        PB_CHROMA_V_TAP(rp2, wc2);
        cb_v[cx] = (int8_t)(c0 + (acc_cb >> 6));
        cr_v[cx] = (int8_t)(r0 + (acc_cr >> 6));
    }
#undef PB_CHROMA_V_TAP
}

/* 4X Zoom Luma Pass V + fused USAT YCbCr->RGB666 dither (shared 2x2 chroma RGB deltas -> 2x fewer muls) */
static void __attribute__((noinline, optimize("O2"))) pb_zoom_nr_emit_luma_row(
    const pb_nr_params_t *nr,
    uint16_t dy,
    const int8_t *y_up,
    const int8_t *y_c,
    const int8_t *y_dn,
    const int8_t *cb_v,
    const int8_t *cr_v,
    uint8_t *out)
{
    const int32_t d0 = (dy & 1U) ? 131 : 128; /* 128 + s_dither2x2[dy & 1][0] */
    const int32_t d1 = (dy & 1U) ? 129 : 130; /* 128 + s_dither2x2[dy & 1][1] */

    if (nr->has_luma_nr)
    {
        const uint8_t *const w_mid = nr->w_y_mid;
        const uint8_t *const w_sh  = nr->w_y_sh;
        for (uint16_t cx = 0U; cx < (PB_SCR_W / 2U); cx++)
        {
            int32_t cb = cb_v[cx];
            int32_t cr = cr_v[cx];
            int32_t dr = (359 * cr + 128) >> 8;
            int32_t dg = ( 88 * cb + 183 * cr + 128) >> 8;
            int32_t db = (454 * cb + 128) >> 8;

            uint16_t dx0 = (uint16_t)(cx << 1);
            int32_t y0 = y_c[dx0];
            const uint8_t *wl0 = (y0 < -48) ? w_sh : w_mid;
            int32_t du0 = (int32_t)y_up[dx0] - y0;
            int32_t dd0 = (int32_t)y_dn[dx0] - y0;
            uint32_t au0 = (uint32_t)pb_nr_abs(du0);
            uint32_t ad0 = (uint32_t)pb_nr_abs(dd0);
            int32_t a0 = 8;
            if (au0 < 64U) a0 += (int32_t)wl0[au0] * du0;
            if (ad0 < 64U) a0 += (int32_t)wl0[ad0] * dd0;
            pb_put_rgb666_ycbcr_d(out, y0 + (a0 >> 4) + d0, dr, dg, db);

            uint16_t dx1 = (uint16_t)(dx0 + 1U);
            int32_t y1 = y_c[dx1];
            const uint8_t *wl1 = (y1 < -48) ? w_sh : w_mid;
            int32_t du1 = (int32_t)y_up[dx1] - y1;
            int32_t dd1 = (int32_t)y_dn[dx1] - y1;
            uint32_t au1 = (uint32_t)pb_nr_abs(du1);
            uint32_t ad1 = (uint32_t)pb_nr_abs(dd1);
            int32_t a1 = 8;
            if (au1 < 64U) a1 += (int32_t)wl1[au1] * du1;
            if (ad1 < 64U) a1 += (int32_t)wl1[ad1] * dd1;
            pb_put_rgb666_ycbcr_d(out + 3U, y1 + (a1 >> 4) + d1, dr, dg, db);
            out += 6U;
        }
    }
    else
    {
        for (uint16_t cx = 0U; cx < (PB_SCR_W / 2U); cx++)
        {
            int32_t cb = cb_v[cx];
            int32_t cr = cr_v[cx];
            int32_t dr = (359 * cr + 128) >> 8;
            int32_t dg = ( 88 * cb + 183 * cr + 128) >> 8;
            int32_t db = (454 * cb + 128) >> 8;

            uint16_t dx0 = (uint16_t)(cx << 1);
            pb_put_rgb666_ycbcr_d(out,      (int32_t)y_c[dx0]      + d0, dr, dg, db);
            pb_put_rgb666_ycbcr_d(out + 3U, (int32_t)y_c[dx0 + 1U] + d1, dr, dg, db);
            out += 6U;
        }
    }
}

/* 4X Zoom Pass V coordinator: Chroma V + 2 RGB666 DMA rows */
static void __attribute__((noinline, optimize("O2"))) pb_zoom_nr_emit_pair(pb_zoom_nr_t *znr, uint16_t out_cy)
{
    const uint16_t max_cy = (PB_VIEW_H / 2U) - 1U;
    uint16_t cm2 = (out_cy >= 2U) ? (uint16_t)(out_cy - 2U) : 0U;
    uint16_t cm1 = (out_cy >= 1U) ? (uint16_t)(out_cy - 1U) : 0U;
    uint16_t cp1 = (out_cy < max_cy) ? (uint16_t)(out_cy + 1U) : max_cy;
    uint16_t cp2 = ((uint16_t)(out_cy + 2U) <= max_cy) ? (uint16_t)(out_cy + 2U) : max_cy;

    int8_t cb_v[PB_SCR_W / 2U];
    int8_t cr_v[PB_SCR_W / 2U];
    pb_zoom_nr_chroma_v(&znr->nr,
                        &znr->chroma[pb_nr_mod5(cm2)],
                        &znr->chroma[pb_nr_mod5(cm1)],
                        &znr->chroma[pb_nr_mod5(out_cy)],
                        &znr->chroma[pb_nr_mod5(cp1)],
                        &znr->chroma[pb_nr_mod5(cp2)],
                        cb_v, cr_v);

    for (uint16_t sub = 0U; sub < 2U; sub++)
    {
        uint16_t dy   = (uint16_t)((out_cy << 1) + sub);
        uint16_t r_up = (dy > 0U) ? (uint16_t)(dy - 1U) : 0U;
        uint16_t r_dn = (dy + 1U < PB_VIEW_H) ? (uint16_t)(dy + 1U) : (uint16_t)(PB_VIEW_H - 1U);

        pb_zoom_nr_emit_luma_row(&znr->nr, dy,
                                 znr->y_h[pb_nr_mod5(r_up)],
                                 znr->y_h[pb_nr_mod5(dy)],
                                 znr->y_h[pb_nr_mod5(r_dn)],
                                 cb_v, cr_v,
                                 znr->dma_buf[znr->buf_idx]);
        if (dy >= PB_BAR_Y && znr->bar_buf != NULL)
        {
            pb_overlay_bar_rgb666(znr->dma_buf[znr->buf_idx], dy, znr->bar_buf);
        }
        pb_dma_send(znr->dma_buf[znr->buf_idx], (uint16_t)(PB_SCR_W * 3U));
        znr->buf_idx ^= 1U;
    }
}

static void __attribute__((noinline, optimize("O2"))) pb_zoom_nr_push_even(
    pb_zoom_nr_t *znr,
    const uint8_t *rgb_row)
{
    int8_t y_pad[PB_SCR_W + 2U];
    for (uint16_t cx = 0U; cx < (PB_SCR_W / 2U); cx++)
    {
        uint16_t dx0 = (uint16_t)(cx << 1);
        uint32_t o0  = (uint32_t)dx0 * 3U;
        int8_t y0, cb0, cr0, y1, cb1, cr1;
        pb_nr_rgb_to_ycbcr(rgb_row[o0],      rgb_row[o0 + 1U], rgb_row[o0 + 2U], &y0, &cb0, &cr0);
        pb_nr_rgb_to_ycbcr(rgb_row[o0 + 3U], rgb_row[o0 + 4U], rgb_row[o0 + 5U], &y1, &cb1, &cr1);
        y_pad[dx0 + 1U]  = y0;
        y_pad[dx0 + 2U]  = y1;
        znr->cb_even[cx] = (int16_t)(cb0 + cb1);
        znr->cr_even[cx] = (int16_t)(cr0 + cr1);
        znr->yh_even[cx] = (int16_t)(y0  + y1);
    }
    y_pad[0] = y_pad[1];
    y_pad[PB_SCR_W + 1U] = y_pad[PB_SCR_W];
    pb_zoom_nr_luma_h(&znr->nr, y_pad, znr->tmp_y0);
}

static void pb_zoom_nr_push_row(pb_zoom_nr_t *znr, uint16_t dy, const uint8_t *rgb_row)
{
    if ((dy & 1U) == 0U)
    {
        pb_zoom_nr_push_even(znr, rgb_row);
    }
    else
    {
        uint16_t in_cy = (uint16_t)(dy >> 1);
        int8_t   tmp_y1[PB_SCR_W];
        pb_zoom_nr_prep_odd(znr, in_cy, rgb_row, tmp_y1);
        if (in_cy >= 2U)
        {
            pb_zoom_nr_emit_pair(znr, (uint16_t)(in_cy - 2U));
        }
        memcpy(znr->y_h[pb_nr_mod5((uint16_t)(dy - 1U))], znr->tmp_y0, PB_SCR_W);
        memcpy(znr->y_h[pb_nr_mod5(dy)],                  tmp_y1,      PB_SCR_W);
    }
}

static void pb_zoom_nr_flush(pb_zoom_nr_t *znr)
{
    const uint16_t last_cy = (PB_VIEW_H / 2U) - 1U;
    pb_zoom_nr_emit_pair(znr, (uint16_t)(last_cy - 1U));
    pb_zoom_nr_emit_pair(znr, last_cy);
}

/* B&W Luma-only NR LUT adjustment: ~30% softer than color to preserve film grain, skip Chroma */
static void pb_nr_init_params_bw(pb_nr_params_t *nr, uint16_t iso)
{
    pb_nr_init_params(nr, iso);
    if (nr->lvl == 0U || !nr->has_luma_nr)
    {
        nr->lvl = 0U; /* ISO <= 320: B&W has no chroma noise -> 100% bypass */
        return;
    }
    memset(nr->w_y_mid, 0, sizeof(nr->w_y_mid));
    memset(nr->w_y_sh,  0, sizeof(nr->w_y_sh));
    uint32_t sy    = 3UL + (((uint32_t)nr->lvl * 20UL) >> 8);
    uint32_t sy_sh = (sy * 11UL) >> 3;
    for (uint32_t d = 0UL; d < sy && d < 64UL; d++)
    {
        nr->w_y_mid[d] = (uint8_t)((3UL * (sy - d) + (sy >> 1)) / sy);
    }
    for (uint32_t d = 0UL; d < sy_sh && d < 64UL; d++)
    {
        nr->w_y_sh[d] = (uint8_t)((3UL * (sy_sh - d) + (sy_sh >> 1)) / sy_sh);
    }
}

/* B&W 4X Zoom Luma Pass V -> film S-curve tone_lut -> neutral 18-bit RGB666 DMA row */
static void __attribute__((noinline, optimize("O2"))) pb_zoom_nr_emit_bw_row(
    pb_zoom_nr_t *znr,
    uint16_t out_dy,
    const uint8_t *tone_lut)
{
    uint16_t r_up = (out_dy > 0U) ? (uint16_t)(out_dy - 1U) : 0U;
    uint16_t r_dn = (out_dy + 1U < PB_VIEW_H) ? (uint16_t)(out_dy + 1U) : (uint16_t)(PB_VIEW_H - 1U);
    const int8_t  *y_up  = znr->y_h[r_up % 3U];
    const int8_t  *y_c   = znr->y_h[out_dy % 3U];
    const int8_t  *y_dn  = znr->y_h[r_dn % 3U];
    const uint8_t *w_mid = znr->nr.w_y_mid;
    const uint8_t *w_sh  = znr->nr.w_y_sh;
    uint8_t       *out   = znr->dma_buf[znr->buf_idx];

    for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
    {
        int32_t y0 = y_c[dx];
        const uint8_t *wl = (y0 < -48) ? w_sh : w_mid;
        int32_t du = (int32_t)y_up[dx] - y0;
        int32_t dd = (int32_t)y_dn[dx] - y0;
        uint32_t au = (uint32_t)pb_nr_abs(du);
        uint32_t ad = (uint32_t)pb_nr_abs(dd);
        int32_t acc = 8;
        if (au < 64U) acc += (int32_t)wl[au] * du;
        if (ad < 64U) acc += (int32_t)wl[ad] * dd;
        uint8_t y8 = (uint8_t)(y0 + (acc >> 4) + 128);
        uint8_t yt = tone_lut[y8];
        pb_put_rgb666_dither(out, yt, yt, yt, dx, out_dy);
        out += 3U;
    }

    if (out_dy >= PB_BAR_Y && znr->bar_buf != NULL)
    {
        pb_overlay_bar_rgb666(znr->dma_buf[znr->buf_idx], out_dy, znr->bar_buf);
    }
    pb_dma_send(znr->dma_buf[znr->buf_idx], (uint16_t)(PB_SCR_W * 3U));
    znr->buf_idx ^= 1U;
}

/* Push one pre-tone B&W row (znr->tmp_y0[320] filled with signed y - 128); emits dy - 1 via 3-row ring */
static void __attribute__((noinline, optimize("O2"))) pb_zoom_nr_push_bw_row(
    pb_zoom_nr_t *znr,
    uint16_t dy,
    const uint8_t *tone_lut)
{
    int8_t y_pad[PB_SCR_W + 2U];
    y_pad[0] = znr->tmp_y0[0];
    memcpy(&y_pad[1], znr->tmp_y0, PB_SCR_W);
    y_pad[PB_SCR_W + 1U] = znr->tmp_y0[PB_SCR_W - 1U];

    /* Run Luma H directly into y_h[dy % 3] (slot (dy-3)%3 is already dead before emitting dy-1) */
    pb_zoom_nr_luma_h(&znr->nr, y_pad, znr->y_h[dy % 3U]);
    if (dy >= 1U)
    {
        pb_zoom_nr_emit_bw_row(znr, (uint16_t)(dy - 1U), tone_lut);
    }
}

static void pb_zoom_nr_flush_bw(pb_zoom_nr_t *znr, const uint8_t *tone_lut)
{
    pb_zoom_nr_emit_bw_row(znr, (uint16_t)(PB_VIEW_H - 1U), tone_lut);
}

/* Fast 1X path: 16-row batch SD read + optional High-ISO NR + 2x bilinear upscale + DMA1 18-bit RGB666 */
static uint8_t __attribute__((optimize("O2"))) dng_render_1x_thumb(FIL *fp, uint32_t thumb_off,
                                                                    uint8_t clip_en,
                                                                    uint16_t nr_iso,
                                                                    uint8_t *scratch_after_idx,
                                                                    const uint8_t (*bar_buf)[160])
{
    const uint32_t row_sz      = C3M1_DNG_THUMB_W * 3UL; /* 480 B */
    pb_nr_params_t nr;
    pb_nr_init_params(&nr, nr_iso);

    const uint8_t  nr_on       = (nr.lvl != 0U) ? 1U : 0U;
    int16_t        need_rows   = (int16_t)(((PB_VIEW_H - 1U) >> 1) + (nr_on ? 3U : 2U)); /* 111 raw or 112 NR */
    if (need_rows > (int16_t)C3M1_DNG_THUMB_H) need_rows = (int16_t)C3M1_DNG_THUMB_H;

    uint8_t       *dma_buf[2]  = { scratch_after_idx, scratch_after_idx + (PB_SCR_W * 3U) };
    uint8_t       *batch       = scratch_after_idx + (PB_SCR_W * 6U); /* 17 x 480 B = 8,160 B */
    uint8_t        buf_idx     = 0U;

    if (f_lseek(fp, thumb_off) != FR_OK) return 0U;
    int16_t file_next_y = 0;
    int16_t buf_y0      = 0;
    int16_t buf_cnt     = 0;

    /* 1X thumbnail NR workspace inside scratch_after_idx tail (2,400 B, 0 B stack) */
    uint8_t  w_tc[64];
    uint8_t  w_ty_mid[64];
    uint8_t  w_ty_sh[64];
    uint8_t  thumb_has_luma = 0U;
    int8_t  (*ty_h)[C3M1_DNG_THUMB_W]  = NULL;
    int8_t  (*tcb_h)[C3M1_DNG_THUMB_W] = NULL;
    int8_t  (*tcr_h)[C3M1_DNG_THUMB_W] = NULL;
    uint8_t (*den_rgb)[C3M1_DNG_THUMB_W * 3U] = NULL;
    int16_t  h_next_y = 0;
    int16_t  den_y[2] = { -1, -1 };

    if (nr_on)
    {
        memset(w_tc, 0, sizeof(w_tc));
        memset(w_ty_mid, 0, sizeof(w_ty_mid));
        memset(w_ty_sh, 0, sizeof(w_ty_sh));
        uint32_t sc = 12UL + (((uint32_t)nr.lvl * 44UL) >> 8);
        for (uint32_t e = 0UL; e < sc && e < 64UL; e++)
        {
            w_tc[e] = (uint8_t)((4UL * (sc - e) + (sc >> 1)) / sc);
        }
        if (nr.lvl >= 128U)
        {
            thumb_has_luma = 1U;
            uint32_t sy    = 4UL + (((uint32_t)nr.lvl * 14UL) >> 8); /* 11@800, 14@1600, 18@3200 */
            uint32_t sy_sh = (sy * 11UL) >> 3;
            for (uint32_t d = 0UL; d < sy && d < 64UL; d++)
            {
                w_ty_mid[d] = (uint8_t)((3UL * (sy - d) + (sy >> 1)) / sy);
            }
            for (uint32_t d = 0UL; d < sy_sh && d < 64UL; d++)
            {
                w_ty_sh[d] = (uint8_t)((3UL * (sy_sh - d) + (sy_sh >> 1)) / sy_sh);
            }
        }
        uint8_t *nr_ptr = batch + 17UL * row_sz;
        ty_h    = (int8_t (*)[C3M1_DNG_THUMB_W])nr_ptr;                  nr_ptr += 3UL * C3M1_DNG_THUMB_W;
        tcb_h   = (int8_t (*)[C3M1_DNG_THUMB_W])nr_ptr;                  nr_ptr += 3UL * C3M1_DNG_THUMB_W;
        tcr_h   = (int8_t (*)[C3M1_DNG_THUMB_W])nr_ptr;                  nr_ptr += 3UL * C3M1_DNG_THUMB_W;
        den_rgb = (uint8_t (*)[C3M1_DNG_THUMB_W * 3U])nr_ptr;
    }

    lcd_set_rgb666(1U);
    lcd_set_window(0U, 0U, PB_SCR_W, PB_VIEW_H);
    lcd_write_ram_prepare();

    for (uint16_t dy = 0U; dy < PB_VIEW_H; dy++)
    {
        int16_t sy0 = (int16_t)(dy >> 1);
        int16_t sy1 = ((dy & 1U) && (sy0 + 1 < (int16_t)C3M1_DNG_THUMB_H)) ? (int16_t)(sy0 + 1) : sy0;
        const uint8_t *r0;
        const uint8_t *r1;

        if (!nr_on)
        {
            if (sy1 >= (buf_y0 + buf_cnt))
            {
                /* Finish any in-flight LCD DMA before polled SDIO f_read to avoid AHB contention */
                pb_dma_wait();
                int16_t keep = 0;
                if (buf_cnt > 0)
                {
                    memcpy(batch, batch + (uint32_t)(buf_cnt - 1) * row_sz, row_sz);
                    keep   = 1;
                    buf_y0 = (int16_t)(file_next_y - 1);
                }
                else
                {
                    buf_y0 = 0;
                }
                int16_t remain  = (int16_t)(need_rows - file_next_y);
                int16_t to_read = (remain > 16) ? 16 : remain;
                if (to_read <= 0)
                {
                    lcd_set_rgb666(0U);
                    return 0U;
                }
                UINT br = 0U;
                uint32_t req_bytes = (uint32_t)to_read * row_sz;
                if (f_read(fp, batch + (uint32_t)keep * row_sz, req_bytes, &br) != FR_OK || br != req_bytes)
                {
                    lcd_set_rgb666(0U);
                    return 0U;
                }
                file_next_y = (int16_t)(file_next_y + to_read);
                buf_cnt     = (int16_t)(keep + to_read);
            }

            r0 = batch + (uint32_t)(sy0 - buf_y0) * row_sz;
            r1 = batch + (uint32_t)(sy1 - buf_y0) * row_sz;
        }
        else
        {
            int16_t targets[2] = { sy0, sy1 };
            uint8_t n_tgt      = (sy1 != sy0) ? 2U : 1U;
            for (uint8_t ti = 0U; ti < n_tgt; ti++)
            {
                int16_t t = targets[ti];
                if (den_y[t & 1] == t)
                {
                    continue;
                }
                int16_t need_h = (t + 1 < need_rows) ? (int16_t)(t + 1) : (int16_t)(need_rows - 1);
                while (h_next_y <= need_h)
                {
                    if (h_next_y >= (buf_y0 + buf_cnt))
                    {
                        pb_dma_wait();
                        int16_t remain  = (int16_t)(need_rows - file_next_y);
                        int16_t to_read = (remain > 16) ? 16 : remain;
                        if (to_read <= 0)
                        {
                            lcd_set_rgb666(0U);
                            return 0U;
                        }
                        UINT br = 0U;
                        uint32_t req_bytes = (uint32_t)to_read * row_sz;
                        if (f_read(fp, batch, req_bytes, &br) != FR_OK || br != req_bytes)
                        {
                            lcd_set_rgb666(0U);
                            return 0U;
                        }
                        buf_y0      = file_next_y;
                        file_next_y = (int16_t)(file_next_y + to_read);
                        buf_cnt     = to_read;
                    }
                    uint8_t hs = (uint8_t)(h_next_y % 3);
                    pb_thumb_nr_h_row(batch + (uint32_t)(h_next_y - buf_y0) * row_sz,
                                      w_tc, w_ty_mid, w_ty_sh, thumb_has_luma,
                                      ty_h[hs], tcb_h[hs], tcr_h[hs]);
                    h_next_y++;
                }
                int16_t t_up = (t > 0) ? (int16_t)(t - 1) : 0;
                int16_t t_dn = (t + 1 < need_rows) ? (int16_t)(t + 1) : (int16_t)(need_rows - 1);
                uint8_t su   = (uint8_t)(t_up % 3);
                uint8_t sc_i = (uint8_t)(t    % 3);
                uint8_t sd   = (uint8_t)(t_dn % 3);
                pb_thumb_nr_v_row(w_tc, w_ty_mid, w_ty_sh, thumb_has_luma,
                                  ty_h[su],  ty_h[sc_i],  ty_h[sd],
                                  tcb_h[su], tcb_h[sc_i], tcb_h[sd],
                                  tcr_h[su], tcr_h[sc_i], tcr_h[sd],
                                  den_rgb[t & 1]);
                den_y[t & 1] = t;
            }
            r0 = den_rgb[sy0 & 1];
            r1 = den_rgb[sy1 & 1];
        }

        uint8_t *out = dma_buf[buf_idx];

        /* dma_buf[buf_idx] is already idle (prev row's pb_dma_send waited on it before launching the other half) */
        for (uint16_t sx = 0U; sx < C3M1_DNG_THUMB_W; sx++)
        {
            uint32_t o0 = (uint32_t)sx * 3U;
            uint32_t o1 = (sx + 1U < C3M1_DNG_THUMB_W) ? (o0 + 3U) : o0;

            uint8_t rv0 = (uint8_t)(((uint16_t)r0[o0 + 0U] + r1[o0 + 0U] + 1U) >> 1);
            uint8_t gv0 = (uint8_t)(((uint16_t)r0[o0 + 1U] + r1[o0 + 1U] + 1U) >> 1);
            uint8_t bv0 = (uint8_t)(((uint16_t)r0[o0 + 2U] + r1[o0 + 2U] + 1U) >> 1);

            uint8_t rv1 = (uint8_t)(((uint16_t)r0[o1 + 0U] + r1[o1 + 0U] + 1U) >> 1);
            uint8_t gv1 = (uint8_t)(((uint16_t)r0[o1 + 1U] + r1[o1 + 1U] + 1U) >> 1);
            uint8_t bv1 = (uint8_t)(((uint16_t)r0[o1 + 2U] + r1[o1 + 2U] + 1U) >> 1);

            uint16_t dx = (uint16_t)(sx << 1);
            uint8_t rm = (uint8_t)(((uint16_t)rv0 + rv1 + 1U) >> 1);
            uint8_t gm = (uint8_t)(((uint16_t)gv0 + gv1 + 1U) >> 1);
            uint8_t bm = (uint8_t)(((uint16_t)bv0 + bv1 + 1U) >> 1);
            if (clip_en)
            {
                pb_put_rgb666_1x_clip(out,      rv0, gv0, bv0, dx, dy);
                pb_put_rgb666_1x_clip(out + 3U, rm,  gm,  bm,  (uint16_t)(dx + 1U), dy);
            }
            else
            {
                pb_put_rgb666_dither(out,      rv0, gv0, bv0, dx, dy);
                pb_put_rgb666_dither(out + 3U, rm,  gm,  bm,  (uint16_t)(dx + 1U), dy);
            }
            out += 6U;
        }
        if (dy >= PB_BAR_Y && bar_buf != NULL)
        {
            pb_overlay_bar_rgb666(dma_buf[buf_idx], dy, bar_buf);
        }
        pb_dma_send(dma_buf[buf_idx], (uint16_t)(PB_SCR_W * 3U));
        buf_idx ^= 1U;
    }
    pb_dma_wait();
    lcd_set_rgb666(0U);
    return 1U;
}

/* Fallback 1X path for legacy/external DNGs without embedded preview (18-bit RGB666 + dither + B&W/HDR) */
static uint8_t __attribute__((optimize("O2"))) dng_render_1x_raw(FIL *fp, const dng_info_t *info,
                                                                  const uint16_t *lin_tbl,
                                                                  const uint8_t *srgb_lut,
                                                                  uint8_t clip_en,
                                                                  uint8_t *scratch_after_idx,
                                                                  const uint8_t (*bar_buf)[160])
{
    uint32_t bpp        = (info->raw_bps == 16U) ? 2U : 1U;
    uint32_t row_bytes  = (uint32_t)info->raw_w * bpp;
    if (row_bytes > 1300U) return 0U;

    uint8_t *pair_buf = scratch_after_idx + 512U; /* Preserve lin_tbl at +0..511 */
    uint16_t bl_g  = (uint16_t)(((uint32_t)info->bl4[0] + info->bl4[3] + 1UL) >> 1);
    uint16_t bl_r  = info->bl4[1];
    uint16_t bl_b  = info->bl4[2];
    const uint8_t  is_hdr16 = (info->raw_bps == 16U && info->has_bev && info->baseline_ev_x100 >= 0) ? 1U : 0U;
    uint16_t       wl       = info->white_level;
    uint32_t       ref_white = 1UL;
    if (is_hdr16)
    {
        uint32_t hdr_gain = pb_ev_x100_to_gain_q8(info->baseline_ev_x100);
        ref_white = (uint32_t)(((uint64_t)wl * 256ULL + (hdr_gain >> 1)) / hdr_gain);
        if (ref_white < 1UL) ref_white = 1UL;
    }
    else if (info->raw_bps == 16U && wl != 4095U && wl > 0U)
    {
        bl_g = (uint16_t)(((uint32_t)bl_g * 4095UL) / wl);
        bl_r = (uint16_t)(((uint32_t)bl_r * 4095UL) / wl);
        bl_b = (uint16_t)(((uint32_t)bl_b * 4095UL) / wl);
    }

    const uint8_t        bw_mode = info->color_mode;
    const uint8_t *const bw_tone = c3m1_color_bw_tone_lut(bw_mode);
    uint16_t wbr_n, wbg_n, wbb_n;
    pb_norm_wb_q8(info->wb_r_q8, info->wb_b_q8, &wbr_n, &wbg_n, &wbb_n);

    lcd_set_rgb666(1U);
    lcd_set_window(0U, 0U, PB_SCR_W, PB_VIEW_H);
    lcd_write_ram_prepare();

    int32_t last_pair_y = -1;
    for (uint16_t dy = 0U; dy < PB_VIEW_H; dy++)
    {
        uint16_t sy = (uint16_t)((dy * 2U) & ~1U);
        if (sy + 1U >= info->raw_h) sy = (uint16_t)((info->raw_h - 2U) & ~1U);

        if ((int32_t)sy != last_pair_y)
        {
            UINT br = 0U;
            if (f_lseek(fp, info->raw_off + (uint32_t)sy * row_bytes) != FR_OK ||
                f_read(fp, pair_buf, row_bytes * 2U, &br) != FR_OK ||
                br != row_bytes * 2U)
            {
                lcd_set_rgb666(0U);
                return 0U;
            }
            last_pair_y = (int32_t)sy;
        }

        const uint8_t *even_row = pair_buf;
        const uint8_t *odd_row  = pair_buf + row_bytes;

        for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
        {
            uint16_t sx = (uint16_t)(dx * 2U);
            if (sx + 1U >= info->raw_w) sx = (uint16_t)((info->raw_w - 2U) & ~1U);

            uint8_t r8, g8, b8;
            if (is_hdr16)
            {
                uint32_t g0 = rd16(even_row, (uint32_t)sx * 2UL);
                uint32_t r0 = rd16(even_row, (uint32_t)(sx + 1U) * 2UL);
                uint32_t b0 = rd16(odd_row,  (uint32_t)sx * 2UL);
                uint32_t g1 = rd16(odd_row,  (uint32_t)(sx + 1U) * 2UL);
                uint16_t r16 = (r0 > bl_r) ? (uint16_t)(r0 - bl_r) : 0U;
                uint32_t gm  = (g0 + g1 + 1UL) >> 1;
                uint16_t g16 = (gm > bl_g) ? (uint16_t)(gm - bl_g) : 0U;
                uint16_t b16 = (b0 > bl_b) ? (uint16_t)(b0 - bl_b) : 0U;
                if (bw_tone != NULL)
                {
                    uint8_t y8 = c3m1_color_hdr_to_bw_y8(r16, g16, b16, info->wb_r_q8, info->wb_b_q8,
                                                         ref_white, (uint32_t)wl, bw_mode, srgb_lut);
                    r8 = bw_tone[y8]; g8 = r8; b8 = r8;
                }
                else
                {
                    c3m1_color_hdr_to_srgb8(r16, g16, b16, info->wb_r_q8, info->wb_b_q8,
                                            ref_white, (uint32_t)wl, srgb_lut, &r8, &g8, &b8);
                }
            }
            else
            {
                uint16_t g0 = dng_sample_12(even_row, sx,      info->raw_bps, lin_tbl, wl);
                uint16_t r0 = dng_sample_12(even_row, sx + 1U, info->raw_bps, lin_tbl, wl);
                uint16_t b0 = dng_sample_12(odd_row,  sx,      info->raw_bps, lin_tbl, wl);
                uint16_t g1 = dng_sample_12(odd_row,  sx + 1U, info->raw_bps, lin_tbl, wl);

                uint16_t r12 = sub_bl12(r0, bl_r);
                uint16_t g12 = sub_bl12(((uint32_t)g0 + g1 + 1U) >> 1, bl_g);
                uint16_t b12 = sub_bl12(b0, bl_b);
                if (bw_tone != NULL)
                {
                    uint8_t y8 = pb_color_bw_cam12_to_y8_norm(r12, g12, b12, wbr_n, wbg_n, wbb_n, bw_mode, srgb_lut);
                    r8 = bw_tone[y8]; g8 = r8; b8 = r8;
                }
                else
                {
                    pb_color_cam12_to_srgb8_norm(r12, g12, b12, wbr_n, wbg_n, wbb_n, srgb_lut, &r8, &g8, &b8);
                }
            }
            if (dy >= PB_BAR_Y && bar_buf != NULL)
            {
                pb_wr_rgb666_bar_px(r8, g8, b8, dx, dy, clip_en, bar_buf);
            }
            else if (clip_en)
            {
                pb_wr_rgb666_1x_clip(r8, g8, b8, dx, dy);
            }
            else
            {
                pb_wr_rgb666_dither(r8, g8, b8, dx, dy);
            }
        }
    }
    lcd_set_rgb666(0U);
    return 1U;
}

/* INFO-mode native 1:1 blit of 160x120 DNG embedded preview at top-left + CLIP hatch + YRGB histogram */
static uint8_t __attribute__((optimize("O2"))) dng_render_info_thumb(FIL *fp, uint32_t thumb_off,
                                                                      uint8_t *scratch_after_idx,
                                                                      pb_hist_t *hist)
{
    const uint32_t row_sz = C3M1_DNG_THUMB_W * 3UL; /* 480 B */
    uint8_t *batch_buf    = scratch_after_idx;      /* 16 x 480 B = 7,680 B */
    if (f_lseek(fp, thumb_off) != FR_OK) return 0U;

    lcd_set_rgb666(1U);
    lcd_set_window(PB_INFO_TH_X, PB_INFO_TH_Y, PB_INFO_TH_W, PB_INFO_TH_H);
    lcd_write_ram_prepare();

    uint16_t dy = 0U;
    while (dy < PB_INFO_TH_H)
    {
        uint16_t rows      = ((uint16_t)(PB_INFO_TH_H - dy) > 16U) ? 16U : (uint16_t)(PB_INFO_TH_H - dy);
        uint32_t req_bytes = (uint32_t)rows * row_sz;
        UINT br = 0U;
        if (f_read(fp, batch_buf, req_bytes, &br) != FR_OK || br != req_bytes)
        {
            lcd_set_rgb666(0U);
            return 0U;
        }
        for (uint16_t r = 0U; r < rows; r++, dy++)
        {
            const uint8_t *row_buf = batch_buf + (uint32_t)r * row_sz;
            for (uint16_t dx = 0U; dx < PB_INFO_TH_W; dx++)
            {
                uint32_t o = (uint32_t)dx * 3U;
                uint8_t r8 = row_buf[o + 0U];
                uint8_t g8 = row_buf[o + 1U];
                uint8_t b8 = row_buf[o + 2U];
                pb_hist_add_rgb8(hist, r8, g8, b8);
                pb_wr_rgb666_1x_clip(r8, g8, b8, dx, dy);
            }
        }
    }
    lcd_set_rgb666(0U);
    return 1U;
}

/* INFO-mode fallback for legacy/external DNGs without embedded preview: 160x120 RAW + CLIP + YRGB hist + B&W/HDR */
static uint8_t __attribute__((optimize("O2"))) dng_render_info_raw(FIL *fp, const dng_info_t *info,
                                                                    const uint16_t *lin_tbl,
                                                                    const uint8_t *srgb_lut,
                                                                    uint8_t *scratch_after_idx,
                                                                    pb_hist_t *hist)
{
    uint32_t bpp       = (info->raw_bps == 16U) ? 2U : 1U;
    uint32_t row_bytes = (uint32_t)info->raw_w * bpp;
    if (row_bytes > 1300U) return 0U;

    uint8_t *pair_buf = scratch_after_idx + 512U; /* Preserve lin_tbl at +0..511 */
    uint16_t bl_g  = (uint16_t)(((uint32_t)info->bl4[0] + info->bl4[3] + 1UL) >> 1);
    uint16_t bl_r  = info->bl4[1];
    uint16_t bl_b  = info->bl4[2];
    const uint8_t  is_hdr16 = (info->raw_bps == 16U && info->has_bev && info->baseline_ev_x100 >= 0) ? 1U : 0U;
    uint16_t       wl       = info->white_level;
    uint32_t       ref_white = 1UL;
    if (is_hdr16)
    {
        uint32_t hdr_gain = pb_ev_x100_to_gain_q8(info->baseline_ev_x100);
        ref_white = (uint32_t)(((uint64_t)wl * 256ULL + (hdr_gain >> 1)) / hdr_gain);
        if (ref_white < 1UL) ref_white = 1UL;
    }
    else if (info->raw_bps == 16U && wl != 4095U && wl > 0U)
    {
        bl_g = (uint16_t)(((uint32_t)bl_g * 4095UL) / wl);
        bl_r = (uint16_t)(((uint32_t)bl_r * 4095UL) / wl);
        bl_b = (uint16_t)(((uint32_t)bl_b * 4095UL) / wl);
    }

    const uint8_t        bw_mode = info->color_mode;
    const uint8_t *const bw_tone = c3m1_color_bw_tone_lut(bw_mode);
    uint16_t wbr_n, wbg_n, wbb_n;
    pb_norm_wb_q8(info->wb_r_q8, info->wb_b_q8, &wbr_n, &wbg_n, &wbb_n);

    lcd_set_rgb666(1U);
    lcd_set_window(PB_INFO_TH_X, PB_INFO_TH_Y, PB_INFO_TH_W, PB_INFO_TH_H);
    lcd_write_ram_prepare();

    for (uint16_t dy = 0U; dy < PB_INFO_TH_H; dy++)
    {
        uint16_t sy = (uint16_t)(((uint32_t)dy * info->raw_h / PB_INFO_TH_H) & ~1UL);
        if (sy + 1U >= info->raw_h) sy = (uint16_t)((info->raw_h - 2U) & ~1U);

        UINT br = 0U;
        if (f_lseek(fp, info->raw_off + (uint32_t)sy * row_bytes) != FR_OK ||
            f_read(fp, pair_buf, row_bytes * 2U, &br) != FR_OK ||
            br != row_bytes * 2U)
        {
            lcd_set_rgb666(0U);
            return 0U;
        }

        const uint8_t *even_row = pair_buf;
        const uint8_t *odd_row  = pair_buf + row_bytes;

        for (uint16_t dx = 0U; dx < PB_INFO_TH_W; dx++)
        {
            uint16_t sx = (uint16_t)(((uint32_t)dx * info->raw_w / PB_INFO_TH_W) & ~1UL);
            if (sx + 1U >= info->raw_w) sx = (uint16_t)((info->raw_w - 2U) & ~1U);

            uint8_t r8, g8, b8;
            if (is_hdr16)
            {
                uint32_t g0 = rd16(even_row, (uint32_t)sx * 2UL);
                uint32_t r0 = rd16(even_row, (uint32_t)(sx + 1U) * 2UL);
                uint32_t b0 = rd16(odd_row,  (uint32_t)sx * 2UL);
                uint32_t g1 = rd16(odd_row,  (uint32_t)(sx + 1U) * 2UL);
                uint16_t r16 = (r0 > bl_r) ? (uint16_t)(r0 - bl_r) : 0U;
                uint32_t gm  = (g0 + g1 + 1UL) >> 1;
                uint16_t g16 = (gm > bl_g) ? (uint16_t)(gm - bl_g) : 0U;
                uint16_t b16 = (b0 > bl_b) ? (uint16_t)(b0 - bl_b) : 0U;
                if (bw_tone != NULL)
                {
                    uint8_t y8 = c3m1_color_hdr_to_bw_y8(r16, g16, b16, info->wb_r_q8, info->wb_b_q8,
                                                         ref_white, (uint32_t)wl, bw_mode, srgb_lut);
                    r8 = bw_tone[y8]; g8 = r8; b8 = r8;
                }
                else
                {
                    c3m1_color_hdr_to_srgb8(r16, g16, b16, info->wb_r_q8, info->wb_b_q8,
                                            ref_white, (uint32_t)wl, srgb_lut, &r8, &g8, &b8);
                }
            }
            else
            {
                uint16_t g0 = dng_sample_12(even_row, sx,      info->raw_bps, lin_tbl, wl);
                uint16_t r0 = dng_sample_12(even_row, sx + 1U, info->raw_bps, lin_tbl, wl);
                uint16_t b0 = dng_sample_12(odd_row,  sx,      info->raw_bps, lin_tbl, wl);
                uint16_t g1 = dng_sample_12(odd_row,  sx + 1U, info->raw_bps, lin_tbl, wl);

                uint16_t r12 = sub_bl12(r0, bl_r);
                uint16_t g12 = sub_bl12(((uint32_t)g0 + g1 + 1U) >> 1, bl_g);
                uint16_t b12 = sub_bl12(b0, bl_b);
                if (bw_tone != NULL)
                {
                    uint8_t y8 = pb_color_bw_cam12_to_y8_norm(r12, g12, b12, wbr_n, wbg_n, wbb_n, bw_mode, srgb_lut);
                    r8 = bw_tone[y8]; g8 = r8; b8 = r8;
                }
                else
                {
                    pb_color_cam12_to_srgb8_norm(r12, g12, b12, wbr_n, wbg_n, wbb_n, srgb_lut, &r8, &g8, &b8);
                }
            }
            pb_hist_add_rgb8(hist, r8, g8, b8);
            pb_wr_rgb666_1x_clip(r8, g8, b8, dx, dy);
        }
    }
    lcd_set_rgb666(0U);
    return 1U;
}

/* Exact fast y % 35 for y in [0..480] (zero hardware udiv) */
static inline __attribute__((always_inline)) uint32_t pb_mod35(uint32_t y)
{
    return y - ((y * 1873UL) >> 16) * 35UL;
}

static inline void pb_warp_knot_vga(int32_t dx, uint32_t dy2, int32_t dy,
                                    int32_t kr0_q16, int32_t kr1_q16,
                                    int32_t *out_sx_q16, int32_t *out_sy_q16)
{
    uint32_t d2     = (uint32_t)(dx * dx) + dy2;
    uint32_t r2_q16 = (d2 * 13422UL) >> 15;
    int32_t  s_q16  = kr0_q16 + ((kr1_q16 * (int32_t)r2_q16) >> 16);
    int32_t  sx     = (320L << 16) + dx * s_q16;
    int32_t  sy     = (240L << 16) + dy * s_q16;
    if (sx < 0L)           sx = 0L;
    if (sx > (639L << 16)) sx = (639L << 16);
    if (sy < 0L)           sy = 0L;
    if (sy > (479L << 16)) sy = (479L << 16);
    *out_sx_q16 = sx;
    *out_sy_q16 = sy;
}

/* Warped 1:1 RAW crop zoom renderer (honors DNG OpcodeList3 WarpRectilinear + FixVignetteRadial,
 * 35-row cropped circular ring + optional High-ISO NR + DMA1 18-bit RGB666 + 2x2 dither). */
static uint8_t __attribute__((noinline, optimize("O2"))) dng_render_zoom_raw_warped(FIL *fp, const dng_info_t *info,
                                                                                     const uint16_t *lin_tbl,
                                                                                     const uint8_t *srgb_lut,
                                                                                     uint16_t pan_x, uint16_t pan_y,
                                                                                     uint16_t nr_iso,
                                                                                     uint8_t *scratch_after_idx,
                                                                                     uint8_t *work_buf,
                                                                                     uint16_t work_buf_size,
                                                                                     const uint8_t (*bar_buf)[160])
{
    const uint32_t bpp       = (info->raw_bps == 16U) ? 2U : 1U;
    const uint32_t row_bytes = (uint32_t)info->raw_w * bpp;
    const uint16_t raw_w     = info->raw_w;
    const uint16_t raw_h     = info->raw_h;
    const int32_t  kr0       = info->warp_kr0_q16;
    const int32_t  kr1       = info->warp_kr1_q16;

    /* Compute global source X/Y bounds over the 320x220 viewport from the 3x3 monotonic extrema.
     * Note: ex_x[2] = pan_x + 320 matches knot k=20 right endpoint so req_max never undershoots by 1 row. */
    uint16_t ex_x[3] = { pan_x,
                         (pan_x > 320U) ? pan_x : ((pan_x + 320U < 320U) ? (uint16_t)(pan_x + 320U) : 320U),
                         (uint16_t)(pan_x + 320U) };
    uint16_t ex_y[3] = { pan_y,
                         (pan_y > 240U) ? pan_y : ((pan_y + PB_VIEW_H - 1U < 240U) ? (uint16_t)(pan_y + PB_VIEW_H - 1U) : 240U),
                         (uint16_t)(pan_y + PB_VIEW_H - 1U) };
    int32_t min_sx = 639L, max_sx = 0L, min_sy = 479L, max_sy = 0L;
    for (uint8_t iy = 0U; iy < 3U; iy++)
    {
        int32_t  dy  = (int32_t)ex_y[iy] - 240L;
        uint32_t dy2 = (uint32_t)(dy * dy);
        int32_t  row_lo = 479L, row_hi = 0L;
        for (uint8_t ix = 0U; ix < 3U; ix++)
        {
            int32_t sx_q16, sy_q16;
            pb_warp_knot_vga((int32_t)ex_x[ix] - 320L, dy2, dy, kr0, kr1, &sx_q16, &sy_q16);
            int32_t sx = (sx_q16 + 32768L) >> 16;
            int32_t sy = (sy_q16 + 32768L) >> 16;
            if (sx < min_sx) min_sx = sx;
            if (sx > max_sx) max_sx = sx;
            if (sy < min_sy) min_sy = sy;
            if (sy > max_sy) max_sy = sy;
            if (sy < row_lo) row_lo = sy;
            if (sy > row_hi) row_hi = sy;
        }
        if ((row_hi - row_lo + 3L) > 35L) return 0U; /* Guard against extreme external OpcodeList3 */
    }

    int32_t crop_x0   = (min_sx > 2L) ? (min_sx - 2L) : 0L;
    int32_t crop_x1   = (max_sx + 2L < (int32_t)raw_w) ? (max_sx + 2L) : ((int32_t)raw_w - 1L);
    uint32_t crop_w   = (uint32_t)(crop_x1 - crop_x0 + 1L);
    if (crop_w > 340UL) return 0U; /* Fallback safety guard */

    int32_t first_row = (min_sy > 1L) ? (min_sy - 1L) : 0L;
    int32_t last_row  = (max_sy + 1L < (int32_t)raw_h) ? (max_sy + 1L) : ((int32_t)raw_h - 1L);
    if (f_lseek(fp, info->raw_off + (uint32_t)first_row * row_bytes) != FR_OK) return 0U;

    const uint8_t        bw_mode = info->color_mode;
    const uint8_t *const bw_tone = c3m1_color_bw_tone_lut(bw_mode);

    pb_zoom_nr_t znr;
    if (bw_tone != NULL)
    {
        pb_nr_init_params_bw(&znr.nr, nr_iso);
    }
    else
    {
        pb_nr_init_params(&znr.nr, nr_iso);
    }
    uint8_t nr_on = (znr.nr.lvl != 0U) ? 1U : 0U;

    /* Memory layout:
     *   work_buf[0..1919]             = dma_buf[2] (dual 960 B 18-bit RGB666)
     *   work_buf[1920..3219]          = sd_stage (1,300 B, for 16-bit rows)
     *   work_buf[3220..3859]          = optional 640 B y_h[3..4] spillover (only 16-bit warped NR with crop_w > 334)
     *   scratch_after_idx[0..511]     = lin_tbl (512 B, preserved for 8-bit; unused for 16-bit)
     *   scratch_after_idx[ring..]     = 35-row cropped ring (<= 11,900 B 8-bit, <= 23,800 B 16-bit)
     *   scratch_after_idx[batch..]    = 16-row SD batch for 8-bit (<= 10,304 B) + 4,000 B NR rings */
    uint8_t       *dma_buf[2]   = { work_buf, work_buf + (PB_SCR_W * 3U) };
    const uint8_t  is_8bit      = (info->raw_bps == 8U) ? 1U : 0U;
    uint8_t       *ring_base    = is_8bit ? (scratch_after_idx + 512U) : scratch_after_idx;
    const uint32_t crop_stride  = crop_w * bpp;
    const uint32_t crop_x0_off  = (uint32_t)crop_x0 * bpp;
    int32_t        sd_batch_cap = is_8bit ? 16 : 1;
    uint8_t       *sd_batch     = is_8bit ? (ring_base + 35UL * crop_stride) : (work_buf + (PB_SCR_W * 6U));

    if (nr_on)
    {
        znr.dma_buf[0] = dma_buf[0];
        znr.dma_buf[1] = dma_buf[1];
        znr.bar_buf    = bar_buf;
        znr.buf_idx    = 0U;
        if (is_8bit)
        {
            uint32_t nr_off = (512UL + 35UL * crop_stride + (uint32_t)sd_batch_cap * row_bytes + 3UL) & ~3UL;
            if (nr_off + PB_ZOOM_NR_TOTAL_BYTES <= PB_AFTER_IDX_NEEDED)
            {
                uint8_t *nr_ptr = scratch_after_idx + nr_off;
                znr.chroma = (pb_chroma_row_t *)nr_ptr;
                nr_ptr    += PB_ZOOM_NR_CHROMA_BYTES;
                for (uint8_t r = 0U; r < 5U; r++)
                {
                    znr.y_h[r] = (int8_t *)(nr_ptr + (uint32_t)r * PB_SCR_W);
                }
            }
            else
            {
                nr_on = 0U;
            }
        }
        else
        {
            uint32_t ring_end = (35UL * crop_stride + 3UL) & ~3UL;
            if (ring_end + PB_ZOOM_NR_TOTAL_BYTES <= PB_AFTER_IDX_NEEDED)
            {
                uint8_t *nr_ptr = scratch_after_idx + ring_end;
                znr.chroma = (pb_chroma_row_t *)nr_ptr;
                nr_ptr    += PB_ZOOM_NR_CHROMA_BYTES;
                for (uint8_t r = 0U; r < 5U; r++)
                {
                    znr.y_h[r] = (int8_t *)(nr_ptr + (uint32_t)r * PB_SCR_W);
                }
            }
            else if (ring_end + PB_ZOOM_NR_CHROMA_BYTES + 3UL * PB_SCR_W <= PB_AFTER_IDX_NEEDED &&
                     (uint32_t)work_buf_size >= (PB_SCR_W * 6UL + 1300UL + 2UL * PB_SCR_W))
            {
                uint8_t *nr_ptr = scratch_after_idx + ring_end;
                znr.chroma = (pb_chroma_row_t *)nr_ptr;
                nr_ptr    += PB_ZOOM_NR_CHROMA_BYTES;
                znr.y_h[0] = (int8_t *)(nr_ptr);
                znr.y_h[1] = (int8_t *)(nr_ptr + PB_SCR_W);
                znr.y_h[2] = (int8_t *)(nr_ptr + 2UL * PB_SCR_W);
                uint8_t *wb_nr = work_buf + (PB_SCR_W * 6U) + 1300U;
                znr.y_h[3] = (int8_t *)(wb_nr);
                znr.y_h[4] = (int8_t *)(wb_nr + PB_SCR_W);
            }
            else
            {
                nr_on = 0U;
            }
        }
    }
    const uint8_t raw_imp_thr = (is_8bit && nr_on) ? znr.nr.raw_imp_thr : 0U;

    uint16_t bl_g0 = info->bl4[0];
    uint16_t bl_r  = info->bl4[1];
    uint16_t bl_b  = info->bl4[2];
    uint16_t bl_g1 = info->bl4[3];
    if (!is_8bit && info->white_level != 4095U && info->white_level > 0U)
    {
        bl_g0 = (uint16_t)(((uint32_t)bl_g0 * 4095UL) / info->white_level);
        bl_r  = (uint16_t)(((uint32_t)bl_r  * 4095UL) / info->white_level);
        bl_b  = (uint16_t)(((uint32_t)bl_b  * 4095UL) / info->white_level);
        bl_g1 = (uint16_t)(((uint32_t)bl_g1 * 4095UL) / info->white_level);
    }
    const uint16_t bl_g_avg = (uint16_t)(((uint32_t)bl_g0 + bl_g1 + 1UL) >> 1);

    const uint16_t wb_r      = info->wb_r_q8;
    const uint16_t wb_b      = info->wb_b_q8;
    uint16_t       wbr_n, wbg_n, wbb_n;
    pb_norm_wb_q8(wb_r, wb_b, &wbr_n, &wbg_n, &wbb_n);
    const uint32_t vig_scale = info->vig_scale_q16;
    const uint8_t  is_hdr16  = (!is_8bit && info->has_bev && info->baseline_ev_x100 >= 0) ? 1U : 0U;
    uint16_t       wl        = info->white_level;
    uint32_t       ref_white = 1UL;
    if (is_hdr16)
    {
        uint32_t hdr_gain = pb_ev_x100_to_gain_q8(info->baseline_ev_x100);
        ref_white = (uint32_t)(((uint64_t)wl * 256ULL + (hdr_gain >> 1)) / hdr_gain);
        if (ref_white < 1UL) ref_white = 1UL;
    }

    int32_t file_next_y = first_row;
    uint8_t buf_idx     = 0U;

    lcd_set_rgb666(1U);
    lcd_set_window(0U, 0U, PB_SCR_W, PB_VIEW_H);
    lcd_write_ram_prepare();

    for (uint16_t dy = 0U; dy < PB_VIEW_H; dy++)
    {
        int32_t  fy  = (int32_t)pan_y + (int32_t)dy;
        int32_t  ldy = fy - 240L;
        uint32_t dy2 = (uint32_t)(ldy * ldy);

        /* Compute [req_min..req_max] source rows (including +-1 Bayer margin) and cap batch
         * read so file_next_y never exceeds req_min + 35 (prevents evicting live rows in 35-row ring). */
        int32_t req_min = last_row;
        int32_t req_max = first_row;
        for (uint8_t ix = 0U; ix < 3U; ix++)
        {
            int32_t sx_q16, sy_q16;
            pb_warp_knot_vga((int32_t)ex_x[ix] - 320L, dy2, ldy, kr0, kr1, &sx_q16, &sy_q16);
            int32_t sy_c = (sy_q16 + 32768L) >> 16;
            if (sy_c - 1L < req_min) req_min = sy_c - 1L;
            if (sy_c + 1L > req_max) req_max = sy_c + 1L;
        }
        if (req_min < first_row) req_min = first_row;
        if (req_min > last_row)  req_min = last_row;
        if (req_max < first_row) req_max = first_row;
        if (req_max > last_row)  req_max = last_row;

        while (file_next_y <= req_max)
        {
            pb_dma_wait();
            int32_t max_safe_next = req_min + 35;
            int32_t min_need_next = req_max + 1;
            int32_t cap_next      = (max_safe_next >= min_need_next) ? max_safe_next : min_need_next;
            int32_t remain        = (last_row + 1) - file_next_y;
            int32_t to_read       = (remain > sd_batch_cap) ? sd_batch_cap : remain;
            if (file_next_y + to_read > cap_next)
            {
                to_read = cap_next - file_next_y;
            }
            if (to_read <= 0)
            {
                lcd_set_rgb666(0U);
                return 0U;
            }
            UINT     br        = 0U;
            uint32_t req_bytes = (uint32_t)to_read * row_bytes;
            if (f_read(fp, sd_batch, req_bytes, &br) != FR_OK || br != req_bytes)
            {
                lcd_set_rgb666(0U);
                return 0U;
            }
            for (int32_t r = 0; r < to_read; r++)
            {
                uint32_t slot = pb_mod35((uint32_t)(file_next_y + r));
                memcpy(ring_base + slot * crop_stride,
                       sd_batch + (uint32_t)r * row_bytes + crop_x0_off,
                       crop_stride);
            }
            file_next_y += to_read;
        }

        int32_t  ring_hi = file_next_y - 1;
        int32_t  ring_lo = (file_next_y > 35) ? (file_next_y - 35) : first_row;
        if (ring_lo < first_row) ring_lo = first_row;

        uint8_t *out = nr_on ? znr.dma_buf[znr.buf_idx] : dma_buf[buf_idx];
        int32_t  sx0, sy0;
        pb_warp_knot_vga((int32_t)pan_x - 320L, dy2, ldy, kr0, kr1, &sx0, &sy0);

        for (uint16_t k = 1U; k <= 20U; k++)
        {
            int32_t sx1, sy1;
            pb_warp_knot_vga((int32_t)(pan_x + k * 16U) - 320L, dy2, ldy, kr0, kr1, &sx1, &sy1);

            int32_t dsx = (sx1 - sx0) >> 4;
            int32_t dsy = (sy1 - sy0) >> 4;
            int32_t cx  = sx0 + 32768L;
            int32_t cy  = sy0 + 32768L;

            /* Evaluate smooth radial vignette once per 16-pixel span at midpoint in source coords */
            uint32_t gq8 = 256UL;
            if (vig_scale != 0UL)
            {
                uint16_t mx = (uint16_t)((uint32_t)(cx + dsx * 8L) >> 16);
                uint16_t my = (uint16_t)((uint32_t)(cy + dsy * 8L) >> 16);
                gq8 = c3m1_lens_vig_gain_q8(mx, my, vig_scale);
            }

            uint16_t dx_base = (uint16_t)((k - 1U) * 16U);
            for (uint16_t i = 0U; i < 16U; i++)
            {
                int32_t sx = (int32_t)((uint32_t)cx >> 16);
                int32_t sy = (int32_t)((uint32_t)cy >> 16);
                cx += dsx;
                cy += dsy;

                if (sx < crop_x0) sx = crop_x0;
                if (sx > crop_x1) sx = crop_x1;
                if (sy < ring_lo) sy = ring_lo;
                if (sy > ring_hi) sy = ring_hi;

                /* Reflect at ring/crop borders to preserve exact GRBG Bayer parity */
                int32_t  y_up = (sy > ring_lo) ? (sy - 1) : ((ring_hi > ring_lo) ? (sy + 1) : sy);
                int32_t  y_dn = (sy < ring_hi) ? (sy + 1) : ((ring_hi > ring_lo) ? (sy - 1) : sy);
                uint32_t lx   = (uint32_t)(sx - crop_x0);
                uint32_t lxl  = (lx > 0UL) ? (lx - 1UL) : ((crop_w > 1UL) ? 1UL : 0UL);
                uint32_t lxr  = (lx + 1UL < crop_w) ? (lx + 1UL) : ((crop_w > 1UL) ? (crop_w - 2UL) : 0UL);

                const uint8_t *rp = ring_base + pb_mod35((uint32_t)y_up) * crop_stride;
                const uint8_t *rc = ring_base + pb_mod35((uint32_t)sy)   * crop_stride;
                const uint8_t *rn = ring_base + pb_mod35((uint32_t)y_dn) * crop_stride;

                uint8_t r8, g8, b8;
                if (is_8bit)
                {
                    uint8_t c_raw = rc[lx];
                    if (raw_imp_thr > 0U)
                    {
                        if (((sy ^ sx) & 1) == 0)
                        {
                            c_raw = pb_nr_despeckle_u8(c_raw, rp[lxl], rp[lxr], rn[lxl], rn[lxr], raw_imp_thr);
                        }
                        else if (sy - 2 >= ring_lo && sy + 2 <= ring_hi && lx >= 2UL && lx + 2UL < crop_w)
                        {
                            const uint8_t *rp2 = ring_base + pb_mod35((uint32_t)(sy - 2)) * crop_stride;
                            const uint8_t *rn2 = ring_base + pb_mod35((uint32_t)(sy + 2)) * crop_stride;
                            c_raw = pb_nr_despeckle_u8(c_raw, rc[lx - 2UL], rc[lx + 2UL], rp2[lx], rn2[lx], raw_imp_thr);
                        }
                    }

                    uint32_t r12, g12, b12;
                    if ((sy & 1) == 0)
                    {
                        if ((sx & 1) == 0)
                        {
                            g12 = lin_tbl[c_raw];
                            r12 = ((uint32_t)lin_tbl[rc[lxl]] + lin_tbl[rc[lxr]] + 1U) >> 1;
                            b12 = ((uint32_t)lin_tbl[rp[lx]]  + lin_tbl[rn[lx]]  + 1U) >> 1;
                        }
                        else
                        {
                            r12 = lin_tbl[c_raw];
                            g12 = ((uint32_t)lin_tbl[rc[lxl]] + lin_tbl[rc[lxr]] +
                                   lin_tbl[rp[lx]] + lin_tbl[rn[lx]] + 2U) >> 2;
                            b12 = ((uint32_t)lin_tbl[rp[lxl]] + lin_tbl[rp[lxr]] +
                                   lin_tbl[rn[lxl]] + lin_tbl[rn[lxr]] + 2U) >> 2;
                        }
                    }
                    else
                    {
                        if ((sx & 1) == 0)
                        {
                            b12 = lin_tbl[c_raw];
                            g12 = ((uint32_t)lin_tbl[rc[lxl]] + lin_tbl[rc[lxr]] +
                                   lin_tbl[rp[lx]] + lin_tbl[rn[lx]] + 2U) >> 2;
                            r12 = ((uint32_t)lin_tbl[rp[lxl]] + lin_tbl[rp[lxr]] +
                                   lin_tbl[rn[lxl]] + lin_tbl[rn[lxr]] + 2U) >> 2;
                        }
                        else
                        {
                            g12 = lin_tbl[c_raw];
                            b12 = ((uint32_t)lin_tbl[rc[lxl]] + lin_tbl[rc[lxr]] + 1U) >> 1;
                            r12 = ((uint32_t)lin_tbl[rp[lx]]  + lin_tbl[rn[lx]]  + 1U) >> 1;
                        }
                    }
                    uint16_t bl_g_site = (((sy & 1) == (sx & 1)) ? (((sy & 1) == 0) ? bl_g0 : bl_g1) : bl_g_avg);
                    uint32_t rc12 = sub_bl12(r12, bl_r);
                    uint32_t gc12 = sub_bl12(g12, bl_g_site);
                    uint32_t bc12 = sub_bl12(b12, bl_b);
                    if (gq8 > 257UL)
                    {
                        rc12 = (rc12 * gq8 + 128UL) >> 8; if (rc12 > 4095UL) rc12 = 4095UL;
                        gc12 = (gc12 * gq8 + 128UL) >> 8; if (gc12 > 4095UL) gc12 = 4095UL;
                        bc12 = (bc12 * gq8 + 128UL) >> 8; if (bc12 > 4095UL) bc12 = 4095UL;
                    }
                    if (bw_tone != NULL)
                    {
                        r8 = pb_color_bw_cam12_to_y8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                          wbr_n, wbg_n, wbb_n, bw_mode, srgb_lut);
                        g8 = r8;
                        b8 = r8;
                    }
                    else
                    {
                        pb_color_cam12_to_srgb8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                     wbr_n, wbg_n, wbb_n, srgb_lut, &r8, &g8, &b8);
                    }
                }
                else if (is_hdr16)
                {
                    uint32_t r16, g16, b16;
                    if ((sy & 1) == 0)
                    {
                        if ((sx & 1) == 0)
                        {
                            g16 = rd16(rc, lx * 2UL);
                            r16 = ((uint32_t)rd16(rc, lxl * 2UL) + rd16(rc, lxr * 2UL) + 1UL) >> 1;
                            b16 = ((uint32_t)rd16(rp, lx  * 2UL) + rd16(rn, lx  * 2UL) + 1UL) >> 1;
                        }
                        else
                        {
                            r16 = rd16(rc, lx * 2UL);
                            g16 = ((uint32_t)rd16(rc, lxl * 2UL) + rd16(rc, lxr * 2UL) +
                                   rd16(rp, lx * 2UL) + rd16(rn, lx * 2UL) + 2UL) >> 2;
                            b16 = ((uint32_t)rd16(rp, lxl * 2UL) + rd16(rp, lxr * 2UL) +
                                   rd16(rn, lxl * 2UL) + rd16(rn, lxr * 2UL) + 2UL) >> 2;
                        }
                    }
                    else
                    {
                        if ((sx & 1) == 0)
                        {
                            b16 = rd16(rc, lx * 2UL);
                            g16 = ((uint32_t)rd16(rc, lxl * 2UL) + rd16(rc, lxr * 2UL) +
                                   rd16(rp, lx * 2UL) + rd16(rn, lx * 2UL) + 2UL) >> 2;
                            r16 = ((uint32_t)rd16(rp, lxl * 2UL) + rd16(rp, lxr * 2UL) +
                                   rd16(rn, lxl * 2UL) + rd16(rn, lxr * 2UL) + 2UL) >> 2;
                        }
                        else
                        {
                            g16 = rd16(rc, lx * 2UL);
                            b16 = ((uint32_t)rd16(rc, lxl * 2UL) + rd16(rc, lxr * 2UL) + 1UL) >> 1;
                            r16 = ((uint32_t)rd16(rp, lx  * 2UL) + rd16(rn, lx  * 2UL) + 1UL) >> 1;
                        }
                    }
                    if (gq8 > 257UL)
                    {
                        r16 = (r16 * gq8 + 128UL) >> 8; if (r16 > wl) r16 = wl;
                        g16 = (g16 * gq8 + 128UL) >> 8; if (g16 > wl) g16 = wl;
                        b16 = (b16 * gq8 + 128UL) >> 8; if (b16 > wl) b16 = wl;
                    }
                    if (bw_tone != NULL)
                    {
                        r8 = c3m1_color_hdr_to_bw_y8((uint16_t)r16, (uint16_t)g16, (uint16_t)b16,
                                                     wb_r, wb_b, ref_white, (uint32_t)wl,
                                                     bw_mode, srgb_lut);
                        g8 = r8;
                        b8 = r8;
                    }
                    else
                    {
                        c3m1_color_hdr_to_srgb8((uint16_t)r16, (uint16_t)g16, (uint16_t)b16,
                                                wb_r, wb_b, ref_white, (uint32_t)wl,
                                                srgb_lut, &r8, &g8, &b8);
                    }
                }
                else
                {
                    uint32_t r12, g12, b12;
                    if ((sy & 1) == 0)
                    {
                        if ((sx & 1) == 0)
                        {
                            g12 = dng_sample_12(rc, (uint16_t)lx, 16U, lin_tbl, wl);
                            r12 = ((uint32_t)dng_sample_12(rc, (uint16_t)lxl, 16U, lin_tbl, wl) +
                                   dng_sample_12(rc, (uint16_t)lxr, 16U, lin_tbl, wl) + 1U) >> 1;
                            b12 = ((uint32_t)dng_sample_12(rp, (uint16_t)lx, 16U, lin_tbl, wl) +
                                   dng_sample_12(rn, (uint16_t)lx, 16U, lin_tbl, wl) + 1U) >> 1;
                        }
                        else
                        {
                            r12 = dng_sample_12(rc, (uint16_t)lx, 16U, lin_tbl, wl);
                            g12 = ((uint32_t)dng_sample_12(rc, (uint16_t)lxl, 16U, lin_tbl, wl) +
                                   dng_sample_12(rc, (uint16_t)lxr, 16U, lin_tbl, wl) +
                                   dng_sample_12(rp, (uint16_t)lx, 16U, lin_tbl, wl) +
                                   dng_sample_12(rn, (uint16_t)lx, 16U, lin_tbl, wl) + 2U) >> 2;
                            b12 = ((uint32_t)dng_sample_12(rp, (uint16_t)lxl, 16U, lin_tbl, wl) +
                                   dng_sample_12(rp, (uint16_t)lxr, 16U, lin_tbl, wl) +
                                   dng_sample_12(rn, (uint16_t)lxl, 16U, lin_tbl, wl) +
                                   dng_sample_12(rn, (uint16_t)lxr, 16U, lin_tbl, wl) + 2U) >> 2;
                        }
                    }
                    else
                    {
                        if ((sx & 1) == 0)
                        {
                            b12 = dng_sample_12(rc, (uint16_t)lx, 16U, lin_tbl, wl);
                            g12 = ((uint32_t)dng_sample_12(rc, (uint16_t)lxl, 16U, lin_tbl, wl) +
                                   dng_sample_12(rc, (uint16_t)lxr, 16U, lin_tbl, wl) +
                                   dng_sample_12(rp, (uint16_t)lx, 16U, lin_tbl, wl) +
                                   dng_sample_12(rn, (uint16_t)lx, 16U, lin_tbl, wl) + 2U) >> 2;
                            r12 = ((uint32_t)dng_sample_12(rp, (uint16_t)lxl, 16U, lin_tbl, wl) +
                                   dng_sample_12(rp, (uint16_t)lxr, 16U, lin_tbl, wl) +
                                   dng_sample_12(rn, (uint16_t)lxl, 16U, lin_tbl, wl) +
                                   dng_sample_12(rn, (uint16_t)lxr, 16U, lin_tbl, wl) + 2U) >> 2;
                        }
                        else
                        {
                            g12 = dng_sample_12(rc, (uint16_t)lx, 16U, lin_tbl, wl);
                            b12 = ((uint32_t)dng_sample_12(rc, (uint16_t)lxl, 16U, lin_tbl, wl) +
                                   dng_sample_12(rc, (uint16_t)lxr, 16U, lin_tbl, wl) + 1U) >> 1;
                            r12 = ((uint32_t)dng_sample_12(rp, (uint16_t)lx, 16U, lin_tbl, wl) +
                                   dng_sample_12(rn, (uint16_t)lx, 16U, lin_tbl, wl) + 1U) >> 1;
                        }
                    }
                    uint16_t bl_g_site = (((sy & 1) == (sx & 1)) ? (((sy & 1) == 0) ? bl_g0 : bl_g1) : bl_g_avg);
                    uint32_t rc12 = sub_bl12(r12, bl_r);
                    uint32_t gc12 = sub_bl12(g12, bl_g_site);
                    uint32_t bc12 = sub_bl12(b12, bl_b);
                    if (gq8 > 257UL)
                    {
                        rc12 = (rc12 * gq8 + 128UL) >> 8; if (rc12 > 4095UL) rc12 = 4095UL;
                        gc12 = (gc12 * gq8 + 128UL) >> 8; if (gc12 > 4095UL) gc12 = 4095UL;
                        bc12 = (bc12 * gq8 + 128UL) >> 8; if (bc12 > 4095UL) bc12 = 4095UL;
                    }
                    if (bw_tone != NULL)
                    {
                        r8 = pb_color_bw_cam12_to_y8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                          wbr_n, wbg_n, wbb_n, bw_mode, srgb_lut);
                        g8 = r8;
                        b8 = r8;
                    }
                    else
                    {
                        pb_color_cam12_to_srgb8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                     wbr_n, wbg_n, wbb_n, srgb_lut, &r8, &g8, &b8);
                    }
                }
                if (nr_on)
                {
                    if (bw_tone != NULL)
                    {
                        znr.tmp_y0[dx_base + i] = (int8_t)((int32_t)r8 - 128);
                    }
                    else
                    {
                        out[0] = r8;
                        out[1] = g8;
                        out[2] = b8;
                    }
                }
                else if (bw_tone != NULL)
                {
                    uint8_t yt = bw_tone[r8];
                    pb_put_rgb666_dither(out, yt, yt, yt, (uint16_t)(dx_base + i), dy);
                }
                else
                {
                    pb_put_rgb666_dither(out, r8, g8, b8, (uint16_t)(dx_base + i), dy);
                }
                out += 3U;
            }
            sx0 = sx1;
            sy0 = sy1;
        }

        if (nr_on)
        {
            if (bw_tone != NULL)
            {
                pb_zoom_nr_push_bw_row(&znr, dy, bw_tone);
            }
            else
            {
                pb_zoom_nr_push_row(&znr, dy, znr.dma_buf[znr.buf_idx]);
            }
        }
        else
        {
            if (dy >= PB_BAR_Y && bar_buf != NULL)
            {
                pb_overlay_bar_rgb666(dma_buf[buf_idx], dy, bar_buf);
            }
            pb_dma_send(dma_buf[buf_idx], (uint16_t)(PB_SCR_W * 3U));
            buf_idx ^= 1U;
        }
    }
    if (nr_on)
    {
        if (bw_tone != NULL)
        {
            pb_zoom_nr_flush_bw(&znr, bw_tone);
        }
        else
        {
            pb_zoom_nr_flush(&znr);
        }
    }
    pb_dma_wait();
    lcd_set_rgb666(0U);
    return 1U;
}

/* Unwarped 1:1 RAW crop zoom renderer (batch SD read + optional High-ISO NR + DMA1 18-bit RGB666) */
static uint8_t __attribute__((noinline, optimize("O2"))) dng_render_zoom_raw_unwarped(FIL *fp, const dng_info_t *info,
                                                                                       const uint16_t *lin_tbl,
                                                                                       const uint8_t *srgb_lut,
                                                                                       uint16_t pan_x, uint16_t pan_y,
                                                                                       uint16_t nr_iso,
                                                                                       uint8_t *scratch_after_idx,
                                                                                       uint8_t *work_buf,
                                                                                       const uint8_t (*bar_buf)[160])
{
    const uint32_t bpp       = (info->raw_bps == 16U) ? 2U : 1U;
    const uint32_t row_bytes = (uint32_t)info->raw_w * bpp;
    const uint8_t  is_8bit   = (info->raw_bps == 8U) ? 1U : 0U;

    const uint8_t        bw_mode = info->color_mode;
    const uint8_t *const bw_tone = c3m1_color_bw_tone_lut(bw_mode);

    pb_zoom_nr_t znr;
    if (bw_tone != NULL)
    {
        pb_nr_init_params_bw(&znr.nr, nr_iso);
    }
    else
    {
        pb_nr_init_params(&znr.nr, nr_iso);
    }
    uint8_t nr_on         = (znr.nr.lvl != 0U) ? 1U : 0U;
    uint8_t use_despeckle = (is_8bit && nr_on && znr.nr.raw_imp_thr > 0U) ? 1U : 0U;

    /* lin_tbl lives at scratch_after_idx + 0..511; use work_buf for dual 960 B DMA + +512 for batch */
    uint8_t *dma_buf[2] = { work_buf, work_buf + (PB_SCR_W * 3U) };
    uint8_t *batch      = scratch_after_idx + 512U;
    int32_t  keep_rows  = use_despeckle ? 4 : 2;
    int32_t  max_batch  = is_8bit ? (use_despeckle ? 14 : 16) : (nr_on ? 12 : 16);
    uint8_t  buf_idx    = 0U;
    const uint32_t vig_scale = info->vig_scale_q16;

    if (nr_on)
    {
        uint32_t nr_off = (512UL + (uint32_t)(keep_rows + max_batch) * row_bytes + 3UL) & ~3UL;
        if (nr_off + PB_ZOOM_NR_TOTAL_BYTES <= PB_AFTER_IDX_NEEDED)
        {
            uint8_t *nr_ptr = scratch_after_idx + nr_off;
            znr.chroma = (pb_chroma_row_t *)nr_ptr;
            nr_ptr    += PB_ZOOM_NR_CHROMA_BYTES;
            for (uint8_t r = 0U; r < 5U; r++)
            {
                znr.y_h[r] = (int8_t *)(nr_ptr + (uint32_t)r * PB_SCR_W);
            }
            znr.dma_buf[0] = dma_buf[0];
            znr.dma_buf[1] = dma_buf[1];
            znr.bar_buf    = bar_buf;
            znr.buf_idx    = 0U;
        }
        else
        {
            nr_on         = 0U;
            use_despeckle = 0U;
            keep_rows     = 2;
            max_batch     = 16;
        }
    }
    const uint8_t raw_imp_thr = use_despeckle ? znr.nr.raw_imp_thr : 0U;

    uint16_t bl_g0 = info->bl4[0];
    uint16_t bl_r  = info->bl4[1];
    uint16_t bl_b  = info->bl4[2];
    uint16_t bl_g1 = info->bl4[3];
    if (info->raw_bps == 16U && info->white_level != 4095U && info->white_level > 0U)
    {
        bl_g0 = (uint16_t)(((uint32_t)bl_g0 * 4095UL) / info->white_level);
        bl_r  = (uint16_t)(((uint32_t)bl_r  * 4095UL) / info->white_level);
        bl_b  = (uint16_t)(((uint32_t)bl_b  * 4095UL) / info->white_level);
        bl_g1 = (uint16_t)(((uint32_t)bl_g1 * 4095UL) / info->white_level);
    }
    const uint16_t bl_g_avg = (uint16_t)(((uint32_t)bl_g0 + bl_g1 + 1UL) >> 1);

    int32_t margin    = use_despeckle ? 2 : 1;
    int32_t first_row = ((int32_t)pan_y >= margin) ? ((int32_t)pan_y - margin) : 0;
    int32_t last_row  = ((int32_t)pan_y + (int32_t)PB_VIEW_H - 1 + margin < (int32_t)info->raw_h)
                      ? ((int32_t)pan_y + (int32_t)PB_VIEW_H - 1 + margin)
                      : ((int32_t)info->raw_h - 1);
    if (f_lseek(fp, info->raw_off + (uint32_t)first_row * row_bytes) != FR_OK) return 0U;
    int32_t file_next_y = first_row;
    int32_t buf_y0      = first_row;
    int32_t buf_cnt     = 0;

    const uint16_t raw_w   = info->raw_w;
    const uint16_t wb_r    = info->wb_r_q8;
    const uint16_t wb_b    = info->wb_b_q8;
    uint16_t       wbr_n, wbg_n, wbb_n;
    pb_norm_wb_q8(wb_r, wb_b, &wbr_n, &wbg_n, &wbb_n);

    lcd_set_rgb666(1U);
    lcd_set_window(0U, 0U, PB_SCR_W, PB_VIEW_H);
    lcd_write_ram_prepare();

    for (uint16_t dy = 0U; dy < PB_VIEW_H; dy++)
    {
        int32_t sy   = (int32_t)pan_y + (int32_t)dy;
        if (sy >= (int32_t)info->raw_h) sy = (int32_t)info->raw_h - 1;
        int32_t y_up  = (sy > 0) ? (sy - 1) : ((info->raw_h > 1U) ? 1 : 0);
        int32_t y_dn  = (sy + 1 < (int32_t)info->raw_h) ? (sy + 1) : ((info->raw_h > 1U) ? ((int32_t)info->raw_h - 2) : 0);
        int32_t y_up2 = use_despeckle ? ((sy >= 2) ? (sy - 2) : ((sy + 2 < (int32_t)info->raw_h) ? (sy + 2) : sy)) : sy;
        int32_t y_dn2 = use_despeckle ? ((sy + 2 < (int32_t)info->raw_h) ? (sy + 2) : ((sy >= 2) ? (sy - 2) : sy)) : sy;

        int32_t y_need_hi = (sy > y_up) ? sy : y_up;
        if (y_dn  > y_need_hi) y_need_hi = y_dn;
        if (y_up2 > y_need_hi) y_need_hi = y_up2;
        if (y_dn2 > y_need_hi) y_need_hi = y_dn2;
        if (y_need_hi >= (buf_y0 + buf_cnt))
        {
            /* Finish any in-flight LCD DMA before polled SDIO f_read to avoid AHB contention */
            pb_dma_wait();
            int32_t keep = (buf_cnt >= keep_rows) ? keep_rows : buf_cnt;
            if (keep > 0)
            {
                memmove(batch, batch + (uint32_t)(buf_cnt - keep) * row_bytes, (uint32_t)keep * row_bytes);
                buf_y0 = file_next_y - keep;
            }
            else
            {
                buf_y0 = file_next_y;
            }
            int32_t remain  = (last_row + 1) - file_next_y;
            int32_t to_read = (remain > max_batch) ? max_batch : remain;
            if (to_read <= 0)
            {
                lcd_set_rgb666(0U);
                return 0U;
            }
            UINT br = 0U;
            uint32_t req_bytes = (uint32_t)to_read * row_bytes;
            if (f_read(fp, batch + (uint32_t)keep * row_bytes, req_bytes, &br) != FR_OK || br != req_bytes)
            {
                lcd_set_rgb666(0U);
                return 0U;
            }
            file_next_y += to_read;
            buf_cnt      = keep + to_read;
        }
        if (y_up2 < buf_y0) y_up2 = sy;
        if (y_dn2 < buf_y0) y_dn2 = sy;

        const uint8_t *rp2 = batch + (uint32_t)(y_up2 - buf_y0) * row_bytes;
        const uint8_t *rp  = batch + (uint32_t)(y_up  - buf_y0) * row_bytes;
        const uint8_t *rc  = batch + (uint32_t)(sy    - buf_y0) * row_bytes;
        const uint8_t *rn  = batch + (uint32_t)(y_dn  - buf_y0) * row_bytes;
        const uint8_t *rn2 = batch + (uint32_t)(y_dn2 - buf_y0) * row_bytes;
        uint8_t       *out = nr_on ? znr.dma_buf[znr.buf_idx] : dma_buf[buf_idx];

        /* dma_buf[buf_idx] is already idle (prev row's pb_dma_send waited on it before launching the other half) */
        if (is_8bit)
        {
            /* Fast branch-free 8-bit LinearizationTable path (all C3M1 v0.1.2+ DNGs) */
            if ((sy & 1) == 0)
            {
                for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
                {
                    uint16_t x  = (uint16_t)(pan_x + dx);
                    if (x >= raw_w) x = (uint16_t)(raw_w - 1U);
                    uint16_t xl = (x > 0U) ? (uint16_t)(x - 1U) : ((raw_w > 1U) ? 1U : 0U);
                    uint16_t xr = (x + 1U < raw_w) ? (uint16_t)(x + 1U) : ((raw_w > 1U) ? (uint16_t)(raw_w - 2U) : 0U);
                    uint8_t  c_raw = rc[x];
                    uint32_t r12, g12, b12;
                    if ((x & 1U) == 0U)
                    {
                        if (raw_imp_thr > 0U)
                        {
                            c_raw = pb_nr_despeckle_u8(c_raw, rp[xl], rp[xr], rn[xl], rn[xr], raw_imp_thr);
                        }
                        g12 = lin_tbl[c_raw];
                        r12 = ((uint32_t)lin_tbl[rc[xl]] + lin_tbl[rc[xr]] + 1U) >> 1;
                        b12 = ((uint32_t)lin_tbl[rp[x]]  + lin_tbl[rn[x]]  + 1U) >> 1;
                    }
                    else
                    {
                        if (raw_imp_thr > 0U)
                        {
                            uint16_t xl2 = (x >= 2U) ? (uint16_t)(x - 2U) : ((x + 2U < raw_w) ? (uint16_t)(x + 2U) : x);
                            uint16_t xr2 = (x + 2U < raw_w) ? (uint16_t)(x + 2U) : ((x >= 2U) ? (uint16_t)(x - 2U) : x);
                            c_raw = pb_nr_despeckle_u8(c_raw, rc[xl2], rc[xr2], rp2[x], rn2[x], raw_imp_thr);
                        }
                        r12 = lin_tbl[c_raw];
                        g12 = ((uint32_t)lin_tbl[rc[xl]] + lin_tbl[rc[xr]] +
                                         lin_tbl[rp[x]]  + lin_tbl[rn[x]]  + 2U) >> 2;
                        b12 = ((uint32_t)lin_tbl[rp[xl]] + lin_tbl[rp[xr]] +
                                         lin_tbl[rn[xl]] + lin_tbl[rn[xr]] + 2U) >> 2;
                    }
                    uint16_t bl_g_site = ((x & 1U) == 0U) ? bl_g0 : bl_g_avg;
                    uint32_t rc12 = sub_bl12(r12, bl_r);
                    uint32_t gc12 = sub_bl12(g12, bl_g_site);
                    uint32_t bc12 = sub_bl12(b12, bl_b);
                    if (vig_scale != 0UL)
                    {
                        uint32_t gq8 = c3m1_lens_vig_gain_q8(x, (uint16_t)sy, vig_scale);
                        rc12 = (rc12 * gq8 + 128UL) >> 8; if (rc12 > 4095UL) rc12 = 4095UL;
                        gc12 = (gc12 * gq8 + 128UL) >> 8; if (gc12 > 4095UL) gc12 = 4095UL;
                        bc12 = (bc12 * gq8 + 128UL) >> 8; if (bc12 > 4095UL) bc12 = 4095UL;
                    }
                    uint8_t r8, g8, b8;
                    if (bw_tone != NULL)
                    {
                        r8 = pb_color_bw_cam12_to_y8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                          wbr_n, wbg_n, wbb_n, bw_mode, srgb_lut);
                        if (nr_on)
                        {
                            znr.tmp_y0[dx] = (int8_t)((int32_t)r8 - 128);
                        }
                        else
                        {
                            uint8_t yt = bw_tone[r8];
                            pb_put_rgb666_dither(out, yt, yt, yt, dx, dy);
                        }
                    }
                    else
                    {
                        pb_color_cam12_to_srgb8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                     wbr_n, wbg_n, wbb_n, srgb_lut, &r8, &g8, &b8);
                        if (nr_on)
                        {
                            out[0] = r8;
                            out[1] = g8;
                            out[2] = b8;
                        }
                        else
                        {
                            pb_put_rgb666_dither(out, r8, g8, b8, dx, dy);
                        }
                    }
                    out += 3U;
                }
            }
            else
            {
                for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
                {
                    uint16_t x  = (uint16_t)(pan_x + dx);
                    if (x >= raw_w) x = (uint16_t)(raw_w - 1U);
                    uint16_t xl = (x > 0U) ? (uint16_t)(x - 1U) : ((raw_w > 1U) ? 1U : 0U);
                    uint16_t xr = (x + 1U < raw_w) ? (uint16_t)(x + 1U) : ((raw_w > 1U) ? (uint16_t)(raw_w - 2U) : 0U);
                    uint8_t  c_raw = rc[x];
                    uint32_t r12, g12, b12;
                    if ((x & 1U) == 0U)
                    {
                        if (raw_imp_thr > 0U)
                        {
                            uint16_t xl2 = (x >= 2U) ? (uint16_t)(x - 2U) : ((x + 2U < raw_w) ? (uint16_t)(x + 2U) : x);
                            uint16_t xr2 = (x + 2U < raw_w) ? (uint16_t)(x + 2U) : ((x >= 2U) ? (uint16_t)(x - 2U) : x);
                            c_raw = pb_nr_despeckle_u8(c_raw, rc[xl2], rc[xr2], rp2[x], rn2[x], raw_imp_thr);
                        }
                        b12 = lin_tbl[c_raw];
                        g12 = ((uint32_t)lin_tbl[rc[xl]] + lin_tbl[rc[xr]] +
                                         lin_tbl[rp[x]]  + lin_tbl[rn[x]]  + 2U) >> 2;
                        r12 = ((uint32_t)lin_tbl[rp[xl]] + lin_tbl[rp[xr]] +
                                         lin_tbl[rn[xl]] + lin_tbl[rn[xr]] + 2U) >> 2;
                    }
                    else
                    {
                        if (raw_imp_thr > 0U)
                        {
                            c_raw = pb_nr_despeckle_u8(c_raw, rp[xl], rp[xr], rn[xl], rn[xr], raw_imp_thr);
                        }
                        g12 = lin_tbl[c_raw];
                        b12 = ((uint32_t)lin_tbl[rc[xl]] + lin_tbl[rc[xr]] + 1U) >> 1;
                        r12 = ((uint32_t)lin_tbl[rp[x]]  + lin_tbl[rn[x]]  + 1U) >> 1;
                    }
                    uint16_t bl_g_site = ((x & 1U) != 0U) ? bl_g1 : bl_g_avg;
                    uint32_t rc12 = sub_bl12(r12, bl_r);
                    uint32_t gc12 = sub_bl12(g12, bl_g_site);
                    uint32_t bc12 = sub_bl12(b12, bl_b);
                    if (vig_scale != 0UL)
                    {
                        uint32_t gq8 = c3m1_lens_vig_gain_q8(x, (uint16_t)sy, vig_scale);
                        rc12 = (rc12 * gq8 + 128UL) >> 8; if (rc12 > 4095UL) rc12 = 4095UL;
                        gc12 = (gc12 * gq8 + 128UL) >> 8; if (gc12 > 4095UL) gc12 = 4095UL;
                        bc12 = (bc12 * gq8 + 128UL) >> 8; if (bc12 > 4095UL) bc12 = 4095UL;
                    }
                    uint8_t r8, g8, b8;
                    if (bw_tone != NULL)
                    {
                        r8 = pb_color_bw_cam12_to_y8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                          wbr_n, wbg_n, wbb_n, bw_mode, srgb_lut);
                        if (nr_on)
                        {
                            znr.tmp_y0[dx] = (int8_t)((int32_t)r8 - 128);
                        }
                        else
                        {
                            uint8_t yt = bw_tone[r8];
                            pb_put_rgb666_dither(out, yt, yt, yt, dx, dy);
                        }
                    }
                    else
                    {
                        pb_color_cam12_to_srgb8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                     wbr_n, wbg_n, wbb_n, srgb_lut, &r8, &g8, &b8);
                        if (nr_on)
                        {
                            out[0] = r8;
                            out[1] = g8;
                            out[2] = b8;
                        }
                        else
                        {
                            pb_put_rgb666_dither(out, r8, g8, b8, dx, dy);
                        }
                    }
                    out += 3U;
                }
            }
        }
        else if (info->has_bev && info->baseline_ev_x100 >= 0)
        {
            /* Full 16-bit precision path for merged 12/14/16-bit HDR DNG (no 12-bit shadow squash) */
            uint16_t wl        = info->white_level;
            uint32_t hdr_gain  = pb_ev_x100_to_gain_q8(info->baseline_ev_x100);
            uint32_t ref_white = (uint32_t)(((uint64_t)wl * 256ULL + (hdr_gain >> 1)) / hdr_gain);
            if (ref_white < 1UL) ref_white = 1UL;
            for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
            {
                uint16_t x  = (uint16_t)(pan_x + dx);
                if (x >= raw_w) x = (uint16_t)(raw_w - 1U);
                uint16_t xl = (x > 0U) ? (uint16_t)(x - 1U) : ((raw_w > 1U) ? 1U : 0U);
                uint16_t xr = (x + 1U < raw_w) ? (uint16_t)(x + 1U) : ((raw_w > 1U) ? (uint16_t)(raw_w - 2U) : 0U);
                uint32_t r16, g16, b16;
                if ((sy & 1) == 0)
                {
                    if ((x & 1U) == 0U)
                    {
                        g16 = rd16(rc, (uint32_t)x * 2UL);
                        r16 = ((uint32_t)rd16(rc, (uint32_t)xl * 2UL) +
                               (uint32_t)rd16(rc, (uint32_t)xr * 2UL) + 1UL) >> 1;
                        b16 = ((uint32_t)rd16(rp, (uint32_t)x  * 2UL) +
                               (uint32_t)rd16(rn, (uint32_t)x  * 2UL) + 1UL) >> 1;
                    }
                    else
                    {
                        r16 = rd16(rc, (uint32_t)x * 2UL);
                        g16 = ((uint32_t)rd16(rc, (uint32_t)xl * 2UL) +
                               (uint32_t)rd16(rc, (uint32_t)xr * 2UL) +
                               (uint32_t)rd16(rp, (uint32_t)x  * 2UL) +
                               (uint32_t)rd16(rn, (uint32_t)x  * 2UL) + 2UL) >> 2;
                        b16 = ((uint32_t)rd16(rp, (uint32_t)xl * 2UL) +
                               (uint32_t)rd16(rp, (uint32_t)xr * 2UL) +
                               (uint32_t)rd16(rn, (uint32_t)xl * 2UL) +
                               (uint32_t)rd16(rn, (uint32_t)xr * 2UL) + 2UL) >> 2;
                    }
                }
                else
                {
                    if ((x & 1U) == 0U)
                    {
                        b16 = rd16(rc, (uint32_t)x * 2UL);
                        g16 = ((uint32_t)rd16(rc, (uint32_t)xl * 2UL) +
                               (uint32_t)rd16(rc, (uint32_t)xr * 2UL) +
                               (uint32_t)rd16(rp, (uint32_t)x  * 2UL) +
                               (uint32_t)rd16(rn, (uint32_t)x  * 2UL) + 2UL) >> 2;
                        r16 = ((uint32_t)rd16(rp, (uint32_t)xl * 2UL) +
                               (uint32_t)rd16(rp, (uint32_t)xr * 2UL) +
                               (uint32_t)rd16(rn, (uint32_t)xl * 2UL) +
                               (uint32_t)rd16(rn, (uint32_t)xr * 2UL) + 2UL) >> 2;
                    }
                    else
                    {
                        g16 = rd16(rc, (uint32_t)x * 2UL);
                        b16 = ((uint32_t)rd16(rc, (uint32_t)xl * 2UL) +
                               (uint32_t)rd16(rc, (uint32_t)xr * 2UL) + 1UL) >> 1;
                        r16 = ((uint32_t)rd16(rp, (uint32_t)x  * 2UL) +
                               (uint32_t)rd16(rn, (uint32_t)x  * 2UL) + 1UL) >> 1;
                    }
                }
                if (vig_scale != 0UL)
                {
                    uint32_t gq8 = c3m1_lens_vig_gain_q8(x, (uint16_t)sy, vig_scale);
                    r16 = (r16 * gq8 + 128UL) >> 8; if (r16 > wl) r16 = wl;
                    g16 = (g16 * gq8 + 128UL) >> 8; if (g16 > wl) g16 = wl;
                    b16 = (b16 * gq8 + 128UL) >> 8; if (b16 > wl) b16 = wl;
                }
                uint8_t r8, g8, b8;
                if (bw_tone != NULL)
                {
                    r8 = c3m1_color_hdr_to_bw_y8((uint16_t)r16, (uint16_t)g16, (uint16_t)b16,
                                                 wb_r, wb_b, ref_white, (uint32_t)wl,
                                                 bw_mode, srgb_lut);
                    if (nr_on)
                    {
                        znr.tmp_y0[dx] = (int8_t)((int32_t)r8 - 128);
                    }
                    else
                    {
                        uint8_t yt = bw_tone[r8];
                        pb_put_rgb666_dither(out, yt, yt, yt, dx, dy);
                    }
                }
                else
                {
                    c3m1_color_hdr_to_srgb8((uint16_t)r16, (uint16_t)g16, (uint16_t)b16,
                                            wb_r, wb_b, ref_white, (uint32_t)wl,
                                            srgb_lut, &r8, &g8, &b8);
                    if (nr_on)
                    {
                        out[0] = r8;
                        out[1] = g8;
                        out[2] = b8;
                    }
                    else
                    {
                        pb_put_rgb666_dither(out, r8, g8, b8, dx, dy);
                    }
                }
                out += 3U;
            }
        }
        else
        {
            /* Legacy 16-bit DNG path (no BaselineExposure) */
            uint16_t wl = info->white_level;
            for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
            {
                uint16_t x  = (uint16_t)(pan_x + dx);
                if (x >= raw_w) x = (uint16_t)(raw_w - 1U);
                uint16_t xl = (x > 0U) ? (uint16_t)(x - 1U) : ((raw_w > 1U) ? 1U : 0U);
                uint16_t xr = (x + 1U < raw_w) ? (uint16_t)(x + 1U) : ((raw_w > 1U) ? (uint16_t)(raw_w - 2U) : 0U);
                uint32_t r12, g12, b12;
                if ((sy & 1) == 0)
                {
                    if ((x & 1U) == 0U)
                    {
                        g12 = dng_sample_12(rc, x, 16U, lin_tbl, wl);
                        r12 = ((uint32_t)dng_sample_12(rc, xl, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rc, xr, 16U, lin_tbl, wl) + 1U) >> 1;
                        b12 = ((uint32_t)dng_sample_12(rp, x, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rn, x, 16U, lin_tbl, wl) + 1U) >> 1;
                    }
                    else
                    {
                        r12 = dng_sample_12(rc, x, 16U, lin_tbl, wl);
                        g12 = ((uint32_t)dng_sample_12(rc, xl, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rc, xr, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rp, x,  16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rn, x,  16U, lin_tbl, wl) + 2U) >> 2;
                        b12 = ((uint32_t)dng_sample_12(rp, xl, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rp, xr, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rn, xl, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rn, xr, 16U, lin_tbl, wl) + 2U) >> 2;
                    }
                }
                else
                {
                    if ((x & 1U) == 0U)
                    {
                        b12 = dng_sample_12(rc, x, 16U, lin_tbl, wl);
                        g12 = ((uint32_t)dng_sample_12(rc, xl, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rc, xr, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rp, x,  16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rn, x,  16U, lin_tbl, wl) + 2U) >> 2;
                        r12 = ((uint32_t)dng_sample_12(rp, xl, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rp, xr, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rn, xl, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rn, xr, 16U, lin_tbl, wl) + 2U) >> 2;
                    }
                    else
                    {
                        g12 = dng_sample_12(rc, x, 16U, lin_tbl, wl);
                        b12 = ((uint32_t)dng_sample_12(rc, xl, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rc, xr, 16U, lin_tbl, wl) + 1U) >> 1;
                        r12 = ((uint32_t)dng_sample_12(rp, x, 16U, lin_tbl, wl) +
                               (uint32_t)dng_sample_12(rn, x, 16U, lin_tbl, wl) + 1U) >> 1;
                    }
                }
                uint16_t bl_g_site = ((((uint16_t)sy & 1U) == (x & 1U)) ? (((sy & 1) == 0) ? bl_g0 : bl_g1) : bl_g_avg);
                uint32_t rc12 = sub_bl12(r12, bl_r);
                uint32_t gc12 = sub_bl12(g12, bl_g_site);
                uint32_t bc12 = sub_bl12(b12, bl_b);
                if (vig_scale != 0UL)
                {
                    uint32_t gq8 = c3m1_lens_vig_gain_q8(x, (uint16_t)sy, vig_scale);
                    rc12 = (rc12 * gq8 + 128UL) >> 8; if (rc12 > 4095UL) rc12 = 4095UL;
                    gc12 = (gc12 * gq8 + 128UL) >> 8; if (gc12 > 4095UL) gc12 = 4095UL;
                    bc12 = (bc12 * gq8 + 128UL) >> 8; if (bc12 > 4095UL) bc12 = 4095UL;
                }
                uint8_t r8, g8, b8;
                if (bw_tone != NULL)
                {
                    r8 = pb_color_bw_cam12_to_y8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                      wbr_n, wbg_n, wbb_n, bw_mode, srgb_lut);
                    if (nr_on)
                    {
                        znr.tmp_y0[dx] = (int8_t)((int32_t)r8 - 128);
                    }
                    else
                    {
                        uint8_t yt = bw_tone[r8];
                        pb_put_rgb666_dither(out, yt, yt, yt, dx, dy);
                    }
                }
                else
                {
                    pb_color_cam12_to_srgb8_norm((uint16_t)rc12, (uint16_t)gc12, (uint16_t)bc12,
                                                 wbr_n, wbg_n, wbb_n, srgb_lut, &r8, &g8, &b8);
                    if (nr_on)
                    {
                        out[0] = r8;
                        out[1] = g8;
                        out[2] = b8;
                    }
                    else
                    {
                        pb_put_rgb666_dither(out, r8, g8, b8, dx, dy);
                    }
                }
                out += 3U;
            }
        }
        if (nr_on)
        {
            if (bw_tone != NULL)
            {
                pb_zoom_nr_push_bw_row(&znr, dy, bw_tone);
            }
            else
            {
                pb_zoom_nr_push_row(&znr, dy, znr.dma_buf[znr.buf_idx]);
            }
        }
        else
        {
            if (dy >= PB_BAR_Y && bar_buf != NULL)
            {
                pb_overlay_bar_rgb666(dma_buf[buf_idx], dy, bar_buf);
            }
            pb_dma_send(dma_buf[buf_idx], (uint16_t)(PB_SCR_W * 3U));
            buf_idx ^= 1U;
        }
    }
    if (nr_on)
    {
        if (bw_tone != NULL)
        {
            pb_zoom_nr_flush_bw(&znr, bw_tone);
        }
        else
        {
            pb_zoom_nr_flush(&znr);
        }
    }
    pb_dma_wait();
    lcd_set_rgb666(0U);
    return 1U;
}

/* True 1:1 RAW crop zoom dispatcher (keeps warped & unwarped sibling stack frames non-overlapping) */
static uint8_t dng_render_zoom_raw(FIL *fp, const dng_info_t *info,
                                   const uint16_t *lin_tbl,
                                   const uint8_t *srgb_lut,
                                   uint16_t pan_x, uint16_t pan_y,
                                   uint16_t nr_iso,
                                   uint8_t *scratch_after_idx,
                                   uint8_t *work_buf,
                                   uint16_t work_buf_size,
                                   const uint8_t (*bar_buf)[160])
{
    uint32_t bpp       = (info->raw_bps == 16U) ? 2U : 1U;
    uint32_t row_bytes = (uint32_t)info->raw_w * bpp;
    if (row_bytes > 1300U || work_buf == NULL) return 0U;

    if (info->has_warp && info->raw_w >= 640U && info->raw_w <= 644U && info->raw_h == 480U)
    {
        if (dng_render_zoom_raw_warped(fp, info, lin_tbl, srgb_lut, pan_x, pan_y, nr_iso,
                                       scratch_after_idx, work_buf, work_buf_size, bar_buf))
        {
            return 1U;
        }
    }
    return dng_render_zoom_raw_unwarped(fp, info, lin_tbl, srgb_lut, pan_x, pan_y, nr_iso,
                                        scratch_after_idx, work_buf, bar_buf);
}

/* ======================== JPEG Renderers (via c3m1_jdec) ======================== */
typedef struct
{
    FIL            *fp;
    uint16_t        img_w;
    uint16_t        img_h;
    uint16_t        pan_x;
    uint16_t        pan_y;
    uint8_t         clip_en;
    pb_hist_t      *hist;
    uint16_t       *strip_buf; /* Optional 320x8 BE RGB565 strip buffer (5,120 B) for 640x480 1X DMA fast path */
    const uint8_t (*bar_buf)[160];
} jpg_render_ctx_t;

static void jpg_parse_exif_meta(FIL *fp, uint8_t *buf, uint16_t buf_sz, pb_info_meta_t *meta)
{
    if (meta == NULL || buf == NULL || buf_sz < 64U) return;
    meta->datetime[0] = '\0';

    uint32_t read_len = (buf_sz >= 2048U) ? 2048U : buf_sz;
    UINT br = 0U;
    if (f_lseek(fp, 0U) != FR_OK || f_read(fp, buf, read_len, &br) != FR_OK || br < 24U)
    {
        return;
    }
    if (buf[0] != 0xFFU || buf[1] != 0xD8U) return;

    uint32_t pos = 2U;
    while (pos + 4U <= br)
    {
        if (buf[pos] != 0xFFU) break;
        while (pos < br && buf[pos] == 0xFFU) pos++;
        if (pos >= br) break;
        uint8_t marker = buf[pos++];
        if (marker == 0xDAU || marker == 0xD9U) break; /* SOS / EOI */
        if (pos + 2U > br) break;

        uint16_t seg_len = (uint16_t)(((uint16_t)buf[pos] << 8) | buf[pos + 1U]);
        if (seg_len < 2U) break;

        if (marker == 0xE1U && seg_len >= 16U && tiff_range_ok(pos + 2U, 14U, br) &&
            memcmp(&buf[pos + 2U], "Exif\0\0", 6U) == 0)
        {
            uint32_t tiff_base = pos + 8U;
            uint32_t tiff_len  = (uint32_t)(seg_len - 8U);
            if (tiff_len > br - tiff_base) tiff_len = br - tiff_base;
            const uint8_t *tb = &buf[tiff_base];

            if (tiff_len >= 10U)
            {
                uint8_t be = 0U;
                if (tb[0] == 'I' && tb[1] == 'I') be = 0U;
                else if (tb[0] == 'M' && tb[1] == 'M') be = 1U;
                else break;

                if (exif_rd16(tb, 2U, be) == 42U)
                {
                    uint32_t ifd0 = exif_rd32(tb, 4U, be);
                    uint32_t exif_off = 0U;
                    if (tiff_range_ok(ifd0, 2U, tiff_len))
                    {
                        uint16_t cnt = exif_rd16(tb, ifd0, be);
                        uint32_t ep  = ifd0 + 2U;
                        for (uint16_t i = 0U; i < cnt; i++, ep += 12U)
                        {
                            if (!tiff_range_ok(ep, 12U, tiff_len)) break;
                            uint16_t tag   = exif_rd16(tb, ep, be);
                            uint16_t type  = exif_rd16(tb, ep + 2U, be);
                            uint32_t count = exif_rd32(tb, ep + 4U, be);
                            uint32_t val   = exif_rd32(tb, ep + 8U, be);
                            if (tag == 0x0132U && type == 2U && count >= 19U &&
                                tiff_range_ok(val, 19U, tiff_len))
                            {
                                pb_copy_exif_datetime(&tb[val], meta->datetime);
                            }
                            else if (tag == 0x8769U && count >= 1U)
                            {
                                exif_off = val;
                            }
                        }
                    }
                    if (exif_off > 0U)
                    {
                        pb_parse_exif_ifd(tb, tiff_len, exif_off, be, meta->datetime,
                                          &meta->exp_num, &meta->exp_den, &meta->has_exp,
                                          &meta->iso, &meta->has_iso,
                                          &meta->ev_bias_x10, &meta->has_ev,
                                          &meta->fnum_x10, &meta->has_fnum,
                                          &meta->focal_x10, &meta->focal_eq35);
                    }
                }
            }
            break;
        }
        pos += seg_len;
    }
}

static uint32_t jpg_fatfs_read(void *user_ctx, uint8_t *dst, uint32_t max_len)
{
    jpg_render_ctx_t *jc = (jpg_render_ctx_t *)user_ctx;
    UINT br = 0U;
    if (f_read(jc->fp, dst, max_len, &br) != FR_OK) return 0U;
    return (uint32_t)br;
}

static inline __attribute__((always_inline)) uint16_t rgb565_avg4(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{
    uint32_t r = ((a >> 11) + (b >> 11) + (c >> 11) + (d >> 11) + 2U) >> 2;
    uint32_t g = (((a >> 5) & 63U) + ((b >> 5) & 63U) + ((c >> 5) & 63U) + ((d >> 5) & 63U) + 2U) >> 2;
    uint32_t bl = ((a & 31U) + (b & 31U) + (c & 31U) + (d & 31U) + 2U) >> 2;
    return (uint16_t)((r << 11) | (g << 5) | bl);
}

static void __attribute__((optimize("O2"))) jpg_1x_mcu_cb(void *user_ctx, uint16_t mx, uint16_t my,
                                                          uint8_t mw, uint8_t mh, const uint16_t *rgb565)
{
    jpg_render_ctx_t *jc = (jpg_render_ctx_t *)user_ctx;
    uint16_t w = jc->img_w;
    uint16_t h = jc->img_h;

    /* Fast 2:1 strip-buffered DMA path for native 640x480 4:2:0 (16x16 -> 8 rows) & grayscale (8x8 -> 4 rows) */
    if (w == 640U && h == 480U &&
        ((mw == 16U && mh == 16U) || (mw == 8U && mh == 8U)) &&
        jc->strip_buf != NULL)
    {
        uint16_t out_w = (uint16_t)(mw >> 1);
        uint16_t out_h = (uint16_t)(mh >> 1);
        uint16_t dy0   = (uint16_t)(my >> 1);
        if (dy0 >= PB_VIEW_H) return;
        uint16_t rows  = ((uint16_t)(dy0 + out_h) <= PB_VIEW_H) ? out_h : (uint16_t)(PB_VIEW_H - dy0);
        uint16_t dx0   = (uint16_t)(mx >> 1);
        uint8_t  clip  = jc->clip_en;

        for (uint16_t r = 0U; r < rows; r++)
        {
            const uint16_t *r0  = &rgb565[(uint16_t)((r << 1) * mw)];
            const uint16_t *r1  = r0 + mw;
            uint16_t       *dst = &jc->strip_buf[(uint32_t)r * PB_1X_W + dx0];
            uint16_t        dy  = (uint16_t)(dy0 + r);
            for (uint16_t c = 0U; c < out_w; c++)
            {
                uint16_t px = rgb565_avg4(r0[c << 1], r0[(c << 1) + 1U], r1[c << 1], r1[(c << 1) + 1U]);
                if (clip)
                {
                    px = pb_apply_rgb565_1x_clip(px, (uint16_t)(dx0 + c), dy);
                }
                dst[c] = (uint16_t)__REV16((uint32_t)px);
            }
        }

        if ((uint16_t)(mx + mw) >= w)
        {
            if (jc->bar_buf != NULL && (uint16_t)(dy0 + rows) > PB_BAR_Y)
            {
                for (uint16_t r = 0U; r < rows; r++)
                {
                    uint16_t dy = (uint16_t)(dy0 + r);
                    if (dy >= PB_BAR_Y)
                    {
                        pb_overlay_bar_rgb565_be((uint8_t *)&jc->strip_buf[(uint32_t)r * PB_1X_W], dy, jc->bar_buf);
                    }
                }
            }
            lcd_set_window(0U, dy0, PB_1X_W, rows);
            lcd_write_ram_prepare();
            pb_dma_send((const uint8_t *)jc->strip_buf, (uint16_t)((uint32_t)PB_1X_W * rows * 2UL));
            pb_dma_wait();
        }
        return;
    }

    uint16_t mx1 = (uint16_t)(mx + mw);
    uint16_t my1 = (uint16_t)(my + mh);
    if (mx1 > w) mx1 = w;
    if (my1 > h) my1 = h;

    uint16_t dx0 = (uint16_t)(((uint32_t)mx  * PB_1X_W) / w);
    uint16_t dx1 = (uint16_t)(((uint32_t)mx1 * PB_1X_W) / w);
    uint16_t dy0 = (uint16_t)(((uint32_t)my  * PB_1X_H) / h);
    uint16_t dy1 = (uint16_t)(((uint32_t)my1 * PB_1X_H) / h);
    if (dx1 > PB_1X_W)   dx1 = PB_1X_W;
    if (dy1 > PB_VIEW_H) dy1 = PB_VIEW_H;
    if (dx1 <= dx0 || dy1 <= dy0) return;

    lcd_set_window(dx0, dy0, (uint16_t)(dx1 - dx0), (uint16_t)(dy1 - dy0));
    lcd_write_ram_prepare();

    for (uint16_t dy = dy0; dy < dy1; dy++)
    {
        uint32_t gy = ((uint32_t)dy * h) / PB_1X_H;
        uint8_t  ly = (gy > my) ? (uint8_t)(gy - my) : 0U;
        if (ly >= mh) ly = (uint8_t)(mh - 1U);
        uint8_t  ly2 = (ly + 1U < mh) ? (uint8_t)(ly + 1U) : ly;
        const uint16_t *r0 = &rgb565[(uint16_t)ly  * mw];
        const uint16_t *r1 = &rgb565[(uint16_t)ly2 * mw];

        for (uint16_t dx = dx0; dx < dx1; dx++)
        {
            uint32_t gx = ((uint32_t)dx * w) / PB_1X_W;
            uint8_t  lx = (gx > mx) ? (uint8_t)(gx - mx) : 0U;
            if (lx >= mw) lx = (uint8_t)(mw - 1U);
            uint8_t  lx2 = (lx + 1U < mw) ? (uint8_t)(lx + 1U) : lx;
            uint16_t px  = rgb565_avg4(r0[lx], r0[lx2], r1[lx], r1[lx2]);
            if (jc->clip_en)
            {
                px = pb_apply_rgb565_1x_clip(px, dx, dy);
            }
            if (dy >= PB_BAR_Y && jc->bar_buf != NULL)
            {
                px = pb_apply_bar_rgb565_px(px, dx, dy, jc->bar_buf);
            }
            lcd_wr_pixel(px);
        }
    }
}

static void jpg_zoom_mcu_cb(void *user_ctx, uint16_t mx, uint16_t my,
                            uint8_t mw, uint8_t mh, const uint16_t *rgb565)
{
    jpg_render_ctx_t *jc = (jpg_render_ctx_t *)user_ctx;
    uint16_t vx0 = jc->pan_x;
    uint16_t vy0 = jc->pan_y;
    uint16_t vx1 = (uint16_t)(vx0 + PB_SCR_W);
    uint16_t vy1 = (uint16_t)(vy0 + PB_VIEW_H);

    uint16_t sx0 = (mx > vx0) ? mx : vx0;
    uint16_t sy0 = (my > vy0) ? my : vy0;
    uint16_t sx1 = (uint16_t)(mx + mw);
    uint16_t sy1 = (uint16_t)(my + mh);
    if (sx1 > vx1) sx1 = vx1;
    if (sy1 > vy1) sy1 = vy1;
    if (sx1 > jc->img_w) sx1 = jc->img_w;
    if (sy1 > jc->img_h) sy1 = jc->img_h;
    if (sx1 <= sx0 || sy1 <= sy0) return;

    uint16_t dst_x0 = (uint16_t)(sx0 - vx0);
    lcd_set_window(dst_x0, (uint16_t)(sy0 - vy0),
                   (uint16_t)(sx1 - sx0), (uint16_t)(sy1 - sy0));
    lcd_write_ram_prepare();

    for (uint16_t sy = sy0; sy < sy1; sy++)
    {
        const uint16_t *row = &rgb565[(uint16_t)(sy - my) * mw + (sx0 - mx)];
        uint16_t cnt = (uint16_t)(sx1 - sx0);
        uint16_t dy  = (uint16_t)(sy - vy0);
        if (dy >= PB_BAR_Y && jc->bar_buf != NULL)
        {
            for (uint16_t i = 0U; i < cnt; i++)
            {
                lcd_wr_pixel(pb_apply_bar_rgb565_px(row[i], (uint16_t)(dst_x0 + i), dy, jc->bar_buf));
            }
        }
        else
        {
            for (uint16_t i = 0U; i < cnt; i++)
            {
                lcd_wr_pixel(row[i]);
            }
        }
    }
}

static void jpg_info_mcu_cb(void *user_ctx, uint16_t mx, uint16_t my,
                            uint8_t mw, uint8_t mh, const uint16_t *rgb565)
{
    jpg_render_ctx_t *jc = (jpg_render_ctx_t *)user_ctx;
    uint16_t w = jc->img_w;
    uint16_t h = jc->img_h;

    uint16_t mx1 = (uint16_t)(mx + mw);
    uint16_t my1 = (uint16_t)(my + mh);
    if (mx1 > w) mx1 = w;
    if (my1 > h) my1 = h;

    uint16_t dx0 = (uint16_t)(((uint32_t)mx  * PB_INFO_TH_W) / w);
    uint16_t dx1 = (uint16_t)(((uint32_t)mx1 * PB_INFO_TH_W) / w);
    uint16_t dy0 = (uint16_t)(((uint32_t)my  * PB_INFO_TH_H) / h);
    uint16_t dy1 = (uint16_t)(((uint32_t)my1 * PB_INFO_TH_H) / h);
    if (dx1 > PB_INFO_TH_W) dx1 = PB_INFO_TH_W;
    if (dy1 > PB_INFO_TH_H) dy1 = PB_INFO_TH_H;
    if (dx1 <= dx0 || dy1 <= dy0) return;

    lcd_set_window((uint16_t)(PB_INFO_TH_X + dx0), (uint16_t)(PB_INFO_TH_Y + dy0),
                   (uint16_t)(dx1 - dx0), (uint16_t)(dy1 - dy0));
    lcd_write_ram_prepare();

    for (uint16_t dy = dy0; dy < dy1; dy++)
    {
        uint32_t gy = ((uint32_t)dy * h) / PB_INFO_TH_H;
        uint8_t  ly = (gy > my) ? (uint8_t)(gy - my) : 0U;
        if (ly >= mh) ly = (uint8_t)(mh - 1U);
        uint8_t  ly2 = (ly + 2U < mh) ? (uint8_t)(ly + 2U) : ((ly + 1U < mh) ? (uint8_t)(ly + 1U) : ly);
        const uint16_t *r0 = &rgb565[(uint16_t)ly  * mw];
        const uint16_t *r1 = &rgb565[(uint16_t)ly2 * mw];

        for (uint16_t dx = dx0; dx < dx1; dx++)
        {
            uint32_t gx = ((uint32_t)dx * w) / PB_INFO_TH_W;
            uint8_t  lx = (gx > mx) ? (uint8_t)(gx - mx) : 0U;
            if (lx >= mw) lx = (uint8_t)(mw - 1U);
            uint8_t  lx2 = (lx + 2U < mw) ? (uint8_t)(lx + 2U) : ((lx + 1U < mw) ? (uint8_t)(lx + 1U) : lx);
            uint16_t px  = rgb565_avg4(r0[lx], r0[lx2], r1[lx], r1[lx2]);
            if (jc->hist != NULL)
            {
                pb_hist_add_rgb565(jc->hist, px);
            }
            lcd_wr_pixel(pb_apply_rgb565_1x_clip(px, dx, dy));
        }
    }
}

static uint8_t jpg_render_file(const char *path, uint8_t zoom_mode, uint8_t disp_mode,
                               uint16_t pan_x, uint16_t pan_y,
                               uint16_t *out_w, uint16_t *out_h,
                               uint8_t *work_buf, uint16_t work_buf_sz,
                               pb_hist_t *hist, pb_info_meta_t *meta,
                               uint8_t *scratch_after_idx,
                               const uint8_t (*bar_buf)[160])
{
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0U;

    if (meta != NULL)
    {
        meta->jpg_bytes = (uint32_t)f_size(&f);
        jpg_parse_exif_meta(&f, work_buf, work_buf_sz, meta);
        if (f_lseek(&f, 0U) != FR_OK)
        {
            f_close(&f);
            return 0U;
        }
    }

    c3m1_jdec_t *jd = (c3m1_jdec_t *)scratch_after_idx;
    jpg_render_ctx_t jc = {
        .fp        = &f,
        .img_w     = 0U,
        .img_h     = 0U,
        .pan_x     = pan_x,
        .pan_y     = pan_y,
        .clip_en   = (disp_mode == PB_DISP_CLIP) ? 1U : 0U,
        .hist      = hist,
        .strip_buf = (uint16_t *)(scratch_after_idx + PB_JDEC_STRIP_OFF),
        .bar_buf   = bar_buf,
    };

    if (!c3m1_jdec_prepare(jd, jpg_fatfs_read, &jc))
    {
        f_close(&f);
        return 0U;
    }
    jc.img_w = jd->width;
    jc.img_h = jd->height;
    if (out_w != NULL) *out_w = jd->width;
    if (out_h != NULL) *out_h = jd->height;
    if (meta != NULL)
    {
        meta->img_w = jd->width;
        meta->img_h = jd->height;
        if (jd->ncomp == 1U) meta->jpg_sub = 4U;
        else if (jd->max_hsamp == 2U && jd->max_vsamp == 2U) meta->jpg_sub = 1U;
        else if (jd->max_hsamp == 2U && jd->max_vsamp == 1U) meta->jpg_sub = 2U;
        else meta->jpg_sub = 3U;
    }

    uint8_t ok;
    if (zoom_mode)
    {
        if (jd->width < PB_SCR_W || jd->height < PB_VIEW_H)
        {
            lcd_fill(0U, 0U, PB_SCR_W - 1U, PB_BAR_Y - 1U, BLACK);
            jc.bar_buf = NULL;
        }
        ok = c3m1_jdec_decode(jd, pan_x, pan_y, PB_SCR_W, PB_VIEW_H, jpg_zoom_mcu_cb, &jc);
    }
    else if (disp_mode == PB_DISP_INFO)
    {
        ok = c3m1_jdec_decode(jd, 0U, 0U, jd->width, jd->height, jpg_info_mcu_cb, &jc);
    }
    else
    {
        ok = c3m1_jdec_decode(jd, 0U, 0U, jd->width, jd->height, jpg_1x_mcu_cb, &jc);
    }

    f_close(&f);
    return ok;
}

/* ======================== UI Overlays & Delete Modal ======================== */
static void pb_draw_zoom_minimap(uint16_t img_w, uint16_t img_h, uint16_t pan_x, uint16_t pan_y)
{
    if (img_w == 0U || img_h == 0U) return;
    const uint16_t mx = 280U, my = 4U, mw = 34U, mh = 26U;
    lcd_fill(mx, my, (uint16_t)(mx + mw - 1U), (uint16_t)(my + mh - 1U), BLACK);
    lcd_draw_rectangle(mx, my, (uint16_t)(mx + mw - 1U), (uint16_t)(my + mh - 1U), GRAY);

    uint16_t rx = (uint16_t)(mx + 1U + ((uint32_t)pan_x * (mw - 2U)) / img_w);
    uint16_t ry = (uint16_t)(my + 1U + ((uint32_t)pan_y * (mh - 2U)) / img_h);
    uint16_t rw = (uint16_t)(((uint32_t)PB_SCR_W  * (mw - 2U)) / img_w);
    uint16_t rh = (uint16_t)(((uint32_t)PB_VIEW_H * (mh - 2U)) / img_h);
    if (rw < 4U) rw = 4U;
    if (rh < 4U) rh = 4U;
    if (rw > (mw - 2U)) rw = (uint16_t)(mw - 2U);
    if (rh > (mh - 2U)) rh = (uint16_t)(mh - 2U);
    if (rx + rw > mx + mw - 1U) rx = (uint16_t)(mx + mw - 1U - rw);
    if (ry + rh > my + mh - 1U) ry = (uint16_t)(my + mh - 1U - rh);
    lcd_draw_rectangle(rx, ry, (uint16_t)(rx + rw - 1U), (uint16_t)(ry + rh - 1U), YELLOW);
}

static void pb_fill_fatfs_datetime(const char *path, char out[20], uint8_t *work_buf)
{
    FILINFO *fno = (FILINFO *)work_buf;
    if (f_stat(path, fno) != FR_OK) return;
    if (fno->fdate == 0U) return;

    unsigned int y = 1980U + (unsigned int)((fno->fdate >> 9) & 0x7FU);
    unsigned int m = (unsigned int)((fno->fdate >> 5) & 0x0FU);
    unsigned int d = (unsigned int)(fno->fdate & 0x1FU);
    unsigned int H = (unsigned int)((fno->ftime >> 11) & 0x1FU);
    unsigned int M = (unsigned int)((fno->ftime >> 5) & 0x3FU);
    unsigned int S = (unsigned int)((fno->ftime & 0x1FU) * 2U);
    if (m < 1U || m > 12U || d < 1U || d > 31U) return;

    snprintf(out, 20U, "%04u-%02u-%02u %02u:%02u:%02u", y, m, d, H, M, S);
}

static void pb_draw_single_hist(uint16_t x0, uint16_t y0, const char *label,
                                uint16_t label_col, uint16_t bar_col,
                                const uint16_t bins[64], uint32_t total_px)
{
    /* Outer box: 130x29 px (128x27 inner = 64 bins * 2 px wide, max bar height 25 px) */
    const uint16_t box_w   = 130U;
    const uint16_t box_h   = 29U;
    const uint16_t max_h   = 25U;
    const uint16_t x1      = (uint16_t)(x0 + box_w - 1U);
    const uint16_t y1      = (uint16_t)(y0 + box_h - 1U);
    const uint16_t inner_y = (uint16_t)(y1 - 1U);

    g_back_color = BLACK;
    lcd_show_string((uint16_t)(x0 - 10U), (uint16_t)(y0 + 8U), 8U, 12U, 12U, label, label_col);

    lcd_fill((uint16_t)(x0 + 1U), (uint16_t)(y0 + 1U), (uint16_t)(x1 - 1U), inner_y, PB_COL_HIST_BG);
    lcd_draw_rectangle(x0, y0, x1, y1, PB_COL_DARKGRAY);

    /* Subtle Sony-style 1/4 vertical gridlines */
    for (uint8_t q = 1U; q <= 3U; q++)
    {
        uint16_t gx = (uint16_t)(x0 + 1U + q * 32U);
        lcd_fill(gx, (uint16_t)(y0 + 1U), gx, inner_y, PB_COL_HIST_GRID);
    }

    /* Normalize against interior bins [1..62] so border clipping spikes don't crush midtones */
    uint32_t peak = 0UL;
    for (uint8_t k = 1U; k < 63U; k++)
    {
        if (bins[k] > peak) peak = bins[k];
    }
    if (peak == 0UL)
    {
        peak = (bins[0] > bins[63]) ? bins[0] : bins[63];
    }
    if (peak == 0UL) return;

    for (uint8_t k = 0U; k < 64U; k++)
    {
        uint32_t v = bins[k];
        if (v == 0UL) continue;
        uint32_t bh = (v * max_h + (peak >> 1)) / peak;
        if (bh == 0UL) bh = 1UL;
        if (bh > max_h) bh = max_h;
        uint16_t bx0 = (uint16_t)(x0 + 1U + (uint16_t)k * 2U);
        uint16_t by0 = (uint16_t)(inner_y - bh + 1U);
        lcd_fill(bx0, by0, (uint16_t)(bx0 + 1U), inner_y, bar_col);
    }

    /* Highlight / shadow edge clipping indicators (>1% of pixels in extreme bin) */
    if (total_px > 0UL)
    {
        if ((uint32_t)bins[63] * 100UL >= total_px)
        {
            lcd_fill((uint16_t)(x1 - 1U), (uint16_t)(y0 + 1U), (uint16_t)(x1 - 1U), inner_y, RED);
        }
        if ((uint32_t)bins[0] * 100UL >= total_px)
        {
            lcd_fill((uint16_t)(x0 + 1U), (uint16_t)(y0 + 1U), (uint16_t)(x0 + 1U), inner_y, PB_COL_HIST_B);
        }
    }
}

static void pb_format_shutter(uint32_t num, uint32_t den, char *buf, uint8_t buf_len)
{
    if (num == 0UL || den == 0UL)
    {
        snprintf(buf, buf_len, "--");
        return;
    }
    if (num >= den)
    {
        uint32_t tenths = (uint32_t)(((uint64_t)num * 10ULL + (uint64_t)(den >> 1)) / (uint64_t)den);
        if ((tenths % 10UL) == 0UL)
        {
            snprintf(buf, buf_len, "%lus", (unsigned long)(tenths / 10UL));
        }
        else
        {
            snprintf(buf, buf_len, "%lu.%lus", (unsigned long)(tenths / 10UL), (unsigned long)(tenths % 10UL));
        }
        return;
    }
    uint32_t inv = (uint32_t)(((uint64_t)den + (uint64_t)(num >> 1)) / (uint64_t)num);
    if (inv < 1UL) inv = 1UL;
    snprintf(buf, buf_len, "1/%lus", (unsigned long)inv);
}

static uint8_t pb_str_ends_with_ci(const char *s, const char *suf)
{
    size_t sl = strlen(s);
    size_t ul = strlen(suf);
    if (sl < ul) return 0U;
    const char *p = s + (sl - ul);
    for (size_t i = 0U; i < ul; i++)
    {
        char a = p[i];
        char b = suf[i];
        if (a >= 'A' && a <= 'Z') a = (char)(a + ('a' - 'A'));
        if (b >= 'A' && b <= 'Z') b = (char)(b + ('a' - 'A'));
        if (a != b) return 0U;
    }
    return 1U;
}

static void pb_scan_jsr_dir_stats(const char *jsr_dir, uint8_t is_hdr, pb_info_meta_t *meta)
{
    DIR jdir;
    FILINFO jfno;
    if (f_opendir(&jdir, jsr_dir) != FR_OK) return;

    uint8_t hdr_cnt  = 0U;
    uint8_t brk0_cnt = 0U;

    for (;;)
    {
        if (f_readdir(&jdir, &jfno) != FR_OK || jfno.fname[0] == '\0') break;
        if (jfno.fattrib & AM_DIR) continue;

        meta->jsr_files++;
        meta->jsr_bytes += (uint32_t)jfno.fsize;

        if (pb_str_ends_with_ci(jfno.fname, "_dark.dng"))
        {
            meta->jsr_has_dark = 1U;
        }
        else if (is_hdr)
        {
            if (pb_str_ends_with_ci(jfno.fname, "_hdr.dng") && hdr_cnt < 255U)
            {
                hdr_cnt++;
            }
            else if (pb_str_ends_with_ci(jfno.fname, "_0.dng") && brk0_cnt < 255U)
            {
                brk0_cnt++;
            }
        }
        else
        {
            uint16_t dnum = 0U;
            uint8_t  dflg = 0U;
            uint8_t  dvar = 0U;
            if (parse_cdi_filename(jfno.fname, &dnum, &dflg, &dvar) &&
                dflg == PB_FLAG_DNG && dvar == PB_VAR_NORMAL && meta->jsr_lights < 255U)
            {
                meta->jsr_lights++;
            }
        }
    }
    f_closedir(&jdir);

    if (is_hdr)
    {
        meta->jsr_lights = (hdr_cnt > 0U) ? hdr_cnt : brk0_cnt;
    }
}

static void pb_format_jsr_badge(const pb_entry_t *e, uint8_t hdr_bits, char *buf, size_t sz)
{
    unsigned int n = (unsigned int)pb_jsr_count(e->variant);
    if (hdr_bits != 0U)
    {
        snprintf(buf, sz, "JSR %uF H%u", n, (unsigned int)hdr_bits);
    }
    else if (e->flags & (PB_FLAG_JSR_HDR | PB_FLAG_JSR_BRK0))
    {
        snprintf(buf, sz, "JSR %uF HDR", n);
    }
    else
    {
        snprintf(buf, sz, "JSR %uF", n);
    }
}

static void pb_format_jsr_short(const pb_entry_t *e, uint8_t hdr_bits, char *buf, size_t sz)
{
    unsigned int n = (unsigned int)pb_jsr_count(e->variant);
    if (hdr_bits != 0U)
    {
        snprintf(buf, sz, "J%u H%u", n, (unsigned int)hdr_bits);
    }
    else if (e->flags & (PB_FLAG_JSR_HDR | PB_FLAG_JSR_BRK0))
    {
        snprintf(buf, sz, "J%u HDR", n);
    }
    else
    {
        snprintf(buf, sz, "JSR %uF", n);
    }
}

static void pb_draw_info_overlay(const pb_entry_t *e, uint16_t cur, uint16_t total,
                                 const pb_hist_t *hist, const pb_info_meta_t *meta,
                                 uint8_t hdr_bits)
{
    char line[64];

    /* 1px crisp frame around top-left 160x120 thumbnail */
    lcd_draw_rectangle((uint16_t)(PB_INFO_TH_X - 1U), (uint16_t)(PB_INFO_TH_Y - 1U),
                       (uint16_t)(PB_INFO_TH_X + PB_INFO_TH_W), (uint16_t)(PB_INFO_TH_Y + PB_INFO_TH_H),
                       PB_COL_DARKGRAY);

    /* Top-right: 4 stacked Sony-style Y / R / G / B histograms (y = 5..126, aligns with thumbnail!) */
    pb_draw_single_hist(182U,  5U, "Y", WHITE,         WHITE,         hist->y, hist->total_px);
    pb_draw_single_hist(182U, 36U, "R", PB_COL_HIST_R, PB_COL_HIST_R, hist->r, hist->total_px);
    pb_draw_single_hist(182U, 67U, "G", PB_COL_HIST_G, PB_COL_HIST_G, hist->g, hist->total_px);
    pb_draw_single_hist(182U, 98U, "B", PB_COL_HIST_B, PB_COL_HIST_B, hist->b, hist->total_px);

    /* Horizontal divider separating visual area and photo details */
    lcd_draw_hline(4U, 130U, 312U, PB_COL_DARKGRAY);

    g_back_color = BLACK;

    /* Row 1 (y=134): Folder/File + Format badge + Index */
    if (pb_is_jsr(e))
    {
        unsigned int y = 1980U + (unsigned int)(e->date_packed >> 9);
        unsigned int m = (unsigned int)((e->date_packed >> 5) & 0x0FU);
        unsigned int d = (unsigned int)(e->date_packed & 0x1FU);
        snprintf(line, sizeof(line), "%04u%02u%02u/CDI%05u_%u/",
                 y, m, d, (unsigned int)e->num, (unsigned int)pb_jsr_count(e->variant));
    }
    else
    {
        const char *suf = pb_variant_suffix(e->variant);
        if (e->date_packed == 0U)
        {
            snprintf(line, sizeof(line), "CAYE_CAM/CDI%05u%s", (unsigned int)e->num, suf);
        }
        else
        {
            unsigned int y = 1980U + (unsigned int)(e->date_packed >> 9);
            unsigned int m = (unsigned int)((e->date_packed >> 5) & 0x0FU);
            unsigned int d = (unsigned int)(e->date_packed & 0x1FU);
            snprintf(line, sizeof(line), "%04u%02u%02u/CDI%05u%s", y, m, d, (unsigned int)e->num, suf);
        }
    }
    lcd_show_string(4U, 134U, 132U, 12U, 12U, line, WHITE);

    char        fmt_buf[16];
    const char *fmt_str;
    uint16_t    fmt_col;
    if (pb_is_jsr(e))
    {
        pb_format_jsr_badge(e, hdr_bits, fmt_buf, sizeof(fmt_buf));
        fmt_str = fmt_buf;
        fmt_col = MAGENTA;
    }
    else if ((e->flags & (PB_FLAG_DNG | PB_FLAG_JPG)) == (PB_FLAG_DNG | PB_FLAG_JPG))
    {
        fmt_str = "DNG+JPG";
        fmt_col = YELLOW;
    }
    else if (e->flags & PB_FLAG_DNG)
    {
        fmt_str = (e->variant == PB_VAR_HDR) ? "HDR DNG" : "DNG";
        fmt_col = CYAN;
    }
    else
    {
        fmt_str = "JPG";
        fmt_col = GREEN;
    }
    lcd_show_string(142U, 134U, 78U, 12U, 12U, fmt_str, fmt_col);

    snprintf(line, sizeof(line), "#%u/%u", (unsigned int)(cur + 1U), (unsigned int)total);
    uint16_t idx_w = (uint16_t)(strlen(line) * 6U);
    lcd_show_string((uint16_t)(316U - idx_w), 134U, idx_w, 12U, 12U, line, GRAY);

    /* Row 2 (y=148): Exposure parameters (Shutter, ISO, EV Comp, WB) */
    lcd_show_string(4U, 148U, 30U, 12U, 12U, "EXP", GRAY);
    if (meta->has_exp)
    {
        pb_format_shutter(meta->exp_num, meta->exp_den, line, sizeof(line));
        lcd_show_string(38U, 148U, 48U, 12U, 12U, line, WHITE);
    }
    else
    {
        lcd_show_string(38U, 148U, 48U, 12U, 12U, "--", GRAY);
    }

    if (meta->has_iso)
    {
        snprintf(line, sizeof(line), "ISO %u", (unsigned int)meta->iso);
        lcd_show_string(92U, 148U, 48U, 12U, 12U, line, CYAN);
    }
    else
    {
        lcd_show_string(92U, 148U, 48U, 12U, 12U, "ISO --", GRAY);
    }

    if (meta->has_ev)
    {
        int16_t ev10 = meta->ev_bias_x10;
        uint16_t aev = (ev10 < 0) ? (uint16_t)(-ev10) : (uint16_t)ev10;
        snprintf(line, sizeof(line), "%c%u.%uEV", (ev10 < 0) ? '-' : '+',
                 (unsigned int)(aev / 10U), (unsigned int)(aev % 10U));
        lcd_show_string(146U, 148U, 36U, 12U, 12U, line, (ev10 != 0) ? YELLOW : WHITE);
    }
    else
    {
        lcd_show_string(146U, 148U, 36U, 12U, 12U, "--EV", GRAY);
    }

    if (meta->has_fnum)
    {
        if (meta->fnum_x10 == 0U)
        {
            snprintf(line, sizeof(line), "F0");
        }
        else
        {
            snprintf(line, sizeof(line), "F%u.%u",
                     (unsigned int)(meta->fnum_x10 / 10U), (unsigned int)(meta->fnum_x10 % 10U));
        }
        lcd_show_string(190U, 148U, 30U, 12U, 12U, line, WHITE);
    }
    else
    {
        lcd_show_string(190U, 148U, 30U, 12U, 12U, "F--", GRAY);
    }

    if (meta->wb_kelvin > 0U)
    {
        lcd_show_string(228U, 148U, 12U, 12U, 12U, "WB", GRAY);
        snprintf(line, sizeof(line), "%uK", (unsigned int)meta->wb_kelvin);
        lcd_show_string(246U, 148U, 36U, 12U, 12U, line, YELLOW);
    }

    /* Row 3 (y=162): Capture Date & Time + Focal Length (nominal + 35mm EQ) */
    lcd_show_string(4U, 162U, 30U, 12U, 12U, "DATE", GRAY);
    lcd_show_string(38U, 162U, 120U, 12U, 12U,
                    (meta->datetime[0] != '\0') ? meta->datetime : "--", WHITE);
    if (meta->focal_x10 > 0U || meta->focal_eq35 > 0U)
    {
        uint16_t eq35 = (meta->focal_eq35 > 0U)
                      ? meta->focal_eq35
                      : (uint16_t)(((uint32_t)meta->focal_x10 * 9UL + 5UL) / 10UL);
        lcd_show_string(190U, 162U, 12U, 12U, 12U, "FL", GRAY);
        if (meta->focal_x10 > 0U)
        {
            char fbuf[12];
            if ((meta->focal_x10 % 10U) != 0U)
            {
                snprintf(fbuf, sizeof(fbuf), "%u.%umm",
                         (unsigned int)(meta->focal_x10 / 10U), (unsigned int)(meta->focal_x10 % 10U));
            }
            else
            {
                snprintf(fbuf, sizeof(fbuf), "%umm", (unsigned int)(meta->focal_x10 / 10U));
            }
            snprintf(line, sizeof(line), "%s (EQ %umm)", fbuf, (unsigned int)eq35);
        }
        else
        {
            snprintf(line, sizeof(line), "EQ %umm", (unsigned int)eq35);
        }
        lcd_show_string(208U, 162U, 108U, 12U, 12U, line, CYAN);
    }

    /* Row 4 (y=176): Resolution + Bit depth / Subsampling */
    lcd_show_string(4U, 176U, 30U, 12U, 12U, "SPEC", GRAY);
    if (meta->raw_bps != 0U)
    {
        const char *bits;
        if (meta->raw_bps == 16U)
        {
            if (meta->is_hdr)
            {
                if (meta->white_level > 24000U)
                {
                    bits = "16b-HDR";
                }
                else if (meta->white_level > 6000U)
                {
                    bits = "14b-HDR";
                }
                else
                {
                    bits = "12b-HDR";
                }
            }
            else
            {
                bits = "16b-RAW";
            }
        }
        else
        {
            bits = meta->has_lin_tbl ? "8b-LOG" : "8b-RAW";
        }
        if (pb_is_jsr(e))
        {
            snprintf(line, sizeof(line), "%ux%u  %s (JSR %uF)",
                     (unsigned int)meta->img_w, (unsigned int)meta->img_h,
                     bits, (unsigned int)pb_jsr_count(e->variant));
        }
        else
        {
            snprintf(line, sizeof(line), "%ux%u  %s",
                     (unsigned int)meta->img_w, (unsigned int)meta->img_h, bits);
        }
        lcd_show_string(38U, 176U, 260U, 12U, 12U, line, WHITE);
    }
    else
    {
        const char *sub = "JPEG";
        if (meta->jpg_sub == 1U)      sub = "JPEG 4:2:0";
        else if (meta->jpg_sub == 2U) sub = "JPEG 4:2:2";
        else if (meta->jpg_sub == 3U) sub = "JPEG 4:4:4";
        else if (meta->jpg_sub == 4U) sub = "JPEG GRAY";
        snprintf(line, sizeof(line), "%ux%u  %s",
                 (unsigned int)meta->img_w, (unsigned int)meta->img_h, sub);
        lcd_show_string(38U, 176U, 200U, 12U, 12U, line, WHITE);
    }

    /* Row 5 (y=190): File Size(s) / JSR Burst Summary */
    lcd_show_string(4U, 190U, 30U, 12U, 12U, "SIZE", GRAY);
    uint16_t size_col = WHITE;
    if (pb_is_jsr(e) && meta->jsr_files > 0U)
    {
        uint8_t exp_n = pb_jsr_count(e->variant);
        char    lstr[16];
        char    sstr[16];
        if (meta->jsr_lights == exp_n)
        {
            snprintf(lstr, sizeof(lstr), "%uL", (unsigned int)meta->jsr_lights);
        }
        else
        {
            snprintf(lstr, sizeof(lstr), "%u/%uL", (unsigned int)meta->jsr_lights, (unsigned int)exp_n);
            size_col = YELLOW;
        }
        if (!meta->jsr_has_dark)
        {
            size_col = YELLOW;
        }
        if (meta->jsr_bytes >= (1024UL * 1024UL))
        {
            uint32_t mb10 = (meta->jsr_bytes * 10UL + (512UL * 1024UL)) / (1024UL * 1024UL);
            snprintf(sstr, sizeof(sstr), "%lu.%luMB", (unsigned long)(mb10 / 10UL), (unsigned long)(mb10 % 10UL));
        }
        else
        {
            snprintf(sstr, sizeof(sstr), "%luKB", (unsigned long)((meta->jsr_bytes + 512UL) / 1024UL));
        }
        snprintf(line, sizeof(line), "%s%s  %s (%uF)",
                 lstr, meta->jsr_has_dark ? "+DARK" : " NO-DARK",
                 sstr, (unsigned int)meta->jsr_files);
    }
    else if (meta->dng_bytes > 0UL && meta->jpg_bytes > 0UL)
    {
        snprintf(line, sizeof(line), "DNG %luKB  +  JPG %luKB",
                 (unsigned long)((meta->dng_bytes + 512UL) / 1024UL),
                 (unsigned long)((meta->jpg_bytes + 512UL) / 1024UL));
    }
    else if (meta->dng_bytes > 0UL)
    {
        snprintf(line, sizeof(line), "DNG %luKB",
                 (unsigned long)((meta->dng_bytes + 512UL) / 1024UL));
    }
    else if (meta->jpg_bytes > 0UL)
    {
        snprintf(line, sizeof(line), "JPG %luKB",
                 (unsigned long)((meta->jpg_bytes + 512UL) / 1024UL));
    }
    else
    {
        snprintf(line, sizeof(line), "--");
    }
    lcd_show_string(38U, 190U, 276U, 12U, 12U, line, size_col);

    /* Row 6 (y=204): Live Clipping % & Mean Luma */
    lcd_show_string(4U, 204U, 30U, 12U, 12U, "CLIP", GRAY);
    if (hist->total_px > 0UL)
    {
        uint32_t hi_t10 = (hist->hi_px * 1000UL + (hist->total_px >> 1)) / hist->total_px;
        uint32_t lo_t10 = (hist->lo_px * 1000UL + (hist->total_px >> 1)) / hist->total_px;
        uint32_t avg_y  = (hist->sum_y + (hist->total_px >> 1)) / hist->total_px;
        uint32_t avg_p  = (avg_y * 100UL + 127UL) / 255UL;

        uint16_t hi_col = (hi_t10 >= 10UL) ? RED           : ((hi_t10 > 0UL) ? YELLOW : WHITE);
        uint16_t lo_col = (lo_t10 >= 10UL) ? PB_COL_HIST_B : ((lo_t10 > 0UL) ? CYAN   : WHITE);

        lcd_show_string(38U, 204U, 12U, 12U, 12U, "HI", GRAY);
        snprintf(line, sizeof(line), "%lu.%lu%%", (unsigned long)(hi_t10 / 10UL), (unsigned long)(hi_t10 % 10UL));
        lcd_show_string(54U, 204U, 42U, 12U, 12U, line, hi_col);

        lcd_show_string(106U, 204U, 12U, 12U, 12U, "LO", GRAY);
        snprintf(line, sizeof(line), "%lu.%lu%%", (unsigned long)(lo_t10 / 10UL), (unsigned long)(lo_t10 % 10UL));
        lcd_show_string(122U, 204U, 42U, 12U, 12U, line, lo_col);

        lcd_show_string(182U, 204U, 18U, 12U, 12U, "AVG", GRAY);
        snprintf(line, sizeof(line), "%lu%% (Y:%lu)", (unsigned long)avg_p, (unsigned long)avg_y);
        lcd_show_string(206U, 204U, 108U, 12U, 12U, line, WHITE);
    }
}

static void pb_build_bar_1x(uint8_t bar_buf[12][160], const pb_entry_t *e, uint16_t cur, uint16_t total,
                            uint8_t disp_mode, uint8_t hdr_bits)
{
    char        line[32];
    char        fmt_buf[12];
    const char *fmt_str;
    uint8_t     fmt_col;

    memset(bar_buf, 0, 12U * 160U);

    if (pb_is_jsr(e))
    {
        pb_format_jsr_short(e, hdr_bits, fmt_buf, sizeof(fmt_buf));
        fmt_str = fmt_buf;
        fmt_col = PB_PAL_MAGENTA;
    }
    else if ((e->flags & (PB_FLAG_DNG | PB_FLAG_JPG)) == (PB_FLAG_DNG | PB_FLAG_JPG))
    {
        fmt_str = "DNG+JPG";
        fmt_col = PB_PAL_YELLOW;
    }
    else if (e->flags & PB_FLAG_DNG)
    {
        switch (e->variant)
        {
            case PB_VAR_HDR_0: fmt_str = "DNG _0 "; fmt_col = PB_PAL_CYAN;   break;
            case PB_VAR_HDR_1: fmt_str = "DNG _1 "; fmt_col = PB_PAL_CYAN;   break;
            case PB_VAR_HDR_2: fmt_str = "DNG _2 "; fmt_col = PB_PAL_CYAN;   break;
            case PB_VAR_HDR_3: fmt_str = "DNG _3 "; fmt_col = PB_PAL_CYAN;   break;
            case PB_VAR_HDR:   fmt_str = "HDR DNG"; fmt_col = PB_PAL_YELLOW; break;
            default:           fmt_str = "DNG    "; fmt_col = PB_PAL_CYAN;   break;
        }
    }
    else
    {
        fmt_str = "JPG    ";
        fmt_col = PB_PAL_GREEN;
    }

    snprintf(line, sizeof(line), "CDI%05u", (unsigned int)e->num);
    pb_bar_put_str(bar_buf, 2U, 48U, line, PB_PAL_WHITE);
    pb_bar_put_str(bar_buf, 54U, 42U, fmt_str, fmt_col);

    snprintf(line, sizeof(line), "%u/%u", (unsigned int)(cur + 1U), (unsigned int)total);
    pb_bar_put_str(bar_buf, 100U, 42U, line, PB_PAL_WHITE);

    const char *hint = "4X:ZM SH:CL L:DEL A:ESC";
    if (disp_mode == PB_DISP_CLIP)
    {
        pb_bar_put_str(bar_buf, 148U, 24U, "CLIP", PB_PAL_ORANGE);
        hint = "4X:ZM SH:IN L:DEL A:ESC";
    }
    else if (disp_mode == PB_DISP_INFO)
    {
        pb_bar_put_str(bar_buf, 148U, 24U, "INFO", PB_PAL_CYAN);
        hint = "4X:ZM SH:1X L:DEL A:ESC";
    }

    pb_bar_put_str(bar_buf, 180U, 138U, hint, PB_PAL_HINT);
}

static void pb_build_bar_zoom(uint8_t bar_buf[12][160], const pb_entry_t *e,
                              uint16_t pan_x, uint16_t pan_y, uint8_t hdr_bits)
{
    char line[24];
    memset(bar_buf, 0, 12U * 160U);

    pb_bar_put_str(bar_buf, 2U, 24U, "ZOOM", PB_PAL_YELLOW);

    snprintf(line, sizeof(line), "X:%-3u Y:%-3u", (unsigned int)pan_x, (unsigned int)pan_y);
    pb_bar_put_str(bar_buf, 32U, 66U, line, PB_PAL_CYAN);

    if (pb_is_jsr(e))
    {
        char jtag[12];
        pb_format_jsr_short(e, hdr_bits, jtag, sizeof(jtag));
        pb_bar_put_str(bar_buf, 102U, 42U, jtag, PB_PAL_MAGENTA);
    }

    pb_bar_put_str(bar_buf, 150U, 168U, "DIAL:UD A/S:LR 4X:1X LEV:DEL", PB_PAL_HINT);
}

/* Direct solid-black bottom bar paint (used for INFO dashboard & read-error fallback, no top line) */
static void pb_draw_bar_solid(const uint8_t (*bar_buf)[160])
{
    lcd_set_window(0U, PB_BAR_Y, PB_SCR_W, PB_BAR_H);
    lcd_write_ram_prepare();
    for (uint16_t y = PB_BAR_Y; y < (PB_BAR_Y + PB_BAR_H); y++)
    {
        if (y >= (PB_BAR_Y + 4U) && y < (PB_BAR_Y + 16U))
        {
            const uint8_t *row = bar_buf[y - (PB_BAR_Y + 4U)];
            for (uint16_t i = 0U; i < 160U; i++)
            {
                uint8_t b = row[i];
                lcd_wr_pixel(s_pb_pal[b & 0x0FU]);
                lcd_wr_pixel(s_pb_pal[b >> 4]);
            }
        }
        else
        {
            for (uint16_t x = 0U; x < PB_SCR_W; x++) lcd_wr_pixel(BLACK);
        }
    }
}

static void pb_draw_delete_modal(const pb_entry_t *e, uint8_t confirm_yes)
{
    char info[40];

    g_back_color = BLACK;
    lcd_fill(24U, 54U, 295U, 166U, BLACK);
    lcd_draw_rectangle(24U, 54U, 295U, 166U, RED);
    lcd_draw_rectangle(25U, 55U, 294U, 165U, RED);

    if (pb_is_jsr(e))
    {
        lcd_show_string(100U, 64U, 128U, 16U, 16U, "DELETE JSR SET?", RED);
        snprintf(info, sizeof(info), "CDI%05u_%u/ (ALL FILES)",
                 (unsigned int)e->num, (unsigned int)pb_jsr_count(e->variant));
    }
    else
    {
        const char *fmt = ((e->flags & (PB_FLAG_DNG | PB_FLAG_JPG)) == (PB_FLAG_DNG | PB_FLAG_JPG))
                          ? "DNG+JPG"
                          : ((e->flags & PB_FLAG_DNG) ? "DNG" : "JPG");
        lcd_show_string(108U, 64U, 120U, 16U, 16U, "DELETE PHOTO?", RED);
        snprintf(info, sizeof(info), "CDI%05u%s (%s)", (unsigned int)e->num,
                 pb_variant_suffix(e->variant), fmt);
    }
    uint16_t info_x = (uint16_t)(160U - (strlen(info) * 3U));
    lcd_show_string(info_x, 86U, 240U, 12U, 12U, info, WHITE);
    lcd_show_string(109U, 102U, 140U, 12U, 12U, "CANNOT BE UNDONE!", YELLOW);

    if (!confirm_yes)
    {
        lcd_fill(72U, 122U, 142U, 142U, GREEN);
        g_back_color = GREEN;
        lcd_show_string(99U, 124U, 40U, 16U, 16U, "NO", BLACK);

        lcd_fill(178U, 122U, 248U, 142U, BLACK);
        lcd_draw_rectangle(178U, 122U, 248U, 142U, GRAY);
        g_back_color = BLACK;
        lcd_show_string(201U, 124U, 40U, 16U, 16U, "YES", GRAY);
    }
    else
    {
        lcd_fill(72U, 122U, 142U, 142U, BLACK);
        lcd_draw_rectangle(72U, 122U, 142U, 142U, GRAY);
        g_back_color = BLACK;
        lcd_show_string(99U, 124U, 40U, 16U, 16U, "NO", GRAY);

        lcd_fill(178U, 122U, 248U, 142U, RED);
        g_back_color = RED;
        lcd_show_string(201U, 124U, 40U, 16U, 16U, "YES", WHITE);
    }
    g_back_color = BLACK;
    lcd_show_string(82U, 150U, 180U, 12U, 12U, "DIAL:SEL  SHOT:OK  ASA:ESC", GRAY);
}

/* Returns 1 if entire photo entry was deleted, 0 if cancelled / partial / failed */
static uint8_t pb_run_delete_confirm(c3m1_pb_ctx_t *ctx, pb_entry_t *e, uint32_t *last_act_tick)
{
    if (ctx->is_low_bat_fn != NULL && ctx->is_low_bat_fn())
    {
        pb_draw_center_box("LOW BATTERY!", RED);
        delay_ms(900);
        return 0U;
    }

    uint8_t confirm_yes = 0U;
    uint8_t last_dial   = key_read_shutter_dial();
    pb_draw_delete_modal(e, confirm_yes);

    for (;;)
    {
        if (!ctx->is_sd_present_fn())
        {
            ctx->sd_removed_in_pb = 1U;
            return 0U;
        }
        if (c3m1_power_service_modal(last_act_tick))
        {
            ctx->woke_from_sleep = 1U;
            return 0U;
        }

        uint8_t dial_now = key_read_shutter_dial();
        if (dial_now != last_dial)
        {
            last_dial      = dial_now;
            *last_act_tick = HAL_GetTick();
            confirm_yes    = confirm_yes ? 0U : 1U;
            pb_draw_delete_modal(e, confirm_yes);
        }

        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            *last_act_tick = HAL_GetTick();
        }
        if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG | BTN_EVT_FILM | BTN_EVT_FILM_LONG))
        {
            return 0U;
        }
        if (evts & BTN_EVT_SHOT)
        {
            if (!confirm_yes) return 0U;

            if (pb_is_jsr(e))
            {
                char jsr_dir[44];
                pb_build_jsr_dir(ctx->photo_dir, e, jsr_dir, sizeof(jsr_dir));
                pb_draw_center_box("DELETING...", YELLOW);

                uint8_t del_ok      = 0U;
                uint8_t deleted_any = 0U;
                FILINFO *jfno       = (FILINFO *)ctx->work_buf;
                char    *fpath      = (char *)(ctx->work_buf + 512U);
                for (uint8_t pass = 0U; pass < 2U; pass++)
                {
                    DIR jdir;
                    uint16_t rem = 0U;
                    if (f_opendir(&jdir, jsr_dir) != FR_OK) break;
                    for (;;)
                    {
                        FRESULT rd = f_readdir(&jdir, jfno);
                        if (rd != FR_OK)
                        {
                            rem++;
                            break;
                        }
                        if (jfno->fname[0] == '\0') break;
                        if (jfno->fattrib & AM_DIR)
                        {
                            rem++;
                            continue;
                        }
                        snprintf(fpath, 320U, "%s/%s", jsr_dir, jfno->fname);
                        if (f_unlink(fpath) == FR_OK)
                        {
                            deleted_any = 1U;
                        }
                        else
                        {
                            rem++;
                        }
                    }
                    f_closedir(&jdir);
                    if (rem == 0U && f_unlink(jsr_dir) == FR_OK)
                    {
                        del_ok = 1U;
                        break;
                    }
                }

                if (del_ok)
                {
                    if (e->date_packed != 0U)
                    {
                        char day_dir[32];
                        pb_build_day_dir(ctx->photo_dir, e->date_packed, day_dir, sizeof(day_dir));
                        (void)f_unlink(day_dir);
                    }
                    pb_draw_center_box("DELETED", GREEN);
                    delay_ms(450);
                    return 1U;
                }
                pb_draw_center_box(deleted_any ? "PARTIAL DELETE!" : "DELETE FAILED!", RED);
                delay_ms(800);
                return 0U;
            }

            char path[64];
            uint8_t deleted_any = 0U;
            if (e->flags & PB_FLAG_DNG)
            {
                pb_build_photo_path(ctx->photo_dir, e, "dng", path, sizeof(path));
                if (f_unlink(path) == FR_OK)
                {
                    e->flags &= (uint8_t)~PB_FLAG_DNG;
                    deleted_any = 1U;
                }
            }
            if (e->flags & PB_FLAG_JPG)
            {
                pb_build_photo_path(ctx->photo_dir, e, "jpg", path, sizeof(path));
                if (f_unlink(path) == FR_OK)
                {
                    e->flags &= (uint8_t)~PB_FLAG_JPG;
                    deleted_any = 1U;
                }
            }
            if (deleted_any && e->flags == 0U)
            {
                /* Best-effort prune empty YYYYMMDD folder (FatFs refuses non-empty dirs) */
                if (e->date_packed != 0U)
                {
                    pb_build_day_dir(ctx->photo_dir, e->date_packed, path, sizeof(path));
                    (void)f_unlink(path);
                }
                pb_draw_center_box("DELETED", GREEN);
                delay_ms(450);
                return 1U;
            }
            pb_draw_center_box(deleted_any ? "PARTIAL DELETE!" : "DELETE FAILED!", RED);
            delay_ms(800);
            return 0U;
        }
        delay_ms(10);
    }
}

/* ======================== Main Render Dispatcher ======================== */
static void pb_render_current(c3m1_pb_ctx_t *ctx, const pb_entry_t *e,
                              uint16_t cur, uint16_t total,
                              uint8_t zoom_mode, uint8_t disp_mode,
                              uint16_t pan_x, uint16_t pan_y,
                              uint16_t *out_img_w, uint16_t *out_img_h,
                              uint8_t *scratch_after_idx)
{
    char path[64];
    uint8_t ok = 0U;
    uint8_t hdr_bits = 0U;
    uint8_t bar_ready = 0U;
    uint16_t img_w = 640U, img_h = 480U;
    uint8_t is_info = (!zoom_mode && disp_mode == PB_DISP_INFO) ? 1U : 0U;
    uint8_t clip_en = (!zoom_mode && disp_mode == PB_DISP_CLIP) ? 1U : 0U;

    uint8_t      (*bar_buf)[160] = ctx->bar_buf;
    pb_hist_t      hist;
    pb_info_meta_t meta;
    if (is_info)
    {
        memset(&hist, 0, sizeof(hist));
        memset(&meta, 0, sizeof(meta));
        /* Clear viewport above solid bottom bar before drawing top-left thumbnail + histograms + details */
        lcd_fill(0U, 0U, PB_SCR_W - 1U, PB_BAR_Y - 1U, BLACK);
    }

    if (e->flags & PB_FLAG_DNG)
    {
        pb_build_photo_path(ctx->photo_dir, e, "dng", path, sizeof(path));
        FIL f;
        if (f_open(&f, path, FA_READ) == FR_OK)
        {
            /* Place 512-byte LinearizationTable at start of scratch_after_idx (+0..511) */
            uint16_t *lin_tbl = (uint16_t *)scratch_after_idx;
            dng_info_t info;
            if (dng_parse_header(&f, ctx->work_buf, ctx->work_buf_size, &info, lin_tbl))
            {
                if (info.valid)
                {
                    img_w = (info.raw_w >= 640U) ? 640U : info.raw_w;
                    img_h = info.raw_h;
                }
                if (info.raw_bps == 16U && info.has_bev)
                {
                    if (info.white_level > 24000U)      hdr_bits = 16U;
                    else if (info.white_level > 6000U)  hdr_bits = 14U;
                    else                                hdr_bits = 12U;
                }
                if (is_info)
                {
                    meta.dng_bytes   = (uint32_t)f_size(&f);
                    meta.img_w       = img_w;
                    meta.img_h       = img_h;
                    meta.white_level = info.white_level;
                    meta.raw_bps     = (uint8_t)info.raw_bps;
                    meta.has_lin_tbl = info.has_lin_tbl;
                    meta.is_hdr      = (info.raw_bps == 16U && info.has_bev) ? 1U : 0U;
                    meta.exp_num     = info.exp_num;
                    meta.exp_den     = info.exp_den;
                    meta.fnum_x10    = info.fnum_x10;
                    meta.focal_x10   = info.focal_x10;
                    meta.focal_eq35  = info.focal_eq35;
                    meta.iso         = info.iso;
                    meta.ev_bias_x10 = info.ev_bias_x10;
                    meta.has_exp     = info.has_exp;
                    meta.has_fnum    = info.has_fnum;
                    meta.has_iso     = info.has_iso;
                    meta.has_ev      = info.has_ev;
                    if (info.has_wb)
                    {
                        meta.wb_kelvin = c3m1_wb_gains_to_kelvin(info.wb_r_q8, info.wb_b_q8);
                    }
                    if (info.datetime[0] != '\0')
                    {
                        memcpy(meta.datetime, info.datetime, sizeof(meta.datetime));
                    }
                    if (info.has_thumb)
                    {
                        ok = dng_render_info_thumb(&f, info.thumb_off, scratch_after_idx, &hist);
                    }
                    else if (info.valid)
                    {
                        ok = dng_render_info_raw(&f, &info, lin_tbl, ctx->srgb_lut, scratch_after_idx, &hist);
                    }
                }
                else if (!zoom_mode)
                {
                    if (bar_buf != NULL)
                    {
                        pb_build_bar_1x(bar_buf, e, cur, total, disp_mode, hdr_bits);
                        bar_ready = 1U;
                    }
                    uint16_t nr_iso = (ctx->high_iso_nr && info.has_iso) ? info.iso : 0U;
                    if (info.color_mode != 0U && nr_iso < 800U)
                    {
                        nr_iso = 0U; /* B&W 4x4 embedded thumb has no chroma noise; skip NR below ISO 800 */
                    }
                    if (info.has_thumb)
                    {
                        ok = dng_render_1x_thumb(&f, info.thumb_off, clip_en, nr_iso, scratch_after_idx, bar_buf);
                    }
                    else if (info.valid)
                    {
                        ok = dng_render_1x_raw(&f, &info, lin_tbl, ctx->srgb_lut, clip_en, scratch_after_idx, bar_buf);
                    }
                }
                else if (info.valid)
                {
                    if (bar_buf != NULL)
                    {
                        pb_build_bar_zoom(bar_buf, e, pan_x, pan_y, hdr_bits);
                        bar_ready = 1U;
                    }
                    uint16_t nr_iso = (ctx->high_iso_nr && info.has_iso) ? info.iso : 0U;
                    ok = dng_render_zoom_raw(&f, &info, lin_tbl, ctx->srgb_lut, pan_x, pan_y,
                                             nr_iso, scratch_after_idx, ctx->work_buf, ctx->work_buf_size, bar_buf);
                }
            }
            f_close(&f);
        }

        if (is_info && ok)
        {
            if (meta.datetime[0] == '\0')
            {
                pb_fill_fatfs_datetime(path, meta.datetime, ctx->work_buf);
            }
            if (pb_is_jsr(e))
            {
                char jsr_dir[44];
                pb_build_jsr_dir(ctx->photo_dir, e, jsr_dir, sizeof(jsr_dir));
                pb_scan_jsr_dir_stats(jsr_dir,
                                      (e->flags & (PB_FLAG_JSR_HDR | PB_FLAG_JSR_BRK0)) ? 1U : 0U,
                                      &meta);
            }
            else if (e->flags & PB_FLAG_JPG)
            {
                char jpg_path[64];
                FILINFO jfno;
                pb_build_photo_path(ctx->photo_dir, e, "jpg", jpg_path, sizeof(jpg_path));
                if (f_stat(jpg_path, &jfno) == FR_OK)
                {
                    meta.jpg_bytes = (uint32_t)jfno.fsize;
                }
            }
        }
    }
    if (!ok && (e->flags & PB_FLAG_JPG))
    {
        if (is_info)
        {
            memset(&hist, 0, sizeof(hist));
            memset(&meta, 0, sizeof(meta));
        }
        else if (!bar_ready && bar_buf != NULL)
        {
            if (zoom_mode) pb_build_bar_zoom(bar_buf, e, pan_x, pan_y, hdr_bits);
            else           pb_build_bar_1x(bar_buf, e, cur, total, disp_mode, hdr_bits);
            bar_ready = 1U;
        }
        pb_build_photo_path(ctx->photo_dir, e, "jpg", path, sizeof(path));
        ok = jpg_render_file(path, zoom_mode, disp_mode, pan_x, pan_y,
                             &img_w, &img_h, ctx->work_buf, ctx->work_buf_size,
                             is_info ? &hist : NULL, is_info ? &meta : NULL,
                             scratch_after_idx, is_info ? NULL : bar_buf);
        if (is_info && ok && meta.datetime[0] == '\0')
        {
            pb_fill_fatfs_datetime(path, meta.datetime, ctx->work_buf);
        }
    }

    if (!ok)
    {
        lcd_fill(0U, 0U, PB_SCR_W - 1U, PB_BAR_Y - 1U, BLACK);
        pb_draw_center_box("READ ERROR!", RED);
        if (bar_buf != NULL)
        {
            if (!bar_ready)
            {
                if (zoom_mode) pb_build_bar_zoom(bar_buf, e, pan_x, pan_y, hdr_bits);
                else           pb_build_bar_1x(bar_buf, e, cur, total, disp_mode, hdr_bits);
            }
            pb_draw_bar_solid(bar_buf);
        }
    }
    else if (zoom_mode)
    {
        pb_draw_zoom_minimap(img_w, img_h, pan_x, pan_y);
        if (bar_buf != NULL && (img_w < PB_SCR_W || img_h < PB_VIEW_H))
        {
            pb_draw_bar_solid(bar_buf);
        }
    }
    else if (is_info)
    {
        pb_draw_info_overlay(e, cur, total, &hist, &meta, hdr_bits);
        if (bar_buf != NULL)
        {
            pb_build_bar_1x(bar_buf, e, cur, total, disp_mode, hdr_bits);
            pb_draw_bar_solid(bar_buf);
        }
    }

    if (out_img_w != NULL) *out_img_w = img_w;
    if (out_img_h != NULL) *out_img_h = img_h;

    /* Force fresh continuous dial debounce after blocking SD/LCD render */
    key_dial_reset_debounce();
}

/* ======================== Public Playback Entry ======================== */
void c3m1_pb_run(c3m1_pb_ctx_t *ctx)
{
    ctx->sd_removed_in_pb = 0U;
    ctx->woke_from_sleep  = 0U;

    uint32_t scratch_sz = 0U;
    uint8_t *scratch = c3m1_jpeg_get_scratch_buf(&scratch_sz);
    if (scratch == NULL || scratch_sz < (PB_INDEX_BYTES + PB_AFTER_IDX_NEEDED))
    {
        return;
    }

    pb_entry_t *entries           = (pb_entry_t *)scratch;
    uint8_t    *scratch_after_idx = scratch + PB_INDEX_BYTES;

    lcd_clear(BLACK);
    pb_draw_center_box("LOADING...", CYAN);

    uint16_t total = pb_scan_photos(ctx->photo_dir, entries, PB_MAX_PHOTOS, ctx->skip_hdr_brk, ctx->work_buf);
    if (total == 0U)
    {
        if (!ctx->is_sd_present_fn())
        {
            ctx->sd_removed_in_pb = 1U;
        }
        else
        {
            pb_draw_center_box("NO PHOTOS!", ORANGE);
            delay_ms(900);
        }
        goto pb_exit;
    }

    /* Always start from the newest (last) photo in clean 1X mode */
    uint16_t cur       = (uint16_t)(total - 1U);
    uint8_t  zoom_mode = 0U;
    uint8_t  disp_mode = PB_DISP_CLEAN;
    uint16_t img_w     = 640U;
    uint16_t img_h     = 480U;
    uint16_t pan_x     = 160U;
    uint16_t pan_y     = 130U;

    key_init_state();
    uint8_t  last_dial     = key_read_shutter_dial();
    uint32_t last_act_tick = HAL_GetTick();

    pb_render_current(ctx, &entries[cur], cur, total, 0U, disp_mode, 0U, 0U,
                      &img_w, &img_h, scratch_after_idx);

    for (;;)
    {
        if (!ctx->is_sd_present_fn())
        {
            ctx->sd_removed_in_pb = 1U;
            break;
        }
        if (c3m1_power_service_modal(&last_act_tick))
        {
            ctx->woke_from_sleep = 1U;
            return;
        }

        uint8_t dial_now = key_read_shutter_dial();
        int8_t  dd       = dial_signed_delta(last_dial, dial_now);
        if (dd != 0)
        {
            last_dial     = dial_now;
            last_act_tick = HAL_GetTick();
            if (!zoom_mode)
            {
                /* 1X playback: dial steps forward / backward with wrap-around */
                int32_t next = (int32_t)cur + (int32_t)dd;
                while (next < 0) next += (int32_t)total;
                cur = (uint16_t)(next % (int32_t)total);
                pb_render_current(ctx, &entries[cur], cur, total, 0U, disp_mode, 0U, 0U,
                                  &img_w, &img_h, scratch_after_idx);
            }
            else
            {
                /* Zoom 1:1: dial pans UP / DOWN */
                uint16_t max_y = (img_h > PB_VIEW_H) ? (uint16_t)(img_h - PB_VIEW_H) : 0U;
                int32_t  ny    = (int32_t)pan_y + (int32_t)dd * (int32_t)PB_PAN_STEP_Y;
                if (ny < 0) ny = 0;
                if (ny > (int32_t)max_y) ny = (int32_t)max_y;
                if ((uint16_t)ny != pan_y)
                {
                    pan_y = (uint16_t)ny;
                    pb_render_current(ctx, &entries[cur], cur, total, 1U, PB_DISP_CLEAN, pan_x, pan_y,
                                      &img_w, &img_h, scratch_after_idx);
                }
            }
            last_act_tick = HAL_GetTick();
        }

        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            last_act_tick = HAL_GetTick();
        }
        if (evts & BTN_EVT_FILM)
        {
            /* Short-flick FILM lever (1X or Zoom) -> open delete confirmation modal */
            uint8_t deleted = pb_run_delete_confirm(ctx, &entries[cur], &last_act_tick);
            if (ctx->woke_from_sleep)
            {
                return;
            }
            if (ctx->sd_removed_in_pb)
            {
                break;
            }

            if (deleted)
            {
                for (uint16_t i = cur; i + 1U < total; i++)
                {
                    entries[i] = entries[i + 1U];
                }
                total--;
                if (total == 0U)
                {
                    pb_draw_center_box("NO PHOTOS!", ORANGE);
                    delay_ms(800);
                    break;
                }
                if (cur >= total)
                {
                    cur = (uint16_t)(total - 1U);
                }
                zoom_mode = 0U;
            }

            uint8_t  render_disp = zoom_mode ? PB_DISP_CLEAN : disp_mode;
            uint16_t render_px   = zoom_mode ? pan_x : 0U;
            uint16_t render_py   = zoom_mode ? pan_y : 0U;
            pb_render_current(ctx, &entries[cur], cur, total, zoom_mode, render_disp,
                              render_px, render_py, &img_w, &img_h, scratch_after_idx);
            last_dial = key_read_shutter_dial();
        }
        else if (!zoom_mode)
        {
            if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
            {
                /* Pressing or holding ASA in 1X view -> exit playback */
                break;
            }
            if (evts & BTN_EVT_4X)
            {
                /* Enter 1:1 zoom centered */
                zoom_mode = 1U;
                pan_x = (img_w > PB_SCR_W)  ? (uint16_t)((img_w - PB_SCR_W) / 2U)  : 0U;
                pan_y = (img_h > PB_VIEW_H) ? (uint16_t)((img_h - PB_VIEW_H) / 2U) : 0U;
                pb_render_current(ctx, &entries[cur], cur, total, 1U, PB_DISP_CLEAN, pan_x, pan_y,
                                  &img_w, &img_h, scratch_after_idx);
            }
            else if (evts & BTN_EVT_SHOT)
            {
                /* Short press SHOT -> cycle 1X display: CLEAN -> CLIP -> INFO -> CLEAN */
                disp_mode = (uint8_t)((disp_mode + 1U) % 3U);
                pb_render_current(ctx, &entries[cur], cur, total, 0U, disp_mode, 0U, 0U,
                                  &img_w, &img_h, scratch_after_idx);
            }
        }
        else
        {
            if (evts & BTN_EVT_4X)
            {
                /* Press 4X again -> exit zoom back to current 1X display mode */
                zoom_mode = 0U;
                pb_render_current(ctx, &entries[cur], cur, total, 0U, disp_mode, 0U, 0U,
                                  &img_w, &img_h, scratch_after_idx);
            }
            else if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
            {
                /* ASA -> pan LEFT */
                uint16_t nx = (pan_x > PB_PAN_STEP_X) ? (uint16_t)(pan_x - PB_PAN_STEP_X) : 0U;
                if (nx != pan_x)
                {
                    pan_x = nx;
                    pb_render_current(ctx, &entries[cur], cur, total, 1U, PB_DISP_CLEAN, pan_x, pan_y,
                                      &img_w, &img_h, scratch_after_idx);
                }
            }
            else if (evts & BTN_EVT_SHOT)
            {
                /* SHOT -> pan RIGHT */
                uint16_t max_x = (img_w > PB_SCR_W) ? (uint16_t)(img_w - PB_SCR_W) : 0U;
                uint16_t nx    = (uint16_t)(pan_x + PB_PAN_STEP_X);
                if (nx > max_x) nx = max_x;
                if (nx != pan_x)
                {
                    pan_x = nx;
                    pb_render_current(ctx, &entries[cur], cur, total, 1U, PB_DISP_CLEAN, pan_x, pan_y,
                                      &img_w, &img_h, scratch_after_idx);
                }
            }
        }

        delay_ms(10);
    }

pb_exit:
    /* Wait for all top buttons / lever to be released before returning to viewfinder */
    {
        uint32_t t0 = HAL_GetTick();
        while (key_any_top_button_pressed() && (HAL_GetTick() - t0) < 1000U)
        {
            delay_ms(10);
        }
    }
    key_init_state();
    lcd_clear(BLACK);
}
