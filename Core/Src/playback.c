/**
 * @file        playback.c
 * @brief       On-camera photo playback (DNG embedded preview / legacy RAW / Baseline JPEG,
 *              1:1 zoom & pan with mini viewport map, and paired DNG+JPG deletion).
 */

#include <stdio.h>
#include <string.h>

#include "playback.h"
#include "c3m1_dng.h"
#include "c3m1_jdec.h"
#include "c3m1_jpeg.h"
#include "fatfs.h"
#include "../../BSP/KEY/key.h"
#include "../../BSP/LCD/lcd.h"
#include "../../SYSTEM/delay/delay.h"

#define PB_SCR_W            320U
#define PB_VIEW_H           220U
#define PB_BAR_Y            220U

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
_Static_assert((PB_INDEX_BYTES + sizeof(c3m1_jdec_t)) <= (18U * 644U), "Playback index + JPEG decoder exceeds shared scratch");

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
    uint16_t bl_g;          /* 12-bit Green BlackLevel */
    uint16_t bl_rb;         /* 12-bit R/B BlackLevel */
    uint16_t wb_r_q8;       /* Q8 Red WB gain */
    uint16_t wb_b_q8;       /* Q8 Blue WB gain */
    int16_t  baseline_ev_x100; /* Signed BaselineExposure (0xC62A) * 100 */
    uint32_t exp_num;       /* EXIF ExposureTime numerator */
    uint32_t exp_den;       /* EXIF ExposureTime denominator */
    uint16_t iso;           /* EXIF ISOSpeedRatings */
    int16_t  ev_bias_x10;   /* EXIF ExposureBiasValue in 0.1 EV steps */
    uint8_t  has_lin_tbl;
    uint8_t  has_wb;
    uint8_t  has_bev;
    uint8_t  has_exp;
    uint8_t  has_iso;
    uint8_t  has_ev;
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
    uint16_t iso;           /* EXIF ISOSpeedRatings */
    int16_t  ev_bias_x10;   /* EXIF ExposureBiasValue in 0.1 EV steps */
    uint8_t  raw_bps;       /* 8 or 16 for DNG, 0 for JPG-only */
    uint8_t  has_lin_tbl;
    uint8_t  is_hdr;        /* 1 for 16-bit merged HDR DNG (BaselineExposure present) */
    uint8_t  jpg_sub;       /* 1=4:2:0, 2=4:2:2, 3=4:4:4, 4=GRAY */
    uint8_t  has_exp;
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
    if (num == 0U || num > 65535UL) return 0U;

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
    if (num == 0U || num > 65535UL || name[8] != '_') return 0U;

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
        const char *fsuf = (e->flags & PB_FLAG_JSR_HDR) ? "_hdr" : "";
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
    if (a->num != b->num) return (a->num < b->num) ? 1U : 0U;
    if (a->date_packed != b->date_packed) return (a->date_packed < b->date_packed) ? 1U : 0U;
    return (a->variant < b->variant) ? 1U : 0U;
}

static void pb_add_scanned_file(pb_entry_t *entries, uint16_t *count_ptr,
                                uint16_t max_entries, uint16_t date_packed,
                                uint16_t num, uint8_t flag, uint8_t variant)
{
    uint16_t count = *count_ptr;

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

static uint16_t pb_scan_photos(const char *dir_path, pb_entry_t *entries,
                               uint16_t max_entries, uint8_t skip_hdr_brk)
{
    DIR root_dir;
    FILINFO fno;
    uint16_t count = 0U;

    if (f_opendir(&root_dir, dir_path) != FR_OK)
    {
        return 0U;
    }

    for (;;)
    {
        if (f_readdir(&root_dir, &fno) != FR_OK || fno.fname[0] == '\0')
        {
            break;
        }

        if (fno.fattrib & AM_DIR)
        {
            uint16_t date_packed = 0U;
            if (!parse_yyyymmdd_dir(fno.fname, &date_packed)) continue;

            char sub_path[32];
            pb_build_day_dir(dir_path, date_packed, sub_path, sizeof(sub_path));

            DIR sub_dir;
            if (f_opendir(&sub_dir, sub_path) == FR_OK)
            {
                for (;;)
                {
                    if (f_readdir(&sub_dir, &fno) != FR_OK || fno.fname[0] == '\0')
                    {
                        break;
                    }
                    if (fno.fattrib & AM_DIR)
                    {
                        uint16_t jnum = 0U;
                        uint8_t  jvar = 0U;
                        if (parse_jsr_dir(fno.fname, &jnum, &jvar))
                        {
                            char probe[72];
                            uint8_t jn = pb_jsr_count(jvar);
                            snprintf(probe, sizeof(probe), "%s/CDI%05u_%u/CDI%05u_hdr.dng",
                                     sub_path, (unsigned int)jnum, (unsigned int)jn, (unsigned int)jnum);
                            if (f_stat(probe, NULL) == FR_OK)
                            {
                                pb_add_scanned_file(entries, &count, max_entries, date_packed,
                                                    jnum, (uint8_t)(PB_FLAG_DNG | PB_FLAG_JSR_HDR), jvar);
                            }
                            else
                            {
                                snprintf(probe, sizeof(probe), "%s/CDI%05u_%u/CDI%05u.dng",
                                         sub_path, (unsigned int)jnum, (unsigned int)jn, (unsigned int)jnum);
                                if (f_stat(probe, NULL) == FR_OK)
                                {
                                    pb_add_scanned_file(entries, &count, max_entries, date_packed,
                                                        jnum, PB_FLAG_DNG, jvar);
                                }
                            }
                        }
                        continue;
                    }

                    uint16_t num = 0U;
                    uint8_t  flag = 0U;
                    uint8_t  variant = PB_VAR_NORMAL;
                    if (!parse_cdi_filename(fno.fname, &num, &flag, &variant)) continue;

                    pb_add_scanned_file(entries, &count, max_entries, date_packed, num, flag, variant);
                }
                f_closedir(&sub_dir);
            }
            continue;
        }

        /* Backward-compatible: also scan legacy photos directly under CAYE_CAM/ */
        uint16_t num = 0U;
        uint8_t  flag = 0U;
        uint8_t  variant = PB_VAR_NORMAL;
        if (!parse_cdi_filename(fno.fname, &num, &flag, &variant)) continue;

        pb_add_scanned_file(entries, &count, max_entries, 0U, num, flag, variant);
    }
    f_closedir(&root_dir);

    /* Shell sort ascending by (num, date_packed) so index (count - 1) is always newest */
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
                              int16_t *ev_x10, uint8_t *has_ev)
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

            case 0x8827U: /* ISOSpeedRatings (SHORT) */
                if (type == 3U && count >= 1U)
                {
                    uint16_t v = 0U;
                    if (count == 1U)
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
                    int32_t n = (int32_t)exif_rd32(tb, val, be);
                    int32_t d = (int32_t)exif_rd32(tb, val + 4U, be);
                    if (d != 0)
                    {
                        if (d < 0) { n = -n; d = -d; }
                        int64_t num10 = (int64_t)n * 10;
                        int64_t ev10  = (num10 >= 0) ? ((num10 + (d / 2)) / d) : ((num10 - (d / 2)) / d);
                        if (ev10 < -99) ev10 = -99;
                        if (ev10 >  99) ev10 =  99;
                        *ev_x10 = (int16_t)ev10;
                        *has_ev = 1U;
                    }
                }
                break;

            default:
                break;
        }
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
                    info->bl_g  = rd16(hdr, val + 0U);
                    info->bl_rb = rd16(hdr, val + 2U);
                }
                else if (type == 3U && count == 1U)
                {
                    info->bl_g = info->bl_rb = (uint16_t)(val & 0xFFFFU);
                }
                else if (type == 4U && count == 1U)
                {
                    info->bl_g = info->bl_rb = (uint16_t)val;
                }
                else if (type == 5U && count >= 1U && tiff_range_ok(val, 8U, hdr_len))
                {
                    uint32_t num = rd32(hdr, val);
                    uint32_t den = rd32(hdr, val + 4U);
                    uint16_t bl  = (den > 0U) ? (uint16_t)(num / den) : 0U;
                    info->bl_g = info->bl_rb = bl;
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
                        if (wbb > 1024U) wbb = 1024U;
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
                        if (ev < -500) ev = -500;
                        if (ev >  500) ev =  500;
                        info->baseline_ev_x100 = (int16_t)ev;
                        info->has_bev          = 1U;
                    }
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
    info->bl_g        = 24U;
    info->bl_rb       = 24U;
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
                          &info->ev_bias_x10, &info->has_ev);
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

static const uint8_t s_dither2x2[2][2] = { { 0U, 2U }, { 3U, 1U } };

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

static inline uint16_t pb_apply_rgb565_1x_clip(uint16_t px, uint16_t x, uint16_t y)
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

/* Fast 1X path: division-free exact 2x bilinear upscale 160x120 -> 320x220 in 18-bit RGB666 + 2x2 dither */
static uint8_t __attribute__((optimize("O2"))) dng_render_1x_thumb(FIL *fp, uint32_t thumb_off,
                                                                    uint8_t clip_en,
                                                                    uint8_t *scratch_after_idx)
{
    uint8_t *row_buf[2] = {
        scratch_after_idx,
        scratch_after_idx + (C3M1_DNG_THUMB_W * 3U),
    };
    int16_t loaded_y[2] = { -1, -1 };

    if (f_lseek(fp, thumb_off) != FR_OK) return 0U;
    int16_t file_next_y = 0;

    lcd_set_rgb666(1U);
    lcd_set_window(0U, 0U, PB_SCR_W, PB_VIEW_H);
    lcd_write_ram_prepare();

    for (uint16_t dy = 0U; dy < PB_VIEW_H; dy++)
    {
        int16_t sy0 = (int16_t)(dy >> 1);
        int16_t sy1 = ((dy & 1U) && (sy0 + 1 < (int16_t)C3M1_DNG_THUMB_H)) ? (int16_t)(sy0 + 1) : sy0;

        while (file_next_y <= sy1)
        {
            uint8_t slot = (uint8_t)(file_next_y & 1);
            UINT br = 0U;
            if (f_read(fp, row_buf[slot], C3M1_DNG_THUMB_W * 3U, &br) != FR_OK ||
                br != C3M1_DNG_THUMB_W * 3U)
            {
                lcd_set_rgb666(0U);
                return 0U;
            }
            loaded_y[slot] = file_next_y;
            file_next_y++;
        }

        const uint8_t *r0 = (loaded_y[0] == sy0) ? row_buf[0] : row_buf[1];
        const uint8_t *r1 = (loaded_y[0] == sy1) ? row_buf[0] : row_buf[1];

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
                pb_wr_rgb666_1x_clip(rv0, gv0, bv0, dx, dy);
                pb_wr_rgb666_1x_clip(rm, gm, bm, (uint16_t)(dx + 1U), dy);
            }
            else
            {
                pb_wr_rgb666_dither(rv0, gv0, bv0, dx, dy);
                pb_wr_rgb666_dither(rm, gm, bm, (uint16_t)(dx + 1U), dy);
            }
        }
    }
    lcd_set_rgb666(0U);
    return 1U;
}

/* Fallback 1X path for legacy DNGs without embedded preview (full 320x220 in 18-bit RGB666 + dither) */
static uint8_t __attribute__((optimize("O2"))) dng_render_1x_raw(FIL *fp, const dng_info_t *info,
                                                                  const uint16_t *lin_tbl,
                                                                  const uint8_t *srgb_lut,
                                                                  uint8_t clip_en,
                                                                  uint8_t *scratch_after_idx)
{
    uint32_t bpp        = (info->raw_bps == 16U) ? 2U : 1U;
    uint32_t row_bytes  = (uint32_t)info->raw_w * bpp;
    if (row_bytes > 1300U) return 0U;

    uint8_t *pair_buf = scratch_after_idx;
    uint16_t bl_g  = info->bl_g;
    uint16_t bl_rb = info->bl_rb;
    if (info->raw_bps == 16U && info->white_level != 4095U && info->white_level > 0U)
    {
        bl_g  = (uint16_t)(((uint32_t)bl_g  * 4095UL) / info->white_level);
        bl_rb = (uint16_t)(((uint32_t)bl_rb * 4095UL) / info->white_level);
    }

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

            uint16_t g0 = dng_sample_12(even_row, sx,      info->raw_bps, lin_tbl, info->white_level);
            uint16_t r0 = dng_sample_12(even_row, sx + 1U, info->raw_bps, lin_tbl, info->white_level);
            uint16_t b0 = dng_sample_12(odd_row,  sx,      info->raw_bps, lin_tbl, info->white_level);
            uint16_t g1 = dng_sample_12(odd_row,  sx + 1U, info->raw_bps, lin_tbl, info->white_level);

            uint16_t r12 = sub_bl12(r0, bl_rb);
            uint16_t g12 = sub_bl12(((uint32_t)g0 + g1 + 1U) >> 1, bl_g);
            uint16_t b12 = sub_bl12(b0, bl_rb);

            uint8_t r8, g8, b8;
            c3m1_color_cam12_to_srgb8(r12, g12, b12, info->wb_r_q8, info->wb_b_q8, srgb_lut, &r8, &g8, &b8);
            if (clip_en)
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
    uint8_t *row_buf = scratch_after_idx;
    if (f_lseek(fp, thumb_off) != FR_OK) return 0U;

    lcd_set_rgb666(1U);
    lcd_set_window(PB_INFO_TH_X, PB_INFO_TH_Y, PB_INFO_TH_W, PB_INFO_TH_H);
    lcd_write_ram_prepare();

    for (uint16_t dy = 0U; dy < PB_INFO_TH_H; dy++)
    {
        UINT br = 0U;
        if (f_read(fp, row_buf, C3M1_DNG_THUMB_W * 3U, &br) != FR_OK ||
            br != C3M1_DNG_THUMB_W * 3U)
        {
            lcd_set_rgb666(0U);
            return 0U;
        }
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
    lcd_set_rgb666(0U);
    return 1U;
}

/* INFO-mode fallback for legacy DNGs without embedded preview: 160x120 subsampled RAW + CLIP + YRGB hist */
static uint8_t __attribute__((optimize("O2"))) dng_render_info_raw(FIL *fp, const dng_info_t *info,
                                                                    const uint16_t *lin_tbl,
                                                                    const uint8_t *srgb_lut,
                                                                    uint8_t *scratch_after_idx,
                                                                    pb_hist_t *hist)
{
    uint32_t bpp       = (info->raw_bps == 16U) ? 2U : 1U;
    uint32_t row_bytes = (uint32_t)info->raw_w * bpp;
    if (row_bytes > 1300U) return 0U;

    uint8_t *pair_buf = scratch_after_idx;
    uint16_t bl_g  = info->bl_g;
    uint16_t bl_rb = info->bl_rb;
    if (info->raw_bps == 16U && info->white_level != 4095U && info->white_level > 0U)
    {
        bl_g  = (uint16_t)(((uint32_t)bl_g  * 4095UL) / info->white_level);
        bl_rb = (uint16_t)(((uint32_t)bl_rb * 4095UL) / info->white_level);
    }

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

            uint16_t g0 = dng_sample_12(even_row, sx,      info->raw_bps, lin_tbl, info->white_level);
            uint16_t r0 = dng_sample_12(even_row, sx + 1U, info->raw_bps, lin_tbl, info->white_level);
            uint16_t b0 = dng_sample_12(odd_row,  sx,      info->raw_bps, lin_tbl, info->white_level);
            uint16_t g1 = dng_sample_12(odd_row,  sx + 1U, info->raw_bps, lin_tbl, info->white_level);

            uint16_t r12 = sub_bl12(r0, bl_rb);
            uint16_t g12 = sub_bl12(((uint32_t)g0 + g1 + 1U) >> 1, bl_g);
            uint16_t b12 = sub_bl12(b0, bl_rb);

            uint8_t r8, g8, b8;
            c3m1_color_cam12_to_srgb8(r12, g12, b12, info->wb_r_q8, info->wb_b_q8, srgb_lut, &r8, &g8, &b8);
            pb_hist_add_rgb8(hist, r8, g8, b8);
            pb_wr_rgb666_1x_clip(r8, g8, b8, dx, dy);
        }
    }
    lcd_set_rgb666(0U);
    return 1U;
}

/* True 1:1 RAW crop zoom renderer (fast branch-free 8-bit path + 18-bit RGB666 + 2x2 dither) */
static uint8_t __attribute__((optimize("O2"))) dng_render_zoom_raw(FIL *fp, const dng_info_t *info,
                                                                    const uint16_t *lin_tbl,
                                                                    const uint8_t *srgb_lut,
                                                                    uint16_t pan_x, uint16_t pan_y,
                                                                    uint8_t *scratch_after_idx)
{
    uint32_t bpp       = (info->raw_bps == 16U) ? 2U : 1U;
    uint32_t row_bytes = (uint32_t)info->raw_w * bpp;
    if (row_bytes > 1300U) return 0U;

    uint8_t *ring[3] = {
        scratch_after_idx,
        scratch_after_idx + row_bytes,
        scratch_after_idx + row_bytes * 2U,
    };

    uint16_t bl_g  = info->bl_g;
    uint16_t bl_rb = info->bl_rb;
    if (info->raw_bps == 16U && info->white_level != 4095U && info->white_level > 0U)
    {
        bl_g  = (uint16_t)(((uint32_t)bl_g  * 4095UL) / info->white_level);
        bl_rb = (uint16_t)(((uint32_t)bl_rb * 4095UL) / info->white_level);
    }

    int32_t first_row = (pan_y > 0U) ? ((int32_t)pan_y - 1) : 0;
    if (f_lseek(fp, info->raw_off + (uint32_t)first_row * row_bytes) != FR_OK) return 0U;
    int32_t file_next_y = first_row;

    const uint16_t raw_w   = info->raw_w;
    const uint16_t wb_r    = info->wb_r_q8;
    const uint16_t wb_b    = info->wb_b_q8;
    const uint8_t  is_8bit = (info->raw_bps == 8U) ? 1U : 0U;

    lcd_set_rgb666(1U);
    lcd_set_window(0U, 0U, PB_SCR_W, PB_VIEW_H);
    lcd_write_ram_prepare();

    for (uint16_t dy = 0U; dy < PB_VIEW_H; dy++)
    {
        int32_t sy   = (int32_t)pan_y + (int32_t)dy;
        int32_t y_up = (sy > 0) ? (sy - 1) : 0;
        int32_t y_dn = (sy + 1 < (int32_t)info->raw_h) ? (sy + 1) : ((int32_t)info->raw_h - 1);

        while (file_next_y <= y_dn)
        {
            uint8_t slot = (uint8_t)(file_next_y % 3);
            UINT br = 0U;
            if (f_read(fp, ring[slot], row_bytes, &br) != FR_OK || br != row_bytes)
            {
                lcd_set_rgb666(0U);
                return 0U;
            }
            file_next_y++;
        }

        const uint8_t *rp = ring[y_up % 3];
        const uint8_t *rc = ring[sy   % 3];
        const uint8_t *rn = ring[y_dn % 3];

        if (is_8bit)
        {
            /* Fast branch-free 8-bit LinearizationTable path (all C3M1 v0.1.2+ DNGs) */
            if ((sy & 1) == 0)
            {
                for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
                {
                    uint16_t x  = (uint16_t)(pan_x + dx);
                    if (x >= raw_w) x = (uint16_t)(raw_w - 1U);
                    uint16_t xl = (x > 0U) ? (uint16_t)(x - 1U) : 0U;
                    uint16_t xr = (x + 1U < raw_w) ? (uint16_t)(x + 1U) : (uint16_t)(raw_w - 1U);
                    uint32_t r12, g12, b12;
                    if ((x & 1U) == 0U)
                    {
                        g12 = lin_tbl[rc[x]];
                        r12 = ((uint32_t)lin_tbl[rc[xl]] + lin_tbl[rc[xr]] + 1U) >> 1;
                        b12 = ((uint32_t)lin_tbl[rp[x]]  + lin_tbl[rn[x]]  + 1U) >> 1;
                    }
                    else
                    {
                        r12 = lin_tbl[rc[x]];
                        g12 = ((uint32_t)lin_tbl[rc[xl]] + lin_tbl[rc[xr]] +
                                         lin_tbl[rp[x]]  + lin_tbl[rn[x]]  + 2U) >> 2;
                        b12 = ((uint32_t)lin_tbl[rp[xl]] + lin_tbl[rp[xr]] +
                                         lin_tbl[rn[xl]] + lin_tbl[rn[xr]] + 2U) >> 2;
                    }
                    uint8_t r8, g8, b8;
                    c3m1_color_cam12_to_srgb8(sub_bl12(r12, bl_rb), sub_bl12(g12, bl_g),
                                              sub_bl12(b12, bl_rb), wb_r, wb_b, srgb_lut, &r8, &g8, &b8);
                    pb_wr_rgb666_dither(r8, g8, b8, dx, dy);
                }
            }
            else
            {
                for (uint16_t dx = 0U; dx < PB_SCR_W; dx++)
                {
                    uint16_t x  = (uint16_t)(pan_x + dx);
                    if (x >= raw_w) x = (uint16_t)(raw_w - 1U);
                    uint16_t xl = (x > 0U) ? (uint16_t)(x - 1U) : 0U;
                    uint16_t xr = (x + 1U < raw_w) ? (uint16_t)(x + 1U) : (uint16_t)(raw_w - 1U);
                    uint32_t r12, g12, b12;
                    if ((x & 1U) == 0U)
                    {
                        b12 = lin_tbl[rc[x]];
                        g12 = ((uint32_t)lin_tbl[rc[xl]] + lin_tbl[rc[xr]] +
                                         lin_tbl[rp[x]]  + lin_tbl[rn[x]]  + 2U) >> 2;
                        r12 = ((uint32_t)lin_tbl[rp[xl]] + lin_tbl[rp[xr]] +
                                         lin_tbl[rn[xl]] + lin_tbl[rn[xr]] + 2U) >> 2;
                    }
                    else
                    {
                        g12 = lin_tbl[rc[x]];
                        b12 = ((uint32_t)lin_tbl[rc[xl]] + lin_tbl[rc[xr]] + 1U) >> 1;
                        r12 = ((uint32_t)lin_tbl[rp[x]]  + lin_tbl[rn[x]]  + 1U) >> 1;
                    }
                    uint8_t r8, g8, b8;
                    c3m1_color_cam12_to_srgb8(sub_bl12(r12, bl_rb), sub_bl12(g12, bl_g),
                                              sub_bl12(b12, bl_rb), wb_r, wb_b, srgb_lut, &r8, &g8, &b8);
                    pb_wr_rgb666_dither(r8, g8, b8, dx, dy);
                }
            }
        }
        else if (info->has_bev && info->baseline_ev_x100 > 0)
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
                uint16_t xl = (x > 0U) ? (uint16_t)(x - 1U) : 0U;
                uint16_t xr = (x + 1U < raw_w) ? (uint16_t)(x + 1U) : (uint16_t)(raw_w - 1U);
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
                uint8_t r8, g8, b8;
                c3m1_color_hdr_to_srgb8((uint16_t)r16, (uint16_t)g16, (uint16_t)b16,
                                        wb_r, wb_b, ref_white, (uint32_t)wl,
                                        srgb_lut, &r8, &g8, &b8);
                pb_wr_rgb666_dither(r8, g8, b8, dx, dy);
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
                uint16_t xl = (x > 0U) ? (uint16_t)(x - 1U) : 0U;
                uint16_t xr = (x + 1U < raw_w) ? (uint16_t)(x + 1U) : (uint16_t)(raw_w - 1U);
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
                uint16_t rc12 = sub_bl12(r12, bl_rb);
                uint16_t gc12 = sub_bl12(g12, bl_g);
                uint16_t bc12 = sub_bl12(b12, bl_rb);
                uint8_t r8, g8, b8;
                c3m1_color_cam12_to_srgb8(rc12, gc12, bc12, wb_r, wb_b, srgb_lut, &r8, &g8, &b8);
                pb_wr_rgb666_dither(r8, g8, b8, dx, dy);
            }
        }
    }
    lcd_set_rgb666(0U);
    return 1U;
}

/* ======================== JPEG Renderers (via c3m1_jdec) ======================== */
typedef struct
{
    FIL       *fp;
    uint16_t   img_w;
    uint16_t   img_h;
    uint16_t   pan_x;
    uint16_t   pan_y;
    uint8_t    clip_en;
    pb_hist_t *hist;
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
                                          &meta->ev_bias_x10, &meta->has_ev);
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

static inline uint16_t rgb565_avg4(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{
    uint32_t r = ((a >> 11) + (b >> 11) + (c >> 11) + (d >> 11) + 2U) >> 2;
    uint32_t g = (((a >> 5) & 63U) + ((b >> 5) & 63U) + ((c >> 5) & 63U) + ((d >> 5) & 63U) + 2U) >> 2;
    uint32_t bl = ((a & 31U) + (b & 31U) + (c & 31U) + (d & 31U) + 2U) >> 2;
    return (uint16_t)((r << 11) | (g << 5) | bl);
}

static void jpg_1x_mcu_cb(void *user_ctx, uint16_t mx, uint16_t my,
                          uint8_t mw, uint8_t mh, const uint16_t *rgb565)
{
    jpg_render_ctx_t *jc = (jpg_render_ctx_t *)user_ctx;
    uint16_t w = jc->img_w;
    uint16_t h = jc->img_h;

    uint16_t mx1 = (uint16_t)(mx + mw);
    uint16_t my1 = (uint16_t)(my + mh);
    if (mx1 > w) mx1 = w;
    if (my1 > h) my1 = h;

    uint16_t dx0 = (uint16_t)(((uint32_t)mx  * PB_1X_W) / w);
    uint16_t dx1 = (uint16_t)(((uint32_t)mx1 * PB_1X_W) / w);
    uint16_t dy0 = (uint16_t)(((uint32_t)my  * PB_1X_H) / h);
    uint16_t dy1 = (uint16_t)(((uint32_t)my1 * PB_1X_H) / h);
    if (dx1 > PB_1X_W)  dx1 = PB_1X_W;
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
            lcd_wr_pixel(jc->clip_en ? pb_apply_rgb565_1x_clip(px, dx, dy) : px);
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

    lcd_set_window((uint16_t)(sx0 - vx0), (uint16_t)(sy0 - vy0),
                   (uint16_t)(sx1 - sx0), (uint16_t)(sy1 - sy0));
    lcd_write_ram_prepare();

    for (uint16_t sy = sy0; sy < sy1; sy++)
    {
        const uint16_t *row = &rgb565[(uint16_t)(sy - my) * mw + (sx0 - mx)];
        uint16_t cnt = (uint16_t)(sx1 - sx0);
        for (uint16_t i = 0U; i < cnt; i++)
        {
            lcd_wr_pixel(row[i]);
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
                               uint8_t *scratch_after_idx)
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
        .fp      = &f,
        .img_w   = 0U,
        .img_h   = 0U,
        .pan_x   = pan_x,
        .pan_y   = pan_y,
        .clip_en = (disp_mode == PB_DISP_CLIP) ? 1U : 0U,
        .hist    = hist,
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
        ok = c3m1_jdec_decode(jd, pan_x, pan_y, PB_SCR_W, PB_VIEW_H, jpg_zoom_mcu_cb, &jc);
    }
    else if (disp_mode == PB_DISP_INFO)
    {
        ok = c3m1_jdec_decode(jd, 0U, 0U, jd->width, jd->height, jpg_info_mcu_cb, &jc);
    }
    else
    {
        /* Stop decoding once past visible top 220 rows of 320x240 projection */
        uint16_t stop_h = (uint16_t)(((uint32_t)jd->height * PB_VIEW_H + PB_1X_H - 1U) / PB_1X_H);
        if (stop_h > jd->height) stop_h = jd->height;
        ok = c3m1_jdec_decode(jd, 0U, 0U, jd->width, stop_h, jpg_1x_mcu_cb, &jc);
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

static uint16_t pb_wb_gains_to_kelvin(uint16_t wb_r_q8, uint16_t wb_b_q8)
{
    static const uint16_t r_tbl[16] = {
        34U, 47U, 66U, 87U, 105U, 133U, 161U, 189U,
        216U, 241U, 264U, 286U, 306U, 325U, 342U, 370U,
    };
    static const uint16_t k_tbl[16] = {
        2500U, 2800U, 3200U, 3600U, 4000U, 4500U, 5000U, 5500U,
        6000U, 6500U, 7000U, 7500U, 8000U, 8500U, 9000U, 9900U,
    };
    if (wb_b_q8 == 0U) return 5500U;
    uint32_t ratio = ((uint32_t)wb_r_q8 * 256UL + (wb_b_q8 >> 1)) / wb_b_q8;
    if (ratio <= r_tbl[0])  return k_tbl[0];
    if (ratio >= r_tbl[15]) return k_tbl[15];
    for (uint8_t i = 0U; i < 15U; i++)
    {
        if (ratio <= r_tbl[i + 1U])
        {
            uint32_t span = (uint32_t)(r_tbl[i + 1U] - r_tbl[i]);
            uint32_t k    = (uint32_t)k_tbl[i] +
                            (((ratio - r_tbl[i]) * (uint32_t)(k_tbl[i + 1U] - k_tbl[i]) + (span >> 1)) / span);
            return (uint16_t)(((k + 50UL) / 100UL) * 100UL);
        }
    }
    return 5500U;
}

static void pb_fill_fatfs_datetime(const char *path, char out[20])
{
    FILINFO fno;
    if (f_stat(path, &fno) != FR_OK) return;
    if (fno.fdate == 0U) return;

    unsigned int y = 1980U + (unsigned int)((fno.fdate >> 9) & 0x7FU);
    unsigned int m = (unsigned int)((fno.fdate >> 5) & 0x0FU);
    unsigned int d = (unsigned int)(fno.fdate & 0x1FU);
    unsigned int H = (unsigned int)((fno.ftime >> 11) & 0x1FU);
    unsigned int M = (unsigned int)((fno.ftime >> 5) & 0x3FU);
    unsigned int S = (unsigned int)((fno.ftime & 0x1FU) * 2U);
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
            if (pb_str_ends_with_ci(jfno.fname, "_hdr.dng") && meta->jsr_lights < 255U)
            {
                meta->jsr_lights++;
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
}

static void pb_format_jsr_badge(const pb_entry_t *e, uint8_t hdr_bits, char *buf, size_t sz)
{
    unsigned int n = (unsigned int)pb_jsr_count(e->variant);
    if (hdr_bits != 0U)
    {
        snprintf(buf, sz, "JSR %uF H%u", n, (unsigned int)hdr_bits);
    }
    else if (e->flags & PB_FLAG_JSR_HDR)
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
    else if (e->flags & PB_FLAG_JSR_HDR)
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

    if (meta->wb_kelvin > 0U)
    {
        lcd_show_string(190U, 148U, 12U, 12U, 12U, "WB", GRAY);
        snprintf(line, sizeof(line), "%uK", (unsigned int)meta->wb_kelvin);
        lcd_show_string(208U, 148U, 36U, 12U, 12U, line, YELLOW);
    }

    /* Row 3 (y=162): Capture Date & Time */
    lcd_show_string(4U, 162U, 30U, 12U, 12U, "DATE", GRAY);
    lcd_show_string(38U, 162U, 180U, 12U, 12U,
                    (meta->datetime[0] != '\0') ? meta->datetime : "--", WHITE);

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

static void pb_draw_bar_1x(const pb_entry_t *e, uint16_t cur, uint16_t total,
                           uint8_t disp_mode, uint8_t hdr_bits)
{
    char        line[32];
    char        fmt_buf[12];
    const char *fmt_str;
    uint16_t    fmt_col;

    if (pb_is_jsr(e))
    {
        pb_format_jsr_short(e, hdr_bits, fmt_buf, sizeof(fmt_buf));
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
        switch (e->variant)
        {
            case PB_VAR_HDR_0: fmt_str = "DNG _0 "; fmt_col = CYAN;   break;
            case PB_VAR_HDR_1: fmt_str = "DNG _1 "; fmt_col = CYAN;   break;
            case PB_VAR_HDR_2: fmt_str = "DNG _2 "; fmt_col = CYAN;   break;
            case PB_VAR_HDR_3: fmt_str = "DNG _3 "; fmt_col = CYAN;   break;
            case PB_VAR_HDR:   fmt_str = "HDR DNG"; fmt_col = YELLOW; break;
            default:           fmt_str = "DNG    "; fmt_col = CYAN;   break;
        }
    }
    else
    {
        fmt_str = "JPG    ";
        fmt_col = GREEN;
    }

    g_back_color = BLACK;
    lcd_fill(0U, PB_BAR_Y, PB_SCR_W - 1U, 239U, BLACK);
    lcd_draw_hline(0U, PB_BAR_Y, PB_SCR_W, GRAY);

    snprintf(line, sizeof(line), "CDI%05u", (unsigned int)e->num);
    lcd_show_string(2U, PB_BAR_Y + 4U, 48U, 12U, 12U, line, WHITE);

    lcd_show_string(54U, PB_BAR_Y + 4U, 42U, 12U, 12U, fmt_str, fmt_col);

    snprintf(line, sizeof(line), "%u/%u", (unsigned int)(cur + 1U), (unsigned int)total);
    lcd_show_string(100U, PB_BAR_Y + 4U, 42U, 12U, 12U, line, WHITE);

    const char *hint = "4X:ZM SH:CL L:DEL A:ESC";
    if (disp_mode == PB_DISP_CLIP)
    {
        lcd_show_string(148U, PB_BAR_Y + 4U, 24U, 12U, 12U, "CLIP", ORANGE);
        hint = "4X:ZM SH:IN L:DEL A:ESC";
    }
    else if (disp_mode == PB_DISP_INFO)
    {
        lcd_show_string(148U, PB_BAR_Y + 4U, 24U, 12U, 12U, "INFO", CYAN);
        hint = "4X:ZM SH:1X L:DEL A:ESC";
    }

    lcd_show_string(180U, PB_BAR_Y + 4U, 138U, 12U, 12U, hint, GRAY);
}

static void pb_draw_bar_zoom(const pb_entry_t *e, uint16_t pan_x, uint16_t pan_y, uint8_t hdr_bits)
{
    char line[24];
    g_back_color = BLACK;
    lcd_fill(0U, PB_BAR_Y, PB_SCR_W - 1U, 239U, BLACK);
    lcd_draw_hline(0U, PB_BAR_Y, PB_SCR_W, GRAY);

    lcd_show_string(2U, PB_BAR_Y + 4U, 24U, 12U, 12U, "ZOOM", YELLOW);

    snprintf(line, sizeof(line), "X:%-3u Y:%-3u", (unsigned int)pan_x, (unsigned int)pan_y);
    lcd_show_string(32U, PB_BAR_Y + 4U, 66U, 12U, 12U, line, CYAN);

    if (pb_is_jsr(e))
    {
        char jtag[12];
        pb_format_jsr_short(e, hdr_bits, jtag, sizeof(jtag));
        lcd_show_string(102U, PB_BAR_Y + 4U, 42U, 12U, 12U, jtag, MAGENTA);
    }

    lcd_show_string(150U, PB_BAR_Y + 4U, 168U, 12U, 12U, "DIAL:UD A/S:LR 4X:1X LEV:DEL", GRAY);
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
static uint8_t pb_run_delete_confirm(c3m1_pb_ctx_t *ctx, pb_entry_t *e)
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

        uint8_t dial_now = key_read_shutter_dial();
        if (dial_now != last_dial)
        {
            last_dial   = dial_now;
            confirm_yes = confirm_yes ? 0U : 1U;
            pb_draw_delete_modal(e, confirm_yes);
        }

        uint8_t evts = key_poll_events();
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
                for (uint8_t pass = 0U; pass < 2U; pass++)
                {
                    DIR jdir;
                    FILINFO jfno;
                    uint16_t rem = 0U;
                    if (f_opendir(&jdir, jsr_dir) != FR_OK) break;
                    for (;;)
                    {
                        if (f_readdir(&jdir, &jfno) != FR_OK || jfno.fname[0] == '\0') break;
                        if (jfno.fattrib & AM_DIR)
                        {
                            rem++;
                            continue;
                        }
                        char fpath[112];
                        snprintf(fpath, sizeof(fpath), "%s/%.64s", jsr_dir, jfno.fname);
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
    uint16_t img_w = 640U, img_h = 480U;
    uint8_t is_info = (!zoom_mode && disp_mode == PB_DISP_INFO) ? 1U : 0U;
    uint8_t clip_en = (!zoom_mode && disp_mode == PB_DISP_CLIP) ? 1U : 0U;

    pb_hist_t      hist;
    pb_info_meta_t meta;
    if (is_info)
    {
        memset(&hist, 0, sizeof(hist));
        memset(&meta, 0, sizeof(meta));
        /* Clear viewport before drawing top-left thumbnail + histograms + details */
        lcd_fill(0U, 0U, PB_SCR_W - 1U, PB_VIEW_H - 1U, BLACK);
    }

    if (e->flags & PB_FLAG_DNG)
    {
        pb_build_photo_path(ctx->photo_dir, e, "dng", path, sizeof(path));
        FIL f;
        if (f_open(&f, path, FA_READ) == FR_OK)
        {
            /* Place 512-byte LinearizationTable at end of scratch (after 3-row 3,864 B RAW ring) */
            uint16_t *lin_tbl = (uint16_t *)(scratch_after_idx + 3904U);
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
                    meta.iso         = info.iso;
                    meta.ev_bias_x10 = info.ev_bias_x10;
                    meta.has_exp     = info.has_exp;
                    meta.has_iso     = info.has_iso;
                    meta.has_ev      = info.has_ev;
                    if (info.has_wb)
                    {
                        meta.wb_kelvin = pb_wb_gains_to_kelvin(info.wb_r_q8, info.wb_b_q8);
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
                    if (info.has_thumb)
                    {
                        ok = dng_render_1x_thumb(&f, info.thumb_off, clip_en, scratch_after_idx);
                    }
                    else if (info.valid)
                    {
                        ok = dng_render_1x_raw(&f, &info, lin_tbl, ctx->srgb_lut, clip_en, scratch_after_idx);
                    }
                }
                else if (info.valid)
                {
                    ok = dng_render_zoom_raw(&f, &info, lin_tbl, ctx->srgb_lut, pan_x, pan_y, scratch_after_idx);
                }
            }
            f_close(&f);
        }

        if (is_info && ok)
        {
            if (meta.datetime[0] == '\0')
            {
                pb_fill_fatfs_datetime(path, meta.datetime);
            }
            if (pb_is_jsr(e))
            {
                char jsr_dir[44];
                pb_build_jsr_dir(ctx->photo_dir, e, jsr_dir, sizeof(jsr_dir));
                pb_scan_jsr_dir_stats(jsr_dir, (e->flags & PB_FLAG_JSR_HDR) ? 1U : 0U, &meta);
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
        pb_build_photo_path(ctx->photo_dir, e, "jpg", path, sizeof(path));
        ok = jpg_render_file(path, zoom_mode, disp_mode, pan_x, pan_y,
                             &img_w, &img_h, ctx->work_buf, ctx->work_buf_size,
                             is_info ? &hist : NULL, is_info ? &meta : NULL,
                             scratch_after_idx);
        if (is_info && ok && meta.datetime[0] == '\0')
        {
            pb_fill_fatfs_datetime(path, meta.datetime);
        }
    }

    if (!ok)
    {
        lcd_fill(0U, 0U, PB_SCR_W - 1U, PB_VIEW_H - 1U, BLACK);
        pb_draw_center_box("READ ERROR!", RED);
    }
    else if (zoom_mode)
    {
        pb_draw_zoom_minimap(img_w, img_h, pan_x, pan_y);
    }
    else if (is_info)
    {
        pb_draw_info_overlay(e, cur, total, &hist, &meta, hdr_bits);
    }

    if (out_img_w != NULL) *out_img_w = img_w;
    if (out_img_h != NULL) *out_img_h = img_h;

    if (!zoom_mode)
    {
        pb_draw_bar_1x(e, cur, total, disp_mode, hdr_bits);
    }
    else
    {
        pb_draw_bar_zoom(e, pan_x, pan_y, hdr_bits);
    }

    /* Force fresh continuous dial debounce after blocking SD/LCD render */
    key_dial_reset_debounce();
}

/* ======================== Public Playback Entry ======================== */
void c3m1_pb_run(c3m1_pb_ctx_t *ctx)
{
    ctx->sd_removed_in_pb = 0U;

    uint32_t scratch_sz = 0U;
    uint8_t *scratch = c3m1_jpeg_get_scratch_buf(&scratch_sz);
    if (scratch == NULL || scratch_sz < (PB_INDEX_BYTES + sizeof(c3m1_jdec_t)))
    {
        return;
    }

    pb_entry_t *entries           = (pb_entry_t *)scratch;
    uint8_t    *scratch_after_idx = scratch + PB_INDEX_BYTES;

    lcd_clear(BLACK);
    pb_draw_center_box("LOADING...", CYAN);

    uint16_t total = pb_scan_photos(ctx->photo_dir, entries, PB_MAX_PHOTOS, ctx->skip_hdr_brk);
    if (total == 0U)
    {
        pb_draw_center_box("NO PHOTOS!", ORANGE);
        delay_ms(900);
        lcd_clear(BLACK);
        return;
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
    uint8_t last_dial = key_read_shutter_dial();

    pb_render_current(ctx, &entries[cur], cur, total, 0U, disp_mode, 0U, 0U,
                      &img_w, &img_h, scratch_after_idx);

    for (;;)
    {
        if (!ctx->is_sd_present_fn())
        {
            ctx->sd_removed_in_pb = 1U;
            break;
        }

        uint8_t dial_now = key_read_shutter_dial();
        int8_t  dd       = dial_signed_delta(last_dial, dial_now);
        if (dd != 0)
        {
            last_dial = dial_now;
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
        }

        uint8_t evts = key_poll_events();
        if (evts & BTN_EVT_FILM)
        {
            /* Short-flick FILM lever (1X or Zoom) -> open delete confirmation modal */
            uint8_t deleted = pb_run_delete_confirm(ctx, &entries[cur]);
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

    /* Wait for all top buttons / lever to be released before returning to viewfinder */
    uint32_t t0 = HAL_GetTick();
    while (key_any_top_button_pressed() && (HAL_GetTick() - t0) < 1000U)
    {
        delay_ms(10);
    }
    key_init_state();
    lcd_clear(BLACK);
}
