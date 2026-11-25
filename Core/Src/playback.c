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

/* Zoom 1:1 pan step sizes (pixels) */
#define PB_PAN_STEP_X       40U
#define PB_PAN_STEP_Y       32U

/* Photo index capacity inside shared 11.3 KB scratch buffer: 1024 * 4 = 4096 B */
#define PB_MAX_PHOTOS       1024U
#define PB_INDEX_BYTES      (PB_MAX_PHOTOS * sizeof(pb_entry_t))

#define PB_FLAG_DNG         (1U << 0)
#define PB_FLAG_JPG         (1U << 1)

typedef struct
{
    uint16_t num;       /* 1..65535 from CDIxxxxx */
    uint8_t  flags;     /* PB_FLAG_DNG | PB_FLAG_JPG */
    uint8_t  reserved;
} pb_entry_t;

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
    uint8_t  has_lin_tbl;
} dng_info_t;

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
static uint8_t parse_cdi_filename(const char *name, uint16_t *out_num, uint8_t *out_flag)
{
    /* Match CDIxxxxx.dng or CDIxxxxx.jpg (case-insensitive, 5-digit standard C3M1 format) */
    if (strlen(name) != 12U) return 0U;
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
    if (name[8] != '.') return 0U;

    char e0 = name[9], e1 = name[10], e2 = name[11];
    if ((e0 == 'd' || e0 == 'D') && (e1 == 'n' || e1 == 'N') && (e2 == 'g' || e2 == 'G'))
    {
        *out_num  = (uint16_t)num;
        *out_flag = PB_FLAG_DNG;
        return 1U;
    }
    if ((e0 == 'j' || e0 == 'J') && (e1 == 'p' || e1 == 'P') && (e2 == 'g' || e2 == 'G'))
    {
        *out_num  = (uint16_t)num;
        *out_flag = PB_FLAG_JPG;
        return 1U;
    }
    return 0U;
}

static uint16_t pb_scan_photos(const char *dir_path, pb_entry_t *entries, uint16_t max_entries)
{
    DIR dir;
    FILINFO fno;
    uint16_t count = 0U;

    if (f_opendir(&dir, dir_path) != FR_OK)
    {
        return 0U;
    }

    for (;;)
    {
        if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == '\0')
        {
            break;
        }
        if (fno.fattrib & AM_DIR) continue;

        uint16_t num = 0U;
        uint8_t  flag = 0U;
        if (!parse_cdi_filename(fno.fname, &num, &flag)) continue;

        /* Merge paired DNG+JPG if number already exists */
        uint8_t merged = 0U;
        for (uint16_t i = 0; i < count; i++)
        {
            if (entries[i].num == num)
            {
                entries[i].flags |= flag;
                merged = 1U;
                break;
            }
        }
        if (merged) continue;

        if (count < max_entries)
        {
            entries[count].num      = num;
            entries[count].flags    = flag;
            entries[count].reserved = 0U;
            count++;
        }
        else
        {
            /* Keep the newest max_entries photos by evicting the smallest number */
            uint16_t min_i = 0U;
            for (uint16_t i = 1U; i < count; i++)
            {
                if (entries[i].num < entries[min_i].num) min_i = i;
            }
            if (num > entries[min_i].num)
            {
                entries[min_i].num   = num;
                entries[min_i].flags = flag;
            }
        }
    }
    f_closedir(&dir);

    /* Shell sort ascending by photo number so index (count - 1) is always newest */
    for (uint16_t gap = count / 2U; gap > 0U; gap /= 2U)
    {
        for (uint16_t i = gap; i < count; i++)
        {
            pb_entry_t tmp = entries[i];
            uint16_t j = i;
            while (j >= gap && entries[j - gap].num > tmp.num)
            {
                entries[j] = entries[j - gap];
                j = (uint16_t)(j - gap);
            }
            entries[j] = tmp;
        }
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

static void dng_parse_ifd(const uint8_t *hdr, uint32_t hdr_len, uint32_t ifd_off,
                          uint8_t is_subifd, dng_info_t *info, uint16_t *lin_tbl,
                          uint32_t *out_subifd_off)
{
    if (ifd_off + 2U > hdr_len) return;
    uint16_t cnt = rd16(hdr, ifd_off);
    uint32_t pos = ifd_off + 2U;

    uint16_t w = 0U, h = 0U, bps = 0U, photo = 0U, spp = 1U;
    uint32_t strip_off = 0U;

    for (uint16_t i = 0U; i < cnt; i++, pos += 12U)
    {
        if (pos + 12U > hdr_len) break;
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
                else if (val + 2U <= hdr_len) bps = rd16(hdr, val);
                break;
            case 0x0106U: photo = (uint16_t)tiff_read_val_u32(hdr, hdr_len, type, val); break;
            case 0x0111U: strip_off = tiff_read_val_u32(hdr, hdr_len, type, val); break;
            case 0x0115U: spp = (uint16_t)tiff_read_val_u32(hdr, hdr_len, type, val); break;
            case 0x014AU:
                if (!is_subifd && out_subifd_off != NULL) *out_subifd_off = val;
                break;
            case 0xC618U: /* LinearizationTable */
                if (type == 3U && count == 256U && val + 512U <= hdr_len && lin_tbl != NULL)
                {
                    for (uint16_t k = 0U; k < 256U; k++)
                    {
                        lin_tbl[k] = rd16(hdr, val + (uint32_t)k * 2U);
                    }
                    info->has_lin_tbl = 1U;
                }
                break;
            case 0xC61AU: /* BlackLevel */
                if (type == 3U && count == 4U && val + 8U <= hdr_len)
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
                else if (type == 5U && count >= 1U && val + 8U <= hdr_len)
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
                if (type == 5U && count == 3U && val + 24U <= hdr_len)
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
    if (photo == 32803U && w > 0U && h > 0U && (bps == 8U || bps == 16U) && strip_off > 0U)
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

    uint32_t ifd0_off   = rd32(hdr_buf, 4U);
    uint32_t subifd_off = 0U;
    dng_parse_ifd(hdr_buf, br, ifd0_off, 0U, info, lin_tbl, &subifd_off);
    if (subifd_off > 0U)
    {
        dng_parse_ifd(hdr_buf, br, subifd_off, 1U, info, lin_tbl, NULL);
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

/* Fast 1X path: division-free exact 2x bilinear upscale 160x120 -> 320x220 in 18-bit RGB666 + 2x2 dither */
static uint8_t __attribute__((optimize("O2"))) dng_render_1x_thumb(FIL *fp, uint32_t thumb_off,
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
            pb_wr_rgb666_dither(rv0, gv0, bv0, dx, dy);
            pb_wr_rgb666_dither((uint8_t)(((uint16_t)rv0 + rv1 + 1U) >> 1),
                                (uint8_t)(((uint16_t)gv0 + gv1 + 1U) >> 1),
                                (uint8_t)(((uint16_t)bv0 + bv1 + 1U) >> 1),
                                (uint16_t)(dx + 1U), dy);
        }
    }
    lcd_set_rgb666(0U);
    return 1U;
}

/* Fallback 1X path for legacy DNGs without embedded preview (full 320x220 in 18-bit RGB666 + dither) */
static uint8_t __attribute__((optimize("O2"))) dng_render_1x_raw(FIL *fp, const dng_info_t *info,
                                                                  const uint16_t *lin_tbl,
                                                                  const uint8_t *srgb_lut,
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
            pb_wr_rgb666_dither(r8, g8, b8, dx, dy);
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
        else
        {
            /* Legacy 16-bit DNG fallback */
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
                uint8_t r8, g8, b8;
                c3m1_color_cam12_to_srgb8(sub_bl12(r12, bl_rb), sub_bl12(g12, bl_g),
                                          sub_bl12(b12, bl_rb), wb_r, wb_b, srgb_lut, &r8, &g8, &b8);
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
    FIL     *fp;
    uint16_t img_w;
    uint16_t img_h;
    uint16_t pan_x;
    uint16_t pan_y;
} jpg_render_ctx_t;

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
            lcd_wr_pixel(rgb565_avg4(r0[lx], r0[lx2], r1[lx], r1[lx2]));
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

static uint8_t jpg_render_file(const char *path, uint8_t zoom_mode,
                               uint16_t pan_x, uint16_t pan_y,
                               uint16_t *out_w, uint16_t *out_h,
                               uint8_t *scratch_after_idx)
{
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0U;

    c3m1_jdec_t *jd = (c3m1_jdec_t *)scratch_after_idx;
    jpg_render_ctx_t jc = {
        .fp    = &f,
        .img_w = 0U,
        .img_h = 0U,
        .pan_x = pan_x,
        .pan_y = pan_y,
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

    uint8_t ok;
    if (!zoom_mode)
    {
        /* Stop decoding once past visible top 220 rows of 320x240 projection */
        uint16_t stop_h = (uint16_t)(((uint32_t)jd->height * PB_VIEW_H + PB_1X_H - 1U) / PB_1X_H);
        if (stop_h > jd->height) stop_h = jd->height;
        ok = c3m1_jdec_decode(jd, 0U, 0U, jd->width, stop_h, jpg_1x_mcu_cb, &jc);
    }
    else
    {
        ok = c3m1_jdec_decode(jd, pan_x, pan_y, PB_SCR_W, PB_VIEW_H, jpg_zoom_mcu_cb, &jc);
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
    if (rx + rw > mx + mw - 1U) rx = (uint16_t)(mx + mw - 1U - rw);
    if (ry + rh > my + mh - 1U) ry = (uint16_t)(my + mh - 1U - rh);
    lcd_draw_rectangle(rx, ry, (uint16_t)(rx + rw - 1U), (uint16_t)(ry + rh - 1U), YELLOW);
}

static void pb_draw_bar_1x(const pb_entry_t *e, uint16_t cur, uint16_t total)
{
    char line[32];
    const char *fmt_str;
    uint16_t    fmt_col;

    if ((e->flags & (PB_FLAG_DNG | PB_FLAG_JPG)) == (PB_FLAG_DNG | PB_FLAG_JPG))
    {
        fmt_str = "DNG+JPG";
        fmt_col = YELLOW;
    }
    else if (e->flags & PB_FLAG_DNG)
    {
        fmt_str = "DNG    ";
        fmt_col = CYAN;
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

    lcd_show_string(56U, PB_BAR_Y + 4U, 42U, 12U, 12U, fmt_str, fmt_col);

    snprintf(line, sizeof(line), "%u/%u", (unsigned int)(cur + 1U), (unsigned int)total);
    lcd_show_string(106U, PB_BAR_Y + 4U, 66U, 12U, 12U, line, WHITE);

    lcd_show_string(186U, PB_BAR_Y + 4U, 132U, 12U, 12U, "4X:ZM ASA:DEL LEV:ESC", GRAY);
}

static void pb_draw_bar_zoom(uint16_t pan_x, uint16_t pan_y)
{
    char line[24];
    g_back_color = BLACK;
    lcd_fill(0U, PB_BAR_Y, PB_SCR_W - 1U, 239U, BLACK);
    lcd_draw_hline(0U, PB_BAR_Y, PB_SCR_W, GRAY);

    lcd_show_string(2U, PB_BAR_Y + 4U, 48U, 12U, 12U, "ZOOM 1:1", YELLOW);

    snprintf(line, sizeof(line), "X:%-3u Y:%-3u", (unsigned int)pan_x, (unsigned int)pan_y);
    lcd_show_string(58U, PB_BAR_Y + 4U, 66U, 12U, 12U, line, CYAN);

    lcd_show_string(176U, PB_BAR_Y + 4U, 142U, 12U, 12U, "DIAL:UD ASA/SH:LR 4X:1X", GRAY);
}

static void pb_draw_delete_modal(const pb_entry_t *e, uint8_t confirm_yes)
{
    char info[36];
    const char *fmt = ((e->flags & (PB_FLAG_DNG | PB_FLAG_JPG)) == (PB_FLAG_DNG | PB_FLAG_JPG))
                      ? "DNG+JPG"
                      : ((e->flags & PB_FLAG_DNG) ? "DNG" : "JPG");

    g_back_color = BLACK;
    lcd_fill(24U, 54U, 295U, 166U, BLACK);
    lcd_draw_rectangle(24U, 54U, 295U, 166U, RED);
    lcd_draw_rectangle(25U, 55U, 294U, 165U, RED);

    lcd_show_string(108U, 64U, 120U, 16U, 16U, "DELETE PHOTO?", RED);
    snprintf(info, sizeof(info), "CDI%05u (%s)", (unsigned int)e->num, fmt);
    uint16_t info_x = (uint16_t)(160U - (strlen(info) * 3U));
    lcd_show_string(info_x, 86U, 160U, 12U, 12U, info, WHITE);
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

/* Returns 1 if photo was deleted, 0 if cancelled / aborted */
static uint8_t pb_run_delete_confirm(c3m1_pb_ctx_t *ctx, const pb_entry_t *e)
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
        if (evts & (BTN_EVT_ISO | BTN_EVT_FILM))
        {
            return 0U;
        }
        if (evts & BTN_EVT_SHOT)
        {
            if (!confirm_yes) return 0U;

            char path[40];
            uint8_t deleted_any = 0U;
            if (e->flags & PB_FLAG_DNG)
            {
                snprintf(path, sizeof(path), "%s/CDI%05u.dng", ctx->photo_dir, (unsigned int)e->num);
                if (f_unlink(path) == FR_OK) deleted_any = 1U;
            }
            if (e->flags & PB_FLAG_JPG)
            {
                snprintf(path, sizeof(path), "%s/CDI%05u.jpg", ctx->photo_dir, (unsigned int)e->num);
                if (f_unlink(path) == FR_OK) deleted_any = 1U;
            }
            if (deleted_any)
            {
                pb_draw_center_box("DELETED", GREEN);
                delay_ms(450);
                return 1U;
            }
            pb_draw_center_box("DELETE FAILED!", RED);
            delay_ms(800);
            return 0U;
        }
        delay_ms(10);
    }
}

/* ======================== Main Render Dispatcher ======================== */
static void pb_render_current(c3m1_pb_ctx_t *ctx, const pb_entry_t *e,
                              uint16_t cur, uint16_t total,
                              uint8_t zoom_mode, uint16_t pan_x, uint16_t pan_y,
                              uint16_t *out_img_w, uint16_t *out_img_h,
                              uint8_t *scratch_after_idx)
{
    char path[40];
    uint8_t ok = 0U;
    uint16_t img_w = 640U, img_h = 480U;

    if (e->flags & PB_FLAG_DNG)
    {
        snprintf(path, sizeof(path), "%s/CDI%05u.dng", ctx->photo_dir, (unsigned int)e->num);
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
                if (!zoom_mode)
                {
                    if (info.has_thumb)
                    {
                        ok = dng_render_1x_thumb(&f, info.thumb_off, scratch_after_idx);
                    }
                    else if (info.valid)
                    {
                        ok = dng_render_1x_raw(&f, &info, lin_tbl, ctx->srgb_lut, scratch_after_idx);
                    }
                }
                else if (info.valid)
                {
                    ok = dng_render_zoom_raw(&f, &info, lin_tbl, ctx->srgb_lut, pan_x, pan_y, scratch_after_idx);
                }
            }
            f_close(&f);
        }
    }
    else if (e->flags & PB_FLAG_JPG)
    {
        snprintf(path, sizeof(path), "%s/CDI%05u.jpg", ctx->photo_dir, (unsigned int)e->num);
        ok = jpg_render_file(path, zoom_mode, pan_x, pan_y, &img_w, &img_h, scratch_after_idx);
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

    if (out_img_w != NULL) *out_img_w = img_w;
    if (out_img_h != NULL) *out_img_h = img_h;

    if (!zoom_mode)
    {
        pb_draw_bar_1x(e, cur, total);
    }
    else
    {
        pb_draw_bar_zoom(pan_x, pan_y);
    }
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

    uint16_t total = pb_scan_photos(ctx->photo_dir, entries, PB_MAX_PHOTOS);
    if (total == 0U)
    {
        pb_draw_center_box("NO PHOTOS!", ORANGE);
        delay_ms(900);
        lcd_clear(BLACK);
        return;
    }

    /* Always start from the newest (last) photo */
    uint16_t cur       = (uint16_t)(total - 1U);
    uint8_t  zoom_mode = 0U;
    uint16_t img_w     = 640U;
    uint16_t img_h     = 480U;
    uint16_t pan_x     = 160U;
    uint16_t pan_y     = 130U;

    key_init_state();
    uint8_t last_dial = key_read_shutter_dial();

    pb_render_current(ctx, &entries[cur], cur, total, 0U, 0U, 0U,
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
                pb_render_current(ctx, &entries[cur], cur, total, 0U, 0U, 0U,
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
                    pb_render_current(ctx, &entries[cur], cur, total, 1U, pan_x, pan_y,
                                      &img_w, &img_h, scratch_after_idx);
                }
            }
        }

        uint8_t evts = key_poll_events();
        if (!zoom_mode)
        {
            if (evts & BTN_EVT_FILM)
            {
                /* Short flick of film lever -> exit playback */
                break;
            }
            if (evts & BTN_EVT_4X)
            {
                /* Enter 1:1 zoom centered */
                zoom_mode = 1U;
                pan_x = (img_w > PB_SCR_W)  ? (uint16_t)((img_w - PB_SCR_W) / 2U)  : 0U;
                pan_y = (img_h > PB_VIEW_H) ? (uint16_t)((img_h - PB_VIEW_H) / 2U) : 0U;
                pb_render_current(ctx, &entries[cur], cur, total, 1U, pan_x, pan_y,
                                  &img_w, &img_h, scratch_after_idx);
            }
            else if (evts & BTN_EVT_ISO)
            {
                /* Short press ASA -> delete confirmation (removes both DNG and JPG) */
                uint8_t deleted = pb_run_delete_confirm(ctx, &entries[cur]);
                if (ctx->sd_removed_in_pb) break;
                last_dial = key_read_shutter_dial();

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
                    if (cur >= total) cur = (uint16_t)(total - 1U);
                }
                pb_render_current(ctx, &entries[cur], cur, total, 0U, 0U, 0U,
                                  &img_w, &img_h, scratch_after_idx);
            }
        }
        else
        {
            if (evts & BTN_EVT_4X)
            {
                /* Press 4X again -> exit zoom back to 1X playback */
                zoom_mode = 0U;
                pb_render_current(ctx, &entries[cur], cur, total, 0U, 0U, 0U,
                                  &img_w, &img_h, scratch_after_idx);
            }
            else if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
            {
                /* ASA -> pan LEFT */
                uint16_t nx = (pan_x > PB_PAN_STEP_X) ? (uint16_t)(pan_x - PB_PAN_STEP_X) : 0U;
                if (nx != pan_x)
                {
                    pan_x = nx;
                    pb_render_current(ctx, &entries[cur], cur, total, 1U, pan_x, pan_y,
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
                    pb_render_current(ctx, &entries[cur], cur, total, 1U, pan_x, pan_y,
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
