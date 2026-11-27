/**
 * @file        c3m1_cfg.c
 * @brief       SD-card binary configuration loader/saver (0:CAYE_CFG/c3m1.cfg).
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "c3m1_cfg.h"
#include "c3m1_clock.h"
#include "fatfs.h"
#include "fw_version.h"
#include "../../BSP/OV7725/ov7725.h"

_Static_assert(sizeof(c3m1_cfg_t) == 48U, "c3m1_cfg_t must remain 48 bytes");

static const c3m1_lens_profile_t s_lens_profiles[C3M1_LENS_PROFILE_COUNT] = {
    /* 0: UNKNOWN (default: keeps current FOCAL/APERTURE/DIST/VIG unchanged, omits EXIF LensModel) */
    {
        .menu_label = "UNKNOWN",
        .exif_name  = NULL,
        .focal_mm   = 0U,
        .fnum_x10   = 0U,
        .distort    = 0,
        .vignette   = 0U,
    },
    /* 1: Stock C3M1 ~50mm-eq lens (6mm f/1.6, Experiment 6 calibrated kr1 = -0.072) */
    {
        .menu_label = "6mm F1.6",
        .exif_name  = "C3M1 6mm F1.6",
        .focal_mm   = 6U,
        .fnum_x10   = 16U,
        .distort    = -36, /* -36 * 0.002 = -0.072 */
        .vignette   = 0U,
    },
};

c3m1_cfg_t g_cfg;

static uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        for (uint8_t b = 0; b < 8U; b++)
        {
            if (crc & 1UL)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static void cfg_stamp_header_and_crc(c3m1_cfg_t *cfg)
{
    cfg->magic       = C3M1_CFG_MAGIC;
    cfg->schema_ver  = C3M1_CFG_SCHEMA;
    cfg->struct_size = (uint16_t)sizeof(c3m1_cfg_t);
    cfg->fw_major    = FW_VER_MAJOR;
    cfg->fw_minor    = FW_VER_MINOR;
    cfg->fw_patch    = FW_VER_PATCH;
    cfg->crc32       = crc32_ieee((const uint8_t *)cfg, offsetof(c3m1_cfg_t, crc32));
}

void c3m1_cfg_set_defaults(c3m1_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->iso_idx      = 0U; /* AUTO ISO */
    cfg->max_auto_iso = 2U; /* 1600 */
    cfg->min_auto_sht = 1U; /* 1/50 */
    cfg->flicker_mode = 0U; /* 50HZ */
    cfg->bat_disp     = 0U; /* VOLT */
    cfg->film_lever   = 1U; /* ON */
    cfg->hist_en      = 1U; /* ON */
    cfg->peak_color   = 0U; /* RED */
    cfg->grid_en      = 0U; /* OFF */
    cfg->save_mode    = C3M1_SAVE_DNG; /* DNG only */
    cfg->wb_mode      = C3M1_WB_AUTO;  /* AUTO WB */
    cfg->debug_info   = 0U;            /* Debug overlay OFF */
    cfg->ev_comp      = C3M1_EV_COMP_ZERO; /* 0.0 EV */
    cfg->meter_mode   = C3M1_METER_STD;    /* STD multi-zone */
    cfg->auto_sleep   = C3M1_AUTO_SLEEP_DEFAULT; /* 10M */
    cfg->boot_clock   = 1U;                /* Prompt date/time on boot */
    cfg->saved_epoch  = 0UL;               /* Unset -> seed from build time on first boot */
    cfg->pb_skip_brk   = 0U;               /* Show all HDR frames in playback by default */
    cfg->del_hdr_brk   = 0U;               /* Keep HDR _0.._3 bracket frames by default */
    cfg->lens_profile    = C3M1_LENS_PROFILE_UNKNOWN;
    cfg->lens_distort    = 0;                       /* Lens geometric distortion correction OFF (0) */
    cfg->lens_vignette   = 0U;                      /* Lens radial vignetting compensation OFF (0) */
    cfg->lens_focal_mm   = C3M1_LENS_FOCAL_DEFAULT; /* Default 6mm (EQ 54mm) */
    cfg->lens_fnum_x10   = 0U;                      /* Default F0 (writes EXIF FNumber 0/1) */
    cfg->lens_dist_incam = 0U;                      /* Default OFF: DNG OpcodeList3 only */
    cfg->high_iso_nr     = 1U;                      /* Default ON (ISO-adaptive JPEG + DNG playback NR) */
    cfg->anti_alias      = 1U;                      /* Default ON (unified VF + still AA) */
    cfg->color_mode      = C3M1_COLOR_COLOR;        /* Default full color */
    cfg_stamp_header_and_crc(cfg);
}

static uint8_t cfg_sanitize_fields(c3m1_cfg_t *cfg)
{
    uint8_t fixed = 0U;
    if (cfg->iso_idx         >= OV7725_ISO_COUNT)        { cfg->iso_idx         = 0U;                        fixed = 1U; }
    if (cfg->max_auto_iso    >= C3M1_MAX_ISO_COUNT)      { cfg->max_auto_iso    = 2U;                        fixed = 1U; }
    if (cfg->min_auto_sht    >= C3M1_MIN_SHT_COUNT)      { cfg->min_auto_sht    = 1U;                        fixed = 1U; }
    if (cfg->flicker_mode    >= C3M1_FLICKER_COUNT)      { cfg->flicker_mode    = 0U;                        fixed = 1U; }
    if (cfg->bat_disp        >= C3M1_BAT_DISP_COUNT)     { cfg->bat_disp        = 0U;                        fixed = 1U; }
    if (cfg->film_lever      >= C3M1_FILM_LEVER_COUNT)   { cfg->film_lever      = 1U;                        fixed = 1U; }
    if (cfg->hist_en         >= C3M1_HIST_COUNT)         { cfg->hist_en         = 1U;                        fixed = 1U; }
    if (cfg->peak_color      >= C3M1_PEAK_COLOR_COUNT)   { cfg->peak_color      = 0U;                        fixed = 1U; }
    if (cfg->grid_en         >= C3M1_GRID_COUNT)         { cfg->grid_en         = 0U;                        fixed = 1U; }
    if (cfg->save_mode       >= C3M1_SAVE_MODE_COUNT)    { cfg->save_mode       = C3M1_SAVE_DNG;             fixed = 1U; }
    if (cfg->wb_mode         >= C3M1_WB_MODE_COUNT)      { cfg->wb_mode         = C3M1_WB_AUTO;              fixed = 1U; }
    if (cfg->debug_info      >= C3M1_DEBUG_INFO_COUNT)   { cfg->debug_info      = 0U;                        fixed = 1U; }
    if (cfg->ev_comp         >= C3M1_EV_COMP_COUNT)      { cfg->ev_comp         = C3M1_EV_COMP_ZERO;         fixed = 1U; }
    if (cfg->meter_mode      >= C3M1_METER_MODE_COUNT)   { cfg->meter_mode      = C3M1_METER_STD;            fixed = 1U; }
    if (cfg->auto_sleep      >= C3M1_AUTO_SLEEP_COUNT)   { cfg->auto_sleep      = C3M1_AUTO_SLEEP_DEFAULT;   fixed = 1U; }
    if (cfg->boot_clock      >= C3M1_BOOT_CLOCK_COUNT)   { cfg->boot_clock      = 1U;                        fixed = 1U; }
    if (cfg->pb_skip_brk     >= C3M1_PB_SKIP_BRK_COUNT)  { cfg->pb_skip_brk     = 0U;                        fixed = 1U; }
    if (cfg->del_hdr_brk     >= C3M1_DEL_HDR_BRK_COUNT)  { cfg->del_hdr_brk     = 0U;                        fixed = 1U; }
    if (cfg->lens_distort    < C3M1_LENS_DIST_MIN ||
        cfg->lens_distort    > C3M1_LENS_DIST_MAX)       { cfg->lens_distort    = 0;                         fixed = 1U; }
    if (cfg->lens_vignette   > C3M1_LENS_VIG_MAX)        { cfg->lens_vignette   = 0U;                        fixed = 1U; }
    if (cfg->lens_focal_mm   > C3M1_LENS_FOCAL_MAX_MM)   { cfg->lens_focal_mm   = C3M1_LENS_FOCAL_DEFAULT;   fixed = 1U; }
    if (cfg->lens_fnum_x10   > C3M1_LENS_FNUM_MAX_X10)   { cfg->lens_fnum_x10   = 0U;                        fixed = 1U; }
    if (cfg->lens_dist_incam >= C3M1_LENS_DIST_INCAM_CNT){ cfg->lens_dist_incam = 0U;                        fixed = 1U; }
    if (cfg->high_iso_nr     >= C3M1_HIGH_ISO_NR_COUNT)  { cfg->high_iso_nr     = 1U;                        fixed = 1U; }
    if (cfg->anti_alias      >= C3M1_ANTI_ALIAS_COUNT)   { cfg->anti_alias      = 1U;                        fixed = 1U; }
    if (cfg->color_mode      >= C3M1_COLOR_MODE_COUNT)   { cfg->color_mode      = C3M1_COLOR_COLOR;          fixed = 1U; }
    if (cfg->lens_profile    >= C3M1_LENS_PROFILE_COUNT) { cfg->lens_profile    = C3M1_LENS_PROFILE_UNKNOWN; fixed = 1U; }
    if (cfg->lens_profile != C3M1_LENS_PROFILE_UNKNOWN)
    {
        const c3m1_lens_profile_t *p = &s_lens_profiles[cfg->lens_profile];
        if (cfg->lens_focal_mm != p->focal_mm || cfg->lens_fnum_x10 != p->fnum_x10)
        {
            cfg->lens_profile = C3M1_LENS_PROFILE_UNKNOWN;
            fixed = 1U;
        }
    }
    if (cfg->saved_epoch > C3M1_CLOCK_EPOCH_MAX)
    {
        cfg->saved_epoch = C3M1_CLOCK_EPOCH_MAX;
        fixed = 1U;
    }
    return fixed;
}

static const char s_cfg_tmp_path[] = C3M1_CFG_DIR "/c3m1.tmp";

c3m1_cfg_status_t c3m1_cfg_load(c3m1_cfg_t *cfg)
{
    FIL f;
    UINT br = 0;
    c3m1_cfg_t tmp;
    uint8_t recovered_tmp = 0U;

    FRESULT fres = f_open(&f, C3M1_CFG_PATH, FA_READ);
    if (fres != FR_OK)
    {
        if (fres == FR_NO_FILE)
        {
            /* Recover from sudden power-loss between f_unlink(cfg) and f_rename(tmp -> cfg) */
            if (f_open(&f, s_cfg_tmp_path, FA_READ) == FR_OK)
            {
                recovered_tmp = 1U;
            }
            else
            {
                c3m1_cfg_set_defaults(cfg);
                printf("CFG: default (missing, err=%d)\r\n", fres);
                return CFG_LOAD_MISSING;
            }
        }
        else if (fres == FR_NO_PATH)
        {
            c3m1_cfg_set_defaults(cfg);
            printf("CFG: default (missing, err=%d)\r\n", fres);
            return CFG_LOAD_MISSING;
        }
        else
        {
            /* Keep existing in-RAM config intact on transient I/O errors */
            printf("CFG: kept RAM (open io err=%d)\r\n", fres);
            return CFG_LOAD_IO_ERR;
        }
    }

    FSIZE_t fsz = f_size(&f);
    if (fsz != (FSIZE_t)sizeof(c3m1_cfg_t) && fsz != 44U && fsz != 40U)
    {
        f_close(&f);
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (bad size=%lu)\r\n", (unsigned long)fsz);
        return CFG_LOAD_BAD_SIZE;
    }

    uint8_t raw[sizeof(c3m1_cfg_t)];
    fres = f_read(&f, raw, (UINT)fsz, &br);
    f_close(&f);
    if (fres != FR_OK)
    {
        printf("CFG: kept RAM (read io err=%d)\r\n", fres);
        return CFG_LOAD_IO_ERR;
    }
    if (br != (UINT)fsz)
    {
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (short read br=%u)\r\n", (unsigned int)br);
        return CFG_LOAD_BAD_SIZE;
    }

    memset(&tmp, 0, sizeof(tmp));
    uint32_t file_crc = 0UL;
    uint32_t calc_crc = 0UL;
    if (fsz == 40U)
    {
        memcpy(&tmp, raw, 36U);
        memcpy(&file_crc, &raw[36], 4U);
        if (tmp.magic != C3M1_CFG_MAGIC ||
            (tmp.schema_ver < 10U || tmp.schema_ver > 14U) ||
            tmp.struct_size != 40U)
        {
            c3m1_cfg_set_defaults(cfg);
            printf("CFG: default (bad legacy magic/schema)\r\n");
            return CFG_LOAD_BAD_MAGIC;
        }
        calc_crc = crc32_ieee(raw, 36U);
    }
    else if (fsz == 44U)
    {
        memcpy(&tmp, raw, 40U);
        memcpy(&file_crc, &raw[40], 4U);
        if (tmp.magic != C3M1_CFG_MAGIC ||
            (tmp.schema_ver < 15U || tmp.schema_ver > 16U) ||
            tmp.struct_size != 44U)
        {
            c3m1_cfg_set_defaults(cfg);
            printf("CFG: default (bad legacy44 magic/schema)\r\n");
            return CFG_LOAD_BAD_MAGIC;
        }
        calc_crc = crc32_ieee(raw, 40U);
    }
    else
    {
        memcpy(&tmp, raw, sizeof(tmp));
        file_crc = tmp.crc32;
        if (tmp.magic != C3M1_CFG_MAGIC ||
            (tmp.schema_ver < 17U || tmp.schema_ver > C3M1_CFG_SCHEMA) ||
            tmp.struct_size != (uint16_t)sizeof(c3m1_cfg_t))
        {
            c3m1_cfg_set_defaults(cfg);
            printf("CFG: default (bad magic/schema)\r\n");
            return CFG_LOAD_BAD_MAGIC;
        }
        calc_crc = crc32_ieee((const uint8_t *)&tmp, offsetof(c3m1_cfg_t, crc32));
    }

    if (calc_crc != file_crc)
    {
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (crc mismatch)\r\n");
        return CFG_LOAD_BAD_CRC;
    }

    uint8_t migrated_old = (tmp.schema_ver < C3M1_CFG_SCHEMA) ? 1U : 0U;
    if (tmp.schema_ver <= 10U)
    {
        tmp.pb_skip_brk = 0U;
    }
    if (tmp.schema_ver <= 11U)
    {
        tmp.del_hdr_brk = 0U;
    }
    if (tmp.schema_ver <= 12U)
    {
        tmp.lens_distort  = 0;
        tmp.lens_vignette = 0U;
    }
    else if (tmp.schema_ver == 13U)
    {
        int16_t scaled = (int16_t)tmp.lens_distort * 5;
        if (scaled < C3M1_LENS_DIST_MIN) scaled = C3M1_LENS_DIST_MIN;
        if (scaled > C3M1_LENS_DIST_MAX) scaled = C3M1_LENS_DIST_MAX;
        tmp.lens_distort = (int8_t)scaled;
    }
    if (tmp.schema_ver <= 14U)
    {
        tmp.lens_focal_mm = C3M1_LENS_FOCAL_DEFAULT;
        tmp.lens_fnum_x10 = 0U;
    }
    if (tmp.schema_ver <= 15U)
    {
        tmp.lens_dist_incam = 0U;
    }
    if (tmp.schema_ver <= 16U)
    {
        tmp.lens_profile = C3M1_LENS_PROFILE_UNKNOWN;
    }
    if (tmp.schema_ver <= 17U)
    {
        tmp.high_iso_nr = 1U;
        tmp.anti_alias  = 1U;
    }
    if (tmp.schema_ver <= 18U)
    {
        tmp.color_mode = C3M1_COLOR_COLOR;
    }
    if (migrated_old)
    {
        tmp.schema_ver  = C3M1_CFG_SCHEMA;
        tmp.struct_size = (uint16_t)sizeof(c3m1_cfg_t);
    }

    uint8_t orig_fw_major = tmp.fw_major;
    uint8_t orig_fw_minor = tmp.fw_minor;
    uint8_t orig_fw_patch = tmp.fw_patch;

    uint8_t sanitized = cfg_sanitize_fields(&tmp);
    if (migrated_old || sanitized)
    {
        cfg_stamp_header_and_crc(&tmp);
    }

    *cfg = tmp;
    if (recovered_tmp)
    {
        printf("CFG: loaded from %s\r\n", s_cfg_tmp_path);
    }
    if (sanitized)
    {
        printf("CFG: sanitized out-of-range field(s)\r\n");
    }
    if (migrated_old)
    {
        printf("CFG: migrated schema -> v%u\r\n", (unsigned int)C3M1_CFG_SCHEMA);
    }
    if (orig_fw_major != FW_VER_MAJOR ||
        orig_fw_minor != FW_VER_MINOR ||
        orig_fw_patch != FW_VER_PATCH)
    {
        printf("CFG: kept across fw update %u.%u.%u -> %u.%u.%u (schema v%u)\r\n",
               orig_fw_major, orig_fw_minor, orig_fw_patch,
               (unsigned int)FW_VER_MAJOR, (unsigned int)FW_VER_MINOR, (unsigned int)FW_VER_PATCH,
               (unsigned int)C3M1_CFG_SCHEMA);
    }
    printf("CFG: loaded OK (iso=%u fmt=%u maxIso=%u minSht=%u flk=%u bat=%u lever=%u hist=%u peak=%u grid=%u)\r\n",
           cfg->iso_idx, cfg->save_mode, cfg->max_auto_iso, cfg->min_auto_sht,
           cfg->flicker_mode, cfg->bat_disp, cfg->film_lever,
           cfg->hist_en, cfg->peak_color, cfg->grid_en);
    return recovered_tmp ? CFG_LOAD_RECOVERED_TMP : CFG_LOAD_OK;
}

uint8_t c3m1_cfg_heal_tmp(void)
{
    if (f_rename(s_cfg_tmp_path, C3M1_CFG_PATH) == FR_OK)
    {
        printf("CFG: recovered %s -> %s\r\n", s_cfg_tmp_path, C3M1_CFG_PATH);
        return 1U;
    }
    return 0U;
}

uint8_t c3m1_cfg_save(const c3m1_cfg_t *cfg)
{
    FRESULT dres = f_mkdir(C3M1_CFG_DIR);
    if (dres != FR_OK && dres != FR_EXIST)
    {
        printf("CFG: mkdir err=%d\r\n", dres);
        return 0U;
    }

    c3m1_cfg_t out = *cfg;
    out.saved_epoch = c3m1_clock_get_epoch();
    if (cfg == &g_cfg)
    {
        g_cfg.saved_epoch = out.saved_epoch;
    }
    cfg_stamp_header_and_crc(&out);

    /* Atomic write: write complete image to temp file first so sudden SW4 power-off cannot leave a 0-byte cfg */
    FIL f;
    UINT bw = 0;
    FRESULT fres = f_open(&f, s_cfg_tmp_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        printf("CFG: open tmp err=%d\r\n", fres);
        return 0U;
    }

    fres = f_write(&f, &out, sizeof(out), &bw);
    FRESULT cres = f_close(&f);
    if (fres != FR_OK || bw != sizeof(out) || cres != FR_OK)
    {
        (void)f_unlink(s_cfg_tmp_path);
        printf("CFG: write/close err=%d/%d\r\n", fres, cres);
        return 0U;
    }

    (void)f_unlink(C3M1_CFG_PATH);
    FRESULT rres = f_rename(s_cfg_tmp_path, C3M1_CFG_PATH);
    if (rres != FR_OK)
    {
        printf("CFG: rename err=%d\r\n", rres);
        return 0U;
    }

    printf("CFG: saved %s (crc=0x%08lX)\r\n", C3M1_CFG_PATH, (unsigned long)out.crc32);
    return 1U;
}

uint16_t c3m1_cfg_max_auto_gain_q4(const c3m1_cfg_t *cfg)
{
    /* 0:400 (64), 1:800 (128), 2:1600 (256), 3:3200 (496 = 31x OV7725 hardware max GAIN=0xFF) */
    static const uint16_t tbl[C3M1_MAX_ISO_COUNT] = { 64U, 128U, 256U, 496U };
    uint8_t idx = (cfg->max_auto_iso < C3M1_MAX_ISO_COUNT) ? cfg->max_auto_iso : 2U;
    return tbl[idx];
}

uint16_t c3m1_cfg_min_auto_aec(const c3m1_cfg_t *cfg)
{
    /* Exact lines @ 22,959 lines/s (COM4=0x81, CLKRC=0x00):
     *   0: 1/100 (230), 1: 1/50 (459), 2: 1/25 (918), 3: 1/15 (1531), 4: 1/8 (2870)
     */
    static const uint16_t tbl[C3M1_MIN_SHT_COUNT] = { 230U, 459U, 918U, 1531U, 2870U };
    uint8_t idx = (cfg->min_auto_sht < C3M1_MIN_SHT_COUNT) ? cfg->min_auto_sht : 1U;
    return tbl[idx];
}

uint16_t c3m1_cfg_peak_rgb565(const c3m1_cfg_t *cfg)
{
    /* Sony mirrorless peaking palette: 0:RED, 1:YELLOW, 2:WHITE, 3:BLUE (vivid sky blue for 2" IPS) */
    static const uint16_t tbl[C3M1_PEAK_COLOR_COUNT] = { 0xF800U, 0xFFE0U, 0xFFFFU, 0x05FFU };
    uint8_t idx = (cfg->peak_color < C3M1_PEAK_COLOR_COUNT) ? cfg->peak_color : 0U;
    return tbl[idx];
}

uint8_t c3m1_cfg_wb_gains(const c3m1_cfg_t *cfg, uint16_t *wb_r_q8, uint16_t *wb_b_q8)
{
    /*
     * Exact Q8 WB gains matched to OV7725 DNG ColorMatrix1 (sorted ascending by Kelvin):
     *   1: 3200K (Tungsten film) -> R=168, B=651
     *   2: 5500K (Standard film) -> R=274, B=371
     *   3: 6500K (Daylight film) -> R=302, B=322
     */
    static const uint16_t r_tbl[C3M1_WB_MODE_COUNT] = { 256U, 168U, 274U, 302U };
    static const uint16_t b_tbl[C3M1_WB_MODE_COUNT] = { 406U, 651U, 371U, 322U };

    if (cfg->wb_mode == C3M1_WB_AUTO || cfg->wb_mode >= C3M1_WB_MODE_COUNT)
    {
        return 0U; /* AUTO AWB */
    }
    *wb_r_q8 = r_tbl[cfg->wb_mode];
    *wb_b_q8 = b_tbl[cfg->wb_mode];
    return 1U;
}

uint16_t c3m1_cfg_ae_target_q4(const c3m1_cfg_t *cfg)
{
    /*
     * Exact linear-light AE target in Q4: 736 * 2^((idx - 6) / 3) (0.0 EV = 46 * 16 = 736).
     * +-2.0 EV spans 184 (4.5%) .. 2944 (72.2%), staying strictly below the 100% sensor ceiling (+2.43 EV).
     */
    static const uint16_t tbl[C3M1_EV_COMP_COUNT] = {
         184U,  232U,  292U,  368U,  464U,  584U,
         736U,
         927U, 1168U, 1472U, 1855U, 2337U, 2944U,
    };
    uint8_t idx = (cfg->ev_comp < C3M1_EV_COMP_COUNT) ? cfg->ev_comp : C3M1_EV_COMP_ZERO;
    return tbl[idx];
}

const char *c3m1_cfg_ev_comp_label(uint8_t ev_comp_idx)
{
    static const char *const tbl[C3M1_EV_COMP_COUNT] = {
        "-2.0EV", "-1.7EV", "-1.3EV", "-1.0EV", "-0.7EV", "-0.3EV",
        "+0.0EV",
        "+0.3EV", "+0.7EV", "+1.0EV", "+1.3EV", "+1.7EV", "+2.0EV",
    };
    if (ev_comp_idx >= C3M1_EV_COMP_COUNT) ev_comp_idx = C3M1_EV_COMP_ZERO;
    return tbl[ev_comp_idx];
}

/* Common nominal lens focal lengths (mm, 4mm through 500mm; official C3M1 9x crop) */
static const uint16_t s_focal_mm_tbl[C3M1_LENS_FOCAL_COUNT] = {
      0U,
      4U,   5U,   6U,   8U,  10U,  12U,  14U,  15U,  16U,  18U,
     20U,  21U,  24U,  25U,  28U,  30U,  35U,  40U,  45U,  50U,
     55U,  58U,  60U,  70U,  75U,  85U,  90U, 100U, 105U, 120U,
    135U, 150U, 180U, 200U, 250U, 300U, 400U, 500U,
};

uint16_t c3m1_cfg_lens_focal_preset_mm(uint8_t idx)
{
    if (idx >= C3M1_LENS_FOCAL_COUNT) return 0U;
    return s_focal_mm_tbl[idx];
}

uint8_t c3m1_cfg_lens_focal_find_idx(uint16_t focal_mm)
{
    if (focal_mm == 0U) return 0U;
    uint8_t  best_i = 1U;
    uint32_t best_d = 0xFFFFFFFFUL;
    for (uint8_t i = 1U; i < C3M1_LENS_FOCAL_COUNT; i++)
    {
        uint16_t v = s_focal_mm_tbl[i];
        uint32_t d = (v >= focal_mm) ? (uint32_t)(v - focal_mm) : (uint32_t)(focal_mm - v);
        if (d < best_d)
        {
            best_d = d;
            best_i = i;
        }
    }
    return best_i;
}

uint16_t c3m1_cfg_lens_focal_eq35(uint16_t focal_mm)
{
    if (focal_mm == 0U) return 0U;
    /* Official C3M1 9x crop factor */
    uint32_t eq = (uint32_t)focal_mm * 9UL;
    return (eq > 65535UL) ? 65535U : (uint16_t)eq;
}

void c3m1_cfg_format_lens_focal(uint16_t focal_mm, char *out, uint8_t out_sz)
{
    if (focal_mm == 0U)
    {
        snprintf(out, out_sz, "OFF");
    }
    else
    {
        snprintf(out, out_sz, "%umm", (unsigned int)focal_mm);
    }
}

void c3m1_cfg_format_lens_fnum(uint16_t fnum_x10, char *out, uint8_t out_sz)
{
    if (fnum_x10 == 0U)
    {
        snprintf(out, out_sz, "F0");
    }
    else
    {
        snprintf(out, out_sz, "F%u.%u",
                 (unsigned int)(fnum_x10 / 10U), (unsigned int)(fnum_x10 % 10U));
    }
}

void c3m1_cfg_format_lens_dist(int8_t val, char *out, uint8_t out_sz)
{
    if (val == 0)
    {
        snprintf(out, out_sz, "0");
    }
    else
    {
        unsigned int m = (unsigned int)((val < 0) ? -val : val) * 2U; /* 2..200 thousandths */
        snprintf(out, out_sz, "%c0.%03u", (val < 0) ? '-' : '+', m);
    }
}

void c3m1_cfg_format_lens_vig(uint8_t val, char *out, uint8_t out_sz)
{
    if (val == 0U)
    {
        snprintf(out, out_sz, "0");
    }
    else
    {
        unsigned int h = (unsigned int)val * 5U; /* hundredths: 5..250 -> +0.05..+2.50 */
        snprintf(out, out_sz, "+%u.%02u", h / 100U, h % 100U);
    }
}

const c3m1_lens_profile_t *c3m1_cfg_lens_profile_get(uint8_t idx)
{
    if (idx >= C3M1_LENS_PROFILE_COUNT)
    {
        idx = C3M1_LENS_PROFILE_UNKNOWN;
    }
    return &s_lens_profiles[idx];
}

const char *c3m1_cfg_lens_profile_label(uint8_t idx)
{
    return c3m1_cfg_lens_profile_get(idx)->menu_label;
}

const char *c3m1_cfg_lens_profile_exif_name(uint8_t idx)
{
    return c3m1_cfg_lens_profile_get(idx)->exif_name;
}

void c3m1_cfg_apply_lens_profile(c3m1_cfg_t *cfg, uint8_t idx)
{
    if (cfg == NULL) return;
    if (idx >= C3M1_LENS_PROFILE_COUNT)
    {
        idx = C3M1_LENS_PROFILE_UNKNOWN;
    }
    cfg->lens_profile = idx;
    if (idx != C3M1_LENS_PROFILE_UNKNOWN)
    {
        const c3m1_lens_profile_t *p = &s_lens_profiles[idx];
        cfg->lens_focal_mm = p->focal_mm;
        cfg->lens_fnum_x10 = p->fnum_x10;
        cfg->lens_distort  = p->distort;
        cfg->lens_vignette = p->vignette;
    }
}

uint32_t c3m1_cfg_auto_sleep_ms(const c3m1_cfg_t *cfg)
{
    static const uint32_t tbl[C3M1_AUTO_SLEEP_COUNT] = {
        5UL  * 60000UL,
        10UL * 60000UL,
        15UL * 60000UL,
        20UL * 60000UL,
        25UL * 60000UL,
        0UL,
    };
    uint8_t idx = (cfg->auto_sleep < C3M1_AUTO_SLEEP_COUNT) ? cfg->auto_sleep : C3M1_AUTO_SLEEP_DEFAULT;
    return tbl[idx];
}
