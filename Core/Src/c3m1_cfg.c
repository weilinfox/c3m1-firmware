/**
 * @file        c3m1_cfg.c
 * @brief       SD-card binary configuration loader/saver (0:CAYE_CFG/c3m1.cfg).
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "c3m1_cfg.h"
#include "fatfs.h"
#include "fw_version.h"
#include "../../BSP/OV7725/ov7725.h"

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
    cfg->reserved0   = 0U;
    memset(cfg->reserved1, 0, sizeof(cfg->reserved1));
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
    cfg_stamp_header_and_crc(cfg);
}

static uint8_t cfg_fields_valid(const c3m1_cfg_t *cfg)
{
    if (cfg->iso_idx      >= OV7725_ISO_COUNT)      return 0U;
    if (cfg->max_auto_iso >= C3M1_MAX_ISO_COUNT)    return 0U;
    if (cfg->min_auto_sht >= C3M1_MIN_SHT_COUNT)    return 0U;
    if (cfg->flicker_mode >= C3M1_FLICKER_COUNT)    return 0U;
    if (cfg->bat_disp     >= C3M1_BAT_DISP_COUNT)   return 0U;
    if (cfg->film_lever   >= C3M1_FILM_LEVER_COUNT) return 0U;
    if (cfg->hist_en      >= C3M1_HIST_COUNT)       return 0U;
    if (cfg->peak_color   >= C3M1_PEAK_COLOR_COUNT) return 0U;
    if (cfg->grid_en      >= C3M1_GRID_COUNT)       return 0U;
    if (cfg->save_mode    >= C3M1_SAVE_MODE_COUNT)  return 0U;
    if (cfg->wb_mode      >= C3M1_WB_MODE_COUNT)    return 0U;
    if (cfg->debug_info   >= C3M1_DEBUG_INFO_COUNT) return 0U;
    return 1U;
}

c3m1_cfg_status_t c3m1_cfg_load(c3m1_cfg_t *cfg)
{
    FIL f;
    UINT br = 0;
    c3m1_cfg_t tmp;

    FRESULT fres = f_open(&f, C3M1_CFG_PATH, FA_READ);
    if (fres != FR_OK)
    {
        c3m1_cfg_set_defaults(cfg);
        if (fres == FR_NO_FILE || fres == FR_NO_PATH)
        {
            printf("CFG: default (missing, err=%d)\r\n", fres);
            return CFG_LOAD_MISSING;
        }
        printf("CFG: default (open io err=%d)\r\n", fres);
        return CFG_LOAD_IO_ERR;
    }

    FSIZE_t fsz = f_size(&f);
    if (fsz != (FSIZE_t)sizeof(c3m1_cfg_t))
    {
        f_close(&f);
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (bad size=%lu)\r\n", (unsigned long)fsz);
        return CFG_LOAD_BAD_SIZE;
    }

    fres = f_read(&f, &tmp, sizeof(tmp), &br);
    f_close(&f);
    if (fres != FR_OK)
    {
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (read io err=%d)\r\n", fres);
        return CFG_LOAD_IO_ERR;
    }
    if (br != sizeof(tmp))
    {
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (short read br=%u)\r\n", (unsigned int)br);
        return CFG_LOAD_BAD_SIZE;
    }

    if (tmp.magic != C3M1_CFG_MAGIC ||
        tmp.schema_ver != C3M1_CFG_SCHEMA ||
        tmp.struct_size != (uint16_t)sizeof(c3m1_cfg_t))
    {
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (bad magic/schema)\r\n");
        return CFG_LOAD_BAD_MAGIC;
    }

    if (tmp.fw_major != FW_VER_MAJOR ||
        tmp.fw_minor != FW_VER_MINOR ||
        tmp.fw_patch != FW_VER_PATCH)
    {
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (fw mismatch %u.%u.%u != %u.%u.%u)\r\n",
               tmp.fw_major, tmp.fw_minor, tmp.fw_patch,
               (unsigned int)FW_VER_MAJOR, (unsigned int)FW_VER_MINOR, (unsigned int)FW_VER_PATCH);
        return CFG_LOAD_BAD_FW_VER;
    }

    uint32_t calc_crc = crc32_ieee((const uint8_t *)&tmp, offsetof(c3m1_cfg_t, crc32));
    if (calc_crc != tmp.crc32)
    {
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (crc mismatch)\r\n");
        return CFG_LOAD_BAD_CRC;
    }

    if (!cfg_fields_valid(&tmp))
    {
        c3m1_cfg_set_defaults(cfg);
        printf("CFG: default (field out of range)\r\n");
        return CFG_LOAD_BAD_RANGE;
    }

    *cfg = tmp;
    printf("CFG: loaded OK (iso=%u fmt=%u maxIso=%u minSht=%u flk=%u bat=%u lever=%u hist=%u peak=%u grid=%u)\r\n",
           cfg->iso_idx, cfg->save_mode, cfg->max_auto_iso, cfg->min_auto_sht,
           cfg->flicker_mode, cfg->bat_disp, cfg->film_lever,
           cfg->hist_en, cfg->peak_color, cfg->grid_en);
    return CFG_LOAD_OK;
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
    cfg_stamp_header_and_crc(&out);

    FIL f;
    UINT bw = 0;
    FRESULT fres = f_open(&f, C3M1_CFG_PATH, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        printf("CFG: open err=%d\r\n", fres);
        return 0U;
    }

    fres = f_write(&f, &out, sizeof(out), &bw);
    FRESULT cres = f_close(&f);
    if (fres != FR_OK || bw != sizeof(out) || cres != FR_OK)
    {
        printf("CFG: write/close err=%d/%d\r\n", fres, cres);
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
