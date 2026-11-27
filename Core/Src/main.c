/* USER CODE BEGIN Header */
/**
 ****************************************************************************************************
 * @file        main.c
 * @brief       C3M1 (C3MarkI) Digital RAW Camera Firmware
 *              MCU: STM32F103ZET6 | CMOS: OV7725 + AL422B | LCD: 2.0" ST7789V3 (8-bit FSMC)
 ****************************************************************************************************
 */
/* USER CODE END Header */

#include <string.h>

#include "main.h"
#include "adc.h"
#include "boot_anim.h"
#include "c3m1_cfg.h"
#include "c3m1_clock.h"
#include "c3m1_dng.h"
#include "c3m1_jpeg.h"
#include "fatfs.h"
#include "fw_version.h"
#include "menu.h"
#include "playback.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

#include "../../BSP/LED/led.h"
#include "../../BSP/KEY/key.h"
#include "../../BSP/LCD/lcd.h"
#include "../../BSP/OV7725/ov7725.h"
#include "../../BSP/EXTI/exti.h"
#include "../../SYSTEM/delay/delay.h"
#include "../../BSP/STMFLASH/stmflash.h"

/* ======================== Configuration ======================== */
#define FLASH_SAVE_ADDR             0x08070000UL    /* ShutterCount persistent page */
#define C3M1_PHOTO_DIR              "0:CAYE_CAM"    /* Directory for all saved DNG photos */
#define CFG_SAVE_DEBOUNCE_MS        1200U           /* Delay before persisting ISO changes to SD */
#define VIEWFINDER_HEIGHT           240U            /* Full 4:3 320x240 uncropped live view; bottom 20px translucent HUD */
#define OSD_BAR_Y                   220U
#define OSD_BAR_H                   20U
#define BAT_POLL_INTERVAL_MS        2000U
#define BAT_LOW_WARN_MV             3450U           /* Red voltage warning (~16% loaded) */
#define BAT_MID_WARN_MV             3520U           /* Yellow heads-up warning (~25% loaded) */
#define BAT_CUTOFF_MV               3250U           /* Lock SD writes below 3.25V (if real battery >= 2.0V) */
#define BAT_SLEEP_MV                3200U           /* Force low-power sleep below 3.20V (if real battery >= 2.0V) */
#define BAT_RECOVER_PCT             25U             /* Light-load % to auto-switch low-bat sleep -> normal sleep */
#define BAT_PRESENT_MIN_MV          2000U           /* Ignore cutoff when bench-powered via ST-Link only */
#define SD_POLL_INTERVAL_MS         500U
#define TOAST_DURATION_MS           1000U
#define HDR_SHUTTER_DELAY_MS        3000UL          /* Tripod self-timer before HDR bracket capture */
#define C3M1_CAPTURE_SENSOR_TIMEOUT 0xFDU
#define C3M1_CAPTURE_CANCELLED      0xFEU

/* ======================== Global State ======================== */
volatile uint8_t  RAW_shot     = 0;
volatile uint16_t ShutterCount = 0;

FRESULT res;
FATFS   fs;
FIL     file;
UINT    bw;

static uint8_t  g_sd_ready             = 0;
static uint8_t  g_sd_last_pin          = 0;
static uint32_t g_sd_free_kb           = 0UL;
static uint8_t  g_clock_valid          = 0U; /* 1 once seeded from user boot-clock UI or SD saved_epoch */
static uint8_t  g_dial_pos             = 0;
static uint8_t  g_iso_idx              = 0;
static uint8_t  g_zoom_4x              = 0;
static uint8_t  g_drive_mode           = C3M1_DRIVE_SINGLE; /* Volatile runtime-only drive mode */
static uint8_t  g_hdr_mode             = C3M1_HDR_OFF;      /* Volatile runtime-only HDR merge mode */
static uint8_t  g_jsr_mode             = C3M1_JSR_OFF;      /* Volatile runtime-only JSR burst mode */
static uint8_t  g_shutter_cocked       = 1;
static uint16_t g_bat_mv               = 4000U;
static uint8_t  g_osd_dirty            = 1;
static uint8_t  g_wind_box_drawn       = 0;
static uint32_t g_last_activity_tick   = 0;
static uint32_t g_toast_until          = 0;
static uint32_t g_cfg_save_due         = 0;
static uint8_t  g_pending_btn_evts     = BTN_EVT_NONE; /* Captured mid-frame so 88-140ms paints never drop taps */

/* Shared 4 KB buffer for DNG streaming writes and menu f_mkfs() */
static uint8_t  s_sd_io_buf[4096];

/* Always-RAW live viewfinder AE / AWB / LUT state */
static uint16_t g_auto_aec             = 240U; /* ~1/140s initial auto shutter @CLKRC=0x00 */
static uint16_t g_auto_gain_q4         = 16U;  /* 16 = 1.0x (ISO 100) .. 256 = 16.0x (ISO 1600) */
static uint16_t g_wb_r_q8              = 256U; /* 1.00x OV7725 D65 daylight R gain */
static uint16_t g_wb_b_q8              = 406U; /* 1.59x OV7725 D65 daylight B gain */
static uint16_t g_last_wb_kelvin       = 0U;   /* Cached OSD WB Kelvin (100K steps) */
static uint16_t g_lut_last_r_q8        = 0U;
static uint16_t g_lut_last_b_q8        = 0U;
static uint16_t g_lut_last_boost_q8    = 0U;
static uint8_t  g_lut_last_gain_x      = 1U;   /* Active hardware gain_x (1..32) for per-channel 12-bit BlackLevel */
static uint16_t g_preview_boost_q8     = 256U; /* 256 = 1.0x digital boost for extreme slow shutters */
static uint8_t  g_ae_settle_frames     = 0U;   /* Skip 1 frame after SCCB write so new exposure settles */
static uint8_t  g_pending_ae_apply     = 0U;   /* Applied atomically right after VSYNC (g_vsync_state==1) */
static uint8_t  g_exposure_dirty_frames = 1U;  /* >0 right after dial/ISO/AE change */
static char     g_last_auto_s_label[10] = "";
static char     g_last_auto_i_label[8]  = "";
static uint16_t g_meter_eff_q4         = 736U; /* Filtered linear green mean Q4 (736 = 46*16 = 0.0 EV) */
static int8_t   g_meter_ev_x10         = 0;    /* Live exposure meter in 0.1 EV (-50..+30) */
static uint8_t  g_vf_fps               = 0U;
static uint16_t g_vf_frame_cnt         = 0U;
static uint32_t g_vf_fps_tick          = 0U;
static uint16_t g_vf_last_paint_ms     = 0U;

/* Filmic S-curve sRGB OETF table (256 B Flash): linear 0..255 -> sRGB 0..255 with crisp IPS contrast */
static const uint8_t s_srgb_lut[256] = {
      0,   9,  16,  21,  25,  29,  32,  35,  38,  41,  44,  47,  49,  52,  54,  57,
     59,  61,  64,  66,  68,  70,  72,  74,  76,  78,  80,  82,  84,  86,  88,  90,
     92,  93,  95,  97,  99, 100, 102, 104, 105, 107, 109, 110, 112, 113, 115, 116,
    118, 119, 121, 122, 124, 125, 127, 128, 129, 131, 132, 134, 135, 136, 137, 139,
    140, 141, 143, 144, 145, 146, 147, 149, 150, 151, 152, 153, 154, 155, 157, 158,
    159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174,
    175, 176, 176, 177, 178, 179, 180, 181, 182, 183, 183, 184, 185, 186, 187, 187,
    188, 189, 190, 190, 191, 192, 193, 193, 194, 195, 196, 196, 197, 198, 198, 199,
    200, 200, 201, 202, 202, 203, 204, 204, 205, 206, 206, 207, 207, 208, 209, 209,
    210, 210, 211, 212, 212, 213, 213, 214, 214, 215, 215, 216, 216, 217, 218, 218,
    219, 219, 220, 220, 221, 221, 222, 222, 223, 223, 223, 224, 224, 225, 225, 226,
    226, 227, 227, 228, 228, 228, 229, 229, 230, 230, 231, 231, 231, 232, 232, 233,
    233, 233, 234, 234, 235, 235, 235, 236, 236, 237, 237, 237, 238, 238, 239, 239,
    239, 240, 240, 240, 241, 241, 241, 242, 242, 243, 243, 243, 244, 244, 244, 245,
    245, 245, 246, 246, 246, 247, 247, 247, 248, 248, 248, 249, 249, 249, 250, 250,
    250, 251, 251, 251, 252, 252, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255,
};

/* Zero-wait-state SRAM 6-bit sRGB LUTs (768 B contiguous for single-base indexing), 644 B RAW row buf, dual 640 B DMA line bufs */
static uint8_t  s_lut_rgb6[3][256] __attribute__((aligned(4)));
#define s_lut_r6 (s_lut_rgb6[0])
#define s_lut_g6 (s_lut_rgb6[1]) /* Shared by demosaic + zero-unpack focus peaking */
#define s_lut_b6 (s_lut_rgb6[2])
static uint8_t  s_raw_row_buf[648] __attribute__((aligned(4)));
static uint8_t  s_line_buf[2][640] __attribute__((aligned(4)));

/* 256-entry 12-bit inverse LUT for OV7725 GAM1..GAM15 (0..4064, 0.25-LSB precision; shared by DNG 0xC618, LCD LUT & JPEG) */
static const uint16_t s_inv_gamma_12[256] = {
       0,    5,   11,   16,   21,   27,   32,   37,   43,   48,   53,   59,   64,   70,   77,   83,
      90,   96,  102,  109,  115,  122,  128,  134,  141,  147,  154,  160,  166,  173,  179,  186,
     192,  198,  205,  211,  218,  224,  230,  237,  243,  250,  256,  263,  270,  277,  284,  292,
     299,  306,  313,  320,  327,  334,  341,  348,  356,  363,  370,  377,  384,  391,  398,  405,
     412,  420,  427,  434,  441,  448,  455,  462,  469,  476,  484,  491,  498,  505,  512,  519,
     525,  532,  539,  546,  552,  559,  566,  573,  579,  586,  593,  600,  606,  613,  620,  627,
     633,  640,  649,  658,  667,  677,  686,  695,  704,  713,  722,  731,  741,  750,  759,  768,
     779,  789,  800,  811,  821,  832,  843,  853,  864,  875,  885,  896,  908,  919,  931,  943,
     954,  966,  977,  989, 1001, 1012, 1024, 1040, 1056, 1072, 1088, 1104, 1120, 1136, 1152, 1166,
    1180, 1195, 1209, 1223, 1237, 1252, 1266, 1280, 1300, 1319, 1339, 1359, 1378, 1398, 1418, 1438,
    1457, 1477, 1497, 1516, 1536, 1559, 1583, 1606, 1629, 1652, 1676, 1699, 1722, 1745, 1769, 1792,
    1815, 1839, 1862, 1885, 1908, 1932, 1955, 1978, 2001, 2025, 2048, 2071, 2095, 2118, 2141, 2164,
    2188, 2211, 2234, 2257, 2281, 2304, 2332, 2361, 2389, 2418, 2446, 2475, 2503, 2532, 2560, 2588,
    2617, 2645, 2674, 2702, 2731, 2759, 2788, 2816, 2846, 2876, 2906, 2936, 2967, 2997, 3027, 3057,
    3087, 3117, 3147, 3177, 3208, 3238, 3268, 3298, 3328, 3360, 3392, 3424, 3456, 3488, 3520, 3552,
    3584, 3616, 3648, 3680, 3712, 3744, 3776, 3808, 3840, 3872, 3904, 3936, 3968, 4000, 4032, 4064,
};

void SystemClock_Config(void);

/* ======================== Flash Counter & SD Helpers ======================== */
static uint16_t shutter_count_add(uint16_t base, uint16_t delta)
{
    /* Keep valid shutter numbers in 1..65534 (reserve 0xFFFF strictly as erased Flash sentinel) */
    uint32_t s = (uint32_t)base + (uint32_t)delta;
    while (s > (uint32_t)STMFLASH_CNT_MAX_VAL)
    {
        s -= (uint32_t)STMFLASH_CNT_MAX_VAL;
    }
    return (uint16_t)s;
}

static uint16_t shutter_count_load(void)
{
    uint16_t val = 0U;
    (void)stmflash_counter_load(FLASH_SAVE_ADDR, &val, NULL, NULL);
    ShutterCount = val;
    return val;
}

static void shutter_count_save(uint16_t val)
{
    if (!stmflash_counter_save(FLASH_SAVE_ADDR, val))
    {
        printf("FLASH cnt save err (#%u)\r\n", (unsigned int)val);
    }
}

static uint8_t shutter_cnt_get_mode(void)
{
    return stmflash_counter_detect_mode(FLASH_SAVE_ADDR);
}

static uint16_t shutter_cnt_get_slots(void)
{
    uint16_t val   = 0U;
    uint16_t slots = 0U;
    (void)stmflash_counter_load(FLASH_SAVE_ADDR, &val, NULL, &slots);
    return slots;
}

static uint8_t shutter_cnt_set_mode(uint8_t mode)
{
    uint16_t cur = ShutterCount;
    if (!stmflash_counter_set_mode(FLASH_SAVE_ADDR, mode, cur))
    {
        printf("FLASH cnt mode err (%u)\r\n", (unsigned int)mode);
        return 0U;
    }
    uint8_t  det_mode = 0U;
    uint16_t det_slot = 0U;
    uint16_t det_val  = 0U;
    (void)stmflash_counter_load(FLASH_SAVE_ADDR, &det_val, &det_mode, &det_slot);
    ShutterCount = det_val;
    printf("FLASH cnt -> %s (#%u, slot %u)\r\n",
           (det_mode != 0U) ? "WEAR-LVL" : "STOCK",
           (unsigned int)det_val, (unsigned int)det_slot);
    return (det_mode == mode) ? 1U : 0U;
}

static uint8_t sd_ensure_photo_dir(void)
{
    FRESULT dres = f_mkdir(C3M1_PHOTO_DIR);
    if (dres == FR_OK || dres == FR_EXIST)
    {
        return 1U;
    }
    printf("SD mkdir err:%d (%s)\r\n", dres, C3M1_PHOTO_DIR);
    return 0U;
}

static uint8_t sd_ensure_photo_day_dir(const c3m1_datetime_t *dt, char *out_dir, size_t out_sz)
{
    if (!sd_ensure_photo_dir())
    {
        return 0U;
    }
    snprintf(out_dir, out_sz, "%s/%04u%02u%02u",
             C3M1_PHOTO_DIR,
             (unsigned int)dt->year,
             (unsigned int)dt->month,
             (unsigned int)dt->day);
    FRESULT dres = f_mkdir(out_dir);
    if (dres == FR_OK || dres == FR_EXIST)
    {
        return 1U;
    }
    printf("SD mkdir err:%d (%s)\r\n", dres, out_dir);
    return 0U;
}

static uint8_t battery_is_cutoff_low(void)
{
    g_bat_mv = adc_battery_get_mv();
    return (g_bat_mv >= BAT_PRESENT_MIN_MV && g_bat_mv < BAT_CUTOFF_MV) ? 1U : 0U;
}

static void raw_preview_rebuild_luts(uint16_t wb_r_q8, uint16_t wb_b_q8, uint16_t boost_q8, uint8_t gain_x);

static uint16_t wb_gains_to_kelvin(uint16_t wb_r_q8, uint16_t wb_b_q8)
{
    /* Calibrated from OV7725 ColorMatrix1: ratio = (wb_r_q8 * 256) / wb_b_q8 -> CCT (K) */
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
    for (uint8_t i = 0; i < 15U; i++)
    {
        if (ratio <= r_tbl[i + 1U])
        {
            uint32_t span = (uint32_t)(r_tbl[i + 1U] - r_tbl[i]);
            uint32_t k    = (uint32_t)k_tbl[i] +
                            (((ratio - r_tbl[i]) * (uint32_t)(k_tbl[i + 1U] - k_tbl[i]) + (span >> 1)) / span);
            return (uint16_t)(((k + 50UL) / 100UL) * 100UL); /* Round to clean 100K step */
        }
    }
    return 5500U;
}

static int8_t meter_eff_q4_to_ev_x10(uint32_t eff_q4)
{
    /* lut[k] = round(256 * log2(1 + k/16)) for k = 0..16 */
    static const uint16_t log2_lut[17] = {
        0U, 22U, 43U, 64U, 82U, 100U, 117U, 134U, 150U,
        165U, 180U, 194U, 207U, 220U, 233U, 245U, 256U,
    };
    if (eff_q4 <= 23UL) return -50; /* <= -5.0 EV */

    uint32_t msb  = 31UL - (uint32_t)__builtin_clz(eff_q4);
    uint32_t m_q8 = (msb >= 8UL) ? (eff_q4 >> (msb - 8UL)) : (eff_q4 << (8UL - msb));
    uint32_t idx  = (m_q8 - 256UL) >> 4;
    uint32_t rem  = (m_q8 - 256UL) & 0x0FUL;
    if (idx > 15UL) { idx = 15UL; rem = 16UL; }

    uint32_t frac_q8  = log2_lut[idx] + (((log2_lut[idx + 1UL] - log2_lut[idx]) * rem + 8UL) >> 4);
    int32_t  log2_q8  = (int32_t)((msb << 8) + frac_q8);
    int32_t  ev_q8    = log2_q8 - 2438; /* 2438 = round(256 * log2(46 * 16)) -> 18% linear gray = 0.0 EV */
    int32_t  ev_x10   = (ev_q8 >= 0) ? ((ev_q8 * 10 + 128) >> 8) : -(((-ev_q8) * 10 + 128) >> 8);
    if (ev_x10 < -50) ev_x10 = -50;
    if (ev_x10 >  30) ev_x10 =  30;
    return (int8_t)ev_x10;
}

static void cfg_apply_to_runtime(void)
{
    g_iso_idx = (g_cfg.iso_idx < OV7725_ISO_COUNT) ? g_cfg.iso_idx : 0U;
    uint16_t max_gain = c3m1_cfg_max_auto_gain_q4(&g_cfg);
    uint16_t max_aec  = c3m1_cfg_min_auto_aec(&g_cfg);
    if (g_auto_gain_q4 > max_gain) g_auto_gain_q4 = max_gain;
    if (g_auto_aec > max_aec)      g_auto_aec     = max_aec;

    uint16_t wbr = 0U, wbb = 0U;
    if (c3m1_cfg_wb_gains(&g_cfg, &wbr, &wbb))
    {
        g_wb_r_q8 = wbr;
        g_wb_b_q8 = wbb;
        raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, g_preview_boost_q8, g_lut_last_gain_x);
    }

    if (!g_cfg.film_lever)
    {
        g_shutter_cocked = 1U;
        g_wind_box_drawn = 0U;
    }
    g_last_auto_s_label[0] = '\0';
    g_last_auto_i_label[0] = '\0';
    g_last_wb_kelvin       = 0U;
}

static void sd_refresh_free_kb(void)
{
    if (!g_sd_ready)
    {
        g_sd_free_kb = 0UL;
        return;
    }

    DWORD  fre_clust = 0UL;
    FATFS *pfs       = NULL;
    if (f_getfree("0:", &fre_clust, &pfs) == FR_OK && pfs != NULL)
    {
        uint64_t free_kb = ((uint64_t)fre_clust * (uint64_t)pfs->csize) >> 1; /* 512 B/sector -> KB */
        g_sd_free_kb = (free_kb > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)free_kb;
    }
}

static uint8_t sd_try_mount(void)
{
    static const char *const fs_names[] = { "NONE", "FAT12", "FAT16", "FAT32", "exFAT" };

    g_sd_last_pin = sd_card_is_present();
    if (!g_sd_last_pin)
    {
        MX_SDIO_SD_DeInit();
        g_sd_ready   = 0;
        g_sd_free_kb = 0UL;
        printf("NO_SD_CARD\r\n");
        return 0;
    }
    if (MX_SDIO_SD_Init() != 0)
    {
        MX_SDIO_SD_DeInit();
        g_sd_ready   = 0;
        g_sd_free_kb = 0UL;
        printf("SDIO init failed\r\n");
        return 0;
    }
    res = f_mount(&fs, "0:", 1);
    if (res == FR_OK)
    {
        const char *fs_str = (fs.fs_type <= FS_EXFAT) ? fs_names[fs.fs_type] : "UNK";
        g_sd_ready = 1;
        sd_ensure_photo_dir();
        printf("SD_Mounted: %s (%lu MB)\r\n", fs_str, (unsigned long)SD_TOTAL_SIZE_MB(&hsd));
        c3m1_cfg_status_t cst = c3m1_cfg_load(&g_cfg);
        if (cst == CFG_LOAD_IO_ERR)
        {
            delay_ms(20);
            cst = c3m1_cfg_load(&g_cfg);
        }
        if ((cst == CFG_LOAD_OK || cst == CFG_LOAD_RECOVERED_TMP) &&
            !g_clock_valid && g_cfg.saved_epoch != 0UL)
        {
            /* Restore persisted SD clock when booting without prompt or hot-inserting SD after cardless boot */
            c3m1_clock_set_epoch(g_cfg.saved_epoch);
            g_clock_valid = 1U;
        }
        if (cst == CFG_LOAD_RECOVERED_TMP && !battery_is_cutoff_low())
        {
            (void)c3m1_cfg_heal_tmp();
        }
        else if (cst != CFG_LOAD_OK && cst != CFG_LOAD_RECOVERED_TMP &&
                 cst != CFG_LOAD_IO_ERR && !battery_is_cutoff_low())
        {
            c3m1_cfg_save(&g_cfg);
        }
        cfg_apply_to_runtime();
        sd_refresh_free_kb();
        return 1;
    }
    MX_SDIO_SD_DeInit();
    g_sd_ready   = 0;
    g_sd_free_kb = 0UL;
    printf("SD_ERR:%d\r\n", res);
    return 0;
}

static uint8_t sd_remount_after_format(void)
{
    res = f_mount(&fs, "0:", 1);
    if (res == FR_OK)
    {
        g_sd_ready = 1;
        sd_ensure_photo_dir();
        sd_refresh_free_kb();
        return 1;
    }
    g_sd_ready   = 0;
    g_sd_free_kb = 0UL;
    return 0;
}

/* ======================== OSD Status Bar & Toast ======================== */
/* Unified centered modal prompt box (same style as NO SD CARD!) */
#define PROMPT_BOX_X0       24U
#define PROMPT_BOX_Y0       92U
#define PROMPT_BOX_X1       295U
#define PROMPT_BOX_Y1       132U

static void osd_draw_center_box(const char *msg, uint16_t color)
{
    size_t   len   = strlen(msg);
    uint16_t box_w = (PROMPT_BOX_X1 - PROMPT_BOX_X0 + 1U);
    uint16_t txt_w = (uint16_t)(len * 8U); /* 16px font = 8px per char */
    uint16_t tx    = (txt_w < box_w) ? (uint16_t)(PROMPT_BOX_X0 + (box_w - txt_w) / 2U)
                                     : (uint16_t)(PROMPT_BOX_X0 + 4U);

    g_back_color = BLACK;
    lcd_fill(PROMPT_BOX_X0, PROMPT_BOX_Y0, PROMPT_BOX_X1, PROMPT_BOX_Y1, BLACK);
    lcd_draw_rectangle(PROMPT_BOX_X0, PROMPT_BOX_Y0, PROMPT_BOX_X1, PROMPT_BOX_Y1, color);
    lcd_show_string(tx, PROMPT_BOX_Y0 + 12U, box_w - 8U, 16, 16, msg, color);
}

/* Inline translucent bottom HUD buffer: 320x12 @ 4-bit palette index (1,920 B .bss, 0=translucent bg) */
static uint8_t  s_osd_bar_buf[12][160];

/* Transparent top-left mirrorless HUD (remaining shots + shooting mode, NO background shadow) */
#define TOP_HUD_X0          4U
#define TOP_HUD_Y0          4U      /* Pair-aligned even..odd (4..15 = 12px font height) */
#define TOP_HUD_Y1          15U
#define TOP_HUD_W           84U     /* Up to 14 chars x 6px = 84px (x = 4..87) */
static uint8_t  s_top_hud_buf[12][TOP_HUD_W / 2U];

enum
{
    OSD_COL_BG     = 0U,
    OSD_COL_WHITE  = 1U,
    OSD_COL_YELLOW = 2U,
    OSD_COL_CYAN   = 3U,
    OSD_COL_GREEN  = 4U,
    OSD_COL_RED    = 5U,
    OSD_COL_ORANGE = 6U,
    OSD_COL_GRAY   = 7U,
};

static const uint16_t s_osd_pal[8] = {
    BLACK, WHITE, YELLOW, CYAN, GREEN, RED, ORANGE, GRAY,
};

static uint8_t osd_rgb565_to_idx(uint16_t c)
{
    switch (c)
    {
        case WHITE:  return OSD_COL_WHITE;
        case YELLOW: return OSD_COL_YELLOW;
        case CYAN:   return OSD_COL_CYAN;
        case GREEN:  return OSD_COL_GREEN;
        case RED:    return OSD_COL_RED;
        case ORANGE: return OSD_COL_ORANGE;
        case GRAY:   return OSD_COL_GRAY;
        default:     return OSD_COL_WHITE;
    }
}

static void osd_put_str_4bit(uint8_t *buf, uint16_t row_bytes, uint16_t buf_w,
                             uint16_t x, uint16_t max_w, const char *s, uint8_t col_idx)
{
    uint16_t xmax = (uint16_t)(x + max_w);
    if (xmax > buf_w) xmax = buf_w;
    while (*s != '\0' && (x + 6U) <= xmax)
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
                    uint8_t *cell = &buf[(uint16_t)r * row_bytes + bi];
                    if (hi) *cell = (uint8_t)((*cell & 0x0FU) | (col_idx << 4));
                    else    *cell = (uint8_t)((*cell & 0xF0U) | col_idx);
                }
            }
            for (uint8_t r = 0U; r < 4U; r++)
            {
                if (b1 & (0x80U >> r))
                {
                    uint8_t *cell = &buf[(uint16_t)(8U + r) * row_bytes + bi];
                    if (hi) *cell = (uint8_t)((*cell & 0x0FU) | (col_idx << 4));
                    else    *cell = (uint8_t)((*cell & 0xF0U) | col_idx);
                }
            }
        }
        x = (uint16_t)(x + 6U);
    }
}

static void osd_bar_put_str(uint16_t x, uint16_t max_w, const char *s, uint8_t col_idx)
{
    osd_put_str_4bit(&s_osd_bar_buf[0][0], 160U, 320U, x, max_w, s, col_idx);
}

static uint8_t jsr_frame_count(uint8_t jsr_mode)
{
    if (jsr_mode == C3M1_JSR_30F) return 30U;
    if (jsr_mode == C3M1_JSR_14F) return 14U;
    if (jsr_mode == C3M1_JSR_7F)  return 7U;
    return 0U;
}

static uint32_t camera_est_shot_kb(void)
{
    uint32_t hdr_nf = (g_hdr_mode == C3M1_HDR_16B_4F) ? 4UL
                    : ((g_hdr_mode == C3M1_HDR_14B_3F) ? 3UL
                    : ((g_hdr_mode == C3M1_HDR_12B_2F) ? 2UL : 0UL));
    if (g_jsr_mode != C3M1_JSR_OFF)
    {
        uint32_t total_sets = (uint32_t)jsr_frame_count(g_jsr_mode) + 1UL; /* N light + 1 dark */
        uint32_t per_set_kb = (hdr_nf > 0UL)
                            ? (g_cfg.del_hdr_brk ? 704UL : (hdr_nf * 384UL + 704UL))
                            : 384UL;
        return total_sets * per_set_kb;
    }
    if (hdr_nf > 0UL)
    {
        return g_cfg.del_hdr_brk ? 704UL : (hdr_nf * 384UL + 672UL);
    }
    uint32_t single_kb = (g_cfg.save_mode == C3M1_SAVE_DNG_JPG) ? 448UL
                       : ((g_cfg.save_mode == C3M1_SAVE_JPG) ? 72UL : 384UL);
    if (g_drive_mode == C3M1_DRIVE_AEB)
    {
        return 3UL * single_kb; /* 3 independent bracketed frames (no HDR merge) */
    }
    return single_kb;
}

static void osd_rebuild_top_hud_buf(void)
{
    char    cnt_str[10];
    char    mode_buf[10];
    uint8_t cnt_col = OSD_COL_WHITE;

    memset(s_top_hud_buf, 0, sizeof(s_top_hud_buf));

    if (!g_sd_ready)
    {
        snprintf(cnt_str, sizeof(cnt_str), "----");
        cnt_col = OSD_COL_RED;
    }
    else
    {
        uint32_t shot_kb = camera_est_shot_kb();
        uint32_t rem     = g_sd_free_kb / shot_kb;
        if (rem > 99999UL)
        {
            snprintf(cnt_str, sizeof(cnt_str), ">99999");
        }
        else
        {
            snprintf(cnt_str, sizeof(cnt_str), "%lu", (unsigned long)rem);
            if (rem == 0UL)       cnt_col = OSD_COL_RED;
            else if (rem < 50UL)  cnt_col = OSD_COL_YELLOW;
        }
    }

    uint16_t cnt_w = (uint16_t)(strlen(cnt_str) * 6U);
    osd_put_str_4bit(&s_top_hud_buf[0][0], TOP_HUD_W / 2U, TOP_HUD_W, 0U, cnt_w, cnt_str, cnt_col);

    const char *mode_str;
    uint8_t     mode_col;
    if (g_jsr_mode != C3M1_JSR_OFF)
    {
        uint8_t n = jsr_frame_count(g_jsr_mode);
        if (g_hdr_mode == C3M1_HDR_16B_4F)
        {
            snprintf(mode_buf, sizeof(mode_buf), "H16x%u", (unsigned int)n);
        }
        else if (g_hdr_mode == C3M1_HDR_14B_3F)
        {
            snprintf(mode_buf, sizeof(mode_buf), "H14x%u", (unsigned int)n);
        }
        else if (g_hdr_mode == C3M1_HDR_12B_2F)
        {
            snprintf(mode_buf, sizeof(mode_buf), "H12x%u", (unsigned int)n);
        }
        else
        {
            snprintf(mode_buf, sizeof(mode_buf), "JSR%u", (unsigned int)n);
        }
        mode_str = mode_buf;
        mode_col = OSD_COL_CYAN;
    }
    else if (g_hdr_mode == C3M1_HDR_16B_4F)
    {
        mode_str = "H16";
        mode_col = OSD_COL_CYAN;
    }
    else if (g_hdr_mode == C3M1_HDR_14B_3F)
    {
        mode_str = "H14";
        mode_col = OSD_COL_CYAN;
    }
    else if (g_hdr_mode == C3M1_HDR_12B_2F)
    {
        mode_str = "H12";
        mode_col = OSD_COL_CYAN;
    }
    else if (g_drive_mode == C3M1_DRIVE_AEB)
    {
        if (g_cfg.save_mode == C3M1_SAVE_DNG_JPG)      mode_str = "AEB+J";
        else if (g_cfg.save_mode == C3M1_SAVE_JPG)     mode_str = "AEBJ";
        else                                           mode_str = "AEB";
        mode_col = OSD_COL_ORANGE;
    }
    else if (g_drive_mode == C3M1_DRIVE_BURST)
    {
        if (g_cfg.save_mode == C3M1_SAVE_DNG_JPG)
        {
            mode_str = "B:D+J";
            mode_col = OSD_COL_YELLOW;
        }
        else if (g_cfg.save_mode == C3M1_SAVE_JPG)
        {
            mode_str = "B:JPG";
            mode_col = OSD_COL_GREEN;
        }
        else
        {
            mode_str = "B:DNG";
            mode_col = OSD_COL_CYAN;
        }
    }
    else if (g_cfg.save_mode == C3M1_SAVE_DNG_JPG)
    {
        mode_str = "D+J";
        mode_col = OSD_COL_YELLOW;
    }
    else if (g_cfg.save_mode == C3M1_SAVE_JPG)
    {
        mode_str = "JPG";
        mode_col = OSD_COL_GREEN;
    }
    else
    {
        mode_str = "DNG";
        mode_col = OSD_COL_CYAN;
    }

    uint16_t mode_x = (uint16_t)(cnt_w + 6U);
    osd_put_str_4bit(&s_top_hud_buf[0][0], TOP_HUD_W / 2U, TOP_HUD_W,
                     mode_x, (uint16_t)(TOP_HUD_W - mode_x), mode_str, mode_col);

    if (g_cfg.color_mode != C3M1_COLOR_COLOR)
    {
        const char *bw_str = (g_cfg.color_mode == C3M1_COLOR_BW_RED) ? "BWR"
                           : ((g_cfg.color_mode == C3M1_COLOR_BW_HI) ? "BW+" : "BW");
        uint8_t     bw_col = (g_cfg.color_mode == C3M1_COLOR_BW_RED) ? OSD_COL_RED
                           : ((g_cfg.color_mode == C3M1_COLOR_BW_HI) ? OSD_COL_YELLOW : OSD_COL_WHITE);
        uint16_t    bw_x   = (uint16_t)(mode_x + (uint16_t)strlen(mode_str) * 6U + 4U);
        uint16_t    bw_w   = (uint16_t)(strlen(bw_str) * 6U);
        if ((uint16_t)(bw_x + bw_w) <= TOP_HUD_W)
        {
            osd_put_str_4bit(&s_top_hud_buf[0][0], TOP_HUD_W / 2U, TOP_HUD_W, bw_x, bw_w, bw_str, bw_col);
        }
    }
}

static void osd_rebuild_bar_buf(const char *status_override, uint16_t status_color)
{
    char line[48];
    const char *st_text;
    uint8_t st_col;
    uint8_t bat_col = OSD_COL_GREEN;
    if (g_bat_mv >= BAT_PRESENT_MIN_MV && g_bat_mv < BAT_LOW_WARN_MV)
    {
        bat_col = OSD_COL_RED;
    }
    else if (g_bat_mv >= BAT_PRESENT_MIN_MV && g_bat_mv < BAT_MID_WARN_MV)
    {
        bat_col = OSD_COL_YELLOW;
    }

    memset(s_osd_bar_buf, 0, sizeof(s_osd_bar_buf));

    char shutter_str[10];
    char iso_str[8];
    ov7725_format_shutter_value(g_dial_pos, g_auto_aec, shutter_str, sizeof(shutter_str));
    ov7725_format_iso_value(g_iso_idx, g_auto_gain_q4, iso_str, sizeof(iso_str));

    snprintf(line, sizeof(line), "%s%-7s",
             ov7725_is_auto_shutter(g_dial_pos) ? "SA:" : "S:",
             shutter_str);
    osd_bar_put_str(2U, 60U, line, OSD_COL_WHITE);

    snprintf(line, sizeof(line), "%s%-4s",
             (g_iso_idx == 0U) ? "IA:" : "I:",
             iso_str);
    osd_bar_put_str(68U, 42U, line, OSD_COL_WHITE);

    {
        int8_t  ev     = g_meter_ev_x10;
        uint8_t ev_abs = (ev < 0) ? (uint8_t)(-ev) : (uint8_t)ev;
        char    ev_sgn = (ev < 0) ? '-' : '+';
        uint8_t auto_active = (ov7725_is_auto_shutter(g_dial_pos) || g_iso_idx == 0U) ? 1U : 0U;
        uint8_t ev_col;
        if (auto_active && g_cfg.ev_comp != C3M1_EV_COMP_ZERO)
        {
            int32_t target_x10 = ((int32_t)g_cfg.ev_comp - (int32_t)C3M1_EV_COMP_ZERO) * 10 / 3;
            int32_t err_x10    = (int32_t)ev - target_x10;
            if (err_x10 < 0) err_x10 = -err_x10;
            ev_col = (err_x10 <= 4) ? OSD_COL_CYAN : ((err_x10 <= 13) ? OSD_COL_YELLOW : OSD_COL_RED);
        }
        else
        {
            ev_col = (ev_abs <= 3U) ? OSD_COL_WHITE : ((ev_abs <= 13U) ? OSD_COL_YELLOW : OSD_COL_RED);
        }
        snprintf(line, sizeof(line), "%c%u.%uEV", ev_sgn, (unsigned int)(ev_abs / 10U), (unsigned int)(ev_abs % 10U));
        osd_bar_put_str(116U, 36U, line, ev_col);
    }

    uint16_t wb_k = wb_gains_to_kelvin(g_wb_r_q8, g_wb_b_q8);
    if (g_cfg.wb_mode == C3M1_WB_AUTO)
    {
        snprintf(line, sizeof(line), "A%uK", (unsigned int)wb_k);
        osd_bar_put_str(158U, 36U, line, OSD_COL_WHITE);
    }
    else
    {
        static const uint16_t k_preset[C3M1_WB_MODE_COUNT] = { 5500U, 3200U, 5500U, 6500U };
        uint16_t pk = (g_cfg.wb_mode < C3M1_WB_MODE_COUNT) ? k_preset[g_cfg.wb_mode] : 5500U;
        snprintf(line, sizeof(line), "%uK", (unsigned int)pk);
        osd_bar_put_str(158U, 36U, line, OSD_COL_YELLOW);
    }
    g_last_wb_kelvin = wb_k;

    if (g_zoom_4x)
    {
        osd_bar_put_str(200U, 12U, "4X", OSD_COL_YELLOW);
    }

    if (status_override != NULL)
    {
        st_text = status_override;
        st_col  = osd_rgb565_to_idx(status_color);
    }
    else if (!g_sd_ready)
    {
        st_text = "NOSD";
        st_col  = OSD_COL_RED;
    }
    else if (g_cfg.film_lever && !g_shutter_cocked)
    {
        st_text = "WIND";
        st_col  = OSD_COL_ORANGE;
    }
    else
    {
        st_text = "RDY ";
        st_col  = OSD_COL_GREEN;
    }
    osd_bar_put_str(260U, 24U, st_text, st_col);

    if (g_cfg.bat_disp == 1U)
    {
        snprintf(line, sizeof(line), " %3u%%", (unsigned int)adc_mv_to_percent(g_bat_mv));
    }
    else
    {
        snprintf(line, sizeof(line), "%u.%02uV",
                 (unsigned int)(g_bat_mv / 1000U),
                 (unsigned int)((g_bat_mv % 1000U) / 10U));
    }
    osd_bar_put_str(288U, 30U, line, bat_col);

    /* Unified centered WIND prompt box whenever film advance lever needs cocking (draw once, no flicker) */
    if (status_override == NULL && g_toast_until == 0 && g_cfg.film_lever && !g_shutter_cocked)
    {
        if (!g_wind_box_drawn)
        {
            osd_draw_center_box("WIND FILM LEVER", ORANGE);
            g_wind_box_drawn = 1U;
        }
    }
    else if (g_shutter_cocked)
    {
        g_wind_box_drawn = 0U;
    }

    osd_rebuild_top_hud_buf();
    g_osd_dirty = 0;
}

/* Direct solid-black bar paint (used at boot & during EXP/SAVE when live DMA is paused) */
static void osd_draw_bar(const char *status_override, uint16_t status_color)
{
    osd_rebuild_bar_buf(status_override, status_color);

    lcd_set_window(0U, OSD_BAR_Y, 320U, OSD_BAR_H);
    lcd_write_ram_prepare();
    for (uint16_t y = OSD_BAR_Y; y < (OSD_BAR_Y + OSD_BAR_H); y++)
    {
        if (y == OSD_BAR_Y)
        {
            for (uint16_t x = 0U; x < 320U; x++) lcd_wr_pixel(GRAY);
        }
        else if (y >= (OSD_BAR_Y + 4U) && y < (OSD_BAR_Y + 16U))
        {
            const uint8_t *row = s_osd_bar_buf[y - (OSD_BAR_Y + 4U)];
            for (uint16_t i = 0U; i < 160U; i++)
            {
                uint8_t b = row[i];
                lcd_wr_pixel(s_osd_pal[b & 0x0FU]);
                lcd_wr_pixel(s_osd_pal[b >> 4]);
            }
        }
        else
        {
            for (uint16_t x = 0U; x < 320U; x++) lcd_wr_pixel(BLACK);
        }
    }
}

static void osd_show_toast(const char *msg, uint16_t color, uint32_t duration_ms)
{
    g_wind_box_drawn = 0U;
    osd_draw_center_box(msg, color);
    uint32_t until = HAL_GetTick() + duration_ms;
    g_toast_until = (until == 0U) ? 1U : until;
}

/* ======================== Always-RAW LUT / AE / AWB ======================== */
/* Calibrated OV7725 COM13=0xE6 per-channel 12-bit BlackLevel (0..4064, eliminates shadow color cast):
 *   ISO 100 (1x):  G=25, R/B=23   |   ISO 1600 (16x): G=85, R/B=91
 */
static inline uint16_t c3m1_black_level_g12(uint8_t gain_x)
{
    return (uint16_t)(21U + (uint16_t)gain_x * 4U);
}

static inline uint16_t c3m1_black_level_rb12(uint8_t gain_x)
{
    return (uint16_t)(19U + ((uint16_t)gain_x * 9U) / 2U);
}

static inline uint8_t linear12_to_srgb8(uint32_t v12)
{
    if (v12 >= 4064UL)
    {
        return 255U;
    }
    /* Interpolate s_srgb_lut[256] using low 4 fractional bits -> smooth 12-bit shadow preview */
    uint32_t idx  = v12 >> 4;
    uint32_t frac = v12 & 15UL;
    uint32_t s0   = s_srgb_lut[idx];
    uint32_t s1   = s_srgb_lut[idx + 1U];
    return (uint8_t)(s0 + (((s1 - s0) * frac + 8UL) >> 4));
}

static void raw_preview_rebuild_luts(uint16_t wb_r_q8, uint16_t wb_b_q8, uint16_t boost_q8, uint8_t gain_x)
{
    const uint32_t scale_r = ((uint32_t)wb_r_q8 * (uint32_t)boost_q8) >> 8;
    const uint32_t scale_g = (uint32_t)boost_q8;
    const uint32_t scale_b = ((uint32_t)wb_b_q8 * (uint32_t)boost_q8) >> 8;
    const uint16_t bl_g12  = c3m1_black_level_g12(gain_x);
    const uint16_t bl_rb12 = c3m1_black_level_rb12(gain_x);

    for (uint16_t i = 0; i < 256U; i++)
    {
        uint16_t v12    = s_inv_gamma_12[i];
        uint32_t lin_g  = (v12 > bl_g12)  ? (uint32_t)(v12 - bl_g12)  : 0UL;
        uint32_t lin_rb = (v12 > bl_rb12) ? (uint32_t)(v12 - bl_rb12) : 0UL;

        s_lut_r6[i] = (uint8_t)(linear12_to_srgb8((lin_rb * scale_r) >> 8) >> 2);
        s_lut_g6[i] = (uint8_t)(linear12_to_srgb8((lin_g  * scale_g) >> 8) >> 2);
        s_lut_b6[i] = (uint8_t)(linear12_to_srgb8((lin_rb * scale_b) >> 8) >> 2);
    }

    g_lut_last_r_q8     = wb_r_q8;
    g_lut_last_b_q8     = wb_b_q8;
    g_lut_last_boost_q8 = boost_q8;
    g_lut_last_gain_x   = gain_x;
}

/*
 * Map an exposure-value product (ev = aec_lines * gain_q4 @22,959 lines/s) onto the
 * configurable 4-zone anti-flicker P-curve:
 *   Zone 1 (Bright): ISO 100, shutter 1/23000..z1_aec (1/100 @50Hz, 1/120 @60Hz)
 *   Zone 2 (Indoor): diagonal ramp z1_aec..z2_aec & ISO 100..z2_gain
 *   Zone 3 (Dim):    hold z2_aec (1/50 @50Hz, 1/60 @60Hz), ramp ISO -> max_gain_q4
 *   Zone 4 (Dark):   if MIN AUTO SHT allows > z2_aec, extend shutter -> max_aec @ max_gain_q4
 */
static void ae_apply_p_curve_ev(uint32_t new_ev, uint32_t ratio_q8)
{
    uint16_t max_gain_q4 = c3m1_cfg_max_auto_gain_q4(&g_cfg);
    uint16_t max_aec     = c3m1_cfg_min_auto_aec(&g_cfg);
    uint16_t z1_aec      = (g_cfg.flicker_mode == 1U) ? 191U : 230U; /* 1/120 @60Hz, 1/100 @50Hz/OFF */
    uint16_t z2_aec      = (g_cfg.flicker_mode == 1U) ? 383U : 459U; /* 1/60  @60Hz, 1/50  @50Hz/OFF */
    if (z1_aec > max_aec) z1_aec = max_aec;
    if (z2_aec > max_aec) z2_aec = max_aec;
    uint16_t z2_gain_q4 = (max_gain_q4 < 64U) ? max_gain_q4 : 64U;

    const uint32_t ev_min = (uint32_t)OV7725_AUTO_MIN_AEC * 16UL;
    const uint32_t ev_z1  = (uint32_t)z1_aec * 16UL;
    const uint32_t ev_z2  = (uint32_t)z2_aec * (uint32_t)z2_gain_q4;
    const uint32_t ev_z3  = (uint32_t)z2_aec * (uint32_t)max_gain_q4;
    const uint32_t ev_max = (uint32_t)max_aec * (uint32_t)max_gain_q4;

    if (new_ev < ev_min) new_ev = ev_min;
    if (new_ev > ev_max) new_ev = ev_max;

    if (new_ev <= ev_z1)
    {
        g_auto_gain_q4 = 16U;
        uint16_t next_aec = (uint16_t)((new_ev + 8UL) / 16UL);
        if (next_aec == g_auto_aec)
        {
            if (ratio_q8 < 256UL && next_aec > OV7725_AUTO_MIN_AEC) next_aec--;
            else if (ratio_q8 > 256UL && next_aec < z1_aec) next_aec++;
        }
        g_auto_aec = next_aec;
    }
    else if (new_ev <= ev_z2 && ev_z2 > ev_z1)
    {
        uint32_t t_q8 = ((new_ev - ev_z1) * 256UL) / (ev_z2 - ev_z1);
        uint32_t aec_target = (uint32_t)z1_aec + (((uint32_t)(z2_aec - z1_aec) * t_q8) >> 8);
        g_auto_aec = (uint16_t)aec_target;
        g_auto_gain_q4 = (uint16_t)((new_ev + (aec_target >> 1)) / aec_target);
    }
    else if (new_ev <= ev_z3)
    {
        g_auto_aec = z2_aec;
        g_auto_gain_q4 = (uint16_t)((new_ev + (z2_aec >> 1)) / z2_aec);
    }
    else
    {
        g_auto_gain_q4 = max_gain_q4;
        g_auto_aec = (uint16_t)((new_ev + (max_gain_q4 >> 1)) / max_gain_q4);
    }
    if (g_auto_aec < OV7725_AUTO_MIN_AEC) g_auto_aec = OV7725_AUTO_MIN_AEC;
    if (g_auto_aec > max_aec)             g_auto_aec = max_aec;
    if (g_auto_gain_q4 < 16U)             g_auto_gain_q4 = 16U;
    if (g_auto_gain_q4 > max_gain_q4)     g_auto_gain_q4 = max_gain_q4;
}

/*
 * Seed AUTO shutter/ISO state from the current physical exposure when switching from a
 * manual shutter or fixed ISO into AUTO, preventing stale-AEC + pegged-ISO exposure spikes
 * (e.g., 1/8000s @ AUTO ISO 1600 -> AUTO shutter flashing white and rebounding to 1/23000s).
 */
static void ae_seed_on_mode_transition(uint8_t prev_dial, uint8_t prev_iso)
{
    uint16_t max_gain_q4 = c3m1_cfg_max_auto_gain_q4(&g_cfg);
    uint16_t max_aec     = c3m1_cfg_min_auto_aec(&g_cfg);
    uint8_t  entered_auto_s = (ov7725_is_auto_shutter(g_dial_pos) && !ov7725_is_auto_shutter(prev_dial)) ? 1U : 0U;
    uint8_t  entered_auto_i = (g_iso_idx == 0U && prev_iso != 0U) ? 1U : 0U;

    if (entered_auto_s)
    {
        uint32_t prev_aec  = ov7725_get_manual_aec_fast(prev_dial);
        uint16_t prev_gain = ov7725_get_iso_gain_q4(prev_iso, g_auto_gain_q4);
        if (g_iso_idx == 0U)
        {
            ae_apply_p_curve_ev(prev_aec * (uint32_t)prev_gain, 256UL);
        }
        else
        {
            if (prev_aec < OV7725_AUTO_MIN_AEC) prev_aec = OV7725_AUTO_MIN_AEC;
            if (prev_aec > max_aec)             prev_aec = max_aec;
            g_auto_aec = (uint16_t)prev_aec;
        }
    }
    else if (entered_auto_i)
    {
        uint16_t prev_gain = ov7725_get_iso_gain_q4(prev_iso, g_auto_gain_q4);
        if (ov7725_is_auto_shutter(g_dial_pos))
        {
            ae_apply_p_curve_ev((uint32_t)g_auto_aec * (uint32_t)prev_gain, 256UL);
        }
        else
        {
            if (prev_gain < 16U)         prev_gain = 16U;
            if (prev_gain > max_gain_q4) prev_gain = max_gain_q4;
            g_auto_gain_q4 = prev_gain;
        }
    }
}

static void raw_apply_preview_exposure(uint8_t mark_dirty)
{
    uint8_t auto_gain_reg = ov7725_gain_q4_to_reg(g_auto_gain_q4);
    uint16_t boost  = ov7725_apply_always_raw_preview(g_dial_pos, g_iso_idx, g_auto_aec, auto_gain_reg);
    uint8_t  gain_x = ov7725_get_active_gain_x();
    g_preview_boost_q8 = boost;
    if (boost != g_lut_last_boost_q8 || gain_x != g_lut_last_gain_x)
    {
        raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, boost, gain_x);
    }
    if (mark_dirty)
    {
        g_exposure_dirty_frames = 2U;
        g_ae_settle_frames      = 0U;
    }
    if (ov7725_is_auto_shutter(g_dial_pos))
    {
        char cur_s[10];
        ov7725_format_shutter_value(g_dial_pos, g_auto_aec, cur_s, sizeof(cur_s));
        if (strcmp(cur_s, g_last_auto_s_label) != 0)
        {
            snprintf(g_last_auto_s_label, sizeof(g_last_auto_s_label), "%s", cur_s);
            g_osd_dirty = 1;
        }
    }
    if (g_iso_idx == 0U)
    {
        char cur_i[8];
        ov7725_format_iso_value(g_iso_idx, g_auto_gain_q4, cur_i, sizeof(cur_i));
        if (strcmp(cur_i, g_last_auto_i_label) != 0)
        {
            snprintf(g_last_auto_i_label, sizeof(g_last_auto_i_label), "%s", cur_i);
            g_osd_dirty = 1;
        }
    }
}

static void raw_preview_update_ae_awb(uint32_t wb_sum_r, uint32_t wb_sum_g, uint32_t wb_sum_b,
                                      uint32_t wb_cnt, uint32_t ae_sum_g, uint32_t ae_cnt,
                                      uint32_t hi_cnt, const uint16_t lin_bins[16],
                                      uint32_t lin_total)
{
    /* 1. Gray-World AWB (only in AUTO WB mode; fixed Kelvin presets lock g_wb_r/b_q8) */
    if (g_cfg.wb_mode == C3M1_WB_AUTO && wb_cnt >= 128UL)
    {
        uint32_t avg_r = wb_sum_r / wb_cnt;
        uint32_t avg_g = wb_sum_g / wb_cnt;
        uint32_t avg_b = wb_sum_b / wb_cnt;
        if (avg_r >= 4UL && avg_g >= 4UL && avg_b >= 4UL)
        {
            uint32_t tr = (wb_sum_g * 256UL) / wb_sum_r;
            uint32_t tb = (wb_sum_g * 256UL) / wb_sum_b;
            if (tr < 150UL) tr = 150UL;
            if (tr > 560UL) tr = 560UL;
            if (tb < 180UL) tb = 180UL;
            if (tb > 680UL) tb = 680UL;

            g_wb_r_q8 = (uint16_t)((3UL * g_wb_r_q8 + tr + 2UL) >> 2);
            g_wb_b_q8 = (uint16_t)((3UL * g_wb_b_q8 + tb + 2UL) >> 2);

            int32_t dr = (int32_t)g_wb_r_q8 - (int32_t)g_lut_last_r_q8;
            int32_t db = (int32_t)g_wb_b_q8 - (int32_t)g_lut_last_b_q8;
            if (dr < 0) dr = -dr;
            if (db < 0) db = -db;
            if (dr >= 4 || db >= 4 || g_preview_boost_q8 != g_lut_last_boost_q8)
            {
                raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, g_preview_boost_q8, g_lut_last_gain_x);
            }

            uint16_t cur_k = wb_gains_to_kelvin(g_wb_r_q8, g_wb_b_q8);
            if (cur_k != g_last_wb_kelvin)
            {
                g_last_wb_kelvin = cur_k;
                g_osd_dirty = 1;
            }
        }
    }

    /* 2. Live exposure meter (runs in BOTH Manual and Auto modes once exposure settles) */
    if (ae_cnt > 0UL && g_exposure_dirty_frames == 0U)
    {
        uint32_t raw_eff_q4 = ((ae_sum_g * (uint32_t)g_preview_boost_q8) >> 4) / ae_cnt;
        if (raw_eff_q4 > 4080UL) raw_eff_q4 = 4080UL;
        g_meter_eff_q4 = (uint16_t)((3UL * g_meter_eff_q4 + raw_eff_q4 + 2UL) >> 2);
        int8_t new_ev = meter_eff_q4_to_ev_x10(g_meter_eff_q4);
        if (new_ev != g_meter_ev_x10)
        {
            g_meter_ev_x10 = new_ev;
            g_osd_dirty = 1;
        }
    }

    /* 3. Software closed-loop AE (waits for dirty dial/ISO frames and 1 settle frame after SCCB) */
    uint8_t auto_s = ov7725_is_auto_shutter(g_dial_pos);
    uint8_t auto_i = (g_iso_idx == 0U) ? 1U : 0U;
    if ((!auto_s && !auto_i) || ae_cnt == 0UL)
    {
        return;
    }

    if (g_exposure_dirty_frames > 0U)
    {
        return;
    }

    if (g_ae_settle_frames > 0U)
    {
        g_ae_settle_frames--;
        return;
    }

    /* Account for digital LUT boost on extreme slow shutters when ISO is AUTO (Q4 precision) */
    uint32_t eff_q4    = ((ae_sum_g * (uint32_t)g_preview_boost_q8) >> 4) / ae_cnt;
    uint32_t target_q4 = (uint32_t)c3m1_cfg_ae_target_q4(&g_cfg);
    uint32_t hi_pct    = (hi_cnt * 100UL) / ((lin_total > 0UL) ? lin_total : ae_cnt);

    uint32_t ratio_q8 = 256UL;
    if (g_cfg.meter_mode == C3M1_METER_ETTR && lin_total >= 64UL)
    {
        /*
         * ETTR (Expose To The Right): place 98th-percentile highlight at ~85% linear (3456 Q4)
         * scaled by EXP COMP; use fast hi_pct pull-down when clipped so bright jumps converge fast.
         */
        uint32_t top2_thresh = (lin_total + 25UL) / 50UL;
        uint32_t acc = 0UL;
        uint32_t p98_lin = 12UL;
        for (int32_t b = 15; b >= 0; b--)
        {
            acc += lin_bins[b];
            if (acc >= top2_thresh)
            {
                p98_lin = (uint32_t)b * 16UL + 8UL;
                break;
            }
        }
        uint32_t p98_q4 = (p98_lin * (uint32_t)g_preview_boost_q8) >> 4;
        uint32_t ettr_target_q4 = (3456UL * target_q4) / 736UL;
        if (ettr_target_q4 < 384UL)  ettr_target_q4 = 384UL;
        if (ettr_target_q4 > 3920UL) ettr_target_q4 = 3920UL;

        uint32_t dead_lo = (ettr_target_q4 * 14UL) >> 4;
        uint32_t dead_hi = (ettr_target_q4 * 18UL) >> 4;
        if (hi_pct >= 35UL)
        {
            ratio_q8 = 90UL;  /* Heavy clipping -> fast -1.5 EV pull-down */
        }
        else if (hi_pct >= 12UL)
        {
            ratio_q8 = 128UL; /* Moderate clipping -> fast -1.0 EV pull-down */
        }
        else if (hi_pct >= 4UL && p98_q4 > dead_hi)
        {
            ratio_q8 = 180UL; /* Light clipping -> crisp -0.5 EV pull-down */
        }
        else if (p98_q4 >= dead_lo && p98_q4 <= dead_hi)
        {
            return;
        }
        else
        {
            uint32_t denom = (p98_q4 < 48UL) ? 48UL : p98_q4;
            uint32_t raw_ratio = (ettr_target_q4 * 256UL) / denom;
            if (p98_q4 < (ettr_target_q4 >> 1) || p98_q4 > ((ettr_target_q4 * 28UL) >> 4))
            {
                ratio_q8 = (256UL + 6UL * raw_ratio) / 7UL;
                if (ratio_q8 < 64UL)   ratio_q8 = 64UL;
                if (ratio_q8 > 1024UL) ratio_q8 = 1024UL;
            }
            else
            {
                ratio_q8 = (256UL + 2UL * raw_ratio) / 3UL;
                if (ratio_q8 < 144UL) ratio_q8 = 144UL;
                if (ratio_q8 > 448UL) ratio_q8 = 448UL;
            }
        }
    }
    else
    {
        /*
         * Fast dual-speed mirrorless AE (STD multi-zone or CENTER 3x weighted):
         *   - Large error (< 0.5x or > 1.75x target): 85% slew [0.25x..4.0x] (+-2.0 EV/step)
         *   - Moderate error: 67% damping [0.56x..1.75x] (+-0.8 EV/step)
         *   - Deadband +-12.5% [14/16..18/16] (~+-0.18 EV < 1/3 EV step)
         */
        uint32_t fast_lo  = (target_q4 * 8UL) >> 4;
        uint32_t fast_hi  = (target_q4 * 28UL) >> 4;
        uint32_t dead_lo  = (target_q4 * 14UL) >> 4;
        uint32_t dead_hi  = (target_q4 * 18UL) >> 4;
        if (eff_q4 < fast_lo || eff_q4 > fast_hi)
        {
            uint32_t denom = (eff_q4 < 24UL) ? 24UL : eff_q4;
            uint32_t raw_ratio = (target_q4 * 256UL) / denom;
            ratio_q8 = (256UL + 6UL * raw_ratio) / 7UL;
            if (ratio_q8 < 64UL)   ratio_q8 = 64UL;   /* 0.25x min (-2.0 EV/step) */
            if (ratio_q8 > 1024UL) ratio_q8 = 1024UL; /* 4.00x max (+2.0 EV/step) */
        }
        else if (g_cfg.meter_mode == C3M1_METER_STD && g_cfg.ev_comp <= C3M1_EV_COMP_ZERO && hi_pct >= 12UL && eff_q4 > ((target_q4 * 10UL) >> 4))
        {
            ratio_q8 = (hi_pct >= 30UL) ? 144UL : 192UL;
        }
        else if (g_cfg.meter_mode == C3M1_METER_STD && g_cfg.ev_comp <= C3M1_EV_COMP_ZERO && hi_pct >= 8UL && eff_q4 >= ((target_q4 * 8UL) >> 4))
        {
            return;
        }
        else if (eff_q4 >= dead_lo && eff_q4 <= dead_hi)
        {
            return;
        }
        else
        {
            uint32_t denom = (eff_q4 < 16UL) ? 16UL : eff_q4;
            uint32_t raw_ratio = (target_q4 * 256UL) / denom;
            ratio_q8 = (256UL + 2UL * raw_ratio) / 3UL;
            if (ratio_q8 < 144UL) ratio_q8 = 144UL; /* 0.56x min (-0.8 EV/step) */
            if (ratio_q8 > 448UL) ratio_q8 = 448UL; /* 1.75x max (+0.8 EV/step) */
        }
    }

    uint16_t old_aec = g_auto_aec;
    uint8_t  old_gain_reg = ov7725_gain_q4_to_reg(g_auto_gain_q4);
    uint16_t max_gain_q4 = c3m1_cfg_max_auto_gain_q4(&g_cfg);
    uint16_t max_aec     = c3m1_cfg_min_auto_aec(&g_cfg);

    if (auto_s && auto_i)
    {
        uint32_t ev     = (uint32_t)g_auto_aec * (uint32_t)g_auto_gain_q4;
        uint32_t new_ev = (ev * ratio_q8 + 128UL) >> 8;
        ae_apply_p_curve_ev(new_ev, ratio_q8);
    }
    else if (auto_s)
    {
        uint32_t new_aec = ((uint32_t)g_auto_aec * ratio_q8 + 128UL) >> 8;
        if (new_aec == g_auto_aec)
        {
            if (ratio_q8 > 256UL) new_aec++;
            else if (ratio_q8 < 256UL && new_aec > OV7725_AUTO_MIN_AEC) new_aec--;
        }
        if (new_aec < OV7725_AUTO_MIN_AEC) new_aec = OV7725_AUTO_MIN_AEC;
        if (new_aec > max_aec)             new_aec = max_aec;
        g_auto_aec = (uint16_t)new_aec;
    }
    else
    {
        uint32_t new_gain = ((uint32_t)g_auto_gain_q4 * ratio_q8 + 128UL) >> 8;
        if (new_gain == g_auto_gain_q4)
        {
            if (ratio_q8 > 256UL) new_gain++;
            else if (ratio_q8 < 256UL && new_gain > 16UL) new_gain--;
        }
        if (new_gain < 16UL)        new_gain = 16UL;
        if (new_gain > max_gain_q4) new_gain = max_gain_q4;
        g_auto_gain_q4 = (uint16_t)new_gain;
    }

    if (g_auto_aec != old_aec || ov7725_gain_q4_to_reg(g_auto_gain_q4) != old_gain_reg)
    {
        /* Defer SCCB write to VBLANK right after next VSYNC; skip 1 frame for sensor settle */
        g_pending_ae_apply = 1U;
        g_ae_settle_frames = 1U;
    }
}

/* ======================== DMA1 FSMC Pipelining & Viewfinder Refresh ======================== */
static void lcd_dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    DMA1_Channel1->CCR  = 0U;
    DMA1_Channel1->CPAR = (uint32_t)&LCD->LCD_RAM;
}

static inline void lcd_dma_wait(void)
{
    if (DMA1_Channel1->CCR & DMA_CCR_EN)
    {
        while ((DMA1->ISR & DMA_ISR_TCIF1) == 0U)
        {
        }
        DMA1_Channel1->CCR = 0U;
    }
}

static inline void lcd_dma_send(const uint8_t *src, uint16_t len)
{
    lcd_dma_wait();
    DMA1->IFCR           = DMA_IFCR_CGIF1;
    DMA1_Channel1->CMAR  = (uint32_t)src;
    DMA1_Channel1->CNDTR = len;
    /* MEM2MEM=1, DIR=1 (CMAR->CPAR), MINC=1, PINC=0, 8-bit, HIGH priority */
    DMA1_Channel1->CCR   = DMA_CCR_MEM2MEM | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_EN;
}

static void fifo_prepare_read(void)
{
    /*
     * Strict write-then-read: always clear WEN=0 and read only after g_vsync_state==2.
     * Hardware testing on C3M1 confirmed that AL422B simultaneous Read-While-Write
     * (WCK=18 MHz + RCLK + FSMC DMA) causes occasional DRAM write/pointer glitches
     * (single-frame blue color-shift patches / lower-screen shifts).
     */
    OV7725_WEN(0);
    OV7725_OE(0);
    __NOP();

    OV7725_RRST(0);
    OV7725_RCLK(0);
    OV7725_RCLK(1);
    OV7725_RCLK(0);
    OV7725_RRST(1);
    OV7725_RCLK(1);
}

/*
 * AL422B RCLK spec: t_RL >= 7ns, t_RH >= 7ns, t_RC >= 20ns (50 MHz max).
 * At 72 MHz HCLK, consecutive APB2 STR to BRR/BSRR takes >= 2 HCLK (27.8ns) each
 * (~18 MHz, 4x timing margin, verified clean on C3M1 hardware in Experiment 4).
 */
#define FIFO_SKIP_1(port, pin) do { \
    (port)->BRR  = (pin);           \
    (port)->BSRR = (pin);           \
} while (0)

static inline __attribute__((always_inline)) void fifo_skip_4(void)
{
    GPIO_TypeDef *const port = OV7725_RCLK_GPIO_Port;
    const uint32_t      pin  = OV7725_RCLK_Pin;
    FIFO_SKIP_1(port, pin);
    FIFO_SKIP_1(port, pin);
    FIFO_SKIP_1(port, pin);
    FIFO_SKIP_1(port, pin);
}

static void __attribute__((optimize("O2"))) fifo_skip_bytes(uint32_t n)
{
    GPIO_TypeDef *const port = OV7725_RCLK_GPIO_Port;
    const uint32_t      pin  = OV7725_RCLK_Pin;
    uint32_t blocks = n >> 4;
    while (blocks--)
    {
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
    }
    uint32_t rem = n & 15UL;
    while (rem--)
    {
        FIFO_SKIP_1(port, pin);
    }
}

static void __attribute__((optimize("O3"))) fifo_read_bytes(uint8_t *dst, uint16_t count)
{
    GPIO_TypeDef *const rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t      rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const data_port = GPIOG;

    if ((((uintptr_t)dst) & 3U) == 0U)
    {
        uint32_t *dst32 = (uint32_t *)dst;
        uint32_t  words = (uint32_t)count >> 2;
        while (words--)
        {
            rclk_port->BRR  = rclk_pin;
            uint32_t b0     = (data_port->IDR >> 1) & 0xFFUL;
            rclk_port->BSRR = rclk_pin;

            rclk_port->BRR  = rclk_pin;
            uint32_t b1     = (data_port->IDR >> 1) & 0xFFUL;
            rclk_port->BSRR = rclk_pin;

            rclk_port->BRR  = rclk_pin;
            uint32_t b2     = (data_port->IDR >> 1) & 0xFFUL;
            rclk_port->BSRR = rclk_pin;

            rclk_port->BRR  = rclk_pin;
            uint32_t b3     = (data_port->IDR >> 1) & 0xFFUL;
            rclk_port->BSRR = rclk_pin;

            *dst32++ = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
        }
        dst    = (uint8_t *)dst32;
        count &= 3U;
    }

    while (count--)
    {
        rclk_port->BRR  = rclk_pin;
        uint8_t px      = (uint8_t)(data_port->IDR >> 1);
        rclk_port->BSRR = rclk_pin;
        *dst++ = px;
    }
}

/* Read only the 320 R bytes from a 640-byte even GRBG row (4x unrolled, 32-bit packed store) */
static void __attribute__((optimize("O3"))) fifo_read_r_320(uint8_t *dst)
{
    GPIO_TypeDef *const rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t      rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const data_port = GPIOG;
    uint32_t           *dst32     = (uint32_t *)dst;
    uint32_t            n         = 80U; /* 80 x 4 = 320 R bytes */

    do
    {
        rclk_port->BRR  = rclk_pin;
        rclk_port->BSRR = rclk_pin;
        rclk_port->BRR  = rclk_pin;
        uint32_t r0     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;

        rclk_port->BRR  = rclk_pin;
        rclk_port->BSRR = rclk_pin;
        rclk_port->BRR  = rclk_pin;
        uint32_t r1     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;

        rclk_port->BRR  = rclk_pin;
        rclk_port->BSRR = rclk_pin;
        rclk_port->BRR  = rclk_pin;
        uint32_t r2     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;

        rclk_port->BRR  = rclk_pin;
        rclk_port->BSRR = rclk_pin;
        rclk_port->BRR  = rclk_pin;
        uint32_t r3     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;

        *dst32++ = r0 | (r1 << 8) | (r2 << 16) | (r3 << 24);
    } while (--n != 0U);
}

/* Single-cycle Cortex-M3 fused arithmetic-right-shift + unsigned saturate: USAT Rd, #sat, Rn, ASR #shift */
#define C3M1_USAT_ASR(val, sat, shift) __extension__ ({ \
    uint32_t _res; \
    __asm__("usat %0, #" #sat ", %1, asr #" #shift : "=r"(_res) : "r"((int32_t)(val))); \
    _res; \
})

/* Strictly neutral RGB565 (R5 == G6/2 == B5, zero green cast) viewfinder B&W S-curve LUTs in Flash (.rodata) */
static const uint16_t s_vf_bw_rgb565_le[3][64] = {
    {
        0x0000U, 0x0000U, 0x0000U, 0x0841U, 0x0841U, 0x1082U, 0x1082U, 0x1082U,
        0x18C3U, 0x18C3U, 0x2104U, 0x2104U, 0x2945U, 0x2945U, 0x3186U, 0x3186U,
        0x39C7U, 0x39C7U, 0x4208U, 0x4A49U, 0x4A49U, 0x528AU, 0x528AU, 0x5ACBU,
        0x5ACBU, 0x630CU, 0x6B4DU, 0x6B4DU, 0x738EU, 0x738EU, 0x7BCFU, 0x8410U,
        0x8410U, 0x8C51U, 0x8C51U, 0x9492U, 0x9CD3U, 0x9CD3U, 0xA514U, 0xA514U,
        0xAD55U, 0xB596U, 0xB596U, 0xBDD7U, 0xBDD7U, 0xC618U, 0xC618U, 0xCE59U,
        0xCE59U, 0xD69AU, 0xD69AU, 0xDEDBU, 0xDEDBU, 0xE71CU, 0xE71CU, 0xEF5DU,
        0xEF5DU, 0xF79EU, 0xF79EU, 0xF79EU, 0xFFDFU, 0xFFDFU, 0xFFDFU, 0xFFDFU,
    },
    {
        0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0841U,
        0x0841U, 0x0841U, 0x1082U, 0x1082U, 0x1082U, 0x18C3U, 0x18C3U, 0x2104U,
        0x2104U, 0x2945U, 0x2945U, 0x3186U, 0x39C7U, 0x39C7U, 0x4208U, 0x4A49U,
        0x528AU, 0x528AU, 0x5ACBU, 0x630CU, 0x6B4DU, 0x738EU, 0x738EU, 0x7BCFU,
        0x8410U, 0x8C51U, 0x9492U, 0x9CD3U, 0xA514U, 0xA514U, 0xAD55U, 0xB596U,
        0xBDD7U, 0xBDD7U, 0xC618U, 0xCE59U, 0xCE59U, 0xD69AU, 0xD69AU, 0xDEDBU,
        0xE71CU, 0xE71CU, 0xE71CU, 0xEF5DU, 0xEF5DU, 0xF79EU, 0xF79EU, 0xF79EU,
        0xF79EU, 0xFFDFU, 0xFFDFU, 0xFFDFU, 0xFFDFU, 0xFFDFU, 0xFFDFU, 0xFFDFU,
    },
    {
        0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0841U, 0x0841U, 0x0841U, 0x1082U,
        0x1082U, 0x18C3U, 0x18C3U, 0x2104U, 0x2104U, 0x2945U, 0x2945U, 0x3186U,
        0x3186U, 0x39C7U, 0x39C7U, 0x4208U, 0x4A49U, 0x4A49U, 0x528AU, 0x528AU,
        0x5ACBU, 0x630CU, 0x630CU, 0x6B4DU, 0x738EU, 0x738EU, 0x7BCFU, 0x7BCFU,
        0x8410U, 0x8C51U, 0x9492U, 0x9492U, 0x9CD3U, 0xA514U, 0xA514U, 0xAD55U,
        0xAD55U, 0xB596U, 0xBDD7U, 0xBDD7U, 0xC618U, 0xC618U, 0xCE59U, 0xCE59U,
        0xD69AU, 0xDEDBU, 0xDEDBU, 0xDEDBU, 0xE71CU, 0xE71CU, 0xEF5DU, 0xEF5DU,
        0xF79EU, 0xF79EU, 0xF79EU, 0xFFDFU, 0xFFDFU, 0xFFDFU, 0xFFDFU, 0xFFDFU,
    },
};

/* 64-step radial sRGB vignette gain LUT (Q8: 256=1.00x .. 400=1.56x) + 20-span horizontal dx^2 */
static uint16_t s_vf_vig_lut[65];
static uint8_t  s_vf_vig_lut_val = 0xFFU;
static const uint16_t s_vf_vig_dx2[20] = {
    23104U, 18496U, 14400U, 10816U, 7744U, 5184U, 3136U, 1600U, 576U, 64U,
       64U,   576U,  1600U,  3136U, 5184U, 7744U, 10816U, 14400U, 18496U, 23104U
};

static inline __attribute__((always_inline)) uint32_t vf_bw_pack_rgb565(
    const uint16_t *bwlut, uint8_t is_red, uint32_t r6, uint32_t g6, uint32_t b6)
{
    uint32_t y6 = is_red ? (((r6 << 1) + r6 + g6) >> 2)
                         : ((r6 + (g6 << 1) + b6) >> 2);
    return (uint32_t)bwlut[y6];
}

static inline __attribute__((always_inline)) uint32_t vf_bw_vig_pack_rgb565(
    const uint16_t *bwlut, uint8_t is_red, uint32_t r6, uint32_t g6, uint32_t b6, int32_t g_q8)
{
    uint32_t y6 = is_red ? (((r6 << 1) + r6 + g6) >> 2)
                         : ((r6 + (g6 << 1) + b6) >> 2);
    uint32_t yv = C3M1_USAT_ASR((int32_t)(y6 * (uint32_t)g_q8 + 128UL), 6, 8);
    return (uint32_t)bwlut[yv];
}

/*
 * Fast Cortex-M3 1.75x sRGB chroma expansion (bit-exact algebraic form, zero mul/div/literal load):
 *   c_out = c + 3/4 * (c - y) = (7*c - 3*y) / 4, directly fused into 5/6/5-bit USAT+ASR.
 * Compensates OV7725 Bayer filter crosstalk (~35% spectral overlap) while keeping neutral grays invariant.
 */
static inline __attribute__((always_inline)) uint32_t vf_chroma_pack_rgb565(int32_t r6, int32_t g6, int32_t b6)
{
    int32_t  y  = (r6 + (g6 << 1) + b6) >> 2;
    int32_t  y3 = y + (y << 1);
    uint32_t ro = C3M1_USAT_ASR(((r6 << 3) - r6) - y3, 5, 3);
    uint32_t go = C3M1_USAT_ASR(((g6 << 3) - g6) - y3, 6, 2);
    uint32_t bo = C3M1_USAT_ASR(((b6 << 3) - b6) - y3, 5, 3);
    return (ro << 11) | (go << 5) | bo;
}

/*
 * Zero-overhead fused 1.75x chroma + radial vignette gain (Q8):
 *   a_q8 = 7 * g_q8, b_q8 = 3 * g_q8 -> (c*a_q8 - y*b_q8) >> (11/10) with single-cycle USAT+ASR.
 * Same 13 cycles/pixel as vf_chroma_pack_rgb565 (0 extra cycles vs vignette OFF!) and bit-exact at g_q8=256.
 */
static inline __attribute__((always_inline)) uint32_t vf_chroma_vig_pack_rgb565(
    int32_t r6, int32_t g6, int32_t b6, int32_t a_q8, int32_t b_q8)
{
    int32_t  y  = (r6 + (g6 << 1) + b6) >> 2;
    int32_t  yb = y * b_q8;
    uint32_t ro = C3M1_USAT_ASR(r6 * a_q8 - yb, 5, 11);
    uint32_t go = C3M1_USAT_ASR(g6 * a_q8 - yb, 6, 10);
    uint32_t bo = C3M1_USAT_ASR(b6 * a_q8 - yb, 5, 11);
    return (ro << 11) | (go << 5) | bo;
}

/* Demosaic 320 odd-row (B, G1) pairs + R-row into LE warp ring with fused 64-step vignette */
static void __attribute__((optimize("O3"), noinline)) raw_demosaic_320_to_ring_vig(
    const uint8_t *r_row, uint8_t *dst, uint16_t yp)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint8_t *const  lut       = s_lut_rgb6[0];
    const uint16_t       *r16       = (const uint16_t *)r_row;
    uint32_t             *dst32     = (uint32_t *)dst;
    int32_t               dy        = (int32_t)yp * 2L - 120L;
    uint32_t              dy2       = (uint32_t)(dy * dy);
    const uint8_t         cm        = g_cfg.color_mode;

    if (cm != C3M1_COLOR_COLOR)
    {
        const uint16_t *const bwlut  = s_vf_bw_rgb565_le[cm - 1U];
        const uint8_t         is_red = (cm == C3M1_COLOR_BW_RED) ? 1U : 0U;
        for (uint32_t bx = 0U; bx < 20U; bx++)
        {
            uint32_t idx  = (((uint32_t)s_vf_vig_dx2[bx] + dy2) * 105UL) >> 16;
            if (idx > 63UL) idx = 63UL;
            int32_t  g_q8 = (int32_t)s_vf_vig_lut[idx];
            uint32_t k    = 8U;
            #pragma GCC unroll 1
            do
            {
                uint32_t r01 = *r16++;
                rclk_port->BRR  = rclk_pin;
                uint32_t idr_b0 = data_port->IDR;
                rclk_port->BSRR = rclk_pin;
                uint32_t r0_6   = lut[r01 & 0xFFU];
                rclk_port->BRR  = rclk_pin;
                uint32_t idr_g0 = data_port->IDR;
                rclk_port->BSRR = rclk_pin;
                uint32_t g0_6   = lut[((idr_g0 >> 1) & 0xFFU) | 0x100U];
                uint32_t b0_6   = lut[((idr_b0 >> 1) & 0xFFU) | 0x200U];
                uint32_t px0    = vf_bw_vig_pack_rgb565(bwlut, is_red, r0_6, g0_6, b0_6, g_q8);
                __asm__ volatile("" : "+r"(px0));

                rclk_port->BRR  = rclk_pin;
                uint32_t idr_b1 = data_port->IDR;
                rclk_port->BSRR = rclk_pin;
                uint32_t r1_6   = lut[r01 >> 8];
                rclk_port->BRR  = rclk_pin;
                uint32_t idr_g1 = data_port->IDR;
                rclk_port->BSRR = rclk_pin;
                uint32_t g1_6   = lut[((idr_g1 >> 1) & 0xFFU) | 0x100U];
                uint32_t b1_6   = lut[((idr_b1 >> 1) & 0xFFU) | 0x200U];
                uint32_t px1    = vf_bw_vig_pack_rgb565(bwlut, is_red, r1_6, g1_6, b1_6, g_q8);

                *dst32++ = px0 | (px1 << 16);
            } while (--k != 0U);
        }
        return;
    }

    for (uint32_t bx = 0U; bx < 20U; bx++)
    {
        uint32_t idx  = (((uint32_t)s_vf_vig_dx2[bx] + dy2) * 105UL) >> 16;
        if (idx > 63UL) idx = 63UL;
        int32_t  g_q8 = (int32_t)s_vf_vig_lut[idx];
        int32_t  a_q8 = (g_q8 << 3) - g_q8;
        int32_t  b_q8 = (g_q8 << 1) + g_q8;
        uint32_t k    = 8U;
        #pragma GCC unroll 1
        do
        {
            uint32_t r01 = *r16++;
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_b0 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t r0_6   = lut[r01 & 0xFFU];
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_g0 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t g0_6   = lut[((idr_g0 >> 1) & 0xFFU) | 0x100U];
            uint32_t b0_6   = lut[((idr_b0 >> 1) & 0xFFU) | 0x200U];
            uint32_t px0    = vf_chroma_vig_pack_rgb565((int32_t)r0_6, (int32_t)g0_6, (int32_t)b0_6, a_q8, b_q8);
            __asm__ volatile("" : "+r"(px0));

            rclk_port->BRR  = rclk_pin;
            uint32_t idr_b1 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t r1_6   = lut[r01 >> 8];
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_g1 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t g1_6   = lut[((idr_g1 >> 1) & 0xFFU) | 0x100U];
            uint32_t b1_6   = lut[((idr_b1 >> 1) & 0xFFU) | 0x200U];
            uint32_t px1    = vf_chroma_vig_pack_rgb565((int32_t)r1_6, (int32_t)g1_6, (int32_t)b1_6, a_q8, b_q8);

            *dst32++ = px0 | (px1 << 16);
        } while (--k != 0U);
    }
}

/* Demosaic 320 odd-row (B, G1) pairs + R-row into BE RGB565 buffer + G6 row with fused 64-step vignette */
static void __attribute__((optimize("O3"), noinline)) raw_demosaic_320_to_buf_vig(
    const uint8_t *r_row, uint8_t *dst, uint8_t *g_dst, uint16_t yp)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint8_t *const  lut       = s_lut_rgb6[0];
    const uint16_t       *r16       = (const uint16_t *)r_row;
    uint32_t             *dst32     = (uint32_t *)dst;
    uint16_t             *g16       = (uint16_t *)g_dst;
    int32_t               dy        = (int32_t)yp * 2L - 120L;
    uint32_t              dy2       = (uint32_t)(dy * dy);
    const uint8_t         cm        = g_cfg.color_mode;

    if (cm != C3M1_COLOR_COLOR)
    {
        const uint16_t *const bwlut  = s_vf_bw_rgb565_le[cm - 1U];
        const uint8_t         is_red = (cm == C3M1_COLOR_BW_RED) ? 1U : 0U;
        for (uint32_t bx = 0U; bx < 20U; bx++)
        {
            uint32_t idx  = (((uint32_t)s_vf_vig_dx2[bx] + dy2) * 105UL) >> 16;
            if (idx > 63UL) idx = 63UL;
            int32_t  g_q8 = (int32_t)s_vf_vig_lut[idx];
            uint32_t k    = 8U;
            #pragma GCC unroll 1
            do
            {
                uint32_t r01 = *r16++;
                rclk_port->BRR  = rclk_pin;
                uint32_t idr_b0 = data_port->IDR;
                rclk_port->BSRR = rclk_pin;
                uint32_t r0_6   = lut[r01 & 0xFFU];
                rclk_port->BRR  = rclk_pin;
                uint32_t idr_g0 = data_port->IDR;
                rclk_port->BSRR = rclk_pin;
                uint32_t g0_6   = lut[((idr_g0 >> 1) & 0xFFU) | 0x100U];
                uint32_t b0_6   = lut[((idr_b0 >> 1) & 0xFFU) | 0x200U];
                uint32_t px0    = vf_bw_vig_pack_rgb565(bwlut, is_red, r0_6, g0_6, b0_6, g_q8);
                __asm__ volatile("" : "+r"(px0), "+r"(g0_6));

                rclk_port->BRR  = rclk_pin;
                uint32_t idr_b1 = data_port->IDR;
                rclk_port->BSRR = rclk_pin;
                uint32_t r1_6   = lut[r01 >> 8];
                rclk_port->BRR  = rclk_pin;
                uint32_t idr_g1 = data_port->IDR;
                rclk_port->BSRR = rclk_pin;
                uint32_t g1_6   = lut[((idr_g1 >> 1) & 0xFFU) | 0x100U];
                uint32_t b1_6   = lut[((idr_b1 >> 1) & 0xFFU) | 0x200U];
                uint32_t px1    = vf_bw_vig_pack_rgb565(bwlut, is_red, r1_6, g1_6, b1_6, g_q8);

                *g16++   = (uint16_t)(g0_6 | (g1_6 << 8));
                *dst32++ = __REV16(px0 | (px1 << 16));
            } while (--k != 0U);
        }
        return;
    }

    for (uint32_t bx = 0U; bx < 20U; bx++)
    {
        uint32_t idx  = (((uint32_t)s_vf_vig_dx2[bx] + dy2) * 105UL) >> 16;
        if (idx > 63UL) idx = 63UL;
        int32_t  g_q8 = (int32_t)s_vf_vig_lut[idx];
        int32_t  a_q8 = (g_q8 << 3) - g_q8;
        int32_t  b_q8 = (g_q8 << 1) + g_q8;
        uint32_t k    = 8U;
        #pragma GCC unroll 1
        do
        {
            uint32_t r01 = *r16++;
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_b0 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t r0_6   = lut[r01 & 0xFFU];
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_g0 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t g0_6   = lut[((idr_g0 >> 1) & 0xFFU) | 0x100U];
            uint32_t b0_6   = lut[((idr_b0 >> 1) & 0xFFU) | 0x200U];
            uint32_t px0    = vf_chroma_vig_pack_rgb565((int32_t)r0_6, (int32_t)g0_6, (int32_t)b0_6, a_q8, b_q8);
            __asm__ volatile("" : "+r"(px0), "+r"(g0_6));

            rclk_port->BRR  = rclk_pin;
            uint32_t idr_b1 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t r1_6   = lut[r01 >> 8];
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_g1 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t g1_6   = lut[((idr_g1 >> 1) & 0xFFU) | 0x100U];
            uint32_t b1_6   = lut[((idr_b1 >> 1) & 0xFFU) | 0x200U];
            uint32_t px1    = vf_chroma_vig_pack_rgb565((int32_t)r1_6, (int32_t)g1_6, (int32_t)b1_6, a_q8, b_q8);

            *g16++   = (uint16_t)(g0_6 | (g1_6 << 8));
            *dst32++ = __REV16(px0 | (px1 << 16));
        } while (--k != 0U);
    }
}

/* Demosaic 320 odd-row (B, G1) pairs + R-row into little-endian RGB565 warp ring (zero g_dst, zero REV16) */
static void __attribute__((optimize("O3"))) raw_demosaic_320_to_ring(const uint8_t *r_row,
                                                                     uint8_t *dst)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint8_t *const  lut       = s_lut_rgb6[0];
    const uint16_t       *r16       = (const uint16_t *)r_row;
    uint32_t             *dst32     = (uint32_t *)dst;
    uint32_t              n         = 160U;
    const uint8_t         cm        = g_cfg.color_mode;

    if (cm != C3M1_COLOR_COLOR)
    {
        const uint16_t *const bwlut  = s_vf_bw_rgb565_le[cm - 1U];
        const uint8_t         is_red = (cm == C3M1_COLOR_BW_RED) ? 1U : 0U;
        do
        {
            uint32_t r01 = *r16++;
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_b0 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t r0_6   = lut[r01 & 0xFFU];
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_g0 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t g0_6   = lut[((idr_g0 >> 1) & 0xFFU) | 0x100U];
            uint32_t b0_6   = lut[((idr_b0 >> 1) & 0xFFU) | 0x200U];
            uint32_t px0    = vf_bw_pack_rgb565(bwlut, is_red, r0_6, g0_6, b0_6);
            __asm__ volatile("" : "+r"(px0));

            rclk_port->BRR  = rclk_pin;
            uint32_t idr_b1 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t r1_6   = lut[r01 >> 8];
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_g1 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t g1_6   = lut[((idr_g1 >> 1) & 0xFFU) | 0x100U];
            uint32_t b1_6   = lut[((idr_b1 >> 1) & 0xFFU) | 0x200U];
            uint32_t px1    = vf_bw_pack_rgb565(bwlut, is_red, r1_6, g1_6, b1_6);
            *dst32++        = px0 | (px1 << 16);
        } while (--n != 0U);
        return;
    }

    do
    {
        uint32_t r01 = *r16++;

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_b0 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint32_t r0_6 = lut[r01 & 0xFFU];

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_g0 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint32_t g0_6 = lut[((idr_g0 >> 1) & 0xFFU) | 0x100U];
        uint32_t b0_6 = lut[((idr_b0 >> 1) & 0xFFU) | 0x200U];
        uint32_t px0  = vf_chroma_pack_rgb565((int32_t)r0_6, (int32_t)g0_6, (int32_t)b0_6);
        __asm__ volatile("" : "+r"(px0));

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_b1 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint32_t r1_6 = lut[r01 >> 8];

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_g1 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint32_t g1_6 = lut[((idr_g1 >> 1) & 0xFFU) | 0x100U];
        uint32_t b1_6 = lut[((idr_b1 >> 1) & 0xFFU) | 0x200U];
        uint32_t px1  = vf_chroma_pack_rgb565((int32_t)r1_6, (int32_t)g1_6, (int32_t)b1_6);

        *dst32++ = px0 | (px1 << 16);
    } while (--n != 0U);
}

/* Demosaic 320 odd-row (B, G1) pairs + R-row into big-endian RGB565 SRAM buffer + G6 row (2x unrolled, single-base LUT, zero spills) */
static void __attribute__((optimize("O3"))) raw_demosaic_320_to_buf(const uint8_t *r_row,
                                                                    uint8_t *dst,
                                                                    uint8_t *g_dst)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint8_t *const  lut       = s_lut_rgb6[0]; /* [0..255]=R6, [256..511]=G6, [512..767]=B6 */
    const uint16_t       *r16       = (const uint16_t *)r_row;
    uint32_t             *dst32     = (uint32_t *)dst;
    uint16_t             *g16       = (uint16_t *)g_dst;
    uint32_t              n         = 160U;
    const uint8_t         cm        = g_cfg.color_mode;

    if (cm != C3M1_COLOR_COLOR)
    {
        const uint16_t *const bwlut  = s_vf_bw_rgb565_le[cm - 1U];
        const uint8_t         is_red = (cm == C3M1_COLOR_BW_RED) ? 1U : 0U;
        do
        {
            uint32_t r01 = *r16++;
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_b0 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t r0_6   = lut[r01 & 0xFFU];
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_g0 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t g0_6   = lut[((idr_g0 >> 1) & 0xFFU) | 0x100U];
            uint32_t b0_6   = lut[((idr_b0 >> 1) & 0xFFU) | 0x200U];
            uint32_t px0    = vf_bw_pack_rgb565(bwlut, is_red, r0_6, g0_6, b0_6);
            __asm__ volatile("" : "+r"(px0), "+r"(g0_6));

            rclk_port->BRR  = rclk_pin;
            uint32_t idr_b1 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t r1_6   = lut[r01 >> 8];
            rclk_port->BRR  = rclk_pin;
            uint32_t idr_g1 = data_port->IDR;
            rclk_port->BSRR = rclk_pin;
            uint32_t g1_6   = lut[((idr_g1 >> 1) & 0xFFU) | 0x100U];
            uint32_t b1_6   = lut[((idr_b1 >> 1) & 0xFFU) | 0x200U];
            uint32_t px1    = vf_bw_pack_rgb565(bwlut, is_red, r1_6, g1_6, b1_6);

            *g16++   = (uint16_t)(g0_6 | (g1_6 << 8));
            *dst32++ = __REV16(px0 | (px1 << 16));
        } while (--n != 0U);
        return;
    }

    do
    {
        uint32_t r01 = *r16++;

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_b0 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint32_t r0_6 = lut[r01 & 0xFFU];

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_g0 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint32_t g0_6 = lut[((idr_g0 >> 1) & 0xFFU) | 0x100U];
        uint32_t b0_6 = lut[((idr_b0 >> 1) & 0xFFU) | 0x200U];
        uint32_t px0  = vf_chroma_pack_rgb565((int32_t)r0_6, (int32_t)g0_6, (int32_t)b0_6);
        __asm__ volatile("" : "+r"(px0), "+r"(g0_6));

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_b1 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint32_t r1_6 = lut[r01 >> 8];

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_g1 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint32_t g1_6 = lut[((idr_g1 >> 1) & 0xFFU) | 0x100U];
        uint32_t b1_6 = lut[((idr_b1 >> 1) & 0xFFU) | 0x200U];
        uint32_t px1  = vf_chroma_pack_rgb565((int32_t)r1_6, (int32_t)g1_6, (int32_t)b1_6);

        *g16++   = (uint16_t)(g0_6 | (g1_6 << 8));
        *dst32++ = __REV16(px0 | (px1 << 16));
    } while (--n != 0U);
}

/* Same as above, also sampling every 4th pixel for AE/AWB/ETTR/CENTER stats (4x unrolled, single-base LUT, optional fused vignette) */
static void __attribute__((optimize("O3"))) raw_demosaic_320_sampled(
    const uint8_t *r_row, uint8_t *dst, uint8_t *g_dst, uint8_t is_le_ring,
    int16_t vig_yp, uint8_t in_center_y, uint8_t center_weight,
    uint32_t *wb_sum_r, uint32_t *wb_sum_g, uint32_t *wb_sum_b, uint32_t *wb_cnt,
    uint32_t *ae_sum_g, uint32_t *ae_cnt, uint32_t *hi_cnt,
    uint16_t lin_bins[16], uint32_t *lin_total)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint8_t *const  lut       = s_lut_rgb6[0];
    const uint16_t        bl_g      = c3m1_black_level_g12(g_lut_last_gain_x);
    const uint16_t        bl_rb     = c3m1_black_level_rb12(g_lut_last_gain_x);
    const uint32_t       *r32       = (const uint32_t *)r_row;
    uint32_t             *dst32     = (uint32_t *)dst;
    uint32_t             *g32       = (uint32_t *)g_dst;
    const uint8_t         cw_row    = (center_weight && in_center_y) ? 1U : 0U;
    const uint8_t         cm        = g_cfg.color_mode;
    const uint16_t *const bwlut     = (cm != C3M1_COLOR_COLOR) ? s_vf_bw_rgb565_le[cm - 1U] : NULL;
    const uint8_t         is_red    = (cm == C3M1_COLOR_BW_RED) ? 1U : 0U;
    uint32_t              dy2       = 0UL;
    if (vig_yp >= 0)
    {
        int32_t dy = (int32_t)vig_yp * 2L - 120L;
        dy2 = (uint32_t)(dy * dy);
    }
    int32_t g_q8 = 256L;
    int32_t a_q8 = 1792L;
    int32_t b_q8 = 768L;

    uint32_t sr = 0, sg = 0, sb = 0, wc = 0, ag = 0, ac = 0, hc = 0;

    for (uint32_t q = 0U; q < 80U; q++)
    {
        if (vig_yp >= 0 && (q & 3U) == 0U)
        {
            uint32_t idx = (((uint32_t)s_vf_vig_dx2[q >> 2] + dy2) * 105UL) >> 16;
            if (idx > 63UL) idx = 63UL;
            g_q8 = (int32_t)s_vf_vig_lut[idx];
            a_q8 = (g_q8 << 3) - g_q8;
            b_q8 = (g_q8 << 1) + g_q8;
        }
        uint32_t r0123 = *r32++;

        /* Pixel 0 (x = 4*q): demosaic + AE/AWB/ETTR sample */
        rclk_port->BRR  = rclk_pin;
        uint32_t b0     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;

        uint32_t r0     = r0123 & 0xFFUL;

        rclk_port->BRR  = rclk_pin;
        uint32_t g0     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;

        uint32_t g0_6   = lut[g0 + 256U];
        uint32_t px0    = (bwlut != NULL)
                          ? vf_bw_vig_pack_rgb565(bwlut, is_red, lut[r0], g0_6, lut[b0 + 512U], g_q8)
                          : vf_chroma_vig_pack_rgb565((int32_t)lut[r0], (int32_t)g0_6, (int32_t)lut[b0 + 512U], a_q8, b_q8);

        {
            uint16_t r12 = s_inv_gamma_12[r0];
            uint16_t g12 = s_inv_gamma_12[g0];
            uint16_t b12 = s_inv_gamma_12[b0];
            uint8_t  rl  = (uint8_t)((r12 > bl_rb) ? ((r12 - bl_rb) >> 4) : 0U);
            uint8_t  gl  = (uint8_t)((g12 > bl_g)  ? ((g12 - bl_g)  >> 4) : 0U);
            uint8_t  bl  = (uint8_t)((b12 > bl_rb) ? ((b12 - bl_rb) >> 4) : 0U);
            uint8_t  mx  = rl;
            if (gl > mx) mx = gl;
            if (bl > mx) mx = bl;
            lin_bins[mx >> 4]++;

            uint32_t w = (cw_row && q >= 20U && q < 60U) ? 6UL : 1UL;
            ag += (uint32_t)gl * w;
            ac += w;
            if (r12 >= 3920U || g12 >= 3920U || b12 >= 3920U)
            {
                hc++;
            }
            else if (rl >= 8U && gl >= 8U && bl >= 8U)
            {
                sr += rl;
                sg += gl;
                sb += bl;
                wc++;
            }
        }

        /* Pixel 1 */
        rclk_port->BRR  = rclk_pin;
        uint32_t b1     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;
        rclk_port->BRR  = rclk_pin;
        uint32_t g1     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;
        uint32_t g1_6   = lut[g1 + 256U];
        uint32_t px1    = (bwlut != NULL)
                          ? vf_bw_vig_pack_rgb565(bwlut, is_red, lut[(r0123 >> 8) & 0xFFUL], g1_6, lut[b1 + 512U], g_q8)
                          : vf_chroma_vig_pack_rgb565((int32_t)lut[(r0123 >> 8) & 0xFFUL], (int32_t)g1_6, (int32_t)lut[b1 + 512U], a_q8, b_q8);
        uint32_t w01    = px0 | (px1 << 16);
        *dst32++        = is_le_ring ? w01 : __REV16(w01);

        /* Pixel 2 */
        rclk_port->BRR  = rclk_pin;
        uint32_t b2     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;
        rclk_port->BRR  = rclk_pin;
        uint32_t g2     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;
        uint32_t g2_6   = lut[g2 + 256U];
        uint32_t px2    = (bwlut != NULL)
                          ? vf_bw_vig_pack_rgb565(bwlut, is_red, lut[(r0123 >> 16) & 0xFFUL], g2_6, lut[b2 + 512U], g_q8)
                          : vf_chroma_vig_pack_rgb565((int32_t)lut[(r0123 >> 16) & 0xFFUL], (int32_t)g2_6, (int32_t)lut[b2 + 512U], a_q8, b_q8);

        /* Pixel 3 */
        rclk_port->BRR  = rclk_pin;
        uint32_t b3     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;
        rclk_port->BRR  = rclk_pin;
        uint32_t g3     = (data_port->IDR >> 1) & 0xFFUL;
        rclk_port->BSRR = rclk_pin;
        uint32_t g3_6   = lut[g3 + 256U];
        uint32_t px3    = (bwlut != NULL)
                          ? vf_bw_vig_pack_rgb565(bwlut, is_red, lut[r0123 >> 24], g3_6, lut[b3 + 512U], g_q8)
                          : vf_chroma_vig_pack_rgb565((int32_t)lut[r0123 >> 24], (int32_t)g3_6, (int32_t)lut[b3 + 512U], a_q8, b_q8);
        uint32_t w23    = px2 | (px3 << 16);
        *dst32++        = is_le_ring ? w23 : __REV16(w23);

        if (!is_le_ring)
        {
            *g32++ = g0_6 | (g1_6 << 8) | (g2_6 << 16) | (g3_6 << 24);
        }
    }

    *wb_sum_r  += sr;
    *wb_sum_g  += sg;
    *wb_sum_b  += sb;
    *wb_cnt    += wc;
    *ae_sum_g  += ag;
    *ae_cnt    += ac;
    *hi_cnt    += hc;
    *lin_total += 80UL;
}

/* 4x unrolled 1:1 GRBG demosaic using 32-bit aligned row reads and reused B/G neighbors */
static void __attribute__((optimize("O3"))) raw_demosaic_1x1_to_buf(
    const uint8_t *even_row, const uint8_t *odd_row, uint8_t *dst, uint8_t *g_dst)
{
    const uint8_t  *const lut   = s_lut_rgb6[0];
    const uint32_t       *ev32  = (const uint32_t *)even_row;
    const uint32_t       *od32  = (const uint32_t *)odd_row;
    uint32_t             *dst32 = (uint32_t *)dst;
    uint32_t             *g32   = (uint32_t *)g_dst;

    uint32_t e_cur = *ev32++;
    uint32_t o_cur = *od32++;
    int32_t  b0_6  = (int32_t)lut[(o_cur & 0xFFU) | 0x200U];
    uint32_t n     = 80U; /* 80 x 4 = 320 pixels */
    const uint8_t cm = g_cfg.color_mode;

    if (cm != C3M1_COLOR_COLOR)
    {
        const uint16_t *const bwlut  = s_vf_bw_rgb565_le[cm - 1U];
        const uint8_t         is_red = (cm == C3M1_COLOR_BW_RED) ? 1U : 0U;
        do
        {
            uint32_t e_nxt = *ev32++;
            uint32_t o_nxt = *od32++;

            uint32_t g0   = e_cur & 0xFFU;
            uint32_t r0_6 = lut[(e_cur >> 8) & 0xFFU];
            uint32_t g2   = (e_cur >> 16) & 0xFFU;
            uint32_t r1_6 = lut[e_cur >> 24];

            uint32_t g1   = (o_cur >> 8) & 0xFFU;
            uint32_t b1_6 = lut[((o_cur >> 16) & 0xFFU) | 0x200U];
            uint32_t g3   = o_cur >> 24;

            uint32_t gm0  = lut[((g0 + g1) >> 1) | 0x100U];
            uint32_t px0  = vf_bw_pack_rgb565(bwlut, is_red, r0_6, gm0, (uint32_t)b0_6);

            uint32_t gm1  = lut[((g2 + g1) >> 1) | 0x100U];
            uint32_t px1  = vf_bw_pack_rgb565(bwlut, is_red, r0_6, gm1, b1_6);
            *dst32++      = __REV16(px0 | (px1 << 16));

            uint32_t gm2  = lut[((g2 + g3) >> 1) | 0x100U];
            uint32_t px2  = vf_bw_pack_rgb565(bwlut, is_red, r1_6, gm2, b1_6);

            uint32_t b2_6 = lut[(o_nxt & 0xFFU) | 0x200U];
            uint32_t gm3  = lut[(((e_nxt & 0xFFU) + g3) >> 1) | 0x100U];
            uint32_t px3  = vf_bw_pack_rgb565(bwlut, is_red, r1_6, gm3, b2_6);
            *dst32++      = __REV16(px2 | (px3 << 16));

            *g32++        = gm0 | (gm1 << 8) | (gm2 << 16) | (gm3 << 24);

            e_cur = e_nxt;
            o_cur = o_nxt;
            b0_6  = (int32_t)b2_6;
        } while (--n != 0U);
        return;
    }

    do
    {
        uint32_t e_nxt = *ev32++;
        uint32_t o_nxt = *od32++;

        uint32_t g0   = e_cur & 0xFFU;
        int32_t  r0_6 = (int32_t)lut[(e_cur >> 8) & 0xFFU];
        uint32_t g2   = (e_cur >> 16) & 0xFFU;
        int32_t  r1_6 = (int32_t)lut[e_cur >> 24];

        uint32_t g1   = (o_cur >> 8) & 0xFFU;
        int32_t  b1_6 = (int32_t)lut[((o_cur >> 16) & 0xFFU) | 0x200U];
        uint32_t g3   = o_cur >> 24;

        uint32_t gm0  = lut[((g0 + g1) >> 1) | 0x100U];
        uint32_t px0  = vf_chroma_pack_rgb565(r0_6, (int32_t)gm0, b0_6);

        uint32_t gm1  = lut[((g2 + g1) >> 1) | 0x100U];
        uint32_t px1  = vf_chroma_pack_rgb565(r0_6, (int32_t)gm1, b1_6);
        *dst32++      = __REV16(px0 | (px1 << 16));

        uint32_t gm2  = lut[((g2 + g3) >> 1) | 0x100U];
        uint32_t px2  = vf_chroma_pack_rgb565(r1_6, (int32_t)gm2, b1_6);

        int32_t  b2_6 = (int32_t)lut[(o_nxt & 0xFFU) | 0x200U];
        uint32_t gm3  = lut[(((e_nxt & 0xFFU) + g3) >> 1) | 0x100U];
        uint32_t px3  = vf_chroma_pack_rgb565(r1_6, (int32_t)gm3, b2_6);
        *dst32++      = __REV16(px2 | (px3 << 16));

        *g32++        = gm0 | (gm1 << 8) | (gm2 << 16) | (gm3 << 24);

        e_cur = e_nxt;
        o_cur = o_nxt;
        b0_6  = b2_6;
    } while (--n != 0U);
}

static void lcd_send_wind_strips(const uint8_t *line_rgb, uint16_t y)
{
    const uint16_t right_x = PROMPT_BOX_X1 + 1U;
    const uint16_t right_w = 320U - right_x;

    lcd_dma_wait();
    lcd_set_window(0, y, PROMPT_BOX_X0, 1);
    lcd_write_ram_prepare();
    lcd_dma_send(line_rgb, (uint16_t)(PROMPT_BOX_X0 * 2U));

    lcd_dma_wait();
    lcd_set_window(right_x, y, right_w, 1);
    lcd_write_ram_prepare();
    lcd_dma_send(line_rgb + (right_x * 2U), (uint16_t)(right_w * 2U));
    lcd_dma_wait();
}

/* ======================== Viewfinder Overlays (Histogram / Peaking / Grid) ======================== */
#define HIST_X0                     250U    /* 66px wide: 1px border + 64 bins + 1px border (250..315) */
#define HIST_Y0                     192U    /* Pair-aligned even..odd (192..217 = 26px: 1+24+1) */
#define HIST_Y1                     217U
#define HIST_BAR_MAX                22U

#define DBG_X0                      250U    /* Aligned right above histogram (250..315 = 66px) */
#define DBG_Y0                      162U    /* Pair-aligned even..odd (162..189 = 28px: 1+12+2+12+1) */
#define DBG_Y1                      189U
#define DBG_INNER_H                 26U

static uint64_t s_dbg_mask[DBG_INNER_H];
static uint8_t  s_dbg_mask_fps = 0xFFU;
static uint16_t s_dbg_mask_cnt = 0xFFFFU;

static uint8_t  s_peak_g[2][320] __attribute__((aligned(4))); /* Ping-pong G6 rows: zero memcpy */
static uint16_t s_hist_bins[64];
static uint8_t  s_hist_bar_h[64];
static uint8_t  s_hist_clipped = 0U;

static void __attribute__((optimize("O3"))) vf_hist_sample_row(const uint8_t *line,
                                                               uint16_t *bins,
                                                               uint32_t *clip_cnt,
                                                               uint32_t *sample_cnt)
{
    const uint16_t *line16 = (const uint16_t *)line;
    uint32_t clip = 0UL;

    for (uint32_t x = 0U; x < 320U; x += 4U)
    {
        uint32_t px  = __REV16((uint32_t)line16[x]);
        uint32_t r5  = px >> 11;
        uint32_t g6  = (px >> 5) & 0x3FUL;
        uint32_t b5  = px & 0x1FUL;
        uint32_t r6  = (r5 << 1) | (r5 >> 4); /* 0..31 -> 0..63 */
        uint32_t b6  = (b5 << 1) | (b5 >> 4); /* 0..31 -> 0..63 */
        uint32_t bin = (r6 + (g6 << 1) + b6) >> 2;
        bins[bin]++;
        if (r5 >= 30U || g6 >= 61U || b5 >= 30U)
        {
            clip++;
        }
    }

    *clip_cnt   += clip;
    *sample_cnt += 80UL;
}

static void vf_hist_finalize(const uint16_t *bins, uint32_t clip_cnt, uint32_t sample_cnt)
{
    uint16_t max_bin = (uint16_t)(sample_cnt / 24U);
    if (max_bin < 1U) max_bin = 1U;
    for (uint8_t i = 0; i < 64U; i++)
    {
        if (bins[i] > max_bin) max_bin = bins[i];
    }
    for (uint8_t i = 0; i < 64U; i++)
    {
        uint32_t h = ((uint32_t)bins[i] * HIST_BAR_MAX + (max_bin >> 1)) / max_bin;
        if (h > HIST_BAR_MAX) h = HIST_BAR_MAX;
        s_hist_bar_h[i] = (uint8_t)h;
    }
    s_hist_clipped = (sample_cnt > 0U && (clip_cnt * 50U) >= sample_cnt) ? 1U : 0U;
}

static void __attribute__((optimize("O3"))) vf_apply_focus_peaking(uint8_t *line,
                                                                   const uint8_t *cur_g,
                                                                   const uint8_t *prev_g,
                                                                   uint8_t has_prev,
                                                                   uint8_t is_4x,
                                                                   uint16_t peak_col)
{
    /*
     * Zero-copy single-pass 1px Sony-style focus peaking (4x unrolled, fall-through fast path):
     *  - cur_g/prev_g populated inline during demosaic via s_lut_g6 (no unpack, no memcpy)
     *  - 1px horizontal NMS hairline + unsigned-range vertical check
     */
    int32_t thresh_x = (int32_t)((is_4x ? 19U : 23U) + g_lut_last_gain_x);
    int32_t thresh_y = is_4x ? thresh_x : (thresh_x + 7);
    if (thresh_x > 42) thresh_x = 42;
    if (thresh_y > 48) thresh_y = 48;

    const uint16_t peak_be = (uint16_t)__REV16((uint32_t)peak_col);

    const uint8_t *cg   = cur_g + 1;
    uint16_t      *lp16 = (uint16_t *)line + 1;
    int32_t g_cur   = cg[0];
    int32_t gx_prev = 0;
    int32_t gx_cur  = g_cur - (int32_t)cur_g[0];
    if (gx_cur < 0) gx_cur = -gx_cur;

    if (!has_prev)
    {
        for (uint32_t n = 318U; n != 0U; n--)
        {
            int32_t g_next  = *++cg;
            int32_t d_x     = g_next - g_cur;
            int32_t gx_next = (d_x ^ (d_x >> 31)) - (d_x >> 31);

            if (__builtin_expect(gx_cur >= thresh_x && gx_cur > gx_prev && gx_cur >= gx_next, 0))
            {
                *lp16 = peak_be;
            }
            lp16++;
            g_cur   = g_next;
            gx_prev = gx_cur;
            gx_cur  = gx_next;
        }
        return;
    }

    const uint8_t  *pg  = prev_g + 1;
    const uint32_t  ty  = (uint32_t)thresh_y;
    const uint32_t  ty2 = ty << 1;

#define PEAK_STEP_2D(idx) do { \
        int32_t  _gn = cg[(idx) + 1U]; \
        int32_t  _dx = _gn - g_cur; \
        int32_t  _gxn = (_dx ^ (_dx >> 31)) - (_dx >> 31); \
        uint32_t _dv = (uint32_t)((g_cur - (int32_t)pg[idx]) + (int32_t)ty); \
        if (__builtin_expect(_dv >= ty2, 0) || \
            __builtin_expect(gx_cur >= thresh_x && gx_cur > gx_prev && gx_cur >= _gxn, 0)) \
        { \
            lp16[idx] = peak_be; \
        } \
        g_cur   = _gn; \
        gx_prev = gx_cur; \
        gx_cur  = _gxn; \
    } while (0)

    /* 79 x 4 + 2 = 318 interior pixels (x = 1..318) */
    for (uint32_t n = 79U; n != 0U; n--)
    {
        PEAK_STEP_2D(0U);
        PEAK_STEP_2D(1U);
        PEAK_STEP_2D(2U);
        PEAK_STEP_2D(3U);
        cg   += 4U;
        pg   += 4U;
        lp16 += 4U;
    }
    PEAK_STEP_2D(0U);
    PEAK_STEP_2D(1U);

#undef PEAK_STEP_2D
}

static inline void vf_blend_grid_px(uint8_t *p)
{
    uint16_t px = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    px = (uint16_t)(((px & 0xF7DEU) >> 1) + 0x528AU);
    p[0] = (uint8_t)(px >> 8);
    p[1] = (uint8_t)px;
}

static void __attribute__((optimize("O2"))) vf_overlay_grid_v(uint8_t *line)
{
    vf_blend_grid_px(&line[106U * 2U]);
    vf_blend_grid_px(&line[213U * 2U]);
}

static void __attribute__((optimize("O2"))) vf_overlay_grid_h(uint8_t *line)
{
    for (uint16_t x = 0; x < 320U; x++)
    {
        vf_blend_grid_px(&line[x * 2U]);
    }
}

/* 75% translucent crisp white pixel for center-weighted metering reticle */
static inline void vf_blend_reticle_px(uint8_t *p)
{
    uint16_t c = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    c = (uint16_t)(((c >> 2) & 0x39E7U) + 0xC618U);
    p[0] = (uint8_t)(c >> 8);
    p[1] = (uint8_t)c;
}

/* Vertical arms of CENTER metering zone (x=80..239, y=61..179) + center crosshair vertical */
static void __attribute__((optimize("O2"))) vf_overlay_center_v(uint8_t *line, uint16_t y)
{
    if ((y >= 62U && y <= 75U) || (y >= 164U && y <= 178U))
    {
        vf_blend_reticle_px(&line[80U * 2U]);
        vf_blend_reticle_px(&line[239U * 2U]);
    }
    if (y >= 116U && y <= 123U)
    {
        vf_blend_reticle_px(&line[160U * 2U]);
    }
}

/* 1px horizontal arms of CENTER metering zone (y=61, y=179) + center crosshair horizontal (y=119) */
static void __attribute__((optimize("O2"))) vf_overlay_center_h(uint8_t *line, uint16_t y)
{
    if (y == 61U || y == 179U)
    {
        for (uint16_t x = 80U; x <= 95U; x++)   vf_blend_reticle_px(&line[x * 2U]);
        for (uint16_t x = 224U; x <= 239U; x++) vf_blend_reticle_px(&line[x * 2U]);
    }
    else if (y == 119U)
    {
        for (uint16_t x = 156U; x <= 164U; x++) vf_blend_reticle_px(&line[x * 2U]);
    }
}

static void __attribute__((optimize("O2"))) vf_overlay_hist_row(uint8_t *line, uint16_t y)
{
    uint8_t *p = &line[HIST_X0 * 2U];

    if (y == HIST_Y0 || y == HIST_Y1)
    {
        for (uint16_t i = 0; i < 66U; i++)
        {
            *p++ = 0x63U;
            *p++ = 0x0CU;
        }
        return;
    }

    /* Left border */
    *p++ = 0x63U;
    *p++ = 0x0CU;

    uint8_t row_from_bot = (uint8_t)((HIST_Y1 - 1U) - y);
    for (uint8_t b = 0; b < 64U; b++)
    {
        if (row_from_bot < s_hist_bar_h[b])
        {
            if (s_hist_clipped && b >= 62U)
            {
                *p++ = 0xF8U; *p++ = 0x00U; /* Red highlight clipping bar */
            }
            else
            {
                *p++ = 0xFFU; *p++ = 0xFFU; /* Crisp white histogram bar */
            }
        }
        else if (b == 16U || b == 32U || b == 48U)
        {
            *p++ = 0x21U; *p++ = 0x04U;     /* Subtle quarter-stop vertical guide */
        }
        else
        {
            *p++ = 0x10U; *p++ = 0x82U;     /* Dark charcoal chart background */
        }
    }

    /* Right border (turns bright RED when >2% highlight clipping detected) */
    if (s_hist_clipped)
    {
        *p++ = 0xF8U;
        *p++ = 0x00U;
    }
    else
    {
        *p++ = 0x63U;
        *p++ = 0x0CU;
    }
}

static void vf_dbg_draw_mask_line(const char *txt, uint8_t row_off, uint8_t col_off)
{
    for (uint8_t ci = 0U; txt[ci] != '\0'; ci++)
    {
        const uint8_t *glyph = lcd_get_font12(txt[ci]);
        uint8_t base_x = (uint8_t)(col_off + ci * 6U);
        if ((uint8_t)(base_x + 6U) > 64U)
        {
            break;
        }
        for (uint8_t cx = 0U; cx < 6U; cx++)
        {
            uint8_t  b0  = glyph[cx * 2U];
            uint8_t  b1  = glyph[cx * 2U + 1U];
            uint64_t bit = 1ULL << (base_x + cx);
            for (uint8_t r = 0U; r < 8U; r++)
            {
                if (b0 & (0x80U >> r)) s_dbg_mask[row_off + r] |= bit;
            }
            for (uint8_t r = 0U; r < 4U; r++)
            {
                if (b1 & (0x80U >> r)) s_dbg_mask[row_off + 8U + r] |= bit;
            }
        }
    }
}

static void vf_dbg_rebuild_mask(uint8_t fps, uint16_t disp_cnt)
{
    char line0[12];
    char line1[12];
    if (fps == 0U)
    {
        snprintf(line0, sizeof(line0), "FPS:    --");
    }
    else
    {
        snprintf(line0, sizeof(line0), "FPS:    %2u", (unsigned int)fps);
    }
    snprintf(line1, sizeof(line1), "SHT: %05u", (unsigned int)disp_cnt);

    memset(s_dbg_mask, 0, sizeof(s_dbg_mask));
    /* 10 chars = 60 px inside 64 px inner width -> 2 px padding on left & right */
    vf_dbg_draw_mask_line(line0, 0U,  2U);
    vf_dbg_draw_mask_line(line1, 14U, 2U);
    s_dbg_mask_fps = fps;
    s_dbg_mask_cnt = disp_cnt;
}

static void __attribute__((optimize("O3"))) vf_overlay_dbg_row(uint8_t *line, uint16_t y)
{
    uint16_t *p16 = (uint16_t *)(line + DBG_X0 * 2U);
    const uint16_t brd_be = 0x0C63U; /* Big-endian 0x630C */
    const uint16_t bg_be  = 0x8210U; /* Big-endian 0x1082 */

    if (y == DBG_Y0 || y == DBG_Y1)
    {
        for (uint32_t i = 0U; i < 66U; i++)
        {
            *p16++ = brd_be;
        }
        return;
    }

    *p16++ = brd_be;

    uint8_t  inner_y  = (uint8_t)(y - (DBG_Y0 + 1U));
    uint64_t row_bits = s_dbg_mask[inner_y];
    uint32_t lo       = (uint32_t)row_bits;
    uint32_t hi       = (uint32_t)(row_bits >> 32);
    /* Line 0 (FPS): GREEN 0x07E0 -> BE 0xE007; Line 1 (SHT): CYAN 0x07FF -> BE 0xFF07 */
    uint16_t fg_be    = (inner_y < 13U) ? 0xE007U : 0xFF07U;

    for (uint32_t col = 0U; col < 32U; col++)
    {
        *p16++ = (lo & 1UL) ? fg_be : bg_be;
        lo >>= 1;
    }
    for (uint32_t col = 0U; col < 32U; col++)
    {
        *p16++ = (hi & 1UL) ? fg_be : bg_be;
        hi >>= 1;
    }

    *p16++ = brd_be;
}

static void __attribute__((optimize("O2"))) vf_overlay_top_hud_row(uint8_t *line, uint16_t y)
{
    uint8_t       *p   = &line[TOP_HUD_X0 * 2U];
    const uint8_t *row = s_top_hud_buf[y - TOP_HUD_Y0];

    for (uint16_t i = 0U; i < (TOP_HUD_W / 2U); i++)
    {
        uint8_t b  = row[i];
        uint8_t c0 = (uint8_t)(b & 0x0FU);
        uint8_t c1 = (uint8_t)(b >> 4);
        if (c0 != 0U)
        {
            uint16_t col = s_osd_pal[c0];
            p[0] = (uint8_t)(col >> 8);
            p[1] = (uint8_t)col;
        }
        p += 2U;

        if (c1 != 0U)
        {
            uint16_t col = s_osd_pal[c1];
            p[0] = (uint8_t)(col >> 8);
            p[1] = (uint8_t)col;
        }
        p += 2U;
    }
}

static void __attribute__((optimize("O3"))) vf_overlay_osd_row(uint8_t *line, uint16_t y)
{
    uint32_t *p32 = (uint32_t *)line;
    if (y == OSD_BAR_Y)
    {
        for (uint32_t i = 0U; i < 160U; i++)
        {
            uint32_t w = __REV16(*p32);
            w = ((w & 0xF7DEF7DEUL) >> 1) + 0x528A528AUL;
            *p32++ = __REV16(w);
        }
        return;
    }

    if (y < (OSD_BAR_Y + 4U) || y >= (OSD_BAR_Y + 16U))
    {
        for (uint32_t i = 0U; i < 160U; i++)
        {
            uint32_t w = __REV16(*p32);
            *p32++ = __REV16((w >> 2) & 0x39E739E7UL);
        }
        return;
    }

    const uint8_t *row = s_osd_bar_buf[y - (OSD_BAR_Y + 4U)];
    for (uint32_t i = 0U; i < 160U; i++)
    {
        uint8_t b = row[i];
        if (__builtin_expect(b == 0U, 1))
        {
            uint32_t w = __REV16(*p32);
            *p32++ = __REV16((w >> 2) & 0x39E739E7UL);
        }
        else
        {
            uint32_t w  = __REV16(*p32);
            uint8_t  c0 = (uint8_t)(b & 0x0FU);
            uint8_t  c1 = (uint8_t)(b >> 4);
            uint32_t p0 = (c0 != 0U) ? (uint32_t)s_osd_pal[c0] : ((w >> 2) & 0x39E7UL);
            uint32_t p1 = (c1 != 0U) ? (uint32_t)s_osd_pal[c1] : (((w >> 18) & 0x39E7UL));
            *p32++ = __REV16(p0 | (p1 << 16));
        }
    }
}

static uint32_t u32_isqrt(uint32_t x)
{
    uint32_t res = 0UL;
    uint32_t bit = 1UL << 30;
    while (bit > x) bit >>= 2;
    while (bit != 0UL)
    {
        if (x >= res + bit)
        {
            x   -= res + bit;
            res  = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

static void vf_vig_rebuild_lut(uint8_t vig_steps)
{
    if (s_vf_vig_lut_val == vig_steps) return;
    s_vf_vig_lut_val = vig_steps;
    for (uint32_t i = 0UL; i <= 64UL; i++)
    {
        if (vig_steps == 0U)
        {
            s_vf_vig_lut[i] = 256U;
        }
        else
        {
            /* Linear gain g = 1 + (vig*0.05)*(i/64) in Q16 -> sRGB gain g^(1/2.2) in Q8 via isqrt + Pade(1,1) */
            uint32_t g_q16 = 65536UL + (((uint32_t)vig_steps * 512UL * i + 5UL) / 10UL);
            uint32_t s_q8  = u32_isqrt(g_q16);
            uint32_t d     = (s_q8 > 256UL) ? (s_q8 - 256UL) : 0UL;
            uint32_t num   = 11UL * 256UL + 5UL * d;
            uint32_t den   = 11UL * 256UL + 6UL * d;
            uint32_t g_srgb = (s_q8 * num + (den >> 1)) / den;
            if (g_srgb < 256UL) g_srgb = 256UL;
            s_vf_vig_lut[i] = (uint16_t)g_srgb;
        }
    }
}

static uint16_t vf_req_max_src_yp(uint16_t yp, int32_t kr0_q16, int32_t kr1_q16)
{
    int32_t  dy  = (int32_t)yp * 2L - 120L;
    uint32_t dy2 = (uint32_t)(dy * dy);
    int32_t  dyp = (int32_t)yp - 60L;

    /* Radial scale is monotonic in r^2: extrema are always at center (dx=0) and left edge (dx=-160) */
    uint32_t r2_c  = (dy2 * 53687UL) >> 15;
    int32_t  s_c   = kr0_q16 + ((kr1_q16 * (int32_t)r2_c) >> 16);
    int32_t  syp_c = 60L + ((dyp * s_c + 32768L) >> 16);

    uint32_t r2_e  = ((25600UL + dy2) * 53687UL) >> 15;
    int32_t  s_e   = kr0_q16 + ((kr1_q16 * (int32_t)r2_e) >> 16);
    int32_t  syp_e = 60L + ((dyp * s_e + 32768L) >> 16);

    int32_t req = (syp_c > syp_e) ? syp_c : syp_e;
    if (req < 0L)   req = 0L;
    if (req > 119L) req = 119L;
    return (uint16_t)req;
}

/*
 * Fast 1X row warp (-O3, zero-spill): 20 piecewise-linear 16-pixel spans (<0.04 px err at |kr1|<=0.200)
 * exploiting exact left-right radial symmetry (11 knots instead of 21), hoisting constant-row pointer,
 * 4-pixel-per-iter no-unroll inner loops (zero GCC stack spills), 32-bit packed REV16 + 32-bit G6 stores.
 */
static void __attribute__((optimize("O3"))) vf_warp_row_1x(uint8_t *dst_line, uint8_t *dst_g,
                                                           const uint16_t *const ring_rows[16],
                                                           uint16_t yp,
                                                           int32_t kr0_q16, int32_t kr1_q16)
{
    const int32_t   dy  = (int32_t)yp * 2L - 120L;
    const uint32_t  dy2 = (uint32_t)(dy * dy);
    const int32_t   dyp = (int32_t)yp - 60L;

    /* Compute 11 symmetric knots (dx = -160..-16, 0) and mirror to right half (+16..+160) */
    int32_t sx_k[21];
    int32_t sy_k[21];
    #pragma GCC unroll 1
    for (int32_t j = 0; j < 10; j++)
    {
        int32_t  d      = (10L - j) * 16L;
        uint32_t d2     = (uint32_t)(d * d) + dy2;
        uint32_t r2_q16 = (d2 * 53687UL) >> 15;
        int32_t  s_q16  = kr0_q16 + ((kr1_q16 * (int32_t)r2_q16) >> 16);
        int32_t  off_x  = d * s_q16;
        int32_t  sx_l   = (160L << 16) - off_x;
        int32_t  sx_r   = (160L << 16) + off_x;
        int32_t  syp    = ( 60L << 16) + dyp * s_q16;
        if (sx_l < 0L)           sx_l = 0L;
        if (sx_l > (319L << 16)) sx_l = (319L << 16);
        if (sx_r < 0L)           sx_r = 0L;
        if (sx_r > (319L << 16)) sx_r = (319L << 16);
        if (syp < 0L)            syp  = 0L;
        if (syp > (119L << 16))  syp  = (119L << 16);
        sx_k[j]      = sx_l;
        sy_k[j]      = syp;
        sx_k[20 - j] = sx_r;
        sy_k[20 - j] = syp;
    }
    {
        uint32_t r2_q16 = (dy2 * 53687UL) >> 15;
        int32_t  s_q16  = kr0_q16 + ((kr1_q16 * (int32_t)r2_q16) >> 16);
        int32_t  syp    = (60L << 16) + dyp * s_q16;
        if (syp < 0L)           syp = 0L;
        if (syp > (119L << 16)) syp = (119L << 16);
        sx_k[10] = (160L << 16);
        sy_k[10] = syp;
    }

    uint32_t *dst32 = (uint32_t *)dst_line;

    if (dst_g != NULL)
    {
        uint32_t *g32 = (uint32_t *)dst_g;
        #pragma GCC unroll 1
        for (uint32_t k = 0U; k < 20U; k++)
        {
            int32_t  sx0 = sx_k[k];
            int32_t  sy0 = sy_k[k];
            int32_t  dsx = (sx_k[k + 1U] - sx0) >> 4;
            int32_t  dsy = (sy_k[k + 1U] - sy0) >> 4;
            int32_t  cx  = sx0 + 32768L;
            int32_t  cy  = sy0 + 32768L;
            uint32_t iy0 = (uint32_t)cy >> 16;
            uint32_t iy1 = (uint32_t)(cy + dsy * 15L) >> 16;

            if (__builtin_expect(iy0 == iy1, 1))
            {
                const uint16_t *const srow = ring_rows[iy0 & 15U];
                uint32_t i = 4U;
                #pragma GCC unroll 1
                do
                {
                    uint32_t px0 = srow[(uint32_t)cx >> 16]; cx += dsx;
                    uint32_t px1 = srow[(uint32_t)cx >> 16]; cx += dsx;
                    *dst32++ = __REV16(px0 | (px1 << 16));
                    uint32_t g01 = ((px0 >> 5) & 0x3FU) | (((px1 >> 5) & 0x3FU) << 8);
                    px0 = srow[(uint32_t)cx >> 16]; cx += dsx;
                    px1 = srow[(uint32_t)cx >> 16]; cx += dsx;
                    *dst32++ = __REV16(px0 | (px1 << 16));
                    *g32++ = g01 | (((px0 >> 5) & 0x3FU) << 16) | (((px1 >> 5) & 0x3FU) << 24);
                } while (--i != 0U);
            }
            else
            {
                uint32_t i = 4U;
                #pragma GCC unroll 1
                do
                {
                    uint32_t px0 = ring_rows[((uint32_t)cy >> 16) & 15U][(uint32_t)cx >> 16];
                    cx += dsx; cy += dsy;
                    uint32_t px1 = ring_rows[((uint32_t)cy >> 16) & 15U][(uint32_t)cx >> 16];
                    cx += dsx; cy += dsy;
                    *dst32++ = __REV16(px0 | (px1 << 16));
                    uint32_t g01 = ((px0 >> 5) & 0x3FU) | (((px1 >> 5) & 0x3FU) << 8);
                    px0 = ring_rows[((uint32_t)cy >> 16) & 15U][(uint32_t)cx >> 16];
                    cx += dsx; cy += dsy;
                    px1 = ring_rows[((uint32_t)cy >> 16) & 15U][(uint32_t)cx >> 16];
                    cx += dsx; cy += dsy;
                    *dst32++ = __REV16(px0 | (px1 << 16));
                    *g32++ = g01 | (((px0 >> 5) & 0x3FU) << 16) | (((px1 >> 5) & 0x3FU) << 24);
                } while (--i != 0U);
            }
        }
    }
    else
    {
        #pragma GCC unroll 1
        for (uint32_t k = 0U; k < 20U; k++)
        {
            int32_t  sx0 = sx_k[k];
            int32_t  sy0 = sy_k[k];
            int32_t  dsx = (sx_k[k + 1U] - sx0) >> 4;
            int32_t  dsy = (sy_k[k + 1U] - sy0) >> 4;
            int32_t  cx  = sx0 + 32768L;
            int32_t  cy  = sy0 + 32768L;
            uint32_t iy0 = (uint32_t)cy >> 16;
            uint32_t iy1 = (uint32_t)(cy + dsy * 15L) >> 16;

            if (__builtin_expect(iy0 == iy1, 1))
            {
                const uint16_t *const srow = ring_rows[iy0 & 15U];
                uint32_t i = 4U;
                #pragma GCC unroll 1
                do
                {
                    uint32_t px0 = srow[(uint32_t)cx >> 16]; cx += dsx;
                    uint32_t px1 = srow[(uint32_t)cx >> 16]; cx += dsx;
                    *dst32++ = __REV16(px0 | (px1 << 16));
                    px0 = srow[(uint32_t)cx >> 16]; cx += dsx;
                    px1 = srow[(uint32_t)cx >> 16]; cx += dsx;
                    *dst32++ = __REV16(px0 | (px1 << 16));
                } while (--i != 0U);
            }
            else
            {
                uint32_t i = 4U;
                #pragma GCC unroll 1
                do
                {
                    uint32_t px0 = ring_rows[((uint32_t)cy >> 16) & 15U][(uint32_t)cx >> 16];
                    cx += dsx; cy += dsy;
                    uint32_t px1 = ring_rows[((uint32_t)cy >> 16) & 15U][(uint32_t)cx >> 16];
                    cx += dsx; cy += dsy;
                    *dst32++ = __REV16(px0 | (px1 << 16));
                    px0 = ring_rows[((uint32_t)cy >> 16) & 15U][(uint32_t)cx >> 16];
                    cx += dsx; cy += dsy;
                    px1 = ring_rows[((uint32_t)cy >> 16) & 15U][(uint32_t)cx >> 16];
                    cx += dsx; cy += dsy;
                    *dst32++ = __REV16(px0 | (px1 << 16));
                } while (--i != 0U);
            }
        }
    }
}

/* Fast Thumb-2 LDRD/STRD 32-bit SWAR 50/50 average on big-endian RGB565 rows (8 px/iter, ~2.9 ms / 110 rows) */
static void __attribute__((optimize("O3"))) vf_avg_row_be(uint8_t *__restrict dst,
                                                          const uint8_t *__restrict row0,
                                                          const uint8_t *__restrict row1)
{
#if defined(__thumb2__)
    uint32_t *d = (uint32_t *)dst;
    const uint32_t *a = (const uint32_t *)row0;
    const uint32_t *b = (const uint32_t *)row1;
    uint32_t n = 40U; /* 40 x 4 words = 160 words = 320 pixels */
    const uint32_t mask = 0x7BEF7BEFUL;
    uint32_t a0, a1, b0, b1, t;

    __asm__ volatile (
        "1:\n\t"
        "ldrd  %[a0], %[a1], [%[a]], #8\n\t"
        "ldrd  %[b0], %[b1], [%[b]], #8\n\t"
        "rev16 %[a0], %[a0]\n\t"
        "rev16 %[b0], %[b0]\n\t"
        "eor   %[t],  %[a0], %[b0]\n\t"
        "and   %[a0], %[a0], %[b0]\n\t"
        "and   %[t],  %[m],  %[t], lsr #1\n\t"
        "add   %[a0], %[a0], %[t]\n\t"
        "rev16 %[a0], %[a0]\n\t"
        "rev16 %[a1], %[a1]\n\t"
        "rev16 %[b1], %[b1]\n\t"
        "eor   %[t],  %[a1], %[b1]\n\t"
        "and   %[a1], %[a1], %[b1]\n\t"
        "and   %[t],  %[m],  %[t], lsr #1\n\t"
        "add   %[a1], %[a1], %[t]\n\t"
        "rev16 %[a1], %[a1]\n\t"
        "strd  %[a0], %[a1], [%[d]], #8\n\t"
        "ldrd  %[a0], %[a1], [%[a]], #8\n\t"
        "ldrd  %[b0], %[b1], [%[b]], #8\n\t"
        "rev16 %[a0], %[a0]\n\t"
        "rev16 %[b0], %[b0]\n\t"
        "eor   %[t],  %[a0], %[b0]\n\t"
        "and   %[a0], %[a0], %[b0]\n\t"
        "and   %[t],  %[m],  %[t], lsr #1\n\t"
        "add   %[a0], %[a0], %[t]\n\t"
        "rev16 %[a0], %[a0]\n\t"
        "rev16 %[a1], %[a1]\n\t"
        "rev16 %[b1], %[b1]\n\t"
        "eor   %[t],  %[a1], %[b1]\n\t"
        "and   %[a1], %[a1], %[b1]\n\t"
        "and   %[t],  %[m],  %[t], lsr #1\n\t"
        "add   %[a1], %[a1], %[t]\n\t"
        "rev16 %[a1], %[a1]\n\t"
        "strd  %[a0], %[a1], [%[d]], #8\n\t"
        "subs  %[n],  %[n],  #1\n\t"
        "bne   1b\n\t"
        : [d] "+r"(d), [a] "+r"(a), [b] "+r"(b), [n] "+r"(n),
          [a0] "=&r"(a0), [a1] "=&r"(a1), [b0] "=&r"(b0), [b1] "=&r"(b1), [t] "=&r"(t)
        : [m] "r"(mask)
        : "cc", "memory"
    );
#else
    uint32_t       *d32 = (uint32_t *)dst;
    const uint32_t *a32 = (const uint32_t *)row0;
    const uint32_t *b32 = (const uint32_t *)row1;
    uint32_t        n   = 80U;

    do
    {
        uint32_t a0 = __REV16(a32[0]);
        uint32_t b0 = __REV16(b32[0]);
        uint32_t a1 = __REV16(a32[1]);
        uint32_t b1 = __REV16(b32[1]);
        uint32_t m0 = (a0 & b0) + (((a0 ^ b0) & 0xF7DEF7DEUL) >> 1);
        uint32_t m1 = (a1 & b1) + (((a1 ^ b1) & 0xF7DEF7DEUL) >> 1);
        d32[0] = __REV16(m0);
        d32[1] = __REV16(m1);
        a32 += 2U;
        b32 += 2U;
        d32 += 2U;
    } while (--n != 0U);
#endif
}

static void __attribute__((optimize("O2"))) vf_prefetch_src_pairs(
    uint8_t *vf_ring, uint16_t req, uint16_t pair_cnt, uint16_t *next_src_yp,
    uint8_t vf_vig, uint8_t center_weight,
    uint32_t *wb_sum_r, uint32_t *wb_sum_g, uint32_t *wb_sum_b, uint32_t *wb_cnt,
    uint32_t *ae_sum_g, uint32_t *ae_cnt, uint32_t *hi_cnt,
    uint16_t lin_bins[16], uint32_t *lin_total)
{
    uint16_t s = *next_src_yp;
    while (s <= req && s < pair_cnt)
    {
        uint8_t *dst_src = vf_ring + (uint32_t)(s & 15U) * 640UL;
        fifo_read_r_320(s_raw_row_buf);
        fifo_skip_4();
        if ((s & 7U) == 0U)
        {
            uint8_t in_center_y = (s >= 30U && s < 90U) ? 1U : 0U;
            raw_demosaic_320_sampled(s_raw_row_buf, dst_src, NULL, 1U,
                                     (vf_vig > 0U) ? (int16_t)s : -1,
                                     in_center_y, center_weight,
                                     wb_sum_r, wb_sum_g, wb_sum_b, wb_cnt,
                                     ae_sum_g, ae_cnt, hi_cnt,
                                     lin_bins, lin_total);
        }
        else if (vf_vig > 0U)
        {
            raw_demosaic_320_to_ring_vig(s_raw_row_buf, dst_src, s);
        }
        else
        {
            raw_demosaic_320_to_ring(s_raw_row_buf, dst_src);
        }
        fifo_skip_4();
        if (s + 1U < pair_cnt)
        {
            fifo_skip_bytes(2UL * 644UL);
        }
        s++;
    }
    *next_src_yp = s;
}

static inline void vsync_reset_idle(void)
{
    __disable_irq();
    OV7725_WEN(0);
    g_vsync_state = 0U;
    __enable_irq();
}

static uint8_t wait_vsync_state(uint8_t target, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (g_vsync_state != target)
    {
        if ((HAL_GetTick() - t0) >= timeout_ms)
        {
            __disable_irq();
            OV7725_WEN(0);
            __enable_irq();
            return 0U;
        }
    }
    return 1U;
}

static void __attribute__((optimize("O2"))) ov7725_camera_refresh(void)
{
    if (g_vsync_state != 2U)
    {
        return;
    }

    /* Hold screen briefly while toast message is visible, keep FIFO cycling */
    if (g_toast_until != 0)
    {
        if ((int32_t)(HAL_GetTick() - g_toast_until) < 0)
        {
            vsync_reset_idle();
            return;
        }
        g_toast_until = 0;
        if (g_cfg.film_lever && !g_shutter_cocked)
        {
            osd_draw_center_box("WIND FILM LEVER", ORANGE);
            g_wind_box_drawn = 1U;
        }
    }

    uint32_t t_paint_start = HAL_GetTick();
    uint8_t  keep_wind_box = (g_cfg.film_lever && !g_shutter_cocked) ? 1U : 0U;

    uint32_t wb_sum_r = 0, wb_sum_g = 0, wb_sum_b = 0, wb_cnt = 0;
    uint32_t ae_sum_g = 0, ae_cnt = 0, hi_cnt = 0, lin_total = 0;
    uint16_t lin_bins[16] = { 0 };
    uint32_t hist_clip = 0, hist_samples = 0;
    uint8_t  hist_en       = g_cfg.hist_en;
    uint8_t  grid_en       = g_cfg.grid_en;
    uint8_t  dbg_en        = g_cfg.debug_info;
    uint8_t  center_weight = (g_cfg.meter_mode == C3M1_METER_CENTER) ? 1U : 0U;
    uint16_t peak_col      = c3m1_cfg_peak_rgb565(&g_cfg);
    uint8_t  buf_idx       = 0U;

    if (hist_en)
    {
        memset(s_hist_bins, 0, sizeof(s_hist_bins));
    }
    if (dbg_en)
    {
        uint16_t dbg_cnt = g_shutter_cocked ? shutter_count_add(ShutterCount, 1U) : ShutterCount;
        if (s_dbg_mask_fps != g_vf_fps || s_dbg_mask_cnt != dbg_cnt)
        {
            vf_dbg_rebuild_mask(g_vf_fps, dbg_cnt);
        }
    }

    fifo_prepare_read();

    if (!g_zoom_4x)
    {
        /*
         * Full 4:3 1X view: 320x120 GRBG LUT demosaic (2x vertical -> full 320x240 uncropped)
         * with inline translucent 20px bottom HUD, overlapped with DMA1 FSMC writes.
         */
        if (!keep_wind_box)
        {
            lcd_set_window(0, 0, 320, VIEWFINDER_HEIGHT);
            lcd_write_ram_prepare();
        }
        else
        {
            lcd_set_window(0, 0, 320, PROMPT_BOX_Y0);
            lcd_write_ram_prepare();
        }

        const uint16_t pair_cnt = VIEWFINDER_HEIGHT / 2U; /* 120 pairs = full 240 LCD rows (480 RAW rows) */
        const int8_t   vf_dist  = g_cfg.lens_dist_incam ? g_cfg.lens_distort : 0;
        const uint8_t  vf_vig   = g_cfg.lens_vignette;
        if (vf_vig > 0U)
        {
            vf_vig_rebuild_lut(vf_vig);
        }

        uint32_t scratch_sz = 0UL;
        uint8_t *scratch    = c3m1_jpeg_get_scratch_buf(&scratch_sz);
        int32_t  vf_kr0_q16 = 65536L, vf_kr1_q16 = 0L;
        uint8_t *vf_ring    = NULL;
        const uint16_t *vf_ring_rows[16];
        uint16_t next_src_yp = 0U;
        if (vf_dist != 0 && scratch != NULL && scratch_sz >= (16UL * 640UL))
        {
            vf_ring = scratch;
            for (uint8_t i = 0U; i < 16U; i++)
            {
                vf_ring_rows[i] = (const uint16_t *)(vf_ring + (uint32_t)i * 640UL);
            }
            c3m1_lens_dist_params_q16(vf_dist, &vf_kr0_q16, &vf_kr1_q16);
            /* Pre-read initial source rows needed for yp = 0 (overlaps nothing yet) */
            vf_prefetch_src_pairs(vf_ring, vf_req_max_src_yp(0U, vf_kr0_q16, vf_kr1_q16),
                                  pair_cnt, &next_src_yp, vf_vig, center_weight,
                                  &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                  &ae_sum_g, &ae_cnt, &hi_cnt, lin_bins, &lin_total);
        }

        if (vf_ring == NULL)
        {
            /* Pre-read yp=0 R bytes so subsequent pairs overlap fifo_read_r_320 with Row y0 DMA */
            fifo_read_r_320(s_raw_row_buf);
            fifo_skip_4();
        }

        /* Unified U1 anti-aliasing (zero-memcpy in-place y0 DMA + balanced y0/y1 DMA overlap, 0 B .bss):
         * Borrows 2x640 B clean-row buffers + 320 B 3rd G6 peaking slot past vf_ring (scratch[10240..11839]).
         * While prev y1 DMA streams from line1, CPU computes row[yp+1] -> blends y1 into line1 -> modifies
         * cur_clean in-place for y0 (zero memcpy!) -> streams y0 from cur_clean while prefetching yp+2. */
        const uint8_t aa_en = (g_cfg.anti_alias && scratch != NULL &&
                               scratch_sz >= (18UL * 640UL + 320UL)) ? 1U : 0U;

        if (aa_en)
        {
            uint8_t *aa_clean[2] = { scratch + (16UL * 640UL), scratch + (17UL * 640UL) };
            uint8_t *peak_g[3]   = { s_peak_g[0], s_peak_g[1], scratch + (18UL * 640UL) };
            uint8_t *line1       = s_line_buf[1];
            uint8_t  g_prev      = 2U;
            uint8_t  g_cur       = 0U;
            uint8_t  g_next      = 1U;

            /* Produce clean row yp=0 into aa_clean[0] + peak_g[0], then prefetch FIFO for yp=1 */
            if (vf_ring != NULL)
            {
                vf_warp_row_1x(aa_clean[0], peak_g[0], vf_ring_rows, 0U, vf_kr0_q16, vf_kr1_q16);
                vf_prefetch_src_pairs(vf_ring, vf_req_max_src_yp(1U, vf_kr0_q16, vf_kr1_q16),
                                      pair_cnt, &next_src_yp, vf_vig, center_weight,
                                      &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                      &ae_sum_g, &ae_cnt, &hi_cnt, lin_bins, &lin_total);
            }
            else
            {
                raw_demosaic_320_sampled(s_raw_row_buf, aa_clean[0], peak_g[0], 0U,
                                         (vf_vig > 0U) ? 0 : -1,
                                         0U, center_weight,
                                         &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                         &ae_sum_g, &ae_cnt, &hi_cnt,
                                         lin_bins, &lin_total);
                fifo_skip_4();
                fifo_skip_bytes(2UL * 644UL);
                fifo_read_r_320(s_raw_row_buf);
                fifo_skip_4();
            }
            if (hist_en)
            {
                vf_hist_sample_row(aa_clean[0], s_hist_bins, &hist_clip, &hist_samples);
            }

            for (uint16_t yp = 0U; yp < pair_cnt; yp++)
            {
                uint16_t y0         = (uint16_t)(yp * 2U);
                uint16_t y1         = (uint16_t)(y0 + 1U);
                uint8_t *cur_clean  = aa_clean[yp & 1U];
                uint8_t *next_clean = aa_clean[(yp ^ 1U) & 1U];

                uint8_t in_top_hud = (y0 >= TOP_HUD_Y0 && y1 <= TOP_HUD_Y1) ? 1U : 0U;
                uint8_t in_hist    = (hist_en && y0 >= HIST_Y0 && y1 <= HIST_Y1) ? 1U : 0U;
                uint8_t in_dbg     = (dbg_en  && y0 >= DBG_Y0  && y1 <= DBG_Y1)  ? 1U : 0U;
                uint8_t need_hgrid = (grid_en && (y1 == 79U || y1 == 159U)) ? 1U : 0U;
                uint8_t need_ch    = (center_weight && (y1 == 61U || y1 == 119U || y1 == 179U)) ? 1U : 0U;
                uint8_t in_wind    = (keep_wind_box && y1 >= PROMPT_BOX_Y0 && y0 <= PROMPT_BOX_Y1) ? 1U : 0U;

                /* Step A (runs IN PARALLEL with prev y1 DMA in line1): produce clean row yp+1 into next_clean */
                if (yp + 1U < pair_cnt)
                {
                    uint16_t nyp = (uint16_t)(yp + 1U);
                    if (vf_ring != NULL)
                    {
                        vf_warp_row_1x(next_clean, ((uint16_t)(nyp * 2U) < OSD_BAR_Y) ? peak_g[g_next] : NULL,
                                       vf_ring_rows, nyp, vf_kr0_q16, vf_kr1_q16);
                    }
                    else
                    {
                        if ((nyp & 7U) == 0U)
                        {
                            uint8_t in_center_y = (nyp >= 30U && nyp < 90U) ? 1U : 0U;
                            raw_demosaic_320_sampled(s_raw_row_buf, next_clean, peak_g[g_next], 0U,
                                                     (vf_vig > 0U) ? (int16_t)nyp : -1,
                                                     in_center_y, center_weight,
                                                     &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                                     &ae_sum_g, &ae_cnt, &hi_cnt,
                                                     lin_bins, &lin_total);
                        }
                        else if (vf_vig > 0U)
                        {
                            raw_demosaic_320_to_buf_vig(s_raw_row_buf, next_clean, peak_g[g_next], nyp);
                        }
                        else
                        {
                            raw_demosaic_320_to_buf(s_raw_row_buf, next_clean, peak_g[g_next]);
                        }
                        fifo_skip_4();
                    }
                    if (hist_en && (nyp & 7U) == 0U)
                    {
                        vf_hist_sample_row(next_clean, s_hist_bins, &hist_clip, &hist_samples);
                    }
                }

                /* Blend y1 into line1 BEFORE cur_clean is overlaid in-place (prev y1 DMA already finished) */
                lcd_dma_wait();
                if (y0 < OSD_BAR_Y && (yp + 1U) < pair_cnt)
                {
                    vf_avg_row_be(line1, cur_clean, next_clean);
                }
                else
                {
                    /* Bottom 20px darkened HUD band: skip blend (invisible under 25% darkened text bar) */
                    memcpy(line1, cur_clean, 640U);
                }

                /* Overlay y0 IN-PLACE on cur_clean (ZERO memcpy!) and launch y0 DMA directly from cur_clean */
                if (y0 < OSD_BAR_Y)
                {
                    vf_apply_focus_peaking(cur_clean, peak_g[g_cur], peak_g[g_prev], (yp > 0U) ? 1U : 0U, 0U, peak_col);
                    if (grid_en)       vf_overlay_grid_v(cur_clean);
                    if (center_weight) vf_overlay_center_v(cur_clean, y0);
                    if (in_top_hud)    vf_overlay_top_hud_row(cur_clean, y0);
                    if (in_hist)       vf_overlay_hist_row(cur_clean, y0);
                    if (in_dbg)        vf_overlay_dbg_row(cur_clean, y0);
                }
                else
                {
                    vf_overlay_osd_row(cur_clean, y0);
                }

                if (in_wind)
                {
                    if (y0 >= PROMPT_BOX_Y0 && y0 <= PROMPT_BOX_Y1)
                    {
                        lcd_send_wind_strips(cur_clean, y0);
                    }
                    else
                    {
                        lcd_dma_send(cur_clean, 640U);
                        lcd_dma_wait();
                    }
                }
                else
                {
                    lcd_dma_send(cur_clean, 640U);
                }

                /* Step B (runs IN PARALLEL with y0 DMA in cur_clean): y1 overlays + FIFO prefetch for yp+2 */
                if (y0 < OSD_BAR_Y)
                {
                    if (grid_en)       vf_overlay_grid_v(line1);
                    if (need_hgrid)    vf_overlay_grid_h(line1);
                    if (center_weight) vf_overlay_center_v(line1, y1);
                    if (need_ch)       vf_overlay_center_h(line1, y1);
                    if (in_top_hud)    vf_overlay_top_hud_row(line1, y1);
                    if (in_hist)       vf_overlay_hist_row(line1, y1);
                    if (in_dbg)        vf_overlay_dbg_row(line1, y1);
                }
                else
                {
                    vf_overlay_osd_row(line1, y1);
                }

                if ((yp + 2U) < pair_cnt)
                {
                    if (vf_ring != NULL)
                    {
                        vf_prefetch_src_pairs(vf_ring,
                                              vf_req_max_src_yp((uint16_t)(yp + 2U), vf_kr0_q16, vf_kr1_q16),
                                              pair_cnt, &next_src_yp, vf_vig, center_weight,
                                              &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                              &ae_sum_g, &ae_cnt, &hi_cnt, lin_bins, &lin_total);
                    }
                    else
                    {
                        fifo_skip_bytes(2UL * 644UL);
                        fifo_read_r_320(s_raw_row_buf);
                        fifo_skip_4();
                    }
                }

                if (in_wind)
                {
                    if (y1 >= PROMPT_BOX_Y0 && y1 <= PROMPT_BOX_Y1)
                    {
                        lcd_send_wind_strips(line1, y1);
                    }
                    else
                    {
                        lcd_dma_wait();
                        lcd_set_window(0, y1, 320, VIEWFINDER_HEIGHT - y1);
                        lcd_write_ram_prepare();
                        lcd_dma_send(line1, 640U);
                    }
                }
                else
                {
                    lcd_dma_send(line1, 640U);
                }

                /* Rotate 3-slot peaking G6 indices (zero modulo) */
                g_prev = g_cur;
                g_cur  = g_next;
                g_next = (g_next == 2U) ? 0U : (uint8_t)(g_next + 1U);

                if ((yp & 3U) == 3U && !keep_wind_box && key_is_shot_pressed())
                {
                    uint8_t se = key_poll_events();
                    g_pending_btn_evts |= se;
                    if (se & BTN_EVT_SHOT)
                    {
                        __disable_irq();
                        if (g_vsync_state == 2U)
                        {
                            g_vsync_state = 0U;
                        }
                        __enable_irq();
                        lcd_dma_wait();
                        return;
                    }
                }
                if ((yp & 15U) == 15U)
                {
                    g_pending_btn_evts |= key_poll_events();
                    (void)key_read_shutter_dial();
                }
            }
        }
        else
        for (uint16_t yp = 0; yp < pair_cnt; yp++)
        {
            uint16_t y0 = (uint16_t)(yp * 2U);
            uint16_t y1 = (uint16_t)(y0 + 1U);
            uint8_t *cur_line = s_line_buf[buf_idx];

            uint8_t *cur_g  = s_peak_g[yp & 1U];
            uint8_t *prev_g = s_peak_g[(yp ^ 1U) & 1U];

            if (vf_ring != NULL)
            {
                /* Warped 1X path: runs IN PARALLEL while DMA1 finishes transmitting previous Row y1 */
                vf_warp_row_1x(cur_line, (y0 < OSD_BAR_Y) ? cur_g : NULL,
                               vf_ring_rows, yp, vf_kr0_q16, vf_kr1_q16);
                if (hist_en && (yp & 7U) == 0U)
                {
                    vf_hist_sample_row(cur_line, s_hist_bins, &hist_clip, &hist_samples);
                }
            }
            else
            {
                /* Unwarped fast path: s_raw_row_buf was prefetched during previous Row y0 DMA */
                if ((yp & 7U) == 0U)
                {
                    uint8_t in_center_y = (yp >= 30U && yp < 90U) ? 1U : 0U;
                    raw_demosaic_320_sampled(s_raw_row_buf, cur_line, cur_g, 0U,
                                             (vf_vig > 0U) ? (int16_t)yp : -1,
                                             in_center_y, center_weight,
                                             &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                             &ae_sum_g, &ae_cnt, &hi_cnt,
                                             lin_bins, &lin_total);
                }
                else if (vf_vig > 0U)
                {
                    raw_demosaic_320_to_buf_vig(s_raw_row_buf, cur_line, cur_g, yp);
                }
                else
                {
                    raw_demosaic_320_to_buf(s_raw_row_buf, cur_line, cur_g);
                }
                fifo_skip_4();
                if (hist_en && (yp & 7U) == 0U)
                {
                    vf_hist_sample_row(cur_line, s_hist_bins, &hist_clip, &hist_samples);
                }
            }

            /* Always-on Sony-style 1px focus peaking (skip inside bottom HUD for cleaner look & speed) */
            if (y0 < OSD_BAR_Y)
            {
                vf_apply_focus_peaking(cur_line, cur_g, prev_g, (yp > 0U) ? 1U : 0U, 0U, peak_col);
                if (grid_en)
                {
                    vf_overlay_grid_v(cur_line);
                }
                if (center_weight)
                {
                    vf_overlay_center_v(cur_line, y0);
                }
            }

            if (y0 >= OSD_BAR_Y)
            {
                /* Translucent 20px bottom HUD band (y = 220..239): composite y0 & y1 independently */
                uint8_t *alt_line = s_line_buf[buf_idx ^ 1U];
                lcd_dma_wait();
                memcpy(alt_line, cur_line, 640U);
                vf_overlay_osd_row(cur_line, y0);
                lcd_dma_send(cur_line, 640U);
                vf_overlay_osd_row(alt_line, y1);
                if (yp + 1U < pair_cnt)
                {
                    if (vf_ring != NULL)
                    {
                        vf_prefetch_src_pairs(vf_ring,
                                              vf_req_max_src_yp((uint16_t)(yp + 1U), vf_kr0_q16, vf_kr1_q16),
                                              pair_cnt, &next_src_yp, vf_vig, center_weight,
                                              &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                              &ae_sum_g, &ae_cnt, &hi_cnt, lin_bins, &lin_total);
                    }
                    else
                    {
                        fifo_skip_bytes(2UL * 644UL);
                        fifo_read_r_320(s_raw_row_buf);
                        fifo_skip_4();
                    }
                }
                lcd_dma_send(alt_line, 640U);
                /* Keep cur_line free (do not flip buf_idx) while alt_line completes DMA */
            }
            else if (!keep_wind_box || y1 < PROMPT_BOX_Y0 || y0 > PROMPT_BOX_Y1)
            {
                uint8_t in_top_hud = (y0 >= TOP_HUD_Y0 && y1 <= TOP_HUD_Y1) ? 1U : 0U;
                uint8_t in_hist    = (hist_en && y0 >= HIST_Y0 && y1 <= HIST_Y1) ? 1U : 0U;
                uint8_t in_dbg     = (dbg_en  && y0 >= DBG_Y0  && y1 <= DBG_Y1)  ? 1U : 0U;
                uint8_t need_hgrid = (grid_en && (y1 == 79U || y1 == 159U)) ? 1U : 0U;
                uint8_t need_ch    = (center_weight && (y1 == 61U || y1 == 119U || y1 == 179U)) ? 1U : 0U;
                uint8_t top_save[TOP_HUD_W * 2U];

                if (in_top_hud)
                {
                    memcpy(top_save, &cur_line[TOP_HUD_X0 * 2U], sizeof(top_save));
                    vf_overlay_top_hud_row(cur_line, y0);
                }
                if (in_hist)
                {
                    vf_overlay_hist_row(cur_line, y0);
                }
                if (in_dbg)
                {
                    vf_overlay_dbg_row(cur_line, y0);
                }

                /* Full-width fast DMA pipeline: send Row y0, prefetch next FIFO skip+R in parallel, send Row y1 */
                lcd_dma_send(cur_line, 640U);
                if (yp + 1U < pair_cnt)
                {
                    if (vf_ring != NULL)
                    {
                        vf_prefetch_src_pairs(vf_ring,
                                              vf_req_max_src_yp((uint16_t)(yp + 1U), vf_kr0_q16, vf_kr1_q16),
                                              pair_cnt, &next_src_yp, vf_vig, center_weight,
                                              &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                              &ae_sum_g, &ae_cnt, &hi_cnt, lin_bins, &lin_total);
                    }
                    else
                    {
                        fifo_skip_bytes(2UL * 644UL);
                        fifo_read_r_320(s_raw_row_buf);
                        fifo_skip_4();
                    }
                }
                if (in_top_hud || need_hgrid || need_ch || in_hist || in_dbg)
                {
                    lcd_dma_wait();
                    if (in_top_hud)
                    {
                        memcpy(&cur_line[TOP_HUD_X0 * 2U], top_save, sizeof(top_save));
                        vf_overlay_top_hud_row(cur_line, y1);
                    }
                    if (need_hgrid) vf_overlay_grid_h(cur_line);
                    if (need_ch)    vf_overlay_center_h(cur_line, y1);
                    if (in_hist)    vf_overlay_hist_row(cur_line, y1);
                    if (in_dbg)     vf_overlay_dbg_row(cur_line, y1);
                }
                lcd_dma_send(cur_line, 640U);
                buf_idx ^= 1U;
            }
            else
            {
                /* WIND box band (y = 92..132): send left/right strips cleanly without flicker */
                if (y0 >= PROMPT_BOX_Y0 && y0 <= PROMPT_BOX_Y1)
                {
                    lcd_send_wind_strips(cur_line, y0);
                }
                else
                {
                    lcd_dma_send(cur_line, 640U);
                    lcd_dma_wait();
                }

                if (y1 >= PROMPT_BOX_Y0 && y1 <= PROMPT_BOX_Y1)
                {
                    lcd_send_wind_strips(cur_line, y1);
                }
                else
                {
                    /* y1 == PROMPT_BOX_Y1 + 1 (133): open bottom full-width LCD window */
                    lcd_dma_wait();
                    lcd_set_window(0, y1, 320, VIEWFINDER_HEIGHT - y1);
                    lcd_write_ram_prepare();
                    lcd_dma_send(cur_line, 640U);
                }

                if (yp + 1U < pair_cnt)
                {
                    if (vf_ring != NULL)
                    {
                        vf_prefetch_src_pairs(vf_ring,
                                              vf_req_max_src_yp((uint16_t)(yp + 1U), vf_kr0_q16, vf_kr1_q16),
                                              pair_cnt, &next_src_yp, vf_vig, center_weight,
                                              &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                              &ae_sum_g, &ae_cnt, &hi_cnt, lin_bins, &lin_total);
                    }
                    else
                    {
                        fifo_skip_bytes(2UL * 644UL);
                        fifo_read_r_320(s_raw_row_buf);
                        fifo_skip_4();
                    }
                }
                buf_idx ^= 1U;
            }

            if ((yp & 3U) == 3U && !keep_wind_box && key_is_shot_pressed())
            {
                /* Fast-path shutter check every ~2 ms: keep in-flight frame (state==1) or re-arm (state==2->0) */
                uint8_t se = key_poll_events();
                g_pending_btn_evts |= se;
                if (se & BTN_EVT_SHOT)
                {
                    __disable_irq();
                    if (g_vsync_state == 2U)
                    {
                        g_vsync_state = 0U;
                    }
                    __enable_irq();
                    lcd_dma_wait();
                    return;
                }
            }
            if ((yp & 15U) == 15U)
            {
                /* Poll buttons & dial every ~8 ms mid-frame so short taps & dial stay crisp */
                g_pending_btn_evts |= key_poll_events();
                (void)key_read_shutter_dial();
            }
        }
        lcd_dma_wait();
    }
    else
    {
        /* 4X focus magnify: native 1:1 center 320x240 crop (160..479, 120..359), DMA-pipelined */
        uint8_t *row_a = s_raw_row_buf;         /* 322 bytes (4-byte aligned, 324B stride) */
        uint8_t *row_b = s_raw_row_buf + 324U;  /* 322 bytes (4-byte aligned, 324B stride) */

        /* Skip top 120 raw rows + 160 left margin cols in one burst (even rows preserve GRBG phase) */
        fifo_skip_bytes(120UL * 644UL + 160UL);

        /* Pre-read raw row 120 (cols 160..481 = 322 bytes) */
        fifo_read_bytes(row_a, 322U);

        if (!keep_wind_box)
        {
            lcd_set_window(0, 0, 320, VIEWFINDER_HEIGHT);
            lcd_write_ram_prepare();
        }
        else
        {
            lcd_set_window(0, 0, 320, PROMPT_BOX_Y0);
            lcd_write_ram_prepare();
        }

        for (uint16_t y = 0; y < VIEWFINDER_HEIGHT; y++)
        {
            uint8_t *next_buf       = (y & 1U) ? row_a : row_b;
            const uint8_t *even_row = (y & 1U) ? next_buf : row_a;
            const uint8_t *odd_row  = (y & 1U) ? row_b    : next_buf;
            uint8_t *cur_line       = s_line_buf[buf_idx];

            /* Skip 162 right cols of prev row + 160 left cols of next row (322 B) & read 322 B while DMA runs */
            fifo_skip_bytes(322UL);
            fifo_read_bytes(next_buf, 322U);

            if ((y & 7U) == 0U)
            {
                uint16_t bl_g  = c3m1_black_level_g12(g_lut_last_gain_x);
                uint16_t bl_rb = c3m1_black_level_rb12(g_lut_last_gain_x);
                uint8_t in_cy  = (y >= 60U && y < 180U) ? 1U : 0U;
                for (uint16_t x = 0; x < 320U; x += 4U)
                {
                    uint16_t g12 = (uint16_t)(((uint32_t)s_inv_gamma_12[even_row[x]] +
                                               (uint32_t)s_inv_gamma_12[odd_row[x + 1U]]) >> 1);
                    uint16_t r12 = s_inv_gamma_12[even_row[x + 1U]];
                    uint16_t b12 = s_inv_gamma_12[odd_row[x]];
                    uint8_t gl = (uint8_t)((g12 > bl_g)  ? ((g12 - bl_g)  >> 4) : 0U);
                    uint8_t rl = (uint8_t)((r12 > bl_rb) ? ((r12 - bl_rb) >> 4) : 0U);
                    uint8_t bl = (uint8_t)((b12 > bl_rb) ? ((b12 - bl_rb) >> 4) : 0U);
                    uint8_t mx = rl;
                    if (gl > mx) mx = gl;
                    if (bl > mx) mx = bl;
                    lin_bins[mx >> 4]++;
                    lin_total++;

                    uint32_t w = (center_weight && in_cy && x >= 80U && x < 240U) ? 6UL : 1UL;
                    ae_sum_g += (uint32_t)gl * w;
                    ae_cnt   += w;
                    if (r12 >= 3920U || g12 >= 3920U || b12 >= 3920U)
                    {
                        hi_cnt++;
                    }
                    else if (rl >= 8U && gl >= 8U && bl >= 8U)
                    {
                        wb_sum_r += rl;
                        wb_sum_g += gl;
                        wb_sum_b += bl;
                        wb_cnt++;
                    }
                }
            }

            uint8_t *cur_g  = s_peak_g[y & 1U];
            uint8_t *prev_g = s_peak_g[(y ^ 1U) & 1U];
            raw_demosaic_1x1_to_buf(even_row, odd_row, cur_line, cur_g);
            if (hist_en && (y & 7U) == 0U)
            {
                vf_hist_sample_row(cur_line, s_hist_bins, &hist_clip, &hist_samples);
            }
            if (y < OSD_BAR_Y)
            {
                vf_apply_focus_peaking(cur_line, cur_g, prev_g, (y > 0U) ? 1U : 0U, 1U, peak_col);
                if (center_weight)
                {
                    vf_overlay_center_v(cur_line, y);
                    vf_overlay_center_h(cur_line, y);
                }
            }
            if (y >= TOP_HUD_Y0 && y <= TOP_HUD_Y1)
            {
                vf_overlay_top_hud_row(cur_line, y);
            }
            if (hist_en && y >= HIST_Y0 && y <= HIST_Y1)
            {
                vf_overlay_hist_row(cur_line, y);
            }
            if (dbg_en && y >= DBG_Y0 && y <= DBG_Y1)
            {
                vf_overlay_dbg_row(cur_line, y);
            }
            if (y >= OSD_BAR_Y)
            {
                vf_overlay_osd_row(cur_line, y);
            }

            if (keep_wind_box && y >= PROMPT_BOX_Y0 && y <= PROMPT_BOX_Y1)
            {
                lcd_send_wind_strips(cur_line, y);
            }
            else
            {
                if (keep_wind_box && y == (PROMPT_BOX_Y1 + 1U))
                {
                    lcd_dma_wait();
                    lcd_set_window(0, y, 320, VIEWFINDER_HEIGHT - y);
                    lcd_write_ram_prepare();
                }
                lcd_dma_send(cur_line, 640U);
            }

            buf_idx ^= 1U;
            if ((y & 7U) == 7U && !keep_wind_box && key_is_shot_pressed())
            {
                /* Fast-path shutter check every ~3 ms in 4X: keep in-flight frame (state==1) or re-arm (state==2->0) */
                uint8_t se = key_poll_events();
                g_pending_btn_evts |= se;
                if (se & BTN_EVT_SHOT)
                {
                    __disable_irq();
                    if (g_vsync_state == 2U)
                    {
                        g_vsync_state = 0U;
                    }
                    __enable_irq();
                    lcd_dma_wait();
                    return;
                }
            }
            if ((y & 15U) == 15U)
            {
                /* Poll buttons & dial every ~6 ms during 4X paint (prevents dial stall & dropped taps) */
                g_pending_btn_evts |= key_poll_events();
                (void)key_read_shutter_dial();
            }
        }
        lcd_dma_wait();
    }

    if (hist_en)
    {
        vf_hist_finalize(s_hist_bins, hist_clip, hist_samples);
    }

    /* Run AWB/AE BEFORE releasing g_vsync_state so SCCB writes never collide with active FIFO recording */
    raw_preview_update_ae_awb(wb_sum_r, wb_sum_g, wb_sum_b, wb_cnt,
                              ae_sum_g, ae_cnt, hi_cnt, lin_bins, lin_total);

    vsync_reset_idle();
    g_vf_last_paint_ms = (uint16_t)(HAL_GetTick() - t_paint_start);

    if (g_exposure_dirty_frames > 0U)
    {
        g_exposure_dirty_frames--;
    }

    /* Measure real live viewfinder FPS once per second */
    g_vf_frame_cnt++;
    uint32_t now_tick = HAL_GetTick();
    if (g_vf_fps_tick == 0U)
    {
        g_vf_fps_tick = now_tick;
    }
    else if ((now_tick - g_vf_fps_tick) >= 1000U)
    {
        uint8_t new_fps = (g_vf_frame_cnt > 99U) ? 99U : (uint8_t)g_vf_frame_cnt;
        g_vf_frame_cnt = 0U;
        g_vf_fps_tick  = now_tick;
        if (new_fps != g_vf_fps)
        {
            g_vf_fps = new_fps;
        }
#ifdef C3M1_VF_UART_LOG
        printf("VF:%ufps paint:%ums aec:%u wb:%u/%u\r\n",
               (unsigned int)g_vf_fps, (unsigned int)g_vf_last_paint_ms,
               (unsigned int)g_auto_aec, (unsigned int)g_wb_r_q8, (unsigned int)g_wb_b_q8);
#else
        (void)g_vf_last_paint_ms;
#endif
    }
}

/* ======================== RAW DNG Capture ======================== */
static void wait_skip_frames(uint8_t count)
{
    RAW_shot = 0;
    while (RAW_shot < count)
    {
        if (!wait_vsync_state(2U, 250UL))
        {
            vsync_reset_idle();
            break;
        }
        vsync_reset_idle();
        RAW_shot++;
    }
}

static void led1_blink_error(void)
{
    for (uint8_t k = 0; k < 3U; k++)
    {
        LED1(0);
        delay_ms(80);
        LED1(1);
        delay_ms(80);
    }
}

typedef struct
{
    FIL      *fp;
    uint32_t *last_led_tick;
} jpg_sd_io_t;

static void jpg_fifo_read_row(void *user, uint8_t *row644)
{
    (void)user;
    fifo_read_bytes(row644, 644U);
}

static uint8_t jpg_sd_write(void *user, const uint8_t *data, uint32_t len)
{
    jpg_sd_io_t *io = (jpg_sd_io_t *)user;
    UINT written = 0;
    FRESULT fres = f_write(io->fp, data, (UINT)len, &written);
    if (fres != FR_OK || written != (UINT)len)
    {
        printf("JPG write err:%d\r\n", fres);
        return 0U;
    }
    if ((HAL_GetTick() - *io->last_led_tick) >= 80U)
    {
        LED1_TOGGLE();
        *io->last_led_tick = HAL_GetTick();
    }
    return 1U;
}

static void camera_lines_to_exp_rational(uint32_t fast_lines, uint32_t *out_num, uint32_t *out_den)
{
    if (fast_lines < 1UL) fast_lines = 1UL;
    if (fast_lines >= 22959UL)
    {
        uint32_t tenths = (fast_lines * 10UL + 11479UL) / 22959UL;
        if ((tenths % 10UL) == 0UL)
        {
            *out_num = tenths / 10UL;
            *out_den = 1UL;
        }
        else
        {
            *out_num = tenths;
            *out_den = 10UL;
        }
        return;
    }

    *out_num = 1UL;
    if (fast_lines == 1UL)
    {
        *out_den = 23000UL;
        return;
    }
    if (fast_lines == 2UL)
    {
        *out_den = 11500UL;
        return;
    }
    uint32_t denom = (22959UL + (fast_lines >> 1)) / fast_lines;
    if (denom >= 1000UL)     denom = ((denom + 50UL) / 100UL) * 100UL;
    else if (denom >= 100UL) denom = ((denom + 5UL) / 10UL) * 10UL;
    if (denom < 1UL) denom = 1UL;
    *out_den = denom;
}

static uint16_t camera_get_capture_iso(void)
{
    if (g_iso_idx > 0U && g_iso_idx < OV7725_ISO_COUNT)
    {
        return (uint16_t)(100U << (g_iso_idx - 1U));
    }
    static const uint16_t s_iso_steps[16] = {
        100U, 125U, 160U, 200U, 250U, 320U, 400U, 500U,
        640U, 800U, 1000U, 1250U, 1600U, 2000U, 2500U, 3200U
    };
    uint32_t iso = ((uint32_t)g_auto_gain_q4 * 100UL + 8UL) / 16UL;
    uint16_t best = s_iso_steps[0];
    uint32_t best_diff = 10000UL;
    for (uint8_t i = 0U; i < 16U; i++)
    {
        uint32_t d = (iso > s_iso_steps[i]) ? (iso - s_iso_steps[i]) : (s_iso_steps[i] - iso);
        if (d < best_diff)
        {
            best_diff = d;
            best = s_iso_steps[i];
        }
    }
    return best;
}

static void camera_build_capture_exif(uint32_t exact_fast_lines, c3m1_dng_exif_t *out)
{
    static const uint16_t s_man_num[15] = {
        1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 2U
    };
    static const uint16_t s_man_den[15] = {
        8000U, 4000U, 2000U, 1000U, 500U, 300U, 150U, 100U,
        50U, 25U, 8U, 4U, 2U, 1U, 1U
    };

    if (exact_fast_lines > 0UL)
    {
        camera_lines_to_exp_rational(exact_fast_lines, &out->exp_num, &out->exp_den);
    }
    else if (!ov7725_is_auto_shutter(g_dial_pos) && g_dial_pos < 15U)
    {
        out->exp_num = s_man_num[g_dial_pos];
        out->exp_den = s_man_den[g_dial_pos];
    }
    else
    {
        camera_lines_to_exp_rational((uint32_t)g_auto_aec, &out->exp_num, &out->exp_den);
    }

    out->iso        = camera_get_capture_iso();
    out->ev_num     = (int32_t)g_cfg.ev_comp - (int32_t)C3M1_EV_COMP_ZERO;
    out->ev_den     = 3;
    out->fnum_x10   = g_cfg.lens_fnum_x10;
    out->focal_x10  = (uint16_t)(g_cfg.lens_focal_mm * 10U);
    out->focal_eq35 = c3m1_cfg_lens_focal_eq35(g_cfg.lens_focal_mm);
    out->lens_model = c3m1_cfg_lens_profile_exif_name(g_cfg.lens_profile);
    out->color_mode = g_cfg.color_mode;
}

static void camera_get_dng_lens(c3m1_dng_lens_t *out)
{
    out->distort  = g_cfg.lens_distort;
    out->vignette = g_cfg.lens_vignette;
}

static uint16_t thumb_req_max_src_ty(uint16_t ty, int32_t kr0_q16, int32_t kr1_q16)
{
    static const uint16_t s_chk_x[3] = { 2U, 322U, 638U };
    uint16_t py = (uint16_t)(ty * 4U + 2U);
    uint16_t req = 0U;
    for (uint8_t k = 0U; k < 3U; k++)
    {
        uint16_t sx_q4, sy_q4;
        c3m1_lens_warp_vga_q4(s_chk_x[k], py, kr0_q16, kr1_q16, &sx_q4, &sy_q4);
        uint16_t sty0 = (uint16_t)(sy_q4 >> 6);
        if (sty0 >= C3M1_DNG_THUMB_H) sty0 = C3M1_DNG_THUMB_H - 1U;
        uint16_t sty1 = (sty0 + 1U < C3M1_DNG_THUMB_H) ? (uint16_t)(sty0 + 1U) : sty0;
        if (sty1 > req) req = sty1;
    }
    return req;
}

static void __attribute__((optimize("O2"))) thumb_warp_row_rgb888(uint8_t *dst_rgb480, const uint8_t *ring16,
                                                                  uint16_t ty, int32_t kr0_q16, int32_t kr1_q16)
{
    uint16_t py = (uint16_t)(ty * 4U + 2U);
    for (uint16_t tx = 0U; tx < C3M1_DNG_THUMB_W; tx++)
    {
        uint16_t sx_q4, sy_q4;
        c3m1_lens_warp_vga_q4((uint16_t)(tx * 4U + 2U), py, kr0_q16, kr1_q16, &sx_q4, &sy_q4);
        uint16_t stx_q4 = (uint16_t)(sx_q4 >> 2);
        uint16_t sty_q4 = (uint16_t)(sy_q4 >> 2);
        uint16_t stx0 = (uint16_t)(stx_q4 >> 4);
        uint16_t sty0 = (uint16_t)(sty_q4 >> 4);
        if (stx0 >= C3M1_DNG_THUMB_W) stx0 = C3M1_DNG_THUMB_W - 1U;
        if (sty0 >= C3M1_DNG_THUMB_H) sty0 = C3M1_DNG_THUMB_H - 1U;
        uint16_t stx1 = (stx0 + 1U < C3M1_DNG_THUMB_W) ? (uint16_t)(stx0 + 1U) : stx0;
        uint16_t sty1 = (sty0 + 1U < C3M1_DNG_THUMB_H) ? (uint16_t)(sty0 + 1U) : sty0;
        uint8_t  fx   = (uint8_t)(stx_q4 & 0x0FU);
        uint8_t  fy   = (uint8_t)(sty_q4 & 0x0FU);

        uint32_t w00 = (uint32_t)(16U - fx) * (uint32_t)(16U - fy);
        uint32_t w10 = (uint32_t)fx         * (uint32_t)(16U - fy);
        uint32_t w01 = (uint32_t)(16U - fx) * (uint32_t)fy;
        uint32_t w11 = (uint32_t)fx         * (uint32_t)fy;

        const uint8_t *r0  = ring16 + (uint32_t)(sty0 & 15U) * (C3M1_DNG_THUMB_W * 3U);
        const uint8_t *r1  = ring16 + (uint32_t)(sty1 & 15U) * (C3M1_DNG_THUMB_W * 3U);
        const uint8_t *p00 = &r0[stx0 * 3U];
        const uint8_t *p10 = &r0[stx1 * 3U];
        const uint8_t *p01 = &r1[stx0 * 3U];
        const uint8_t *p11 = &r1[stx1 * 3U];

        dst_rgb480[tx * 3U + 0U] = (uint8_t)((w00 * p00[0] + w10 * p10[0] + w01 * p01[0] + w11 * p11[0] + 128UL) >> 8);
        dst_rgb480[tx * 3U + 1U] = (uint8_t)((w00 * p00[1] + w10 * p10[1] + w01 * p01[1] + w11 * p11[1] + 128UL) >> 8);
        dst_rgb480[tx * 3U + 2U] = (uint8_t)((w00 * p00[2] + w10 * p10[2] + w01 * p01[2] + w11 * p11[2] + 128UL) >> 8);
    }
}

static void __attribute__((optimize("O2"))) dng_decode_thumb_src_row(uint8_t *dst_rgb480, uint16_t src_ty,
                                                                     uint8_t *even_row0, uint8_t *odd_row0,
                                                                     uint8_t *even_row1, uint8_t *odd_row1,
                                                                     uint16_t bl_g, uint16_t bl_rb, uint32_t vig_scale)
{
    const uint8_t use_box4 = (even_row1 != NULL && odd_row1 != NULL) ? 1U : 0U;
    fifo_read_bytes(even_row0, 644U);
    fifo_read_bytes(odd_row0,  644U);
    if (use_box4)
    {
        fifo_read_bytes(even_row1, 644U);
        fifo_read_bytes(odd_row1,  644U);
    }
    else if (src_ty + 1U < C3M1_DNG_THUMB_H)
    {
        fifo_skip_bytes(2UL * 644UL);
    }

    uint16_t py = (uint16_t)(src_ty * 4U + (use_box4 ? 2U : 0U));
    for (uint16_t tx = 0U; tx < C3M1_DNG_THUMB_W; tx++)
    {
        uint16_t x = (uint16_t)(tx * 4U);
        uint16_t r12, g12, b12;
        if (use_box4)
        {
            /* Full 4x4 linear-light box average (4R + 8G + 4B GRBG quads): eliminates 1X playback moire & aliasing */
            r12 = (uint16_t)(((uint32_t)s_inv_gamma_12[even_row0[x + 1U]] +
                              (uint32_t)s_inv_gamma_12[even_row0[x + 3U]] +
                              (uint32_t)s_inv_gamma_12[even_row1[x + 1U]] +
                              (uint32_t)s_inv_gamma_12[even_row1[x + 3U]] + 2UL) >> 2);
            b12 = (uint16_t)(((uint32_t)s_inv_gamma_12[odd_row0[x + 0U]] +
                              (uint32_t)s_inv_gamma_12[odd_row0[x + 2U]] +
                              (uint32_t)s_inv_gamma_12[odd_row1[x + 0U]] +
                              (uint32_t)s_inv_gamma_12[odd_row1[x + 2U]] + 2UL) >> 2);
            g12 = (uint16_t)(((uint32_t)s_inv_gamma_12[even_row0[x + 0U]] +
                              (uint32_t)s_inv_gamma_12[even_row0[x + 2U]] +
                              (uint32_t)s_inv_gamma_12[odd_row0[x + 1U]]  +
                              (uint32_t)s_inv_gamma_12[odd_row0[x + 3U]]  +
                              (uint32_t)s_inv_gamma_12[even_row1[x + 0U]] +
                              (uint32_t)s_inv_gamma_12[even_row1[x + 2U]] +
                              (uint32_t)s_inv_gamma_12[odd_row1[x + 1U]]  +
                              (uint32_t)s_inv_gamma_12[odd_row1[x + 3U]]  + 4UL) >> 3);
        }
        else
        {
            g12 = (uint16_t)(((uint32_t)s_inv_gamma_12[even_row0[x]] +
                              (uint32_t)s_inv_gamma_12[odd_row0[x + 1U]]) >> 1);
            r12 = s_inv_gamma_12[even_row0[x + 1U]];
            b12 = s_inv_gamma_12[odd_row0[x]];
        }

        uint16_t rl = (r12 > bl_rb) ? (uint16_t)(r12 - bl_rb) : 0U;
        uint16_t gl = (g12 > bl_g)  ? (uint16_t)(g12 - bl_g)  : 0U;
        uint16_t bl = (b12 > bl_rb) ? (uint16_t)(b12 - bl_rb) : 0U;
        if (vig_scale != 0UL)
        {
            uint32_t gq8 = c3m1_lens_vig_gain_q8((uint16_t)(x + (use_box4 ? 2U : 0U)), py, vig_scale);
            uint32_t rr  = ((uint32_t)rl * gq8 + 128UL) >> 8;
            uint32_t gg  = ((uint32_t)gl * gq8 + 128UL) >> 8;
            uint32_t bb  = ((uint32_t)bl * gq8 + 128UL) >> 8;
            rl = (rr > 4064UL) ? 4064U : (uint16_t)rr;
            gl = (gg > 4064UL) ? 4064U : (uint16_t)gg;
            bl = (bb > 4064UL) ? 4064U : (uint16_t)bb;
        }

        uint8_t sr, sg, sb;
        const uint8_t *bw_tone = c3m1_color_bw_tone_lut(g_cfg.color_mode);
        if (bw_tone != NULL)
        {
            uint8_t y8 = c3m1_color_bw_cam12_to_y8(rl, gl, bl, g_wb_r_q8, g_wb_b_q8, g_cfg.color_mode, s_srgb_lut);
            sr = bw_tone[y8];
            sg = sr;
            sb = sr;
        }
        else
        {
            c3m1_color_cam12_to_srgb8(rl, gl, bl, g_wb_r_q8, g_wb_b_q8, s_srgb_lut, &sr, &sg, &sb);
        }
        dst_rgb480[tx * 3U + 0U] = sr;
        dst_rgb480[tx * 3U + 1U] = sg;
        dst_rgb480[tx * 3U + 2U] = sb;
    }
}

static uint8_t save_dng_from_fifo(FIL *fp, uint16_t bl_g, uint16_t bl_rb,
                                  const c3m1_dng_exif_t *exif,
                                  const c3m1_dng_lens_t *lens,
                                  uint32_t *last_led_tick)
{
    uint8_t *const buf     = s_sd_io_buf;
    const uint16_t buf_cap = (uint16_t)sizeof(s_sd_io_buf);
    const uint32_t vig_scale = (lens != NULL) ? c3m1_lens_vig_scale_q16(lens->vignette) : 0UL;
    const int8_t   distort   = (lens != NULL && g_cfg.lens_dist_incam) ? lens->distort : 0;
    UINT written = 0;
    FRESULT fres;

    /* 1. Write 2,048-byte multi-IFD DNG 1.4 header (IFD0 preview + SubIFD RAW + ExifIFD + OpcodeList3) */
    c3m1_dng_build_header(buf, s_inv_gamma_12, bl_g, bl_rb, g_wb_r_q8, g_wb_b_q8, exif, lens);
    fres = f_write(fp, buf, C3M1_DNG_HEADER_SIZE, &written);
    if (fres != FR_OK || written != C3M1_DNG_HEADER_SIZE)
    {
        printf("DNG hdr err:%d\r\n", fres);
        return 0U;
    }

    /* 2. Pass 1: Stream 160x120 RGB embedded preview thumbnail (57,600 B) from AL422B */
    fifo_prepare_read();
    uint32_t scratch_sz = 0UL;
    uint8_t *scratch    = c3m1_jpeg_get_scratch_buf(&scratch_sz);
    uint8_t *const even_row0 = s_raw_row_buf;         /* 648 B, 4B aligned */
    uint8_t *const odd_row0  = (uint8_t *)s_line_buf; /* 644 B inside contiguous 1,280 B s_line_buf */
    /* Borrow 2x644 B (4B aligned) past ring16 (7,680 B) for 4x4 box AA when anti_alias=ON (0 B .bss) */
    uint8_t *const even_row1 = (g_cfg.anti_alias && scratch != NULL && scratch_sz >= (7680UL + 1288UL))
                             ? (scratch + 7680UL) : NULL;
    uint8_t *const odd_row1  = (even_row1 != NULL) ? (scratch + 7680UL + 644UL) : NULL;
    uint16_t buf_idx = 0U;

    int32_t  kr0_q16 = 65536L, kr1_q16 = 0L;
    uint8_t *ring16  = NULL;
    uint16_t next_src_ty = 0U;
    if (distort != 0 && scratch != NULL && scratch_sz >= (16UL * C3M1_DNG_THUMB_W * 3UL))
    {
        ring16 = scratch;
        c3m1_lens_dist_params_q16(distort, &kr0_q16, &kr1_q16);
    }

    for (uint16_t ty = 0; ty < C3M1_DNG_THUMB_H; ty++)
    {
        uint8_t thumb_row[C3M1_DNG_THUMB_W * 3U];
        uint8_t *row_dst = ((uint32_t)buf_idx + (C3M1_DNG_THUMB_W * 3U) <= buf_cap)
                         ? &buf[buf_idx]
                         : thumb_row;
        if (ring16 != NULL)
        {
            uint16_t req = thumb_req_max_src_ty(ty, kr0_q16, kr1_q16);
            while (next_src_ty <= req && next_src_ty < C3M1_DNG_THUMB_H)
            {
                dng_decode_thumb_src_row(ring16 + (uint32_t)(next_src_ty & 15U) * (C3M1_DNG_THUMB_W * 3U),
                                         next_src_ty, even_row0, odd_row0, even_row1, odd_row1,
                                         bl_g, bl_rb, vig_scale);
                next_src_ty++;
            }
            thumb_warp_row_rgb888(row_dst, ring16, ty, kr0_q16, kr1_q16);
        }
        else
        {
            dng_decode_thumb_src_row(row_dst, ty, even_row0, odd_row0, even_row1, odd_row1,
                                     bl_g, bl_rb, vig_scale);
        }

        if (row_dst == &buf[buf_idx])
        {
            buf_idx = (uint16_t)(buf_idx + C3M1_DNG_THUMB_W * 3U);
            if (buf_idx >= buf_cap)
            {
                fres = f_write(fp, buf, buf_cap, &written);
                if (fres != FR_OK || written != buf_cap)
                {
                    printf("DNG thumb err:%d\r\n", fres);
                    return 0U;
                }
                buf_idx = 0U;
                if ((HAL_GetTick() - *last_led_tick) >= 80U)
                {
                    LED1_TOGGLE();
                    *last_led_tick = HAL_GetTick();
                }
            }
        }
        else
        {
            /* Split across exact 4,096 B (8-sector aligned) boundary to avoid FatFs partial-sector writes */
            uint16_t first = (uint16_t)(buf_cap - buf_idx);
            uint16_t rem   = (uint16_t)((C3M1_DNG_THUMB_W * 3U) - first);
            memcpy(&buf[buf_idx], thumb_row, first);
            fres = f_write(fp, buf, buf_cap, &written);
            if (fres != FR_OK || written != buf_cap)
            {
                printf("DNG thumb err:%d\r\n", fres);
                return 0U;
            }
            memcpy(buf, &thumb_row[first], rem);
            buf_idx = rem;
            if ((HAL_GetTick() - *last_led_tick) >= 80U)
            {
                LED1_TOGGLE();
                *last_led_tick = HAL_GetTick();
            }
        }
    }

    /* 3. Pass 2: Reset AL422B read pointer and stream full 644x480 companded Bayer RAW (309,120 B) */
    fifo_prepare_read();
    uint8_t *raw_buf = scratch;
    const uint32_t raw_chunk = 30720UL; /* 60 full 512 B sectors; raw_chunk + 644 = 31,364 <= 32,200 B */

    if (raw_buf != NULL && scratch_sz >= (raw_chunk + C3M1_DNG_RAW_W))
    {
        /* Carry Pass 1's 256 B tail into raw_buf[0..255] so every 30 KB write stays 512 B sector-aligned */
        uint32_t raw_pos = buf_idx;
        if (raw_pos > 0UL)
        {
            memcpy(raw_buf, buf, raw_pos);
        }
        for (uint16_t row = 0; row < C3M1_DNG_RAW_H; row++)
        {
            fifo_read_bytes(&raw_buf[raw_pos], C3M1_DNG_RAW_W);
            raw_pos += C3M1_DNG_RAW_W;
            if (raw_pos >= raw_chunk)
            {
                fres = f_write(fp, raw_buf, (UINT)raw_chunk, &written);
                if (fres != FR_OK || written != (UINT)raw_chunk)
                {
                    printf("DNG raw err:%d\r\n", fres);
                    return 0U;
                }
                uint32_t rem = raw_pos - raw_chunk;
                if (rem > 0UL)
                {
                    memcpy(raw_buf, raw_buf + raw_chunk, rem);
                }
                raw_pos = rem;
                if ((HAL_GetTick() - *last_led_tick) >= 80U)
                {
                    LED1_TOGGLE();
                    *last_led_tick = HAL_GetTick();
                }
            }
        }
        if (raw_pos > 0UL)
        {
            fres = f_write(fp, raw_buf, (UINT)raw_pos, &written);
            if (fres != FR_OK || written != (UINT)raw_pos)
            {
                printf("DNG tail err:%d\r\n", fres);
                return 0U;
            }
        }
        return 1U;
    }

    for (uint16_t row = 0; row < C3M1_DNG_RAW_H; row++)
    {
        fifo_read_bytes(even_row0, 644U);
        uint16_t src_pos = 0U;
        while (src_pos < 644U)
        {
            uint16_t space = (uint16_t)(buf_cap - buf_idx);
            uint16_t take  = (uint16_t)(644U - src_pos);
            if (take > space) take = space;
            memcpy(&buf[buf_idx], &even_row0[src_pos], take);
            buf_idx = (uint16_t)(buf_idx + take);
            src_pos = (uint16_t)(src_pos + take);
            if (buf_idx >= buf_cap)
            {
                fres = f_write(fp, buf, buf_cap, &written);
                buf_idx = 0U;
                if (fres != FR_OK || written != buf_cap)
                {
                    printf("DNG raw err:%d\r\n", fres);
                    return 0U;
                }
                if ((HAL_GetTick() - *last_led_tick) >= 80U)
                {
                    LED1_TOGGLE();
                    *last_led_tick = HAL_GetTick();
                }
            }
        }
    }

    if (buf_idx > 0U)
    {
        fres = f_write(fp, buf, buf_idx, &written);
        if (fres != FR_OK || written != buf_idx)
        {
            printf("DNG tail err:%d\r\n", fres);
            return 0U;
        }
    }
    return 1U;
}

static uint8_t save_jpg_from_fifo(FIL *fp, uint16_t bl_g, uint16_t bl_rb,
                                  const c3m1_dng_exif_t *exif,
                                  uint32_t *last_led_tick, uint32_t *out_bytes)
{
    jpg_sd_io_t io = { .fp = fp, .last_led_tick = last_led_tick };
    c3m1_jpeg_ctx_t jctx = {
        .inv_gamma_12 = s_inv_gamma_12,
        .srgb_lut     = s_srgb_lut,
        .bl_g         = bl_g,
        .bl_rb        = bl_rb,
        .wb_r_q8      = g_wb_r_q8,
        .wb_b_q8      = g_wb_b_q8,
        .exp_num      = (exif != NULL) ? exif->exp_num : 1UL,
        .exp_den      = (exif != NULL) ? exif->exp_den : 100UL,
        .iso          = (exif != NULL) ? exif->iso     : 100U,
        .ev_num       = (exif != NULL) ? exif->ev_num  : 0,
        .ev_den       = (exif != NULL) ? exif->ev_den  : 3,
        .fnum_x10     = (exif != NULL) ? exif->fnum_x10   : 0U,
        .focal_x10    = (exif != NULL) ? exif->focal_x10  : 0U,
        .focal_eq35    = (exif != NULL) ? exif->focal_eq35 : 0U,
        .lens_model    = (exif != NULL) ? exif->lens_model : NULL,
        .lens_distort  = g_cfg.lens_dist_incam ? g_cfg.lens_distort : 0,
        .lens_vignette = g_cfg.lens_vignette,
        .anti_alias    = g_cfg.anti_alias,
        .high_iso_nr   = g_cfg.high_iso_nr,
        .color_mode    = g_cfg.color_mode,
        .out_buf      = s_sd_io_buf,
        .out_buf_size = (uint16_t)sizeof(s_sd_io_buf),
        .read_row_fn  = jpg_fifo_read_row,
        .write_fn     = jpg_sd_write,
        .user         = &io,
    };
    fifo_prepare_read();
    return c3m1_jpeg_encode_bayer_vga(&jctx, out_bytes);
}

static uint8_t ov7725_camera_output_ex(uint16_t *io_count, uint8_t commit_flash)
{
    uint16_t        count_tmp = (io_count != NULL) ? *io_count : shutter_count_load();
    FRESULT         fres;
    char            day_dir[24];
    char            filename[48];
    uint32_t        last_led_tick;
    c3m1_datetime_t cap_dt;

    /*
     * CRITICAL: Latch one complete 644x480 raw frame into AL422B (FIFO_WEN=0)
     * BEFORE running f_open(), f_write(), or stmflash_write() (which stalls
     * STM32F103 CPU/IRQs for 20-40ms). FIFO stays locked across DNG preview,
     * DNG full RAW, and VGA JPEG passes via AL422B /RRST pointer reset.
     */
    if (!wait_vsync_state(2U, 500UL))
    {
        vsync_reset_idle();
        led1_blink_error();
        return C3M1_CAPTURE_SENSOR_TIMEOUT;
    }

    uint32_t t0 = HAL_GetTick();
    LED1_TOGGLE(); /* Begin blinking to indicate SD write phase */
    last_led_tick = t0;

    c3m1_clock_get_datetime(&cap_dt);
    if (!sd_ensure_photo_day_dir(&cap_dt, day_dir, sizeof(day_dir)))
    {
        vsync_reset_idle();
        led1_blink_error();
        return 1;
    }

    count_tmp = shutter_count_add(count_tmp, 1U);

    uint8_t cap_gain_x = (g_iso_idx == 0U)
                       ? (uint8_t)((g_auto_gain_q4 + 8U) >> 4)
                       : (uint8_t)(1U << (g_iso_idx - 1U));
    if (cap_gain_x < 1U) cap_gain_x = 1U;
    uint16_t bl_g  = c3m1_black_level_g12(cap_gain_x);
    uint16_t bl_rb = c3m1_black_level_rb12(cap_gain_x);
    c3m1_dng_exif_t cap_exif;
    camera_build_capture_exif(0UL, &cap_exif);

    uint8_t want_dng      = (g_cfg.save_mode != C3M1_SAVE_JPG) ? 1U : 0U;
    uint8_t want_jpg      = (g_cfg.save_mode != C3M1_SAVE_DNG) ? 1U : 0U;
    uint8_t flash_written = 0U;
    uint8_t count_bumped  = 0U;
    uint8_t err           = 0U;

    if (want_dng)
    {
        snprintf(filename, sizeof(filename), "%s/CDI%05u.dng", day_dir, (unsigned int)count_tmp);
        fres = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
        if (fres != FR_OK)
        {
            printf("SD open err:%d (%s)\r\n", fres, filename);
            err = 1U;
        }
        else
        {
            c3m1_dng_lens_t cap_lens;
            camera_get_dng_lens(&cap_lens);
            ShutterCount = count_tmp;
            if (io_count != NULL) *io_count = count_tmp;
            count_bumped = 1U;
            if (commit_flash)
            {
                shutter_count_save(count_tmp);
                flash_written = 1U;
            }
            uint8_t ok = save_dng_from_fifo(&file, bl_g, bl_rb, &cap_exif, &cap_lens, &last_led_tick);
            FRESULT cres = f_close(&file);
            if (!ok || cres != FR_OK)
            {
                err = 2U;
            }
            else
            {
                printf("Saved %s (368KB, %lums)\r\n", filename, (unsigned long)(HAL_GetTick() - t0));
            }
        }
    }

    if (err == 0U && want_jpg)
    {
        uint32_t tj = HAL_GetTick();
        uint32_t jpg_bytes = 0UL;
        snprintf(filename, sizeof(filename), "%s/CDI%05u.jpg", day_dir, (unsigned int)count_tmp);
        fres = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
        if (fres != FR_OK)
        {
            printf("SD open err:%d (%s)\r\n", fres, filename);
            err = 3U;
        }
        else
        {
            if (!count_bumped)
            {
                ShutterCount = count_tmp;
                if (io_count != NULL) *io_count = count_tmp;
                count_bumped = 1U;
            }
            if (commit_flash && !flash_written)
            {
                shutter_count_save(count_tmp);
                flash_written = 1U;
            }
            uint8_t ok = save_jpg_from_fifo(&file, bl_g, bl_rb, &cap_exif, &last_led_tick, &jpg_bytes);
            FRESULT cres = f_close(&file);
            if (!ok || cres != FR_OK)
            {
                err = 4U;
            }
            else
            {
                printf("Saved %s (%luB, %lums)\r\n",
                       filename, (unsigned long)jpg_bytes, (unsigned long)(HAL_GetTick() - tj));
            }
        }
    }

    vsync_reset_idle();

    if (err != 0U)
    {
        led1_blink_error();
        return err;
    }

    LED1(1); /* OFF on clean completion */
    return 0;
}

static uint8_t ov7725_camera_output(void)
{
    return ov7725_camera_output_ex(NULL, 1U);
}

/* ======================== In-Camera 12-bit (2F) / 14-bit (3F) HDR Merge ======================== */
#define C3M1_HDR_BLEND_LO       195U    /* 8-bit Gamma-companded raw: ~53% full well */
#define C3M1_HDR_BLEND_HI       235U    /* 8-bit Gamma-companded raw: ~83% full well */
#define C3M1_HDR_BLEND_SPAN     40U
#define C3M1_HDR_ACTIVE_W       640U
#define C3M1_HDR_SAMPLE_STEP    20U
#define C3M1_HDR_SAMPLE_BYTES   (4U * (C3M1_HDR_ACTIVE_W / C3M1_HDR_SAMPLE_STEP) * \
                                 (C3M1_DNG_RAW_H / C3M1_HDR_SAMPLE_STEP)) /* 3,072 B */
#define C3M1_HDR_FAST_SETTLE_MAX 4592UL /* <= 200ms/frame: wait extra settle VSYNC for exact OV7725 AEC */

static int32_t hdr_ratio_to_ev_x100(uint32_t num, uint32_t den)
{
    static const uint16_t log2_lut[17] = {
        0U, 22U, 43U, 64U, 82U, 100U, 117U, 134U, 150U,
        165U, 180U, 194U, 207U, 220U, 233U, 245U, 256U,
    };
    if (num == 0UL || den == 0UL) return 0;

    int32_t sign = 1;
    if (num < den)
    {
        uint32_t t = num;
        num = den;
        den = t;
        sign = -1;
    }

    uint32_t q8   = (uint32_t)(((uint64_t)num * 256ULL + (den >> 1)) / den);
    if (q8 < 256UL) q8 = 256UL;
    uint32_t msb  = 31UL - (uint32_t)__builtin_clz(q8);
    uint32_t m_q8 = (msb >= 8UL) ? (q8 >> (msb - 8UL)) : (q8 << (8UL - msb));
    uint32_t idx  = (m_q8 - 256UL) >> 4;
    uint32_t rem  = (m_q8 - 256UL) & 0x0FUL;
    if (idx > 15UL) { idx = 15UL; rem = 16UL; }

    uint32_t frac_q8 = log2_lut[idx] + (((log2_lut[idx + 1UL] - log2_lut[idx]) * rem + 8UL) >> 4);
    int32_t  ev_q8   = (int32_t)(((msb - 8UL) << 8) + frac_q8);
    int32_t  ev_x100 = (ev_q8 * 100 + 128) >> 8;
    ev_x100 *= sign;
    if (ev_x100 < -500) ev_x100 = -500;
    if (ev_x100 >  500) ev_x100 =  500;
    return ev_x100;
}

static uint8_t camera_needs_slow_capture(void)
{
    return (ov7725_is_slow_shutter(g_dial_pos) ||
            (ov7725_is_auto_shutter(g_dial_pos) && g_auto_aec > OV7725_PREVIEW_SLOW_AEC)) ? 1U : 0U;
}

/* Switch sensor from 45fps preview into exact slow-shutter exposure and latch 1 frame into AL422B */
static uint8_t camera_capture_slow_to_fifo(void)
{
    /* Sync to start of a preview frame so all SCCB writes complete before VSYNC_A */
    vsync_reset_idle();
    (void)wait_vsync_state(1U, 200UL);

    ov7725_apply_raw_exposure(g_dial_pos, g_iso_idx, g_auto_aec,
                              ov7725_gain_q4_to_reg(g_auto_gain_q4));

    /* Wait for VSYNC_A (new DM_LN/AEC/GAIN latch into OV7725) */
    vsync_reset_idle();
    (void)wait_vsync_state(1U, 200UL);

    /* Disarm transition frame; arm AL422B to capture VSYNC_B (true long-exposure frame) */
    vsync_reset_idle();
    (void)wait_vsync_state(1U, 2600UL);

    /*
     * Wait for EXTI VSYNC ISR (case 1 -> WEN=0, g_vsync_state=2) to finish latching all
     * DM_LN dummy lines + 480 active rows. OV7725 inserts DM_LN before active HREF, so
     * locking early on a fixed 80ms timer truncates slow frames before Row 0 arrives.
     */
    uint8_t ok = wait_vsync_state(2U, 2600UL);
    __disable_irq();
    OV7725_WEN(0);
    g_vsync_state = 2;
    __enable_irq();
    return ok;
}

/* Capture the metered 0 EV frame using the exact single-shot capture path */
static void hdr_capture_metered_frame(void)
{
    if (camera_needs_slow_capture())
    {
        (void)camera_capture_slow_to_fifo();
        return;
    }

    vsync_reset_idle();
    if (g_exposure_dirty_frames > 0U)
    {
        wait_skip_frames(1U);
        g_exposure_dirty_frames = 0U;
    }
    if (!wait_vsync_state(2U, 500UL))
    {
        __disable_irq();
        OV7725_WEN(0);
        g_vsync_state = 2;
        __enable_irq();
    }
}

/* Apply exact bracket exposure lines and latch one fully settled frame into AL422B FIFO */
static void hdr_capture_frame_to_fifo(uint32_t fast_lines, uint8_t gain_reg, uint32_t *out_actual_lines)
{
    uint8_t is_slow_clk = 0U;

    /* Sync to start of current VSYNC so all SCCB writes land before next VSYNC */
    vsync_reset_idle();
    (void)wait_vsync_state(1U, 2600UL);

    uint32_t actual = ov7725_apply_exact_lines(fast_lines, gain_reg, &is_slow_clk);
    if (out_actual_lines != NULL)
    {
        *out_actual_lines = actual;
    }

    (void)is_slow_clk;
    uint32_t frame_lines = (actual > 490UL) ? (actual + 50UL) : 510UL;
    uint32_t max_wait_ms = (frame_lines * 1000UL) / 22959UL + 500UL;

    /* VSYNC_A: new AEC/DM_LN/GAIN registers latch into OV7725 */
    vsync_reset_idle();
    (void)wait_vsync_state(1U, 2600UL);

    /* VSYNC_B: discard first post-latch frame */
    vsync_reset_idle();
    (void)wait_vsync_state(1U, max_wait_ms);

    /*
     * OV7725 upward-AEC settle: when AEC increases with DM_LN=0, the frame after
     * VSYNC_A misses its rolling-shutter reset match and exposes a full 510-line frame.
     * Wait one extra VSYNC (VSYNC_C, only +22ms at <=490 lines) for all fast/mid frames
     * (<=200ms) so AL422B latches a 100% settled exact-line exposure.
     */
    if (actual <= C3M1_HDR_FAST_SETTLE_MAX)
    {
        vsync_reset_idle();
        (void)wait_vsync_state(1U, max_wait_ms);
    }

    /*
     * Wait for the next VSYNC (EXTI ISR case 1 -> WEN=0, g_vsync_state=2) so all DM_LN
     * dummy lines + 480 active rows finish writing into AL422B.
     */
    (void)wait_vsync_state(2U, max_wait_ms);

    __disable_irq();
    OV7725_WEN(0);
    g_vsync_state = 2;
    __enable_irq();
}

static uint8_t hdr_save_frame_dng(const char *path, uint16_t bl_g, uint16_t bl_rb,
                                  uint32_t frame_fast_lines, uint32_t base_lines,
                                  const c3m1_dng_lens_t *lens,
                                  uint32_t *last_led_tick)
{
    FRESULT fres = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        printf("HDR frame open err:%d (%s)\r\n", fres, path);
        return 0U;
    }

    c3m1_dng_exif_t fexif;
    camera_build_capture_exif(frame_fast_lines, &fexif);
    int32_t brk_ev_x100 = hdr_ratio_to_ev_x100(frame_fast_lines, base_lines);
    int32_t brk_thirds  = (brk_ev_x100 >= 0) ? ((brk_ev_x100 * 3 + 50) / 100)
                                             : ((brk_ev_x100 * 3 - 50) / 100);
    fexif.ev_num += brk_thirds;
    uint8_t ok   = save_dng_from_fifo(&file, bl_g, bl_rb, &fexif, lens, last_led_tick);
    FRESULT cres = f_close(&file);
    return (ok && cres == FR_OK) ? 1U : 0U;
}

static inline uint32_t hdr_lin_sub(uint8_t raw8, uint16_t bl12)
{
    uint16_t v = s_inv_gamma_12[raw8];
    return (v > bl12) ? (uint32_t)(v - bl12) : 0UL;
}

/* Subsample 2x2 GRBG blocks on a 20px grid (3,072 B) from a saved single-frame DNG */
static uint8_t hdr_read_samples(const char *path, uint8_t *samples)
{
    if (f_open(&file, path, FA_READ) != FR_OK)
    {
        return 0U;
    }

    uint8_t ok = 1U;
    for (uint32_t row = 0UL; row < C3M1_DNG_RAW_H; row += C3M1_HDR_SAMPLE_STEP)
    {
        UINT bytes_read = 0U;
        if (f_lseek(&file, C3M1_DNG_RAW_OFFSET + row * C3M1_DNG_RAW_W) != FR_OK ||
            f_read(&file, s_sd_io_buf, 2U * C3M1_DNG_RAW_W, &bytes_read) != FR_OK ||
            bytes_read != (2U * C3M1_DNG_RAW_W))
        {
            ok = 0U;
            break;
        }
        for (uint32_t col = 0UL; col < C3M1_HDR_ACTIVE_W; col += C3M1_HDR_SAMPLE_STEP)
        {
            *samples++ = s_sd_io_buf[col];
            *samples++ = s_sd_io_buf[col + 1UL];
            *samples++ = s_sd_io_buf[C3M1_DNG_RAW_W + col];
            *samples++ = s_sd_io_buf[C3M1_DNG_RAW_W + col + 1UL];
        }
    }

    return (f_close(&file) == FR_OK && ok) ? 1U : 0U;
}

/*
 * Empirical Q8 signal-ratio refinement between two adjacent bracket frames using a
 * two-pass coarse/fine histogram median. Rejects noisy/sparse estimates or ratios
 * outside [0.65x..1.55x] of the nominal exposure-line ratio so dark/high-ISO frames
 * fall back cleanly to exact line ratios.
 */
static uint32_t hdr_measure_ratio_q8(const uint8_t *shorter, const uint8_t *longer,
                                     uint16_t bl_g, uint16_t bl_rb,
                                     uint32_t nominal_q8, uint16_t *bins)
{
    uint32_t count = 0UL, median_rank = 0UL, median_bin = 0UL, consistent = 0UL;
    memset(bins, 0, 256U * sizeof(uint16_t));

    for (uint8_t pass = 0U; pass < 2U; pass++)
    {
        for (uint32_t idx = 0UL; idx < C3M1_HDR_SAMPLE_BYTES; idx++)
        {
            uint8_t low  = shorter[idx];
            uint8_t high = longer[idx];
            if (low < 30U || low > 180U || high < 60U || high > 180U)
            {
                continue;
            }
            uint32_t phase  = idx & 3UL;
            uint16_t black  = (phase == 0UL || phase == 3UL) ? bl_g : bl_rb;
            uint32_t signal = hdr_lin_sub(low, black);
            if (signal == 0UL)
            {
                continue;
            }
            uint32_t ratio = (hdr_lin_sub(high, black) * 256UL + (signal >> 1)) / signal;
            uint32_t bin   = ratio >> 5;
            if (pass == 0U)
            {
                bins[(bin < 255UL) ? bin : 255UL]++;
                count++;
            }
            else
            {
                uint32_t center = median_bin * 32UL + 16UL;
                if (ratio >= (center - (center >> 2)) && ratio <= (center + (center >> 2)))
                {
                    consistent++;
                }
                if (bin == median_bin)
                {
                    bins[ratio & 31UL]++;
                }
            }
        }

        if (pass == 0U)
        {
            if (count < 64UL)
            {
                return 0UL;
            }
            median_rank = count >> 1;
            while (median_bin < 255UL && median_rank >= (uint32_t)bins[median_bin])
            {
                median_rank -= (uint32_t)bins[median_bin];
                median_bin++;
            }
            if (median_bin < 8UL || median_bin >= 255UL)
            {
                return 0UL;
            }
            memset(bins, 0, 32U * sizeof(uint16_t));
        }
    }

    if ((consistent * 4UL) < (count * 3UL))
    {
        return 0UL;
    }

    uint32_t fine_bin = 0UL;
    while (fine_bin < 31UL && median_rank >= (uint32_t)bins[fine_bin])
    {
        median_rank -= (uint32_t)bins[fine_bin];
        fine_bin++;
    }

    uint32_t est_q8 = median_bin * 32UL + fine_bin;
    uint32_t min_ok = (nominal_q8 * 166UL) >> 8; /* 0.65x nominal */
    uint32_t max_ok = (nominal_q8 * 397UL) >> 8; /* 1.55x nominal */
    if (est_q8 < min_ok || est_q8 > max_ok)
    {
        return 0UL;
    }
    return est_q8;
}

/* Compute one shared blend weight [0..C3M1_HDR_BLEND_SPAN] for a 2x2 GRBG block from its brightest pixel */
static inline uint32_t hdr_block_weight(const uint8_t *block)
{
    uint8_t peak = (block[0] > block[1]) ? block[0] : block[1];
    if (block[C3M1_DNG_RAW_W] > peak)
    {
        peak = block[C3M1_DNG_RAW_W];
    }
    if (block[C3M1_DNG_RAW_W + 1U] > peak)
    {
        peak = block[C3M1_DNG_RAW_W + 1U];
    }
    if (peak <= C3M1_HDR_BLEND_LO)
    {
        return 0UL;
    }
    if (peak >= C3M1_HDR_BLEND_HI)
    {
        return C3M1_HDR_BLEND_SPAN;
    }
    return (uint32_t)(peak - C3M1_HDR_BLEND_LO);
}

static inline uint32_t hdr_blend(uint32_t signal, uint8_t shorter, uint16_t black,
                                 uint32_t ratio_q8, uint32_t weight)
{
    if (weight == 0UL)
    {
        return signal;
    }
    uint32_t scaled = (hdr_lin_sub(shorter, black) * ratio_q8 + 128UL) >> 8;
    return ((C3M1_HDR_BLEND_SPAN - weight) * signal + weight * scaled + (C3M1_HDR_BLEND_SPAN >> 1)) /
           C3M1_HDR_BLEND_SPAN;
}

/* Merge one 2x2 GRBG block across 2..4 frames with synchronized channel weights */
static inline void hdr_merge_block(const uint8_t *const frames[4], uint8_t nf,
                                   const uint32_t ratio_q8[4],
                                   uint16_t bl_g, uint16_t bl_rb,
                                   uint16_t white_level, uint16_t result[4])
{
    uint32_t w[4];
    for (uint8_t f = 1U; f < nf; f++)
    {
        w[f] = hdr_block_weight(frames[f]);
    }

    for (uint8_t phase = 0U; phase < 4U; phase++)
    {
        uint32_t offset = (uint32_t)(phase >> 1) * C3M1_DNG_RAW_W + (uint32_t)(phase & 1U);
        uint16_t black  = (phase == 0U || phase == 3U) ? bl_g : bl_rb;
        uint32_t signal = hdr_lin_sub(frames[nf - 1U][offset], black);
        for (int8_t f = (int8_t)(nf - 2U); f >= 0; f--)
        {
            signal = hdr_blend(signal, frames[f][offset], black, ratio_q8[f], w[f + 1]);
        }
        signal = (signal + 2UL) >> 2;
        result[phase] = (signal > (uint32_t)white_level) ? white_level : (uint16_t)signal;
    }
}

typedef struct
{
    uint8_t  valid;
    uint32_t ratio_q8[4];
    uint32_t ref_white;
    uint16_t wl;
    int32_t  bev_x100;
} hdr_lock_t;

static void hdr_build_bracket(uint8_t nf, uint32_t base_lines, uint32_t e_req[4])
{
    e_req[0] = 1UL;
    e_req[1] = 4UL;
    e_req[2] = 16UL;
    e_req[3] = 64UL;

    if (nf == 2U)
    {
        /* 12-bit 2F symmetric bracket: -1 / +1 EV (span 4x = +2 EV over 10-bit -> 12-bit) */
        e_req[0] = (base_lines >= 2UL) ? (base_lines >> 1) : 1UL;
        e_req[1] = e_req[0] * 4UL;
        if (e_req[1] > OV7725_MAX_FAST_LINES)
        {
            e_req[1] = OV7725_MAX_FAST_LINES;
            e_req[0] = e_req[1] >> 2;
        }
    }
    else if (nf == 3U)
    {
        /*
         * 14-bit 3F bracket: keep _1 at exact metered 0 EV across [2..45917] lines;
         * shift cleanly only at the two hardware limits (1 line or 2.0s) so all 3
         * frames stay strictly distinct (e_req[0] < e_req[1] < e_req[2]).
         */
        if (base_lines <= 1UL)
        {
            e_req[0] = 1UL;
            e_req[1] = 4UL;
            e_req[2] = 16UL;
        }
        else if (base_lines >= OV7725_MAX_FAST_LINES)
        {
            e_req[2] = OV7725_MAX_FAST_LINES;
            e_req[1] = e_req[2] >> 2;
            e_req[0] = e_req[1] >> 2;
        }
        else
        {
            e_req[0] = (base_lines + 2UL) / 4UL;
            if (e_req[0] < 1UL) e_req[0] = 1UL;
            e_req[1] = base_lines;
            e_req[2] = base_lines * 4UL;
            if (e_req[2] > OV7725_MAX_FAST_LINES) e_req[2] = OV7725_MAX_FAST_LINES;
        }
    }
    else
    {
        /* 16-bit 4F symmetric bracket: -3 / -1 / +1 / +3 EV (1:4:16:64 = +6 EV -> true 16-bit) */
        uint32_t s0 = (base_lines >= 8UL) ? ((base_lines + 4UL) >> 3) : 1UL;
        if (s0 * 64UL > OV7725_MAX_FAST_LINES)
        {
            s0 = OV7725_MAX_FAST_LINES >> 6; /* 717 lines -> 45,888 lines (<= 2.0s) */
        }
        e_req[0] = s0;
        e_req[1] = s0 * 4UL;
        e_req[2] = s0 * 16UL;
        e_req[3] = s0 * 64UL;
    }
}

/*
 * Capture nf bracketed frames (sub_paths[0..nf-1]) and stream-merge into one 16-bit Linear HDR DNG.
 *   - standalone == 1: standalone HDR restores preview right after frame nf-1 and uses 3F metered-first order.
 *   - standalone == 0: JSR + HDR captures ascending 0..nf-1 and locks ratio_q8/wl/ref_white/bev_x100
 *     on the first frame into *lock, reusing them across all subsequent light frames + dark frame.
 *   - Bracket sub-frames are kept intact here; caller deletes them after completion if g_cfg.del_hdr_brk is ON.
 */
static uint8_t __attribute__((optimize("O2"))) hdr_capture_and_merge_one(
    uint8_t nf, const uint32_t e_req[4], uint32_t base_lines,
    uint8_t cap_gain_reg, uint16_t bl_g, uint16_t bl_rb,
    const char sub_paths[4][64], const char *path_out,
    uint8_t standalone, hdr_lock_t *lock, const c3m1_dng_lens_t *lens,
    const char *ui_prefix, uint16_t *shots_fired, uint32_t *last_led_tick)
{
    uint32_t t_all = HAL_GetTick();
    uint32_t e_act[4] = { 1UL, 4UL, 16UL, 64UL };
    uint8_t  err = 0U;
    const c3m1_dng_lens_t *sub_lens = lens;
    char     step_msg[28];

    if (standalone && nf == 3U)
    {
        /* Standalone 3F captures metered 0 EV frame _1 first, then shortest _0, then longest _2 */
        snprintf(step_msg, sizeof(step_msg), "%s 1/3...", ui_prefix);
        osd_draw_center_box(step_msg, YELLOW);
        if (e_req[1] == base_lines)
        {
            hdr_capture_metered_frame();
            e_act[1] = base_lines;
        }
        else
        {
            hdr_capture_frame_to_fifo(e_req[1], cap_gain_reg, &e_act[1]);
        }
        (*shots_fired)++;
        if (!hdr_save_frame_dng(sub_paths[1], bl_g, bl_rb, e_act[1], base_lines, sub_lens, last_led_tick))
        {
            err = 2U;
        }
        if (err == 0U)
        {
            snprintf(step_msg, sizeof(step_msg), "%s 2/3...", ui_prefix);
            osd_draw_center_box(step_msg, YELLOW);
            hdr_capture_frame_to_fifo(e_req[0], cap_gain_reg, &e_act[0]);
            (*shots_fired)++;
            if (!hdr_save_frame_dng(sub_paths[0], bl_g, bl_rb, e_act[0], base_lines, sub_lens, last_led_tick))
            {
                err = 1U;
            }
        }
    }
    else
    {
        /* Capture shorter frames _0 .. _(nf-2) in ascending exposure order */
        for (uint8_t f = 0U; f + 1U < nf && err == 0U; f++)
        {
            snprintf(step_msg, sizeof(step_msg), "%s %u/%u...",
                     ui_prefix, (unsigned int)(f + 1U), (unsigned int)nf);
            osd_draw_center_box(step_msg, YELLOW);
            hdr_capture_frame_to_fifo(e_req[f], cap_gain_reg, &e_act[f]);
            (*shots_fired)++;
            if (!hdr_save_frame_dng(sub_paths[f], bl_g, bl_rb, e_act[f], base_lines, sub_lens, last_led_tick))
            {
                err = (uint8_t)(f + 1U);
            }
        }
    }

    /* Final frame (longest) -> latch in AL422B FIFO for zero-SD-reread HDR merge */
    if (err == 0U)
    {
        snprintf(step_msg, sizeof(step_msg), "%s %u/%u...", ui_prefix, (unsigned int)nf, (unsigned int)nf);
        osd_draw_center_box(step_msg, YELLOW);
        hdr_capture_frame_to_fifo(e_req[nf - 1U], cap_gain_reg, &e_act[nf - 1U]);
        (*shots_fired)++;
    }

    if (standalone)
    {
        /* Restore normal viewfinder exposure while AL422B holds the locked final frame */
        raw_apply_preview_exposure(1U);
    }

    if (err == 0U)
    {
        if (!hdr_save_frame_dng(sub_paths[nf - 1U], bl_g, bl_rb, e_act[nf - 1U], base_lines, sub_lens, last_led_tick))
        {
            err = 3U;
        }
    }

    if (err != 0U)
    {
        return err;
    }

    if (standalone)
    {
        osd_draw_bar("MRG ", CYAN);
        osd_draw_center_box("MERGING...", CYAN);
    }
    else
    {
        snprintf(step_msg, sizeof(step_msg), "%s MRG...", ui_prefix);
        osd_draw_center_box(step_msg, CYAN);
    }

    uint32_t ratio_q8[4] = { 256UL, 256UL, 256UL, 256UL };
    uint32_t ref_white;
    uint16_t wl;
    int32_t  bev_x100;

    uint32_t scratch_sz = 0UL;
    uint8_t *scratch = c3m1_jpeg_get_scratch_buf(&scratch_sz);
    const uint32_t tf_bytes = (uint32_t)(3U * sizeof(FIL));
    uint32_t work_scratch_sz = 0UL;
    FIL *tf = NULL;
    if (scratch == NULL || scratch_sz < (12U * C3M1_DNG_RAW_W + tf_bytes + 4UL))
    {
        err = 4U;
    }
    else
    {
        /* Place 3x FIL (1,776 B) at the 4-byte-aligned tail of scratch to save stack with 0 B .bss */
        work_scratch_sz = (scratch_sz - tf_bytes) & ~3UL;
        tf = (FIL *)(scratch + work_scratch_sz);
    }

    if (err == 0U && lock != NULL && lock->valid)
    {
        for (uint8_t f = 0U; f < 4U; f++) ratio_q8[f] = lock->ratio_q8[f];
        wl        = lock->wl;
        ref_white = lock->ref_white;
        bev_x100  = lock->bev_x100;
    }
    else if (err == 0U)
    {
        uint32_t e_last = e_act[nf - 1U];
        uint32_t adj_nom_q8[3] = { 256UL, 256UL, 256UL };
        for (uint8_t f = 0U; f + 1U < nf; f++)
        {
            ratio_q8[f]   = (uint32_t)(((uint64_t)e_last * 256ULL + (e_act[f] >> 1)) / e_act[f]);
            adj_nom_q8[f] = (uint32_t)(((uint64_t)e_act[f + 1U] * 256ULL + (e_act[f] >> 1)) / e_act[f]);
        }

        if (scratch_sz >= (2U * C3M1_HDR_SAMPLE_BYTES + 256U * sizeof(uint16_t)))
        {
            uint8_t  *samp_a = scratch;
            uint8_t  *samp_b = scratch + C3M1_HDR_SAMPLE_BYTES;
            uint16_t *bins   = (uint16_t *)(scratch + 2U * C3M1_HDR_SAMPLE_BYTES);
            uint32_t  adj_scale[3];
            uint8_t   any_est = 0U;
            for (uint8_t f = 0U; f + 1U < nf; f++)
            {
                adj_scale[f] = adj_nom_q8[f];
            }
            if (hdr_read_samples(sub_paths[0], samp_a))
            {
                uint8_t *prev_s = samp_a;
                uint8_t *curr_s = samp_b;
                for (uint8_t f = 0U; f + 1U < nf; f++)
                {
                    if (!hdr_read_samples(sub_paths[f + 1U], curr_s))
                    {
                        break;
                    }
                    uint32_t est = hdr_measure_ratio_q8(prev_s, curr_s, bl_g, bl_rb, adj_nom_q8[f], bins);
                    if (est != 0UL)
                    {
                        adj_scale[f] = est;
                        any_est = 1U;
                    }
                    uint8_t *tmp = prev_s;
                    prev_s = curr_s;
                    curr_s = tmp;
                }
            }
            if (any_est)
            {
                uint32_t cand_q8[4] = { 256UL, 256UL, 256UL, 256UL };
                uint32_t acc = 256UL;
                for (int8_t f = (int8_t)(nf - 2U); f >= 0; f--)
                {
                    acc = (uint32_t)(((uint64_t)acc * (uint64_t)adj_scale[f] + 128ULL) >> 8);
                    cand_q8[f] = acc;
                }
                uint32_t max_span = ((nf == 4U) ? 128UL : 32UL) * 256UL;
                if (cand_q8[0] >= 256UL && cand_q8[0] <= max_span)
                {
                    for (uint8_t f = 0U; f < nf; f++)
                    {
                        ratio_q8[f] = cand_q8[f];
                    }
                }
            }
        }

        uint32_t bl_max = (bl_g > bl_rb) ? (uint32_t)bl_g : (uint32_t)bl_rb;
        uint32_t max_10 = (4064UL > bl_max) ? ((4064UL - bl_max) >> 2) : 960UL;
        uint32_t wl32   = (uint32_t)(((uint64_t)max_10 * ratio_q8[0] + 128ULL) >> 8);
        if (wl32 < max_10)     wl32 = max_10;
        if (wl32 > UINT16_MAX) wl32 = UINT16_MAX;
        wl = (uint16_t)wl32;

        if (nf == 3U)
        {
            uint32_t mid_white = (uint32_t)(((uint64_t)max_10 * ratio_q8[1] + 128ULL) >> 8);
            ref_white = (uint32_t)(((uint64_t)mid_white * e_act[1] + (base_lines >> 1)) / base_lines);
        }
        else
        {
            ref_white = (uint32_t)(((uint64_t)wl * e_act[0] + (base_lines >> 1)) / base_lines);
        }
        if (ref_white < 256UL) ref_white = 256UL;
        bev_x100 = hdr_ratio_to_ev_x100(wl, ref_white);

        if (lock != NULL)
        {
            for (uint8_t f = 0U; f < 4U; f++) lock->ratio_q8[f] = ratio_q8[f];
            lock->wl        = wl;
            lock->ref_white = ref_white;
            lock->bev_x100  = bev_x100;
            lock->valid     = 1U;
        }
    }
    else
    {
        wl        = 4064U;
        ref_white = 1000UL;
        bev_x100  = 0;
    }

    /* Open saved individual DNG frames _0 .. _(nf-2) for raw read-back + output merged DNG */
    uint8_t tf_open[3] = { 0U, 0U, 0U };
    uint8_t out_open   = 0U;

    for (uint8_t f = 0U; f + 1U < nf && err == 0U; f++)
    {
        if (f_open(&tf[f], sub_paths[f], FA_READ) == FR_OK)
        {
            tf_open[f] = 1U;
        }
        else
        {
            err = (uint8_t)(5U + f);
        }
    }

    if (err == 0U)
    {
        if (f_open(&file, path_out, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
        {
            out_open = 1U;
        }
        else
        {
            err = 8U;
        }
    }

    /* 1. Write 2,048-byte 16-bit Linear HDR DNG 1.4 header (with base exposure EXIF + OpcodeList3) */
    if (err == 0U)
    {
        UINT written = 0U;
        c3m1_dng_exif_t hdr_exif;
        camera_build_capture_exif(0UL, &hdr_exif);
        c3m1_dng_build_hdr_header(s_sd_io_buf, wl, bev_x100, g_wb_r_q8, g_wb_b_q8, &hdr_exif, lens);
        if (f_write(&file, s_sd_io_buf, C3M1_DNG_HEADER_SIZE, &written) != FR_OK ||
            written != C3M1_DNG_HEADER_SIZE)
        {
            err = 9U;
        }
    }

    /* 2. Pass 1: Stream tone-mapped 160x120 RGB embedded preview (57,600 B) */
    if (err == 0U)
    {
        const uint32_t vig_scale = (lens != NULL) ? c3m1_lens_vig_scale_q16(lens->vignette) : 0UL;
        const int8_t   distort   = (lens != NULL && g_cfg.lens_dist_incam) ? lens->distort : 0;
        uint8_t *fb[4];
        for (uint8_t f = 0U; f < nf; f++)
        {
            fb[f] = scratch + (uint32_t)(f * 2U) * C3M1_DNG_RAW_W;
        }
        int32_t  kr0_q16 = 65536L, kr1_q16 = 0L;
        uint8_t *ring16  = NULL;
        uint16_t next_src_ty = 0U;
        uint32_t fb_end  = (uint32_t)nf * 2UL * C3M1_DNG_RAW_W;
        if (distort != 0 && work_scratch_sz >= (fb_end + 16UL * C3M1_DNG_THUMB_W * 3UL))
        {
            ring16 = scratch + fb_end;
            c3m1_lens_dist_params_q16(distort, &kr0_q16, &kr1_q16);
        }

        fifo_prepare_read();
        uint16_t buf_idx = 0U;

        for (uint16_t ty = 0U; ty < C3M1_DNG_THUMB_H; ty++)
        {
            uint16_t req = (ring16 != NULL) ? thumb_req_max_src_ty(ty, kr0_q16, kr1_q16) : ty;
            while (next_src_ty <= req && next_src_ty < C3M1_DNG_THUMB_H)
            {
                uint32_t row_off = C3M1_DNG_RAW_OFFSET + (uint32_t)next_src_ty * 4UL * C3M1_DNG_RAW_W;
                for (uint8_t f = 0U; f + 1U < nf; f++)
                {
                    UINT br = 0U;
                    if (f_lseek(&tf[f], row_off) != FR_OK ||
                        f_read(&tf[f], fb[f], C3M1_DNG_RAW_W * 2U, &br) != FR_OK ||
                        br != (C3M1_DNG_RAW_W * 2U))
                    {
                        err = 10U;
                        break;
                    }
                }
                if (err != 0U) break;

                fifo_read_bytes(fb[nf - 1U], C3M1_DNG_RAW_W * 2U);
                if (next_src_ty + 1U < C3M1_DNG_THUMB_H)
                {
                    fifo_skip_bytes(2UL * C3M1_DNG_RAW_W);
                }

                uint8_t *dst_row = (ring16 != NULL)
                                 ? (ring16 + (uint32_t)(next_src_ty & 15U) * (C3M1_DNG_THUMB_W * 3U))
                                 : &s_sd_io_buf[buf_idx];
                uint16_t py = (uint16_t)(next_src_ty * 4U);
                for (uint16_t tx = 0U; tx < C3M1_DNG_THUMB_W; tx++)
                {
                    uint16_t x = (uint16_t)(tx * 4U);
                    const uint8_t *fptrs[4];
                    for (uint8_t f = 0U; f < nf; f++) fptrs[f] = fb[f] + x;
                    uint16_t block[4];
                    hdr_merge_block(fptrs, nf, ratio_q8, bl_g, bl_rb, wl, block);
                    uint8_t sr, sg, sb;
                    uint32_t r16 = block[1];
                    uint32_t g16 = ((uint32_t)block[0] + (uint32_t)block[3] + 1UL) >> 1;
                    uint32_t b16 = block[2];
                    if (vig_scale != 0UL)
                    {
                        uint32_t gq8 = c3m1_lens_vig_gain_q8(x, py, vig_scale);
                        r16 = (r16 * gq8 + 128UL) >> 8;
                        g16 = (g16 * gq8 + 128UL) >> 8;
                        b16 = (b16 * gq8 + 128UL) >> 8;
                        if (r16 > wl) r16 = wl;
                        if (g16 > wl) g16 = wl;
                        if (b16 > wl) b16 = wl;
                    }
                    const uint8_t *bw_tone = c3m1_color_bw_tone_lut(g_cfg.color_mode);
                    if (bw_tone != NULL)
                    {
                        uint8_t y8 = c3m1_color_hdr_to_bw_y8((uint16_t)r16, (uint16_t)g16, (uint16_t)b16,
                                                             g_wb_r_q8, g_wb_b_q8, ref_white, (uint32_t)wl,
                                                             g_cfg.color_mode, s_srgb_lut);
                        sr = bw_tone[y8];
                        sg = sr;
                        sb = sr;
                    }
                    else
                    {
                        c3m1_color_hdr_to_srgb8((uint16_t)r16, (uint16_t)g16, (uint16_t)b16,
                                                g_wb_r_q8, g_wb_b_q8, ref_white, (uint32_t)wl,
                                                s_srgb_lut, &sr, &sg, &sb);
                    }
                    dst_row[tx * 3U + 0U] = sr;
                    dst_row[tx * 3U + 1U] = sg;
                    dst_row[tx * 3U + 2U] = sb;
                }
                next_src_ty++;
            }
            if (err != 0U) break;

            if (ring16 != NULL)
            {
                thumb_warp_row_rgb888(&s_sd_io_buf[buf_idx], ring16, ty, kr0_q16, kr1_q16);
            }
            buf_idx = (uint16_t)(buf_idx + C3M1_DNG_THUMB_W * 3U);

            if (buf_idx >= 3840U)
            {
                UINT written = 0U;
                if (f_write(&file, s_sd_io_buf, buf_idx, &written) != FR_OK || written != buf_idx)
                {
                    err = 11U;
                    break;
                }
                buf_idx = 0U;
                if ((HAL_GetTick() - *last_led_tick) >= 80UL)
                {
                    LED1_TOGGLE();
                    *last_led_tick = HAL_GetTick();
                }
            }
        }
    }

    /* 3. Pass 2: Stream full 644x480 16-bit LE merged linear Bayer RAW in 4,096 B sector-aligned chunks */
    if (err == 0U)
    {
        uint8_t *rb[4];
        for (uint8_t f = 0U; f < nf; f++)
        {
            rb[f] = scratch + (uint32_t)(f * 2U) * C3M1_DNG_RAW_W;
        }
        uint8_t *pair_stage = scratch + (uint32_t)(nf * 2U) * C3M1_DNG_RAW_W; /* 2,576 B (4 rows) */

        for (uint8_t f = 0U; f + 1U < nf; f++)
        {
            if (f_lseek(&tf[f], C3M1_DNG_RAW_OFFSET) != FR_OK)
            {
                err = 12U;
                break;
            }
        }
        if (err == 0U)
        {
            fifo_prepare_read();
            const uint16_t pair_in_bytes  = 2U * C3M1_DNG_RAW_W; /* 1,288 B */
            const uint16_t pair_out_bytes = 4U * C3M1_DNG_RAW_W; /* 2,576 B */
            const uint16_t io_cap         = (uint16_t)sizeof(s_sd_io_buf); /* 4,096 B */
            uint16_t       io_idx         = 0U;

            for (uint16_t row = 0U; row < C3M1_DNG_RAW_H; row += 2U)
            {
                for (uint8_t f = 0U; f + 1U < nf; f++)
                {
                    UINT bytes = 0U;
                    if (f_read(&tf[f], rb[f], pair_in_bytes, &bytes) != FR_OK || bytes != pair_in_bytes)
                    {
                        err = 13U;
                        break;
                    }
                }
                if (err != 0U) break;
                fifo_read_bytes(rb[nf - 1U], pair_in_bytes);

                for (uint16_t col = 0U; col < C3M1_DNG_RAW_W; col += 2U)
                {
                    const uint8_t *fptrs[4];
                    for (uint8_t f = 0U; f < nf; f++) fptrs[f] = rb[f] + col;
                    uint16_t block[4];
                    hdr_merge_block(fptrs, nf, ratio_q8, bl_g, bl_rb, wl, block);
                    for (uint8_t phase = 0U; phase < 4U; phase++)
                    {
                        uint16_t dst = (uint16_t)(2U * (col + (phase & 1U)) +
                                                  (uint16_t)(phase >> 1) * pair_in_bytes);
                        pair_stage[dst]      = (uint8_t)(block[phase] & 0xFFU);
                        pair_stage[dst + 1U] = (uint8_t)(block[phase] >> 8);
                    }
                }

                uint16_t src = 0U;
                while (src < pair_out_bytes)
                {
                    uint16_t take = (uint16_t)(pair_out_bytes - src);
                    uint16_t room = (uint16_t)(io_cap - io_idx);
                    if (take > room) take = room;
                    memcpy(&s_sd_io_buf[io_idx], &pair_stage[src], take);
                    io_idx = (uint16_t)(io_idx + take);
                    src    = (uint16_t)(src + take);
                    if (io_idx == io_cap)
                    {
                        UINT written = 0U;
                        if (f_write(&file, s_sd_io_buf, io_cap, &written) != FR_OK || written != io_cap)
                        {
                            err = 15U;
                            break;
                        }
                        io_idx = 0U;
                        if ((HAL_GetTick() - *last_led_tick) >= 80UL)
                        {
                            LED1_TOGGLE();
                            *last_led_tick = HAL_GetTick();
                        }
                    }
                }
                if (err != 0U)
                {
                    break;
                }
            }

            if (err == 0U && io_idx > 0U)
            {
                UINT written = 0U;
                if (f_write(&file, s_sd_io_buf, io_idx, &written) != FR_OK || written != io_idx)
                {
                    err = 15U;
                }
            }
        }
    }

    if (out_open && f_close(&file) != FR_OK && err == 0U) err = 16U;
    if (tf != NULL)
    {
        for (uint8_t f = 0U; f < 3U; f++)
        {
            if (tf_open[f]) (void)f_close(&tf[f]);
        }
    }

    if (err == 0U)
    {
        printf("Saved HDR%u %s (+%uF, Q8=%lu/%lu/%lu, WL=%u, BEV=%+ld.%02ldEV, %lums)\r\n",
               (unsigned int)(8U + 2U * nf), path_out, (unsigned int)nf,
               (unsigned long)ratio_q8[0], (unsigned long)ratio_q8[1], (unsigned long)ratio_q8[2],
               (unsigned int)wl,
               (long)(bev_x100 / 100), (long)((bev_x100 < 0 ? -bev_x100 : bev_x100) % 100),
               (unsigned long)(HAL_GetTick() - t_all));
    }
    return err;
}

static uint8_t __attribute__((optimize("O2"))) camera_take_hdr_photo(void)
{
    const uint8_t nf = (g_hdr_mode == C3M1_HDR_16B_4F) ? 4U
                     : ((g_hdr_mode == C3M1_HDR_14B_3F) ? 3U : 2U);
    uint32_t last_led_tick = HAL_GetTick();

    /* Respect exact current ISO, shutter (manual or AUTO + EXP COMP + METER MODE), and WB */
    uint8_t  cap_gain_reg = (g_iso_idx == 0U)
                          ? ov7725_gain_q4_to_reg(g_auto_gain_q4)
                          : ov7725_iso_to_gain_reg(g_iso_idx);
    uint8_t  cap_gain_x   = (g_iso_idx == 0U)
                          ? (uint8_t)((g_auto_gain_q4 + 8U) >> 4)
                          : (uint8_t)(1U << (g_iso_idx - 1U));
    if (cap_gain_x < 1U) cap_gain_x = 1U;
    uint16_t bl_g  = c3m1_black_level_g12(cap_gain_x);
    uint16_t bl_rb = c3m1_black_level_rb12(cap_gain_x);

    uint32_t base_lines = ov7725_is_auto_shutter(g_dial_pos)
                        ? (uint32_t)g_auto_aec
                        : ov7725_get_manual_aec_fast(g_dial_pos);
    if (base_lines < 1UL) base_lines = 1UL;

    uint32_t e_req[4];
    hdr_build_bracket(nf, base_lines, e_req);

    c3m1_datetime_t cap_dt;
    char day_dir[24];
    char paths[4][64];
    char path_out[64];
    c3m1_clock_get_datetime(&cap_dt);
    if (!sd_ensure_photo_day_dir(&cap_dt, day_dir, sizeof(day_dir)))
    {
        led1_blink_error();
        return 1U;
    }

    uint16_t prev_count  = shutter_count_load();
    uint16_t count_tmp   = shutter_count_add(prev_count, 1U);
    uint16_t shots_fired = 0U;

    for (uint8_t f = 0U; f < nf; f++)
    {
        snprintf(paths[f], sizeof(paths[f]), "%s/CDI%05u_%u.dng",
                 day_dir, (unsigned int)count_tmp, (unsigned int)f);
    }
    snprintf(path_out, sizeof(path_out), "%s/CDI%05u_hdr.dng", day_dir, (unsigned int)count_tmp);

    c3m1_dng_lens_t cap_lens;
    camera_get_dng_lens(&cap_lens);

    osd_draw_bar("HDR ", YELLOW);
    uint8_t err = hdr_capture_and_merge_one(nf, e_req, base_lines, cap_gain_reg, bl_g, bl_rb,
                                            paths, path_out, 1U, NULL, &cap_lens, "HDR",
                                            &shots_fired, &last_led_tick);

    if (err == 0U && g_cfg.del_hdr_brk)
    {
        for (uint8_t f = 0U; f < nf; f++)
        {
            (void)f_unlink(paths[f]);
        }
    }

    if (shots_fired > 0U)
    {
        uint16_t new_count = shutter_count_add(prev_count, shots_fired);
        ShutterCount = new_count;
        shutter_count_save(new_count);
    }

    vsync_reset_idle();

    if (err != 0U)
    {
        printf("HDR err:%u\r\n", (unsigned int)err);
        led1_blink_error();
        return err;
    }

    LED1(1);
    return 0U;
}

/* ======================== 3-Frame AEB (0 / -2 / +2 EV, No HDR Merge) ======================== */
static uint8_t aeb_save_frame(const char *stem, uint16_t bl_g, uint16_t bl_rb,
                              uint32_t frame_fast_lines, uint32_t base_lines,
                              const c3m1_dng_lens_t *lens, uint32_t *last_led_tick)
{
    c3m1_dng_exif_t fexif;
    camera_build_capture_exif(frame_fast_lines, &fexif);
    int32_t brk_ev_x100 = hdr_ratio_to_ev_x100(frame_fast_lines, base_lines);
    int32_t brk_thirds  = (brk_ev_x100 >= 0) ? ((brk_ev_x100 * 3 + 50) / 100)
                                             : ((brk_ev_x100 * 3 - 50) / 100);
    fexif.ev_num += brk_thirds;

    uint8_t want_dng = (g_cfg.save_mode != C3M1_SAVE_JPG) ? 1U : 0U;
    uint8_t want_jpg = (g_cfg.save_mode != C3M1_SAVE_DNG) ? 1U : 0U;
    char    path[64];

    if (want_dng)
    {
        snprintf(path, sizeof(path), "%s.dng", stem);
        FRESULT fres = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
        if (fres != FR_OK)
        {
            printf("AEB DNG open err:%d (%s)\r\n", fres, path);
            return 0U;
        }
        uint8_t ok   = save_dng_from_fifo(&file, bl_g, bl_rb, &fexif, lens, last_led_tick);
        FRESULT cres = f_close(&file);
        if (!ok || cres != FR_OK)
        {
            return 0U;
        }
    }

    if (want_jpg)
    {
        uint32_t jpg_bytes = 0UL;
        snprintf(path, sizeof(path), "%s.jpg", stem);
        FRESULT fres = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
        if (fres != FR_OK)
        {
            printf("AEB JPG open err:%d (%s)\r\n", fres, path);
            return 0U;
        }
        uint8_t ok   = save_jpg_from_fifo(&file, bl_g, bl_rb, &fexif, last_led_tick, &jpg_bytes);
        FRESULT cres = f_close(&file);
        if (!ok || cres != FR_OK)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t __attribute__((optimize("O2"))) camera_take_aeb_photo(void)
{
    uint32_t t_all = HAL_GetTick();
    uint32_t last_led_tick = t_all;

    /* Respect exact current ISO, shutter (manual or AUTO + EXP COMP + METER MODE), and WB */
    uint8_t  cap_gain_reg = (g_iso_idx == 0U)
                          ? ov7725_gain_q4_to_reg(g_auto_gain_q4)
                          : ov7725_iso_to_gain_reg(g_iso_idx);
    uint8_t  cap_gain_x   = (g_iso_idx == 0U)
                          ? (uint8_t)((g_auto_gain_q4 + 8U) >> 4)
                          : (uint8_t)(1U << (g_iso_idx - 1U));
    if (cap_gain_x < 1U) cap_gain_x = 1U;
    uint16_t bl_g  = c3m1_black_level_g12(cap_gain_x);
    uint16_t bl_rb = c3m1_black_level_rb12(cap_gain_x);

    uint32_t base_lines = ov7725_is_auto_shutter(g_dial_pos)
                        ? (uint32_t)g_auto_aec
                        : ov7725_get_manual_aec_fast(g_dial_pos);
    if (base_lines < 1UL) base_lines = 1UL;

    uint32_t e_req[4];
    uint32_t e_act[3] = { 1UL, 4UL, 16UL };
    hdr_build_bracket(3U, base_lines, e_req);

    c3m1_datetime_t cap_dt;
    char day_dir[24];
    char stems[3][56];
    c3m1_clock_get_datetime(&cap_dt);
    if (!sd_ensure_photo_day_dir(&cap_dt, day_dir, sizeof(day_dir)))
    {
        led1_blink_error();
        return 1U;
    }

    uint16_t prev_count  = shutter_count_load();
    uint16_t count_tmp   = shutter_count_add(prev_count, 1U);
    uint16_t shots_fired = 0U;
    uint8_t  err         = 0U;

    /* Same filename numbering as HDR (_0 = -2EV, _1 = 0EV, _2 = +2EV), without _hdr merge */
    for (uint8_t f = 0U; f < 3U; f++)
    {
        snprintf(stems[f], sizeof(stems[f]), "%s/CDI%05u_%u",
                 day_dir, (unsigned int)count_tmp, (unsigned int)f);
    }

    c3m1_dng_lens_t cap_lens;
    camera_get_dng_lens(&cap_lens);

    osd_draw_bar("AEB ", YELLOW);

    /* Frame 1/3: metered 0 EV (_1) first for minimal lag */
    osd_draw_center_box("AEB 1/3...", YELLOW);
    if (e_req[1] == base_lines)
    {
        hdr_capture_metered_frame();
        e_act[1] = base_lines;
    }
    else
    {
        hdr_capture_frame_to_fifo(e_req[1], cap_gain_reg, &e_act[1]);
    }
    shots_fired++;
    if (!aeb_save_frame(stems[1], bl_g, bl_rb, e_act[1], base_lines, &cap_lens, &last_led_tick))
    {
        err = 1U;
    }

    /* Frame 2/3: underexposed -2 EV (_0) */
    if (err == 0U)
    {
        osd_draw_center_box("AEB 2/3...", YELLOW);
        hdr_capture_frame_to_fifo(e_req[0], cap_gain_reg, &e_act[0]);
        shots_fired++;
        if (!aeb_save_frame(stems[0], bl_g, bl_rb, e_act[0], base_lines, &cap_lens, &last_led_tick))
        {
            err = 2U;
        }
    }

    /* Frame 3/3: overexposed +2 EV (_2) */
    if (err == 0U)
    {
        osd_draw_center_box("AEB 3/3...", YELLOW);
        hdr_capture_frame_to_fifo(e_req[2], cap_gain_reg, &e_act[2]);
        shots_fired++;
    }

    /* Restore normal viewfinder exposure while AL422B holds frame 3 (or on early error) */
    raw_apply_preview_exposure(1U);

    if (err == 0U)
    {
        if (!aeb_save_frame(stems[2], bl_g, bl_rb, e_act[2], base_lines, &cap_lens, &last_led_tick))
        {
            err = 3U;
        }
    }

    if (shots_fired > 0U)
    {
        uint16_t new_count = shutter_count_add(prev_count, shots_fired);
        ShutterCount = new_count;
        shutter_count_save(new_count);
    }

    vsync_reset_idle();

    if (err != 0U)
    {
        printf("AEB err:%u\r\n", (unsigned int)err);
        led1_blink_error();
        return err;
    }

    printf("Saved AEB3 %s/CDI%05u_0..2 (+3F, %lums)\r\n",
           day_dir, (unsigned int)count_tmp, (unsigned long)(HAL_GetTick() - t_all));
    LED1(1);
    return 0U;
}

/* ======================== JSR Super-Resolution Burst (+ Dark Frame) ======================== */
static void jsr_latch_next_frame(uint32_t actual_lines)
{
    vsync_reset_idle();

    uint32_t frame_lines = (actual_lines > 490UL) ? (actual_lines + 50UL) : 510UL;
    uint32_t max_wait_ms = (frame_lines * 2000UL) / 22959UL + 600UL;
    (void)wait_vsync_state(2U, max_wait_ms);

    __disable_irq();
    OV7725_WEN(0);
    g_vsync_state = 2;
    __enable_irq();
}

/*
 * Modal prompt after N light frames finish: instructs user to completely cover the lens
 * and press SHOT for the matched-exposure dark frame (or ASA to skip).
 * Returns 1 = capture dark frame, 0 = user skipped, 2 = SD removed.
 */
static uint8_t jsr_wait_dark_frame_prompt(uint8_t saved_lights, uint8_t total_lights)
{
    char done_msg[36];
    snprintf(done_msg, sizeof(done_msg), "Saved %u/%u light frames",
             (unsigned int)saved_lights, (unsigned int)total_lights);

    g_back_color = BLACK;
    lcd_fill(14, 42, 305, 198, BLACK);
    lcd_draw_rectangle(14, 42, 305, 198, YELLOW);
    lcd_draw_hline(15, 70, 290, CYAN);
    lcd_show_string(104, 48, 160, 16, 16, "JSR DARK FRAME", CYAN);
    lcd_show_string(76,  80, 180, 16, 16, "COVER LENS COMPLETELY", RED);
    lcd_show_string(67, 106, 200, 12, 12, "Block all light before shooting", YELLOW);
    lcd_show_string(67, 130, 200, 12, 12, "SHOT : Capture final dark frame", WHITE);
    lcd_show_string(94, 148, 160, 12, 12, "ASA  : Skip dark frame", GRAY);
    lcd_show_string((uint16_t)(160U - strlen(done_msg) * 3U), 174, 240, 12, 12, done_msg, GREEN);
    osd_draw_bar("DARK", YELLOW);

    key_init_state();
    uint32_t last_led_tick = HAL_GetTick();
    LED1(0);

    for (;;)
    {
        if (!sd_card_is_present())
        {
            LED1(1);
            g_sd_ready = 0;
            lcd_fill(14, 42, 305, 198, BLACK);
            osd_show_toast("NO SD CARD!", RED, TOAST_DURATION_MS);
            return 2U;
        }

        uint32_t now = HAL_GetTick();
        g_last_activity_tick = now; /* Never auto-sleep while user is covering lens */

        uint8_t evts = key_poll_events();
        if (evts & BTN_EVT_SHOT)
        {
            key_init_state();
            LED1(0);
            lcd_fill(14, 42, 305, 198, BLACK);
            return 1U;
        }
        if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
        {
            key_init_state();
            LED1(1);
            lcd_fill(14, 42, 305, 198, BLACK);
            return 0U;
        }

        if ((now - last_led_tick) >= 500UL)
        {
            LED1_TOGGLE();
            last_led_tick = now;
        }

        delay_ms(20);
    }
}

static uint8_t __attribute__((optimize("O2"))) camera_take_jsr_burst(void)
{
    const uint8_t n_lights = jsr_frame_count(g_jsr_mode);
    const uint8_t use_hdr  = (g_hdr_mode != C3M1_HDR_OFF) ? 1U : 0U;
    const uint8_t hdr_nf   = (g_hdr_mode == C3M1_HDR_16B_4F) ? 4U
                           : ((g_hdr_mode == C3M1_HDR_14B_3F) ? 3U : 2U);
    uint32_t t_all         = HAL_GetTick();
    uint32_t last_led_tick = t_all;

    /* Lock ISO, shutter lines, black levels, and AWB gains ONCE for the entire burst + dark frame */
    uint8_t  cap_gain_reg = (g_iso_idx == 0U)
                          ? ov7725_gain_q4_to_reg(g_auto_gain_q4)
                          : ov7725_iso_to_gain_reg(g_iso_idx);
    uint8_t  cap_gain_x   = (g_iso_idx == 0U)
                          ? (uint8_t)((g_auto_gain_q4 + 8U) >> 4)
                          : (uint8_t)(1U << (g_iso_idx - 1U));
    if (cap_gain_x < 1U) cap_gain_x = 1U;
    uint16_t bl_g  = c3m1_black_level_g12(cap_gain_x);
    uint16_t bl_rb = c3m1_black_level_rb12(cap_gain_x);

    uint32_t base_lines = ov7725_is_auto_shutter(g_dial_pos)
                        ? (uint32_t)g_auto_aec
                        : ov7725_get_manual_aec_fast(g_dial_pos);
    if (base_lines < 1UL) base_lines = 1UL;

    uint32_t e_req[4];
    if (use_hdr)
    {
        hdr_build_bracket(hdr_nf, base_lines, e_req);
    }

    c3m1_datetime_t cap_dt;
    char day_dir[24];
    char jsr_dir[40];
    char path_out[64];
    char sub_paths[4][64];

    c3m1_clock_get_datetime(&cap_dt);
    if (!sd_ensure_photo_day_dir(&cap_dt, day_dir, sizeof(day_dir)))
    {
        led1_blink_error();
        return 1U;
    }

    uint16_t prev_count  = shutter_count_load();
    uint16_t first_id    = shutter_count_add(prev_count, 1U);
    uint16_t cur_id      = first_id;
    uint16_t shots_fired = 0U;

    /* Folder named after the first frame + burst count suffix: CDIxxxxx_7 / _14 / _30 */
    snprintf(jsr_dir, sizeof(jsr_dir), "%s/CDI%05u_%u",
             day_dir, (unsigned int)first_id, (unsigned int)n_lights);
    FRESULT dres = f_mkdir(jsr_dir);
    if (dres != FR_OK && dres != FR_EXIST)
    {
        printf("JSR mkdir err:%d (%s)\r\n", dres, jsr_dir);
        led1_blink_error();
        return 1U;
    }

    c3m1_dng_lens_t cap_lens;
    camera_get_dng_lens(&cap_lens);

    hdr_lock_t hdr_lock;
    memset(&hdr_lock, 0, sizeof(hdr_lock));

    uint16_t hdr_set_ids[31]; /* Track completed HDR set IDs (<=30 light + 1 dark) for post-burst cleanup */
    uint8_t  hdr_set_cnt   = 0U;
    uint32_t locked_actual = base_lines;
    uint8_t  saved_lights  = 0U;
    uint8_t  err           = 0U;
    uint8_t  cancelled     = 0U;
    char     step_msg[24];

    osd_draw_bar("JSR ", CYAN);
    key_init_state();

    /* 1. Capture N locked-exposure light frames inside CDIxxxxx_<N>/ (identical naming to normal / HDR) */
    for (uint8_t i = 0U; i < n_lights; i++)
    {
        if (!sd_card_is_present())
        {
            g_sd_ready = 0;
            err = 1U;
            break;
        }
        if (i > 0U && (key_poll_events() & (BTN_EVT_ISO | BTN_EVT_ISO_LONG)))
        {
            cancelled = 1U;
            break;
        }

        g_last_activity_tick = HAL_GetTick();

        if (use_hdr)
        {
            char prefix[16];
            snprintf(prefix, sizeof(prefix), "J%u/%u", (unsigned int)(i + 1U), (unsigned int)n_lights);
            for (uint8_t f = 0U; f < hdr_nf; f++)
            {
                snprintf(sub_paths[f], sizeof(sub_paths[f]), "%s/CDI%05u_%u.dng",
                         jsr_dir, (unsigned int)cur_id, (unsigned int)f);
            }
            snprintf(path_out, sizeof(path_out), "%s/CDI%05u_hdr.dng", jsr_dir, (unsigned int)cur_id);
            err = hdr_capture_and_merge_one(hdr_nf, e_req, base_lines, cap_gain_reg, bl_g, bl_rb,
                                            sub_paths, path_out, 0U, &hdr_lock, &cap_lens, prefix,
                                            &shots_fired, &last_led_tick);
            if (err != 0U)
            {
                break;
            }
            if (hdr_set_cnt < 31U)
            {
                hdr_set_ids[hdr_set_cnt++] = cur_id;
            }
            cur_id = shutter_count_add(cur_id, hdr_nf);
        }
        else
        {
            snprintf(step_msg, sizeof(step_msg), "JSR %u/%u...",
                     (unsigned int)(i + 1U), (unsigned int)n_lights);
            osd_draw_center_box(step_msg, YELLOW);
            snprintf(path_out, sizeof(path_out), "%s/CDI%05u.dng", jsr_dir, (unsigned int)cur_id);

            if (i == 0U)
            {
                hdr_capture_frame_to_fifo(base_lines, cap_gain_reg, &locked_actual);
            }
            else
            {
                jsr_latch_next_frame(locked_actual);
            }
            shots_fired++;

            if (!hdr_save_frame_dng(path_out, bl_g, bl_rb, locked_actual, base_lines, &cap_lens, &last_led_tick))
            {
                err = 2U;
                break;
            }
            cur_id = shutter_count_add(cur_id, 1U);
        }
        saved_lights++;
    }

    /* 2. Prompt user to cover lens completely and press SHOT for matched dark frame (CDIxxxxx_dark.dng, no lens opcodes) */
    uint8_t dark_saved = 0U;
    if (err == 0U && !cancelled)
    {
        uint8_t choice = jsr_wait_dark_frame_prompt(saved_lights, n_lights);
        if (choice == 2U)
        {
            err = 1U;
        }
        else if (choice == 1U)
        {
            snprintf(path_out, sizeof(path_out), "%s/CDI%05u_dark.dng", jsr_dir, (unsigned int)cur_id);
            osd_draw_bar("DARK", YELLOW);
            if (use_hdr)
            {
                for (uint8_t f = 0U; f < hdr_nf; f++)
                {
                    snprintf(sub_paths[f], sizeof(sub_paths[f]), "%s/CDI%05u_%u.dng",
                             jsr_dir, (unsigned int)cur_id, (unsigned int)f);
                }
                err = hdr_capture_and_merge_one(hdr_nf, e_req, base_lines, cap_gain_reg, bl_g, bl_rb,
                                                sub_paths, path_out, 0U, &hdr_lock, NULL, "DARK",
                                                &shots_fired, &last_led_tick);
                if (err == 0U && hdr_set_cnt < 31U)
                {
                    hdr_set_ids[hdr_set_cnt++] = cur_id;
                    cur_id = shutter_count_add(cur_id, hdr_nf);
                }
            }
            else
            {
                osd_draw_center_box("DARK FRAME...", YELLOW);
                hdr_capture_frame_to_fifo(base_lines, cap_gain_reg, &locked_actual);
                shots_fired++;
                if (!hdr_save_frame_dng(path_out, bl_g, bl_rb, locked_actual, base_lines, NULL, &last_led_tick))
                {
                    err = 3U;
                }
                else
                {
                    cur_id = shutter_count_add(cur_id, 1U);
                }
            }
            if (err == 0U)
            {
                dark_saved = 1U;
            }
        }
    }

    /* 3. After ALL frames & merges finish, optionally delete HDR bracket sub-frames (_0.._(nf-1)) */
    if (use_hdr && g_cfg.del_hdr_brk && hdr_set_cnt > 0U && sd_card_is_present())
    {
        osd_draw_center_box("CLEANING...", CYAN);
        for (uint8_t k = 0U; k < hdr_set_cnt; k++)
        {
            for (uint8_t f = 0U; f < hdr_nf; f++)
            {
                snprintf(path_out, sizeof(path_out), "%s/CDI%05u_%u.dng",
                         jsr_dir, (unsigned int)hdr_set_ids[k], (unsigned int)f);
                (void)f_unlink(path_out);
            }
        }
    }

    /* Restore normal viewfinder exposure & persist exact exposed frame count in one Flash write */
    raw_apply_preview_exposure(1U);
    if (shots_fired > 0U)
    {
        uint16_t new_count = shutter_count_add(prev_count, shots_fired);
        ShutterCount = new_count;
        shutter_count_save(new_count);
    }

    vsync_reset_idle();

    if (cancelled)
    {
        LED1(1);
        snprintf(step_msg, sizeof(step_msg), "JSR STOP (%uF)", (unsigned int)saved_lights);
        osd_show_toast(step_msg, ORANGE, 1200U);
        printf("JSR cancelled at %u/%u frames (%s)\r\n",
               (unsigned int)saved_lights, (unsigned int)n_lights, jsr_dir);
        return C3M1_CAPTURE_CANCELLED;
    }

    if (err != 0U)
    {
        printf("JSR err:%u after %u/%u frames (%s)\r\n",
               (unsigned int)err, (unsigned int)saved_lights, (unsigned int)n_lights, jsr_dir);
        led1_blink_error();
        return err;
    }

    LED1(1);
    if (dark_saved)
    {
        snprintf(step_msg, sizeof(step_msg), "JSR %uF+DARK OK", (unsigned int)saved_lights);
    }
    else
    {
        snprintf(step_msg, sizeof(step_msg), "JSR %uF (NO DARK)", (unsigned int)saved_lights);
    }
    osd_show_toast(step_msg, GREEN, 1500U);
    printf("Saved JSR %s (%u light%s, HDR=%u, ISO=%ux, %lums)\r\n",
           jsr_dir, (unsigned int)saved_lights, dark_saved ? "+1 dark" : " (no dark)",
           (unsigned int)(use_hdr ? (8U + 2U * hdr_nf) : 10U),
           (unsigned int)cap_gain_x, (unsigned long)(HAL_GetTick() - t_all));
    return 0U;
}

/*
 * Non-blocking 3-second tripod self-timer before HDR / JSR capture:
 *   - Displays "<TAG> DELAY 3s/2s/1s" with classic self-timer LED1 blink cadence
 *     (slow 2 Hz on 3s/2s, fast 5 Hz on final 1s).
 *   - Pressing ASA, FILM lever, or 4X cancels cleanly without tripping shutter.
 *   - Returns 1 when countdown finishes, 0 if cancelled or SD removed.
 */
static uint8_t hdr_wait_shutter_delay(const char *tag)
{
    uint32_t start_tick    = HAL_GetTick();
    uint32_t last_led_tick = start_tick;
    uint32_t last_sec      = 0UL;

    key_init_state();
    osd_draw_bar("DLY ", YELLOW);
    LED1(0); /* Start ON */

    for (;;)
    {
        if (!sd_card_is_present())
        {
            LED1(1);
            g_sd_ready = 0;
            osd_show_toast("NO SD CARD!", RED, TOAST_DURATION_MS);
            return 0U;
        }

        uint8_t evts = key_poll_events();
        if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG |
                    BTN_EVT_FILM | BTN_EVT_FILM_LONG |
                    BTN_EVT_4X | BTN_EVT_4X_LONG))
        {
            char cmsg[24];
            LED1(1);
            key_init_state();
            snprintf(cmsg, sizeof(cmsg), "%s CANCELLED", tag);
            osd_show_toast(cmsg, ORANGE, 800U);
            return 0U;
        }

        uint32_t now     = HAL_GetTick();
        uint32_t elapsed = now - start_tick;
        if (elapsed >= HDR_SHUTTER_DELAY_MS)
        {
            break;
        }

        uint32_t rem_ms  = HDR_SHUTTER_DELAY_MS - elapsed;
        uint32_t sec_rem = (rem_ms + 999UL) / 1000UL;
        if (sec_rem != last_sec)
        {
            char msg[24];
            last_sec = sec_rem;
            snprintf(msg, sizeof(msg), "%s DELAY %lus", tag, (unsigned long)sec_rem);
            osd_draw_center_box(msg, YELLOW);
        }

        uint32_t blink_ms = (rem_ms <= 1000UL) ? 100UL : 250UL;
        if ((now - last_led_tick) >= blink_ms)
        {
            LED1_TOGGLE();
            last_led_tick = now;
        }

        delay_ms(10);
    }

    key_init_state();
    LED1(0); /* Solid ON as capture begins */
    return 1U;
}

/* Dismiss slow-shutter EXPOSING... box by painting the latched capture above the bottom HUD (~25-45 ms),
 * applying the same 1X lens distortion warp, vignette LUT, and U1 AA as the live viewfinder. */
static void __attribute__((optimize("O2"))) camera_show_latched_above_bar(void)
{
    uint16_t prev_boost = g_lut_last_boost_q8;
    uint8_t  prev_gx    = g_lut_last_gain_x;
    uint8_t  cap_gx     = (uint8_t)((camera_get_capture_iso() + 50U) / 100U);
    if (cap_gx < 1U) cap_gx = 1U;

    raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, 256U, cap_gx);

    const uint16_t pairs    = OSD_BAR_Y / 2U;         /* 110 pairs = 220 rows above bottom HUD */
    const uint16_t pair_cnt = VIEWFINDER_HEIGHT / 2U; /* 120 full source pairs (480 RAW rows) */
    const int8_t   vf_dist  = g_cfg.lens_dist_incam ? g_cfg.lens_distort : 0;
    const uint8_t  vf_vig   = g_cfg.lens_vignette;
    if (vf_vig > 0U)
    {
        vf_vig_rebuild_lut(vf_vig);
    }

    uint32_t scratch_sz = 0UL;
    uint8_t *scratch    = c3m1_jpeg_get_scratch_buf(&scratch_sz);
    int32_t  vf_kr0_q16 = 65536L, vf_kr1_q16 = 0L;
    uint8_t *vf_ring    = NULL;
    const uint16_t *vf_ring_rows[16];
    uint16_t next_src_yp = 0U;
    if (vf_dist != 0 && scratch != NULL && scratch_sz >= (16UL * 640UL))
    {
        vf_ring = scratch;
        for (uint8_t i = 0U; i < 16U; i++)
        {
            vf_ring_rows[i] = (const uint16_t *)(vf_ring + (uint32_t)i * 640UL);
        }
        c3m1_lens_dist_params_q16(vf_dist, &vf_kr0_q16, &vf_kr1_q16);
    }

    const uint8_t aa_en = (g_cfg.anti_alias && scratch != NULL &&
                           scratch_sz >= (16UL * 640UL + 2UL * 640UL)) ? 1U : 0U;

    fifo_prepare_read();
    lcd_set_window(0U, 0U, 320U, OSD_BAR_Y);
    lcd_write_ram_prepare();

    if (aa_en)
    {
        uint8_t *aa_clean[2] = { scratch + (16UL * 640UL), scratch + (17UL * 640UL) };
        uint8_t *line1       = s_line_buf[1];

        /* Produce yp = 0 clean row into aa_clean[0] */
        if (vf_ring != NULL)
        {
            uint16_t req = vf_req_max_src_yp(0U, vf_kr0_q16, vf_kr1_q16);
            while (next_src_yp <= req && next_src_yp < pair_cnt)
            {
                uint8_t *dst_src = vf_ring + (uint32_t)(next_src_yp & 15U) * 640UL;
                fifo_read_r_320(s_raw_row_buf);
                fifo_skip_4();
                if (vf_vig > 0U) raw_demosaic_320_to_ring_vig(s_raw_row_buf, dst_src, next_src_yp);
                else             raw_demosaic_320_to_ring(s_raw_row_buf, dst_src);
                fifo_skip_4();
                if (next_src_yp + 1U < pair_cnt) fifo_skip_bytes(2UL * 644UL);
                next_src_yp++;
            }
            vf_warp_row_1x(aa_clean[0], NULL, vf_ring_rows, 0U, vf_kr0_q16, vf_kr1_q16);
        }
        else
        {
            fifo_read_r_320(s_raw_row_buf);
            fifo_skip_4();
            if (vf_vig > 0U) raw_demosaic_320_to_buf_vig(s_raw_row_buf, aa_clean[0], s_peak_g[0], 0U);
            else             raw_demosaic_320_to_buf(s_raw_row_buf, aa_clean[0], s_peak_g[0]);
            fifo_skip_4();
            fifo_skip_bytes(2UL * 644UL);
        }

        for (uint16_t yp = 0U; yp < pairs; yp++)
        {
            uint8_t *cur_clean  = aa_clean[yp & 1U];
            uint8_t *next_clean = aa_clean[(yp ^ 1U) & 1U];
            uint16_t nyp        = (uint16_t)(yp + 1U);

            /* Produce next_clean in parallel with prev y1 DMA in line1, blend y1, then DMA cur_clean directly */
            if (vf_ring != NULL)
            {
                uint16_t req = vf_req_max_src_yp(nyp, vf_kr0_q16, vf_kr1_q16);
                while (next_src_yp <= req && next_src_yp < pair_cnt)
                {
                    uint8_t *dst_src = vf_ring + (uint32_t)(next_src_yp & 15U) * 640UL;
                    fifo_read_r_320(s_raw_row_buf);
                    fifo_skip_4();
                    if (vf_vig > 0U) raw_demosaic_320_to_ring_vig(s_raw_row_buf, dst_src, next_src_yp);
                    else             raw_demosaic_320_to_ring(s_raw_row_buf, dst_src);
                    fifo_skip_4();
                    if (next_src_yp + 1U < pair_cnt) fifo_skip_bytes(2UL * 644UL);
                    next_src_yp++;
                }
                vf_warp_row_1x(next_clean, NULL, vf_ring_rows, nyp, vf_kr0_q16, vf_kr1_q16);
            }
            else
            {
                fifo_read_r_320(s_raw_row_buf);
                fifo_skip_4();
                if (vf_vig > 0U) raw_demosaic_320_to_buf_vig(s_raw_row_buf, next_clean, s_peak_g[0], nyp);
                else             raw_demosaic_320_to_buf(s_raw_row_buf, next_clean, s_peak_g[0]);
                fifo_skip_4();
                if (nyp + 1U <= pairs) fifo_skip_bytes(2UL * 644UL);
            }

            lcd_dma_wait();
            vf_avg_row_be(line1, cur_clean, next_clean);
            lcd_dma_send(cur_clean, 640U);
            lcd_dma_send(line1, 640U);
        }
    }
    else
    {
        uint8_t buf_idx = 0U;
        if (vf_ring != NULL)
        {
            for (uint16_t yp = 0U; yp < pairs; yp++)
            {
                uint16_t req = vf_req_max_src_yp(yp, vf_kr0_q16, vf_kr1_q16);
                while (next_src_yp <= req && next_src_yp < pair_cnt)
                {
                    uint8_t *dst_src = vf_ring + (uint32_t)(next_src_yp & 15U) * 640UL;
                    fifo_read_r_320(s_raw_row_buf);
                    fifo_skip_4();
                    if (vf_vig > 0U) raw_demosaic_320_to_ring_vig(s_raw_row_buf, dst_src, next_src_yp);
                    else             raw_demosaic_320_to_ring(s_raw_row_buf, dst_src);
                    fifo_skip_4();
                    if (next_src_yp + 1U < pair_cnt)
                    {
                        fifo_skip_bytes(2UL * 644UL);
                    }
                    next_src_yp++;
                }

                uint8_t *cur_line = s_line_buf[buf_idx];
                vf_warp_row_1x(cur_line, NULL, vf_ring_rows, yp, vf_kr0_q16, vf_kr1_q16);
                lcd_dma_send(cur_line, 640U);
                lcd_dma_send(cur_line, 640U);
                buf_idx ^= 1U;
            }
        }
        else
        {
            for (uint16_t yp = 0U; yp < pairs; yp++)
            {
                uint8_t *cur_line = s_line_buf[buf_idx];
                fifo_read_r_320(s_raw_row_buf);
                fifo_skip_4();
                if (vf_vig > 0U) raw_demosaic_320_to_buf_vig(s_raw_row_buf, cur_line, s_peak_g[0], yp);
                else             raw_demosaic_320_to_buf(s_raw_row_buf, cur_line, s_peak_g[0]);
                fifo_skip_4();

                lcd_dma_send(cur_line, 640U);
                if (yp + 1U < pairs)
                {
                    fifo_skip_bytes(2UL * 644UL);
                }
                lcd_dma_send(cur_line, 640U);
                buf_idx ^= 1U;
            }
        }
    }
    lcd_dma_wait();

    raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, prev_boost, prev_gx);
}

static uint8_t camera_capture_one_to_fifo(uint8_t is_first_frame, const char *save_bar_label)
{
    if (camera_needs_slow_capture())
    {
        /*
         * Slow shutter (1/25..2s or AUTO slow shutter): preview was capped at 500 lines for live viewing.
         * Write the true DM_LN/AEC/GAIN registers atomically right after a VSYNC (NO sensor reset!),
         * integrate one exact slow-shutter frame, latch its 480 active rows inside VBLANK,
         * and restore live preview registers in parallel with SD write.
         */
        uint8_t show_exposing = (!ov7725_is_auto_shutter(g_dial_pos) && g_dial_pos >= 10U) ? 1U : 0U;
        if (show_exposing)
        {
            osd_draw_bar("EXP ", YELLOW);
            osd_draw_center_box("EXPOSING...", YELLOW);
        }
        else
        {
            osd_draw_bar(save_bar_label, YELLOW);
        }

        uint8_t ok = camera_capture_slow_to_fifo();

        /* Begin switching sensor back to preview while AL422B holds the latched frame */
        raw_apply_preview_exposure(1U);
        if (!ok)
        {
            return 0U;
        }
        osd_draw_bar(save_bar_label, YELLOW);
        /* Paint the exact latched capture above the bottom bar (~20 ms) so frozen screen == saved file */
        camera_show_latched_above_bar();
        return 1U;
    }
    else
    {
        /*
         * Fast shutters (1/8000..1/25) & AUTO: OV7725 is ALREADY running in 644x480 GRBG RAW
         * at the exact physical capture exposure -> true zero-shutter-lag (0..22ms)!
         * Latch g_vsync_state == 2 first, then paint that exact locked FIFO frame to the LCD
         * (~20 ms) before SD write so the frozen frame on screen is guaranteed identical to the file.
         */
        osd_draw_bar(save_bar_label, YELLOW);
        if (is_first_frame)
        {
            if (g_exposure_dirty_frames > 0U)
            {
                vsync_reset_idle();
                wait_skip_frames(1U);
                g_exposure_dirty_frames = 0U;
            }
        }
        else
        {
            /* Subsequent continuous burst shots must latch a brand-new frame */
            vsync_reset_idle();
        }

        if (!wait_vsync_state(2U, 500UL))
        {
            return 0U;
        }
        camera_show_latched_above_bar();
        return 1U;
    }
}

static uint8_t camera_take_burst_photo(void)
{
    uint16_t prev_count  = shutter_count_load();
    uint16_t burst_count = prev_count;
    uint16_t burst_idx   = 0U;
    uint8_t  err         = 0U;

    for (;;)
    {
        char bar_lbl[8];
        burst_idx++;
        uint16_t disp_n = (burst_idx > 999U) ? 999U : burst_idx;
        snprintf(bar_lbl, sizeof(bar_lbl), "B%03u", (unsigned int)disp_n);

        LED1(0);
        if (!camera_capture_one_to_fifo((burst_idx == 1U) ? 1U : 0U, bar_lbl))
        {
            vsync_reset_idle();
            led1_blink_error();
            err = C3M1_CAPTURE_SENSOR_TIMEOUT;
            break;
        }
        err = ov7725_camera_output_ex(&burst_count, 0U);
        if (err != 0U)
        {
            break;
        }

        if (!key_is_shot_pressed())
        {
            break;
        }
        if (!sd_card_is_present())
        {
            g_sd_ready = 0;
            break;
        }
        if ((burst_idx & 3U) == 0U)
        {
            g_bat_mv = adc_battery_get_mv();
            if (battery_is_cutoff_low())
            {
                break;
            }
        }
    }

    if (burst_count != prev_count)
    {
        ShutterCount = burst_count;
        shutter_count_save(burst_count);
    }

    /* Sync debounced key state cleanly (avoids ghost re-trigger if stopped early while still held) */
    key_init_state();

    if (err == 0U)
    {
        LED1(1);
    }
    return err;
}

static void camera_take_photo(void)
{
    /* Completely ignore shutter presses while film advance lever is uncocked (no stutter) */
    if (g_cfg.film_lever && !g_shutter_cocked)
    {
        return;
    }

    /* Protect SD card from mid-write brownout when real Li-ion battery is critically low */
    if (battery_is_cutoff_low())
    {
        osd_show_toast("LOW BATTERY!", RED, TOAST_DURATION_MS);
        g_osd_dirty = 1;
        return;
    }

    if (!g_sd_ready || !sd_card_is_present())
    {
        g_sd_ready = 0;
        if (!sd_try_mount())
        {
            osd_show_toast("NO SD CARD!", RED, TOAST_DURATION_MS);
            g_osd_dirty = 1;
            return;
        }
    }

    /* Sync any just-changed shutter dial or pending AE update BEFORE capture so sensor == EXIF/BlackLevel */
    uint8_t dial_now = key_read_shutter_dial();
    if (dial_now != g_dial_pos)
    {
        uint8_t prev_dial = g_dial_pos;
        g_dial_pos = dial_now;
        ae_seed_on_mode_transition(prev_dial, g_iso_idx);
        g_last_auto_s_label[0] = '\0';
        g_last_auto_i_label[0] = '\0';
        g_pending_ae_apply = 0U;
        raw_apply_preview_exposure(1U);
        g_osd_dirty = 1;
    }
    else if (g_pending_ae_apply != 0U)
    {
        g_pending_ae_apply = 0U;
        raw_apply_preview_exposure(1U);
    }

    LED1(0); /* Solid ON during exposure / frame latching (defer pending cfg save until after capture) */

    uint8_t err;
    if (g_jsr_mode != C3M1_JSR_OFF)
    {
        if (!hdr_wait_shutter_delay("JSR"))
        {
            vsync_reset_idle();
            g_osd_dirty = 1;
            return;
        }
        err = camera_take_jsr_burst();
    }
    else if (g_hdr_mode != C3M1_HDR_OFF)
    {
        if (!hdr_wait_shutter_delay("HDR"))
        {
            vsync_reset_idle();
            g_osd_dirty = 1;
            return;
        }
        err = camera_take_hdr_photo();
    }
    else if (g_drive_mode == C3M1_DRIVE_AEB)
    {
        err = camera_take_aeb_photo();
    }
    else if (g_drive_mode == C3M1_DRIVE_BURST)
    {
        err = camera_take_burst_photo();
    }
    else
    {
        if (!camera_capture_one_to_fifo(1U, "SAVE"))
        {
            vsync_reset_idle();
            led1_blink_error();
            err = C3M1_CAPTURE_SENSOR_TIMEOUT;
        }
        else
        {
            err = ov7725_camera_output();
        }
    }
    if (err == 0)
    {
        if (g_cfg.film_lever)
        {
            g_shutter_cocked = 0;
            g_wind_box_drawn = 0;
        }
        /* Refresh SD-persisted clock timestamp after each capture (guards against sudden SW4 switch-off) */
        if (g_sd_ready && sd_card_is_present() && !battery_is_cutoff_low())
        {
            g_cfg_save_due = 0U;
            c3m1_cfg_save(&g_cfg);
        }
    }
    else if (err == C3M1_CAPTURE_SENSOR_TIMEOUT)
    {
        osd_show_toast("SENSOR TIMEOUT!", RED, TOAST_DURATION_MS);
    }
    else if (err != C3M1_CAPTURE_CANCELLED)
    {
        osd_show_toast("SD WRITE ERROR!", RED, TOAST_DURATION_MS);
    }

    if (g_sd_ready && sd_card_is_present())
    {
        sd_refresh_free_kb();
    }

    g_pending_btn_evts = BTN_EVT_NONE;
    vsync_reset_idle();
    g_osd_dirty = 1;
}

static void camera_open_menu(void)
{
    if (!g_sd_ready || !sd_card_is_present())
    {
        osd_show_toast("NO SD CARD!", RED, TOAST_DURATION_MS);
        g_osd_dirty = 1;
        return;
    }

    /* Flush any pending ISO save before entering menu */
    if (g_cfg_save_due != 0U)
    {
        g_cfg_save_due = 0U;
        if (!battery_is_cutoff_low())
        {
            c3m1_cfg_save(&g_cfg);
        }
    }

    /* Freeze FIFO recording cleanly while Menu owns LCD and SDIO */
    __disable_irq();
    OV7725_WEN(0);
    g_vsync_state = 2;
    __enable_irq();

    c3m1_menu_ctx_t mctx = {
        .cfg              = &g_cfg,
        .drive_mode       = &g_drive_mode,
        .hdr_mode         = &g_hdr_mode,
        .jsr_mode         = &g_jsr_mode,
        .fs               = &fs,
        .work_buf         = s_sd_io_buf,
        .work_buf_size    = (uint16_t)sizeof(s_sd_io_buf),
        .shutter_count    = &ShutterCount,
        .remount_sd_fn    = sd_remount_after_format,
        .is_low_bat_fn    = battery_is_cutoff_low,
        .is_sd_present_fn = sd_card_is_present,
        .get_cnt_mode_fn  = shutter_cnt_get_mode,
        .get_cnt_slots_fn = shutter_cnt_get_slots,
        .set_cnt_mode_fn  = shutter_cnt_set_mode,
        .sd_removed_in_menu = 0U,
    };
    c3m1_menu_run(&mctx);

    if (mctx.sd_removed_in_menu)
    {
        f_mount(NULL, "0:", 0);
        MX_SDIO_SD_DeInit();
        g_sd_ready    = 0;
        g_sd_last_pin = 0;
        g_sd_free_kb  = 0UL;
        osd_show_toast("SD REMOVED", ORANGE, 800);
    }
    else if (g_sd_ready && sd_card_is_present())
    {
        sd_refresh_free_kb();
    }

    uint8_t prev_dial = g_dial_pos;
    cfg_apply_to_runtime();
    g_dial_pos = key_read_shutter_dial();
    ae_seed_on_mode_transition(prev_dial, g_iso_idx);
    g_pending_ae_apply = 0U;
    raw_apply_preview_exposure(1U);
    g_wind_box_drawn = 0U;

    g_pending_btn_evts = BTN_EVT_NONE;
    vsync_reset_idle();
    g_osd_dirty = 1;
}

static void camera_open_playback(void)
{
    if (!g_sd_ready || !sd_card_is_present())
    {
        osd_show_toast("NO SD CARD!", RED, TOAST_DURATION_MS);
        g_osd_dirty = 1;
        return;
    }

    /* Flush any pending ISO save before entering playback */
    if (g_cfg_save_due != 0U)
    {
        g_cfg_save_due = 0U;
        if (!battery_is_cutoff_low())
        {
            c3m1_cfg_save(&g_cfg);
        }
    }

    /* Freeze FIFO recording cleanly while Playback owns LCD and SDIO */
    __disable_irq();
    OV7725_WEN(0);
    g_vsync_state = 2;
    __enable_irq();

    c3m1_pb_ctx_t pctx = {
        .photo_dir        = C3M1_PHOTO_DIR,
        .srgb_lut         = s_srgb_lut,
        .work_buf         = s_sd_io_buf,
        .work_buf_size    = (uint16_t)sizeof(s_sd_io_buf),
        .is_low_bat_fn    = battery_is_cutoff_low,
        .is_sd_present_fn = sd_card_is_present,
        .skip_hdr_brk     = g_cfg.pb_skip_brk,
        .high_iso_nr      = g_cfg.high_iso_nr,
        .sd_removed_in_pb = 0U,
    };
    c3m1_pb_run(&pctx);

    if (pctx.sd_removed_in_pb)
    {
        f_mount(NULL, "0:", 0);
        MX_SDIO_SD_DeInit();
        g_sd_ready    = 0;
        g_sd_last_pin = 0;
        g_sd_free_kb  = 0UL;
        osd_show_toast("SD REMOVED", ORANGE, 800);
    }
    else if (g_sd_ready && sd_card_is_present())
    {
        sd_refresh_free_kb();
    }

    uint8_t prev_dial = g_dial_pos;
    g_dial_pos = key_read_shutter_dial();
    ae_seed_on_mode_transition(prev_dial, g_iso_idx);
    g_pending_ae_apply = 0U;
    raw_apply_preview_exposure(1U);
    g_wind_box_drawn = 0U;

    g_pending_btn_evts = BTN_EVT_NONE;
    vsync_reset_idle();
    g_osd_dirty = 1;
}

static void sysclk_enter_hsi_8mhz(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_NONE;
    (void)HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    (void)HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_OFF;
    osc.PLL.PLLState   = RCC_PLL_OFF;
    (void)HAL_RCC_OscConfig(&osc);
}

static void low_bat_draw_warning(uint16_t mv)
{
    lcd_clear(BLACK);
    lcd_draw_rectangle(20U, 62U, 299U, 174U, RED);
    lcd_draw_rectangle(21U, 63U, 298U, 173U, RED);
    g_back_color = BLACK;
    lcd_show_string(112U, 76U, 104U, 16U, 16U, "LOW BATTERY!", RED);

    char vline[32];
    snprintf(vline, sizeof(vline), "%u.%02uV - SLEEPING (NOT OFF)",
             (unsigned int)(mv / 1000U), (unsigned int)((mv % 1000U) / 10U));
    size_t vlen = strlen(vline);
    uint16_t vx = (vlen * 6U < 272U) ? (uint16_t)(24U + (272U - vlen * 6U) / 2U) : 24U;
    lcd_show_string(vx, 108U, 272U, 12U, 12U, vline, YELLOW);

    const char *hint = "SWITCH OFF TO CHARGE";
    size_t hlen = strlen(hint);
    uint16_t hx = (hlen * 6U < 272U) ? (uint16_t)(24U + (272U - hlen * 6U) / 2U) : 24U;
    lcd_show_string(hx, 132U, 272U, 12U, 12U, hint, WHITE);

    const char *led_hint = "LED BLINK = POWER STILL ON";
    size_t llen = strlen(led_hint);
    uint16_t lx = (llen * 6U < 272U) ? (uint16_t)(24U + (272U - llen * 6U) / 2U) : 24U;
    lcd_show_string(lx, 152U, 272U, 12U, 12U, led_hint, ORANGE);
    LCD_BL(1);
}

/*
 * Critical low-battery forced sleep (< 3.20V, real battery >= 2.00V):
 *   - Displays a 15.0 s warning box clearly stating the camera is sleeping (not powered off).
 *   - Powers down LCD backlight/ST7789 (SLPIN), OV7725 (Soft Sleep + tri-state bus), FIFO,
 *     and ADC1; drops to 8 MHz HSI SWD-safe __WFI() with a 60 Hz TIM6 tick (+10 ppm).
 *   - Keeps software clock advancing accurately at 60 Hz (1 s / 60 ticks) while emitting an
 *     ultra-low-duty LED1 double-blink every 3.00 s (4.4% duty, distinct from normal breathing).
 *   - Every 3.00 s (with LED OFF), runs an ultra-low-power ~1.1 ms 8 MHz HSI ADC sample;
 *     once light-load battery stays >= 25% (BAT_RECOVER_PCT) for 2 consecutive checks (6 s),
 *     returns 1 while staying in 8 MHz HSI screen-off state so caller hands off seamlessly
 *     to normal breathing-LED sleep with zero screen flash.
 *   - Pressing any top button briefly re-shows the warning for 5.0 s (preserving soft clock),
 *     or hands off immediately if light-load battery has already recovered >= 25%.
 */
static uint8_t camera_enter_low_bat_sleep(uint16_t mv)
{
    printf("BAT: critical %umV (<%umV), entering low-bat sleep\r\n",
           (unsigned int)mv, (unsigned int)BAT_SLEEP_MV);

    /* 1. Freeze FIFO & VSYNC cleanly */
    __disable_irq();
    OV7725_WEN(0);
    g_vsync_state = 2;
    __enable_irq();
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    FIFO_WE_BUF_DISABLE();
    OV7725_OE(1);

    /* 2. Show 15.0 s warning modal and power down OV7725 immediately */
    low_bat_draw_warning(mv);
    ov7725_set_sleep(1U);
    delay_ms(15000);

    /* 3. Enter ultra-low-power 8 MHz HSI 60 Hz sleep loop with clock keeping & recovery poll */
    for (;;)
    {
        /* Flush elapsed SysTick ms into soft clock BEFORE suspending SysTick */
        (void)c3m1_clock_get_epoch();
        lcd_sleep_enter();

        ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_TSVREFE);
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;

        HAL_TIM_Base_Stop_IT(&htim6);
        sysclk_enter_hsi_8mhz();
        HAL_SuspendTick();
        CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);

        /* TIM6 @ 8 MHz APB1: (3+1) * (33332+1) = 133,332 cycles -> 60.0006 Hz (+10 ppm) */
        TIM6->CR1  = 0U;
        TIM6->PSC  = 3U;
        TIM6->ARR  = 33332U;
        TIM6->EGR  = TIM_EGR_UG;
        TIM6->SR   = 0U;
        TIM6->DIER = TIM_DIER_UIE;
        TIM6->CR1  = TIM_CR1_CEN;

        uint16_t tick        = 0U; /* 0..179 = 3.00 s cycle @ 60 Hz */
        uint8_t  subsec      = 0U; /* 0..59  = 1.00 s soft clock accumulator */
        uint8_t  recover_cnt = 0U; /* Consecutive 3 s checks >= BAT_RECOVER_PCT */
        uint8_t  seen_rel    = 0U;
        uint8_t  rel_cnt     = 0U;
        uint8_t  press_cnt   = 0U;
        LED1(1);

        for (;;)
        {
            g_tim6_sleep_tick = 0U;
            __DSB();
            __WFI();
            if (!g_tim6_sleep_tick)
            {
                continue;
            }

            /* Double-blink every 3.00 s @ 60 Hz: 66.7ms ON, 116.7ms OFF, 66.7ms ON, 2750ms OFF (~4.4% duty) */
            if (tick < 4U || (tick >= 11U && tick < 15U))
            {
                LED1(0); /* Active LOW = ON */
            }
            else
            {
                LED1(1); /* Open-Drain Hi-Z = OFF */
            }

            subsec++;
            if (subsec >= 60U)
            {
                subsec = 0U;
                c3m1_clock_advance_sec(1U);
            }

            tick++;
            if (tick >= 180U)
            {
                tick = 0U;
                /* Ultra-low-power ~1.1 ms 8 MHz HSI battery poll (LED OFF, screen OFF, no PLL) */
                uint16_t cur_mv = adc_battery_sample_hsi8_mv();
                if (cur_mv >= BAT_PRESENT_MIN_MV &&
                    adc_mv_to_percent_light(cur_mv) >= BAT_RECOVER_PCT)
                {
                    recover_cnt++;
                    if (recover_cnt >= 2U)
                    {
                        LED1(1);
                        TIM6->CR1  = 0U;
                        TIM6->DIER = 0U;
                        TIM6->SR   = 0U;
                        return 1U;
                    }
                }
                else
                {
                    recover_cnt = 0U;
                }
            }

            /* Poll any top button: require 50 ms (3 ticks) release first, then 50 ms press to re-show banner */
            if (key_any_top_button_pressed())
            {
                rel_cnt = 0U;
                if (seen_rel)
                {
                    press_cnt++;
                    if (press_cnt >= 3U)
                    {
                        break;
                    }
                }
            }
            else
            {
                press_cnt = 0U;
                if (!seen_rel)
                {
                    rel_cnt++;
                    if (rel_cnt >= 3U)
                    {
                        seen_rel = 1U;
                    }
                }
            }
        }

        /* Briefly wake LCD to remind user that power switch is still ON & battery is critically low */
        LED1(1);
        TIM6->CR1  = 0U;
        TIM6->DIER = 0U;
        TIM6->SR   = 0U;

        HAL_ResumeTick();
        SystemClock_Config();
        delay_init(72);
        c3m1_clock_resync_tick();
        MX_ADC1_Init();
        mv = adc_battery_get_mv();

        if (mv >= BAT_PRESENT_MIN_MV && adc_mv_to_percent_light(mv) >= BAT_RECOVER_PCT)
        {
            /* Already recovered >= 25% light-load -> drop straight back to 8 MHz HSI normal sleep */
            (void)c3m1_clock_get_epoch();
            ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_TSVREFE);
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
            sysclk_enter_hsi_8mhz();
            HAL_SuspendTick();
            CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
            return 1U;
        }

        lcd_sleep_exit();
        low_bat_draw_warning(mv);
        delay_ms(5000);
    }
}

static uint8_t sleep_non_wake_btn_pressed(void)
{
    if (HAL_GPIO_ReadPin(KEYSHOT_GPIO_Port, KEYSHOT_Pin) == GPIO_PIN_RESET) return 1U;
    if (HAL_GPIO_ReadPin(KEY4X_GPIO_Port,   KEY4X_Pin)   == GPIO_PIN_RESET) return 1U;
    if (HAL_GPIO_ReadPin(FILMKEY_GPIO_Port, FILMKEY_Pin) == GPIO_PIN_RESET) return 1U;
    return 0U;
}

/*
 * Brief battery-peek overlay during normal sleep (triggered by SHOT / 4X / FILM):
 *   - Samples ADC1 BEFORE waking LCD backlight (with OV7725 still in Soft Sleep) to report
 *     the true lightest-load battery voltage alongside its percentage.
 *   - Displays "%u.%02uV %3u%%" in the bottom-right corner (matching HUD bar style/colors)
 *     on a black screen for 3.0 s; pressing ASA during the peek wakes immediately.
 *   - Returns 1 if ASA wake was pressed during the peek, 0 to return to sleep.
 */
static uint8_t sleep_show_battery_peek(uint8_t *sd_removed_in_sleep, uint32_t *out_elapsed_ms)
{
    LED1(1);
    TIM6->CR1  = 0U;
    TIM6->DIER = 0U;
    TIM6->SR   = 0U;

    HAL_ResumeTick();
    SystemClock_Config();
    delay_init(72);

    uint32_t t0 = HAL_GetTick();

    /* Sample lightest-load voltage BEFORE waking ST7789 / backlight (OV7725 remains asleep) */
    MX_ADC1_Init();
    uint16_t mv  = adc_battery_get_mv();
    uint8_t  pct = adc_mv_to_percent_light(mv);
    ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_TSVREFE);
    RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;

    uint16_t bat_col = GREEN;
    if (mv >= BAT_PRESENT_MIN_MV && pct <= 16U)
    {
        bat_col = RED;
    }
    else if (mv >= BAT_PRESENT_MIN_MV && pct <= 25U)
    {
        bat_col = YELLOW;
    }

    char bat_str[16];
    snprintf(bat_str, sizeof(bat_str), "%u.%02uV %3u%%",
             (unsigned int)(mv / 1000U), (unsigned int)((mv % 1000U) / 10U),
             (unsigned int)pct);

    lcd_sleep_exit();
    lcd_clear(BLACK);
    g_back_color = BLACK;
    lcd_show_string(258U, (uint16_t)(OSD_BAR_Y + 4U), 62U, 12U, 12U, bat_str, bat_col);
    delay_ms(20); /* Let ST7789 scan out 1 full refresh frame before enabling backlight */
    LCD_BL(1);

    uint8_t  wake_req    = 0U;
    uint8_t  asa_low_cnt = 0U;
    uint32_t show_t0     = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - show_t0) < 3000U)
    {
        if (!sd_card_is_present())
        {
            *sd_removed_in_sleep = 1U;
        }
        if (HAL_GPIO_ReadPin(KEYISO_GPIO_Port, KEYISO_Pin) == GPIO_PIN_RESET)
        {
            asa_low_cnt++;
            if (asa_low_cnt >= 3U)
            {
                wake_req = 1U;
                break;
            }
        }
        else
        {
            asa_low_cnt = 0U;
        }
        delay_ms(10);
    }

    if (out_elapsed_ms != NULL)
    {
        *out_elapsed_ms = (uint32_t)(HAL_GetTick() - t0);
    }

    if (wake_req)
    {
        LCD_BL(0);
    }
    else
    {
        lcd_sleep_enter();
        sysclk_enter_hsi_8mhz();
        HAL_SuspendTick();
        CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
        TIM6->CR1  = 0U;
        TIM6->PSC  = 7U;
        TIM6->ARR  = 249U;
        TIM6->EGR  = TIM_EGR_UG;
        TIM6->SR   = 0U;
        TIM6->DIER = TIM_DIER_UIE;
        TIM6->CR1  = TIM_CR1_CEN;
    }
    return wake_req;
}

/*
 * Low-power auto-sleep with silky 100 Hz gamma-corrected LED1 breathing pulse:
 *   - already_asleep_hsi8 = 1: seamlessly takes over from camera_enter_low_bat_sleep() after
 *     battery recovers >= 25% (hardware already in 8 MHz HSI screen-off state -> zero flicker).
 *   - early_boot = 1: entered before OV7725/SD boot init; returns to main() on ASA wake so
 *     normal boot completes cleanly.
 *   - Short-press ASA (KEYISO / PF1) wakes immediately and swallows that press cleanly.
 */
static void camera_enter_sleep_ex(uint8_t already_asleep_hsi8, uint8_t early_boot)
{
    uint8_t sd_removed_in_sleep = 0U;

    for (;;)
    {
        if (!already_asleep_hsi8)
        {
            /* Always persist latest running clock & any pending config to SD before sleeping */
            g_cfg_save_due = 0U;
            if (!early_boot && g_sd_ready && sd_card_is_present() && !battery_is_cutoff_low())
            {
                c3m1_cfg_save(&g_cfg);
            }

            printf("SLEEP: enter\r\n");
            g_toast_until = 0U;

            /* 1. Freeze FIFO & VSYNC, power down LCD backlight/controller, sensor, ADC1, TIM6 */
            __disable_irq();
            OV7725_WEN(0);
            g_vsync_state = 2;
            __enable_irq();
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            FIFO_WE_BUF_DISABLE();
            OV7725_OE(1);

            lcd_sleep_enter();
            ov7725_set_sleep(1U);

            ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_TSVREFE);
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;

            HAL_TIM_Base_Stop_IT(&htim6);
            /* Flush awake elapsed ticks into in-RAM soft clock before suspending SysTick */
            (void)c3m1_clock_get_epoch();
            sysclk_enter_hsi_8mhz();
            HAL_SuspendTick(); /* Must follow HAL_RCC_ClockConfig so only TIM6 wakes __WFI() */
        }

        /* 2. Configure TIM6 @ 8 MHz APB1 for 250 us ticks (40 slots = 10 ms / 100 Hz PWM) */
        CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
        TIM6->CR1  = 0U;
        TIM6->PSC  = 7U;    /* 8 MHz / 8 = 1 MHz */
        TIM6->ARR  = 249U;  /* 250 us per interrupt */
        TIM6->EGR  = TIM_EGR_UG;
        TIM6->SR   = 0U;
        TIM6->DIER = TIM_DIER_UIE;
        TIM6->CR1  = TIM_CR1_CEN;

        uint8_t  slot            = 0U;
        uint16_t phase           = 0U; /* 0..299 = 3.00 s cycle @ 100 Hz */
        uint8_t  duty            = 0U; /* 0..36 out of 40 */
        uint8_t  sleep_10ms      = 0U; /* Counts 100 x 10 ms = 1.00 s to keep software clock running */
        uint8_t  asa_seen_high   = 0U;
        uint8_t  asa_high_cnt    = 0U;
        uint8_t  asa_low_cnt     = 0U;
        uint8_t  other_seen_high = 0U;
        uint8_t  other_high_cnt  = 0U;
        uint8_t  other_low_cnt   = 0U;
        uint8_t  woke_from_peek  = 0U;
        LED1(1);                       /* Open-Drain Hi-Z = OFF */

        while (1)
        {
            g_tim6_sleep_tick = 0U;
            __DSB();
            __WFI();
            if (!g_tim6_sleep_tick)
            {
                continue;
            }

            if (slot < duty)
            {
                LED1(0); /* Active LOW = ON */
            }
            else
            {
                LED1(1); /* Open-Drain Hi-Z = OFF */
            }

            slot++;
            if (slot >= 40U)
            {
                slot = 0U;

                if (!sd_card_is_present())
                {
                    sd_removed_in_sleep = 1U;
                }

                sleep_10ms++;
                if (sleep_10ms >= 100U)
                {
                    sleep_10ms = 0U;
                    c3m1_clock_advance_sec(1U);
                }

                /* Poll ASA (KEYISO / PF1, active LOW): require 30 ms release first, then 30 ms press to wake */
                if (HAL_GPIO_ReadPin(KEYISO_GPIO_Port, KEYISO_Pin) == GPIO_PIN_RESET)
                {
                    asa_high_cnt = 0U;
                    if (asa_seen_high)
                    {
                        asa_low_cnt++;
                        if (asa_low_cnt >= 3U)
                        {
                            break;
                        }
                    }
                }
                else
                {
                    asa_low_cnt = 0U;
                    if (!asa_seen_high)
                    {
                        asa_high_cnt++;
                        if (asa_high_cnt >= 3U)
                        {
                            asa_seen_high = 1U;
                        }
                    }
                }

                /* Poll non-wake buttons (SHOT / 4X / FILM): briefly peek lightest-load battery in bottom-right */
                if (sleep_non_wake_btn_pressed())
                {
                    other_high_cnt = 0U;
                    if (other_seen_high)
                    {
                        other_low_cnt++;
                        if (other_low_cnt >= 3U)
                        {
                            uint32_t peek_ms = 0UL;
                            uint8_t  wake    = sleep_show_battery_peek(&sd_removed_in_sleep, &peek_ms);
                            uint32_t add_10ms = (uint32_t)sleep_10ms + ((peek_ms + 5UL) / 10UL);
                            if (add_10ms >= 100UL)
                            {
                                c3m1_clock_advance_sec(add_10ms / 100UL);
                            }
                            sleep_10ms = (uint8_t)(add_10ms % 100UL);
                            if (wake)
                            {
                                woke_from_peek = 1U;
                                break;
                            }
                            other_seen_high = 0U;
                            other_high_cnt  = 0U;
                            other_low_cnt   = 0U;
                            asa_seen_high   = 1U;
                            asa_low_cnt     = 0U;
                        }
                    }
                }
                else
                {
                    other_low_cnt = 0U;
                    if (!other_seen_high)
                    {
                        other_high_cnt++;
                        if (other_high_cnt >= 3U)
                        {
                            other_seen_high = 1U;
                        }
                    }
                }

                /* Gamma-2.0 breathing pulse: 1.28 s inhale, 1.28 s exhale, 0.44 s gentle pause */
                phase++;
                if (phase >= 300U)
                {
                    phase = 0U;
                }
                uint32_t tri = 0UL;
                if (phase < 128U)
                {
                    tri = phase;
                }
                else if (phase < 256U)
                {
                    tri = 255U - phase;
                }
                duty = (uint8_t)(((tri * tri * 36UL) + 8064UL) / 16129UL);
            }
        }

        /* 3. Wake up: LED OFF, resume SysTick BEFORE HSE/PLL lock timeouts, restore 72 MHz */
        LED1(1);
        TIM6->CR1  = 0U;
        TIM6->DIER = 0U;
        TIM6->SR   = 0U;

        if (!woke_from_peek)
        {
            HAL_ResumeTick();
            SystemClock_Config();
            delay_init(72);
        }
        c3m1_clock_resync_tick();
        MX_TIM6_Init();
        MX_ADC1_Init();

        if (early_boot)
        {
            if (!woke_from_peek)
            {
                lcd_sleep_exit();
            }
            g_bat_mv = adc_battery_get_mv();
            if (g_bat_mv >= BAT_PRESENT_MIN_MV && g_bat_mv < BAT_SLEEP_MV)
            {
                (void)camera_enter_low_bat_sleep(g_bat_mv);
                already_asleep_hsi8 = 1U;
                continue;
            }
            OV7725_OE(0);
            FIFO_WE_BUF_ENABLE();
            g_vsync_state = 0;
            __HAL_GPIO_EXTI_CLEAR_IT(OV7725_VSYNC_Pin);
            HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            LCD_BL(1);
            printf("SLEEP: wake (boot)\r\n");
            return;
        }

        /* Guard against SD card removal/swap during sleep before writing post-sleep clock */
        if (sd_removed_in_sleep)
        {
            f_mount(NULL, "0:", 0);
            MX_SDIO_SD_DeInit();
            g_sd_ready    = 0;
            g_sd_last_pin = 0; /* Main loop hot-plug poller will cleanly remount if a new card is present */
        }
        else if (g_sd_ready && sd_card_is_present() && !battery_is_cutoff_low())
        {
            c3m1_cfg_save(&g_cfg);
        }

        /* 4. Wake OV7725 & FIFO, re-sync dial/exposure, swallow wake ASA press */
        ov7725_set_sleep(0U);
        OV7725_OE(0);
        FIFO_WE_BUF_ENABLE();
        g_vsync_state = 0;
        __HAL_GPIO_EXTI_CLEAR_IT(OV7725_VSYNC_Pin);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

        uint8_t prev_dial = g_dial_pos;
        key_init_state(); /* long_fired = pressed -> swallows current ASA press cleanly */
        g_dial_pos = key_read_shutter_dial();
        ae_seed_on_mode_transition(prev_dial, g_iso_idx);
        g_last_auto_s_label[0] = '\0';
        g_last_auto_i_label[0] = '\0';
        g_pending_ae_apply     = 0U;
        g_wind_box_drawn       = 0U;
        raw_apply_preview_exposure(1U);

        /* 5. Wake ST7789 (120 ms settle overlaps sensor warm-up; skip if already awake from peek) */
        if (!woke_from_peek)
        {
            lcd_sleep_exit();
        }
        g_bat_mv = adc_battery_get_mv();
        if (g_bat_mv >= BAT_PRESENT_MIN_MV && g_bat_mv < BAT_SLEEP_MV)
        {
            (void)camera_enter_low_bat_sleep(g_bat_mv);
            already_asleep_hsi8 = 1U;
            continue;
        }

        osd_rebuild_bar_buf(NULL, WHITE);

        vsync_reset_idle(); /* Discard any early post-sleep frame; capture 1 fully settled fresh frame */
        if (wait_vsync_state(2U, 150UL))
        {
            ov7725_camera_refresh();
        }
        else
        {
            lcd_clear(BLACK);
            osd_draw_bar(NULL, WHITE);
        }

        LCD_BL(1);
        g_pending_btn_evts = BTN_EVT_NONE;
        g_osd_dirty = 1;
        printf("SLEEP: wake\r\n");
        return;
    }
}

static void camera_enter_sleep(void)
{
    camera_enter_sleep_ex(0U, 0U);
}

/* ======================== Main Entry ======================== */
int main(void)
{
    uint32_t last_bat_tick    = 0;
    uint32_t last_sd_tick     = 0;
    uint8_t  low_bat_sleep_cnt = 0;

    HAL_Init();
    SystemClock_Config();
    delay_init(72);

    MX_GPIO_Init();
    MX_FSMC_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();
    MX_FATFS_Init();

    printf("C3M1 Firmware %s (%s) Boot\r\n", FW_VERSION_STR, FW_GIT_HASH_STR);

    c3m1_cfg_set_defaults(&g_cfg);

    lcd_init();
    lcd_dma_init();

    g_bat_mv = adc_battery_get_mv();
    if (g_bat_mv >= BAT_PRESENT_MIN_MV && g_bat_mv < BAT_SLEEP_MV)
    {
        if (camera_enter_low_bat_sleep(g_bat_mv))
        {
            camera_enter_sleep_ex(1U, 1U);
        }
        g_bat_mv = adc_battery_get_mv();
    }

    uint32_t boot_t0 = boot_anim_start();

    shutter_count_load();

    printf("BAT: %umV (%u%%) NUM:%u (%s)\r\n",
           (unsigned int)g_bat_mv, (unsigned int)adc_battery_get_percent(),
           (unsigned int)ShutterCount,
           (shutter_cnt_get_mode() != 0U) ? "WL" : "STOCK");

    uint8_t sd_ok = sd_try_mount();

    key_init_state();
    g_dial_pos = key_read_shutter_dial();

    /* Probe OV7725 once, then stay permanently in 644x480 GRBG RAW mode (zero mode switches) */
    if (ov7725_init() == 0)
    {
        printf("CMOS_Init_Successfully\r\n");
    }
    ov7725_RAW_init();
    raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, g_preview_boost_q8, 1U);
    raw_apply_preview_exposure(1U);

    boot_anim_finish(boot_t0);

    if (!sd_ok)
    {
        osd_show_toast("NO SD CARD!", RED, 1200);
        delay_ms(1200);
        g_toast_until = 0;
        lcd_clear(BLACK);
    }

    uint32_t seed_epoch = (g_sd_ready && g_cfg.saved_epoch != 0UL)
                        ? g_cfg.saved_epoch
                        : c3m1_clock_get_build_epoch();

    if (g_cfg.boot_clock)
    {
        __disable_irq();
        OV7725_WEN(0);
        g_vsync_state = 2;
        __enable_irq();

        c3m1_datetime_t dt;
        c3m1_clock_from_epoch(seed_epoch, &dt);
        c3m1_menu_set_clock_ui(&dt);
        c3m1_clock_set_datetime(&dt);
        g_clock_valid = 1U;

        if (g_sd_ready && sd_card_is_present() && !battery_is_cutoff_low())
        {
            c3m1_cfg_save(&g_cfg);
        }
        g_dial_pos = key_read_shutter_dial();
        raw_apply_preview_exposure(1U);
    }
    else
    {
        c3m1_clock_set_epoch(seed_epoch);
        if (g_sd_ready && g_cfg.saved_epoch != 0UL)
        {
            g_clock_valid = 1U;
        }
    }

    vsync_reset_idle();
    osd_draw_bar(NULL, WHITE);
    g_last_activity_tick = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 1. Poll top-board buttons FIRST so BTN_EVT_SHOT can grab an already-latched frame at 0ms lag */
        uint8_t evts = (uint8_t)(g_pending_btn_evts | key_poll_events());
        g_pending_btn_evts = BTN_EVT_NONE;
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            g_last_activity_tick = now;
        }

        if (evts & BTN_EVT_4X_LONG)
        {
            camera_open_menu();
            g_last_activity_tick = HAL_GetTick();
            continue;
        }

        if (evts & BTN_EVT_ISO_LONG)
        {
            camera_enter_sleep();
            g_last_activity_tick = HAL_GetTick();
            last_bat_tick = g_last_activity_tick;
            last_sd_tick  = g_last_activity_tick;
            continue;
        }

        if (evts & BTN_EVT_FILM_LONG)
        {
            camera_open_playback();
            g_last_activity_tick = HAL_GetTick();
            continue;
        }

        if (evts & BTN_EVT_ISO)
        {
            uint8_t prev_iso = g_iso_idx;
            g_iso_idx = (uint8_t)((g_iso_idx + 1U) % OV7725_ISO_COUNT);
            g_cfg.iso_idx = g_iso_idx;
            if (g_sd_ready)
            {
                g_cfg_save_due = now + CFG_SAVE_DEBOUNCE_MS;
                if (g_cfg_save_due == 0U) g_cfg_save_due = 1U;
            }
            ae_seed_on_mode_transition(g_dial_pos, prev_iso);
            g_last_auto_s_label[0] = '\0';
            g_last_auto_i_label[0] = '\0';
            g_pending_ae_apply = 0U;
            raw_apply_preview_exposure(1U);
            g_osd_dirty = 1;
        }

        if (evts & BTN_EVT_4X)
        {
            /* Instant 1:1 center 320x220 crop toggle: zero SCCB writes, zero dropped frames */
            g_zoom_4x = g_zoom_4x ? 0U : 1U;
            g_osd_dirty = 1;
        }

        if (evts & BTN_EVT_FILM)
        {
            g_shutter_cocked = 1;
            g_osd_dirty = 1;
        }

        if (evts & BTN_EVT_SHOT)
        {
            camera_take_photo();
            g_last_activity_tick = HAL_GetTick();
        }

        /* 2. Poll 16-position BCD shutter speed dial */
        uint8_t dial_now = key_read_shutter_dial();
        if (dial_now != g_dial_pos)
        {
            g_last_activity_tick = now;
            uint8_t prev_dial = g_dial_pos;
            g_dial_pos = dial_now;
            ae_seed_on_mode_transition(prev_dial, g_iso_idx);
            g_last_auto_s_label[0] = '\0';
            g_last_auto_i_label[0] = '\0';
            g_pending_ae_apply = 0U;
            raw_apply_preview_exposure(1U);
            g_osd_dirty = 1;
        }

        /* 3. Apply any pending AE register update atomically inside VBLANK (right after VSYNC) */
        if (g_pending_ae_apply && g_vsync_state == 1)
        {
            g_pending_ae_apply = 0U;
            raw_apply_preview_exposure(0U);
        }

        /* 4. Stream Always-RAW live viewfinder frame (real-time GRBG LUT demosaic) */
        ov7725_camera_refresh();

        /* 4b. Debounced persist of last ISO mode to 0:CAYE_CFG/c3m1.cfg */
        if (g_cfg_save_due != 0U && (int32_t)(now - g_cfg_save_due) >= 0)
        {
            g_cfg_save_due = 0U;
            if (g_sd_ready && sd_card_is_present() && !battery_is_cutoff_low())
            {
                c3m1_cfg_save(&g_cfg);
            }
        }

        /* 5. Periodic SD card hot-plug detection (clean DeInit on removal, debounced remount on insert) */
        if ((now - last_sd_tick) >= SD_POLL_INTERVAL_MS)
        {
            last_sd_tick = now;
            uint8_t present = sd_card_is_present();
            if (present != g_sd_last_pin)
            {
                if (present)
                {
                    delay_ms(150); /* Allow mechanical contacts & card VCC capacitor to settle */
                    if (sd_card_is_present())
                    {
                        /* Freeze FIFO recording cleanly while SDIO mounts & loads config */
                        __disable_irq();
                        OV7725_WEN(0);
                        g_vsync_state = 2;
                        __enable_irq();

                        uint8_t mounted = sd_try_mount();
                        vsync_reset_idle();

                        if (mounted)
                        {
                            raw_apply_preview_exposure(1U);
                            osd_show_toast("SD CARD READY", GREEN, 800);
                        }
                        else
                        {
                            osd_show_toast("SD MOUNT FAIL!", RED, TOAST_DURATION_MS);
                        }
                    }
                }
                else
                {
                    f_mount(NULL, "0:", 0);
                    MX_SDIO_SD_DeInit();
                    g_sd_ready = 0;
                    g_sd_last_pin = 0;
                    g_sd_free_kb = 0UL;
                    g_cfg_save_due = 0;
                    osd_show_toast("SD REMOVED", ORANGE, 800);
                    printf("SD card removed\r\n");
                }
                g_osd_dirty = 1;
                g_last_activity_tick = HAL_GetTick();
            }
        }

        /* 6. Periodic VREFINT-calibrated battery voltage sampling (20mV hysteresis) */
        if ((now - last_bat_tick) >= BAT_POLL_INTERVAL_MS)
        {
            last_bat_tick = now;
            uint16_t mv = adc_battery_get_mv();
            int32_t diff = (int32_t)mv - (int32_t)g_bat_mv;
            if (diff < 0) diff = -diff;
            if (diff >= 20)
            {
                g_bat_mv = mv;
                g_osd_dirty = 1;
            }
            if (mv >= BAT_PRESENT_MIN_MV && mv < BAT_SLEEP_MV)
            {
                low_bat_sleep_cnt++;
                if (low_bat_sleep_cnt >= 2U)
                {
                    g_bat_mv = mv;
                    if (camera_enter_low_bat_sleep(mv))
                    {
                        camera_enter_sleep_ex(1U, 0U);
                    }
                    low_bat_sleep_cnt    = 0U;
                    g_last_activity_tick = HAL_GetTick();
                    last_bat_tick        = g_last_activity_tick;
                    last_sd_tick         = g_last_activity_tick;
                }
            }
            else
            {
                low_bat_sleep_cnt = 0U;
            }
        }

        /* 7. Refresh inline translucent HUD buffer whenever state changes (composited next DMA frame) */
        if (g_osd_dirty)
        {
            osd_rebuild_bar_buf(NULL, WHITE);
        }

        /* 8. Auto-sleep on sustained inactivity (breathing LED1, short-press ASA to wake) */
        uint32_t sleep_ms = c3m1_cfg_auto_sleep_ms(&g_cfg);
        if (sleep_ms > 0UL && (uint32_t)(HAL_GetTick() - g_last_activity_tick) >= sleep_ms)
        {
            camera_enter_sleep();
            g_last_activity_tick = HAL_GetTick();
            last_bat_tick = g_last_activity_tick;
            last_sd_tick  = g_last_activity_tick;
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
