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
#include "c3m1_dng.h"
#include "c3m1_jpeg.h"
#include "fatfs.h"
#include "fw_version.h"
#include "menu.h"
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
#define VIEWFINDER_HEIGHT           220U            /* Top 220 rows for live view; bottom 20 rows for OSD */
#define OSD_BAR_Y                   220U
#define OSD_BAR_H                   20U
#define BAT_POLL_INTERVAL_MS        2000U
#define BAT_LOW_WARN_MV             3450U           /* Red voltage warning */
#define BAT_MID_WARN_MV             3600U           /* Yellow heads-up warning */
#define BAT_CUTOFF_MV               3250U           /* Lock SD writes below 3.25V (if real battery >= 2.0V) */
#define BAT_PRESENT_MIN_MV          2000U           /* Ignore cutoff when bench-powered via ST-Link only */
#define SD_POLL_INTERVAL_MS         500U
#define TOAST_DURATION_MS           1000U

/* ======================== Global State ======================== */
extern uint8_t g_ov7725_frame;
volatile uint8_t  RAW_shot     = 0;
volatile uint16_t ShutterCount = 0;

FRESULT res;
FATFS   fs;
FIL     file;
UINT    bw;

static uint8_t  g_sd_ready             = 0;
static uint8_t  g_sd_last_pin          = 0;
static uint8_t  g_dial_pos             = 0;
static uint8_t  g_iso_idx              = 0;
static uint8_t  g_zoom_4x              = 0;
static uint8_t  g_shutter_cocked       = 1;
static uint16_t g_bat_mv               = 4000U;
static uint8_t  g_osd_dirty            = 1;
static uint8_t  g_wind_box_drawn       = 0;
static uint32_t g_toast_until          = 0;
static uint32_t g_cfg_save_due         = 0;

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
static uint8_t  g_ae_frame_div         = 0U;
static uint8_t  g_pending_ae_apply     = 0U;   /* Applied atomically right after VSYNC (g_vsync_state==1) */
static uint8_t  g_exposure_dirty_frames = 1U;  /* >0 right after dial/ISO/AE change */
static char     g_last_auto_s_label[10] = "";
static char     g_last_auto_i_label[8]  = "";
static uint8_t  g_vf_fps               = 0U;
static uint16_t g_vf_frame_cnt         = 0U;
static uint32_t g_vf_fps_tick          = 0U;
static uint16_t g_vf_last_paint_ms     = 0U;

/* Exact IEC 61966-2-1 sRGB OETF table (256 B Flash): linear 0..255 -> sRGB 0..255 */
static const uint8_t s_srgb_lut[256] = {
      0,  13,  22,  28,  34,  38,  42,  46,  50,  53,  56,  59,  61,  64,  66,  69,
     71,  73,  75,  77,  79,  81,  83,  85,  86,  88,  90,  92,  93,  95,  96,  98,
     99, 101, 102, 104, 105, 106, 108, 109, 110, 112, 113, 114, 115, 117, 118, 119,
    120, 121, 122, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136,
    137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 148, 149, 150, 151,
    152, 153, 154, 155, 155, 156, 157, 158, 159, 159, 160, 161, 162, 163, 163, 164,
    165, 166, 167, 167, 168, 169, 170, 170, 171, 172, 173, 173, 174, 175, 175, 176,
    177, 178, 178, 179, 180, 180, 181, 182, 182, 183, 184, 185, 185, 186, 187, 187,
    188, 189, 189, 190, 190, 191, 192, 192, 193, 194, 194, 195, 196, 196, 197, 197,
    198, 199, 199, 200, 200, 201, 202, 202, 203, 203, 204, 205, 205, 206, 206, 207,
    208, 208, 209, 209, 210, 210, 211, 212, 212, 213, 213, 214, 214, 215, 215, 216,
    216, 217, 218, 218, 219, 219, 220, 220, 221, 221, 222, 222, 223, 223, 224, 224,
    225, 226, 226, 227, 227, 228, 228, 229, 229, 230, 230, 231, 231, 232, 232, 233,
    233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238, 238, 239, 239, 240, 240,
    241, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246, 246, 246, 247, 247, 248,
    248, 249, 249, 250, 250, 251, 251, 251, 252, 252, 253, 253, 254, 254, 255, 255,
};

/* Zero-wait-state SRAM RGB565 LUTs (1.75 KB), 644 B RAW row buf, and dual 640 B DMA LCD line bufs */
static uint16_t s_lut_r[256];
static uint16_t s_lut_g[256];
static uint16_t s_lut_b[256];
static uint8_t  s_lut_g6[256]; /* 0..63 sRGB green LUT for zero-unpack focus peaking */
static uint8_t  s_raw_row_buf[644];
static uint8_t  s_line_buf[2][640];

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
static uint16_t shutter_count_load(void)
{
    uint16_t val = 0;
    stmflash_read(FLASH_SAVE_ADDR, &val, 1);
    if (val == 0xFFFFU)
    {
        val = 0;
    }
    ShutterCount = val;
    return val;
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

static uint8_t sd_try_mount(void)
{
    static const char *const fs_names[] = { "NONE", "FAT12", "FAT16", "FAT32", "exFAT" };

    g_sd_last_pin = sd_card_is_present();
    if (!g_sd_last_pin)
    {
        MX_SDIO_SD_DeInit();
        g_sd_ready = 0;
        printf("NO_SD_CARD\r\n");
        return 0;
    }
    if (MX_SDIO_SD_Init() != 0)
    {
        MX_SDIO_SD_DeInit();
        g_sd_ready = 0;
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
        if (c3m1_cfg_load(&g_cfg) != CFG_LOAD_OK && !battery_is_cutoff_low())
        {
            c3m1_cfg_save(&g_cfg);
        }
        cfg_apply_to_runtime();
        return 1;
    }
    MX_SDIO_SD_DeInit();
    g_sd_ready = 0;
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
        return 1;
    }
    g_sd_ready = 0;
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

static void osd_draw_bar(const char *status_override, uint16_t status_color)
{
    char line[48];
    const char *st_text;
    uint16_t st_col;
    uint16_t bat_col = GREEN;
    if (g_bat_mv >= BAT_PRESENT_MIN_MV && g_bat_mv < BAT_LOW_WARN_MV)
    {
        bat_col = RED;
    }
    else if (g_bat_mv >= BAT_PRESENT_MIN_MV && g_bat_mv < BAT_MID_WARN_MV)
    {
        bat_col = YELLOW;
    }

    g_back_color = BLACK;
    lcd_fill(0, OSD_BAR_Y, 319, 239, BLACK);
    lcd_draw_hline(0, OSD_BAR_Y, 320, GRAY);

    /* 1. Shutter (S: / SA:) + 2. ISO (I: / IA:) + 3. Live WB (A5500K / 5500K) */
    char shutter_str[10];
    char iso_str[8];
    ov7725_format_shutter_value(g_dial_pos, g_auto_aec, shutter_str, sizeof(shutter_str));
    ov7725_format_iso_value(g_iso_idx, g_auto_gain_q4, iso_str, sizeof(iso_str));

    snprintf(line, sizeof(line), "%s%-7s",
             ov7725_is_auto_shutter(g_dial_pos) ? "SA:" : "S:",
             shutter_str);
    lcd_show_string(2, OSD_BAR_Y + 4, 56, 12, 12, line, WHITE);

    snprintf(line, sizeof(line), "%s%-4s",
             (g_iso_idx == 0U) ? "IA:" : "I:",
             iso_str);
    lcd_show_string(60, OSD_BAR_Y + 4, 44, 12, 12, line, WHITE);

    uint16_t wb_k = wb_gains_to_kelvin(g_wb_r_q8, g_wb_b_q8);
    if (g_cfg.wb_mode == C3M1_WB_AUTO)
    {
        snprintf(line, sizeof(line), "A%uK", (unsigned int)wb_k);
        lcd_show_string(108, OSD_BAR_Y + 4, 40, 12, 12, line, WHITE);
    }
    else
    {
        static const uint16_t k_preset[C3M1_WB_MODE_COUNT] = { 5500U, 3200U, 5500U, 6500U };
        uint16_t pk = (g_cfg.wb_mode < C3M1_WB_MODE_COUNT) ? k_preset[g_cfg.wb_mode] : 5500U;
        snprintf(line, sizeof(line), "%uK", (unsigned int)pk);
        lcd_show_string(108, OSD_BAR_Y + 4, 40, 12, 12, line, YELLOW);
    }
    g_last_wb_kelvin = wb_k;

    /* 4. 4X indicator or live viewfinder FPS */
    if (g_zoom_4x)
    {
        lcd_show_string(154, OSD_BAR_Y + 4, 20, 12, 12, "4X", YELLOW);
    }
    else if (g_vf_fps > 0U)
    {
        snprintf(line, sizeof(line), "%2uF", (unsigned int)g_vf_fps);
        lcd_show_string(154, OSD_BAR_Y + 4, 20, 12, 12, line, GRAY);
    }

    /* 5. Film-style frame counter (advances to next shot number when film lever is cocked) */
    uint16_t disp_num = g_shutter_cocked ? (uint16_t)(ShutterCount + 1U) : ShutterCount;
    snprintf(line, sizeof(line), "#%05u", (unsigned int)disp_num);
    lcd_show_string(180, OSD_BAR_Y + 4, 40, 12, 12, line, CYAN);

    /* 6. State (RDY / WIND / NOSD / SAVE) */
    if (status_override != NULL)
    {
        st_text = status_override;
        st_col  = status_color;
    }
    else if (!g_sd_ready)
    {
        st_text = "NOSD";
        st_col  = RED;
    }
    else if (g_cfg.film_lever && !g_shutter_cocked)
    {
        st_text = "WIND";
        st_col  = ORANGE;
    }
    else
    {
        st_text = "RDY ";
        st_col  = GREEN;
    }
    lcd_show_string(230, OSD_BAR_Y + 4, 30, 12, 12, st_text, st_col);

    /* 7. VREFINT-calibrated battery voltage (3.85V) or Li-ion percentage (85%) */
    if (g_cfg.bat_disp == 1U)
    {
        snprintf(line, sizeof(line), " %3u%%", (unsigned int)adc_battery_get_percent());
    }
    else
    {
        snprintf(line, sizeof(line), "%u.%02uV",
                 (unsigned int)(g_bat_mv / 1000U),
                 (unsigned int)((g_bat_mv % 1000U) / 10U));
    }
    lcd_show_string(282, OSD_BAR_Y + 4, 36, 12, 12, line, bat_col);

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

    g_osd_dirty = 0;
}

static void osd_show_toast(const char *msg, uint16_t color, uint32_t duration_ms)
{
    g_wind_box_drawn = 0U;
    osd_draw_center_box(msg, color);
    g_toast_until = HAL_GetTick() + duration_ms;
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
        uint16_t v12   = s_inv_gamma_12[i];
        uint32_t lin_g = (v12 > bl_g12)  ? (uint32_t)(v12 - bl_g12)  : 0UL;
        uint32_t lin_rb = (v12 > bl_rb12) ? (uint32_t)(v12 - bl_rb12) : 0UL;

        uint8_t sr = linear12_to_srgb8((lin_rb * scale_r) >> 8);
        s_lut_r[i] = (uint16_t)(((uint16_t)(sr & 0xF8U)) << 8);

        uint8_t sg = linear12_to_srgb8((lin_g * scale_g) >> 8);
        s_lut_g[i]  = (uint16_t)(((uint16_t)(sg & 0xFCU)) << 3);
        s_lut_g6[i] = (uint8_t)(sg >> 2);

        uint8_t sb = linear12_to_srgb8((lin_rb * scale_b) >> 8);
        s_lut_b[i] = (uint16_t)(sb >> 3);
    }

    g_lut_last_r_q8     = wb_r_q8;
    g_lut_last_b_q8     = wb_b_q8;
    g_lut_last_boost_q8 = boost_q8;
    g_lut_last_gain_x   = gain_x;
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
                                      uint32_t hi_cnt)
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

    /* 2. Software closed-loop AE (runs every 2 frames so sensor exposure registers settle) */
    uint8_t auto_s = ov7725_is_auto_shutter(g_dial_pos);
    uint8_t auto_i = (g_iso_idx == 0U) ? 1U : 0U;
    if ((!auto_s && !auto_i) || ae_cnt == 0UL)
    {
        return;
    }

    g_ae_frame_div++;
    if (g_ae_frame_div < 2U)
    {
        return;
    }
    g_ae_frame_div = 0U;

    uint32_t avg_g = ae_sum_g / ae_cnt;
    /* Account for digital LUT boost on extreme slow shutters when ISO is AUTO */
    uint32_t eff_g = (avg_g * (uint32_t)g_preview_boost_q8) >> 8;
    uint32_t hi_pct = (hi_cnt * 100UL) / ae_cnt;

    /*
     * Dual-speed mirrorless AE (target 18% linear gray ~46/255, +-20% deadband [38..56]):
     *   - Large error (eff_g < 20 or eff_g > 95): fast slew [0.35x..2.8x] (~1.5 EV/step)
     *   - Moderate/near target: gentle damping [0.75x..1.35x] + hysteresis highlight guard
     */
    uint32_t ratio_q8 = 256UL;
    if (eff_g < 20UL || eff_g > 95UL)
    {
        uint32_t denom = (eff_g < 3UL) ? 3UL : eff_g;
        uint32_t raw_ratio = (46UL * 256UL) / denom;
        /* Fast 75% step for large scene jumps (converges 6+ EV in <0.4s) */
        ratio_q8 = (256UL + 3UL * raw_ratio) >> 2;
        if (ratio_q8 < 90UL)  ratio_q8 = 90UL;  /* 0.35x min (-1.5 EV/step) */
        if (ratio_q8 > 720UL) ratio_q8 = 720UL; /* 2.81x max (+1.5 EV/step) */
    }
    else if (hi_pct >= 12UL && eff_g > 28UL)
    {
        ratio_q8 = (hi_pct >= 25UL) ? 204UL : 224UL; /* Gentle highlight pull-down */
    }
    else if (hi_pct >= 8UL && eff_g >= 24UL)
    {
        return; /* Hysteresis hold: never boost back up while highlights are near clipping */
    }
    else if (eff_g >= 38UL && eff_g <= 56UL)
    {
        return; /* Inside deadband: hold steady */
    }
    else
    {
        uint32_t raw_ratio = (46UL * 256UL) / eff_g;
        /* Smooth 50% fine damping near target -> silky landing, zero overshoot/flash */
        ratio_q8 = (256UL + raw_ratio) >> 1;
        if (ratio_q8 < 192UL) ratio_q8 = 192UL; /* 0.75x min */
        if (ratio_q8 > 346UL) ratio_q8 = 346UL; /* 1.35x max */
    }

    uint16_t old_aec = g_auto_aec;
    uint8_t  old_gain_reg = ov7725_gain_q4_to_reg(g_auto_gain_q4);
    uint16_t max_gain_q4 = c3m1_cfg_max_auto_gain_q4(&g_cfg);
    uint16_t max_aec     = c3m1_cfg_min_auto_aec(&g_cfg);

    if (auto_s && auto_i)
    {
        /*
         * Configurable anti-flicker P-curve @22,959 lines/s (COM4=0x81, CLKRC=0x00):
         *   Zone 1 (Bright): ISO 100, shutter 1/23000..z1_aec (1/100 @50Hz, 1/120 @60Hz)
         *   Zone 2 (Indoor): diagonal ramp z1_aec..z2_aec & ISO 100..z2_gain
         *   Zone 3 (Dim):    hold z2_aec (1/50 @50Hz, 1/60 @60Hz), ramp ISO -> max_gain_q4
         *   Zone 4 (Dark):   if MIN AUTO SHT allows > z2_aec, extend shutter -> max_aec @ max_gain_q4
         */
        uint16_t z1_aec = (g_cfg.flicker_mode == 1U) ? 191U : 230U; /* 1/120 @60Hz, 1/100 @50Hz/OFF */
        uint16_t z2_aec = (g_cfg.flicker_mode == 1U) ? 383U : 459U; /* 1/60  @60Hz, 1/50  @50Hz/OFF */
        if (z1_aec > max_aec) z1_aec = max_aec;
        if (z2_aec > max_aec) z2_aec = max_aec;
        uint16_t z2_gain_q4 = (max_gain_q4 < 64U) ? max_gain_q4 : 64U;

        uint32_t ev     = (uint32_t)g_auto_aec * (uint32_t)g_auto_gain_q4;
        uint32_t new_ev = (ev * ratio_q8 + 128UL) >> 8;
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
        /* Defer SCCB write to VBLANK right after next VSYNC (g_vsync_state==1, 0ms paint cost) */
        g_pending_ae_apply = 1U;
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

#define FIFO_SKIP_1(port, pin) do { \
    (port)->BRR  = (pin); __NOP();  \
    (port)->BSRR = (pin);           \
} while (0)

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

static void __attribute__((optimize("O2"))) fifo_read_bytes(uint8_t *dst, uint16_t count)
{
    GPIO_TypeDef *const rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t      rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const data_port = GPIOG;
    uint8_t *const      end       = dst + count;
    while (dst != end)
    {
        rclk_port->BRR = rclk_pin;
        uint8_t px = (uint8_t)(data_port->IDR >> 1);
        rclk_port->BSRR = rclk_pin;
        *dst++ = px;
    }
}

/* Read only the 320 R bytes from a 640-byte even GRBG row (2x unrolled, interleaved store) */
static void __attribute__((optimize("O2"))) fifo_read_r_320(uint8_t *dst)
{
    GPIO_TypeDef *const rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t      rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const data_port = GPIOG;

    /* Byte 0 (G0): skip */
    rclk_port->BRR  = rclk_pin; __NOP();
    rclk_port->BSRR = rclk_pin;
    /* Byte 1 (R0): read */
    rclk_port->BRR  = rclk_pin;
    uint8_t prev_r  = (uint8_t)(data_port->IDR >> 1);
    rclk_port->BSRR = rclk_pin;

    /* 159 x 2 = 318 pairs + 1 tail pair = 319 pairs (+ initial pair = 320 R bytes) */
    uint8_t *const end = dst + 318U;
    while (dst != end)
    {
        rclk_port->BRR  = rclk_pin;
        *dst++          = prev_r;
        rclk_port->BSRR = rclk_pin;
        __NOP();

        rclk_port->BRR  = rclk_pin;
        prev_r          = (uint8_t)(data_port->IDR >> 1);
        rclk_port->BSRR = rclk_pin;

        rclk_port->BRR  = rclk_pin;
        *dst++          = prev_r;
        rclk_port->BSRR = rclk_pin;
        __NOP();

        rclk_port->BRR  = rclk_pin;
        prev_r          = (uint8_t)(data_port->IDR >> 1);
        rclk_port->BSRR = rclk_pin;
    }

    rclk_port->BRR  = rclk_pin;
    *dst++          = prev_r;
    rclk_port->BSRR = rclk_pin;
    __NOP();

    rclk_port->BRR  = rclk_pin;
    prev_r          = (uint8_t)(data_port->IDR >> 1);
    rclk_port->BSRR = rclk_pin;

    *dst = prev_r;
}

/* Demosaic 320 odd-row (B, G1) pairs + R-row into big-endian RGB565 SRAM buffer + G6 row (2x unrolled) */
static void __attribute__((optimize("O2"))) raw_demosaic_320_to_buf(const uint8_t *r_row,
                                                                    uint8_t *dst,
                                                                    uint8_t *g_dst)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint16_t *const lr        = s_lut_r;
    const uint16_t *const lg        = s_lut_g;
    const uint16_t *const lb        = s_lut_b;
    const uint8_t  *const lg6       = s_lut_g6;
    const uint8_t *const  r_end     = r_row + 320U;

    while (r_row != r_end)
    {
        rclk_port->BRR  = rclk_pin;
        uint32_t idr_b0 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint8_t r0 = *r_row++;

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_g0 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint8_t g0 = (uint8_t)(idr_g0 >> 1);
        *g_dst++ = lg6[g0];
        uint16_t px0 = lr[r0] | lg[g0] | lb[(idr_b0 >> 1) & 0xFFU];
        *dst++ = (uint8_t)(px0 >> 8);
        *dst++ = (uint8_t)px0;

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_b1 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint8_t r1 = *r_row++;

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_g1 = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint8_t g1 = (uint8_t)(idr_g1 >> 1);
        *g_dst++ = lg6[g1];
        uint16_t px1 = lr[r1] | lg[g1] | lb[(idr_b1 >> 1) & 0xFFU];
        *dst++ = (uint8_t)(px1 >> 8);
        *dst++ = (uint8_t)px1;
    }
}

/* Same as above, also sampling every 4th pixel for AE/AWB stats */
static void __attribute__((optimize("O2"))) raw_demosaic_320_sampled(
    const uint8_t *r_row, uint8_t *dst, uint8_t *g_dst,
    uint32_t *wb_sum_r, uint32_t *wb_sum_g, uint32_t *wb_sum_b, uint32_t *wb_cnt,
    uint32_t *ae_sum_g, uint32_t *ae_cnt, uint32_t *hi_cnt)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint16_t *const lr        = s_lut_r;
    const uint16_t *const lg        = s_lut_g;
    const uint16_t *const lb        = s_lut_b;
    const uint8_t  *const lg6       = s_lut_g6;

    uint32_t sr = 0, sg = 0, sb = 0, wc = 0, ag = 0, ac = 0, hc = 0;

    for (uint16_t x = 0; x < 320U; x++)
    {
        rclk_port->BRR  = rclk_pin;
        uint8_t b       = (uint8_t)(data_port->IDR >> 1);
        rclk_port->BSRR = rclk_pin;

        uint8_t r = *r_row++;

        rclk_port->BRR  = rclk_pin;
        uint8_t g       = (uint8_t)(data_port->IDR >> 1);
        rclk_port->BSRR = rclk_pin;

        *g_dst++ = lg6[g];
        uint16_t px = lr[r] | lg[g] | lb[b];
        *dst++ = (uint8_t)(px >> 8);
        *dst++ = (uint8_t)px;

        if ((x & 3U) == 0U)
        {
            /* Linearize sampled pixels (12-bit) and subtract per-channel BlackLevel for unbiased AE/AWB */
            uint16_t bl_g  = c3m1_black_level_g12(g_lut_last_gain_x);
            uint16_t bl_rb = c3m1_black_level_rb12(g_lut_last_gain_x);
            uint16_t r12   = s_inv_gamma_12[r];
            uint16_t g12   = s_inv_gamma_12[g];
            uint16_t b12   = s_inv_gamma_12[b];
            uint8_t rl = (uint8_t)((r12 > bl_rb) ? ((r12 - bl_rb) >> 4) : 0U);
            uint8_t gl = (uint8_t)((g12 > bl_g)  ? ((g12 - bl_g)  >> 4) : 0U);
            uint8_t bl = (uint8_t)((b12 > bl_rb) ? ((b12 - bl_rb) >> 4) : 0U);
            ag += gl;
            ac++;
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
    }

    *wb_sum_r += sr;
    *wb_sum_g += sg;
    *wb_sum_b += sb;
    *wb_cnt   += wc;
    *ae_sum_g += ag;
    *ae_cnt   += ac;
    *hi_cnt   += hc;
}

static void __attribute__((optimize("O2"))) raw_demosaic_1x1_to_buf(
    const uint8_t *even_row, const uint8_t *odd_row, uint8_t *dst, uint8_t *g_dst)
{
    const uint16_t *const lr  = s_lut_r;
    const uint16_t *const lg  = s_lut_g;
    const uint16_t *const lb  = s_lut_b;
    const uint8_t  *const lg6 = s_lut_g6;

    for (uint16_t x = 0; x < 320U; x += 2U)
    {
        uint8_t g0 = even_row[x];
        uint8_t r  = even_row[x + 1U];
        uint8_t b0 = odd_row[x];
        uint8_t g1 = odd_row[x + 1U];

        uint8_t gm0 = (uint8_t)((uint16_t)(g0 + g1) >> 1);
        *g_dst++ = lg6[gm0];
        uint16_t px0 = lr[r] | lg[gm0] | lb[b0];
        *dst++ = (uint8_t)(px0 >> 8);
        *dst++ = (uint8_t)px0;

        uint8_t g2 = even_row[x + 2U];
        uint8_t b1 = odd_row[x + 2U];
        uint8_t gm1 = (uint8_t)((uint16_t)(g2 + g1) >> 1);
        *g_dst++ = lg6[gm1];
        uint16_t px1 = lr[r] | lg[gm1] | lb[b1];
        *dst++ = (uint8_t)(px1 >> 8);
        *dst++ = (uint8_t)px1;
    }
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

static uint8_t  s_peak_g[2][320]; /* Ping-pong G6 rows: zero memcpy */
static uint16_t s_hist_bins[64];
static uint8_t  s_hist_bar_h[64];
static uint8_t  s_hist_clipped = 0U;

static void __attribute__((optimize("O2"))) vf_hist_sample_row(const uint8_t *line,
                                                               uint16_t *bins,
                                                               uint32_t *clip_cnt,
                                                               uint32_t *sample_cnt)
{
    for (uint16_t x = 0; x < 320U; x += 4U)
    {
        uint8_t  hi  = line[x * 2U];
        uint8_t  lo  = line[x * 2U + 1U];
        uint8_t  r5  = hi >> 3;
        uint8_t  g6  = (uint8_t)(((hi & 0x07U) << 3) | (lo >> 5));
        uint8_t  b5  = lo & 0x1FU;
        uint16_t r6  = (uint16_t)((r5 << 1) | (r5 >> 4)); /* 0..31 -> 0..63 */
        uint16_t b6  = (uint16_t)((b5 << 1) | (b5 >> 4)); /* 0..31 -> 0..63 */
        uint8_t  bin = (uint8_t)((r6 + (uint16_t)g6 + (uint16_t)g6 + b6) >> 2); /* Full 0..63 sRGB luma */
        bins[bin]++;
        if (r5 >= 30U || g6 >= 61U || b5 >= 30U)
        {
            (*clip_cnt)++;
        }
        (*sample_cnt)++;
    }
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
     * Zero-copy single-pass 1px Sony-style focus peaking:
     *  - cur_g/prev_g populated inline during demosaic via s_lut_g6 (no unpack, no memcpy)
     *  - 1px horizontal NMS hairline + unsigned-range vertical check
     */
    int32_t thresh_x = (int32_t)((is_4x ? 19U : 23U) + g_lut_last_gain_x);
    int32_t thresh_y = is_4x ? thresh_x : (thresh_x + 7);
    if (thresh_x > 42) thresh_x = 42;
    if (thresh_y > 48) thresh_y = 48;

    const uint8_t p_hi = (uint8_t)(peak_col >> 8);
    const uint8_t p_lo = (uint8_t)(peak_col & 0xFFU);

    const uint8_t *cg = cur_g + 1;
    uint8_t       *lp = line + 2;
    int32_t g_cur   = cg[0];
    int32_t gx_prev = 0;
    int32_t gx_cur  = g_cur - (int32_t)cur_g[0];
    if (gx_cur < 0) gx_cur = -gx_cur;

    if (!has_prev)
    {
        for (uint32_t n = 318U; n != 0U; n--)
        {
            int32_t g_next  = *++cg;
            int32_t gx_next = g_next - g_cur;
            if (gx_next < 0) gx_next = -gx_next;

            if (gx_cur >= thresh_x && gx_cur > gx_prev && gx_cur >= gx_next)
            {
                lp[0] = p_hi;
                lp[1] = p_lo;
            }
            lp += 2;
            g_cur   = g_next;
            gx_prev = gx_cur;
            gx_cur  = gx_next;
        }
        return;
    }

    const uint8_t  *pg  = prev_g + 1;
    const uint32_t  ty  = (uint32_t)thresh_y;
    const uint32_t  ty2 = ty << 1;

    for (uint32_t n = 318U; n != 0U; n--)
    {
        int32_t g_next  = *++cg;
        int32_t gx_next = g_next - g_cur;
        if (gx_next < 0) gx_next = -gx_next;

        if ((uint32_t)((g_cur - (int32_t)(*pg++)) + (int32_t)ty) >= ty2 ||
            (gx_cur >= thresh_x && gx_cur > gx_prev && gx_cur >= gx_next))
        {
            lp[0] = p_hi;
            lp[1] = p_lo;
        }
        lp += 2;
        g_cur   = g_next;
        gx_prev = gx_cur;
        gx_cur  = gx_next;
    }
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

static void __attribute__((optimize("O2"))) ov7725_camera_refresh(void)
{
    if (g_vsync_state != 2)
    {
        return;
    }

    /* Hold screen briefly while toast message is visible, keep FIFO cycling */
    if (g_toast_until != 0)
    {
        if ((int32_t)(HAL_GetTick() - g_toast_until) < 0)
        {
            g_vsync_state = 0;
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
    uint32_t ae_sum_g = 0, ae_cnt = 0, hi_cnt = 0;
    uint32_t hist_clip = 0, hist_samples = 0;
    uint8_t  hist_en   = g_cfg.hist_en;
    uint8_t  grid_en   = g_cfg.grid_en;
    uint16_t peak_col  = c3m1_cfg_peak_rgb565(&g_cfg);
    uint8_t  buf_idx   = 0U;

    if (hist_en)
    {
        memset(s_hist_bins, 0, sizeof(s_hist_bins));
    }

    fifo_prepare_read();

    if (!g_zoom_4x)
    {
        /*
         * 1X full view: 320x110 GRBG LUT demosaic (2x vertical line-doubled to 320x220)
         * overlapped with DMA1 MEM2MEM FSMC writes (~26ms total -> 15~20fps fast, 12~15fps slow).
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

        const uint16_t pair_cnt = VIEWFINDER_HEIGHT / 2U; /* 110 pairs = 220 LCD rows */

        for (uint16_t yp = 0; yp < pair_cnt; yp++)
        {
            uint16_t y0 = (uint16_t)(yp * 2U);
            uint16_t y1 = (uint16_t)(y0 + 1U);
            uint8_t *cur_line = s_line_buf[buf_idx];

            uint8_t *cur_g  = s_peak_g[yp & 1U];
            uint8_t *prev_g = s_peak_g[(yp ^ 1U) & 1U];

            /* Read 320 R bytes from even RAW row while DMA1 finishes previous Row 1 */
            fifo_read_r_320(s_raw_row_buf);
            fifo_skip_bytes(4UL);

            /* Read odd RAW row (B, G1) and build 640-byte RGB565 line + G6 row in SRAM */
            if ((yp & 3U) == 0U)
            {
                raw_demosaic_320_sampled(s_raw_row_buf, cur_line, cur_g,
                                         &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                         &ae_sum_g, &ae_cnt, &hi_cnt);
                if (hist_en)
                {
                    vf_hist_sample_row(cur_line, s_hist_bins, &hist_clip, &hist_samples);
                }
            }
            else
            {
                raw_demosaic_320_to_buf(s_raw_row_buf, cur_line, cur_g);
            }
            fifo_skip_bytes(4UL);

            /* Always-on Sony-style 1px focus peaking + optional 3x3 vertical framing lines */
            vf_apply_focus_peaking(cur_line, cur_g, prev_g, (yp > 0U) ? 1U : 0U, 0U, peak_col);
            if (grid_en)
            {
                vf_overlay_grid_v(cur_line);
            }

            if (!keep_wind_box || y1 < PROMPT_BOX_Y0 || y0 > PROMPT_BOX_Y1)
            {
                uint8_t in_hist    = (hist_en && y0 >= HIST_Y0 && y1 <= HIST_Y1) ? 1U : 0U;
                uint8_t need_hgrid = (grid_en && (y1 == 73U || y1 == 147U)) ? 1U : 0U;

                if (in_hist)
                {
                    vf_overlay_hist_row(cur_line, y0);
                }

                /* Full-width fast DMA pipeline: send Row y0, skip 2 raw rows in parallel, send Row y1 */
                lcd_dma_send(cur_line, 640U);
                if (yp + 1U < pair_cnt)
                {
                    fifo_skip_bytes(2UL * 644UL);
                }
                if (need_hgrid || in_hist)
                {
                    lcd_dma_wait();
                    if (need_hgrid) vf_overlay_grid_h(cur_line);
                    if (in_hist)    vf_overlay_hist_row(cur_line, y1);
                }
                lcd_dma_send(cur_line, 640U);
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
                    fifo_skip_bytes(2UL * 644UL);
                }
            }

            buf_idx ^= 1U;
        }
        lcd_dma_wait();
    }
    else
    {
        /* 4X focus magnify: native 1:1 center 320x220 crop (160..479, 130..349), DMA-pipelined */
        uint8_t *row_a = s_raw_row_buf;         /* 322 bytes */
        uint8_t *row_b = s_raw_row_buf + 322U;  /* 322 bytes */

        /* Skip top 130 raw rows (even -> preserves GRBG phase) */
        fifo_skip_bytes(130UL * 644UL);

        /* Pre-read raw row 130 (cols 160..481 = 322 bytes) */
        fifo_skip_bytes(160UL);
        fifo_read_bytes(row_a, 322U);
        fifo_skip_bytes(162UL);

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

            /* Read next raw row while DMA1 streams previous LCD row in background */
            fifo_skip_bytes(160UL);
            fifo_read_bytes(next_buf, 322U);
            fifo_skip_bytes(162UL);

            if ((y & 3U) == 0U)
            {
                uint16_t bl_g  = c3m1_black_level_g12(g_lut_last_gain_x);
                uint16_t bl_rb = c3m1_black_level_rb12(g_lut_last_gain_x);
                for (uint16_t x = 0; x < 320U; x += 4U)
                {
                    uint16_t g12 = (uint16_t)(((uint32_t)s_inv_gamma_12[even_row[x]] +
                                               (uint32_t)s_inv_gamma_12[odd_row[x + 1U]]) >> 1);
                    uint16_t r12 = s_inv_gamma_12[even_row[x + 1U]];
                    uint16_t b12 = s_inv_gamma_12[odd_row[x]];
                    uint8_t gl = (uint8_t)((g12 > bl_g)  ? ((g12 - bl_g)  >> 4) : 0U);
                    uint8_t rl = (uint8_t)((r12 > bl_rb) ? ((r12 - bl_rb) >> 4) : 0U);
                    uint8_t bl = (uint8_t)((b12 > bl_rb) ? ((b12 - bl_rb) >> 4) : 0U);
                    ae_sum_g += gl;
                    ae_cnt++;
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
            if (hist_en && (y & 3U) == 0U)
            {
                vf_hist_sample_row(cur_line, s_hist_bins, &hist_clip, &hist_samples);
            }
            vf_apply_focus_peaking(cur_line, cur_g, prev_g, (y > 0U) ? 1U : 0U, 1U, peak_col);
            if (hist_en && y >= HIST_Y0 && y <= HIST_Y1)
            {
                vf_overlay_hist_row(cur_line, y);
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
        }
        lcd_dma_wait();
    }

    if (hist_en)
    {
        vf_hist_finalize(s_hist_bins, hist_clip, hist_samples);
    }

    /* Run AWB/AE BEFORE releasing g_vsync_state so SCCB writes never collide with active FIFO recording */
    raw_preview_update_ae_awb(wb_sum_r, wb_sum_g, wb_sum_b, wb_cnt, ae_sum_g, ae_cnt, hi_cnt);

    g_vsync_state = 0;
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
            if (!g_zoom_4x)
            {
                g_osd_dirty = 1;
            }
        }
        printf("VF:%ufps paint:%ums aec:%u wb:%u/%u\r\n",
               (unsigned int)g_vf_fps, (unsigned int)g_vf_last_paint_ms,
               (unsigned int)g_auto_aec, (unsigned int)g_wb_r_q8, (unsigned int)g_wb_b_q8);
    }
}

/* ======================== RAW DNG Capture ======================== */
static void wait_skip_frames(uint8_t count)
{
    RAW_shot = 0;
    while (RAW_shot < count)
    {
        if (g_vsync_state == 2)
        {
            g_vsync_state = 0;
            RAW_shot++;
        }
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

static uint8_t save_dng_from_fifo(FIL *fp, uint16_t bl_g, uint16_t bl_rb, uint32_t *last_led_tick)
{
    uint8_t *const buf     = s_sd_io_buf;
    const uint16_t buf_cap = (uint16_t)sizeof(s_sd_io_buf);
    UINT written = 0;
    FRESULT fres;

    /* 1. Write 1,536-byte multi-IFD DNG 1.4 header (IFD0 160x120 preview + SubIFD 644x480 RAW + AsShotNeutral) */
    c3m1_dng_build_header(buf, s_inv_gamma_12, bl_g, bl_rb, g_wb_r_q8, g_wb_b_q8);
    fres = f_write(fp, buf, C3M1_DNG_HEADER_SIZE, &written);
    if (fres != FR_OK || written != C3M1_DNG_HEADER_SIZE)
    {
        printf("DNG hdr err:%d\r\n", fres);
        return 0U;
    }

    /* 2. Pass 1: Stream 160x120 RGB embedded preview thumbnail (57,600 B) from AL422B */
    fifo_prepare_read();
    uint8_t *const even_row = s_raw_row_buf;     /* 644 B */
    uint8_t *const odd_row  = &s_line_buf[0][0]; /* 644 B inside contiguous 1,280 B s_line_buf */
    uint16_t buf_idx = 0U;

    for (uint16_t ty = 0; ty < C3M1_DNG_THUMB_H; ty++)
    {
        fifo_read_bytes(even_row, 644U);
        fifo_read_bytes(odd_row,  644U);
        if (ty + 1U < C3M1_DNG_THUMB_H)
        {
            fifo_skip_bytes(2UL * 644UL);
        }

        for (uint16_t tx = 0; tx < C3M1_DNG_THUMB_W; tx++)
        {
            uint16_t x   = (uint16_t)(tx * 4U);
            uint16_t g12 = (uint16_t)(((uint32_t)s_inv_gamma_12[even_row[x]] +
                                       (uint32_t)s_inv_gamma_12[odd_row[x + 1U]]) >> 1);
            uint16_t r12 = s_inv_gamma_12[even_row[x + 1U]];
            uint16_t b12 = s_inv_gamma_12[odd_row[x]];

            uint8_t sr, sg, sb;
            c3m1_color_cam12_to_srgb8((r12 > bl_rb) ? (uint16_t)(r12 - bl_rb) : 0U,
                                      (g12 > bl_g)  ? (uint16_t)(g12 - bl_g)  : 0U,
                                      (b12 > bl_rb) ? (uint16_t)(b12 - bl_rb) : 0U,
                                      g_wb_r_q8, g_wb_b_q8, s_srgb_lut, &sr, &sg, &sb);
            buf[buf_idx++] = sr;
            buf[buf_idx++] = sg;
            buf[buf_idx++] = sb;
        }

        /* 8 rows * 480 B = 3,840 B <= 4,096 B (divides 120 rows evenly: 15 flushes) */
        if (buf_idx >= 3840U)
        {
            fres = f_write(fp, buf, buf_idx, &written);
            if (fres != FR_OK || written != buf_idx)
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

    /* 3. Pass 2: Reset AL422B read pointer and stream full 644x480 companded Bayer RAW (309,120 B) */
    fifo_prepare_read();
    buf_idx = 0U;
    for (uint16_t row = 0; row < C3M1_DNG_RAW_H; row++)
    {
        fifo_read_bytes(even_row, 644U);
        uint16_t src_pos = 0U;
        while (src_pos < 644U)
        {
            uint16_t space = (uint16_t)(buf_cap - buf_idx);
            uint16_t take  = (uint16_t)(644U - src_pos);
            if (take > space) take = space;
            memcpy(&buf[buf_idx], &even_row[src_pos], take);
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
        .out_buf      = s_sd_io_buf,
        .out_buf_size = (uint16_t)sizeof(s_sd_io_buf),
        .read_row_fn  = jpg_fifo_read_row,
        .write_fn     = jpg_sd_write,
        .user         = &io,
    };
    fifo_prepare_read();
    return c3m1_jpeg_encode_bayer_vga(&jctx, out_bytes);
}

static uint8_t ov7725_camera_output(void)
{
    uint16_t count_tmp = shutter_count_load();
    FRESULT  fres;
    char     filename[32];
    uint32_t last_led_tick;

    /*
     * CRITICAL: Latch one complete 644x480 raw frame into AL422B (FIFO_WEN=0)
     * BEFORE running f_open(), f_write(), or stmflash_write() (which stalls
     * STM32F103 CPU/IRQs for 20-40ms). FIFO stays locked across DNG preview,
     * DNG full RAW, and VGA JPEG passes via AL422B /RRST pointer reset.
     */
    while (g_vsync_state != 2)
    {
    }

    uint32_t t0 = HAL_GetTick();
    LED1_TOGGLE(); /* Begin blinking to indicate SD write phase */
    last_led_tick = t0;

    if (!sd_ensure_photo_dir())
    {
        g_vsync_state = 0;
        led1_blink_error();
        return 1;
    }

    count_tmp++;
    ShutterCount = count_tmp;

    uint8_t cap_gain_x = (g_iso_idx == 0U)
                       ? (uint8_t)((g_auto_gain_q4 + 8U) >> 4)
                       : (uint8_t)(1U << (g_iso_idx - 1U));
    if (cap_gain_x < 1U) cap_gain_x = 1U;
    uint16_t bl_g  = c3m1_black_level_g12(cap_gain_x);
    uint16_t bl_rb = c3m1_black_level_rb12(cap_gain_x);

    uint8_t want_dng      = (g_cfg.save_mode != C3M1_SAVE_JPG) ? 1U : 0U;
    uint8_t want_jpg      = (g_cfg.save_mode != C3M1_SAVE_DNG) ? 1U : 0U;
    uint8_t flash_written = 0U;
    uint8_t err           = 0U;

    if (want_dng)
    {
        snprintf(filename, sizeof(filename), "%s/CDI%05u.dng", C3M1_PHOTO_DIR, (unsigned int)count_tmp);
        fres = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
        if (fres != FR_OK)
        {
            printf("SD open err:%d (%s)\r\n", fres, filename);
            err = 1U;
        }
        else
        {
            stmflash_write(FLASH_SAVE_ADDR, &count_tmp, 1);
            flash_written = 1U;
            uint8_t ok = save_dng_from_fifo(&file, bl_g, bl_rb, &last_led_tick);
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
        snprintf(filename, sizeof(filename), "%s/CDI%05u.jpg", C3M1_PHOTO_DIR, (unsigned int)count_tmp);
        fres = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
        if (fres != FR_OK)
        {
            printf("SD open err:%d (%s)\r\n", fres, filename);
            err = 3U;
        }
        else
        {
            if (!flash_written)
            {
                stmflash_write(FLASH_SAVE_ADDR, &count_tmp, 1);
                flash_written = 1U;
            }
            uint8_t ok = save_jpg_from_fifo(&file, bl_g, bl_rb, &last_led_tick, &jpg_bytes);
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

    g_vsync_state = 0;

    if (err != 0U)
    {
        led1_blink_error();
        return err;
    }

    LED1(1); /* OFF on clean completion */
    return 0;
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

    if (g_cfg_save_due != 0U)
    {
        g_cfg_save_due = 0U;
        c3m1_cfg_save(&g_cfg);
    }

    LED1(0); /* Solid ON during exposure / frame latching */

    uint8_t need_slow_cap = ov7725_is_slow_shutter(g_dial_pos) ||
                            (ov7725_is_auto_shutter(g_dial_pos) && g_auto_aec > OV7725_PREVIEW_SLOW_AEC);
    if (need_slow_cap)
    {
        /*
         * Slow shutter (1/25..2s or AUTO slow shutter): preview was capped at 500 lines for 15F viewing.
         * Write the true DM_LN/AEC/GAIN registers atomically right after a VSYNC (NO sensor reset!),
         * integrate one exact slow-shutter frame, latch its 480 active rows inside VBLANK,
         * and restore 15F preview registers in parallel with SD DNG write.
         */
        if (!ov7725_is_auto_shutter(g_dial_pos) && g_dial_pos >= 10U)
        {
            osd_draw_bar("EXP ", YELLOW);
            osd_draw_center_box("EXPOSING...", YELLOW);
        }
        else
        {
            osd_draw_bar("SAVE", YELLOW);
        }

        /* Sync to start of a 12fps preview frame so all 7 SCCB writes complete before VSYNC_A */
        __disable_irq();
        OV7725_WEN(0);
        g_vsync_state = 0;
        __enable_irq();
        while (g_vsync_state != 1) {}

        ov7725_apply_raw_exposure(g_dial_pos, g_iso_idx, g_auto_aec,
                                  ov7725_gain_q4_to_reg(g_auto_gain_q4));

        /* Wait for VSYNC_A (new DM_LN/AEC/GAIN latch into OV7725) */
        __disable_irq();
        OV7725_WEN(0);
        g_vsync_state = 0;
        __enable_irq();
        while (g_vsync_state != 1) {}

        /* Disarm transition frame; arm AL422B to capture VSYNC_B (true long-exposure frame) */
        __disable_irq();
        OV7725_WEN(0);
        g_vsync_state = 0;
        __enable_irq();
        while (g_vsync_state != 1) {}

        /* All 480 active lines stream out in ~44ms @CLKRC=0x01; latch safely inside VBLANK */
        uint32_t t0 = HAL_GetTick();
        while (g_vsync_state != 2 && (HAL_GetTick() - t0) < 80U)
        {
        }
        __disable_irq();
        OV7725_WEN(0);
        g_vsync_state = 2;
        __enable_irq();

        /* Begin switching sensor back to 12fps preview while AL422B holds the latched frame */
        raw_apply_preview_exposure(1U);
        osd_draw_bar("SAVE", YELLOW);
    }
    else
    {
        /*
         * Fast shutters (1/8000..1/25) & AUTO: OV7725 is ALREADY running in 644x480 GRBG RAW
         * at the exact physical capture exposure -> true zero-shutter-lag (0..33ms)!
         */
        osd_draw_bar("SAVE", YELLOW);
        if (g_exposure_dirty_frames > 0U)
        {
            g_vsync_state = 0;
            wait_skip_frames(1U);
            g_exposure_dirty_frames = 0U;
        }
    }

    uint8_t err = ov7725_camera_output();
    if (err == 0)
    {
        if (g_cfg.film_lever)
        {
            g_shutter_cocked = 0;
            g_wind_box_drawn = 0;
        }
    }
    else
    {
        osd_show_toast("SD WRITE ERROR!", RED, TOAST_DURATION_MS);
    }

    g_vsync_state = 0;
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
        .fs               = &fs,
        .work_buf         = s_sd_io_buf,
        .work_buf_size    = (uint16_t)sizeof(s_sd_io_buf),
        .shutter_count    = &ShutterCount,
        .remount_sd_fn    = sd_remount_after_format,
        .is_low_bat_fn    = battery_is_cutoff_low,
        .is_sd_present_fn = sd_card_is_present,
        .sd_removed_in_menu = 0U,
    };
    c3m1_menu_run(&mctx);

    if (mctx.sd_removed_in_menu)
    {
        f_mount(NULL, "0:", 0);
        MX_SDIO_SD_DeInit();
        g_sd_ready    = 0;
        g_sd_last_pin = 0;
        osd_show_toast("SD REMOVED", ORANGE, 800);
    }

    cfg_apply_to_runtime();
    g_dial_pos = key_read_shutter_dial();
    g_pending_ae_apply = 0U;
    raw_apply_preview_exposure(1U);
    g_wind_box_drawn = 0U;
    g_vsync_state = 0;
    g_osd_dirty = 1;
}

/* ======================== Main Entry ======================== */
int main(void)
{
    uint32_t last_bat_tick = 0;
    uint32_t last_sd_tick  = 0;

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
    uint32_t boot_t0 = boot_anim_start();

    shutter_count_load();

    g_bat_mv = adc_battery_get_mv();
    printf("BAT: %umV (%u%%) NUM:%u\r\n",
           (unsigned int)g_bat_mv, (unsigned int)adc_battery_get_percent(),
           (unsigned int)ShutterCount);

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

    g_vsync_state = 0;
    osd_draw_bar(NULL, WHITE);

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 1. Poll top-board buttons FIRST so BTN_EVT_SHOT can grab an already-latched frame at 0ms lag */
        uint8_t evts = key_poll_events();

        if (evts & BTN_EVT_ISO_LONG)
        {
            camera_open_menu();
            continue;
        }

        if (evts & BTN_EVT_ISO)
        {
            g_iso_idx = (uint8_t)((g_iso_idx + 1U) % OV7725_ISO_COUNT);
            g_cfg.iso_idx = g_iso_idx;
            if (g_sd_ready)
            {
                g_cfg_save_due = now + CFG_SAVE_DEBOUNCE_MS;
                if (g_cfg_save_due == 0U) g_cfg_save_due = 1U;
            }
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
        }

        /* 2. Poll 16-position BCD shutter speed dial */
        uint8_t dial_now = key_read_shutter_dial();
        if (dial_now != g_dial_pos)
        {
            g_dial_pos = dial_now;
            g_last_auto_s_label[0] = '\0';
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
                        if (sd_try_mount())
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
                    g_cfg_save_due = 0;
                    osd_show_toast("SD REMOVED", ORANGE, 800);
                    printf("SD card removed\r\n");
                }
                g_osd_dirty = 1;
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
        }

        /* 6. Redraw bottom OSD bar only when state changes */
        if (g_osd_dirty)
        {
            osd_draw_bar(NULL, WHITE);
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
