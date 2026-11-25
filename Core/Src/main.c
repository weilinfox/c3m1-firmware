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
#include "fatfs.h"
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
#define C3M1_REQUIRE_FILM_LEVER     1               /* 1: must advance lever after each shot; 0: free shoot */
#define VIEWFINDER_HEIGHT           220U            /* Top 220 rows for live view; bottom 20 rows for OSD */
#define OSD_BAR_Y                   220U
#define OSD_BAR_H                   20U
#define BAT_POLL_INTERVAL_MS        2000U
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
static uint8_t  g_bat_percent          = 100;
static uint8_t  g_osd_dirty            = 1;
static uint32_t g_toast_until          = 0;

/* Always-RAW live viewfinder AE / AWB / LUT state */
static uint16_t g_auto_aec             = 240U; /* ~1/140s initial auto shutter @CLKRC=0x00 */
static uint16_t g_auto_gain_q4         = 16U;  /* 16 = 1.0x (ISO 100) .. 256 = 16.0x (ISO 1600) */
static uint16_t g_wb_r_q8              = 340U; /* ~1.33x daylight R gain (preview LUT only) */
static uint16_t g_wb_b_q8              = 370U; /* ~1.45x daylight B gain (preview LUT only) */
static uint16_t g_lut_last_r_q8        = 0U;
static uint16_t g_lut_last_b_q8        = 0U;
static uint16_t g_lut_last_boost_q8    = 0U;
static uint16_t g_preview_boost_q8     = 256U; /* 256 = 1.0x digital boost for extreme slow shutters */
static uint8_t  g_ae_frame_div         = 0U;
static uint8_t  g_exposure_dirty_frames = 1U;  /* >0 right after dial/ISO/AE change */
static char     g_last_auto_label[10]  = "";
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

/* Zero-wait-state SRAM RGB565 LUTs (1.5 KB), 644 B RAW row buf, and dual 640 B DMA LCD line bufs */
static uint16_t s_lut_r[256];
static uint16_t s_lut_g[256];
static uint16_t s_lut_b[256];
static uint8_t  s_raw_row_buf[644];
static uint8_t  s_line_buf[2][640];

/* 592-byte DNG 1.4 (TIFF/EP) header: 644x480 GRBG 16-bit container, ActiveArea/Crop 640x480
 * Aligned with C3M1 hardware-verified calibration (30 IFD0 tags, StripOffsets = 0x250):
 *   - 0x033: BitsPerSample count = 1
 *   - 0x066: Orientation = 1 (Normal upright, paired with OV7725 COM3 = 0xD0)
 *   - 0x0A6: Tag 0x0132 DateTime -> "2026:08:17 00:00:00" at 0x228
 *   - 0x0C6: Tag 0x828E CFAPattern = GRBG (01 00 02 01, verified under VFLIP=1)
 *   - 0x180: X/YResolution = 72/1
 *   - 0x1A0..0x1E7: Calibrated ColorMatrix1
 *   - 0x1F8..0x227: NoiseProfile
 */
const uint8_t dng_header[] = {
0x49, 0x49, 0x2A, 0x00, 0x08, 0x00, 0x00, 0x00, 0x1E, 0x00, 0xFE, 0x00, 0x04, 0x00, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x84, 0x02,
0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x02, 0x01,
0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0x01, 0x03, 0x00, 0x01, 0x00,
0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x23, 0x80,
0x00, 0x00, 0x11, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x02, 0x00, 0x00, 0x12, 0x01,
0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x16, 0x01, 0x04, 0x00, 0x01, 0x00,
0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x17, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x6F,
0x09, 0x00, 0x1A, 0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x1B, 0x01,
0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x28, 0x01, 0x03, 0x00, 0x01, 0x00,
0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x32, 0x01, 0x02, 0x00, 0x14, 0x00, 0x00, 0x00, 0x28, 0x02,
0x00, 0x00, 0x8D, 0x82, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x8E, 0x82,
0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x01, 0x16, 0x92, 0x01, 0x00, 0x04, 0x00,
0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x12, 0xC6, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x04,
0x00, 0x00, 0x16, 0xC6, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x17, 0xC6,
0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x19, 0xC6, 0x03, 0x00, 0x02, 0x00,
0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x1A, 0xC6, 0x03, 0x00, 0x04, 0x00, 0x00, 0x00, 0x88, 0x01,
0x00, 0x00, 0x1D, 0xC6, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x1E, 0xC6,
0x05, 0x00, 0x02, 0x00, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00, 0x1F, 0xC6, 0x03, 0x00, 0x02, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xC6, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x02,
0xE0, 0x01, 0x21, 0xC6, 0x0A, 0x00, 0x09, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x00, 0x00, 0x8D, 0xC6,
0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0xE8, 0x01, 0x00, 0x00, 0x41, 0xC7, 0x07, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0xC7, 0x0C, 0x00, 0x06, 0x00, 0x00, 0x00, 0xF8, 0x01,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x48, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xF4, 0x01, 0xF4, 0x01, 0xF4, 0x01, 0xF4, 0x01,
0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
0x46, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0xCE, 0xFE, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00,
0x92, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x46, 0xFE, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00,
0x6C, 0x05, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x34, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
0x98, 0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
0x58, 0x06, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xE0, 0x01, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E, 0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E, 0x32, 0x30, 0x32, 0x36, 0x3A, 0x30, 0x38, 0x3A,
0x31, 0x37, 0x20, 0x30, 0x30, 0x3A, 0x30, 0x30, 0x3A, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
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

static void shutter_count_reset(void)
{
    uint16_t zero = 0;
    stmflash_write(FLASH_SAVE_ADDR, &zero, 1);
    ShutterCount = 0;
    printf("ShutterCount reset to 0\r\n");
}

static uint8_t sd_try_mount(void)
{
    g_sd_last_pin = sd_card_is_present();
    if (!g_sd_last_pin)
    {
        g_sd_ready = 0;
        printf("NO_SD_CARD\r\n");
        return 0;
    }
    if (MX_SDIO_SD_Init() != 0)
    {
        g_sd_ready = 0;
        printf("SDIO init failed\r\n");
        return 0;
    }
    res = f_mount(&fs, "0:", 1);
    if (res == FR_OK)
    {
        g_sd_ready = 1;
        printf("SD_Card_Mounted_Successfully\r\n");
        return 1;
    }
    g_sd_ready = 0;
    printf("SD_ERR:%d\r\n", res);
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
    uint16_t bat_col = (g_bat_percent <= 15U) ? RED : GREEN;

    g_back_color = BLACK;
    lcd_fill(0, OSD_BAR_Y, 319, 239, BLACK);
    lcd_draw_hline(0, OSD_BAR_Y, 320, GRAY);

    /* Left: Shutter speed (live auto readout in AUTO mode) + ISO */
    char shutter_str[10];
    ov7725_format_shutter_label(g_dial_pos, g_auto_aec, shutter_str, sizeof(shutter_str));
    snprintf(line, sizeof(line), "S:%-6s I:%-4s",
             shutter_str,
             ov7725_iso_label(g_iso_idx));
    lcd_show_string(4, OSD_BAR_Y + 4, 130, 12, 12, line, WHITE);

    /* Center-left: 4X indicator or live viewfinder FPS */
    if (g_zoom_4x)
    {
        lcd_show_string(134, OSD_BAR_Y + 4, 24, 12, 12, "4X", YELLOW);
    }
    else if (g_vf_fps > 0U)
    {
        snprintf(line, sizeof(line), "%2uF", (unsigned int)g_vf_fps);
        lcd_show_string(134, OSD_BAR_Y + 4, 24, 12, 12, line, GRAY);
    }

    /* Center: Shutter counter */
    snprintf(line, sizeof(line), "#%05u", (unsigned int)ShutterCount);
    lcd_show_string(162, OSD_BAR_Y + 4, 48, 12, 12, line, CYAN);

    /* Center-right: State (RDY / WIND / NOSD / SAVE) */
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
    else if (C3M1_REQUIRE_FILM_LEVER && !g_shutter_cocked)
    {
        st_text = "WIND";
        st_col  = ORANGE;
    }
    else
    {
        st_text = "RDY ";
        st_col  = GREEN;
    }
    lcd_show_string(214, OSD_BAR_Y + 4, 36, 12, 12, st_text, st_col);

    /* Right: Battery percentage */
    snprintf(line, sizeof(line), "B:%3u%%", (unsigned int)g_bat_percent);
    lcd_show_string(272, OSD_BAR_Y + 4, 44, 12, 12, line, bat_col);

    /* Unified centered WIND prompt box whenever film advance lever needs cocking */
    if (status_override == NULL && g_toast_until == 0 && C3M1_REQUIRE_FILM_LEVER && !g_shutter_cocked)
    {
        osd_draw_center_box("WIND FILM LEVER", ORANGE);
    }

    g_osd_dirty = 0;
}

static void osd_show_toast(const char *msg, uint16_t color, uint32_t duration_ms)
{
    osd_draw_center_box(msg, color);
    g_toast_until = HAL_GetTick() + duration_ms;
}

/* ======================== Always-RAW LUT / AE / AWB ======================== */
static void raw_preview_rebuild_luts(uint16_t wb_r_q8, uint16_t wb_b_q8, uint16_t boost_q8)
{
    const uint32_t scale_r = ((uint32_t)wb_r_q8 * (uint32_t)boost_q8) >> 8;
    const uint32_t scale_g = (uint32_t)boost_q8;
    const uint32_t scale_b = ((uint32_t)wb_b_q8 * (uint32_t)boost_q8) >> 8;

    for (uint16_t i = 0; i < 256U; i++)
    {
        /* Subtract DNG BlackLevel (~2 in 8-bit = 512 in 16-bit container) */
        uint32_t lin = (i > 2U) ? (uint32_t)(i - 2U) : 0UL;

        uint32_t vr = (lin * scale_r) >> 8;
        if (vr > 255UL) vr = 255UL;
        uint8_t sr = s_srgb_lut[vr];
        s_lut_r[i] = (uint16_t)(((uint16_t)(sr & 0xF8U)) << 8);

        uint32_t vg = (lin * scale_g) >> 8;
        if (vg > 255UL) vg = 255UL;
        uint8_t sg = s_srgb_lut[vg];
        s_lut_g[i] = (uint16_t)(((uint16_t)(sg & 0xFCU)) << 3);

        uint32_t vb = (lin * scale_b) >> 8;
        if (vb > 255UL) vb = 255UL;
        uint8_t sb = s_srgb_lut[vb];
        s_lut_b[i] = (uint16_t)(sb >> 3);
    }

    g_lut_last_r_q8     = wb_r_q8;
    g_lut_last_b_q8     = wb_b_q8;
    g_lut_last_boost_q8 = boost_q8;
}

static void raw_apply_preview_exposure(uint8_t mark_dirty)
{
    uint8_t auto_gain_reg = ov7725_gain_q4_to_reg(g_auto_gain_q4);
    uint16_t boost = ov7725_apply_always_raw_preview(g_dial_pos, g_iso_idx, g_auto_aec, auto_gain_reg);
    g_preview_boost_q8 = boost;
    if (boost != g_lut_last_boost_q8)
    {
        raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, boost);
    }
    if (mark_dirty)
    {
        g_exposure_dirty_frames = 2U;
    }
    if (ov7725_is_auto_shutter(g_dial_pos))
    {
        char cur_lbl[10];
        ov7725_format_shutter_label(g_dial_pos, g_auto_aec, cur_lbl, sizeof(cur_lbl));
        if (strcmp(cur_lbl, g_last_auto_label) != 0)
        {
            snprintf(g_last_auto_label, sizeof(g_last_auto_label), "%s", cur_lbl);
            g_osd_dirty = 1;
        }
    }
}

static void raw_preview_update_ae_awb(uint32_t wb_sum_r, uint32_t wb_sum_g, uint32_t wb_sum_b,
                                      uint32_t wb_cnt, uint32_t ae_sum_g, uint32_t ae_cnt,
                                      uint32_t hi_cnt)
{
    /* 1. Gray-World AWB (affects preview LUTs only; DNG raw pixels remain untouched) */
    if (wb_cnt >= 128UL)
    {
        uint32_t avg_r = wb_sum_r / wb_cnt;
        uint32_t avg_g = wb_sum_g / wb_cnt;
        uint32_t avg_b = wb_sum_b / wb_cnt;
        if (avg_r >= 4UL && avg_g >= 4UL && avg_b >= 4UL)
        {
            uint32_t tr = (wb_sum_g * 256UL) / wb_sum_r;
            uint32_t tb = (wb_sum_g * 256UL) / wb_sum_b;
            if (tr < 180UL) tr = 180UL;
            if (tr > 560UL) tr = 560UL;
            if (tb < 180UL) tb = 180UL;
            if (tb > 560UL) tb = 560UL;

            g_wb_r_q8 = (uint16_t)((3UL * g_wb_r_q8 + tr + 2UL) >> 2);
            g_wb_b_q8 = (uint16_t)((3UL * g_wb_b_q8 + tb + 2UL) >> 2);

            int32_t dr = (int32_t)g_wb_r_q8 - (int32_t)g_lut_last_r_q8;
            int32_t db = (int32_t)g_wb_b_q8 - (int32_t)g_lut_last_b_q8;
            if (dr < 0) dr = -dr;
            if (db < 0) db = -db;
            if (dr >= 4 || db >= 4 || g_preview_boost_q8 != g_lut_last_boost_q8)
            {
                raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, g_preview_boost_q8);
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

    /* Target 18% linear gray (~46/255 -> sRGB ~118/255) with +-20% deadband */
    uint32_t ratio_q8 = 256UL;
    if (hi_pct >= 12UL && eff_g > 26UL)
    {
        ratio_q8 = 200UL; /* Highlight protection */
    }
    else if (eff_g >= 38UL && eff_g <= 56UL)
    {
        return; /* Inside deadband: hold steady */
    }
    else
    {
        uint32_t denom = (eff_g < 4UL) ? 4UL : eff_g;
        uint32_t raw_ratio = (46UL * 256UL) / denom;
        ratio_q8 = (256UL + raw_ratio) >> 1; /* 50% damped proportional step */
        if (ratio_q8 < 160UL) ratio_q8 = 160UL;
        if (ratio_q8 > 410UL) ratio_q8 = 410UL;
    }

    uint16_t old_aec = g_auto_aec;
    uint8_t  old_gain_reg = ov7725_gain_q4_to_reg(g_auto_gain_q4);

    if (auto_s && auto_i)
    {
        /* Program line (@CLKRC=0x00, 33.6kHz): ISO 100 1/8000..1/50 -> ISO 100..1600 -> 1/30s */
        uint32_t ev = (uint32_t)g_auto_aec * (uint32_t)g_auto_gain_q4;
        uint32_t new_ev = (ev * ratio_q8 + 128UL) >> 8;
        if (new_ev < 128UL) new_ev = 128UL;
        if (new_ev > (uint32_t)OV7725_PREVIEW_MAX_AEC * 256UL)
        {
            new_ev = (uint32_t)OV7725_PREVIEW_MAX_AEC * 256UL;
        }

        if (new_ev <= 672UL * 16UL)
        {
            g_auto_gain_q4 = 16U;
            g_auto_aec = (uint16_t)(new_ev / 16UL);
        }
        else if (new_ev <= 672UL * 256UL)
        {
            g_auto_aec = 672U;
            g_auto_gain_q4 = (uint16_t)(new_ev / 672UL);
        }
        else
        {
            g_auto_gain_q4 = 256U;
            g_auto_aec = (uint16_t)(new_ev / 256UL);
        }
        if (g_auto_aec < 8U) g_auto_aec = 8U;
        if (g_auto_aec > OV7725_PREVIEW_MAX_AEC) g_auto_aec = OV7725_PREVIEW_MAX_AEC;
        if (g_auto_gain_q4 < 16U) g_auto_gain_q4 = 16U;
        if (g_auto_gain_q4 > 256U) g_auto_gain_q4 = 256U;
    }
    else if (auto_s)
    {
        uint32_t new_aec = ((uint32_t)g_auto_aec * ratio_q8 + 128UL) >> 8;
        if (new_aec == g_auto_aec)
        {
            if (ratio_q8 > 256UL) new_aec++;
            else if (ratio_q8 < 256UL && new_aec > 8UL) new_aec--;
        }
        if (new_aec < 8UL) new_aec = 8UL;
        if (new_aec > OV7725_PREVIEW_MAX_AEC) new_aec = OV7725_PREVIEW_MAX_AEC;
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
        if (new_gain < 16UL) new_gain = 16UL;
        if (new_gain > 256UL) new_gain = 256UL;
        g_auto_gain_q4 = (uint16_t)new_gain;
    }

    if (g_auto_aec != old_aec || ov7725_gain_q4_to_reg(g_auto_gain_q4) != old_gain_reg)
    {
        raw_apply_preview_exposure(0U);
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
    (port)->BSRR = (pin); __NOP();  \
} while (0)

static void __attribute__((optimize("O2"))) fifo_skip_bytes(uint32_t n)
{
    GPIO_TypeDef *const port = OV7725_RCLK_GPIO_Port;
    const uint32_t      pin  = OV7725_RCLK_Pin;
    uint32_t blocks = n >> 3;
    while (blocks--)
    {
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
        FIFO_SKIP_1(port, pin); FIFO_SKIP_1(port, pin);
    }
    uint32_t rem = n & 7UL;
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

/* Read only the 320 R bytes from a 640-byte even GRBG row (interleaves store during G0 pulse) */
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

    uint8_t *const end = dst + 319U;
    while (dst != end)
    {
        rclk_port->BRR  = rclk_pin;
        *dst++          = prev_r;
        rclk_port->BSRR = rclk_pin;
        __NOP(); /* Guarantee AL422B tRH >= 20ns & tAC >= 25ns */

        rclk_port->BRR  = rclk_pin;
        prev_r          = (uint8_t)(data_port->IDR >> 1);
        rclk_port->BSRR = rclk_pin;
    }
    *dst = prev_r;
}

/* Demosaic 320 odd-row (B, G1) pairs + R-row into big-endian RGB565 SRAM buffer (no FSMC wait) */
static void __attribute__((optimize("O2"))) raw_demosaic_320_to_buf(const uint8_t *r_row, uint8_t *dst)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint16_t *const lr        = s_lut_r;
    const uint16_t *const lg        = s_lut_g;
    const uint16_t *const lb        = s_lut_b;
    const uint8_t *const  r_end     = r_row + 320U;

    while (r_row != r_end)
    {
        rclk_port->BRR  = rclk_pin;
        uint32_t idr_b  = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint8_t r = *r_row++;

        rclk_port->BRR  = rclk_pin;
        uint32_t idr_g  = data_port->IDR;
        rclk_port->BSRR = rclk_pin;

        uint16_t px = lr[r] | lg[(idr_g >> 1) & 0xFFU] | lb[(idr_b >> 1) & 0xFFU];
        *dst++ = (uint8_t)(px >> 8);
        *dst++ = (uint8_t)px;
    }
}

/* Same as above, also sampling every 4th pixel for AE/AWB stats */
static void __attribute__((optimize("O2"))) raw_demosaic_320_sampled(
    const uint8_t *r_row, uint8_t *dst,
    uint32_t *wb_sum_r, uint32_t *wb_sum_g, uint32_t *wb_sum_b, uint32_t *wb_cnt,
    uint32_t *ae_sum_g, uint32_t *ae_cnt, uint32_t *hi_cnt)
{
    GPIO_TypeDef *const   rclk_port = OV7725_RCLK_GPIO_Port;
    const uint32_t        rclk_pin  = OV7725_RCLK_Pin;
    GPIO_TypeDef *const   data_port = GPIOG;
    const uint16_t *const lr        = s_lut_r;
    const uint16_t *const lg        = s_lut_g;
    const uint16_t *const lb        = s_lut_b;

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

        uint16_t px = lr[r] | lg[g] | lb[b];
        *dst++ = (uint8_t)(px >> 8);
        *dst++ = (uint8_t)px;

        if ((x & 3U) == 0U)
        {
            ag += g;
            ac++;
            if (r >= 245U || g >= 245U || b >= 245U)
            {
                hc++;
            }
            else if (r >= 8U && g >= 8U && b >= 8U)
            {
                sr += r;
                sg += g;
                sb += b;
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
    const uint8_t *even_row, const uint8_t *odd_row, uint8_t *dst)
{
    const uint16_t *const lr = s_lut_r;
    const uint16_t *const lg = s_lut_g;
    const uint16_t *const lb = s_lut_b;

    for (uint16_t x = 0; x < 320U; x += 2U)
    {
        uint8_t g0 = even_row[x];
        uint8_t r  = even_row[x + 1U];
        uint8_t b0 = odd_row[x];
        uint8_t g1 = odd_row[x + 1U];

        uint16_t px0 = lr[r] | lg[(uint16_t)(g0 + g1) >> 1] | lb[b0];
        *dst++ = (uint8_t)(px0 >> 8);
        *dst++ = (uint8_t)px0;

        uint8_t g2 = even_row[x + 2U];
        uint8_t b1 = odd_row[x + 2U];
        uint16_t px1 = lr[r] | lg[(uint16_t)(g2 + g1) >> 1] | lb[b1];
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
        if (C3M1_REQUIRE_FILM_LEVER && !g_shutter_cocked)
        {
            osd_draw_center_box("WIND FILM LEVER", ORANGE);
        }
    }

    uint32_t t_paint_start = HAL_GetTick();
    uint8_t  keep_wind_box = (C3M1_REQUIRE_FILM_LEVER && !g_shutter_cocked) ? 1U : 0U;

    uint32_t wb_sum_r = 0, wb_sum_g = 0, wb_sum_b = 0, wb_cnt = 0;
    uint32_t ae_sum_g = 0, ae_cnt = 0, hi_cnt = 0;
    uint8_t  buf_idx = 0U;

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

            /* Read 320 R bytes from even RAW row while DMA1 finishes previous Row 1 */
            fifo_read_r_320(s_raw_row_buf);
            fifo_skip_bytes(4UL);

            /* Read odd RAW row (B, G1) and build 640-byte RGB565 line in SRAM */
            if ((yp & 1U) == 0U)
            {
                raw_demosaic_320_sampled(s_raw_row_buf, cur_line,
                                         &wb_sum_r, &wb_sum_g, &wb_sum_b, &wb_cnt,
                                         &ae_sum_g, &ae_cnt, &hi_cnt);
            }
            else
            {
                raw_demosaic_320_to_buf(s_raw_row_buf, cur_line);
            }
            fifo_skip_bytes(4UL);

            if (!keep_wind_box || y1 < PROMPT_BOX_Y0 || y0 > PROMPT_BOX_Y1)
            {
                /* Full-width fast DMA pipeline: send Row y0, skip 2 raw rows in parallel, send Row y1 */
                lcd_dma_send(cur_line, 640U);
                if (yp + 1U < pair_cnt)
                {
                    fifo_skip_bytes(2UL * 644UL);
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
                for (uint16_t x = 0; x < 320U; x += 4U)
                {
                    uint8_t g = (uint8_t)((uint16_t)(even_row[x] + odd_row[x + 1U]) >> 1);
                    uint8_t r = even_row[x + 1U];
                    uint8_t b = odd_row[x];
                    ae_sum_g += g;
                    ae_cnt++;
                    if (r >= 245U || g >= 245U || b >= 245U)
                    {
                        hi_cnt++;
                    }
                    else if (r >= 8U && g >= 8U && b >= 8U)
                    {
                        wb_sum_r += r;
                        wb_sum_g += g;
                        wb_sum_b += b;
                        wb_cnt++;
                    }
                }
            }

            raw_demosaic_1x1_to_buf(even_row, odd_row, cur_line);

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

    raw_preview_update_ae_awb(wb_sum_r, wb_sum_g, wb_sum_b, wb_cnt, ae_sum_g, ae_cnt, hi_cnt);
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

static uint8_t ov7725_camera_output(void)
{
    uint16_t count_tmp = shutter_count_load();
    FRESULT  fres;
    char     filename[20];
    static uint8_t buf[4096];
    uint16_t buf_idx = 0;
    uint32_t last_led_tick;

    /*
     * CRITICAL: Latch one complete 644x480 raw frame into AL422B (FIFO_WEN=0)
     * BEFORE running f_open(), f_write(header), or stmflash_write() (which
     * stalls STM32F103 CPU/IRQs for 20-40ms during page erase). Prevents Frame 2
     * from wrapping around the 384KB AL422B ring buffer on slow SD cards.
     * LED1 stays solid ON throughout exposure/latching, then blinks during SD write.
     */
    while (g_vsync_state != 2)
    {
    }

    LED1_TOGGLE(); /* Begin blinking to indicate SD write phase */
    last_led_tick = HAL_GetTick();

    count_tmp++;
    ShutterCount = count_tmp;

    /* 8.3 short filename valid across full uint16_t range: CDI00000.dng ~ CDI65535.dng */
    snprintf(filename, sizeof(filename), "0:CDI%05u.dng", (unsigned int)count_tmp);

    fres = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        printf("SD open err:%d (%s)\r\n", fres, filename);
        g_vsync_state = 0;
        led1_blink_error();
        return 1;
    }

    fres = f_write(&file, dng_header, sizeof(dng_header), &bw);
    if (fres != FR_OK || bw != sizeof(dng_header))
    {
        printf("SD hdr write err:%d\r\n", fres);
        f_close(&file);
        g_vsync_state = 0;
        led1_blink_error();
        return 2;
    }

    stmflash_write(FLASH_SAVE_ADDR, &count_tmp, 1);
    printf("SDWriting: %s\r\n", filename);

    fifo_prepare_read();

    for (uint16_t i = 0; i < 480U; i++)
    {
        for (uint16_t j = 0; j < 644U; j++)
        {
            OV7725_RCLK(0);
            uint8_t pixel = OV7725_DATA;
            OV7725_RCLK(1);

            /* Little-endian 16-bit container, 8-bit raw in MSB (pixel << 8) */
            buf[buf_idx++] = 0x00U;
            buf[buf_idx++] = pixel;

            if (buf_idx >= sizeof(buf))
            {
                fres = f_write(&file, buf, sizeof(buf), &bw);
                buf_idx = 0;
                if (fres != FR_OK || bw != sizeof(buf))
                {
                    printf("SD write err:%d\r\n", fres);
                    f_close(&file);
                    g_vsync_state = 0;
                    led1_blink_error();
                    return 3;
                }
                if ((HAL_GetTick() - last_led_tick) >= 80U)
                {
                    LED1_TOGGLE();
                    last_led_tick = HAL_GetTick();
                }
            }
        }
    }

    if (buf_idx > 0)
    {
        fres = f_write(&file, buf, buf_idx, &bw);
        buf_idx = 0;
        if (fres != FR_OK)
        {
            printf("SD tail err:%d\r\n", fres);
            f_close(&file);
            g_vsync_state = 0;
            led1_blink_error();
            return 3;
        }
    }

    g_vsync_state = 0;

    fres = f_close(&file);
    if (fres != FR_OK)
    {
        printf("SD close err:%d\r\n", fres);
        led1_blink_error();
        return 4;
    }
    LED1(1); /* OFF on clean completion */
    printf("SD_Write_DONE!\r\n");
    return 0;
}

static void camera_take_photo(void)
{
#if C3M1_REQUIRE_FILM_LEVER
    /* Completely ignore shutter presses while film advance lever is uncocked (no stutter) */
    if (!g_shutter_cocked)
    {
        return;
    }
#endif

    if (!g_sd_ready)
    {
        if (!sd_try_mount())
        {
            osd_show_toast("NO SD CARD!", RED, TOAST_DURATION_MS);
            g_osd_dirty = 1;
            return;
        }
    }

    LED1(0); /* Solid ON during exposure / frame latching */

    if (ov7725_is_slow_shutter(g_dial_pos))
    {
        /*
         * Slow shutter (1/8s..2s): preview was capped at ~1/12s for >=12fps smooth viewing.
         * Write the true DM_LN/AEC/GAIN registers atomically right after a VSYNC (NO sensor reset!),
         * integrate one exact slow-shutter frame, latch its 480 active rows (~30ms) inside VBLANK,
         * and restore 12fps preview registers in parallel with SD DNG write.
         */
        osd_draw_bar("EXP ", YELLOW);
        osd_draw_center_box("EXPOSING...", YELLOW);

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

        /* All 480 active lines stream out in ~30ms after VSYNC_B; latch safely inside VBLANK */
        uint32_t t0 = HAL_GetTick();
        while (g_vsync_state != 2 && (HAL_GetTick() - t0) < 65U)
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
#if C3M1_REQUIRE_FILM_LEVER
        g_shutter_cocked = 0;
#endif
    }
    else
    {
        osd_show_toast("SD WRITE ERROR!", RED, TOAST_DURATION_MS);
    }

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

    printf("C3M1 Firmware Boot (Always-RAW Viewfinder)\r\n");

    lcd_init();
    lcd_dma_init();
    delay_ms(5);

    /* Boot-time NUMRST check */
    if (HAL_GPIO_ReadPin(NUMRST_GPIO_Port, NUMRST_Pin) == GPIO_PIN_RESET)
    {
        shutter_count_reset();
        osd_show_toast("COUNTER RESET -> 0", YELLOW, 800);
        delay_ms(800);
        g_toast_until = 0;
        lcd_clear(BLACK);
    }
    else
    {
        shutter_count_load();
    }

    g_bat_percent = adc_battery_get_percent();
    printf("BAT: %umV (%u%%) NUM:%u\r\n",
           adc_battery_get_mv(), g_bat_percent, (unsigned int)ShutterCount);

    if (!sd_try_mount())
    {
        osd_show_toast("NO SD CARD!", RED, 1200);
        delay_ms(1200);
        g_toast_until = 0;
        lcd_clear(BLACK);
    }

    key_init_state();
    g_dial_pos = key_read_shutter_dial();

    /* Probe OV7725 once, then stay permanently in 644x480 GRBG RAW mode (zero mode switches) */
    if (ov7725_init() == 0)
    {
        printf("CMOS_Init_Successfully\r\n");
    }
    ov7725_RAW_init();
    raw_preview_rebuild_luts(g_wb_r_q8, g_wb_b_q8, g_preview_boost_q8);
    raw_apply_preview_exposure(1U);
    g_vsync_state = 0;

    osd_draw_bar(NULL, WHITE);

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 1. Poll top-board buttons FIRST so BTN_EVT_SHOT can grab an already-latched frame at 0ms lag */
        uint8_t evts = key_poll_events();

        if (evts & BTN_EVT_ISO)
        {
            g_iso_idx = (uint8_t)((g_iso_idx + 1U) % OV7725_ISO_COUNT);
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

        if (evts & BTN_EVT_NUMRST_LONG)
        {
            shutter_count_reset();
            osd_show_toast("COUNTER RESET -> 0", YELLOW, TOAST_DURATION_MS);
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
            g_last_auto_label[0] = '\0';
            raw_apply_preview_exposure(1U);
            g_osd_dirty = 1;
        }

        /* 3. Stream Always-RAW live viewfinder frame (real-time GRBG LUT demosaic) */
        ov7725_camera_refresh();

        /* 4. Periodic SD card hot-plug detection */
        if ((now - last_sd_tick) >= SD_POLL_INTERVAL_MS)
        {
            last_sd_tick = now;
            uint8_t present = sd_card_is_present();
            if (present != g_sd_last_pin)
            {
                if (present)
                {
                    delay_ms(50);
                    sd_try_mount();
                }
                else
                {
                    f_mount(NULL, "0:", 0);
                    g_sd_ready = 0;
                    g_sd_last_pin = 0;
                    printf("SD card removed\r\n");
                }
                g_osd_dirty = 1;
            }
        }

        /* 5. Periodic battery voltage sampling */
        if ((now - last_bat_tick) >= BAT_POLL_INTERVAL_MS)
        {
            last_bat_tick = now;
            uint8_t pct = adc_battery_get_percent();
            if (pct != g_bat_percent)
            {
                g_bat_percent = pct;
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
