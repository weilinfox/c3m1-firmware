/* USER CODE BEGIN Header */
/**
 ****************************************************************************************************
 * @file        main.c
 * @brief       C3M1 (C3MarkI) Digital RAW Camera Firmware
 *              MCU: STM32F103ZET6 | CMOS: OV7725 + AL422B | LCD: 2.0" ST7789V3 (8-bit FSMC)
 ****************************************************************************************************
 */
/* USER CODE END Header */

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

static uint8_t  g_sd_ready       = 0;
static uint8_t  g_sd_last_pin    = 0;
static uint8_t  g_dial_pos       = 0;
static uint8_t  g_iso_idx        = 0;
static uint8_t  g_zoom_4x        = 0;
static uint8_t  g_shutter_cocked = 1;
static uint8_t  g_bat_percent    = 100;
static uint8_t  g_osd_dirty      = 1;
static uint32_t g_toast_until    = 0;

/* 592-byte DNG 1.4 (TIFF/EP) header: 644x480 BGGR 16-bit container, ActiveArea/Crop 640x480
 * Aligned with C3M1 production calibration (30 IFD0 tags, StripOffsets = 0x250):
 *   - 0x033: BitsPerSample count = 1
 *   - 0x066: Orientation = 3 (180 deg, matching inverted CMOS mounting)
 *   - 0x0A6: Tag 0x0132 DateTime -> "2026:08:17 00:00:00" at 0x228
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
0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x16, 0x01, 0x04, 0x00, 0x01, 0x00,
0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x17, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x6F,
0x09, 0x00, 0x1A, 0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x1B, 0x01,
0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x28, 0x01, 0x03, 0x00, 0x01, 0x00,
0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x32, 0x01, 0x02, 0x00, 0x14, 0x00, 0x00, 0x00, 0x28, 0x02,
0x00, 0x00, 0x8D, 0x82, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x8E, 0x82,
0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x02, 0x01, 0x01, 0x00, 0x16, 0x92, 0x01, 0x00, 0x04, 0x00,
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
static void osd_draw_bar(const char *status_override, uint16_t status_color)
{
    char line[48];
    const char *st_text;
    uint16_t st_col;
    uint16_t bat_col = (g_bat_percent <= 15U) ? RED : GREEN;

    g_back_color = BLACK;
    lcd_fill(0, OSD_BAR_Y, 319, 239, BLACK);
    lcd_draw_hline(0, OSD_BAR_Y, 320, GRAY);

    /* Left: Shutter speed + ISO */
    snprintf(line, sizeof(line), "S:%-6s I:%-4s",
             ov7725_shutter_label(g_dial_pos),
             ov7725_iso_label(g_iso_idx));
    lcd_show_string(4, OSD_BAR_Y + 4, 130, 12, 12, line, WHITE);

    /* Center-left: 4X indicator */
    if (g_zoom_4x)
    {
        lcd_show_string(134, OSD_BAR_Y + 4, 24, 12, 12, "4X", YELLOW);
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

    g_osd_dirty = 0;
}

static void osd_show_toast(const char *msg, uint16_t color, uint32_t duration_ms)
{
    g_back_color = BLACK;
    lcd_fill(24, 92, 295, 132, BLACK);
    lcd_draw_rectangle(24, 92, 295, 132, color);
    lcd_show_string(36, 104, 250, 16, 16, msg, color);
    g_toast_until = HAL_GetTick() + duration_ms;
}

/* ======================== Viewfinder Refresh (1X & 4X) ======================== */
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

static void ov7725_camera_refresh(void)
{
    if (g_vsync_state != 2)
    {
        return;
    }

    /* Hold screen briefly while toast message is visible, keep FIFO cycling */
    if (g_toast_until != 0 && (int32_t)(HAL_GetTick() - g_toast_until) < 0)
    {
        g_vsync_state = 0;
        return;
    }
    g_toast_until = 0;

    lcd_set_window(0, 0, 320, VIEWFINDER_HEIGHT);
    lcd_write_ram_prepare();
    fifo_prepare_read();

    if (!g_zoom_4x)
    {
        /* Normal QVGA 320x240 -> stream top 220 rows directly byte-for-byte */
        uint32_t pixels = 320UL * VIEWFINDER_HEIGHT;
        while (pixels--)
        {
            OV7725_RCLK(0);
            LCD->LCD_RAM = OV7725_DATA; /* RGB565 high byte */
            OV7725_RCLK(1);
            OV7725_RCLK(0);
            LCD->LCD_RAM = OV7725_DATA; /* RGB565 low byte */
            OV7725_RCLK(1);
        }
    }
    else
    {
        /* 4X focus magnify: VGA center 160x120 -> scale 2x2 into 320x220 (110 source rows) */
        static uint8_t row_buf[160U * 2U];
        const uint16_t src_rows = VIEWFINDER_HEIGHT / 2U;

        for (uint16_t r = 0; r < src_rows; r++)
        {
            for (uint16_t k = 0; k < 320U; k++)
            {
                OV7725_RCLK(0);
                row_buf[k] = OV7725_DATA;
                OV7725_RCLK(1);
            }
            /* Emit each 160-pixel row twice vertically, doubling each pixel horizontally */
            for (uint8_t dup_y = 0; dup_y < 2U; dup_y++)
            {
                for (uint16_t c = 0; c < 320U; c += 2U)
                {
                    uint8_t hi = row_buf[c];
                    uint8_t lo = row_buf[c + 1U];
                    LCD->LCD_RAM = hi;
                    LCD->LCD_RAM = lo;
                    LCD->LCD_RAM = hi;
                    LCD->LCD_RAM = lo;
                }
            }
        }
    }

    g_vsync_state = 0;
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

static uint8_t ov7725_camera_output(void)
{
    uint16_t count_tmp = shutter_count_load();
    FRESULT  fres;
    char     filename[20];
    static uint8_t buf[4096];
    uint16_t buf_idx = 0;

    count_tmp++;
    ShutterCount = count_tmp;

    /* 8.3 short filename valid across full uint16_t range: CDI00000.dng ~ CDI65535.dng */
    snprintf(filename, sizeof(filename), "0:CDI%05u.dng", (unsigned int)count_tmp);

    fres = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        printf("SD open err:%d (%s)\r\n", fres, filename);
        return 1;
    }

    fres = f_write(&file, dng_header, sizeof(dng_header), &bw);
    if (fres != FR_OK || bw != sizeof(dng_header))
    {
        printf("SD hdr write err:%d\r\n", fres);
        f_close(&file);
        return 2;
    }

    stmflash_write(FLASH_SAVE_ADDR, &count_tmp, 1);
    printf("SDWriting: %s\r\n", filename);

    while (1)
    {
        if (g_vsync_state == 2)
        {
            LED1(0); /* Turn ON red LED while writing SD card */
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
                            g_vsync_state = 0;
                            f_close(&file);
                            LED1(1);
                            return 3;
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
                    g_vsync_state = 0;
                    f_close(&file);
                    LED1(1);
                    return 3;
                }
            }

            g_vsync_state = 0;
            break;
        }
    }

    fres = f_close(&file);
    LED1(1);
    if (fres != FR_OK)
    {
        printf("SD close err:%d\r\n", fres);
        return 4;
    }
    printf("SD_Write_DONE!\r\n");
    return 0;
}

static void camera_take_photo(void)
{
    uint16_t auto_aec = 0x0060U;
    uint8_t  auto_gain = 0x00U;

    if (!g_sd_ready)
    {
        if (!sd_try_mount())
        {
            osd_show_toast("NO SD CARD!", RED, TOAST_DURATION_MS);
            g_osd_dirty = 1;
            return;
        }
    }

#if C3M1_REQUIRE_FILM_LEVER
    if (!g_shutter_cocked)
    {
        osd_show_toast("Push Film Advance Lever", ORANGE, TOAST_DURATION_MS);
        g_osd_dirty = 1;
        return;
    }
#endif

    osd_draw_bar("SAVE", YELLOW);

    /* Snapshot live auto-exposure/gain before switching into VGA RAW */
    ov7725_snapshot_auto_exposure(&auto_aec, &auto_gain);

    /* Switch OV7725 to 644x480 RAW Bayer + apply locked shutter/ISO */
    ov7725_RAW_init();
    ov7725_apply_raw_exposure(g_dial_pos, g_iso_idx, auto_aec, auto_gain);
    g_vsync_state = 0;

    /* Drop 5 transition frames for sensor & FIFO to settle */
    wait_skip_frames(5);

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

    /* Restore live viewfinder mode and user exposure/zoom settings */
    ov7725_init();
    ov7725_set_preview_mode(g_zoom_4x);
    ov7725_apply_exposure(g_dial_pos, g_iso_idx, 0);
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

    printf("C3M1 Firmware Boot\r\n");

    lcd_init();
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

    /* Initialize OV7725 in live-view mode and apply current dial/ISO */
    if (ov7725_init() == 0)
    {
        printf("CMOS_Init_Successfully\r\n");
    }
    ov7725_set_preview_mode(g_zoom_4x);
    ov7725_apply_exposure(g_dial_pos, g_iso_idx, 0);
    g_vsync_state = 0;

    osd_draw_bar(NULL, WHITE);

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 1. Stream live viewfinder frame if ready */
        ov7725_camera_refresh();

        /* 2. Poll debounced top-board buttons, film lever, and NUMRST */
        uint8_t evts = key_poll_events();

        if (evts & BTN_EVT_ISO)
        {
            g_iso_idx = (uint8_t)((g_iso_idx + 1U) % OV7725_ISO_COUNT);
            ov7725_apply_exposure(g_dial_pos, g_iso_idx, 0);
            g_osd_dirty = 1;
        }

        if (evts & BTN_EVT_4X)
        {
            g_zoom_4x = g_zoom_4x ? 0U : 1U;
            ov7725_set_preview_mode(g_zoom_4x);
            g_vsync_state = 0;
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

        /* 3. Poll 16-position BCD shutter speed dial */
        uint8_t dial_now = key_read_shutter_dial();
        if (dial_now != g_dial_pos)
        {
            g_dial_pos = dial_now;
            ov7725_apply_exposure(g_dial_pos, g_iso_idx, 0);
            g_osd_dirty = 1;
        }

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
