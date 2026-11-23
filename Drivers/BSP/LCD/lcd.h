/**
 * @file        lcd.h
 * @brief       C3M1 2.0-inch IPS ST7789V3 8-bit 8080 FSMC LCD driver.
 */

#ifndef __LCD_H
#define __LCD_H

#include "stdlib.h"
#include "../../SYSTEM/sys/sys.h"

/* FSMC Bank1 NE1 (CS tied to GND on top board), A10 = LCD_DC (RS), 8-bit bus */
#define LCD_FSMC_NEX            1
#define LCD_FSMC_AX             10

/* 8-bit FSMC address mapping:
 *   HADDR[25:0] maps 1:1 to FSMC_A[25:0].
 *   LCD_REG = 0x600003FF (A10 = 0 -> Command)
 *   LCD_RAM = 0x60000400 (A10 = 1 -> Data)
 */
typedef struct
{
    volatile uint8_t LCD_REG;
    volatile uint8_t LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE        ((uint32_t)((0x60000000UL + (0x04000000UL * (LCD_FSMC_NEX - 1))) | ((1UL << LCD_FSMC_AX) - 1UL)))
#define LCD             ((LCD_TypeDef *)LCD_BASE)

/* ST7789V3 panel tuning options (ZJY200K-IF09 2.0" IPS) */
#define LCD_ST7789_INVERSION    1       /* 1: send 0x21 INVON (standard for IPS panels), 0: 0x20 INVOFF */
#define LCD_ST7789_RGB_ORDER    0x00    /* 0x00: RGB, 0x08: BGR */

/* Backlight (PF4 LCDEN, active HIGH) & Hardware Reset (PF5 LCDRST, active LOW) */
#define LCD_BL(x)       do{ (x) ? \
                            HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET); \
                        }while(0)

#define LCD_RST(x)      do{ (x) ? \
                            HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET); \
                        }while(0)

typedef struct
{
    uint16_t width;     /* LCD width */
    uint16_t height;    /* LCD height */
    uint16_t id;        /* LCD controller ID (0x7789) */
    uint8_t  dir;       /* 0: portrait (240x320), 1: landscape (320x240) */
    uint8_t  wramcmd;   /* Write GRAM command (0x2C) */
    uint8_t  setxcmd;   /* Column address set (0x2A) */
    uint8_t  setycmd;   /* Row address set (0x2B) */
} _lcd_dev;

extern _lcd_dev lcddev;
extern uint16_t g_point_color;
extern uint16_t g_back_color;

/* Scan directions */
#define L2R_U2D         0
#define L2R_D2U         1
#define R2L_U2D         2
#define R2L_D2U         3
#define U2D_L2R         4
#define U2D_R2L         5
#define D2U_L2R         6
#define D2U_R2L         7
#define DFT_SCAN_DIR    L2R_U2D

/* RGB565 colors */
#define WHITE           0xFFFF
#define BLACK           0x0000
#define RED             0xF800
#define GREEN           0x07E0
#define BLUE            0x001F
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define CYAN            0x07FF
#define GRAY            0x8430
#define DARKBLUE        0x01CF
#define LIGHTBLUE       0x7D7C
#define ORANGE          0xFD20

/* Low-level 8-bit bus primitives */
static inline void lcd_wr_regno(uint8_t regno)
{
    LCD->LCD_REG = regno;
}

static inline void lcd_wr_data(uint8_t data)
{
    LCD->LCD_RAM = data;
}

static inline void lcd_wr_pixel(uint16_t color)
{
    LCD->LCD_RAM = (uint8_t)(color >> 8);
    LCD->LCD_RAM = (uint8_t)(color & 0xFFU);
}

void lcd_write_reg(uint8_t regno, uint8_t data);
void lcd_init(void);
void lcd_display_on(void);
void lcd_display_off(void);
void lcd_scan_dir(uint8_t dir);
void lcd_display_dir(uint8_t dir);

void lcd_write_ram_prepare(void);
void lcd_set_cursor(uint16_t x, uint16_t y);
void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color);
void lcd_clear(uint16_t color);
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);
void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color);
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color);
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color);
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, const char *p, uint16_t color);

#endif /* __LCD_H */
