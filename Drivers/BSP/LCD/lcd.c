/**
 * @file        lcd.c
 * @brief       C3M1 2.0-inch IPS ST7789V3 8-bit 8080 FSMC LCD driver.
 */

#include "lcd.h"
#include "lcdfont.h"
#include "c3m1_log.h"
#include "usart.h"
#include "../../SYSTEM/delay/delay.h"

uint16_t g_point_color = WHITE;
uint16_t g_back_color  = BLACK;

_lcd_dev lcddev;

static void lcd_opt_delay(volatile uint32_t i)
{
    while (i--)
    {
        __NOP();
    }
}

static uint8_t lcd_rd_data(void)
{
    volatile uint8_t ram;
    lcd_opt_delay(4);
    ram = LCD->LCD_RAM;
    return ram;
}

void lcd_write_reg(uint8_t regno, uint8_t data)
{
    LCD->LCD_REG = regno;
    LCD->LCD_RAM = data;
}

void lcd_write_ram_prepare(void)
{
    LCD->LCD_REG = lcddev.wramcmd;
}

void lcd_display_on(void)
{
    lcd_wr_regno(0x29);
}

void lcd_display_off(void)
{
    lcd_wr_regno(0x28);
}

void lcd_sleep_enter(void)
{
    LCD_BL(0);
    lcd_wr_regno(0x28); /* DISPOFF */
    lcd_clear(BLACK);   /* Scrub stale viewfinder GRAM while backlight & display are OFF */
    lcd_wr_regno(0x10); /* SLPIN */
    delay_ms(5);
}

void lcd_sleep_exit(void)
{
    lcd_wr_regno(0x11); /* SLPOUT */
    delay_ms(120);      /* ST7789 datasheet requires >=120ms analog settle */
    lcd_wr_regno(0x29); /* DISPON; caller enables LCD_BL(1) after painting fresh frame */
}

void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    uint16_t ex = sx + width - 1;
    uint16_t ey = sy + height - 1;

    lcd_wr_regno(lcddev.setxcmd);
    lcd_wr_data((uint8_t)(sx >> 8));
    lcd_wr_data((uint8_t)(sx & 0xFFU));
    lcd_wr_data((uint8_t)(ex >> 8));
    lcd_wr_data((uint8_t)(ex & 0xFFU));

    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data((uint8_t)(sy >> 8));
    lcd_wr_data((uint8_t)(sy & 0xFFU));
    lcd_wr_data((uint8_t)(ey >> 8));
    lcd_wr_data((uint8_t)(ey & 0xFFU));
}

void lcd_set_cursor(uint16_t x, uint16_t y)
{
    lcd_set_window(x, y, 1, 1);
}

void lcd_scan_dir(uint8_t dir)
{
    uint8_t regval = LCD_ST7789_RGB_ORDER;
    uint16_t temp;

    if (lcddev.dir == 1)
    {
        switch (dir)
        {
            case L2R_U2D: dir = D2U_L2R; break;
            case L2R_D2U: dir = D2U_R2L; break;
            case R2L_U2D: dir = U2D_L2R; break;
            case R2L_D2U: dir = U2D_R2L; break;
            case U2D_L2R: dir = L2R_D2U; break;
            case U2D_R2L: dir = L2R_U2D; break;
            case D2U_L2R: dir = R2L_D2U; break;
            case D2U_R2L: dir = R2L_U2D; break;
            default: break;
        }
    }

    switch (dir)
    {
        case L2R_U2D: regval |= (0 << 7) | (0 << 6) | (0 << 5); break;
        case L2R_D2U: regval |= (1 << 7) | (0 << 6) | (0 << 5); break;
        case R2L_U2D: regval |= (0 << 7) | (1 << 6) | (0 << 5); break;
        case R2L_D2U: regval |= (1 << 7) | (1 << 6) | (0 << 5); break;
        case U2D_L2R: regval |= (0 << 7) | (0 << 6) | (1 << 5); break;
        case U2D_R2L: regval |= (0 << 7) | (1 << 6) | (1 << 5); break;
        case D2U_L2R: regval |= (1 << 7) | (0 << 6) | (1 << 5); break;
        case D2U_R2L: regval |= (1 << 7) | (1 << 6) | (1 << 5); break;
        default: break;
    }

    lcd_write_reg(0x36, regval);

    if (regval & 0x20U)
    {
        if (lcddev.width < lcddev.height)
        {
            temp = lcddev.width;
            lcddev.width = lcddev.height;
            lcddev.height = temp;
        }
    }
    else
    {
        if (lcddev.width > lcddev.height)
        {
            temp = lcddev.width;
            lcddev.width = lcddev.height;
            lcddev.height = temp;
        }
    }

    lcd_set_window(0, 0, lcddev.width, lcddev.height);
}

void lcd_display_dir(uint8_t dir)
{
    lcddev.dir = dir;
    lcddev.wramcmd = 0x2C;
    lcddev.setxcmd = 0x2A;
    lcddev.setycmd = 0x2B;

    if (dir == 0)
    {
        lcddev.width = 240;
        lcddev.height = 320;
    }
    else
    {
        lcddev.width = 320;
        lcddev.height = 240;
    }

    lcd_scan_dir(DFT_SCAN_DIR);
}

static void lcd_st7789v3_reginit(void)
{
    lcd_wr_regno(0x11);     /* Sleep Out */
    delay_ms(120);

    lcd_wr_regno(0x36);     /* MADCTL */
    lcd_wr_data(LCD_ST7789_RGB_ORDER);

    lcd_wr_regno(0x3A);     /* COLMOD: 16-bit RGB565 */
    lcd_wr_data(0x55);

    lcd_wr_regno(0xB2);     /* Porch Setting */
    lcd_wr_data(0x0C);
    lcd_wr_data(0x0C);
    lcd_wr_data(0x00);
    lcd_wr_data(0x33);
    lcd_wr_data(0x33);

    lcd_wr_regno(0xB7);     /* Gate Control */
    lcd_wr_data(0x35);

    lcd_wr_regno(0xBB);     /* VCOM Setting */
    lcd_wr_data(0x32);

    lcd_wr_regno(0xC0);     /* LCM Control */
    lcd_wr_data(0x2C);

    lcd_wr_regno(0xC2);     /* VDV and VRH Command Enable */
    lcd_wr_data(0x01);

    lcd_wr_regno(0xC3);     /* VRH Set */
    lcd_wr_data(0x15);

    lcd_wr_regno(0xC4);     /* VDV Set */
    lcd_wr_data(0x20);

    lcd_wr_regno(0xC6);     /* Frame Rate Control (60Hz) */
    lcd_wr_data(0x0F);

    lcd_wr_regno(0xD0);     /* Power Control 1 */
    lcd_wr_data(0xA4);
    lcd_wr_data(0xA1);

    lcd_wr_regno(0xE0);     /* Positive Voltage Gamma */
    lcd_wr_data(0xD0);
    lcd_wr_data(0x08);
    lcd_wr_data(0x0E);
    lcd_wr_data(0x09);
    lcd_wr_data(0x09);
    lcd_wr_data(0x05);
    lcd_wr_data(0x31);
    lcd_wr_data(0x33);
    lcd_wr_data(0x48);
    lcd_wr_data(0x17);
    lcd_wr_data(0x14);
    lcd_wr_data(0x15);
    lcd_wr_data(0x31);
    lcd_wr_data(0x34);

    lcd_wr_regno(0xE1);     /* Negative Voltage Gamma */
    lcd_wr_data(0xD0);
    lcd_wr_data(0x08);
    lcd_wr_data(0x0E);
    lcd_wr_data(0x09);
    lcd_wr_data(0x09);
    lcd_wr_data(0x15);
    lcd_wr_data(0x31);
    lcd_wr_data(0x33);
    lcd_wr_data(0x48);
    lcd_wr_data(0x17);
    lcd_wr_data(0x14);
    lcd_wr_data(0x15);
    lcd_wr_data(0x31);
    lcd_wr_data(0x34);

#if LCD_ST7789_INVERSION
    lcd_wr_regno(0x21);     /* Display Inversion ON (IPS panel) */
#else
    lcd_wr_regno(0x20);     /* Display Inversion OFF */
#endif

    lcd_wr_regno(0x29);     /* Display ON */
}

void lcd_init(void)
{
    uint16_t read_id;

    /* Hardware reset via PF5 (LCDRST) */
    LCD_BL(0);
    LCD_RST(1);
    delay_ms(10);
    LCD_RST(0);
    delay_ms(20);
    LCD_RST(1);
    delay_ms(120);

    /* Probe ST7789 ID (0x04 -> dummy, 0x85, 0x85, 0x52) */
    lcd_wr_regno(0x04);
    (void)lcd_rd_data();
    (void)lcd_rd_data();
    read_id = ((uint16_t)lcd_rd_data()) << 8;
    read_id |= lcd_rd_data();
    C3M1_LOG("LCD RDDID: 0x%04X\r\n", read_id);
    (void)read_id;

    lcddev.id = 0x7789;
    lcd_st7789v3_reginit();

    /* C3M1 viewfinder uses landscape 320x240 */
    lcd_display_dir(1);
    lcd_clear(BLACK);
    LCD_BL(1);
}

void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= lcddev.width || y >= lcddev.height) return;
    lcd_set_cursor(x, y);
    lcd_write_ram_prepare();
    lcd_wr_pixel(color);
}

void lcd_clear(uint16_t color)
{
    uint32_t total = (uint32_t)lcddev.width * (uint32_t)lcddev.height;
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFFU);

    lcd_set_window(0, 0, lcddev.width, lcddev.height);
    lcd_write_ram_prepare();
    while (total--)
    {
        LCD->LCD_RAM = hi;
        LCD->LCD_RAM = lo;
    }
}

void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    if (sx > ex || sy > ey || sx >= lcddev.width || sy >= lcddev.height) return;
    if (ex >= lcddev.width)  ex = lcddev.width - 1;
    if (ey >= lcddev.height) ey = lcddev.height - 1;

    uint16_t w = ex - sx + 1;
    uint16_t h = ey - sy + 1;
    uint32_t total = (uint32_t)w * (uint32_t)h;
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFFU);

    lcd_set_window(sx, sy, w, h);
    lcd_write_ram_prepare();
    while (total--)
    {
        LCD->LCD_RAM = hi;
        LCD->LCD_RAM = lo;
    }
}

void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color)
{
    if (len == 0 || x >= lcddev.width || y >= lcddev.height) return;
    lcd_fill(x, y, x + len - 1, y, color);
}

void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int xerr = 0, yerr = 0;
    int delta_x = (int)x2 - (int)x1;
    int delta_y = (int)y2 - (int)y1;
    int incx = (delta_x > 0) ? 1 : (delta_x < 0 ? -1 : 0);
    int incy = (delta_y > 0) ? 1 : (delta_y < 0 ? -1 : 0);
    if (delta_x < 0) delta_x = -delta_x;
    if (delta_y < 0) delta_y = -delta_y;
    int distance = (delta_x > delta_y) ? delta_x : delta_y;
    int row = x1, col = y1;

    for (int t = 0; t <= distance; t++)
    {
        lcd_draw_point((uint16_t)row, (uint16_t)col, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) { xerr -= distance; row += incx; }
        if (yerr > distance) { yerr -= distance; col += incy; }
    }
}

void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (x1 > x2) { uint16_t t = x1; x1 = x2; x2 = t; }
    if (y1 > y2) { uint16_t t = y1; y1 = y2; y2 = t; }
    lcd_fill(x1, y1, x2, y1, color);
    if (y2 > y1)
    {
        lcd_fill(x1, y2, x2, y2, color);
    }
    if (y2 > (uint16_t)(y1 + 1U))
    {
        lcd_fill(x1, (uint16_t)(y1 + 1U), x1, (uint16_t)(y2 - 1U), color);
        if (x2 > x1)
        {
            lcd_fill(x2, (uint16_t)(y1 + 1U), x2, (uint16_t)(y2 - 1U), color);
        }
    }
}

const uint8_t *lcd_get_font12(char chr)
{
    if (chr < ' ' || chr > '~') chr = ' ';
    return asc2_1206[(uint8_t)(chr - ' ')];
}

void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color)
{
    const uint8_t *pfont = NULL;

    if (chr < ' ' || chr > '~') return;
    if (x >= lcddev.width || y >= lcddev.height) return;
    uint8_t idx = (uint8_t)(chr - ' ');

    switch (size)
    {
        case 12: pfont = asc2_1206[idx]; break;
        case 16: pfont = asc2_1608[idx]; break;
        case 24: pfont = asc2_2412[idx]; break;
        /* case 32 (asc2_3216, 6,080 B Flash) unused by C3M1 UI; omitted for --gc-sections */
        default: return;
    }

    uint16_t cw = (uint16_t)(size >> 1);
    uint16_t ch = size;
    if ((uint16_t)(x + cw) > lcddev.width)  cw = (uint16_t)(lcddev.width - x);
    if ((uint16_t)(y + ch) > lcddev.height) ch = (uint16_t)(lcddev.height - y);
    uint8_t bytes_per_col = (uint8_t)((size + 7U) >> 3);

    if (mode == 0U)
    {
        uint8_t fg_hi = (uint8_t)(color >> 8);
        uint8_t fg_lo = (uint8_t)(color & 0xFFU);
        uint8_t bg_hi = (uint8_t)(g_back_color >> 8);
        uint8_t bg_lo = (uint8_t)(g_back_color & 0xFFU);

        lcd_set_window(x, y, cw, ch);
        lcd_write_ram_prepare();
        for (uint16_t row = 0U; row < ch; row++)
        {
            uint8_t bit_mask = (uint8_t)(0x80U >> (row & 7U));
            const uint8_t *col_ptr = pfont + (row >> 3);
            for (uint16_t col = 0U; col < cw; col++)
            {
                if ((*col_ptr) & bit_mask)
                {
                    LCD->LCD_RAM = fg_hi;
                    LCD->LCD_RAM = fg_lo;
                }
                else
                {
                    LCD->LCD_RAM = bg_hi;
                    LCD->LCD_RAM = bg_lo;
                }
                col_ptr += bytes_per_col;
            }
        }
    }
    else
    {
        for (uint16_t col = 0U; col < cw; col++)
        {
            const uint8_t *col_ptr = pfont + (uint16_t)(col * bytes_per_col);
            for (uint16_t row = 0U; row < ch; row++)
            {
                if (col_ptr[row >> 3] & (uint8_t)(0x80U >> (row & 7U)))
                {
                    lcd_draw_point((uint16_t)(x + col), (uint16_t)(y + row), color);
                }
            }
        }
    }
}

static uint32_t lcd_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size, uint16_t color)
{
    uint8_t t, temp, enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (uint8_t)((num / lcd_pow(10, len - t - 1)) % 10);
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                lcd_show_char(x + (size / 2) * t, y, ' ', size, 0, color);
                continue;
            }
            enshow = 1;
        }
        lcd_show_char(x + (size / 2) * t, y, (char)(temp + '0'), size, 0, color);
    }
}

void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, const char *p, uint16_t color)
{
    uint16_t x0 = x;
    uint16_t xmax = x + width;
    uint16_t ymax = y + height;

    while ((*p <= '~') && (*p >= ' '))
    {
        if (x + (size / 2) > xmax)
        {
            x = x0;
            y += size;
        }
        if (y + size > ymax) break;
        lcd_show_char(x, y, *p, size, 0, color);
        x += size / 2;
        p++;
    }
}
