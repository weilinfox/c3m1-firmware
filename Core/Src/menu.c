/**
 * @file        menu.c
 * @brief       Pure-English ASCII menu UI for C3M1 (320x240 ST7789V3).
 *              Navigation: 16-pos BCD dial (+/- wrap), SHOT=select/cycle, ASA=back/exit.
 */

#include <stdio.h>
#include <string.h>

#include "menu.h"
#include "main.h"
#include "adc.h"
#include "fw_version.h"
#include "sdio.h"
#include "../../BSP/KEY/key.h"
#include "../../BSP/LCD/lcd.h"
#include "../../SYSTEM/delay/delay.h"

/* RGB565 theme colors */
#define MENU_COL_HEADER_BG   0x0128U  /* Deep navy */
#define MENU_COL_SEL_BG      0x194AU  /* Highlight steel-blue */
#define MENU_COL_DIVIDER     0x4208U  /* Subtle dark gray */
#define MENU_COL_DIM         0x9CD3U  /* Muted light gray */

#define MENU_VISIBLE_ROWS    7U
#define MENU_ROW_Y0          28U
#define MENU_ROW_STEP        26U
#define MENU_ROW_H           24U

enum
{
    MITEM_COLOR_MODE = 0,
    MITEM_DRIVE_MODE,
    MITEM_SAVE_MODE,
    MITEM_HDR_MODE,
    MITEM_JSR_MODE,
    MITEM_WB_MODE,
    MITEM_EV_COMP,
    MITEM_METER_MODE,
    MITEM_MAX_ISO,
    MITEM_MIN_SHT,
    MITEM_LENS_PROFILE,
    MITEM_LENS_FOCAL,
    MITEM_LENS_APERTURE,
    MITEM_LENS_DIST,
    MITEM_LENS_VIG,
    MITEM_INCAM_DIST,
    MITEM_HIGH_ISO_NR,
    MITEM_ANTI_ALIAS,
    MITEM_FLICKER,
    MITEM_PEAK_COLOR,
    MITEM_HISTOGRAM,
    MITEM_GRID,
    MITEM_BAT_DISP,
    MITEM_FILM_LEVER,
    MITEM_AUTO_SLEEP,
    MITEM_BOOT_CLOCK,
    MITEM_PB_SKIP_BRK,
    MITEM_DEL_HDR_BRK,
    MITEM_FORMAT_SD,
    MITEM_CNT_FORMAT,
    MITEM_DEBUG_INFO,
    MITEM_SD_INFO,
    MITEM_EXIT,
    MITEM_COUNT
};

static const char *const s_item_labels[MITEM_COUNT] = {
    "COLOR MODE",
    "DRIVE MODE",
    "FILE FORMAT",
    "HDR MERGE",
    "JSR BURST",
    "WHITE BAL",
    "EXP COMP",
    "METER MODE",
    "MAX AUTO ISO",
    "MIN AUTO SHT",
    "LENS PROFILE",
    "LENS FOCAL",
    "LENS APERTURE",
    "LENS DIST",
    "LENS VIG",
    "INCAM DISTORT",
    "HIGH ISO NR",
    "ANTI-ALIASING",
    "ANTI-FLICKER",
    "PEAK COLOR",
    "HISTOGRAM",
    "GRID 3X3",
    "BATTERY DISP",
    "FILM LEVER",
    "AUTO SLEEP",
    "BOOT CLOCK",
    "PB SKIP BRK",
    "DEL HDR BRK",
    "FORMAT SD",
    "COUNTER FMT",
    "DEBUG INFO",
    "SYSTEM INFO",
    "EXIT MENU",
};

static const char *const s_color_mode_vals[C3M1_COLOR_MODE_COUNT] = { "COLOR", "B&W", "B&W HI", "B&W RED" };
static const char *const s_save_mode_vals[C3M1_SAVE_MODE_COUNT]   = { "DNG", "DNG+JPG", "JPG" };
static const char *const s_drive_mode_vals[C3M1_DRIVE_MODE_COUNT] = { "SINGLE", "BURST", "AEB 3F" };
static const char *const s_hdr_mode_vals[C3M1_HDR_MODE_COUNT]     = { "OFF", "12B 2F", "14B 3F", "16B 4F" };
static const char *const s_jsr_mode_vals[C3M1_JSR_MODE_COUNT]     = { "OFF", "7F", "14F", "30F" };
static const char *const s_wb_mode_vals[C3M1_WB_MODE_COUNT]       = { "AUTO", "3200K", "5500K", "6500K" };
static const char *const s_meter_mode_vals[C3M1_METER_MODE_COUNT] = { "STD", "CENTER", "ETTR" };
static const char *const s_max_iso_vals[C3M1_MAX_ISO_COUNT]       = { "400", "800", "1600", "3200" };
static const char *const s_min_sht_vals[C3M1_MIN_SHT_COUNT]       = { "1/100", "1/50", "1/25", "1/15", "1/8" };
static const char *const s_flicker_vals[C3M1_FLICKER_COUNT]       = { "50HZ", "60HZ", "OFF" };
static const char *const s_peak_col_vals[C3M1_PEAK_COLOR_COUNT]   = { "RED", "YELLOW", "WHITE", "BLUE" };
static const char *const s_on_off_vals[2]                         = { "OFF", "ON" };
static const char *const s_bat_disp_vals[C3M1_BAT_DISP_COUNT]     = { "VOLT", "PCT" };
static const char *const s_auto_sleep_vals[C3M1_AUTO_SLEEP_COUNT] = { "5M", "10M", "15M", "20M", "25M", "OFF" };
static const char *const s_cnt_fmt_vals[2]                        = { "STOCK", "WEAR-LVL" };

static int8_t dial_wrap_delta(uint8_t prev, uint8_t now)
{
    int8_t d = (int8_t)((now - prev) & 0x0FU);
    if (d > 8)
    {
        d -= 16;
    }
    return d;
}

static void menu_draw_header(const char *title, const char *hint)
{
    lcd_fill(0, 0, 319, 23, MENU_COL_HEADER_BG);
    lcd_draw_hline(0, 24, 320, CYAN);
    g_back_color = MENU_COL_HEADER_BG;
    lcd_show_string(8, 4, 130, 16, 16, title, WHITE);
    lcd_show_string(136, 6, 180, 12, 12, hint, CYAN);
}

static void menu_draw_footer(const char *status_msg, uint16_t status_col)
{
    char ver_line[44];
    lcd_fill(0, 218, 319, 239, BLACK);
    lcd_draw_hline(0, 218, 320, MENU_COL_DIVIDER);
    g_back_color = BLACK;
    snprintf(ver_line, sizeof(ver_line), "%s (%s)", FW_VERSION_STR, FW_GIT_HASH_STR);
    lcd_show_string(6, 223, 240, 12, 12, ver_line, MENU_COL_DIM);
    if (status_msg != NULL)
    {
        size_t slen = strlen(status_msg);
        uint16_t sw = (uint16_t)(slen * 6U);
        uint16_t sx = (sw < 100U) ? (uint16_t)(314U - sw) : 214U;
        lcd_show_string(sx, 223, (uint16_t)(318U - sx), 12, 12, status_msg, status_col);
    }
}

static void menu_draw_idle_footer(const c3m1_menu_ctx_t *ctx)
{
    if (ctx->is_low_bat_fn())
    {
        menu_draw_footer("LOW BAT", RED);
    }
    else if (ctx->sd_ready)
    {
        menu_draw_footer("CFG OK", GREEN);
    }
    else if (ctx->is_sd_present_fn())
    {
        menu_draw_footer("BAD SD", ORANGE);
    }
    else
    {
        menu_draw_footer("NO SD", RED);
    }
}

static uint8_t menu_check_sd_and_power(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick)
{
    if (had_sd && !ctx->is_sd_present_fn())
    {
        ctx->sd_removed_in_menu = 1U;
        return 1U;
    }
    if (c3m1_power_service_modal(last_act_tick))
    {
        ctx->woke_from_sleep = 1U;
        return 1U;
    }
    return 0U;
}

static int16_t menu_accel_delta(int8_t dd, uint32_t now, uint32_t *last_tick, int8_t *last_dir)
{
    int8_t  dir  = (dd > 0) ? 1 : -1;
    int16_t step = (int16_t)dd;
    if (*last_dir == dir && *last_tick != 0UL && (uint32_t)(now - *last_tick) < 110UL)
    {
        step = (int16_t)(step * 5);
    }
    *last_tick = now;
    *last_dir  = dir;
    return step;
}

static const char *menu_item_value(const c3m1_menu_ctx_t *ctx, uint8_t idx)
{
    const c3m1_cfg_t *c = ctx->cfg;
    switch (idx)
    {
        case MITEM_COLOR_MODE:
            return s_color_mode_vals[c->color_mode % C3M1_COLOR_MODE_COUNT];
        case MITEM_SAVE_MODE:
            return s_save_mode_vals[c->save_mode % C3M1_SAVE_MODE_COUNT];
        case MITEM_DRIVE_MODE:
        {
            uint8_t dm = (ctx->drive_mode != NULL) ? (*ctx->drive_mode % C3M1_DRIVE_MODE_COUNT) : 0U;
            return s_drive_mode_vals[dm];
        }
        case MITEM_HDR_MODE:
        {
            uint8_t hm = (ctx->hdr_mode != NULL) ? (*ctx->hdr_mode % C3M1_HDR_MODE_COUNT) : 0U;
            return s_hdr_mode_vals[hm];
        }
        case MITEM_JSR_MODE:
        {
            uint8_t jm = (ctx->jsr_mode != NULL) ? (*ctx->jsr_mode % C3M1_JSR_MODE_COUNT) : 0U;
            return s_jsr_mode_vals[jm];
        }
        case MITEM_WB_MODE:
            return s_wb_mode_vals[c->wb_mode % C3M1_WB_MODE_COUNT];
        case MITEM_EV_COMP:
            return c3m1_cfg_ev_comp_label(c->ev_comp);
        case MITEM_METER_MODE:
            return s_meter_mode_vals[c->meter_mode % C3M1_METER_MODE_COUNT];
        case MITEM_MAX_ISO:
            return s_max_iso_vals[c->max_auto_iso % C3M1_MAX_ISO_COUNT];
        case MITEM_LENS_PROFILE:
            return c3m1_cfg_lens_profile_label(c->lens_profile);
        case MITEM_MIN_SHT:
            return s_min_sht_vals[c->min_auto_sht % C3M1_MIN_SHT_COUNT];
        case MITEM_LENS_FOCAL:
        {
            static char s_focal_buf[10];
            c3m1_cfg_format_lens_focal(c->lens_focal_mm, s_focal_buf, sizeof(s_focal_buf));
            return s_focal_buf;
        }
        case MITEM_LENS_APERTURE:
        {
            static char s_fnum_buf[10];
            c3m1_cfg_format_lens_fnum(c->lens_fnum_x10, s_fnum_buf, sizeof(s_fnum_buf));
            return s_fnum_buf;
        }
        case MITEM_LENS_DIST:
        {
            static char s_dist_buf[10];
            c3m1_cfg_format_lens_dist(c->lens_distort, s_dist_buf, sizeof(s_dist_buf));
            return s_dist_buf;
        }
        case MITEM_LENS_VIG:
        {
            static char s_vig_buf[10];
            c3m1_cfg_format_lens_vig(c->lens_vignette, s_vig_buf, sizeof(s_vig_buf));
            return s_vig_buf;
        }
        case MITEM_INCAM_DIST:
            return s_on_off_vals[c->lens_dist_incam & 1U];
        case MITEM_HIGH_ISO_NR:
            return s_on_off_vals[c->high_iso_nr & 1U];
        case MITEM_ANTI_ALIAS:
            return s_on_off_vals[c->anti_alias & 1U];
        case MITEM_FLICKER:
            return s_flicker_vals[c->flicker_mode % C3M1_FLICKER_COUNT];
        case MITEM_PEAK_COLOR:
            return s_peak_col_vals[c->peak_color % C3M1_PEAK_COLOR_COUNT];
        case MITEM_HISTOGRAM:
            return s_on_off_vals[c->hist_en & 1U];
        case MITEM_GRID:
            return s_on_off_vals[c->grid_en & 1U];
        case MITEM_BAT_DISP:
            return s_bat_disp_vals[c->bat_disp % C3M1_BAT_DISP_COUNT];
        case MITEM_FILM_LEVER:
            return s_on_off_vals[c->film_lever & 1U];
        case MITEM_AUTO_SLEEP:
            return s_auto_sleep_vals[c->auto_sleep % C3M1_AUTO_SLEEP_COUNT];
        case MITEM_BOOT_CLOCK:
            return s_on_off_vals[c->boot_clock & 1U];
        case MITEM_PB_SKIP_BRK:
            return s_on_off_vals[c->pb_skip_brk & 1U];
        case MITEM_DEL_HDR_BRK:
            return s_on_off_vals[c->del_hdr_brk & 1U];
        case MITEM_FORMAT_SD:
            return "RUN >";
        case MITEM_CNT_FORMAT:
        {
            uint8_t m = (ctx->get_cnt_mode_fn != NULL) ? (ctx->get_cnt_mode_fn() & 1U) : 0U;
            return s_cnt_fmt_vals[m];
        }
        case MITEM_DEBUG_INFO:
            return s_on_off_vals[c->debug_info & 1U];
        case MITEM_SD_INFO:
            return "VIEW >";
        case MITEM_EXIT:
            return "BACK";
        default:
            return "";
    }
}

static void menu_draw_row(const c3m1_menu_ctx_t *ctx, uint8_t vis_row, uint8_t item_idx, uint8_t selected)
{
    uint16_t y  = (uint16_t)(MENU_ROW_Y0 + vis_row * MENU_ROW_STEP);
    uint16_t bg = selected ? MENU_COL_SEL_BG : BLACK;
    uint16_t fg = selected ? WHITE : MENU_COL_DIM;
    uint16_t vc = selected ? YELLOW : CYAN;
    if (item_idx == MITEM_PEAK_COLOR)
    {
        vc = c3m1_cfg_peak_rgb565(ctx->cfg);
    }
    const char *val = menu_item_value(ctx, item_idx);

    lcd_fill(4, y, 311, (uint16_t)(y + MENU_ROW_H - 1U), bg);
    if (selected)
    {
        lcd_draw_rectangle(4, y, 311, (uint16_t)(y + MENU_ROW_H - 1U), CYAN);
    }

    g_back_color = bg;
    if (selected)
    {
        lcd_show_string(10, (uint16_t)(y + 4U), 12, 16, 16, ">", YELLOW);
    }
    lcd_show_string(24, (uint16_t)(y + 4U), 180, 16, 16, s_item_labels[item_idx], fg);

    size_t vlen = strlen(val);
    uint16_t vw = (uint16_t)(vlen * 8U);
    uint16_t vx = (vw <= 160U) ? (uint16_t)(304U - vw) : 144U;
    lcd_show_string(vx, (uint16_t)(y + 4U), (uint16_t)(304U - vx), 16, 16, val, vc);
}

static void menu_draw_scrollbar(uint8_t scroll_top)
{
    const uint16_t x0 = 315U;
    const uint16_t y0 = MENU_ROW_Y0;
    const uint16_t track_h = (uint16_t)(MENU_VISIBLE_ROWS * MENU_ROW_STEP - 2U);
    lcd_fill(x0, y0, 318U, (uint16_t)(y0 + track_h), MENU_COL_DIVIDER);

    uint16_t thumb_h = (uint16_t)((track_h * MENU_VISIBLE_ROWS) / MITEM_COUNT);
    if (thumb_h < 24U) thumb_h = 24U;
    uint16_t max_scroll = (uint16_t)(MITEM_COUNT - MENU_VISIBLE_ROWS);
    uint16_t thumb_y = (max_scroll > 0U)
                     ? (uint16_t)(y0 + ((track_h - thumb_h) * scroll_top) / max_scroll)
                     : y0;
    lcd_fill(x0, thumb_y, 318U, (uint16_t)(thumb_y + thumb_h), CYAN);
}

static void menu_draw_list(const c3m1_menu_ctx_t *ctx, uint8_t sel, uint8_t scroll_top)
{
    lcd_fill(0, 25, 319, 217, BLACK);
    for (uint8_t r = 0; r < MENU_VISIBLE_ROWS; r++)
    {
        uint8_t idx = (uint8_t)(scroll_top + r);
        if (idx < MITEM_COUNT)
        {
            menu_draw_row(ctx, r, idx, (idx == sel) ? 1U : 0U);
        }
    }
    if (MITEM_COUNT > MENU_VISIBLE_ROWS)
    {
        menu_draw_scrollbar(scroll_top);
    }
}

static void menu_draw_modal_msg(const char *line1, const char *line2, uint16_t border_col)
{
    lcd_fill(24, 72, 295, 162, BLACK);
    lcd_draw_rectangle(24, 72, 295, 162, border_col);
    g_back_color = BLACK;

    size_t l1 = strlen(line1);
    uint16_t x1 = (l1 * 8U < 264U) ? (uint16_t)(28U + (264U - l1 * 8U) / 2U) : 28U;
    lcd_show_string(x1, 90, 264, 16, 16, line1, border_col);

    if (line2 != NULL && line2[0] != '\0')
    {
        size_t l2 = strlen(line2);
        uint16_t x2 = (l2 * 6U < 264U) ? (uint16_t)(28U + (264U - l2 * 6U) / 2U) : 28U;
        lcd_show_string(x2, 122, 264, 12, 12, line2, WHITE);
    }
}

/* Modal NO/YES confirmation (defaults to NO; dial toggles, SHOT confirms, ASA cancels) */
static uint8_t menu_confirm_dialog(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick,
                                   const char *title, const char *sub, uint16_t col)
{
    uint8_t choice = 0U; /* 0 = NO, 1 = YES */
    uint8_t last_dial = key_read_shutter_dial();
    uint8_t need_redraw = 1U;

    while (1)
    {
        if (menu_check_sd_and_power(ctx, had_sd, last_act_tick))
        {
            return 0U;
        }

        if (need_redraw)
        {
            lcd_fill(24, 62, 295, 174, BLACK);
            lcd_draw_rectangle(24, 62, 295, 174, col);
            g_back_color = BLACK;

            size_t lt = strlen(title);
            uint16_t xt = (lt * 8U < 264U) ? (uint16_t)(28U + (264U - lt * 8U) / 2U) : 28U;
            lcd_show_string(xt, 76, 264, 16, 16, title, col);

            if (sub != NULL)
            {
                size_t ls = strlen(sub);
                uint16_t xs = (ls * 6U < 264U) ? (uint16_t)(28U + (264U - ls * 6U) / 2U) : 28U;
                lcd_show_string(xs, 100, 264, 12, 12, sub, MENU_COL_DIM);
            }

            /* NO button */
            uint16_t no_bg = (choice == 0U) ? MENU_COL_SEL_BG : BLACK;
            lcd_fill(58, 126, 140, 154, no_bg);
            lcd_draw_rectangle(58, 126, 140, 154, (choice == 0U) ? CYAN : MENU_COL_DIVIDER);
            g_back_color = no_bg;
            lcd_show_string(91, 132, 32, 16, 16, "NO", (choice == 0U) ? YELLOW : MENU_COL_DIM);

            /* YES button */
            uint16_t yes_bg = (choice == 1U) ? MENU_COL_SEL_BG : BLACK;
            lcd_fill(180, 126, 262, 154, yes_bg);
            lcd_draw_rectangle(180, 126, 262, 154, (choice == 1U) ? col : MENU_COL_DIVIDER);
            g_back_color = yes_bg;
            lcd_show_string(209, 132, 40, 16, 16, "YES", (choice == 1U) ? col : MENU_COL_DIM);

            need_redraw = 0U;
        }

        uint8_t evts = key_poll_events();
        uint8_t dial = key_read_shutter_dial();
        if (dial != last_dial)
        {
            last_dial = dial;
            *last_act_tick = HAL_GetTick();
            choice ^= 1U;
            need_redraw = 1U;
        }
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            *last_act_tick = HAL_GetTick();
        }
        if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
        {
            return 0U;
        }
        if (evts & BTN_EVT_SHOT)
        {
            return choice;
        }
        delay_ms(10);
    }
}

static void menu_show_system_info(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick)
{
    static const char *const fs_names[] = { "NONE", "FAT12", "FAT16", "FAT32", "exFAT" };
    char line[48];

    menu_draw_header("SYSTEM INFO", "PRESS SHOT/ASA:BACK");
    lcd_fill(0, 25, 319, 217, BLACK);
    g_back_color = BLACK;

    const char *fs_str;
    uint32_t total_mb = 0UL;
    uint32_t free_mb  = 0UL;
    if (ctx->sd_ready)
    {
        fs_str = (ctx->fs->fs_type <= FS_EXFAT) ? fs_names[ctx->fs->fs_type] : "UNK";
        total_mb = (uint32_t)SD_TOTAL_SIZE_MB(&hsd);
        DWORD fre_clust = 0UL;
        FATFS *pfs      = NULL;
        if (f_getfree("0:", &fre_clust, &pfs) == FR_OK && pfs != NULL)
        {
            uint64_t free_sectors = (uint64_t)fre_clust * (uint64_t)pfs->csize;
            free_mb = (uint32_t)(free_sectors >> 11); /* 512 B/sector -> >>11 = MB */
        }
    }
    else
    {
        fs_str = ctx->is_sd_present_fn() ? "UNFORMATTED" : "NO SD CARD";
    }

    uint16_t bat_mv  = adc_battery_get_mv();
    uint8_t  bat_pct = adc_mv_to_percent(bat_mv);

    uint16_t y = 32U;
    snprintf(line, sizeof(line), "FILESYSTEM : %s", fs_str);
    lcd_show_string(14, y, 296, 16, 16, line, ctx->sd_ready ? WHITE : ORANGE); y += 22U;

    if (ctx->sd_ready || (g_sd_sta == 0U && ctx->is_sd_present_fn()))
    {
        uint32_t div     = sdio_get_active_clkdiv();
        uint32_t mhz_x10 = (720U + (div + 2U) / 2U) / (div + 2U);
        uint16_t bus_col = (div <= 1U) ? GREEN : ((div <= 4U) ? YELLOW : ORANGE);
        if (div <= 1U)
        {
            snprintf(line, sizeof(line), "SD BUS     : %lu.%lu MHz 4b DMA2",
                     (unsigned long)(mhz_x10 / 10U), (unsigned long)(mhz_x10 % 10U));
        }
        else
        {
            snprintf(line, sizeof(line), "SD BUS     : %lu.%lu MHz (FB) DMA2",
                     (unsigned long)(mhz_x10 / 10U), (unsigned long)(mhz_x10 % 10U));
        }
        lcd_show_string(14, y, 296, 16, 16, line, bus_col);
    }
    else
    {
        lcd_show_string(14, y, 296, 16, 16, "SD BUS     : ---", MENU_COL_DIM);
    }
    y += 22U;

    snprintf(line, sizeof(line), "SD TOTAL   : %lu MB", (unsigned long)total_mb);
    lcd_show_string(14, y, 296, 16, 16, line, WHITE); y += 22U;

    snprintf(line, sizeof(line), "SD FREE    : %lu MB", (unsigned long)free_mb);
    lcd_show_string(14, y, 296, 16, 16, line, ctx->sd_ready ? GREEN : MENU_COL_DIM); y += 22U;

    uint8_t  cmode = (ctx->get_cnt_mode_fn != NULL) ? ctx->get_cnt_mode_fn() : 0U;
    uint16_t cslot = (ctx->get_cnt_slots_fn != NULL) ? ctx->get_cnt_slots_fn() : 0U;
    if (cmode != 0U)
    {
        snprintf(line, sizeof(line), "SHUTTER    : #%05u (WL %u)",
                 (unsigned int)(*ctx->shutter_count), (unsigned int)cslot);
    }
    else
    {
        snprintf(line, sizeof(line), "SHUTTER    : #%05u (STOCK)",
                 (unsigned int)(*ctx->shutter_count));
    }
    lcd_show_string(14, y, 296, 16, 16, line, CYAN); y += 22U;

    snprintf(line, sizeof(line), "BATTERY    : %u.%02uV (%u%%)",
             (unsigned int)(bat_mv / 1000U), (unsigned int)((bat_mv % 1000U) / 10U),
             (unsigned int)bat_pct);
    lcd_show_string(14, y, 296, 16, 16, line, YELLOW); y += 26U;

    snprintf(line, sizeof(line), "CFG: %s", C3M1_CFG_PATH);
    lcd_show_string(14, y, 296, 12, 12, line, MENU_COL_DIM); y += 18U;

    snprintf(line, sizeof(line), "FW : %s (%s)", FW_VERSION_STR, FW_GIT_HASH_STR);
    lcd_show_string(14, y, 296, 12, 12, line, CYAN);

    menu_draw_footer("INFO", CYAN);

    while (1)
    {
        if (menu_check_sd_and_power(ctx, had_sd, last_act_tick))
        {
            return;
        }
        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            *last_act_tick = HAL_GetTick();
        }
        if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG | BTN_EVT_SHOT))
        {
            return;
        }
        delay_ms(10);
    }
}

static uint8_t menu_do_format_sd(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick)
{
    if (!ctx->is_sd_present_fn())
    {
        menu_draw_modal_msg("NO SD CARD!", "INSERT CARD TO FORMAT", RED);
        delay_ms(1200);
        return 0U;
    }

    if (ctx->is_low_bat_fn())
    {
        menu_draw_modal_msg("LOW BATTERY!", "CHARGE > 3.25V TO FORMAT", RED);
        delay_ms(1200);
        return 0U;
    }

    if (!menu_confirm_dialog(ctx, had_sd, last_act_tick, "FORMAT SD CARD?", "ALL PHOTOS WILL BE ERASED", RED))
    {
        return 0U;
    }

    menu_draw_modal_msg("FORMATTING SD...", "DO NOT REMOVE CARD", YELLOW);
    menu_draw_footer("BUSY", YELLOW);

    if (ctx->prepare_format_fn != NULL && !ctx->prepare_format_fn())
    {
        menu_draw_modal_msg("SD INIT FAILED!", "CHECK / RESEAT CARD", RED);
        menu_draw_footer("FAIL", RED);
        delay_ms(1500);
        return 0U;
    }

    /* Unmount, format with FM_ANY (FAT32 <=32GB / exFAT >=64GB), then remount & restore dirs/cfg */
    f_mount(NULL, "0:", 0);
    MKFS_PARM opt;
    memset(&opt, 0, sizeof(opt));
    opt.fmt = FM_ANY;

    FRESULT fres = f_mkfs("0:", &opt, ctx->work_buf, ctx->work_buf_size);
    *last_act_tick = HAL_GetTick();
    if (fres == FR_OK && ctx->remount_sd_fn())
    {
        ctx->sd_ready = 1U;
        if (c3m1_cfg_save(ctx->cfg))
        {
            menu_draw_modal_msg("FORMAT COMPLETE!", "CAYE_CAM & CFG READY", GREEN);
            menu_draw_footer("DONE", GREEN);
            delay_ms(1200);
            return 1U;
        }
        else
        {
            menu_draw_modal_msg("FORMAT COMPLETE!", "CFG SAVE ERR", YELLOW);
            menu_draw_footer("CFG ERR", YELLOW);
            delay_ms(1200);
            return 2U;
        }
    }

    char err_str[32];
    snprintf(err_str, sizeof(err_str), "FATFS ERR = %d", (int)fres);
    menu_draw_modal_msg("FORMAT FAILED!", err_str, RED);
    menu_draw_footer("FAIL", RED);
    delay_ms(1500);
    ctx->sd_ready = ctx->remount_sd_fn() ? 1U : 0U;
    if (!ctx->sd_ready && !ctx->is_sd_present_fn())
    {
        ctx->sd_removed_in_menu = 1U;
    }
    return 0U;
}

static void menu_draw_ev_comp_popup(uint8_t val)
{
    static const char *const major_lbl[5] = { "-2", "-1", "0", "+1", "+2" };

    g_back_color = BLACK;
    lcd_fill(18U, 54U, 301U, 174U, BLACK);
    lcd_draw_rectangle(18U, 54U, 301U, 174U, CYAN);
    lcd_draw_rectangle(19U, 55U, 300U, 173U, CYAN);

    lcd_show_string(108U, 62U, 120U, 16U, 16U, "EXPOSURE COMP", WHITE);

    uint16_t vcol = (val == C3M1_EV_COMP_ZERO) ? GREEN : YELLOW;
    lcd_show_string(136U, 82U, 64U, 16U, 16U, c3m1_cfg_ev_comp_label(val), vcol);

    /* 13 ticks (0..12): x = 40 + i * 20 (0.0EV at exact center x = 160) */
    lcd_draw_hline(40U, 120U, 241U, MENU_COL_DIVIDER);
    for (uint8_t i = 0U; i < C3M1_EV_COMP_COUNT; i++)
    {
        uint16_t tx = (uint16_t)(40U + (uint16_t)i * 20U);
        if ((i % 3U) == 0U)
        {
            uint16_t tc = (i == C3M1_EV_COMP_ZERO) ? GREEN : WHITE;
            lcd_draw_line(tx, 112U, tx, 126U, tc);
            const char *lbl = major_lbl[i / 3U];
            uint16_t lx = (uint16_t)(tx - (strlen(lbl) * 3U));
            lcd_show_string(lx, 130U, 18U, 12U, 12U, lbl, tc);
        }
        else
        {
            lcd_draw_line(tx, 117U, tx, 124U, GRAY);
        }
    }

    /* Highlighted needle & top marker at current selection */
    uint16_t sel_x = (uint16_t)(40U + (uint16_t)val * 20U);
    lcd_fill((uint16_t)(sel_x - 3U), 104U, (uint16_t)(sel_x + 3U), 110U, YELLOW);
    lcd_draw_line(sel_x, 111U, sel_x, 126U, YELLOW);

    lcd_show_string(73U, 152U, 180U, 12U, 12U, "DIAL:ADJ 4X:0 SHOT:OK ASA:ESC", GRAY);
}

static uint8_t menu_edit_ev_comp(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick)
{
    uint8_t orig_val  = (ctx->cfg->ev_comp < C3M1_EV_COMP_COUNT) ? ctx->cfg->ev_comp : C3M1_EV_COMP_ZERO;
    uint8_t val       = orig_val;
    uint8_t last_dial = key_read_shutter_dial();
    menu_draw_ev_comp_popup(val);

    for (;;)
    {
        if (menu_check_sd_and_power(ctx, had_sd, last_act_tick))
        {
            return 0U;
        }

        uint8_t dial_now = key_read_shutter_dial();
        int8_t  dd       = dial_wrap_delta(last_dial, dial_now);
        if (dd != 0)
        {
            last_dial = dial_now;
            *last_act_tick = HAL_GetTick();
            int16_t nv = (int16_t)val + (int16_t)dd;
            if (nv < 0) nv = 0;
            if (nv >= (int16_t)C3M1_EV_COMP_COUNT) nv = (int16_t)(C3M1_EV_COMP_COUNT - 1U);
            if ((uint8_t)nv != val)
            {
                val = (uint8_t)nv;
                menu_draw_ev_comp_popup(val);
            }
        }

        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            *last_act_tick = HAL_GetTick();
        }
        if (evts & BTN_EVT_4X)
        {
            if (val != C3M1_EV_COMP_ZERO)
            {
                val = C3M1_EV_COMP_ZERO;
                menu_draw_ev_comp_popup(val);
            }
        }
        else if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
        {
            return 0U; /* Cancel without saving */
        }
        else if (evts & BTN_EVT_SHOT)
        {
            ctx->cfg->ev_comp = val;
            return (val != orig_val) ? 1U : 0U; /* Skip SD write if unchanged */
        }
        delay_ms(10);
    }
}

static void menu_draw_lens_popup(const char *title, const char *val_str,
                                 const char *min_lbl, const char *mid_lbl, const char *max_lbl,
                                 int16_t val, int16_t vmin, int16_t vmax,
                                 int16_t def_val, const char *hint)
{
    g_back_color = BLACK;
    lcd_fill(18U, 54U, 301U, 174U, BLACK);
    lcd_draw_rectangle(18U, 54U, 301U, 174U, CYAN);
    lcd_draw_rectangle(19U, 55U, 300U, 173U, CYAN);

    size_t lt = strlen(title);
    uint16_t xt = (lt * 8U < 276U) ? (uint16_t)(22U + (276U - lt * 8U) / 2U) : 22U;
    lcd_show_string(xt, 62U, 276U, 16U, 16U, title, WHITE);

    uint16_t vcol = (val == def_val) ? GREEN : YELLOW;
    size_t lv = strlen(val_str);
    uint16_t xv = (lv * 8U < 276U) ? (uint16_t)(22U + (276U - lv * 8U) / 2U) : 22U;
    lcd_show_string(xv, 82U, 276U, 16U, 16U, val_str, vcol);

    /* Horizontal ruler x = 40..280 (span 240 px) */
    lcd_draw_hline(40U, 120U, 241U, MENU_COL_DIVIDER);
    lcd_draw_line(40U,  113U, 40U,  126U, (vmin == def_val) ? GREEN : WHITE);
    lcd_draw_line(160U, 113U, 160U, 126U, (vmin < 0)        ? GREEN : WHITE);
    lcd_draw_line(280U, 113U, 280U, 126U, WHITE);

    lcd_show_string((uint16_t)(40U  - strlen(min_lbl) * 3U), 130U, 36U, 12U, 12U, min_lbl, (vmin == def_val) ? GREEN : WHITE);
    lcd_show_string((uint16_t)(160U - strlen(mid_lbl) * 3U), 130U, 36U, 12U, 12U, mid_lbl, (vmin < 0)        ? GREEN : WHITE);
    lcd_show_string((uint16_t)(280U - strlen(max_lbl) * 3U), 130U, 36U, 12U, 12U, max_lbl, WHITE);

    int32_t span  = (int32_t)vmax - (int32_t)vmin;
    int32_t pos   = (span > 0) ? (((int32_t)(val - vmin) * 240L + (span / 2)) / span) : 0L;
    uint16_t sel_x = (uint16_t)(40L + pos);
    lcd_fill((uint16_t)(sel_x - 3U), 104U, (uint16_t)(sel_x + 3U), 110U, YELLOW);
    lcd_draw_line(sel_x, 111U, sel_x, 126U, YELLOW);

    size_t lh = strlen(hint);
    uint16_t xh = (lh * 6U < 276U) ? (uint16_t)(22U + (276U - lh * 6U) / 2U) : 22U;
    lcd_show_string(xh, 152U, 276U, 12U, 12U, hint, GRAY);
}

static void menu_format_focal_popup(uint16_t mm, char *out, size_t out_sz)
{
    if (mm == 0U)
    {
        snprintf(out, out_sz, "OFF");
    }
    else
    {
        snprintf(out, out_sz, "%umm (EQ %umm)",
                 (unsigned int)mm, (unsigned int)c3m1_cfg_lens_focal_eq35(mm));
    }
}

static uint8_t menu_edit_lens_focal(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick)
{
    int16_t  def_idx   = (int16_t)c3m1_cfg_lens_focal_find_idx(C3M1_LENS_FOCAL_DEFAULT);
    int16_t  idx       = (int16_t)c3m1_cfg_lens_focal_find_idx(ctx->cfg->lens_focal_mm);
    uint16_t orig_mm   = ctx->cfg->lens_focal_mm;
    uint8_t  last_dial = key_read_shutter_dial();
    char vbuf[24];

    menu_format_focal_popup(c3m1_cfg_lens_focal_preset_mm((uint8_t)idx), vbuf, sizeof(vbuf));
    menu_draw_lens_popup("LENS FOCAL", vbuf, "OFF", "45mm", "500mm",
                         idx, 0, (int16_t)(C3M1_LENS_FOCAL_COUNT - 1U),
                         def_idx, "DIAL:ADJ 4X:6mm SHOT:OK ASA:ESC");

    for (;;)
    {
        if (menu_check_sd_and_power(ctx, had_sd, last_act_tick))
        {
            return 0U;
        }

        uint8_t dial_now = key_read_shutter_dial();
        int8_t  dd       = dial_wrap_delta(last_dial, dial_now);
        if (dd != 0)
        {
            last_dial = dial_now;
            *last_act_tick = HAL_GetTick();
            int16_t ni = idx + (int16_t)dd;
            if (ni < 0) ni = 0;
            if (ni >= (int16_t)C3M1_LENS_FOCAL_COUNT) ni = (int16_t)(C3M1_LENS_FOCAL_COUNT - 1U);
            if (ni != idx)
            {
                idx = ni;
                menu_format_focal_popup(c3m1_cfg_lens_focal_preset_mm((uint8_t)idx), vbuf, sizeof(vbuf));
                menu_draw_lens_popup("LENS FOCAL", vbuf, "OFF", "45mm", "500mm",
                                     idx, 0, (int16_t)(C3M1_LENS_FOCAL_COUNT - 1U),
                                     def_idx, "DIAL:ADJ 4X:6mm SHOT:OK ASA:ESC");
            }
        }

        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            *last_act_tick = HAL_GetTick();
        }
        if (evts & BTN_EVT_4X)
        {
            if (idx != def_idx)
            {
                idx = def_idx;
                menu_format_focal_popup(C3M1_LENS_FOCAL_DEFAULT, vbuf, sizeof(vbuf));
                menu_draw_lens_popup("LENS FOCAL", vbuf, "OFF", "45mm", "500mm",
                                     idx, 0, (int16_t)(C3M1_LENS_FOCAL_COUNT - 1U),
                                     def_idx, "DIAL:ADJ 4X:6mm SHOT:OK ASA:ESC");
            }
        }
        else if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
        {
            return 0U;
        }
        else if (evts & BTN_EVT_SHOT)
        {
            uint16_t new_focal = c3m1_cfg_lens_focal_preset_mm((uint8_t)idx);
            if (new_focal == orig_mm) return 0U;
            ctx->cfg->lens_focal_mm = new_focal;
            if (ctx->cfg->lens_profile != C3M1_LENS_PROFILE_UNKNOWN &&
                new_focal != c3m1_cfg_lens_profile_get(ctx->cfg->lens_profile)->focal_mm)
            {
                ctx->cfg->lens_profile = C3M1_LENS_PROFILE_UNKNOWN;
            }
            return 1U;
        }
        delay_ms(10);
    }
}

static uint8_t menu_edit_lens_aperture(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick)
{
    int16_t  orig_val  = (ctx->cfg->lens_fnum_x10 <= C3M1_LENS_FNUM_MAX_X10) ? (int16_t)ctx->cfg->lens_fnum_x10 : 0;
    int16_t  val       = orig_val;
    uint8_t  last_dial = key_read_shutter_dial();
    uint32_t accel_tk  = 0UL;
    int8_t   accel_dir = 0;
    char vbuf[10];

    c3m1_cfg_format_lens_fnum((uint16_t)val, vbuf, sizeof(vbuf));
    menu_draw_lens_popup("LENS APERTURE", vbuf, "F0", "F16", "F32",
                         val, 0, (int16_t)C3M1_LENS_FNUM_MAX_X10,
                         0, "DIAL:ADJ 4X:F0 SHOT:OK ASA:ESC");

    for (;;)
    {
        if (menu_check_sd_and_power(ctx, had_sd, last_act_tick))
        {
            return 0U;
        }

        uint8_t dial_now = key_read_shutter_dial();
        int8_t  dd       = dial_wrap_delta(last_dial, dial_now);
        if (dd != 0)
        {
            uint32_t now = HAL_GetTick();
            last_dial      = dial_now;
            *last_act_tick = now;
            int16_t nv = val + menu_accel_delta(dd, now, &accel_tk, &accel_dir);
            if (nv < 0) nv = 0;
            if (nv > (int16_t)C3M1_LENS_FNUM_MAX_X10) nv = (int16_t)C3M1_LENS_FNUM_MAX_X10;
            if (nv != val)
            {
                val = nv;
                c3m1_cfg_format_lens_fnum((uint16_t)val, vbuf, sizeof(vbuf));
                menu_draw_lens_popup("LENS APERTURE", vbuf, "F0", "F16", "F32",
                                     val, 0, (int16_t)C3M1_LENS_FNUM_MAX_X10,
                                     0, "DIAL:ADJ 4X:F0 SHOT:OK ASA:ESC");
            }
        }

        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            *last_act_tick = HAL_GetTick();
        }
        if (evts & BTN_EVT_4X)
        {
            if (val != 0)
            {
                val = 0;
                c3m1_cfg_format_lens_fnum(0U, vbuf, sizeof(vbuf));
                menu_draw_lens_popup("LENS APERTURE", vbuf, "F0", "F16", "F32",
                                     val, 0, (int16_t)C3M1_LENS_FNUM_MAX_X10,
                                     0, "DIAL:ADJ 4X:F0 SHOT:OK ASA:ESC");
            }
        }
        else if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
        {
            return 0U;
        }
        else if (evts & BTN_EVT_SHOT)
        {
            if (val == orig_val) return 0U;
            uint16_t new_fnum = (uint16_t)val;
            ctx->cfg->lens_fnum_x10 = new_fnum;
            if (ctx->cfg->lens_profile != C3M1_LENS_PROFILE_UNKNOWN &&
                new_fnum != c3m1_cfg_lens_profile_get(ctx->cfg->lens_profile)->fnum_x10)
            {
                ctx->cfg->lens_profile = C3M1_LENS_PROFILE_UNKNOWN;
            }
            return 1U;
        }
        delay_ms(10);
    }
}

static uint8_t menu_edit_lens_dist(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick)
{
    int16_t orig_val = ctx->cfg->lens_distort;
    if (orig_val < C3M1_LENS_DIST_MIN || orig_val > C3M1_LENS_DIST_MAX) orig_val = 0;
    int16_t  val       = orig_val;
    uint8_t  last_dial = key_read_shutter_dial();
    uint32_t accel_tk  = 0UL;
    int8_t   accel_dir = 0;
    char vbuf[10];

    c3m1_cfg_format_lens_dist((int8_t)val, vbuf, sizeof(vbuf));
    menu_draw_lens_popup("LENS DISTORT (kr1)", vbuf, "-0.20", "0", "+0.20",
                         val, C3M1_LENS_DIST_MIN, C3M1_LENS_DIST_MAX,
                         0, "DIAL:ADJ 4X:0 SHOT:OK ASA:ESC");

    for (;;)
    {
        if (menu_check_sd_and_power(ctx, had_sd, last_act_tick))
        {
            return 0U;
        }

        uint8_t dial_now = key_read_shutter_dial();
        int8_t  dd       = dial_wrap_delta(last_dial, dial_now);
        if (dd != 0)
        {
            uint32_t now = HAL_GetTick();
            last_dial      = dial_now;
            *last_act_tick = now;
            int16_t nv = val + menu_accel_delta(dd, now, &accel_tk, &accel_dir);
            if (nv < C3M1_LENS_DIST_MIN) nv = C3M1_LENS_DIST_MIN;
            if (nv > C3M1_LENS_DIST_MAX) nv = C3M1_LENS_DIST_MAX;
            if (nv != val)
            {
                val = nv;
                c3m1_cfg_format_lens_dist((int8_t)val, vbuf, sizeof(vbuf));
                menu_draw_lens_popup("LENS DISTORT (kr1)", vbuf, "-0.20", "0", "+0.20",
                                     val, C3M1_LENS_DIST_MIN, C3M1_LENS_DIST_MAX,
                                     0, "DIAL:ADJ 4X:0 SHOT:OK ASA:ESC");
            }
        }

        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            *last_act_tick = HAL_GetTick();
        }
        if (evts & BTN_EVT_4X)
        {
            if (val != 0)
            {
                val = 0;
                c3m1_cfg_format_lens_dist(0, vbuf, sizeof(vbuf));
                menu_draw_lens_popup("LENS DISTORT (kr1)", vbuf, "-0.20", "0", "+0.20",
                                     val, C3M1_LENS_DIST_MIN, C3M1_LENS_DIST_MAX,
                                     0, "DIAL:ADJ 4X:0 SHOT:OK ASA:ESC");
            }
        }
        else if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
        {
            return 0U;
        }
        else if (evts & BTN_EVT_SHOT)
        {
            if (val == orig_val) return 0U;
            ctx->cfg->lens_distort = (int8_t)val;
            return 1U;
        }
        delay_ms(10);
    }
}

static uint8_t menu_edit_lens_vig(c3m1_menu_ctx_t *ctx, uint8_t had_sd, uint32_t *last_act_tick)
{
    int16_t  orig_val  = (ctx->cfg->lens_vignette <= C3M1_LENS_VIG_MAX) ? (int16_t)ctx->cfg->lens_vignette : 0;
    int16_t  val       = orig_val;
    uint8_t  last_dial = key_read_shutter_dial();
    uint32_t accel_tk  = 0UL;
    int8_t   accel_dir = 0;
    char vbuf[10];

    c3m1_cfg_format_lens_vig((uint8_t)val, vbuf, sizeof(vbuf));
    menu_draw_lens_popup("LENS VIGNETTE (k0)", vbuf, "0", "+1.25", "+2.50",
                         val, 0, (int16_t)C3M1_LENS_VIG_MAX,
                         0, "DIAL:ADJ 4X:0 SHOT:OK ASA:ESC");

    for (;;)
    {
        if (menu_check_sd_and_power(ctx, had_sd, last_act_tick))
        {
            return 0U;
        }

        uint8_t dial_now = key_read_shutter_dial();
        int8_t  dd       = dial_wrap_delta(last_dial, dial_now);
        if (dd != 0)
        {
            uint32_t now = HAL_GetTick();
            last_dial      = dial_now;
            *last_act_tick = now;
            int16_t nv = val + menu_accel_delta(dd, now, &accel_tk, &accel_dir);
            if (nv < 0) nv = 0;
            if (nv > (int16_t)C3M1_LENS_VIG_MAX) nv = (int16_t)C3M1_LENS_VIG_MAX;
            if (nv != val)
            {
                val = nv;
                c3m1_cfg_format_lens_vig((uint8_t)val, vbuf, sizeof(vbuf));
                menu_draw_lens_popup("LENS VIGNETTE (k0)", vbuf, "0", "+1.25", "+2.50",
                                     val, 0, (int16_t)C3M1_LENS_VIG_MAX,
                                     0, "DIAL:ADJ 4X:0 SHOT:OK ASA:ESC");
            }
        }

        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            *last_act_tick = HAL_GetTick();
        }
        if (evts & BTN_EVT_4X)
        {
            if (val != 0)
            {
                val = 0;
                c3m1_cfg_format_lens_vig(0U, vbuf, sizeof(vbuf));
                menu_draw_lens_popup("LENS VIGNETTE (k0)", vbuf, "0", "+1.25", "+2.50",
                                     val, 0, (int16_t)C3M1_LENS_VIG_MAX,
                                     0, "DIAL:ADJ 4X:0 SHOT:OK ASA:ESC");
            }
        }
        else if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
        {
            return 0U;
        }
        else if (evts & BTN_EVT_SHOT)
        {
            if (val == orig_val) return 0U;
            ctx->cfg->lens_vignette = (uint8_t)val;
            return 1U;
        }
        delay_ms(10);
    }
}

void c3m1_menu_run(c3m1_menu_ctx_t *ctx)
{
    uint8_t  sel           = 0U;
    uint8_t  scroll_top    = 0U;
    uint8_t  last_dial     = key_read_shutter_dial();
    uint8_t  dirty         = 0U;
    uint8_t  had_sd        = ctx->is_sd_present_fn() ? 1U : 0U;
    uint32_t last_act_tick = HAL_GetTick();

    ctx->sd_removed_in_menu = 0U;
    ctx->woke_from_sleep    = 0U;

    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
    menu_draw_list(ctx, sel, scroll_top);
    menu_draw_idle_footer(ctx);

    while (1)
    {
        if (menu_check_sd_and_power(ctx, had_sd, &last_act_tick))
        {
            break;
        }

        uint8_t evts = key_poll_events();
        uint8_t dial = key_read_shutter_dial();

        if (dial != last_dial)
        {
            int8_t d = dial_wrap_delta(last_dial, dial);
            last_dial = dial;
            last_act_tick = HAL_GetTick();
            if (d != 0)
            {
                uint8_t old_sel = sel;
                uint8_t old_top = scroll_top;
                int16_t next = (int16_t)sel + (int16_t)d;
                while (next < 0) next += (int16_t)MITEM_COUNT;
                while (next >= (int16_t)MITEM_COUNT) next -= (int16_t)MITEM_COUNT;
                sel = (uint8_t)next;

                if (sel < scroll_top)
                {
                    scroll_top = sel;
                }
                else if (sel >= (uint8_t)(scroll_top + MENU_VISIBLE_ROWS))
                {
                    scroll_top = (uint8_t)(sel - MENU_VISIBLE_ROWS + 1U);
                }

                if (scroll_top != old_top)
                {
                    menu_draw_list(ctx, sel, scroll_top);
                }
                else
                {
                    menu_draw_row(ctx, (uint8_t)(old_sel - scroll_top), old_sel, 0U);
                    menu_draw_row(ctx, (uint8_t)(sel - scroll_top), sel, 1U);
                }
            }
        }

        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            last_act_tick = HAL_GetTick();
        }

        if (evts & (BTN_EVT_ISO | BTN_EVT_ISO_LONG))
        {
            break; /* Short or long press ASA -> exit menu */
        }

        if (evts & BTN_EVT_SHOT)
        {
            uint8_t setting_changed = 0U;
            switch (sel)
            {
                case MITEM_COLOR_MODE:
                    ctx->cfg->color_mode = (uint8_t)((ctx->cfg->color_mode + 1U) % C3M1_COLOR_MODE_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_SAVE_MODE:
                    ctx->cfg->save_mode = (uint8_t)((ctx->cfg->save_mode + 1U) % C3M1_SAVE_MODE_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_DRIVE_MODE:
                    if (ctx->drive_mode != NULL)
                    {
                        *ctx->drive_mode = (uint8_t)((*ctx->drive_mode + 1U) % C3M1_DRIVE_MODE_COUNT);
                        if (*ctx->drive_mode != C3M1_DRIVE_SINGLE)
                        {
                            if (ctx->hdr_mode != NULL) *ctx->hdr_mode = C3M1_HDR_OFF;
                            if (ctx->jsr_mode != NULL) *ctx->jsr_mode = C3M1_JSR_OFF;
                            menu_draw_list(ctx, sel, scroll_top);
                        }
                        else
                        {
                            menu_draw_row(ctx, (uint8_t)(sel - scroll_top), sel, 1U);
                        }
                        menu_draw_footer("TEMP", CYAN);
                    }
                    break;

                case MITEM_HDR_MODE:
                    if (ctx->hdr_mode != NULL)
                    {
                        *ctx->hdr_mode = (uint8_t)((*ctx->hdr_mode + 1U) % C3M1_HDR_MODE_COUNT);
                        if (*ctx->hdr_mode != C3M1_HDR_OFF && ctx->drive_mode != NULL &&
                            *ctx->drive_mode != C3M1_DRIVE_SINGLE)
                        {
                            *ctx->drive_mode = C3M1_DRIVE_SINGLE;
                            menu_draw_list(ctx, sel, scroll_top);
                        }
                        else
                        {
                            menu_draw_row(ctx, (uint8_t)(sel - scroll_top), sel, 1U);
                        }
                        menu_draw_footer("TEMP", CYAN);
                    }
                    break;

                case MITEM_JSR_MODE:
                    if (ctx->jsr_mode != NULL)
                    {
                        *ctx->jsr_mode = (uint8_t)((*ctx->jsr_mode + 1U) % C3M1_JSR_MODE_COUNT);
                        if (*ctx->jsr_mode != C3M1_JSR_OFF && ctx->drive_mode != NULL &&
                            *ctx->drive_mode != C3M1_DRIVE_SINGLE)
                        {
                            *ctx->drive_mode = C3M1_DRIVE_SINGLE;
                            menu_draw_list(ctx, sel, scroll_top);
                        }
                        else
                        {
                            menu_draw_row(ctx, (uint8_t)(sel - scroll_top), sel, 1U);
                        }
                        menu_draw_footer("TEMP", CYAN);
                    }
                    break;

                case MITEM_WB_MODE:
                    ctx->cfg->wb_mode = (uint8_t)((ctx->cfg->wb_mode + 1U) % C3M1_WB_MODE_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_EV_COMP:
                {
                    uint8_t saved = menu_edit_ev_comp(ctx, had_sd, &last_act_tick);
                    if (ctx->sd_removed_in_menu || ctx->woke_from_sleep) goto menu_exit;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    if (saved)
                    {
                        setting_changed = 1U;
                    }
                    else
                    {
                        menu_draw_idle_footer(ctx);
                    }
                    break;
                }

                case MITEM_METER_MODE:
                    ctx->cfg->meter_mode = (uint8_t)((ctx->cfg->meter_mode + 1U) % C3M1_METER_MODE_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_MAX_ISO:
                    ctx->cfg->max_auto_iso = (uint8_t)((ctx->cfg->max_auto_iso + 1U) % C3M1_MAX_ISO_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_LENS_PROFILE:
                {
                    uint8_t next_prof = (uint8_t)((ctx->cfg->lens_profile + 1U) % C3M1_LENS_PROFILE_COUNT);
                    c3m1_cfg_apply_lens_profile(ctx->cfg, next_prof);
                    menu_draw_list(ctx, sel, scroll_top);
                    setting_changed = 1U;
                    break;
                }

                case MITEM_MIN_SHT:
                    ctx->cfg->min_auto_sht = (uint8_t)((ctx->cfg->min_auto_sht + 1U) % C3M1_MIN_SHT_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_LENS_FOCAL:
                {
                    uint8_t saved = menu_edit_lens_focal(ctx, had_sd, &last_act_tick);
                    if (ctx->sd_removed_in_menu || ctx->woke_from_sleep) goto menu_exit;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    if (saved)
                    {
                        setting_changed = 1U;
                    }
                    else
                    {
                        menu_draw_idle_footer(ctx);
                    }
                    break;
                }

                case MITEM_LENS_APERTURE:
                {
                    uint8_t saved = menu_edit_lens_aperture(ctx, had_sd, &last_act_tick);
                    if (ctx->sd_removed_in_menu || ctx->woke_from_sleep) goto menu_exit;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    if (saved)
                    {
                        setting_changed = 1U;
                    }
                    else
                    {
                        menu_draw_idle_footer(ctx);
                    }
                    break;
                }

                case MITEM_LENS_DIST:
                {
                    uint8_t saved = menu_edit_lens_dist(ctx, had_sd, &last_act_tick);
                    if (ctx->sd_removed_in_menu || ctx->woke_from_sleep) goto menu_exit;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    if (saved)
                    {
                        setting_changed = 1U;
                    }
                    else
                    {
                        menu_draw_idle_footer(ctx);
                    }
                    break;
                }

                case MITEM_LENS_VIG:
                {
                    uint8_t saved = menu_edit_lens_vig(ctx, had_sd, &last_act_tick);
                    if (ctx->sd_removed_in_menu || ctx->woke_from_sleep) goto menu_exit;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    if (saved)
                    {
                        setting_changed = 1U;
                    }
                    else
                    {
                        menu_draw_idle_footer(ctx);
                    }
                    break;
                }

                case MITEM_INCAM_DIST:
                    ctx->cfg->lens_dist_incam ^= 1U;
                    setting_changed = 1U;
                    break;

                case MITEM_HIGH_ISO_NR:
                    ctx->cfg->high_iso_nr ^= 1U;
                    setting_changed = 1U;
                    break;

                case MITEM_ANTI_ALIAS:
                    ctx->cfg->anti_alias ^= 1U;
                    setting_changed = 1U;
                    break;

                case MITEM_FLICKER:
                    ctx->cfg->flicker_mode = (uint8_t)((ctx->cfg->flicker_mode + 1U) % C3M1_FLICKER_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_PEAK_COLOR:
                    ctx->cfg->peak_color = (uint8_t)((ctx->cfg->peak_color + 1U) % C3M1_PEAK_COLOR_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_HISTOGRAM:
                    ctx->cfg->hist_en = (uint8_t)((ctx->cfg->hist_en + 1U) % C3M1_HIST_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_GRID:
                    ctx->cfg->grid_en = (uint8_t)((ctx->cfg->grid_en + 1U) % C3M1_GRID_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_BAT_DISP:
                    ctx->cfg->bat_disp = (uint8_t)((ctx->cfg->bat_disp + 1U) % C3M1_BAT_DISP_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_FILM_LEVER:
                    ctx->cfg->film_lever = (uint8_t)((ctx->cfg->film_lever + 1U) % C3M1_FILM_LEVER_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_AUTO_SLEEP:
                    ctx->cfg->auto_sleep = (uint8_t)((ctx->cfg->auto_sleep + 1U) % C3M1_AUTO_SLEEP_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_BOOT_CLOCK:
                    ctx->cfg->boot_clock = (uint8_t)((ctx->cfg->boot_clock + 1U) % C3M1_BOOT_CLOCK_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_PB_SKIP_BRK:
                    ctx->cfg->pb_skip_brk = (uint8_t)((ctx->cfg->pb_skip_brk + 1U) % C3M1_PB_SKIP_BRK_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_DEL_HDR_BRK:
                    ctx->cfg->del_hdr_brk = (uint8_t)((ctx->cfg->del_hdr_brk + 1U) % C3M1_DEL_HDR_BRK_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_FORMAT_SD:
                {
                    uint8_t fmt_res = menu_do_format_sd(ctx, had_sd, &last_act_tick);
                    if (ctx->sd_removed_in_menu || ctx->woke_from_sleep) goto menu_exit;
                    if (ctx->sd_ready) had_sd = 1U;
                    if (fmt_res == 1U) dirty = 0U;
                    else if (fmt_res == 2U) dirty = 1U;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    menu_draw_idle_footer(ctx);
                    break;
                }

                case MITEM_CNT_FORMAT:
                    if (ctx->is_low_bat_fn())
                    {
                        menu_draw_footer("LOW BAT", RED);
                    }
                    else if (ctx->get_cnt_mode_fn != NULL && ctx->set_cnt_mode_fn != NULL)
                    {
                        uint8_t next_mode = (ctx->get_cnt_mode_fn() == 0U) ? 1U : 0U;
                        if (ctx->set_cnt_mode_fn(next_mode))
                        {
                            menu_draw_row(ctx, (uint8_t)(sel - scroll_top), sel, 1U);
                            menu_draw_footer((next_mode != 0U) ? "WL ACTIVE" : "STOCK FMT", YELLOW);
                        }
                        else
                        {
                            menu_draw_footer("FLASH ERR", RED);
                        }
                    }
                    break;

                case MITEM_DEBUG_INFO:
                    ctx->cfg->debug_info = (uint8_t)((ctx->cfg->debug_info + 1U) % C3M1_DEBUG_INFO_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_SD_INFO:
                    menu_show_system_info(ctx, had_sd, &last_act_tick);
                    if (ctx->sd_removed_in_menu || ctx->woke_from_sleep) goto menu_exit;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    menu_draw_idle_footer(ctx);
                    break;

                case MITEM_EXIT:
                    goto menu_exit;

                default:
                    break;
            }

            if (setting_changed)
            {
                menu_draw_row(ctx, (uint8_t)(sel - scroll_top), sel, 1U);
                if (!ctx->sd_ready)
                {
                    menu_draw_footer("RAM ONLY", ORANGE);
                }
                else if (ctx->is_low_bat_fn())
                {
                    dirty = 1U;
                    menu_draw_footer("LOW BAT", RED);
                }
                else if (c3m1_cfg_save(ctx->cfg))
                {
                    dirty = 0U;
                    menu_draw_footer("SAVED", YELLOW);
                }
                else
                {
                    dirty = 1U;
                    menu_draw_footer("SD ERR", RED);
                }
            }
        }

        delay_ms(10);
    }

menu_exit:
    if (dirty && ctx->sd_ready && !ctx->sd_removed_in_menu && !ctx->woke_from_sleep &&
        ctx->is_sd_present_fn() && !ctx->is_low_bat_fn())
    {
        c3m1_cfg_save(ctx->cfg);
    }

    if (ctx->woke_from_sleep)
    {
        return;
    }

    /* Wait for all top buttons to be released before returning so SHOT never triggers a photo */
    uint32_t t0 = HAL_GetTick();
    while (key_any_top_button_pressed() && (HAL_GetTick() - t0) < 1000U)
    {
        delay_ms(10);
    }
    key_init_state();
    lcd_clear(BLACK);
}

static void clock_ui_draw_box(uint16_t x0, uint16_t x1, const char *label, const char *val, uint8_t active)
{
    uint16_t bg     = active ? MENU_COL_SEL_BG : BLACK;
    uint16_t border = active ? YELLOW : MENU_COL_DIVIDER;
    uint16_t lcol   = active ? CYAN : MENU_COL_DIM;
    uint16_t vcol   = active ? YELLOW : WHITE;

    /* Label centered above box */
    size_t   llen = strlen(label);
    uint16_t lw   = (uint16_t)(llen * 8U);
    uint16_t lx   = (uint16_t)(x0 + ((x1 - x0 + 1U) - lw) / 2U);
    g_back_color = BLACK;
    lcd_show_string(lx, 62U, lw, 16U, 16U, label, lcol);

    /* Value box (y = 84..134) */
    lcd_fill(x0, 84U, x1, 134U, bg);
    lcd_draw_rectangle(x0, 84U, x1, 134U, border);
    if (active)
    {
        lcd_draw_rectangle((uint16_t)(x0 + 1U), 85U, (uint16_t)(x1 - 1U), 133U, YELLOW);
    }

    size_t   vlen = strlen(val);
    uint16_t vw   = (uint16_t)(vlen * 12U);
    uint16_t vx   = (uint16_t)(x0 + ((x1 - x0 + 1U) - vw) / 2U);
    g_back_color = bg;
    lcd_show_string(vx, 97U, vw, 24U, 24U, val, vcol);
}

static void clock_ui_draw_page(const c3m1_datetime_t *dt, uint8_t step)
{
    uint8_t page = (step < 3U) ? 0U : 1U;
    char s0[8], s1[8], s2[8], ctx_line[28];

    menu_draw_header((page == 0U) ? "SET DATE (1/2)" : "SET TIME (2/2)",
                     (step == 5U) ? "DIAL:ADJ SHOT:DONE" : "DIAL:ADJ SHOT:NEXT");

    lcd_fill(0U, 25U, 319U, 217U, BLACK);

    if (page == 0U)
    {
        snprintf(s0, sizeof(s0), "%04u", (unsigned int)dt->year);
        snprintf(s1, sizeof(s1), "%02u", (unsigned int)dt->month);
        snprintf(s2, sizeof(s2), "%02u", (unsigned int)dt->day);

        clock_ui_draw_box( 35U, 109U, "YEAR", s0, (step == 0U) ? 1U : 0U);
        clock_ui_draw_box(139U, 197U, "MON",  s1, (step == 1U) ? 1U : 0U);
        clock_ui_draw_box(227U, 285U, "DAY",  s2, (step == 2U) ? 1U : 0U);

        g_back_color = BLACK;
        lcd_show_string(118U, 97U, 12U, 24U, 24U, "-", MENU_COL_DIM);
        lcd_show_string(206U, 97U, 12U, 24U, 24U, "-", MENU_COL_DIM);

        snprintf(ctx_line, sizeof(ctx_line), "TIME  %02u:%02u:%02u",
                 (unsigned int)dt->hour, (unsigned int)dt->min, (unsigned int)dt->sec);
    }
    else
    {
        snprintf(s0, sizeof(s0), "%02u", (unsigned int)dt->hour);
        snprintf(s1, sizeof(s1), "%02u", (unsigned int)dt->min);
        snprintf(s2, sizeof(s2), "%02u", (unsigned int)dt->sec);

        clock_ui_draw_box( 38U, 101U, "HOUR", s0, (step == 3U) ? 1U : 0U);
        clock_ui_draw_box(128U, 191U, "MIN",  s1, (step == 4U) ? 1U : 0U);
        clock_ui_draw_box(218U, 281U, "SEC",  s2, (step == 5U) ? 1U : 0U);

        g_back_color = BLACK;
        lcd_show_string(109U, 97U, 12U, 24U, 24U, ":", MENU_COL_DIM);
        lcd_show_string(199U, 97U, 12U, 24U, 24U, ":", MENU_COL_DIM);

        snprintf(ctx_line, sizeof(ctx_line), "DATE  %04u-%02u-%02u",
                 (unsigned int)dt->year, (unsigned int)dt->month, (unsigned int)dt->day);
    }

    g_back_color = BLACK;
    uint16_t cw = (uint16_t)(strlen(ctx_line) * 8U);
    lcd_show_string((uint16_t)((320U - cw) / 2U), 160U, cw, 16U, 16U, ctx_line, MENU_COL_DIM);

    /* Footer hint bar */
    lcd_fill(0U, 218U, 319U, 239U, BLACK);
    lcd_draw_hline(0U, 218U, 320U, MENU_COL_DIVIDER);
    const char *hint = (step == 0U) ? "DIAL:ADJ   SHOT:NEXT"
                     : (step == 5U) ? "DIAL:ADJ   4X:BACK   SHOT:DONE"
                                    : "DIAL:ADJ   4X:BACK   SHOT:NEXT";
    uint16_t hw = (uint16_t)(strlen(hint) * 6U);
    lcd_show_string((uint16_t)((320U - hw) / 2U), 223U, hw, 12U, 12U, hint, CYAN);
}

static uint8_t wrap_range(int32_t v, int32_t lo, int32_t hi)
{
    int32_t span = hi - lo + 1;
    while (v < lo) v += span;
    while (v > hi) v -= span;
    return (uint8_t)v;
}

uint8_t c3m1_menu_set_clock_ui(c3m1_datetime_t *dt)
{
    uint8_t step = 0U; /* 0:YEAR, 1:MON, 2:DAY, 3:HOUR, 4:MIN, 5:SEC */

    key_init_state();
    clock_ui_draw_page(dt, step);
    /* Settle BCD dial debounce AFTER key_init_state() so power-on noise never jumps YEAR */
    for (uint8_t k = 0U; k < 9U; k++)
    {
        delay_ms(10);
        (void)key_read_shutter_dial();
    }
    uint8_t  last_dial     = key_read_shutter_dial();
    uint32_t last_act_tick = HAL_GetTick();

    while (step < 6U)
    {
        if (c3m1_power_service_modal(&last_act_tick))
        {
            return 0U; /* Exited via sleep wake; do not clobber sleep-advanced clock */
        }

        uint8_t dial_now = key_read_shutter_dial();
        int8_t  dd       = dial_wrap_delta(last_dial, dial_now);
        if (dd != 0)
        {
            last_dial     = dial_now;
            last_act_tick = HAL_GetTick();
            switch (step)
            {
                case 0U:
                {
                    int32_t ny = (int32_t)dt->year + (int32_t)dd;
                    if (ny < (int32_t)C3M1_CLOCK_YEAR_MIN) ny = (int32_t)C3M1_CLOCK_YEAR_MIN;
                    if (ny > (int32_t)C3M1_CLOCK_YEAR_MAX) ny = (int32_t)C3M1_CLOCK_YEAR_MAX;
                    dt->year = (uint16_t)ny;
                    uint8_t max_d = c3m1_clock_days_in_month(dt->year, dt->month);
                    if (dt->day > max_d) dt->day = max_d;
                    break;
                }
                case 1U:
                {
                    dt->month = wrap_range((int32_t)dt->month + (int32_t)dd, 1, 12);
                    uint8_t max_d = c3m1_clock_days_in_month(dt->year, dt->month);
                    if (dt->day > max_d) dt->day = max_d;
                    break;
                }
                case 2U:
                {
                    uint8_t max_d = c3m1_clock_days_in_month(dt->year, dt->month);
                    dt->day = wrap_range((int32_t)dt->day + (int32_t)dd, 1, (int32_t)max_d);
                    break;
                }
                case 3U:
                    dt->hour = wrap_range((int32_t)dt->hour + (int32_t)dd, 0, 23);
                    break;
                case 4U:
                    dt->min = wrap_range((int32_t)dt->min + (int32_t)dd, 0, 59);
                    break;
                case 5U:
                    dt->sec = wrap_range((int32_t)dt->sec + (int32_t)dd, 0, 59);
                    break;
                default:
                    break;
            }
            clock_ui_draw_page(dt, step);
        }

        uint8_t evts = key_poll_events();
        if (evts != BTN_EVT_NONE || key_any_top_button_pressed())
        {
            last_act_tick = HAL_GetTick();
        }
        if (evts & BTN_EVT_SHOT)
        {
            step++;
            if (step < 6U)
            {
                clock_ui_draw_page(dt, step);
            }
        }
        else if (evts & BTN_EVT_4X)
        {
            if (step > 0U)
            {
                step--;
                clock_ui_draw_page(dt, step);
            }
        }

        delay_ms(10);
    }

    /* Wait for buttons released so confirming SEC never triggers an immediate photo */
    uint32_t t0 = HAL_GetTick();
    while (key_any_top_button_pressed() && (HAL_GetTick() - t0) < 1000U)
    {
        delay_ms(10);
    }
    key_init_state();
    lcd_clear(BLACK);
    return 1U;
}
