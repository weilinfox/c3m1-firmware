/**
 * @file        menu.c
 * @brief       Pure-English ASCII menu UI for C3M1 (320x240 ST7789V3).
 *              Navigation: 16-pos BCD dial (+/- wrap), SHOT=select/cycle, ASA=back/exit.
 */

#include <stdio.h>
#include <string.h>

#include "menu.h"
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
    MITEM_MAX_ISO = 0,
    MITEM_MIN_SHT,
    MITEM_FLICKER,
    MITEM_BAT_DISP,
    MITEM_FILM_LEVER,
    MITEM_FORMAT_SD,
    MITEM_SD_INFO,
    MITEM_EXIT,
    MITEM_COUNT
};

static const char *const s_item_labels[MITEM_COUNT] = {
    "MAX AUTO ISO",
    "MIN AUTO SHT",
    "ANTI-FLICKER",
    "BATTERY DISP",
    "FILM LEVER",
    "FORMAT SD",
    "SYSTEM INFO",
    "EXIT MENU",
};

static const char *const s_max_iso_vals[C3M1_MAX_ISO_COUNT]       = { "400", "800", "1600" };
static const char *const s_min_sht_vals[C3M1_MIN_SHT_COUNT]       = { "1/100", "1/50", "1/25", "1/15", "1/8" };
static const char *const s_flicker_vals[C3M1_FLICKER_COUNT]       = { "50HZ", "60HZ", "OFF" };
static const char *const s_bat_disp_vals[C3M1_BAT_DISP_COUNT]     = { "VOLT", "PCT" };
static const char *const s_film_lever_vals[C3M1_FILM_LEVER_COUNT] = { "OFF", "ON" };

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
        lcd_show_string(254, 223, 62, 12, 12, status_msg, status_col);
    }
}

static const char *menu_item_value(const c3m1_menu_ctx_t *ctx, uint8_t idx)
{
    const c3m1_cfg_t *c = ctx->cfg;
    switch (idx)
    {
        case MITEM_MAX_ISO:
            return s_max_iso_vals[c->max_auto_iso % C3M1_MAX_ISO_COUNT];
        case MITEM_MIN_SHT:
            return s_min_sht_vals[c->min_auto_sht % C3M1_MIN_SHT_COUNT];
        case MITEM_FLICKER:
            return s_flicker_vals[c->flicker_mode % C3M1_FLICKER_COUNT];
        case MITEM_BAT_DISP:
            return s_bat_disp_vals[c->bat_disp % C3M1_BAT_DISP_COUNT];
        case MITEM_FILM_LEVER:
            return s_film_lever_vals[c->film_lever % C3M1_FILM_LEVER_COUNT];
        case MITEM_FORMAT_SD:
            return "RUN >";
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
    uint16_t vx = (vw < 100U) ? (uint16_t)(304U - vw) : 204U;
    lcd_show_string(vx, (uint16_t)(y + 4U), 104, 16, 16, val, vc);
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
static uint8_t menu_confirm_dialog(c3m1_menu_ctx_t *ctx, const char *title, const char *sub, uint16_t col)
{
    uint8_t choice = 0U; /* 0 = NO, 1 = YES */
    uint8_t last_dial = key_read_shutter_dial();
    uint8_t need_redraw = 1U;

    while (1)
    {
        if (!ctx->is_sd_present_fn())
        {
            ctx->sd_removed_in_menu = 1U;
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
            choice ^= 1U;
            need_redraw = 1U;
        }
        if (evts & BTN_EVT_ISO)
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

static void menu_show_system_info(c3m1_menu_ctx_t *ctx)
{
    static const char *const fs_names[] = { "NONE", "FAT12", "FAT16", "FAT32", "exFAT" };
    char line[48];

    menu_draw_header("SYSTEM INFO", "PRESS SHOT/ASA:BACK");
    lcd_fill(0, 25, 319, 217, BLACK);
    g_back_color = BLACK;

    const char *fs_str = (ctx->fs->fs_type <= FS_EXFAT) ? fs_names[ctx->fs->fs_type] : "UNK";
    uint32_t total_mb = (uint32_t)SD_TOTAL_SIZE_MB(&hsd);
    uint32_t free_mb  = 0UL;
    DWORD fre_clust   = 0UL;
    FATFS *pfs        = NULL;
    if (f_getfree("0:", &fre_clust, &pfs) == FR_OK && pfs != NULL)
    {
        uint64_t free_sectors = (uint64_t)fre_clust * (uint64_t)pfs->csize;
        free_mb = (uint32_t)(free_sectors >> 11); /* 512 B/sector -> >>11 = MB */
    }

    uint16_t bat_mv  = adc_battery_get_mv();
    uint8_t  bat_pct = adc_battery_get_percent();

    uint16_t y = 36U;
    snprintf(line, sizeof(line), "FILESYSTEM : %s", fs_str);
    lcd_show_string(14, y, 296, 16, 16, line, WHITE); y += 24U;

    snprintf(line, sizeof(line), "SD TOTAL   : %lu MB", (unsigned long)total_mb);
    lcd_show_string(14, y, 296, 16, 16, line, WHITE); y += 24U;

    snprintf(line, sizeof(line), "SD FREE    : %lu MB", (unsigned long)free_mb);
    lcd_show_string(14, y, 296, 16, 16, line, GREEN); y += 24U;

    snprintf(line, sizeof(line), "SHUTTER    : #%05u", (unsigned int)(*ctx->shutter_count));
    lcd_show_string(14, y, 296, 16, 16, line, CYAN); y += 24U;

    snprintf(line, sizeof(line), "BATTERY    : %u.%02uV (%u%%)",
             (unsigned int)(bat_mv / 1000U), (unsigned int)((bat_mv % 1000U) / 10U),
             (unsigned int)bat_pct);
    lcd_show_string(14, y, 296, 16, 16, line, YELLOW); y += 24U;

    snprintf(line, sizeof(line), "CFG: %s", C3M1_CFG_PATH);
    lcd_show_string(14, y, 296, 12, 12, line, MENU_COL_DIM); y += 18U;

    snprintf(line, sizeof(line), "FW : %s (%s)", FW_VERSION_STR, FW_GIT_HASH_STR);
    lcd_show_string(14, y, 296, 12, 12, line, CYAN);

    menu_draw_footer("INFO", CYAN);

    while (1)
    {
        if (!ctx->is_sd_present_fn())
        {
            ctx->sd_removed_in_menu = 1U;
            return;
        }
        uint8_t evts = key_poll_events();
        if (evts & (BTN_EVT_ISO | BTN_EVT_SHOT))
        {
            return;
        }
        delay_ms(10);
    }
}

static uint8_t menu_do_format_sd(c3m1_menu_ctx_t *ctx)
{
    if (ctx->is_low_bat_fn())
    {
        menu_draw_modal_msg("LOW BATTERY!", "CHARGE > 3.25V TO FORMAT", RED);
        delay_ms(1200);
        return 0U;
    }

    if (!menu_confirm_dialog(ctx, "FORMAT SD CARD?", "ALL PHOTOS WILL BE ERASED", RED))
    {
        return 0U;
    }

    menu_draw_modal_msg("FORMATTING SD...", "DO NOT REMOVE CARD", YELLOW);
    menu_draw_footer("BUSY", YELLOW);

    /* Unmount, format with FM_ANY (FAT32 <=32GB / exFAT >=64GB), then remount & restore dirs/cfg */
    f_mount(NULL, "0:", 0);
    MKFS_PARM opt;
    memset(&opt, 0, sizeof(opt));
    opt.fmt = FM_ANY;

    FRESULT fres = f_mkfs("0:", &opt, ctx->work_buf, ctx->work_buf_size);
    if (fres == FR_OK && ctx->remount_sd_fn())
    {
        c3m1_cfg_save(ctx->cfg);
        menu_draw_modal_msg("FORMAT COMPLETE!", "CAYE_CAM & CFG READY", GREEN);
        menu_draw_footer("DONE", GREEN);
        delay_ms(1200);
        return 1U;
    }

    char err_str[32];
    snprintf(err_str, sizeof(err_str), "FATFS ERR = %d", (int)fres);
    menu_draw_modal_msg("FORMAT FAILED!", err_str, RED);
    menu_draw_footer("FAIL", RED);
    delay_ms(1500);
    if (!ctx->remount_sd_fn())
    {
        ctx->sd_removed_in_menu = 1U;
    }
    return 0U;
}

void c3m1_menu_run(c3m1_menu_ctx_t *ctx)
{
    uint8_t sel        = 0U;
    uint8_t scroll_top = 0U;
    uint8_t last_dial  = key_read_shutter_dial();
    uint8_t dirty      = 0U;

    ctx->sd_removed_in_menu = 0U;

    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
    menu_draw_list(ctx, sel, scroll_top);
    menu_draw_footer("CFG OK", GREEN);

    while (1)
    {
        if (!ctx->is_sd_present_fn())
        {
            ctx->sd_removed_in_menu = 1U;
            break;
        }

        uint8_t evts = key_poll_events();
        uint8_t dial = key_read_shutter_dial();

        if (dial != last_dial)
        {
            int8_t d = dial_wrap_delta(last_dial, dial);
            last_dial = dial;
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

        if (evts & BTN_EVT_ISO)
        {
            break; /* Short press ASA -> exit menu */
        }

        if (evts & BTN_EVT_SHOT)
        {
            uint8_t setting_changed = 0U;
            switch (sel)
            {
                case MITEM_MAX_ISO:
                    ctx->cfg->max_auto_iso = (uint8_t)((ctx->cfg->max_auto_iso + 1U) % C3M1_MAX_ISO_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_MIN_SHT:
                    ctx->cfg->min_auto_sht = (uint8_t)((ctx->cfg->min_auto_sht + 1U) % C3M1_MIN_SHT_COUNT);
                    setting_changed = 1U;
                    break;

                case MITEM_FLICKER:
                    ctx->cfg->flicker_mode = (uint8_t)((ctx->cfg->flicker_mode + 1U) % C3M1_FLICKER_COUNT);
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

                case MITEM_FORMAT_SD:
                    menu_do_format_sd(ctx);
                    if (ctx->sd_removed_in_menu) goto menu_exit;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    menu_draw_footer("CFG OK", GREEN);
                    break;

                case MITEM_SD_INFO:
                    menu_show_system_info(ctx);
                    if (ctx->sd_removed_in_menu) goto menu_exit;
                    last_dial = key_read_shutter_dial();
                    menu_draw_header("C3M1 MENU", "DIAL:NAV SHOT:OK ASA:ESC");
                    menu_draw_list(ctx, sel, scroll_top);
                    menu_draw_footer("CFG OK", GREEN);
                    break;

                case MITEM_EXIT:
                    goto menu_exit;

                default:
                    break;
            }

            if (setting_changed)
            {
                dirty = 1U;
                menu_draw_row(ctx, (uint8_t)(sel - scroll_top), sel, 1U);
                if (!ctx->is_low_bat_fn() && c3m1_cfg_save(ctx->cfg))
                {
                    dirty = 0U;
                    menu_draw_footer("SAVED ", YELLOW);
                }
                else
                {
                    menu_draw_footer("SD ERR", RED);
                }
            }
        }

        delay_ms(10);
    }

menu_exit:
    if (dirty && !ctx->sd_removed_in_menu && ctx->is_sd_present_fn() && !ctx->is_low_bat_fn())
    {
        c3m1_cfg_save(ctx->cfg);
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
