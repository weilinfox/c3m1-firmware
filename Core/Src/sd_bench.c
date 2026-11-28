#include "sd_bench.h"

#if C3M1_SD_BENCH

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "sdio.h"
#include "c3m1_log.h"
#include "../../BSP/LCD/lcd.h"
#include "../../BSP/KEY/key.h"
#include "../../SYSTEM/delay/delay.h"

#define BENCH_FILE_PATH     "0:/_SDBENCH.TMP"
#define BENCH_CHUNK_BYTES   4096U  /* 8 sectors, matches DNG/JPEG s_sd_io_buf */
#define BENCH_RAW_CHUNKS    64U    /* 256 KB raw multi-block DMA read x2 */
#define BENCH_FAT_CHUNKS    64U    /* 256 KB FatFs multi-block write + read + verify */
#define BENCH_UNALIGN_LEN   4092U  /* Odd-offset (buf + 1) unaligned bounce test */
#define BENCH_UNALIGN_LOOPS 4U     /* 16 KB unaligned write + readback verify */

typedef struct
{
    uint32_t clk_div;
    const char *clk_label;
} bench_speed_t;

static const bench_speed_t s_speeds[] = {
    { 1U, "24.0M" },
    { 2U, "18.0M" },
    { 3U, "14.4M" },
    { 4U, "12.0M" },
    { 6U, " 9.0M" },
};

static uint32_t bench_xorshift(uint32_t s)
{
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

static void bench_fill_pattern(uint8_t *buf, uint32_t len, uint32_t seed)
{
    uint32_t s = seed ? seed : 0xA5A55A5AUL;
    uint32_t i = 0U;
    while (i + 4U <= len)
    {
        s = bench_xorshift(s);
        buf[i + 0U] = (uint8_t)(s & 0xFFU);
        buf[i + 1U] = (uint8_t)((s >> 8) & 0xFFU);
        buf[i + 2U] = (uint8_t)((s >> 16) & 0xFFU);
        buf[i + 3U] = (uint8_t)((s >> 24) & 0xFFU);
        i += 4U;
    }
    while (i < len)
    {
        s = bench_xorshift(s);
        buf[i++] = (uint8_t)(s & 0xFFU);
    }
}

static uint32_t bench_verify_pattern(const uint8_t *buf, uint32_t len, uint32_t seed)
{
    uint32_t s = seed ? seed : 0xA5A55A5AUL;
    uint32_t i = 0U;
    while (i + 4U <= len)
    {
        s = bench_xorshift(s);
        if (buf[i + 0U] != (uint8_t)(s & 0xFFU))         return i + 1U;
        if (buf[i + 1U] != (uint8_t)((s >> 8) & 0xFFU))  return i + 2U;
        if (buf[i + 2U] != (uint8_t)((s >> 16) & 0xFFU)) return i + 3U;
        if (buf[i + 3U] != (uint8_t)((s >> 24) & 0xFFU)) return i + 4U;
        i += 4U;
    }
    while (i < len)
    {
        s = bench_xorshift(s);
        if (buf[i] != (uint8_t)(s & 0xFFU)) return i + 1U;
        i++;
    }
    return 0U; /* 0 = exact match */
}

static uint32_t bench_fnv1a(const uint8_t *buf, uint32_t len, uint32_t h)
{
    for (uint32_t i = 0U; i < len; i++)
    {
        h ^= (uint32_t)buf[i];
        h *= 16777619UL;
    }
    return h;
}

static void bench_fmt_mbs(char *out, size_t out_sz, uint32_t bytes, uint32_t ms)
{
    if (ms == 0U)
    {
        ms = 1U;
    }
    /* MB/s * 100 using integer math (1 MB = 1,048,576 B) */
    uint32_t kb_s   = (uint32_t)(((uint64_t)bytes * 1000ULL) / ((uint64_t)ms * 1024ULL));
    uint32_t mb_x100 = (kb_s * 100U) / 1024U;
    snprintf(out, out_sz, "%u.%02uM", (unsigned int)(mb_x100 / 100U), (unsigned int)(mb_x100 % 100U));
}

static void bench_decode_err(uint32_t err, char *out, size_t out_sz)
{
    char flags[36];
    flags[0] = '\0';
    if (err & 0x80000000UL) strncat(flags, "TMO ", sizeof(flags) - strlen(flags) - 1U);
    if (err & 0x40000000UL) strncat(flags, "NOCD ", sizeof(flags) - strlen(flags) - 1U);
    if (err & 0x20000000UL) strncat(flags, "BUSY ", sizeof(flags) - strlen(flags) - 1U);
    if (err & HAL_SD_ERROR_DATA_CRC_FAIL) strncat(flags, "DCRC ", sizeof(flags) - strlen(flags) - 1U);
    if (err & HAL_SD_ERROR_CMD_CRC_FAIL)  strncat(flags, "CCRC ", sizeof(flags) - strlen(flags) - 1U);
    if (err & HAL_SD_ERROR_DATA_TIMEOUT)  strncat(flags, "DTO ", sizeof(flags) - strlen(flags) - 1U);
    if (err & HAL_SD_ERROR_CMD_RSP_TIMEOUT) strncat(flags, "CTO ", sizeof(flags) - strlen(flags) - 1U);
    if (err & HAL_SD_ERROR_TX_UNDERRUN)   strncat(flags, "TXU ", sizeof(flags) - strlen(flags) - 1U);
    if (err & HAL_SD_ERROR_RX_OVERRUN)    strncat(flags, "RXO ", sizeof(flags) - strlen(flags) - 1U);
    if (err & HAL_SD_ERROR_DMA)           strncat(flags, "DMA ", sizeof(flags) - strlen(flags) - 1U);
    if (flags[0] == '\0')
    {
        snprintf(out, out_sz, "0x%08lX", (unsigned long)err);
    }
    else
    {
        snprintf(out, out_sz, "0x%08lX %s", (unsigned long)err, flags);
    }
}

static void bench_draw_header(void)
{
    g_back_color = BLACK;
    lcd_clear(BLACK);
    lcd_fill(0, 0, 319, 21, DARKBLUE);
    g_back_color = DARKBLUE;
    lcd_show_string(6, 5, 310, 12, 12, "SDIO 4-BIT DMA2 SPEED BENCH (HWFC=OFF)", CYAN);
    g_back_color = BLACK;
    lcd_draw_hline(0, 42, 320, GRAY);
    lcd_show_string(4,   48, 44, 12, 12, "CLK",    YELLOW);
    lcd_show_string(50,  48, 60, 12, 12, "RAW-R",  YELLOW);
    lcd_show_string(114, 48, 60, 12, 12, "FAT-W",  YELLOW);
    lcd_show_string(178, 48, 60, 12, 12, "FAT-R",  YELLOW);
    lcd_show_string(242, 48, 76, 12, 12, "RESULT", YELLOW);
    lcd_draw_hline(0, 64, 320, GRAY);
    lcd_draw_hline(0, 182, 320, GRAY);
    lcd_fill(0, 222, 319, 239, DARKBLUE);
    g_back_color = DARKBLUE;
    lcd_show_string(6, 225, 310, 12, 12, "[SHOT] RETEST (SWAP OK)   [4X] CAMERA", WHITE);
    g_back_color = BLACK;
}

static void bench_run_once(FATFS *fs_ptr, FIL *fil_ptr, uint8_t *io_buf_4k)
{
    bench_draw_header();

    if (!sd_card_is_present())
    {
        lcd_show_string(6, 26, 308, 12, 12, "NO SD CARD! INSERT CARD & PRESS [SHOT]", RED);
        return;
    }

    /* Probe card identity once at conservative 9 MHz for header info */
    uint8_t info_rc = sdio_init_with_div(6U);
    if (info_rc == 0U)
    {
        HAL_SD_CardInfoTypeDef ci;
        memset(&ci, 0, sizeof(ci));
        (void)get_sd_card_info(&ci);
        uint32_t mb = (uint32_t)(((uint64_t)ci.LogBlockNbr * (uint64_t)ci.LogBlockSize) >> 20);
        const char *ctype = (ci.CardType == CARD_SDHC_SDXC) ? "SDHC/XC" : "SDSC";
        char line[56];
        snprintf(line, sizeof(line), "CARD: %s  %luMB  256KB R/W + UNALIGN",
                 ctype, (unsigned long)mb);
        lcd_show_string(6, 26, 308, 12, 12, line, WHITE);
    }
    else
    {
        lcd_show_string(6, 26, 308, 12, 12, "CARD: INIT @9M FAILED (TESTING ALL DIVS)", ORANGE);
    }

    uint8_t  pass_cnt      = 0U;
    uint8_t  any_fail      = 0U;
    uint8_t  file_created  = 0U;
    char     detail1[54]   = "ALL 5 SPEEDS PASSED! (0 ERRORS)";
    char     detail2[54]   = "SAFE TO SWAP CARD AND PRESS [SHOT]";
    uint16_t detail_color  = GREEN;

    for (uint8_t idx = 0U; idx < (uint8_t)(sizeof(s_speeds) / sizeof(s_speeds[0])); idx++)
    {
        uint16_t y = (uint16_t)(72U + idx * 21U);
        uint32_t div = s_speeds[idx].clk_div;
        const char *lbl = s_speeds[idx].clk_label;

        lcd_fill(0, y, 319, y + 15U, BLACK);
        lcd_show_string(4,   y, 44, 12, 12, lbl,      WHITE);
        lcd_show_string(50,  y, 60, 12, 12, "...",    GRAY);
        lcd_show_string(114, y, 60, 12, 12, "...",    GRAY);
        lcd_show_string(178, y, 60, 12, 12, "...",    GRAY);
        lcd_show_string(242, y, 76, 12, 12, "RUN...", YELLOW);

        /* 1. Re-init SDIO at target divider */
        uint8_t irc = sdio_init_with_div(div);
        if (irc != 0U)
        {
            uint32_t err = sdio_get_last_error();
            char dec[44];
            bench_decode_err(err, dec, sizeof(dec));
            lcd_show_string(242, y, 76, 12, 12, "FAIL:INIT", RED);
            if (!any_fail)
            {
                any_fail = 1U;
                detail_color = ORANGE;
                snprintf(detail1, sizeof(detail1), "FIRST FAIL @%s: INIT rc=%u", lbl, (unsigned int)irc);
                snprintf(detail2, sizeof(detail2), "ERR: %s", dec);
            }
            continue;
        }

        /* 2. Raw multi-block DMA read (256 KB x2 + FNV-1a verify, read-only LBA 0..511) */
        uint32_t t0 = HAL_GetTick();
        uint32_t h1 = 2166136261UL;
        uint8_t  raw_ok = 1U;
        for (uint32_t c = 0U; c < BENCH_RAW_CHUNKS; c++)
        {
            if (sd_read_disk(io_buf_4k, c * 8U, 8U) != HAL_OK)
            {
                raw_ok = 0U;
                break;
            }
            h1 = bench_fnv1a(io_buf_4k, BENCH_CHUNK_BYTES, h1);
        }
        uint32_t raw_ms = HAL_GetTick() - t0;

        if (raw_ok)
        {
            uint32_t h2 = 2166136261UL;
            for (uint32_t c = 0U; c < BENCH_RAW_CHUNKS; c++)
            {
                if (sd_read_disk(io_buf_4k, c * 8U, 8U) != HAL_OK)
                {
                    raw_ok = 0U;
                    break;
                }
                h2 = bench_fnv1a(io_buf_4k, BENCH_CHUNK_BYTES, h2);
            }
            if (raw_ok && h1 != h2)
            {
                raw_ok = 2U; /* CRC passed silently? hash mismatch */
            }
        }

        if (raw_ok != 1U)
        {
            uint32_t err = sdio_get_last_error();
            char dec[44];
            bench_decode_err(err, dec, sizeof(dec));
            lcd_show_string(50,  y, 60, 12, 12, "ERR", RED);
            lcd_show_string(242, y, 76, 12, 12, (raw_ok == 2U) ? "FAIL:HASH" : "FAIL:RAWR", RED);
            if (!any_fail)
            {
                any_fail = 1U;
                detail_color = ORANGE;
                snprintf(detail1, sizeof(detail1), "FIRST FAIL @%s: %s",
                         lbl, (raw_ok == 2U) ? "RAW HASH MISMATCH" : "RAW DMA READ");
                snprintf(detail2, sizeof(detail2), "ERR: %s", dec);
            }
            continue;
        }

        char s_raw[12];
        bench_fmt_mbs(s_raw, sizeof(s_raw), BENCH_RAW_CHUNKS * BENCH_CHUNK_BYTES, raw_ms);
        lcd_show_string(50, y, 60, 12, 12, s_raw, WHITE);

        /* 3. FatFs mount + 256 KB multi-block write + readback + byte verify + unaligned test */
        FRESULT fr = f_mount(fs_ptr, "0:", 1);
        if (fr != FR_OK)
        {
            uint32_t err = sdio_get_last_error();
            char dec[44];
            bench_decode_err(err, dec, sizeof(dec));
            lcd_show_string(242, y, 76, 12, 12, "FAIL:MNT", RED);
            if (!any_fail)
            {
                any_fail = 1U;
                detail_color = ORANGE;
                snprintf(detail1, sizeof(detail1), "FIRST FAIL @%s: f_mount fr=%d", lbl, (int)fr);
                snprintf(detail2, sizeof(detail2), "ERR: %s", dec);
            }
            f_mount(NULL, "0:", 0);
            continue;
        }

        /* 3a. Write 256 KB */
        fr = f_open(fil_ptr, BENCH_FILE_PATH, FA_CREATE_ALWAYS | FA_WRITE);
        uint32_t w_ms = 0U;
        if (fr == FR_OK)
        {
            file_created = 1U;
            uint32_t tw0 = HAL_GetTick();
            for (uint32_t c = 0U; c < BENCH_FAT_CHUNKS; c++)
            {
                UINT bw = 0U;
                uint32_t seed = 0xC3000000UL | (div << 16) | (c + 1U);
                bench_fill_pattern(io_buf_4k, BENCH_CHUNK_BYTES, seed);
                fr = f_write(fil_ptr, io_buf_4k, BENCH_CHUNK_BYTES, &bw);
                if (fr != FR_OK || bw != BENCH_CHUNK_BYTES)
                {
                    if (fr == FR_OK) fr = FR_DISK_ERR;
                    break;
                }
            }
            if (fr == FR_OK)
            {
                fr = f_close(fil_ptr);
            }
            else
            {
                (void)f_close(fil_ptr);
            }
            w_ms = HAL_GetTick() - tw0;
        }

        if (fr != FR_OK)
        {
            uint32_t err = sdio_get_last_error();
            char dec[44];
            bench_decode_err(err, dec, sizeof(dec));
            lcd_show_string(114, y, 60, 12, 12, "ERR", RED);
            lcd_show_string(242, y, 76, 12, 12, "FAIL:WR", RED);
            if (!any_fail)
            {
                any_fail = 1U;
                detail_color = ORANGE;
                snprintf(detail1, sizeof(detail1), "FIRST FAIL @%s: FAT WRITE fr=%d", lbl, (int)fr);
                snprintf(detail2, sizeof(detail2), "ERR: %s", dec);
            }
            f_mount(NULL, "0:", 0);
            continue;
        }

        char s_wr[12];
        bench_fmt_mbs(s_wr, sizeof(s_wr), BENCH_FAT_CHUNKS * BENCH_CHUNK_BYTES, w_ms);
        lcd_show_string(114, y, 60, 12, 12, s_wr, WHITE);

        /* 3b. Read back 256 KB & byte-for-byte verify */
        uint32_t r_ms = 0U;
        uint32_t bad_pos = 0U;
        uint32_t bad_chunk = 0U;
        fr = f_open(fil_ptr, BENCH_FILE_PATH, FA_READ);
        if (fr == FR_OK)
        {
            uint32_t tr0 = HAL_GetTick();
            for (uint32_t c = 0U; c < BENCH_FAT_CHUNKS; c++)
            {
                UINT br = 0U;
                fr = f_read(fil_ptr, io_buf_4k, BENCH_CHUNK_BYTES, &br);
                if (fr != FR_OK || br != BENCH_CHUNK_BYTES)
                {
                    if (fr == FR_OK) fr = FR_DISK_ERR;
                    break;
                }
                uint32_t seed = 0xC3000000UL | (div << 16) | (c + 1U);
                bad_pos = bench_verify_pattern(io_buf_4k, BENCH_CHUNK_BYTES, seed);
                if (bad_pos != 0U)
                {
                    bad_chunk = c;
                    break;
                }
            }
            (void)f_close(fil_ptr);
            r_ms = HAL_GetTick() - tr0;
        }

        if (fr != FR_OK || bad_pos != 0U)
        {
            uint32_t err = sdio_get_last_error();
            char dec[44];
            bench_decode_err(err, dec, sizeof(dec));
            lcd_show_string(178, y, 60, 12, 12, "ERR", RED);
            lcd_show_string(242, y, 76, 12, 12, (bad_pos != 0U) ? "FAIL:VER" : "FAIL:RD", RED);
            if (!any_fail)
            {
                any_fail = 1U;
                detail_color = ORANGE;
                if (bad_pos != 0U)
                {
                    snprintf(detail1, sizeof(detail1), "FIRST FAIL @%s: VERIFY blk=%lu +%lu",
                             lbl, (unsigned long)bad_chunk, (unsigned long)(bad_pos - 1U));
                }
                else
                {
                    snprintf(detail1, sizeof(detail1), "FIRST FAIL @%s: FAT READ fr=%d", lbl, (int)fr);
                }
                snprintf(detail2, sizeof(detail2), "ERR: %s", dec);
            }
            f_mount(NULL, "0:", 0);
            continue;
        }

        char s_rd[12];
        bench_fmt_mbs(s_rd, sizeof(s_rd), BENCH_FAT_CHUNKS * BENCH_CHUNK_BYTES, r_ms);
        lcd_show_string(178, y, 60, 12, 12, s_rd, WHITE);

        /* 3c. Unaligned buffer (buf + 1) 16 KB write + readback verify */
        uint8_t *unaligned = io_buf_4k + 1U;
        fr = f_open(fil_ptr, BENCH_FILE_PATH, FA_CREATE_ALWAYS | FA_WRITE);
        if (fr == FR_OK)
        {
            for (uint32_t u = 0U; u < BENCH_UNALIGN_LOOPS; u++)
            {
                UINT bw = 0U;
                bench_fill_pattern(unaligned, BENCH_UNALIGN_LEN, 0x55AA0000UL | (div << 8) | u);
                fr = f_write(fil_ptr, unaligned, BENCH_UNALIGN_LEN, &bw);
                if (fr != FR_OK || bw != BENCH_UNALIGN_LEN)
                {
                    if (fr == FR_OK) fr = FR_DISK_ERR;
                    break;
                }
            }
            if (fr == FR_OK) fr = f_close(fil_ptr);
            else (void)f_close(fil_ptr);
        }
        if (fr == FR_OK)
        {
            fr = f_open(fil_ptr, BENCH_FILE_PATH, FA_READ);
            if (fr == FR_OK)
            {
                for (uint32_t u = 0U; u < BENCH_UNALIGN_LOOPS; u++)
                {
                    UINT br = 0U;
                    fr = f_read(fil_ptr, unaligned, BENCH_UNALIGN_LEN, &br);
                    if (fr != FR_OK || br != BENCH_UNALIGN_LEN)
                    {
                        if (fr == FR_OK) fr = FR_DISK_ERR;
                        break;
                    }
                    if (bench_verify_pattern(unaligned, BENCH_UNALIGN_LEN,
                                             0x55AA0000UL | (div << 8) | u) != 0U)
                    {
                        bad_pos = 1U;
                        break;
                    }
                }
                (void)f_close(fil_ptr);
            }
        }

        if (f_unlink(BENCH_FILE_PATH) == FR_OK)
        {
            file_created = 0U;
        }
        f_mount(NULL, "0:", 0);

        if (fr != FR_OK || bad_pos != 0U)
        {
            uint32_t err = sdio_get_last_error();
            char dec[44];
            bench_decode_err(err, dec, sizeof(dec));
            lcd_show_string(242, y, 76, 12, 12, "FAIL:UAL", RED);
            if (!any_fail)
            {
                any_fail = 1U;
                detail_color = ORANGE;
                snprintf(detail1, sizeof(detail1), "FIRST FAIL @%s: UNALIGN fr=%d", lbl, (int)fr);
                snprintf(detail2, sizeof(detail2), "ERR: %s", dec);
            }
            continue;
        }

        pass_cnt++;
        lcd_show_string(242, y, 76, 12, 12, "PASS", GREEN);
        C3M1_LOG("BENCH %s: RAW=%s W=%s R=%s PASS\r\n", lbl, s_raw, s_wr, s_rd);
    }

    /* Final cleanup if a high-speed fail left _SDBENCH.TMP behind */
    if (file_created && sdio_init_with_div(6U) == 0U)
    {
        if (f_mount(fs_ptr, "0:", 1) == FR_OK)
        {
            (void)f_unlink(BENCH_FILE_PATH);
            f_mount(NULL, "0:", 0);
        }
    }

    /* DeInit SDIO so user can safely hot-swap SDHC / SDXC cards right on this screen */
    MX_SDIO_SD_DeInit();

    if (pass_cnt == 0U)
    {
        detail_color = RED;
    }
    lcd_show_string(6, 188, 308, 12, 12, detail1, detail_color);
    lcd_show_string(6, 204, 308, 12, 12, detail2, WHITE);
}

void sd_bench_run_ui(FATFS *fs_ptr, FIL *fil_ptr, uint8_t *io_buf_4k)
{
    /* Unmount any boot-mounted volume first */
    f_mount(NULL, "0:", 0);
    MX_SDIO_SD_DeInit();

    for (;;)
    {
        bench_run_once(fs_ptr, fil_ptr, io_buf_4k);
        key_init_state();

        for (;;)
        {
            uint8_t ev = key_poll_events();
            if (ev & BTN_EVT_SHOT)
            {
                delay_ms(120);
                break; /* Re-run full sweep (e.g. after card swap) */
            }
            if (ev & (BTN_EVT_4X | BTN_EVT_4X_LONG))
            {
                lcd_clear(BLACK);
                return; /* Exit to normal camera viewfinder */
            }
            delay_ms(20);
        }
    }
}

#endif /* C3M1_SD_BENCH */
