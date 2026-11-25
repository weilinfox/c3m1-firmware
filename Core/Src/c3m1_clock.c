/**
 * @file        c3m1_clock.c
 * @brief       Software real-time clock anchored to 2000-01-01 00:00:00 (epoch = 0).
 */

#include "c3m1_clock.h"
#include "fw_version.h"
#include "main.h"

static uint32_t s_base_epoch = 0UL;
static uint32_t s_base_tick  = 0UL;
static uint16_t s_sub_ms     = 0U;
static uint8_t  s_inited     = 0U;

static uint8_t is_leap_year(uint16_t year)
{
    return (((year % 4U) == 0U && (year % 100U) != 0U) || ((year % 400U) == 0U)) ? 1U : 0U;
}

uint8_t c3m1_clock_days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t s_mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if (month < 1U || month > 12U) return 31U;
    if (month == 2U && is_leap_year(year)) return 29U;
    return s_mdays[month - 1U];
}

uint32_t c3m1_clock_to_epoch(const c3m1_datetime_t *dt)
{
    uint16_t year  = dt->year;
    uint8_t  month = dt->month;
    uint8_t  day   = dt->day;
    uint8_t  hour  = dt->hour;
    uint8_t  min   = dt->min;
    uint8_t  sec   = dt->sec;

    if (year < C3M1_CLOCK_YEAR_MIN) year = C3M1_CLOCK_YEAR_MIN;
    if (year > C3M1_CLOCK_YEAR_MAX) year = C3M1_CLOCK_YEAR_MAX;
    if (month < 1U)  month = 1U;
    if (month > 12U) month = 12U;
    uint8_t max_d = c3m1_clock_days_in_month(year, month);
    if (day < 1U)    day = 1U;
    if (day > max_d) day = max_d;
    if (hour > 23U)  hour = 23U;
    if (min  > 59U)  min  = 59U;
    if (sec  > 59U)  sec  = 59U;

    uint32_t days = 0UL;
    for (uint16_t y = C3M1_CLOCK_YEAR_MIN; y < year; y++)
    {
        days += is_leap_year(y) ? 366UL : 365UL;
    }
    for (uint8_t m = 1U; m < month; m++)
    {
        days += c3m1_clock_days_in_month(year, m);
    }
    days += (uint32_t)(day - 1U);

    return days * 86400UL + (uint32_t)hour * 3600UL + (uint32_t)min * 60UL + (uint32_t)sec;
}

void c3m1_clock_from_epoch(uint32_t epoch, c3m1_datetime_t *dt)
{
    uint32_t days = epoch / 86400UL;
    uint32_t rem  = epoch % 86400UL;

    dt->hour = (uint8_t)(rem / 3600UL);
    rem %= 3600UL;
    dt->min  = (uint8_t)(rem / 60UL);
    dt->sec  = (uint8_t)(rem % 60UL);

    uint16_t year = C3M1_CLOCK_YEAR_MIN;
    while (year < C3M1_CLOCK_YEAR_MAX)
    {
        uint32_t yd = is_leap_year(year) ? 366UL : 365UL;
        if (days < yd) break;
        days -= yd;
        year++;
    }

    uint8_t month = 1U;
    while (month < 12U)
    {
        uint32_t md = c3m1_clock_days_in_month(year, month);
        if (days < md) break;
        days -= md;
        month++;
    }

    dt->year  = year;
    dt->month = month;
    dt->day   = (uint8_t)(days + 1UL);
}

uint32_t c3m1_clock_get_build_epoch(void)
{
    c3m1_datetime_t dt = {
        .year  = (uint16_t)FW_BUILD_YEAR,
        .month = (uint8_t)FW_BUILD_MON,
        .day   = (uint8_t)FW_BUILD_DAY,
        .hour  = (uint8_t)FW_BUILD_HOUR,
        .min   = (uint8_t)FW_BUILD_MIN,
        .sec   = (uint8_t)FW_BUILD_SEC,
    };
    return c3m1_clock_to_epoch(&dt);
}

static void clock_sync_now(void)
{
    if (!s_inited)
    {
        s_base_epoch = c3m1_clock_get_build_epoch();
        s_base_tick  = HAL_GetTick();
        s_sub_ms     = 0U;
        s_inited     = 1U;
        return;
    }

    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = (uint32_t)(now - s_base_tick);
    if (elapsed > 0UL)
    {
        uint32_t total_ms = (uint32_t)s_sub_ms + elapsed;
        s_base_epoch += total_ms / 1000UL;
        s_sub_ms      = (uint16_t)(total_ms % 1000UL);
        s_base_tick   = now;
    }
}

void c3m1_clock_set_epoch(uint32_t epoch)
{
    s_base_epoch = epoch;
    s_base_tick  = HAL_GetTick();
    s_sub_ms     = 0U;
    s_inited     = 1U;
}

uint32_t c3m1_clock_get_epoch(void)
{
    clock_sync_now();
    return s_base_epoch;
}

void c3m1_clock_set_datetime(const c3m1_datetime_t *dt)
{
    c3m1_clock_set_epoch(c3m1_clock_to_epoch(dt));
}

void c3m1_clock_get_datetime(c3m1_datetime_t *dt)
{
    c3m1_clock_from_epoch(c3m1_clock_get_epoch(), dt);
}

void c3m1_clock_advance_sec(uint32_t secs)
{
    if (!s_inited)
    {
        s_base_epoch = c3m1_clock_get_build_epoch();
        s_inited     = 1U;
    }
    s_base_epoch += secs;
}

void c3m1_clock_resync_tick(void)
{
    s_base_tick = HAL_GetTick();
}

uint32_t c3m1_clock_get_fattime(void)
{
    c3m1_datetime_t dt;
    c3m1_clock_get_datetime(&dt);
    uint32_t y = (dt.year >= 1980U) ? (uint32_t)(dt.year - 1980U) : 20UL;
    return (y << 25)
         | ((uint32_t)dt.month << 21)
         | ((uint32_t)dt.day   << 16)
         | ((uint32_t)dt.hour  << 11)
         | ((uint32_t)dt.min   << 5)
         | ((uint32_t)dt.sec   >> 1);
}

void c3m1_clock_format_exif(char out[20])
{
    c3m1_datetime_t dt;
    c3m1_clock_get_datetime(&dt);
    uint16_t y = (dt.year <= 9999U) ? dt.year : 2000U;
    out[0]  = (char)('0' + (y / 1000U));
    out[1]  = (char)('0' + ((y / 100U) % 10U));
    out[2]  = (char)('0' + ((y / 10U) % 10U));
    out[3]  = (char)('0' + (y % 10U));
    out[4]  = ':';
    out[5]  = (char)('0' + ((dt.month / 10U) % 10U));
    out[6]  = (char)('0' + (dt.month % 10U));
    out[7]  = ':';
    out[8]  = (char)('0' + ((dt.day / 10U) % 10U));
    out[9]  = (char)('0' + (dt.day % 10U));
    out[10] = ' ';
    out[11] = (char)('0' + ((dt.hour / 10U) % 10U));
    out[12] = (char)('0' + (dt.hour % 10U));
    out[13] = ':';
    out[14] = (char)('0' + ((dt.min / 10U) % 10U));
    out[15] = (char)('0' + (dt.min % 10U));
    out[16] = ':';
    out[17] = (char)('0' + ((dt.sec / 10U) % 10U));
    out[18] = (char)('0' + (dt.sec % 10U));
    out[19] = '\0';
}
