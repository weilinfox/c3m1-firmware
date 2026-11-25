/**
 * @file        c3m1_clock.h
 * @brief       Software real-time clock (2000..2099) seeded from firmware build time
 *              or SD-persisted epoch, kept running across active use and auto-sleep.
 */

#ifndef C3M1_CLOCK_H_
#define C3M1_CLOCK_H_

#include <stdint.h>

#define C3M1_CLOCK_YEAR_MIN  2000U
#define C3M1_CLOCK_YEAR_MAX  2099U

typedef struct
{
    uint16_t year;   /* 2000..2099 */
    uint8_t  month;  /* 1..12 */
    uint8_t  day;    /* 1..31 */
    uint8_t  hour;   /* 0..23 */
    uint8_t  min;    /* 0..59 */
    uint8_t  sec;    /* 0..59 */
} c3m1_datetime_t;

uint8_t  c3m1_clock_days_in_month(uint16_t year, uint8_t month);
uint32_t c3m1_clock_to_epoch(const c3m1_datetime_t *dt);
void     c3m1_clock_from_epoch(uint32_t epoch, c3m1_datetime_t *dt);

uint32_t c3m1_clock_get_build_epoch(void);
void     c3m1_clock_set_epoch(uint32_t epoch);
uint32_t c3m1_clock_get_epoch(void);

void     c3m1_clock_set_datetime(const c3m1_datetime_t *dt);
void     c3m1_clock_get_datetime(c3m1_datetime_t *dt);

/* Keep clock ticking accurately across low-power sleep while SysTick is suspended */
void     c3m1_clock_advance_sec(uint32_t secs);
void     c3m1_clock_resync_tick(void);

/* Output formatters for FatFs get_fattime() and DNG/JPEG EXIF DateTime ("YYYY:MM:DD HH:MM:SS") */
uint32_t c3m1_clock_get_fattime(void);
void     c3m1_clock_format_exif(char out[20]);

#endif /* C3M1_CLOCK_H_ */
