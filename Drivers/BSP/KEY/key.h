/**
 * @file        key.h
 * @brief       C3M1 top-board button, 16-position BCD shutter dial, film lever,
 *              and mainboard NUMRST input driver.
 */

#ifndef KEY_KEY_H_
#define KEY_KEY_H_

#include "../../SYSTEM/sys/sys.h"

/* Button event bitmask returned by key_poll_events() */
#define BTN_EVT_NONE        0x00U
#define BTN_EVT_ISO         (1U << 0)   /* KEYISO (PF1) pressed */
#define BTN_EVT_SHOT        (1U << 1)   /* KEYSHOT (PF2) pressed */
#define BTN_EVT_4X          (1U << 2)   /* KEY4X (PF3) pressed */
#define BTN_EVT_FILM        (1U << 3)   /* FILMKEY (PF10) lever triggered */
#define BTN_EVT_NUMRST_LONG (1U << 4)   /* NUMRST (PA3) held >= 1.5s */

void    key_init_state(void);
uint8_t key_poll_events(void);
uint8_t key_read_shutter_dial(void);    /* Debounced 0..15 BCD position */

#endif /* KEY_KEY_H_ */
