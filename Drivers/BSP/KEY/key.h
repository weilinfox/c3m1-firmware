/**
 * @file        key.h
 * @brief       C3M1 top-board button, 16-position BCD shutter dial, and film lever driver.
 */

#ifndef KEY_KEY_H_
#define KEY_KEY_H_

#include "../../SYSTEM/sys/sys.h"

/* Button event bitmask returned by key_poll_events() */
#define BTN_EVT_NONE        0x00U
#define BTN_EVT_ISO         (1U << 0)   /* KEYISO (PF1) short press (released before 600ms) */
#define BTN_EVT_SHOT        (1U << 1)   /* KEYSHOT (PF2) pressed (immediate for 0ms lag) */
#define BTN_EVT_4X          (1U << 2)   /* KEY4X (PF3) short press (released before 600ms) */
#define BTN_EVT_FILM        (1U << 3)   /* FILMKEY (PF10) short press (released before 600ms) */
#define BTN_EVT_ISO_LONG    (1U << 4)   /* KEYISO (PF1) held >= 600ms (manual sleep & save clock) */
#define BTN_EVT_FILM_LONG   (1U << 5)   /* FILMKEY (PF10) held >= 600ms (open playback) */
#define BTN_EVT_4X_LONG     (1U << 6)   /* KEY4X (PF3) held >= 600ms (open menu) */

void    key_init_state(void);
uint8_t key_poll_events(void);
uint8_t key_read_shutter_dial(void);    /* Debounced 0..15 BCD position */
void    key_dial_reset_debounce(void);  /* Reset dial debounce window after blocking ops */
uint8_t key_any_top_button_pressed(void); /* Non-zero while ISO/SHOT/4X/FILM is physically held */
uint8_t key_is_shot_pressed(void);        /* Non-zero while KEYSHOT (PF2) is physically held */

#endif /* KEY_KEY_H_ */
