/**
 * @file        boot_anim.h
 * @brief       Minimal ~0.5s startup animation for 白玉楼制作所 (320x240 ST7789V3).
 */

#ifndef C3M1_BOOT_ANIM_H_
#define C3M1_BOOT_ANIM_H_

#include <stdint.h>

/**
 * @brief  Play logo fade-in + title reveal (~350ms) and return start tick.
 *         Caller may perform SD/sensor init while the finished splash remains visible,
 *         then call boot_anim_finish() to pad to 500ms total.
 */
uint32_t boot_anim_start(void);

/**
 * @brief  Wait until at least 500ms have elapsed since boot_anim_start(), then clear screen.
 */
void boot_anim_finish(uint32_t start_tick);

#endif /* C3M1_BOOT_ANIM_H_ */
