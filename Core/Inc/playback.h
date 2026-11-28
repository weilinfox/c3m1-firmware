/**
 * @file        playback.h
 * @brief       On-camera photo playback (1X full-frame, 1:1 zoom & pan, paired DNG+JPG delete).
 */

#ifndef PLAYBACK_H_
#define PLAYBACK_H_

#include <stdint.h>

typedef struct
{
    const char     *photo_dir;          /* e.g. "0:CAYE_CAM" */
    const uint8_t  *srgb_lut;           /* 256-entry IEC 61966-2-1 sRGB OETF table */
    uint8_t        *work_buf;           /* Caller-owned 4 KB buffer (s_sd_io_buf) */
    uint16_t        work_buf_size;
    uint8_t       (*bar_buf)[160];      /* Caller-owned 12x160 4-bit bottom HUD buffer (s_osd_bar_buf) */
    uint8_t       (*is_low_bat_fn)(void);
    uint8_t       (*is_sd_present_fn)(void);
    uint8_t         skip_hdr_brk;       /* Input: 1 to skip _0/_1/_2 when _hdr exists */
    uint8_t         high_iso_nr;        /* Input: 1 to apply ISO-adaptive high-ISO NR on DNG playback */
    uint8_t         sd_removed_in_pb;   /* Output: 1 if SD card was pulled during playback */
    uint8_t         woke_from_sleep;    /* Output: 1 if playback exited via low-bat or auto-sleep wake */
} c3m1_pb_ctx_t;

/**
 * @brief  Run modal photo playback loop starting from the newest photo.
 *         Controls:
 *           1X view:   DIAL = prev/next, 4X = zoom 1:1, SHOT = cycle CLEAN/CLIP/INFO,
 *                      FILM lever = delete confirm, ASA = exit playback
 *           Zoom view: 4X = back to 1X, DIAL = pan up/down, ASA = pan left, SHOT = pan right,
 *                      FILM lever = delete confirm
 */
void c3m1_pb_run(c3m1_pb_ctx_t *ctx);

#endif /* PLAYBACK_H_ */
