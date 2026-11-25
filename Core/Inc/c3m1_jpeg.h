/**
 * @file        c3m1_jpeg.h
 * @brief       Streaming pure-integer Baseline JPEG encoder (640x480 YCbCr 4:2:0, Q~90)
 *              with 3x3 bilinear GRBG Bayer demosaic, 12-bit linearization, per-channel
 *              BlackLevel subtraction, Gray-World AWB, and calibrated sRGB color matrix.
 */

#ifndef C3M1_JPEG_H_
#define C3M1_JPEG_H_

#include <stdint.h>

typedef struct
{
    const uint16_t *inv_gamma_12;   /* 256-entry 12-bit LinearizationTable (0..4064) */
    const uint8_t  *srgb_lut;       /* 256-entry IEC 61966-2-1 sRGB OETF table */
    uint16_t        bl_g;           /* 12-bit Green BlackLevel */
    uint16_t        bl_rb;          /* 12-bit Red/Blue BlackLevel */
    uint16_t        wb_r_q8;        /* Gray-World R gain (Q8, 256 = 1.0) */
    uint16_t        wb_b_q8;        /* Gray-World B gain (Q8, 256 = 1.0) */
    uint8_t        *out_buf;        /* Caller-owned I/O buffer (e.g. 4096 B) */
    uint16_t        out_buf_size;
    void          (*read_row_fn)(void *user, uint8_t *row644);
    uint8_t       (*write_fn)(void *user, const uint8_t *data, uint32_t len);
    void           *user;
} c3m1_jpeg_ctx_t;

/**
 * @brief  Convert 12-bit linear camera RGB (post-BlackLevel) to calibrated 8-bit sRGB.
 *         Shared by DNG 160x120 embedded preview thumbnail and VGA JPEG encoder.
 */
void c3m1_color_cam12_to_srgb8(uint16_t r12, uint16_t g12, uint16_t b12,
                               uint16_t wb_r_q8, uint16_t wb_b_q8,
                               const uint8_t srgb_lut[256],
                               uint8_t *out_r, uint8_t *out_g, uint8_t *out_b);

/**
 * @brief  Encode a 644x480 GRBG 10-bit Gamma-companded Bayer frame into a 640x480 Baseline JPEG.
 * @return 1 on success, 0 on write error.
 */
uint8_t c3m1_jpeg_encode_bayer_vga(const c3m1_jpeg_ctx_t *ctx, uint32_t *out_total_bytes);

#endif /* C3M1_JPEG_H_ */
