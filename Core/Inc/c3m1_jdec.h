/**
 * @file        c3m1_jdec.h
 * @brief       Pure-integer streaming Baseline DCT JPEG decoder (SOF0, 4:2:0/4:2:2/4:4:4/Gray)
 *              with ROI fast-skip and early-exit for STM32F103 playback & 1:1 zoom.
 */

#ifndef C3M1_JDEC_H_
#define C3M1_JDEC_H_

#include <stdint.h>

typedef uint32_t (*c3m1_jdec_read_fn)(void *user_ctx, uint8_t *dst, uint32_t max_len);
typedef void     (*c3m1_jdec_mcu_cb)(void *user_ctx,
                                     uint16_t mcu_x, uint16_t mcu_y,
                                     uint8_t mcu_w, uint8_t mcu_h,
                                     const uint16_t *rgb565);

typedef struct
{
    int32_t  maxcode[17];   /* Max canonical code + 1 for bit length 1..16 (-1 if none) */
    int32_t  delta[17];     /* valptr[len] - mincode[len] */
    uint8_t  fast_sym[256]; /* 8-bit fast lookup symbol */
    uint8_t  fast_len[256]; /* 8-bit fast lookup bit length (0 = slow path) */
    uint8_t  huffval[256];
    uint8_t  valid;
} c3m1_jhuff_t;

typedef struct
{
    uint8_t  id;
    uint8_t  hsamp;
    uint8_t  vsamp;
    uint8_t  qt_idx;
    uint8_t  dc_idx;
    uint8_t  ac_idx;
    int16_t  dc_pred;
} c3m1_jcomp_t;

typedef struct
{
    /* Image geometry */
    uint16_t width;
    uint16_t height;
    uint8_t  ncomp;
    uint8_t  max_hsamp;
    uint8_t  max_vsamp;
    uint8_t  mcu_w;
    uint8_t  mcu_h;
    uint16_t restart_interval;

    /* Stream reader */
    c3m1_jdec_read_fn read_fn;
    void             *read_ctx;
    uint8_t           inbuf[512];
    uint16_t          in_pos;
    uint16_t          in_len;
    uint8_t           eof;
    uint8_t           marker_pending; /* Non-zero if a marker byte was encountered inside ECS */
    uint8_t           stream_err;     /* Set if entropy stream overruns EOF / premature marker */
    uint8_t           scan_order[3];  /* SOS component decode order -> index into comp[] */

    /* Bitstream accumulator (MSB-aligned in top bits of bit_buf) */
    uint32_t          bit_buf;
    int32_t           bit_cnt;
    int32_t           pad_bits;       /* Synthetic zero-pad bits appended after EOF/marker */

    /* Quantization & Huffman tables */
    uint16_t          qtbl[4][64];
    uint8_t           qtbl_valid[4];
    c3m1_jhuff_t      huff_dc[2];
    c3m1_jhuff_t      huff_ac[2];
    c3m1_jcomp_t      comp[3];

    /* Scratch MCU buffers */
    int16_t           blocks[6][64];  /* Max 4xY + 1xCb + 1xCr */
    uint16_t          mcu_rgb565[256]; /* Up to 16x16 RGB565 tile */
} c3m1_jdec_t;

/**
 * @brief  Parse JPEG headers up to Start-Of-Scan (SOS).
 * @return 1 on success, 0 on unsupported/invalid JPEG.
 */
uint8_t c3m1_jdec_prepare(c3m1_jdec_t *jd, c3m1_jdec_read_fn read_fn, void *read_ctx);

/**
 * @brief  Decode MCUs overlapping [roi_x..roi_x+roi_w) x [roi_y..roi_y+roi_h).
 *         MCUs outside ROI skip IDCT & RGB conversion; stops once y >= roi_y + roi_h.
 * @return 1 on success, 0 on decode error.
 */
uint8_t c3m1_jdec_decode(c3m1_jdec_t *jd,
                         uint16_t roi_x, uint16_t roi_y,
                         uint16_t roi_w, uint16_t roi_h,
                         c3m1_jdec_mcu_cb mcu_cb, void *cb_ctx);

#endif /* C3M1_JDEC_H_ */
