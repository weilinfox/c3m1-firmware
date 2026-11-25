/**
 * @file        c3m1_dng.h
 * @brief       DNG 1.4 (TIFF/EP) header builder with IFD0 160x120 RGB embedded preview
 *              and SubIFD 644x480 10-bit Gamma-companded GRBG Bayer RAW.
 */

#ifndef C3M1_DNG_H_
#define C3M1_DNG_H_

#include <stdint.h>

#define C3M1_DNG_HEADER_SIZE    1536U
#define C3M1_DNG_THUMB_W        160U
#define C3M1_DNG_THUMB_H        120U
#define C3M1_DNG_THUMB_BYTES    (C3M1_DNG_THUMB_W * C3M1_DNG_THUMB_H * 3U) /* 57,600 B */
#define C3M1_DNG_RAW_W          644U
#define C3M1_DNG_RAW_H          480U
#define C3M1_DNG_RAW_BYTES      (C3M1_DNG_RAW_W * C3M1_DNG_RAW_H)          /* 309,120 B */
#define C3M1_DNG_HDR_RAW_BYTES  (C3M1_DNG_RAW_W * C3M1_DNG_RAW_H * 2U)     /* 618,240 B (16-bit LE) */
#define C3M1_DNG_THUMB_OFFSET   C3M1_DNG_HEADER_SIZE
#define C3M1_DNG_RAW_OFFSET     (C3M1_DNG_THUMB_OFFSET + C3M1_DNG_THUMB_BYTES)

/**
 * @brief  Build complete 1,536-byte DNG 1.4 header (IFD0 160x120 RGB preview + SubIFD RAW).
 * @param  out          Output buffer of at least C3M1_DNG_HEADER_SIZE (1536) bytes.
 * @param  inv_gamma_12 256-entry 12-bit LinearizationTable (0..4064).
 * @param  bl_g         12-bit BlackLevel for Green channels (GRBG indices 0 & 3).
 * @param  bl_rb        12-bit BlackLevel for Red/Blue channels (GRBG indices 1 & 2).
 * @param  wb_r_q8      Capture AWB Red gain (Q8, 256 = 1.0) -> written as AsShotNeutral (0xC628).
 * @param  wb_b_q8      Capture AWB Blue gain (Q8, 256 = 1.0) -> written as AsShotNeutral (0xC628).
 */
void c3m1_dng_build_header(uint8_t *out,
                           const uint16_t inv_gamma_12[256],
                           uint16_t bl_g,
                           uint16_t bl_rb,
                           uint16_t wb_r_q8,
                           uint16_t wb_b_q8);

/**
 * @brief  Build 1,536-byte DNG 1.4 header for merged 12-bit / 14-bit linear 16-bit HDR RAW.
 *         BlackLevel is 0 (already subtracted prior to HDR fusion); BaselineExposure (0xC62A)
 *         aligns Lightroom/ACR default brightness to the user's metered base exposure.
 */
void c3m1_dng_build_hdr_header(uint8_t *out,
                               uint16_t white_level,
                               int32_t baseline_ev_x100,
                               uint16_t wb_r_q8,
                               uint16_t wb_b_q8);

#endif /* C3M1_DNG_H_ */
