/**
 * @file        c3m1_dng.h
 * @brief       DNG 1.4 (TIFF/EP) header builder with IFD0 160x120 RGB embedded preview
 *              and SubIFD 644x480 10-bit Gamma-companded GRBG Bayer RAW.
 */

#ifndef C3M1_DNG_H_
#define C3M1_DNG_H_

#include <stdint.h>

#define C3M1_DNG_HEADER_SIZE    2048U
#define C3M1_DNG_THUMB_W        160U
#define C3M1_DNG_THUMB_H        120U
#define C3M1_DNG_THUMB_BYTES    (C3M1_DNG_THUMB_W * C3M1_DNG_THUMB_H * 3U) /* 57,600 B */
#define C3M1_DNG_RAW_W          644U
#define C3M1_DNG_RAW_H          480U
#define C3M1_DNG_RAW_BYTES      (C3M1_DNG_RAW_W * C3M1_DNG_RAW_H)          /* 309,120 B */
#define C3M1_DNG_HDR_RAW_BYTES  (C3M1_DNG_RAW_W * C3M1_DNG_RAW_H * 2U)     /* 618,240 B (16-bit LE) */
#define C3M1_DNG_THUMB_OFFSET   C3M1_DNG_HEADER_SIZE
#define C3M1_DNG_RAW_OFFSET     (C3M1_DNG_THUMB_OFFSET + C3M1_DNG_THUMB_BYTES)

typedef struct
{
    uint32_t exp_num;       /* EXIF ExposureTime numerator (seconds), e.g. 1 or 2 */
    uint32_t exp_den;       /* EXIF ExposureTime denominator, e.g. 100 (1/100s) or 1 (2s) */
    uint16_t iso;           /* EXIF ISOSpeedRatings (100..3200) */
    int32_t  ev_num;        /* EXIF ExposureBiasValue numerator (1/ev_den EV steps) */
    int32_t  ev_den;        /* EXIF ExposureBiasValue denominator (e.g. 3) */
} c3m1_dng_exif_t;

typedef struct
{
    int8_t  distort;        /* -100..+100 -> WarpRectilinear kr1 = distort * 0.002 (0=OFF) */
    uint8_t vignette;       /*    0..50   -> FixVignetteRadial k0 = vignette * 0.05 (0=OFF) */
} c3m1_dng_lens_t;

/**
 * @brief  Build complete 2,048-byte DNG 1.4 header (IFD0 160x120 RGB preview + SubIFD RAW + ExifIFD).
 * @param  out          Output buffer of at least C3M1_DNG_HEADER_SIZE (2048) bytes.
 * @param  inv_gamma_12 256-entry 12-bit LinearizationTable (0..4064).
 * @param  bl_g         12-bit BlackLevel for Green channels (GRBG indices 0 & 3).
 * @param  bl_rb        12-bit BlackLevel for Red/Blue channels (GRBG indices 1 & 2).
 * @param  wb_r_q8      Capture AWB Red gain (Q8, 256 = 1.0) -> written as AsShotNeutral (0xC628).
 * @param  wb_b_q8      Capture AWB Blue gain (Q8, 256 = 1.0) -> written as AsShotNeutral (0xC628).
 * @param  exif         Optional EXIF exposure parameters (ExposureTime, ISO, ExposureBiasValue).
 * @param  lens         Optional lens correction params -> written as DNG 1.4 OpcodeList3 (0xC74E).
 */
void c3m1_dng_build_header(uint8_t *out,
                           const uint16_t inv_gamma_12[256],
                           uint16_t bl_g,
                           uint16_t bl_rb,
                           uint16_t wb_r_q8,
                           uint16_t wb_b_q8,
                           const c3m1_dng_exif_t *exif,
                           const c3m1_dng_lens_t *lens);

/**
 * @brief  Build 2,048-byte DNG 1.4 header for merged 12/14/16-bit linear HDR RAW.
 *         BlackLevel is 0 (already subtracted prior to HDR fusion); BaselineExposure (0xC62A)
 *         aligns Lightroom/ACR default brightness to the user's metered base exposure.
 */
void c3m1_dng_build_hdr_header(uint8_t *out,
                               uint16_t white_level,
                               int32_t baseline_ev_x100,
                               uint16_t wb_r_q8,
                               uint16_t wb_b_q8,
                               const c3m1_dng_exif_t *exif,
                               const c3m1_dng_lens_t *lens);

#endif /* C3M1_DNG_H_ */
