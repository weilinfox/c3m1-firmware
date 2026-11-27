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
    uint32_t        exp_num;        /* EXIF ExposureTime numerator (seconds) */
    uint32_t        exp_den;        /* EXIF ExposureTime denominator */
    uint16_t        iso;            /* EXIF ISOSpeedRatings */
    int32_t         ev_num;         /* EXIF ExposureBiasValue numerator */
    int32_t         ev_den;         /* EXIF ExposureBiasValue denominator */
    uint16_t        fnum_x10;       /* EXIF FNumber * 10 (0.1 step), 0 = write 0/1 (F0) */
    uint16_t        focal_x10;      /* EXIF FocalLength * 10 (0.1mm), 0 = omit */
    uint16_t        focal_eq35;     /* EXIF FocalLengthIn35mmFilm (mm), 0 = omit */
    const char     *lens_model;     /* Optional EXIF LensModel (0xA434) ASCII string */
    int8_t          lens_distort;   /* -100..+100 -> WarpRectilinear kr1 = val * 0.002 (0=OFF) */
    uint8_t         lens_vignette;  /* 0..50 -> g(r) = 1 + (val*0.05)*r^2 (0=OFF) */
    uint8_t         anti_alias;     /* 1: Malvar-He-Cutler 5x5 demosaic (unwarped), 0: 3x3 bilinear */
    uint8_t        *out_buf;        /* Caller-owned I/O buffer (e.g. 4096 B) */
    uint16_t        out_buf_size;
    void          (*read_row_fn)(void *user, uint8_t *row644);
    uint8_t       (*write_fn)(void *user, const uint8_t *data, uint32_t len);
    void           *user;
} c3m1_jpeg_ctx_t;

/* Exact fixed-point mirror of DNG FixVignetteRadial g(r) = 1 + (vig_steps*0.05)*r^2 for 640x480 */
static inline uint32_t c3m1_lens_vig_scale_q16(uint8_t vig_steps)
{
    return (vig_steps > 0U) ? (((uint32_t)vig_steps * 16384UL + 1562UL) / 3125UL) : 0UL;
}

static inline __attribute__((always_inline)) uint32_t c3m1_lens_vig_gain_q8(uint16_t x, uint16_t y, uint32_t scale_q16)
{
    int32_t dx = (int32_t)x - 320;
    int32_t dy = (int32_t)y - 240;
    uint32_t d2 = (uint32_t)(dx * dx + dy * dy);
    return 256UL + ((d2 * scale_q16 + 32768UL) >> 16);
}

/* Exact fixed-point mirror of DNG WarpRectilinear (maxfov inscribed zero-black-border kr0) */
static inline void c3m1_lens_dist_params_q16(int8_t distort, int32_t *out_kr0_q16, int32_t *out_kr1_q16)
{
    if (distort < -100) distort = -100;
    if (distort >  100) distort =  100;
    int32_t kr1 = ((int32_t)distort * 65536L + ((distort >= 0) ? 250L : -250L)) / 500L;
    int32_t kr0;
    if (distort < 0)
    {
        kr0 = 65536L - ((kr1 * 36L - 50L) / 100L);
    }
    else
    {
        kr0 = 65536L - kr1;
    }
    *out_kr0_q16 = kr0;
    *out_kr1_q16 = kr1;
}

/* Map output 640x480 pixel (x, y) to source 640x480 Q4 subpixel (sx_q4, sy_q4) */
static inline void c3m1_lens_warp_vga_q4(uint16_t x, uint16_t y,
                                         int32_t kr0_q16, int32_t kr1_q16,
                                         uint16_t *out_sx_q4, uint16_t *out_sy_q4)
{
    int32_t  dx     = (int32_t)x - 320L;
    int32_t  dy     = (int32_t)y - 240L;
    uint32_t d2     = (uint32_t)(dx * dx + dy * dy);
    uint32_t r2_q16 = (d2 * 13422UL) >> 15; /* 65536 / 160000 = 13421.77 / 32768 */
    int32_t  s_q16  = kr0_q16 + ((kr1_q16 * (int32_t)r2_q16) >> 16);
    int32_t  sx_q4  = (320L << 4) + ((dx * s_q16) >> 12);
    int32_t  sy_q4  = (240L << 4) + ((dy * s_q16) >> 12);
    if (sx_q4 < 0L) sx_q4 = 0L;
    if (sx_q4 > (639L << 4)) sx_q4 = (639L << 4);
    if (sy_q4 < 0L) sy_q4 = 0L;
    if (sy_q4 > (479L << 4)) sy_q4 = (479L << 4);
    *out_sx_q4 = (uint16_t)sx_q4;
    *out_sy_q4 = (uint16_t)sy_q4;
}

/**
 * @brief  Convert 12-bit linear camera RGB (post-BlackLevel) to calibrated 8-bit sRGB.
 *         Shared by DNG 160x120 embedded preview thumbnail and VGA JPEG encoder.
 */
void c3m1_color_cam12_to_srgb8(uint16_t r12, uint16_t g12, uint16_t b12,
                               uint16_t wb_r_q8, uint16_t wb_b_q8,
                               const uint8_t srgb_lut[256],
                               uint8_t *out_r, uint8_t *out_g, uint8_t *out_b);

/**
 * @brief  Convert merged linear HDR camera RGB into display sRGB 8-bit with hue-preserving
 *         Reinhard-with-white-point highlight rolloff (WB + CCM first, then shared RGB scale).
 * @param  r16, g16, b16  Black-subtracted merged linear RGB samples.
 * @param  wb_r_q8, wb_b_q8 Capture AWB gains in Q8 (256 = 1.0x).
 * @param  ref_white      Merged linear value corresponding to 0 EV diffuse white.
 * @param  white_level    Raw sensor saturation ceiling in the same linear domain as r16/g16/b16.
 */
void c3m1_color_hdr_to_srgb8(uint16_t r16, uint16_t g16, uint16_t b16,
                             uint16_t wb_r_q8, uint16_t wb_b_q8,
                             uint32_t ref_white, uint32_t white_level,
                             const uint8_t srgb_lut[256],
                             uint8_t *out_r, uint8_t *out_g, uint8_t *out_b);

/**
 * @brief  Encode a 644x480 GRBG 10-bit Gamma-companded Bayer frame into a 640x480 Baseline JPEG.
 * @return 1 on success, 0 on write error.
 */
uint8_t c3m1_jpeg_encode_bayer_vga(const c3m1_jpeg_ctx_t *ctx, uint32_t *out_total_bytes);

/**
 * @brief  Borrow the 31.4 KB 4-byte-aligned Bayer ring buffer when JPEG capture is idle
 *         (reused by Playback, DNG warped thumbnail, and 1X warped live viewfinder).
 */
uint8_t *c3m1_jpeg_get_scratch_buf(uint32_t *size_out);

#endif /* C3M1_JPEG_H_ */
