/**
 * @file        c3m1_dng.c
 * @brief       Programmatic DNG 1.4 (TIFF/EP) multi-IFD header builder.
 *              Layout:
 *                [0..1535]      TIFF header + IFD0 (160x120 RGB preview) + SubIFD (644x480 CFA RAW) + tag data
 *                [1536..59135]  160x120x3 uncompressed RGB preview thumbnail (57,600 B)
 *                [59136..368255] 644x480 8-bit Gamma-companded GRBG Bayer RAW (309,120 B)
 */

#include <string.h>
#include "c3m1_dng.h"
#include "c3m1_clock.h"

/* TIFF field types */
#define TIFF_BYTE       1U
#define TIFF_ASCII      2U
#define TIFF_SHORT      3U
#define TIFF_LONG       4U
#define TIFF_RATIONAL   5U
#define TIFF_UNDEFINED  7U
#define TIFF_SRATIONAL  10U
#define TIFF_DOUBLE     12U

static void wr16(uint8_t *buf, uint32_t off, uint16_t v)
{
    buf[off]     = (uint8_t)(v & 0xFFU);
    buf[off + 1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *buf, uint32_t off, uint32_t v)
{
    buf[off]     = (uint8_t)(v & 0xFFU);
    buf[off + 1] = (uint8_t)((v >> 8) & 0xFFU);
    buf[off + 2] = (uint8_t)((v >> 16) & 0xFFU);
    buf[off + 3] = (uint8_t)(v >> 24);
}

static void wr32_be(uint8_t *buf, uint32_t off, uint32_t v)
{
    buf[off + 0U] = (uint8_t)(v >> 24);
    buf[off + 1U] = (uint8_t)((v >> 16) & 0xFFU);
    buf[off + 2U] = (uint8_t)((v >> 8) & 0xFFU);
    buf[off + 3U] = (uint8_t)(v & 0xFFU);
}

static void wr_f64_be(uint8_t *buf, uint32_t off, double v)
{
    uint8_t raw[8];
    memcpy(raw, &v, 8U);
    for (uint8_t i = 0U; i < 8U; i++)
    {
        buf[off + i] = raw[7U - i];
    }
}

/*
 * Build DNG 1.4 OpcodeList3 (0xC74E) payload in mandatory BIG-ENDIAN byte order:
 *   1. FixVignetteRadial (ID=3, g(r) = 1 + k0*r^2) runs first in original sensor coordinates.
 *   2. WarpRectilinear   (ID=1, single plane, r_src = r*(1 + kr1*r^2)) runs second.
 * Returns exact payload length in bytes (0 if both parameters are zero or lens == NULL).
 */
static uint32_t build_opcode_list3(uint8_t *dst, const c3m1_dng_lens_t *lens)
{
    if (lens == NULL || (lens->distort == 0 && lens->vignette == 0U))
    {
        return 0UL;
    }

    uint32_t count = 0UL;
    uint32_t pos   = 4UL;

    if (lens->vignette != 0U)
    {
        double k0 = (double)lens->vignette * 0.05;
        wr32_be(dst, pos + 0U,  3UL);          /* Opcode ID = 3 (FixVignetteRadial) */
        wr32_be(dst, pos + 4U,  0x01030000UL); /* DNG Spec Version 1.3.0.0 */
        wr32_be(dst, pos + 8U,  1UL);          /* Flags: Optional */
        wr32_be(dst, pos + 12U, 56UL);         /* 7 x float64 = 56 B */
        pos += 16U;
        wr_f64_be(dst, pos + 0U,  k0);         /* k0 */
        wr_f64_be(dst, pos + 8U,  0.0);        /* k1 */
        wr_f64_be(dst, pos + 16U, 0.0);        /* k2 */
        wr_f64_be(dst, pos + 24U, 0.0);        /* k3 */
        wr_f64_be(dst, pos + 32U, 0.0);        /* k4 */
        wr_f64_be(dst, pos + 40U, 0.5);        /* cx */
        wr_f64_be(dst, pos + 48U, 0.5);        /* cy */
        pos += 56U;
        count++;
    }

    if (lens->distort != 0)
    {
        double kr1 = (double)lens->distort * 0.002;
        /* Max-FOV inscribed zero-black-border scale on 4:3 active area (r_tb^2=0.36, r_corner^2=1.0):
         *   barrel     (kr1 < 0): kr0 = 1.0 - 0.36 * kr1 -> 100% top/bottom center FOV kept
         *   pincushion (kr1 > 0): kr0 = 1.0 - kr1        -> 100% corner FOV kept, no black corners */
        double kr0 = (kr1 < 0.0) ? (1.0 - 0.36 * kr1) : (1.0 - kr1);
        wr32_be(dst, pos + 0U,  1UL);          /* Opcode ID = 1 (WarpRectilinear) */
        wr32_be(dst, pos + 4U,  0x01030000UL); /* DNG Spec Version 1.3.0.0 */
        wr32_be(dst, pos + 8U,  1UL);          /* Flags: Optional */
        wr32_be(dst, pos + 12U, 68UL);         /* uint32 planes + 8 x float64 = 68 B */
        pos += 16U;
        wr32_be(dst, pos, 1UL);                /* 1 plane (uniform geometric undistortion, no CA) */
        pos += 4U;
        wr_f64_be(dst, pos + 0U,  kr0);        /* kr0 (inscribed max-FOV scale) */
        wr_f64_be(dst, pos + 8U,  kr1);        /* kr1 */
        wr_f64_be(dst, pos + 16U, 0.0);        /* kr2 */
        wr_f64_be(dst, pos + 24U, 0.0);        /* kr3 */
        wr_f64_be(dst, pos + 32U, 0.0);        /* kt0 */
        wr_f64_be(dst, pos + 40U, 0.0);        /* kt1 */
        wr_f64_be(dst, pos + 48U, 0.5);        /* cx */
        wr_f64_be(dst, pos + 56U, 0.5);        /* cy */
        pos += 64U;
        count++;
    }

    wr32_be(dst, 0U, count);
    return pos;
}

static void wr_entry(uint8_t *buf, uint32_t *pos, uint16_t tag, uint16_t type, uint32_t count, uint32_t val)
{
    uint32_t p = *pos;
    wr16(buf, p,      tag);
    wr16(buf, p + 2,  type);
    wr32(buf, p + 4,  count);
    wr32(buf, p + 8,  val);
    *pos = p + 12U;
}

static uint32_t alloc_data(uint32_t *data_pos, uint32_t len)
{
    uint32_t off = *data_pos;
    *data_pos = (off + len + 3U) & ~3UL; /* 4-byte align all external tag payloads */
    return off;
}

static void write_exif_ifd(uint8_t *out, uint32_t exif_off, uint32_t off_dt,
                           uint32_t off_exp, uint32_t off_ev,
                           const c3m1_dng_exif_t *exif)
{
    uint32_t exp_num = (exif != NULL && exif->exp_num > 0UL && exif->exp_den > 0UL) ? exif->exp_num : 1UL;
    uint32_t exp_den = (exif != NULL && exif->exp_num > 0UL && exif->exp_den > 0UL) ? exif->exp_den : 100UL;
    uint16_t iso     = (exif != NULL && exif->iso > 0U) ? exif->iso : 100U;
    int32_t  ev_num  = (exif != NULL && exif->ev_den > 0) ? exif->ev_num : 0;
    int32_t  ev_den  = (exif != NULL && exif->ev_den > 0) ? exif->ev_den : 3;

    wr32(out, off_exp + 0U, exp_num);
    wr32(out, off_exp + 4U, exp_den);
    wr32(out, off_ev  + 0U, (uint32_t)ev_num);
    wr32(out, off_ev  + 4U, (uint32_t)ev_den);

    /* ExifIFD: 5 entries in strictly ascending tag order */
    uint32_t p = exif_off;
    wr16(out, p, 5U);
    p += 2U;
    wr_entry(out, &p, 0x829AU, TIFF_RATIONAL,  1U,  off_exp); /* ExposureTime */
    wr_entry(out, &p, 0x8827U, TIFF_SHORT,     1U,  iso);     /* ISOSpeedRatings */
    wr_entry(out, &p, 0x9003U, TIFF_ASCII,     20U, off_dt);  /* DateTimeOriginal */
    wr_entry(out, &p, 0x9004U, TIFF_ASCII,     20U, off_dt);  /* DateTimeDigitized */
    wr_entry(out, &p, 0x9204U, TIFF_SRATIONAL, 1U,  off_ev);  /* ExposureBiasValue */
    wr32(out, p, 0U);
}

void c3m1_dng_build_header(uint8_t *out,
                           const uint16_t inv_gamma_12[256],
                           uint16_t bl_g,
                           uint16_t bl_rb,
                           uint16_t wb_r_q8,
                           uint16_t wb_b_q8,
                           const c3m1_dng_exif_t *exif,
                           const c3m1_dng_lens_t *lens)
{
    /*
     * Physically grounded OV7725 ColorMatrix1 (CIE XYZ_D50 -> Camera RAW, Q10000):
     *   Derived from Bradford-adapted D65 XYZ->sRGB * OV7725 crosstalk * measured D65 channel
     *   sensitivity (R:G:B = 1.00 : 1.00 : 0.63). Matches c3m1_jpeg.c CCM so DNG "As Shot"
     *   in Adobe Camera Raw and direct-out VGA JPEG share consistent color & white balance.
     */
    static const int32_t s_color_matrix1[9] = {
         25285, -11589, -3383,
         -5654,  14617,  1012,
           172,     52,  7370,
    };

    /* Exact IEEE 754 double NoiseProfile (3 planes) */
    static const uint8_t s_noise_pair[16] = {
        0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
        0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E,
    };

    static const char s_make[]   = "CAYE";
    static const char s_model[]  = "C3M1";
    static const char s_unique[] = "CAYE C3M1";
    /* Legacy hardcoded DNG date kept in source for reference only; unused at runtime (overridden by c3m1_clock) */
    static const char s_legacy_dt[] __attribute__((unused)) = "2026:08:17 00:00:00";
    char s_dt[20];
    c3m1_clock_format_exif(s_dt);

    memset(out, 0, C3M1_DNG_HEADER_SIZE);

    /* TIFF Little-Endian Header */
    out[0] = 'I';
    out[1] = 'I';
    wr16(out, 2U, 42U);
    wr32(out, 4U, 8U); /* IFD0 offset */

    uint8_t op3_buf[160];
    uint32_t op3_len = build_opcode_list3(op3_buf, lens);
    uint8_t has_op3  = (op3_len > 0UL) ? 1U : 0U;

    const uint16_t ifd0_cnt    = 27U;
    const uint32_t ifd0_off    = 8U;
    const uint32_t subifd_off  = (ifd0_off + 2U + (uint32_t)ifd0_cnt * 12U + 4U + 3U) & ~3UL;
    const uint16_t subifd_cnt  = has_op3 ? 25U : 24U;
    const uint32_t exififd_off = (subifd_off + 2U + (uint32_t)subifd_cnt * 12U + 4U + 3U) & ~3UL;
    uint32_t data_pos          = (exififd_off + 2U + 5UL * 12U + 4U + 3U) & ~3UL;

    /* ---- Allocate external tag payloads ---- */
    uint32_t off_bps_rgb = alloc_data(&data_pos, 6U);
    wr16(out, off_bps_rgb + 0U, 8U);
    wr16(out, off_bps_rgb + 2U, 8U);
    wr16(out, off_bps_rgb + 4U, 8U);

    uint32_t off_make = alloc_data(&data_pos, sizeof(s_make));
    memcpy(&out[off_make], s_make, sizeof(s_make));

    uint32_t off_model = alloc_data(&data_pos, sizeof(s_model));
    memcpy(&out[off_model], s_model, sizeof(s_model));

    uint32_t off_unique = alloc_data(&data_pos, sizeof(s_unique));
    memcpy(&out[off_unique], s_unique, sizeof(s_unique));

    uint32_t off_res = alloc_data(&data_pos, 8U);
    wr32(out, off_res + 0U, 72U);
    wr32(out, off_res + 4U, 1U);

    uint32_t off_dt = alloc_data(&data_pos, sizeof(s_dt));
    memcpy(&out[off_dt], s_dt, sizeof(s_dt));

    uint32_t off_exp = alloc_data(&data_pos, 8U);
    uint32_t off_ev  = alloc_data(&data_pos, 8U);

    uint32_t off_cm1 = alloc_data(&data_pos, 9U * 8U);
    for (uint8_t i = 0; i < 9U; i++)
    {
        wr32(out, off_cm1 + (uint32_t)i * 8U + 0U, (uint32_t)s_color_matrix1[i]);
        wr32(out, off_cm1 + (uint32_t)i * 8U + 4U, 10000U);
    }

    /* AsShotNeutral (0xC628): inverse of capture AWB gains, normalized into (0, 1] */
    if (wb_r_q8 == 0U) wb_r_q8 = 256U;
    if (wb_b_q8 == 0U) wb_b_q8 = 256U;
    uint32_t wb_min = 256UL;
    if (wb_r_q8 < wb_min) wb_min = wb_r_q8;
    if (wb_b_q8 < wb_min) wb_min = wb_b_q8;
    uint32_t off_asn = alloc_data(&data_pos, 3U * 8U);
    wr32(out, off_asn + 0U,  wb_min); wr32(out, off_asn + 4U,  wb_r_q8);
    wr32(out, off_asn + 8U,  wb_min); wr32(out, off_asn + 12U, 256UL);
    wr32(out, off_asn + 16U, wb_min); wr32(out, off_asn + 20U, wb_b_q8);

    uint32_t off_bl = alloc_data(&data_pos, 4U * 2U);
    wr16(out, off_bl + 0U, bl_g);   /* G (even row, even col) */
    wr16(out, off_bl + 2U, bl_rb);  /* R (even row, odd col) */
    wr16(out, off_bl + 4U, bl_rb);  /* B (odd row, even col) */
    wr16(out, off_bl + 6U, bl_g);   /* G (odd row, odd col) */

    uint32_t off_scale = alloc_data(&data_pos, 2U * 8U);
    wr32(out, off_scale + 0U,  1U); wr32(out, off_scale + 4U,  1U);
    wr32(out, off_scale + 8U,  1U); wr32(out, off_scale + 12U, 1U);

    uint32_t off_active = alloc_data(&data_pos, 4U * 4U);
    wr32(out, off_active + 0U,  0U);
    wr32(out, off_active + 4U,  0U);
    wr32(out, off_active + 8U,  480U);
    wr32(out, off_active + 12U, 640U);

    uint32_t off_noise = alloc_data(&data_pos, 48U);
    memcpy(&out[off_noise + 0U],  s_noise_pair, 16U);
    memcpy(&out[off_noise + 16U], s_noise_pair, 16U);
    memcpy(&out[off_noise + 32U], s_noise_pair, 16U);

    uint32_t off_op3 = 0UL;
    if (has_op3)
    {
        off_op3 = alloc_data(&data_pos, op3_len);
        memcpy(&out[off_op3], op3_buf, op3_len);
    }

    uint32_t off_lin = alloc_data(&data_pos, 256U * 2U);
    for (uint16_t i = 0; i < 256U; i++)
    {
        wr16(out, off_lin + (uint32_t)i * 2U, inv_gamma_12[i]);
    }

    /* ---- Write IFD0: 160x120 RGB Embedded Preview (tags strictly ascending) ---- */
    uint32_t p = ifd0_off;
    wr16(out, p, ifd0_cnt);
    p += 2U;

    wr_entry(out, &p, 0x00FEU, TIFF_LONG,      1U, 1U);                     /* NewSubfileType: Preview */
    wr_entry(out, &p, 0x0100U, TIFF_LONG,      1U, C3M1_DNG_THUMB_W);       /* ImageWidth = 160 */
    wr_entry(out, &p, 0x0101U, TIFF_LONG,      1U, C3M1_DNG_THUMB_H);       /* ImageLength = 120 */
    wr_entry(out, &p, 0x0102U, TIFF_SHORT,     3U, off_bps_rgb);            /* BitsPerSample = 8,8,8 */
    wr_entry(out, &p, 0x0103U, TIFF_SHORT,     1U, 1U);                     /* Compression = None */
    wr_entry(out, &p, 0x0106U, TIFF_SHORT,     1U, 2U);                     /* Photometric = RGB */
    wr_entry(out, &p, 0x010FU, TIFF_ASCII,     sizeof(s_make), off_make);   /* Make */
    wr_entry(out, &p, 0x0110U, TIFF_ASCII,     sizeof(s_model), off_model); /* Model */
    wr_entry(out, &p, 0x0111U, TIFF_LONG,      1U, C3M1_DNG_THUMB_OFFSET);  /* StripOffsets */
    wr_entry(out, &p, 0x0112U, TIFF_SHORT,     1U, 1U);                     /* Orientation = Upright */
    wr_entry(out, &p, 0x0115U, TIFF_SHORT,     1U, 3U);                     /* SamplesPerPixel = 3 */
    wr_entry(out, &p, 0x0116U, TIFF_LONG,      1U, C3M1_DNG_THUMB_H);       /* RowsPerStrip = 120 */
    wr_entry(out, &p, 0x0117U, TIFF_LONG,      1U, C3M1_DNG_THUMB_BYTES);   /* StripByteCounts = 57600 */
    wr_entry(out, &p, 0x011AU, TIFF_RATIONAL,  1U, off_res);                /* XResolution = 72 */
    wr_entry(out, &p, 0x011BU, TIFF_RATIONAL,  1U, off_res);                /* YResolution = 72 */
    wr_entry(out, &p, 0x011CU, TIFF_SHORT,     1U, 1U);                     /* PlanarConfiguration = 1 */
    wr_entry(out, &p, 0x0128U, TIFF_SHORT,     1U, 2U);                     /* ResolutionUnit = inch */
    wr_entry(out, &p, 0x0132U, TIFF_ASCII,     sizeof(s_dt), off_dt);       /* DateTime */
    wr_entry(out, &p, 0x014AU, TIFF_LONG,      1U, subifd_off);             /* SubIFDs -> RAW IFD */
    wr_entry(out, &p, 0x8769U, TIFF_LONG,      1U, exififd_off);            /* ExifIFD -> Exposure/ISO/EV */
    wr_entry(out, &p, 0x9216U, TIFF_BYTE,      4U, 0x00000001UL);           /* TIFF/EPStandardID 1.0.0.0 */
    wr_entry(out, &p, 0xC612U, TIFF_BYTE,      4U, 0x00000401UL);           /* DNGVersion 1.4.0.0 */
    wr_entry(out, &p, 0xC613U, TIFF_BYTE,      4U, 0x00000101UL);           /* DNGBackwardVersion 1.1.0.0 */
    wr_entry(out, &p, 0xC614U, TIFF_ASCII,     sizeof(s_unique), off_unique); /* UniqueCameraModel */
    wr_entry(out, &p, 0xC621U, TIFF_SRATIONAL, 9U, off_cm1);                  /* ColorMatrix1 */
    wr_entry(out, &p, 0xC628U, TIFF_RATIONAL,  3U, off_asn);                  /* AsShotNeutral (AWB) */
    wr_entry(out, &p, 0xC65AU, TIFF_SHORT,     1U, 21U);                      /* CalibrationIlluminant1 = D65 */
    wr32(out, p, 0U); /* Next IFD = 0 */

    /* ---- Write SubIFD: Full-Resolution 644x480 GRBG Bayer RAW (tags strictly ascending) ---- */
    p = subifd_off;
    wr16(out, p, subifd_cnt);
    p += 2U;

    wr_entry(out, &p, 0x00FEU, TIFF_LONG,      1U, 0U);                     /* NewSubfileType: Full-res */
    wr_entry(out, &p, 0x0100U, TIFF_LONG,      1U, C3M1_DNG_RAW_W);         /* ImageWidth = 644 */
    wr_entry(out, &p, 0x0101U, TIFF_LONG,      1U, C3M1_DNG_RAW_H);         /* ImageLength = 480 */
    wr_entry(out, &p, 0x0102U, TIFF_SHORT,     1U, 8U);                     /* BitsPerSample = 8 */
    wr_entry(out, &p, 0x0103U, TIFF_SHORT,     1U, 1U);                     /* Compression = None */
    wr_entry(out, &p, 0x0106U, TIFF_SHORT,     1U, 32803U);                 /* Photometric = CFA */
    wr_entry(out, &p, 0x0111U, TIFF_LONG,      1U, C3M1_DNG_RAW_OFFSET);    /* StripOffsets */
    wr_entry(out, &p, 0x0115U, TIFF_SHORT,     1U, 1U);                     /* SamplesPerPixel = 1 */
    wr_entry(out, &p, 0x0116U, TIFF_LONG,      1U, C3M1_DNG_RAW_H);         /* RowsPerStrip = 480 */
    wr_entry(out, &p, 0x0117U, TIFF_LONG,      1U, C3M1_DNG_RAW_BYTES);     /* StripByteCounts = 309120 */
    wr_entry(out, &p, 0x011CU, TIFF_SHORT,     1U, 1U);                     /* PlanarConfiguration = 1 */
    wr_entry(out, &p, 0x828DU, TIFF_SHORT,     2U, 0x00020002UL);           /* CFARepeatPatternDim 2x2 */
    wr_entry(out, &p, 0x828EU, TIFF_BYTE,      4U, 0x01020001UL);           /* CFAPattern GRBG */
    wr_entry(out, &p, 0xC616U, TIFF_BYTE,      3U, 0x00020100UL);           /* CFAPlaneColor RGB */
    wr_entry(out, &p, 0xC617U, TIFF_SHORT,     1U, 1U);                     /* CFALayout = Rectangular */
    wr_entry(out, &p, 0xC618U, TIFF_SHORT,     256U, off_lin);              /* LinearizationTable */
    wr_entry(out, &p, 0xC619U, TIFF_SHORT,     2U, 0x00020002UL);           /* BlackLevelRepeatDim 2x2 */
    wr_entry(out, &p, 0xC61AU, TIFF_SHORT,     4U, off_bl);                 /* BlackLevel GRBG */
    wr_entry(out, &p, 0xC61DU, TIFF_LONG,      1U, 4064U);                  /* WhiteLevel = 4064 */
    wr_entry(out, &p, 0xC61EU, TIFF_RATIONAL,  2U, off_scale);              /* DefaultScale 1/1, 1/1 */
    wr_entry(out, &p, 0xC61FU, TIFF_SHORT,     2U, 0x00000000UL);           /* DefaultCropOrigin 0,0 */
    wr_entry(out, &p, 0xC620U, TIFF_SHORT,     2U, 0x01E00280UL);           /* DefaultCropSize 640x480 */
    wr_entry(out, &p, 0xC68DU, TIFF_LONG,      4U, off_active);             /* ActiveArea 0,0,480,640 */
    if (has_op3)
    {
        wr_entry(out, &p, 0xC74EU, TIFF_UNDEFINED, op3_len, off_op3);       /* OpcodeList3 (Lens Corr) */
    }
    wr_entry(out, &p, 0xC761U, TIFF_DOUBLE,    6U, off_noise);              /* NoiseProfile */
    wr32(out, p, 0U); /* Next IFD = 0 */

    write_exif_ifd(out, exififd_off, off_dt, off_exp, off_ev, exif);
}

void c3m1_dng_build_hdr_header(uint8_t *out,
                               uint16_t white_level,
                               int32_t baseline_ev_x100,
                               uint16_t wb_r_q8,
                               uint16_t wb_b_q8,
                               const c3m1_dng_exif_t *exif,
                               const c3m1_dng_lens_t *lens)
{
    static const int32_t s_color_matrix1[9] = {
         25285, -11589, -3383,
         -5654,  14617,  1012,
           172,     52,  7370,
    };

    static const uint8_t s_noise_pair[16] = {
        0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
        0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E,
    };

    static const char s_make[]   = "CAYE";
    static const char s_model[]  = "C3M1";
    static const char s_unique[] = "CAYE C3M1";
    char s_dt[20];
    c3m1_clock_format_exif(s_dt);

    memset(out, 0, C3M1_DNG_HEADER_SIZE);

    out[0] = 'I';
    out[1] = 'I';
    wr16(out, 2U, 42U);
    wr32(out, 4U, 8U);

    uint8_t op3_buf[160];
    uint32_t op3_len = build_opcode_list3(op3_buf, lens);
    uint8_t has_op3  = (op3_len > 0UL) ? 1U : 0U;

    const uint16_t ifd0_cnt    = 28U;
    const uint32_t ifd0_off    = 8U;
    const uint32_t subifd_off  = (ifd0_off + 2U + (uint32_t)ifd0_cnt * 12U + 4U + 3U) & ~3UL;
    const uint16_t subifd_cnt  = has_op3 ? 24U : 23U;
    const uint32_t exififd_off = (subifd_off + 2U + (uint32_t)subifd_cnt * 12U + 4U + 3U) & ~3UL;
    uint32_t data_pos          = (exififd_off + 2U + 5UL * 12U + 4U + 3U) & ~3UL;

    uint32_t off_bps_rgb = alloc_data(&data_pos, 6U);
    wr16(out, off_bps_rgb + 0U, 8U);
    wr16(out, off_bps_rgb + 2U, 8U);
    wr16(out, off_bps_rgb + 4U, 8U);

    uint32_t off_make = alloc_data(&data_pos, sizeof(s_make));
    memcpy(&out[off_make], s_make, sizeof(s_make));

    uint32_t off_model = alloc_data(&data_pos, sizeof(s_model));
    memcpy(&out[off_model], s_model, sizeof(s_model));

    uint32_t off_unique = alloc_data(&data_pos, sizeof(s_unique));
    memcpy(&out[off_unique], s_unique, sizeof(s_unique));

    uint32_t off_res = alloc_data(&data_pos, 8U);
    wr32(out, off_res + 0U, 72U);
    wr32(out, off_res + 4U, 1U);

    uint32_t off_dt = alloc_data(&data_pos, sizeof(s_dt));
    memcpy(&out[off_dt], s_dt, sizeof(s_dt));

    uint32_t off_exp = alloc_data(&data_pos, 8U);
    uint32_t off_ev  = alloc_data(&data_pos, 8U);

    uint32_t off_cm1 = alloc_data(&data_pos, 9U * 8U);
    for (uint8_t i = 0U; i < 9U; i++)
    {
        wr32(out, off_cm1 + (uint32_t)i * 8U + 0U, (uint32_t)s_color_matrix1[i]);
        wr32(out, off_cm1 + (uint32_t)i * 8U + 4U, 10000U);
    }

    if (wb_r_q8 == 0U) wb_r_q8 = 256U;
    if (wb_b_q8 == 0U) wb_b_q8 = 256U;
    uint32_t wb_min = 256UL;
    if (wb_r_q8 < wb_min) wb_min = wb_r_q8;
    if (wb_b_q8 < wb_min) wb_min = wb_b_q8;
    uint32_t off_asn = alloc_data(&data_pos, 3U * 8U);
    wr32(out, off_asn + 0U,  wb_min); wr32(out, off_asn + 4U,  wb_r_q8);
    wr32(out, off_asn + 8U,  wb_min); wr32(out, off_asn + 12U, 256UL);
    wr32(out, off_asn + 16U, wb_min); wr32(out, off_asn + 20U, wb_b_q8);

    uint32_t off_bev = alloc_data(&data_pos, 8U);
    wr32(out, off_bev + 0U, (uint32_t)baseline_ev_x100);
    wr32(out, off_bev + 4U, 100U);

    uint32_t off_bl = alloc_data(&data_pos, 4U * 2U); /* Already black-subtracted -> (0,0,0,0) */
    wr16(out, off_bl + 0U, 0U);
    wr16(out, off_bl + 2U, 0U);
    wr16(out, off_bl + 4U, 0U);
    wr16(out, off_bl + 6U, 0U);

    uint32_t off_scale = alloc_data(&data_pos, 2U * 8U);
    wr32(out, off_scale + 0U,  1U); wr32(out, off_scale + 4U,  1U);
    wr32(out, off_scale + 8U,  1U); wr32(out, off_scale + 12U, 1U);

    uint32_t off_active = alloc_data(&data_pos, 4U * 4U);
    wr32(out, off_active + 0U,  0U);
    wr32(out, off_active + 4U,  0U);
    wr32(out, off_active + 8U,  480U);
    wr32(out, off_active + 12U, 640U);

    uint32_t off_noise = alloc_data(&data_pos, 48U);
    memcpy(&out[off_noise + 0U],  s_noise_pair, 16U);
    memcpy(&out[off_noise + 16U], s_noise_pair, 16U);
    memcpy(&out[off_noise + 32U], s_noise_pair, 16U);

    uint32_t off_op3 = 0UL;
    if (has_op3)
    {
        off_op3 = alloc_data(&data_pos, op3_len);
        memcpy(&out[off_op3], op3_buf, op3_len);
    }

    /* ---- IFD0: 160x120 Tone-Mapped RGB Embedded Preview (28 tags, strictly ascending) ---- */
    uint32_t p = ifd0_off;
    wr16(out, p, ifd0_cnt);
    p += 2U;

    wr_entry(out, &p, 0x00FEU, TIFF_LONG,      1U, 1U);
    wr_entry(out, &p, 0x0100U, TIFF_LONG,      1U, C3M1_DNG_THUMB_W);
    wr_entry(out, &p, 0x0101U, TIFF_LONG,      1U, C3M1_DNG_THUMB_H);
    wr_entry(out, &p, 0x0102U, TIFF_SHORT,     3U, off_bps_rgb);
    wr_entry(out, &p, 0x0103U, TIFF_SHORT,     1U, 1U);
    wr_entry(out, &p, 0x0106U, TIFF_SHORT,     1U, 2U);
    wr_entry(out, &p, 0x010FU, TIFF_ASCII,     sizeof(s_make), off_make);
    wr_entry(out, &p, 0x0110U, TIFF_ASCII,     sizeof(s_model), off_model);
    wr_entry(out, &p, 0x0111U, TIFF_LONG,      1U, C3M1_DNG_THUMB_OFFSET);
    wr_entry(out, &p, 0x0112U, TIFF_SHORT,     1U, 1U);
    wr_entry(out, &p, 0x0115U, TIFF_SHORT,     1U, 3U);
    wr_entry(out, &p, 0x0116U, TIFF_LONG,      1U, C3M1_DNG_THUMB_H);
    wr_entry(out, &p, 0x0117U, TIFF_LONG,      1U, C3M1_DNG_THUMB_BYTES);
    wr_entry(out, &p, 0x011AU, TIFF_RATIONAL,  1U, off_res);
    wr_entry(out, &p, 0x011BU, TIFF_RATIONAL,  1U, off_res);
    wr_entry(out, &p, 0x011CU, TIFF_SHORT,     1U, 1U);
    wr_entry(out, &p, 0x0128U, TIFF_SHORT,     1U, 2U);
    wr_entry(out, &p, 0x0132U, TIFF_ASCII,     sizeof(s_dt), off_dt);
    wr_entry(out, &p, 0x014AU, TIFF_LONG,      1U, subifd_off);
    wr_entry(out, &p, 0x8769U, TIFF_LONG,      1U, exififd_off);            /* ExifIFD */
    wr_entry(out, &p, 0x9216U, TIFF_BYTE,      4U, 0x00000001UL);
    wr_entry(out, &p, 0xC612U, TIFF_BYTE,      4U, 0x00000401UL);
    wr_entry(out, &p, 0xC613U, TIFF_BYTE,      4U, 0x00000101UL);
    wr_entry(out, &p, 0xC614U, TIFF_ASCII,     sizeof(s_unique), off_unique);
    wr_entry(out, &p, 0xC621U, TIFF_SRATIONAL, 9U, off_cm1);
    wr_entry(out, &p, 0xC628U, TIFF_RATIONAL,  3U, off_asn);
    wr_entry(out, &p, 0xC62AU, TIFF_SRATIONAL, 1U, off_bev);                /* BaselineExposure */
    wr_entry(out, &p, 0xC65AU, TIFF_SHORT,     1U, 21U);
    wr32(out, p, 0U);

    /* ---- SubIFD: Full-Resolution 644x480 16-bit Linear HDR GRBG RAW (23 tags, ascending) ---- */
    p = subifd_off;
    wr16(out, p, subifd_cnt);
    p += 2U;

    wr_entry(out, &p, 0x00FEU, TIFF_LONG,      1U, 0U);
    wr_entry(out, &p, 0x0100U, TIFF_LONG,      1U, C3M1_DNG_RAW_W);
    wr_entry(out, &p, 0x0101U, TIFF_LONG,      1U, C3M1_DNG_RAW_H);
    wr_entry(out, &p, 0x0102U, TIFF_SHORT,     1U, 16U);                    /* BitsPerSample = 16 */
    wr_entry(out, &p, 0x0103U, TIFF_SHORT,     1U, 1U);
    wr_entry(out, &p, 0x0106U, TIFF_SHORT,     1U, 32803U);
    wr_entry(out, &p, 0x0111U, TIFF_LONG,      1U, C3M1_DNG_RAW_OFFSET);
    wr_entry(out, &p, 0x0115U, TIFF_SHORT,     1U, 1U);
    wr_entry(out, &p, 0x0116U, TIFF_LONG,      1U, C3M1_DNG_RAW_H);
    wr_entry(out, &p, 0x0117U, TIFF_LONG,      1U, C3M1_DNG_HDR_RAW_BYTES); /* 618,240 B */
    wr_entry(out, &p, 0x011CU, TIFF_SHORT,     1U, 1U);
    wr_entry(out, &p, 0x828DU, TIFF_SHORT,     2U, 0x00020002UL);
    wr_entry(out, &p, 0x828EU, TIFF_BYTE,      4U, 0x01020001UL);
    wr_entry(out, &p, 0xC616U, TIFF_BYTE,      3U, 0x00020100UL);
    wr_entry(out, &p, 0xC617U, TIFF_SHORT,     1U, 1U);
    wr_entry(out, &p, 0xC619U, TIFF_SHORT,     2U, 0x00020002UL);
    wr_entry(out, &p, 0xC61AU, TIFF_SHORT,     4U, off_bl);
    wr_entry(out, &p, 0xC61DU, TIFF_LONG,      1U, (uint32_t)white_level);  /* 12-bit / 14-bit WL */
    wr_entry(out, &p, 0xC61EU, TIFF_RATIONAL,  2U, off_scale);
    wr_entry(out, &p, 0xC61FU, TIFF_SHORT,     2U, 0x00000000UL);
    wr_entry(out, &p, 0xC620U, TIFF_SHORT,     2U, 0x01E00280UL);
    wr_entry(out, &p, 0xC68DU, TIFF_LONG,      4U, off_active);
    if (has_op3)
    {
        wr_entry(out, &p, 0xC74EU, TIFF_UNDEFINED, op3_len, off_op3);
    }
    wr_entry(out, &p, 0xC761U, TIFF_DOUBLE,    6U, off_noise);
    wr32(out, p, 0U);

    write_exif_ifd(out, exififd_off, off_dt, off_exp, off_ev, exif);
}
