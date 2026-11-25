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

/* TIFF field types */
#define TIFF_BYTE       1U
#define TIFF_ASCII      2U
#define TIFF_SHORT      3U
#define TIFF_LONG       4U
#define TIFF_RATIONAL   5U
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

void c3m1_dng_build_header(uint8_t *out,
                           const uint16_t inv_gamma_12[256],
                           uint16_t bl_g,
                           uint16_t bl_rb)
{
    /* Calibrated OV7725 ColorMatrix1 (XYZ -> Camera RGB, Q10) */
    static const int32_t s_color_matrix1[9] = {
         1094, -306,  146,
         -442, 1388,  564,
         -104,  250, 1624,
    };

    /* Exact IEEE 754 double NoiseProfile (3 planes) */
    static const uint8_t s_noise_pair[16] = {
        0x62, 0x9C, 0x59, 0x57, 0xFC, 0xFC, 0x4C, 0x3F,
        0x31, 0x48, 0x38, 0xA0, 0x26, 0xCD, 0xD2, 0x3E,
    };

    static const char s_make[]   = "CAYE";
    static const char s_model[]  = "C3M1";
    static const char s_unique[] = "CAYE C3M1";
    static const char s_dt[]     = "2026:08:17 00:00:00";

    memset(out, 0, C3M1_DNG_HEADER_SIZE);

    /* TIFF Little-Endian Header */
    out[0] = 'I';
    out[1] = 'I';
    wr16(out, 2U, 42U);
    wr32(out, 4U, 8U); /* IFD0 offset */

    const uint16_t ifd0_cnt   = 24U;
    const uint32_t ifd0_off   = 8U;
    const uint32_t subifd_off = (ifd0_off + 2U + (uint32_t)ifd0_cnt * 12U + 4U + 3U) & ~3UL; /* 304 */
    const uint16_t subifd_cnt = 24U;
    uint32_t data_pos = (subifd_off + 2U + (uint32_t)subifd_cnt * 12U + 4U + 3U) & ~3UL;    /* 600 */

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

    uint32_t off_cm1 = alloc_data(&data_pos, 9U * 8U);
    for (uint8_t i = 0; i < 9U; i++)
    {
        wr32(out, off_cm1 + (uint32_t)i * 8U + 0U, (uint32_t)s_color_matrix1[i]);
        wr32(out, off_cm1 + (uint32_t)i * 8U + 4U, 1024U);
    }

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
    wr_entry(out, &p, 0x9216U, TIFF_BYTE,      4U, 0x00000001UL);           /* TIFF/EPStandardID 1.0.0.0 */
    wr_entry(out, &p, 0xC612U, TIFF_BYTE,      4U, 0x00000401UL);           /* DNGVersion 1.4.0.0 */
    wr_entry(out, &p, 0xC613U, TIFF_BYTE,      4U, 0x00000101UL);           /* DNGBackwardVersion 1.1.0.0 */
    wr_entry(out, &p, 0xC614U, TIFF_ASCII,     sizeof(s_unique), off_unique); /* UniqueCameraModel */
    wr_entry(out, &p, 0xC621U, TIFF_SRATIONAL, 9U, off_cm1);                /* ColorMatrix1 */
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
    wr_entry(out, &p, 0xC761U, TIFF_DOUBLE,    6U, off_noise);              /* NoiseProfile */
    wr32(out, p, 0U); /* Next IFD = 0 */
}
