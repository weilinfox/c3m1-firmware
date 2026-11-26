/**
 * @file        c3m1_jpeg.c
 * @brief       Pure-integer Baseline JPEG encoder (640x480 YCbCr 4:2:0, Q=90)
 *              for STM32F103ZET6 (zero float, zero heap, 11.3 KB static strip buffer).
 */

#include <string.h>
#include "c3m1_jpeg.h"
#include "c3m1_clock.h"

#define JPEG_W          640U
#define JPEG_H          480U
#define RAW_STRIDE      644U
#define MCU_COLS        (JPEG_W / 16U)  /* 40 */
#define MCU_ROWS        (JPEG_H / 16U)  /* 30 */

/* 18-row Bayer strip buffer (1 top border + 16 active + 1 bottom border = 11,592 B .bss) */
static uint8_t s_bayer_strip[18][RAW_STRIDE] __attribute__((aligned(4)));

uint8_t *c3m1_jpeg_get_scratch_buf(uint32_t *size_out)
{
    if (size_out != NULL) *size_out = (uint32_t)sizeof(s_bayer_strip);
    return &s_bayer_strip[0][0];
}

/* Natural-order to zig-zag index map */
static const uint8_t s_zigzag[64] = {
     0,  1,  5,  6, 14, 15, 27, 28,
     2,  4,  7, 13, 16, 26, 29, 42,
     3,  8, 12, 17, 25, 30, 41, 43,
     9, 11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54,
    20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61,
    35, 36, 48, 49, 57, 58, 62, 63,
};

/* Standard ISO/IEC 10918-1 Annex K quantization tables scaled to Quality = 90 (zig-zag order) */
static const uint8_t s_qtable_y_zz[64] = {
     3,  2,  2,  3,  2,  2,  3,  3,
     3,  3,  4,  3,  3,  4,  5,  8,
     5,  5,  4,  4,  5, 10,  7,  7,
     6,  8, 12, 10, 12, 12, 11, 10,
    11, 11, 13, 14, 18, 16, 13, 14,
    17, 14, 11, 11, 16, 22, 16, 17,
    19, 20, 21, 21, 21, 12, 15, 23,
    24, 22, 20, 24, 18, 20, 21, 20,
};

static const uint8_t s_qtable_c_zz[64] = {
     3,  4,  4,  5,  4,  5,  9,  5,
     5,  9, 20, 13, 11, 13, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20,
};

/* Standard Annex K Huffman table specifications (bits[1..16] + symbol values) */
static const uint8_t s_dc_lum_bits[16] = { 0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0 };
static const uint8_t s_dc_lum_vals[12] = { 0,1,2,3,4,5,6,7,8,9,10,11 };

static const uint8_t s_dc_chr_bits[16] = { 0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0 };
static const uint8_t s_dc_chr_vals[12] = { 0,1,2,3,4,5,6,7,8,9,10,11 };

static const uint8_t s_ac_lum_bits[16] = { 0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,125 };
static const uint8_t s_ac_lum_vals[162] = {
    0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
    0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,
    0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,
    0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
    0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
    0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
    0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,
    0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,
    0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,
    0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
    0xF9,0xFA,
};

static const uint8_t s_ac_chr_bits[16] = { 0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,119 };
static const uint8_t s_ac_chr_vals[162] = {
    0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
    0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xA1,0xB1,0xC1,0x09,0x23,0x33,0x52,0xF0,
    0x15,0x62,0x72,0xD1,0x0A,0x16,0x24,0x34,0xE1,0x25,0xF1,0x17,0x18,0x19,0x1A,0x26,
    0x27,0x28,0x29,0x2A,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,
    0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,
    0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x82,0x83,0x84,0x85,0x86,0x87,
    0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,
    0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,
    0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,
    0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
    0xF9,0xFA,
};

typedef struct
{
    uint16_t code[256];
    uint8_t  len[256];
} huff_lut_t;

typedef struct
{
    const c3m1_jpeg_ctx_t *ctx;
    uint32_t out_pos;
    uint32_t total_bytes;
    uint32_t bitbuf;
    uint8_t  bitcnt;
    uint8_t  io_err;
} jstream_t;

static void build_huff_lut(const uint8_t bits[16], const uint8_t *vals, huff_lut_t *lut)
{
    memset(lut, 0, sizeof(*lut));
    uint32_t code = 0;
    uint16_t k = 0;
    for (uint8_t l = 1U; l <= 16U; l++)
    {
        for (uint8_t i = 0; i < bits[l - 1U]; i++)
        {
            uint8_t sym = vals[k++];
            lut->code[sym] = (uint16_t)code;
            lut->len[sym]  = l;
            code++;
        }
        code <<= 1;
    }
}

static void js_flush(jstream_t *js)
{
    if (js->out_pos > 0U && !js->io_err)
    {
        if (!js->ctx->write_fn(js->ctx->user, js->ctx->out_buf, js->out_pos))
        {
            js->io_err = 1U;
        }
        js->total_bytes += js->out_pos;
        js->out_pos = 0U;
    }
}

static inline void js_put_byte(jstream_t *js, uint8_t b)
{
    if (js->io_err)
    {
        return;
    }
    js->ctx->out_buf[js->out_pos++] = b;
    if (js->out_pos >= js->ctx->out_buf_size)
    {
        js_flush(js);
    }
}

static void js_put_bytes(jstream_t *js, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        js_put_byte(js, data[i]);
    }
}

static inline void js_put_bits(jstream_t *js, uint32_t bits, uint8_t nbits)
{
    js->bitbuf = (js->bitbuf << nbits) | (bits & ((1UL << nbits) - 1UL));
    js->bitcnt = (uint8_t)(js->bitcnt + nbits);
    while (js->bitcnt >= 8U)
    {
        js->bitcnt -= 8U;
        uint8_t b = (uint8_t)(js->bitbuf >> js->bitcnt);
        js_put_byte(js, b);
        if (b == 0xFFU)
        {
            js_put_byte(js, 0x00U); /* Mandatory JPEG entropy byte-stuffing */
        }
    }
}

static void js_flush_bits(jstream_t *js)
{
    if (js->bitcnt > 0U)
    {
        uint8_t pad = (uint8_t)(8U - js->bitcnt);
        js_put_bits(js, (1UL << pad) - 1UL, pad);
    }
}

static void write_dqt(jstream_t *js, uint8_t id, const uint8_t qzz[64])
{
    uint8_t hdr[5] = { 0xFFU, 0xDBU, 0x00U, 67U, id };
    js_put_bytes(js, hdr, 5U);
    js_put_bytes(js, qzz, 64U);
}

static void write_dht(jstream_t *js, uint8_t tc_th, const uint8_t bits[16],
                      const uint8_t *vals, uint16_t nvals)
{
    uint16_t len = (uint16_t)(19U + nvals);
    uint8_t hdr[5] = { 0xFFU, 0xC4U, (uint8_t)(len >> 8), (uint8_t)(len & 0xFFU), tc_th };
    js_put_bytes(js, hdr, 5U);
    js_put_bytes(js, bits, 16U);
    js_put_bytes(js, vals, nvals);
}

static void write_exif_app1(jstream_t *js)
{
    char dt[20];
    c3m1_clock_format_exif(dt);

    /* Minimal EXIF 2.32 APP1 (134 B total): Make="CAYE", Model="C3M1", DateTime/Original/Digitized */
    uint8_t app1[134] = {
        0xFF, 0xE1, 0x00, 0x84,
        'E', 'x', 'i', 'f', 0x00, 0x00,
        /* TIFF header (LE, IFD0 @ offset 8) */
        'I', 'I', 0x2A, 0x00, 0x08, 0x00, 0x00, 0x00,
        /* IFD0: 4 entries */
        0x04, 0x00,
        0x0F, 0x01, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 62, 0x00, 0x00, 0x00, /* Make (0x010F) */
        0x10, 0x01, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 68, 0x00, 0x00, 0x00, /* Model (0x0110) */
        0x32, 0x01, 0x02, 0x00, 20,   0x00, 0x00, 0x00, 74, 0x00, 0x00, 0x00, /* DateTime (0x0132) */
        0x69, 0x87, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 94, 0x00, 0x00, 0x00, /* ExifIFD (0x8769) */
        0x00, 0x00, 0x00, 0x00,
        /* Payloads @ TIFF offset 62, 68, 74 */
        'C', 'A', 'Y', 'E', 0x00, 0x00,
        'C', '3', 'M', '1', 0x00, 0x00,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        /* ExifIFD @ TIFF offset 94: 2 entries */
        0x02, 0x00,
        0x03, 0x90, 0x02, 0x00, 20, 0x00, 0x00, 0x00, 74, 0x00, 0x00, 0x00, /* DateTimeOriginal */
        0x04, 0x90, 0x02, 0x00, 20, 0x00, 0x00, 0x00, 74, 0x00, 0x00, 0x00, /* DateTimeDigitized */
        0x00, 0x00, 0x00, 0x00,
    };
    memcpy(&app1[10U + 74U], dt, 20U);
    js_put_bytes(js, app1, sizeof(app1));
}

static void write_jpeg_headers(jstream_t *js)
{
    /* SOI + APP0 (JFIF 1.01) + APP1 (EXIF DateTime) */
    static const uint8_t soi_jfif[20] = {
        0xFF, 0xD8,
        0xFF, 0xE0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00,
        0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
    };
    js_put_bytes(js, soi_jfif, sizeof(soi_jfif));
    write_exif_app1(js);

    /* Quantization tables (0=Y, 1=Cb/Cr) */
    write_dqt(js, 0U, s_qtable_y_zz);
    write_dqt(js, 1U, s_qtable_c_zz);

    /* SOF0: 640x480 Baseline DCT, YCbCr 4:2:0 */
    static const uint8_t sof0[19] = {
        0xFF, 0xC0, 0x00, 0x11, 0x08,
        (uint8_t)(JPEG_H >> 8), (uint8_t)(JPEG_H & 0xFFU),
        (uint8_t)(JPEG_W >> 8), (uint8_t)(JPEG_W & 0xFFU),
        0x03,
        0x01, 0x22, 0x00, /* Y:  2x2, QT 0 */
        0x02, 0x11, 0x01, /* Cb: 1x1, QT 1 */
        0x03, 0x11, 0x01, /* Cr: 1x1, QT 1 */
    };
    js_put_bytes(js, sof0, sizeof(sof0));

    /* Standard Annex K Huffman tables */
    write_dht(js, 0x00U, s_dc_lum_bits, s_dc_lum_vals, sizeof(s_dc_lum_vals));
    write_dht(js, 0x10U, s_ac_lum_bits, s_ac_lum_vals, sizeof(s_ac_lum_vals));
    write_dht(js, 0x01U, s_dc_chr_bits, s_dc_chr_vals, sizeof(s_dc_chr_vals));
    write_dht(js, 0x11U, s_ac_chr_bits, s_ac_chr_vals, sizeof(s_ac_chr_vals));

    /* SOS: Start of Scan */
    static const uint8_t sos[14] = {
        0xFF, 0xDA, 0x00, 0x0C, 0x03,
        0x01, 0x00, /* Y:  DC 0, AC 0 */
        0x02, 0x11, /* Cb: DC 1, AC 1 */
        0x03, 0x11, /* Cr: DC 1, AC 1 */
        0x00, 0x3F, 0x00,
    };
    js_put_bytes(js, sos, sizeof(sos));
}

/* ---------------- IJG Loeffler Pure-Integer 8x8 FDCT (CONST_BITS=13, PASS1_BITS=2) ---------------- */
#define CONST_BITS  13
#define PASS1_BITS  2
#define DESCALE(x, n) (((x) + (1L << ((n) - 1))) >> (n))

#define FIX_0_298631336  2446L
#define FIX_0_390180644  3196L
#define FIX_0_541196100  4433L
#define FIX_0_765366865  6270L
#define FIX_0_899976223  7373L
#define FIX_1_175875602  9633L
#define FIX_1_501321110  12299L
#define FIX_1_847759065  15137L
#define FIX_1_961570560  16069L
#define FIX_2_053119869  16819L
#define FIX_2_562915447  20995L
#define FIX_3_072711026  25172L

static void __attribute__((optimize("O2"))) fdct_islow(int32_t data[64])
{
    int32_t *dataptr = data;

    /* Pass 1: process rows */
    for (int ctr = 7; ctr >= 0; ctr--)
    {
        int32_t tmp0 = dataptr[0] + dataptr[7];
        int32_t tmp7 = dataptr[0] - dataptr[7];
        int32_t tmp1 = dataptr[1] + dataptr[6];
        int32_t tmp6 = dataptr[1] - dataptr[6];
        int32_t tmp2 = dataptr[2] + dataptr[5];
        int32_t tmp5 = dataptr[2] - dataptr[5];
        int32_t tmp3 = dataptr[3] + dataptr[4];
        int32_t tmp4 = dataptr[3] - dataptr[4];

        int32_t tmp10 = tmp0 + tmp3;
        int32_t tmp13 = tmp0 - tmp3;
        int32_t tmp11 = tmp1 + tmp2;
        int32_t tmp12 = tmp1 - tmp2;

        dataptr[0] = (tmp10 + tmp11) * (1L << PASS1_BITS);
        dataptr[4] = (tmp10 - tmp11) * (1L << PASS1_BITS);

        int32_t z1 = (tmp12 + tmp13) * FIX_0_541196100;
        dataptr[2] = DESCALE(z1 + tmp13 * FIX_0_765366865, CONST_BITS - PASS1_BITS);
        dataptr[6] = DESCALE(z1 - tmp12 * FIX_1_847759065, CONST_BITS - PASS1_BITS);

        z1 = tmp4 + tmp7;
        int32_t z2 = tmp5 + tmp6;
        int32_t z3 = tmp4 + tmp6;
        int32_t z4 = tmp5 + tmp7;
        int32_t z5 = (z3 + z4) * FIX_1_175875602;

        tmp4 *= FIX_0_298631336;
        tmp5 *= FIX_2_053119869;
        tmp6 *= FIX_3_072711026;
        tmp7 *= FIX_1_501321110;
        z1   *= -FIX_0_899976223;
        z2   *= -FIX_2_562915447;
        z3   *= -FIX_1_961570560;
        z4   *= -FIX_0_390180644;

        z3 += z5;
        z4 += z5;

        dataptr[7] = DESCALE(tmp4 + z1 + z3, CONST_BITS - PASS1_BITS);
        dataptr[5] = DESCALE(tmp5 + z2 + z4, CONST_BITS - PASS1_BITS);
        dataptr[3] = DESCALE(tmp6 + z2 + z3, CONST_BITS - PASS1_BITS);
        dataptr[1] = DESCALE(tmp7 + z1 + z4, CONST_BITS - PASS1_BITS);

        dataptr += 8;
    }

    /* Pass 2: process columns */
    dataptr = data;
    for (int ctr = 7; ctr >= 0; ctr--)
    {
        int32_t tmp0 = dataptr[8 * 0] + dataptr[8 * 7];
        int32_t tmp7 = dataptr[8 * 0] - dataptr[8 * 7];
        int32_t tmp1 = dataptr[8 * 1] + dataptr[8 * 6];
        int32_t tmp6 = dataptr[8 * 1] - dataptr[8 * 6];
        int32_t tmp2 = dataptr[8 * 2] + dataptr[8 * 5];
        int32_t tmp5 = dataptr[8 * 2] - dataptr[8 * 5];
        int32_t tmp3 = dataptr[8 * 3] + dataptr[8 * 4];
        int32_t tmp4 = dataptr[8 * 3] - dataptr[8 * 4];

        int32_t tmp10 = tmp0 + tmp3;
        int32_t tmp13 = tmp0 - tmp3;
        int32_t tmp11 = tmp1 + tmp2;
        int32_t tmp12 = tmp1 - tmp2;

        dataptr[8 * 0] = DESCALE(tmp10 + tmp11, PASS1_BITS);
        dataptr[8 * 4] = DESCALE(tmp10 - tmp11, PASS1_BITS);

        int32_t z1 = (tmp12 + tmp13) * FIX_0_541196100;
        dataptr[8 * 2] = DESCALE(z1 + tmp13 * FIX_0_765366865, CONST_BITS + PASS1_BITS);
        dataptr[8 * 6] = DESCALE(z1 - tmp12 * FIX_1_847759065, CONST_BITS + PASS1_BITS);

        z1 = tmp4 + tmp7;
        int32_t z2 = tmp5 + tmp6;
        int32_t z3 = tmp4 + tmp6;
        int32_t z4 = tmp5 + tmp7;
        int32_t z5 = (z3 + z4) * FIX_1_175875602;

        tmp4 *= FIX_0_298631336;
        tmp5 *= FIX_2_053119869;
        tmp6 *= FIX_3_072711026;
        tmp7 *= FIX_1_501321110;
        z1   *= -FIX_0_899976223;
        z2   *= -FIX_2_562915447;
        z3   *= -FIX_1_961570560;
        z4   *= -FIX_0_390180644;

        z3 += z5;
        z4 += z5;

        dataptr[8 * 7] = DESCALE(tmp4 + z1 + z3, CONST_BITS + PASS1_BITS);
        dataptr[8 * 5] = DESCALE(tmp5 + z2 + z4, CONST_BITS + PASS1_BITS);
        dataptr[8 * 3] = DESCALE(tmp6 + z2 + z3, CONST_BITS + PASS1_BITS);
        dataptr[8 * 1] = DESCALE(tmp7 + z1 + z4, CONST_BITS + PASS1_BITS);

        dataptr++;
    }
}

static inline uint8_t vli_bits(int32_t val, uint32_t *out_code)
{
    if (val == 0)
    {
        *out_code = 0U;
        return 0U;
    }
    int32_t mag = (val < 0) ? -val : val;
    uint8_t n = 0U;
    int32_t t = mag;
    while (t > 0)
    {
        n++;
        t >>= 1;
    }
    *out_code = (val < 0) ? (uint32_t)(val + ((1L << n) - 1L)) : (uint32_t)val;
    return n;
}

static void __attribute__((optimize("O2"))) encode_block(jstream_t *js,
                                                         int32_t blk[64],
                                                         const uint8_t qzz[64],
                                                         const huff_lut_t *dc_lut,
                                                         const huff_lut_t *ac_lut,
                                                         int32_t *prev_dc)
{
    fdct_islow(blk);

    /* Quantize into zig-zag order (fdct_islow output is scaled by 8x) */
    int32_t zz[64];
    for (uint8_t k = 0; k < 64U; k++)
    {
        int32_t v = blk[k];
        int32_t q8 = (int32_t)qzz[s_zigzag[k]] << 3;
        int32_t half = q8 >> 1;
        zz[s_zigzag[k]] = (v >= 0) ? ((v + half) / q8) : -((-v + half) / q8);
    }

    /* Encode DC difference */
    int32_t diff = zz[0] - *prev_dc;
    *prev_dc = zz[0];
    uint32_t vcode = 0U;
    uint8_t  vlen  = vli_bits(diff, &vcode);
    js_put_bits(js, dc_lut->code[vlen], dc_lut->len[vlen]);
    if (vlen > 0U)
    {
        js_put_bits(js, vcode, vlen);
    }

    /* Encode AC coefficients with RLE + EOB */
    uint8_t last_nz = 63U;
    while (last_nz > 0U && zz[last_nz] == 0)
    {
        last_nz--;
    }

    uint8_t run = 0U;
    for (uint8_t k = 1U; k <= last_nz; k++)
    {
        int32_t ac = zz[k];
        if (ac == 0)
        {
            run++;
            if (run == 16U)
            {
                js_put_bits(js, ac_lut->code[0xF0U], ac_lut->len[0xF0U]); /* ZRL */
                run = 0U;
            }
        }
        else
        {
            vlen = vli_bits(ac, &vcode);
            uint8_t sym = (uint8_t)((run << 4) | vlen);
            js_put_bits(js, ac_lut->code[sym], ac_lut->len[sym]);
            js_put_bits(js, vcode, vlen);
            run = 0U;
        }
    }

    if (last_nz < 63U)
    {
        js_put_bits(js, ac_lut->code[0x00U], ac_lut->len[0x00U]); /* EOB */
    }
}

/* ---------------- Calibrated Color Pipeline (Shared by JPEG & DNG Preview) ---------------- */
static inline uint8_t lin12_to_srgb8(int32_t v12, const uint8_t srgb_lut[256])
{
    if (v12 <= 0)    return 0U;
    if (v12 >= 4064) return 255U;
    uint32_t idx  = (uint32_t)v12 >> 4;
    uint32_t frac = (uint32_t)v12 & 15UL;
    uint32_t s0   = srgb_lut[idx];
    uint32_t s1   = srgb_lut[idx + 1U];
    return (uint8_t)(s0 + (((s1 - s0) * frac + 8UL) >> 4));
}

static inline void c3m1_apply_ccm(int32_t rl, int32_t gl, int32_t bl, int32_t rgb[3])
{
    /*
     * Calibrated OV7725 crosstalk inverse + natural-vivid saturation (Q8, row sums == 256):
     * counters OV7725 broad CFA spectral overlap for lively, true-to-life color on IPS & JPEG
     * while keeping neutral grays (rl == gl == bl) strictly invariant.
     */
    rgb[0] = (384 * rl -  92 * gl -  36 * bl + 128) >> 8;
    rgb[1] = (-64 * rl + 380 * gl -  60 * bl + 128) >> 8;
    rgb[2] = (-20 * rl - 104 * gl + 380 * bl + 128) >> 8;
}

void __attribute__((optimize("O2"))) c3m1_color_cam12_to_srgb8(uint16_t r12, uint16_t g12, uint16_t b12,
                                                               uint16_t wb_r_q8, uint16_t wb_b_q8,
                                                               const uint8_t srgb_lut[256],
                                                               uint8_t *out_r, uint8_t *out_g, uint8_t *out_b)
{
    int32_t rl = (int32_t)(((uint32_t)r12 * (uint32_t)wb_r_q8) >> 8);
    int32_t gl = (int32_t)g12;
    int32_t bl = (int32_t)(((uint32_t)b12 * (uint32_t)wb_b_q8) >> 8);
    if (rl > 4064) rl = 4064;
    if (gl > 4064) gl = 4064;
    if (bl > 4064) bl = 4064;

    int32_t rgb[3];
    c3m1_apply_ccm(rl, gl, bl, rgb);
    *out_r = lin12_to_srgb8(rgb[0], srgb_lut);
    *out_g = lin12_to_srgb8(rgb[1], srgb_lut);
    *out_b = lin12_to_srgb8(rgb[2], srgb_lut);
}

void __attribute__((optimize("O2"))) c3m1_color_hdr_to_srgb8(uint16_t r16, uint16_t g16, uint16_t b16,
                                                             uint16_t wb_r_q8, uint16_t wb_b_q8,
                                                             uint32_t ref_white, uint32_t white_level,
                                                             const uint8_t srgb_lut[256],
                                                             uint8_t *out_r, uint8_t *out_g, uint8_t *out_b)
{
    if (ref_white == 0UL || white_level == 0UL)
    {
        *out_r = 0U;
        *out_g = 0U;
        *out_b = 0U;
        return;
    }

    /*
     * Clamp white-balanced channels at the sensor's neutral saturation ceiling
     * (white_level * min(wb_r, 1.0, wb_b)) BEFORE CCM, matching single-frame DNG
     * and ACR so clipped highlights stay pure white instead of tinting toward
     * the high-gain WB channel (typically blue/purple when wb_b > 1.0).
     */
    uint32_t min_wb = 256UL;
    if ((uint32_t)wb_r_q8 < min_wb) min_wb = (uint32_t)wb_r_q8;
    if ((uint32_t)wb_b_q8 < min_wb) min_wb = (uint32_t)wb_b_q8;
    int32_t wb_ceil = (int32_t)((white_level * min_wb) >> 8);
    if (wb_ceil < 1) wb_ceil = 1;

    int32_t rl = (int32_t)(((uint32_t)r16 * (uint32_t)wb_r_q8) >> 8);
    int32_t gl = (int32_t)g16;
    int32_t bl = (int32_t)(((uint32_t)b16 * (uint32_t)wb_b_q8) >> 8);
    if (rl > wb_ceil) rl = wb_ceil;
    if (gl > wb_ceil) gl = wb_ceil;
    if (bl > wb_ceil) bl = wb_ceil;

    int32_t rgb[3];
    c3m1_apply_ccm(rl, gl, bl, rgb);
    for (uint8_t ch = 0U; ch < 3U; ch++)
    {
        if (rgb[ch] < 0)
        {
            rgb[ch] = 0;
        }
    }

    uint32_t peak = (uint32_t)((rgb[0] > rgb[1]) ? rgb[0] : rgb[1]);
    if ((uint32_t)rgb[2] > peak)
    {
        peak = (uint32_t)rgb[2];
    }
    if (peak == 0UL)
    {
        *out_r = 0U;
        *out_g = 0U;
        *out_b = 0U;
        return;
    }

    /* Map 0 EV white to 4064; compress HDR highlight headroom [2400..x_max] -> [2400..4064] with shared RGB scale */
    uint32_t x_max = (uint32_t)(((uint64_t)(uint32_t)wb_ceil * 4064ULL + (uint64_t)(ref_white >> 1)) /
                                (uint64_t)ref_white);
    uint32_t level = (uint32_t)(((uint64_t)peak * 4064ULL + (uint64_t)(ref_white >> 1)) / (uint64_t)ref_white);
    if (level > 2400UL)
    {
        if (x_max <= 4064UL)
        {
            if (level > 4064UL)
            {
                level = 4064UL;
            }
        }
        else
        {
            uint32_t span = x_max - 2400UL;
            uint32_t d    = (level >= x_max) ? span : (level - 2400UL);
            uint64_t den  = (uint64_t)span * (uint64_t)(d + 1664UL);
            uint64_t num  = 1664ULL * (uint64_t)d * (uint64_t)(span + 1664UL);
            level = 2400UL + (uint32_t)((num + (den >> 1)) / den);
            if (level > 4064UL)
            {
                level = 4064UL;
            }
        }
    }

    int32_t r12 = (int32_t)(((uint32_t)rgb[0] * level + (peak >> 1)) / peak);
    int32_t g12 = (int32_t)(((uint32_t)rgb[1] * level + (peak >> 1)) / peak);
    int32_t b12 = (int32_t)(((uint32_t)rgb[2] * level + (peak >> 1)) / peak);
    *out_r = lin12_to_srgb8(r12, srgb_lut);
    *out_g = lin12_to_srgb8(g12, srgb_lut);
    *out_b = lin12_to_srgb8(b12, srgb_lut);
}

/* ---------------- 3x3 Bilinear GRBG Bayer Demosaic + Streaming VGA JPEG ---------------- */
static inline uint16_t lin_sub(uint16_t v12, uint16_t bl)
{
    return (v12 > bl) ? (uint16_t)(v12 - bl) : 0U;
}

uint8_t __attribute__((optimize("O2"))) c3m1_jpeg_encode_bayer_vga(const c3m1_jpeg_ctx_t *ctx,
                                                                   uint32_t *out_total_bytes)
{
    huff_lut_t dc_lum, ac_lum, dc_chr, ac_chr;
    build_huff_lut(s_dc_lum_bits, s_dc_lum_vals, &dc_lum);
    build_huff_lut(s_ac_lum_bits, s_ac_lum_vals, &ac_lum);
    build_huff_lut(s_dc_chr_bits, s_dc_chr_vals, &dc_chr);
    build_huff_lut(s_ac_chr_bits, s_ac_chr_vals, &ac_chr);

    jstream_t js;
    memset(&js, 0, sizeof(js));
    js.ctx = ctx;

    write_jpeg_headers(&js);

    const uint16_t *const lin   = ctx->inv_gamma_12;
    const uint8_t  *const srgb  = ctx->srgb_lut;
    const uint16_t        bl_g  = ctx->bl_g;
    const uint16_t        bl_rb = ctx->bl_rb;
    const uint16_t        wbr   = ctx->wb_r_q8;
    const uint16_t        wbb   = ctx->wb_b_q8;

    int32_t prev_dc_y = 0, prev_dc_cb = 0, prev_dc_cr = 0;

    for (uint8_t mcu_y = 0; mcu_y < MCU_ROWS; mcu_y++)
    {
        /* Fill 18-row sliding Bayer strip (row 0 = top border, rows 1..16 = active, row 17 = bottom border) */
        if (mcu_y == 0U)
        {
            for (uint8_t r = 1U; r <= 17U; r++)
            {
                ctx->read_row_fn(ctx->user, s_bayer_strip[r]);
            }
            /* Mirror row 2 into row 0 (same even/odd GRBG parity as row -1 -> row +1) */
            memcpy(s_bayer_strip[0], s_bayer_strip[2], RAW_STRIDE);
        }
        else if (mcu_y < (MCU_ROWS - 1U))
        {
            memcpy(s_bayer_strip[0], s_bayer_strip[16], RAW_STRIDE);
            memcpy(s_bayer_strip[1], s_bayer_strip[17], RAW_STRIDE);
            for (uint8_t r = 2U; r <= 17U; r++)
            {
                ctx->read_row_fn(ctx->user, s_bayer_strip[r]);
            }
        }
        else
        {
            /* Last strip (rows 464..479): 15 raw rows left in FIFO */
            memcpy(s_bayer_strip[0], s_bayer_strip[16], RAW_STRIDE);
            memcpy(s_bayer_strip[1], s_bayer_strip[17], RAW_STRIDE);
            for (uint8_t r = 2U; r <= 16U; r++)
            {
                ctx->read_row_fn(ctx->user, s_bayer_strip[r]);
            }
            /* Mirror row 15 into row 17 (preserves even/odd GRBG parity) */
            memcpy(s_bayer_strip[17], s_bayer_strip[15], RAW_STRIDE);
        }

        for (uint8_t mcu_x = 0; mcu_x < MCU_COLS; mcu_x++)
        {
            int32_t y_blk[4][64];
            int32_t cb_blk[64];
            int32_t cr_blk[64];
            memset(cb_blk, 0, sizeof(cb_blk));
            memset(cr_blk, 0, sizeof(cr_blk));

            const uint16_t x0 = (uint16_t)mcu_x * 16U;

            for (uint8_t dy = 0; dy < 16U; dy++)
            {
                const uint8_t *rp = s_bayer_strip[dy];      /* y - 1 */
                const uint8_t *rc = s_bayer_strip[dy + 1U]; /* y     */
                const uint8_t *rn = s_bayer_strip[dy + 2U]; /* y + 1 */
                const uint8_t  even_y = (uint8_t)((dy & 1U) == 0U);
                const uint8_t  y_sub_hi = (dy >= 8U) ? 2U : 0U;
                const uint8_t  by = (uint8_t)(dy & 7U);
                const uint8_t  cy = (uint8_t)(dy >> 1);

                for (uint8_t dx = 0; dx < 16U; dx++)
                {
                    const uint16_t x  = (uint16_t)(x0 + dx);
                    const uint16_t xm = (x > 0U) ? (uint16_t)(x - 1U) : 1U;
                    const uint16_t xp = (uint16_t)(x + 1U); /* x <= 639 -> xp <= 640 < 644 */
                    const uint8_t  even_x = (uint8_t)((dx & 1U) == 0U);

                    uint16_t r12, g12, b12;
                    /* GRBG layout: (even_y,even_x)=G, (even_y,odd_x)=R, (odd_y,even_x)=B, (odd_y,odd_x)=G */
                    if (even_y)
                    {
                        if (even_x)
                        {
                            /* G site */
                            g12 = lin[rc[x]];
                            r12 = (uint16_t)(((uint32_t)lin[rc[xm]] + lin[rc[xp]]) >> 1);
                            b12 = (uint16_t)(((uint32_t)lin[rp[x]]  + lin[rn[x]])  >> 1);
                        }
                        else
                        {
                            /* R site */
                            r12 = lin[rc[x]];
                            g12 = (uint16_t)(((uint32_t)lin[rc[xm]] + lin[rc[xp]] + lin[rp[x]] + lin[rn[x]]) >> 2);
                            b12 = (uint16_t)(((uint32_t)lin[rp[xm]] + lin[rp[xp]] + lin[rn[xm]] + lin[rn[xp]]) >> 2);
                        }
                    }
                    else
                    {
                        if (even_x)
                        {
                            /* B site */
                            b12 = lin[rc[x]];
                            g12 = (uint16_t)(((uint32_t)lin[rc[xm]] + lin[rc[xp]] + lin[rp[x]] + lin[rn[x]]) >> 2);
                            r12 = (uint16_t)(((uint32_t)lin[rp[xm]] + lin[rp[xp]] + lin[rn[xm]] + lin[rn[xp]]) >> 2);
                        }
                        else
                        {
                            /* G site */
                            g12 = lin[rc[x]];
                            b12 = (uint16_t)(((uint32_t)lin[rc[xm]] + lin[rc[xp]]) >> 1);
                            r12 = (uint16_t)(((uint32_t)lin[rp[x]]  + lin[rn[x]])  >> 1);
                        }
                    }

                    uint8_t sr, sg, sb;
                    c3m1_color_cam12_to_srgb8(lin_sub(r12, bl_rb), lin_sub(g12, bl_g), lin_sub(b12, bl_rb),
                                              wbr, wbb, srgb, &sr, &sg, &sb);

                    int32_t yv  = (( 77 * (int32_t)sr + 150 * (int32_t)sg +  29 * (int32_t)sb + 128) >> 8) - 128;
                    int32_t cbv = ((-43 * (int32_t)sr -  85 * (int32_t)sg + 128 * (int32_t)sb + 128) >> 8);
                    int32_t crv = ((128 * (int32_t)sr - 107 * (int32_t)sg -  21 * (int32_t)sb + 128) >> 8);

                    uint8_t blk_idx = (uint8_t)(y_sub_hi | ((dx >= 8U) ? 1U : 0U));
                    y_blk[blk_idx][(uint8_t)(by * 8U + (dx & 7U))] = yv;

                    uint8_t c_idx = (uint8_t)(cy * 8U + (dx >> 1));
                    cb_blk[c_idx] += cbv;
                    cr_blk[c_idx] += crv;
                }
            }

            for (uint8_t i = 0; i < 64U; i++)
            {
                cb_blk[i] >>= 2;
                cr_blk[i] >>= 2;
            }

            encode_block(&js, y_blk[0], s_qtable_y_zz, &dc_lum, &ac_lum, &prev_dc_y);
            encode_block(&js, y_blk[1], s_qtable_y_zz, &dc_lum, &ac_lum, &prev_dc_y);
            encode_block(&js, y_blk[2], s_qtable_y_zz, &dc_lum, &ac_lum, &prev_dc_y);
            encode_block(&js, y_blk[3], s_qtable_y_zz, &dc_lum, &ac_lum, &prev_dc_y);
            encode_block(&js, cb_blk,   s_qtable_c_zz, &dc_chr, &ac_chr, &prev_dc_cb);
            encode_block(&js, cr_blk,   s_qtable_c_zz, &dc_chr, &ac_chr, &prev_dc_cr);

            if (js.io_err)
            {
                return 0U;
            }
        }
    }

    js_flush_bits(&js);
    js_put_byte(&js, 0xFFU);
    js_put_byte(&js, 0xD9U); /* EOI */
    js_flush(&js);

    if (out_total_bytes != NULL)
    {
        *out_total_bytes = js.total_bytes;
    }
    return js.io_err ? 0U : 1U;
}

