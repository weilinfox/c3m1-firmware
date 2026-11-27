/**
 * @file        c3m1_jpeg.c
 * @brief       Pure-integer Baseline JPEG encoder (640x480 YCbCr 4:2:0, Q=90)
 *              for STM32F103ZET6 (zero float, zero heap, 11.3 KB static strip buffer).
 */

#pragma GCC optimize("O2")

#include <string.h>
#include "c3m1_jpeg.h"
#include "c3m1_clock.h"

#define JPEG_W          640U
#define JPEG_H          480U
#define RAW_STRIDE      644U
#define MCU_COLS        (JPEG_W / 16U)  /* 40 */
#define MCU_ROWS        (JPEG_H / 16U)  /* 30 */
#define BAYER_RING_ROWS 50U             /* Covers full WarpRectilinear kr1 in [-0.200, +0.200] (max span 48) */

/* 50-row shared Bayer ring buffer (32,200 B .bss; reused by JPEG, DNG thumb, 1X viewfinder, Playback) */
static uint8_t s_bayer_strip[BAYER_RING_ROWS][RAW_STRIDE] __attribute__((aligned(4)));

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

/*
 * Natural-order Q16 reciprocal & half-step LUTs in Flash (.rodata) for zero-division quantization:
 *   q8 = q * 8, half = q * 4, recip = ceil(65536 / q)
 *   (((mag + half) >> 3) * recip) >> 16 == (mag + half) / q8 (verified bit-exact for mag in [0..16384]).
 */
static const uint16_t s_qrecip_y_nat[64] = {
    21846U, 32768U, 32768U, 21846U, 13108U,  8192U,  6554U,  5462U,
    32768U, 32768U, 21846U, 16384U, 13108U,  5462U,  5462U,  5958U,
    21846U, 21846U, 21846U, 13108U,  8192U,  5958U,  4682U,  5958U,
    21846U, 21846U, 16384U, 10923U,  6554U,  3856U,  4096U,  5462U,
    16384U, 16384U,  9363U,  5958U,  4682U,  2979U,  3121U,  4370U,
    13108U,  9363U,  5958U,  5042U,  4096U,  3121U,  2850U,  3641U,
     6554U,  5042U,  4096U,  3856U,  3121U,  2731U,  2731U,  3277U,
     4682U,  3641U,  3450U,  3277U,  2979U,  3277U,  3121U,  3277U,
};

static const uint8_t s_qhalf_y_nat[64] = {
    12U,  8U,  8U, 12U, 20U, 32U, 40U, 48U,
     8U,  8U, 12U, 16U, 20U, 48U, 48U, 44U,
    12U, 12U, 12U, 20U, 32U, 44U, 56U, 44U,
    12U, 12U, 16U, 24U, 40U, 68U, 64U, 48U,
    16U, 16U, 28U, 44U, 56U, 88U, 84U, 60U,
    20U, 28U, 44U, 52U, 64U, 84U, 92U, 72U,
    40U, 52U, 64U, 68U, 84U, 96U, 96U, 80U,
    56U, 72U, 76U, 80U, 88U, 80U, 84U, 80U,
};

static const uint16_t s_qrecip_c_nat[64] = {
    21846U, 16384U, 13108U,  7282U,  3277U,  3277U,  3277U,  3277U,
    16384U, 16384U, 13108U,  5042U,  3277U,  3277U,  3277U,  3277U,
    13108U, 13108U,  5958U,  3277U,  3277U,  3277U,  3277U,  3277U,
     7282U,  5042U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,
     3277U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,
     3277U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,
     3277U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,
     3277U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,  3277U,
};

static const uint8_t s_qhalf_c_nat[64] = {
    12U, 16U, 20U, 36U, 80U, 80U, 80U, 80U,
    16U, 16U, 20U, 52U, 80U, 80U, 80U, 80U,
    20U, 20U, 44U, 80U, 80U, 80U, 80U, 80U,
    36U, 52U, 80U, 80U, 80U, 80U, 80U, 80U,
    80U, 80U, 80U, 80U, 80U, 80U, 80U, 80U,
    80U, 80U, 80U, 80U, 80U, 80U, 80U, 80U,
    80U, 80U, 80U, 80U, 80U, 80U, 80U, 80U,
    80U, 80U, 80U, 80U, 80U, 80U, 80U, 80U,
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
    uint16_t code[12];
    uint8_t  len[12];
} huff_dc_lut_t;

typedef struct
{
    uint16_t code[256];
    uint8_t  len[256];
} huff_ac_lut_t;

/* Precomputed Annex K Huffman LUTs in Flash (.rodata): saves 3,072 B stack and 0 B RAM */
static const huff_dc_lut_t s_huff_dc_lum = {
    .code = {
        0x0000U, 0x0002U, 0x0003U, 0x0004U, 0x0005U, 0x0006U, 0x000EU, 0x001EU, 0x003EU, 0x007EU, 0x00FEU, 0x01FEU,
    },
    .len = {
         2U,  3U,  3U,  3U,  3U,  3U,  4U,  5U,  6U,  7U,  8U,  9U,
    },
};

static const huff_dc_lut_t s_huff_dc_chr = {
    .code = {
        0x0000U, 0x0001U, 0x0002U, 0x0006U, 0x000EU, 0x001EU, 0x003EU, 0x007EU, 0x00FEU, 0x01FEU, 0x03FEU, 0x07FEU,
    },
    .len = {
         2U,  2U,  2U,  3U,  4U,  5U,  6U,  7U,  8U,  9U, 10U, 11U,
    },
};

static const huff_ac_lut_t s_huff_ac_lum = {
    .code = {
        0x000AU, 0x0000U, 0x0001U, 0x0004U, 0x000BU, 0x001AU, 0x0078U, 0x00F8U,
        0x03F6U, 0xFF82U, 0xFF83U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x000CU, 0x001BU, 0x0079U, 0x01F6U, 0x07F6U, 0xFF84U, 0xFF85U,
        0xFF86U, 0xFF87U, 0xFF88U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x001CU, 0x00F9U, 0x03F7U, 0x0FF4U, 0xFF89U, 0xFF8AU, 0xFF8BU,
        0xFF8CU, 0xFF8DU, 0xFF8EU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x003AU, 0x01F7U, 0x0FF5U, 0xFF8FU, 0xFF90U, 0xFF91U, 0xFF92U,
        0xFF93U, 0xFF94U, 0xFF95U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x003BU, 0x03F8U, 0xFF96U, 0xFF97U, 0xFF98U, 0xFF99U, 0xFF9AU,
        0xFF9BU, 0xFF9CU, 0xFF9DU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x007AU, 0x07F7U, 0xFF9EU, 0xFF9FU, 0xFFA0U, 0xFFA1U, 0xFFA2U,
        0xFFA3U, 0xFFA4U, 0xFFA5U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x007BU, 0x0FF6U, 0xFFA6U, 0xFFA7U, 0xFFA8U, 0xFFA9U, 0xFFAAU,
        0xFFABU, 0xFFACU, 0xFFADU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x00FAU, 0x0FF7U, 0xFFAEU, 0xFFAFU, 0xFFB0U, 0xFFB1U, 0xFFB2U,
        0xFFB3U, 0xFFB4U, 0xFFB5U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x01F8U, 0x7FC0U, 0xFFB6U, 0xFFB7U, 0xFFB8U, 0xFFB9U, 0xFFBAU,
        0xFFBBU, 0xFFBCU, 0xFFBDU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x01F9U, 0xFFBEU, 0xFFBFU, 0xFFC0U, 0xFFC1U, 0xFFC2U, 0xFFC3U,
        0xFFC4U, 0xFFC5U, 0xFFC6U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x01FAU, 0xFFC7U, 0xFFC8U, 0xFFC9U, 0xFFCAU, 0xFFCBU, 0xFFCCU,
        0xFFCDU, 0xFFCEU, 0xFFCFU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x03F9U, 0xFFD0U, 0xFFD1U, 0xFFD2U, 0xFFD3U, 0xFFD4U, 0xFFD5U,
        0xFFD6U, 0xFFD7U, 0xFFD8U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x03FAU, 0xFFD9U, 0xFFDAU, 0xFFDBU, 0xFFDCU, 0xFFDDU, 0xFFDEU,
        0xFFDFU, 0xFFE0U, 0xFFE1U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x07F8U, 0xFFE2U, 0xFFE3U, 0xFFE4U, 0xFFE5U, 0xFFE6U, 0xFFE7U,
        0xFFE8U, 0xFFE9U, 0xFFEAU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0xFFEBU, 0xFFECU, 0xFFEDU, 0xFFEEU, 0xFFEFU, 0xFFF0U, 0xFFF1U,
        0xFFF2U, 0xFFF3U, 0xFFF4U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x07F9U, 0xFFF5U, 0xFFF6U, 0xFFF7U, 0xFFF8U, 0xFFF9U, 0xFFFAU, 0xFFFBU,
        0xFFFCU, 0xFFFDU, 0xFFFEU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
    },
    .len = {
         4U,  2U,  2U,  3U,  4U,  5U,  7U,  8U, 10U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  4U,  5U,  7U,  9U, 11U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  5U,  8U, 10U, 12U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  6U,  9U, 12U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  6U, 10U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  7U, 11U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  7U, 12U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  8U, 12U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  9U, 15U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  9U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  9U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U, 10U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U, 10U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U, 11U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
        11U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
    },
};

static const huff_ac_lut_t s_huff_ac_chr = {
    .code = {
        0x0000U, 0x0001U, 0x0004U, 0x000AU, 0x0018U, 0x0019U, 0x0038U, 0x0078U,
        0x01F4U, 0x03F6U, 0x0FF4U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x000BU, 0x0039U, 0x00F6U, 0x01F5U, 0x07F6U, 0x0FF5U, 0xFF88U,
        0xFF89U, 0xFF8AU, 0xFF8BU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x001AU, 0x00F7U, 0x03F7U, 0x0FF6U, 0x7FC2U, 0xFF8CU, 0xFF8DU,
        0xFF8EU, 0xFF8FU, 0xFF90U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x001BU, 0x00F8U, 0x03F8U, 0x0FF7U, 0xFF91U, 0xFF92U, 0xFF93U,
        0xFF94U, 0xFF95U, 0xFF96U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x003AU, 0x01F6U, 0xFF97U, 0xFF98U, 0xFF99U, 0xFF9AU, 0xFF9BU,
        0xFF9CU, 0xFF9DU, 0xFF9EU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x003BU, 0x03F9U, 0xFF9FU, 0xFFA0U, 0xFFA1U, 0xFFA2U, 0xFFA3U,
        0xFFA4U, 0xFFA5U, 0xFFA6U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x0079U, 0x07F7U, 0xFFA7U, 0xFFA8U, 0xFFA9U, 0xFFAAU, 0xFFABU,
        0xFFACU, 0xFFADU, 0xFFAEU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x007AU, 0x07F8U, 0xFFAFU, 0xFFB0U, 0xFFB1U, 0xFFB2U, 0xFFB3U,
        0xFFB4U, 0xFFB5U, 0xFFB6U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x00F9U, 0xFFB7U, 0xFFB8U, 0xFFB9U, 0xFFBAU, 0xFFBBU, 0xFFBCU,
        0xFFBDU, 0xFFBEU, 0xFFBFU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x01F7U, 0xFFC0U, 0xFFC1U, 0xFFC2U, 0xFFC3U, 0xFFC4U, 0xFFC5U,
        0xFFC6U, 0xFFC7U, 0xFFC8U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x01F8U, 0xFFC9U, 0xFFCAU, 0xFFCBU, 0xFFCCU, 0xFFCDU, 0xFFCEU,
        0xFFCFU, 0xFFD0U, 0xFFD1U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x01F9U, 0xFFD2U, 0xFFD3U, 0xFFD4U, 0xFFD5U, 0xFFD6U, 0xFFD7U,
        0xFFD8U, 0xFFD9U, 0xFFDAU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x01FAU, 0xFFDBU, 0xFFDCU, 0xFFDDU, 0xFFDEU, 0xFFDFU, 0xFFE0U,
        0xFFE1U, 0xFFE2U, 0xFFE3U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x07F9U, 0xFFE4U, 0xFFE5U, 0xFFE6U, 0xFFE7U, 0xFFE8U, 0xFFE9U,
        0xFFEAU, 0xFFEBU, 0xFFECU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x0000U, 0x3FE0U, 0xFFEDU, 0xFFEEU, 0xFFEFU, 0xFFF0U, 0xFFF1U, 0xFFF2U,
        0xFFF3U, 0xFFF4U, 0xFFF5U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
        0x03FAU, 0x7FC3U, 0xFFF6U, 0xFFF7U, 0xFFF8U, 0xFFF9U, 0xFFFAU, 0xFFFBU,
        0xFFFCU, 0xFFFDU, 0xFFFEU, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U,
    },
    .len = {
         2U,  2U,  3U,  4U,  5U,  5U,  6U,  7U,  9U, 10U, 12U,  0U,  0U,  0U,  0U,  0U,
         0U,  4U,  6U,  8U,  9U, 11U, 12U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  5U,  8U, 10U, 12U, 15U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  5U,  8U, 10U, 12U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  6U,  9U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  6U, 10U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  7U, 11U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  7U, 11U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  8U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  9U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  9U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  9U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U,  9U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U, 11U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
         0U, 14U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
        10U, 15U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U, 16U,  0U,  0U,  0U,  0U,  0U,
    },
};

/* Zero-division row % 50 lookup in Flash (.rodata): saves 480 B stack */
#define MOD50_BLOCK \
     0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,10U,11U,12U,13U,14U,15U,16U,17U,18U,19U,20U,21U,22U,23U,24U, \
    25U,26U,27U,28U,29U,30U,31U,32U,33U,34U,35U,36U,37U,38U,39U,40U,41U,42U,43U,44U,45U,46U,47U,48U,49U
static const uint8_t s_mod50[JPEG_H] = {
    MOD50_BLOCK, MOD50_BLOCK, MOD50_BLOCK, MOD50_BLOCK, MOD50_BLOCK,
    MOD50_BLOCK, MOD50_BLOCK, MOD50_BLOCK, MOD50_BLOCK,
     0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,10U,11U,12U,13U,14U,15U,16U,17U,18U,19U,20U,21U,22U,23U,24U,
    25U,26U,27U,28U,29U,
};
#undef MOD50_BLOCK

/* Static row pointer table into s_bayer_strip in Flash (.rodata): saves 200 B stack */
static const uint8_t *const s_ring_rows[BAYER_RING_ROWS] = {
    s_bayer_strip[ 0], s_bayer_strip[ 1], s_bayer_strip[ 2], s_bayer_strip[ 3], s_bayer_strip[ 4],
    s_bayer_strip[ 5], s_bayer_strip[ 6], s_bayer_strip[ 7], s_bayer_strip[ 8], s_bayer_strip[ 9],
    s_bayer_strip[10], s_bayer_strip[11], s_bayer_strip[12], s_bayer_strip[13], s_bayer_strip[14],
    s_bayer_strip[15], s_bayer_strip[16], s_bayer_strip[17], s_bayer_strip[18], s_bayer_strip[19],
    s_bayer_strip[20], s_bayer_strip[21], s_bayer_strip[22], s_bayer_strip[23], s_bayer_strip[24],
    s_bayer_strip[25], s_bayer_strip[26], s_bayer_strip[27], s_bayer_strip[28], s_bayer_strip[29],
    s_bayer_strip[30], s_bayer_strip[31], s_bayer_strip[32], s_bayer_strip[33], s_bayer_strip[34],
    s_bayer_strip[35], s_bayer_strip[36], s_bayer_strip[37], s_bayer_strip[38], s_bayer_strip[39],
    s_bayer_strip[40], s_bayer_strip[41], s_bayer_strip[42], s_bayer_strip[43], s_bayer_strip[44],
    s_bayer_strip[45], s_bayer_strip[46], s_bayer_strip[47], s_bayer_strip[48], s_bayer_strip[49],
};

typedef struct
{
    const c3m1_jpeg_ctx_t *ctx;
    uint32_t out_pos;
    uint32_t total_bytes;
    uint32_t bitbuf;
    uint8_t  bitcnt;
    uint8_t  io_err;
} jstream_t;

static void js_flush(jstream_t *js)
{
    if (js->out_pos > 0U && !js->io_err)
    {
        if (js->ctx->write_fn(js->ctx->user, js->ctx->out_buf, js->out_pos))
        {
            js->total_bytes += js->out_pos;
        }
        else
        {
            js->io_err = 1U;
        }
        js->out_pos = 0U;
    }
}

static inline __attribute__((always_inline)) void js_put_byte(jstream_t *js, uint8_t b)
{
    if (__builtin_expect(js->io_err, 0))
    {
        return;
    }
    js->ctx->out_buf[js->out_pos++] = b;
    if (__builtin_expect(js->out_pos >= js->ctx->out_buf_size, 0))
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

static inline __attribute__((always_inline)) void js_put_bits(jstream_t *js, uint32_t bits, uint8_t nbits)
{
    js->bitbuf = (js->bitbuf << nbits) | (bits & ((1UL << nbits) - 1UL));
    js->bitcnt = (uint8_t)(js->bitcnt + nbits);
    while (js->bitcnt >= 8U)
    {
        js->bitcnt -= 8U;
        uint8_t b = (uint8_t)(js->bitbuf >> js->bitcnt);
        js_put_byte(js, b);
        if (__builtin_expect(b == 0xFFU, 0))
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

static void exif_wr16(uint8_t *b, uint32_t off, uint16_t v)
{
    b[off]      = (uint8_t)(v & 0xFFU);
    b[off + 1U] = (uint8_t)(v >> 8);
}

static void exif_wr32(uint8_t *b, uint32_t off, uint32_t v)
{
    b[off]      = (uint8_t)(v & 0xFFU);
    b[off + 1U] = (uint8_t)((v >> 8) & 0xFFU);
    b[off + 2U] = (uint8_t)((v >> 16) & 0xFFU);
    b[off + 3U] = (uint8_t)(v >> 24);
}

static void exif_wr_ent(uint8_t *b, uint32_t *pos, uint16_t tag, uint16_t type, uint32_t cnt, uint32_t val)
{
    uint32_t p = *pos;
    exif_wr16(b, p,      tag);
    exif_wr16(b, p + 2U, type);
    exif_wr32(b, p + 4U, cnt);
    exif_wr32(b, p + 8U, val);
    *pos = p + 12U;
}

static void write_exif_app1(jstream_t *js)
{
    char dt[20];
    c3m1_clock_format_exif(dt);

    const c3m1_jpeg_ctx_t *c = js->ctx;
    uint32_t exp_num = (c->exp_num > 0UL && c->exp_den > 0UL) ? c->exp_num : 1UL;
    uint32_t exp_den = (c->exp_num > 0UL && c->exp_den > 0UL) ? c->exp_den : 100UL;
    uint16_t iso     = (c->iso > 0U) ? c->iso : 100U;
    int32_t  ev_num  = (c->ev_den > 0) ? c->ev_num : 0;
    int32_t  ev_den  = (c->ev_den > 0) ? c->ev_den : 3;

    uint8_t  has_focal = (c->focal_x10 > 0U) ? 1U : 0U;
    uint8_t  has_eq35  = (c->focal_eq35 > 0U) ? 1U : 0U;
    uint32_t lm_len    = 0UL;
    if (c->lens_model != NULL && c->lens_model[0] != '\0')
    {
        size_t slen = strlen(c->lens_model);
        if (slen > 31U) slen = 31U;
        lm_len = (uint32_t)(slen + 1U);
    }
    uint8_t  has_lm    = (lm_len > 0UL) ? 1U : 0U;
    uint16_t exif_cnt  = (uint16_t)(6U + (has_focal ? 1U : 0U) + (has_eq35 ? 1U : 0U) + (has_lm ? 1U : 0U));
    uint32_t pay_base  = 62UL + 2UL + (uint32_t)exif_cnt * 12UL + 4UL;
    uint32_t pay_pos   = pay_base;

    uint32_t off_make  = pay_pos; pay_pos += 6UL;  /* "CAYE\0" + pad */
    uint32_t off_model = pay_pos; pay_pos += 6UL;  /* "C3M1\0" + pad */
    uint32_t off_dt    = pay_pos; pay_pos += 20UL; /* "YYYY:MM:DD HH:MM:SS\0" */
    uint32_t off_exp   = pay_pos; pay_pos += 8UL;  /* ExposureTime */
    uint32_t off_fnum  = pay_pos; pay_pos += 8UL;  /* FNumber (0/1 when F0) */
    uint32_t off_ev    = pay_pos; pay_pos += 8UL;  /* ExposureBiasValue */
    uint32_t off_focal = 0UL;
    if (has_focal) { off_focal = pay_pos; pay_pos += 8UL; }
    uint32_t off_lm    = 0UL;
    if (lm_len > 4UL) { off_lm = pay_pos; pay_pos += (lm_len + 1UL) & ~1UL; }

    uint32_t app1_len  = 10UL + pay_pos; /* Max 282 B with focal + 32 B LensModel */

    /* EXIF 2.32 APP1: Make, Model, DateTime, ExifIFD (ExposureTime, FNumber, ISO, DateTime*, EV[, Focal][, LensModel]) */
    uint8_t app1[296];
    memset(app1, 0, app1_len);
    app1[0] = 0xFFU; app1[1] = 0xE1U;
    app1[2] = (uint8_t)((app1_len - 2UL) >> 8);
    app1[3] = (uint8_t)((app1_len - 2UL) & 0xFFU);
    memcpy(&app1[4], "Exif\0\0", 6U);

    uint8_t *tb = &app1[10];
    tb[0] = 'I'; tb[1] = 'I';
    exif_wr16(tb, 2U, 42U);
    exif_wr32(tb, 4U, 8U); /* IFD0 @ TIFF offset 8 */

    /* IFD0 (4 entries -> 54 B @ 8..61) */
    uint32_t p = 8U;
    exif_wr16(tb, p, 4U); p += 2U;
    exif_wr_ent(tb, &p, 0x010FU, 2U, 5U,  off_make);  /* Make */
    exif_wr_ent(tb, &p, 0x0110U, 2U, 5U,  off_model); /* Model */
    exif_wr_ent(tb, &p, 0x0132U, 2U, 20U, off_dt);    /* DateTime */
    exif_wr_ent(tb, &p, 0x8769U, 4U, 1U,  62U);       /* ExifIFD */
    exif_wr32(tb, p, 0U);

    /* ExifIFD (6..9 entries @ 62.., strictly ascending tags) */
    p = 62U;
    exif_wr16(tb, p, exif_cnt); p += 2U;
    exif_wr_ent(tb, &p, 0x829AU, 5U,  1U,  off_exp);  /* ExposureTime */
    exif_wr_ent(tb, &p, 0x829DU, 5U,  1U,  off_fnum); /* FNumber */
    exif_wr_ent(tb, &p, 0x8827U, 3U,  1U,  iso);      /* ISOSpeedRatings */
    exif_wr_ent(tb, &p, 0x9003U, 2U,  20U, off_dt);   /* DateTimeOriginal */
    exif_wr_ent(tb, &p, 0x9004U, 2U,  20U, off_dt);   /* DateTimeDigitized */
    exif_wr_ent(tb, &p, 0x9204U, 10U, 1U,  off_ev);   /* ExposureBiasValue */
    if (has_focal)
    {
        exif_wr_ent(tb, &p, 0x920AU, 5U, 1U, off_focal);     /* FocalLength */
    }
    if (has_eq35)
    {
        exif_wr_ent(tb, &p, 0xA405U, 3U, 1U, c->focal_eq35); /* FocalLengthIn35mmFilm */
    }
    if (has_lm)
    {
        uint32_t lm_val = off_lm;
        if (lm_len <= 4UL)
        {
            lm_val = 0UL;
            for (uint32_t i = 0UL; i < lm_len; i++)
            {
                uint8_t ch = (i + 1UL < lm_len) ? (uint8_t)c->lens_model[i] : 0U;
                lm_val |= ((uint32_t)ch << (i * 8UL));
            }
        }
        exif_wr_ent(tb, &p, 0xA434U, 2U, lm_len, lm_val);    /* LensModel */
    }
    exif_wr32(tb, p, 0U);

    /* Payloads */
    memcpy(&tb[off_make],  "CAYE", 5U);
    memcpy(&tb[off_model], "C3M1", 5U);
    memcpy(&tb[off_dt],    dt, 20U);
    if (lm_len > 4UL)
    {
        memcpy(&tb[off_lm], c->lens_model, lm_len - 1UL);
        tb[off_lm + lm_len - 1UL] = '\0';
    }
    exif_wr32(tb, off_exp + 0UL, exp_num);
    exif_wr32(tb, off_exp + 4UL, exp_den);
    if (c->fnum_x10 == 0U)
    {
        exif_wr32(tb, off_fnum + 0UL, 0UL);
        exif_wr32(tb, off_fnum + 4UL, 1UL);
    }
    else
    {
        exif_wr32(tb, off_fnum + 0UL, (uint32_t)c->fnum_x10);
        exif_wr32(tb, off_fnum + 4UL, 10UL);
    }
    exif_wr32(tb, off_ev + 0UL, (uint32_t)ev_num);
    exif_wr32(tb, off_ev + 4UL, (uint32_t)ev_den);
    if (has_focal)
    {
        if ((c->focal_x10 % 10U) == 0U)
        {
            exif_wr32(tb, off_focal + 0UL, (uint32_t)(c->focal_x10 / 10U));
            exif_wr32(tb, off_focal + 4UL, 1UL);
        }
        else
        {
            exif_wr32(tb, off_focal + 0UL, (uint32_t)c->focal_x10);
            exif_wr32(tb, off_focal + 4UL, 10UL);
        }
    }

    js_put_bytes(js, app1, app1_len);
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

static inline __attribute__((always_inline)) uint8_t vli_bits(int32_t val, uint32_t *out_code)
{
    if (val == 0)
    {
        *out_code = 0U;
        return 0U;
    }
    uint32_t mag = (val < 0) ? (uint32_t)(-val) : (uint32_t)val;
    uint8_t  n   = (uint8_t)(32U - (uint32_t)__builtin_clz(mag));
    *out_code    = (val < 0) ? (uint32_t)(val + ((1L << n) - 1L)) : mag;
    return n;
}

static void encode_block(jstream_t *js,
                         int32_t blk[64],
                         const uint16_t qrecip[64],
                         const uint8_t qhalf[64],
                         const huff_dc_lut_t *dc_lut,
                         const huff_ac_lut_t *ac_lut,
                         int32_t *prev_dc)
{
    fdct_islow(blk);

    /* Zero-division Q16 quantization into zig-zag order (fdct_islow output is scaled by 8x) */
    int32_t zz[64];
    for (uint8_t k = 0; k < 64U; k++)
    {
        int32_t  v   = blk[k];
        uint32_t mag = (v >= 0) ? (uint32_t)v : (uint32_t)(-v);
        uint32_t qv  = (((mag + (uint32_t)qhalf[k]) >> 3) * (uint32_t)qrecip[k]) >> 16;
        zz[s_zigzag[k]] = (v >= 0) ? (int32_t)qv : -(int32_t)qv;
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
static inline __attribute__((always_inline)) uint8_t lin12_to_srgb8(int32_t v12, const uint8_t srgb_lut[256])
{
    if (v12 <= 0)    return 0U;
    if (v12 >= 4064) return 255U;
    uint32_t idx  = (uint32_t)v12 >> 4;
    int32_t  frac = (int32_t)((uint32_t)v12 & 15UL);
    int32_t  s0   = (int32_t)srgb_lut[idx];
    int32_t  s1   = (int32_t)srgb_lut[idx + 1U];
    int32_t  res  = s0 + (((s1 - s0) * frac + 8) >> 4);
    if (res < 0)   res = 0;
    if (res > 255) res = 255;
    return (uint8_t)res;
}

static inline __attribute__((always_inline)) void c3m1_apply_ccm(int32_t rl, int32_t gl, int32_t bl, int32_t rgb[3])
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

static inline __attribute__((always_inline)) void color_cam12_to_srgb8_inline(
    uint16_t r12, uint16_t g12, uint16_t b12,
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

void c3m1_color_cam12_to_srgb8(uint16_t r12, uint16_t g12, uint16_t b12,
                               uint16_t wb_r_q8, uint16_t wb_b_q8,
                               const uint8_t srgb_lut[256],
                               uint8_t *out_r, uint8_t *out_g, uint8_t *out_b)
{
    color_cam12_to_srgb8_inline(r12, g12, b12, wb_r_q8, wb_b_q8, srgb_lut, out_r, out_g, out_b);
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
static inline __attribute__((always_inline)) uint16_t lin_sub(uint16_t v12, uint16_t bl)
{
    return (v12 > bl) ? (uint16_t)(v12 - bl) : 0U;
}

/*
 * Fast 1-pass direct Bayer-channel bilinear sampler (GRBG):
 *   - R lattice: (even y, odd x)  -> 4 lookups
 *   - B lattice: (odd y, even x)  -> 4 lookups
 *   - G quincunx: (x+y even)      -> 4 lookups (exact rotated diamond bilinear)
 * Total 12 lin[] lookups per pixel (3x fewer than 4-corner 3x3 demosaic), zero %50 divisions.
 */
static inline __attribute__((always_inline)) void bayer_ring_sample_bilinear12(uint16_t sx_q4, uint16_t sy_q4,
                                                const uint8_t *const *ring_rows,
                                                const uint8_t *mod50,
                                                const uint16_t *lin,
                                                uint16_t bl_g, uint16_t bl_rb,
                                                uint16_t *out_r, uint16_t *out_g, uint16_t *out_b)
{
    /* ---- R channel: x = 2u + 1 (1..639), y = 2v (0..478) ---- */
    uint32_t rx_q5 = (sx_q4 > 16U) ? (uint32_t)(sx_q4 - 16U) : 0UL;
    uint32_t ru0   = rx_q5 >> 5;
    uint32_t rfx   = rx_q5 & 31UL;
    if (ru0 > 319UL) { ru0 = 319UL; rfx = 0UL; }
    uint32_t rx0 = (ru0 << 1) + 1UL;
    uint32_t rx1 = (ru0 < 319UL) ? (rx0 + 2UL) : rx0;

    uint32_t rv0 = (uint32_t)sy_q4 >> 5;
    uint32_t rfy = (uint32_t)sy_q4 & 31UL;
    if (rv0 > 239UL) { rv0 = 239UL; rfy = 0UL; }
    uint32_t ry0 = rv0 << 1;
    uint32_t ry1 = (rv0 < 239UL) ? (ry0 + 2UL) : ry0;

    const uint8_t *rr0 = ring_rows[mod50[ry0]];
    const uint8_t *rr1 = ring_rows[mod50[ry1]];
    uint32_t r_top = (32UL - rfx) * lin[rr0[rx0]] + rfx * lin[rr0[rx1]];
    uint32_t r_bot = (32UL - rfx) * lin[rr1[rx0]] + rfx * lin[rr1[rx1]];
    uint16_t r12   = (uint16_t)(((32UL - rfy) * r_top + rfy * r_bot + 512UL) >> 10);

    /* ---- B channel: x = 2u (0..638), y = 2v + 1 (1..479) ---- */
    uint32_t bu0 = (uint32_t)sx_q4 >> 5;
    uint32_t bfx = (uint32_t)sx_q4 & 31UL;
    if (bu0 > 319UL) { bu0 = 319UL; bfx = 0UL; }
    uint32_t bx0 = bu0 << 1;
    uint32_t bx1 = (bu0 < 319UL) ? (bx0 + 2UL) : bx0;

    uint32_t by_q5 = (sy_q4 > 16U) ? (uint32_t)(sy_q4 - 16U) : 0UL;
    uint32_t bv0   = by_q5 >> 5;
    uint32_t bfy   = by_q5 & 31UL;
    if (bv0 > 239UL) { bv0 = 239UL; bfy = 0UL; }
    uint32_t by0 = (bv0 << 1) + 1UL;
    uint32_t by1 = (bv0 < 239UL) ? (by0 + 2UL) : by0;

    const uint8_t *br0 = ring_rows[mod50[by0]];
    const uint8_t *br1 = ring_rows[mod50[by1]];
    uint32_t b_top = (32UL - bfx) * lin[br0[bx0]] + bfx * lin[br0[bx1]];
    uint32_t b_bot = (32UL - bfx) * lin[br1[bx0]] + bfx * lin[br1[bx1]];
    uint16_t b12   = (uint16_t)(((32UL - bfy) * b_top + bfy * b_bot + 512UL) >> 10);

    /* ---- G channel: exact quincunx diamond (x + y even) ---- */
    uint32_t u_sum = (uint32_t)sx_q4 + (uint32_t)sy_q4;
    uint32_t v_dif = (uint32_t)sx_q4 + (480UL << 4) - (uint32_t)sy_q4;
    int32_t  gu0   = (int32_t)(u_sum >> 5);
    uint32_t guf   = u_sum & 31UL;
    int32_t  gv0   = (int32_t)(v_dif >> 5) - 240L;
    uint32_t gvf   = v_dif & 31UL;

    int32_t gx0 = gu0 + gv0;
    int32_t gy0 = gu0 - gv0;
    int32_t gx1 = gx0 + 1L;
    int32_t gx2 = gx0 + 2L;
    int32_t gym = gy0 - 1L;
    int32_t gyp = gy0 + 1L;

    /* Parity-preserving edge reflection keeps every site strictly on G */
    if (gx0 < 0L)   gx0 += 2L;
    if (gx0 >= 640L) gx0 -= 2L;
    if (gx1 < 0L)   gx1 += 2L;
    if (gx1 >= 640L) gx1 -= 2L;
    if (gx2 >= 640L) gx2 -= 2L;
    if (gy0 < 0L)   gy0 += 2L;
    if (gy0 >= 480L) gy0 -= 2L;
    if (gym < 0L)   gym += 2L;
    if (gym >= 480L) gym -= 2L;
    if (gyp < 0L)   gyp += 2L;
    if (gyp >= 480L) gyp -= 2L;

    uint32_t g00 = lin[ring_rows[mod50[gy0]][gx0]];
    uint32_t g10 = lin[ring_rows[mod50[gyp]][gx1]];
    uint32_t g01 = lin[ring_rows[mod50[gym]][gx1]];
    uint32_t g11 = lin[ring_rows[mod50[gy0]][gx2]];
    uint32_t g_a = (32UL - guf) * g00 + guf * g10;
    uint32_t g_b = (32UL - guf) * g01 + guf * g11;
    uint16_t g12 = (uint16_t)(((32UL - gvf) * g_a + gvf * g_b + 512UL) >> 10);

    *out_r = lin_sub(r12, bl_rb);
    *out_g = lin_sub(g12, bl_g);
    *out_b = lin_sub(b12, bl_rb);
}

uint8_t __attribute__((optimize("O2"))) c3m1_jpeg_encode_bayer_vga(const c3m1_jpeg_ctx_t *ctx,
                                                                   uint32_t *out_total_bytes)
{
    if (out_total_bytes != NULL)
    {
        *out_total_bytes = 0UL;
    }
    if (ctx == NULL || ctx->inv_gamma_12 == NULL || ctx->srgb_lut == NULL ||
        ctx->out_buf == NULL || ctx->out_buf_size == 0U ||
        ctx->read_row_fn == NULL || ctx->write_fn == NULL)
    {
        return 0U;
    }

    jstream_t js;
    memset(&js, 0, sizeof(js));
    js.ctx = ctx;

    write_jpeg_headers(&js);
    if (js.io_err)
    {
        return 0U;
    }

    const uint16_t *const lin       = ctx->inv_gamma_12;
    const uint8_t  *const srgb      = ctx->srgb_lut;
    const uint16_t        bl_g      = ctx->bl_g;
    const uint16_t        bl_rb     = ctx->bl_rb;
    const uint16_t        wbr       = ctx->wb_r_q8;
    const uint16_t        wbb       = ctx->wb_b_q8;
    const uint32_t        vig_scale = c3m1_lens_vig_scale_q16(ctx->lens_vignette);
    int8_t                distort   = ctx->lens_distort;
    if (distort < -100) distort = -100;
    if (distort >  100) distort =  100;

    int32_t prev_dc_y = 0, prev_dc_cb = 0, prev_dc_cr = 0;

    if (distort != 0)
    {
        /*
         * Fast warped path:
         *  - 50-row rolling Bayer ring buffer with Flash mod50 lookup + row pointer table
         *  - Piecewise-bilinear WarpRectilinear across each 16x16 MCU (4 warp calls per 256 px, <0.08 px err)
         *  - Direct 1-pass Bayer-channel bilinear (4 R + 4 B + 4 quincunx G = 12 lin[] lookups/px)
         */
        int32_t kr0_q16 = 65536L, kr1_q16 = 0L;
        c3m1_lens_dist_params_q16(distort, &kr0_q16, &kr1_q16);

        uint16_t next_fifo_row = 0U;

        for (uint8_t mcu_y = 0U; mcu_y < MCU_ROWS; mcu_y++)
        {
            const uint16_t y0    = (uint16_t)mcu_y * 16U;
            const uint16_t y_bot = (uint16_t)(y0 + 15U);

            /* Advance FIFO to cover max source Bayer row (+2 margin for R/B/G bilinear) */
            uint16_t req_max = 2U;
            static const uint16_t s_chk_x[3] = { 0U, 320U, 639U };
            for (uint8_t k = 0U; k < 3U; k++)
            {
                uint16_t sx_q4, sy_q4;
                c3m1_lens_warp_vga_q4(s_chk_x[k], y_bot, kr0_q16, kr1_q16, &sx_q4, &sy_q4);
                uint16_t sy0 = (uint16_t)(sy_q4 >> 4);
                uint16_t hi  = (sy0 + 2U < JPEG_H) ? (uint16_t)(sy0 + 2U) : (uint16_t)(JPEG_H - 1U);
                if (hi > req_max) req_max = hi;
            }
            while (next_fifo_row <= req_max && next_fifo_row < JPEG_H)
            {
                ctx->read_row_fn(ctx->user, s_bayer_strip[s_mod50[next_fifo_row]]);
                next_fifo_row++;
            }

            for (uint8_t mcu_x = 0U; mcu_x < MCU_COLS; mcu_x++)
            {
                int32_t y_blk[4][64];
                int32_t cb_blk[64];
                int32_t cr_blk[64];
                memset(cb_blk, 0, sizeof(cb_blk));
                memset(cr_blk, 0, sizeof(cr_blk));

                const uint16_t x0 = (uint16_t)mcu_x * 16U;
                const uint16_t x1 = (uint16_t)(x0 + 15U);

                /* Evaluate WarpRectilinear at the 4 corners of this 16x16 MCU */
                uint16_t xtl, ytl, xtr, ytr, xbl, ybl, xbr, ybr;
                c3m1_lens_warp_vga_q4(x0, y0,    kr0_q16, kr1_q16, &xtl, &ytl);
                c3m1_lens_warp_vga_q4(x1, y0,    kr0_q16, kr1_q16, &xtr, &ytr);
                c3m1_lens_warp_vga_q4(x0, y_bot, kr0_q16, kr1_q16, &xbl, &ybl);
                c3m1_lens_warp_vga_q4(x1, y_bot, kr0_q16, kr1_q16, &xbr, &ybr);

                /* Q12 corner coordinates & vertical edge increments (exact / 15) */
                int32_t lx  = (int32_t)xtl << 8;
                int32_t ly  = (int32_t)ytl << 8;
                int32_t rx  = (int32_t)xtr << 8;
                int32_t ry  = (int32_t)ytr << 8;
                int32_t dlx = (((int32_t)xbl - (int32_t)xtl) << 8) / 15L;
                int32_t dly = (((int32_t)ybl - (int32_t)ytl) << 8) / 15L;
                int32_t drx = (((int32_t)xbr - (int32_t)xtr) << 8) / 15L;
                int32_t dry = (((int32_t)ybr - (int32_t)ytr) << 8) / 15L;

                for (uint8_t dy = 0U; dy < 16U; dy++)
                {
                    const uint8_t y_sub_hi = (dy >= 8U) ? 2U : 0U;
                    const uint8_t by       = (uint8_t)(dy & 7U);
                    const uint8_t cy       = (uint8_t)(dy >> 1);

                    int32_t sx_q12 = lx;
                    int32_t sy_q12 = ly;
                    int32_t dsx    = (rx - lx) / 15L;
                    int32_t dsy    = (ry - ly) / 15L;
                    lx += dlx; ly += dly;
                    rx += drx; ry += dry;

                    for (uint8_t dx = 0U; dx < 16U; dx++)
                    {
                        uint16_t sx_q4 = (uint16_t)((sx_q12 + 128L) >> 8);
                        uint16_t sy_q4 = (uint16_t)((sy_q12 + 128L) >> 8);
                        sx_q12 += dsx;
                        sy_q12 += dsy;

                        uint16_t rl, gl, bl;
                        bayer_ring_sample_bilinear12(sx_q4, sy_q4, s_ring_rows, s_mod50,
                                                     lin, bl_g, bl_rb, &rl, &gl, &bl);

                        /* Apply FixVignetteRadial in original (pre-warp) source lens coordinates */
                        if (vig_scale != 0UL)
                        {
                            uint32_t gq8 = c3m1_lens_vig_gain_q8((uint16_t)(sx_q4 >> 4),
                                                                 (uint16_t)(sy_q4 >> 4), vig_scale);
                            uint32_t rr  = ((uint32_t)rl * gq8 + 128UL) >> 8;
                            uint32_t gg  = ((uint32_t)gl * gq8 + 128UL) >> 8;
                            uint32_t bb  = ((uint32_t)bl * gq8 + 128UL) >> 8;
                            rl = (rr > 4064UL) ? 4064U : (uint16_t)rr;
                            gl = (gg > 4064UL) ? 4064U : (uint16_t)gg;
                            bl = (bb > 4064UL) ? 4064U : (uint16_t)bb;
                        }

                        uint8_t sr, sg, sb;
                        color_cam12_to_srgb8_inline(rl, gl, bl, wbr, wbb, srgb, &sr, &sg, &sb);

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

                for (uint8_t i = 0U; i < 64U; i++)
                {
                    cb_blk[i] = (cb_blk[i] + 2) >> 2;
                    cr_blk[i] = (cr_blk[i] + 2) >> 2;
                }

                encode_block(&js, y_blk[0], s_qrecip_y_nat, s_qhalf_y_nat, &s_huff_dc_lum, &s_huff_ac_lum, &prev_dc_y);
                encode_block(&js, y_blk[1], s_qrecip_y_nat, s_qhalf_y_nat, &s_huff_dc_lum, &s_huff_ac_lum, &prev_dc_y);
                encode_block(&js, y_blk[2], s_qrecip_y_nat, s_qhalf_y_nat, &s_huff_dc_lum, &s_huff_ac_lum, &prev_dc_y);
                encode_block(&js, y_blk[3], s_qrecip_y_nat, s_qhalf_y_nat, &s_huff_dc_lum, &s_huff_ac_lum, &prev_dc_y);
                encode_block(&js, cb_blk,   s_qrecip_c_nat, s_qhalf_c_nat, &s_huff_dc_chr, &s_huff_ac_chr, &prev_dc_cb);
                encode_block(&js, cr_blk,   s_qrecip_c_nat, s_qhalf_c_nat, &s_huff_dc_chr, &s_huff_ac_chr, &prev_dc_cr);

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

        if (!js.io_err && out_total_bytes != NULL)
        {
            *out_total_bytes = js.total_bytes;
        }
        return js.io_err ? 0U : 1U;
    }

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
                const uint16_t py = (uint16_t)((uint16_t)mcu_y * 16U + dy);
                const int32_t  vdy = (int32_t)py - 240;
                const uint32_t vdy2 = (uint32_t)(vdy * vdy);

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

                    uint16_t rl = lin_sub(r12, bl_rb);
                    uint16_t gl = lin_sub(g12, bl_g);
                    uint16_t bl = lin_sub(b12, bl_rb);
                    if (vig_scale != 0UL)
                    {
                        int32_t  vdx = (int32_t)x - 320;
                        uint32_t gq8 = 256UL + ((((uint32_t)(vdx * vdx) + vdy2) * vig_scale + 32768UL) >> 16);
                        uint32_t rr  = ((uint32_t)rl * gq8 + 128UL) >> 8;
                        uint32_t gg  = ((uint32_t)gl * gq8 + 128UL) >> 8;
                        uint32_t bb  = ((uint32_t)bl * gq8 + 128UL) >> 8;
                        rl = (rr > 4064UL) ? 4064U : (uint16_t)rr;
                        gl = (gg > 4064UL) ? 4064U : (uint16_t)gg;
                        bl = (bb > 4064UL) ? 4064U : (uint16_t)bb;
                    }

                    uint8_t sr, sg, sb;
                    color_cam12_to_srgb8_inline(rl, gl, bl, wbr, wbb, srgb, &sr, &sg, &sb);

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
                cb_blk[i] = (cb_blk[i] + 2) >> 2;
                cr_blk[i] = (cr_blk[i] + 2) >> 2;
            }

            encode_block(&js, y_blk[0], s_qrecip_y_nat, s_qhalf_y_nat, &s_huff_dc_lum, &s_huff_ac_lum, &prev_dc_y);
            encode_block(&js, y_blk[1], s_qrecip_y_nat, s_qhalf_y_nat, &s_huff_dc_lum, &s_huff_ac_lum, &prev_dc_y);
            encode_block(&js, y_blk[2], s_qrecip_y_nat, s_qhalf_y_nat, &s_huff_dc_lum, &s_huff_ac_lum, &prev_dc_y);
            encode_block(&js, y_blk[3], s_qrecip_y_nat, s_qhalf_y_nat, &s_huff_dc_lum, &s_huff_ac_lum, &prev_dc_y);
            encode_block(&js, cb_blk,   s_qrecip_c_nat, s_qhalf_c_nat, &s_huff_dc_chr, &s_huff_ac_chr, &prev_dc_cb);
            encode_block(&js, cr_blk,   s_qrecip_c_nat, s_qhalf_c_nat, &s_huff_dc_chr, &s_huff_ac_chr, &prev_dc_cr);

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

    if (!js.io_err && out_total_bytes != NULL)
    {
        *out_total_bytes = js.total_bytes;
    }
    return js.io_err ? 0U : 1U;
}

