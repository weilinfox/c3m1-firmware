/**
 * @file        c3m1_jdec.c
 * @brief       Pure-integer streaming Baseline DCT JPEG decoder (IJG LLM integer IDCT).
 */

#include <string.h>
#include "c3m1_jdec.h"

/* Standard zigzag -> natural 8x8 raster index */
static const uint8_t s_unzigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
};

static inline uint8_t clamp_u8(int32_t v)
{
    if (v < 0)   return 0U;
    if (v > 255) return 255U;
    return (uint8_t)v;
}

/* ======================== Buffered Byte Stream ======================== */
static int32_t stream_getc(c3m1_jdec_t *jd)
{
    if (jd->in_pos >= jd->in_len)
    {
        if (jd->eof) return -1;
        uint32_t n = jd->read_fn(jd->read_ctx, jd->inbuf, sizeof(jd->inbuf));
        jd->in_pos = 0U;
        jd->in_len = (uint16_t)n;
        if (n == 0U)
        {
            jd->eof = 1U;
            return -1;
        }
    }
    return (int32_t)jd->inbuf[jd->in_pos++];
}

static uint8_t stream_read_exact(c3m1_jdec_t *jd, uint8_t *dst, uint32_t len)
{
    while (len > 0U)
    {
        if (jd->in_pos >= jd->in_len)
        {
            if (jd->eof) return 0U;
            uint32_t n = jd->read_fn(jd->read_ctx, jd->inbuf, sizeof(jd->inbuf));
            jd->in_pos = 0U;
            jd->in_len = (uint16_t)n;
            if (n == 0U)
            {
                jd->eof = 1U;
                return 0U;
            }
        }
        uint32_t avail = (uint32_t)(jd->in_len - jd->in_pos);
        uint32_t chunk = (len < avail) ? len : avail;
        if (dst != NULL)
        {
            memcpy(dst, &jd->inbuf[jd->in_pos], chunk);
            dst += chunk;
        }
        jd->in_pos = (uint16_t)(jd->in_pos + chunk);
        len -= chunk;
    }
    return 1U;
}

static uint8_t stream_read_u16be(c3m1_jdec_t *jd, uint16_t *out)
{
    uint8_t b[2];
    if (!stream_read_exact(jd, b, 2U)) return 0U;
    *out = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    return 1U;
}

/* ======================== Entropy Bitstream ======================== */
static void bit_fill(c3m1_jdec_t *jd, int32_t need)
{
    while (jd->bit_cnt < need)
    {
        if (jd->marker_pending != 0U)
        {
            /* Pad zeros if marker reached prematurely */
            jd->bit_cnt += 8;
            continue;
        }
        int32_t c = stream_getc(jd);
        if (c < 0)
        {
            jd->bit_cnt += 8;
            continue;
        }
        if (c == 0xFF)
        {
            int32_t c2;
            do
            {
                c2 = stream_getc(jd);
            } while (c2 == 0xFF);
            if (c2 < 0)
            {
                jd->marker_pending = 0xD9U;
                jd->bit_cnt += 8;
                continue;
            }
            if (c2 != 0x00)
            {
                /* Encountered a real marker (RSTm or EOI) */
                jd->marker_pending = (uint8_t)c2;
                jd->bit_cnt += 8;
                continue;
            }
            /* 0xFF 0x00 -> stuffed literal 0xFF */
        }
        jd->bit_buf |= ((uint32_t)(uint8_t)c) << (24 - jd->bit_cnt);
        jd->bit_cnt += 8;
    }
}

static inline uint32_t bit_get(c3m1_jdec_t *jd, int32_t n)
{
    if (n <= 0) return 0U;
    if (jd->bit_cnt < n) bit_fill(jd, n);
    uint32_t v = jd->bit_buf >> (32 - n);
    jd->bit_buf <<= n;
    jd->bit_cnt -= n;
    return v;
}

static inline int32_t huff_extend(uint32_t v, int32_t nbits)
{
    if (nbits <= 0) return 0;
    uint32_t half = 1UL << (nbits - 1);
    if (v < half)
    {
        return (int32_t)v - (int32_t)((1UL << nbits) - 1UL);
    }
    return (int32_t)v;
}

static int32_t huff_decode(c3m1_jdec_t *jd, const c3m1_jhuff_t *ht)
{
    if (jd->bit_cnt < 16) bit_fill(jd, 16);

    uint8_t peek8 = (uint8_t)(jd->bit_buf >> 24);
    uint8_t flen  = ht->fast_len[peek8];
    if (flen != 0U)
    {
        jd->bit_buf <<= flen;
        jd->bit_cnt -= (int32_t)flen;
        return (int32_t)ht->fast_sym[peek8];
    }

    int32_t code = (int32_t)(jd->bit_buf >> 24);
    for (int32_t len = 9; len <= 16; len++)
    {
        code = (int32_t)(jd->bit_buf >> (32 - len));
        if (code < ht->maxcode[len])
        {
            jd->bit_buf <<= len;
            jd->bit_cnt -= len;
            int32_t idx = code + ht->delta[len];
            if (idx < 0 || idx >= 256) return -1;
            return (int32_t)ht->huffval[idx];
        }
    }
    return -1;
}

static uint8_t huff_build(c3m1_jhuff_t *ht, const uint8_t bits[16], const uint8_t *vals, uint16_t total)
{
    memset(ht, 0, sizeof(*ht));
    if (total > 256U) return 0U;
    memcpy(ht->huffval, vals, total);

    int32_t code = 0;
    int32_t idx  = 0;
    for (int32_t len = 1; len <= 16; len++)
    {
        int32_t cnt = bits[len - 1];
        if (cnt == 0)
        {
            ht->maxcode[len] = -1;
            ht->delta[len]   = 0;
        }
        else
        {
            if ((code + cnt) > (1L << len))
            {
                return 0U; /* Reject oversubscribed Huffman code space (prevents fast_sym/fast_len OOB) */
            }
            ht->delta[len]   = idx - code;
            if (len <= 8)
            {
                int32_t shift = 8 - len;
                int32_t span  = 1 << shift;
                for (int32_t k = 0; k < cnt; k++)
                {
                    int32_t base = (code + k) << shift;
                    uint8_t sym  = vals[idx + k];
                    for (int32_t s = 0; s < span; s++)
                    {
                        ht->fast_sym[base + s] = sym;
                        ht->fast_len[base + s] = (uint8_t)len;
                    }
                }
            }
            code += cnt;
            idx  += cnt;
            ht->maxcode[len] = code;
        }
        code <<= 1;
    }
    ht->valid = 1U;
    return 1U;
}

/* ======================== Header Parsing ======================== */
static uint8_t parse_dqt(c3m1_jdec_t *jd, uint16_t len)
{
    while (len > 0U)
    {
        uint8_t pq_tq;
        if (!stream_read_exact(jd, &pq_tq, 1U)) return 0U;
        len--;
        uint8_t pq = (uint8_t)(pq_tq >> 4);
        uint8_t tq = (uint8_t)(pq_tq & 0x0FU);
        if (tq >= 4U || pq > 1U) return 0U;

        if (pq == 0U)
        {
            uint8_t raw[64];
            if (len < 64U || !stream_read_exact(jd, raw, 64U)) return 0U;
            len = (uint16_t)(len - 64U);
            for (uint8_t i = 0; i < 64U; i++)
            {
                jd->qtbl[tq][s_unzigzag[i]] = (uint16_t)raw[i];
            }
        }
        else
        {
            if (len < 128U) return 0U;
            for (uint8_t i = 0; i < 64U; i++)
            {
                uint16_t v;
                if (!stream_read_u16be(jd, &v)) return 0U;
                jd->qtbl[tq][s_unzigzag[i]] = v;
            }
            len = (uint16_t)(len - 128U);
        }
        jd->qtbl_valid[tq] = 1U;
    }
    return 1U;
}

static uint8_t parse_dht(c3m1_jdec_t *jd, uint16_t len)
{
    while (len > 0U)
    {
        uint8_t tc_th;
        uint8_t bits[16];
        uint8_t vals[256];
        if (len < 17U || !stream_read_exact(jd, &tc_th, 1U) || !stream_read_exact(jd, bits, 16U))
        {
            return 0U;
        }
        len = (uint16_t)(len - 17U);

        uint16_t total = 0U;
        for (uint8_t i = 0; i < 16U; i++) total = (uint16_t)(total + bits[i]);
        if (total > 256U || len < total || !stream_read_exact(jd, vals, total)) return 0U;
        len = (uint16_t)(len - total);

        uint8_t tc = (uint8_t)(tc_th >> 4);
        uint8_t th = (uint8_t)(tc_th & 0x0FU);
        if (tc > 1U || th > 1U) return 0U;

        c3m1_jhuff_t *dst = (tc == 0U) ? &jd->huff_dc[th] : &jd->huff_ac[th];
        if (!huff_build(dst, bits, vals, total)) return 0U;
    }
    return 1U;
}

static uint8_t parse_sof0(c3m1_jdec_t *jd, uint16_t len)
{
    uint8_t hdr[6];
    if (len < 6U || !stream_read_exact(jd, hdr, 6U)) return 0U;
    len = (uint16_t)(len - 6U);

    if (hdr[0] != 8U) return 0U; /* Only 8-bit Baseline DCT */
    jd->height = (uint16_t)(((uint16_t)hdr[1] << 8) | hdr[2]);
    jd->width  = (uint16_t)(((uint16_t)hdr[3] << 8) | hdr[4]);
    jd->ncomp  = hdr[5];

    if (jd->width == 0U || jd->height == 0U) return 0U;
    if (jd->ncomp != 1U && jd->ncomp != 3U) return 0U;
    if (len != (uint16_t)(jd->ncomp * 3U)) return 0U;

    jd->max_hsamp = 1U;
    jd->max_vsamp = 1U;
    for (uint8_t i = 0; i < jd->ncomp; i++)
    {
        uint8_t c[3];
        if (!stream_read_exact(jd, c, 3U)) return 0U;
        jd->comp[i].id     = c[0];
        jd->comp[i].hsamp  = (uint8_t)(c[1] >> 4);
        jd->comp[i].vsamp  = (uint8_t)(c[1] & 0x0FU);
        jd->comp[i].qt_idx = c[2];
        if (jd->comp[i].hsamp == 0U || jd->comp[i].hsamp > 2U ||
            jd->comp[i].vsamp == 0U || jd->comp[i].vsamp > 2U ||
            jd->comp[i].qt_idx >= 4U)
        {
            return 0U;
        }
        if (jd->comp[i].hsamp > jd->max_hsamp) jd->max_hsamp = jd->comp[i].hsamp;
        if (jd->comp[i].vsamp > jd->max_vsamp) jd->max_vsamp = jd->comp[i].vsamp;
    }

    /* Validate supported chroma subsampling (4:4:4, 4:2:2, 4:2:0, 1x1 Gray) */
    if (jd->ncomp == 1U)
    {
        if (jd->comp[0].hsamp != 1U || jd->comp[0].vsamp != 1U)
        {
            return 0U;
        }
    }
    else if (jd->ncomp == 3U)
    {
        if (jd->comp[1].hsamp != 1U || jd->comp[1].vsamp != 1U ||
            jd->comp[2].hsamp != 1U || jd->comp[2].vsamp != 1U)
        {
            return 0U;
        }
    }
    jd->mcu_w = (uint8_t)(jd->max_hsamp * 8U);
    jd->mcu_h = (uint8_t)(jd->max_vsamp * 8U);
    return 1U;
}

static uint8_t parse_sos(c3m1_jdec_t *jd, uint16_t len)
{
    uint8_t ns;
    if (len < 1U || !stream_read_exact(jd, &ns, 1U)) return 0U;
    len--;
    if (ns != jd->ncomp || len != (uint16_t)(ns * 2U + 3U)) return 0U;

    for (uint8_t i = 0; i < ns; i++)
    {
        uint8_t pair[2];
        if (!stream_read_exact(jd, pair, 2U)) return 0U;
        uint8_t found = 0U;
        for (uint8_t c = 0; c < jd->ncomp; c++)
        {
            if (jd->comp[c].id == pair[0])
            {
                jd->comp[c].dc_idx = (uint8_t)(pair[1] >> 4);
                jd->comp[c].ac_idx = (uint8_t)(pair[1] & 0x0FU);
                if (jd->comp[c].dc_idx > 1U || jd->comp[c].ac_idx > 1U) return 0U;
                found = 1U;
                break;
            }
        }
        if (!found) return 0U;
    }

    uint8_t spec[3];
    if (!stream_read_exact(jd, spec, 3U)) return 0U;
    if (spec[0] != 0U || spec[1] != 63U || spec[2] != 0U) return 0U; /* Baseline sequential only */
    return 1U;
}

uint8_t c3m1_jdec_prepare(c3m1_jdec_t *jd, c3m1_jdec_read_fn read_fn, void *read_ctx)
{
    memset(jd, 0, sizeof(*jd));
    jd->read_fn  = read_fn;
    jd->read_ctx = read_ctx;

    uint8_t soi[2];
    if (!stream_read_exact(jd, soi, 2U) || soi[0] != 0xFFU || soi[1] != 0xD8U)
    {
        return 0U;
    }

    uint8_t got_sof = 0U;
    for (;;)
    {
        int32_t b = stream_getc(jd);
        if (b < 0) return 0U;
        if (b != 0xFF) continue;

        int32_t marker;
        do
        {
            marker = stream_getc(jd);
        } while (marker == 0xFF);
        if (marker < 0) return 0U;
        if (marker == 0x00 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) continue;
        if (marker == 0xD9) return 0U; /* EOI before SOS */

        uint16_t seg_len;
        if (!stream_read_u16be(jd, &seg_len) || seg_len < 2U) return 0U;
        uint16_t payload = (uint16_t)(seg_len - 2U);

        if (marker == 0xDB) /* DQT */
        {
            if (!parse_dqt(jd, payload)) return 0U;
        }
        else if (marker == 0xC4) /* DHT */
        {
            if (!parse_dht(jd, payload)) return 0U;
        }
        else if (marker == 0xDD) /* DRI */
        {
            if (payload != 2U || !stream_read_u16be(jd, &jd->restart_interval)) return 0U;
        }
        else if (marker == 0xC0) /* SOF0 Baseline DCT */
        {
            if (!parse_sof0(jd, payload)) return 0U;
            got_sof = 1U;
        }
        else if (marker == 0xC1 || marker == 0xC2 || marker == 0xC3) /* Progressive / lossless */
        {
            return 0U;
        }
        else if (marker == 0xDA) /* SOS */
        {
            if (!got_sof || !parse_sos(jd, payload)) return 0U;
            /* Verify all referenced tables exist */
            for (uint8_t c = 0; c < jd->ncomp; c++)
            {
                if (!jd->qtbl_valid[jd->comp[c].qt_idx]) return 0U;
                if (!jd->huff_dc[jd->comp[c].dc_idx].valid) return 0U;
                if (!jd->huff_ac[jd->comp[c].ac_idx].valid) return 0U;
            }
            return 1U;
        }
        else
        {
            /* Skip APPn / COM / unknown segments */
            if (!stream_read_exact(jd, NULL, payload)) return 0U;
        }
    }
}

/* ======================== IJG LLM Integer IDCT ======================== */
#define IDCT_CONST_BITS  13
#define IDCT_PASS1_BITS  2

#define FIX_0_298631336  ((int32_t)2446)
#define FIX_0_390180644  ((int32_t)3196)
#define FIX_0_541196100  ((int32_t)4433)
#define FIX_0_765366865  ((int32_t)6270)
#define FIX_0_899976223  ((int32_t)7373)
#define FIX_1_175875602  ((int32_t)9633)
#define FIX_1_501321110  ((int32_t)12299)
#define FIX_1_847759065  ((int32_t)15137)
#define FIX_1_961570560  ((int32_t)16069)
#define FIX_2_053119869  ((int32_t)16819)
#define FIX_2_562915447  ((int32_t)20995)
#define FIX_3_072711026  ((int32_t)25172)

static inline int32_t descale(int32_t x, int32_t n)
{
    return (x + (1L << (n - 1))) >> n;
}

static void idct_islow(int16_t *blk)
{
    int32_t ws[64];

    /* Pass 1: process columns */
    for (int32_t col = 0; col < 8; col++)
    {
        int16_t *in  = blk + col;
        int32_t *out = ws + col;

        if (in[8] == 0 && in[16] == 0 && in[24] == 0 &&
            in[32] == 0 && in[40] == 0 && in[48] == 0 && in[56] == 0)
        {
            int32_t dc = (int32_t)in[0] * (1L << IDCT_PASS1_BITS);
            out[0] = dc; out[8] = dc; out[16] = dc; out[24] = dc;
            out[32] = dc; out[40] = dc; out[48] = dc; out[56] = dc;
            continue;
        }

        int32_t z2 = (int32_t)in[16];
        int32_t z3 = (int32_t)in[48];
        int32_t z1 = (z2 + z3) * FIX_0_541196100;
        int32_t tmp2 = z1 + z3 * (-FIX_1_847759065);
        int32_t tmp3 = z1 + z2 * FIX_0_765366865;

        z2 = (int32_t)in[0];
        z3 = (int32_t)in[32];
        int32_t tmp0 = (z2 + z3) * (1L << IDCT_CONST_BITS);
        int32_t tmp1 = (z2 - z3) * (1L << IDCT_CONST_BITS);

        int32_t tmp10 = tmp0 + tmp3;
        int32_t tmp13 = tmp0 - tmp3;
        int32_t tmp11 = tmp1 + tmp2;
        int32_t tmp12 = tmp1 - tmp2;

        tmp0 = (int32_t)in[56];
        tmp1 = (int32_t)in[40];
        tmp2 = (int32_t)in[24];
        tmp3 = (int32_t)in[8];

        z1 = tmp0 + tmp3;
        z2 = tmp1 + tmp2;
        z3 = tmp0 + tmp2;
        int32_t z4 = tmp1 + tmp3;
        int32_t z5 = (z3 + z4) * FIX_1_175875602;

        tmp0 *= FIX_0_298631336;
        tmp1 *= FIX_2_053119869;
        tmp2 *= FIX_3_072711026;
        tmp3 *= FIX_1_501321110;
        z1   *= (-FIX_0_899976223);
        z2   *= (-FIX_2_562915447);
        z3   *= (-FIX_1_961570560);
        z4   *= (-FIX_0_390180644);
        z3   += z5;
        z4   += z5;

        tmp0 += z1 + z3;
        tmp1 += z2 + z4;
        tmp2 += z2 + z3;
        tmp3 += z1 + z4;

        out[0]  = descale(tmp10 + tmp3, IDCT_CONST_BITS - IDCT_PASS1_BITS);
        out[56] = descale(tmp10 - tmp3, IDCT_CONST_BITS - IDCT_PASS1_BITS);
        out[8]  = descale(tmp11 + tmp2, IDCT_CONST_BITS - IDCT_PASS1_BITS);
        out[48] = descale(tmp11 - tmp2, IDCT_CONST_BITS - IDCT_PASS1_BITS);
        out[16] = descale(tmp12 + tmp1, IDCT_CONST_BITS - IDCT_PASS1_BITS);
        out[40] = descale(tmp12 - tmp1, IDCT_CONST_BITS - IDCT_PASS1_BITS);
        out[24] = descale(tmp13 + tmp0, IDCT_CONST_BITS - IDCT_PASS1_BITS);
        out[32] = descale(tmp13 - tmp0, IDCT_CONST_BITS - IDCT_PASS1_BITS);
    }

    /* Pass 2: process rows and level-shift +128 into [0..255] */
    for (int32_t row = 0; row < 8; row++)
    {
        int32_t *in  = ws + row * 8;
        int16_t *out = blk + row * 8;

        if (in[1] == 0 && in[2] == 0 && in[3] == 0 &&
            in[4] == 0 && in[5] == 0 && in[6] == 0 && in[7] == 0)
        {
            int16_t dc = (int16_t)clamp_u8(descale(in[0], IDCT_PASS1_BITS + 3) + 128);
            out[0] = dc; out[1] = dc; out[2] = dc; out[3] = dc;
            out[4] = dc; out[5] = dc; out[6] = dc; out[7] = dc;
            continue;
        }

        int32_t z2 = in[2];
        int32_t z3 = in[6];
        int32_t z1 = (z2 + z3) * FIX_0_541196100;
        int32_t tmp2 = z1 + z3 * (-FIX_1_847759065);
        int32_t tmp3 = z1 + z2 * FIX_0_765366865;

        int32_t tmp0 = (in[0] + in[4]) * (1L << IDCT_CONST_BITS);
        int32_t tmp1 = (in[0] - in[4]) * (1L << IDCT_CONST_BITS);

        int32_t tmp10 = tmp0 + tmp3;
        int32_t tmp13 = tmp0 - tmp3;
        int32_t tmp11 = tmp1 + tmp2;
        int32_t tmp12 = tmp1 - tmp2;

        tmp0 = in[7];
        tmp1 = in[5];
        tmp2 = in[3];
        tmp3 = in[1];

        z1 = tmp0 + tmp3;
        z2 = tmp1 + tmp2;
        z3 = tmp0 + tmp2;
        int32_t z4 = tmp1 + tmp3;
        int32_t z5 = (z3 + z4) * FIX_1_175875602;

        tmp0 *= FIX_0_298631336;
        tmp1 *= FIX_2_053119869;
        tmp2 *= FIX_3_072711026;
        tmp3 *= FIX_1_501321110;
        z1   *= (-FIX_0_899976223);
        z2   *= (-FIX_2_562915447);
        z3   *= (-FIX_1_961570560);
        z4   *= (-FIX_0_390180644);
        z3   += z5;
        z4   += z5;

        tmp0 += z1 + z3;
        tmp1 += z2 + z4;
        tmp2 += z2 + z3;
        tmp3 += z1 + z4;

        out[0] = (int16_t)clamp_u8(descale(tmp10 + tmp3, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3) + 128);
        out[7] = (int16_t)clamp_u8(descale(tmp10 - tmp3, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3) + 128);
        out[1] = (int16_t)clamp_u8(descale(tmp11 + tmp2, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3) + 128);
        out[6] = (int16_t)clamp_u8(descale(tmp11 - tmp2, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3) + 128);
        out[2] = (int16_t)clamp_u8(descale(tmp12 + tmp1, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3) + 128);
        out[5] = (int16_t)clamp_u8(descale(tmp12 - tmp1, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3) + 128);
        out[3] = (int16_t)clamp_u8(descale(tmp13 + tmp0, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3) + 128);
        out[4] = (int16_t)clamp_u8(descale(tmp13 - tmp0, IDCT_CONST_BITS + IDCT_PASS1_BITS + 3) + 128);
    }
}

/* ======================== MCU Decode & YCbCr -> RGB565 ======================== */
static uint8_t decode_block(c3m1_jdec_t *jd, uint8_t comp_idx, int16_t *dst_blk, uint8_t need_pixels)
{
    c3m1_jcomp_t *c = &jd->comp[comp_idx];
    const c3m1_jhuff_t *hdc = &jd->huff_dc[c->dc_idx];
    const c3m1_jhuff_t *hac = &jd->huff_ac[c->ac_idx];
    const uint16_t *qt = jd->qtbl[c->qt_idx];

    /* DC coefficient */
    int32_t s = huff_decode(jd, hdc);
    if (s < 0 || s > 11) return 0U;
    int32_t diff = (s > 0) ? huff_extend(bit_get(jd, s), s) : 0;
    c->dc_pred = (int16_t)(c->dc_pred + diff);

    if (need_pixels)
    {
        memset(dst_blk, 0, 64U * sizeof(int16_t));
        dst_blk[0] = (int16_t)((int32_t)c->dc_pred * (int32_t)qt[0]);
    }

    /* AC coefficients */
    int32_t k = 1;
    while (k < 64)
    {
        int32_t rs = huff_decode(jd, hac);
        if (rs < 0) return 0U;
        int32_t r = rs >> 4;
        int32_t ss = rs & 0x0F;
        if (ss == 0)
        {
            if (r == 0) break;       /* EOB */
            if (r == 15) { k += 16; continue; } /* ZRL */
            return 0U;
        }
        k += r;
        if (k >= 64) return 0U;
        int32_t ac = huff_extend(bit_get(jd, ss), ss);
        if (need_pixels)
        {
            uint8_t nat = s_unzigzag[k];
            dst_blk[nat] = (int16_t)(ac * (int32_t)qt[nat]);
        }
        k++;
    }

    if (need_pixels)
    {
        idct_islow(dst_blk);
    }
    return 1U;
}

static inline uint16_t rgb888_to_565(int32_t r, int32_t g, int32_t b)
{
    uint16_t r5 = (uint16_t)(clamp_u8(r) >> 3);
    uint16_t g6 = (uint16_t)(clamp_u8(g) >> 2);
    uint16_t b5 = (uint16_t)(clamp_u8(b) >> 3);
    return (uint16_t)((r5 << 11) | (g6 << 5) | b5);
}

static void mcu_to_rgb565(c3m1_jdec_t *jd)
{
    uint16_t *out = jd->mcu_rgb565;

    if (jd->ncomp == 1U)
    {
        const int16_t *yb = jd->blocks[0];
        for (uint8_t i = 0; i < 64U; i++)
        {
            int32_t y = yb[i];
            out[i] = rgb888_to_565(y, y, y);
        }
        return;
    }

    uint8_t y_cnt = (uint8_t)(jd->comp[0].hsamp * jd->comp[0].vsamp);
    const int16_t *cb_blk = jd->blocks[y_cnt];
    const int16_t *cr_blk = jd->blocks[y_cnt + 1U];
    uint8_t hshift = (jd->comp[0].hsamp == 2U) ? 1U : 0U;
    uint8_t vshift = (jd->comp[0].vsamp == 2U) ? 1U : 0U;

    for (uint8_t y = 0; y < jd->mcu_h; y++)
    {
        uint8_t cy  = (uint8_t)(y >> vshift);
        uint8_t by  = (uint8_t)((y >> 3) * jd->comp[0].hsamp);
        uint8_t y_in = (uint8_t)(y & 7U);

        for (uint8_t x = 0; x < jd->mcu_w; x++)
        {
            uint8_t cx  = (uint8_t)(x >> hshift);
            uint8_t bi  = (uint8_t)(by + (x >> 3));
            int32_t yy  = jd->blocks[bi][y_in * 8U + (x & 7U)];
            int32_t cb  = cb_blk[cy * 8U + cx] - 128;
            int32_t cr  = cr_blk[cy * 8U + cx] - 128;

            /* Exact BT.601 fixed-point Q10 */
            int32_t r = yy + ((1436 * cr + 512) >> 10);
            int32_t g = yy - ((352 * cb + 731 * cr + 512) >> 10);
            int32_t b = yy + ((1815 * cb + 512) >> 10);

            out[(uint16_t)y * jd->mcu_w + x] = rgb888_to_565(r, g, b);
        }
    }
}

uint8_t c3m1_jdec_decode(c3m1_jdec_t *jd,
                         uint16_t roi_x, uint16_t roi_y,
                         uint16_t roi_w, uint16_t roi_h,
                         c3m1_jdec_mcu_cb mcu_cb, void *cb_ctx)
{
    uint16_t mcus_x = (uint16_t)(((uint32_t)jd->width  + jd->mcu_w - 1UL) / jd->mcu_w);
    uint16_t mcus_y = (uint16_t)(((uint32_t)jd->height + jd->mcu_h - 1UL) / jd->mcu_h);
    uint32_t roi_x1 = (uint32_t)roi_x + (uint32_t)roi_w;
    uint32_t roi_y1 = (uint32_t)roi_y + (uint32_t)roi_h;

    uint16_t rst_todo = jd->restart_interval;

    for (uint16_t my = 0; my < mcus_y; my++)
    {
        uint32_t py = (uint32_t)my * (uint32_t)jd->mcu_h;
        if (py >= roi_y1)
        {
            break; /* Early exit once past ROI bottom */
        }
        uint8_t y_hit = ((py + (uint32_t)jd->mcu_h) > (uint32_t)roi_y) ? 1U : 0U;

        for (uint16_t mx = 0; mx < mcus_x; mx++)
        {
            uint32_t px = (uint32_t)mx * (uint32_t)jd->mcu_w;
            uint8_t need = (y_hit && (px + (uint32_t)jd->mcu_w) > (uint32_t)roi_x && px < roi_x1) ? 1U : 0U;

            uint8_t blk_i = 0U;
            for (uint8_t c = 0; c < jd->ncomp; c++)
            {
                uint8_t nblk = (uint8_t)(jd->comp[c].hsamp * jd->comp[c].vsamp);
                for (uint8_t b = 0; b < nblk; b++)
                {
                    if (!decode_block(jd, c, jd->blocks[blk_i], need))
                    {
                        return 0U;
                    }
                    blk_i++;
                }
            }

            if (need && mcu_cb != NULL)
            {
                mcu_to_rgb565(jd);
                mcu_cb(cb_ctx, px, py, jd->mcu_w, jd->mcu_h, jd->mcu_rgb565);
            }

            if (jd->restart_interval > 0U && --rst_todo == 0U)
            {
                /* Consume RSTm marker & reset DC predictors */
                jd->bit_buf = 0U;
                jd->bit_cnt = 0;
                if (jd->marker_pending >= 0xD0U && jd->marker_pending <= 0xD7U)
                {
                    jd->marker_pending = 0U;
                }
                else if (jd->marker_pending == 0U)
                {
                    /* Scan forward to RSTm */
                    int32_t ch;
                    while ((ch = stream_getc(jd)) >= 0)
                    {
                        if (ch == 0xFF)
                        {
                            int32_t m2;
                            do { m2 = stream_getc(jd); } while (m2 == 0xFF);
                            if (m2 >= 0xD0 && m2 <= 0xD7) break;
                            if (m2 == 0xD9) { jd->marker_pending = 0xD9U; break; }
                        }
                    }
                }
                for (uint8_t c = 0; c < jd->ncomp; c++) jd->comp[c].dc_pred = 0;
                rst_todo = jd->restart_interval;
            }
        }
    }
    return 1U;
}
