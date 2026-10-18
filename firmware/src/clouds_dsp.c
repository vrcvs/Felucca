/* SPDX-License-Identifier: MIT
 * Copyright (c) 2014 Emilie Gillet
 * Ported to fixed-point C for Felucca, 2026: Leo Kuroshita (@kurogedelic), Hügelton Instruments
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/* Clouds (Emilie Gillet, MIT; clouds/dsp/, pichenettes/eurorack) in fixed point, for Felucca's CLOUDS effect
 * (fx_clouds.c). Included after phys_dsp.c, whose primitives it uses (px_m, px_exp2, px_fasttan).
 *
 * What is here (the names of Clouds' classes in brackets):
 *   cl_buf_*     the recording buffers, 16 bit or 8-bit mu-law, with the crossfade after a freeze [AudioBuffer]
 *   cl_gr_*      the grains and their player [Grain, GranularSamplePlayer]
 *   cl_ws_*      the time stretcher / pitch shifter: two windows aligned by a sign-bit correlator [Window,
 *                WSOLASamplePlayer, Correlator]
 *   cl_lp_*      the looping delay [LoopingSamplePlayer]
 *   cl_pv_*      the phase vocoder [PhaseVocoder, STFT, FrameTransformation] with its own fixed-point FFT
 *   cl_df_*, cl_ps_*, cl_rv_*  the diffuser, the pitch shifter of the looping delay and the reverb [Diffuser,
 *                PitchShifter, Reverb; FxEngine]
 *   cl_svf_*     stmlib's Svf (FREQUENCY_FAST), cl_src_* the 2:1 resamplers [SampleRateConverter]
 *   cl_process   the whole chain [GranularProcessor::Process, ProcessGranular, Prepare]
 *
 * Number formats (no float anywhere): signals Q15 (Clouds' 1.0 = 32768, as its 16-bit codec) in int32 with
 * headroom; parameters 0..1 Q16; pitch in semitones Q16; ratios Q16; the 12-bit lines of the reverb as Clouds'
 * (Q12 in int16), the diffuser's Q13 in int16 (Clouds: float), the pitch shifter's Q15 in int16 (as Clouds').
 *
 * The sample rate. Clouds runs at 32 kHz (16 kHz inside in its low-fidelity modes). Here the processor runs at
 * a rate `rate` with every length in samples scaled by rs = rate / 32000 (Q16): the grain sizes, the delay lines of
 * the diffuser, pitch shifter and reverb, the window of the time stretcher, the crossfades, the loop lengths; the
 * per-sample coefficients that are time constants follow (1 - (1 - k)^(1 / rs)), so do the per-block ones for the
 * block's duration; the filters' cutoffs keep their frequency in Hz (clamped below the new Nyquist). With
 * `outer` set, cl_process takes and returns blocks at twice that rate through a 31-tap half-band (cl_hb_*: flat to
 * 0.2, -57 dB from 0.3 of the high rate; cheaper than Clouds' own src_filter_1x_2_45, which its low-fidelity modes
 * keep inside), and the dry part of BLEND is the caller's signal at the caller's rate. Felucca: 44.1 kHz
 * outside, 22.05 kHz inside (rs 0.689), 11.025 kHz inside the low-fidelity modes: Clouds' structure, every time
 * in seconds as Clouds', 0.69 x its bandwidth (11 kHz instead of 16; 5.5 instead of 8 in low fidelity). The host
 * tests run it at rs 1, rate 32000, no outer resampling, against Clouds' float code (tests/clouds_ref.cpp).
 *
 * Departures from Clouds, on purpose:
 *   - the FFT of the spectral mode has 2048 points (Clouds: 4096) at 22.05 kHz: 93 ms windows (Clouds 128 ms),
 *     hop 512; its frames are processed in slices, a few FFT passes per block in the audio ISR (cl_pv_work),
 *     where Clouds computes them in its main loop; the textures are stored in 16-bit floats (12-bit mantissa) so
 *     6 of them fit beside the phases, as in Clouds;
 *   - the correlator of the time stretcher also works in slices per block (cl_ws_work), budgeted to finish
 *     before the next window is due (Clouds: as fast as its main loop goes);
 *   - SoftLimit is a table (-3..3, then +-1: stmlib's SoftClip); atan2 and the magnitude of the phase
 *     vocoder use an atan table and a table of sqrt(1 + t^2) where Clouds uses atan_lut and a fast rsqrt;
 *   - the grains' smooth window is Clouds' lut_window read every fourth point (linear between);
 *   - the number of grains that may start is capped by gcap (fx_clouds.c lowers it under load);
 *   - while the buffers are cleared after a reset (a change of quality, to or from the spectral mode) the dry
 *     signal passes (Clouds: silence), and the clearing is spread over blocks;
 *   - the output: Clouds' SoftConvert halves its float signal into the codec's; here the BLEND's crossfade is
 *     scaled by sqrt(2) so that BLEND 0 gives the input back at its level, then 2 SoftLimit(x / 2). */
#include "clouds_tables.h"

#define CL_BLK 32                                        /* the longest block (kMaxBlockSize) */
#define CL_MAXG 64                                       /* grains (kMaxNumGrains) */
#define CL_XF 256                                        /* the crossfade after a freeze (kCrossFadeSize) */
#define CL_TAIL 8                                        /* kInterpolationTail */
#define CL_FFT CL_TABLES_FFT                             /* the phase vocoder's FFT (2048; Clouds: 4096) */
#if CL_FFT == 2048
#define CL_FFTL 10                                       /* log2(CL_FFT / 2) */
#elif CL_FFT == 4096
#define CL_FFTL 11
#else
#error "CL_FFT: 2048 or 4096"
#endif
#define CL_HOP (CL_FFT / 4)
#define CL_NTEX 7                                        /* kMaxNumTextures (here the phases are apart: 6 usable) */
#define CL_NBIN (CL_FFT / 2 - 16)                        /* bins kept (kHighFrequencyTruncation 16) */
#define CL_WSMAX 4096                                    /* kMaxWSOLASize (at 32 kHz) */
#define CL_CORRW (CL_WSMAX / 32 + 2)                     /* the correlator's block, words */
#define CL_PSLEN 4096                                    /* the pitch shifter's two lines (FxEngine<4096>) */
#define CL_DFLEN 2048                                    /* the diffuser's (FxEngine<2048>) */
#define CL_ONE 32768

enum { CL_GRANULAR, CL_STRETCH, CL_LOOPING, CL_SPECTRAL, CL_NPLAY };

/* the knobs, as Clouds' Parameters: Q16 0..1 (pitch: semitones Q16) */
typedef struct {
    int32_t position, size, pitch, density, texture, dry_wet, spread, feedback, reverb;
    uint8_t freeze, trigger, gate;
} cl_par_t;

/* ---------------------------------------------------------------- math --- */
static inline int32_t cl_clip16(int32_t x) { return x < -32768 ? -32768 : x > 32767 ? 32767 : x; }
static inline int32_t cl_sat(int32_t x, int32_t m) { return x < -m ? -m : x > m ? m : x; }

/* log2(x), x Q16 > 0: Q16 (a polynomial of the mantissa, error ~3e-5) */
static int32_t cl_log2(uint32_t x)
{
    int32_t e = 0, t, p;
    if (!x)
        return -(32 << 16);
    while (x >= (2u << 16)) {
        x >>= 1;
        e++;
    }
    while (x < (1u << 16)) {
        x <<= 1;
        e--;
    }
    t = (int32_t)x - 65536;                              /* [0, 1), Q16 */
    p = 3006;
    p = -12740 + px_m(t, p, 16);
    p = 27224 + px_m(t, p, 16);
    p = -46444 + px_m(t, p, 16);
    p = 94491 + px_m(t, p, 16);
    return (e << 16) + px_m(t, p, 16);
}

/* 2^(x / 12), x semitones Q16 (-48 .. 48): Q24 (stmlib SemitonesToRatio), and Q16 (truncated, as Clouds' casts) */
static inline uint32_t cl_semi24(int32_t x) { return px_exp2((int32_t)(((int64_t)x * 357913941) >> 32) + (8 << 16)); }
static inline uint32_t cl_semi(int32_t x) { return cl_semi24(x) >> 8; }

/* a one-pole coefficient k (Q30) for e times its time step (Q16): 1 - (1 - k)^e, Q30 (linear k e when k is tiny) */
static int32_t cl_kscale(int32_t k30, int32_t e16)
{
    uint32_t b;
    if (k30 <= 0)
        return 0;
    if (k30 < (1 << 20))
        return (int32_t)(((int64_t)k30 * e16) >> 16);
    if (k30 >= (1 << 30))
        return 1 << 30;
    b = (uint32_t)((((int64_t)1 << 30) - k30) >> 14);  /* 1 - k, Q16 */
    return (int32_t)((((int64_t)1 << 16) - (int64_t)px_exp2(px_m(cl_log2(b), e16, 16))) << 14);
}

/* SoftLimit(x) (stmlib) of x Q15, clamped to +-3 (+-1 out): Q15 */
static inline int32_t cl_soft(int32_t x)
{
    uint32_t t, i, f;
    if (x <= -98304)
        return -32768;
    if (x >= 98304)
        return 32768;
    t = (uint32_t)(x + 98304);
    t = (uint32_t)(((uint64_t)t * 5592406u) >> 16);      /* / 768, Q16 (rounded up: 0 is exactly 0) */
    i = t >> 16;
    f = t & 0xFFFFu;
    return CL_SOFT[i] + (((CL_SOFT[i + 1] - CL_SOFT[i]) * (int32_t)f) >> 16);
}

/* Interpolate(table of 17, x, 16), x Q16: the BLEND crossfade (Q15) */
static inline int32_t cl_xf(const int16_t *t, int32_t x)
{
    int32_t i = x >> 12, f = (x & 4095) << 3;            /* f Q15 */
    if (i >= 16)
        return t[16];
    return t[i] + (((t[i + 1] - t[i]) * f) >> 15);
}

/* the sine of lut_sin: i a 1/1024 turn, Q15 */
static inline int32_t cl_sinlut(int32_t i) { return CL_SIN[i & 1023]; }

/* rsqrt(x), x Q16 >= 1: Q16 */
static int32_t cl_rsqrt(int32_t x)
{
    uint32_t s = px_isqrt64((uint64_t)(uint32_t)x << 16);   /* sqrt(x) Q16 */
    return s ? (int32_t)(((uint64_t)1 << 32) / s) : 65536;
}

/* --------------------------------------------------------- the buffers --- */
typedef struct {
    int16_t *s16;
    uint8_t *s8;                                         /* mu-law (Clouds' low fidelity) */
    int32_t size, head, xfc;                             /* size: samples less the interpolation tail */
    int16_t *tail;                                       /* CL_XF + 1 */
} cl_buf_t;

static void cl_buf_init(cl_buf_t *b, void *mem, uint32_t bytes, int res8, int16_t *tail)
{
    b->s16 = res8 ? 0 : (int16_t *)mem;
    b->s8 = res8 ? (uint8_t *)mem : 0;
    b->size = (int32_t)(res8 ? bytes : bytes / 2u) - CL_TAIL;
    b->head = 0;
    b->xfc = 0;
    b->tail = tail;
}

/* Lin2MuLaw of mu_law.h */
static uint8_t cl_ulaw(int32_t pcm)
{
    int32_t mask, seg;
    pcm >>= 2;
    if (pcm < 0) {
        pcm = -pcm;
        mask = 0x7F;
    } else {
        mask = 0xFF;
    }
    if (pcm > 8159)
        pcm = 8159;
    pcm += 0x84 >> 2;
    seg = pcm <= 0x3F ? 0 : pcm <= 0x7F ? 1 : pcm <= 0xFF ? 2 : pcm <= 0x1FF ? 3 : pcm <= 0x3FF ? 4 :
          pcm <= 0x7FF ? 5 : pcm <= 0xFFF ? 6 : pcm <= 0x1FFF ? 7 : 8;
    if (seg >= 8)
        return (uint8_t)(0x7F ^ mask);
    return (uint8_t)(((seg << 4) | ((pcm >> (seg + 1)) & 0x0F)) ^ mask);
}

static inline void cl_buf_put(cl_buf_t *b, int32_t x)    /* Write(float): x Q15 */
{
    int32_t h = b->head;
    x = cl_clip16(x);
    if (b->s16) {
        b->s16[h] = (int16_t)x;
        if (h < CL_TAIL)
            b->s16[h + b->size] = (int16_t)x;
    } else {
        uint8_t u = cl_ulaw(x);
        b->s8[h] = u;
        if (h < CL_TAIL)
            b->s8[h + b->size] = u;
    }
    if (++h >= b->size)
        h = 0;
    b->head = h;
}

/* WriteFade: n samples of in (stride 2), recorded unless frozen; after a freeze, a crossfade from what was
 * recorded meanwhile into the tail (Clouds' quirks kept: the tail's index runs to CL_XF) */
static void cl_buf_write(cl_buf_t *b, const int32_t *in, int32_t n, int wr)
{
    if (!wr) {
        if (b->xfc < CL_XF)
            while (n--)
                if (b->xfc < CL_XF) {
                    b->tail[b->xfc++] = (int16_t)cl_clip16(*in);
                    in += 2;
                }
    } else if (!b->xfc && b->s16 && b->head >= CL_TAIL && b->head < b->size - n) {
        int16_t *d = b->s16 + b->head;
        b->head += n;
        while (n--) {
            *d++ = (int16_t)cl_clip16(*in);
            in += 2;
        }
    } else {
        while (n--) {
            int32_t s = *in;
            if (b->xfc) {
                int32_t t;
                --b->xfc;
                t = b->tail[CL_XF - b->xfc];
                s += (int32_t)(((int64_t)(t - s) * b->xfc) >> 8);
            }
            cl_buf_put(b, s);
            in += 2;
        }
    }
}

/* the sample at i (0 .. size + tail), Q15 */
#define CL_B16(b, i) ((int32_t)(b)->s16[i])
#define CL_B8(b, i) ((int32_t)CL_ULAW[(b)->s8[i]])

/* Laurent de Soras's Hermite interpolation of four samples, t Q12 */
static inline int32_t cl_herm(int32_t xm1, int32_t x0, int32_t x1, int32_t x2, int32_t t)
{
    int32_t c = (x1 - xm1) >> 1, v = x0 - x1, w = c + v, a = w + v + ((x2 - x0) >> 1), bn = w + a;
    int32_t y = ((a * t) >> 12) - bn;
    y = ((y * t) >> 12) + c;
    return ((y * t) >> 12) + x0;
}

static int32_t cl_rd_herm(const cl_buf_t *b, int32_t i, uint32_t frac)   /* ReadHermite: frac Q16 */
{
    int32_t t = (int32_t)(frac >> 4);
    if (i >= b->size)
        i -= b->size;
    if (b->s16)
        return cl_herm(CL_B16(b, i), CL_B16(b, i + 1), CL_B16(b, i + 2), CL_B16(b, i + 3), t);
    return cl_herm(CL_B8(b, i), CL_B8(b, i + 1), CL_B8(b, i + 2), CL_B8(b, i + 3), t);
}

static int32_t cl_rd_lin(const cl_buf_t *b, int32_t i, uint32_t frac)   /* ReadLinear */
{
    int32_t x0, x1;
    if (i >= b->size)
        i -= b->size;
    if (b->s16) {
        x0 = CL_B16(b, i);
        x1 = CL_B16(b, i + 1);
    } else {
        x0 = CL_B8(b, i);
        x1 = CL_B8(b, i + 1);
    }
    return x0 + (((x1 - x0) * (int32_t)(frac >> 1)) >> 15);
}

/* ------------------------------------------------------------ the SVF --- */
/* stmlib's Svf with set_f_q<FREQUENCY_FAST>: g Q28, r = 1 / q Q28, h Q30; the states in Q23 (signals << 8) */
typedef struct { int32_t g, r, h; } cl_svfk_t;
typedef struct { int32_t s1, s2; } cl_svf_t;

static void cl_svf_set(cl_svfk_t *k, uint32_t f, int32_t q16)   /* f cycles per sample, Q32 (< 0.5) */
{
    int64_t den;
    k->g = px_fasttan(f);
    k->r = (int32_t)(((int64_t)1 << 44) / q16);
    den = ((int64_t)1 << 28) + (((int64_t)k->r * k->g) >> 28) + (((int64_t)k->g * k->g) >> 28);
    k->h = (int32_t)(((int64_t)1 << 58) / den);
}

/* in place on n samples of x (stride 2): mode 0 low-pass, 1 high-pass */
static void cl_svf_run(const cl_svfk_t *k, cl_svf_t *s, int32_t *x, uint32_t n, int hp)
{
    int32_t g = k->g, rg = k->r + k->g, h = k->h, s1 = s->s1, s2 = s->s2;
    while (n--) {
        int32_t in = *x << 8, h_, bp, lp;
        h_ = px_m(in - px_m(rg, s1, 28) - s2, h, 30);
        bp = px_m(g, h_, 28) + s1;
        s1 = px_m(g, h_, 28) + bp;
        lp = px_m(g, bp, 28) + s2;
        s2 = px_m(g, bp, 28) + lp;
        *x = (hp ? h_ : lp) >> 8;
        x += 2;
    }
    s->s1 = s1;
    s->s2 = s2;
}

/* ---------------------------------------------------- 2:1 resamplers --- */
/* SampleRateConverter<-2 / +2, 45, src_filter_1x_2_45>: a ring of the last 45 frames, written twice so the
 * convolution reads them in one run; the filter is symmetric, so pairs of taps share a multiply. 32-bit sums: the
 * input of a down-sampler is 16-bit (or soft-limited to it), an up-sampler's is clamped to +-2 (the wet path) and its
 * taps are Q14 (CL_SRC Q16 / 4, the x 2 of the ratio in them) */
#define CL_SRCN 45
typedef struct {
    int32_t h[2][2 * CL_SRCN];                           /* left, right */
    int32_t p;
} cl_src_t;

static void cl_src_init(cl_src_t *s)
{
    uint32_t i;
    for (i = 0; i < 2u * CL_SRCN; i++)
        s->h[0][i] = s->h[1][i] = 0;
    s->p = CL_SRCN - 1;
}

/* down: n input frames (interleaved l r, |x| <= 32768) -> n / 2 */
static void cl_src_down(cl_src_t *s, const int32_t *in, int32_t *out, uint32_t n)
{
    int32_t p = s->p;
    while (n >= 2u) {
        int32_t yl, yr;
        const int32_t *xl, *xr;
        uint32_t j, k;
        for (k = 0; k < 2u; k++) {
            s->h[0][p + CL_SRCN] = s->h[0][p] = in[0];
            s->h[1][p + CL_SRCN] = s->h[1][p] = in[1];
            in += 2;
            if (--p < 0)
                p += CL_SRCN;
        }
        n -= 2u;
        xl = &s->h[0][p + 1];
        xr = &s->h[1][p + 1];
        yl = xl[22] * (CL_SRC[22] >> 1);                 /* (Q15 taps: 16-bit x Q15 sums fit 32 bits) */
        yr = xr[22] * (CL_SRC[22] >> 1);
        for (j = 0; j < 22u; j++) {
            int32_t h = CL_SRC[j] >> 1;
            yl += (xl[j] + xl[44 - j]) * h;
            yr += (xr[j] + xr[44 - j]) * h;
        }
        out[0] = (yl + 16384) >> 15;
        out[1] = (yr + 16384) >> 15;
        out += 2;
    }
    s->p = p;
}

/* up: n input frames -> 2 n (the ratio's gain: x 2) */
static void cl_src_up(cl_src_t *s, const int32_t *in, int32_t *out, uint32_t n)
{
    int32_t p = s->p;
    while (n--) {
        const int32_t *xl, *xr;
        int32_t el, er, ol, orr;
        uint32_t j;
        s->h[0][p + CL_SRCN] = s->h[0][p] = cl_sat(in[0], 65535);
        s->h[1][p + CL_SRCN] = s->h[1][p] = cl_sat(in[1], 65535);
        in += 2;
        if (--p < 0)
            p += CL_SRCN;
        xl = &s->h[0][p + 1];
        xr = &s->h[1][p + 1];
        /* even phase: taps 0, 2 .. 44 on x[0 .. 22] (symmetric about 11); odd: 1, 3 .. 43 on x[0 .. 21] (about 10.5) */
        el = xl[11] * (CL_SRC[22] >> 2);
        er = xr[11] * (CL_SRC[22] >> 2);
        for (j = 0; j < 11u; j++) {
            int32_t h = CL_SRC[2 * j] >> 2;
            el += (xl[j] + xl[22 - j]) * h;
            er += (xr[j] + xr[22 - j]) * h;
        }
        ol = orr = 0;
        for (j = 0; j < 11u; j++) {
            int32_t h = CL_SRC[2 * j + 1] >> 2;
            ol += (xl[j] + xl[21 - j]) * h;
            orr += (xr[j] + xr[21 - j]) * h;
        }
        out[0] = (el + 4096) >> 13;
        out[1] = (er + 4096) >> 13;
        out[2] = (ol + 4096) >> 13;
        out[3] = (orr + 4096) >> 13;
        out += 4;
    }
    s->p = p;
}

/* the device's outer pair, 44.1 <-> 22.05 kHz (not Clouds': its rate is its own): a 31-tap half-band, CL_HB the taps
 * at odd offsets from the 0.5 centre. Down: the last 31 inputs (a ring written twice), an output every two;
 * up: the last 16 inputs, a pair out per input (the odd phase is the centre tap alone: a delayed copy) */
#define CL_HBN 31
typedef struct {
    int32_t h[2][2 * CL_HBN];
    int32_t p;
} cl_hb_t;

static void cl_hb_init(cl_hb_t *s)
{
    uint32_t i;
    for (i = 0; i < 2u * CL_HBN; i++)
        s->h[0][i] = s->h[1][i] = 0;
    s->p = CL_HBN - 1;
}

static void cl_hb_down(cl_hb_t *s, const int32_t *in, int32_t *out, uint32_t n)   /* |in| <= 32768 */
{
    int32_t p = s->p;
    while (n >= 2u) {
        uint32_t k;
        for (k = 0; k < 2u; k++) {
            s->h[0][p + CL_HBN] = s->h[0][p] = in[0];
            s->h[1][p + CL_HBN] = s->h[1][p] = in[1];
            in += 2;
            if (--p < 0)
                p += CL_HBN;
        }
        n -= 2u;
        for (k = 0; k < 2u; k++) {
            const int32_t *x = &s->h[k][p + 1];        /* x[0] the newest .. x[30] */
            int32_t y = x[15] << 14, j;                  /* 0.5, Q15 */
            for (j = 0; j < 8; j++)
                y += (x[14 - 2 * j] + x[16 + 2 * j]) * (CL_HB[j] >> 1);
            out[k] = (y + 16384) >> 15;
        }
        out += 2;
    }
    s->p = p;
}

static void cl_hb_up(cl_hb_t *s, const int32_t *in, int32_t *out, uint32_t n)   /* (in clamped to +-2) */
{
    int32_t p = s->p;
    while (n--) {
        uint32_t k;
        s->h[0][p + 16] = s->h[0][p] = cl_sat(in[0], 65535);
        s->h[1][p + 16] = s->h[1][p] = cl_sat(in[1], 65535);
        in += 2;
        if (--p < 0)
            p += 16;
        for (k = 0; k < 2u; k++) {
            const int32_t *x = &s->h[k][p + 1];        /* x[0] the newest .. x[15] */
            int32_t y = 0, j;
            for (j = 0; j < 8; j++)                      /* the even phase: h at j and 15 - j, Q13 */
                y += (x[j] + x[15 - j]) * (CL_HB[7 - j] >> 3);
            out[k] = (y + 2048) >> 12;                   /* (x 2: the ratio's gain) */
            out[2 + k] = x[7];                           /* the odd phase: 2 x 0.5 at the centre */
        }
        out += 4;
    }
    s->p = p;
}

/* ------------------------------------------------------------- grains --- */
typedef struct {
    int32_t first, phase, inc, pre;                      /* the play head: Q16 samples from first */
    int32_t env, einc;                                   /* the envelope's phase 0 .. 2, Q28 */
    int32_t smooth, slope;                               /* the window's shape: Q15; Q8 */
    int32_t gl, gr;                                      /* Q15 */
    uint8_t active, quality;                             /* 0 LOW (zero order), 1 MEDIUM (linear), 2 HIGH (Hermite) */
} cl_grain_t;

#define CL_E1 (1 << 28)

/* one grain added into out (n stereo frames) */
static void cl_grain_ola(cl_grain_t *g, const cl_buf_t *b, int nch, int32_t *out, uint32_t n)
{
    int32_t first = g->first, phase = g->phase, inc = g->inc, env = g->env, einc = g->einc;
    int32_t gl = g->gl, gr = g->gr, sm = g->smooth, sl = g->slope, q = g->quality;
    if (!g->active)
        return;
    while (g->pre && n) {
        out += 2;
        --n;
        --g->pre;
    }
    while (n--) {
        int32_t e = env >= CL_E1 ? 2 * CL_E1 - env : env, i, l, r;
        uint32_t fr;
        e >>= 13;                                        /* Q15 */
        if (sm) {
            if (q == 2) {
                int32_t k = e >> 5, f = e & 31, w;       /* CL_WIN: 1024 steps over 0..1 */
                w = CL_WIN[k] + (((CL_WIN[k + (k < 1024)] - CL_WIN[k]) * f) >> 5);
                e += ((w - e) * sm) >> 15;
            }
        } else if (q >= 1) {
            e = (e * sl) >> 8;
            if (e > 32768)
                e = 32768;
        }
        env += einc;
        if (env >= 2 * CL_E1) {
            g->active = 0;
            break;
        }
        i = first + (phase >> 16);
        fr = (uint32_t)phase & 0xFFFFu;
        if (q == 2) {
            l = cl_rd_herm(&b[0], i, fr);
            r = nch == 2 ? cl_rd_herm(&b[1], i, fr) : 0;
        } else if (q == 1) {
            l = cl_rd_lin(&b[0], i, fr);
            r = nch == 2 ? cl_rd_lin(&b[1], i, fr) : 0;
        } else {
            int32_t j = i >= b[0].size ? i - b[0].size : i;
            l = b[0].s16 ? CL_B16(&b[0], j) : CL_B8(&b[0], j);
            r = nch == 2 ? (b[1].s16 ? CL_B16(&b[1], j) : CL_B8(&b[1], j)) : 0;
        }
        l = (l * e) >> 15;
        if (nch == 1) {
            out[0] += (l * gl) >> 15;
            out[1] += (l * gr) >> 15;
        } else {
            r = (r * e) >> 15;
            out[0] += ((l * gl) >> 15) + ((r * (32768 - gr)) >> 15);
            out[1] += ((r * gr) >> 15) + ((l * (32768 - gl)) >> 15);
        }
        out += 2;
        phase += inc;
    }
    g->phase = phase;
    g->env = env;
}

typedef struct {
    int32_t maxg, midfi, nch;
    int32_t ngr;                                         /* num_grains_, Q16 */
    int32_t gnorm;                                       /* gain_normalization_, Q16 */
    int32_t hint;                                        /* grain_size_hint_, samples Q16 */
    int32_t phasor;                                      /* grain_rate_phasor_, samples */
    uint8_t avail[CL_MAXG];
    cl_grain_t g[CL_MAXG];
} cl_gran_t;

/* ---------------------------------------------- the stretcher's windows --- */
typedef struct {
    int32_t first, phase, inc, einc;                     /* einc: 2 / width, Q24 */
    uint8_t done, half, regen;
} cl_win_t;

typedef struct {
    uint32_t *src, *dst;
    int32_t offset, inc, size, cand, best;
    uint32_t score;
    uint8_t done;
} cl_corr_t;

typedef struct {                                         /* ReadSignBits, resumable */
    uint32_t *dst;
    int32_t src, size, phase, inc, n;
    uint32_t bits, cnt;
} cl_sb_t;

typedef struct {
    cl_win_t w[2];
    int32_t wsize, nch;
    int32_t pitch, spitch, pos, sizef;                   /* semitones Q16, Q16 */
    int32_t ratio;                                       /* next_pitch_ratio_, Q16 */
    int32_t ssrc, stgt;
    int32_t env, einc;                                   /* Q16 */
    int32_t elapsed;
    uint8_t loaded;
    /* the correlator's job (cl_ws_work): 0 idle, 1 source bits, 2 target bits, 3 the search */
    uint8_t job;
    int32_t budget, nsamp, jinc;
    cl_sb_t sb;
} cl_ws_t;

/* --------------------------------------------------- the looping delay --- */
typedef struct {
    int64_t phase, ldur, lpt, tstart, tdur, lreset;      /* Q16 samples */
    uint32_t pfr;                                        /* the play head's bits below Q16 (Q24 increments) */
    uint32_t cur;                                        /* current_delay_, Q16 samples */
    int32_t tap, tapc;
    uint8_t sync, nch;
} cl_loop_t;

/* ------------------------------------------------- the phase vocoder --- */
typedef struct {
    int16_t *ana, *syn;
    int32_t bsize, bptr, pptr, blk, ready, done;
    uint16_t *tex[CL_NTEX];
    uint16_t *ph, *phd;
    int32_t ntex;
    int8_t glitch;
} cl_stft_t;

/* ------------------------------------------------------- the effects --- */
typedef struct {                                         /* FxEngine: a ring of lines, written backwards */
    int16_t *buf;
    int32_t size, wp;
} cl_fx_t;

typedef struct {                                         /* stmlib CosineOscillator, approximate */
    int32_t c, y0, y1;                                   /* c Q29, y Q30 */
} cl_cos_t;

/* -------------------------------------------------------- the whole --- */
typedef struct {
    /* the setup */
    int32_t rs, rate;                                    /* lengths against 32 kHz, Q16; Hz */
    uint8_t outer;
    uint8_t mode, pmode;                                 /* CL_GRANULAR .. (pmode: CL_NPLAY = none yet) */
    uint8_t nch, lofi, reset, clearing;
    uint8_t orig_out;                                    /* the host test: Clouds' own output stage, SoftLimit(x / 2) */
    uint8_t *arena;
    uint32_t bytes;
    /* time constants (cl_setup) */
    int32_t k_frz, k_slope_up, k_slope_dn, k_gnorm, k_dly, k_pssize, k_rvlp_e;
    int32_t xfdur, cfdur;                                /* kCrossfadeDuration (the looper), Q16 samples */
    int32_t gcap;                                        /* grains allowed at once (fx_clouds.c) */
    uint32_t rng;
    cl_par_t p;                                          /* this block's knobs */
    /* the clear after a reset */
    uint32_t clr_at, clr_end, clr_s0, clr_s1;
    uint8_t clr_fill;
    /* state */
    int32_t frz_lp;                                      /* freeze_lp_, Q30 */
    int32_t dry_wet;                                     /* Q16 */
    int32_t ovl, wshape;                                 /* the granular mode's overlap, window_shape, Q16 */
    uint8_t detseed;
    int32_t sp_q, sp_rr, sp_rand, sp_warp;               /* the spectral mode's parameters, Q16 */
    cl_buf_t buf[2];
    int16_t tail[2][CL_XF + 1];
    cl_gran_t gr;
    cl_ws_t ws;
    cl_corr_t co;
    cl_loop_t lp;
    cl_svf_t fbf[2], lpf[2], hpf[2];
    cl_hb_t sdn, sup;                                    /* the outer pair (the device) */
    cl_src_t sdn2, sup2;                                 /* Clouds' low-fidelity pair */
    /* the effects */
    cl_fx_t df, ps, rv;
    int32_t df_amt;                                      /* Q16 */
    uint8_t df_on;                                       /* the diffuser ran last block */
    int32_t rv_idle;                                     /* blocks the reverb's amount has been 0 */
    uint32_t ps_phase;                                   /* Q32 */
    int32_t ps_size;                                     /* samples Q16 */
    int32_t ps_ratio;                                    /* Q24 */
    int32_t rv_lp1, rv_lp2;                              /* Q15 */
    cl_cos_t lfo[2];
    int32_t lfo_v[2];                                    /* Q16 0..1 */
    int32_t rv_len[10], rv_base[10];
    int32_t df_len[8], df_base[8];
    /* the phase vocoder */
    cl_stft_t st[2];
    int32_t *fa, *fb;                                    /* the FFT's buffers (fft_buffer, ifft_buffer) */
    uint8_t pv_ch, pv_step, pv_busy;
    /* blocks */
    int32_t in[2 * CL_BLK], out[2 * CL_BLK], fb_[2 * CL_BLK];
    int32_t ind[CL_BLK], outd[CL_BLK];
} cl_t;

/* ------------------------------------------------------------ grains --- */
static void cl_gr_init(cl_gran_t *P, int32_t nch, int32_t maxg, int32_t rs)
{
    int32_t i;
    P->maxg = maxg;
    P->midfi = 3 * maxg / 4;
    P->gnorm = 65536;
    P->ngr = 0;
    P->nch = nch;
    P->hint = (int32_t)((int64_t)1024 * rs);             /* 1024 samples at 32 kHz, Q16 */
    P->phasor = 0;
    for (i = 0; i < CL_MAXG; i++) {
        P->g[i].active = 0;
        P->g[i].env = 2 * CL_E1;
    }
}

static inline uint32_t cl_rnd(cl_t *c) { return c->rng = c->rng * 1664525u + 1013904223u; }

static void cl_gr_schedule(cl_t *c, cl_grain_t *g, const cl_par_t *p, int32_t pre, int32_t bsize, int32_t head, int q)
{
    cl_gran_t *P = &c->gr;
    int32_t gs, size, i, f, pan, gl, gr, ws = c->wshape;
    uint32_t ratio = cl_semi(p->pitch), inv = cl_semi(-p->pitch);
    int64_t avail, eaten, start;
    /* lut_grain_size at SIZE, then this rate: Q8 samples */
    i = p->size >> 8;
    f = p->size & 255;
    gs = CL_GSIZE[i] + (i < 256 ? (((CL_GSIZE[i + 1] - CL_GSIZE[i]) * f) >> 8) : 0);
    gs = (int32_t)(((int64_t)gs * c->rs) >> 8);
    pan = 32768 + px_m(p->spread, (int32_t)(cl_rnd(c) >> 16) - 32768, 16);   /* Q16 0..1 */
    if (P->nch == 1) {                                   /* lut_sin at pan / 4 turn: sin, cos */
        int32_t x = pan >> 8, xf = pan & 255;
        gl = cl_sinlut(x) + (((cl_sinlut(x + 1) - cl_sinlut(x)) * xf) >> 8);
        gr = cl_sinlut(x + 256) + (((cl_sinlut(x + 257) - cl_sinlut(x + 256)) * xf) >> 8);
    } else if (pan < 32768) {
        gl = 32768;
        gr = pan;
    } else {
        gr = 32768;
        gl = 65536 - pan;
    }
    if (ratio > 65536u) {                                /* the play head runs faster than the recording */
        int32_t lim = (int32_t)(((int64_t)bsize * inv) >> 10);   /* buffer * 0.25 / ratio, Q8 */
        if (gs > lim)
            gs = lim;
    }
    eaten = ((int64_t)gs * ratio) >> 16;                 /* Q8 */
    avail = ((int64_t)bsize << 8) - eaten - gs;
    size = (gs >> 8) & ~1;
    start = head - ((((avail * p->position) >> 16) + eaten) >> 8);
    g->pre = pre;
    g->first = (int32_t)((start % bsize + bsize) % bsize);
    g->phase = 0;
    g->inc = (int32_t)ratio;
    g->env = 0;
    g->einc = size > 0 ? (int32_t)(((int64_t)2 << 28) / size) : 2 * CL_E1;
    if (ws >= 32768) {
        g->smooth = ws - 32768;                          /* (ws - 0.5) 2, Q15 */
        g->slope = 0;
    } else {
        g->smooth = 0;
        g->slope = (int32_t)((((int64_t)128 << 16) + (ws + 655) / 2) / (ws + 655));   /* 0.5 / (ws + 0.01), Q8 */
    }
    g->gl = gl;
    g->gr = gr;
    g->quality = (uint8_t)q;
    g->active = 1;
    P->hint += px_m((gs << 8) - P->hint, 107374182, 30);   /* ONE_POLE 0.1 (gs Q8, hint Q16) */
}

static void cl_gr_play(cl_t *c, const cl_par_t *p, int32_t *out, uint32_t n)
{
    cl_gran_t *P = &c->gr;
    const cl_buf_t *b = c->buf;
    int32_t ovl = c->ovl, i, na = 0, act, gn, wg, cap = c->gcap < P->maxg ? c->gcap : P->maxg;
    int64_t target;                                      /* grains, Q32 */
    uint32_t pth = 0, t;
    int seed_trig = p->trigger, nact = 0;
    target = (int64_t)P->maxg * (int64_t)(((((uint64_t)ovl * (uint64_t)ovl) >> 16) * (uint64_t)ovl));   /* max overlap^3 */
    if (!c->detseed) {
        P->phasor = -1000;
        if (P->hint > 0) {                               /* p = target / hint per sample: Q32 */
            int64_t pr = (target << 16) / P->hint;
            pth = pr >= ((int64_t)1 << 32) ? 0xFFFFFFFFu : (uint32_t)pr;
        }
    }
    for (i = 0; i < P->maxg; i++)
        if (!P->g[i].active)
            P->avail[na++] = (uint8_t)i;
        else
            nact++;
    for (t = 0; t < n; t++) {
        int sp, sd;
        P->phasor++;
        sp = (cl_rnd(c) < pth) && !c->detseed && target > ((int64_t)P->ngr << 16);   /* (drawn every sample, as Clouds) */
        sd = target > 0 && (int64_t)P->phasor * target >= ((int64_t)P->hint << 16);   /* phasor >= hint / target */
        if (na && (sp || sd || seed_trig) && nact < cap) {
            na--;
            cl_gr_schedule(c, &P->g[P->avail[na]], p, (int32_t)t, b[0].size, b[0].head - (int32_t)n + (int32_t)t,
                           na < P->midfi ? 1 : 2);
            nact++;
            P->phasor = 0;
            seed_trig = 0;
        }
    }
    for (t = 0; t < 2u * n; t++)
        out[t] = 0;
    for (i = 0; i < P->maxg; i++)
        cl_grain_ola(&P->g[i], b, P->nch, out, n);
    /* the normalization: SLOPE(num_grains_, active, 0.9, 0.2) per block */
    act = (P->maxg - na) << 16;
    P->ngr += px_m(act - P->ngr, act > P->ngr ? c->k_slope_up : c->k_slope_dn, 16);
    gn = P->ngr > (2 << 16) ? cl_rsqrt(P->ngr - 65536) : 65536;
    wg = 65536 + 2 * c->wshape;
    if (wg > 131072)
        wg = 131072;
    gn = px_m(gn, 65536 + px_m(wg - 65536, ovl, 16), 16);
    for (t = 0; t < n; t++) {
        P->gnorm += px_m(gn - P->gnorm, c->k_gnorm, 30);
        out[2 * t] = px_m(out[2 * t], P->gnorm, 16);
        out[2 * t + 1] = px_m(out[2 * t + 1], P->gnorm, 16);
    }
}

/* ----------------------------------------------------- the stretcher --- */
static void cl_win_start(cl_win_t *w, int32_t bsize, int32_t start, int32_t width, int32_t inc)
{
    w->first = ((start % bsize) + bsize) % bsize;
    w->inc = inc;
    w->phase = 0;
    w->done = 0;                                         /* (Clouds 1.1 does this; eurorack's master lost the line
                                                          * in 2023 and its STRETCH mode is silent) */
    w->regen = 0;
    w->einc = width > 0 ? (2 << 24) / width : 2 << 24;
}

static void cl_win_ola(cl_win_t *w, const cl_buf_t *b, int32_t *out, int nch)
{
    int32_t pi, i, e, g, l;
    uint32_t pf;
    if (w->done)
        return;
    pi = w->phase >> 16;
    pf = (uint32_t)w->phase & 0xFFFFu;
    i = w->first + pi;
    e = pi * w->einc;                                    /* Q24 0..2 */
    w->done = e >= (2 << 24);
    w->half = e >= (1 << 24);
    g = e >= (1 << 24) ? (2 << 24) - e : e;
    g = g < 0 ? 0 : g >> 8;                              /* Q16 */
    l = px_m(cl_rd_herm(&b[0], i, pf), g, 16);
    if (nch == 1) {
        out[0] += l;
        out[1] += l;
    } else {
        out[0] += l;
        out[1] += px_m(cl_rd_herm(&b[1], i, pf), g, 16);
    }
    w->phase += w->inc;
}

static int32_t cl_corr_best(const cl_corr_t *k) { return k->offset + ((k->best * (k->inc >> 4)) >> 12); }

static void cl_corr_eval(cl_corr_t *k)                   /* EvaluateNextCandidate */
{
    uint32_t nw, ow, ob, i, x = 0;
    const uint32_t *s, *d;
    if (k->done)
        return;
    nw = (uint32_t)k->size >> 5;
    ow = (uint32_t)k->cand >> 5;
    ob = (uint32_t)k->cand & 31u;
    s = k->src;
    d = &k->dst[ow];
    for (i = 0; i < nw; i++) {
        uint32_t db = d[i] << ob, cnt;
        if (ob)
            db |= d[i + 1] >> (32u - ob);
        cnt = ~(s[i] ^ db);
        cnt = cnt - ((cnt >> 1) & 0x55555555u);
        cnt = (cnt & 0x33333333u) + ((cnt >> 2) & 0x33333333u);
        x += (((cnt + (cnt >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
    }
    if (x > k->score) {
        k->best = k->cand;
        k->score = x;
    }
    ++k->cand;
    k->done = k->cand >= k->size;
}

/* ReadSignBits, a read for 4 of *budget; 1 = done */
static int cl_sb_run(cl_sb_t *s, const cl_buf_t *b, int nch, int32_t *budget)
{
    while ((s->phase >> 16) < s->size) {
        int32_t i = s->src + (s->phase >> 16), v;
        uint32_t f = (uint32_t)s->phase & 0xFFFFu;
        if (*budget <= 0)
            return 0;
        *budget -= 4;
        v = cl_rd_lin(&b[0], i, f);
        if (nch == 2)
            v += cl_rd_lin(&b[1], i, f);
        s->bits |= v > 0 ? 1u : 0u;
        if ((s->cnt & 31u) == 31u) {
            s->dst[s->cnt >> 5] = s->bits;
            s->n += 32;
        }
        ++s->cnt;
        s->bits <<= 1;
        s->phase += s->inc;
    }
    while (s->cnt & 31u) {
        if ((s->cnt & 31u) == 31u) {
            s->dst[s->cnt >> 5] = s->bits;
            s->n += 32;
        }
        ++s->cnt;
        s->bits <<= 1;
    }
    return 1;
}

static void cl_sb_start(cl_sb_t *s, const cl_buf_t *b, uint32_t *dst, int32_t src, int32_t size, int32_t inc)
{
    s->dst = dst;
    s->src = src < 0 ? src + b->size : src;
    s->size = size;
    s->phase = 0;
    s->inc = inc;
    s->n = 0;
    s->bits = 0;
    s->cnt = 0;
}

static void cl_ws_init(cl_t *c)
{
    cl_ws_t *W = &c->ws;
    W->nch = c->nch;
    W->pitch = W->spitch = W->pos = 0;
    W->w[0].done = W->w[1].done = 1;
    W->w[0].regen = W->w[1].regen = 0;
    W->w[0].half = W->w[1].half = 0;
    W->ratio = 65536;
    W->loaded = 1;
    W->job = 0;
    W->ssrc = W->stgt = 0;
    W->wsize = (int32_t)(((int64_t)(CL_WSMAX / 2) * c->rs) >> 16);
    W->env = 0;
    W->einc = 32768;
    W->elapsed = 0;
    c->co.offset = 0;
    c->co.best = 0;
    c->co.done = 1;
    c->co.inc = 65536;
}

static void cl_ws_schedule(cl_t *c, cl_win_t *w)        /* ScheduleAlignedWindow */
{
    cl_ws_t *W = &c->ws;
    const cl_buf_t *b = c->buf;
    int32_t next = cl_corr_best(&c->co), err, sgn, nws, lim, tgt, wmax;
    uint32_t ratio, inv, sf;
    W->loaded = 0;
    W->job = 0;
    cl_win_start(w, b->size, next - (W->wsize >> 1), W->wsize, W->ratio);
    err = W->pitch - W->spitch;
    sgn = err < 0 ? -1 : 1;
    err *= sgn;
    if (err >= (12 << 16))
        err = 12 << 16;
    W->spitch += err * sgn;
    ratio = cl_semi(W->spitch);
    inv = cl_semi(-W->spitch);
    W->ratio = (int32_t)ratio;
    sf = px_exp2(px_m(W->sizef - 65536, 5 << 16, 16));   /* 2^((size - 1) 60 / 12) */
    wmax = (int32_t)(((int64_t)CL_WSMAX * c->rs) >> 16);
    nws = (int32_t)(((int64_t)sf * wmax) >> 16);
    if (nws - W->wsize > 64 || W->wsize - nws > 64) {
        nws = W->wsize + ((nws - W->wsize) >> 5);
        W->wsize = nws - (nws % 4);
    }
    lim = b->size - (int32_t)(((int64_t)2 * W->wsize * inv) >> 16) - 2 * W->wsize;
    if (lim < 0)
        lim = 0;
    tgt = b->head - px_m(lim, W->pos, 16) - W->wsize;
    W->ssrc = next;
    W->stgt = tgt;
}

static void cl_ws_play(cl_t *c, const cl_par_t *p, int32_t *out, uint32_t n)
{
    cl_ws_t *W = &c->ws;
    const cl_buf_t *b = c->buf;
    uint32_t t;
    int i;
    W->elapsed++;
    if (p->trigger) {
        W->env = 0;
        W->einc = W->elapsed ? 65536 / W->elapsed : 6554;
        W->einc = W->einc < 7 ? 7 : W->einc > 6554 ? 6554 : W->einc;   /* 0.0001 .. 0.1 */
        W->elapsed = 0;
    }
    W->env += W->einc;
    if (W->env >= 65536)
        W->env = 65536;
    W->pos = p->position + px_m(65536 - W->env, 65536 - p->position, 16);
    W->pitch = p->pitch;
    W->sizef = p->size;
    if (W->w[0].done && W->w[1].done) {
        W->w[1].regen = 1;
        cl_ws_schedule(c, &W->w[0]);
    }
    for (t = 0; t < n; t++) {
        out[0] = out[1] = 0;
        for (i = 0; i < 2; i++)
            cl_win_ola(&W->w[i], b, out, W->nch);
        for (i = 0; i < 2; i++)
            if (W->w[i].half && !W->w[i].regen) {
                W->w[i].regen = 1;
                cl_ws_schedule(c, &W->w[1 - i]);
                cl_win_ola(&W->w[1 - i], b, out, W->nch);
            }
        out += 2;
    }
}

/* the correlator's work for one block (Clouds' Prepare: LoadCorrelator, then EvaluateSomeCandidates), spread so
 * that a search ends before the next window is due: a read costs 4, a candidate its words + 2 */
static void cl_ws_work(cl_t *c, uint32_t blk)
{
    cl_ws_t *W = &c->ws;
    cl_corr_t *k = &c->co;
    const cl_buf_t *b = c->buf;
    int32_t budget;
    if (W->loaded && k->done)
        return;
    if (!W->loaded && !W->job) {                         /* LoadCorrelator */
        int32_t stride = (W->wsize << 16) / 2048, n1, nb;
        stride = stride < 65536 ? 65536 : stride > 131072 ? 131072 : stride;
        W->jinc = px_m(stride, W->ratio < 81920 ? 81920 : W->ratio, 16);   /* x max(ratio, 1.25) */
        n1 = (int32_t)(((int64_t)W->wsize << 16) / W->jinc) + 1;
        nb = (int32_t)((((int64_t)W->wsize << 15) / W->ratio) / (int32_t)blk) - 2;   /* blocks until half the window
                                                                                     * has played (at its pitch) */
        nb = nb < 1 ? 1 : nb;
        W->budget = (12 * n1 + n1 * (n1 / 32 + 2)) / nb * 5 / 4 + 64;
        cl_sb_start(&W->sb, b, k->src, W->ssrc, W->wsize, W->jinc);
        W->job = 1;
    }
    budget = W->budget;
    if (W->job == 1) {
        if (!cl_sb_run(&W->sb, b, W->nch, &budget))
            return;
        W->nsamp = W->sb.n;
        cl_sb_start(&W->sb, b, k->dst, W->stgt - W->wsize, 2 * W->wsize, W->jinc);
        W->job = 2;
    }
    if (W->job == 2) {
        if (!cl_sb_run(&W->sb, b, W->nch, &budget))
            return;
        k->offset = W->stgt - W->wsize + (W->wsize >> 1);   /* StartSearch */
        k->inc = W->jinc;
        k->score = 0;
        k->best = 0;
        k->cand = 0;
        k->size = W->nsamp;
        k->done = k->size <= 0;
        W->loaded = 1;
        W->job = 3;
    }
    while (!k->done && budget > 0) {
        cl_corr_eval(k);
        budget -= (k->size >> 5) + 2;
    }
}

/* ------------------------------------------------- the looping delay --- */
static void cl_lp_init(cl_loop_t *L, int nch)
{
    L->nch = (uint8_t)nch;
    L->phase = 0;
    L->cur = 0;
    L->lpt = 0;
    L->ldur = 0;
    L->tap = 0;
    L->tapc = 0;
    L->sync = 0;
    L->tdur = 65536;
    L->tstart = 0;
    L->lreset = 0;
}

static void cl_lp_play(cl_t *c, const cl_par_t *p, int32_t *out, uint32_t n)
{
    cl_loop_t *L = &c->lp;
    const cl_buf_t *b = c->buf;
    int32_t maxd = b->size - (c->xfdur >> 16), nch = L->nch;
    L->tapc += (int32_t)n;
    if (L->tapc > maxd) {
        L->tap = 0;
        L->tapc = 0;
        L->sync = 0;
    }
    if (p->trigger) {
        L->tap = L->tapc;
        L->tapc = 0;
        L->sync = L->tap > (int32_t)(((int64_t)128 * c->rs) >> 16);
        L->lreset = L->phase;
        L->phase = 0;
    }
    if (!p->freeze) {
        uint32_t tgt = L->sync ? (uint32_t)L->tap << 16 : (uint32_t)(((int64_t)p->position * maxd));
        while (n--) {
            int32_t d, di;
            L->cur += (uint32_t)(int32_t)(((int64_t)((int64_t)tgt - L->cur) * c->k_dly) >> 30);
            di = (b->head - 4 - (int32_t)n + b->size) << 12;
            d = di - (int32_t)(L->cur >> 4);
            out[0] = cl_rd_herm(&b[0], d >> 12, ((uint32_t)d << 4) & 0xFFFFu);
            out[1] = nch == 2 ? cl_rd_herm(&b[1], d >> 12, ((uint32_t)d << 4) & 0xFFFFu) : out[0];
            out += 2;
        }
        L->phase = 0;
    } else {
        int64_t lpt = ((int64_t)p->position * maxd * 15) >> 4;   /* Q16 */
        uint64_t d3 = ((((uint64_t)p->size * (uint64_t)p->size) >> 16) * (uint64_t)p->size);   /* size^3, Q32 */
        int64_t ldur = (int64_t)((((uint64_t)42949673 + ((d3 * 4252017623u) >> 32)) * (uint64_t)maxd) >> 16), inc;   /* (0.01 + 0.99 size^3) maxd, Q16 */
        lpt += c->xfdur;
        if (L->sync)
            ldur = (int64_t)L->tap << 16;
        if (lpt + ldur >= ((int64_t)maxd << 16))
            lpt = ((int64_t)maxd << 16) - ldur;
        uint32_t inc24 = L->sync ? 1u << 24 : cl_semi24(p->pitch);
        inc = inc24 >> 8;
        while (n--) {
            int64_t g = 65536, di, pos;
            int32_t l, r;
            if (L->phase >= L->ldur || L->phase == 0) {
                if (L->phase >= L->ldur)
                    L->lreset = L->ldur;
                if (L->lreset >= L->ldur)
                    L->lreset = L->ldur;
                L->tstart = L->ldur - L->lreset + L->lpt;
                L->phase = 0;
                L->tdur = c->cfdur < ((c->cfdur * inc) >> 16) ? c->cfdur : (c->cfdur * inc) >> 16;
                L->lpt = lpt;
                L->ldur = ldur;
            }
            L->pfr += inc24 & 255u;
            L->phase += inc + (L->pfr >> 8);
            L->pfr &= 255u;
            if (L->tdur != 0) {
                g = (L->phase << 16) / L->tdur;
                g = g < 0 ? 0 : g > 65536 ? 65536 : g;
            }
            di = (int64_t)(b->head - 4 + b->size) << 12;
            pos = di - ((L->ldur - L->phase + L->lpt) >> 4);
            l = cl_rd_herm(&b[0], (int32_t)(pos >> 12), ((uint32_t)pos << 4) & 0xFFFFu);
            r = nch == 2 ? cl_rd_herm(&b[1], (int32_t)(pos >> 12), ((uint32_t)pos << 4) & 0xFFFFu) : l;
            out[0] = px_m(l, (int32_t)g, 16);
            out[1] = px_m(r, (int32_t)g, 16);
            if (g != 65536) {
                int32_t gt = 65536 - (int32_t)g;
                pos = di - ((-L->phase + L->tstart) >> 4);
                l = cl_rd_herm(&b[0], (int32_t)(pos >> 12), ((uint32_t)pos << 4) & 0xFFFFu);
                r = nch == 2 ? cl_rd_herm(&b[1], (int32_t)(pos >> 12), ((uint32_t)pos << 4) & 0xFFFFu) : l;
                out[0] += px_m(l, gt, 16);
                out[1] += px_m(r, gt, 16);
            }
            out += 2;
        }
    }
}

/* ------------------------------------------------------- FxEngine --- */
/* the lines of an FxEngine in one ring of `size` int16, written backwards one sample at a time; a line at base,
 * read `off` samples back */
static inline int32_t cl_fx_at(const cl_fx_t *e, int32_t base, int32_t off)   /* (base + off < size) */
{
    int32_t i = e->wp + base + off;
    if (i >= e->size)
        i -= e->size;
    return e->buf[i];
}
static inline void cl_fx_put(cl_fx_t *e, int32_t base, int32_t off, int32_t v)
{
    int32_t i = e->wp + base + off;
    if (i >= e->size)
        i -= e->size;
    e->buf[i] = (int16_t)cl_clip16(v);
}
static inline void cl_fx_step(cl_fx_t *e) { e->wp = e->wp ? e->wp - 1 : e->size - 1; }
/* Interpolate: off Q16 samples; the line's values (raw int16) */
static inline int32_t cl_fx_lerp(const cl_fx_t *e, int32_t base, int32_t off16)
{
    int32_t oi = off16 >> 16, f = (off16 >> 1) & 32767, a = cl_fx_at(e, base, oi), b = cl_fx_at(e, base, oi + 1);
    return a + (((b - a) * f) >> 15);
}

/* ---- the diffuser: 4 + 4 allpasses (0.625), lines in Q13 (Clouds: float) */
static const int16_t CL_DF_LEN[8] = {126, 180, 269, 444, 151, 205, 245, 405};

static void cl_df_run(cl_t *c, int32_t *x, uint32_t n)
{
    cl_fx_t *e = &c->df;
    int16_t *buf = e->buf;
    int32_t amt = c->df_amt, ch, k, wp = e->wp;
    const uint32_t M = CL_DFLEN - 1u;
    while (n--) {
        wp = (wp - 1) & (int32_t)M;
        for (ch = 0; ch < 2; ch++) {
            int32_t acc = x[ch];
            for (k = 0; k < 4; k++) {
                int32_t base = wp + c->df_base[4 * ch + k];
                int32_t rd = buf[(base + c->df_len[4 * ch + k] - 1) & M] << 2;   /* Q13 -> Q15 */
                acc += (rd * 2560) >> 12;                /* Read(TAIL, kap) */
                buf[base & M] = (int16_t)cl_clip16((acc + 2) >> 2);   /* WriteAllPass(-kap), Q13 rounded */
                acc = ((acc * -2560) >> 12) + rd;
            }
            x[ch] += px_m(acc - x[ch], amt, 16);
        }
        x += 2;
    }
    e->wp = wp;
}

/* ---- the pitch shifter of the looping delay (two 2047-sample lines, Q15) */
static void cl_ps_run(cl_t *c, int32_t *x, uint32_t n)
{
    cl_fx_t *e = &c->ps;
    int32_t sz = c->ps_size;                             /* Q16 */
    int64_t inc = (((int64_t)((1 << 24) - c->ps_ratio)) << 24) / (sz > 0 ? sz : 65536);   /* Q32 per sample */
    while (n--) {
        uint32_t ph;
        int32_t tri, o1, o2, ch, szq16 = sz;
        cl_fx_step(e);
        c->ps_phase += (uint32_t)(int32_t)inc;
        if (c->ps_phase == 0)
            c->ps_phase = 1;
        ph = c->ps_phase;
        tri = (int32_t)((ph >= 0x80000000u ? ~ph : ph) >> 16);   /* 2 min(phase, 1 - phase), Q15 */
        o1 = (int32_t)(((uint64_t)ph * (uint32_t)szq16) >> 32);   /* phase * size, Q16 */
        o2 = o1 + (szq16 >> 1);
        if (o2 >= szq16)
            o2 -= szq16;
        for (ch = 0; ch < 2; ch++) {
            int32_t base = ch * (CL_PSLEN / 2), a, b2;
            cl_fx_put(e, base, 0, x[ch]);
            a = cl_fx_lerp(e, base, o1);
            b2 = cl_fx_lerp(e, base, o2);
            x[ch] = ((a * tri) >> 15) + ((b2 * (32768 - tri)) >> 15);
        }
        x += 2;
    }
}

/* x c >> 12, rounded (a floor biases a loop's level) */
static inline int32_t cl_m12(int32_t x, int32_t c) { return (x * c + 2048) >> 12; }

/* the 12-bit lines' Compress: Q15 -> Q12, truncated towards zero as Clouds' cast (a floor would feed the loop) */
static inline int32_t cl_q12(int32_t x) { return (x + ((x >> 31) & 7)) >> 3; }

/* ---- the reverb (Griesinger, as Dattorro: 4 allpasses in, a loop of 2 x (2 allpasses + a delay)), 12-bit lines */
static const int16_t CL_RV_LEN[10] = {113, 162, 241, 399, 1653, 2038, 3411, 1913, 1663, 4782};

static void cl_cos_init(cl_cos_t *o, int32_t f)           /* f: cycles per tick, Q32 of a cycle (< 0.5) */
{
    int64_t fr = (int64_t)f - ((int64_t)1 << 30), sign = 16, x;   /* frequency - 0.25, Q32 */
    if (fr < 0) {
        fr = -fr;
    } else if (fr > ((int64_t)1 << 31)) {
        fr -= (int64_t)1 << 31;
    } else {
        sign = -16;
    }
    x = (fr * (((int64_t)1 << 32) - 2 * fr)) >> 32;      /* f (1 - 2 f), Q32 */
    o->c = (int32_t)((sign * x) >> 3);                   /* Q29 (+-2) */
    o->y1 = (int32_t)(((int64_t)o->c * (1 << 28)) >> 29);   /* iir * 0.25, Q30 */
    o->y0 = 1 << 29;                                     /* 0.5, Q30 */
}
static inline int32_t cl_cos_next(cl_cos_t *o)           /* Q30 value + 0.5 */
{
    int32_t t = o->y0;
    o->y0 = (int32_t)((((int64_t)o->c * o->y0 + ((int64_t)1 << 28)) >> 29) - o->y1);
    o->y1 = t;
    return t + (1 << 29);
}

/* x: n stereo frames in place. amount, krt, klp Q12 (32-bit products: the signals stay within the lines' +-8, 2^18) */
static void cl_rv_run(cl_t *c, int32_t *x, uint32_t n, int32_t amount, int32_t krt, int32_t klp)
{
    cl_fx_t *e = &c->rv;
    const int32_t kap = 2867, gain = 819;                /* diffusion 0.7, input gain 0.2 (Q12) */
    const int32_t *L = c->rv_len, *B = c->rv_base;
    int32_t lp1 = c->rv_lp1, lp2 = c->rv_lp2;
    int32_t o_ap1 = (int32_t)(((int64_t)10 << 16) * c->rs >> 16), a_ap1 = (int32_t)(((int64_t)60 << 16) * c->rs >> 16);
    int32_t o_d2 = (int32_t)(((int64_t)4680 << 16) * c->rs >> 16), a_d2 = (int32_t)(((int64_t)100 << 16) * c->rs >> 16);
    int32_t w100 = (int32_t)((100 * (int64_t)c->rs + 32768) >> 16);
    while (n--) {
        int32_t acc, apout, wet, rd, k;
        cl_fx_step(e);
        if ((e->wp & 31) == 0) {
            c->lfo_v[0] = cl_cos_next(&c->lfo[0]) >> 14;  /* Q16 */
            c->lfo_v[1] = cl_cos_next(&c->lfo[1]) >> 14;
        }
        /* smear AP1 inside the loop */
        cl_fx_put(e, B[0], w100, cl_fx_lerp(e, B[0], o_ap1 + px_m(a_ap1, c->lfo_v[0], 16)));
        acc = cl_m12(cl_sat(x[0] + x[1], 1 << 18), gain);
        for (k = 0; k < 4; k++) {                        /* the 4 diffusers */
            rd = cl_fx_at(e, B[k], L[k] - 1) << 3;
            acc += cl_m12(rd, kap);
            cl_fx_put(e, B[k], 0, cl_q12(acc));
            acc = cl_m12(cl_sat(acc, 1 << 18), -kap) + rd;
        }
        apout = cl_sat(acc, 1 << 18);
        /* left: del2 (modulated) -> lp -> dap1a, dap1b -> del1 */
        acc = apout + cl_m12((cl_fx_lerp(e, B[9], o_d2 + px_m(a_d2, c->lfo_v[1], 16)) << 3), krt);
        lp1 += cl_m12(cl_sat(acc, 1 << 18) - lp1, klp);
        acc = lp1;
        rd = cl_fx_at(e, B[4], L[4] - 1) << 3;
        acc += cl_m12(rd, -kap);
        cl_fx_put(e, B[4], 0, cl_q12(acc));
        acc = cl_m12(cl_sat(acc, 1 << 18), kap) + rd;
        rd = cl_fx_at(e, B[5], L[5] - 1) << 3;
        acc += cl_m12(rd, kap);
        cl_fx_put(e, B[5], 0, cl_q12(acc));
        acc = cl_m12(cl_sat(acc, 1 << 18), -kap) + rd;
        cl_fx_put(e, B[6], 0, cl_q12(acc));
        wet = cl_sat(acc * 2, 1 << 19);
        x[0] += cl_m12(wet - cl_sat(x[0], 1 << 19), amount);
        /* right: del1 -> lp -> dap2a, dap2b -> del2 */
        acc = apout + cl_m12((cl_fx_at(e, B[6], L[6] - 1) << 3), krt);
        lp2 += cl_m12(cl_sat(acc, 1 << 18) - lp2, klp);
        acc = lp2;
        rd = cl_fx_at(e, B[7], L[7] - 1) << 3;
        acc += cl_m12(rd, kap);
        cl_fx_put(e, B[7], 0, cl_q12(acc));
        acc = cl_m12(cl_sat(acc, 1 << 18), -kap) + rd;
        rd = cl_fx_at(e, B[8], L[8] - 1) << 3;
        acc += cl_m12(rd, -kap);
        cl_fx_put(e, B[8], 0, cl_q12(acc));
        acc = cl_m12(cl_sat(acc, 1 << 18), kap) + rd;
        cl_fx_put(e, B[9], 0, cl_q12(acc));
        wet = cl_sat(acc * 2, 1 << 19);
        x[1] += cl_m12(wet - cl_sat(x[1], 1 << 19), amount);
        x += 2;
    }
    c->rv_lp1 = lp1;
    c->rv_lp2 = lp2;
}

/* ------------------------------------------------------ the phase vocoder --- */
/* A real FFT of CL_FFT points as a complex one of CL_FFT / 2 (interleaved re, im) and the split. Every pass halves
 * (no overflow: a pass's outputs are no larger than its inputs), the split halves once more: the forward transform
 * gives X / N, the inverse (with one more half in its split) gives x / (2 N). */
#define CL_FFTC (CL_FFT / 2)
#define CL_FQ (CL_FFT / 4)                               /* a quarter turn of the twiddles */
#define CL_MAG (1 << 30)                                 /* the largest magnitude kept */

static inline void cl_tw(uint32_t k, int32_t *co, int32_t *si)   /* cos, sin of 2 pi k / CL_FFT, Q30 */
{
    uint32_t q, r;
    int32_t s, c;
    k &= CL_FFT - 1u;
    q = k / CL_FQ;
    r = k & (CL_FQ - 1u);
    s = (q & 1u) ? CL_TWQ[CL_FQ - r] : CL_TWQ[r];
    c = (q & 1u) ? CL_TWQ[r] : CL_TWQ[CL_FQ - r];
    *si = (q & 2u) ? -s : s;
    *co = (q == 1u || q == 2u) ? -c : c;
}

static void cl_bitrev(int32_t *x)
{
    uint32_t i, j = 0, m;
    for (i = 0; i < CL_FFTC; i++) {
        if (i < j) {
            int32_t t = x[2 * i];
            x[2 * i] = x[2 * j];
            x[2 * j] = t;
            t = x[2 * i + 1];
            x[2 * i + 1] = x[2 * j + 1];
            x[2 * j + 1] = t;
        }
        m = CL_FFTC >> 1;
        while (m >= 1u && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }
}

/* pass s (0 .. CL_FFTL - 1) of the radix-2 transform, in place; inv: e^{+i} */
static void cl_fft_pass(int32_t *x, uint32_t s, int inv)
{
    uint32_t len = 2u << s, half = len >> 1, kst = (CL_FFT / 2u) >> s, i, j;
    for (j = 0; j < half; j++) {
        int32_t co, si;
        cl_tw(j * kst, &co, &si);
        if (!inv)
            si = -si;
        for (i = j; i < CL_FFTC; i += len) {
            int32_t *a = &x[2 * i], *b = &x[2 * (i + half)];
            int32_t tr = (int32_t)(((int64_t)b[0] * co - (int64_t)b[1] * si) >> 31);   /* (w b) / 2 */
            int32_t ti = (int32_t)(((int64_t)b[0] * si + (int64_t)b[1] * co) >> 31);
            int32_t ar = a[0] >> 1, ai = a[1] >> 1;
            a[0] = ar + tr;
            a[1] = ai + ti;
            b[0] = ar - tr;
            b[1] = ai - ti;
        }
    }
}

/* z (interleaved, CL_FFTC points) -> X: re in X[0 .. CL_FFTC), im in X[CL_FFTC ..) (ShyFFT's layout) */
static void cl_fft_split(const int32_t *z, int32_t *X)
{
    uint32_t k;
    for (k = 0; k < CL_FFTC; k++) {
        uint32_t m = (CL_FFTC - k) & (CL_FFTC - 1u);
        int32_t ar = z[2 * k] >> 1, ai = z[2 * k + 1] >> 1, br = z[2 * m] >> 1, bi = z[2 * m + 1] >> 1;
        int32_t er = ar + br, ei = ai - bi;              /* (A + conj B) / 2 */
        int32_t orr = ai + bi, oi = br - ar;             /* -i (A - conj B) / 2 */
        int32_t co, si, wr, wi;
        cl_tw(k, &co, &si);                              /* W^k = co - i si */
        wr = (int32_t)(((int64_t)orr * co + (int64_t)oi * si) >> 30);
        wi = (int32_t)(((int64_t)oi * co - (int64_t)orr * si) >> 30);
        X[k] = (er >> 1) + (wr >> 1);
        X[CL_FFTC + k] = (ei >> 1) + (wi >> 1);
    }
}

/* X (as cl_fft_split's, the Nyquist bin 0) -> Z / 2 (interleaved), whose inverse transform is the frame */
static void cl_fft_unsplit(const int32_t *X, int32_t *z)
{
    uint32_t k;
    for (k = 0; k < CL_FFTC; k++) {
        uint32_t m = CL_FFTC - k;
        int32_t xr = X[k] >> 1, xi = X[CL_FFTC + k] >> 1;
        int32_t yr = k ? X[m] >> 1 : 0, yi = k ? X[CL_FFTC + m] >> 1 : 0;
        int32_t er = (xr + yr) >> 1, ei = (xi - yi) >> 1;     /* (X + conj Y) / 4 */
        int32_t dr = xr - yr, di = xi + yi;                    /* (X - conj Y) / 2 */
        int32_t co, si, fr, fi;
        cl_tw(k, &co, &si);                                    /* conj(W^k) = co + i si */
        fr = (int32_t)(((int64_t)dr * co - (int64_t)di * si) >> 31);   /* / 4 */
        fi = (int32_t)(((int64_t)dr * si + (int64_t)di * co) >> 31);
        z[2 * k] = er - fi;                                    /* (Fe + i Fo) / 2 */
        z[2 * k + 1] = ei + fr;
    }
}

/* the textures: magnitudes in 16 bits, a 12-bit mantissa and a 4-bit exponent (8 .. 2^30) */
static inline uint16_t cl_f16(int32_t v)
{
    uint32_t u = v < 0 ? 0u : (uint32_t)v, e;
    if (u < 32768u)
        return (uint16_t)(u >> 3);
    e = 31u - (uint32_t)__builtin_clz(u) - 14u;
    if (e > 15u)
        return 0xFFFFu;
    return (uint16_t)(e << 12 | ((u >> (e + 2u)) - 4096u));
}
static inline int32_t cl_unf16(uint16_t h)
{
    uint32_t e = h >> 12, m = h & 4095u;
    return e ? (int32_t)((4096u + m) << (e + 2u)) : (int32_t)(m << 3);
}

/* atan2 and the magnitude (Clouds: stmlib fast_atan2r): t = the smaller side over the larger, its angle and
 * sqrt(1 + t^2) from tables; the angle a 1/65536 turn, the magnitude in the input's units (at most CL_MAG) */
static inline uint16_t cl_r2p(int32_t re, int32_t im, int32_t *mag)
{
    uint32_t ax = re < 0 ? (uint32_t)-re : (uint32_t)re, ay = im < 0 ? (uint32_t)-im : (uint32_t)im;
    uint32_t big = ax > ay ? ax : ay, sm = ax > ay ? ay : ax, t, i, f, a;
    uint64_t m;
    if (!big) {
        *mag = 0;
        return 0;
    }
    t = (uint32_t)(((uint64_t)sm << 16) / big);          /* Q16, 0 .. 1 */
    i = t >> 8;
    f = t & 255u;
    if (i >= 256u) {
        i = 255u;
        f = 256u;
    }
    m = ((uint64_t)big * (CL_HYP[i] + (((CL_HYP[i + 1] - CL_HYP[i]) * f) >> 8))) >> 15;
    *mag = m > CL_MAG ? CL_MAG : (int32_t)m;
    a = CL_ATN[i] + (((CL_ATN[i + 1] - CL_ATN[i]) * f + 128u) >> 8);   /* 0 .. 8192 */
    if (ay > ax)
        a = 16384u - a;
    if (re < 0)
        a = im < 0 ? 32768u + a : 32768u - a;
    else if (im < 0)
        a = 65536u - a;
    return (uint16_t)a;
}

/* ---- FrameTransformation, on one channel's frame */
static const int32_t CL_WARP[6][4] = {                   /* kWarpPolynomials, Q16 */
    {693908, -975333, 346955, 0}, {-480596, 589824, -117418, 8192}, {0, 0, 65536, 0},
    {0, 32768, 32768, 0}, {-480596, 622592, -158379, 16384}, {-480596, 622592, -158379, 16384},
};

/* the two textures POSITION falls between, and the fraction */
static void cl_pv_tex(const cl_stft_t *S, int32_t position, uint16_t **ta, uint16_t **tb, int32_t *fr)
{
    int32_t nt = S->ntex, ix = position * (nt - 1), ii = ix >> 16;
    if (ii > nt - 1)
        ii = nt - 1;
    *fr = ix - (ii << 16);
    *ta = S->tex[ii];
    *tb = S->tex[ii + (position >= 65536 || ii >= nt - 1 ? 0 : 1)];
}

/* RectangularToPolar, then StoreMagnitudes (X: re | im -> magnitudes in X[0 ..]) */
static void cl_pv_store(cl_t *c, cl_stft_t *S, int32_t *X)
{
    int32_t *re = X, *im = X + CL_FFTC, i, fr, ga, gb, fb = c->sp_rr;
    uint16_t *ta, *tb;
    for (i = 1; i < CL_NBIN; i++) {
        int32_t m;
        uint16_t a = cl_r2p(re[i], im[i], &m);
        S->phd[i] = (uint16_t)(a - S->ph[i]);
        S->ph[i] = a;
        re[i] = m;
    }
    re[0] = 0;
    cl_pv_tex(S, c->p.position, &ta, &tb, &fr);
    ga = 65536 - fr;
    gb = fr;
    if (fb >= 32768) {
        fb = 2 * (fb - 32768);
        if (fb < 32768) {
            ga = px_m(ga, 65536 - fb, 16);
            gb = px_m(gb, 65536 - fb, 16);
            for (i = 0; i < CL_NBIN; i++) {
                int32_t x = re[i], v = cl_unf16(ta[i]);
                ta[i] = cl_f16(v + px_m(x - v, ga, 16));
                v = cl_unf16(tb[i]);
                tb[i] = cl_f16(v + px_m(x - v, gb, 16));
            }
        } else {
            int32_t t = px_m(fb - 32768, 45875, 16) + 32768;   /* (fb - 0.5) 0.7 + 0.5 */
            int32_t gn = t - 32768, gna, gnb, goa, gob;
            gn = 2 * px_m(gn, gn, 16) + 32768;
            gna = px_m(ga, gn, 16);
            gnb = px_m(gb, gn, 16);
            goa = 65536 - px_m(ga, 65536 - t, 16);
            gob = 65536 - px_m(gb, 65536 - t, 16);
            for (i = 0; i < CL_NBIN; i++) {
                int32_t x = re[i];
                ta[i] = cl_f16(px_m(cl_unf16(ta[i]), goa, 16) + px_m(x, gna, 16));
                tb[i] = cl_f16(px_m(cl_unf16(tb[i]), gob, 16) + px_m(x, gnb, 16));
            }
        }
    } else {
        uint32_t th;
        fb *= 2;
        fb = px_m(fb, fb, 16);
        th = (uint32_t)(((int64_t)fb * 65535) >> 16);
        for (i = 0; i < CL_NBIN; i++) {
            int32_t x = re[i], v;
            if ((cl_rnd(c) >> 16) <= th) {
                v = cl_unf16(ta[i]);
                ta[i] = cl_f16(v + px_m(x - v, ga, 16));
                v = cl_unf16(tb[i]);
                tb[i] = cl_f16(v + px_m(x - v, gb, 16));
            }
        }
    }
}

/* ReplayMagnitudes into A, WarpMagnitudes A -> B (Clouds: into fft_out, its temp) */
static void cl_pv_replay_warp(cl_t *c, cl_stft_t *S, int32_t *A, int32_t *B)
{
    int32_t i, fr, k[4], w = c->sp_warp * 4, wi = w >> 16, wf = w & 0xFFFF;
    uint16_t *ta, *tb;
    cl_pv_tex(S, c->p.position, &ta, &tb, &fr);
    for (i = 0; i < CL_NBIN; i++) {
        int32_t a = cl_unf16(ta[i]);
        A[i] = a + px_m(cl_unf16(tb[i]) - a, fr, 16);
    }
    if (wi > 4) {
        wi = 4;
        wf = 65536;
    }
    for (i = 0; i < 4; i++)
        k[i] = CL_WARP[wi][i] + px_m(CL_WARP[wi + 1][i] - CL_WARP[wi][i], wf, 16);
    for (i = 1; i < CL_NBIN; i++) {
        int32_t f = (int32_t)(((int64_t)i << 16) / CL_NBIN), x, xi, xf;
        x = k[3] + px_m(f, k[2] + px_m(f, k[1] + px_m(f, k[0], 16), 16), 16);   /* d + f (c + f (b + a f)) */
        x = (int32_t)(((int64_t)x * CL_NBIN));           /* the source bin, Q16 */
        if (x < 0)
            x = 0;
        if (x > ((CL_NBIN - 1) << 16))
            x = (CL_NBIN - 1) << 16;
        xi = x >> 16;
        xf = x & 0xFFFF;
        B[i] = A[xi] + px_m(A[xi + 1] - A[xi], xf, 16);
    }
}

/* ShiftMagnitudes B -> A (A[CL_NBIN ..] its scratch, as Clouds), AddGlitch, QuantizeMagnitudes */
static void cl_pv_shift(cl_t *c, cl_stft_t *S, int32_t *A, int32_t *B)
{
    int32_t *tmp = A + CL_NBIN, i, ratio = (int32_t)cl_semi(c->p.pitch), q = c->sp_q;
    if (ratio == 65536) {
        for (i = 0; i < CL_NBIN; i++)
            tmp[i] = B[i];
    } else if (ratio > 65536) {
        int32_t ix = 65536, inc = (int32_t)(((int64_t)1 << 32) / ratio);
        for (i = 1; i < CL_NBIN; i++) {
            int32_t xi = ix >> 16, xf = ix & 0xFFFF;
            tmp[i] = B[xi] + px_m(B[xi + 1] - B[xi], xf, 16);
            ix += inc;
        }
    } else {
        int32_t ix = 65536;
        for (i = 0; i < CL_NBIN; i++)
            tmp[i] = 0;
        for (i = 1; i < CL_NBIN; i++) {
            int32_t xi = ix >> 16, xf = ix & 0xFFFF, v = B[i], p = px_m(v, xf, 16);
            tmp[xi] += v - p;
            tmp[xi + 1] += p;
            ix += ratio;
        }
    }
    for (i = 0; i < CL_NBIN; i++)
        A[i] = tmp[i] > CL_MAG ? CL_MAG : tmp[i];
    if (c->p.gate) {                                     /* AddGlitch */
        switch (S->glitch) {
        case 0: {                                        /* spectral hold and blow */
            int32_t held = 0;
            for (i = 0; i < CL_NBIN; i++) {
                if (((cl_rnd(c) >> 16) & 15u) == 0)
                    held = A[i];
                A[i] = held;
                held = held >= CL_MAG / 101 * 100 ? CL_MAG : held + held / 100;
            }
            break;
        }
        case 1: {                                        /* shift up with aliasing */
            int32_t fac = 65536 + (int32_t)(((cl_rnd(c) >> 16) & 7u) << 14), src = 0;
            for (i = 0; i < CL_NBIN; i++) {
                src += fac;
                if (src >= (CL_NBIN << 16))
                    src = 0;
                A[i] = A[src >> 16];
            }
            break;
        }
        case 2: {                                        /* kill the largest bin, boost the second */
            int32_t b = 0, j;
            for (j = 1; j < CL_NBIN; j++)
                if (A[j] > A[b])
                    b = j;
            A[b] = 0;
            b = 0;
            for (j = 1; j < CL_NBIN; j++)
                if (A[j] > A[b])
                    b = j;
            A[b] = A[b] >= CL_MAG / 8 ? CL_MAG : A[b] * 8;
            break;
        }
        default:                                         /* a nasty high-pass */
            for (i = 0; i < CL_NBIN; i++)
                if (((cl_rnd(c) >> 16) & 15u) == 0)
                    A[i] = (int32_t)(((int64_t)A[i] * i) >> 4) > CL_MAG ? CL_MAG : (int32_t)(((int64_t)A[i] * i) >> 4);
            break;
        }
    }
    if (q <= 31457) {                                    /* QuantizeMagnitudes: <= 0.48, steps */
        int32_t a = q * 2, e = px_m(9 << 16, 65536 - px_m(a, a, 16), 16);
        uint32_t st = px_exp2(e), inv = (uint32_t)(((uint64_t)1 << 32) / st);   /* 2^(16 + 9 (1 - a^2)) */
        for (i = 0; i < CL_NBIN; i++) {
            uint32_t k = (uint32_t)(((uint64_t)(uint32_t)A[i] * inv) >> 32);
            A[i] = (int32_t)(k * st);
        }
    } else if (q >= 34079) {                             /* >= 0.52: towards 4 x (1 - x)^3 of the peak */
        int32_t a = (q - 34079) * 2, nrm = 0;
        uint32_t inv;
        for (i = 0; i < CL_NBIN; i++)
            if (A[i] > nrm)
                nrm = A[i];
        inv = (uint32_t)(((uint64_t)1 << 32) / (uint32_t)(nrm + 1));
        for (i = 1; i < CL_NBIN; i++) {
            int32_t x = (int32_t)(((uint64_t)(uint32_t)A[i] * inv) >> 16), y = 65536 - x, w;   /* Q16 */
            w = px_m(px_m(px_m(4 * x, y, 16), y, 16), y, 16);
            A[i] = px_m(x + px_m(w - x, a, 16), nrm, 16);
        }
    }
}

/* SetPhases and PolarToRectangular: A (magnitudes) -> A (re | im) */
static void cl_pv_phases(cl_t *c, cl_stft_t *S, int32_t *A)
{
    uint32_t *syn = (uint32_t *)&A[CL_FFTC];
    int32_t i, r = c->sp_rand, ratio = (int32_t)cl_semi(c->p.pitch), amt;
    for (i = 0; i < CL_NBIN; i++) {
        syn[i] = S->ph[i];
        S->ph[i] = (uint16_t)(S->ph[i] + (uint16_t)(((uint32_t)S->phd[i] * (uint32_t)ratio) >> 16));
    }
    r = px_m(r - 3277, 69468, 16);                       /* (r - 0.05) 1.06 */
    r = r < 0 ? 0 : r > 65536 ? 65536 : r;
    r = px_m(r, r, 16);
    amt = r >> 1;                                        /* r 32768 */
    for (i = 0; i < CL_NBIN; i++)                        /* (drawn even at 0, as Clouds) */
        syn[i] += (uint32_t)(((int32_t)(int16_t)(cl_rnd(c) >> 16) * amt) >> 14);
    for (i = 1; i < CL_NBIN; i++) {                      /* fast_p2r: lut_sin at the angle's top 10 bits */
        uint32_t an = (uint16_t)syn[i] >> 6;
        int32_t m = A[i];
        A[i] = (int32_t)(((int64_t)m * CL_SIN[an + 256]) >> 15);
        A[CL_FFTC + i] = (int32_t)(((int64_t)m * CL_SIN[an]) >> 15);
    }
    for (i = CL_NBIN; i < CL_FFTC; i++)
        A[i] = A[CL_FFTC + i] = 0;
    A[0] = 0;
    A[CL_FFTC] = 0;
    if (!c->p.gate)
        S->glitch = (int8_t)((cl_rnd(c) >> 16) & 3u);
}

/* ---- the STFT */
static void cl_stft_reset(cl_stft_t *S)
{
    S->bptr = 0;
    S->pptr = (2 * CL_HOP) % S->bsize;
    S->blk = 0;
    S->ready = S->done = 0;
}

/* STFT::Process: n samples of channel ch of in (stride 2) into the analysis buffer, the synthesis out */
static void cl_stft_io(cl_stft_t *S, const int32_t *in, int32_t *out, int32_t n)
{
    while (n) {
        int32_t k = CL_HOP - S->blk, i;
        if (k > n)
            k = n;
        for (i = 0; i < k; i++) {
            S->ana[S->bptr + i] = (int16_t)cl_clip16(*in);
            *out = S->syn[S->bptr + i] << 1;              /* / 16384, Q15 */
            in += 2;
            out += 2;
        }
        S->blk += k;
        n -= k;
        S->bptr += k;
        if (S->bptr >= S->bsize)
            S->bptr -= S->bsize;
        if (S->blk >= CL_HOP) {
            S->blk -= CL_HOP;
            ++S->ready;
        }
    }
}

/* STFT::Buffer in CL_PVSTEPS steps per channel (the FFT a pass a step): the window, the bit reversal, the passes,
 * the split, polar + store, replay + warp, shift + glitch + quantize, phases + rectangular, the inverse split, the bit
 * reversal, the passes, the overlap-add */
#define CL_PV_SPLIT (CL_FFTL + 2)
#define CL_PV_IREV (CL_PV_SPLIT + 6)
#define CL_PVSTEPS (CL_PV_IREV + CL_FFTL + 2)
static void cl_pv_step(cl_t *c)
{
    cl_stft_t *S = &c->st[c->pv_ch];
    int32_t *A = c->fa, *B = c->fb, i;
    uint32_t s = c->pv_step;
    if (s == 0) {
        int32_t sp = S->pptr;
        for (i = 0; i < CL_FFT; i++) {
            A[i] = (int32_t)S->ana[sp] * CL_PVWIN[i];
            if (++sp >= S->bsize)
                sp -= S->bsize;
        }
    } else if (s == 1) {
        cl_bitrev(A);
    } else if (s < CL_PV_SPLIT) {
        cl_fft_pass(A, s - 2u, 0);
    } else if (s == CL_PV_SPLIT) {
        cl_fft_split(A, B);
        B[0] = 0;
        B[CL_FFTC] = 0;
    } else if (s == CL_PV_SPLIT + 1) {
        if (!c->p.freeze)
            cl_pv_store(c, S, B);
    } else if (s == CL_PV_SPLIT + 2) {
        cl_pv_replay_warp(c, S, A, B);
    } else if (s == CL_PV_SPLIT + 3) {
        cl_pv_shift(c, S, A, B);
    } else if (s == CL_PV_SPLIT + 4) {
        cl_pv_phases(c, S, A);
    } else if (s == CL_PV_SPLIT + 5) {
        cl_fft_unsplit(A, B);
    } else if (s == CL_PV_IREV) {
        cl_bitrev(B);
    } else if (s < CL_PVSTEPS - 1) {
        cl_fft_pass(B, s - CL_PV_IREV - 1u, 1);
    } else {
        int32_t dp = S->pptr;
        for (i = 0; i < CL_FFT; i++) {
            int32_t x = (int32_t)(((int64_t)B[i] * CL_PVWIN[i]) >> (30 - (CL_FFTL + 1)));   /* x w 4 hop / 2^30 */
            if (i < CL_FFT - CL_HOP)
                x += S->syn[dp];
            S->syn[dp] = (int16_t)cl_clip16(x);
            if (++dp >= S->bsize)
                dp -= S->bsize;
        }
        ++S->done;
        S->pptr += CL_HOP;
        if (S->pptr >= S->bsize)
            S->pptr -= S->bsize;
    }
}

/* the frames due, `steps` steps at most (Clouds' Prepare: PhaseVocoder::Buffer) */
static void cl_pv_work(cl_t *c, int32_t steps)
{
    while (steps-- > 0) {
        if (!c->pv_busy) {                               /* a channel with a frame ready? */
            int32_t k;
            for (k = 0; k < c->nch; k++)
                if (c->st[k].ready != c->st[k].done)
                    break;
            if (k == c->nch)
                return;
            c->pv_ch = (uint8_t)k;
            c->pv_step = 0;
            c->pv_busy = 1;
        }
        cl_pv_step(c);
        if (++c->pv_step >= CL_PVSTEPS)
            c->pv_busy = 0;
    }
}

static uint8_t *cl_take(uint8_t **p, uint8_t *end, uint32_t bytes)
{
    uint8_t *r = *p;
    bytes = (bytes + 7u) & ~7u;
    if (r + bytes > end)
        return 0;
    *p = r + bytes;
    return r;
}

/* PhaseVocoder::Init on the sample area: [lo0, hi0) and [lo1, hi1) (stereo: the halves; mono: one) */
static int cl_pv_init(cl_t *c, uint8_t *lo0, uint8_t *hi0, uint8_t *lo1, uint8_t *hi1)
{
    uint8_t *p[2], *e[2];
    int32_t ch, nt = c->nch == 2 ? 2 : CL_NTEX - 1;     /* Clouds' memory holds 3 textures a channel in stereo, 7
                                                          * in mono (one of them its phases): as many here */
    p[0] = lo0;
    e[0] = hi0;
    p[1] = lo1;
    e[1] = hi1;
    c->fa = (int32_t *)cl_take(&p[0], e[0], 4u * CL_FFT);
    c->fb = (int32_t *)cl_take(&p[c->nch - 1], e[c->nch - 1], 4u * CL_FFT);
    if (!c->fa || !c->fb)
        return 0;
    for (ch = 0; ch < c->nch; ch++) {
        cl_stft_t *S = &c->st[ch];
        int32_t fr;
        S->bsize = CL_FFT + CL_HOP;
        S->ana = (int16_t *)cl_take(&p[ch], e[ch], 2u * 3u * CL_FFT);
        S->ph = (uint16_t *)cl_take(&p[ch], e[ch], 2u * 2u * CL_NBIN);
        if (!S->ana || !S->ph)
            return 0;
        S->syn = S->ana + S->bsize;
        S->phd = S->ph + CL_NBIN;
        S->glitch = 0;
        cl_stft_reset(S);
        fr = (int32_t)((e[ch] - p[ch]) / (2 * CL_NBIN + 8));
        nt = fr < nt ? fr : nt;
    }
    if (nt < 2)
        return 0;
    for (ch = 0; ch < c->nch; ch++) {
        int32_t k;
        c->st[ch].ntex = nt;
        for (k = 0; k < nt; k++)
            c->st[ch].tex[k] = (uint16_t *)cl_take(&p[ch], e[ch], 2u * CL_NBIN);
    }
    c->pv_busy = 0;
    return 1;
}

/* ------------------------------------------------------------ the whole --- */
/* a length of Clouds' (samples at 32 kHz) at this rate */
static inline int32_t cl_len(const cl_t *c, int32_t x) { int32_t r = (int32_t)(((int64_t)x * c->rs + 32768) >> 16); return r < 1 ? 1 : r; }

/* the processor in `bytes` of `arena` (8-byte aligned): its state first, then its lines and samples. rate: the
 * processor's (Hz); outer: blocks come at twice it; blk: the caller's block (<= CL_BLK). 0: too small */
static cl_t *cl_new(void *arena, uint32_t bytes, int32_t rate, int outer, uint32_t blk)
{
    cl_t *c = (cl_t *)arena;
    uint32_t i, *w = (uint32_t *)arena;
    int32_t inv_rs, bt, inner = (int32_t)(outer ? blk / 2u : blk);
    if (bytes < sizeof(cl_t) + 96u * 1024u || blk > CL_BLK || (blk & 3u))
        return 0;
    for (i = 0; i < sizeof(cl_t) / 4u; i++)
        w[i] = 0;
    c->arena = (uint8_t *)arena;
    c->bytes = bytes;
    c->rate = rate;
    c->rs = (int32_t)(((int64_t)rate << 16) / 32000);
    c->outer = (uint8_t)(outer != 0);
    c->mode = CL_GRANULAR;
    c->pmode = CL_NPLAY;
    c->nch = 2;
    c->reset = 1;
    c->gcap = CL_MAXG;
    c->rng = 0x21u;
    inv_rs = (int32_t)(((int64_t)1 << 32) / c->rs);      /* 1 / rs, Q16 */
    bt = (int32_t)(((int64_t)inner * 1000 << 16) / rate);   /* this block's time / Clouds' (32 at 32 kHz), Q16 */
    c->k_frz = cl_kscale(536871, bt);                    /* 0.0005 a block, Q30 */
    c->k_slope_up = cl_kscale(966367642, bt) >> 14;      /* 0.9, Q16 */
    c->k_slope_dn = cl_kscale(214748365, bt) >> 14;      /* 0.2 */
    c->k_gnorm = cl_kscale(10737418, inv_rs);            /* 0.01 a sample, Q30 */
    c->k_dly = cl_kscale(53687, inv_rs);                 /* 0.00005 a sample */
    c->k_pssize = cl_kscale(53687091, bt);               /* 0.05 a block, Q30 */
    c->k_rvlp_e = inv_rs;
    c->xfdur = cl_len(c, 64) << 16;                      /* kCrossfadeDuration */
    c->cfdur = c->xfdur;
    cl_hb_init(&c->sdn);
    cl_hb_init(&c->sup);
    cl_src_init(&c->sdn2);
    cl_src_init(&c->sup2);
    return c;
}

/* the bytes an arena for `rate` needs before its samples (the state, the effects' lines) */
static __attribute__((unused)) uint32_t cl_overhead(int32_t rate)
{
    int32_t rs = (int32_t)(((int64_t)rate << 16) / 32000), i, b = 0;
    for (i = 0; i < 10; i++) {
        int32_t l = (int32_t)(((int64_t)CL_RV_LEN[i] * rs + 32768) >> 16);
        b += (l < 1 ? 1 : l) + 1;
    }
    return ((sizeof(cl_t) + 7u) & ~7u) + ((2u * CL_DFLEN + 7u) & ~7u) + ((2u * (uint32_t)b + 7u) & ~7u) +
           ((2u * CL_PSLEN + 7u) & ~7u);
}

/* PLAYBACK mode and quality (Clouds' set_playback_mode, set_quality: bit 0 mono, bit 1 low fidelity) */
static void cl_config(cl_t *c, uint32_t mode, uint32_t quality)
{
    uint8_t nch = (quality & 1u) ? 1 : 2, lofi = (quality & 2u) ? 1 : 0;
    if (nch != c->nch || lofi != c->lofi)
        c->reset = 1;
    c->nch = nch;
    c->lofi = lofi;
    c->mode = (uint8_t)(mode < CL_NPLAY ? mode : CL_GRANULAR);
}

static void cl_filters_reset(cl_t *c)
{
    int i;
    for (i = 0; i < 2; i++) {
        c->fbf[i].s1 = c->fbf[i].s2 = 0;
        c->lpf[i].s1 = c->lpf[i].s2 = 0;
        c->hpf[i].s1 = c->hpf[i].s2 = 0;
    }
}

/* Prepare's reset: the layout, every player from scratch, and the clear of the lines and samples (cl_clear) */
static void cl_alloc(cl_t *c)
{
    uint8_t *p = c->arena + ((sizeof(cl_t) + 7u) & ~7u), *end = c->arena + (c->bytes & ~7u), *s0;
    int32_t i, b, nb;
    /* the diffuser */
    for (i = 0, b = 0; i < 8; i++) {
        c->df_len[i] = cl_len(c, CL_DF_LEN[i]);
        c->df_base[i] = b;
        b += c->df_len[i] + 1;
    }
    c->df.buf = (int16_t *)cl_take(&p, end, 2u * CL_DFLEN);
    c->df.size = CL_DFLEN;
    c->df.wp = 0;
    /* the reverb */
    for (i = 0, b = 0; i < 10; i++) {
        c->rv_len[i] = cl_len(c, CL_RV_LEN[i]);
        c->rv_base[i] = b;
        b += c->rv_len[i] + 1;
    }
    c->rv.buf = (int16_t *)cl_take(&p, end, 2u * (uint32_t)b);
    c->rv.size = b;
    c->rv.wp = 0;
    cl_cos_init(&c->lfo[0], (int32_t)(((int64_t)32 << 31) / c->rate));   /* 0.5 Hz, ticks of 32 samples */
    cl_cos_init(&c->lfo[1], (int32_t)(((int64_t)32 * 1288490189) / c->rate));   /* 0.3 Hz */
    c->lfo_v[0] = c->lfo_v[1] = 32768;
    c->rv_lp1 = c->rv_lp2 = 0;
    /* the pitch shifter, and the correlator in the same memory (never both: LOOPING, STRETCH) */
    c->ps.buf = (int16_t *)cl_take(&p, end, 2u * CL_PSLEN);
    c->ps.size = CL_PSLEN;
    c->ps.wp = 0;
    c->ps_phase = 0;
    c->ps_size = (int32_t)(((int64_t)2047 * c->rs));    /* Q16 */
    c->co.src = (uint32_t *)c->ps.buf;
    c->co.dst = c->co.src + CL_CORRW;
    /* the samples */
    s0 = p;
    nb = (int32_t)((end - s0) & ~15);
    c->clr_at = (uint32_t)((uint8_t *)c->df.buf - c->arena);
    c->clr_s0 = (uint32_t)(s0 - c->arena);
    c->clr_end = (uint32_t)(end - c->arena);
    c->clr_fill = (uint8_t)(c->lofi && c->mode != CL_SPECTRAL ? 0xFFu : 0u);   /* (mu-law 255 is 0) */
    c->clearing = 1;
    if (c->mode == CL_SPECTRAL) {
        if (c->nch == 2)
            cl_pv_init(c, s0, s0 + nb / 2, s0 + nb / 2, s0 + nb);
        else
            cl_pv_init(c, s0, s0 + nb, s0, s0 + nb);
    } else {
        int32_t k, half = c->nch == 2 ? nb / 2 : nb, ng;
        for (k = 0; k < c->nch; k++)
            cl_buf_init(&c->buf[k], s0 + k * half, (uint32_t)half, c->lofi, c->tail[k]);
        ng = ((c->nch == 1 ? 40 : 32) * (c->lofi ? 23 : 16)) >> 4;
        cl_gr_init(&c->gr, c->nch, ng, c->rs);
        cl_ws_init(c);
        cl_lp_init(&c->lp, c->nch);
    }
}

/* a part of the clear (up to `bytes`); 1 = done */
static int cl_clear(cl_t *c, uint32_t bytes)
{
    uint32_t a = c->clr_at, e = a + bytes;
    if (e > c->clr_end)
        e = c->clr_end;
    while (a < e) {
        uint32_t stop = a < c->clr_s0 ? (e < c->clr_s0 ? e : c->clr_s0) : e;
        uint32_t v = a < c->clr_s0 ? 0u : c->clr_fill * 0x01010101u, *q = (uint32_t *)(c->arena + a);
        for (; a + 4u <= stop; a += 4u)
            *q++ = v;
        while (a < stop)
            c->arena[a++] = (uint8_t)v;
    }
    c->clr_at = a;
    if (a >= c->clr_end)
        c->clearing = 0;
    return !c->clearing;
}

static void cl_prepare(cl_t *c)
{
    int changed = c->pmode != c->mode;
    int benign = c->pmode != CL_SPECTRAL && c->mode != CL_SPECTRAL && c->pmode != CL_NPLAY;
    if (!c->reset && changed && benign) {
        uint32_t i;
        cl_filters_reset(c);
        for (i = 0; i < CL_PSLEN; i++)                   /* pitch_shifter_.Clear() */
            c->ps.buf[i] = 0;
        c->ps.wp = 0;
        c->pmode = c->mode;
    }
    if (c->reset || (changed && !benign)) {
        cl_alloc(c);
        c->reset = 0;
        c->pmode = c->mode;
    }
}

/* ProcessGranular: the buffers recorded, the mode's player (n frames of in -> out, at the granular rate) */
static void cl_core(cl_t *c, const int32_t *in, int32_t *out, uint32_t n)
{
    const cl_par_t *p = &c->p;
    int32_t k;
    if (c->mode != CL_SPECTRAL)
        for (k = 0; k < c->nch; k++)
            cl_buf_write(&c->buf[k], in + k, (int32_t)n, !p->freeze);
    switch (c->mode) {
    case CL_GRANULAR: {
        int32_t d = p->density, t = p->texture;
        c->detseed = d < 32768;
        c->ovl = d >= 34734 ? px_m(d - 34734, 138936, 16) : d <= 30802 ? px_m(30802 - d, 138936, 16) : 0;   /* 0.53, 0.47, 2.12 */
        c->wshape = t < 49152 ? px_m(t, 87359, 16) : 65536;   /* 1.333 */
        if (c->ovl > 65536)
            c->ovl = 65536;
        if (c->wshape > 65536)
            c->wshape = 65536;
        cl_gr_play(c, p, out, n);
        break;
    }
    case CL_STRETCH:
        cl_ws_play(c, p, out, n);
        break;
    case CL_LOOPING:
        cl_lp_play(c, p, out, n);
        break;
    default: {
        int32_t w = p->size - 32768, r = p->density - 32768;
        c->sp_q = p->texture;
        c->sp_rr = 655 + px_m(64881, p->density, 16);    /* 0.01 + 0.99 density */
        c->sp_warp = 4 * px_m(px_m(w, w, 16), w, 16) + 32768;
        r = px_m(px_m(r, r, 16), 275251, 16) - 3277;     /* (density - 0.5)^2 4.2 - 0.05 */
        c->sp_rand = r < 0 ? 0 : r > 65536 ? 65536 : r;
        for (k = 0; k < c->nch; k++)
            cl_stft_io(&c->st[k], in + k, out + k, (int32_t)n);
        if (c->nch == 1)
            for (k = 0; k < (int32_t)n; k++)
                out[2 * k + 1] = out[2 * k];
        break;
    }
    }
}

/* one block: il, ir (n samples at the caller's rate, Q15) -> ol, or (may be il, ir) */
static void cl_process(cl_t *c, const cl_par_t *par, const int32_t *il, const int32_t *ir, int32_t *ol, int32_t *orr,
                       uint32_t n)
{
    uint32_t m = c->outer ? n >> 1 : n, i;
    int32_t *in = c->in, *out = c->out, fb, frz, fbg, k;
    int32_t x[2 * CL_BLK];
    const cl_par_t *p = &c->p;
    c->p = *par;
    cl_prepare(c);
    if (c->clearing) {                                   /* the buffers are being cleared: the dry passes */
        cl_clear(c, 8192u);
        for (i = 0; i < n; i++) {
            ol[i] = il[i];
            orr[i] = ir[i];
        }
        return;
    }
    /* the input at the processor's rate, 16 bit as Clouds' codec, mono mixed if so */
    for (i = 0; i < n; i++) {
        x[2 * i] = cl_clip16(il[i]);
        x[2 * i + 1] = cl_clip16(ir[i]);
    }
    if (c->outer)
        cl_hb_down(&c->sdn, x, in, n);
    else
        for (i = 0; i < 2u * n; i++)
            in[i] = x[i];
    if (c->nch == 1)
        for (i = 0; i < m; i++)
            in[2 * i] = in[2 * i + 1] = (in[2 * i] + in[2 * i + 1]) >> 1;
    /* the feedback, high-passed (no build-up of the lowest frequencies) */
    c->frz_lp += px_m((p->freeze ? 1 << 30 : 0) - c->frz_lp, c->k_frz, 30);
    fb = p->feedback;
    {
        int32_t hz16 = (20 << 16) + 100 * px_m(fb, fb, 16);   /* Hz, Q16 */
        int32_t sr = c->lofi ? c->rate / 2 : c->rate;   /* (Clouds: sample_rate(), the granular rate) */
        uint32_t f = (uint32_t)(((int64_t)hz16 << 16) / sr);
        cl_svfk_t K;
        cl_svf_set(&K, f, 65536);
        cl_svf_run(&K, &c->fbf[0], c->fb_, m, 1);
        cl_svf_run(&K, &c->fbf[1], c->fb_ + 1, m, 1);
    }
    frz = c->frz_lp >> 14;                               /* Q16 */
    fbg = px_m(fb, 65536 - frz, 16);
    if (fbg) {
        int32_t g14 = px_m(fbg, 91750, 16);              /* fb_gain 1.4 */
        for (i = 0; i < 2u * m; i++)
            in[i] += px_m(fbg, cl_soft(px_m(g14, c->fb_[i], 16) + in[i]) - in[i], 16);
    }
    /* the mode's player (low fidelity: at half that rate) */
    if (c->lofi) {
        cl_src_down(&c->sdn2, in, c->ind, m);
        cl_core(c, c->ind, c->outd, m / 2u);
        cl_src_up(&c->sup2, c->outd, out, m / 2u);
    } else {
        cl_core(c, in, out, m);
    }
    /* the diffusion, the pitch shifter, the filters */
    if (c->mode != CL_SPECTRAL) {
        int32_t t = p->texture;
        c->df_amt = c->mode == CL_GRANULAR ? (t > 49152 ? (t - 49152) * 4 : 0) : p->density;
        if (c->df_amt) {                                 /* (at 0 it changes nothing: skipped; its lines cleared
                                                          * before it comes back, as if it had run on silence) */
            if (!c->df_on) {
                uint32_t k;
                for (k = 0; k < CL_DFLEN; k++)
                    c->df.buf[k] = 0;
                c->df_on = 1;
            }
            cl_df_run(c, out, m);
        } else {
            c->df_on = 0;
        }
    }
    if (c->mode == CL_LOOPING && (!p->freeze || c->lp.sync)) {
        int32_t s3 = px_m(px_m(p->size, p->size, 16), p->size, 16);
        int32_t tgt = (int32_t)((((int64_t)128 << 16) + (int64_t)(2047 - 128) * s3) * c->rs >> 16);   /* Q16 */
        c->ps_size += px_m(tgt - c->ps_size, c->k_pssize, 30);
        c->ps_ratio = (int32_t)cl_semi24(p->pitch);
        cl_ps_run(c, out, m);
    }
    if (c->mode == CL_LOOPING || c->mode == CL_STRETCH) {
        int32_t t = p->texture, lc, hc, lq;
        uint32_t fl, fh;
        lc = (int32_t)(((int64_t)32768 * cl_semi((t < 32768 ? t - 32768 : 0) * 216)) >> 16);   /* 0.5 x .., Q16 */
        hc = (int32_t)(((int64_t)16384 * cl_semi((t < 32768 ? -32768 : t - 65536) * 216)) >> 16);  /* 0.25 x */
        lc = lc > 32702 ? 32702 : lc < 0 ? 0 : lc;       /* 0.499 */
        hc = hc > 32702 ? 32702 : hc < 0 ? 0 : hc;
        lq = 65536 + px_m(3 * (65536 - fb), 32768 - lc, 16);
        /* the cutoffs are Clouds' fractions of 32 kHz: the same in Hz here, below 0.499 of this rate */
        fl = (uint32_t)(((int64_t)lc << 16) / c->rs);
        fh = (uint32_t)(((int64_t)hc << 16) / c->rs);
        fl = fl > 32702u ? 32702u : fl;
        fh = fh > 32702u ? 32702u : fh;
        {
            cl_svfk_t K;
            cl_svf_set(&K, fl << 16, lq);
            cl_svf_run(&K, &c->lpf[0], out, m, 0);
            cl_svf_run(&K, &c->lpf[1], out + 1, m, 0);
            cl_svf_set(&K, fh << 16, 65536);
            cl_svf_run(&K, &c->hpf[0], out, m, 1);
            cl_svf_run(&K, &c->hpf[1], out + 1, m, 1);
        }
    }
    for (i = 0; i < 2u * m; i++)                         /* what is fed back (not the reverb) */
        c->fb_[i] = out[i];
    /* the reverb */
    {
        int32_t ra = px_m(p->reverb, 62259, 16) + px_m(px_m(fb, 131072 - fb, 16), frz, 16);   /* 0.95 */
        int32_t klp, lp;
        ra = ra < 0 ? 0 : ra > 65536 ? 65536 : ra;
        lp = 39322 + px_m(24248, fb, 16);                /* 0.6 + 0.37 feedback, a sample at 32 kHz */
        klp = cl_kscale(lp << 14, c->k_rvlp_e) >> 18;    /* Q12 here */
        /* (at amount 0 the reverb changes nothing; its tail, at the shortest time, is 60 dB down in ~0.7 s: after 1 s
         * of 0 it stops, and runs again from its silent lines) */
        c->rv_idle = ra ? 0 : c->rv_idle + 1;
        if (c->rv_idle < c->rate / (int32_t)m)
            cl_rv_run(c, out, m, px_m(ra, 35389, 16) >> 4, 1434 + px_m(2580, ra, 16), klp);   /* 0.54; 0.35 + 0.63 */
    }
    /* back at the caller's rate, then BLEND's crossfade with the dry */
    if (c->outer)
        cl_hb_up(&c->sup, out, x, m);
    else
        for (i = 0; i < 2u * n; i++)
            x[i] = out[i];
    {
        int32_t d0 = c->dry_wet, dd = par->dry_wet - d0, g12 = 39322;   /* post_gain 1.2 */
        for (i = 0; i < n; i++) {
            int32_t dw = d0 + (int32_t)(((int64_t)dd * (int32_t)(i + 1)) / (int32_t)n), fi = cl_xf(CL_XFI, dw), fo = cl_xf(CL_XFO, dw);
            int32_t wi = px_m(fi, g12, 15);
            for (k = 0; k < 2; k++) {
                int32_t dry = k ? ir[i] : il[i], y;
                y = px_m(dry, fo, 15) + px_m(x[2 * i + k], wi, 15);
                if (c->orig_out)
                    y = cl_soft(y >> 1);
                else
                    y = 2 * cl_soft(px_m(y, 46341, 15) >> 1);   /* sqrt 2: BLEND 0 is the dry at its level */
                if (k)
                    orr[i] = y;
                else
                    ol[i] = y;
            }
        }
        c->dry_wet = par->dry_wet;
    }
    /* Prepare's work, spread over the blocks */
    if (c->mode == CL_SPECTRAL) {
        int32_t blk = (int32_t)(c->lofi ? m / 2u : m), st = (CL_PVSTEPS * c->nch * blk + CL_HOP - 1) / CL_HOP + 1;
        cl_pv_work(c, st);
        for (k = 0; k < c->nch; k++)
            if (c->st[k].ready - c->st[k].done > 1)      /* behind: catch up */
                cl_pv_work(c, CL_PVSTEPS);
    } else if (c->mode == CL_STRETCH) {
        cl_ws_work(c, c->lofi ? m / 2u : m);
    }
}
