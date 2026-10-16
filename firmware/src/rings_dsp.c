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
/* Rings (Emilie Gillet, MIT; rings/dsp/, pichenettes/eurorack) in fixed point, for the PHYS engine's Rings
 * models (eng_phys.c). Included after phys_dsp.c, whose primitives it uses (px_m, px_exp2, px_sin, px_rand ..).
 *
 * What is here (the names of Rings' classes in brackets):
 *   rg_modal_*    the modal resonator [Resonator] with the internal exciter's pulse and its low-pass
 *   rg_str_*      the string [String] (dispersion, curved bridge, damping filters) with the noise burst [Plucker]
 *                 and the sympathetic strings of [Part::RenderStringVoice]; the quantised-chord tuning
 *   rg_fm_*       the FM voice [FMVoice]
 *   rg_rev_*      the reverb [Reverb] (STRING AND REVERB, and two of the effects of the string synth)
 *   rg_dp_*       the string synth, the easter egg DISASTROUS PEACE [StringSynthPart, StringSynthVoice,
 *                 StringSynthOscillator, StringSynthEnvelope], with its chorus, ensemble and formant filters
 *   rg_lim_*      the limiter [Limiter]
 * One FELUCCA voice is one RINGS voice: the engine hands each of them its own state (rg_voice_t); the
 * polyphony (1, 2 or 4) is the cap of the part's voices (voice.c trk_nvoice), the resolution of the modal
 * resonator (64 / polyphony - 4 modes in Rings), the number of strings a voice has (8 / polyphony) and the
 * width of the chord tables, exactly as in Rings. The part-level processing (the reverb, the chorus, the
 * ensemble, the formant filters and the limiter) runs on the mix of the part's voices (eng_phys.c phys_post).
 *
 * Number formats (no float anywhere):
 *   signals         Q20 (1.0 = 2^20); inside the modes Q22 (as phys_dsp.c); in the delay lines Q13 (16-bit)
 *   frequencies     cycles per sample, Q32; parameters 0..1 are Q16
 *   filters         the Simper SVF of phys_dsp.c (g Q28, r + g Q28, g h Q30) with Rings' tan approximations
 *
 * Departures from Rings, on purpose:
 *   - the sample rate is 44.1 kHz where Rings' is 48: every time constant keeps its value in seconds, the
 *     lengths in samples (the reverb's, the plucker's comb) are those of Rings at that rate;
 *   - control rate: Rings works in blocks of 24 samples, this in 32 (CTL); a parameter that Rings ramps
 *     over a block (position, damping, brightness) is ramped here where it matters (the delay, the mode
 *     amplitudes) and stepped once per block otherwise;
 *   - RG_MODES_TOTAL modes in all (Rings: 64): the voices share them, per voice min(64 / polyphony - 4,
 *     RG_MODES_TOTAL / polyphony); a build with RG_MODES_TOTAL 64 is Rings' resolution (tests/rings_ref.cpp);
 *   - the strings' lines are 1024 + 512 samples of 16 bit (Rings: 2048 + 1024 floats): the string plays
 *     as in Rings down to 43 Hz, below it resampled as Rings does below 11.7 Hz (rg_str_*); the reverb and
 *     the effects have 0.42 x the lengths (RG_REVSCALE) in 16-bit lines: a smaller room, the same topology;
 *   - there is no audio input (the FM-1 has none): the internal exciter is always on, the strum is the note-on
 *     (and a jump of the pitch of a sounding voice, like Rings' internal strum on a note CV), the note
 *     filter's median and lag are the voice's own glide;
 *   - the output is mono: a voice plays the modes' odd + even sum, a string's out - aux (its pickup comb), the
 *     FM voice's out; the string synth's effects process a mono sum;
 *   - the sine of the FM voice and the effects' LFOs are px_sin's polynomial instead of lut_sine. */
#include "rings_tables.h"

#ifndef RG_MODES_TOTAL
#define RG_MODES_TOTAL 40                                /* modes of all the voices of a part (Rings: 64) */
#endif
#define RG_MAXV 4                                        /* voices: Rings' kMaxPolyphony */
#define RG_MAXRES (RG_MODES_TOTAL < 60 ? RG_MODES_TOTAL : 60)   /* modes of a voice at polyphony 1 */
#define RG_NSTR 8                                        /* strings: kNumStrings */
#define RG_LINE 1024                                     /* string delay line, samples (Rings: 2048) */
#define RG_LMASK (RG_LINE - 1)
#define RG_SLINE 512                                     /* the dispersion all-pass line (Rings: 1024) */
#define RG_SMASK (RG_SLINE - 1)
#define RG_STRBYTES ((RG_LINE + RG_SLINE) * 2)           /* one string's lines in the arena */
#define RG_ONE (1 << 20)
#define RG_FMAX 2143260078u                              /* 0.499 cycles */
#define RG_F49 2104533975u                               /* 0.49 */

enum { RG_MODAL, RG_SYMP, RG_STRING, RG_FM, RG_QUANT, RG_STRREV, RG_DP, RG_NMODEL };   /* ResonatorModel + the string synth */

/* the patch of a block: Q16 0..1, and what the voice plays */
typedef struct {
    uint32_t f;                                          /* the voice's frequency, cycles Q32 */
    int32_t structure, brightness, damping, position;    /* Q16 */
    int32_t poly;                                        /* 1, 2, 4 */
    int32_t voice;                                       /* 0 .. poly - 1 */
    int32_t chord;                                       /* 0 .. 10: the quantised chords, the string synth's */
    int32_t fx;                                          /* the string synth's effect, 0 .. 5 */
    int32_t note;                                        /* the voice's note, MIDI semitones Q16 */
    int32_t tonic;                                       /* the sympathetic strings' reference note (Rings' tonic), MIDI Q16 */
} rg_patch_t;

/* ---------------------------------------------------------------- math --- */
static inline int32_t rg_lerp_tab(const int32_t *t, int32_t x16)   /* Interpolate(lut, x, size): x Q16 0..1 (table of 257) */
{
    int32_t i = x16 >> 8, f = (x16 & 255) << 8;
    if (i >= 256)
        return t[256];
    return t[i] + px_m(t[i + 1] - t[i], f, 16);
}

/* 2^(x / 12), x semitones Q16: Q16 */
static inline uint32_t rg_semi(int32_t x) { return px_exp2(px_m(x, 87381, 20)); }

/* a frequency times 2^(x / 12) (x semitones Q16), clamped to 0.499 */
static inline uint32_t rg_fshift(uint32_t f, int32_t x)
{
    uint64_t v = ((uint64_t)f * rg_semi(x)) >> 16;
    return v > RG_FMAX ? RG_FMAX : (uint32_t)v;
}

/* tan(pi f) exactly (stmlib FREQUENCY_ACCURATE), f Q32 limited to 0.44 (g + r must stay under 8 in Q28): Q28 */
static int32_t rg_tan_acc(uint32_t f)
{
    int32_t s, c;
    if (f > 1889785610u)
        f = 1889785610u;                                 /* 0.44 */
    s = px_sin(f >> 1);
    c = px_sin((f >> 1) + 0x40000000u);
    return (int32_t)(((uint64_t)s << 28) / (uint32_t)c);
}

/* stmlib FREQUENCY_DIRTY: tan(pi f) ~ f (pi + 0.3736 pi^3 f^2) (good below 8 kHz); f Q32 (< 0.5), Q28 */
static int32_t rg_tan_dirty(uint32_t f)
{
    int64_t x = f >> 1, x2 = (x * x) >> 31;              /* Q31 */
    int64_t p = 52707179 + ((x2 * 194371346) >> 31);     /* pi + 0.3736 pi^3 f^2, Q24 */
    return (int32_t)((x * p) >> 27);
}

typedef px_svf_t rg_svf_t;                               /* g Q28, r + g Q28, g h Q30 */

/* the coefficients from g (Q28) and q (Q8, > 0) */
static void rg_svf_g(rg_svf_t *c, int32_t g, uint32_t q8)
{
    uint32_t r = (uint32_t)((((uint64_t)1 << 36) + q8 - 1u) / q8);       /* 1 / q, rounded up */
    uint64_t den = ((uint64_t)1 << 28) + (((uint64_t)r * (uint32_t)g) >> 28) + (((uint64_t)g * (uint32_t)g) >> 28);
    uint32_t d24 = (uint32_t)((den + 15u) >> 4);
    uint32_t h = (uint32_t)(((uint64_t)1 << 54) / d24);
    c->g = g;
    c->rpg = (int32_t)(r + (uint32_t)g);
    c->gh = (int32_t)(((uint64_t)h * (uint32_t)g) >> 28);
}

/* one pass of the SVF (states s[2], Q24), input Q24: lp in *lp, bp returned; as phys_dsp.c's strike filter */
static inline int32_t rg_svf_step(const rg_svf_t *c, int32_t *s, int64_t in, int32_t *lp)
{
    int32_t t, bp;
    t = (int32_t)((c->gh * (in - px_m(c->rpg, s[0], 28) - s[1])) >> 30);
    bp = s[0] + t;
    s[0] = bp + t;
    t = px_m(c->g, bp, 28);
    *lp = s[1] + t;
    s[1] = *lp + t;
    return bp;
}

/* ------------------------------------------------------- the modal voice --- */
typedef struct {
    int32_t s[RG_MAXRES][2];                             /* the modes' states, Q22 */
    int32_t e1, e2;                                      /* the excitation low-pass, Q24 */
    int32_t pos;                                         /* the position the amplitudes were last made for, Q16 */
} rg_modal_t;

typedef struct { int32_t g, rpg, gh, a, da; } rg_mode_t;
typedef struct {                                         /* per block */
    rg_mode_t m[RG_MAXRES];
    uint32_t n;                                          /* modes below 0.49 */
    rg_svf_t ex;                                         /* the excitation low-pass */
    int64_t pulse;                                       /* the strum's pulse, Q24 (0 = none) */
    int32_t ramp;                                        /* 1 = the amplitudes move over the block */
} rg_modal_blk_t;

/* the modes a voice has at polyphony poly: Resonator::set_resolution, capped to the modes the build has */
static inline uint32_t rg_resolution(int32_t poly)
{
    int32_t r = 64 / poly - 4, c = RG_MODES_TOTAL / poly;
    r = r < c ? r : c;
    r -= r & 1;
    return (uint32_t)(r < 2 ? 2 : r > RG_MAXRES ? RG_MAXRES : r);
}

/* Part::RenderModalVoice + Resonator::ComputeFilters, the per-block part. strum: the pulse of the internal exciter */
static __attribute__((noinline)) void rg_modal_block(rg_modal_blk_t *B, rg_modal_t *M, const rg_patch_t *P, int strum)
{
    int32_t st = rg_lerp_tab(RG_STIFF, P->structure);                  /* Q16, signed */
    int32_t brs = px_m(P->brightness, P->brightness, 16);             /* brightness^2 (Part::RenderModalVoice) */
    int32_t att = 65536 - P->structure, b, qloss, qrate, pos = P->position;
    int32_t cutoff = px_m(P->brightness, 131072 - P->brightness, 16);   /* Part::Process: b (2 - b) */
    uint32_t res = rg_resolution(P->poly), f = P->f, fc, i;
    uint64_t q8, harm, stretch = 65536;
    int32_t dph;
    /* the internal exciter: a pulse, low-passed at f 2^((cutoff - 0.5) 8) (q 1.5), 0.25 2^(2 fc^2) / fc */
    fc = rg_fshift(f, (cutoff - 32768) * 96);        /* semitones Q16 */
    if (fc < 65536u)
        fc = 65536u;
    if (fc > 2143289344u)                            /* min (.., 0.499) */
        fc = 2143289344u;
    rg_svf_g(&B->ex, rg_tan_dirty(fc), 384);
    B->pulse = 0;
    if (strum) {
        uint32_t c2 = (uint32_t)(((uint64_t)fc * fc) >> 32);
        uint64_t mult = px_exp2((int32_t)(c2 >> 15));    /* 2^(2 fc^2): Q16 */
        B->pulse = (int64_t)((mult << 38) / fc);
    }
    /* Resonator::ComputeFilters */
    att = px_m(att, att, 16);
    att = px_m(att, att, 16);
    att = px_m(att, att, 16);                            /* (1 - structure)^8 */
    b = px_m(brs, 65536 - px_m(13107, att, 16), 16);     /* brightness (1 - 0.2 att) */
    qloss = px_m(px_m(b, 131072 - b, 16), 55706, 16) + 9830;            /* b (2 - b) 0.85 + 0.15 */
    qrate = px_m(px_m(P->structure, 131072 - P->structure, 16), 6554, 16);   /* s (2 - s) 0.1 */
    q8 = ((uint64_t)500 * (uint32_t)rg_lerp_tab(RG_DEC4, P->damping)) >> 8;   /* 500 q-table, Q8 */
    harm = f;
    dph = pos;
    for (i = 0; i < res; i++) {
        uint64_t mf = (harm * stretch) >> 16;
        int32_t a, a1;
        if (mf >= RG_F49)
            break;
        {
            rg_svf_t c;
            rg_svf_g(&c, px_fasttan((uint32_t)mf), (uint32_t)(256u + ((mf * q8) >> 32)));
            B->m[i].g = c.g;
            B->m[i].rpg = c.rpg;
            B->m[i].gh = c.gh;
        }
        /* the amplitude 0.5 + 0.5 cos(2 pi position i) (CosineOscillator), now and at the previous block's position */
        a = (px_sin(((uint32_t)dph * i << 16) + 0x40000000u) >> 3) + (1 << 27);       /* 0.5 + 0.5 cos, Q28 */
        a1 = (px_sin(((uint32_t)M->pos * i << 16) + 0x40000000u) >> 3) + (1 << 27);
        B->m[i].a = a1;
        B->m[i].da = a - a1;
        stretch = (uint64_t)((int64_t)stretch + st);
        st = px_m(st, st < 0 ? 60948 : 64225, 16);       /* * 0.93 / 0.98 */
        qloss += px_m(qrate, 65536 - qloss, 16);
        harm += f;
        q8 = (q8 * (uint32_t)qloss) >> 16;
    }
    B->n = i;
    B->ramp = M->pos != pos;
    M->pos = pos;
    for (; i < RG_MAXRES; i++)
        M->s[i][0] = M->s[i][1] = 0;
    for (i = 0; i < B->n; i++) {
        M->s[i][0] = px_clamp(M->s[i][0], -(1 << 28), 1 << 28);
        M->s[i][1] = px_clamp(M->s[i][1], -(1 << 28), 1 << 28);
    }
}

/* n (<= 32) samples of the resonator into out (Q20): the pulse through the excitation low-pass (input * 0.125),
 * then the modes (band-pass), each at its amplitude */
static void rg_modal_run(const rg_modal_blk_t *B, rg_modal_t *M, int32_t *out, uint32_t n)
{
    uint32_t i, k, nm = B->n;
    int32_t x[32], a[RG_MAXRES], da[RG_MAXRES];
    if (n > 32u)
        n = 32u;
    for (i = 0; i < n; i++)
        x[i] = 0;
    if (B->pulse || M->e1 || M->e2) {
        int32_t s[2] = {M->e1, M->e2}, big = 0, lp;
        for (i = 0; i < n; i++) {
            int32_t bp = rg_svf_step(&B->ex, s, !i ? B->pulse : 0, &lp);
            x[i] = lp >> 5;                              /* Q24 -> Q22, * 0.125 */
            big |= (lp + 256) >> 9;
            (void)bp;
        }
        if (!big && s[0] > -256 && s[0] < 256 && s[1] > -256 && s[1] < 256)
            s[0] = s[1] = 0;                             /* rung out */
        M->e1 = px_clamp(s[0], -(1 << 30), 1 << 30);
        M->e2 = px_clamp(s[1], -(1 << 30), 1 << 30);
    }
    for (k = 0; k < nm; k++) {
        a[k] = B->m[k].a;
        da[k] = B->ramp ? B->m[k].da / (int32_t)n : 0;
    }
    for (i = 0; i < n; i++) {
        int64_t acc = 0;
        const rg_mode_t *c = B->m;
        int32_t (*st)[2] = M->s, xi = x[i];
        for (k = 0; k < nm; k++, c++, st++) {
            int32_t s1 = st[0][0], s2 = st[0][1], t, v;
            t = px_m(c->gh, xi - px_m(c->rpg, s1, 28) - s2, 30);
            v = s1 + t;
            st[0][0] = v + t;
            t = px_m(c->g, v, 28);
            st[0][1] = s2 + t + t;
            acc += (int64_t)a[k] * v;
            a[k] += da[k];
        }
        out[i] = (int32_t)(acc >> 30);                   /* Q28 * Q22 -> Q20 */
    }
}

/* ------------------------------------------------------------ the string --- */
/* One string [String]: a delay line of 16-bit values (Q12, +-8), read with Hermite interpolation, a dispersion
 * all-pass, the curved bridge, the FIR + IIR loss filters, a comb pickup. */
typedef struct {
    uint32_t f;                                          /* the frequency it glides to its target at, cycles Q32 */
    uint32_t wp, swp;                                    /* write pointers of the main and the dispersion line */
    uint32_t delay;                                      /* samples Q16, ramped over the block */
    int32_t cpos;                                        /* the pickup position (clamped), Q16 */
    int32_t pcomp;                                       /* the IIR delay compensation, Q30 */
    int32_t pdamp, pbr;                                  /* the FIR filter's loss and brightness, Q30 */
    int32_t dnoise, curved;                              /* dispersion noise, curved bridge (Q20) */
    int32_t dcx, dcy, dce;                               /* the DC blocker (curved bridge) */
    int32_t fx1, fx2;                                    /* the FIR's x_, x__ (Q20) */
    int32_t qe;                                          /* the delay line's requantisation error (Q20): fed back, so that a quiet loop does not die in the 16-bit steps */
    int32_t sv[2];                                       /* the IIR low-pass (Q24) */
    uint32_t src;                                        /* the resampler's phase, Q30 */
    int32_t o0, o1, a0, a1;                              /* the last two out / pickup samples */
    int32_t ein;                                         /* the input since the last step */
    uint32_t rng;
    uint32_t lfo;                                        /* the pickup LFO, turns Q32 */
} rg_str_t;

typedef struct {                                         /* per block */
    uint32_t delay;                                      /* target, Q16 */
    int32_t src_ratio;                                   /* Q30 */
    int32_t cpos, comp, damp, br;                        /* targets: Q16, Q30, Q30, Q30 */
    rg_svf_t iir;
    int32_t nf, ng;                                      /* dispersion noise filter (Q30), its gain (Q16) */
    int32_t m_ap, noise_amt, curving, ac_amt, ap_gain;   /* Q30 (ac_amt Q16) */
    uint8_t disp;                                        /* the dispersion branch (STRING models) */
} rg_str_blk_t;

static inline uint32_t rg_period(uint32_t f) { return (uint32_t)(((uint64_t)1 << 48) / f); }

/* Squash (Part): a steep S curve of 0..1, Q16 */
static int32_t rg_squash(int32_t x)
{
    int32_t k;
    if (x < 32768) {
        x *= 2;
        x = px_m(x, x, 16);
        x = px_m(x, x, 16);
        x = px_m(x, x, 16);
        x = px_m(x, x, 16);
        return x >> 1;
    }
    k = 131072 - 2 * x;
    k = px_m(k, k, 16);
    k = px_m(k, k, 16);
    k = px_m(k, k, 16);
    k = px_m(k, k, 16);
    return 65536 - (k >> 1);
}

/* String::ProcessInternal, the per-block part. f: cycles Q32; disp: -1..1 Q16 (signed); the others 0..1 Q16 */
static __attribute__((noinline)) void rg_str_block(rg_str_blk_t *B, rg_str_t *S, uint32_t f, int32_t disp, int32_t brightness,
                                                     int32_t damping, int32_t position, int has_disp)
{
    uint32_t delay, ratio, df;
    int32_t lf, b2, dcut, ad, nl, an, ti;
    uint64_t rt60, a, x;
    int32_t coef, idx, fr;
    if (f < 65536u)
        f = 65536u;
    delay = rg_period(f);
    if (delay > (uint32_t)(RG_LINE - 4) << 16)
        delay = (uint32_t)(RG_LINE - 4) << 16;
    if (delay < 4u << 16)
        delay = 4u << 16;
    ratio = (uint32_t)(((uint64_t)delay * f) >> 18);     /* delay * frequency, Q30 */
    if (ratio >= 1073634406u) {                          /* >= 0.9999: no resampling */
        ratio = 1u << 30;
        S->src = 1u << 30;
    }
    B->src_ratio = (int32_t)ratio;
    B->delay = delay;
    ad = position - 32768;
    B->cpos = 32768 - px_m(64225, ad < 0 ? -ad : ad, 16);   /* 0.5 - 0.98 |pos - 0.5| */
    lf = px_m(damping, 131072 - damping, 16);            /* d (2 - d) */
    rt60 = (uint64_t)3087 * px_exp2(lf * 8);             /* 0.07 s * 2^(8 lf), samples Q16 */
    a = (((uint64_t)delay * 10u) << 20) / (ratio >> 10); /* 10 delay / ratio, Q16 */
    x = (a << 16) / rt60;                                /* octaves, Q16 */
    if (x > 693600u)                                     /* -127 semitones */
        x = 693600u;
    coef = (int32_t)(px_exp2(-(int32_t)x) << 14);        /* the loss per pass, Q30 */
    b2 = px_m(brightness, brightness, 16);
    dcut = 24 * 65536 + px_m(px_m(damping, damping, 16), 48 * 65536, 16) + px_m(b2, 24 * 65536, 16);
    if (dcut > 84 * 65536)
        dcut = 84 * 65536;
    df = rg_fshift(f, dcut);
    B->br = b2 << 14;
    if (damping >= 62259) {                              /* crossfade to infinite decay */
        ti = (damping - 62259) * 20;
        coef += (int32_t)(((int64_t)ti * ((1 << 30) - coef)) >> 16);
        B->br += (int32_t)(((int64_t)ti * ((1 << 30) - B->br)) >> 16);
        df += (uint32_t)(((uint64_t)(2147332013u - df) * (uint32_t)ti) >> 16);   /* -> 0.4999 */
        dcut += px_m(ti, 128 * 65536 - dcut, 16);
    }
    if (df > RG_FMAX)
        df = RG_FMAX;
    B->damp = coef;
    rg_svf_g(&B->iir, rg_tan_acc(df), 128);              /* q 0.5 */
    idx = dcut >> 16;
    fr = dcut & 0xFFFF;
    if (idx > 255)
        idx = 255;
    B->comp = (1 << 30) - (RG_SVFSHIFT[idx] + px_m(RG_SVFSHIFT[idx + 1] - RG_SVFSHIFT[idx], fr, 16));
    B->disp = (uint8_t)has_disp;
    if (!S->rng)
        S->rng = 0x6A09E667u ^ f;
    if (!S->delay) {                                     /* a fresh string starts at its targets (String::Init) */
        S->delay = delay;
        S->cpos = B->cpos;
        S->pcomp = B->comp;
        S->pdamp = B->damp;
        S->pbr = B->br;
    }
    if (has_disp) {
        int32_t sp = disp <= 0 ? 0 : px_m(px_m(disp, 131072 - disp, 16), 31130, 16);   /* d (2 - d) 0.475 */
        int32_t ap_ = disp < 0 ? -disp : disp;
        nl = disp < 0 ? -disp : 0;                       /* bridge curving */
        an = disp > 49152 ? 4 * (disp - 49152) : 0;      /* 4 (d - 0.75) */
        B->m_ap = sp << 14;
        B->noise_amt = px_m(px_m(an, an, 16), 1638, 16) << 14;   /* ^2 * 0.025 */
        B->ac_amt = nl;
        B->curving = px_m(px_m(nl, nl, 16), 655, 16) << 14;      /* ^2 * 0.01 */
        B->ap_gain = -(int32_t)(((int64_t)663568203 * disp) / (9830 + ap_));   /* -0.618 d / (0.15 + |d|) */
        {
            uint32_t nfq = px_exp2((brightness - 65536) * 4);        /* 2^(4 (b - 1)): the noise filter, Q16 */
            B->nf = (int32_t)(nfq << 14);
            B->ng = (int32_t)(((uint64_t)1 << 32) / (13107u + nfq));  /* 1 / (0.2 + nf), Q16 */
        }
    }
}

static inline int32_t rg_hermite(const int16_t *l, uint32_t wp, uint32_t delay)   /* DelayLine::ReadHermite, Q12 -> Q20 */
{
    /* in Q20 throughout: the 16-bit steps' rounding would otherwise bias every pass the same way (a DC that builds up) */
    uint32_t t = wp + (delay >> 16);
    int32_t xm1 = l[(t - 1u) & RG_LMASK] * 256, x0 = l[t & RG_LMASK] * 256, x1 = l[(t + 1u) & RG_LMASK] * 256,
            x2 = l[(t + 2u) & RG_LMASK] * 256;
    int64_t f = (int64_t)(delay & 0xFFFF);
    int32_t c = (x1 - xm1) >> 1, v = x0 - x1, w = c + v, a = w + v + ((x2 - x0) >> 1), bn = w + a;
    int64_t y = ((a * f) >> 16) - bn;
    y = ((y * f) >> 16) + c;
    return (int32_t)((y * f) >> 16) + x0;
}

static inline int32_t rg_lread(const int16_t *l, uint32_t wp, uint32_t delay)     /* DelayLine::Read: linear, Q20 */
{
    uint32_t t = wp + (delay >> 16);
    int32_t a = l[t & RG_LMASK], b = l[(t + 1u) & RG_LMASK];
    return (a + (((b - a) * (int32_t)(delay & 0xFFFF)) >> 16)) << 8;
}

/* n samples of the string excited by in[] (Q20): out, pick += (Q20). line, sline: its delay lines */
static void rg_str_run(const rg_str_blk_t *B, rg_str_t *S, int16_t *line, int16_t *sline, const int32_t *in, int32_t *out,
                       int32_t *pick, uint32_t n)
{
    uint32_t i, wp = S->wp, swp = S->swp, delay = S->delay, rs = S->rng;
    int32_t cpos = S->cpos, pcomp = S->pcomp, pdamp = S->pdamp, pbr = S->pbr;
    int32_t dcpos, dcomp, ddamp, dbr, dnoise = S->dnoise, curved = S->curved;
    int32_t dcx = S->dcx, dcy = S->dcy, dce = S->dce, fx1 = S->fx1, fx2 = S->fx2, ein = S->ein, qe = S->qe;
    int32_t o0 = S->o0, o1 = S->o1, a0 = S->a0, a1 = S->a1;
    uint32_t src = S->src;
    int32_t ddelay = ((int32_t)B->delay - (int32_t)delay) / (int32_t)n;
    dcpos = (B->cpos - cpos) / (int32_t)n;
    dcomp = (B->comp - pcomp) / (int32_t)n;
    ddamp = (B->damp - pdamp) / (int32_t)n;
    dbr = (B->br - pbr) / (int32_t)n;
    for (i = 0; i < n; i++) {
        ein += in[i];
        src += (uint32_t)B->src_ratio;
        if (src > 1u << 30) {
            uint32_t dl = delay, cd;
            int32_t s;
            src -= 1u << 30;
            cd = (uint32_t)(((uint64_t)dl * (uint32_t)cpos) >> 16);
            dl = (uint32_t)(((uint64_t)dl * (uint32_t)pcomp) >> 30);       /* the IIR's delay */
            dl -= 1u << 16;                                                /* the FIR's */
            if (B->disp) {
                int32_t noise = (int32_t)(px_rand(&rs) >> 11) - (1 << 20), sac, val;
                uint32_t ap, mn;
                int64_t fm;
                noise = px_m(noise, B->ng, 16);
                dnoise += px_m(noise - dnoise, B->nf, 30);
                fm = (1 << 30) + px_m(dnoise, B->noise_amt, 20) - px_m(curved, B->curving, 20);
                dl = (uint32_t)(((uint64_t)dl * (uint64_t)(fm < 0 ? 0 : fm)) >> 30);
                ap = (uint32_t)(((uint64_t)dl * (uint32_t)B->m_ap) >> 30);
                mn = dl - ap;
                if (ap >= 4u << 16 && mn >= 4u << 16) {
                    int32_t rd, w;
                    s = rg_hermite(line, wp, mn);
                    rd = (int32_t)sline[(swp + (ap >> 16)) & RG_SMASK] << 8;   /* DelayLine::Allpass */
                    w = s + px_m(B->ap_gain, rd, 30);
                    sline[swp] = (int16_t)px_clamp((w + 128) >> 8, -32768, 32767);
                    swp = (swp - 1u) & RG_SMASK;
                    s = rd - px_m(w, B->ap_gain, 30);
                } else {
                    s = rg_hermite(line, wp, dl);
                }
                {   /* the DC blocker (1 - 20 / sr), then the bridge */
                    int64_t acc = (int64_t)1073254874 * dcy + dce;
                    sac = s - dcx + (int32_t)(acc >> 30);
                    dce = (int32_t)(acc & 0x3FFFFFFF);
                    dcx = s;
                    dcy = sac;
                }
                s += px_m(sac - s, B->ac_amt, 16);
                val = (s < 0 ? -s : s) - 26214;                            /* |s| - 0.025 */
                curved = val > 0 ? (s > 0 ? 2 * val : -3 * val) : 0;
            } else {
                s = rg_hermite(line, wp, dl);
            }
            s = px_clamp(s + ein, -(7 << 20), 7 << 20);
            ein = 0;
            {   /* the FIR damping filter: damping (h0 x_ + h1 (x + x__)), h0 = (1 + br) / 2, h1 = (1 - br) / 4 */
                int32_t h0 = ((1 << 30) + pbr) >> 1, h1 = ((1 << 30) - pbr) >> 2;
                int32_t y = (int32_t)(((int64_t)pdamp * (int32_t)(((int64_t)h0 * fx1 + (int64_t)h1 * (s + fx2) + (1 << 29)) >> 30) + (1 << 29)) >> 30);   /* rounded: a floor would bias every pass */
                fx2 = fx1;
                fx1 = s;
                s = y;
            }
            {   /* the IIR low-pass */
                int32_t lp;
                rg_svf_step(&B->iir, S->sv, (int64_t)s << 4, &lp);
                s = (lp + 8) >> 4;
            }
            s = px_clamp(s, -(7 << 20), 7 << 20);
            {
                int32_t w = s + qe, q = (w + 128) >> 8;
                qe = w - (q << 8);
                line[wp] = (int16_t)q;
            }
            wp = (wp - 1u) & RG_LMASK;
            o1 = o0;
            a1 = a0;
            o0 = s;
            a0 = rg_lread(line, wp, cd);
            pbr += dbr;
            pdamp += ddamp;
            pcomp += dcomp;
            delay += (uint32_t)ddelay;
            cpos += dcpos;
        }
        out[i] += o1 + px_m(o0 - o1, (int32_t)src, 30);
        pick[i] += a1 + px_m(a0 - a1, (int32_t)src, 30);
    }
    S->wp = wp;
    S->swp = swp;
    S->delay = delay;
    S->cpos = cpos;
    S->pcomp = pcomp;
    S->pdamp = pdamp;
    S->pbr = pbr;
    S->dnoise = dnoise;
    S->curved = curved;
    S->dcx = dcx;
    S->dcy = dcy;
    S->dce = dce;
    S->fx1 = fx1;
    S->fx2 = fx2;
    S->qe = qe;
    S->ein = px_clamp(ein, -(7 << 20), 7 << 20);
    S->o0 = o0;
    S->o1 = o1;
    S->a0 = a0;
    S->a1 = a1;
    S->src = src;
    S->rng = rs;
}

/* ------------------------------------------------------------- Plucker --- */
/* The internal exciter of the strings [Plucker]: a burst of noise a fraction of a period long (the pluck
 * position), a comb at that fraction, a low-pass. */
typedef struct {
    int16_t line[256];                                   /* the comb's delay line (Q12) */
    uint32_t wp, remaining, period;                      /* period: comb delay, Q16 */
    int32_t gain;                                        /* comb gain, Q30 */
    int32_t sv[2];
    rg_svf_t svf;
    uint32_t rng;
    uint8_t live;                                        /* 0 once the tail has rung out */
} rg_pl_t;

static void rg_pl_trigger(rg_pl_t *P, uint32_t f, uint32_t cutoff, int32_t position)
{
    int32_t ratio = px_m(position, 58982, 16) + 3277;    /* position 0.9 + 0.05 */
    uint64_t per = ((uint64_t)rg_period(f) * (uint32_t)ratio) >> 16;   /* the comb period, Q16 samples */
    P->remaining = (uint32_t)(per >> 16);
    while (per >= (255u << 16))
        per >>= 1;
    P->period = (uint32_t)per;
    P->gain = px_m(65536 - position, 52429, 16) << 14;   /* (1 - position) 0.8 */
    rg_svf_g(&P->svf, rg_tan_dirty(cutoff > RG_FMAX ? RG_FMAX : cutoff), 256);   /* q 1 */
    if (!P->rng)
        P->rng = 0x9E3779B9u ^ f;
    P->live = 1;
}

static void rg_pl_run(rg_pl_t *P, int32_t *out, uint32_t n)
{
    uint32_t i, wp = P->wp;
    int32_t busy = 0;
    if (!P->live) {
        for (i = 0; i < n; i++)
            out[i] = 0;
        return;
    }
    for (i = 0; i < n; i++) {
        int32_t in = 0, lp, c;
        uint32_t t = wp + (P->period >> 16);
        int32_t a = P->line[t & 255u], b = P->line[(t + 1u) & 255u];
        if (P->remaining) {
            in = (int32_t)(px_rand(&P->rng) >> 11) - (1 << 20);   /* 2 rand - 1 */
            P->remaining--;
        }
        c = (a + (((b - a) * (int32_t)(P->period & 0xFFFF)) >> 16)) << 8;
        c = in + px_m(P->gain, c, 30);
        P->line[wp] = (int16_t)px_clamp((c + 128) >> 8, -32768, 32767);
        wp = (wp - 1u) & 255u;
        rg_svf_step(&P->svf, P->sv, (int64_t)c << 4, &lp);
        out[i] = lp >> 4;
        busy |= (c + 512) >> 10;
    }
    P->wp = wp;
    if (!P->remaining && !busy) {                        /* rung out: stop (and keep the line clean) */
        for (i = 0; i < 256; i++)
            P->line[i] = 0;
        P->sv[0] = P->sv[1] = 0;
        P->live = 0;
    }
}

/* ------------------------------------------------- the sympathetic notes --- */
/* Part::ComputeSympatheticStringsNotes; every note in semitones Q16 relative to the voice's note. */
#define RGC(x) ((int16_t)((x) * 100.0 + ((x) < 0 ? -0.5 : 0.5)))   /* hundredths of a semitone */
#ifndef RG_ORIGINAL_CHORDS                              /* Bryan Noll's chords, as Rings' dsp.h defines BRYAN_CHORDS */
static const int16_t RG_CH1[11][8] = {
    {RGC(-12), RGC(-0.01), 0, RGC(0.01), RGC(0.02), RGC(11.98), RGC(11.99), RGC(12)},
    {RGC(-12), RGC(-5), 0, RGC(6.99), RGC(7), RGC(11.99), RGC(12), RGC(19)},
    {RGC(-12), RGC(-5), 0, RGC(5), RGC(7), RGC(11.99), RGC(12), RGC(17)},
    {RGC(-12), RGC(-5), 0, RGC(3), RGC(7), RGC(3.01), RGC(12), RGC(19)},
    {RGC(-12), RGC(-5), 0, RGC(3), RGC(7), RGC(3.01), RGC(10), RGC(19)},
    {RGC(-12), RGC(-5), 0, RGC(3), RGC(14), RGC(3.01), RGC(10), RGC(19)},
    {RGC(-12), RGC(-5), 0, RGC(3), RGC(7), RGC(3.01), RGC(10), RGC(17)},
    {RGC(-12), RGC(-5), 0, RGC(2), RGC(7), RGC(9), RGC(16), RGC(19)},
    {RGC(-12), RGC(-5), 0, RGC(4), RGC(7), RGC(11), RGC(14), RGC(19)},
    {RGC(-12), RGC(-5), 0, RGC(4), RGC(7), RGC(11), RGC(10.99), RGC(19)},
    {RGC(-12), RGC(-5), 0, RGC(4), RGC(7), RGC(11.99), RGC(12), RGC(19)},
};
static const int16_t RG_CH2[11][4] = {
    {RGC(-12), 0, RGC(0.01), RGC(12)}, {RGC(-12), RGC(6.99), RGC(7), RGC(12)}, {RGC(-12), RGC(5), RGC(7), RGC(12)},
    {RGC(-12), RGC(3), RGC(11.99), RGC(12)}, {RGC(-12), RGC(3), RGC(10), RGC(12)}, {RGC(-12), RGC(3), RGC(10), RGC(14)},
    {RGC(-12), RGC(3), RGC(10), RGC(17)}, {RGC(-12), RGC(2), RGC(9), RGC(16)}, {RGC(-12), RGC(4), RGC(11), RGC(14)},
    {RGC(-12), RGC(4), RGC(7), RGC(11)}, {RGC(-12), RGC(4), RGC(7), RGC(12)},
};
static const int16_t RG_CH4[11][2] = {
    {0, RGC(-12)}, {0, RGC(2)}, {0, RGC(3)}, {0, RGC(4)}, {0, RGC(5)}, {0, RGC(7)},
    {0, RGC(9)}, {0, RGC(10)}, {0, RGC(11)}, {0, RGC(12)}, {RGC(-12), RGC(12)},
};
#else
static const int16_t RG_CH1[11][8] = {
    {RGC(-12), 0, RGC(0.01), RGC(0.02), RGC(0.03), RGC(11.98), RGC(11.99), RGC(12)},
    {RGC(-12), 0, RGC(3), RGC(3.01), RGC(7), RGC(9.99), RGC(10), RGC(19)},
    {RGC(-12), 0, RGC(3), RGC(3.01), RGC(7), RGC(11.99), RGC(12), RGC(19)},
    {RGC(-12), 0, RGC(3), RGC(3.01), RGC(7), RGC(13.99), RGC(14), RGC(19)},
    {RGC(-12), 0, RGC(3), RGC(3.01), RGC(7), RGC(16.99), RGC(17), RGC(19)},
    {RGC(-12), 0, RGC(6.98), RGC(6.99), RGC(7), RGC(12), RGC(18.99), RGC(19)},
    {RGC(-12), 0, RGC(3.99), RGC(4), RGC(7), RGC(16.99), RGC(17), RGC(19)},
    {RGC(-12), 0, RGC(3.99), RGC(4), RGC(7), RGC(13.99), RGC(14), RGC(19)},
    {RGC(-12), 0, RGC(3.99), RGC(4), RGC(7), RGC(11.99), RGC(12), RGC(19)},
    {RGC(-12), 0, RGC(3.99), RGC(4), RGC(7), RGC(10.99), RGC(11), RGC(19)},
    {RGC(-12), 0, RGC(4.99), RGC(5), RGC(7), RGC(11.99), RGC(12), RGC(17)},
};
static const int16_t RG_CH2[11][4] = {
    {RGC(-12), 0, RGC(0.01), RGC(12)}, {RGC(-12), RGC(3), RGC(7), RGC(10)}, {RGC(-12), RGC(3), RGC(7), RGC(12)},
    {RGC(-12), RGC(3), RGC(7), RGC(14)}, {RGC(-12), RGC(3), RGC(7), RGC(17)}, {RGC(-12), RGC(7), RGC(12), RGC(19)},
    {RGC(-12), RGC(4), RGC(7), RGC(17)}, {RGC(-12), RGC(4), RGC(7), RGC(14)}, {RGC(-12), RGC(4), RGC(7), RGC(12)},
    {RGC(-12), RGC(4), RGC(7), RGC(11)}, {RGC(-12), RGC(5), RGC(7), RGC(12)},
};
static const int16_t RG_CH4[11][2] = {
    {0, RGC(-12)}, {0, RGC(0.01)}, {0, RGC(2)}, {0, RGC(3)}, {0, RGC(4)}, {0, RGC(5)},
    {0, RGC(7)}, {0, RGC(10)}, {0, RGC(11)}, {0, RGC(12)}, {RGC(-12), RGC(12)},
};
#endif

static inline int32_t rg_c16(int16_t h) { return (int32_t)(((int64_t)h * 65536) / 100); }   /* hundredths -> Q16 */

/* the tuning of the strings of one voice, semitones Q16 relative to its note. quant: the chord (0..10),
 * else the voice's STRUCTURE (Q16) picks among the register of intervals; tonic: the root relative to the note */
static void rg_sym_notes(int poly, int quant, int32_t structure, int32_t chord, int32_t tonic, int32_t *d, int nstr)
{
    int i;
    if (quant) {
        for (i = 0; i < nstr; i++)
            d[i] = rg_c16(poly == 1 ? RG_CH1[chord][i] : poly == 2 ? RG_CH2[chord][i] : RG_CH4[chord][i]);
    } else {
        int32_t notes[9], par = structure;
        static const int16_t det[4] = {852, 721, 459, 1114};   /* 0.013 .. 0.017 semitones */
        int ndet = (nstr - 1) >> 1, first = nstr - ndet;
        notes[0] = tonic;
        notes[1] = -12 * 65536;
        notes[2] = -(7 * 65536 + 1280);                  /* 7.01955 */
        notes[3] = 0;
        notes[4] = 7 * 65536 + 1280;
        notes[5] = 12 * 65536;
        notes[6] = 19 * 65536 + 1280;
        notes[7] = notes[8] = 24 * 65536;
        for (i = 0; i < first; i++) {
            int32_t nt = 3 << 16, ni, fr, a, b;
            if (i) {
                nt = par * 7;
                par += px_m(65536 - par, 13107, 16);
            }
            ni = nt >> 16;
            fr = rg_squash(nt & 0xFFFF);
            a = notes[ni];
            b = notes[ni + 1];
            d[i] = a + px_m(b - a, fr, 16);
            if (i + first < nstr)
                d[i + first] = d[i] + det[i & 3];
        }
    }
}

/* ------------------------------------------------------------- Reverb --- */
/* The reverb [Reverb]: Griesinger's topology (four diffusing all-passes, then two loops of two all-passes and a
 * delay, cross-fed), the lengths x 0.37 so that the whole fits 8192 samples of 16 bit (Rings: 32768). The loop gain
 * is scaled with them (1 - (1 - kr) 0.37) so that the decay time stays. */
#define RG_RVN 8192
#define RG_RVM (RG_RVN - 1)
enum { RV_AP1, RV_AP2, RV_AP3, RV_AP4, RV_DAP1A, RV_DAP1B, RV_DEL1, RV_DAP2A, RV_DAP2B, RV_DEL2, RV_N };
static const uint16_t RV_LEN[RV_N] = {55, 79, 118, 195, 807, 995, 1668, 934, 813, 2338};
#define RV_BASE(k) rg_rvbase(k)
static inline uint32_t rg_rvbase(uint32_t k)
{
    uint32_t b = 0, i;
    for (i = 0; i < k; i++)
        b += RV_LEN[i] + 1u;
    return b;
}

typedef struct {
    int16_t *buf;                                        /* RG_RVN values (Q14: +-2, as fine as Rings' 16 bits at +-1 within 6 dB) */
    uint32_t wp;
    uint32_t lph1, lph2;                                 /* LFO phases (turns Q32) */
    int32_t lv1, lv2;                                    /* LFO values, Q30 */
    int32_t lp1, lp2;                                    /* Q20 */
    int32_t amount, gain, time, diff, lp;                /* Q16, as Reverb::set_* */
} rg_rev_t;

static void rg_rev_init(rg_rev_t *R, int16_t *buf)
{
    uint32_t i;
    for (i = 0; i < RG_RVN; i++)
        buf[i] = 0;
    R->buf = buf;
    R->wp = R->lph1 = R->lph2 = 0;
    R->lv1 = R->lv2 = 1 << 30;                          /* (1.0 at the start, as the original) */
    R->lp1 = R->lp2 = 0;
    R->amount = 0;
    R->gain = 13107;
    R->time = 45875;
    R->diff = 40960;
    R->lp = 45875;
}

static inline int32_t rg_rv_rd(const rg_rev_t *R, uint32_t k, uint32_t off)    /* buffer[wp + base + off], Q20 */
{
    return (int32_t)R->buf[(R->wp + rg_rvbase(k) + off) & RG_RVM] << 6;
}

static inline void rg_rv_wr(rg_rev_t *R, uint32_t k, int32_t v)
{
    R->buf[(R->wp + rg_rvbase(k)) & RG_RVM] = (int16_t)px_clamp((v + 32) >> 6, -32768, 32767);   /* rounded: a floor biases every write the same way, a DC that the loop builds up */
}

/* Interpolate(d, offset (Q16 samples), ...) */
static inline int32_t rg_rv_int(const rg_rev_t *R, uint32_t k, int32_t off16)
{
    uint32_t o = (uint32_t)off16 >> 16, base = rg_rvbase(k);
    int32_t a = R->buf[(R->wp + o + base) & RG_RVM], b = R->buf[(R->wp + o + base + 1u) & RG_RVM];
    return (a + (((b - a) * (off16 & 0xFFFF)) >> 16)) << 6;
}

/* log2 of x (Q16, 0 < x <= 1) -> Q16 (<= 0): the mantissa's log by x (1.3465 - 0.3465 x), under 0.5 % off */
static int32_t rg_log2q(int32_t x)
{
    int32_t e = 0, m;
    if (x < 1)
        x = 1;
    while (x < 32768) {
        x <<= 1;
        e--;
    }
    while (x >= 65536) {
        x >>= 1;
        e++;
    }
    m = x - 32768;                                       /* the mantissa 1 + m / 32768 -> (m / 32768) in Q16 = 2 m */
    m <<= 1;
    return (e - 1) * 65536 + px_m(m, 88246 - px_m(22708, m, 16), 16);
}

/* Reverb::Process on l[], r[] (Q20) in place */
static void rg_rev_run(rg_rev_t *R, int32_t *l, int32_t *r, uint32_t n)
{
    uint32_t i;
    int32_t kap = R->diff, klp = R->lp, amount = R->amount, gain = R->gain;
    int32_t krt = (int32_t)px_exp2(px_m(36000, rg_log2q(R->time), 16));    /* time ^ 0.37: the loop's gain over its shortened length */
    for (i = 0; i < n; i++) {
        int32_t acc, apout, prev = 0, wet, x;
        R->wp = (R->wp - 1u) & RG_RVM;
        if ((R->wp & 31u) == 0) {                        /* the LFOs, every 32 samples (0.5 and 0.3 Hz) */
            R->lph1 += 1431656u;                              /* 0.5 / 48000 * 32 turns (the original: 48000 whatever the rate) */
            R->lph2 += 858993u;
            R->lv1 = (px_sin(R->lph1 + 0x40000000u) >> 1) + (1 << 29);      /* 0..1 like the original's cosine oscillator */
            R->lv2 = (px_sin(R->lph2 + 0x40000000u) >> 1) + (1 << 29);
        }
        acc = px_m(l[i] + r[i], gain, 16);
        /* four diffusers */
#define RV_AP(k, c)                                                                      \
    prev = rg_rv_rd(R, k, RV_LEN[k] - 1u);                                               \
    acc += px_m(prev, kap, 16);                                                          \
    rg_rv_wr(R, k, acc);                                                                 \
    acc = px_m(acc, -(c), 16) + prev;
        RV_AP(RV_AP1, kap) RV_AP(RV_AP2, kap) RV_AP(RV_AP3, kap) RV_AP(RV_AP4, kap)
        apout = acc;
        /* loop 1: del2 -> lp -> dap1a, dap1b -> del1; the left output */
        acc = apout + px_m(rg_rv_int(R, RV_DEL2, 151820000 + (int32_t)(((int64_t)R->lv2 * 1212416) >> 30)), krt, 16);
        R->lp1 += px_m(acc - R->lp1, klp, 16);
        acc = R->lp1;
        prev = rg_rv_rd(R, RV_DAP1A, RV_LEN[RV_DAP1A] - 1u);
        acc += px_m(prev, -kap, 16);
        rg_rv_wr(R, RV_DAP1A, acc);
        acc = px_m(acc, kap, 16) + prev;
        prev = rg_rv_rd(R, RV_DAP1B, RV_LEN[RV_DAP1B] - 1u);
        acc += px_m(prev, kap, 16);
        rg_rv_wr(R, RV_DAP1B, acc);
        acc = px_m(acc, -kap, 16) + prev;
        rg_rv_wr(R, RV_DEL1, acc);
        wet = acc * 2;
        x = l[i];
        l[i] = x + px_m(wet - x, amount, 16);
        /* loop 2 */
        acc = apout + px_m(rg_rv_int(R, RV_DEL1, 108148000 + (int32_t)(((int64_t)R->lv1 * 969933) >> 30)), krt, 16);
        R->lp2 += px_m(acc - R->lp2, klp, 16);
        acc = R->lp2;
        prev = rg_rv_rd(R, RV_DAP2A, RV_LEN[RV_DAP2A] - 1u);
        acc += px_m(prev, kap, 16);
        rg_rv_wr(R, RV_DAP2A, acc);
        acc = px_m(acc, -kap, 16) + prev;
        prev = rg_rv_rd(R, RV_DAP2B, RV_LEN[RV_DAP2B] - 1u);
        acc += px_m(prev, -kap, 16);
        rg_rv_wr(R, RV_DAP2B, acc);
        acc = px_m(acc, kap, 16) + prev;
        rg_rv_wr(R, RV_DEL2, acc);
        wet = acc * 2;
        x = r[i];
        r[i] = x + px_m(wet - x, amount, 16);
        R->lp1 = px_clamp(R->lp1, -(6 << 20), 6 << 20);
        R->lp2 = px_clamp(R->lp2, -(6 << 20), 6 << 20);
    }
}

/* ------------------------------------------------------------ Limiter --- */
typedef struct { int32_t peak; } rg_lim_t;              /* Q20 */

/* Limiter::Process on one signal, x[] Q20 in place; gain: the model's pre-gain, Q16 */
static void rg_lim_run(rg_lim_t *L, int32_t *x, uint32_t n, int32_t gain)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        int32_t v = px_m(x[i], gain, 16), a = v < 0 ? -v : v, e = a - L->peak, g = 1 << 14, v2;
        L->peak += px_m(e, e > 0 ? 53687091 : 21475, 30);            /* SLOPE .05 up, .00002 down */
        if (L->peak > RG_ONE)
            g = (int32_t)((1u << 30) / (uint32_t)(L->peak >> 4));    /* 1 / peak, Q14 */
        v = px_m(px_m(v, g, 14), 52429, 16);                         /* * gain * 0.8 */
        v = px_clamp(v, -(5 << 20), 5 << 20);
        v2 = px_m(v, v, 20);
        /* SoftLimit: x (27 + x^2) / (27 + 9 x^2) */
        x[i] = px_m(v, (int32_t)(((uint32_t)((27 << 20) + v2) >> 10 << 14) / (uint32_t)(((27 << 20) + 9 * v2) >> 10)), 14);
    }
}

/* ----------------------------------------------------------- FM voice --- */
typedef struct {
    uint32_t cph, mph;                                   /* phases, turns Q32 */
    uint32_t cinc, minc;                                 /* the increments the block started at */
    int32_t pbr, pfb;                                    /* brightness, feedback (Q16, signed) */
    int32_t aenv, benv, gain, fma, prev;                 /* Q30, Q30 (the slow decays need it), Q16, Q16, Q20 */
} rg_fm_t;

typedef struct {                                         /* per block */
    uint32_t cinc, minc;
    int32_t br, fb, envamt, adecay, bdecay;              /* Q16, Q16 (signed), Q16, Q30, Q30 */
    int32_t nomod;                                       /* the carrier alone (polyphonic: out - aux of Rings) */
} rg_fm_blk_t;

static inline int32_t rg_fm_decay(int32_t damping, int32_t oct)   /* 1 - 0.001^(1 / (0.1 sr 2^(oct d))), Q30 */
{
    uint64_t e = px_exp2((int32_t)(((int64_t)damping * oct) >> 0));    /* oct: whole octaves per unit of d */
    uint64_t x = 701283000000000ull / (4410u * e);                     /* 9.9658 / rt60, Q30 */
    return (int32_t)((x * 45426u) >> 16);                              /* * ln 2 */
}

static void rg_fm_block(rg_fm_blk_t *B, rg_fm_t *V, const rg_patch_t *P, int strum)
{
    int32_t i = P->structure >> 9, fr = (P->structure & 511) << 7, semi;
    if (i >= 128)
        semi = RG_FMQ[128];
    else
        semi = RG_FMQ[i] + px_m(RG_FMQ[i + 1] - RG_FMQ[i], fr, 16);
    B->cinc = P->f > RG_FMAX ? RG_FMAX : P->f;
    B->minc = rg_fshift(P->f, semi);
    B->nomod = 0;
    B->br = P->brightness;
    B->fb = (P->position - 32768) * 2;
    B->envamt = P->damping < 58982 ? 65536 : (65536 - P->damping) * 10;
    B->adecay = rg_fm_decay(P->damping, 8);
    B->bdecay = rg_fm_decay(P->damping, 7);
    if (strum) {
        V->aenv = V->benv = 1 << 30;
    }
    if (!V->cinc) {                                      /* FMVoice::Init: 220 Hz, brightness .5, feedback 0: the block ramps from there */
        V->cinc = V->minc = 21426140u;
        V->pbr = 32768;
        V->pfb = 0;
    }
}

static void rg_fm_run(const rg_fm_blk_t *B, rg_fm_t *V, int32_t *out, uint32_t n)
{
    uint32_t i, cph = V->cph, mph = V->mph, ci = V->cinc, mi = V->minc;
    int32_t dci = ((int32_t)B->cinc - (int32_t)ci) / (int32_t)n, dmi = ((int32_t)B->minc - (int32_t)mi) / (int32_t)n;
    int32_t br = V->pbr, dbr = (B->br - br) / (int32_t)n, fb = V->pfb, dfb = (B->fb - fb) / (int32_t)n;
    int32_t aenv = V->aenv, benv = V->benv, gain = V->gain, fma = V->fma, prev = V->prev;
    for (i = 0; i < n; i++) {
        int32_t bv, fmin, fmax, fenv, fm, delta, g, pf, mfb, mod, car;
        int64_t m;
        aenv -= (int32_t)(((int64_t)aenv * B->adecay) >> 30);
        benv -= (int32_t)(((int64_t)benv * B->bdecay) >> 30);
        br += dbr;
        bv = px_m(br, br, 16);
        fmin = bv < 32768 ? 0 : 2 * bv - 65536;
        fmax = bv < 32768 ? 2 * bv : 65536;
        fenv = 32768 + px_m(B->envamt, (benv >> 14) - 32768, 16);
        fm = (fmin + px_m(fmax, fenv, 16)) * 2;
        delta = 328 + px_m(fmax, 983, 16);
        fma += px_clamp(fm - fma, -delta, delta);
        fb += dfb;
        pf = fb < 0 ? px_m(px_m(fb, fb, 16), 32768, 16) : 0;                 /* 0.5 fb^2 */
        mi += (uint32_t)dmi;
        ci += (uint32_t)dci;
        mph += (uint32_t)(((uint64_t)mi * (uint32_t)(65536 + (int32_t)(((int64_t)prev * pf) >> 20))) >> 16);
        cph += ci;
        mfb = fb > 0 ? px_m(px_m(fb, fb, 16), 16384, 16) : 0;                /* 0.25 fb^2 */
        mod = px_sin(mph + (uint32_t)(((int64_t)mfb * prev) >> 4));
        m = ((int64_t)fma * mod) >> 14;                                      /* turns Q32 */
        car = px_sin(cph + (uint32_t)m);
        prev += px_m((car >> 10) - prev, 6554, 16);                          /* ONE_POLE 0.1 */
        g = 65536 + px_m(B->envamt, (aenv >> 14) - 65536, 16);
        gain += px_m(g - gain, 328 + px_m(2949, fma, 16), 16);
        out[i] = px_m((car >> 10) + (B->nomod ? 0 : (mod >> 10) >> 1), gain, 16);
    }
    V->cph = cph;
    V->mph = mph;
    V->cinc = B->cinc;
    V->minc = B->minc;
    V->pbr = B->br;
    V->pfb = B->fb;
    V->aenv = aenv;
    V->benv = benv;
    V->gain = gain;
    V->fma = fma;
    V->prev = prev;
}

/* ---------------------------------------------------------------- voices --- */
/* The state of the Rings models, one set in an arena owned by one part at a time (eng_phys.c). */
typedef struct {
    rg_str_t s[RG_NSTR];
    rg_pl_t pl[RG_MAXV];
    int32_t dc[RG_MAXV][3];                              /* the DC blocker on the exciter (1 - 10 / sr): x, y, error */
    rg_rev_t rev;                                        /* STRING AND REVERB */
    int16_t sl[RG_MAXV][RG_SLINE];                       /* dispersion lines */
    int16_t ln[RG_NSTR][RG_LINE];                        /* the strings' lines (4 .. 7 and rvx hold the reverb when 1 string/voice) */
    int16_t rvx[RG_RVN - 4 * RG_LINE];                   /* the rest of the reverb's RG_RVN values (it follows ln) */
} rg_strset_t;
#define RG_RVBUF(A) ((int16_t *)(A)->ln + 4 * RG_LINE)   /* STRING AND REVERB's reverb: 4 strings, then this */

typedef struct { rg_modal_t m[RG_MAXV]; } rg_modset_t;
typedef struct { rg_fm_t v[RG_MAXV]; } rg_fmset_t;

#define RG_LFOI(hz) ((uint32_t)((hz) * 32.0 / 44100.0 * 4294967296.0))   /* LFO phase per block (CTL 32) */
static const uint32_t RG_LFOS[RG_NSTR] = {RG_LFOI(0.5), RG_LFOI(0.4), RG_LFOI(0.35), RG_LFOI(0.23), RG_LFOI(0.211),
                                          RG_LFOI(0.2), RG_LFOI(0.171), 0};

/* one voice of SYMP / STRING / QUANT / STRREV: n <= 32 samples, out += (string - pickup) */
static __attribute__((noinline)) void rg_strv_run(rg_strset_t *A, const rg_patch_t *P, int model, int strum, int32_t *out,
                                                    uint32_t n)
{
    int v = P->voice, poly = P->poly, symp = model == RG_SYMP || model == RG_QUANT, nstr = symp ? 8 / poly : 1, k;
    int32_t rel[RG_NSTR], ob[32], ab[32], si[32], ex[32], disp = 0, st = P->structure;
    uint32_t i, fc, cutoff = (uint32_t)px_m(P->brightness, 131072 - P->brightness, 16);
    rg_pl_t *pl = &A->pl[v];
    if (n > 32u)
        n = 32u;
    fc = rg_fshift(P->f, ((int32_t)cutoff - 32768) * 96);            /* the exciter's cutoff */
    if (strum)
        rg_pl_trigger(pl, P->f, fc >= (RG_FMAX >> 3) ? RG_FMAX : fc << 3, P->position);
    rg_pl_run(pl, ex, n);
    {   /* the DC blocker (1 - 10 / sr) */
        int32_t *d = A->dc[v];
        for (i = 0; i < n; i++) {
            int64_t acc = (int64_t)1073498345 * d[1] + d[2];
            int32_t y = ex[i] - d[0] + (int32_t)(acc >> 30);
            d[2] = (int32_t)(acc & 0x3FFFFFFF);
            d[0] = ex[i];
            d[1] = y;
            ex[i] = y;
        }
    }
    for (i = 0; i < n; i++)
        ob[i] = ab[i] = 0;
    if (symp)
        rg_sym_notes(poly, model == RG_QUANT, st, P->chord, P->tonic - P->note, rel, nstr);
    else
        rel[0] = 0;
    if (!symp)                                           /* the dispersion (STRUCTURE .24 / .26 split) */
        disp = st < 15729 ? (int32_t)(((int64_t)(st - 15729) * 273019) >> 16)
               : st > 17039 ? (int32_t)(((int64_t)(st - 17039) * 88562) >> 16) : 0;
    for (k = 0; k < nstr; k++) {
        int idx = v + k * poly;
        rg_str_t *S = &A->s[idx];
        rg_str_blk_t B;
        int32_t b = P->brightness, d = P->damping, pos = P->position, glide = 65536;
        uint32_t ft = rg_fshift(P->f, rel[k]);
        const int32_t *in = ex;
        if (model == RG_STRREV)
            d = px_m(d, 131072 - d, 16);
        if (k > 0) {                                     /* sympathetic: brighter, longer, the pickup moves */
            int32_t lfo, amt, ad = pos - 32768;
            b = px_m(b, 131072 - b, 16);
            b = px_m(b, 131072 - b, 16);
            d = 45875 + px_m(P->damping, 17695, 16);                 /* 0.7 + damping 0.27 */
            amt = px_m(32768 - (ad < 0 ? -ad : ad), 58982, 16);      /* (0.5 - |0.5 - pos|) 0.9 */
            lfo = px_sin(S->lfo + 0x40000000u) >> 14;                 /* cos, Q16 */
            pos += px_m(lfo, amt, 16);
            glide = (int32_t)px_exp2(3 * (b - 65536));               /* 2^(3 (b - 1)) */
            in = si;
        }
        S->lfo += RG_LFOS[idx];
        if (!S->f)
            S->f = ft;
        S->f = (uint32_t)((int64_t)S->f + (((int64_t)ft - (int64_t)S->f) * glide >> 16));
        d += (int32_t)(((int64_t)k * (62259 - d)) / nstr);           /* damping + index (0.95 - damping) */
        pos = px_clamp(pos, 0, 65536);
        rg_str_block(&B, S, S->f, disp, b, px_clamp(d, 0, 65536), pos, !symp);
        rg_str_run(&B, S, A->ln[idx], A->sl[v], in, ob, ab, n);
        if (k == 0 && nstr > 1) {
            int32_t g = 13107 / nstr;
            for (i = 0; i < n; i++)
                si[i] = px_m(ob[i] - ab[i], g, 16);
        }
    }
    if (model == RG_STRREV && poly == 1) {               /* (Part feeds the reverb with out + aux of the single voice) */
        for (i = 0; i < n; i++)
            out[i] += ob[i] + ab[i];
    } else {
        for (i = 0; i < n; i++)
            out[i] += ob[i] - ab[i];
    }
}

/* one FM voice */
static void rg_fmv_run(rg_fmset_t *A, const rg_patch_t *P, int strum, int32_t *out, uint32_t n)
{
    rg_fm_blk_t B;
    uint32_t i;
    int32_t y[32];
    if (n > 32u)
        n = 32u;
    rg_fm_block(&B, &A->v[P->voice], P, strum);
    B.nomod = P->poly > 1;                               /* (Part: odd / even voices go out as out - aux: the carrier) */
    rg_fm_run(&B, &A->v[P->voice], y, n);
    for (i = 0; i < n; i++)
        out[i] += y[i];
}

/* the limiter's pre-gain per model, Q16 (Part::model_gains_) */
static const int32_t RG_GAIN[RG_NMODEL] = {91750, 65536, 91750, 45875, 65536, 91750, 65536};

/* the part-level processing of the resonator models over n samples of the mix x (Q20): the reverb of STRING AND REVERB
 * (Part::Process: amount .1 + .5 d, time .35 + .63 d, lp .3 + .6 b, input gain .2), then the limiter */
static void rg_post(rg_strset_t *A, rg_lim_t *L, int model, const rg_patch_t *P, int32_t *x, uint32_t n)
{
    if (model == RG_STRREV) {
        rg_rev_t *R = &A->rev;
        int32_t l[32], r[32];
        uint32_t i;
        if (n > 32u)
            n = 32u;
        R->amount = 6554 + px_m(P->damping, 32768, 16);
        R->diff = 40960;
        R->time = 22938 + px_m(P->damping, 41288, 16);
        R->gain = 13107;
        R->lp = 19661 + px_m(P->brightness, 39322, 16);
        for (i = 0; i < n; i++)
            l[i] = r[i] = x[i];
        rg_rev_run(R, l, r, n);
        for (i = 0; i < n; i++)
            x[i] = (l[i] + r[i]) >> 1;
    }
    rg_lim_run(L, x, n, RG_GAIN[model]);
}

/* ------------------------------------------------- Disastrous Peace --- */
/* The string synth [StringSynthPart]: up to 12 voices of three harmonics each (a dark square with a saw, then two
 * bright squares with a saw, polyBLEP), played as chords by 1, 2 or 4 groups with a registration (BRIGHTNESS), an
 * AD envelope per group (DAMPING) and one of six effects (formant filter, chorus, reverb, a second formant and
 * reverb, ensemble) whose amount is POSITION. The effects work on the pair the voices are split over (the chord's
 * notes alternate between the two); the output is the mono sum of the limited pair.
 *
 * What differs: the note arrives with the strum (Rings' median / lag NoteFilter and its 3-block acquisition are
 * left out: the group takes the pitch at once); the reverb is the shared, scaled one (8192 samples). */
#define RG_DPV 12                                        /* voices */
#define RG_DPH 3                                         /* harmonics per voice */
#define RG_DPL 2048                                      /* chorus / ensemble line (Rings: 2047 + 1) */
#define RG_DPM (RG_DPL - 1)
#define RG_PH(x) ((uint32_t)((x) * 4294967296.0))        /* a per-sample phase increment, turns Q32 */
enum { DPF_FORMANT, DPF_CHORUS, DPF_REVERB, DPF_FORMANT2, DPF_ENSEMBLE, DPF_REVERB2 };

typedef struct {                                         /* StringSynthOscillator */
    uint32_t phase, inc;                                 /* cycles Q32 */
    int32_t gain, gsaw;                                  /* Q16 */
    int32_t cur, nxt, scur, snxt, filt;                  /* the polyBLEP's this / next samples, the integrator (Q20) */
    uint8_t high;
} rg_dpo_t;

typedef struct {                                         /* StringSynthEnvelope (AD) */
    int32_t start, value;                                /* Q16 */
    uint32_t phase;                                      /* Q30 */
    int32_t seg;                                         /* 0 attack, 1 decay, 2 done */
    int32_t tonic, chord;                                /* the group: its note (MIDI Q16), its chord */
} rg_dpg_t;

typedef struct {
    rg_dpo_t osc[RG_DPV][RG_DPH];
    rg_dpg_t grp[4];
    int32_t sv[3][2];                                    /* the formant filters (Q24) */
    int32_t active, poly, fx;
    uint32_t lph[2];                                     /* the effects' LFOs, turns Q32 */
    uint32_t wp;                                         /* the chorus / ensemble write pointer */
    int32_t peak;                                        /* the limiter (Q20) */
    rg_rev_t rev;
    union {
        int16_t chorus[RG_DPL];
        int16_t ens[2][RG_DPL];
        int16_t rev[RG_RVN];
    } fx_mem;
} rg_dpset_t;

static void rg_dp_clear_fx(rg_dpset_t *A)
{
    uint32_t i;
    for (i = 0; i < RG_RVN; i++)
        A->fx_mem.rev[i] = 0;
    rg_rev_init(&A->rev, A->fx_mem.rev);
    A->wp = 0;
}

static void rg_dp_init(rg_dpset_t *A)
{
    memset(A, 0, sizeof *A);
    A->poly = 1;
    A->fx = DPF_ENSEMBLE;
    A->peak = RG_ONE / 2;
    A->grp[0].seg = A->grp[1].seg = A->grp[2].seg = A->grp[3].seg = 2;
    rg_dp_clear_fx(A);
}

/* ---- the oscillator: n samples added to out (Q20) ---- */
static void rg_dpo_run(rg_dpo_t *o, int dark, uint32_t tinc, int32_t tg, int32_t ts, int32_t *out, uint32_t n)
{
    uint32_t phase = o->phase, inc = o->inc, i;
    int32_t dinc, dg, ds, gain = o->gain, gsaw = o->gsaw, nxt = o->nxt, snxt = o->snxt, filt = o->filt;
    int high = o->high;
    if (tinc >= 730144440u) {                            /* >= 0.17: fade out the highs, silent from 0.25 */
        int32_t d = (int32_t)(((uint64_t)(tinc - 730144440u) * 25u) >> 17);
        tg = px_m(tg, 65536 - d, 16);
        if (tinc >= 1073741824u)
            return;
    }
    if (!tg && !ts && !gain && !gsaw) {                  /* silent and staying so: nothing to render (the phase freezes) */
        o->inc = tinc;
        return;
    }
    dinc = dark ? (int32_t)(tinc - inc) / (int32_t)n : 0;
    dg = (tg - gain) / (int32_t)n;
    ds = (ts - gsaw) / (int32_t)n;
    for (i = 0; i < n; i++) {
        int32_t cur = nxt, scur = snxt, x, sample, saw, coef;
        uint32_t p0 = phase;
        int wrapped;
        nxt = snxt = 0;
        inc = dark ? inc + (uint32_t)dinc : tinc;
        phase += inc;
        wrapped = phase < p0;
        if (!high && !wrapped && phase >= 0x80000000u) {
            uint32_t t = (uint32_t)(((uint64_t)(phase - 0x80000000u) << 16) / inc), u = 65536u - t;
            cur += (int32_t)((t * t) >> 13);
            nxt -= (int32_t)((u * u) >> 13);
            high = 1;
        }
        if (wrapped) {
            uint32_t t = (uint32_t)(((uint64_t)phase << 16) / inc), u = 65536u - t;
            int32_t a = (int32_t)((t * t) >> 13), b = -(int32_t)((u * u) >> 13);
            cur -= a;
            nxt -= b;
            scur -= a;
            snxt -= b;
            high = 0;
        }
        nxt += phase < 0x80000000u ? 0 : RG_ONE;
        snxt += (int32_t)(phase >> 12);
        coef = (int32_t)(inc >> 15);                     /* 2 inc, Q16 */
        if (dark) {
            x = 4 * (cur - (RG_ONE >> 1));
            filt += px_m(coef, x - filt, 16);
            sample = filt;
        } else {
            x = 2 * cur - RG_ONE;
            filt += px_m(coef, x - filt, 16);
            sample = (x - filt) >> 1;
        }
        saw = 2 * scur - RG_ONE;
        gain += dg;
        gsaw += ds;
        out[i] += px_m(sample, gain, 16) + px_m(saw, gsaw, 16);
    }
    o->phase = phase;
    o->inc = inc;
    o->gain = tg;
    o->gsaw = ts;
    o->nxt = nxt;
    o->snxt = snxt;
    o->filt = filt;
    o->high = (uint8_t)high;
}

/* ---- the envelope: one step per block; arate, drate: Q30 per block. flags: 1 rising edge ---- */
static int32_t rg_dpe_run(rg_dpg_t *e, int flags, uint32_t arate, uint32_t drate)
{
    uint32_t inc = 0, ph;
    int32_t t, target;
    if (flags & 1) {
        e->start = e->seg == 2 ? 0 : e->value;
        e->seg = 0;
        e->phase = 0;
    } else if (e->phase >= (1u << 30)) {
        e->start = e->seg == 0 ? 65536 : 0;
        e->seg++;
        e->phase = 0;
    }
    if (e->seg < 2)
        inc = e->seg == 0 ? arate : drate;
    ph = e->phase > (1u << 30) ? (1u << 30) : e->phase;
    t = (int32_t)(ph >> 14);
    if (e->seg == 1) {                                   /* quartic: 1 - (1 - t)^4 */
        t = 65536 - t;
        t = px_m(t, t, 16);
        t = px_m(t, t, 16);
        t = 65536 - t;
    }
    target = e->seg == 0 ? 65536 : 0;
    e->phase += inc;
    e->value = e->start + px_m(target - e->start, t, 16);
    return e->value;
}

/* ---- registrations (Q16): the harmonics' square / saw amplitudes, interpolated by BRIGHTNESS ---- */
#define RQ(x) ((int32_t)((x) * 65536.0 + 0.5))
static const int32_t RG_DPREG[11][6] = {
    {RQ(1), 0, 0, 0, 0, 0},       {RQ(1), RQ(1), 0, 0, 0, 0},   {RQ(1), 0, RQ(1), 0, 0, 0},
    {RQ(1), RQ(.1), 0, 0, RQ(1), 0}, {RQ(1), RQ(.5), RQ(1), 0, RQ(1), 0}, {RQ(1), RQ(1), RQ(1), RQ(1), RQ(1), RQ(1)},
    {0, RQ(1), RQ(1), RQ(1), RQ(1), 0}, {0, RQ(.5), RQ(1), 0, RQ(1), 0}, {0, 0, RQ(1), 0, RQ(1), 0},
    {0, 0, 0, 0, RQ(1), 0},       {0, 0, 0, 0, 0, RQ(1)},
};

static void rg_dp_registration(int32_t gain, int32_t reg, int32_t *amp)
{
    int32_t r = px_m(reg, 655334, 16), i = r >> 16, f = r & 0xFFFF, k;
    int64_t total = 0;
    if (i > 9)
        i = 9;
    for (k = 0; k < 6; k++) {
        amp[k] = RG_DPREG[i][k] + px_m(RG_DPREG[i + 1][k] - RG_DPREG[i][k], f, 16);
        total += amp[k];
    }
    for (k = 0; k < 6; k++)
        amp[k] = px_m(gain, (int32_t)(((int64_t)amp[k] << 16) / total), 16);
}

/* ---- chords (hundredths of a semitone), the string synth's own tables ---- */
#ifndef RG_ORIGINAL_CHORDS
static const int16_t RG_DPC1[11][8] = {
    {RGC(-12), RGC(-0.01), 0, RGC(0.01), RGC(0.02), RGC(11.99), RGC(12), RGC(24)},
    {RGC(-12), RGC(-5.01), RGC(-5), 0, RGC(7), RGC(12), RGC(19), RGC(24)},
    {RGC(-12), RGC(-5), 0, RGC(5), RGC(7), RGC(12), RGC(17), RGC(24)},
    {RGC(-12), RGC(-5), 0, RGC(0.01), RGC(3), RGC(12), RGC(19), RGC(24)},
    {RGC(-12), RGC(-5.01), RGC(-5), 0, RGC(3), RGC(10), RGC(19), RGC(24)},
    {RGC(-12), RGC(-5), 0, RGC(3), RGC(10), RGC(14), RGC(19), RGC(24)},
    {RGC(-12), RGC(-5.01), RGC(-5), 0, RGC(3), RGC(10), RGC(17), RGC(24)},
    {RGC(-12), RGC(-5), 0, RGC(2), RGC(9), RGC(16), RGC(19), RGC(24)},
    {RGC(-12), RGC(-5), 0, RGC(4), RGC(11), RGC(14), RGC(19), RGC(24)},
    {RGC(-12), RGC(-5), 0, RGC(4), RGC(7), RGC(11), RGC(19), RGC(24)},
    {RGC(-12), RGC(-5), 0, RGC(4), RGC(7), RGC(12), RGC(19), RGC(24)},
};
static const int16_t RG_DPC2[11][6] = {
    {RGC(-12), RGC(-0.01), 0, RGC(0.01), RGC(12), RGC(12.01)}, {RGC(-12), RGC(-5.01), RGC(-5), 0, RGC(7), RGC(12)},
    {RGC(-12), RGC(-5), 0, RGC(5), RGC(7), RGC(12)},           {RGC(-12), RGC(-5), 0, RGC(0.01), RGC(3), RGC(12)},
    {RGC(-12), RGC(-5.01), RGC(-5), 0, RGC(3), RGC(10)},       {RGC(-12), RGC(-5), 0, RGC(3), RGC(10), RGC(14)},
    {RGC(-12), RGC(-5), 0, RGC(3), RGC(10), RGC(17)},          {RGC(-12), RGC(-5), 0, RGC(2), RGC(9), RGC(16)},
    {RGC(-12), RGC(-5), 0, RGC(4), RGC(11), RGC(14)},          {RGC(-12), RGC(-5), 0, RGC(4), RGC(7), RGC(11)},
    {RGC(-12), RGC(-5), 0, RGC(4), RGC(7), RGC(12)},
};
static const int16_t RG_DPC4[11][3] = {
    {0, RGC(0.01), RGC(12)}, {0, RGC(7), RGC(12)}, {RGC(5), RGC(7), RGC(12)}, {0, RGC(3), RGC(12)},
    {0, RGC(3), RGC(10)},    {RGC(3), RGC(10), RGC(14)}, {RGC(3), RGC(10), RGC(17)}, {RGC(2), RGC(9), RGC(16)},
    {RGC(4), RGC(11), RGC(14)}, {RGC(4), RGC(7), RGC(11)}, {RGC(4), RGC(7), RGC(12)},
};
#else
static const int16_t RG_DPC1[11][8] = {
    {RGC(-24), RGC(-12), 0, RGC(0.01), RGC(0.02), RGC(11.99), RGC(12), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(3), RGC(7), RGC(10), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(3), RGC(7), RGC(12), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(3), RGC(7), RGC(14), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(3), RGC(7), RGC(17), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(6.99), RGC(7), RGC(18.99), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(4), RGC(7), RGC(17), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(4), RGC(7), RGC(14), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(4), RGC(7), RGC(12), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(4), RGC(7), RGC(11), RGC(19), RGC(24)},
    {RGC(-24), RGC(-12), 0, RGC(5), RGC(7), RGC(12), RGC(17), RGC(24)},
};
static const int16_t RG_DPC2[11][6] = {
    {RGC(-24), RGC(-12), 0, RGC(0.01), RGC(12), RGC(12.01)}, {RGC(-24), RGC(-12), 0, RGC(3), RGC(7), RGC(10)},
    {RGC(-24), RGC(-12), 0, RGC(3), RGC(7), RGC(12)},        {RGC(-24), RGC(-12), 0, RGC(3), RGC(7), RGC(14)},
    {RGC(-24), RGC(-12), 0, RGC(3), RGC(7), RGC(17)},        {RGC(-24), RGC(-12), 0, RGC(6.99), RGC(12), RGC(19)},
    {RGC(-24), RGC(-12), 0, RGC(4), RGC(7), RGC(17)},        {RGC(-24), RGC(-12), 0, RGC(4), RGC(7), RGC(14)},
    {RGC(-24), RGC(-12), 0, RGC(4), RGC(7), RGC(12)},        {RGC(-24), RGC(-12), 0, RGC(4), RGC(7), RGC(11)},
    {RGC(-24), RGC(-12), 0, RGC(5), RGC(7), RGC(12)},
};
static const int16_t RG_DPC4[11][3] = {
    {0, RGC(0.01), RGC(12)}, {0, RGC(3), RGC(10)}, {0, RGC(3), RGC(7)}, {0, RGC(3), RGC(14)},
    {0, RGC(3), RGC(17)},    {0, RGC(7), RGC(19)}, {0, RGC(4), RGC(17)}, {0, RGC(4), RGC(14)},
    {0, RGC(4), RGC(7)},     {0, RGC(4), RGC(11)}, {0, RGC(5), RGC(7)},
};
#endif

/* the formants of five vowels (Hz) */
static const uint16_t RG_FORMANTS[5][3] = {{700, 1100, 2400}, {500, 1300, 1700}, {400, 2000, 2500}, {600, 800, 2400}, {300, 900, 2200}};

/* a note (MIDI semitones Q16) as cycles per sample */
static inline uint32_t rg_note_f(int32_t note)
{
    return rg_fshift(RG_PH(440.0 / 44100.0), note - 69 * 65536);
}

/* the stereo limiter of the string synth on the pair (l, r) (Limiter, pre-gain 1: the peak sees l, r and l + r, the
 * original sees r negated), then the mono sum of the two limited signals / 2 into l */
static void rg_dp_limit(rg_dpset_t *A, int32_t *l, int32_t *r, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        int32_t a = l[i] < 0 ? -l[i] : l[i], b = r[i] < 0 ? -r[i] : r[i], s = l[i] + r[i], c = s < 0 ? -s : s, pk, e, g = 1 << 14;
        int32_t v[2], k;
        pk = a > b ? a : b;
        pk = pk > c ? pk : c;
        e = pk - A->peak;
        A->peak += px_m(e, e > 0 ? 53687091 : 21475, 30);
        if (A->peak > RG_ONE)
            g = (int32_t)((1u << 30) / (uint32_t)(A->peak >> 4));
        v[0] = l[i];
        v[1] = r[i];
        for (k = 0; k < 2; k++) {
            int32_t x = px_m(px_m(v[k], g, 14), 52429, 16), x2;
            x = px_clamp(x, -(5 << 20), 5 << 20);
            x2 = px_m(x, x, 20);
            v[k] = px_m(x, (int32_t)(((uint32_t)((27 << 20) + x2) >> 10 << 14) / (uint32_t)(((27 << 20) + 9 * x2) >> 10)), 14);
        }
        l[i] = (v[0] + v[1]) >> 1;
    }
}

static inline int32_t rg_dp_rd(const int16_t *line, uint32_t wp, int32_t off16)    /* interpolated read, Q12 -> Q20 */
{
    uint32_t o = (uint32_t)off16 >> 16;
    int32_t a = line[(wp + o) & RG_DPM], b = line[(wp + o + 1u) & RG_DPM];
    return (a + (((b - a) * (off16 & 0xFFFF)) >> 16)) << 8;
}

static inline void rg_dp_wr(int16_t *line, uint32_t wp, int32_t v)
{
    line[wp & RG_DPM] = (int16_t)px_clamp((v + 128) >> 8, -32768, 32767);
}

/* one block of the string synth: mono out (Q20), n <= 32 samples */
static __attribute__((noinline)) void rg_dp_run(rg_dpset_t *A, const rg_patch_t *P, int strum, int32_t *mono, uint32_t n)
{
    int32_t l[32], r[32], env[4], k, g, cn;
    int flags[4] = {0, 0, 0, 0}, poly = P->poly, fx = P->fx, chord_size;
    uint32_t i, arate, drate;
    int32_t shape = P->damping, attack = 0, drone = 0;
    if (n > 32u)
        n = 32u;
    if (poly != 1 && poly != 2)
        poly = 4;
    if (poly != A->poly) {                               /* StringSynthPart::set_polyphony */
        for (k = A->poly; k < poly; k++)
            A->grp[k].tonic = A->grp[0].tonic + k * 655;
        A->poly = poly;
        if (A->active >= poly)
            A->active = 0;
    }
    if (fx < 0 || fx > 5)
        fx = DPF_ENSEMBLE;
    if (fx % 3 != A->fx % 3)
        rg_dp_clear_fx(A);
    A->fx = fx;
    if (strum) {                                         /* a new note takes the next group */
        flags[A->active] = 2;
        A->active = (A->active + 1) % poly;
        flags[A->active] = 1;
        A->grp[A->active].tonic = P->note;
    }
    A->grp[A->active].chord = P->chord;
    /* ProcessEnvelopes: attack from SHAPE .5, decay time 0.18 s * 2^(7 d), in blocks of 32 samples */
    if (shape >= 32768)
        attack = (shape - 32768) * 2;
    {
        uint64_t at = ((uint64_t)px_exp2(attack * 8) * 451584u) >> 16;          /* 0.005 s * 1378.1 blocks/s * 2^(8 a), Q16 */
        uint64_t dt = ((uint64_t)px_exp2(px_m(shape, 7 * 65536, 16)) * 16257000u) >> 16;   /* 0.18 s * 1378.1 * 2^(7 d), Q16 */
        arate = (uint32_t)((1ull << 46) / (at ? at : 1));
        drate = (uint32_t)((1ull << 46) / (dt ? dt : 1));
    }
    if (shape >= 64225)
        drone = shape >= 65417 ? 65536 : (shape - 64225) * 55;                   /* (shape - 0.98) 55, to 1 */
    for (k = 0; k < poly; k++) {
        int32_t v = rg_dpe_run(&A->grp[k], flags[k], arate, drate);
        env[k] = v + px_m(65536 - v, drone, 16);
    }
    for (i = 0; i < n; i++)
        l[i] = r[i] = 0;
    chord_size = RG_DPV / poly < 8 ? RG_DPV / poly : 8;
    for (g = 0; g < poly; g++) {
        int32_t harm[6];
        rg_dp_registration(env[g] >> 2, P->brightness, harm);
        for (cn = 0; cn < chord_size; cn++) {
            int16_t c = poly == 1 ? RG_DPC1[A->grp[g].chord][cn] : poly == 2 ? RG_DPC2[A->grp[g].chord][cn] : RG_DPC4[A->grp[g].chord][cn];
            int32_t note = A->grp[g].tonic + P->tonic + (int32_t)(((int64_t)c * 65536) / 100);
            int32_t amp[6], nh = poly >= 2 && cn < 2 ? 2 : 3, h, ca = (c >= 0 && c <= 1700) ? 65536 : 45875;
            uint32_t f = rg_note_f(note);
            int32_t *dst = ((g + cn) & 1) ? l : r;      /* (Rings: out for odd, aux for even) */
            rg_dpo_t *o = A->osc[g * chord_size + cn];
            for (h = 0; h < 6; h++)
                amp[h] = px_m(ca, harm[h], 16);
            for (h = nh; h < 3; h++) {                   /* fold the truncated harmonics */
                amp[2 * (nh - 1)] += amp[2 * h];
                amp[2 * (nh - 1) + 1] += amp[2 * h + 1];
            }
            rg_dpo_run(&o[0], 1, f, amp[0], amp[1], dst, n);
            for (h = 1; h < nh; h++) {
                f = f > 0x7FFFFFFFu ? 0xFFFFFFFFu : f << 1;
                rg_dpo_run(&o[h], 0, f, amp[2 * h], amp[2 * h + 1], dst, n);
            }
        }
    }
    /* the effect on the pair (l = Rings' out, r = its aux) */
    switch (fx) {
    case DPF_FORMANT:
    case DPF_FORMANT2: {
        int32_t in[32], vowel = px_m(P->position, 262111, 16), vi = vowel >> 16, vf = vowel & 0xFFFF;   /* x 3.999 */
        int32_t q8 = fx == DPF_FORMANT ? 25 * 256 : 10 * 256;
        for (i = 0; i < n; i++) {
            in[i] = l[i] + r[i];
            l[i] = r[i] = 0;
        }
        for (k = 0; k < 3; k++) {
            int32_t fhz = RG_FORMANTS[vi][k] * 65536 + px_m(((int32_t)RG_FORMANTS[vi + 1][k] - (int32_t)RG_FORMANTS[vi][k]) * 65536, vf, 16);
            uint32_t fc;
            int32_t pan = k * 19661 + 13107;
            rg_svf_t c;
            if (fx == DPF_FORMANT2)
                fhz = px_m(fhz, 72090, 16);             /* x 1.1 */
            fc = (uint32_t)(((uint64_t)fhz << 16) / 44100u);            /* f / sr, Q32 (fhz: Hz Q16) */
            rg_svf_g(&c, rg_tan_dirty(fc), (uint32_t)q8);
            for (i = 0; i < n; i++) {
                int32_t lp, bp = rg_svf_step(&c, A->sv[k], (int64_t)in[i] << 4, &lp) >> 4;
                l[i] += px_m(px_m(bp, pan, 16), 32768, 16);            /* out: pan / 2, aux: (1 - pan) / 2 */
                r[i] += px_m(px_m(bp, 65536 - pan, 16), 32768, 16);
            }
        }
        break;
    }
    case DPF_CHORUS: {
        int32_t amount = P->position, dry = 65536 - (amount >> 1);
        int32_t depth = (int32_t)(((int64_t)(9830 + (amount >> 1)) * 384));      /* (0.15 + 0.5 p) 384 samples, Q16 */
        int16_t *ln = A->fx_mem.chorus;
        for (i = 0; i < n; i++) {
            int32_t s1, c1, s2, c2, wet;
            A->wp = (A->wp - 1u) & RG_DPM;
            A->lph[0] += RG_PH(4.17e-06);
            A->lph[1] += RG_PH(5.417e-06);
            s1 = px_sin(A->lph[0]);
            c1 = px_sin(A->lph[0] + 0x40000000u);
            s2 = px_sin(A->lph[1]);
            c2 = px_sin(A->lph[1] + 0x40000000u);
            rg_dp_wr(ln, A->wp, (l[i] + r[i]) >> 1);
            wet = (rg_dp_rd(ln, A->wp, (int32_t)(((int64_t)s1 * depth) >> 30) + 1200 * 65536) >> 1)
                  + (rg_dp_rd(ln, A->wp, (int32_t)(((int64_t)s2 * depth) >> 30) + 800 * 65536) >> 1);
            {
                int32_t wet2 = (rg_dp_rd(ln, A->wp, (int32_t)(((int64_t)c1 * depth) >> 30) + 800 * 65536) >> 1)
                               + (rg_dp_rd(ln, A->wp, (int32_t)(((int64_t)c2 * depth) >> 30) + 1200 * 65536) >> 1);
                l[i] = px_m(wet, amount, 16) + px_m(l[i], dry, 16);
                r[i] = px_m(wet2, amount, 16) + px_m(r[i], dry, 16);
            }
        }
        break;
    }
    case DPF_ENSEMBLE: {
        int32_t pos = P->position, amount = px_m(pos, 131072 - pos, 16), dry = 65536 - (amount >> 1);
        int32_t depth = (int32_t)(((int64_t)(13107 + px_m(px_m(pos, pos, 16), 52429, 16)) * 128));   /* (0.2 + 0.8 p^2) 128, Q16 */
        for (i = 0; i < n; i++) {
            int32_t sl[3], fa[3], m[3], j, wet, wet2, a = depth, b = depth / 10;
            uint32_t ph1, ph2;
            A->wp = (A->wp - 1u) & RG_DPM;
            A->lph[0] += RG_PH(1.57e-05);
            A->lph[1] += RG_PH(1.37e-04);
            ph1 = A->lph[0];
            ph2 = A->lph[1];
            for (j = 0; j < 3; j++) {
                sl[j] = px_sin(ph1 + (uint32_t)j * 0x55555555u);
                fa[j] = px_sin(ph2 + (uint32_t)j * 0x55555555u);
                m[j] = (int32_t)(((int64_t)sl[j] * a + (int64_t)fa[j] * b) >> 30) + 1024 * 65536;
            }
            rg_dp_wr(A->fx_mem.ens[0], A->wp, l[i]);
            rg_dp_wr(A->fx_mem.ens[1], A->wp, r[i]);
            wet = px_m(rg_dp_rd(A->fx_mem.ens[0], A->wp, m[0]) + rg_dp_rd(A->fx_mem.ens[0], A->wp, m[1])
                       + rg_dp_rd(A->fx_mem.ens[1], A->wp, m[2]), 21627, 16);          /* x 0.33 */
            wet2 = px_m(rg_dp_rd(A->fx_mem.ens[1], A->wp, m[0]) + rg_dp_rd(A->fx_mem.ens[1], A->wp, m[1])
                        + rg_dp_rd(A->fx_mem.ens[0], A->wp, m[2]), 21627, 16);
            l[i] = px_m(wet, amount, 16) + px_m(l[i], dry, 16);
            r[i] = px_m(wet2, amount, 16) + px_m(r[i], dry, 16);
        }
        break;
    }
    default: {                                           /* DPF_REVERB, DPF_REVERB2 */
        rg_rev_t *R = &A->rev;
        R->amount = P->position >> 1;
        R->diff = 40960;
        R->time = fx == DPF_REVERB ? 32768 + px_m(P->position, 32113, 16) : 19661 + px_m(P->position, 39322, 16);
        R->gain = 13107;
        R->lp = fx == DPF_REVERB ? 19661 : 39322;
        rg_rev_run(R, l, r, n);
        break;
    }
    }
    rg_dp_limit(A, l, r, n);
    for (i = 0; i < n; i++)
        mono[i] += l[i];
}
