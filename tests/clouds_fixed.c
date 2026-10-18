/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* The CLOUDS effect's fixed-point processor (src/clouds_dsp.c) as plain C calls for tests/clouds_ref.cpp, which
 * compares it with Clouds' own float code, and for tests/clouds_test.c. Built with build/gen's tables (2048-point
 * FFT, the device's) or build/host/cl4096's (Clouds' 4096 points: the reference test). */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "phys_dsp.c"
#include "clouds_dsp.c"

#ifdef CF_DEV                                            /* (the reference test links both: cf_ native, cfd_ device) */
#define CFN(x) cfd_##x
#else
#define CFN(x) cf_##x
#endif
static uint64_t *CF_ARENA;
static cl_t *CF;

/* rate: the processor's; outer: blocks at twice it (the device); sample_bytes: the samples' area (0: what is left
 * of 128 KiB, the device's); 0 = ok */
int CFN(init)(int rate, int outer, int blk, int sample_bytes)
{
    uint32_t bytes = sample_bytes > 0 ? cl_overhead(rate) + (uint32_t)sample_bytes : 131072u;
    free(CF_ARENA);
    CF_ARENA = (uint64_t *)calloc(1, (bytes + 7u) & ~7u);
    CF = cl_new(CF_ARENA, bytes, rate, outer, (uint32_t)blk);
    return CF ? 0 : 1;
}

void CFN(config)(int mode, int quality) { cl_config(CF, (uint32_t)mode, (uint32_t)quality); }
/* the reset done and the buffers cleared at once (Clouds' Prepare does both before its first block) */
void CFN(prime)(void)
{
    cl_prepare(CF);
    while (CF->clearing)
        cl_clear(CF, 1u << 20);
}
void CFN(orig_out)(int on) { CF->orig_out = (uint8_t)on; }
int CFN(clearing)(void) { return CF->clearing || CF->reset || CF->pmode != CF->mode; }
void CFN(gcap)(int g) { CF->gcap = g; }
uint32_t CFN(sizeof)(void) { return (uint32_t)sizeof(cl_t); }
int CFN(bufsize)(void) { return CF->buf[0].size; }
int CFN(ntex)(void) { return CF->st[0].ntex; }

/* par: position size pitch(semitones) density texture dry_wet spread feedback reverb freeze trigger gate; in, out:
 * n interleaved stereo frames, Clouds' 1.0 = 32768 */
void CFN(block)(const double *par, const int32_t *in, int32_t *out, int n)
{
    cl_par_t p;
    int32_t l[CL_BLK], r[CL_BLK], ol[CL_BLK], orr[CL_BLK];
    int i;
#define Q16(x) ((int32_t)((x) * 65536.0 + ((x) < 0 ? -0.5 : 0.5)))
    p.position = Q16(par[0]);
    p.size = Q16(par[1]);
    p.pitch = Q16(par[2]);
    p.density = Q16(par[3]);
    p.texture = Q16(par[4]);
    p.dry_wet = Q16(par[5]);
    p.spread = Q16(par[6]);
    p.feedback = Q16(par[7]);
    p.reverb = Q16(par[8]);
    p.freeze = par[9] != 0;
    p.trigger = par[10] != 0;
    p.gate = par[11] != 0;
    for (i = 0; i < n; i++) {
        l[i] = in[2 * i];
        r[i] = in[2 * i + 1];
    }
    cl_process(CF, &p, l, r, ol, orr, (uint32_t)n);
    for (i = 0; i < n; i++) {
        out[2 * i] = ol[i];
        out[2 * i + 1] = orr[i];
    }
}
