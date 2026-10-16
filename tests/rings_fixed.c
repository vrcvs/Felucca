/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* The PHYS engine's Rings models (src/rings_dsp.c) as plain C calls for tests/rings_ref.cpp, which compares them
 * with Rings' own float code. Built by run_tests.sh with the firmware sources (through hostsim.c) and
 * -DRG_MODES_TOTAL=64: Rings' resolution. Parameters 0..1 as Rings takes them; the output as float (1.0 = Rings' 1.0). */
#define main hostsim_main
#include "hostsim.c"
#undef main

/* one Rings part rendered the way Rings does: a strum at each of strum_at[] (samples), the voices taking them in
 * turn (Part::Process: the first strum goes to voice 1 % poly), n samples of mono out (the voices' sum).
 * note: MIDI notes (69 = A440), structure .. position 0..1; model: Rings' ResonatorModel (0 modal, 1 sympathetic
 * strings, 2 string, 3 FM voice, 4 quantised sympathetic strings, 5 string and reverb) */
static union { rg_modset_t m; rg_strset_t s; rg_fmset_t f; rg_dpset_t d; } RGA;
static rg_lim_t RGL;

int rings_fixed_render(int model, int poly, int nstrum, const int *strum_at, const double *note, double structure,
                       double brightness, double damping, double position, int chord, int fx, unsigned seed, float *out, int n)
{
    int next = 0, vset[RG_MAXV] = {0};
    double vnote[RG_MAXV] = {0};
    int i, k, v;
    memset(&RGA, 0, sizeof RGA);
    memset(&RGL, 0, sizeof RGL);
    if (model == RG_DP)
        rg_dp_init(&RGA.d);
    RGL.peak = RG_ONE / 2;
    if (model == RG_STRREV)
        rg_rev_init(&RGA.s.rev, RG_RVBUF(&RGA.s));
    if (model != RG_MODAL && model != RG_FM && model != RG_DP)   /* the noise of the plucked strings: a different one per seed */
        for (k = 0; k < RG_MAXV; k++) {
            RGA.s.pl[k].rng = 0x2545F491u * (seed * 7u + (unsigned)k + 1u);
            RGA.s.s[k].rng = 0x9E3779B9u * (seed * 5u + (unsigned)k + 3u);
        }
    for (i = 0; i < n; i += 32) {
        int m = n - i < 32 ? n - i : 32;
        int32_t acc[32] = {0};
        int strum_voice = -1;
        rg_patch_t P;
        if (model == RG_DP) {                              /* the string synth is one part, not voices */
            int32_t mono[32] = {0};
            static int32_t held;                           /* the note stays between strums */
            rg_patch_t Q;
            memset(&Q, 0, sizeof Q);
            if (i == 0)
                held = (int32_t)(note[0] * 65536);
            Q.note = held;
            Q.poly = poly;
            Q.chord = chord;
            Q.fx = fx;
            Q.brightness = (int32_t)(brightness * 65536);
            Q.damping = (int32_t)(damping * 65536);
            Q.position = (int32_t)(position * 65536);
            for (k = 0; k < nstrum; k++)
                if (strum_at[k] >= i && strum_at[k] < i + 32) {
                    strum_voice = 1;
                    held = Q.note = (int32_t)(note[k] * 65536);
                }
            rg_dp_run(&RGA.d, &Q, strum_voice == 1, mono, (uint32_t)m);
            for (k = 0; k < m; k++)
                out[i + k] = (float)mono[k] / (1 << 20);
            continue;
        }
        for (k = 0; k < nstrum; k++)
            if (strum_at[k] >= i && strum_at[k] < i + 32) {
                next = (next + 1) % poly;
                strum_voice = next;
                vnote[next] = note[k];
                vset[next] = 1;
            }
        P.structure = (int32_t)(structure * 65536);
        P.brightness = (int32_t)(brightness * 65536);
        P.damping = (int32_t)(damping * 65536);
        P.position = (int32_t)(position * 65536);
        P.poly = poly;
        P.chord = chord;
        P.fx = 0;
        P.tonic = (int32_t)((note[0] - 5.0) * 65536);     /* the root: a fourth below the first note */
        for (v = 0; v < poly; v++) {
            int32_t y[32] = {0};
            if (!vset[v])
                continue;
            P.f = (uint32_t)(440.0 * pow(2.0, (vnote[v] - 69.0) / 12.0) / FS * 4294967296.0);
            P.voice = v;
            P.note = (int32_t)(vnote[v] * 65536);
            if (model == RG_MODAL) {
                rg_modal_blk_t B;
                rg_modal_block(&B, &RGA.m.m[v], &P, v == strum_voice);
                rg_modal_run(&B, &RGA.m.m[v], y, (uint32_t)m);
            } else if (model == RG_FM) {
                rg_fmv_run(&RGA.f, &P, v == strum_voice, y, (uint32_t)m);
            } else {
                rg_strv_run(&RGA.s, &P, model, v == strum_voice, y, (uint32_t)m);
            }
            for (k = 0; k < m; k++)
                acc[k] += y[k];
        }
        if (model != RG_MODAL && model != RG_FM)
            rg_post(&RGA.s, &RGL, model, &P, acc, (uint32_t)m);
        else
            rg_lim_run(&RGL, acc, (uint32_t)m, RG_GAIN[model]);
        for (k = 0; k < m; k++)
            out[i + k] = (float)acc[k] / (1 << 20);
    }
    return 0;
}

/* the Plucker alone: one trigger, n samples (the noise differs from Rings': compare spectra / level) */
void rings_fixed_pluck(double hz, double cutoff, double position, unsigned seed, float *out, int n)
{
    static rg_pl_t P;
    int32_t y[32];
    int i, k;
    memset(&P, 0, sizeof P);
    P.rng = 0x13579BDFu ^ (seed * 2654435761u);
    rg_pl_trigger(&P, (uint32_t)(hz / FS * 4294967296.0), (uint32_t)(cutoff * 4294967296.0), (int32_t)(position * 65536));
    for (i = 0; i < n; i += 32) {
        int m = n - i < 32 ? n - i : 32;
        rg_pl_run(&P, y, (uint32_t)m);
        for (k = 0; k < m; k++)
            out[i + k] = (float)y[k] / (1 << 20);
    }
}

/* one String fed with in[] (the exciter of the float original, so that both play the same input): out and pickup */
void rings_fixed_string(double hz, double dispersion, double brightness, double damping, double position, int has_disp,
                        const float *in, float *out, float *aux, int n)
{
    static rg_str_t S;
    static int16_t line[RG_LINE], sline[RG_SLINE];
    int i, k;
    memset(&S, 0, sizeof S);
    memset(line, 0, sizeof line);
    memset(sline, 0, sizeof sline);
    S.rng = 0x13579BDFu;
    for (i = 0; i < n; i += 32) {
        int m = n - i < 32 ? n - i : 32;
        int32_t x[32], o[32] = {0}, a[32] = {0};
        rg_str_blk_t B;
        for (k = 0; k < m; k++)
            x[k] = (int32_t)lrint(in[i + k] * (1 << 20));
        rg_str_block(&B, &S, (uint32_t)(hz / FS * 4294967296.0), (int32_t)(dispersion * 65536), (int32_t)(brightness * 65536),
                     (int32_t)(damping * 65536), (int32_t)(position * 65536), has_disp);
        rg_str_run(&B, &S, line, sline, x, o, a, (uint32_t)m);
        for (k = 0; k < m; k++) {
            out[i + k] = (float)o[k] / (1 << 20);
            aux[i + k] = (float)a[k] / (1 << 20);
        }
    }
}

/* the FM voice alone, triggered at sample 0 (deterministic: compared sample by sample) */
void rings_fixed_fm(double hz, double ratio, double brightness, double damping, double position, float *out, int n)
{
    static rg_fm_t V;
    rg_fm_blk_t B;
    rg_patch_t P;
    int i, k;
    int32_t y[32];
    memset(&V, 0, sizeof V);
    memset(&P, 0, sizeof P);
    P.f = (uint32_t)(hz / FS * 4294967296.0);
    P.structure = (int32_t)(ratio * 65536);
    P.brightness = (int32_t)(brightness * 65536);
    P.damping = (int32_t)(damping * 65536);
    P.position = (int32_t)(position * 65536);
    for (i = 0; i < n; i += 32) {
        int m = n - i < 32 ? n - i : 32;
        rg_fm_block(&B, &V, &P, i == 0);
        rg_fm_run(&B, &V, y, (uint32_t)m);
        for (k = 0; k < m; k++)
            out[i + k] = (float)y[k] / (1 << 20);
    }
}

/* the reverb alone on a mono impulse (amount 1: wet only), n samples, left output */
void rings_fixed_reverb(double time, double lp, double diffusion, float *out, int n)
{
    static rg_rev_t R;
    static int16_t buf[RG_RVN];
    int i, k;
    int32_t l[32], r[32];
    rg_rev_init(&R, buf);
    R.amount = 65536;
    R.gain = 13107;
    R.time = (int32_t)(time * 65536);
    R.lp = (int32_t)(lp * 65536);
    R.diff = (int32_t)(diffusion * 65536);
    for (i = 0; i < n; i += 32) {
        int m = n - i < 32 ? n - i : 32;
        for (k = 0; k < 32; k++)
            l[k] = r[k] = (i + k == 0) ? RG_ONE : 0;
        rg_rev_run(&R, l, r, (uint32_t)m);
        for (k = 0; k < m; k++)
            out[i + k] = (float)l[k] / (1 << 20);
    }
}
