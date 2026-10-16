/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Rings reference test: the PHYS engine's fixed-point Rings models (src/rings_dsp.c) against Rings' own float
 * code (rings/dsp/part.cc, Emilie Gillet, MIT; pichenettes/eurorack, not part of this tree: run_tests.sh skips the
 * test without it, RINGS=path to a checkout with stmlib next to rings).
 *   rings_ref [WAVDIR]     VERBOSE=1: every case; WAVDIR: each case as float | fixed stereo WAV.
 * The float Part runs with its internal exciter, strums at chosen samples; the fixed side is built with 64 modes
 * (Rings' own count: the device runs fewer). The exciters are noise-driven, so the comparison is of spectra:
 * the three strongest peaks of the float render are found in the fixed one (PEAK_CT cents), the decay
 * (DECAY_TOL) and the level (printed, LEVEL_DB tolerance), no blow-up, no DC. */
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "stmlib/stmlib.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/utils/random.h"
using namespace stmlib;
#include "rings/dsp/part.h"
#include "rings/dsp/plucker.h"
#include "rings/dsp/string.h"
#include "rings/dsp/fm_voice.h"
#include "rings/dsp/fx/reverb.h"
#include "rings/dsp/string_synth_part.h"

extern "C" int rings_fixed_render(int model, int poly, int nstrum, const int *strum_at, const double *note,
                                  double structure, double brightness, double damping, double position, int chord,
                                  int fx, unsigned seed, float *out, int n);

static const double FS = 44100.0;
#define PEAK_CT 5.0
#define DECAY_TOL 0.10
#define DECAY_DB 5.0
#define LEVEL_DB 8.0

using cd = std::complex<double>;
static void fft(std::vector<cd> &a)
{
    size_t n = a.size(), j = 0;
    for (size_t i = 1; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        double ang = -2 * M_PI / (double)len;
        cd wl(cos(ang), sin(ang));
        for (size_t i = 0; i < n; i += len) {
            cd w(1);
            for (size_t k = 0; k < len / 2; k++) {
                cd u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

struct peak { double f, db; };

/* spectral peaks (dB, refined) of x[start .. start + N) above max - floor_db, strongest first */
static std::vector<peak> peaks(const std::vector<float> &x, size_t start, size_t N, double floor_db)
{
    std::vector<cd> a(N);
    std::vector<double> m(N / 2);
    std::vector<peak> out;
    double mx = -1e9;
    for (size_t i = 0; i < N; i++) {
        double w = 0.5 - 0.5 * cos(2 * M_PI * (double)i / (double)N);
        a[i] = start + i < x.size() ? x[start + i] * w : 0.0;
    }
    fft(a);
    size_t lo = (size_t)(30.0 * (double)N / FS) + 2;   /* from 30 Hz: sub-audio content does not count */
    for (size_t i = 0; i < N / 2; i++) {
        m[i] = 20 * log10(std::abs(a[i]) + 1e-12);
        if (i >= lo)
            mx = std::max(mx, m[i]);
    }
    for (size_t i = lo; i + 2 < N / 2; i++)
        if (m[i] > m[i - 1] && m[i] >= m[i + 1] && m[i] > mx - floor_db) {
            double d = 0.5 * (m[i - 1] - m[i + 1]) / (m[i - 1] - 2 * m[i] + m[i + 1]);
            out.push_back({((double)i + d) * FS / (double)N, m[i] - 0.25 * (m[i - 1] - m[i + 1]) * d});
        }
    std::sort(out.begin(), out.end(), [](const peak &p, const peak &q) { return p.db > q.db; });
    return out;
}

/* decay: the energy of 0.05..0.35 s over that of [end - 0.6 s, end - 0.3 s), dB. The late energy counts at least
 * as an RMS of FLOOR: the fixed render settles at ~1e-5 of full scale (the rounding of the lowest modes, below one
 * LSB of the 16-bit output), the float one goes on down */
#define FLOOR 3e-5
static double decay(const std::vector<float> &x)
{
    double a = 0, b = 0;
    size_t n = x.size(), i, w = (size_t)(0.3 * FS);
    for (i = (size_t)(0.05 * FS); i < (size_t)(0.35 * FS); i++)
        a += (double)x[i] * x[i];
    for (i = n - (size_t)(0.6 * FS); i < n - (size_t)(0.3 * FS); i++)
        b += (double)x[i] * x[i];
    return 10 * log10((a + 1e-20) / std::max(b, FLOOR * FLOOR * (double)w));
}

/* the early decay: the RMS of 0.05..0.15 s over that of 0.15..0.35 s, dB, both floored at RMS 1e-4 (the fixed render
 * settles near 5e-5 of full scale, the rounding of its lowest modes; the float one goes on down) */
static double decay2(const std::vector<float> &x)
{
    auto r = [&](double a, double b) {
        double s = 0;
        size_t i0 = (size_t)(a * FS), i1 = (size_t)(b * FS);
        for (size_t i = i0; i < i1; i++)
            s += (double)x[i] * x[i];
        return std::max(sqrt(s / (double)(i1 - i0)), 1e-4);
    };
    return 20 * log10(r(0.05, 0.15) / r(0.15, 0.35));
}

static double rms(const std::vector<float> &x, size_t a, size_t b)
{
    double s = 0;
    for (size_t i = a; i < b && i < x.size(); i++)
        s += (double)x[i] * x[i];
    return sqrt(s / (double)(b - a));
}

static void wav(const char *dir, const char *name, const std::vector<float> &l, const std::vector<float> &r)
{
    char p[512];
    snprintf(p, sizeof p, "%s/%s.wav", dir, name);
    FILE *f = fopen(p, "wb");
    if (!f)
        return;
    uint32_t n = (uint32_t)l.size(), b = n * 4u, v = 36 + b, h[4] = {16, 0x00020001u, 44100u, 44100u * 4u},
             h2 = 0x00100004u;
    fwrite("RIFF", 1, 4, f);
    fwrite(&v, 4, 1, f);
    fwrite("WAVEfmt ", 1, 8, f);
    fwrite(h, 4, 4, f);
    fwrite(&h2, 4, 1, f);
    fwrite("data", 1, 4, f);
    fwrite(&b, 4, 1, f);
    for (uint32_t i = 0; i < n; i++) {
        int16_t s[2] = {(int16_t)std::max(-32767.f, std::min(32767.f, l[i] * 8000.f)),
                        (int16_t)std::max(-32767.f, std::min(32767.f, r[i] * 8000.f))};
        fwrite(s, 2, 2, f);
    }
    fclose(f);
}


struct Case { int model, poly, nstrum; int at[4]; double note[4]; double st, br, dm, pos; int chord; };

static void float_render(const Case &c, std::vector<float> &out, std::vector<float> &aux, int n)
{
    static uint16_t buf[65536];
    static rings::Part p;
    memset((void *)&p, 0, sizeof p);
    memset(buf, 0, sizeof buf);
    p.Init(buf);
    p.set_polyphony(c.poly);
    p.set_model((rings::ResonatorModel)c.model);
    rings::Patch pa = {(float)c.st, (float)c.br, (float)c.dm, (float)c.pos};
    rings::PerformanceState ps = {};
    ps.internal_exciter = true;
    ps.internal_strum = false;
    ps.chord = c.chord;
    out.assign(n, 0.f);
    aux.assign(n, 0.f);
    float in[24] = {0}, o[24], a[24];
    int k = 0;
    const double T = c.note[0] - 5;    /* the root: a fourth below the first note (the sympathetic tonic string) */
    ps.note = (float)(c.note[0] - T);  /* the pitch is there before the strum (the strings have settled on it) */
    ps.tonic = (float)T;
    for (int i = 0; i + 24 <= n; i += 24) {
        ps.strum = false;
        for (int j = 0; j < c.nstrum; j++)
            if (c.at[j] >= i && c.at[j] < i + 24) {
                ps.strum = true;
                ps.note = (float)(c.note[j] - T);
                ps.tonic = (float)T;
            }
        p.Process(ps, pa, in, o, a, 24);
        for (int s = 0; s < 24; s++) {
            out[i + s] = o[s];
            aux[i + s] = a[s];
        }
        k++;
    }
}

extern "C" void rings_fixed_pluck(double hz, double cutoff, double position, unsigned seed, float *out, int n);
extern "C" void rings_fixed_string(double hz, double dispersion, double brightness, double damping, double position,
                                   int has_disp, const float *in, float *out, float *aux, int n);
extern "C" void rings_fixed_fm(double hz, double ratio, double brightness, double damping, double position, float *out, int n);
extern "C" void rings_fixed_reverb(double time, double lp, double diffusion, float *out, int n);

/* error of b against a in dB of a's energy */
static double err_db(const std::vector<float> &a, const std::vector<float> &b, size_t i0, size_t i1)
{
    double e = 0, s = 0;
    for (size_t i = i0; i < i1; i++) {
        e += ((double)a[i] - b[i]) * ((double)a[i] - b[i]);
        s += (double)a[i] * a[i];
    }
    return 10 * log10(e / (s + 1e-20) + 1e-30);
}

/* components: the Plucker's level, the String on the float Plucker's burst, the FM voice sample by sample, the
 * reverb's decay. Returns the failures. */
static int components(int *cases)
{
    int fails = 0;
    const int N = (int)(2 * FS);
    const double FQ[] = {55.0, 130.81, 261.63, 523.25, 1046.5};
    /* Plucker: the rms of the burst + comb + low-pass over 20 triggers, float against fixed (the noise differs) */
    for (double hz : FQ) {
        double rf = 0, rx = 0;
        for (int t = 0; t < 20; t++) {
            rings::Plucker pl;
            std::vector<float> e(2400), x(2400);
            pl.Init();
            pl.Trigger((float)(hz / FS), 0.1f, 0.3f);
            for (int i = 0; i < 2400; i += 24)
                pl.Process(&e[i], 24);
            rings_fixed_pluck(hz, 0.1, 0.3, t + 1, x.data(), 2400);
            rf += rms(e, 0, 2400) / 20;
            rx += rms(x, 0, 2400) / 20;
        }
        double d = 20 * log10(rx / rf);
        (*cases)++;
        if (fabs(d) > 3 || getenv("VERBOSE"))
            printf("rings_ref: plucker %.0f Hz: rms float %.4f fixed %.4f (%+.1f dB) %s\n", hz, rf, rx, d, fabs(d) > 3 ? "FAIL" : "ok");
        fails += fabs(d) > 3;
    }
    /* String on the float exciter: the waveform at the start (before the tuning's rounding drifts the phase), the
     * pitch (the strongest peak), the level */
    for (double hz : FQ)
        for (int cfg = 0; cfg < 6; cfg++) {
            static const double DISP[] = {0, 0.32, -0.5, 0.6, 0, -0.9};
            static const double BR[] = {0.3, 0.6, 0.9, 0.5, 0.8, 0.2}, DM[] = {0.3, 0.5, 0.8, 0.7, 0.97, 0.4};
            double dsp = DISP[cfg], br = BR[cfg], dm = DM[cfg], pos = 0.3;
            rings::Plucker pl;
            rings::String s;
            std::vector<float> e(N), o(N, 0.f), a(N, 0.f), xo(N), xa(N);
            pl.Init();
            pl.Trigger((float)(hz / FS), 0.15f, (float)pos);
            for (int i = 480; i + 24 <= N; i += 24)      /* (after 20 blocks of silence: the float String ramps from 220 Hz) */
                pl.Process(&e[i], 24);
            s.Init(cfg != 0 && cfg != 4);
            s.set_frequency((float)(hz / FS));
            s.set_dispersion((float)dsp);
            s.set_brightness((float)br);
            s.set_damping((float)dm);
            s.set_position((float)pos);
            for (int i = 0; i + 24 <= N; i += 24)
                s.Process(&e[i], &o[i], &a[i], 24);
            rings_fixed_string(hz, dsp, br, dm, pos, cfg != 0 && cfg != 4, e.data(), xo.data(), xa.data(), N);
            std::vector<peak> pf = peaks(o, (size_t)(0.02 * FS), 16384, 30), px = peaks(xo, (size_t)(0.02 * FS), 16384, 30);
            /* the strongest partials of the fixed string are partials of the float one (the burst's noise differs) */
            double ct = 0;
            bool pitch_ok = !px.empty() && !pf.empty();
            for (size_t k = 0; k < px.size() && k < 3 && pitch_ok; k++) {
                double best = 1e9;
                for (size_t j = 0; j < pf.size() && j < 8; j++)
                    best = std::min(best, fabs(1200 * log2(px[k].f / pf[j].f)));
                ct = std::max(ct, best);
                pitch_ok = best < 8;
            }
            double lv = 20 * log10((rms(xo, (size_t)(0.1 * FS), (size_t)(0.5 * FS)) + 1e-9) / (rms(o, (size_t)(0.1 * FS), (size_t)(0.5 * FS)) + 1e-9));
            double lim = hz < 90 ? 8 : 6;
            double d1 = 0;
            bool bad = fabs(lv) > lim || !pitch_ok;
            (*cases)++;
            if (bad || getenv("VERBOSE"))
                printf("rings_ref: string %.0f Hz disp %.2f br %.1f dm %.2f: partials off by up to %.1f ct, level %+.1f dB %s\n", hz, dsp, br, dm, ct, lv, bad ? "FAIL" : "ok");
            fails += bad;
        }
    /* FM voice: the strongest partials and the level (the block of 24 / 32 samples ramps differ: phases drift) */
    for (double hz : FQ)
        for (int cfg = 0; cfg < 6; cfg++) {
            static const double RT[] = {0.1, 0.3, 0.5, 0.7, 0.9, 0.5}, BR[] = {0.3, 0.6, 0.9, 0.5, 0.8, 0.2},
                                DM[] = {0.3, 0.5, 0.8, 0.95, 0.4, 0.6}, PS[] = {0.5, 0.8, 0.2, 0.5, 1.0, 0.0};
            static rings::FMVoice vs;
            memset((void *)&vs, 0, sizeof vs);
            rings::FMVoice &v = vs;
            std::vector<float> in(N, 0.f), o(N), a(N), xo(N);
            v.Init();
            v.set_frequency((float)(hz / FS));
            v.set_ratio((float)RT[cfg]);
            v.set_brightness((float)BR[cfg]);
            v.set_damping((float)DM[cfg]);
            v.set_feedback_amount((float)PS[cfg]);
            v.set_position(0.f);
            v.TriggerInternalEnvelope();
            for (int i = 0; i + 24 <= N; i += 24)
                v.Process(&in[i], &o[i], &a[i], 24);
            rings_fixed_fm(hz, RT[cfg], BR[cfg], DM[cfg], PS[cfg], xo.data(), N);
            std::vector<peak> pf = peaks(o, (size_t)(0.3 * FS), 32768, 25), px = peaks(xo, (size_t)(0.3 * FS), 32768, 40);
            double worst = 0;
            for (size_t k = 0, m = 0; k < pf.size() && m < 3; k++) {
                if (pf[k].f > 16000)
                    continue;
                m++;
                double best = 1e9;
                for (auto &q : px)
                    best = std::min(best, fabs(1200 * log2(q.f / pf[k].f)));
                worst = std::max(worst, best);
            }
            double lv = 20 * log10((rms(xo, (size_t)(0.2 * FS), (size_t)(0.6 * FS)) + 1e-9) / (rms(o, (size_t)(0.2 * FS), (size_t)(0.6 * FS)) + 1e-9));
            bool bad = worst > 8 || fabs(lv) > 3;
            (*cases)++;
            if (bad || getenv("VERBOSE"))
                printf("rings_ref: fm %.0f Hz ratio %.1f br %.1f dm %.2f fb %.1f: partials off %.1f ct, level %+.1f dB %s\n", hz, RT[cfg], BR[cfg], DM[cfg], PS[cfg], worst, lv, bad ? "FAIL" : "ok");
            fails += bad;
        }
    /* Reverb: the decay of the impulse response (energy of 0.3..0.6 s over 0.9..1.2 s) */
    for (double tm : {0.4, 0.7, 0.95}) {
        static rings::Reverb rv; memset((void *)&rv, 0, sizeof rv);
        static uint16_t buf[32768];
        std::vector<float> l(N, 0.f), r(N, 0.f), xl(N);
        memset(buf, 0, sizeof buf);
        rv.Init(buf);
        rv.set_amount(1.f);
        rv.set_diffusion(0.625f);
        rv.set_time((float)tm);
        rv.set_input_gain(0.2f);
        rv.set_lp(0.6f);
        l[0] = r[0] = 1.f;
        for (int i = 0; i + 24 <= N; i += 24)
            rv.Process(&l[i], &r[i], 24);
        rings_fixed_reverb(tm, 0.6, 0.625, xl.data(), N);
        auto dec = [&](const std::vector<float> &x) {
            double a = 0, b = 0;
            for (int i = (int)(0.3 * FS); i < (int)(0.6 * FS); i++)
                a += (double)x[i] * x[i];
            for (int i = (int)(0.9 * FS); i < (int)(1.2 * FS); i++)
                b += (double)x[i] * x[i];
            return 10 * log10((a + 1e-20) / (b + 1e-20));
        };
        double df = dec(l), dx = dec(xl), lf = 10 * log10(rms(l, (size_t)(0.1 * FS), (size_t)(0.5 * FS)) + 1e-9),
               lx = 10 * log10(rms(xl, (size_t)(0.1 * FS), (size_t)(0.5 * FS)) + 1e-9);
        (*cases)++;
        bool bad = fabs(df - dx) > 0.4 * fabs(df) + 3;
        if (bad || getenv("VERBOSE"))
            printf("rings_ref: reverb time %.2f: decay float %.1f dB fixed %.1f dB (level %.1f / %.1f dB) %s\n", tm, df, dx, lf, lx, bad ? "FAIL" : "ok");
        fails += bad;
    }
    return fails;
}


/* Disastrous Peace: StringSynthPart against the fixed port (deterministic: the oscillators, the effects' LFOs), the
 * mono sum (out - aux) / 2 of the original's limited pair against the port's mono */
static int dp_tests(int *cases)
{
    int fails = 0;
    const int N = (int)(2 * FS);
    static const double REG[] = {0.1, 0.55, 1.0}, SHAPE[] = {0.3, 0.7, 0.99}, POS[] = {0.2, 0.5, 0.9};
    for (int poly = 1; poly <= 4; poly *= 2)
        for (int fx = 0; fx < 6; fx++)
            for (int cfg = 0; cfg < 3; cfg++)
                for (int chord : {0, 4, 9}) {
                    double reg = REG[cfg], shape = SHAPE[cfg], pos = POS[(cfg + fx) % 3], nt = 57;
                    static uint16_t buf[65536];
                    static rings::StringSynthPart sp;
                    memset((void *)&sp, 0, sizeof sp);
                    memset(buf, 0, sizeof buf);
                    sp.Init(buf);
                    sp.set_polyphony(poly);
                    sp.set_fx((rings::FxType)fx);
                    rings::Patch pa = {0.5f, (float)reg, (float)shape, (float)pos};
                    rings::PerformanceState ps = {};
                    ps.internal_exciter = true;
                    ps.chord = chord;
                    ps.tonic = 0;
                    std::vector<float> fo(N), fa(N), xo(N);
                    float in[24] = {0}, o[24], a[24];
                    const int at[2] = {960, 12000};
                    const double notes[2] = {nt, nt + 5};
                    for (int i = 0; i + 24 <= N; i += 24) {
                        ps.strum = false;
                        for (int j = 0; j < 2; j++)
                            if (at[j] >= i && at[j] < i + 24)
                                ps.strum = true, ps.note = (float)notes[j];
                        if (!ps.strum)
                            ps.note = (float)(i < 12000 ? notes[0] : notes[1]);
                        sp.Process(ps, pa, in, o, a, 24);
                        for (int s = 0; s < 24; s++)
                            fo[i + s] = 0.5f * (o[s] - a[s]), fa[i + s] = 0;
                    }
                    rings_fixed_render(6, poly, 2, at, notes, 0.5, reg, shape, pos, chord, fx, 1, xo.data(), N);
                    std::vector<peak> pf = peaks(fo, (size_t)(0.1 * FS), 32768, 40), px = peaks(xo, (size_t)(0.1 * FS), 32768, 60);
                    int bad = 0;
                    char why[256] = "";
                    for (size_t k = 0, m = 0; k < pf.size() && m < 3; k++) {
                        if (pf[k].f > 12000)
                            continue;
                        m++;
                        double best = 1e9;
                        for (auto &q : px)
                            best = std::min(best, fabs(1200 * log2(q.f / pf[k].f)));
                        if (best > PEAK_CT + 1200 * log2(1 + 1.35 / pf[k].f)) {
                            bad = 1;
                            snprintf(why + strlen(why), sizeof why - strlen(why), " peak %.1f Hz off %.1f ct;", pf[k].f, best);
                        }
                    }
                    double rf = rms(fo, (size_t)(0.1 * FS), (size_t)(0.5 * FS)), rx = rms(xo, (size_t)(0.1 * FS), (size_t)(0.5 * FS));
                    double lv = 20 * log10((rx + 1e-9) / (rf + 1e-9));
                    double rf2 = rms(fo, (size_t)(0.7 * FS), (size_t)(1.2 * FS)), rx2 = rms(xo, (size_t)(0.7 * FS), (size_t)(1.2 * FS));
                    double lv2 = 20 * log10((rx2 + 1e-9) / (rf2 + 1e-9));
                    float mxv = 0;
                    for (float v : xo)
                        mxv = std::max(mxv, fabsf(v));
                    if (fabs(lv) > 3 || fabs(lv2) > 4) { bad = 1; snprintf(why + strlen(why), sizeof why - strlen(why), " level;"); }
                    if (mxv > 1.2) { bad = 1; snprintf(why + strlen(why), sizeof why - strlen(why), " blow-up;"); }
                    (*cases)++;
                    if (bad || getenv("VERBOSE"))
                        printf("rings_ref: dp poly %d fx %d reg %.2f shape %.2f pos %.1f chord %d: rms %.4f/%.4f level %+.1f / %+.1f dB%s %s\n",
                               poly, fx, reg, shape, pos, chord, rf, rx, lv, lv2, why, bad ? "FAIL" : "ok");
                    fails += bad;
                }
    return fails;
}

static const char *MN[] = {"modal", "symp", "string", "fm", "quant", "strrev"};

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : 0;
    const char *only = getenv("MODEL");
    static const double ST[] = {0.1, 0.5, 0.9};
    static const double BR[] = {0.4, 0.8}, DM[] = {0.4, 0.8};
    static const double NOTES[] = {48, 72};
    int fails = 0, cases = 0;
    const int N = (int)(2 * FS);
    if (!only || atoi(only) < 0)
        fails += components(&cases);
    for (int model = 0; model < 6; model++) {
        if (only && atoi(only) < 0)
            break;
        if (only && atoi(only) != model)
            continue;
        for (int poly = 1; poly <= 4; poly *= 2)
            for (double nt : NOTES)
                for (double st : ST)
                    for (double br : BR)
                        for (double dm : DM)
                            for (int chord = 0; chord < (model == 4 ? 2 : 1); chord++) {
                                Case c = {model, poly, poly > 1 ? 2 : 1, {960, 4960, 0, 0}, {nt, nt + 7, 0, 0}, st, br, dm, 0.25, chord ? 5 : 0};
                                /* the noise bursts are random (the level of one run varies by +-8 dB): the level and the decay are the means over
                                 * NSEED runs of each, the level as a mean of dB */
                                const int NSEED = model == 0 || model == 3 ? 1 : 24;      /* (the modal and the FM voices are deterministic) */
                                std::vector<float> fo, fa, xo(N);
                                std::vector<double> rfs, rxs, dfs, dxs;
                                auto mono = [&](std::vector<float> &f, std::vector<float> &a) {
                                    for (int i = 0; i < N; i++)
                                        f[i] = model == 0 ? f[i] + a[i]                 /* modal: the two sets of harmonics */
                                               : model == 5 ? f[i] - a[i]               /* left + right of the reverb */
                                               : model == 3 ? (poly == 1 ? f[i] : f[i] + a[i])
                                               : poly == 1 ? f[i] - a[i] : f[i] + a[i]; /* strings: out - aux per voice */
                                };
                                std::vector<float> fo0, xo0;
                                for (int sd = 0; sd < NSEED; sd++) {
                                    stmlib::Random::Seed(1234567u + 7919u * (unsigned)sd);
                                    float_render(c, fo, fa, N);
                                    rings_fixed_render(model, poly, c.nstrum, c.at, c.note, st, br, dm, c.pos, c.chord, 0, (unsigned)sd + 1, xo.data(), N);
                                    mono(fo, fa);
                                    if (!sd) {
                                        fo0 = fo;
                                        xo0 = xo;
                                    }
                                    rfs.push_back(20 * log10(rms(fo, 0, (size_t)(0.25 * FS)) + 1e-9));
                                    rxs.push_back(20 * log10(rms(xo, 0, (size_t)(0.25 * FS)) + 1e-9));
                                    dfs.push_back(decay2(fo));
                                    dxs.push_back(decay2(xo));
                                }
                                auto med = [](const std::vector<double> &v) { double s = 0; for (double x : v) s += x; return s / (double)v.size(); };
                                fo = fo0;
                                xo = xo0;
                                std::vector<peak> pf = peaks(fo, (size_t)(0.1 * FS), 32768, 40), px = peaks(xo, (size_t)(0.1 * FS), 32768, 60);
                                int bad = 0;
                                char why[256] = "";
                                for (size_t k = 0, m = 0; k < pf.size() && m < 3; k++) {
                                    if (pf[k].f > 16000)
                                        continue;
                                    m++;
                                    double best = 1e9;
                                    for (auto &q : px)
                                        best = std::min(best, fabs(1200 * log2(q.f / pf[k].f)));
                                    if (best > PEAK_CT + 1200 * log2(1 + 1.35 / pf[k].f)) {      /* (+ the FFT's resolution) */
                                        bad = 1;
                                        snprintf(why + strlen(why), sizeof why - strlen(why), " peak %.1f Hz off %.1f ct;", pf[k].f, best);
                                    }
                                }
                                double dd = med(dfs) - med(dxs);
                                double lv = med(rxs) - med(rfs);
                                double rf = pow(10, med(rfs) / 20), rx = pow(10, med(rxs) / 20);
                                float mxv = 0;
                                for (float v : xo)
                                    mxv = std::max(mxv, fabsf(v));
                                if (fabs(dd) > DECAY_DB) { bad = 1; snprintf(why + strlen(why), sizeof why - strlen(why), " decay;"); }
                                if (fabs(lv) > LEVEL_DB) { bad = 1; snprintf(why + strlen(why), sizeof why - strlen(why), " level;"); }
                                if (mxv > 2) { bad = 1; snprintf(why + strlen(why), sizeof why - strlen(why), " blow-up;"); }
                                cases++;
                                if (bad || getenv("VERBOSE"))
                                    printf("rings_ref: %s poly %d note %.0f st %.1f br %.1f dm %.1f ch %d: rms %.4f/%.4f level %+.1f dB decay %+.1f dB%s %s\n",
                                           MN[model], poly, nt, st, br, dm, c.chord, rf, rx, lv, dd, why, bad ? "FAIL" : "ok");
                                fails += bad;
                                if (dir) {
                                    char nm[128];
                                    snprintf(nm, sizeof nm, "%s_p%d_n%.0f_s%.0f_b%.0f_d%.0f_c%d", MN[model], poly, nt, st * 10, br * 10, dm * 10, chord);
                                    wav(dir, nm, fo, xo);
                                }
                            }
    }
    if (!only || atoi(only) == 6)
        fails += dp_tests(&cases);
    printf("rings_ref: %d cases, %d failed\n", cases, fails);
    return fails != 0;
}
