/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Clouds reference test: the CLOUDS effect's fixed-point processor (src/clouds_dsp.c, through tests/clouds_fixed.c)
 * against Clouds' own float code (clouds/dsp/granular_processor.cc, Emilie Gillet, MIT; pichenettes/eurorack, not part
 * of this tree: run_tests.sh skips the test without it, CLOUDS=path to a checkout with stmlib next to clouds; its
 * window.h is patched there with Clouds 1.1's `done_ = false` in Window::Start, which eurorack's master lost in 2023).
 *   clouds_ref [WAVDIR]     VERBOSE=1: the figures of every case; WAVDIR: each case as float | fixed stereo WAV.
 * Part 1, NATIVE: the fixed processor at Clouds' rate (32 kHz, rs 1, no outer resampling, Clouds' 4096-point FFT:
 * clouds_fixed.o built with build/host/cl4096's tables), its buffers as large as Clouds', against the float one on the
 * same input, block for block: the deterministic modes sample for sample (SNR), the random ones by level and spectrum.
 * Part 2, DEVICE: the device's processor (22.05 kHz inside, 44.1 kHz outside through the 2:1 filters, 2048 points,
 * 128 KiB; clouds_fixed_dev.o) against the float one at 32 kHz on the same music: the times in seconds (the delay's
 * echo, the reverb's decay), the pitches (the shifted peak), the levels. */
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <string>
#include <vector>
#include <new>
#include "clouds/dsp/granular_processor.h"
#include "stmlib/utils/random.h"

using namespace clouds;

extern "C" {
int cf_init(int rate, int outer, int blk, int sample_bytes);
void cf_config(int mode, int quality);
void cf_orig_out(int on);
int cf_clearing(void);
void cf_prime(void);
void cfd_prime(void);
void cf_block(const double *par, const int32_t *in, int32_t *out, int n);
int cf_bufsize(void);
int cf_ntex(void);
int cfd_bufsize(void);
int cfd_init(int rate, int outer, int blk, int sample_bytes);
void cfd_config(int mode, int quality);
void cfd_orig_out(int on);
int cfd_clearing(void);
void cfd_block(const double *par, const int32_t *in, int32_t *out, int n);
}

enum { CL_GRANULAR, CL_STRETCH, CL_LOOPING, CL_SPECTRAL };   /* clouds_dsp.c, Clouds' PlaybackMode */
static int fails, cases;
static const char *wavdir;
static int verbose;

/* the knobs: position size pitch density texture dry_wet spread feedback reverb freeze trigger gate */
struct Knobs {
    double v[12];
};
static Knobs knobs(double pos, double size, double pitch, double dens, double tex, double wet, double spread, double fb,
                   double rev, int freeze = 0)
{
    Knobs k = {{pos, size, pitch, dens, tex, wet, spread, fb, rev, (double)freeze, 0, 0}};
    return k;
}

typedef double (*Sig)(double t, int ch);
static double sig_music(double t, int ch)                /* two tones a note every 0.5 s, over a bed of 48 quiet
                                                          * partials 120 Hz .. 7 kHz (the same in time at any rate) */
{
    double env = exp(-fmod(t, 0.5) * 4.0), n = 0;
    for (int k = 0; k < 48; k++)
        n += 0.004 * sin(2 * M_PI * 120.0 * pow(58.3, k / 47.0) * t + 2.1 * k * k + ch);
    return 0.35 * env * (sin(2 * M_PI * 440.0 * t + ch) + 0.5 * sin(2 * M_PI * 660.0 * t)) + n;
}
static double sig_sine(double t, int ch) { (void)ch; return 0.4 * sin(2 * M_PI * 440.0 * t); }
static double sig_click(double t, int ch) { (void)ch; return t < 0.002 ? 0.8 : 0.0; }
static double sig_clicks(double t, int ch) { (void)ch; return fmod(t, 1.0) < 0.002 ? 0.8 : 0.0; }   /* one a second */
static double sig_burst(double t, int ch)                /* 50 ms of noise at the start */
{
    (void)ch;
    return t < 0.05 ? (rand() / (double)RAND_MAX - 0.5) * 0.8 : 0.0;
}

static uint8_t big[118784], small[65536 - 128];
alignas(16) static uint8_t proc_mem[sizeof(GranularProcessor)];

/* the float processor over secs of sig at 32 kHz; the knobs can change at t_change */
static std::vector<double> run_float(int mode, int quality, Sig sig, double secs, const Knobs &k0, const Knobs *k1,
                                     double t_change)
{
    memset(proc_mem, 0, sizeof proc_mem);                /* a new one each time (Init leaves some members as they */
    GranularProcessor &p = *new (proc_mem) GranularProcessor();   /* were: grain_rate_phasor_) */
    std::vector<double> out;
    size_t nb = (size_t)(secs * 32000 / 32);
    srand(1);
    stmlib::Random::Seed(0x21);
    memset(big, 0, sizeof big);
    memset(small, 0, sizeof small);
    p.Init(big, sizeof big, small, sizeof small);
    p.set_playback_mode((PlaybackMode)mode);
    p.set_quality(quality);
    for (size_t b = 0; b < nb; b++) {
        ShortFrame in[32], o[32];
        const Knobs &k = (k1 && b * 32 >= t_change * 32000) ? *k1 : k0;
        Parameters *q = p.mutable_parameters();
        for (int i = 0; i < 32; i++) {
            double t = (double)(b * 32 + i) / 32000.0;
            in[i].l = (short)lrint(32767.0 * std::max(-1.0, std::min(1.0, sig(t, 0))));
            in[i].r = (short)lrint(32767.0 * std::max(-1.0, std::min(1.0, sig(t, 1))));
        }
        q->position = k.v[0];
        q->size = k.v[1];
        q->pitch = k.v[2];
        q->density = k.v[3];
        q->texture = k.v[4];
        q->dry_wet = k.v[5];
        q->stereo_spread = k.v[6];
        q->feedback = k.v[7];
        q->reverb = k.v[8];
        q->freeze = k.v[9] != 0;
        q->trigger = false;
        q->gate = false;
        p.Prepare();
        p.Process(in, o, 32);
        for (int i = 0; i < 32; i++) {
            out.push_back(o[i].l / 32768.0);
            out.push_back(o[i].r / 32768.0);
        }
    }
    return out;
}

/* the fixed one: dev 0 native (32 kHz, sample_bytes as Clouds'), 1 the device's (44.1 kHz outside) */
static std::vector<double> run_fixed(int dev, int mode, int quality, Sig sig, double secs, const Knobs &k0,
                                     const Knobs *k1, double t_change)
{
    std::vector<double> out;
    int fs = dev ? 44100 : 32000, sb = (quality & 1) ? 118784 : 2 * 65408;
    size_t nb = (size_t)(secs * fs / 32);
    srand(1);
    if (dev) {
        cfd_init(22050, 1, 32, 0);
        cfd_orig_out(1);
        cfd_config(mode, quality);
        cfd_prime();
    } else {
        cf_init(32000, 0, 32, sb);
        cf_orig_out(1);
        cf_config(mode, quality);
        cf_prime();
    }
    for (size_t b = 0; b < nb; b++) {
        int32_t in[64], o[64];
        const Knobs &k = (k1 && (double)b * 32 >= t_change * fs) ? *k1 : k0;
        for (int i = 0; i < 32; i++) {
            double t = (double)(b * 32 + i) / fs;
            in[2 * i] = (int32_t)lrint(32767.0 * std::max(-1.0, std::min(1.0, sig(t, 0))));
            in[2 * i + 1] = (int32_t)lrint(32767.0 * std::max(-1.0, std::min(1.0, sig(t, 1))));
        }
        if (dev)
            cfd_block(k.v, in, o, 32);
        else
            cf_block(k.v, in, o, 32);
        for (int i = 0; i < 64; i++)
            out.push_back(o[i] / 32768.0);
    }
    return out;
}

static void write_wav(const std::string &name, const std::vector<double> &a, const std::vector<double> &b, int fs_a)
{
    if (!wavdir)
        return;
    std::string path = std::string(wavdir) + "/" + name + ".wav";
    FILE *f = fopen(path.c_str(), "wb");
    if (!f)
        return;
    size_t n = std::max(a.size(), b.size()) / 2;
    uint32_t l;
    uint16_t s;
    fwrite("RIFF", 1, 4, f);
    l = 36 + (uint32_t)n * 8;
    fwrite(&l, 4, 1, f);
    fwrite("WAVEfmt ", 1, 8, f);
    l = 16;
    fwrite(&l, 4, 1, f);
    s = 1;
    fwrite(&s, 2, 1, f);
    s = 4;
    fwrite(&s, 2, 1, f);
    l = (uint32_t)fs_a;
    fwrite(&l, 4, 1, f);
    l = (uint32_t)fs_a * 8;
    fwrite(&l, 4, 1, f);
    s = 8;
    fwrite(&s, 2, 1, f);
    s = 16;
    fwrite(&s, 2, 1, f);
    fwrite("data", 1, 4, f);
    l = (uint32_t)n * 8;
    fwrite(&l, 4, 1, f);
    for (size_t i = 0; i < n; i++) {
        double v[4] = {i * 2 < a.size() ? a[2 * i] : 0, i * 2 < a.size() ? a[2 * i + 1] : 0,
                       i * 2 < b.size() ? b[2 * i] : 0, i * 2 < b.size() ? b[2 * i + 1] : 0};
        for (int k = 0; k < 4; k++) {
            int16_t x = (int16_t)lrint(std::max(-1.0, std::min(1.0, v[k])) * 32767.0);
            fwrite(&x, 2, 1, f);
        }
    }
    fclose(f);
}

/* figures over [t0, t1) s of an interleaved stereo signal at fs */
static double rms_db(const std::vector<double> &x, int fs, double t0, double t1)
{
    size_t a = (size_t)(t0 * fs) * 2, b = std::min(x.size(), (size_t)(t1 * fs) * 2);
    double e = 0;
    for (size_t i = a; i < b; i++)
        e += x[i] * x[i];
    return 10 * log10(e / std::max<size_t>(1, b - a) + 1e-20);
}
static double snr_db(const std::vector<double> &r, const std::vector<double> &x, double t0, double t1)
{
    size_t a = (size_t)(t0 * 32000) * 2, b = std::min(std::min(r.size(), x.size()), (size_t)(t1 * 32000) * 2);
    double e = 0, d = 0;
    for (size_t i = a; i < b; i++) {
        e += r[i] * r[i];
        d += (r[i] - x[i]) * (r[i] - x[i]);
    }
    return 10 * log10((e + 1e-20) / (d + 1e-20));
}
using cd = std::complex<double>;
static void fft(std::vector<cd> &v)
{
    size_t n = v.size(), j = 0;
    for (size_t i = 1; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(v[i], v[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        double ang = -2 * M_PI / (double)len;
        cd wl(cos(ang), sin(ang));
        for (size_t i = 0; i < n; i += len) {
            cd w(1);
            for (size_t k = 0; k < len / 2; k++) {
                cd u = v[i + k], t = v[i + k + len / 2] * w;
                v[i + k] = u + t;
                v[i + k + len / 2] = u - t;
                w *= wl;
            }
        }
    }
}
/* the strongest frequency (Hz) and the spectral centroid in [t0, t0 + 32768 / fs) of the left channel */
static void spectrum(const std::vector<double> &x, int fs, double t0, double *peak, double *centroid)
{
    const size_t N = 32768;
    std::vector<cd> v(N);
    size_t a = (size_t)(t0 * fs);
    for (size_t i = 0; i < N; i++) {
        double w = 0.5 - 0.5 * cos(2 * M_PI * i / N), s = 2 * (a + i) < x.size() ? x[2 * (a + i)] : 0;
        v[i] = cd(s * w, 0);
    }
    fft(v);
    double best = 0, bi = 0, sm = 0, sw = 0;
    for (size_t k = 2; k < N / 2; k++) {
        double m = std::abs(v[k]);
        if (m > best) {
            best = m;
            bi = (double)k;
        }
        sm += m * k;
        sw += m;
    }
    if (bi > 2 && bi < N / 2 - 1) {                      /* parabolic */
        double l = std::abs(v[(size_t)bi - 1]), c = std::abs(v[(size_t)bi]), r = std::abs(v[(size_t)bi + 1]);
        bi += 0.5 * (l - r) / (l - 2 * c + r);
    }
    *peak = bi * fs / N;
    *centroid = sw > 0 ? sm / sw * fs / N : 0;
}
/* the mean |difference| (dB) of the 1/3-octave band levels (100 Hz .. 0.45 fs) of two signals over [t0, t1), frames of
 * 4096 at their own rates (a and b may have different rates: the bands are in Hz) */
static double band_dist(const std::vector<double> &a, int fa, const std::vector<double> &b, int fb, double t0, double t1,
                        double fmax = 0)
{
    double sum = 0;
    if (fmax <= 0)
        fmax = 0.45 * std::min(fa, fb);
    int nb = 0;
    std::vector<double> ea, eb;
    for (int pass = 0; pass < 2; pass++) {
        const std::vector<double> &x = pass ? b : a;
        int fs = pass ? fb : fa;
        std::vector<double> &e = pass ? eb : ea;
        const size_t N = 4096;
        for (double f = 100; f < fmax; f *= 1.2599)
            e.push_back(1e-20);
        for (size_t s0 = (size_t)(t0 * fs); s0 + N < (size_t)(t1 * fs) && 2 * (s0 + N) < x.size(); s0 += N) {
            std::vector<cd> v(N);
            for (size_t i = 0; i < N; i++)
                v[i] = cd(x[2 * (s0 + i)] * (0.5 - 0.5 * cos(2 * M_PI * i / N)), 0);
            fft(v);
            for (size_t k = 1; k < N / 2; k++) {
                double f = (double)k * fs / N, m = std::norm(v[k]);
                if (f < 100 || f >= fmax)
                    continue;
                size_t band = (size_t)(log(f / 100) / log(1.2599));
                if (band < e.size())
                    e[band] += m / fs;               /* (per Hz of bandwidth: rates comparable) */
            }
        }
    }
    double ta = 0, tb = 0;
    for (size_t i = 0; i < ea.size(); i++) {
        ta += ea[i];
        tb += eb[i];
    }
    for (size_t i = 0; i < ea.size(); i++) {
        if (ea[i] < ta * 1e-4 && eb[i] < tb * 1e-4)  /* (bands 40 dB below the total: not compared) */
            continue;
        sum += fabs(10 * log10(ea[i] / ta) - 10 * log10(eb[i] / tb));
        nb++;
    }
    return nb ? sum / nb : 99;
}

/* the energy envelope: dB per 50 ms from t0, n windows */
static std::vector<double> envelope(const std::vector<double> &x, int fs, double t0, int n)
{
    std::vector<double> e;
    for (int i = 0; i < n; i++)
        e.push_back(rms_db(x, fs, t0 + 0.05 * i, t0 + 0.05 * (i + 1)));
    return e;
}
/* the time (s) of the first sample after t0 above thr (abs, left) */
static double onset(const std::vector<double> &x, int fs, double t0, double thr)
{
    for (size_t i = (size_t)(t0 * fs); 2 * i < x.size(); i++)
        if (fabs(x[2 * i]) > thr)
            return (double)i / fs;
    return -1;
}

static void check(const char *name, bool ok, const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    cases++;
    if (!ok)
        fails++;
    printf("clouds_ref: %-58s %s %s\n", name, buf, ok ? "ok" : "FAIL");
}

/* ---- part 1: native (32 kHz, rs 1), the same input, block for block */
static void native(void)
{
    struct C {
        const char *name;
        int mode, quality;
        Sig sig;
        Knobs k;
        int freeze_at;                                   /* s, 0: never */
        double min_snr;                                  /* sample for sample (0: not compared) */
        double lvl_db;                                   /* the levels' tolerance */
    } cs[] = {
        /* name, mode, quality, input, knobs (position size pitch density texture blend spread feedback reverb),
         * frozen from (s), SNR over 0.5 .. 1.5 s at least (0: not compared: random), the levels' tolerance (dB) */
        {"LOOP delay pos .25", CL_LOOPING, 0, sig_music, knobs(0.25, 0.5, 0, 0, 0.5, 1, 0, 0, 0), 0, 30, 0.5},
        {"LOOP delay pos .7 texture .2 (filters)", CL_LOOPING, 0, sig_music, knobs(0.703125, 0.5, 0, 0, 0.203125, 1, 0, 0, 0), 0, 30, 0.5},
        {"LOOP delay pitch +12 (the pitch shifter)", CL_LOOPING, 0, sig_music, knobs(0.296875, 0.59375, 12, 0, 0.5, 1, 0, 0, 0), 0, 18, 1.0},
        {"LOOP delay feedback .6 density .4 (diffuser)", CL_LOOPING, 0, sig_music, knobs(0.09375, 0.5, 0, 0.40625, 0.5, 1, 0, 0.59375, 0), 0, 25, 1.0},
        {"LOOP frozen at 1 s, size .6, pitch -5", CL_LOOPING, 0, sig_music, knobs(0.40625, 0.59375, -5, 0, 0.5, 1, 0, 0, 0), 1, 25, 1.0},
        {"LOOP reverb 1 blend .7", CL_LOOPING, 0, sig_music, knobs(0.203125, 0.5, 0, 0, 0.5, 0.703125, 0, 0, 1), 0, 20, 1.0},
        {"GRAN deterministic, density .3", CL_GRANULAR, 0, sig_music, knobs(0.296875, 0.5, 0, 0.296875, 0.296875, 1, 0, 0, 0), 0, 30, 1.0},
        {"GRAN deterministic, texture .9 (window, diffusion)", CL_GRANULAR, 0, sig_music, knobs(0.5, 0.296875, 7, 0.203125, 0.90625, 1, 0, 0, 0), 0, 10, 1.0},
        {"GRAN random, density .8, spread .5", CL_GRANULAR, 0, sig_music, knobs(0.40625, 0.59375, 0, 0.796875, 0.5, 1, 0.5, 0, 0), 0, 0, 1.5},
        {"GRAN mono (quality 1)", CL_GRANULAR, 1, sig_music, knobs(0.296875, 0.5, -12, 0.25, 0.59375, 1, 0, 0, 0), 0, 10, 1.0},
        {"GRAN low fidelity (quality 2: mu-law, 16 kHz)", CL_GRANULAR, 2, sig_music, knobs(0.296875, 0.5, 0, 0.296875, 0.296875, 1, 0, 0, 0), 0, 25, 1.5},
        {"STRT stretch, pitch 0", CL_STRETCH, 0, sig_music, knobs(0.296875, 0.5, 0, 0.296875, 0.5, 1, 0, 0, 0), 0, 0, 1.5},
        {"STRT pitch +7", CL_STRETCH, 0, sig_sine, knobs(0.203125, 0.59375, 7, 0.0, 0.5, 1, 0, 0, 0), 0, 0, 1.5},
        {"SPEC defaults", CL_SPECTRAL, 0, sig_music, knobs(0.5, 0.5, 0, 0.5, 0.5, 1, 0, 0, 0), 0, 30, 1.5},
        {"SPEC pitch +12", CL_SPECTRAL, 0, sig_sine, knobs(0.5, 0.5, 12, 0.5, 0.5, 1, 0, 0, 0), 0, 18, 1.5},
        {"SPEC mono, warp, quantize", CL_SPECTRAL, 1, sig_music, knobs(0.296875, 0.796875, 0, 0.703125, 0.203125, 1, 0, 0, 0), 0, 15, 2.0},
    };
    for (auto &c : cs) {
        Knobs k1 = c.k;
        k1.v[9] = 1;
        std::vector<double> f = run_float(c.mode, c.quality, c.sig, 4.0, c.k, c.freeze_at ? &k1 : 0, c.freeze_at);
        std::vector<double> x = run_fixed(0, c.mode, c.quality, c.sig, 4.0, c.k, c.freeze_at ? &k1 : 0, c.freeze_at);
        double lf = rms_db(f, 32000, 0.5, 4.0), lx = rms_db(x, 32000, 0.5, 4.0), snr = snr_db(f, x, 0.5, 1.5);
        double bd = band_dist(f, 32000, x, 32000, 0.5, 4.0), pf, cf, px, cx;
        bool ok = fabs(lf - lx) <= c.lvl_db && (c.min_snr <= 0 || snr >= c.min_snr) && lf > -60 && bd <= 1.0;
        spectrum(f, 32000, 2.5, &pf, &cf);
        spectrum(x, 32000, 2.5, &px, &cx);
        if (c.sig == sig_sine)
            ok = ok && fabs(1200 * log2(px / pf)) < 20;
        else
            ok = ok && fabs(log2(cx / cf)) < 0.25;
        check(c.name, ok, "level %.1f / %.1f dB, SNR %.0f dB, bands %.2f dB, peak %.0f / %.0f Hz, centroid %.0f / %.0f Hz", lf, lx,
              snr, bd, pf, px, cf, cx);
        if (verbose) {
            printf("    SNR by 0.25 s:");
            for (double t = 0; t < 4.0; t += 0.25)
                printf(" %.0f", snr_db(f, x, t, t + 0.25));
            printf("\n    level by 0.25 s (float / fixed):");
            for (double t = 0; t < 4.0; t += 0.25)
                printf(" %.0f/%.0f", rms_db(f, 32000, t, t + 0.25), rms_db(x, 32000, t, t + 0.25));
            printf("\n");
        }
        if (wavdir) {
            std::string n = std::string("native_") + c.name;
            for (auto &ch : n)
                if (!isalnum((unsigned char)ch))
                    ch = '_';
            write_wav(n, f, x, 32000);
        }
    }
}

/* the decay of a tail, dB / s: a line through the 50 ms envelope from 6 dB below its peak to 40 dB below (or the end) */
static double decay_rate(const std::vector<double> &x, int fs, double t0, double t1)
{
    std::vector<double> e = envelope(x, fs, t0, (int)((t1 - t0) / 0.05));
    size_t pk = (size_t)(std::max_element(e.begin(), e.end()) - e.begin());
    double sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0;
    for (size_t i = pk; i < e.size(); i++) {
        if (e[i] > e[pk] - 6)
            continue;
        if (e[i] < e[pk] - 40)
            break;
        double t = 0.05 * i;
        sx += t;
        sy += e[i];
        sxx += t * t;
        sxy += t * e[i];
        n++;
    }
    return n > 2 ? (n * sxy - sx * sy) / (n * sxx - sx * sx) : 0;
}

/* ---- part 1b: the reverb's impulse response (native) */
static void native_reverb(void)
{
    Knobs k = knobs(0.0, 0.5, 0, 0, 0.5, 1, 0, 0, 0.59375);
    std::vector<double> f = run_float(CL_LOOPING, 0, sig_click, 4.0, k, 0, 0), x = run_fixed(0, CL_LOOPING, 0, sig_click, 4.0, k, 0, 0);
    std::vector<double> ef = envelope(f, 32000, 0.1, 50), ex = envelope(x, 32000, 0.1, 50);
    double worst = 0, rf = decay_rate(f, 32000, 0.05, 4.0), rx = decay_rate(x, 32000, 0.05, 4.0);
    for (size_t i = 0; i < ef.size(); i++)
        if (ef[i] > -70)
            worst = std::max(worst, fabs(ef[i] - ex[i]));
    check("LOOP reverb .6: the impulse response's envelope", worst < 2.0 && fabs(rx / rf - 1) < 0.1,
          "50 ms levels within %.2f dB, decay %.1f / %.1f dB/s", worst, rf, rx);
    write_wav("native_reverb_ir", f, x, 32000);
}

/* ---- part 2: the device's processor (44.1 kHz outside, 22.05 inside) against Clouds at 32 kHz, in seconds and Hz */
static void device(void)
{
    /* the times: the delay's echo, the reverb's decay */
    {
        /* (the delay glides to POSITION in ~0.6 s: the click at 5 s; its echo after the pitch shifter's half-line,
         * 5.75 ms at SIZE .5, and the delay: each buffer's own length, Clouds' 32696 samples at 32 kHz, the device's) */
        Knobs k = knobs(0.5, 0.5, 0, 0, 0.5, 1, 0, 0, 0);
        std::vector<double> f = run_float(CL_LOOPING, 0, sig_clicks, 6.0, k, 0, 0), x = run_fixed(1, CL_LOOPING, 0, sig_clicks, 6.0, k, 0, 0);
        double of = onset(f, 32000, 5.01, 0.05) - 5.0, ox = onset(x, 44100, 5.01, 0.05) - 5.0;
        double want = 0.5 * (cfd_bufsize() - 44) / 22050.0 + 0.00575, wantf = 0.5 * (32696 - 64) / 32000.0 + 0.00575;
        check("DEVICE: LOOP pos .5: the echo's time", fabs(of - wantf) < 0.002 && fabs(ox - want) < 0.002 && fabs(ox / of - 1) < 0.06,
              "%.4f s / %.4f s (their buffers: %.4f / %.4f s)", of, ox, wantf, want);
    }
    {
        Knobs k = knobs(0.0, 0.5, 0, 0, 0.5, 1, 0, 0, 0.6);
        std::vector<double> f = run_float(CL_LOOPING, 0, sig_click, 4.0, k, 0, 0), x = run_fixed(1, CL_LOOPING, 0, sig_click, 4.0, k, 0, 0);
        double rf = decay_rate(f, 32000, 0.05, 4.0), rx = decay_rate(x, 44100, 0.05, 4.0), lf = rms_db(f, 32000, 0.1, 1.0),
               lx = rms_db(x, 44100, 0.1, 1.0);
        check("DEVICE: reverb .6: the decay", fabs(rx / rf - 1) < 0.15 && fabs(lf - lx) < 2.0, "%.1f / %.1f dB/s, level %.1f / %.1f dB",
              rf, rx, lf, lx);
        write_wav("device_reverb_ir_float32k", f, f, 32000);
        write_wav("device_reverb_ir_fixed44k", x, x, 44100);
    }
    /* the pitches and levels of each mode */
    struct C {
        const char *name;
        int mode, quality;
        Sig sig;
        Knobs k;
        double want_hz;                                  /* the shifted peak (0: music, levels and bands only; < 0: the bands
                                                          * up to -want_hz Hz only, the low-pass of TEXTURE in) */
    } cs[] = {
        {"DEVICE: GRAN pitch +12", CL_GRANULAR, 0, sig_sine, knobs(0.3, 0.5, 12, 0.3, 0.3, 1, 0, 0, 0), 880},
        {"DEVICE: GRAN music", CL_GRANULAR, 0, sig_music, knobs(0.3, 0.5, 0, 0.3, 0.3, 1, 0, 0, 0), 0},
        {"DEVICE: GRAN random, spread", CL_GRANULAR, 0, sig_music, knobs(0.4, 0.6, 0, 0.8, 0.5, 1, 0.5, 0, 0), 0},
        {"DEVICE: GRAN low fidelity", CL_GRANULAR, 2, sig_music, knobs(0.3, 0.5, 0, 0.3, 0.3, 1, 0, 0, 0), 0},
        {"DEVICE: GRAN mono, pitch -12", CL_GRANULAR, 1, sig_sine, knobs(0.3, 0.5, -12, 0.25, 0.6, 1, 0, 0, 0), 220},
        {"DEVICE: STRT pitch +7", CL_STRETCH, 0, sig_sine, knobs(0.2, 0.6, 7, 0, 0.5, 1, 0, 0, 0), 659.26},
        {"DEVICE: STRT music", CL_STRETCH, 0, sig_music, knobs(0.3, 0.5, 0, 0.3, 0.5, 1, 0, 0, 0), 0},
        {"DEVICE: LOOP pitch +12 (the pitch shifter)", CL_LOOPING, 0, sig_sine, knobs(0.3, 0.6, 12, 0, 0.5, 1, 0, 0, 0), 880},
        {"DEVICE: LOOP feedback, diffuser, filters", CL_LOOPING, 0, sig_music, knobs(0.1, 0.5, 0, 0.4, 0.4, 1, 0, 0.6, 0.3), -4000},
        {"DEVICE: SPEC pitch +12 (2048 points)", CL_SPECTRAL, 0, sig_music, knobs(0.5, 0.5, 12, 0.5, 0.5, 1, 0, 0, 0), 0},
        {"DEVICE: SPEC music", CL_SPECTRAL, 0, sig_music, knobs(0.5, 0.5, 0, 0.5, 0.5, 1, 0, 0, 0), 0},
        {"DEVICE: SPEC low fidelity, mono", CL_SPECTRAL, 3, sig_music, knobs(0.3, 0.7, 0, 0.6, 0.6, 1, 0, 0, 0), 0},
    };
    for (auto &c : cs) {
        std::vector<double> f = run_float(c.mode, c.quality, c.sig, 4.0, c.k, 0, 0);
        std::vector<double> x = run_fixed(1, c.mode, c.quality, c.sig, 4.0, c.k, 0, 0);
        double lf = rms_db(f, 32000, 0.5, 4.0), lx = rms_db(x, 44100, 0.5, 4.0), pf, cf, px, cx;
        double fm = c.want_hz < 0 ? -c.want_hz : c.quality & 2 ? 4500 : 9000, bd = band_dist(f, 32000, x, 44100, 0.5, 4.0, fm);
        bool ok = fabs(lf - lx) <= 1.5 && lf > -60;
        spectrum(f, 32000, 2.5, &pf, &cf);
        spectrum(x, 44100, 2.5, &px, &cx);
        if (c.want_hz > 0)                               /* (at the pitch, or where Clouds puts it: the looper's
                                                          * pitch shifter's grains move the peak of a sine) */
            ok = ok && fabs(1200 * log2(px / pf)) < 15 && fabs(1200 * log2(pf / c.want_hz)) < 60;
        else
            ok = ok && bd <= 2.0;
        check(c.name, ok, "level %.1f / %.1f dB, bands to %.0f Hz %.2f dB, peak %.1f / %.1f Hz", lf, lx, fm, bd, pf, px);
        if (wavdir) {
            std::string n = c.name;
            for (auto &ch : n)
                if (!isalnum((unsigned char)ch))
                    ch = '_';
            write_wav(n + "_float32k", f, f, 32000);
            write_wav(n + "_fixed44k", x, x, 44100);
        }
    }
}

int main(int argc, char **argv)
{
    wavdir = argc > 1 ? argv[1] : 0;
    verbose = getenv("VERBOSE") != 0;
    native();
    native_reverb();
    device();
    printf("clouds_ref: %d cases, %d failed\n", cases, fails);
    return fails ? 1 : 0;
}
