/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* CLOUDS (src/fx_clouds.c) in the mix (fx.c mix_block), on the host:
 *   OFF          MODE OFF after a while ON: the delay's line back cleared (nothing of Clouds' left in it), the delay
 *                sounding again; a project that never turns CLOUDS on is the goldens (regress.c).
 *   modes        GRAN STRT LOOP SPEC x HI ST / LO MO: sound out, no clipping, no DC, no NaN-like runaway; FREEZE keeps
 *                sounding after the input stops (GRAN, LOOP, SPEC).
 *   WET 0        Clouds' dry: the mix of MODE OFF one block late (11 kHz of bandwidth: correlated, level within 2 dB).
 *   routing      SRC T2 with part 2 silent: part 1 (PHYS) around Clouds, the output the mix of OFF one block late,
 *                bit for bit; SRC T2 with part 2 ANALOG playing, FREEZE: its notes frozen.
 *   steps        MODE OFF -> ON -> OFF -> ON under sound: no step larger than the music's own around the switches.
 *   two cores    every scene again with a thread as core 1 (tests/c1_host.h: every split in turn, Clouds among them,
 *                the firmware's choice, fixed): bit for bit the same (SPEC .. ST: the same on every split, mono on
 *                one core).
 *   build/host/clouds_test */
#include "c1_host.h"
#define main hostsim_main
#include "hostsim.c"
#undef main

static int fails;
#define CHECK(c, ...)                                                                                                  \
    do {                                                                                                               \
        if (!(c)) {                                                                                                    \
            printf("clouds: FAIL ");                                                                                   \
            printf(__VA_ARGS__);                                                                                       \
            printf("\n");                                                                                              \
            fails++;                                                                                                   \
        }                                                                                                              \
    } while (0)

#define SECS 3u
#define NF (SECS * FS)
static int32_t outb[2 * NF];

typedef struct {
    const char *name;
    int8_t mode, qual, frz_at, src, wet, sw;         /* frz_at: FREEZE from 1.5 s; sw: MODE toggled during the render */
    uint8_t eng1;                                    /* part 2's engine (part 1: PHYS) */
    uint8_t quiet2;                                  /* part 2 plays no notes */
} cs_t;

static void cp_defaults(void)
{
    uint32_t i;
    for (i = 0; i < CP_COUNT; i++)
        song.cp[i] = CPD[i].def;
}

/* PHYS on part 1 (notes until 1.5 s), part 2 another engine; DELAY sends on */
static uint64_t render(const cs_t *c, int dual, const char *mask)
{
    uint64_t h = 0xCBF29CE484222325ull;
    uint32_t f, i, k;
    host_tracks_init();
    cp_defaults();
    c1h_off = !dual;
    if (mask)
        setenv("C1_MASK", mask, 1);
    host_preset(&trk[0], ENGI_PHYS, 1);
    host_preset(&trk[1], c->eng1, 2);
    trk[0].p[P_DLY] = trk[1].p[P_DLY] = 40;
    if (c->quiet2)                                   /* (the exact check: no sends, which are not late) */
        for (k = 0; k < 2u; k++)
            trk[k].p[P_DLY] = trk[k].p[P_CHOR] = trk[k].p[P_REV] = 0;
    song.cp[CP_MODE] = c->mode;
    song.cp[CP_QUAL] = c->qual;
    song.cp[CP_SRC] = c->src;
    if (c->wet >= 0)
        song.cp[CP_WET] = c->wet;
    for (f = 0; f < NF; f += CTL) {
        uint32_t ms = f / (FS / 1000u);
        if (c->quiet2)                               /* (the bus on before the first note: the master's state the same) */
            ms = ms >= 20u ? ms - 20u : 9999u;
        if (ms % 250u == 0u && ms < 1500u && f % (FS / 1000u) < CTL) {
            trk_note_on(&trk[0], 48u + (ms / 250u) * 3u % 12u, 100);
            if (!c->quiet2)
                trk_note_on(&trk[1], 60u + (ms / 250u) * 5u % 12u, 90);
        }
        if (ms % 250u == 200u && f % (FS / 1000u) < CTL)
            for (k = 0; k < 2u; k++)
                for (i = 36; i < 84; i++)
                    trk_note_off(&trk[k], i);
        if (c->frz_at && ms == 1450u)
            song.cp[CP_FRZ] = 1;
        if (c->sw) {
            if (ms == 700u || ms == 1900u)
                song.cp[CP_MODE] = 0;
            if (ms == 1200u)
                song.cp[CP_MODE] = c->mode;
        }
        mix_block(&outb[2u * f], CTL);
        for (i = 0; i < 2u * CTL; i++) {
            uint32_t u = (uint32_t)outb[2u * f + i];
            for (k = 0; k < 4u; k++) {
                h ^= (u >> (8u * k)) & 0xFFu;
                h *= 0x100000001B3ull;
            }
        }
    }
    return h;
}

/* in a child of its own: every render from the state at boot; the output back too */
static uint64_t render_fork(const cs_t *c, int dual, const char *mask, int32_t *dly_nz)
{
    int fd[2];
    uint64_t h = 0;
    pid_t pid;
    if (pipe(fd))
        return 0;
    pid = fork();
    if (!pid) {
        FILE *w = fdopen(fd[1], "wb");
        int32_t nz = 0;
        uint32_t i;
        h = render(c, dual, mask);
        for (i = 0; i < DLY_LEN; i++)
            nz += dly_buf[i] != 0;
        nz = cld.own == CLD_OFF ? nz : -1 - (int32_t)cld.own;
        if (!w || fwrite(&h, sizeof h, 1, w) != 1 || fwrite(&nz, sizeof nz, 1, w) != 1 ||
            fwrite(outb, sizeof outb, 1, w) != 1)
            _exit(2);
        fclose(w);
        _exit(0);
    }
    close(fd[1]);
    {
        FILE *r = fdopen(fd[0], "rb");
        int32_t nz = 0;
        if (!r || fread(&h, sizeof h, 1, r) != 1 || fread(&nz, sizeof nz, 1, r) != 1 ||
            fread(outb, sizeof outb, 1, r) != 1)
            h = 0;
        if (dly_nz)
            *dly_nz = nz;
        if (r)
            fclose(r);
    }
    waitpid(pid, 0, 0);
    return h;
}

typedef struct { double rms, dc, peak, step; } st_t;

static st_t stats(const int32_t *o, uint32_t a, uint32_t b)
{
    st_t s = {0, 0, 0, 0};
    uint32_t i;
    for (i = a; i < b; i++) {
        double l = o[2u * i], r = o[2u * i + 1u];
        s.rms += l * l + r * r;
        s.dc += l + r;
        if (fabs(l) > s.peak)
            s.peak = fabs(l);
        if (fabs(r) > s.peak)
            s.peak = fabs(r);
        if (i > a && fabs(l - o[2u * i - 2u]) > s.step)
            s.step = fabs(l - o[2u * i - 2u]);
    }
    s.rms = sqrt(s.rms / (2.0 * (b - a)));
    s.dc /= 2.0 * (b - a);
    return s;
}

static int32_t ref[2 * NF];

int main(void)
{
    static const char *const MASKS[] = {"rot", "auto", "16", "17", "26", "1"};
    static const cs_t SC[] = {
        {"GRAN HI ST", 1, 0, 0, 0, -1, 0, 0},       {"GRAN LO MO", 1, 3, 0, 0, -1, 0, 0},
        {"STRT HI ST", 2, 0, 0, 0, -1, 0, 0},       {"STRT LO ST", 2, 2, 0, 0, -1, 0, 0},
        {"LOOP HI ST", 3, 0, 0, 0, -1, 0, 0},       {"LOOP HI MO", 3, 1, 0, 0, -1, 0, 0},
        {"SPEC HI ST", 4, 0, 0, 0, -1, 0, 0},       {"SPEC LO MO", 4, 3, 0, 0, -1, 0, 0},
        {"GRAN FREEZE", 1, 0, 1, 0, 127, 0, 0},     {"LOOP FREEZE", 3, 0, 1, 0, 127, 0, 0},
        {"SPEC FREEZE", 4, 0, 1, 0, 127, 0, 0},     {"GRAN switching", 1, 0, 0, 0, -1, 1, 0},
        {"SPEC switching", 4, 0, 0, 0, -1, 1, 0},   {"ROUTE PHYS only", 1, 0, 1, 0, 127, 0, 0},
        {"ROUTE T2", 1, 0, 1, 2, 127, 0, 0},
    };
    static const cs_t OFF = {"OFF", 0, 0, 0, 0, -1, 0, 0}, WET0 = {"WET 0", 3, 0, 0, 0, 0, 0, 0};
    uint32_t i, m;
    int32_t nz;
    double c = 0, e0 = 0, e1 = 0;
    st_t so, s;

    /* WET 0: the mix of OFF, one block late, band limited */
    render_fork(&OFF, 0, 0, 0);
    memcpy(ref, outb, sizeof ref);
    render_fork(&WET0, 0, 0, 0);
    for (i = FS / 2u; i < NF; i++) {
        double a = ref[2u * (i - CTL)] + ref[2u * (i - CTL) + 1u], b = outb[2u * i] + outb[2u * i + 1u];
        c += a * b;
        e0 += a * a;
        e1 += b * b;
    }
    c /= sqrt(e0 * e1 + 1);
    printf("clouds: WET 0 against OFF one block late: correlation %.3f, level %+.2f dB\n", c,
           10 * log10((e1 + 1) / (e0 + 1)));
    CHECK(c > 0.9 && fabs(10 * log10((e1 + 1) / (e0 + 1))) < 2.0, "WET 0 is not the dry mix");
    so = stats(ref, 0, NF);

    for (i = 0; i < sizeof SC / sizeof SC[0]; i++) {
        cs_t sc = SC[i];
        uint64_t h1, h2;
        uint32_t bad = 0;
        if (!strncmp(sc.name, "ROUTE", 5))
            sc.eng1 = 0;                                 /* ANALOG on part 2 */
        h1 = render_fork(&sc, 0, 0, &nz);
        s = stats(outb, 0, NF);
        if (sc.sw) {                                     /* back OFF at 1.9 s: the line the delay's again, cleared */
            st_t a = stats(outb, FS * 19u / 10u - CTL * 2u, FS * 19u / 10u + CTL * 4u);
            st_t b = stats(outb, FS * 6u / 10u, FS * 8u / 10u);
            printf("clouds: %-16s steps around the switches %.0f (music %.0f), delay line after: %d nonzero\n",
                   sc.name, a.step, b.step, nz);
            CHECK(a.step < 2.0 * b.step + 2000.0, "%s: a step at the switch", sc.name);
            CHECK(nz >= 0, "%s: CLOUDS still owns the line (%d)", sc.name, nz);
        }
        if (sc.frz_at) {                                 /* after the input stops: Clouds' frozen buffer only */
            st_t t = stats(outb, FS * 25u / 10u, NF);
            printf("clouds: %-16s frozen tail rms %.0f (playing %.0f)\n", sc.name, t.rms, s.rms);
            if (!strcmp(sc.name, "ROUTE PHYS only") || !strncmp(sc.name, "GRAN", 4) || !strncmp(sc.name, "LOOP", 4) ||
                !strncmp(sc.name, "SPEC", 4))
                CHECK(t.rms > s.rms * 0.05, "%s: FREEZE went silent", sc.name);
            if (!strcmp(sc.name, "ROUTE T2")) {
                /* T2 (ANALOG) through, PHYS around: the tail is the ANALOG part frozen */
                CHECK(t.rms > s.rms * 0.05, "%s: T2 not frozen", sc.name);
            }
        }
        printf("clouds: %-16s rms %.0f (OFF %.0f) peak %.0f dc %.1f\n", sc.name, s.rms, so.rms, s.peak, s.dc);
        CHECK(s.rms > so.rms * 0.1 && s.rms < so.rms * 6, "%s: level", sc.name);
        CHECK(s.peak < 8.0e6, "%s: runaway (peak %.0f)", sc.name, s.peak);
        CHECK(fabs(s.dc) < so.rms * 0.05 + 50, "%s: DC %.1f", sc.name, s.dc);
        for (m = 0; m < sizeof MASKS / sizeof MASKS[0]; m++) {
            h2 = render_fork(&sc, 1, MASKS[m], 0);
            if (sc.mode == 4 && !(sc.qual & 1) && !m)    /* SPEC stereo: mono on one core (fx_clouds.c cld_config) */
                h1 = h2;
            if (h2 != h1 || !h2) {
                printf("clouds: %-16s split %-4s FAIL (%016llx, one core %016llx)\n", sc.name, MASKS[m],
                       (unsigned long long)h2, (unsigned long long)h1);
                bad++;
            }
        }
        if (!bad)
            printf("clouds: %-16s the same on two cores, every split   ok\n", sc.name);
        fails += bad != 0;
    }
    {   /* routing: SRC T2, part 2 silent: PHYS around Clouds, the OFF mix one block late exactly */
        cs_t a = {"OFF quiet 2", 0, 0, 0, 0, -1, 0, 0, 1}, b = {"T2 quiet 2", 1, 0, 0, 2, -1, 0, 0, 1};
        uint32_t d = 0;
        render_fork(&a, 0, 0, 0);
        memcpy(ref, outb, sizeof ref);
        render_fork(&b, 0, 0, 0);
        for (i = 4u * CTL; i < NF; i++) {
            int x = ref[2u * (i - CTL)] != outb[2u * i] || ref[2u * (i - CTL) + 1u] != outb[2u * i + 1u];
            if (x && getenv("CLD_DBG") && d < 5)
                printf("  %u: %d %d / %d %d\n", i, ref[2u * (i - CTL)], ref[2u * (i - CTL) + 1u], outb[2u * i],
                       outb[2u * i + 1u]);
            d += x;
        }
        printf("clouds: SRC T2, part 2 silent: %u samples differ from OFF one block late\n", d);
        CHECK(d == 0, "SRC T2: part 1 did not go around Clouds");
    }
    printf("clouds: %d failures\n", fails);
    return fails != 0;
}
