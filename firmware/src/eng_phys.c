/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* PHYS: physical models. The DSP is ported to fixed point from DaisySP (Electrosmith, Emilie Gillet;
 * MIT: phys_dsp.c) and Rings' sympathetic strings (Emilie Gillet; MIT: phys_symp.c); this file is the
 * Felucca engine around them.
 *
 * MODEL picks one of four (the values are stored: new ones are appended):
 *   MODAL   a bank of 12 band-pass modes (bells, bars, plates): STRC stretches the partials
 *           (below 0.25 compressed, 0.25..0.3 harmonic, above more and more inharmonic), BRIT
 *           keeps the high modes ringing and opens the strike, DAMP the decay (127 rings on),
 *           POS the strike position (the mode amplitudes follow cos(2 pi pos i), 0..1/4).
 *           Struck by an impulse into a low-pass; with BOW > 0 sustained by random hits (bowed /
 *           blown) at that level instead
 *   STRNG   an extended Karplus-Strong string, plucked by a noise burst one period long: STRC
 *           below ~0.25 a curved bridge (buzzy, sitar-like), above it dispersion (stiff, piano-
 *           like), BRIT the burst's tone and the loop filter, DAMP the decay, POS the pluck
 *           position (a comb on the burst, 0 = none); BOW > 0 again sustains it by random hits
 *   MEMB    a struck drum head: the modal bank with the modes of a circular membrane. HARM blends an
 *           ideal head (inharmonic: toms, timpani) into a loaded one (harmonic overtones: a tabla-like
 *           hand drum), POS goes from the centre (round, only the circular modes) to the rim, BEND
 *           drops the pitch after the strike (up to 12 semitones); BRIT, DAMP as MODAL
 *   SYMP    the plucked string with three sympathetic strings that ring along (a sitar's, a
 *           harp's): CHRD tunes them (OCT 5TH 4TH MAJ MIN SUS 7TH above the note, ROOT: the track's
 *           scale root, its fifth and octave around C3), SYMP how much they take from the string, BUZZ
 *           the curved bridge; BRIT, DAMP as STRNG
 * ACC is the accent at full velocity (brighter, longer, louder strike; velocity scales it), EXC
 * mixes the exciter itself into the output (the mallet / pick). The track's ENV / LFO -> FLT move BRIT.
 *
 * MODEL 4 was DRUM before 1.0: the kit is its own engine now (eng_drum.c); projects and user presets
 * that saved it load as that engine (core.h drum_from_phys).
 *
 * DUST (resonant particles, value 2 before 1.0) was dropped: MODEL 2 is MEMB now. Projects and
 * user presets saved before say so (project.c, upreset.c) and load DUST as MODAL bowed, its nearest
 * kin (random hits into resonators): phys_legacy.
 *
 * Polyphony 3 per part (engine_t.poly): each part's voices 0..2 own a state slot in the part's engine
 * memory (engines.c eng_mem; phys_slot_of: SYMP's, the largest: the string's 512 + 128 sample lines, Q20, and three
 * 256-sample sympathetic lines, 16-bit), 12 slots in all. Voice amplitude: the track's ADSR as for every
 * engine; the presets hold SUS at 127 so the model's own decay is heard and REL damps it after the key.
 * Cost (host, 8 notes asked = 3 voices): see the README and cpu_baseline.txt. */
#include "phys_dsp.c"
#include "phys_symp.c"
#include "rings_dsp.c"

/* PM_MODAL .. PM_SYMP: DaisySP's and Rings' sympathetic strings in a voice each; PM_R*: Rings' models whole (rings_dsp.c:
 * one part-wide set of voices, below) */
enum { PM_MODAL, PM_STRING, PM_MEMB, PM_SYMP, PM_RMODAL, PM_RSYMP, PM_RSTRING, PM_RFM, PM_RQUANT, PM_RSTRREV, PM_RDISP, PM_COUNT };
#define PM_RINGS PM_RMODAL
#define PHYS_POLY 3
/* output: model 1.0 -> this (VOICE_FS scale), then a soft knee; per model (a strike of all the modes in
 * phase peaks high but is short) */
static const int32_t PHYS_GAIN[PM_COUNT] = {28000, 12000, 90000, 12000, 24000, 24000, 24000, 24000, 24000, 24000, 24000};

typedef struct {
    uint8_t model;               /* PM_* the state belongs to */
    union {
        px_modal_t m;            /* MODAL, MEMB */
        px_string_t s;
        px_symp_t y;
    } u;
} phys_slot_t;

ENG_MEM_FITS("PHYS", sizeof(phys_slot_t) * PHYS_POLY);   /* (it sets ENG_MEM_PART) */

static const char *const N_PHYS_MODEL[] = {"MODAL", "STRNG", "MEMB", "SYMP", "R.MOD", "R.SYM", "R.STR", "R.FM", "R.QNT", "R.S+R", "R.DSP"};
static const char *const N_PHYS_POLY[] = {"1", "2", "4", 0};
static const char *const N_PHYS_FX[] = {"FORM", "CHOR", "REVB", "FOR2", "ENSM", "REV2", 0};
static const char *const N_PHYS_CHORD[] = {"OCT", "5TH", "4TH", "MAJ", "MIN", "SUS", "7TH", "ROOT", 0};
/* per model: STRC .. EXC under their own names (phys_desc; range and default as edit[], 0 = edit[]) */
static const param_desc_t PHYS_D_HARM = {"HARM", F_PCT, 0, 127, 64, 0, 0};
static const param_desc_t PHYS_D_BEND = {"BEND", F_PCT, 0, 127, 0, 0, 0};
static const param_desc_t PHYS_D_CHRD = {"CHRD", F_INT, 0, 127, 64, N_PHYS_CHORD, 0};
static const param_desc_t PHYS_D_SYMP = {"SYMP", F_PCT, 0, 127, 20, 0, 0};
static const param_desc_t PHYS_D_BUZZ = {"BUZZ", F_PCT, 0, 127, 0, 0, 0};
static const param_desc_t PHYS_D_STRU = {"STRU", F_PCT, 0, 127, 64, 0, 0};
static const param_desc_t PHYS_D_POLY = {"POLY", F_ENUM, 0, 2, 0, N_PHYS_POLY, 0};
static const param_desc_t PHYS_D_CH11 = {"CHRD", F_INT, 0, 10, 0, 0, 0};
static const param_desc_t PHYS_D_TONC = {"TONC", F_INT, 0, 127, 0, 0, 0};   /* 0 = a fourth below the first note */
static const param_desc_t PHYS_D_TUNE = {"TUNE", F_SEMI, -12, 12, 0, 0, 0};
static const param_desc_t PHYS_D_FX = {"FX", F_ENUM, 0, 5, 0, N_PHYS_FX, 0};
static const param_desc_t PHYS_D_BRIG = {"BRIG", F_PCT, 0, 127, 64, 0, 0};
static const param_desc_t PHYS_D_DAMP = {"DAMP", F_PCT, 0, 127, 64, 0, 0};
static const param_desc_t PHYS_D_POSN = {"POSN", F_PCT, 0, 127, 64, 0, 0};
#define PHYS_RDESC(e5, e6, e7) {0, &PHYS_D_STRU, &PHYS_D_BRIG, &PHYS_D_DAMP, &PHYS_D_POSN, e5, e6, e7}
static const param_desc_t *const PHYS_DESC[PM_COUNT][8] = {
    [PM_RMODAL] = PHYS_RDESC(&PHYS_D_POLY, 0, &PHYS_D_TUNE),
    [PM_RSYMP] = PHYS_RDESC(&PHYS_D_POLY, &PHYS_D_TONC, &PHYS_D_TUNE),
    [PM_RSTRING] = PHYS_RDESC(&PHYS_D_POLY, 0, &PHYS_D_TUNE),
    [PM_RFM] = PHYS_RDESC(&PHYS_D_POLY, 0, &PHYS_D_TUNE),
    [PM_RQUANT] = PHYS_RDESC(&PHYS_D_POLY, &PHYS_D_CH11, &PHYS_D_TUNE),
    [PM_RSTRREV] = PHYS_RDESC(&PHYS_D_POLY, 0, &PHYS_D_TUNE),
    [PM_RDISP] = {0, 0, &PHYS_D_BRIG, &PHYS_D_DAMP, &PHYS_D_POSN, &PHYS_D_POLY, &PHYS_D_CH11, &PHYS_D_FX},
    [PM_MEMB] = {0, &PHYS_D_HARM, 0, 0, 0, 0, &PHYS_D_BEND, 0},
    [PM_SYMP] = {0, &PHYS_D_CHRD, 0, 0, &PHYS_D_SYMP, 0, &PHYS_D_BUZZ, 0},
};

static const param_desc_t *phys_desc(const track_t *t, uint32_t k)
{
    return k < 8u ? PHYS_DESC[(uint32_t)t->p[P_E0] % PM_COUNT][k] : 0;
}

/* SYMP: the sympathetic strings above the note, cents (CHRD's names) */
static const int16_t PHYS_CHORDS[7][PX_NSYMP] = {
    {-1200, 1200, 2400}, {700, 1200, 1900}, {500, 1200, 1700}, {400, 700, 1200},
    {300, 700, 1200}, {200, 700, 1400}, {400, 700, 1000},
};
static const int8_t PHYS_DETUNE[PX_NSYMP] = {1, -1, 0};   /* 1/16 semitone (6 cents): the strings beat a little */

/* a project or user preset saved before 1.0 (MODEL 2 was DUST): its E values as today's */
static void phys_legacy(int16_t *e)
{
    if (e[0] == 2) {                                     /* DUST -> MODAL bowed (random hits into resonators) */
        e[0] = PM_MODAL;
        e[6] = 64;
    }
}

static void phys_rg_note_on(track_t *t, voice_t *v);
static phys_slot_t *phys_slot_of(track_t *t, voice_t *v)
{
    uint32_t i;
    if (t < &trk[0] || t >= &trk[NPART])
        return 0;
    i = (uint32_t)(v - t->v);
    return i < PHYS_POLY ? (phys_slot_t *)ENG_MEM(t - trk) + i : 0;
}

/* a clean state for model md */
#if FELUCCA_CPU2
_Static_assert(NTRK == 4, "phys_nz: a seed per part");
static int32_t phys_nz[NTRK] = {0x6A09E667, 0x6A09E667, 0x6A09E667, 0x6A09E667};   /* phys_render's reset seeds */
#define PHYS_RSEED(t) noise32(&phys_nz[(uint32_t)((t) - trk) % NTRK])   /* the part's own (not rng(), shared) */
#else
#define PHYS_RSEED(t) rng()                             /* (one core: as up to 1.6) */
#endif
static void phys_reset(phys_slot_t *S, uint32_t md, uint32_t seed)
{
    uint32_t i, *w = (uint32_t *)&S->u;
    for (i = 0; i < sizeof S->u / 4u; i++)
        w[i] = 0;
    S->model = (uint8_t)md;
    seed |= 1u;
    switch (md) {
    case PM_STRING:
    case PM_SYMP:                                        /* (the main string is the union's px_string_t) */
        S->u.s.rng = seed;
        S->u.s.src = 1u << 30;
        break;
    default:
        S->u.m.rng = seed;
        break;
    }
}

static void phys_note_on(track_t *t, voice_t *v)
{
    phys_slot_t *S = phys_slot_of(t, v);
    uint32_t md = (uint32_t)t->p[P_E0] % PM_COUNT;
    if (md >= PM_RINGS) {
        phys_rg_note_on(t, v);
        return;
    }
    if (!S)
        return;
    if ((!v->env && !v->env_out) || S->model != md)     /* a fresh voice (not a retrigger): from rest */
        phys_reset(S, md, rng());
    switch (md) {
    case PM_MODAL:
    case PM_MEMB:
        S->u.m.trig = 1;                                 /* a retrigger strikes the ringing body again */
        break;
    default:                                             /* STRNG, SYMP */
        S->u.s.trig = 1;
        break;
    }
}

/* SYMP: the sympathetic strings' pitches (cycles Q32) by CHRD: a chord above the note, or ROOT */
static __attribute__((noinline)) void phys_symp_pitches(const track_t *t, const vmod_t *m, uint32_t *f)
{
    uint32_t c = (uint32_t)t->p[P_E1] >> 4, k;          /* 0..7 */
    for (k = 0; k < PX_NSYMP; k++) {
        int32_t p16 = PHYS_DETUNE[k];
        p16 += c < 7u ? m->pitch16 + ((PHYS_CHORDS[c][k] * 41) >> 8)   /* cents -> 1/16 semitones (0.16) */
                      : (48 + t->p[P_ROOT] + (k == 1u ? 7 : k == 2u ? 12 : 0)) * 16;
        f[k] = pitch_inc(clamp(p16, 0, 2047));
    }
}


/* ------------------------------------------------------ Rings' models --- */
/* PM_R*: Rings' resonators and string synth (rings_dsp.c), whole. The state (about 32 KB) is one arena for the whole
 * device, owned by one part at a time (the last to play a Rings model; its eng_mem is not used): a part plays them
 * with the voice that last started a note as the carrier of the whole sound (the other voices of the part render
 * nothing), because Rings' voices are its own: a strum goes to the next of POLY (1, 2, 4) voices of the model, the
 * earlier ones ring on. The carrier's ADSR and velocity shape the sum. */
typedef struct {
    uint8_t owner;               /* part + 1, 0 = nobody */
    uint8_t model;               /* PM_* */
    uint8_t next;                /* the voice the next strum goes to (round robin) */
    uint8_t strum;               /* bit k: voice k is struck in the next block (the string synth: bit 0) */
    uint8_t vset;                /* bit k: voice k has had a note */
    uint8_t last;                /* the voice of the latest strum */
    const voice_t *car;          /* the carrier */
    int32_t vp16[RG_MAXV];       /* the voices' pitch, 1/16 semitone */
    int32_t tonic;               /* the sympathetic strings' reference note, MIDI Q16, -1 = not set */
    rg_lim_t lim;
    union {
        rg_modset_t m;
        rg_strset_t s;
        rg_fmset_t f;
        rg_dpset_t d;
    } u;
} phys_rg_t;
static phys_rg_t phys_rg __attribute__((section(".pool")));
_Static_assert(sizeof(phys_rg_t) <= 32768u + 256u, "Rings' arena over 32 KiB: the pool's headroom (tools/build.py)");

static void phys_rg_reset(uint32_t part, uint32_t md)
{
    uint32_t i, *w = (uint32_t *)&phys_rg, seed = (uint32_t)rng();
    for (i = 0; i < sizeof phys_rg / 4u; i++)
        w[i] = 0;
    phys_rg.owner = (uint8_t)(part + 1u);
    phys_rg.model = (uint8_t)md;
    phys_rg.tonic = -1;
    phys_rg.lim.peak = RG_ONE / 2;
    switch (md - PM_RINGS) {
    case RG_MODAL:
    case RG_FM:
        break;
    case RG_DP:
        rg_dp_init(&phys_rg.u.d);
        break;
    default:
        if (md - PM_RINGS == RG_STRREV)
            rg_rev_init(&phys_rg.u.s.rev, RG_RVBUF(&phys_rg.u.s));
        for (i = 0; i < RG_MAXV; i++) {
            phys_rg.u.s.pl[i].rng = 0x2545F491u * (seed * 7u + i + 1u) | 1u;
            phys_rg.u.s.s[i].rng = 0x9E3779B9u * (seed * 5u + i + 3u) | 1u;
        }
        break;
    }
}

static uint32_t phys_rg_poly(const track_t *t)
{
    int32_t k = t->p[P_E5];
    return k <= 0 ? 1u : k == 1 ? 2u : 4u;
}

static void phys_rg_note_on(track_t *t, voice_t *v)
{
    uint32_t part = (uint32_t)(t - trk), md = (uint32_t)t->p[P_E0] % PM_COUNT, k, busy = 0, poly = phys_rg_poly(t);
    phys_rg_t *A = &phys_rg;
    if (part >= NPART)
        return;
    for (k = 0; k < NVOICE; k++)
        busy += &t->v[k] != v && t->v[k].active;
    if (A->owner != part + 1u || A->model != md || (!busy && !v->env && !v->env_out))   /* another part's or a silent start: clean */
        phys_rg_reset(part, md);
    A->car = v;
    if (md == PM_RDISP) {
        A->strum |= 1u;
        A->vp16[0] = v->pitch16;
        A->last = 0;
        return;
    }
    A->next = (uint8_t)((A->next + 1u) % poly);
    k = A->next;
    A->strum |= (uint8_t)(1u << k);
    A->vset |= (uint8_t)(1u << k);
    A->vp16[k] = v->pitch16;
    A->last = (uint8_t)k;
    if (A->tonic < 0)
        A->tonic = (int32_t)(t->p[P_E6] > 0 && md == PM_RSYMP ? t->p[P_E6] * 65536 : v->pitch16 * 4096 - 5 * 65536);
}

static void phys_rg_render(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m)
{
    const int16_t *p = t->p;
    phys_rg_t *A = &phys_rg;
    uint32_t part = (uint32_t)(t - trk), md = (uint32_t)p[P_E0] % PM_COUNT, poly = phys_rg_poly(t), i, k, rm = md - PM_RINGS;
    int32_t acc[CTL] = {0}, g = PHYS_GAIN[md], tune = p[P_E7] * 16;
    rg_patch_t P;
    if (part >= NPART || A->car != v || A->owner != part + 1u)   /* (another part took the arena: this one is silent; only a note on takes it) */
        return;
    if (A->model != md)                                  /* MODEL changed under a sounding note */
        phys_rg_reset(part, md);
    if (n > CTL)
        n = CTL;
    P.structure = clamp(p[P_E1] * 516, 0, 65536);
    P.brightness = clamp(p[P_E2] * 516, 0, 65536);
    P.damping = clamp(p[P_E3] * 516, 0, 65536);
    P.position = clamp(p[P_E4] * 516, 0, 65536);
    P.poly = (int32_t)poly;
    P.chord = p[P_E6] > 10 ? 10 : p[P_E6] < 0 ? 0 : p[P_E6];
    P.fx = p[P_E7] > 5 ? 5 : p[P_E7] < 0 ? 0 : p[P_E7];
    P.tonic = A->tonic < 0 ? 0 : A->tonic;
    if (md == PM_RDISP) {
        int32_t mono[CTL] = {0};
        P.f = 0;
        P.voice = 0;
        P.note = m->pitch16 * 4096;
        rg_dp_run(&A->u.d, &P, (A->strum & 1u) != 0, mono, n);
        A->strum = 0;
        for (i = 0; i < n; i++)
            acc[i] = mono[i];
    } else {
        if (A->vset >> A->last & 1u)
            A->vp16[A->last] = m->pitch16;               /* the latest voice follows the bend and the glide */
        for (k = 0; k < poly; k++) {
            int32_t y[CTL] = {0}, p16 = A->vp16[k] + tune;
            int st = A->strum >> k & 1u;
            if (!(A->vset >> k & 1u))
                continue;
            P.f = pitch_inc((uint32_t)clamp(p16, 0, 2047));
            P.voice = (int32_t)k;
            P.note = p16 * 4096;
            switch (rm) {
            case RG_MODAL: {
                rg_modal_blk_t B;
                rg_modal_block(&B, &A->u.m.m[k], &P, st);
                rg_modal_run(&B, &A->u.m.m[k], y, n);
                break;
            }
            case RG_FM:
                rg_fmv_run(&A->u.f, &P, st, y, n);
                break;
            default:
                rg_strv_run(&A->u.s, &P, (int)rm, st, y, n);
                break;
            }
            for (i = 0; i < n; i++)
                acc[i] += y[i];
        }
        A->strum = 0;
        if (rm == RG_MODAL || rm == RG_FM)
            rg_lim_run(&A->lim, acc, n, RG_GAIN[rm]);
        else
            rg_post(&A->u.s, &A->lim, (int)rm, &P, acc, n);
    }
    for (i = 0; i < n; i++) {
        int32_t s = px_m(acc[i], g, 20), a = s < 0 ? -s : s;
        if (a > 24000) {                                 /* soft knee */
            a = 24000 + (softclip(clamp(a - 24000, 0, 1 << 20) * 2) >> 1);
            s = s < 0 ? -a : a;
        }
        out[i] += voice_amp(s, m, i) << 1;
    }
}

static void phys_render(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m)
{
    const int16_t *p = t->p;
    phys_slot_t *S = phys_slot_of(t, v);
    uint32_t md = (uint32_t)p[P_E0] % PM_COUNT, i, f0 = m->inc;
    if (md >= PM_RINGS) {
        phys_rg_render(t, v, out, n, m);
        return;
    }
    int32_t y[CTL], ax[CTL];
    int32_t strc = p[P_E1] * 516, brit = clamp(p[P_E2] * 516 + m->cutoff * 2, 0, 65536);   /* Q16 0..1 */
    int32_t damp = p[P_E3] * 516, pos = p[P_E4], exc = p[P_E7] * 258;   /* exc Q15 */
    int32_t bow = p[P_E6] * 206;                         /* BOW: Dust at up to 0.4 (DaisySP's sustain: 1) */
    int32_t acc = clamp(p[P_E5] * v->vel * 4, 0, 65536);   /* ACC at full velocity, Q16 */
    int32_t g = PHYS_GAIN[md];
    if (!S)
        return;
    if (n > CTL)
        n = CTL;
    if (S->model != md)                                  /* MODEL changed under a sounding note */
        phys_reset(S, md, PHYS_RSEED(t));
    switch (md) {
    case PM_MODAL: {
        px_modal_blk_t B;
        px_modal_block(&B, &S->u.m, f0, strc, brit, damp, acc, pos * 129, bow);   /* pos 0..1/4 */
        px_modal_run(&B, &S->u.m, y, ax, n);
        break;
    }
    case PM_MEMB: {
        px_modal_blk_t B;
        px_memb_block(&B, &S->u.m, f0, strc, brit, damp, acc, pos * 516, p[P_E6] * 6192);   /* BEND 0..12 st */
        px_modal_run(&B, &S->u.m, y, ax, n);
        break;
    }
    case PM_STRING: {
        px_string_blk_t B;
        px_string_block(&B, &S->u.s, f0, strc, brit, damp, acc, bow, pos * 516);   /* 0..1/2 period */
        px_string_excite(&B, &S->u.s, ax, n);
        px_string_run(&B, &S->u.s, ax, y, n);
        break;
    }
    default: {                                           /* SYMP */
        px_string_blk_t B;
        uint32_t f[PX_NSYMP];
        phys_symp_pitches(t, m, f);
        px_string_block(&B, &S->u.y.s, f0, 16384 - p[P_E6] * 129, brit, damp, acc, 0, 0);   /* BUZZ: STRC 0.25 .. 0 */
        px_string_excite(&B, &S->u.y.s, ax, n);
        px_string_run(&B, &S->u.y.s, ax, y, n);
        px_symp_block(&S->u.y, f, brit, damp, pos * 516);
        px_symp_run(&S->u.y, y, y, n);
        break;
    }
    }
    for (i = 0; i < n; i++) {
        int32_t s = px_m(y[i] + px_m(ax[i], exc, 15), g, 20), a = s < 0 ? -s : s;
        if (a > 24000) {                                 /* soft knee: only peaks saturate */
            a = 24000 + (softclip(clamp(a - 24000, 0, 1 << 20) * 2) >> 1);
            s = s < 0 ? -a : a;
        }
        out[i] += voice_amp(s, m, i) << 1;
    }
}

/* MODAL partials by STRC: 34 harmonic, 70 1 : 2.83 : 5.48 : 8.90 (a free bar, a chime: 1 : 2.76 : 5.40 : 8.93),
 * 98 1 : 3.57 : 7.65, 116 1 : 4.0 : 9.0 (a tuned bar: 1 : 4 : 10) */
static const preset_t PHYS_PRESETS[] = {
    /* name, {MODEL, STRC, BRIT, DAMP, POS, ACC, BOW, EXC}, {A D S R}, fenv, mono */
    {"BELL TREE", {PM_MODAL, 70, 100, 106, 12, 100, 0, 6}, {0, 100, 127, 96}, 0, 0, FX(0, 20, 30, 60), PAT(7)},
    {"MARIMBA", {PM_MODAL, 116, 38, 40, 24, 90, 0, 30}, {0, 90, 127, 52}, 0, 0, FX(0, 0, 15, 30), PAT(6)},
    {"PLUCK", {PM_STRING, 56, 72, 84, 40, 96, 0, 0}, {0, 100, 127, 58}, 0, 0, FX(0, 10, 30, 30), PAT(3)},
    {"BOWED METAL", {PM_MODAL, 98, 70, 112, 10, 80, 90, 0}, {60, 90, 127, 90}, 0, 0, FX(0, 40, 20, 70), PAT(5)},
    {"KALIMBA", {PM_MODAL, 34, 16, 56, 100, 90, 0, 48}, {0, 100, 127, 60}, 0, 0, FX(0, 10, 20, 40), PAT(7)},
    /* MEMB: {MODEL, HARM, BRIT, DAMP, POS, ACC, BEND, EXC} */
    {"HAND DRUM", {PM_MEMB, 112, 76, 56, 44, 100, 14, 24}, {0, 100, 127, 64}, 0, 0, FX(0, 0, 10, 30), PAT(6)},
    {"TOMS", {PM_MEMB, 0, 52, 40, 30, 100, 40, 30}, {0, 100, 127, 70}, 0, 0, FX(0, 0, 0, 40), PAT(2)},
    /* SYMP: {MODEL, CHRD, BRIT, DAMP, SYMP, ACC, BUZZ, EXC} */
    {"DRONE STRING", {PM_SYMP, 127, 80, 88, 64, 100, 92, 0}, {0, 100, 127, 90}, 0, 0, FX(0, 10, 20, 50), PAT(3)},
    {"HARP", {PM_SYMP, 8, 62, 74, 60, 90, 0, 0}, {0, 100, 127, 80}, 0, 0, FX(0, 20, 20, 60), PAT(3)},
    /* Rings' models: {MODEL, STRU, BRIG, DAMP, POSN, POLY (0 1, 1 2, 2 4), CHRD / TONC, TUNE / FX} */
    {"R MODAL", {PM_RMODAL, 64, 70, 80, 40, 0, 0, 0}, {0, 100, 127, 90}, 0, 0, FX(0, 10, 20, 50), PAT(3)},
    {"R SYMPATHETIC", {PM_RSYMP, 70, 70, 70, 60, 0, 0, 0}, {0, 100, 127, 90}, 0, 0, FX(0, 10, 20, 50), PAT(3)},
    {"R STRING", {PM_RSTRING, 40, 70, 80, 50, 1, 0, 0}, {0, 100, 127, 80}, 0, 0, FX(0, 10, 20, 40), PAT(3)},
    {"R FM VOICE", {PM_RFM, 64, 70, 60, 40, 0, 0, 0}, {0, 100, 127, 70}, 0, 0, FX(0, 10, 20, 40), PAT(3)},
    {"R QUANTISED", {PM_RQUANT, 64, 70, 70, 60, 1, 4, 0}, {0, 100, 127, 90}, 0, 0, FX(0, 10, 20, 50), PAT(3)},
    {"R STR+REVERB", {PM_RSTRREV, 40, 70, 70, 50, 0, 0, 0}, {0, 100, 127, 90}, 0, 0, FX(0, 0, 10, 40), PAT(3)},
    {"R DISASTROUS", {PM_RDISP, 0, 60, 60, 50, 0, 4, 0}, {0, 100, 127, 100}, 0, 0, FX(0, 0, 10, 40), PAT(3)},
};

static const engine_t ENG_PHYS = {
    .name = "PHYS",
    .page_title = {"BODY", "EXCT"},
    .edit = {
        {"MODEL", F_ENUM, 0, PM_COUNT - 1, 0, N_PHYS_MODEL, 0},
        {"STRC", F_PCT, 0, 127, 64, 0, 0},
        {"BRIT", F_PCT, 0, 127, 80, 0, 0},
        {"DAMP", F_PCT, 0, 127, 80, 0, 0},
        {"POS", F_PCT, 0, 127, 20, 0, 0},
        {"ACC", F_PCT, 0, 127, 90, 0, 0},
        {"BOW", F_PCT, 0, 127, 0, 0, 0},
        {"EXC", F_PCT, 0, 127, 0, 0, 0},
    },
    .presets = PHYS_PRESETS,
    .npresets = NELEM(PHYS_PRESETS),
    .note_on = phys_note_on,
    .render = phys_render,
    .knob = {P_E1, P_E2, P_E3, P_E4},
    .poly = PHYS_POLY,
    .desc = phys_desc,
    .mem = sizeof(phys_slot_t) * PHYS_POLY,
};
