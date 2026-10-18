/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* CLOUDS (RingsClouds): Clouds (Emilie Gillet, MIT; clouds_dsp.c, in fixed point) as an effect after the PHYS parts,
 * Rings -> Clouds as on a rack. One for the whole instrument, as the buses: its knobs are song.cp (core.h CP_*, the
 * project's: project.c PROJ_CLOUDS_OFF), on the FX pages CLOUDS (MODE POSN SIZE PTCH), CLOUDS 2 (DENS TEXT FRZ QUAL),
 * BLEND (WET SPRD FDBK REV: Clouds' BLEND knob's four), CLOUDS 3 (SRC TRIG).
 *
 * What goes through (SRC): the PHYS parts (PHYS, the default), one track (T1 .. T4) or all (ALL): a part that goes through
 * adds its dry (after its DIST, INSERT, SLICER, LEVEL, PAN) into the bus (fx.c mix_part: cap) instead of the dry mix;
 * its sends to the chorus, delay and reverb are as always (before Clouds). Clouds' output, its own BLEND of dry and
 * wet, joins the dry mix. MODE OFF: nothing of this runs, the mix bit for bit as without it.
 *
 * The memory: the DELAY bus's line (fx.c dly_buf, 128 KiB) while MODE is not OFF (PORTING_NOTES.md: the delay and
 * looper are Clouds' then; the DELAY bus is silent, its send kept for when CLOUDS is off): this file's blocks first
 * (cld_head_t), then the processor (clouds_dsp.c: its state, lines, about 1 s of stereo at 22.05 kHz). MODE OFF -> on:
 * the delay's output fades out over a block, the line is Clouds' from the next; on -> OFF: the bus fades out, the line
 * is cleared over the next blocks (8 KiB a block) and the delay starts again from silence.
 *
 * The timing: Clouds processes the block before (the bus is one block, 0.73 ms, late), and so does the dry mix while the
 * bus is on (cld_head_t.dry): what goes through and what does not stay together, a part can change sides (SRC, an
 * engine change) without a step. That lets a second core run Clouds while this one renders the parts (fx.c mix_block,
 * mix_split: CPU2): the output is the same on one core or two. Turning the bus on or off crossfades the late mix with
 * the mix of the block over one block (32 samples).
 *
 * The rate: 44.1 kHz outside, 22.05 kHz inside (clouds_dsp.c: every time in seconds as Clouds', 11 kHz of bandwidth).
 * The load: the grains that may start at once (cl_t.gcap) follow the audio ISR's load (song.cpu_q8): fewer above 80 %,
 * back up below 70 %. SPEC with one core processes a mono sum (QUAL's ST -> MO; PORTING_NOTES.md: the cost). */
#include "clouds_dsp.c"

#if FELUCCA_CPU2 && !defined(C1_ON)
#define C1_ON() 0                                        /* (as fx.c's: no second core answering) */
#endif

#define CLD_HEAD 1024u                                   /* the glue's blocks at the arena's start */
typedef struct {
    int32_t cap[2][2][CTL];                              /* what goes through, this block: [core][left, right] */
    int32_t in[2][CTL];                                  /* .. the block before (Clouds' input, then its output) */
    int32_t dry[2][CTL];                                 /* the dry mix of the block before (the bus on: one block late) */
} cld_head_t;
_Static_assert(sizeof(cld_head_t) <= CLD_HEAD, "CLOUDS: the glue's blocks");
_Static_assert(sizeof(dly_buf) >= 131072u, "CLOUDS: the delay's line is its arena");

enum { CLD_OFF, CLD_TAKE, CLD_ON, CLD_GIVE };            /* dly_buf: the delay's; CLOUDS' from the next block; CLOUDS';
                                                          * being cleared for the delay */
enum { BUS_OFF, BUS_ARM, BUS_IN, BUS_ON, BUS_OUT };      /* the bus: off; storing its first block; fading in; on; out */
static struct {
    cl_t *c;
    cld_head_t *h;
    uint8_t own, bus;
    uint8_t route;                                       /* the parts going through this block (bit k: part k) */
    uint8_t c1;                                          /* Clouds runs on the second core this block */
    uint8_t ran;                                         /* .. it has run this block */
    uint8_t gcap;
    uint16_t gtick;
    uint32_t clr;                                        /* CLD_GIVE: bytes of dly_buf cleared */
    cl_par_t par;
} cld;

#define CLD_ARENA ((uint8_t *)dly_buf)

/* the parts that go through, by SRC */
static uint32_t cld_route(void)
{
    uint32_t src = (uint32_t)song.cp[CP_SRC], m = 0, i;
    if (src >= 1u && src <= NPART)
        return 1u << (src - 1u);
    for (i = 0; i < NPART; i++)
        if (src > NPART || trk[i].engine == ENGI_PHYS)
            m |= 1u << i;
    return m;
}

static inline int32_t cld_q16(int32_t v) { return (clamp(v, 0, 127) * 65536 + 63) / 127; }

/* RingsClouds: a knob with the matrix's offset (mod.c mod_pre: sources onto CPOS .. CREV), clamped to its range */
static inline int32_t cld_knob(uint32_t k)
{
    return clamp(song.cp[k] + mod_cld[k], CPD[k].min, CPD[k].max);
}
/* FREEZE besides the FRZ knob: the FREEZE key held (perform.c), MIDI CC FREEZE, a track's CSEQ FRZ / BOTH (the step's lock, or
 * the track's own), the matrix's CFRZ gate */
static uint32_t cld_extra_freeze(void)
{
    uint32_t i, f = cl_fx_frz | cl_cc_frz | mod_cfrz;
    for (i = 0; i < NTRK; i++)
        f |= (uint32_t)(trk[i].p[P_CSEQ] >> 1) & 1u;
    return f != 0;
}
/* TRIG besides a note: the TRIG key, MIDI CC TRIG, a step with CSEQ TRIG / BOTH, the matrix's CTRG gate (one block's pulse
 * each, cleared here) */
static uint32_t cld_extra_trig(void)
{
    uint32_t f = cl_fx_trg | cl_cc_trg | cl_seq_trg | mod_ctrg;
    cl_fx_trg = cl_cc_trg = cl_seq_trg = 0;
    return f != 0;
}

/* the knobs for Clouds (the main loop writes song.cp: read once a block, here) */
static void cld_params(void)
{
    cl_par_t *p = &cld.par;
    uint32_t trig = song.cp[CP_TRIG] == 1;
    p->position = cld_q16(cld_knob(CP_POS));
    p->size = cld_q16(cld_knob(CP_SIZE));
    p->pitch = clamp(cld_knob(CP_PTCH), -24, 24) << 16;
    p->density = cld_q16(cld_knob(CP_DENS));
    p->texture = cld_q16(cld_knob(CP_TEX));
    p->dry_wet = cld_q16(cld_knob(CP_WET));
    p->spread = cld_q16(cld_knob(CP_SPRD));
    p->feedback = cld_q16(cld_knob(CP_FDBK));
    p->reverb = cld_q16(cld_knob(CP_REV));
    p->freeze = song.cp[CP_FRZ] != 0 || cld_extra_freeze();
    p->trigger = (trig && (cl_noteon & cld.route)) || cld_extra_trig();
    p->gate = 0;
    if (trig) {                                          /* the gate: a key of a part going through held */
        uint32_t i, k;
        for (i = 0; i < NPART; i++)
            if (cld.route >> i & 1u)
                for (k = 0; k < NVOICE; k++)
                    p->gate |= trk[i].v[k].active && trk[i].v[k].gate;
    }
    cl_noteon = 0;
}

/* the grains allowed at once, from the ISR's load (Q8 of a half's time): down 2 a step above 80 %, up 1 below 70 % */
static void cld_load(void)
{
    uint32_t l = song.cpu_q8;
    if (++cld.gtick < 16u)
        return;
    cld.gtick = 0;
    if (l > 205u && cld.gcap > 8u)
        cld.gcap = (uint8_t)(cld.gcap - 2u);
    else if (l < 179u && cld.gcap < CL_MAXG)
        cld.gcap++;
    cld.c->gcap = cld.gcap;
}

static void cld_config(void)
{
    uint32_t mode = (uint32_t)clamp(song.cp[CP_MODE], 1, 4) - 1u, q = (uint32_t)clamp(song.cp[CP_QUAL], 0, 3);
#if FELUCCA_CPU2
    if (mode == CL_SPECTRAL && !C1_ON())                 /* SPEC on one core: mono (its cost, PORTING_NOTES.md) */
        q |= 1u;
#else
    if (mode == CL_SPECTRAL)
        q |= 1u;
#endif
    cl_config(cld.c, mode, q);
}

/* the start of a block (fx.c mix_block, before the parts): who has the line, the bus's state, the parts that go
 * through, the knobs. 1 = Clouds runs this block (on whichever core) */
static int clouds_pre(uint32_t n)
{
    int on = song.cp[CP_MODE] > 0;
    (void)n;
    cld.ran = 0;
    switch (cld.own) {
    case CLD_OFF:
        if (on)
            cld.own = CLD_TAKE;                          /* (fx_buses fades the delay out this block) */
        break;
    case CLD_TAKE:
        cld.h = (cld_head_t *)CLD_ARENA;
        memset(cld.h, 0, sizeof *cld.h);
        cld.c = cl_new(CLD_ARENA + CLD_HEAD, sizeof dly_buf - CLD_HEAD, FS / 2, 1, CTL);
        cld.gcap = CL_MAXG;
        cld.gtick = 0;
        cld.own = CLD_ON;
        cld.bus = BUS_ARM;
        break;
    case CLD_ON:
        if (!on && cld.bus == BUS_ON)
            cld.bus = BUS_OUT;
        else if (!on && cld.bus != BUS_OUT) {            /* (off again before the fade in: straight out) */
            cld.bus = BUS_OFF;
            cld.own = CLD_GIVE;
            cld.clr = 0;
        } else if (cld.bus == BUS_ARM)
            cld.bus = BUS_IN;
        else if (cld.bus == BUS_IN)
            cld.bus = BUS_ON;
        else if (cld.bus == BUS_OUT) {
            cld.bus = BUS_OFF;
            cld.own = CLD_GIVE;
            cld.clr = 0;
        }
        break;
    default:                                             /* CLD_GIVE: the line back to the delay, cleared */
        {
            uint32_t k = sizeof dly_buf - cld.clr < 8192u ? (uint32_t)sizeof dly_buf - cld.clr : 8192u;
            memset(CLD_ARENA + cld.clr, 0, k);
            cld.clr += k;
            if (cld.clr >= sizeof dly_buf) {
                fx.dly_lp = 0;
                cld.own = CLD_OFF;
                cld.c = 0;
                cld.h = 0;
            }
        }
        break;
    }
    cld.route = cld.bus != BUS_OFF ? (uint8_t)cld_route() : 0u;
    if (cld.bus == BUS_OFF) {
        cl_fx_trg = cl_cc_trg = cl_seq_trg = 0;
        return 0;
    }
    memset(cld.h->cap, 0, sizeof cld.h->cap);
    if (cld.bus != BUS_ARM) {
        cld_params();
        cld_config();
        cld_load();
        return 1;
    }
    cl_noteon = 0;
    cl_fx_trg = cl_cc_trg = cl_seq_trg = 0;              /* (RingsClouds: pulses for a bus that is not running are lost) */
    return 0;
}

/* Clouds on the block before (in place); the second core's job or this one's (fx.c) */
static void clouds_run(uint32_t n)
{
    cld_head_t *h = cld.h;
    cl_process(cld.c, &cld.par, h->in[0], h->in[1], h->in[0], h->in[1], n);
    cld.ran = 1;
}

/* the end of the parts (fx.c mix_block, the second core's sums added): the dry mix (mix_l, mix_r: the parts that do not
 * go through) and the bus (cap: those that do) -> the dry mix of this block's output, as the bus's state has it */
static void clouds_post(int32_t *ml, int32_t *mr, uint32_t n)
{
    cld_head_t *h = cld.h;
    const int32_t *cl = h->cap[0][0], *cr = h->cap[0][1];
    uint32_t i;
    if (cld.bus == BUS_OFF)
        return;
    if (cld.bus != BUS_ARM && !cld.ran)
        clouds_run(n);
    for (i = 0; i < n; i++) {
        int32_t dl = ml[i], dr = mr[i];                  /* this block's: the dry and what goes through */
        int32_t ul = dl + cl[i], ur = dr + cr[i];        /* .. undelayed, as without the bus */
        int32_t yl, yr;
        if (cld.bus == BUS_ARM) {
            yl = ul;
            yr = ur;
        } else {
            int32_t ll = h->dry[0][i] + h->in[0][i], lr = h->dry[1][i] + h->in[1][i];   /* late: the dry and Clouds */
            if (cld.bus == BUS_ON) {
                yl = ll;
                yr = lr;
            } else {                                     /* the crossfade (IN: to the late mix; OUT: from it) */
                int32_t g = (int32_t)(((i + 1u) << 15) / n);
                if (cld.bus == BUS_OUT)
                    g = 32768 - g;
                yl = ul + mulq15(ll - ul, g);
                yr = ur + mulq15(lr - ur, g);
            }
        }
        h->dry[0][i] = dl;
        h->dry[1][i] = dr;
        h->in[0][i] = cl[i];
        h->in[1][i] = cr[i];
        ml[i] = yl;
        mr[i] = yr;
    }
}

/* the delay bus (fx.c fx_buses): 0 runs as always, 1 fades out over this block (CLD_TAKE), 2 silent (the line is Clouds') */
static inline uint32_t cld_delay(void) { return cld.own == CLD_OFF ? 0u : cld.own == CLD_TAKE ? 1u : 2u; }

/* the cost of Clouds for the second core's split (fx.c mix_split, ENG_VCOST's units), 0 = not running */
static uint32_t cld_cost(void)
{
    static const uint16_t COST[4] = {1300, 1800, 1100, 2400};   /* GRAN STRT LOOP SPEC (tests/clouds_cpu.sh) */
    if (cld.bus == BUS_OFF || cld.bus == BUS_ARM)
        return 0;
    return COST[(uint32_t)clamp(song.cp[CP_MODE], 1, 4) - 1u];
}
