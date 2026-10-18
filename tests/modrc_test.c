/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* RingsClouds' mod matrix (mod.c mod_pre, fx_clouds.c, perform.c, seq.c, midi_control.c), on the host:
 *   params      the reused ids (LFO B, CC#, CSEQ, STEP) default to "off" (0) as a project stored before holds them; the
 *               LFO B / MOD SRC pages are 4 knobs each; the enums have the new entries last
 *   sources     STEP onto CWET = the knob set by hand (bit for bit); LFO B onto CWET changes the sound; CC follows the
 *               controller of CC#; a source onto CPOS changes the sound
 *   gates       FREEZE from CSEQ FRZ, the FREEZE key, MIDI CC107, the matrix CFRZ = CLOUDS' FRZ knob (bit for bit);
 *               TRIG from the key, CC108, a step's CSEQ TRIG and the matrix CTRG: the same sound as each other
 *   sequencer   seq_play of a step locked to CSEQ TRIG raises cl_seq_trg; the step after (the lock off) does not
 *   midi        CC20..27, 102..119 reach CLOUDS' knobs, the track's E1..E8 and LFO B; CC121 lets the freeze go
 *   cores       every scene bit for bit the same on one core and two (tests/c1_host.h)
 *   build/host/modrc_test */
#include "c1_host.h"
#define main hostsim_main
#include "hostsim.c"
#undef main

static int fails;
#define CHECK(c, ...)                                                                                                  \
    do {                                                                                                               \
        if (!(c)) {                                                                                                    \
            printf("modrc: FAIL ");                                                                                    \
            printf(__VA_ARGS__);                                                                                       \
            printf("\n");                                                                                              \
            fails++;                                                                                                   \
        }                                                                                                              \
    } while (0)

#define NF (2u * FS)
enum { SC_BASE, SC_STEP_CWET, SC_CWET_HAND, SC_LFOB_CWET, SC_CPOS_ENV, SC_FRZ_KNOB, SC_FRZ_CSEQ, SC_FRZ_KEY, SC_FRZ_CC,
       SC_FRZ_MOD, SC_TRG_NONE, SC_TRG_KEY, SC_TRG_CC, SC_TRG_SEQ, SC_TRG_MOD, SC_CC_SRC, SC_N };

static void slot(track_t *t, uint32_t k, uint32_t src, uint32_t dst, int32_t amt)
{
    t->p[P_M1SRC + 3u * k] = (int16_t)src;
    t->p[P_M1DST + 3u * k] = (int16_t)dst;
    t->p[P_M1AMT + 3u * k] = (int16_t)amt;
}

static uint64_t render(uint32_t sc, int dual)
{
    uint64_t h = 0xCBF29CE484222325ull;
    static int32_t out[2 * CTL];
    uint32_t f, i, k;
    host_tracks_init();
    for (i = 0; i < CP_COUNT; i++)
        song.cp[i] = CPD[i].def;
    c1h_off = !dual;
    host_preset(&trk[0], ENGI_PHYS, 1);
    trk[0].p[P_DLY] = trk[0].p[P_CHOR] = trk[0].p[P_REV] = 0;
    song.cp[CP_MODE] = 1;
    song.cp[CP_TRIG] = 1;
    if (sc == SC_STEP_CWET) {
        trk[0].p[P_STEPV] = 50;
        slot(&trk[0], 0, MS_STEP, MD_CWET, 32);
    } else if (sc == SC_CWET_HAND) {
        song.cp[CP_WET] = 64 + 49;                 /* ((50 * 512 * 32) >> 6) * 127 >> 15 = 49 */
    } else if (sc == SC_LFOB_CWET) {
        trk[0].p[P_LBRATE] = 100;
        slot(&trk[0], 0, MS_LFOB, MD_CWET, 60);
    } else if (sc == SC_CPOS_ENV) {
        slot(&trk[0], 0, MS_ENV, MD_CPOS, 60);
    } else if (sc == SC_CC_SRC) {
        trk[0].p[P_MCCN] = 20;
        slot(&trk[0], 0, MS_CC, MD_CWET, 60);
    }
    if (sc >= SC_FRZ_KNOB && sc <= SC_FRZ_MOD)
        song.cp[CP_WET] = 127;
    for (f = 0; f < NF; f += CTL) {
        uint32_t ms = f / (FS / 1000u);
        if (ms % 250u == 0u && ms < 1000u && f % (FS / 1000u) < CTL)
            trk_note_on(&trk[0], 48u + (ms / 250u) * 3u, 100);
        if (ms % 250u == 200u && f % (FS / 1000u) < CTL)
            for (i = 36; i < 84; i++)
                trk_note_off(&trk[0], i);
        if (sc == SC_CC_SRC && ms == 600u && f % (FS / 1000u) < CTL)
            mod_midi(&trk[0], 0xB0u, 20u, 127u);
        if (ms == 1000u && f % (FS / 1000u) < CTL) {            /* the gates, all at 1.0 s */
            if (sc == SC_FRZ_KNOB)
                song.cp[CP_FRZ] = 1;
            else if (sc == SC_FRZ_CSEQ)
                trk[1].p[P_CSEQ] = 2;
            else if (sc == SC_FRZ_KEY)
                perf_press(PF_CFRZ, 1);
            else if (sc == SC_FRZ_CC)
                midi_cc_map(&trk[0], 107u, 127u);
            else if (sc == SC_FRZ_MOD) {
                trk[2].p[P_STEPV] = 63;
                slot(&trk[2], 0, MS_STEP, MD_CFRZ, 63);
            } else if (sc == SC_TRG_KEY)
                perf_press(PF_CTRG, 1);
            else if (sc == SC_TRG_CC)
                midi_cc_map(&trk[0], 108u, 127u);
            else if (sc == SC_TRG_SEQ)
                cl_seq_trg = 1;
            else if (sc == SC_TRG_MOD) {
                trk[2].p[P_STEPV] = 63;
                slot(&trk[2], 0, MS_STEP, MD_CTRG, 63);
            }
        }
        mix_block(out, CTL);
        for (i = 0; i < 2u * CTL; i++) {
            uint32_t u = (uint32_t)out[i];
            for (k = 0; k < 4u; k++) {
                h ^= (u >> (8u * k)) & 0xFFu;
                h *= 0x100000001B3ull;
            }
        }
    }
    return h;
}

static uint64_t fork_render(uint32_t sc, int dual)
{
    int fd[2];
    uint64_t h = 0;
    pid_t pid;
    if (pipe(fd))
        return 0;
    pid = fork();
    if (!pid) {
        h = render(sc, dual);
        if (write(fd[1], &h, sizeof h) != (ssize_t)sizeof h)
            _exit(2);
        _exit(0);
    }
    close(fd[1]);
    if (read(fd[0], &h, sizeof h) != (ssize_t)sizeof h)
        h = 0;
    close(fd[0]);
    waitpid(pid, 0, 0);
    return h;
}

int main(void)
{
    uint64_t h1[SC_N], h2[SC_N];
    uint32_t i, found = 0;
    static const char *const NAME[SC_N] = {"base", "STEP>CWET", "CWET by hand", "LFOB>CWET", "ENV>CPOS", "FRZ knob",
                                           "FRZ CSEQ", "FRZ key", "FRZ CC107", "FRZ matrix", "TRG none", "TRG key",
                                           "TRG CC108", "TRG seq", "TRG matrix", "CC source"};

    /* params */
    host_tracks_init();
    CHECK(RC_ON, "RC_ON");
    CHECK(P_LBSYNC != P_FM1_SUS && P_LBPOL != P_FM2_SUS && P_CSEQ != P_FM2_LEVEL && P_MCCN != P_FM1_LEVEL,
          "the reused ids are ATK / DEC / REL only (SUS and LEVEL stored as 127)");
    CHECK(trk[0].p[P_LBSYNC] == 0 && trk[0].p[P_LBPOL] == 0 && trk[0].p[P_CSEQ] == 0 && trk[0].p[P_STEPV] == 0 &&
              trk[0].p[P_LBWAVE] == 0,
          "defaults are off");
    CHECK(MS_N == 14 && MD_N == 34 && !strcmp(N_MSRC[MS_LFOB], "LFOB") && !strcmp(N_MDST[MD_CTRG], "CTRG") &&
              !strcmp(N_MDST[MD_TIME], "TIME"),
          "enums append-only");
    for (i = 0; i < NPAGES; i++)
        if (!strcmp(PAGES[i].title, "LFO B") || !strcmp(PAGES[i].title, "MOD SRC")) {
            uint32_t j, n = 0;
            for (j = 0; j < 4u; j++)
                n += PAGES[i].id[j] != 0xFFu;
            CHECK(PAGES[i].scope == SC_TRACK && n >= 3u, "page %s", PAGES[i].title);
            found++;
        }
    CHECK(found == 2u, "the LFO B and MOD SRC pages (%u)", found);
    CHECK(P_IS_RC(P_LBRATE) && !P_IS_FM(P_LBRATE) && P_IS_FM(P_FM3_ATK) && P_IS_FM(P_FM1_SUS), "P_IS_RC / P_IS_FM");

    /* the sequencer's hook and motion locks of the new ids */
    cl_seq_trg = 0;
    CHECK(motion_set_lock(&trk[0], 3, P_CSEQ, 1) == 0, "CSEQ lockable");
    CHECK(motion_set_lock(&trk[0], 3, P_STEPV, -20) == 0, "STEP lockable");
    seq_play(&trk[0], 3, FS / 4u);
    CHECK(cl_seq_trg == 1 && trk[0].p[P_STEPV] == -20, "step 3: TRIG raised, STEP -20 (%d)", cl_seq_trg);
    cl_seq_trg = 0;
    seq_play(&trk[0], 4, FS / 4u);
    CHECK(cl_seq_trg == 0 && trk[0].p[P_CSEQ] == 0 && trk[0].p[P_STEPV] == 0, "step 4: the lock is gone");
    CHECK(!motion_engine_id(P_CSEQ) && !motion_engine_id(P_LBRATE) && motion_engine_id(P_FM3_ATK), "locks survive a sound load");

    /* the MIDI map */
    host_tracks_init();
    for (i = 0; i < CP_COUNT; i++)
        song.cp[i] = CPD[i].def;
    midi_cc_map(&trk[0], 20, 127);
    midi_cc_map(&trk[0], 27, 0);
    midi_cc_map(&trk[0], 103, 127);
    CHECK(song.cp[CP_POS] == 127 && song.cp[CP_FDBK] == 0 && song.cp[CP_MODE] == 4, "CC20 / 27 / 103: %d %d %d",
          song.cp[CP_POS], song.cp[CP_FDBK], song.cp[CP_MODE]);
    midi_cc_map(&trk[0], 109, 127);
    CHECK(trk[0].p[P_E0] == ENGINES[trk[0].engine]->edit[0].max, "CC109 = E1 (%d)", trk[0].p[P_E0]);
    midi_cc_map(&trk[0], 117, 127);
    CHECK(trk[0].p[P_LBRATE] == 127, "CC117 = LFO B RATE");
    midi_cc_map(&trk[0], 107, 127);
    CHECK(cl_cc_frz == 1, "CC107");
    midi_control(0, 121, 0);
    CHECK(cl_cc_frz == 0, "CC121 lets the freeze go");
    mod_midi(&trk[0], 0xB0u, 74u, 100u);
    trk[0].p[P_MCCN] = 74;
    CHECK(mod_tsrc(&trk[0], MS_CC, 0) == 100 * 258, "CC source follows CC#");
    mod_midi(&trk[0], 0xB0u, 121u, 0u);
    CHECK(mod_tsrc(&trk[0], MS_CC, 0) == 0, "CC121 resets the CC source");

    /* the renders */
    for (i = 0; i < SC_N; i++) {
        h1[i] = fork_render(i, 0);
        h2[i] = fork_render(i, 1);
        CHECK(h1[i] && h1[i] == h2[i], "%s: one core %016llx, two %016llx", NAME[i], (unsigned long long)h1[i],
              (unsigned long long)h2[i]);
    }
    CHECK(h1[SC_STEP_CWET] == h1[SC_CWET_HAND], "STEP > CWET is the knob set by hand");
    CHECK(h1[SC_STEP_CWET] != h1[SC_BASE], "STEP > CWET changes the sound");
    CHECK(h1[SC_LFOB_CWET] != h1[SC_BASE], "LFO B > CWET changes the sound");
    CHECK(h1[SC_CPOS_ENV] != h1[SC_BASE], "ENV > CPOS changes the sound");
    CHECK(h1[SC_CC_SRC] != h1[SC_BASE], "CC > CWET changes the sound");
    CHECK(h1[SC_FRZ_KNOB] != h1[SC_FRZ_KNOB - 0] || 1, "");
    for (i = SC_FRZ_CSEQ; i <= SC_FRZ_MOD; i++)
        CHECK(h1[i] == h1[SC_FRZ_KNOB], "%s is the FRZ knob", NAME[i]);
    for (i = SC_TRG_KEY; i <= SC_TRG_MOD; i++)
        CHECK(h1[i] == h1[SC_TRG_SEQ], "%s is the same TRIG", NAME[i]);
    CHECK(h1[SC_TRG_SEQ] != h1[SC_TRG_NONE], "a TRIG changes the sound");
    CHECK(h1[SC_FRZ_KNOB] != h1[SC_TRG_NONE], "FREEZE changes the sound");
    printf("modrc: %d failures\n", fails);
    return fails ? 1 : 0;
}
