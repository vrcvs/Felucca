/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* The cost of PHYS presets (Rings' models among them) in instructions per sample, for valgrind (no instruction
 * counter on Linux in regress.c): one part, 8 keys held (as regress.c job_cpu), BLOCKS blocks of 32 samples.
 *   rings_cpu PRESET BLOCKS [POLY]      POLY: Rings' E5 (0 1 voice, 1 two, 2 four), -1 = the preset's
 * tests/rings_cpu.sh runs it twice (BLOCKS 0 and N) under callgrind and prints the difference per sample. */
#define main hostsim_main
#include "hostsim.c"
#undef main
static int32_t last_out[2 * CTL];
int main(int argc, char **argv)
{
    static const uint8_t NOTES[8] = {48, 52, 55, 59, 60, 64, 67, 71};
    uint32_t pi = argc > 1 ? (uint32_t)atoi(argv[1]) : 0, nb = argc > 2 ? (uint32_t)atoi(argv[2]) : 0, k, i;
    int poly = argc > 3 ? atoi(argv[3]) : -1;
    host_tracks_init();
    host_preset(&trk[0], 9, pi);
    trk[0].p[P_VOICE] = V_POLY;
    trk[0].p[P_SUS] = 127;
    trk[0].p[P_AMODE] = 0;
    if (poly >= 0)
        trk[0].p[P_E5] = (int16_t)poly;
    for (i = 0; i < 8; i++)
        trk_note_on(&trk[0], NOTES[i], 100);
    for (k = 0; k < nb; k++)
        mix_block(last_out, CTL);
    printf("%u %d\n", pi, last_out[5]);
    return 0;
}
