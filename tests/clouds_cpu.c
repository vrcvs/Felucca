/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* The cost of CLOUDS (src/fx_clouds.c) in instructions per sample, for valgrind: one PHYS part (a preset, 8 keys
 * held, as tests/rings_cpu.c) through Clouds in MODE / QUAL, BLOCKS blocks of 32 samples after 1 s of warm-up (the
 * buffer full, the grains running).
 *   clouds_cpu PRESET BLOCKS MODE QUAL [FREEZE]     MODE 0 = OFF (the part alone)
 * tests/clouds_cpu.sh runs it twice (BLOCKS 0 and N) under callgrind and prints the difference per sample. */
#define main hostsim_main
#include "hostsim.c"
#undef main
static int32_t last_out[2 * CTL];
int main(int argc, char **argv)
{
    static const uint8_t NOTES[8] = {48, 52, 55, 59, 60, 64, 67, 71};
    uint32_t pi = argc > 1 ? (uint32_t)atoi(argv[1]) : 0, nb = argc > 2 ? (uint32_t)atoi(argv[2]) : 0, k, i;
    host_tracks_init();
    for (i = 0; i < CP_COUNT; i++)
        song.cp[i] = CPD[i].def;
    song.cp[CP_MODE] = (int16_t)(argc > 3 ? atoi(argv[3]) : 1);
    song.cp[CP_QUAL] = (int16_t)(argc > 4 ? atoi(argv[4]) : 0);
    song.cp[CP_FRZ] = (int16_t)(argc > 5 ? atoi(argv[5]) : 0);
    song.cp[CP_DENS] = 110;                              /* (many grains: the dear side of DENSITY) */
    host_preset(&trk[0], ENGI_PHYS, pi);
    trk[0].p[P_VOICE] = V_POLY;
    trk[0].p[P_SUS] = 127;
    trk[0].p[P_AMODE] = 0;
    for (i = 0; i < 8; i++)
        trk_note_on(&trk[0], NOTES[i], 100);
    for (k = 0; k < FS / CTL; k++)
        mix_block(last_out, CTL);
    for (k = 0; k < nb; k++)
        mix_block(last_out, CTL);
    printf("%u %d\n", pi, last_out[5]);
    return 0;
}
