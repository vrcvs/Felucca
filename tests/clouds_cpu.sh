#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
# Instructions per sample of CLOUDS (src/fx_clouds.c) under valgrind (callgrind), from the repo root:
#   tests/clouds_cpu.sh [PRESET]        (default: PHYS preset 1; needs valgrind, gcc)
# One PHYS part (8 keys) alone, then through Clouds in each MODE and QUAL: the difference is Clouds' cost. The unit is
# rings_cpu.sh's (host instructions; one core is about 5 900 per sample at 44.1 kHz, PORTING_NOTES.md).
set -e
mkdir -p build/t
gcc -O2 -w -Ibuild/gen -Ifirmware/src -Itests -o build/t/clouds_cpu tests/clouds_cpu.c -lm
BL=${BLOCKS:-1378}
P=${1:-1}
ir() { valgrind --tool=callgrind --callgrind-out-file=/dev/null build/t/clouds_cpu "$@" 2>&1 | sed -n 's/.*Collected : \([0-9]*\).*/\1/p'; }
a=$(ir $P 0 0 0); b=$(ir $P $BL 0 0)
off=$(( (b - a) / (BL * 32) ))
echo "PHYS $P alone: $off instr/sample"
for m in 1 2 3 4; do
    for q in 0 1 2 3; do
        a=$(ir $P 0 $m $q); b=$(ir $P $BL $m $q)
        echo "MODE $m QUAL $q: $(( (b - a) / (BL * 32) - off )) instr/sample for Clouds"
    done
done
