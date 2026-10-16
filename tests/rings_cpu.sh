#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
# Instructions per sample of the PHYS presets under valgrind (callgrind), from the repo root:
#   tests/rings_cpu.sh [PRESET ...]        (default: all 16; needs valgrind, gcc)
# Not the Mac's kernel-counted numbers of tests/cpu_baseline.txt but the same unit (host instructions): compare a Rings
# preset with the factory ones of this run (PLUCK, MARIMBA, BELL TREE ...), not with the file. One core is about
# 5 900 host instructions per sample at 44.1 kHz (PORTING_NOTES.md), 1.7 % per 100.
set -e
mkdir -p build/t
gcc -O2 -w -Ibuild/gen -Ifirmware/src -Itests -o build/t/rings_cpu tests/rings_cpu.c -lm
BL=${BLOCKS:-1378}
ir() { valgrind --tool=callgrind --callgrind-out-file=/dev/null build/t/rings_cpu "$@" 2>&1 | sed -n 's/.*Collected : \([0-9]*\).*/\1/p'; }
presets=${*:-0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15}
for p in $presets; do
    for poly in -1 2; do
        [ $p -lt 9 ] && [ $poly = 2 ] && continue
        a=$(ir $p 0 $poly); b=$(ir $p $BL $poly)
        echo "$p poly $poly $(( (b - a) / (BL * 32) )) instr/sample"
    done
done
