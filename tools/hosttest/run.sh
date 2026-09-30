#!/bin/sh
# Host tests for the demodulator (Rx.c, Dsp.c) and receiver state
# (Receiver.c). Run from anywhere: ./tools/hosttest/run.sh
#
# Norcroft's `long` is 32 bits; the host's is 64. To catch arithmetic that
# would overflow on the Pi, the sources are copied with `long` rewritten to
# int32_t and compiled with signed-overflow trapping.
#
# ssb_test: USB/LSB/CW (sideband rejection, tuning without a retune, AGC,
#           squelch, chunking, 2.4 MSPS input, mirror).
# rx_test:  NFM/AM (synthetic signals; prints numbers, compare against
#           docs/ROADMAP.md rather than pass/fail, except the identical-
#           output checks).
# rcv_test: Receiver.c defaults, steps, formatting, retune policy.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../app/!RTLSDRView"
OUT=${TMPDIR:-/tmp}/rtlsdr_hosttest
mkdir -p "$OUT/gen"
for f in Rx Dsp; do
  { echo '#include <stdint.h>'; sed -E 's/unsigned long/uint32_t/g; s/\blong\b/int32_t/g' "$SRC/c/$f.c"; } > "$OUT/gen/$f.c"
  { echo '#include <stdint.h>'; sed -E 's/unsigned long/uint32_t/g; s/\blong\b/int32_t/g' "$SRC/h/$f.h"; } > "$OUT/gen/$f.h"
done
CF="-O1 -g -std=gnu99 -w -fsanitize=signed-integer-overflow -fno-sanitize-recover=signed-integer-overflow -I$OUT/gen"
gcc $CF -o "$OUT/ssb_test" "$HERE/ssb_test.c" "$OUT/gen/Rx.c" "$OUT/gen/Dsp.c" -lm
gcc $CF -o "$OUT/rx_test" "$HERE/rx_test.c" "$OUT/gen/Rx.c" "$OUT/gen/Dsp.c" -lm
gcc -O1 -g -std=gnu99 -w -I"$HERE/stub" -I"$OUT/gen" -I"$SRC/h" -o "$OUT/rcv_test" "$HERE/rcv_test.c" "$SRC/c/Receiver.c" "$OUT/gen/Rx.c" "$OUT/gen/Dsp.c" -lm
echo "== rcv_test"; "$OUT/rcv_test"
echo "== ssb_test"; "$OUT/ssb_test"
echo "== rx_test"; "$OUT/rx_test"
