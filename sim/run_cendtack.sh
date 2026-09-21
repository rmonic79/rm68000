#!/bin/sh
# This file is part of rm68000, an MC68000 core in SystemVerilog.
# Copyright (c) 2026 Umberto Parisi (rmonic79)
#
# rm68000 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
# later version. It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details: LICENSE in the root of the repository, or <https://www.gnu.org/licenses/>.
#
# A commercial license, for use without the obligations of the GPL (for example in closed-source products), is
# available from the author, who can also be contacted for other licensing needs.

# run_cendtack.sh <build folder> — the runs of Makefile.cendtack, from the build folder (docs/clock-dtack.md, section 6.1).
# 1) Standard conditions: every supported divider (12, 8, 6, 4, 3, 2) idle, and the memory map, short / long / burst
#    stalls and saturation on a selection of them. Each run must end with RESULT PASS: every check of tb_cendtack.cpp,
#    with expected values from docs/clock-dtack.md (sections 3.2 and 4).
# 2) The two conditions of section 4.5 where the memory looks busy while the processor is NOT waiting, on six of them:
#      BUSYBACK=1  bus_busy raised again after DTACK was given: RESULT PASS, and the run identical clock by clock (hash of
#                  cpu_cen, cpu_cenb and DTACKn at every clock) with the same distance from the nominal clock as the run
#                  without the condition, since nothing is counted after DTACK and DTACK is never withdrawn (sections 4.5
#                  and 4.9); the condition must occur (pulses with bus_busy after DTACK > 0);
#      MAPLAT=0    the map raises bus_cs and bus_busy in the same clock as the strobes: RESULT PASS, that is the busy clock
#                  of the strobes is not part of Lb (section 4.4) and its pulse is not lost (section 4.5); over the six runs
#                  the condition must occur (a pulse in that clock with bus_busy high).
# Prints one line per run and a final VERDICT.
cd "$1" || exit 1
rm -f cd_*.log
ok=1
run_case() {   # run_case "<env>": prints <RESULT> <distance> <trace hash> <busy after DTACK> <busy in the strobe clock> <failed checks>
  L="cd_$(echo $1 | tr ' =' '__').log"
  [ -f "$L" ] || env $1 ./Vtop_cendtack > "$L" 2>&1
  s=$(grep -E '^SUMMARY ' "$L" | tail -1)
  if [ -n "$s" ]; then echo "${s#SUMMARY }"; else echo "NONE - - 0 0 no-SUMMARY-line"; fi
}
echo "--- standard conditions (every check of the bench)"
for cfg in "DEN=12" "DEN=8" "DEN=6" "DEN=4" "DEN=3" "DEN=2" "DEN=12 MODE=map" "DEN=8 MODE=map" "DEN=6 MODE=map" "DEN=3 MODE=map" "DEN=12 MODE=short" "DEN=3 MODE=short" "DEN=8 MODE=long" "DEN=12 MODE=long" "DEN=4 MODE=long" "DEN=8 MODE=burst" "DEN=2 MODE=burst" "DEN=12 MODE=sat"; do
  set -- $(run_case "$cfg"); echo "[$cfg] $1 | distance from the nominal clock $2 | failed checks: $6"; [ "$1" = "PASS" ] || ok=0
done
echo "--- memory busy while the processor is not waiting (section 4.5)"
nstrobe=0
for base in "DEN=8 MODE=map" "DEN=12 MODE=map" "DEN=12 MODE=short" "DEN=8 MODE=long" "DEN=12 MODE=long" "DEN=8 MODE=burst"; do
  set -- $(run_case "$base"); r0=$1; d0=$2; h0=$3
  set -- $(run_case "$base BUSYBACK=1"); r1=$1; d1=$2; h1=$3; n1=$4; f1=$6
  set -- $(run_case "$base MAPLAT=0"); r2=$1; d2=$2; n2=$5; f2=$6
  same="different"; [ "$h1" = "$h0" ] && same="identical"
  v1="OK"; { [ "$r0" = "PASS" ] && [ "$r1" = "PASS" ] && [ "$h1" = "$h0" ] && [ "$d1" = "$d0" ] && [ "$n1" -gt 0 ]; } || { v1="FAIL"; ok=0; }
  v2="OK"; [ "$r2" = "PASS" ] || { v2="FAIL"; ok=0; }
  nstrobe=$((nstrobe + n2))
  echo "[$base] BUSYBACK=1: $r1, trace $same, distance $d1 (without the condition $d0), pulses with busy after DTACK $n1, failed checks: $f1 -> $v1 | MAPLAT=0: $r2, distance $d2, pulses with busy in the strobe clock $n2, failed checks: $f2 -> $v2"
done
v3="OK"; [ "$nstrobe" -gt 0 ] || { v3="FAIL"; ok=0; }
echo "MAPLAT=0 condition exercised: pulses with busy in the strobe clock over the six runs $nstrobe -> $v3"
echo "VERDICT: $([ $ok -eq 1 ] && echo PASS || echo FAIL)"
[ $ok -eq 1 ]
