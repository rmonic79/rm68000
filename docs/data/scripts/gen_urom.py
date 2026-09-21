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

# Generates the micro ROM of the silicon core from the decoded micro ROM of the die (mc68000_micro_decoded.csv):
#  - rtl/cpu_si/m68k_urom.mem : 1024 words x 17 bits ($readmemb, bits 16..0 from the left), 548 used (the 544 of the die plus
#    4 added words), the rest 0
# Usage: python gen_urom.py <repo_root>
import csv, sys
# License header written at the top of every generated file (the same text as in the hand-written sources).
LICENSE_HEADER = ('// This file is part of rm68000, an MC68000 core in SystemVerilog.\n'
                  '// Copyright (c) 2026 Umberto Parisi (rmonic79)\n'
                  '//\n'
                  '// rm68000 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public\n'
                  '// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any\n'
                  '// later version. It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the\n'
                  '// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for\n'
                  '// more details: LICENSE in the root of the repository, or <https://www.gnu.org/licenses/>.\n'
                  '//\n'
                  '// A commercial license, for use without the obligations of the GPL (for example in closed-source products), is\n'
                  '// available from the author, who can also be contacted for other licensing needs.\n'
                  '\n')
R = sys.argv[1]
rows = list(csv.DictReader(open(f'{R}/docs/data/mc68000_micro_decoded.csv')))
# HOLE IN THE DUMP: word 1c8 (rstp2) is the only one whose label is all zero (line 104, word 8 of microcode.txt is
# empty). In the chip it must continue to rstp3 (002): format I, TY db, I=0, FC 0, NMA 002 -> in the permuted physical
# layout of the die A1 <- bit 12 (raw17 = bit0..bit16: only bit 12 set).
HOLES = {0x1c8: ('0' * 12 + '1' + '0' * 4, '002')}
# illg1/illg2 (3ff/3fe, free in the die): built from word trap1 (1d0) as an entry of the instruction exceptions; the core does not
# reach them (the instruction exceptions enter at 0e3, A_TRAP in m68k_exc_si.sv): see docs/data/deviations/ustore.tsv
r1d0 = next(r for r in rows if r['addr'] == '1d0')
# 3ff: (pc) -> au - 2, next 3fe ; 3fe: au -> pc, (psw) -> ftu, next 347 (trap2). FIG. 8 format bits with the NMA permutation of m68k_useq.sv
rows.append(dict(r1d0, addr='3ff', label='illg1', I='0', fmt='0', ty='0', cbc='', cond='', nma_or_base='3fe', fc='0', raw17='00000111111011100'))
# stopw (3fd, free in the die): wait word of the STOP = 'jump a1' format of 327 (aaa01) with I = 1 ('dbi' word: exc samples the
# IPL at its end); exc sends its dispatch to 3fc while stopped, until an interrupt arrives (a0 = A_INT at dispatch).
# stopw2 (3fc, free in the die): the other word of the wait loop, format I, I = 1 (samples the request like 3fd: the reference
# core latches it at every word), TY 0 (no dispatch), FC 0, next 3fd: the
# loop 3fc, 3fd has the four-clock period of the die's loop 327, 3a2. Both with the empty nanoword (HAND 0x3fd, 0x3fc in
# gen_nrom.py). raw17 of 3fc from the physical layout of the NMA bits (A0..A9 <- raw bits 11, 12, 7, 8, 9, 10, 5, 6, 13, 14).
r327 = next(r for r in rows if r['addr'] == '327')
rows.append(dict(r327, addr='3fd', label='stopw', I='1', raw17='1' + r327['raw17'][1:]))   # bit0 I = 1: 'dbi' word -> exc samples IPL at every pass
rows.append(dict(r327, addr='3fc', label='stopw2', I='1', fmt='0', ty='0', cbc='', cond='', nma_or_base='3fd', fc='0', raw17='10000111111101100'))
rows.append(dict(r1d0, addr='3fe', label='illg2', I='0', fmt='0', ty='0', cbc='', cond='', nma_or_base='347', fc='0', raw17='00000101000111100'))
mem = ['0' * 17] * 1024
exp = []
for r in rows:
    a = int(r['addr'], 16); raw = r['raw17']            # raw17 = bit0..bit16 from the left
    if a in HOLES: raw, r['nma_or_base'] = HOLES[a]
    mem[a] = raw[::-1]
    fmt = int(r['fmt']); ty = int(r['ty']) if r['ty'] != '' else 0
    cbc = int(r['cbc']) if r['cbc'] != '' else 0
    exp.append((a, int(r['I']), fmt, ty, int(raw[4]), cbc, int(r['fc']), int(r['nma_or_base'], 16), r['label']))
with open(f'{R}/rtl/cpu_si/m68k_urom.mem', 'w', newline='\n') as f:
    f.write(LICENSE_HEADER)
    f.write('// MC68000 micro ROM: 1024 x 17 bits, content = die bits (public data, see docs/microcode-sources.md) plus the words added or repaired by gen_urom.py\n')
    f.write('// format of patent US 4,325,121 FIG. 8: bit0 I, bit1 fmt, bit2-3 TY / bit2-6 CBC, bit5-14 NMA (permuted, see m68k_useq.sv), bit15-16 FC\n')
    f.write('// generated by docs/data/scripts/gen_urom.py - DO NOT edit by hand\n')
    for w in mem: f.write(w + '\n')
print("words", len(exp), "fmt1", sum(e[2] for e in exp), "ty!=0", sum(1 for e in exp if not e[2] and e[3]))
