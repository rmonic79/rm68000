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

# die_a123.py - A1/A2/A3 + illegal/privileged table for all 65536 opcodes from the PLAs of the MC68000 die
# (silicon dump: a1-decode.txt, a23-decode.txt, external folder galibert_m68k). Source: die analysis of a standard
# chip. The names (adr-names.txt) are NOT used: the outputs are addresses of the die micro ROM, which is what the core uses.
# Dump format (deduced from the structure, checked against the decoded entries in a1-dump.txt and the micro ROM successors):
#  a1-decode.txt: "top" rows (they start with a space: line A / line F groups), "middle" rows = col 0-9 A1 address (inverted
#   bits) + col 11-44 34-bit term ([33:18] IR bits that must be 0, [17:2] IR bits that must be 1, bit 0 = "long"
#   precondition, bit 1 = "byte/word" precondition) + from col 46 relative offsets of the rows that inhibit the term; "bottom"
#   rows (after two consecutive rows without an address) = size terms: 0-6 long, 7-19 byte/word. The first middle row wins.
#  a23-decode.txt: "top" rows up to the first row that starts with a space: col 0-9 = A3 (inverted), col 11-20 = A2 (inverted),
#   col 22-55 34-bit term ([15:0] bits at 1, [31:16] bits at 0, bit 32/33 = precondition on bottom rows 0/1), inhibitors from
#   col 57. The A2 and A3 outputs are the OR of the raw bits of ALL the active terms (a term with an empty field does not
#   contribute): e.g. CMPI.W #imm,Dn has one term with A2 = rcaw1 and one with A3 = cpdw1. "bottom" rows: col 22 mode ('i'
#   illegal, 'p' privileged, ' ' precondition), col 24-55 32-bit term, inhibitors from col 57.
# Usage: python die_a123.py <repo> <galibert_dir>   -> <repo>/docs/data/mc68000_a123_die.tsv
#   columns: op  a1  a2  a3  ill  priv  linea  linef   (hexadecimal die addresses; 3ff = none)
import sys
R = sys.argv[1]; G = sys.argv[2]
def field(l, col, ln=-1):
    if col >= len(l): return ''
    if ln == -1:
        e = col
        while e < len(l) and l[e] not in ' \t': e += 1
        return l[col:e]
    return l[col:col + ln]
def bits(s):
    v = 0
    for c in s: v = (v << 1) | (1 if c == '1' else 0)
    return v
def inhib(l, col):
    out = []; pos = col
    while True:
        s = field(l, pos)
        if s == '': break
        out.append(int(s)); pos += len(s) + 1
    return out
# ---------------- A1 ----------------
L1 = open(f'{G}/a1-decode.txt', encoding='utf-8', errors='replace').read().split('\n')
top1 = []; mid1 = []; bot1 = []
i = 0
while i < len(L1) and field(L1[i], 0) == '':
    v = bits(field(L1[i], 11))
    if v: top1.append(v)
    i += 1
while i < len(L1) and not (field(L1[i], 0) == '' and (i + 1 >= len(L1) or field(L1[i + 1], 0) == '')):
    l = L1[i]
    if field(l, 0) != '':
        v = bits(field(l, 11))
        if v: mid1.append((bits(field(l, 0)) ^ 0x3ff, v, inhib(l, 46)))
    i += 1
for l in L1[i:]:
    v = bits(field(l, 11))
    if v: bot1.append(v)
def tb1(k, val):
    m = bot1[k]; v1 = m & 0xffff; v0 = (m >> 16) & 0xffff
    return (val & v1) == v1 and ((~val) & v0) == v0
def is_long(val): return any(tb1(k, val) for k in range(0, 7))
def is_bw(val): return any(tb1(k, val) for k in range(7, min(20, len(bot1))))
def test_mid(k, val):
    adr, m, inh = mid1[k]; v1 = (m >> 2) & 0xffff; v0 = (m >> 18) & 0xffff
    if m & 1 and not is_long(val): return False   # bit 0 = long precondition (checked: 0a7 = e#l1, an exact nano match, used with bit 0)
    if m & 2 and not is_bw(val): return False     # bit 1 = byte/word precondition (006 = adrw1: A2 term active for non-long)
    res = (val & v1) == v1 and ((~val) & v0) == v0
    if res:
        for d in inh:
            if test_mid(k + d, val): return False
    return res
def a1_of(val):
    for k in range(len(mid1)):
        if test_mid(k, val): return mid1[k][0]
    return 0x3ff
def top1_of(k, val):
    m = top1[k]; v1 = (m >> 2) & 0xffff; v0 = (m >> 18) & 0xffff
    return (val & v1) == v1 and ((~val) & v0) == v0
# ---------------- A2/A3 ----------------
L2 = open(f'{G}/a23-decode.txt', encoding='utf-8', errors='replace').read().split('\n')
top2 = []; bot2 = []
i = 0
while i < len(L2) and field(L2[i], 0, 1) != ' ': i += 1
for l in L2[:i]:
    v = bits(field(l, 22))
    if v: top2.append((bits(field(l, 0)), bits(field(l, 11)), v, inhib(l, 57)))      # (raw a3, raw a2, term, inhibitors)
for l in L2[i:]:
    if field(l, 0, 1) != '':
        bot2.append((bits(field(l, 24)), field(l, 22, 1), inhib(l, 57)))
def tb2(k, val):
    m, mode, inh = bot2[k]; v1 = m & 0xffff; v0 = (m >> 16) & 0xffff
    res = (val & v1) == v1 and ((~val) & v0) == v0
    if res:
        for d in inh:
            if tb2(k + d, val): return False
    return res
def tt2(k, val):
    a3r, a2r, m, inh = top2[k]; v1 = m & 0xffff; v0 = (m >> 16) & 0xffff
    if (m >> 32) & 1 and not tb2(0, val): return False
    if (m >> 33) & 1 and not tb2(1, val): return False
    res = (val & v1) == v1 and ((~val) & v0) == v0
    if res:
        for d in inh:
            if tt2(k + d, val): return False
    return res
def a23_of(val):
    r2 = 0; r3 = 0
    for k in range(len(top2)):
        if tt2(k, val): r3 |= top2[k][0]; r2 |= top2[k][1]
    return r2 ^ 0x3ff, r3 ^ 0x3ff
def ill_of(val):
    if (val & 0xf000) in (0xa000, 0xf000): return False
    return any(mode == 'i' and tb2(k, val) for k, (m, mode, inh) in enumerate(bot2))
def priv_of(val):
    return any(mode == 'p' and tb2(k, val) for k, (m, mode, inh) in enumerate(bot2))
out = f'{R}/docs/data/mc68000_a123_die.tsv'
n1 = n23 = nill = npriv = 0
with open(out, 'w', encoding='utf-8', newline='\n') as f:
    f.write('# GENERATED by scripts/die_a123.py from the die PLAs (a1-decode.txt, a23-decode.txt): op a1 a2 a3 ill priv linea linef (die addresses, 3ff = none)\n')
    for op in range(65536):
        a1 = a1_of(op); a2, a3 = a23_of(op); ill = ill_of(op); pr = priv_of(op)
        la = int(top1_of(0, op)) if top1 else 0; lf = int(top1_of(1, op)) if len(top1) > 1 else 0
        n1 += a1 != 0x3ff; n23 += (a2 != 0x3ff) or (a3 != 0x3ff); nill += ill; npriv += pr
        f.write(f'{op:04x}\t{a1:03x}\t{a2:03x}\t{a3:03x}\t{int(ill)}\t{int(pr)}\t{la}\t{lf}\n')
print(f'die_a123: A1 middle {len(mid1)} bottom {len(bot1)} | A2/A3 top {len(top2)} bottom {len(bot2)} | opcodes with A1 {n1}, with A2/A3 {n23}, illegal {nill}, privileged {npriv} -> {out}')
