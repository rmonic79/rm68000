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

# Conditional branch unit (patent FIG. 13/14, App. C/D/E). From the transcription of App. C
# (us4325121_appC.tsv: CBC code -> variable, values -> C1 C0) generates rtl/cpu_si/m68k_branch_pkg.sv (table
# code x value -> C1C0, 32 codes x 16 values); cross-check with the die trace
# (jumpcc: the targets are listed in the order of the values of the variable, bits 7:6 of the target = C1 C0).
# Variables (App. C remarks + App. D): i11 = IRC11; auz = AU==0; c z n v = PSW; nz1/nz2 = {n,z}; ms0 = {ird8, alue0};
# m01 = {auz, ird8, alue1, alue0}; ze = LOCZ (local copy of ALU==0); nv = {n,v}; d4 = DCR4; enl = {END, ird6}; cc = App. E.
# Usage: python gen_branch.py <repo_root> <galibert_dir>
import re, sys, csv, collections
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
R = sys.argv[1]; G = sys.argv[2]; D = f'{R}/docs/data'
VARS = {'i11': 1, 'auz': 1, 'c': 1, 'z': 1, 'nz1': 2, 'n': 1, 'nz2': 2, 'ms0': 2, 'm01': 4, 'ze': 1, 'nv': 2, 'd4': 1, 'v': 1, 'enl': 2, 'cc': 1}
rows = collections.defaultdict(list)   # code -> [(var, value_pattern, c1c0)]
for l in open(f'{D}/us4325121_appC.tsv', encoding='utf-8'):
    if l.startswith('#') or not l.strip(): continue
    f = l.rstrip('\n').split('\t')
    code = int(f[0], 16); var = f[1]; vals = f[3].split(' ')[0]; c1c0 = int(f[4]) * 2 + int(f[5])
    for pat in vals.split(','): rows[code].append((var, pat, c1c0))
table = {}   # (code, value) -> c1c0 ; value = integer over the bits of the variable (msb = first character)
var_of = {}
for code, lst in rows.items():
    var = lst[0][0]; var_of[code] = var; w = VARS[var]
    for v in range(1 << w):
        s = format(v, f'0{w}b'); hit = None
        for _, pat, c in lst:
            if len(pat) == w and all(p == 'x' or p == ch for p, ch in zip(pat, s)): hit = c; break
        assert hit is not None, (code, var, s)
        table[(code, v)] = hit
# check against the die trace
micro = {int(r['addr'], 16): r for r in csv.DictReader(open(f'{D}/mc68000_micro_decoded.csv'))}
cur = None; n_ok = 0; n_bad = 0; bad = []
for l in open(f'{G}/all.txt', encoding='utf-8', errors='replace'):
    m = re.match(r'^([0-9a-f]{3}) (\w+):', l)
    if m: cur = int(m.group(1), 16); continue
    if cur is None or not l.startswith('  ') or not l.strip().startswith('jumpcc'): continue
    m = re.match(r'jumpcc (\S+) (.*)$', l.strip()); tg = [int(x, 16) for x in m.group(2).split(',')]
    code = int(micro[cur]['cbc']); var = var_of[code]; w = VARS[var]
    # order of the values as in App. C (patterns in the printed order, with x expanded in increasing order)
    seq = []
    for _, pat, c in rows[code]:
        for v in range(1 << w):
            s = format(v, f'0{w}b')
            if all(p == 'x' or p == ch for p, ch in zip(pat, s)) and c not in seq: seq.append(c)
    got = [(t >> 6) & 3 for t in tg if t >= 0]
    if got == seq[:len(got)]: n_ok += 1
    else: n_bad += 1; bad.append(f'{cur:03x} {micro[cur]["label"]} cbc={code:x} trace {got} appC {seq}')
nl = '\n'
with open(f'{R}/rtl/cpu_si/m68k_branch_pkg.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// App. C table (US 4,325,121): CBC code x variable value -> C1C0. GENERATED by gen_branch.py' + nl)
    f.write('package m68k_branch_pkg;' + nl + '/* verilator lint_off UNUSEDPARAM */' + nl)
    f.write('// index = {cbc[4:0], value[3:0]} (value right-aligned, msb = first character of App. C); unused codes -> 2\'b00' + nl)
    f.write('localparam logic [1:0] SI_BRANCH_TABLE [0:511] = \'{' + nl)
    ent = []
    for code in range(32):
        for v in range(16):
            c = table.get((code, v), 0) if code in var_of and v < (1 << VARS[var_of[code]]) else 0
            ent.append((f"  2'd{c}", f"  // cbc {code:02x} {var_of.get(code, '-')} value {v}"))
    for i, (v, c) in enumerate(ent): f.write(v + (',' if i < len(ent) - 1 else ' ') + c + nl)
    f.write('};' + nl)
    for code in sorted(var_of): f.write(f'localparam logic [4:0] CBC_{var_of[code].upper()}{"P" if code & 0x10 else ""} = 5\'h{code:02x};' + nl)
    f.write('endpackage' + nl)
print(f'App. C codes {len(var_of)} | table entries {len(table)} | jumpcc trace: consistent {n_ok}, different {n_bad}')
for b in bad[:6]: print('  ', b)
