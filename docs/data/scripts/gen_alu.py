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

# FIG. 17 table (ALU/CC matrix, 15 rows x 5 columns) -> rtl/cpu_si/m68k_alu_pkg.sv.
# Cell = "OP CODE" with a 5-character CODE in the order X N Z V C; in column 1 "CODE1 / 1 CODE2" = initiate entry / finish entry
# (patent col. 30: "first entry ... only the initiate bit is set ... second entry ... only the finish bit"). Column 6 = column 4
# with lss inserting 1. CC tokens: K keep, D don't care (= keep), 0 clear, N/Z/V/C last state, ~C complement of the carry,
# C* = C or carry of the decimal correction (m68k_alu_si), ~C*, A = X and C from the arithmetic shift carry, V' = N xor C.
# Usage: python gen_alu.py <repo_root>
import re, sys
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
R = sys.argv[1]; D = f'{R}/docs/data'
OPS = ['NONE', 'AND', 'OR', 'EOR', 'ADD', 'ADDX', 'ADD1', 'SUB', 'SUBX', 'SUB0', 'EXT', 'LSS', 'LSR', 'LSL', 'ASR', 'ASL', 'ROL', 'ROR', 'ROXL', 'ROXR', 'SLAA']
ALIAS = {'ADDX': 'ADDX', 'RDL': 'ROL', 'RDR': 'ROR', 'SLAA_X': 'SLAA', 'EDR': 'EOR'}
TOK = ['K', 'D', '0', 'N', 'Z', 'V', 'C', '~C', 'C*', '~C*', 'A', "V'", '1', 'X']   # X = copy of X into C (ROXd with a count of 0)
def tokens(code):
    out = []; i = 0
    while i < len(code):
        for t in ("~C*", "C*", "V'", "~C"):
            if code.startswith(t, i): out.append(t); i += len(t); break
        else: out.append(code[i]); i += 1
    assert len(out) == 5, (code, out)
    return out
cells = {}   # (row, col) -> (op, init_tokens, fin_tokens)
for l in open(f'{D}/mc68000_alu_matrix_fig17.tsv', encoding='utf-8'):
    if l.startswith('#') or l.startswith('row'): continue
    f = l.rstrip('\n').split('\t'); row = int(f[0])
    for col in range(1, 6):
        c = f[col].strip()
        if c == '-' or c == '': cells[(row, col)] = ('NONE', ['K'] * 5, ['K'] * 5); continue
        parts = c.split(' / ')
        op_code = parts[0].split(' ', 1); op = ALIAS.get(op_code[0], op_code[0])
        assert op in OPS, (row, col, op)
        ini = tokens(op_code[1]) if len(op_code) > 1 else ['K'] * 5
        fin = tokens(parts[1].split(' ', 1)[1]) if len(parts) > 1 else ini
        cells[(row, col)] = (op, ini, fin)
# Row 1 column 5 (DIVS/DIVU, the only '5i' word: dvuma 0e6 = overflow exit): the transcription says "EXT" with no CC code;
# Motorola manual: V = 1, C = 0, N/Z undefined (here K).
cells[(1, 5)] = ('EXT', tokens('KKK10'), tokens('KKK10'))
# Row 11 columns 2/3 (NOT = SUB0: 0 - a - 1 = ~a): the transcription says KNZV~C, but 0 - a - 1 always borrows (C = 1) and the
# 68000 clears C (manual; NOT.B then Scc): C = 0.
cells[(11, 2)] = ('SUB0', tokens('KNZV0'), tokens('KNZV0')); cells[(11, 3)] = ('SUB0', tokens('KNZV0'), tokens('KNZV0'))
# Row 15 column 5 (TAS, TST, SCC, MOVB): the transcription has an empty cell, but the index-mode words (aixw0/aixl0/rmxw0/leax0/...:
# "(dbin) -> ab* -> alu", column 5) are the same for every instruction and the d8 must be sign-extended here too (without it,
# TST.W/.L, all the Scc and MOVEA.W/.L with d8(An,Xn) would access An+Xn without d8). Column 5 = EXT for EVERY row, including
# row 0 (no FIG. 18 term: MOVEA), CC kept. 16-row table: index = row*5 + column-1.
cells[(15, 5)] = ('EXT', tokens('KKKKK'), tokens('KKKKK'))
# Row 0 = no FIG. 18 term active (line A/F, MOVEA with the qualifier): in the die no row is inhibited, i.e. ALL rows are active;
# columns 1 (AND) and 5 (EXT) are the same in every row and stay defined, the others do not. With column 1 at NONE the trap
# words (AND with -1 of PCL/SR/PCH, cc 'n') would stack zero for line A/F. CC kept.
cells[(0, 1)] = ('AND', ['K'] * 5, ['K'] * 5)
for col in range(2, 5): cells[(0, col)] = ('NONE', ['K'] * 5, ['K'] * 5)
cells[(0, 5)] = ('EXT', tokens('KKKKK'), tokens('KKKKK'))
# Rows 8 (ROXR) and 11 (ROXL) column 1, initiate entry: ROXd with a register count of 0 (mod 64) leaves the operand and sets C = X
# (68000 manual): the die flow is sriw1 -> srrw2 (column 1 initiate, AND with -1 of the operand) -> exit on auz with no shift steps.
# The transcription says KNZ00 (C = 0, right for LSd/ASd/ROd).
cells[(8, 1)] = ('AND', tokens('KNZ0X'), tokens('KNZKK')); cells[(11, 1)] = ('AND', tokens('KNZ0X'), tokens('KNZKK'))
# Row 9 column 2 (SBCD/NBCD, first step: binary difference with borrow in X): Z KEPT as in the first step of ABCD (row 3
# column 2 'ADDX CDDDD'); with the transcription '~CNZV~C' Z would be cleared on the intermediate difference, and the correction
# (column 3, Z = Z_prev & zero) would no longer give Z = 1 on a zero result with initial Z = 1 (manual: Z unchanged if the result is 0).
cells[(9, 2)] = ('SUBX', tokens('~CNDV~C'), tokens('~CNDV~C'))
nl = '\n'
with open(f'{R}/rtl/cpu_si/m68k_alu_pkg.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// FIG. 17 table (US 4,325,121) for m68k_alu_si.sv - GENERATED by gen_alu.py from the transcription docs/data/mc68000_alu_matrix_fig17.tsv' + nl)
    f.write('package m68k_alu_pkg;' + nl)
    for i, o in enumerate(OPS): f.write(f'localparam logic [4:0] AOP_{o} = 5\'d{i};' + nl)
    for i, t in enumerate(TOK): f.write(f'localparam logic [3:0] ACC_{re.sub(r"[^A-Za-z0-9]", lambda m: {"~": "N", "*": "S", chr(39): "P"}[m.group(0)], t)} = 4\'d{i};' + nl)
    f.write('// entry = {op[4:0], init X N Z V C (5x4), finish X N Z V C (5x4)} = 45 bits; index = row*5 + (column-1); row 0 = no row (AND in column 1, EXT in column 5: docs/data/deviations/alu.tsv)' + nl)
    f.write('localparam logic [44:0] SI_ALU_TABLE [0:79] = \'{' + nl)
    ent = []; entv = {}   # entv: (row, column) -> (operation index, 45-bit entry)
    for row in range(0, 16):
        for col in range(1, 6):
            op, ini, fin = cells[(row, col)]
            v = OPS.index(op) << 40
            for k, t in enumerate(ini): v |= TOK.index(t) << (36 - 4 * k)
            for k, t in enumerate(fin): v |= TOK.index(t) << (16 - 4 * k)
            ent.append((f"  45'h{v:012x}", f"  // r{row} c{col}: {op} {''.join(ini)} / {''.join(fin)}"))
            entv[(row, col)] = (OPS.index(op), v)
    for i, (v, c) in enumerate(ent): f.write(v + (',' if i < len(ent) - 1 else ' ') + c + nl)
    f.write('};' + nl)
    # One-hot operation code of every entry, in the order of SI_ALU_TABLE (index = row*5 + column-1): bit k = (op == k).
    W1H = len(OPS); order = [(row, col) for row in range(0, 16) for col in range(1, 6)]
    f.write('// ONE-HOT operation code per entry: bit k = (entry[44:40] == k), computed from SI_ALU_TABLE;' + nl)
    f.write('// registered at op_ld next to the entry (m68k_alux_si) and used by m68k_alu_si to select without decoding.' + nl)
    f.write(f"localparam logic [{W1H - 1}:0] SI_ALU_OP1H [0:{len(order) - 1}] = '{{" + nl)
    for i, rc in enumerate(order):
        op = entv[rc][0]
        f.write(f"  {W1H}'h{1 << op:06x}" + (',' if i < len(order) - 1 else '') + f'   // {i}: op {op}' + nl)
    f.write('};' + nl)
    # The same two tables with direct index {row[3:0], colx[2:0]}: colx 1..5 = column, colx 0/6/7 = no entry (0 = NONE).
    order2 = [(row, cx) for row in range(0, 16) for cx in range(0, 8)]
    f.write('// the same tables with DIRECT INDEX {row[3:0], colx[2:0]} (128 entries): entry = TABLE[row*5+colx-1] for colx 1..5,' + nl)
    f.write('// otherwise 0 / NONE: no arithmetic on the index in front of the mux.' + nl)
    f.write(f"localparam logic [44:0] SI_ALU_T2 [0:{len(order2) - 1}] = '{{" + nl)
    for i, (row, cx) in enumerate(order2):
        v = entv[(row, cx)][1] if 1 <= cx <= 5 else 0
        f.write(f"  45'h{v:012x}" + (',' if i < len(order2) - 1 else '') + f'   // r{row} c{cx}' + nl)
    f.write('};' + nl)
    f.write(f"localparam logic [{W1H - 1}:0] SI_ALU_OP1H2 [0:{len(order2) - 1}] = '{{" + nl)
    for i, (row, cx) in enumerate(order2):
        op = entv[(row, cx)][0] if 1 <= cx <= 5 else OPS.index('NONE')
        f.write(f"  {W1H}'h{1 << op:06x}" + (',' if i < len(order2) - 1 else '') + f'   // r{row} c{cx}' + nl)
    f.write('};' + nl + 'endpackage' + nl)
print(f'cells {len(cells)} | operations used {sorted(set(c[0] for c in cells.values()))}')
