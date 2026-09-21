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

# Row decoder of the ALU/CC matrix (FIG. 18 -> PLA FIG. 19A/B): IRD -> one of the 15 rows of FIG. 17.
# Semantics (patent col. ~: "rows set off in parenthesis ... further decode row N"): each numbered term T is a product
# of IRD bits (n = bit at 1, n' = bit at 0); the "(k) i" terms that FOLLOW T disable it when their product is true.
# An active term inhibits the listed rows; the selected row is the one NOT inhibited by any term (one remains).
# Writes the term form of the decoder to <out_dir>/m68k_alu_rowdec.sv. The rtl/cpu_si/m68k_alu_rowdec.sv of the repository is
# NOT this form: it is the exhaustive table that gen_rowdec_case.py builds from the dump of this form (sim/dump_rowdec.cpp ->
# docs/data/mc68000_alu_rowdec_dump.txt). So that the table is never overwritten, this script never writes into the
# repository: it stops if <out_dir> lies inside <repo_root> or if <out_dir>/m68k_alu_rowdec.sv already exists.
# Usage: python gen_alu_rowdec.py <repo_root> <out_dir>
import os, re, sys
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
if len(sys.argv) != 3: sys.exit('usage: python gen_alu_rowdec.py <repo_root> <out_dir>')
R = sys.argv[1]; D = f'{R}/docs/data'; OUT = sys.argv[2]
def inside(p, root):   # True if p is root or lies under it (paths on different drives are never inside)
    p, root = os.path.normcase(os.path.realpath(p)), os.path.normcase(os.path.realpath(root))
    try: return os.path.commonpath([p, root]) == root
    except ValueError: return False
if inside(OUT, R): sys.exit(f'gen_alu_rowdec.py: {OUT} is inside the repository {R}: write the term form elsewhere')
TARGET = os.path.join(OUT, 'm68k_alu_rowdec.sv')
if os.path.exists(TARGET): sys.exit(f'gen_alu_rowdec.py: {TARGET} already exists: not overwritten')
os.makedirs(OUT, exist_ok=True)
terms = []   # [idx, bits[(n,val)], name, inhibited[], qualifiers[[(n,val)]]]
for l in open(f'{D}/mc68000_alu_rows_fig18.tsv', encoding='utf-8'):
    if l.startswith('#'): continue
    f = l.rstrip('\n').split('\t')
    if len(f) < 4: continue
    idx, dec, name, inh = f[0], f[1], f[2], f[3]
    bits = [(int(m.group(1)), m.group(2) is None) for m in (re.match(r"(\d+)(')?$", tok) for tok in dec.split()) if m]
    if idx.startswith('('):
        if terms: terms[-1][4].append(bits)
    else:
        terms.append([int(idx), bits, name, [int(x) for x in inh.split()] if inh else [], []])
# The PLA of FIG. 19A/B (transcribed in mc68000_alu_rows_fig19_check.tsv) is the hardware: where it differs from the printed table
# (8 lines: 1, 18, 19, 20, 22, 23, 28, 39) the bits and inhibited rows of the PLA win; the "(k) i" qualifiers stay from the table.
pla = {}
for l in open(f'{D}/mc68000_alu_rows_fig19_check.tsv', encoding='utf-8'):
    if l.startswith('#'): continue
    f = l.rstrip('\n').split('\t')
    if len(f) >= 3 and f[0].isdigit():
        pla[int(f[0])] = ([(int(m.group(1)), m.group(2) is None) for m in (re.match(r"(\d+)(')?$", tok) for tok in f[1].split()) if m],
                          [int(x) for x in f[2].split()] if f[2].strip() else [])
# ONLY the lines where the PLA confirms the misprint and the opcode map are adopted (1, 20 cmpi, 39 left shift); the other
# divergences (18, 19, 22, 23, 28) contradict the opcode map, so the table stays (re-read: mc68000_alu_rows_fig19_recheck.tsv).
PLA_ADOPT = {1, 20, 39}
n_pla = 0; n_div = []
for t in terms:
    if t[0] in pla and (t[1], t[3]) != pla[t[0]]:
        if t[0] in PLA_ADOPT: t[1], t[3] = pla[t[0]]; n_pla += 1
        else: n_div.append(t[0])
# DBcc (DCNT): FIG. 17 row 2 lists DCNT and line 13 is called "ldq,dcnt,movw,movl,bcc", but the printed bits (15' 13) do not cover
# 0101 cccc 11 001 rrr, which would fall in line 14 (scc,movb) -> row 15, column 2 = OR (does not decrement). Term 45 'dcnt'
# with the inhibited rows of line 13 (row 2 remains: ADD) and a qualifier on line 14.
DCNT = [(15, False), (14, True), (13, False), (12, True), (7, True), (6, True), (5, False), (4, False), (3, True)]
_t13 = next(t for t in terms if t[0] == 13); _t14 = next(t for t in terms if t[0] == 14)
terms.insert(terms.index(_t13) + 1, [45, DCNT, 'dcnt', list(_t13[3]), []]); _t14[4].append(DCNT)
# Address register destination = the PSW is not updated (68000 manual): MOVEA (001x xxx 001), ADDA/SUBA (1x01 xxx x11),
# ADDQ/SUBQ #n,An (0101 xxx x ss 001, not Scc/DBcc). The die words are the same as for the Dn forms (rorm1/raql1 '2i'): the
# inhibition is in the PSW write, from the IRD. Decoder output cc_inh (the alux clears init/fin); the row stays the natural one
# (MOVEA/ADDA 2, SUBA 5).
CCINH = [[(15, False), (14, False), (13, True), (8, False), (7, False), (6, True)],                              # MOVEA.W/.L
         [(15, True), (13, False), (12, True), (7, True), (6, True)],                                             # ADDA/SUBA
         [(15, False), (14, True), (13, False), (12, True), (5, False), (4, False), (3, True)]]                   # ADDQ/SUBQ An (0101, mode 001; Scc/DBcc have bits 7:6 = 11 and must be excluded)
CCINH_NOT = [[], [], [(7, True), (6, True)]]   # for each term: product that disables it
def rows(op):
    inh = set()
    for idx, bits, name, ih, quals in terms:
        if all(((op >> b) & 1) == v for b, v in bits) and not any(all(((op >> b) & 1) == v for b, v in q) for q in quals): inh |= set(ih)
    return [r for r in range(1, 16) if r not in inh]
nl = '\n'
def sv_term(bits): return ' & '.join(f'{"" if v else "~"}ird[{b}]' for b, v in bits) if bits else "1'b1"
with open(TARGET, 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// m68k_alu_rowdec.sv - row decoder of the ALU/CC matrix (US 4,325,121 FIG. 18/19): IRD -> row 1..15 of FIG. 17.' + nl)
    f.write('// GENERATED by docs/data/scripts/gen_alu_rowdec.py from the transcription docs/data/mc68000_alu_rows_fig18.tsv, DO NOT edit by hand.' + nl)
    f.write('// row_n[k] = 1 if row k+1 is NOT inhibited by any active term (only one row stays active for valid instructions).' + nl)
    f.write('module m68k_alu_rowdec (' + nl + '  /* verilator lint_off UNUSEDSIGNAL */' + nl + '  input  logic [15:0] ird,' + nl + '  /* verilator lint_on UNUSEDSIGNAL */' + nl + '  output logic [14:0] row_n,     // one-hot (bit k = row k+1)' + nl + '  output logic [3:0]  row,       // row number 1..15 (0 = none)' + nl + '  output logic        cc_inh     // An destination (MOVEA, ADDA/SUBA, ADDQ/SUBQ An): the PSW is not updated' + nl + ');' + nl)
    for idx, bits, name, ih, quals in terms:
        q = ''.join(f' & ~({sv_term(qb)})' for qb in quals)
        f.write(f'  logic t{idx}; assign t{idx} = ({sv_term(bits)}){q};  // {name}' + nl)
    f.write('  logic [14:0] inh;' + nl + '  always_comb begin' + nl + '    inh = \'0;' + nl)
    for idx, bits, name, ih, quals in terms:
        if ih: f.write(f'    if (t{idx}) inh = inh | 15\'b{"".join("1" if r in ih else "0" for r in range(15, 0, -1))};' + nl)
    f.write('    row_n = ~inh;' + nl + '    row = 4\'d0;' + nl + '    for (int k = 14; k >= 0; k--) if (row_n[k]) row = 4\'(k + 1);' + nl
            + '    if (inh == 15\'d0) row = 4\'d0;   // no active term (line A/F, instructions without ALU): no row' + nl + '  end' + nl)
    f.write('  assign cc_inh = ' + ' | '.join(f'(({sv_term(b)})' + (f' & ~({sv_term(n)})' if n else '') + ')' for b, n in zip(CCINH, CCINH_NOT)) + ';   // An destination: MOVEA, ADDA/SUBA, ADDQ/SUBQ An' + nl + 'endmodule' + nl)
tests = {'ADD': 0xD041, 'ADDX': 0xD141, 'SUB': 0x9041, 'SUBX': 0x9141, 'AND': 0xC041, 'OR': 0x8041, 'EOR': 0xB141, 'CMP': 0xB041, 'CMPA': 0xB0C1, 'CMPM': 0xB149,
         'MULU': 0xC0C1, 'MULS': 0xC1C1, 'DIVU': 0x80C1, 'DIVS': 0x81C1, 'ABCD': 0xC101, 'SBCD': 0x8101, 'ADDQ': 0x5041, 'SUBQ': 0x5141, 'MOVEQ': 0x7001, 'MOVE.W': 0x3041, 'MOVE.B': 0x1041, 'MOVE.L': 0x2041,
         'DBcc': 0x51C9, 'Scc': 0x50C0, 'Bcc': 0x6700, 'ADDI': 0x0640, 'SUBI': 0x0440, 'ANDI': 0x0240, 'ORI': 0x0040, 'EORI': 0x0A40, 'CMPI': 0x0C40,
         'BTST#': 0x0800, 'BCHG#': 0x0840, 'BCLR#': 0x0880, 'BSET#': 0x08C0, 'BTSTD': 0x0100, 'BCHGD': 0x0140, 'BCLRD': 0x0180, 'BSETD': 0x01C0,
         'NEGX': 0x4040, 'CLR': 0x4240, 'NEG': 0x4440, 'NOT': 0x4640, 'EXT': 0x4880, 'NBCD': 0x4800, 'SWAP': 0x4840, 'TST': 0x4A40, 'TAS': 0x4AC0, 'CHK': 0x4181,
         'ASR': 0xE040, 'ASL': 0xE140, 'LSR': 0xE048, 'LSL': 0xE148, 'ROXR': 0xE050, 'ROXL': 0xE150, 'ROR': 0xE058, 'ROL': 0xE158,
         'ASRm': 0xE0D0, 'ASLm': 0xE1D0, 'LSRm': 0xE2D0, 'LSLm': 0xE3D0, 'ROXRm': 0xE4D0, 'ROXLm': 0xE5D0, 'RORm': 0xE6D0, 'ROLm': 0xE7D0, 'ASR.L': 0xE080, 'ASR.B': 0xE000, 'LSR.L#': 0xE088}
want = {'ADD': 2, 'ADDX': 12, 'SUB': 5, 'SUBX': 10, 'AND': 4, 'OR': 14, 'EOR': 13, 'CMP': 6, 'CMPA': 6, 'CMPM': 6, 'MULU': 7, 'MULS': 7, 'DIVU': 1, 'DIVS': 1, 'ABCD': 3, 'SBCD': 9,
        'ADDQ': 2, 'SUBQ': 5, 'MOVEQ': 2, 'MOVE.W': 2, 'MOVE.B': 5, 'MOVE.L': 2, 'DBcc': 2, 'Bcc': 2, 'Scc': 15, 'ADDI': 2, 'SUBI': 5, 'ANDI': 4, 'ORI': 14, 'EORI': 13, 'CMPI': 6,
        'BTST#': 13, 'BCHG#': 13, 'BCLR#': 14, 'BSET#': 14, 'BTSTD': 13, 'BCHGD': 13, 'BCLRD': 14, 'BSETD': 14, 'NEGX': 10, 'CLR': 4, 'NEG': 5, 'NOT': 11, 'EXT': 8, 'NBCD': 9, 'SWAP': 9, 'TST': 15, 'TAS': 15, 'CHK': 6,
        'ASR': 2, 'ASL': 3, 'LSR': 5, 'LSL': 4, 'ROXR': 8, 'ROXL': 11, 'ROR': 10, 'ROL': 9, 'ASRm': 2, 'ASLm': 3, 'LSRm': 5, 'LSLm': 4, 'ROXRm': 8, 'ROXLm': 11, 'RORm': 10, 'ROLm': 9, 'ASR.L': 2, 'ASR.B': 2, 'LSR.L#': 5}
ok = 0; bad = []
for n, op in tests.items():
    r = rows(op)
    if r == [want[n]]: ok += 1
    else: bad.append(f'{n}={r}(exp.{want[n]})')
print(f'terms {len(terms)} (qualified {sum(1 for t in terms if t[4])}, corrected by the PLA {n_pla}, PLA divergences not adopted {n_div}) | test opcodes {len(tests)}: expected row {ok}, different {len(bad)}')
print('  different:', ' '.join(bad))
