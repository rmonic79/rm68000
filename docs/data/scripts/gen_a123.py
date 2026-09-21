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

# A1/A2/A3 decoder (Instruction Register Sequence Decoder, patent FIG. 5/21). Builds the table opcode -> (A1, A2, A3) over
# all 65536 values of IR from the die PLA table (mc68000_a123_die.tsv, from die_a123.py), corrected by the die listing
# (all.txt), and compares it with the transcription of FIG. 21B-21U (us4325121_fig21_a123.tsv: line = IR[15:12] from the
# title, row = IR[11:6] from the note, column = IR[5:0] from the heading).
# The FIG. 21 tables predate the final opcode map (DBcc in line 0111, i/r bit of the shifts inverted, ADDX/SUBX ...):
# for A1 the die wins; FIG. 21 supplies A2/A3 where the die value has no word; illegal -> A1 of TRAP1 + flag.
# Generates rtl/cpu_si/m68k_a123_si.sv. Usage: python gen_a123.py <repo_root> <galibert_dir>
import re, sys, collections
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
names = {}
for l in open(f'{G}/adr-names.txt'):
    m = re.match(r'^([0-9a-f]{3})\s+(\S+)', l.strip())
    if m: names[m.group(2).lower()] = int(m.group(1), 16)
ALIAS = {'hblw1': 'ablw1', 'abwwl': 'abww1', 'a12w0': 'aixw0', 'pnwl': 'pinw1', 'ixw01': 'aixw1', 'apsw1': 'adsw1', 'apsl1': 'adsl1',
         'dcnt1': 'dbcc1', '(trap)': 'trap1'}
rows = [l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_fig21_a123.tsv', encoding='utf-8') if not l.startswith('#')]
# Row corrections from the transcription notes (misprints of the figure, not of the transcription): sheet 37 (MOVE.W), row
# (RXA)+: A3 printed MMMW1 (the -(Ax) routine) in the whole row, expected MMIW1 as in sheets 35/36 (MOVE.B/L: MMIW1/MMIL1).
# Without the correction MOVE.W (A0)+,(A1)+ would write to A1-2. Key (figure, row, printed label) -> label.
ROWFIX = {('21F', '(RXA)+', 'MMMW1'): 'MMIW1'}
for r in rows:
    if len(r) > 6 and (r[0], r[2], r[6]) in ROWFIX: r[6] = ROWFIX[(r[0], r[2], r[6])]
COLPAT = {'RYD': ['000XXX'], 'RYA': ['001XXX'], '(RYA)': ['010XXX'], '(RYA)+': ['011XXX'], '-(RYA)': ['100XXX'], '(RYA)+d16': ['101XXX'],
          '(RYA)+(X)+d8': ['110XXX'], 'ABSW': ['111000'], 'ABSL': ['111001'], '(PC)+d16': ['111010'], '(PC)+(X)+d8': ['111011'],
          '#': ['111100'], 'UNUSED': ['111101', '11111X']}
LINE = {'21B': 0, '21C': 0, '21D': 1, '21E': 2, '21F': 3, '21G': 4, '21H': 4, '21I': 4, '21J': 5, '21K': 6, '21L': 7, '21M': 8, '21N': 9,
        '21P': 10, '21Q': 11, '21R': 12, '21S': 13, '21T': 14, '21U': 15}
# FIG. 21 labels = names of the patent era: first the App. H -> die mapping (us4325121_appH_die_map.tsv), then adr-names
pat2die = {}
for l in open(f'{D}/us4325121_appH_die_map.tsv', encoding='utf-8'):
    if l.startswith('#'): continue
    f = l.rstrip('\n').split('\t')
    if len(f) > 5 and f[5] in ('ok', 'ok_type', 'ambiguous', 'mismatch', 'no_trace'):
        try: pat2die[f[0].lower()] = int(f[2], 16)
        except ValueError: pass
def lab(x):
    x = x.lower().strip()
    if x in ('-', ''): return None
    x = ALIAS.get(x, x)
    if x in pat2die: return pat2die[x]
    return names.get(x, -1)
def match(pat, v, w):   # pat of w characters (0/1/X), v integer
    return all(p == 'X' or int(p) == ((v >> (w - 1 - i)) & 1) for i, p in enumerate(pat))
# row pattern IR[11:6] from the note: forms "IR[11:6]=XXX000 / 1XXX00", "IR[11:9]=XXX IR[8:6]=000", "XXX 100";
# multi-instruction row of 21I (IR[11:6]=111001): the column is the IR[5:0] code of the instruction (Motorola manual)
MULTI = {'RYD': ['00XXXX'], 'RYA': ['00XXXX'], '(RYA)': ['010XXX'], '(RYA)+': ['011XXX'], '-(RYA)': ['100XXX'], '(RYA)+d16': ['101XXX'],
         '(RYA)+(X)+d8': ['110110'], 'ABSW': ['110000'], 'ABSL': ['110001'], '(PC)+d16': ['110010'], '(PC)+(X)+d8': ['110011'], '#': ['110111'], 'UNUSED': ['110101']}
def rowpats(r):
    note = r[7]
    if 'multi-instruction row' in note: return ['111001']
    m = re.search(r'expected ([01X]{6})', note)   # pattern misprinted in the figure, corrected by the note (sheet 35: (RXA)+ printed XXX100, expected XXX011)
    if m: return [m.group(1)]
    m = re.search(r'IR\[11:9\]=([01X]{3}) IR\[8:6\]=([01X]{3})', note)
    if m: return [m.group(1) + m.group(2)]
    m = re.search(r'IR\[11:6\]=([01X /]+)', note)
    if not m: return ['XXXXXX']
    pats = [p.replace(' ', '') for p in m.group(1).split('/')]
    return [p for p in pats if len(p) == 6] or ['XXXXXX']
def colpats(r):
    if 'multi-instruction row' in r[7]: return MULTI[r[3]]
    if r[0] == '21K' and r[3] == 'RYA': return ['0001XX', '000X1X', '111XX1', '001XXX']   # Bcc.B: displacement != 0 (note of 21K)
    return COLPAT[r[3]]
def spec(r): return max(sum(1 for c in rp if c != 'X') for rp in rowpats(r))
tbl = {}; conf = 0; unres = collections.Counter(); cells = 0; spec_of = {}
for r in sorted(rows, key=spec):
    fig, col, a1, a2, a3 = r[0], r[3], r[4], r[5], r[6]
    if fig not in LINE or col not in COLPAT: continue
    v1, v2, v3 = lab(a1), lab(a2), lab(a3)
    for x, s in ((a1, v1), (a2, v2), (a3, v3)):
        if s == -1: unres[x.lower()] += 1
    cells += 1
    for rp in rowpats(r):
        for cp in colpats(r):
            for op in range(LINE[fig] << 12, (LINE[fig] + 1) << 12):
                if match(rp, (op >> 6) & 63, 6) and match(cp, op & 63, 6):
                    val = (v1 if v1 not in (None, -1) else 0, v2 if v2 not in (None, -1) else 0, v3 if v3 not in (None, -1) else 0, v1 is not None)
                    if op in tbl and tbl[op][:3] != val[:3] and spec_of[op] == spec(r): conf += 1
                    tbl[op] = val; spec_of[op] = spec(r)
# --- A1 from the die (a1-dump.txt)
ent = []
for l in open(f'{G}/a1-dump.txt'):
    m = re.match(r'\s*(\d+): ([0-9a-f]{3}) (\S+)\s+([0-9a-f]{4}) ([0-9a-f]{4}) ([0-9a-f]{4}) (\d) ?(.*)$', l.rstrip('\n'))
    if m: ent.append((int(m.group(2), 16), m.group(3), int(m.group(4), 16), int(m.group(5), 16), int(m.group(7))))
op_ent = [e for e in ent if e[4] == 0]; ea_ent = [e for e in ent if e[4] in (1, 2)]
# exact entries outside the (row, column) model: Bcc/BSR with 16-bit displacement (d8 = 0) and the 4E7x group (NOP/RESET/STOP/RTE/RTR/RTS/TRAPV)
op_ent = [(0x0a9, 'bsrw1', 0x6100, 0xffff, 0), (0x068, 'bbcw1', 0x6000, 0xf0ff, 0), (0x3a6, 'rset1', 0x4e70, 0xffff, 0), (0x363, 'b', 0x4e71, 0xffff, 0),
          (0x3a2, 'stop1', 0x4e72, 0xffff, 0), (0x12a, 'rtr1', 0x4e73, 0xfffb, 0), (0x126, 'rts1', 0x4e75, 0xffff, 0), (0x06d, 'trpv1', 0x4e76, 0xffff, 0)]
TRAP1 = names['trap1']
names_rev = {v: k for k, v in names.items()}
def is_long(op):   # long class of the EA entries: MOVE.L/MOVEA.L, or size field 10 in lines 0000/0100/1000/1001/1011/1100/1101
    return ((op >> 12) == 2) or (((op >> 12) in (0, 4, 8, 9, 11, 12, 13)) and ((op >> 6) & 3) == 2)
final = {}; src_cnt = collections.Counter(); agree = disagree = 0; dis_ex = collections.Counter(); ea_rule_bad = 0
# --- die table (scripts/die_a123.py: A1 PLA + A2/A3 PLA + illegal/privilege rows): it is THE source; FIG. 21 only for comparison ---
DIE = {}
for l in open(f'{D}/mc68000_a123_die.tsv', encoding='utf-8'):
    if l.startswith('#'): continue
    q = l.rstrip('\n').split('\t')
    DIE[int(q[0], 16)] = (int(q[1], 16), int(q[2], 16), int(q[3], 16), int(q[4]), int(q[5]), int(q[6]), int(q[7]))
# --- the die LISTING (all.txt: simulation of the real PLAs) corrects the table where they differ: A1 = first word of the instance,
# A2/A3 = word that follows "jump a2"/"jump a3"; the instance "### opcode mask" applies to the opcodes with (op & mask) == opcode.
# Examples: A1 of the memory .L forms of NEGX/CLR/NEG/NOT/TST (.W entries in the table); A3 of TST.L mem (288 -> 3cb) and EOR mem (091 -> 299).
_inst = []; _cur = None; _words = []
def _flush():
    if _cur is None or not _words: return
    a1 = _words[0][0]; a2 = a3 = None
    for i, (a, j) in enumerate(_words):
        if j == 'a2' and i + 1 < len(_words): a2 = _words[i + 1][0]
        if j == 'a3' and i + 1 < len(_words): a3 = _words[i + 1][0]
    _inst.append((_cur[0], _cur[1], a1, a2, a3))
for l in open(f'{G}/all.txt', encoding='utf-8', errors='replace'):
    m = re.match(r'^### ([0-9a-f]{4}) ([0-9a-f]{4}) ', l)
    if m: _flush(); _cur = (int(m.group(1), 16), int(m.group(2), 16)); _words = []; continue
    m = re.match(r'^([0-9a-f]{3}) (\S+):', l)
    if m: _words.append([int(m.group(1), 16), None]); continue
    m = re.match(r'^\s+jump (a[123]|[0-9a-f]{3})', l)
    if m and _words: _words[-1][1] = m.group(1)
_flush()
def _matches(opc, mask):
    free = (~mask) & 0xffff; sub = free
    while True:
        yield opc | sub
        if sub == 0: break
        sub = (sub - 1) & free
lst_fix = collections.Counter()
for opc, mask, a1, a2, a3 in _inst:
    for op in _matches(opc, mask):
        d = list(DIE[op])
        if a1 != d[0]: d[0] = a1; lst_fix['a1'] += 1
        if a2 is not None and a2 != d[1]: d[1] = a2; lst_fix['a2'] += 1
        if a3 is not None and a3 != d[2]: d[2] = a3; lst_fix['a3'] += 1
        DIE[op] = tuple(d)
print('die listing (all.txt): instances', len(_inst), '| corrections to the PLA table', dict(lst_fix))
# BCLR #imm,(An) (0890-0897): the decoded A2 PLA (and the listing, which derives from it) give 00b = LONG pair adrl1/adrl2 (two reads,
# AT = An+2: the write would go to An+2); FIG. 21B 'STATIC BCLR (RYA)' = ADRW1 (006), like static BSET/BCHG and dynamic BCLR
# in the same PLA, and the manual (16 clocks: one read).
for op in range(0x0890, 0x0898): DIE[op] = (DIE[op][0], 0x006, DIE[op][2]) + DIE[op][3:]
priv_set = set(); VALID = set(names.values())   # addresses with a word in the die (adr-names)
for op in range(65536):
    da1, da2, da3, dill, dpriv, dla, dlf = DIE[op]
    t = tbl.get(op); fig_a1 = t[0] if t and t[3] else None
    # illegal ('i' PLA of the die) or no A1 (lines A/F included: the exception block tells them apart from IR) -> TRAP1 entry + flag
    if dill or da1 == 0x3ff: a1 = 0x3ff; src = 'illegal'   # 3ff = illg1 (illegal/privilege/line A-F entry)
    else:
        a1 = da1; src = 'die'
        if fig_a1 is not None:
            if fig_a1 == a1: agree += 1
            else: disagree += 1; dis_ex[(names_rev.get(a1, '%03x' % a1), '%03x' % fig_a1)] += 1
    # A2/A3 of the die without a word (091 for EOR Dn,<ea>: 512 opcodes; 288 for 4a80..): misread of the A2/A3 PLA in die_a123.py ->
    # FIG. 21 wins (EOR (ea): MORW1 = 299, like the die trace 'eor.w dd ais': adrw2 -> a3 = 299).
    fa2 = t[1] if t and t[1] else None; fa3 = t[2] if t and t[2] else None
    a2 = da2 if (da2 != 0x3ff and da2 in VALID) else (fa2 if fa2 else TRAP1)
    a3 = da3 if (da3 != 0x3ff and da3 in VALID) else (fa3 if fa3 else TRAP1)
    if dpriv: priv_set.add(op)
    src_cnt[src] += 1
    final[op] = (a1, a2, a3, src)
# privilege expression (Motorola manual: MOVE to SR, ORI/ANDI/EORI to SR, RESET, STOP, RTE, MOVE USP), compared with the die below
def priv_expr(op):
    return ((op >> 6) == 0x11b) or op in (0x4e70, 0x4e72, 0x4e73, 0x007c, 0x027c, 0x0a7c) or (op >> 4) == 0x4e6
priv_calc = {op for op in range(65536) if priv_expr(op)}
if priv_calc != priv_set:
    print('PRIVILEGE: expression != die; die only:', ['%04x' % o for o in sorted(priv_set - priv_calc)][:20], 'expression only:', ['%04x' % o for o in sorted(priv_calc - priv_set)][:20])
n_valid = sum(1 for v in final.values() if v[3] != 'illegal'); n_ill = 65536 - n_valid
nl = '\n'
# --- two-level model: row class (IR[15:6]) x column class (EA mode, with register for mode 7) -> residual A1, A2, A3.
# Checked: no cell (row, column class) with more than one non-null value (0 = unused cell = don't care).
def colc(op):
    m = (op >> 3) & 7; r = op & 7
    return m if m < 7 else 7 + min(r, 5)
hit1 = [next((e[0] for e in op_ent if (op & e[3]) == e[2]), None) for op in range(65536)]   # only the 8 exact entries; the rest in the table
cell = {}
for op in range(65536):
    a1, a2, a3, src = final[op]
    k = (op >> 6, colc(op))
    c = cell.setdefault(k, [set(), set(), set()])
    if hit1[op] is None: c[0].add(a1)
    if a2: c[1].add(a2)
    if a3: c[2].add(a3)
amb = sum(1 for c in cell.values() for v in c if len(v) > 1)
assert amb == 0, f'ambiguous cells in the two-level model: {amb}'
def cellval(k, i, dflt):
    v = cell.get(k, [set(), set(), set()])[i]
    return next(iter(v)) if v else dflt
rowsig = {}
for r in range(1024):
    rowsig[r] = tuple((cellval((r, c), 0, 0x3ff), cellval((r, c), 1, TRAP1), cellval((r, c), 2, TRAP1)) for c in range(13))
classes = {}; cls_of = {}
for r in range(1024): cls_of[r] = classes.setdefault(rowsig[r], len(classes))
NCLS = len(classes); CW = max(1, (NCLS - 1).bit_length())
final2 = {}
for op in range(65536):
    sg = rowsig[op >> 6][colc(op)]
    a1 = hit1[op] if hit1[op] is not None else sg[0]
    a2, a3 = sg[1], sg[2]
    ill = (a1 == 0x3ff)
    final2[op] = (a1, a2, a3, ill)
chk = sum(1 for op in range(65536) if final2[op][0] != final[op][0] or (final[op][1] and final2[op][1] != final[op][1]) or (final[op][2] and final2[op][2] != final[op][2]))
assert chk == 0, f'two-level model differs from the table: {chk}'
n_ill = sum(1 for v in final2.values() if v[3])
import os
if os.environ.get('A123_DUMP'):
    with open(os.environ['A123_DUMP'], 'w', newline=nl) as fd:
        for op in range(65536): fd.write('%04x\t%03x\t%03x\t%03x\t%d\n' % (op, final2[op][0], final2[op][1], final2[op][2], int(final2[op][3])))
# A2 don't care per cell: does at least one opcode of the cell read it? (mc68000_a123_need.tsv from a123_need.py)
need2 = {}
for l in open(f'{D}/mc68000_a123_need.tsv', encoding='utf-8'):
    if l.startswith('#'): continue
    o, v = l.split('\t'); need2[int(o, 16)] = v.strip() == '1'
assert len(need2) == 65536
cell2 = set()
for op in range(65536):
    if need2[op]: cell2.add((cls_of[op >> 6], colc(op)))
with open(f'{R}/rtl/cpu_si/m68k_a123_si.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write("// m68k_a123_si.sv - decoder of the A1/A2/A3 start addresses from the IR (US 4,325,121 FIG. 5/21 + PLA tables of the die). GENERATED by gen_a123.py." + nl)
    f.write("// Two-level structure (like the die PLA): A1 = exact opcode entries (m[] below) or table; A2/A3 = table" + nl)
    f.write(f"// [row class (IR[15:6] -> {NCLS} classes)][column class (EA mode, register for mode 7: 13)]." + nl)
    # The row class can arrive ALREADY REGISTERED (cls_in): in the core m68k_ir_si computes it from
    # irc and registers it on the same edge on which ir <= irc, so it equals cls_of(ir) in the same
    # cycle. This takes the levels of the row-class function off the head of every ir -> a1/illegal
    # path. The rest of the PLA (table and illegal compare) is unchanged.
    assert CW == 7, 'CW = %d: update the width of cls_q in m68k_ir_si.sv' % CW
    f.write("module m68k_a123_si #(parameter bit EXT_CLS = 1'b0) (" + nl + '  input  logic [15:0] ir,' + nl + '  /* verilator lint_off UNUSEDSIGNAL */' + nl + f'  input  logic [{CW-1}:0]  cls_in,   // registered row class (used only with EXT_CLS)' + nl + '  /* verilator lint_on UNUSEDSIGNAL */' + nl
 + '  output logic [9:0]  a1, a2, a3,' + nl + '  output logic        illegal,' + nl + '  output logic        priv          // privileged instruction (Motorola manual list): exception 8 in user mode' + nl + ');' + nl)
    f.write(f'  logic [{CW-1}:0] cls, cls_c; logic [3:0] colc; logic [9:0] a1t; logic hit;' + nl)
    f.write('  assign cls = EXT_CLS ? cls_in : cls_c;   // EXT_CLS = 1: row class computed from irc and registered by m68k_ir_si' + nl)
    f.write('  logic ill_m;   // the opcode entries whose value is 3ff (if any)' + nl)
    # ILL_T: `illegal` as a DATA bit of the table (ill_t), instead of the compare `a1t == 10'h3ff`,
    # a ten-input AND that waits for ALL the bits of a1t. As a bit of the same `case` it is born at
    # the same depth as a1t. Identical by construction: ill_t is 1 exactly in the cells with
    # a1t = 3ff, and the cells left at the default have a1t = TRAP1 (0x1d0), hence 0.
    # Off: the table bit becomes an eleventh PLA output with its own OR tree over all the cells,
    # which costs more area than it saves.
    ILL_T = False
    # ILL3: `3ff` recognized from the minimal subset of a1t bits that tells it apart from the other
    # table values (search and exhaustive cross-check below). Off: no overall timing gain.
    ILL3 = False
    if ILL_T: f.write('  logic ill_t;' + nl)
    f.write("  assign colc = (ir[5:3] != 3'd7) ? {1'b0, ir[5:3]} : (ir[2:0] == 3'd0) ? 4'd7 : (ir[2:0] == 3'd1) ? 4'd8 : (ir[2:0] == 3'd2) ? 4'd9 : (ir[2:0] == 3'd3) ? 4'd10 : (ir[2:0] == 3'd4) ? 4'd11 : 4'd12;" + nl)
    f.write('  // row class: 10-input function (complete case)' + nl + '  always_comb begin' + nl + f"    cls_c = {CW}'d0;" + nl + '    case (ir[15:6])' + nl)
    for r in range(1024): f.write(f"      10'd{r}: cls_c = {CW}'d{cls_of[r]};" + nl)
    f.write(f"      default: cls_c = {CW}'d0;" + nl + '    endcase' + nl + '  end' + nl)
    f.write('  // tables [class, column] -> residual A1, A2, A3 (unused cells = TRAP1)' + nl + '  always_comb begin' + nl)
    f.write(f"    // A2 = 'x where no opcode of the cell reads it (a123_need.py): don't care for the minimizer, 0 in simulation" + nl)
    f.write(f"    a1t = 10'h{TRAP1:03x}; a2 = 'x; a3 = 10'h{TRAP1:03x};" + (" ill_t = 1'b0;" if ILL_T else '') + nl + '    case ({cls, colc})' + nl)
    for sg, c in classes.items():
        for col in range(13):
            v1, v2, v3 = sg[col]
            if (v1, v2, v3) == (TRAP1, TRAP1, TRAP1): continue
            a2s = f"10'h{v2:03x}" if (c, col) in cell2 else "'x"
            ilt = (" ill_t = 1'b1;" if v1 == 0x3ff else '') if ILL_T else ''
            f.write(f"      {{{CW}'d{c}, 4'd{col}}}: begin a1t = 10'h{v1:03x}; a2 = {a2s}; a3 = 10'h{v3:03x};{ilt} end" + nl)
    f.write('      default: ;' + nl + '    endcase' + nl)
    f.write('  end' + nl)
    f.write("  // A1: exact opcode entries (where two masks overlap, the entry with the lower index wins: & ~m[j]), otherwise the table" + nl)
    # opcode entries: FLAT logic (AND-OR): m_i = match; for the few pairs of masks that intersect the lower index wins
    ov = {}
    for i in range(len(op_ent)):
        for j in range(i):
            cm = op_ent[i][3] & op_ent[j][3]
            if (op_ent[i][2] & cm) == (op_ent[j][2] & cm) and op_ent[i][0] != op_ent[j][0]: ov.setdefault(i, []).append(j)
    f.write(f'  logic [{len(op_ent)-1}:0] m;' + nl)
    for i, e in enumerate(op_ent):
        excl = ''.join(f' & ~m[{j}]' for j in ov.get(i, []))
        f.write(f"  assign m[{i}] = ((ir & 16'h{e[3]:04x}) == 16'h{e[2]:04x}){excl};  // {e[1]}" + nl)
    f.write('  assign hit = |m;' + nl)
    # One level after m[]: the OR of the opcode entries and the mux with a1t merge into a single
    # AND-OR. Identical by construction: with hit true the second term is 0 and the OR of the
    # entries remains; with hit false the OR is 0 and a1t remains.
    f.write('  assign a1 = '
            + ' | '.join(f"({{10{{m[{i}]}}}} & 10'h{e[0]:03x})" for i, e in enumerate(op_ent))
            + ' | ({10{~hit}} & a1t);' + nl)
    print('opcode entries with precedence exclusion:', {op_ent[i][1]: [op_ent[j][1] for j in v] for i, v in ov.items()})
    ill_idx = [i for i, e in enumerate(op_ent) if e[0] == 0x3ff]
    # EXHAUSTIVE EQUIVALENCE, in seconds: for all 65,536 opcodes the emitted form
    #   illegal = hit ? ill_m : (a1t == 3ff)
    # must give the same bit as the reference form
    #   illegal = (a1 == 3ff).
    # hit and ill_m are rebuilt from the same opcode entries that the generator emits, with the
    # same precedence rule (lower index wins).
    nd = 0
    for op in range(65536):
        mm = []
        for i, e in enumerate(op_ent):
            if (op & e[3]) != e[2]: mm.append(0); continue
            mm.append(0 if any(mm[j] for j in ov.get(i, [])) else 1)
        hit_p = any(mm)
        illm_p = any(mm[i] for i in ill_idx)
        sg = rowsig[op >> 6][colc(op)]
        a1t_p = sg[0]
        new_ill = illm_p if hit_p else (a1t_p == 0x3ff)
        old_ill = (final2[op][0] == 0x3ff)
        if new_ill != old_ill: nd += 1
    assert nd == 0, f'EXHAUSTIVE EQUIVALENCE FAILED on {nd} opcodes'
    print(f'exhaustive equivalence: 65536 opcodes, 0 differences (illegal {sum(1 for op in range(65536) if final2[op][0] == 0x3ff)})')
    f.write('  assign ill_m = ' + (' | '.join(f'm[{i}]' for i in ill_idx) if ill_idx else "1'b0") + ';' + nl)
    f.write("  // illegal compares a1t, the output of the TABLE, instead of a1: this way it does not wait for" + nl)
    f.write('  // the 10-bit mux of the opcode entries on the ir -> illegal path.' + nl)
    f.write('  // Equivalent by construction to (a1 == 3ff): no opcode entry is 3ff (ill_m = 0), so with hit true a1' + nl)
    f.write("  // cannot be 3ff; with hit false a1 IS a1t. No extra mux: a1t already exists." + nl)
    if ILL_T:
        f.write('  assign illegal = hit ? ill_m : ill_t;' + nl)
    elif not ILL3:
        f.write("  assign illegal = hit ? ill_m : (a1t == 10'h3ff);   // 3ff = illg1: illegal/privilege/line A-F" + nl)
    else:
        # `a1t == 3ff` is a TEN-input AND: it waits for all the bits of a1t, the last one after
        # several levels of OR. But a1t takes only the values of the table: the minimal subset of
        # bits that tells 3ff apart from them is enough.
        vals = set(sg[col][0] for sg in classes for col in range(13)) | {TRAP1}
        from itertools import combinations as _cmb
        chosen = None
        for k in range(1, 11):
            for bits in _cmb(range(10), k):
                if all(any(((v >> b) & 1) == 0 for b in bits) for v in vals if v != 0x3ff):
                    chosen = bits; break
            if chosen: break
        assert chosen, 'no subset of bits tells 3ff apart'
        for v in vals:
            assert all((v >> b) & 1 for b in chosen) == (v == 0x3ff), 'value %03x' % v
        print('illegal from %d bits %s over %d values of a1t: cross-check without exceptions'
              % (len(chosen), sorted(chosen, reverse=True), len(vals)))
        f.write('  assign illegal = hit ? ill_m : ('
                + ' & '.join('a1t[%d]' % b for b in sorted(chosen, reverse=True))
                + ');   // 3ff recognized from only the bits that tell it apart' + nl)
    print('opcode entries whose value is 3ff:', len(ill_idx), 'of', len(op_ent))
    f.write("  assign priv = (ir[15:6] == 10'h11b) | (ir == 16'h4e70) | (ir == 16'h4e72) | (ir == 16'h4e73) | (ir == 16'h007c) | (ir == 16'h027c) | (ir == 16'h0a7c) | (ir[15:4] == 12'h4e6);   // Motorola manual list; the generator compares it with the 'p' set of the die PLA" + nl)
    f.write('endmodule' + nl)
    # the same class function as a module of its own, to compute it from irc in m68k_ir_si
    f.write(nl + '// row class alone: the SAME table as the case above, for m68k_ir_si (from irc).' + nl)
    f.write(f'module m68k_a123_cls (input logic [9:0] hi, output logic [{CW-1}:0] cls);' + nl + '  always_comb begin' + nl + f"    cls = {CW}'d0;" + nl + '    case (hi)' + nl)
    for r in range(1024): f.write(f"      10'd{r}: cls = {CW}'d{cls_of[r]};" + nl)
    f.write(f"      default: cls = {CW}'d0;" + nl + '    endcase' + nl + '  end' + nl + 'endmodule' + nl)
print(f'two-level decoder: row classes {NCLS} ({CW} bits) | non-empty table cells {sum(1 for sg in classes for col in range(13) if sg[col] != (TRAP1, TRAP1, TRAP1))} | die opcode entries {len(op_ent)} | illegal {n_ill}')
print(f'A1 FIG.21 vs die where both exist: equal {agree}, different {disagree} | A1 from: {dict(src_cnt)} | long rule vs FIG.21 different {ea_rule_bad} | unresolved labels {dict(unres)}')
