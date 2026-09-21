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

# Bus-side registers AT (32), AOB (32), DOB (16), DBIN (16) that load ONLY the driven bus sections.
# Load rules: docs/silicon-datapath.md §6. For each destination atom 'D<-X' the generator produces
# the load specifications "part=source" (source = bus section with an optional fallback through the L-D coupler,
# or external port au/alu/edb) and emits:
#  - rtl/cpu_si/m68k_regs_bd.sv  (GENERATED): dec + bus sections + ports -> at, aob, dob, dbin (registers, enable en)
#  - sim/m68k_regs_top.sv (GENERATED): control store + decoder + fabric + registers (testbench top)
# Usage: python gen_regs.py <repo_root>
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
R = sys.argv[1]; D = f'{R}/docs/data'
dec = {}; codes = {}
for l in open(f'{R}/rtl/cpu_si/m68k_nano_dec_pkg.sv'):
    m = re.match(r'localparam int ND_(\w+)_LSB = (\d+), ND_\w+_W = (\d+), ND_\w+_ONEHOT = (\d);  // (.*)$', l.rstrip('\n'))
    if m: dec[m.group(5)] = (int(m.group(2)), int(m.group(3)), int(m.group(4)))
    m = re.match(r'localparam int ND_(\w+)__(\w+) = (\d+);  // (.*)$', l.rstrip('\n'))
    if m: codes[m.group(4)] = int(m.group(3))
atoms = set(); words = {}
for l in open(f'{D}/mc68000_nano_encoding.tsv', encoding='utf-8'):
    f = l.rstrip('\n').split('\t')
    if re.match(r'^[0-9a-f]{3}\t', l): A = {x for x in f[4].split(' ; ') if x}; atoms |= A; words[int(f[0], 16)] = A
PORTS = {'au': 32, 'alu': 16, 'edb': 16}
def bus_src(b, sec, fb=None):   # source = section 'sec' of bus b if driven, otherwise fallback 'fb'
    s = f'{b}.{sec}'
    return s + (f'|{b}.{fb}' if fb else '')
def rule(at):   # -> list of "part=source" or None
    m = re.match(r'^(at|ath|atl|aob|dob|dobh|dobl|dbin|dbinh|dbinl)<-(.+)$', at)
    if not m: return None
    dst, X = m.group(1), m.group(2)
    if X in PORTS:
        w = PORTS[X]
        if dst == 'aob': return [f'aob_h={X}:31:16', f'aob_l={X}:15:0']
        if dst == 'dob': return [f'dob_hi={X}:15:8', f'dob_lo={X}:7:0']
        if dst == 'dbin': return [f'dbin_hi={X}:15:8', f'dbin_lo={X}:7:0']
        if dst == 'dbinh': return [f'dbin_hi={X}:15:8']
        if dst == 'dbinl': return [f'dbin_lo={X}:7:0']
        raise Exception(at)
    mb = re.match(r'^(ab|db|\*)(h|l|d|e|\*)?$', X)
    if not mb: raise Exception(at)
    b, suf = mb.group(1), mb.group(2) or ''
    buses = ['ab', 'db'] if b == '*' else [b]
    def src(sec, fb=None): return '|'.join(bus_src(bb, sec, fb) for bb in buses)
    out = []
    if dst in ('at', 'aob'):                                  # 32-bit address-side register: H from H, L from L (fallback D)
        p = dst if dst == 'at' else 'aob'
        if suf == 'h': out = [f'{p}_h=' + src('h')]
        elif suf == 'l': out = [f'{p}_l=' + src('l')]
        elif suf == 'd': out = [f'{p}_l=' + src('d')]
        elif suf == '*': out = [f'{p}_h=' + src('h'), f'{p}_l=' + src('d', 'l')]
        else: out = [f'{p}_h=' + src('h'), f'{p}_l=' + src('l', 'd')]
    # ath<-X: H, fallback L and then D (couplers abd->abl->abh): App. H ablw1 "(dbin) -> ab -> ath" loads into ATH the high word
    # of the long absolute address read into DBIN (DATA section). Without the fallback ATH would not load
    # (CLR.W $C00050.L would access 0x50).
    elif dst == 'ath': out = ['at_h=' + '|'.join(f'{bb}.h|{bb}.l|{bb}.d' for bb in buses)]
    elif dst == 'atl':
        out = ['at_l=' + (src('l') if suf == 'l' else src('d') if suf == 'd' else src('d', 'l') if suf == '*' else src('l', 'd'))]
    elif dst in ('dob', 'dobh', 'dobl'):                      # data-side register: word from D (fallback L)
        # dob<-X without suffix: D, fallback L then H (couplers): bsrw2 "(ath) -> ab -> dob" drives only ABH
        w = src('l') if suf == 'l' else src('h') if suf == 'h' else src('d') if suf == 'd' else '|'.join(f'{bb}.d|{bb}.l|{bb}.h' for bb in buses)
        if dst in ('dob', 'dobh'): out.append('dob_hi=' + w + '#15:8')
        if dst in ('dob', 'dobl'): out.append('dob_lo=' + w + '#7:0')
    else: raise Exception(at)
    return out
rules = {at: rule(at) for at in sorted(atoms) if rule(at)}
nl = '\n'
PARTS = [('at_h', 16), ('at_l', 16), ('aob_h', 16), ('aob_l', 16), ('dob_hi', 8), ('dob_lo', 8), ('dbin_hi', 8), ('dbin_lo', 8)]
def sv_src(s):   # "ab.l|ab.d#15:8" -> list (cond, expression); "au:31:16" -> [(None, expr)]
    if ':' in s and '.' not in s:
        p, hi, lo = s.split(':'); return [(None, f'src_{p}[{hi}:{lo}]')]
    byte = None
    if '#' in s: s, byte = s.split('#')
    out = []
    for t in s.split('|'):
        b, sec = t.split('.')
        e = f'{b}_{sec}'
        if byte: hi, lo = byte.split(':'); e = f'{b}_{sec}[{hi}:{lo}]'
        out.append((f'{b}_{sec}_drv', e))
    return out
with open(f'{R}/rtl/cpu_si/m68k_regs_bd.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// m68k_regs_bd.sv - bus-side registers AT/AOB/DOB/DBIN of the silicon-structured datapath.' + nl)
    f.write('// GENERATED by docs/data/scripts/gen_regs.py (rules: docs/silicon-datapath.md section 6), do NOT edit by hand.' + nl)
    f.write('// Each register part loads ONLY if the source bus section is driven (L<->D fallback through the coupler).' + nl)
    f.write('import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;' + nl + 'module m68k_regs_bd (' + nl)
    f.write('  input  logic clk, rst_n, en,' + nl + '  /* verilator lint_off UNUSEDSIGNAL */' + nl + '  input  logic [NDEC_W-1:0] dec,' + nl + '  /* verilator lint_on UNUSEDSIGNAL */' + nl)
    f.write('  input  logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,' + nl)
    f.write('  input  logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,' + nl)
    f.write('  // the DOB reads the NARROW copy of the AB sections (subset proven by the census in' + nl)
    f.write('  // gen_bus_sec.py): same value in the words that load the DOB from AB, shorter tree.' + nl)
    f.write('  input  logic [15:0] ab_dob,' + nl)
    f.write('  input  logic        ab_dob_drv,' + nl)
    f.write('  input  logic [31:0] src_au,' + nl + '  input  logic [15:0] src_alu, src_edb,' + nl)
    f.write('  output logic [31:0] at, aob,' + nl + '  output logic [15:0] dob, dbin,' + nl)
    f.write('  output logic [31:0] aob_n,        // value of AOB being loaded in this word (for the bus cycle started in the same word)' + nl)
    f.write('  output logic [15:0] dob_n,        // same for DOB (data to write)' + nl)
    f.write('  output logic [15:0] dbin_n        // DBIN being loaded (transparent latch of the die: (dbin) in the same word as dbin<-edb sees the data just arrived)' + nl + ');' + nl)
    f.write('  logic [15:0] at_h, at_l, aob_h, aob_l; logic [7:0] dob_hi, dob_lo, dbin_hi, dbin_lo;' + nl)
    f.write('  assign at = {at_h, at_l}; assign aob = {aob_h, aob_l}; assign dob = {dob_hi, dob_lo}; assign dbin = {dbin_hi, dbin_lo};' + nl)
    # next value of AOB/DOB: same load logic, combinational
    f.write('  logic [15:0] aob_h_n, aob_l_n; logic [7:0] dob_hi_n, dob_lo_n, dbin_hi_n, dbin_lo_n;' + nl)
    f.write('  assign aob_n = {aob_h_n, aob_l_n}; assign dob_n = {dob_hi_n, dob_lo_n}; assign dbin_n = {dbin_hi_n, dbin_lo_n};' + nl)
    # --- groups per data: part -> {spec (chain of sections) -> [conditions of the atoms]} ; structural check per word
    import collections as _c
    def cond_of(at):
        L = at.split('<-')[0] + '<-'; lsb, dw, oh = dec[L]; c = codes[at]
        return f'dec[{lsb + c}]' if oh else f'dec[{lsb} +: {dw}] == {dw}\'d{c}'
    grouped = {}; grouped_atoms = {}
    for at, specs in rules.items():
        for sp in specs:
            part, s2 = sp.split('='); grouped.setdefault(part, _c.OrderedDict()).setdefault(s2, []).append(cond_of(at))
            grouped_atoms.setdefault(part, {}).setdefault(s2, []).append(at)
    # bus sections driven in a word: same rules as gen_bus_sec.rule() (suffix h/l/d/e/*)
    import re as _re
    _W32 = {'at','pc','rx','rxa','ry','rya','sp','au','aob'}
    def _secs_of(at):
        m = _re.match(r'^(ab|db)(h|l|d|e|[*])?<-(.+)$', at)
        if not m: return None
        bus, suf, S = m.group(1), m.group(2) or '', m.group(3).strip('()')
        w = 32 if S in _W32 else 16
        nat = 'h' if S.endswith('h') else 'l' if (S.endswith('l') and not S.endswith('dl')) or S == 'ftu' else 'd'
        if suf == 'h': x = {'h'}
        elif suf == 'l': x = {'l'}
        elif suf == 'd': x = {'d'}
        elif suf == 'e': x = {'l', 'h'}
        elif suf == '': x = {'h', 'l'} if w == 32 else {nat}
        else: x = {'h', 'l', 'd'} if w == 32 else {nat}
        return bus, x
    def _driven(A):   # word -> {'ab_h_drv', 'ab_l_drv', ...} always true in that word
        out = set()
        for x in A:
            r = _secs_of(x)
            if r: out |= {f'{r[0]}_{z}_drv' for z in r[1]}
        return out
    _census = {}   # (part, s2) -> ('first' | 'one' | None)
    for part, gs in grouped_atoms.items():
        for s2, ats in gs.items():
            W = [a_ for a_, A in words.items() if set(ats) & set(A)]
            alts = [c for c, _ in sv_src(s2) if c and c.endswith('_drv')]
            if not W or not alts: _census[(part, s2)] = None; continue
            drv = [_driven(words[a_]) for a_ in W]
            if all(alts[0] in d for d in drv): _census[(part, s2)] = 'first'
            elif all(any(c in d for c in alts) for d in drv): _census[(part, s2)] = 'one'
            else: _census[(part, s2)] = None
    print('regs: census of the driven sections per group:', {f'{p}={s}': v for (p, s), v in _census.items()})
    conflicts = 0
    for a_, A in words.items():
        per = {}
        for at in A:
            if at in rules:
                for sp in rules[at]:
                    part, s2 = sp.split('='); per.setdefault(part, set()).add(s2)
        conflicts += sum(1 for v in per.values() if len(v) > 1)
    assert conflicts == 0, f'words with two different data on the same part: {conflicts}'
    for part, groups in grouped.items():
        for k, (s2, conds) in enumerate(groups.items()):
            f.write(f'  logic sel_{part}_{k}; assign sel_{part}_{k} = ' + ' | '.join(f'({c})' for c in conds) + f';   // {s2}' + nl)
    # PROOF: every word with DOB<-AB has at least one active AB source.
    DOB_AB = {'dob<-ab', 'dob<-ab*', 'dobh<-ab', 'dobl<-ab', 'dobl<-ab*'}
    import re as _re
    _n = 0
    for _a, _A in words.items():
        if not (DOB_AB & set(_A)): continue
        _ab = [x for x in _A if _re.match(r'^ab(h|l|d|e|[*])?<-', x)]
        assert _ab, 'word %03x: DOB<-AB without any AB source' % _a
        _n += 1
    print('regs: words with DOB<-AB proven: %d (always with an AB source: the flag is redundant)' % _n)
    def dob_narrow(alts):
        # the DOB takes a single wire from the AB: every word that loads it from the AB has only one
        # source, so the sections it drives carry the same value and the priority chain
        # d -> l -> h disappears (one level less, and only the allowed sources in the cone).
        # And the ab_dob_drv flag is NOT needed: exhaustive census below - all the words
        # with DOB<-AB drive the AB (none with zero sources), so when sel_dob_* is high
        # ab_dob_drv is high by construction. As an 8-input OR IN SERIES with the selection it would sit on
        # the path dec_q -> sel_ab_h_2 -> ab_dob_drv -> dob_n -> data_out. Without it only the data remains.
        out, seen = [], False
        for cnd, e in alts:
            if cnd in ('ab_h_drv', 'ab_l_drv', 'ab_d_drv'):
                if seen: continue
                seen = True
                slice_ = e[e.index('['):] if '[' in e else ''
                out.append(('ab_dob_drv', 'ab_dob' + slice_))
            else:
                out.append((cnd, e))
        if len(out) == 1 and out[0][0] == 'ab_dob_drv': out = [(None, out[0][1])]
        return out
    def aob_from_au(part, alts):
        # AOB loaded from a DB section with db<-au: it takes src_au directly. Same value
        # (with db<-au the sections db_h/db_l come exactly from src_au), but the address does not cross
        # the fabric: one level less on the cone that leads to the AOB.
        if part not in ('aob_h', 'aob_l') or not alts or not alts[0][0] or not alts[0][0].startswith('db_'):
            return alts
        meta = 'src_au[31:16]' if part.endswith('_h') else 'src_au[15:0]'
        return [('dec[ND_DB_FROM_LSB + ND_DB_FROM__AU]', meta)] + alts
    def emit_part(part, lhs, op):
        groups = grouped.get(part, {})
        for k, (s2, conds) in enumerate(groups.items()):
            alts = sv_src(s2)
            if part.startswith('dob'): alts = dob_narrow(alts)
            alts = aob_from_au(part, alts)
            # the drv flags do NOT decide the ENABLE when the census proves that in every word of the
            # group the section of the first branch is always driven ('first' -> plain assignment) or that
            # at least one of the listed sections always is ('one' -> last branch without condition:
            # the flop enable becomes the selection alone, one level from dec; the flags stay only in the
            # data mux). Exact: the 'no load' branch is unreachable in those words. This takes the drv
            # flag off the path dec_pq -> sel_db_d -> db_d_drv -> at_h enable.
            mode = _census.get((part, s2))
            f.write(f'      {"if" if k == 0 else "else if"} (sel_{part}_{k}) begin ')
            if alts[0][0] is None: f.write(f'{lhs} {op} {alts[0][1]};')
            elif mode == 'first' and alts[0][0].endswith('_drv'): f.write(f'{lhs} {op} {alts[0][1]};   /* always driven */')
            elif mode == 'one' and all(c.endswith('_drv') for c, _ in alts):
                f.write(' else '.join(f'if ({cnd}) {lhs} {op} {e};' for cnd, e in alts[:-1]) + (' else ' if len(alts) > 1 else '') + f'{lhs} {op} {alts[-1][1]};   /* always driven */')
            else: f.write(' else '.join(f'if ({cnd}) {lhs} {op} {e};' for cnd, e in alts))
            f.write(' end' + nl)
    # load in FLAT and EXCLUSIVE form: uses = (sel of the group & section condition & ~the previous
    # conditions of the group), enable = OR of the uses, value = OR of (OR of the uses that pick that data & data).
    # Exact because at most one group is active per word (conflicts = 0, proven above) and the priority between the sections of a
    # group is reproduced in the conditions; the 'first'/'one' modes are reproduced as in the chain.
    import collections as _cr
    W_PART = dict(PARTS)
    def flat_part(part):
        groups = grouped.get(part, {})
        uses = []   # (name, condition, data)
        for k, (s2, conds) in enumerate(groups.items()):
            alts = sv_src(s2)
            if part.startswith('dob'): alts = dob_narrow(alts)
            alts = aob_from_au(part, alts)
            mode = _census.get((part, s2))
            sel = f'sel_{part}_{k}'
            if alts[0][0] is None: uses.append((f'{part}_u{k}_0', sel, alts[0][1])); continue
            if mode == 'first' and alts[0][0].endswith('_drv'): uses.append((f'{part}_u{k}_0', sel, alts[0][1])); continue
            one_src = (mode == 'one' and all(c.endswith('_drv') for c, _ in alts))
            prev_conds = []
            for j, (cnd, e) in enumerate(alts):
                last = one_src and j == len(alts) - 1
                terms = [sel] + ([] if last else [f'({cnd})']) + [f'~({c})' for c in prev_conds]
                uses.append((f'{part}_u{k}_{j}', ' & '.join(terms), e))
                prev_conds.append(cnd)
        return uses
    for part in [q for q, _ in PARTS]:
        w = W_PART[part]; uses = flat_part(part)
        f.write(f'  // {part}: flat and exclusive load' + nl)
        for use_name, cond, e in uses: f.write(f'  logic {use_name}; assign {use_name} = {cond};   // {e}' + nl)
        f.write(f'  logic {part}_en; assign {part}_en = ' + (' | '.join(n for n, _, _ in uses) if uses else "1'b0") + ';' + nl)
        by_data = _cr.OrderedDict()
        for use_name, cond, e in uses: by_data.setdefault(e, []).append(use_name)
        for i, (e, use_names) in enumerate(by_data.items()):
            f.write(f'  logic {part}_s{i}; assign {part}_s{i} = ' + ' | '.join(use_names) + f';   // {e}' + nl)
        f.write(f'  logic [{w-1}:0] {part}_val; assign {part}_val = ' + (' | '.join('({%d{%s_s%d}} & %s)' % (w, part, i, e) for i, (e, _) in enumerate(by_data.items())) if by_data else "'0") + ';' + nl)
    f.write('  always_comb begin' + nl)
    for part in ('aob_h', 'aob_l', 'dob_hi', 'dob_lo', 'dbin_hi', 'dbin_lo'): f.write(f'    {part}_n = {part}_en ? {part}_val : {part};' + nl)
    f.write('  end' + nl)
    f.write('  always_ff @(posedge clk) begin' + nl + '    if (!rst_n) begin' + nl)
    f.write('      at_h <= \'0; at_l <= \'0; aob_h <= \'0; aob_l <= \'0; dob_hi <= \'0; dob_lo <= \'0; dbin_hi <= \'0; dbin_lo <= \'0;' + nl)
    f.write('    end else if (en) begin' + nl)
    for part, _w in PARTS: f.write(f'      if ({part}_en) {part} <= {part}_val;' + nl)
    f.write('    end' + nl + '  end' + nl)
    f.write('`ifdef VERILATOR' + nl + '  // SAFETY NET: the priority-chain form (next values and shadow registers), compared at every clock' + nl)
    f.write('  logic [15:0] aob_h_n_v, aob_l_n_v; logic [7:0] dob_hi_n_v, dob_lo_n_v, dbin_hi_n_v, dbin_lo_n_v;' + nl)
    f.write('  logic [15:0] at_h_v, at_l_v, aob_h_v, aob_l_v; logic [7:0] dob_hi_v, dob_lo_v, dbin_hi_v, dbin_lo_v;' + nl)
    f.write('  always_comb begin' + nl + '    aob_h_n_v = aob_h; aob_l_n_v = aob_l; dob_hi_n_v = dob_hi; dob_lo_n_v = dob_lo; dbin_hi_n_v = dbin_hi; dbin_lo_n_v = dbin_lo;' + nl)
    for part in ('aob_h', 'aob_l', 'dob_hi', 'dob_lo', 'dbin_hi', 'dbin_lo'): emit_part(part, part + '_n_v', '=')
    f.write('  end' + nl)
    f.write('  always_ff @(posedge clk) begin' + nl + '    if (!rst_n) begin' + nl)
    f.write('      at_h_v <= \'0; at_l_v <= \'0; aob_h_v <= \'0; aob_l_v <= \'0; dob_hi_v <= \'0; dob_lo_v <= \'0; dbin_hi_v <= \'0; dbin_lo_v <= \'0;' + nl)
    f.write('    end else if (en) begin' + nl)
    for part, _w in PARTS: emit_part(part, part + '_v', '<=')
    f.write('    end' + nl + '  end' + nl)
    f.write('  always @(posedge clk) if (rst_n && (aob_h_n !== aob_h_n_v || aob_l_n !== aob_l_n_v || dob_hi_n !== dob_hi_n_v || dob_lo_n !== dob_lo_n_v || dbin_hi_n !== dbin_hi_n_v || dbin_lo_n !== dbin_lo_n_v' + nl)
    f.write('      || at_h !== at_h_v || at_l !== at_l_v || aob_h !== aob_h_v || aob_l !== aob_l_v || dob_hi !== dob_hi_v || dob_lo !== dob_lo_v || dbin_hi !== dbin_hi_v || dbin_lo !== dbin_lo_v)) begin' + nl)
    f.write('    $display("priority chain MISMATCH t=%0t", $time); $fatal(1);' + nl + '  end' + nl + '`endif' + nl)
    f.write('endmodule' + nl)
    print('regs_bd: data groups per part:', {part: len(g) for part, g in grouped.items()})

# testbench top: control store + decoder + fabric + registers
srcs = []
for l in open(f'{R}/rtl/cpu_si/m68k_bus_sec.sv'):
    m = re.match(r'\s+input\s+logic \[(\d+):0\] src_(\w+),', l)
    if m: srcs.append((m.group(2), int(m.group(1)) + 1))
with open(f'{R}/sim/m68k_regs_top.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// m68k_regs_top.sv - control store + decoder + section fabric + AT/AOB/DOB/DBIN registers (testbench top). Core' + nl)
    f.write('// configuration: ORG_OVR_LUT = 0 (1 = testbench configuration). It exposes the current microword `uw` (positioning of the testbench along the' + nl)
    f.write('// microcode) and the values being loaded aob_n / dob_n / dbin_n.' + nl)
    f.write('import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;' + nl)
    f.write('module m68k_regs_top #(parameter MEM_FILE = "m68k_urom.mem", parameter ORIGIN_FILE = "m68k_norigin.mem", parameter NANO_FILE = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",' + nl)
    f.write("                       parameter bit ORG_OVR_LUT = 1'b0) (" + nl)
    f.write('  input  logic clk, rst_n, en, input logic [9:0] a1, a2, a3, input logic a0_sub, input logic [9:0] a0, input logic a0s_sub, input logic [9:0] a0s, input logic [1:0] c1c0,' + nl)
    f.write('  output logic [9:0] upc, output logic [NDEC_W-1:0] dec, output logic [16:0] uw,' + nl)
    for p, w in srcs: f.write(f'  input  logic [{w - 1}:0] src_{p},' + nl)
    f.write('  input  logic [15:0] src_edb,' + nl)
    f.write('  output logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,' + nl)
    f.write('  output logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,' + nl)
    f.write('  output logic [31:0] at, aob, output logic [15:0] dob, dbin, output logic [31:0] aob_n, output logic [15:0] dob_n, dbin_n' + nl + ');' + nl)
    f.write('  logic [15:0] ab_dob_w; logic ab_dob_drv_w;   // narrow copy of the AB sections for the DOB (m68k_bus_sec)' + nl)
    f.write('  m68k_bus_top #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .ORG_OVR_LUT(ORG_OVR_LUT)) u_bus (.clk(clk), .rst_n(rst_n), .en(en), .a1(a1), .a2(a2), .a3(a3),' + nl)
    f.write('    .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s), .c1c0(c1c0), .upc(upc), .dec(dec), .uw(uw),' + nl)
    f.write('    ' + ', '.join(f'.src_{p}(src_{p})' for p, _ in srcs) + ',' + nl)
    f.write('    .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),' + nl)
    f.write('    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),' + nl)
    f.write('    .ab_dob(ab_dob_w), .ab_dob_drv(ab_dob_drv_w));' + nl)
    f.write('  m68k_regs_bd u_regs (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),' + nl)
    f.write('    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),' + nl)
    f.write('    .ab_dob(ab_dob_w), .ab_dob_drv(ab_dob_drv_w),' + nl)
    f.write('    .src_au(src_au), .src_alu(src_alu), .src_edb(src_edb), .at(at), .aob(aob), .dob(dob), .dbin(dbin), .aob_n(aob_n), .dob_n(dob_n), .dbin_n(dbin_n));' + nl + 'endmodule' + nl)
print(f'destination atoms {len(rules)} | load specifications {sum(len(v) for v in rules.values())} | parts {len(PARTS)}')
