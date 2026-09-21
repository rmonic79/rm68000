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

# Fabric of the AB/DB bus sections (H/L/D) driven by the per-resource decoder (m68k_nano_dec_pkg.sv).
# Section rules: docs/silicon-datapath.md §5. For each source atom 'X<-S' (X = bus+suffix) the generator
# computes which sections of the bus are driven and with which value; it emits:
#  - rtl/cpu_si/m68k_bus_sec.sv  (GENERATED): source ports + dec -> ab_h/ab_l/ab_d/db_h/db_l/db_d + 'driven' flag
#  - sim/m68k_bus_top.sv  (GENERATED): control store + decoder + fabric (for the testbench)
# Usage: python gen_bus_sec.py <repo_root>
import os, re, sys, collections
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
DIAG = os.environ.get('GEN_DIAG_DIR')   # folder for the f_groups/gd/gq diagnostic files; not set = they are not written
G_FORCED = {'sel_ab_h_2', 'sel_db_l_3'}   # selectors also registered in `g` (they start long paths: see the comment on `g` below)
G_RF = 15   # register file read port selectors in g[26:12] (masks in m68k_nstore_dec.sv, proven against the decoder)
G_W = 12    # selectors registered at en in g: 8 (the wide ones and G_FORCED) + 4 multi-condition ones of RAW class
GQ_W = 25   # multi-condition selectors of NON-RAW class, flopped at every clock from dec_c in gq (= same function of dec_q), + the non-RAW halves of the driven flags
GD_W = 6    # RAW halves of the driven flags (sections with at least one RAW selector): flopped at en in gd
R = sys.argv[1]; D = f'{R}/docs/data'
W32 = {'at', 'pc', 'rx', 'rxa', 'ry', 'rya', 'sp', 'au', 'aob'}
REGS = {'rx', 'rxa', 'rxd', 'rxdl', 'rxh', 'rxl', 'ry', 'rya', 'ryd', 'rydl', 'ryh', 'ryl', 'sp', 'sph', 'spl', 'pc', 'pch', 'pcl'}   # register file sources
def ident(s):
    s = s.replace('<-', '_from_').replace('+=', '_const_').replace('*', 'star').replace('(', 'p_').replace(')', '')
    s = s.replace('+', 'plus').replace('-', 'minus').replace(',', '_or_')
    return re.sub(r'[^A-Za-z0-9]+', '_', s).strip('_').upper()
# decoder: resource -> (lsb, w, onehot), value -> code
dec = {}; codes = {}
for l in open(f'{R}/rtl/cpu_si/m68k_nano_dec_pkg.sv'):
    m = re.match(r'localparam int ND_(\w+)_LSB = (\d+), ND_\w+_W = (\d+), ND_\w+_ONEHOT = (\d);  // (.*)$', l.rstrip('\n'))
    if m: dec[m.group(5)] = (int(m.group(2)), int(m.group(3)), int(m.group(4)))
    m = re.match(r'localparam int ND_(\w+)__(\w+) = (\d+);  // (.*)$', l.rstrip('\n'))
    if m: codes[m.group(4)] = int(m.group(3))
# class of the dec bits: DEC_RAW = 1 -> dec = dec_pq (registered at en: same value in the same cycle, like g);
# DEC_RAW = 0 -> dec = dec_q = dec_c one clock earlier (the same function of dec_c is registered at every clock).
_s = open(f'{R}/rtl/cpu_si/m68k_nstore_dec.sv', encoding='utf-8').read()
_m = re.search(r"DEC_RAW = (\d+)'b([01]+);", _s)
DEC_RAW = {b: _m.group(2)[int(_m.group(1)) - 1 - b] == '1' for b in range(int(_m.group(1)))}
def dec_bits(expr):
    out = set()
    for lsb, w in re.findall(r'dec\[(\d+)(?: \+: (\d+))?\]', expr):
        out |= set(range(int(lsb), int(lsb) + (int(w) if w else 1)))
    return out
late_sels = []   # [(name, expression on dec)] of the selectors registered at every clock from dec_c, in the order of the gq bits
def sel_expr(name, expr):
    # a selector that uses more than one bit of dec, ALL of non-RAW class, becomes gq[j] = flop at every clock of
    # expr(dec_c) (= expr(dec_q) = expr(dec)); mixed bits or a single bit: the expression stays as it is.
    bits = dec_bits(expr)
    if len(bits) > 1 and all(not DEC_RAW[b] for b in bits):
        late_sels.append((name, expr)); return f'gq[{len(late_sels) - 1}]'
    return expr
words = {}
for l in open(f'{D}/mc68000_nano_encoding.tsv', encoding='utf-8'):
    f = l.rstrip('\n').split('\t')
    if re.match(r'^[0-9a-f]{3}\t', l): words[int(f[0], 16)] = [x for x in f[4].split(' ; ') if x]
atoms = sorted(set(a for A in words.values() for a in A))
def rule(at):   # -> (bus, {section: (src, part)}, port, width), or None if it is not a bus-source atom
    m = re.match(r'^(ab|db)(h|l|d|e|\*)?<-(.+)$', at)
    if not m: return None
    bus, suf, S = m.group(1), m.group(2) or '', m.group(3).strip('()')
    w = 32 if S in W32 else 16
    nat = 'h' if S.endswith('h') else 'l' if (S.endswith('l') and not S.endswith('dl')) or S == 'ftu' else 'd'
    port = S
    if S in REGS:   # register: it comes from the register file read port of THIS bus (rf_ab/rf_db, 32 bits); H/L part from the name
        port = f'rf_{bus}'; w = 32
        hi, lo = (port, 'H'), (port, 'L')
        if S not in W32: hi = lo = (port, 'H' if S.endswith('h') else 'L')
    else:
        hi, lo = (S, 'H'), (S, 'L')          # part of the value: H = bits 31:16, L = bits 15:0 (16 bits: always L)
        if w == 16: hi = lo
    sec = {}
    if at == 'dbe<-(rxdl)': lo = (port, 'L6')   # shift count from a register: modulo 64 (MC68000UM: a count of 64 gives 0 shifts)
    if suf == 'h': sec = {'h': hi}
    elif suf == 'l': sec = {'l': lo}
    elif suf == 'd': sec = {'d': lo}
    elif suf == 'e': sec = {'l': lo, 'h': ('sext', 'L')}
    elif suf == '':
        if w == 32: sec = {'h': hi, 'l': lo}
        elif nat == 'h': sec = {'h': hi}
        elif nat == 'l': sec = {'l': lo}
        else: sec = {'d': lo}                                # DATA section source: D only (H/L are left to other sources)
    else:                                                      # 'ab*'/'db*': at least the DATA section
        sec = {'h': hi, 'l': lo, 'd': lo} if w == 32 else {nat: lo}
    return bus, sec, port, w
srcs = {}
rules = {}
for at in atoms:
    r = rule(at)
    if r: rules[at] = r; srcs[r[2]] = r[3]
# conflicts: two atoms of the same word that drive the same section of the same bus
conf = 0
for a, A in words.items():
    drv = collections.Counter()
    for at in A:
        if at in rules:
            bus, sec, S, w = rules[at]
            for s in sec: drv[(bus, s)] += 1
    conf += sum(1 for v in drv.values() if v > 1)
nl = '\n'
ports = sorted(srcs)
with open(f'{R}/rtl/cpu_si/m68k_bus_sec.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// m68k_bus_sec.sv - fabric of the AB/DB bus sections (HIGH/LOW/DATA) of the silicon-structured datapath.' + nl)
    f.write('// GENERATED by docs/data/scripts/gen_bus_sec.py (rules: docs/silicon-datapath.md section 5), DO NOT edit by hand.' + nl)
    f.write('// Inputs: values of the sources (registers/units outside this module) + dec (per-resource decoder). Outputs: the three sections' + nl)
    f.write('// of each bus and the "driven" flag per section (the destinations load only the driven sections).' + nl)
    f.write('import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;' + nl + 'module m68k_bus_sec (' + nl)
    f.write('  /* verilator lint_off UNUSEDSIGNAL */' + nl + '  input  logic [NDEC_W-1:0] dec,' + nl + '  /* verilator lint_on UNUSEDSIGNAL */' + nl + '  input  logic [' + str(G_W - 1) + ':0] g,' + nl + '  input  logic [' + str(GQ_W - 1) + ':0] gq,   // non-RAW class selectors, flopped at every clock from dec_c (m68k_nstore_dec)' + nl + '  input  logic [' + str(GD_W - 1) + ':0] gd,   // RAW half of the driven flags (flopped at en in m68k_nstore_dec)' + nl)
    for p in ports: f.write(f'  input  logic [{srcs[p] - 1}:0] src_{p},' + nl)
    f.write('  output logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,' + nl)
    f.write('  output logic [15:0] ab_ld,   // ab_l_drv ? ab_l : ab_d, from the triples (one LUT, same level as ab_l)' + nl)
    f.write('  output logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,' + nl)
    f.write('  // the value that the DOB takes from the AB bus, and its flag: a single wire, because' + nl)
    f.write('  // every word with DOB<-AB has a single source on the bus (census over the 512 words)' + nl)
    f.write('  output logic [15:0] ab_dob,' + nl)
    f.write('  output logic        ab_dob_drv,' + nl)
    f.write('  // the write data of the register file per port (AB/DB) and half (H/L): a single wire' + nl)
    f.write('  // for each, flat AND-OR of only the sources that the census allows (below).' + nl)
    f.write('  output logic [15:0] ab_wl, ab_wh, db_wl, db_wh' + nl + ');' + nl)
    # PROOF of the distribution: for every used word and every sign-extended atom in it,
    # the low section of the same bus must be driven by THAT atom only. If it were driven
    # by another source, `sel_h & bus_l[15]` would not equal `sel_atom & src_atom[15]`.
    nse = 0
    for a, A in words.items():
        for at in A:
            if at not in rules: continue
            bus, sec, S, w = rules[at]
            if sec.get('h', (None,))[0] != 'sext': continue
            low_srcs = [x for x in A if x in rules and rules[x][0] == bus and 'l' in rules[x][1]]
            assert low_srcs == [at], 'word %03x: sign extension %s but the low section belongs to %s' % (a, at, low_srcs)
            nse += 1
    print(f'bus_sec: sign-extended words proven: {nse} (low section always from the same source)')
    groups = {}   # (bus, section) -> {value -> [conditions]}
    for at in atoms:
        if at not in rules: continue
        bus, sec, S, w = rules[at]
        L = at.split('<-')[0] + '<-'; lsb, dw, oh = dec[L]; c = codes[at]
        cond = f'dec[{lsb + c}]' if oh else f'dec[{lsb} +: {dw}] == {dw}\'d{c}'
        for sx in ('h', 'l', 'd'):
            if sx not in sec: continue
            src, part = sec[sx]
            if src == 'sext':
                # The sign extension does not read bit 15 of the already computed low section: it
                # takes the high bit of ITS OWN source. Distributing the AND over the OR makes the
                # high section a single flat AND-OR (two logic levels instead of four) that does not
                # wait for the low section. This holds because in a sign-extended word the low
                # section is driven by THAT source (proven above).
                s2, p2 = sec['l']
                val = ("16'd0" if p2 == 'L6'
                       else f'{{16{{src_{s2}[31]}}}}' if p2 == 'H'
                       else f'{{16{{src_{s2}[15]}}}}')

            elif part == 'H': val = f'src_{src}[31:16]'
            elif part == 'L6': val = f'{{10\'d0, src_{src}[5:0]}}'
            else: val = f'src_{src}[15:0]'
            groups.setdefault((bus, sx), collections.OrderedDict()).setdefault(val, []).append((cond, at))
    wide_sels = []
    sec_sels = {}   # per section: [(name, class, expression on dec)]
    for (bus, sx), g in groups.items():
        for k, (val, conds) in enumerate(g.items()):
            # selectors with more than 6 inputs are not built here: they are predicates on the
            # nanoword fields, computed in m68k_nstore_dec and registered at en (one logic level
            # less, and no cone crossing two entities). The order of the g bits is this one.
            # Besides the wide selectors, `g` also holds those that start the longest paths
            # (G_FORCED): sel_ab_h_2 (DOB chain) and sel_db_l_3 (address error chain). They become
            # flop outputs: one logic level and one route less at the start of those chains.
            if len(set(c for c, _ in conds)) > 6 or f'sel_{bus}_{sx}_{k}' in G_FORCED or (len(set(c for c, _ in conds)) > 1 and all(DEC_RAW[b] for c, _ in conds for b in dec_bits(c))):   # also the multi-condition ones of RAW class

                f.write(f'  logic sel_{bus}_{sx}_{k}; assign sel_{bus}_{sx}_{k} = g[{len(wide_sels)}];   // {val}: ' + ' '.join(at for _, at in conds) + nl)
                wide_sels.append(f'sel_{bus}_{sx}_{k}')
                sec_sels.setdefault((bus, sx), []).append((f'sel_{bus}_{sx}_{k}', 'raw', None))
            else:
                f.write(f'  logic sel_{bus}_{sx}_{k}; assign sel_{bus}_{sx}_{k} = ' + sel_expr(f'sel_{bus}_{sx}_{k}', ' | '.join(f'({c})' for c, _ in conds)) + f';   // {val}: ' + ' '.join(at for _, at in conds) + nl)
                _e = ' | '.join(f'({c})' for c, _ in conds); _b = dec_bits(_e)   # class of the selector
                sec_sels.setdefault((bus, sx), []).append((f'sel_{bus}_{sx}_{k}', 'raw' if all(DEC_RAW[b] for b in _b) else ('q' if all(not DEC_RAW[b] for b in _b) else 'mixed'), _e))
    # block for m68k_nstore_dec.sv: masks and f_groups of the selectors registered in `g`.
    # Every atom is a code of one nano ROM slot, so a selector (set of atoms) is exactly a set
    # of codes per slot: a constant mask indexed by the field.
    # PROOF: the set of atoms covered by the masks must be IDENTICAL to that of the selector
    # (assert below). The block is pasted by hand into m68k_nstore_dec.sv, which is not
    # generated; if the selectors in G_FORCED change, this text changes and must be pasted again.
    slots = []
    for l2 in open(f'{D}/mc68000_nano_encoding.tsv', encoding='utf-8'):
        f2 = l2.rstrip(chr(10)).split(chr(9))
        if f2[0].startswith('slot'):
            mp = {}
            for tok in f2[3].split(' '):
                if '=' in tok:
                    c2, a2 = tok.split('=', 1); mp[int(c2)] = a2
            slots.append((int(f2[1]), int(f2[2]), mp))
    sel_atoms = {}
    for (bus, sx), gg in groups.items():
        for k, (val, conds) in enumerate(gg.items()):
            sel_atoms[f'sel_{bus}_{sx}_{k}'] = set(at for _, at in conds)
    mask_lines, func_lines = [], []
    for idx, name in enumerate(wide_sels):
        wanted = sel_atoms[name]
        covered, or_terms = set(), []
        for si, (lsb, w2, mp) in enumerate(slots):
            cod = sorted(c2 for c2, a2 in mp.items() if a2 in wanted)
            if not cod: continue
            covered |= set(mp[c2] for c2 in cod)
            n = 1 << w2
            bits = ''.join('1' if i in cod else '0' for i in range(n - 1, -1, -1))
            M = 'M_' + name[4:].upper() + f'_{si}'
            mask_lines.append(f"  localparam logic [{n-1}:0] {M} = {n}'b{bits};   // field nw[{lsb} +: {w2}] in " + '{' + ','.join(str(c2) for c2 in cod) + '}')
            or_terms.append(f'{M}[w[{lsb} +: {w2}]]')
        assert covered == wanted, f'{name}: masks cover {sorted(covered)} instead of {sorted(wanted)}'
        func_lines.append(f"    f_groups[{idx}] = " + ' | '.join(or_terms) + f';   // {name}')
    block_txt = (f'  // {len(wide_sels)} registered selectors: ' + ' '.join(wide_sels) + nl
             + nl.join(mask_lines) + nl
             + f'  function automatic logic [{len(wide_sels)-1}:0] f_groups (input logic [NROM_W-1:0] w);' + nl
             + nl.join(func_lines) + nl + '  endfunction' + nl)
    if DIAG: open(os.path.join(DIAG, 'f_groups.txt'), 'w', encoding='utf-8').write(block_txt)
    # the driven flags: RAW half = mask (like f_groups) registered at en in gd; non-RAW half = OR on dec_c
    # registered at every clock (gq); mixed selectors in the direct OR. Same value at every clock.
    drv_raw = []; drv_assign = {}
    for (bus, sx) in [(b_, x_) for b_ in ('ab', 'db') for x_ in ('d', 'l', 'h')]:
        lst = sec_sels.get((bus, sx), [])
        drv_terms = []
        raw_names = [nm_ for nm_, cl_, _ in lst if cl_ == 'raw']
        if raw_names:
            drv_raw.append((f'drv_{bus}_{sx}_r', set().union(*[sel_atoms[nm_] for nm_ in raw_names])))
            drv_terms.append(f'gd[{len(drv_raw) - 1}]')
        q_exprs = [e_ for nm_, cl_, e_ in lst if cl_ == 'q']
        if q_exprs:
            late_sels.append((f'drv_{bus}_{sx}_q', ' | '.join(f'({e_})' for e_ in q_exprs)))
            drv_terms.append(f'gq[{len(late_sels) - 1}]')
        drv_terms += [nm_ for nm_, cl_, _ in lst if cl_ == 'mixed']
        drv_assign[(bus, sx)] = ' | '.join(drv_terms) if drv_terms else "1'b0"
    assert len(drv_raw) == GD_W, 'RAW halves of the flags = %d: update GD_W' % len(drv_raw)
    mask_lines2, func_lines2 = [], []
    for idx, (name, wanted) in enumerate(drv_raw):
        covered, or_terms = set(), []
        for si, (lsb, w2, mp) in enumerate(slots):
            cod = sorted(c2 for c2, a2 in mp.items() if a2 in wanted)
            if not cod: continue
            covered |= set(mp[c2] for c2 in cod)
            n = 1 << w2
            bits = ''.join('1' if i in cod else '0' for i in range(n - 1, -1, -1))
            M = 'M_' + name.upper() + f'_{si}'
            mask_lines2.append(f"  localparam logic [{n-1}:0] {M} = {n}'b{bits};   // field nw[{lsb} +: {w2}] in " + '{' + ','.join(str(c2) for c2 in cod) + '}')
            or_terms.append(f'{M}[w[{lsb} +: {w2}]]')
        assert covered == wanted, f'{name}: masks cover {sorted(covered)} instead of {sorted(wanted)}'
        func_lines2.append(f"    f_drv[{idx}] = " + ' | '.join(or_terms) + f';   // {name}')
    block_txt2 = (f"  // {len(drv_raw)} RAW halves of the driven flags (masks on the slots, like f_groups): " + ' '.join(n_ for n_, _ in drv_raw) + nl
             + nl.join(mask_lines2) + nl
             + f'  function automatic logic [{len(drv_raw)-1}:0] f_drv (input logic [NROM_W-1:0] w);' + nl
             + nl.join(func_lines2) + nl + '  endfunction' + nl)
    if DIAG: open(os.path.join(DIAG, 'gd.txt'), 'w', encoding='utf-8').write(block_txt2)
    for (bus, sx), e_ in drv_assign.items():
        f.write(f'  assign {bus}_{sx}_drv = {e_};   // driven flag from flops: RAW half (gd, at en) | non-RAW half (gq, at every clock)' + nl)
    print(f'bus_sec: driven flags from flops: {len(drv_raw)} RAW halves (gd), {sum(1 for k_ in drv_assign if "gq[" in drv_assign[k_])} non-RAW halves (gq)')
    print(f'bus_sec: f_groups for {len(wide_sels)} selectors' + (f' written to {DIAG}/f_groups.txt' if DIAG else '') + ' (atoms proven identical)')
    # the width of `g` in the ports comes from the constant G_W (they are emitted before `wide_sels` exists):
    # if the number of registered selectors changes, everything stops here instead of generating a file
    # with the wrong width.
    assert len(wide_sels) == G_W, 'registered selectors = %d: update G_W in gen_bus_sec.py, m68k_nstore_dec.sv and m68k_cpu_si.sv' % len(wide_sels)
    # every section in an explicit TWO-level tree: triples (3 selectors + 3 data = 6 inputs = one LUT6 per bit)
    # kept with (* keep *) (like t_s* in m68k_nrom.sv), and the final OR of the triples (<= 4 inputs = one LUT).
    # Without the constraint synthesis may chain the terms; with the selectors already at depth 0 (flop outputs)
    # the constrained tree is 2 levels from the flops. Same function, no extra cycle.
    TRIPLE_N = 3
    assert not conf, 'the triple tree assumes exclusive selections (conf == 0)'
    triples = {}
    for (bus, sx) in [(b, x) for b in ('ab', 'db') for x in ('d', 'l', 'h')]:
        items = list(groups.get((bus, sx), {}).items())
        names = []
        for j in range(0, len(items), TRIPLE_N):
            triple_name = f'{bus}_{sx}_t{j // TRIPLE_N}'
            or_terms = ' | '.join(f'({{16{{sel_{bus}_{sx}_{k}}}}} & {val})' for k, (val, conds) in list(enumerate(items))[j:j + TRIPLE_N])
            f.write(f'  (* keep *) logic [15:0] {triple_name}; assign {triple_name} = {or_terms};   // triple ' + ' '.join(f'sel_{bus}_{sx}_{k}' for k in range(j, min(j + TRIPLE_N, len(items)))) + nl)
            names.append(triple_name)
        triples[(bus, sx)] = names
    # l/d coupling of the AB at the second level of the fabric: OR of the ab_l triples | (~ab_l_drv & ab_d triple)
    assert len(triples[('ab', 'd')]) == 1, 'ab_ld: ab_d with more than one triple'
    f.write('  assign ab_ld = ' + ' | '.join(triples[('ab', 'l')]) + ' | ({16{~ab_l_drv}} & ' + triples[('ab', 'd')][0] + ');   // = ab_l_drv ? ab_l : ab_d (with ab_l_drv = 0 the triples of ab_l are 0)' + nl)
    f.write('  always_comb begin' + nl)
    for (bus, sx) in [(b, x) for b in ('ab', 'db') for x in ('d', 'l', 'h')]:   # sections in the order d, l, h
        g = groups.get((bus, sx), {})
        f.write(f"    {bus}_{sx} = '0;" + nl)   # the flag is an assign from flops
        for k, (val, conds) in enumerate(g.items()):
            # with conf == 0 (no word drives the same section twice) the selections are exclusive by
            # construction: no priority term, one logic level less on every bus
            prev = (' & ~(' + ' | '.join(f'sel_{bus}_{sx}_{j}' for j in range(k)) + ')') if (k and conf) else ''
            pass   # driven flag: assign above
            pass   # the section value is the OR of the kept triples, written after the loop
        if triples[(bus, sx)]: f.write(f"    {bus}_{sx} = " + ' | '.join(triples[(bus, sx)]) + ';   // OR of the triples' + nl)
    # AB sections dedicated to the DOB. Exhaustive census over the 512 words: in the microwords where the DOB
    # takes from the AB (atoms dob<-ab, dob<-ab*, dobh<-ab, dobl<-ab, dobl<-ab*) the AB is driven only by a subset
    # of the sources. The others - ab<-aob, ab<-au, ab<-ftu, ab<-(at), the ALU variants and the sign extension -
    # are PROVEN impossible there: keeping them in the AND-OR tree that reaches the DOB costs logic levels and
    # routes for nothing, and they start long paths (dec[27] = ab<-aob, FTU -> data_out).
    DOB_AB = {'dob<-ab', 'dob<-ab*', 'dobh<-ab', 'dobl<-ab', 'dobl<-ab*'}
    pdob = sorted(a for a, A in words.items() if DOB_AB & set(A))
    dob_atoms = set(at for a in pdob for at in words[a])
    ab_sections = {sx: list(groups.get(('ab', sx), {}).items()) for sx in ('d', 'l', 'h')}
    keep_k = {sx: [k for k, (val, conds) in enumerate(ab_sections[sx]) if any(at in dob_atoms for _, at in conds)]
             for sx in ('d', 'l', 'h')}
    for a in pdob:   # check: no atom of those words drives an AB section outside the subset
        for sx in ('d', 'l', 'h'):
            for k, (val, conds) in enumerate(ab_sections[sx]):
                if any(at in words[a] for _, at in conds):
                    assert k in keep_k[sx], f'word {a:03x}: ab_{sx} k={k} ({val}) outside the DOB subset'
    # Every word with DOB<-AB has ONE AB source only (census): the sections that source drives all
    # carry the same value, so the d -> l -> h priority of the DOB mux always picks that value and a
    # single wire is enough. Five distinct values in all, a two-level tree instead of three sections
    # plus the priority.
    for a in pdob:
        n_ab = sum(1 for sx in ('d', 'l', 'h') for k, (val, conds) in enumerate(ab_sections[sx])
                   if any(at in words[a] for _, at in conds))
        ab_srcs = set(at for sx in ('d', 'l', 'h') for k, (val, conds) in enumerate(ab_sections[sx])
                  for _, at in conds if at in words[a])
        assert n_ab == 0 or len(ab_srcs) == 1, f'word {a:03x}: {len(ab_srcs)} AB sources with DOB<-AB: {sorted(ab_srcs)}'
    per_val = collections.OrderedDict()
    for sx in ('d', 'l', 'h'):
        for k in keep_k[sx]:
            val = ab_sections[sx][k][0]
            assert 'ab_l[15]' not in val, 'sign extension among the DOB sources: not expected'
            per_val.setdefault(val, []).append(f'sel_ab_{sx}_{k}')
    f.write('    // the value that the DOB takes from the AB: only the sources that the census allows' + nl)
    f.write('    ab_dob_drv = ' + ' | '.join(s for ss in per_val.values() for s in ss) + ';' + nl)
    f.write('    ab_dob = ' + ' | '.join(
        '({16{' + (ss[0] if len(ss) == 1 else ' | '.join(ss)) + f'}}}} & {val})' for val, ss in per_val.items()) + ';' + nl)
    f.write('  end' + nl)
    # REGISTER FILE WRITE DATA: one narrow wire per port (AB/DB) and half (H/L).
    # By definition the data of each half is a chain on the drv flags chosen per atom group, e.g.
    #   wd_ab_l = grp(rxdl<-ab*,rydl<-ab,rydl<-ab*) ? (ab_d_drv ? ab_d : ab_l) : (ab_l_drv ? ab_l : ab_d)
    # i.e. AFTER the sections (9-term AND-OR for ab_l) another mux level, then the port mux w_dl, then
    # the MLAB. Here, with the write rules of gen_regfile.rule (copied into wrule(); the group conditions
    # are the same as in the .sv), for EVERY word that writes that half the section chosen by the chain
    # and its value are computed. The source atoms that win in some word are the only sources of the
    # wire (flat AND-OR, dec-only selectors); those that always lose (e.g. ab*<-(dbin) in asxw2,
    # abd<-alu in cmml6) do not enter. Where two winners are active in the same word (046 dbcc2:
    # ab*<-alu beats ab<-au; 342 swap2: dbd<-alu beats db<-au) the loser is qualified with the
    # complement of the winner (one more condition in the same LUT as the selector: no extra level).
    # PROOF (assert): for every word with the write active exactly ONE selector is on and carries the
    # value of the chain; no precedence cycle between winners. Words without a write do not matter: the
    # wire is consumed only with we_* (enable from dec only), and the m68k_regfile_top testbench, which
    # drives the flags at random, keeps the chain form (WD_NARROW = 0). Clocked safety net in
    # m68k_bus_top (below) and in regfile_si.
    WREG = {'rx': 'rx', 'rxa': 'rxa', 'rxh': 'rx', 'rxl': 'rx', 'rxdl': 'rxd', 'ry': 'ry', 'rya': 'rya', 'rydl': 'ryd', 'ryh': 'ry', 'ryl': 'ry', 'sp': 'sp', 'sph': 'sp', 'spl': 'sp'}
    def wrule(at):   # register write atom -> [(port, half, section chain)] like gen_regfile.rule (without the PC)
        m = re.match(r'^(rx|rxa|rxh|rxl|rxdl|ry|rya|rydl|ryh|ryl|sp|sph|spl)<-(ab|db)(h|l|d|e|\*)?$', at)
        if not m: return None
        dst, bus, suf = m.group(1), m.group(2), m.group(3) or ''
        if dst in ('rxh', 'ryh', 'sph'): return [(bus, 'h', ['h', 'l', 'd'])]
        if dst in ('rxl', 'ryl', 'spl'): return [(bus, 'l', ['l', 'd'])]
        if dst in ('rxdl', 'rydl'): return [(bus, 'l', ['d', 'l'] if suf in ('', 'd', '*') else ['l', 'd'])]
        if suf == 'h': return [(bus, 'h', ['h', 'l', 'd'])]
        if suf == 'l': return [(bus, 'l', ['l', 'd'])]
        if suf == 'd': return [(bus, 'l', ['d'])]
        if suf == '*': return [(bus, 'h', ['h']), (bus, 'l', ['d', 'l'])]
        return [(bus, 'h', ['h']), (bus, 'l', ['l', 'd'])]
    wgrp = {}   # (bus, half) -> {chain: [write atoms]}
    for at in atoms:
        for (bus, half, chain) in (wrule(at) or []):
            wgrp.setdefault((bus, half), collections.OrderedDict()).setdefault('|'.join(chain), []).append(at)
    def cond_at(at):
        L = at.split('<-')[0] + '<-'; lsb, dw, oh = dec[L]; c = codes[at]
        return f'dec[{lsb + c}]' if oh else f'dec[{lsb} +: {dw}] == {dw}\'d{c}'
    def val_of(at, sx):   # SV value that the source atom puts on section sx (same text as the fabric above)
        bus_, sec_, S_, w_ = rules[at]; src_, part_ = sec_[sx]
        if src_ == 'sext':
            s2, p2 = sec_['l']
            return "16'd0" if p2 == 'L6' else (f'{{16{{src_{s2}[31]}}}}' if p2 == 'H' else f'{{16{{src_{s2}[15]}}}}')
        return f'src_{src_}[31:16]' if part_ == 'H' else (f'{{10\'d0, src_{src_}[5:0]}}' if part_ == 'L6' else f'src_{src_}[15:0]')
    def chain_sv(bus, chain):   # the .sv chain on the sections/flags: (ab_d_drv ? ab_d : ab_l)
        e = f'{bus}_{chain[-1]}'
        for sx in reversed(chain[:-1]): e = f'({bus}_{sx}_drv ? {bus}_{sx} : {e})'
        return e
    wd_check = []   # lines of the safety net for m68k_bus_top
    f.write('  // write data of the register file: dec-only selectors, one per distinct value; the losers' + nl)
    f.write('  // qualified with ~winner. Census and per-word proof in gen_bus_sec.py.' + nl)
    for (bus, half), gg in wgrp.items():
        name = f'{bus}_w{half}'
        W = sorted(a for a, A in words.items() if any(at in A for ats in gg.values() for at in ats))
        for a in W:   # a single write per half and per word
            assert sum(1 for ats in gg.values() for at in ats if at in words[a]) == 1, f'{name}: word {a:03x} with two writes'
        winner = {}   # word -> (winning source atom, value)
        for a in W:
            chain = [c for c, ats in gg.items() if any(at in words[a] for at in ats)][0].split('|')
            drv = {}
            for at in words[a]:
                if at in rules and rules[at][0] == bus:
                    for sx in rules[at][1]: assert sx not in drv; drv[sx] = at
            sx = next((s for s in chain if s in drv), None)
            assert sx is not None, f'{name}: word {a:03x} writes without a driven section'
            winner[a] = (drv[sx], val_of(drv[sx], sx))
        kept = collections.OrderedDict()   # value -> [winning atoms]
        for a in W:
            at, v = winner[a]
            if at not in kept.setdefault(v, []): kept[v].append(at)
        katoms = set(at for ats in kept.values() for at in ats)
        beats = collections.defaultdict(set)   # loser -> {winners that beat it}
        for a in W:
            for at in words[a]:
                if at in katoms and at != winner[a][0]: beats[at].add(winner[a][0])
        for lo, ws in beats.items():
            for w_ in ws: assert lo not in beats.get(w_, set()), f'{name}: precedence cycle {lo} / {w_}'
        # PROOF per word: exactly one selector on, with the value of the chain
        for a in W:
            on = [v for v, ats in kept.items() if any(at in words[a] and not any(b in words[a] for b in beats.get(at, ())) for at in ats)]
            assert on == [winner[a][1]], f'{name}: word {a:03x} selectors {on} instead of {winner[a][1]}'
        terms = []
        for k, (v, ats) in enumerate(kept.items()):
            sel = ' | '.join(f'({cond_at(at)})' + (' & ~(' + ' | '.join(f'({cond_at(b)})' for b in sorted(beats[at])) + ')' if at in beats else '') for at in ats)
            f.write(f'  logic sel_{name}_{k}; assign sel_{name}_{k} = {sel_expr(f'sel_{name}_{k}', sel)};   // {v}: ' + ' '.join(ats) + nl)
            terms.append(f'({{16{{sel_{name}_{k}}}}} & {v})')
        f.write(f'  assign {name} = ' + ' | '.join(terms) + ';' + nl)
        for chain_s, ats in gg.items():
            grp_ = ' | '.join(f'({cond_at(at)})' for at in ats)
            wd_check.append((name, grp_, chain_sv(bus, chain_s.split('|')), ' '.join(ats)))
        print(f'bus_sec: {name}: words {len(W)}, groups {dict((c, len(ats)) for c, ats in gg.items())}, values {len(kept)}, qualified {dict((k_, sorted(v_)) for k_, v_ in beats.items())}, never winners {sorted(set(at for a in W for at in words[a] if at in rules and rules[at][0] == bus) - katoms)}')
    # the non-RAW class selectors: block for m68k_nstore_dec.sv (flopped at every clock from dec_c) and its safety net
    assert len(late_sels) == GQ_W, 'non-RAW selectors = %d: update GQ_W in gen_bus_sec.py, m68k_nstore_dec.sv and m68k_cpu_si.sv' % len(late_sels)
    block_txt = ''.join(f'  assign gq_c[{j}] = ' + e.replace('dec[', 'dec_c[') + f';   // {n}' + nl for j, (n, e) in enumerate(late_sels))
    block_txt += '--SAFETY NET--' + nl
    block_txt += ''.join(f'  assign gq_v[{j}] = {e};   // {n}' + nl for j, (n, e) in enumerate(late_sels))
    if DIAG: open(os.path.join(DIAG, 'gq.txt'), 'w', encoding='utf-8').write(block_txt)
    print(f'bus_sec: {len(late_sels)} non-RAW class selectors registered at every clock (gq)' + (f', block in {DIAG}/gq.txt' if DIAG else ''))
    f.write('endmodule' + nl)
    print('bus_sec: distinct values per section:', {f'{b}_{x}': len(g) for (b, x), g in groups.items()}, '| from g:', wide_sels)
    print('bus_sec: words with DOB<-AB:', len(pdob), '| sections kept for the DOB:',
          {f'ab_{sx}': f'{len(keep_k[sx])} of {len(ab_sections[sx])}' for sx in ('d', 'l', 'h')})

with open(f'{R}/sim/m68k_bus_top.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// m68k_bus_top.sv - control store + decoder + bus section fabric (testbench top). Core configuration: ORG_OVR_LUT = 0' + nl)
    f.write('// (1 = testbench configuration, a0s override on any word). It exposes the current microword `uw` (the testbench reads its format' + nl)
    f.write('// and TY to position itself by walking the microcode up to a dispatch).' + nl)
    f.write('import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;' + nl)
    f.write('module m68k_bus_top #(parameter MEM_FILE = "m68k_urom.mem", parameter ORIGIN_FILE = "m68k_norigin.mem", parameter NANO_FILE = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",' + nl)
    f.write("                      parameter bit ORG_OVR_LUT = 1'b0) (" + nl)
    f.write('  input  logic clk, rst_n, en, input logic [9:0] a1, a2, a3, input logic a0_sub, input logic [9:0] a0, input logic a0s_sub, input logic [9:0] a0s, input logic [1:0] c1c0,' + nl)
    f.write('  output logic [9:0] upc, output logic [NDEC_W-1:0] dec, output logic [16:0] uw,' + nl)
    for p in ports: f.write(f'  input  logic [{srcs[p] - 1}:0] src_{p},' + nl)
    f.write('  output logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,' + nl)
    f.write('  output logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,' + nl)
    f.write('  output logic [15:0] ab_dob,' + nl)
    f.write('  output logic        ab_dob_drv' + nl + ');' + nl)
    f.write('  /* verilator lint_off UNUSEDSIGNAL */' + nl)
    f.write('  logic [NROM_W-1:0] nw; logic [NROM_IDX_W-1:0] nidx; logic [9:0] uaddr; logic ir_load, fmt, bit4; logic [1:0] ty, fc; logic [4:0] cbc; logic [' + str(G_W + G_RF - 1) + ':0] g_w; logic [' + str(GQ_W - 1) + ':0] gq_w; logic [' + str(GD_W - 1) + ':0] gd_w;' + nl)
    f.write('  logic [15:0] ab_wl, ab_wh, db_wl, db_wh;   // narrow wires of the register file: here only for the safety net' + nl)
    f.write('  /* verilator lint_on UNUSEDSIGNAL */' + nl)
    f.write('  m68k_nstore_dec #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .ORG_OVR_LUT(ORG_OVR_LUT)) u_cs (.clk(clk), .rst_n(rst_n), .en(en), .a1(a1), .a2(a2), .a3(a3),' + nl)
    f.write('    .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s), .c1c0(c1c0), .upc(upc), .uw(uw), .nw(nw), .dec(dec), .nidx(nidx), .uaddr(uaddr),' + nl)
    f.write('    .ir_load(ir_load), .fmt(fmt), .bit4(bit4), .ty(ty), .fc(fc), .cbc(cbc), .g(g_w), .gq(gq_w), .gd(gd_w), .c_ft_q(), .we_ab_h_q(), .we_ab_l_q(), .we_db_h_q(), .we_db_l_q(),' + nl)
    f.write('    .nwl_pre(), .nw_pre());   // the early nanoword is not needed by this testbench' + nl)
    f.write('  m68k_bus_sec u_bus (.dec(dec), .g(g_w[' + str(G_W - 1) + ':0]), .gq(gq_w), .gd(gd_w), ' + ', '.join(f'.src_{p}(src_{p})' for p in ports) + ',' + nl)
    f.write('    .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d), .ab_ld(),' + nl)
    f.write('    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),' + nl)
    f.write('    .ab_dob(ab_dob), .ab_dob_drv(ab_dob_drv),' + nl)
    f.write('    .ab_wl(ab_wl), .ab_wh(ab_wh), .db_wl(db_wl), .db_wh(db_wh));' + nl)
    f.write('`ifdef VERILATOR' + nl)
    f.write('  // SAFETY NET, simulation only: at every en, in every word that writes that half of the register file,' + nl)
    f.write('  // the narrow wire must equal EXACTLY the chain on the drv flags that the register file used (the tb drives the sources).' + nl)
    f.write('  always @(posedge clk) if (rst_n && en) begin' + nl)
    for name, grp_, ch, ats in wd_check:
        f.write(f'    if ({grp_}) if ({name} !== {ch}) begin $display("narrow wire MISMATCH {name} t=%0t upc=%03x wire %04x chain %04x", $time, upc, {name}, {ch}); $fatal(1); end' + nl)
    f.write('  end' + nl + '`endif' + nl)
    f.write('endmodule' + nl)
print(f'bus-source atoms {len(rules)} | sources {len(ports)} ({sum(srcs.values())} bits) | words with a section driven twice {conf}')
