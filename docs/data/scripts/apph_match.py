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

# Maps the App. H blocks (us4325121_appH.tsv) to die addresses through Galibert's labels (adr-names.txt),
# tolerating w<->l swaps and renames; each match is checked against the behavioural trace all.txt
# (ops per address) and against the TYPE of the decoded micro ROM.
# Usage: python apph_match.py <repo_data_dir> <galibert_dir>
import re, sys, collections
D = sys.argv[1]; G = sys.argv[2]
B = [l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_appH.tsv', encoding='utf-8') if l.startswith('p')]
names = {}   # Galibert label -> addr
for l in open(f'{G}/adr-names.txt'):
    m = re.match(r'^([0-9a-f]{3})\s+(\S+)', l.strip())
    if m: names[m.group(2).lower()] = int(m.group(1), 16)
# behavioural trace: union of the ops per address
ops = collections.defaultdict(set); cur = None
for l in open(f'{G}/all.txt', encoding='utf-8', errors='replace'):
    m = re.match(r'^([0-9a-f]{3}) (\w+):', l)
    if m: cur = int(m.group(1), 16); continue
    if cur is not None and l.startswith('  ') and not l.strip().startswith('jump'): ops[cur].add(l.strip())
    elif not l.startswith('  ') and not m: cur = None
# decoded micro ROM (for TYPE)
micro = {}
for l in open(f'{D}/mc68000_micro_decoded.csv', encoding='utf-8'):
    f = l.rstrip('\n').split(',')
    if f[0] == 'addr': continue
    micro[int(f[0], 16)] = dict(I=int(f[2]), fmt=int(f[3]), ty=f[4])
nma = {}
for l in open(f'{D}/mc68000_micro_decoded.csv', encoding='utf-8'):
    f = l.rstrip('\n').split(',')
    if f[0] != 'addr' and f[3] == '0' and f[4] == '0': nma[int(f[0], 16)] = int(f[7], 16)

def feat_G(a):
    F = set()
    for o in ops.get(a, ()):
        o2 = re.sub(r'\b[ad][0-7]([hl])\b', r'R\1', o); o2 = re.sub(r'reg\[[^\]]*\]([hl])', r'R\1', o2)
        if o2 == 'dbh = auh': F.add('au->db')
        elif o2 == 'abh = auh': F.add('au->ab')
        elif o2 == 'aobh = dbh': F.add('db->aob')
        elif o2 == 'aobh = abh': F.add('ab->aob')
        elif o2 == 'aob = au': F.add('au->aob')
        elif o2 == 'pc = au': F.add('au->pc')
        elif o2 == 'pch = dbh': F.add('db->pc')
        elif o2 == 'pch = abh': F.add('ab->pc')
        elif o2 == 'ath = dbh': F.add('db->at')
        elif o2 == 'ath = abh': F.add('ab->at')
        elif o2 == 'dbh = ath': F.add('at->db')
        elif o2 == 'abh = ath': F.add('at->ab')
        elif o2 == 'dth = abh': F.add('ab->dt')
        elif o2 == 'dth = dbh': F.add('db->dt')
        elif o2 == 'abh = dth': F.add('dt->ab')
        elif o2 == 'dbh = dth': F.add('dt->db')
        elif o2 == 'dbh = pch': F.add('pc->db')
        elif o2 == 'abh = pch': F.add('pc->ab')
        elif o2 == 'dbh = Rh': F.add('reg->db')
        elif o2 == 'abh = Rh': F.add('reg->ab')
        elif o2 == 'Rh = dbh': F.add('db->reg')
        elif o2 == 'Rh = abh': F.add('ab->reg')
        elif o2 == 'abd = dbin': F.add('dbin->ab')
        elif o2 == 'dbd = dbin': F.add('dbin->db')
        elif o2.startswith('dbin') and o2.endswith('= ebd'): F.add('edb->dbin')
        elif o2 == 'irc = ebd': F.add('edb->irc')
        elif o2 == 'ird = ir': F.add('ir->ird')
        elif o2 in ('abh = sign(abl)', 'dbh = sign(dbl)'): F.add('sext')
        elif o2.startswith('au = dbh/l'):
            r = o2[len('au = dbh/l'):].strip()
            F.add('au=' + ('0' if r == '' else '+ab' if 'abh' in r else r.replace(' ', '')))
        elif o2.startswith('bus access mode'):
            F.add('write' if o2.endswith('write') else 'read')
    return F

ADDR_BUS = {'db', 'ab', 'dbe', 'abe', 'dbh', 'dbl', 'abh', 'abl'}   # address sections: visible in the trace
def busfam(b):
    b = b.strip()
    if b.startswith('db'): return 'db'
    if b.startswith('ab'): return 'ab'
    return 'any'
# features that the trace shows only for some variants (size/register): they do not count as mismatches
SOFT = {'sext', 'dt->ab', 'dt->db', 'ab->dt', 'db->dt'}

def feat_H(b):
    F = set(); acc = b[4]
    if len(acc) >= 2 and acc[0] in 'it':
        if acc[1] == 'r': F.add('read')
        elif acc[1] == 'w': F.add('write')
    for t in [x.strip() for x in b[10].split(';')]:
        if not t or t.startswith("'"): continue
        parts = [p.strip() for p in t.split('->')]
        if len(parts) == 2:
            s, d = parts
            if s == 'au':
                for x in d.split(','):
                    x = x.strip(); F.add('au->' + x)
            elif s == 'edb':
                for x in d.split(','):
                    x = x.strip(); F.add('edb->' + ('dbin' if x.startswith('dbin') else x))
            elif s == '(ir)' and d == 'ird': F.add('ir->ird')
            elif re.match(r'^[+-]?[0-9]', s) and d == 'au':
                F.add('au=' + ('0' if s == '0' else s.replace(' ', '')))
            continue
        if len(parts) == 3:
            s, bus, d = parts; fam = busfam(bus)
            if bus.endswith('e'): F.add('sext')
            src = None
            if s == 'au': src = 'au'
            elif s.startswith('(pc'): src = 'pc'
            elif s.startswith('(at'): src = 'at'
            elif s == '(dbin)': src = 'dbin'
            elif re.match(r'^\((rx|ry|sp|us)', s): src = 'reg'
            elif s.startswith('(dt'): src = 'dt'
            bus = bus.strip(); addrbus = bus in ADDR_BUS
            star = bus.endswith('*') or bus in ('*', '*e')          # 'ab*','db*' = at least the DATA section; '*' = ab or db
            q = '?' if (star and fam != 'any') else ''               # on ab*/db* the registers may or may not appear in the trace
            low = lambda tok: tok.strip('()').endswith(('l', 'd'))   # low half / DATA section: the trace does not show it
            if src == 'reg' and ((not addrbus and not star) or low(s)): src = None
            if src in ('at', 'pc') and low(s): src = None
            if src: F.add(f'{src}->{fam}' + (q if src == 'reg' else ''))
            for x in [y.strip() for y in d.split(',')]:
                if x == 'aob': F.add(f'{fam}->aob')
                elif x.startswith('at') and not low(x): F.add(f'{fam}->at')
                elif x.startswith('pc') and not low(x): F.add(f'{fam}->pc')
                elif re.match(r'^(rx|ry|sp|us)', x) and (addrbus or star) and not low(x): F.add(f'{fam}->reg' + q)
                elif x.startswith('dt'): F.add(f'{fam}->dt')
                elif x == 'au' and fam == 'ab': F.add('au=+ab')
                elif x == 'au' and fam == 'any': F.add('au=+ab?')
    return F

def norm(s): return s.lower().replace('1', 'l').replace('0', 'o')
RENAME = {'dcnt': 'dbcc', 'leal': 'laal', 'leaw': 'laaw'}
def swap(n): return n[:3] + {'w': 'l', 'l': 'w'}.get(n[3], n[3]) + n[4:] if len(n) >= 5 else n
gal_by_norm = {norm(k): k for k in names}

def candidates(lab):
    lab = lab.rstrip('?').lower(); c = []
    base = [lab]
    for k, v in RENAME.items():
        if lab.startswith(k): base.append(v + lab[len(k):])
    for x in base:
        for y in (x, swap(x)):
            g = gal_by_norm.get(norm(y))
            if g and g not in [q[0] for q in c]:
                c.append((g, 'exact' if y == lab else 'w/l' if y == swap(lab) else 'rename'))
    return c

SOFT |= {'ir->ird'}   # I bit of the MICROword, not of the nanoword: not a mismatch of the nano content
def score(FH, FG):
    fh = set(FH) - SOFT; fg = set(FG) - SOFT
    # optional features ('?'): they count if present in the trace, they are not mismatches if absent
    opt = {f[:-1] for f in fh if f.endswith('?')}; fh = {f for f in fh if not f.endswith('?')}
    # bus '*' / '*e' (App. B: ADDRESS BUS or DATA BUS): alternative ab|db, the trace tells which
    anyf = {f for f in fh if 'any' in f}; fh -= anyf
    auH = {f for f in fh if f.startswith('au=')}; auG = {f for f in fg if f.startswith('au=')}
    fh -= auH; fg -= auG
    match = fh & fg; used = set(match)
    for f in anyf:
        alts = {f.replace('any', 'ab'), f.replace('any', 'db')}
        hit = alts & fg
        if hit: match.add(f); used |= hit
        else: fh.add(f)          # stays as missing
    for f in opt:
        alts = {f.replace('any', 'ab'), f.replace('any', 'db')} if 'any' in f else {f}
        used |= (alts & fg)
    miss = (fh - fg) - anyf | (anyf - match)
    extra = fg - used
    aumiss = set()
    for f in auH:
        alts = {'au=' + p for p in f[3:].split(',')}
        if alts & auG: match.add(f)
        else: aumiss.add(f)
    return len(match) - 2 * (len(miss) + len(aumiss) + len(extra)), sorted(miss | aumiss), sorted(extra)

# size (w/l) of the die A1 entries: from the instruction names in a1-dump.txt (…_b_/…_w_/…_l_ or suffix)
a1size = {}
for l in open(f'{G}/a1-dump.txt'):
    m = re.match(r'^\s*\d+:\s+([0-9a-f]{3})\s+(\S+)\s+[0-9a-f]{4}\s+[0-9a-f]{4}\s+\S+\s+\S+\s+(.*)$', l)
    if not m: continue
    sz = set(re.findall(r'\.([bwl])_', m.group(3)))          # names such as move.l_ais_dd / move.b_ais_dd
    if sz:
        a = int(m.group(1), 16); v = 'l' if sz == {'l'} else ('w' if 'l' not in sz else None)
        a1size[a] = v if a not in a1size or a1size[a] == v else None
def size_of(a):
    return a1size.get(a) or size_of.derived.get(a)
size_of.derived = {}
out = []; n_ok = n_amb = n_mis = 0
for b in B:
    lab = b[2]; FH = feat_H(b); cands = candidates(lab)
    scored = []
    for g, kind in cands:
        a = names[g]; s, miss, extra = score(FH, feat_G(a))
        m = micro.get(a); tyH = b[5]
        tyexp = '0' if tyH.startswith('db') else tyH[1] if tyH in ('a1', 'a2', 'a3') else 'bc'
        ty_ok = (m is not None) and ((tyexp == 'bc' and m['fmt'] == 1) or (tyexp != 'bc' and m['fmt'] == 0 and m['ty'] == tyexp))
        scored.append((s + (0 if ty_ok else -3), g, kind, a, miss, extra, ty_ok))
    scored.sort(key=lambda x: -x[0])
    if not scored:
        out.append([lab, b[1], '-', '-', 'none', 'none', '-', '-', '-', '']); n_mis += 1; continue
    amb = len(scored) > 1 and scored[1][0] >= scored[0][0] - 1
    if amb:
        # 1) sequence: the NMA of the candidate (decoded micro ROM) must point to a candidate of 'next'
        nxt = re.findall(r'^([a-z][a-z0-9#]{3,4})', b[9].strip())
        nxt_addrs = {names[g2] for g2, _ in candidates(nxt[0])} if nxt and nxt[0] not in ('-',) else set()
        good = [sc for sc in scored[:2] if micro.get(sc[3], {}).get('fmt') == 0 and nma.get(sc[3]) in nxt_addrs]
        if len(good) == 1: scored = good + [sc for sc in scored if sc is not good[0]]; amb = False
    if amb and len(lab) >= 5 and lab[3] in 'wl':
        # 2) structure: size of the A1 entry (die PLA, a1-dump) or size inherited from the resolved predecessor (NMA)
        want = lab[3]
        good = [sc for sc in scored[:2] if size_of(sc[3]) == want]
        if len(good) == 1: scored = good + [sc for sc in scored if sc is not good[0]]; amb = False
    s, g, kind, a, miss, extra, ty_ok = scored[0]
    if not amb and len(lab) >= 5 and lab[3] in 'wl' and nma.get(a) is not None: size_of.derived[nma[a]] = lab[3]
    if not ops.get(a): status = 'ambiguous' if amb else 'no_trace'
    else: status = 'ambiguous' if amb else ('mismatch' if (miss or extra) else 'ok')
    if status == 'ok' and not ty_ok: status = 'ok_type'   # content ok, patent TYPE != die (class counted separately)
    if status in ('ok', 'ok_type'): n_ok += 1
    elif status == 'ambiguous': n_amb += 1
    elif status == 'no_trace': n_ok += 0
    else: n_mis += 1
    alt = '|'.join('%s=%03x' % (sc[1], sc[3]) for sc in scored[1:]) if amb else ''
    out.append([lab, b[1], '%03x' % a, g, kind, status, str(s), '|'.join(miss) + ('' if ty_ok else '|TYPE'), '|'.join(extra), alt])
with open(f'{D}/us4325121_appH_die_map.tsv', 'w', encoding='utf-8', newline='\n') as f:
    f.write('# App. H -> die mapping (apph_match.py). label pat_addr die_addr label_galibert match_kind status score missing_in_trace extra_in_trace alternatives_if_ambiguous\n')
    for r in out: f.write('\t'.join(r) + '\n')
print("blocks", len(B), "ok", n_ok, "ambiguous", n_amb, "mismatch", n_mis)
print("mismatch:", [(r[0], r[2], r[7], r[8]) for r in out if r[5] in ('mismatch', 'none')])
print("ambiguous:", [(r[0], r[2], r[3]) for r in out if r[5] == 'ambiguous'])
