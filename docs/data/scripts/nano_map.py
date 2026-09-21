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

# nano bit -> field map, by correlating the 68 bits of the die (microcode.txt/rommap.txt) with the
# App. H blocks matched with status ok (us4325121_appH_die_map.tsv). Each semantic "atom" of the block
# (transfer, constant, ACCESS/ALU/REGISTER POINTERS cell) is explained by the smallest group of
# bits (1..4) of which it is an exact function over the sure blocks; the atoms with the same group form a field.
# Usage: python nano_map.py <repo_data_dir> <galibert_dir>
import re, sys, collections, itertools
import numpy as np
D = sys.argv[1]; G = sys.argv[2]
rows = [l.replace(' ', '') for l in open(f'{G}/microcode.txt').read().split('\n') if l.strip()]
rm = {}
for l in open(f'{G}/rommap.txt'):
    m = re.match(r'(\d+): ([0-9a-f]{3})-([0-9a-f]{3})', l.strip())
    if m: rm[int(m.group(1))] = (int(m.group(2), 16), int(m.group(3), 16))
nano_of = {}
for r in sorted((r for r in rm if r <= 84), key=lambda r: -(rm[r][1] - rm[r][0])):
    a0, a1 = rm[r]
    for a in range(a0, a1 + 1): nano_of[a] = (r, a & 3)
def nano_bits(a):      # tuple of 68 bits in Galibert numbering b = 67-k
    r, s = nano_of[a]; return tuple(int(rows[118 - r][k * 4 + s] == '1') for k in range(67, -1, -1))
H = {b[2]: b for b in ([l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_appH.tsv', encoding='utf-8') if l.startswith('p')])}
M = [l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_appH_die_map.tsv', encoding='utf-8') if not l.startswith('#')]

def atoms(b):
    A = set()
    acc = b[4]
    if acc == 'np': A |= {'acc_class=n', 'acc_mode=p'}
    elif len(acc) == 4: A |= {f'acc_class={acc[0]}', f'acc_mode={acc[1]}', f'acc_type={acc[2:]}'}
    elif acc != '-': A.add(f'acc={acc}')
    alu = b[6]
    if alu != 'x' and alu != '-':
        A.add(f'alu_col={alu[0]}')
        if len(alu) > 1: A.add(f'alu_cc={alu[1]}')
        if len(alu) > 2: A.add('alu_byte')
    rp = b[7]
    if rp not in ('-', ''): A |= {f'rp_dst={rp[:2]}', f'rp_src={rp[2:]}'}
    for t in [x.strip() for x in b[10].split(';')]:
        if not t: continue
        if t.startswith("'") or t == 'reset pren': A.add(t.strip("'")); continue
        p = [x.strip() for x in t.split('->')]
        if len(p) == 3:
            s, bus, d = p; A.add(f'{bus}<-{s}')
            for x in d.split(','): A.add(f'{x.strip()}<-{bus}')
        elif len(p) == 2:
            s, d = p
            for x in d.split(','):
                x = x.strip()
                if re.match(r'^[+-]?\d', s): A.add(f'{x}+={s}')
                else: A.add(f'{x}<-{s}')
    return A

sure = [(int(r[2], 16), r[0]) for r in M if r[5] in ('ok', 'ok_type')]
X = np.array([nano_bits(a) for a, _ in sure], dtype=np.uint8)        # N x 68, column j = Galibert bit 67-j
bitname = [67 - j for j in range(68)]
AT = [atoms(H[lab]) for _, lab in sure]
allatoms = sorted(set().union(*AT))
Y = {a: np.array([a in s for s in AT], dtype=np.uint8) for a in allatoms}
N = len(sure)
print("sure blocks", N, "distinct atoms", len(allatoms))

EXC = int(sys.argv[3]) if len(sys.argv) > 3 else 0   # allowed exceptions (0 = exact function; with 7 the map grows and the cross-check gets worse)
def conflicts(cols, y):
    key = np.zeros(N, dtype=np.int64)
    for i, c in enumerate(cols): key |= X[:, c].astype(np.int64) << i
    n1 = np.bincount(key, weights=y, minlength=1 << len(cols)); n = np.bincount(key, minlength=1 << len(cols))
    return int(np.minimum(n1, n - n1).sum()), key, n1, n
def find(y, kmax=4):
    if y.all() or (~y.astype(bool)).all(): return (), {}, 0
    corr = [(abs(np.corrcoef(X[:, c], y)[0, 1]) if X[:, c].std() > 0 else 0, c) for c in range(68)]
    order = [c for _, c in sorted(corr, reverse=True)]
    for k in range(1, kmax + 1):
        pool = order if k <= 2 else order[:30 if k == 3 else 16]
        best = None
        for cols in itertools.combinations(pool, k):
            e, key, n1, n = conflicts(cols, y)
            if best is None or e < best[0]: best = (e, cols, n1, n)
            if e == 0: break
        e, cols, n1, n = best
        if e <= EXC:
            t = {int(k): int(n1[k] * 2 > n[k]) for k in range(len(n)) if n[k] > 0}   # majority
            return cols, t, e
    return None, None, None
res = {}; exc_total = 0
for a in allatoms:
    cols, t, e = find(Y[a])
    res[a] = (cols, t)
    if e: exc_total += e
solved = {a for a, (c, t) in res.items() if c is not None}
unsolved = [a for a in allatoms if res[a][0] is None]
# fields = groups of atoms with the same set of bits
fields = collections.defaultdict(list)
for a in solved:
    fields[tuple(sorted(bitname[c] for c in res[a][0]))].append(a)
with open(f'{D}/mc68000_nano_fields.tsv', 'w', encoding='utf-8', newline='\n') as f:
    f.write('# nano bit -> field map (nano_map.py). bits in Galibert numbering (b=67-k). For each atom: bits, key->presence table\n')
    for a in allatoms:
        cols, t = res[a]
        if cols is None: f.write(f'{a}\tUNRESOLVED\t\t\n'); continue
        bits = ','.join(str(bitname[c]) for c in cols)
        tab = ' '.join(f"{k:0{len(cols)}b}:{v}" for k, v in sorted(t.items())) if cols else 'constant'
        f.write(f'{a}\t{bits}\t{Y[a].sum()}/{N}\t{tab}\n')
    f.write('\n# FIELDS (bit groups) and the atoms that use them\n')
    for bits, al in sorted(fields.items(), key=lambda x: (len(x[0]), x[0])):
        f.write(f'# bit {bits}: ' + ' | '.join(sorted(al)) + '\n')
print("atoms solved", len(solved), "/", len(allatoms), "fields (bit groups)", len(fields), "; exceptions (disagreements) over the", N, "blocks:", exc_total)
print("NOT solved:", unsolved)
# 2-fold cross-check: map from one half, prediction on the other half
rng = np.random.RandomState(1); idx = rng.permutation(N); halves = [idx[:N // 2], idx[N // 2:]]
dis = 0; tot = 0
for h in (0, 1):
    tr, te = halves[h], halves[1 - h]
    for a in solved:
        cols, _ = res[a]
        if not cols: continue
        key = np.zeros(N, dtype=np.int64)
        for i, c in enumerate(cols): key |= X[:, c].astype(np.int64) << i
        table = {}
        for k, v in zip(key[tr], Y[a][tr]): table.setdefault(k, v)
        for k, v in zip(key[te], Y[a][te]):
            tot += 1
            if k in table and table[k] != v: dis += 1
print("cross-check 2-fold: disagreements", dis, "over", tot, "predictions")
# applied to the mismatched and ambiguous blocks: decode again from the bits and compare with the block atoms
def decode(a):
    bits = np.array(nano_bits(a), dtype=np.uint8); out = set()
    for at in solved:
        cols, t = res[at]
        if not cols:
            if Y[at].all(): out.add(at)
            continue
        k = 0
        for i, c in enumerate(cols): k |= int(bits[c]) << i
        if t.get(k, 0): out.add(at)
    return out
names = {}
for l in open(f'{G}/adr-names.txt'):
    m = re.match(r'^([0-9a-f]{3})\s+(\S+)', l.strip())
    if m: names[m.group(2).lower()] = int(m.group(1), 16)
fixed_mis = 0; n_mis = 0; fixed_amb = 0; n_amb = 0; lines = []
for r in M:
    lab = r[0]; want = atoms(H[lab]) & solved
    if r[5] == 'mismatch':
        n_mis += 1; a = int(r[2], 16); got = decode(a) & set(want | (solved - want))
        ok = (decode(a) & solved) == want
        fixed_mis += ok; lines.append(f'mismatch\t{lab}\t{r[2]}\t{"resolved" if ok else "no"}\t{"|".join(sorted(want - decode(a)))}\t{"|".join(sorted((decode(a) & solved) - want))}')
    elif r[5] == 'ambiguous':
        n_amb += 1; cands = [int(r[2], 16)] + [int(x.split('=')[1], 16) for x in r[9].split('|') if '=' in x]
        hits = [c for c in cands if (decode(c) & solved) == want]
        ok = len(hits) == 1
        fixed_amb += ok; lines.append(f'ambiguous\t{lab}\t{",".join("%03x" % c for c in cands)}\t{"resolved->%03x" % hits[0] if ok else "no"}\t{len(hits)} consistent candidates')
with open(f'{D}/mc68000_nano_map_apply.tsv', 'w', encoding='utf-8', newline='\n') as f:
    f.write('# nano map applied to the mismatched/ambiguous blocks (nano_map.py)\n' + '\n'.join(lines) + '\n')
print("mismatches solved by the map", fixed_mis, "/", n_mis, "; ambiguous solved", fixed_amb, "/", n_amb)
