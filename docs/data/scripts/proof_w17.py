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

# proof_w17.py <root> <a123_dump>: STATIC proof that a generic write of the register file (AB/DB ports) never has the PC
# (17) as its index. Index 17 comes only from ry_i / rya_i when the source pointer is 'py' AND the instruction has a
# PC-relative EA (IRD[5:3] = 111, IRD[2:1] = 01). So it is enough to show that, for every valid opcode with that EA, no microword
# reachable inside the instruction writes ry/rya/ryh/ryl (ryd/rydl do not use py), and that the exception routines (which
# can start with that IRD still in the register and the py pointer still valid) do not do it either. Closure as in nrom_reach.py.
import re, sys
R = sys.argv[1]; dump = sys.argv[2]; C = f'{R}/rtl/cpu_si/'
uw = [int(l.strip(), 2) for l in open(C + 'm68k_urom.mem') if l.strip() and not l.startswith('//')]; assert len(uw) == 1024
def nma(w):
    b = lambda i: (w >> i) & 1
    return (b(14) << 9) | (b(13) << 8) | (b(6) << 7) | (b(5) << 6) | (b(10) << 5) | (b(9) << 4) | (b(8) << 3) | (b(7) << 2) | (b(12) << 1) | b(11)
pk = open(C + 'm68k_branch_pkg.sv', encoding='utf-8').read()
body = pk[pk.index("SI_BRANCH_TABLE [0:511] = '{"):]
tab = [int(v) for v in re.findall(r"2'd(\d)", body)][:512]; assert len(tab) == 512
outcomes = {cbc: sorted({tab[(cbc << 4) | v] for v in range(16)}) for cbc in range(32)}
ex = open(C + 'm68k_exc_si.sv', encoding='utf-8').read()
exc = {int(x, 16) for x in re.findall(r"A_(?:RESET|BSER|TRACE|INT|TRAP|STOPW)\s*=\s*10'h([0-9a-f]{3})", ex)}; assert len(exc) >= 4
D = {}
for l in open(dump):
    m = re.findall(r'\b[0-9a-fA-F]{3,4}\b', l)
    if len(m) >= 4: D[int(m[0], 16)] = tuple(int(x, 16) for x in m[1:4])
assert len(D) == 65536
memo = {}
def closure(start, a2, a3):
    key = (start, a2, a3)
    if key in memo: return memo[key]
    seen = set(); front = {start}
    while front:
        x = front.pop()
        if x in seen: continue
        seen.add(x); w = uw[x]; n = nma(w); fmt = (w >> 1) & 1; ty = (w >> 2) & 3; cbc = (w >> 2) & 0x1f
        if fmt:
            for c in outcomes[cbc]: front.add((n & 0x300) | (c << 6) | (n & 0x3f))
        elif ty == 0: front.add(n)
        elif ty == 2: front.add(a2)
        elif ty == 3: front.add(a3)
    memo[key] = seen; return seen
words = {}; names = {}
for l in open(f'{R}/docs/data/mc68000_nano_encoding.tsv', encoding='utf-8'):
    f = l.rstrip('\n').split('\t')
    if re.match(r'^[0-9a-f]{3}\t', l): words[int(f[0], 16)] = [x for x in f[4].split(' ; ') if x]; names[int(f[0], 16)] = f[1]
wr = re.compile(r'^(ry|rya|ryh|ryl)<-')
writers = {a for a, A in words.items() if any(wr.match(x) for x in A)}
print('microwords that write ry/rya/ryh/ryl:', len(writers))
npc = 0; bad = {}
for op, (a1, a2, a3) in D.items():
    if a1 == 0x3ff: continue
    if ((op >> 3) & 7) == 7 and ((op >> 1) & 3) == 1:
        npc += 1
        hit = closure(a1, a2, a3) & writers
        if hit: bad[op] = sorted(hit)
print('valid opcodes with a PC-relative EA:', npc, '| of which with a reachable microword that writes ry*:', len(bad))
for op in sorted(bad)[:12]: print('   %04x:' % op, ' '.join('%03x(%s)' % (a, names.get(a, '?')) for a in bad[op]))
ehit = {}
for e in sorted(exc):
    h = closure(e, 0x3ff, 0x3ff) & writers
    if h: ehit[e] = sorted(h)
print('exception routines with a microword that writes ry*:', {('%03x' % e): [('%03x' % a, names.get(a, '?')) for a in h] for e, h in ehit.items()})
# refinement: the py pointer is cleared at every new instruction (new_ird) and only a word with rp_src=py sets it: if the
# closure of the opcode contains no word with rp_src=py, py is never active inside that instruction and the index is never 17
pyw = {a for a, A in words.items() if 'rp_src=py' in A}
still = {op: h for op, h in bad.items() if closure(*D[op]) & pyw}
print('words with rp_src=py:', len(pyw), '| flagged opcodes that ALSO have a reachable py word:', len(still), '(the %d flagged ones are register shifts/rotates: i=1,type=11 give IRD[5:3]=111, not an EA)' % len(bad))
# cross-check: no opcode that reaches a py word also reaches a word that writes ry*
cross = [op for op, (a1, a2, a3) in D.items() if a1 != 0x3ff and ((op >> 3) & 7) == 7 and ((op >> 1) & 3) == 1 and (closure(a1, a2, a3) & pyw) and (closure(a1, a2, a3) & writers)]   # only the IRDs for which py is the PC: for the others py = ry
print('opcodes with IRD[5:3]=111 and IRD[2:1]=01 that reach both a py word and a write of ry*:', len(cross), ['%04x' % o for o in cross[:8]])
bad = still if not cross else {o: [] for o in cross}
print('OUTCOME:', "PROVEN (no generic write can have index 17)" if not bad and not ehit else 'NOT proven')
