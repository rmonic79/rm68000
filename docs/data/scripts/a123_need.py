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

# a123_need.py: for each opcode, whether its microword flow really reads A2 (a word with ty = 2). Basis of the proof
# that A2 can be don't care ('x) in the PLA cells whose opcodes never read it: a value that is never read has no effect.
# Rules: the flow starts at the opcode's a1 and follows NMA, the branches with only the outcomes the table produces for that
# cbc, and a2/a3 of the same opcode; opcodes with a1 = 3ff (illegal) never start (dispatch overridden by a0 = A_TRAP) and do not
# read A2; the exception routines and trap1 (1d0, missing cells) have no ty = 2/3 words (checked here). A3 is NOT per opcode:
# five end-of-instruction words (004, 005, 024, 026, 21b) read A3 AFTER IR is loaded, i.e. the A3 of the NEXT opcode:
# A3 is kept in full. The script aborts if a ty = 2 word were reached after an IR load (A2 of the next opcode).
# Output: docs/data/mc68000_a123_need.tsv (opcode \t need2). Usage: python a123_need.py <repo_root> <a123_dump>
import re, sys
R = sys.argv[1]; dump = sys.argv[2]; C = f'{R}/rtl/cpu_si/'
rows = [l.strip() for l in open(C + 'm68k_urom.mem') if l.strip() and not l.startswith('//')]
uw = [int(r, 2) for r in rows]; assert len(uw) == 1024
def nma(w):
    b = lambda i: (w >> i) & 1
    return (b(14) << 9) | (b(13) << 8) | (b(6) << 7) | (b(5) << 6) | (b(10) << 5) | (b(9) << 4) | (b(8) << 3) | (b(7) << 2) | (b(12) << 1) | b(11)
pk = open(C + 'm68k_branch_pkg.sv', encoding='utf-8').read()
body = pk[pk.index("SI_BRANCH_TABLE [0:511] = '{"):]
tab = [int(v) for v in re.findall(r"2'd(\d)", body)][:512]; assert len(tab) == 512
outc = {cbc: sorted({tab[(cbc << 4) | v] for v in range(16)}) for cbc in range(32)}
ex = open(C + 'm68k_exc_si.sv', encoding='utf-8').read()
exc = {int(x, 16) for x in re.findall(r"A_(?:RESET|BSER|TRACE|INT|TRAP|STOPW)\s*=\s*10'h([0-9a-f]{3})", ex)}
D = {}
for l in open(dump):
    f = l.rstrip('\n').split('\t')
    if len(f) >= 5: D[int(f[0], 16)] = (int(f[1], 16), int(f[2], 16), int(f[3], 16), int(f[4]))
assert len(D) == 65536
TRAP1 = None
memo = {}
def flow(start, a2, a3):   # -> (words, reads a2, reads a3, ty=2 words after the IR load, ty=3 words after it)
    key = (start, a2, a3)
    if key in memo: return memo[key]
    seen = set(); front = {(start, False)}; use2 = use3 = False; after2 = set(); after3 = set()
    while front:
        x, irl = front.pop()
        if (x, irl) in seen: continue
        seen.add((x, irl)); w = uw[x]; n = nma(w); fmt = (w >> 1) & 1; ty = (w >> 2) & 3; cbc = (w >> 2) & 0x1f; irl2 = irl or bool(w & 1)
        if fmt:
            for c in outc[cbc]: front.add(((n & 0x300) | (c << 6) | (n & 0x3f), irl2))
        elif ty == 0: front.add((n, irl2))
        elif ty == 2: use2 = True; (after2.add(x) if irl else None); front.add((a2, irl2))
        elif ty == 3: use3 = True; (after3.add(x) if irl else None); front.add((a3, irl2))
    memo[key] = ({x for x, _ in seen}, use2, use3, after2, after3); return memo[key]
for e in exc:
    s, u2, u3, _, _ = flow(e, 0x3ff, 0x3ff); assert not u2 and not u3, 'exception routine %03x reads a2/a3' % e
need2 = {}; after2 = set(); after3 = set(); n2 = n3 = 0
for op, (a1, a2, a3, ill) in D.items():
    if ill: need2[op] = False; continue
    s, u2, u3, a2w, a3w = flow(a1, a2, a3); need2[op] = u2; n2 += u2; n3 += u3; after2 |= a2w; after3 |= a3w
assert not after2, 'ty=2 words read after the IR load: ' + ' '.join('%03x' % x for x in sorted(after2))
valid = sum(1 for v in D.values() if not v[3])
with open(f'{R}/docs/data/mc68000_a123_need.tsv', 'w', encoding='utf-8', newline='\n') as f:
    f.write('# opcode -> A2 read in its own flow (a123_need.py); A3 is kept in full: it is also read after the IR load by words '
            + ' '.join('%03x' % x for x in sorted(after3)) + '\n')
    f.write(f'# valid opcodes {valid}: reading A2 {n2}, A3 {n3}; illegal {65536 - valid} (A2 never read)\n')
    for op in range(65536): f.write('%04x\t%d\n' % (op, int(need2[op])))
print(f'valid opcodes {valid} | reading A2 {n2} (don\'t care {valid - n2} + {65536 - valid} illegal) | reading A3 {n3} | A3 after IR: {sorted("%03x" % x for x in after3)} | A2 after IR: {len(after2)}')
