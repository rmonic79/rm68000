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

# nrom_reach.py: reachability closure of the micro ROM, basis of the proof that the unreachable entries of ORIGIN
# and of the nano ROM can be don't care ('x) without changing the behaviour: a word that is never addressed affects nothing.
# Sources: rtl/cpu_si/m68k_urom.mem (die bits: NMA permuted as in m68k_useq.sv, fmt/ty/cbc), the branch table
# (m68k_branch_pkg.sv: for each cbc the c1c0 outcomes that the table can produce over all 16 values), the exception entry
# constants (m68k_exc_si.sv: A_RESET/A_BSER/A_TRACE/A_INT/A_TRAP/A_STOPW) and the a1/a2/a3 PLA for each opcode (A123_DUMP of
# gen_a123.py). Rules: from a valid opcode the flow starts at a1 and follows NMA, the branches (only the outcomes possible for
# that cbc), a2/a3 of the same opcode; an opcode with a1 = 3ff (illegal/privilege/line A-F) does NOT go to 3ff: the dispatch is
# overridden by a0 = A_TRAP (0e3); the exception routines do not use a2/a3 (checked). Output: docs/data/mc68000_nrom_reach.tsv.
# Aborts if a reachable word is empty in the ROM (that would be a bug, not a don't care).
# Usage: python nrom_reach.py <repo_root> <a123_dump>
import re, sys
R = sys.argv[1]; dump = sys.argv[2]; C = f'{R}/rtl/cpu_si/'
rows = [l.strip() for l in open(C + 'm68k_urom.mem') if l.strip() and not l.startswith('//')]
uw = [int(r, 2) for r in rows]; used = {i for i, w in enumerate(uw) if w != 0}
assert len(uw) == 1024
def nma(w):
    b = lambda i: (w >> i) & 1
    return (b(14) << 9) | (b(13) << 8) | (b(6) << 7) | (b(5) << 6) | (b(10) << 5) | (b(9) << 4) | (b(8) << 3) | (b(7) << 2) | (b(12) << 1) | b(11)
pk = open(C + 'm68k_branch_pkg.sv', encoding='utf-8').read()
body = pk[pk.index("SI_BRANCH_TABLE [0:511] = '{"):]
tab = [int(v) for v in re.findall(r"2'd(\d)", body)][:512]; assert len(tab) == 512
outcomes = {cbc: sorted({tab[(cbc << 4) | v] for v in range(16)}) for cbc in range(32)}
ex = open(C + 'm68k_exc_si.sv', encoding='utf-8').read()
exc = {int(x, 16) for x in re.findall(r"A_(?:RESET|BSER|TRACE|INT|TRAP|STOPW)\s*=\s*10'h([0-9a-f]{3})", ex)}
assert len(exc) >= 4, exc
D = {}
for l in open(dump):
    m = re.findall(r'\b[0-9a-fA-F]{3,4}\b', l)
    if len(m) >= 4: D[int(m[0], 16)] = tuple(int(x, 16) for x in m[1:4])
assert len(D) == 65536, len(D)
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
        # ty == 1: a1 dispatch of the next opcode (all the a1 entries are already starting points) or a0 (exceptions: below)
    memo[key] = seen; return seen
reach = set(); ill = 0
for op, (a1, a2, a3) in D.items():
    if a1 == 0x3ff: ill += 1; continue
    reach |= closure(a1, a2, a3)
for e in exc:
    s = closure(e, 0x3ff, 0x3ff)   # sentinel: if a routine used a2/a3, 3ff would appear
    assert 0x3ff not in s, 'exception routine %03x uses a2/a3' % e
    reach |= s
empty_reached = sorted(reach - used)
assert not empty_reached, 'reachable EMPTY words: ' + ' '.join('%03x' % x for x in empty_reached)
o = [l.strip() for l in open(C + 'm68k_norigin.mem') if l.strip() and not l.startswith('//')]
nidx = sorted({(int(o[x >> 2], 16) << 2) | (x & 3) for x in reach})   # index = {row(addr[9:2]), addr[1:0]}
with open(f'{R}/docs/data/mc68000_nrom_reach.tsv', 'w', encoding='utf-8', newline='\n') as f:
    f.write('# reachable micro words (nrom_reach.py): closure from reset/exceptions/PLA a1-a2-a3, branches with only the table outcomes\n')
    f.write(f'# reachable {len(reach)} of 1024 (used in the ROM {len(used)}, used but unreachable {len(used - reach)}: '
            + ' '.join('%03x' % x for x in sorted(used - reach)) + f'); illegal opcodes (a1=3ff, dispatch to 0e3) {ill}; '
            + f'nano indices reached {len(nidx)}\n')
    for x in sorted(reach): f.write('%03x\n' % x)
print(f'reachable {len(reach)}/1024 | used {len(used)} | used unreachable {sorted("%03x" % x for x in used - reach)} | '
      f'empty reachable {len(empty_reached)} | nano indices reached {len(nidx)} | exceptions {sorted("%03x" % x for x in exc)}')
