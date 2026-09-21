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

import os, re, sys
L=open('all.txt',encoding='utf-8',errors='replace').read().split('\n')
lab2addr={}; addr2ops={}
cur=None
for l in L:
    m=re.match(r'^([0-9a-f]{3}) (\w+):',l)
    if m: cur=m.group(1); lab2addr.setdefault(m.group(2),cur); addr2ops.setdefault(cur,[]); continue
    if l.startswith('  ') and cur: addr2ops[cur].append(l.strip())
def norm(s): return s.replace('1','l').replace('0','o')
labs_norm={norm(l):l for l in lab2addr}
def find(lab): return lab2addr.get(lab) or lab2addr.get(labs_norm.get(norm(lab),''))
rows=[r.strip().split(',') for r in open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'us4325121_appA.csv'),encoding='utf-8') if r.strip() and not r.startswith('#')]
print("rows:",len(rows))
bad=0
for r in rows:
    lab,fc,nma,typ=r[1],r[2],r[3],r[4]
    g=find(lab)
    if not g: print("LABEL not in [G]:",lab); bad+=1; continue
    ops=addr2ops[g]; jumps=[o for o in ops if o.startswith('jump')]; j=jumps[0] if jumps else ''
    if typ.startswith('db'):
        tgt=find(nma)
        if tgt is None or j!=('jump '+tgt): print("NMA?  %-6s type=%-3s nma=%-6s [G]@%s: %s"%(lab,typ,nma,g,j)); bad+=1
    elif typ in ('a1','a2','a3'):
        if j!='jump '+typ: print("TYPE? %-6s type=%s [G]@%s: %s"%(lab,typ,g,j)); bad+=1
    elif typ.startswith('bc'):
        if not j.startswith('jumpcc'): print("BC?   %-6s [G]@%s: %s"%(lab,g,j)); bad+=1
        elif j.split()[1]!=r[5]: print("CBC?  %-6s csv=%s [G]=%s"%(lab,r[5],j.split()[1])); bad+=1
print("mismatches:",bad)
