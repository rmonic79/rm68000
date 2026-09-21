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

# Discovers the nanoword fields by correlating the raw bits (microcode.txt) <-> the listing (all.txt).
import re, collections, itertools, json, sys
rows=[l.replace(' ','') for l in open('microcode.txt').read().split('\n') if l.strip()]
rm={}
for l in open('rommap.txt'):
    m=re.match(r'(\d+): ([0-9a-f]{3})-([0-9a-f]{3})',l.strip())
    if m: rm[int(m.group(1))]=(int(m.group(2),16),int(m.group(3),16))
nano_of={}
for r in sorted((r for r in rm if r<=84), key=lambda r: -(rm[r][1]-rm[r][0])):
    a0,a1=rm[r]
    for a in range(a0,a1+1): nano_of[a]=(r,a&3)
micro_addrs=set()
for r in range(118,84,-1): micro_addrs|=set(range(rm[r][0],rm[r][0]+16))
def w_of(a):
    r,s=nano_of[a]; return tuple(int(rows[118-r][k*4+s]=='1') for k in range(68))
L=open('all.txt',encoding='utf-8',errors='replace').read().split('\n')
ops=collections.defaultdict(set); cur=None
def norm(s):
    s=re.sub(r'reg\[irc >> 12\]','Rirc',s); s=re.sub(r'reg\[mvm\.reg\(\)\]','Rmvm',s)
    s=re.sub(r'\b[ad][0-7]([hl]?)\b',r'R\1',s)
    s=re.sub(r'bus access mode (instruction|data|<error>|interrupt) ','bus ',s)
    return s
for l in L:
    m=re.match(r'^([0-9a-f]{3}) (\w+):',l)
    if m: cur=int(m.group(1),16); continue
    if cur is not None and l.startswith('  ') and not l.strip().startswith('jump'):
        s=norm(l.strip())
        if s in ('ir = irc','ird = ir'): continue   # micro bit I
        ops[cur].add(s)
    if not l.startswith('  ') and not m: cur=None
addrs=sorted(a for a in micro_addrs if a in ops)
W={a:w_of(a) for a in addrs}
allops=sorted({o for a in addrs for o in ops[a]})
# groups by destination
def dest(o):
    if ' = ' in o: return o.split(' = ')[0]
    if o.startswith('bus '): return 'bus'
    if '<->' in o: return 'swap:'+o
    return o
groups=collections.defaultdict(set)
for o in allops: groups[dest(o)].add(o)
# merging of obvious groups: (abh,abl) have separate fields; 'au' function; left as is
report={}
def solve(g,alts):
    cls={}
    for a in addrs:
        here=[o for o in alts if o in ops[a]]
        cls[a]=tuple(here) if here else ()
    classes=collections.defaultdict(list)
    for a,c in cls.items(): classes[c].append(a)
    nonempty=[c for c in classes if c!=()]
    none_addrs=classes.get((),[])
    cand=[]; vec={}
    for c in range(68):
        okc=True
        for cl in nonempty:
            if len({W[a][c] for a in classes[cl]})>1: okc=False; break
        if okc: cand.append(c)
    # greedy: separates the classes + none with columns of cand, maximising the distinct partitions
    def keyf(cols): 
        return collections.Counter((tuple(W[a][c] for c in cols), cls[a]) for a in addrs)
    def conflicts(cols):
        d=collections.defaultdict(set)
        for a in addrs: d[tuple(W[a][c] for c in cols)].add(cls[a])
        return sum(len(v)-1 for v in d.values())
    cols=[]
    while conflicts(cols)>0 and len(cols)<12:
        best=min((c for c in cand if c not in cols), key=lambda c: conflicts(cols+[c]), default=None)
        if best is None: break
        cols.append(best)
    if conflicts(cols)>0: return None,None,classes,cand
    # minimise: try removing columns
    changed=True
    while changed:
        changed=False
        for c in list(cols):
            t=[x for x in cols if x!=c]
            if conflicts(t)==0: cols=t; changed=True
    keys={}
    for cl in nonempty: keys[cl]=tuple(W[classes[cl][0]][c] for c in cols)
    return sorted(cols),keys,classes,cand
for g,alts in sorted(groups.items()):
    alts=sorted(alts)
    cols,keys,classes,cand=solve(g,alts)
    ncls={(' & '.join(k) or '-'):len(v) for k,v in classes.items()}
    if cols:
        enc={''.join(map(str,v)):' & '.join(k) for k,v in sorted(keys.items(),key=lambda x:x[1])}
        report[g]={'cols':cols,'enc':enc,'classes':ncls}
        print("%-16s cols=%-22s %s"%(g,cols,enc),flush=True)
    else:
        report[g]={'cols':None,'cand':cand,'classes':ncls}
        print("%-16s NOT separable (<=6 cols) among %d candidates; classes=%s"%(g,len(cand),ncls),flush=True)
json.dump(report,open('nano_fields_greedy.json','w'),indent=1)
