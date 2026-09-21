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

import re, collections, itertools, json, math
exec(open('nano_fields.py').read().split('report={}')[0])   # reuses the parsing: rows, rm, nano_of, W, ops, addrs, allops
# --- structure from the bits alone: 336 physical words
words=[tuple(int(rows[118-r][k*4+s]=='1') for k in range(68)) for r in range(84,0,-1) for s in range(4)]
const=[c for c in range(68) if len({w[c] for w in words})==1]
print("constant columns:",const)
ones=[sum(w[c] for w in words) for c in range(68)]
print("population per column:",ones)
# mutually exclusive pairs
excl={(a,b) for a in range(68) for b in range(a+1,68) if not any(w[a] and w[b] for w in words)}
# maximal one-hot groups (greedy clique)
groups=[]; used=set()
for a in range(68):
    if a in used or a in const: continue
    g=[a]
    for b in range(a+1,68):
        if b in used or b in const: continue
        if all((x,b) in excl for x in g): g.append(b)
    if len(g)>1: groups.append(g); used|=set(g)
print("one-hot groups (mutually exclusive):",groups)
eq=[(a,b) for a in range(68) for b in range(a+1,68) if all(w[a]==w[b] for w in words)]
comp=[(a,b) for a in range(68) for b in range(a+1,68) if all(w[a]!=w[b] for w in words)]
print("equal columns:",eq," complementary:",comp)
# --- decision tree for each op (present/absent) over the 512 addresses
def entropy(items):
    n=len(items); 
    if n==0: return 0
    p=sum(items)/n
    return 0 if p in (0,1) else -(p*math.log2(p)+(1-p)*math.log2(1-p))
def build(idx,y,depth,cols_used):
    ys=[y[i] for i in idx]
    if all(ys): return ('L',1)
    if not any(ys): return ('L',0)
    if depth>8: return ('L',round(sum(ys)/len(ys)))
    best=None
    for c in range(68):
        l=[i for i in idx if W[addrs[i]][c]==0]; r=[i for i in idx if W[addrs[i]][c]==1]
        if not l or not r: continue
        gain=entropy(ys)-(len(l)*entropy([y[i] for i in l])+len(r)*entropy([y[i] for i in r]))/len(idx)
        if best is None or gain>best[0]: best=(gain,c,l,r)
    if best is None: return ('L',round(sum(ys)/len(ys)))
    g,c,l,r=best; cols_used.add(c)
    return ('N',c,build(l,y,depth+1,cols_used),build(r,y,depth+1,cols_used))
def leaves(t): return 1 if t[0]=='L' else leaves(t[2])+leaves(t[3])
def show(t,ind=''):
    if t[0]=='L': return str(t[1])
    return "(c%d? %s : %s)"%(t[1],show(t[3]),show(t[2]))
def predict(t,w):
    while t[0]=='N': t=t[3] if w[t[1]] else t[2]
    return t[1]
out={}
for o in allops:
    y=[1 if o in ops[a] else 0 for a in addrs]
    if sum(y)<2: continue
    cu=set(); t=build(list(range(len(addrs))),y,0,cu)
    err=sum(1 for i,a in enumerate(addrs) if predict(t,W[a])!=y[i])
    out[o]={'n':sum(y),'cols':sorted(cu),'leaves':leaves(t),'err':err,'tree':show(t)}
    print("%-24s n=%3d err=%d leaves=%2d cols=%s"%(o,sum(y),err,leaves(t),sorted(cu)))
json.dump(out,open('nano_tree.json','w'),indent=1)
