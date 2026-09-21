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

import re, collections, itertools, json
exec(open('nano_fields.py').read().split('report={}')[0])   # rows, rm, nano_of, W (col k), ops, addrs, allops
B=lambda a,b: W[a][67-b]          # bit in Galibert numbering (b = 67-k)
N=len(addrs)
def stats(o,pred):
    tp=sum(1 for a in addrs if (o in ops[a]) and pred(a)); nb=sum(1 for a in addrs if pred(a)); no=sum(1 for a in addrs if o in ops[a])
    return tp, nb, no
print("=== single bit: ops with precision>=0.9 (op|bit=1) ===")
for b in range(67,-1,-1):
    nb=sum(1 for a in addrs if B(a,b))
    if nb==0: print("b%02d: never 1"%b); continue
    found=[]
    for o in allops:
        tp,_,no=stats(o,lambda a:B(a,b))
        if no>=2 and tp/nb>=0.9: found.append("%s [p=%.2f r=%.2f]"%(o,tp/nb,tp/no))
    print("b%02d n=%3d: %s"%(b,nb,'; '.join(found)))
print("=== contiguous 2-4 bit windows: purity (most frequent op set per value) ===")
for width in (2,3,4):
    for hi in range(67,width-2,-1):
        bits=list(range(hi,hi-width,-1))
        vals=collections.defaultdict(collections.Counter); cnt=collections.Counter()
        for a in addrs:
            v=''.join(str(B(a,b)) for b in bits); cnt[v]+=1
            for o in ops[a]: vals[v][o]+=1
        # for each value, the ops present in >=90% of the addresses with that value
        desc=[]; distinct=set()
        for v in sorted(cnt):
            core=[o for o,c in vals[v].items() if c>=0.9*cnt[v]]
            desc.append("%s(%d):%s"%(v,cnt[v],'|'.join(core) if core else '-'))
            distinct.add(tuple(sorted(core)))
        if len(distinct)>=min(3,len(cnt)) and len(cnt)>=3:
            print("b%02d..b%02d: %s"%(hi,hi-width+1,'  '.join(desc)))
