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

# Consistency checks of the App. H transcription (us4325121_appH.tsv) against App. A (us4325121_appA.csv).
import os, re, collections, csv, io, sys
D=sys.argv[1] if len(sys.argv)>1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
B=[l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_appH.tsv',encoding='utf-8') if l.startswith('p')]
print("blocks:",len(B)); labs=[b[2] for b in B]; c=collections.Counter(labs)
print("duplicate labels:",{k:v for k,v in c.items() if v>1}); labset=set(labs)
miss=collections.Counter()
for b in B:
    for tok in re.findall(r'[a-z][a-z0-9#]{3,4}',b[9]):
        if tok in ('if','end','irc','true','false','mulu','muls','ird','alue','dcr','where'): continue
        if tok not in labset: miss[tok]+=1
print("next labels not found:",dict(miss)); print("origin labels not found:",sorted({b[3] for b in B if b[3] not in labset}))
fn='routine,label,fc,nma,type,cbc,origin,row,dest1,dest2,dest3,dest4'.split(',')
txt='\n'.join(l for l in open(f'{D}/us4325121_appA.csv') if not l.startswith('#'))
A={r['label']:r for r in csv.DictReader(io.StringIO(txt),fieldnames=fn)}
for name,fh,fa in (('TYPE',5,'type'),('ORIGIN',3,'origin')):
    ok=bad=0; bl=[]
    for b in B:
        a=A.get(b[2])
        if not a: continue
        exp=a[fa] if not (fa=='origin' and a[fa] in ('-','')) else b[2]
        if b[fh]==exp: ok+=1
        else: bad+=1; bl.append((b[2],b[fh],exp))
    print(name,"vs App A: ok",ok,"diff",bad,bl[:20])
