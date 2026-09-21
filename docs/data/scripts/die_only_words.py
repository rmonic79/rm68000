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

# Decodes the die words missing from App. H: atoms from the nano map (mc68000_nano_fields.tsv) + ops of the trace.
# Usage: python die_only_words.py <repo_data_dir> <galibert_dir>
import re, sys, collections
D=sys.argv[1]; G=sys.argv[2]
LAB=['aaa01','dvs0e','dvs12','dvs1f','dvs20','rstp6','rstp7','rstp8','rstp9','rstpa','trac1','trac2','trap7','trap8']
names={}
for l in open(f'{G}/adr-names.txt'):
    m=re.match(r'^([0-9a-f]{3})\s+(\S+)',l.strip())
    if m: names[m.group(2).lower()]=int(m.group(1),16)
rows=[l.replace(' ','') for l in open(f'{G}/microcode.txt').read().split('\n') if l.strip()]
rm={}
for l in open(f'{G}/rommap.txt'):
    m=re.match(r'(\d+): ([0-9a-f]{3})-([0-9a-f]{3})',l.strip())
    if m: rm[int(m.group(1))]=(int(m.group(2),16),int(m.group(3),16))
nano_of={}
for r in sorted((r for r in rm if r<=84), key=lambda r:-(rm[r][1]-rm[r][0])):
    a0,a1=rm[r]
    for a in range(a0,a1+1): nano_of[a]=(r,a&3)
def bits(a):
    r,s=nano_of[a]; return {67-k:int(rows[118-r][k*4+s]=='1') for k in range(68)}
ops=collections.defaultdict(set); cur=None
for l in open(f'{G}/all.txt',encoding='utf-8',errors='replace'):
    m=re.match(r'^([0-9a-f]{3}) (\w+):',l)
    if m: cur=int(m.group(1),16); continue
    if cur is not None and l.startswith('  '): ops[cur].add(l.strip())
    elif not l.startswith('  ') and not m: cur=None
# nano map: atom -> (bits, table)
amap={}
for l in open(f'{D}/mc68000_nano_fields.tsv',encoding='utf-8'):
    if l.startswith('#') or not l.strip(): continue
    f=l.rstrip('\n').split('\t')
    if f[1]=='UNRESOLVED': continue
    if f[1]=='' : continue
    bl=[int(x) for x in f[1].split(',')]; tab={}
    for kv in f[3].split():
        k,v=kv.split(':'); tab[int(k,2)]=int(v)
    amap[f[0]]=(bl,tab)
micro={}
for l in open(f'{D}/mc68000_micro_decoded.csv',encoding='utf-8'):
    f=l.rstrip('\n').split(',')
    if f[0]!='addr': micro[int(f[0],16)]=f
# App. H words that share the same nanoword (same row/slot) -> full content for comparison
H={b[2]:b for b in ([l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_appH.tsv',encoding='utf-8') if l.startswith('p')])}
M=[l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_appH_die_map.tsv',encoding='utf-8') if not l.startswith('#')]
share=collections.defaultdict(list)
for r in M:
    if r[5] in ('ok','ok_type'): share[nano_of[int(r[2],16)]].append(r[0])
out=[]; n_dec=0
for lab in LAB:
    a=names.get(lab)
    if a is None: out.append([lab,'-','-','-','-','-']); continue
    b=bits(a); atoms=[]
    for at,(bl,tab) in amap.items():
        k=0
        for i,bit in enumerate(bl): k|=b[bit]<<i
        if tab.get(k,0): atoms.append(at)
    same=share.get(nano_of[a],[])
    m=micro[a]; jump=[o for o in ops[a] if o.startswith('jump')]; opl=sorted(o for o in ops[a] if not o.startswith('jump'))
    dec=bool(atoms) or bool(same)
    n_dec+=dec
    out.append([lab,'%03x'%a,'fmt=%s ty=%s cbc=%s nma=%s fc=%s I=%s'%(m[3],m[4],m[6] or m[5],m[7],m[8],m[2]),
                ' ; '.join(sorted(atoms)),'|'.join(same),' ; '.join(opl+jump)])
with open(f'{D}/mc68000_die_only_words.tsv','w',encoding='utf-8',newline='\n') as f:
    f.write('# die words missing from the patent (die_only_words.py): label die_addr micro nano_atoms(map 144) same_nanoword_as trace_ops\n')
    for r in out: f.write('\t'.join(r)+'\n')
print("decoded",n_dec,"/",len(LAB))
for r in out: print(r[0],r[1],'| atoms:',len(r[3].split(' ; ')) if r[3] else 0,'| shared with:',r[4] or '-','| trace ops:',len(r[5].split(' ; ')) if r[5] else 0)
