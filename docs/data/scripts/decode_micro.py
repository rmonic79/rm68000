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

# Decodes the MC68000 micro ROM from the raw die bits (Galibert microcode.txt + rommap.txt)
# with the bit semantics of patent US 4,325,121 (bit0 I, bit1 format, bit2-3 TY / bit2-6 CBC,
# bit5-14 NMA / bit7-14 NMBA, bit15-16 FC). Physical layout derived by us:
#  - file row k (1..118) = rommap row 118-k+1; rows 118..85 = micro, 16 words/row, 17 groups of 16 bits,
#    group g = patent bit (16-g), word w = base address+w;
#  - the 10 NMA bits are physically permuted: A0<-b11 A1<-b12 A2<-b7 A3<-b8 A4<-b9 A5<-b10 A6<-b5 A7<-b6 A8<-b13 A9<-b14
#    (check against the all.txt listing, 516 addresses: 399/399 numeric jumps, 47/47 a1/a2/a3, 70/70 jumpcc);
#  - rows 84..1 = nano, 4 words/row (68 groups of 4 bits), word = address & 3, row from rommap; where the rommap ranges
#    are nested (084-08f/088-08b, 0a4-0af/0a8-0ab, 158-15f/158-15b, 178-17f/178-17b) the narrowest range wins, as in
#    nano_map.py, nano_fields.py and die_only_words.py.
import re, csv
rows=[l.replace(' ','') for l in open('microcode.txt').read().split('\n') if l.strip()]
rm={}
for l in open('rommap.txt'):
    m=re.match(r'(\d+): ([0-9a-f]{3})-([0-9a-f]{3})',l.strip())
    if m: rm[int(m.group(1))]=(int(m.group(2),16),int(m.group(3),16))
PERM={0:11,1:12,2:7,3:8,4:9,5:10,6:5,7:6,8:13,9:14}
COND={0:'i11',1:'auz',2:'c',3:'z',4:'nz1',5:'n',6:'nz2',7:'ms0',8:'m01',9:'cc',10:'ze',11:'nv',12:'d4',13:'v',14:'enl'}
names={}
for l in open('adr-names.txt'):
    m=re.match(r'^([0-9a-f]{3})\s+(\S+)',l.strip())
    if m: names[int(m.group(1),16)]=m.group(2)
nano_of={}
for r in sorted((r for r in rm if r<=84), key=lambda r:-(rm[r][1]-rm[r][0])):   # widest first: the inner row overwrites
    a0,a1=rm[r]
    for a in range(a0,a1+1): nano_of[a]=(r,a&3)
out=csv.writer(open('micro_decoded.csv','w',newline=''))
out.writerow(['addr','label','I','fmt','ty','cbc','cond','nma_or_base','fc','nano_row','nano_slot','raw17'])
for r in range(118,84,-1):
    bits=[1 if c=='1' else 0 for c in rows[118-r]]
    a0,_=rm[r]
    for w in range(16):
        a=a0+w; wd=[bits[(16-k)*16+w] for k in range(17)]
        nma=sum(wd[PERM[j]]<<j for j in range(10)); fmt=wd[1]
        ty=wd[2]|(wd[3]<<1); cbc=sum(wd[2+i]<<i for i in range(5))
        cond=(COND.get(cbc&15,'?')+("'" if cbc&16 else '')) if fmt else ''
        out.writerow(['%03x'%a,names.get(a,''),wd[0],fmt,'' if fmt else ty,cbc if fmt else '',cond,
                      '%03x'%(nma&~0xC0 if fmt else nma),wd[15]|(wd[16]<<1),nano_of[a][0],nano_of[a][1],''.join(map(str,wd))])
print("wrote micro_decoded.csv")
