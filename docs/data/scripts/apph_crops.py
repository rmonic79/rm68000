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

# Generates the per-column crops of the App. H pages (Google page images US4325121-drawings-page-N.png, 2320x3408).
# Usage: python apph_crops.py <patent_dir> <out_dir>   (pages 85-174: rotation -90; upright pages: 119,123,127,131,135,139,141,143,147,151,155,159,163,167,171 -> no rotation)
import sys, os
from PIL import Image
import numpy as np
src, out = sys.argv[1], sys.argv[2]; os.makedirs(out, exist_ok=True)
UP={119,123,127,131,135,139,141,143,147,151,155,159,163,167,171}
for p in range(85,175):
    im=Image.open(f'{src}/p{p}.png').convert('L')
    if p in UP:
        a=np.array(im)<128; ys,xs=np.where(a)
        im.crop((xs.min()-10,ys.min()-10,xs.max()+10,ys.max()+10)).save(f'{out}/p{p}_U.png'); continue
    im=im.rotate(-90,expand=True); a=np.array(im)<128
    H=a.shape[0]; band=a[int(H*0.22):int(H*0.97)]; col=band.sum(axis=0); zero=col<=1
    ys,xs=np.where(a); y0,y1=ys.min(),ys.max()
    runs=[]; x=0
    while x<len(col):
        if zero[x]:
            s=x
            while x<len(col) and zero[x]: x+=1
            if x-s>=10: runs.append((s,x))
        else: x+=1
    segs=[]; prev=0
    for s,e in runs:
        if s-prev>=450: segs.append((prev,s))
        prev=e
    if len(col)-prev>=450: segs.append((prev,len(col)))
    def save(name,x0,x1): im.crop((max(0,x0-12),max(0,y0-10),min(im.size[0],x1+12),min(im.size[1],y1+10))).save(name)
    if p==85: save(f'{out}/p85_A.png',315,1860); save(f'{out}/p85_B.png',1865,2680)
    elif len(segs)>=3: save(f'{out}/p{p}_A.png',segs[0][0],segs[1][1]); save(f'{out}/p{p}_B.png',segs[2][0],segs[-1][1])
    elif len(segs)==2: save(f'{out}/p{p}_A.png',segs[0][0],segs[0][1]); save(f'{out}/p{p}_B.png',segs[1][0],segs[1][1])
    elif len(segs)==1: save(f'{out}/p{p}_A.png',segs[0][0],segs[0][1])
    else: save(f'{out}/p{p}_A.png',0,im.size[0])
