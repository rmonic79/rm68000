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

# Nano ROM with OUR OWN ENCODING + ORIGIN table, from the 500 App. H blocks (us4325121_appH.tsv), from the App. H -> die
# mapping (us4325121_appH_die_map.tsv), from the die-only words (mc68000_die_only_words.tsv) and from the die trace
# (all.txt, used ONLY to resolve the '*'/'*e' buses of App. H into the physical bus).
# Encoding: every "atom" of the block (transfer, constant, ACCESS/ALU/REGISTER POINTERS cell) is a boolean action.
# Atoms that NEVER appear together in the same word are packed into the same enumerated SLOT
# (ceil(log2(n+1)) bits, 0 = no action of the slot): it is the mutually exclusive field compression of the microcode
# (the silicon does the same with 68 bits + decoder). Words with the same set of atoms share the nanoword
# (our ORIGIN, finer than the silicon one). RTL decoding: atom = (nw[LSB +: W] == CODE).
# Usage: python gen_nrom.py <repo_root> <galibert_dir>
import re, sys, collections, math, random
# License header written at the top of every generated file (the same text as in the hand-written sources).
LICENSE_HEADER = ('// This file is part of rm68000, an MC68000 core in SystemVerilog.\n'
                  '// Copyright (c) 2026 Umberto Parisi (rmonic79)\n'
                  '//\n'
                  '// rm68000 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public\n'
                  '// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any\n'
                  '// later version. It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the\n'
                  '// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for\n'
                  '// more details: LICENSE in the root of the repository, or <https://www.gnu.org/licenses/>.\n'
                  '//\n'
                  '// A commercial license, for use without the obligations of the GPL (for example in closed-source products), is\n'
                  '// available from the author, who can also be contacted for other licensing needs.\n'
                  '\n')
R = sys.argv[1]; G = sys.argv[2]; D = f'{R}/docs/data'
H = {b[2]: b for b in ([l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_appH.tsv', encoding='utf-8') if l.startswith('p')])}
M = [l.rstrip('\n').split('\t') for l in open(f'{D}/us4325121_appH_die_map.tsv', encoding='utf-8') if not l.startswith('#')]

def atoms(b):   # same as nano_map.py
    A = set()
    acc = b[4]
    if acc == 'np': A |= {'acc_class=n', 'acc_mode=p'}
    elif len(acc) == 4: A |= {f'acc_class={acc[0]}', f'acc_mode={acc[1]}', f'acc_type={acc[2:]}'}
    elif acc != '-': A.add(f'acc={acc}')
    alu = b[6]
    if alu != 'x' and alu != '-':
        A.add('alu_col=' + ('1' if alu[0] == 'i' else alu[0]))   # App. H "in" = "1n" (FIG. 12: columns 1-5, 6, x; 'i' is a misprint for '1')
        if len(alu) > 1: A.add(f'alu_cc={alu[1]}')
        if len(alu) > 2: A.add('alu_byte')
    rp = b[7]
    if rp not in ('-', ''): A |= {f'rp_dst={rp[:2]}', f'rp_src={rp[2:]}'}
    for t in [x.strip() for x in b[10].split(';')]:
        if not t: continue
        if t.startswith("'") or t == 'reset pren': A.add(t.strip("'")); continue
        p = [x.strip() for x in t.split('->')]
        if len(p) == 3:
            s, bus, d = p; A.add(f'{bus}<-{s}')
            for x in d.split(','): A.add(f'{x.strip()}<-{bus}')
        elif len(p) == 2:
            s, d = p
            for x in d.split(','):
                x = x.strip()
                if re.match(r'^[+-]?\d', s): A.add(f'{x}+={s}')
                else: A.add(f'{x}<-{s}')
    return A

names = {}
for l in open(f'{G}/adr-names.txt'):
    m = re.match(r'^([0-9a-f]{3})\s+(\S+)', l.strip())
    if m: names[int(m.group(1), 16)] = m.group(2)

# --- die trace (all.txt): ops per address
ops = collections.defaultdict(set); cur = None
for l in open(f'{G}/all.txt', encoding='utf-8', errors='replace'):
    m = re.match(r'^([0-9a-f]{3}) (\w+):', l)
    if m: cur = int(m.group(1), 16); continue
    if cur is not None and l.startswith('  '): ops[cur].add(l.strip())
    elif not l.startswith('  '): cur = None
# --- content per die address ---
content = {}; src = {}; collisions = 0
for r in M:
    a = int(r[2], 16); A = atoms(H[r[0]])
    if a in content:
        if content[a] != A: collisions += 1
        if r[5] not in ('ok', 'ok_type'): continue
    content[a] = A; src[a] = f'appH:{r[0]}:{r[5]}'
die_only = 0; die_partial = 0
for l in open(f'{D}/mc68000_die_only_words.tsv', encoding='utf-8'):
    if l.startswith('#'): continue
    f = l.rstrip('\n').split('\t'); a = int(f[1], 16)
    if a in content: continue
    same = [x for x in f[4].split('|') if x]
    if same and any(int(m[2], 16) in content for m in M if m[0] == same[0]):
        aa = [int(m[2], 16) for m in M if m[0] == same[0]][0]
        content[a] = set(content[aa]); src[a] = f'die:{f[0]}=nano of {same[0]}'
    else:
        content[a] = set(x for x in f[3].split(' ; ') if x); src[a] = f'die:{f[0]}:partial(nano map)'; die_partial += 1
    die_only += 1
# Die-only words with partial content: the nano map is wrong (e.g. rstp7 "f/w" while the trace says instruction word read):
# the atoms come from the die TRACE (op -> atom table; class: access + dbin=ebd in the same word = t, access only = i,
# dbin=ebd only = f). Not modelled: dt (data temporary), couplers abh<->abl / dbl<->dbd (handled by the fallbacks of the fabric).
def trace_atoms(a):
    A = set(); acc = None; dbin_ld = False
    for o in ops.get(a, ()):
        o2 = re.sub(r'\b[ad][0-7]([hl])\b', r'R\1', o); o2 = re.sub(r'reg\[[^\]]*\]([hl])', r'R\1', o2)
        if o2.startswith('bus access mode'):
            acc = ('w' if o2.endswith('write') else 'r', 'in' if 'instruction' in o2 else 'op')
        elif o2 == 'dbin = ebd': A.add('dbin<-edb'); dbin_ld = True
        elif o2 == 'irc = ebd': A.add('irc<-edb')
        elif o2 == 'ird = ir': A.add('ird<-(ir)')
        elif o2.startswith('au = dbh/l'):
            A.add('au<-db')
            if 'abh' in o2: A.add('au<-ab')
            m = re.search(r'([+-]) (\d+)$', o2); A.add('au+=' + (m.group(1) + m.group(2) if m else '0'))
        elif o2 in ('dbh = auh', 'dbl = aul'): A.add('db<-au')
        elif o2 in ('abh = auh', 'abl = aul'): A.add('ab<-au')
        elif o2 in ('aobh = dbh', 'aobl = dbl'): A.add('aob<-db')
        elif o2 in ('aobh = abh', 'aobl = abl'): A.add('aob<-ab')
        elif o2 == 'aob = au': A.add('aob<-au')
        elif o2 == 'abd = dbin': A.add('abd<-(dbin)')
        elif o2 == 'dbd = dbin': A.add('dbd<-(dbin)')
        elif o2 == 'ath = abh': A.add('ath<-ab')
        elif o2 == 'atl = abl': A.add('atl<-ab')
        elif o2 == 'ath = dbh': A.add('ath<-db')
        elif o2 == 'atl = dbl': A.add('atl<-db')
        elif o2 == 'pc = au': A.add('pc<-au')
        elif o2 in ('pch = dbh', 'pcl = dbl'): A.add('pc<-db')
        elif o2 in ('pch = abh', 'pcl = abl'): A.add('pc<-ab')
        elif o2 in ('dbh = pch', 'dbl = pcl'): A.add('db<-(pc)')
        elif o2 in ('abh = pch', 'abl = pcl'): A.add('ab<-(pc)')
        elif o2 in ('dbh = ath', 'dbl = atl'): A.add('db<-(at)')
        elif o2 in ('abh = ath', 'abl = atl'): A.add('ab<-(at)')
    if acc:
        A |= {f'acc_mode={acc[0]}', f'acc_type={acc[1]}', 'acc_class=' + ('t' if dbin_ld or acc[0] == 'w' else 'i')}
    elif dbin_ld: A |= {'acc_class=f', 'acc_mode=r', 'acc_type=in'}
    else: A |= {'acc_class=n', 'acc_mode=p'}
    return A
n_trace = 0
for a in list(content):
    if src[a].endswith('partial(nano map)') and ops.get(a):
        content[a] = trace_atoms(a); src[a] = src[a].replace('partial(nano map)', 'from the trace'); n_trace += 1
print('die-only words rebuilt from the trace:', n_trace, ' '.join(f'{a:03x}' for a in content if src[a].endswith('from the trace')))
# RESET TAIL FROM THE DIE (state_reset block of all.txt, timing checked against the reference core): 002/296/27e/27f are INTERNAL
# words (Galibert: jump only; 296: ird = ir), the four vector reads (0, 2, 4, 6) are in 2fb/2ff/300/11e ('instruction word read',
# aob = au, au += 2; 11e: ath = abh) = App. H rstp3/rstp4/rstp5/jmal1, and 2b7 (trap8 = jmal2) makes the first fetch at {ATH, DBIN}.
# This way the reads are 16 ticks apart, as in the reference core.
HAND = {
    0x002: {'acc_class=n', 'acc_mode=p'},
    0x296: {'acc_class=n', 'acc_mode=p', 'ird<-(ir)'},
    0x27e: {'acc_class=n', 'acc_mode=p'},
    0x27f: {'acc_class=n', 'acc_mode=p'},
    # dbcc4 (die 0c7): die trace = 'au -> db -> aob,pc ; +2 -> au' + instruction read; App. H dcnt4 also has 'alu -> * -> rxl'
    # (a register write that the die does not do: DBRA would write a second register)
    0x0c7: {'acc_class=i', 'acc_mode=r', 'acc_type=in', 'db<-au', 'aob<-db', 'pc<-db', 'au<-db', 'au+=+2'},
    # illg1/illg2 (3ff/3fe, free addresses in the die): entry of the illegal/privilege/line A-F exceptions; PC of the instruction
    # (Motorola manual) = PC - 2 -> AU -> PC, then the trap2.. chain
    0x3ff: {'acc_class=n', 'acc_mode=p', 'au+=-2', 'au<-db', 'db<-(pc)'},
    0x3fe: {'acc_class=n', 'acc_mode=p', 'pc<-au', 'ftu<-(psw)', 'rp_dst=dx', 'rp_src=dy'},
    # STOP from the die (all.txt 3a2/327): stop1 = au -> pc (+2), (dbin) -> ab -> ftu, ird <- ir ; aaa01 = au -> pc (-2), ird <- ir, ftu -> psw
    # stop1: the interrupt frame must hold the PC of the instruction after the STOP.
    0x3a2: {'acc_class=n', 'acc_mode=p', 'au<-db', 'au+=+2', 'pc<-au', 'abd<-(dbin)', 'ftu<-ab', 'ird<-(ir)'},   # without db<-(pc): DB not driven -> the AU starts from its own value (= the die PC, the next fetch); +2 as in the die: with the wait state (3fd) the frame is the instruction after the STOP
    # aaa01: same transfers and constant as the die (au <- pc - 2, pc <- au). At the entry of stop1 pc = address of the immediate;
    # in the wait state pc = next instruction + 2 and au = pc + 2 (same relation as a normal instruction boundary, frame = pc - 2).
    0x327: {'acc_class=n', 'acc_mode=p', 'au<-db', 'au+=-2', 'db<-(pc)', 'pc<-au', 'ird<-(ir)', 'psw<-ftu'},
    # stopw (3fd) and stopw2 (3fc): STOP wait, no operation (exc keeps the sequencer in the loop 3fc, 3fd until an interrupt arrives)
    0x3fd: {'acc_class=n', 'acc_mode=p'},
    0x3fc: {'acc_class=n', 'acc_mode=p'},
    # MOVEM Rlist,abs.W: smaw1 (1ed) and smaw2 (3c1) have no App. H mapping (they would be EMPTY and the core would stop): smaw1 as
    # smal1 (1e5: mask -> FTU, prefetch), smaw2 as stmd2 (3c9: extended address -> AT, start of the prefetch closed by smaw3 04b)
    0x1ed: {'ab<-(dbin)', 'acc_class=t', 'acc_mode=r', 'acc_type=im', 'aob<-db', 'au+=+2', 'au<-db', 'db<-au', 'dbin<-edb', 'ftu<-ab', 'irc<-edb', 'pc<-db', 'rp_dst=rp', 'rp_src=dy'},
    0x3c1: {'abe<-(dbin)', 'at<-abe', 'acc_class=i', 'acc_mode=r', 'acc_type=in', 'aob<-db', 'au+=+2', 'au<-db', 'db<-au'},
    # mmaw2 (328, empty): end of MOVEM memory->registers with the list exhausted = App. H mmrw2 '(pc) -> db -> aob,au; +2 -> au' (die 328)
    0x328: {'acc_class=i', 'acc_mode=r', 'acc_type=ix', 'aob<-db', 'db<-(pc)', 'au<-db', 'au+=+2'},
    # dbcc6 (die 0c6, missing from App. H, so EMPTY): exit of DBcc with true cc: au = pc + 2 (fall-through), ird <- ir; then bbci3 (0c5)
    # reads at AU. With an empty word the AU would keep the branch target: DBT would always branch.
    0x0c6: {'acc_class=n', 'acc_mode=p', 'au<-db', 'au+=+2', 'db<-(pc)', 'ird<-(ir)'},
    # dbcc5 (die 007): exit with expired counter: instruction read at PC (fall-through) with the strobe of its own data (dbin, irc: 4 clocks),
    # au = pc + 2, ird <- ir; App. H dcnt5 (217) reads at AU (= target + 2): the core would execute the DBcc again.
    0x007: {'acc_class=t', 'acc_mode=r', 'acc_type=ix', 'aob<-db', 'db<-(pc)', 'au<-db', 'au+=+2', 'dbin<-edb', 'irc<-edb', 'ird<-(ir)'},
}
for a, A in HAND.items(): content[a] = set(A); src[a] = f'die:{names.get(a, "?")}:internal (die state_reset)'
# index register: the words whose origin is aixw2/aixw4 read (rx)/(rxl) as reg[IRC 15:12] (die trace rmxw3: 'dbh = reg[irc >> 12]h');
# the same holds for ldmx3/ldmx4 (MOVEM memory->registers, index mode: die 0e1 'dbh = reg[irc >> 12]h'), for jsrx2/jsrx3 (App. H
# origin of jsrx2/jsrx3/jmpx2/jmpx3, die 029/0e9/02d/0ed: JMP/JSR d8(An,Xn)/d8(PC,Xn)) and for mmxw2/mmxw3 (origin of mmxl2/mmxl3:
# MOVE <memory>,d8(An,Xn)/d8(PC,Xn), indexed destination). Without rz these words would read as index the register of the rx field
# of the IRD (e.g. JMP 2(PC,D1.W) would use D7, MOVE.B (A0),2(A6,D0.W) would write at A6+D6+2).
RZ_ORIG = {'aixw2', 'aixw4', 'ldmx3', 'ldmx4', 'jsrx2', 'jsrx3', 'mmxw2', 'mmxw3'}
for r in M:
    if len(r) >= 3 and r[0] in H and H[r[0]][3] in RZ_ORIG:
        try: content[int(r[2], 16)].add('rxsrc=rz')
        except (ValueError, KeyError): pass
for a, lab in {0x2fb: 'rstp3', 0x2ff: 'rstp4', 0x300: 'rstp5', 0x11e: 'jmal1'}.items():
    content[a] = atoms(H[lab]); src[a] = f'appH:{lab}:reassigned (die state_reset, lockstep)'
# Access class from the DIE (die trace all.txt, checked against the reference core and the timing of the manual): the transfers
# stay those of the matching App. H blocks (validated with the die trace); only the access class changes. 00b/008/17a/00c
# (adrw1/ablw3/pdcw2/adsw2 in the die) start the operand read of the EA chains, 005/026 (adrw2/adsl3 in the die) are the second
# read of the long chains.
CLASS_FIX = {0x00b: 't', 0x008: 't', 0x17a: 't', 0x00c: 't',   # 't' = start and wait for its own cycle, with the strobe of its own data (DBIN model of the die)
             # die trace (all.txt): words that START an access ("bus access mode") where App. H has f/n, and vice versa.
             # adrl2/adsl3/pead3 = second read of the long chains (otherwise MOVE.L (An),Dn would read a single word: 8 clocks instead of 12).
             0x005: 't', 0x026: 't', 0x17f: 't', 0x0c7: 'i', 0x150: 'i',
             0x15e: 'f',   # nnrl2 (CLR/NEG/NOT/NEGX .L Dn): end of the read started by nnrl1 (App. H frix), not a second read (it would read the prefetch again)
             0x0e3: 'n',   # dvur2 (entry of the divide-by-zero trap): no access in the die ('au = db - 2; db = pc'); with 'i' it would start a read never closed and the trap chain would not write
             0x15d: 't',   # nnml2 (NEG/NOT/CLR/NEGX .L in memory): write of the low word (App. H 'twop'); the die trace does not show it: without it only the high word would be written
             0x001: 'n', 0x151: 'n', 0x152: 'n', 0x153: 'n', 0x17d: 'n'}
for a, cls in CLASS_FIX.items():
    content[a] = {x for x in content[a] if not x.startswith('acc_class=')} | {f'acc_class={cls}'}
    src[a] = src.get(a, '?') + f':class {cls} from the die (i+f chain, lockstep)'
zzz = [a for a in names if a not in content]
for a in zzz: content[a] = set(); src[a] = f'die:{names[a]}:empty'

# --- resolution of the App. H '*' / '*e' buses with the trace (all.txt: the ops "Xh = Yh" tell which physical bus is used) ---
# (the trace `ops` is loaded at the beginning of the file)
def norm(n):   # App. H name -> trace name: no parentheses, every rx*/ry* register -> R
    n = n.strip('()')
    return 'R' if n.startswith('rx') or n.startswith('ry') else n
def pairs(a):   # (destination, source) pairs observed in the trace
    P = set()
    for o in ops.get(a, ()):
        o2 = re.sub(r'\b[ad][0-7]([hl])\b', r'R\1', o); o2 = re.sub(r'reg\[[^\]]*\]([hl])', r'R\1', o2)
        m = re.match(r'^(\w+)h = (\w+)h$', o2)
        if m: P.add((m.group(1), m.group(2)))
        m = re.match(r'^(\w+)d = (\w+)$', o2)
        if m: P.add((m.group(1), m.group(2)))
        m = re.match(r'^(aob|pc) = au$', o2)
        if m: P.add((m.group(1), 'au'))
        if o2.startswith('au = dbh/l'):
            P.add(('au', 'db'))
            if 'abh' in o2: P.add(('au', 'ab'))
    return P
star_res = 0; star_unres = 0; unres_atoms = collections.Counter()
for a in list(content):
    P = pairs(a); A = content[a]; new = set()
    for at in A:
        if not re.search(r'(^|<-)\*e?($|<-)', at): new.add(at); continue
        star = '*e' if '*e' in at else '*'
        L, Rr = at.split('<-'); hit = []
        for b in ('db', 'ab'):
            be = b + 'e' if star == '*e' else b
            ok = ((b, norm(Rr)) in P) if L == star else ((norm(L), b) in P)
            if ok and at.replace(star, b) not in A and at.replace(star, b + 'e') not in A: hit.append(at.replace(star, be))
        if len(hit) == 1: new.add(hit[0]); star_res += 1
        else: new.add(at); star_unres += 1; unres_atoms[at] += 1
    content[a] = new

# --- residual '*' buses: STATIC resolution by exclusion (the bus not used by any other atom of the word) ---
# source '*<-S' -> 'ab*<-S'/'db*<-S' (all sections), '*e<-S' -> 'abe/dbe<-S'; destination 'D<-*' -> 'D<-ab/db', 'D<-*e' -> 'D<-abe/dbe'
excl_res = 0; excl_unres = 0
def bus_key(at):
    L = at.split('<-')[0]
    return 'ab' if L.startswith('ab') else 'db' if L.startswith('db') else None
for a in list(content):
    A = content[a]
    stars = [at for at in A if re.search(r'(^|<-)\*e?($|<-)', at)]
    if not stars: continue
    used = {bus_key(at) for at in A} - {None}
    free = [b for b in ('ab', 'db') if b not in used]
    if len(free) != 1: excl_unres += len(stars); continue
    b = free[0]; new = set(A) - set(stars)
    for at in stars:
        L, Rr = at.split('<-')
        if L in ('*', '*e'): new.add((b + ('e' if L == '*e' else '*')) + '<-' + Rr)
        else: new.add(L + '<-' + b + ('e' if Rr == '*e' else ''))
        excl_res += 1
    content[a] = new
# --- AU: the die trace ("au = dbh/l [+ abh/l] [+/- N]") is precise and wins over the App. H atoms if in conflict
# (the App. H -> die mapping has 72 mismatches: e.g. pead1 App. H says +2, the die -4). Second operand AB and constant from the die.
au_fix = 0; au_amb_fix = 0
# --- residual 'D<-*' destinations: bus of the SOURCE of the same App. H transfer ("au -> * -> rx ; (at) -> * -> ry" in
# exge2: rx from the resolved bus of au, ry from that of (at)). Without this step both would end on the same port (EXG broken).
pair_res = 0
for a in list(content):
    A = content[a]; stars = [at for at in A if re.search(r'<-\*e?$', at)]
    if not stars: continue
    lab = src[a].split(':')[1] if src[a].startswith('appH:') else None
    if not lab or lab not in H: continue
    new = set(A)
    for t in [x.strip() for x in H[lab][10].split(';')]:
        p3 = [x.strip() for x in t.split('->')]
        if len(p3) != 3 or p3[1] not in ('*', '*e'): continue
        S = p3[0]
        b = next((('ab' if at.startswith('ab') else 'db') for at in A if at.endswith('<-' + S) and re.match(r'^(ab|db)', at)), None)
        if not b: continue
        for d in [x.strip() for x in p3[2].split(',')]:
            for st in ('*', '*e'):
                if f'{d}<-{st}' in new:
                    new.discard(f'{d}<-{st}'); new.add(f'{d}<-{b}' + ('e' if st == '*e' else '')); pair_res += 1
    content[a] = new
print("'*' destinations resolved per transfer (bus of the source):", pair_res)
for a in list(content):
    A = content[a]; tr = [o for o in ops.get(a, ()) if o.startswith('au = dbh/l')]
    au_at = [at for at in A if at.startswith('au<-')]; k_at = [at for at in A if at.startswith('au+=')]
    if not tr or (not au_at and not k_at): continue
    forms = set()
    for o in tr:
        m = re.match(r'au = dbh/l( \+ abh/l)?( ([+-]) (\d+))?$', o)
        if m: forms.add((bool(m.group(1)), 0 if not m.group(2) else int(m.group(4)) * (1 if m.group(3) == '+' else -1)))
    if len({f[0] for f in forms}) != 1 or not forms: continue
    ab_tr = next(iter(forms))[0]; ns = {f[1] for f in forms}
    new = set(A) - set(au_at) - set(k_at)
    # DB input: always; sign-extend DB if App. H says so for DB and there is no AB
    db_e = 'au<-dbe' in au_at and not ab_tr
    dbh = 'au<-dbh' in au_at and 'au<-dbl' not in au_at; dbl = 'au<-dbl' in au_at and 'au<-dbh' not in au_at
    new.add('au<-dbe' if db_e else 'au<-dbh' if dbh else 'au<-dbl' if dbl else 'au<-db')
    if ab_tr: new.add('au<-abe' if any(x in au_at for x in ('au<-abe', 'au<-*e', 'au<-dbe')) else 'au<-ab')
    # constant: that of the die; if the die has two values (byte/word) keep the double form
    # double form (byte/word) if App. H gives it ('-1,-2 -> au': the die trace prints only -2) or the trace itself
    if ns == {1, 2} or 'au+=+1,+2' in k_at: new.add('au+=+1,+2')
    elif ns == {-1, -2} or 'au+=-1,-2' in k_at: new.add('au+=-1,-2')
    elif len(ns) == 1:
        n = next(iter(ns)); new.add('au+=0' if n == 0 else f'au+={n:+d}')
    else: new |= set(k_at)
    if new != A:
        au_fix += 1; au_amb_fix += any(at in ('au<-*', 'au<-*e') for at in au_at); content[a] = new; src[a] += '+au:die'
# --- slots: packing of the mutually exclusive atoms ---
# (after the AU override from the die, which would otherwise put back '+2')
# end words of the long reads (adrl2 005, adsl3 026): the high word (dbd) also goes into ALUE. App. H transcribes
# "(dbin) -> dbd -> alub", but mrgl2 (36f) writes rxh from "(alue) -> db" and mmrl1 (3a9) loads dob from "(alue) -> dbd": without ALUE
# the MOVE.L through memory would lose the high word (die 36f: "dbl <-> dbd; d0h = dbh" = rxh <- dbd). ALUB stays for the pass with flags.
ATOM_ADD = {0x005: {'alue<-dbd'}, 0x026: {'alue<-dbd'}, 0x2be: {'dob<-ab'},
            # DIVS: dvs01/dvs02 save the divisor in ATH (die 'ath = abh'; dvs14 reads its sign);
            # dvs12 (214, die-only): final shift of the pair with 0 shifted in (last quotient bit = 0), ATL stays the remainder of dvs0c
            0x0ac: {'ath<-ab'}, 0x0ae: {'ath<-ab'}, 0x214: {'ab<-alu', 'alu<-ab', 'alu+=-1', 'alu_cc=n', 'alu_col=4'},
            # dvs1f (08a, die-only, ALU invisible in the trace): negation of the quotient (dividend +, divisor -) like dvs1a (0cd);
            # dvs20 (049) copies ALU into ATL and tests nz2 on its flags (n+z -> leaa2 writes {remainder, -q}; /n/z -> overflow)
            0x08a: {'alu+=0', 'alu<-(alub)', 'alu_cc=i', 'alu_col=2'},   # rmdl2: data of the first write (high word) from AB, like rmrl1
            # TAS <mem>: die tasm1 (343) "pc = au" before "au = db" (AT = address of the operand for the write); tasm2 (344) does
            # au <- pc (+0) and the 'b' (363) reads from AU. Without pc<-au the PC would stay at the word already prefetched and the 'b'
            # would read it again (fetch of 0x410 instead of 0x412; the next TRAP would stack PC-2).
            0x343: {'pc<-au'}}
# 'index .W' words (aixw2/aixl2/rmxw2/rmxl2/leax2/peax2/stmx4, die 120..138): App. H "(rxl) -> *e -> au ; au -> ab -> au"
# = die "au = dbh/l + abh/l" with AB = previous AU (32 bits) and the index register, sign-extended from the low word, on section E
# of DB. The unresolved '*e' would leave DB undriven: the AU would add AU+AU (LEA d8(A0,D1.W) = A0+d8+A0+d8 instead of A0+d8+D1).
ATOM_SWAP = {a: ({'*e<-(rxl)', 'au<-abe', 'au<-db'}, {'dbe<-(rxl)', 'au<-dbe', 'au<-ab'}) for a in (0x120, 0x124, 0x128, 0x12c, 0x130, 0x134, 0x138)}
ATOM_FIX = {}   # no override: the +2 of 008/00b/00c/17a is the die's (adrl1/adsl2/pdcl2/abll3: second long read at EA+2);
# without +2 the MOVE.L through memory would read EA+0 again.
for a, (old, new) in ATOM_FIX.items():
    assert old in content[a], (hex(a), old)
    content[a] = (content[a] - {old}) | {new}; src[a] += f':{old}->{new} (lockstep)'
for a, add in ATOM_ADD.items():
    content[a] = set(content[a]) | add
# words with displacement/immediate sign-extended on section E of DB (dbe<-...) and full base on AB: the AU adds
# db + ab (die: "au = dbh/l + abh/l"); the bus resolution above gives au<-abe (base truncated to 16 bits: LEA 0x10(A1) with
# A1=0x102000 would give 0x2010). 23 words: mmxw2 mmxl2 bbcw1 bsri3 bsrw3 ldmx4 adsl1 adsw1 mpow1 mpol1 mpiw1 mpil1 jsrd1 jsrx1 jmpd1
# jmpx1 rmdw1 rmdl1 lead1 ldmd2 link4 mmdw1 mmdl1.
# ldmx4 (.W) is at 021, not at 0e1: the mapping follows the Galibert names, but in the die 'jumpcc i11 021, 0e1' of ldmx2 has the
# branch irc[11]=0 (index .W) on 021, and 0e1 carries 'dbh = reg[irc >> 12]h' (.L word, like aixl5 1e0): us4325121_appH_die_map.tsv
# has the two swapped.
for a in (0x028, 0x02c, 0x068, 0x0a8, 0x0aa, 0x021, 0x1c2, 0x1c6, 0x1ca, 0x1ce, 0x1d2, 0x1d6, 0x2b0, 0x2b2, 0x2b4, 0x2b6, 0x2da, 0x2de, 0x2f2, 0x2f6, 0x30d, 0x38a, 0x38e):
    assert 'au<-abe' in content[a] and any(x.startswith('dbe<-') for x in content[a]), hex(a)
    content[a] = (set(content[a]) - {'au<-abe'}) | {'au<-ab'}
ATOM_SWAP[0x175] = ({'db<-(pc)'}, {'db<-(at)', 'pc<-au'})
# dbcc2 (046), die: "au = db + 2; abh = auh; aobh = abh; ath = abh; dbh = pch": AB <- AU (branch destination) -> AOB and AT;
# DB <- PC -> AU = PC + 2 (prefetch pointer of the branch NOT taken, used by dbcc3 'pc = au'); then dbcc3: au = at + 2 (branch taken).
assert {'aob<-db', 'db<-au'} <= set(content[0x046]), hex(0x046)
content[0x046] = (set(content[0x046]) - {'aob<-db', 'db<-au', 'rxdl<-ab'}) | {'ab<-au', 'aob<-ab', 'at<-ab', 'db<-(pc)', 'rxdl<-ab*'}   # counter from section D (ALU), AB L/H = destination   # dbcc3: die 'pc = au; au = db + 2; dbh = ath'
# chkr2 (153), die: "au = db + 2; dbh = auh; pch = dbh; dbl = aul" WITHOUT transfers from the bus. App. H "edb -> dbin,irc": there the
# fetch starts in chkr1 (irix) and ends in chkr2 (frix); in the die it starts in chkr4 (150) and ends in mmrw3 (34c). With EDB = operand
# just read (memory forms) the IRC would lose the word already prefetched: the TRAP/BRA after the CHK would become the operand (the
# register form is not affected because EDB is still the word in IRC).
assert {'dbin<-edb', 'irc<-edb'} <= set(content[0x153]), hex(0x153)
content[0x153] = set(content[0x153]) - {'dbin<-edb', 'irc<-edb'}
# dvs16/dvs1d (041/0c1): "(atl) -> * -> ath" with unresolved '*' bus (bus_sec does not drive it); die: 'ath = dbh; dbh <-> dbl'
ATOM_SWAP[0x041] = ({'*<-(atl)', 'ath<-*'}, {'dbl<-(atl)', 'ath<-db'})
ATOM_SWAP[0x0c1] = ({'*<-(atl)', 'ath<-*'}, {'dbl<-(atl)', 'ath<-db'})
# PEA d16(An)/d16(PC) (pead1..3 = 17d/17e/17f) and PEA abs.L (paal2 = 15f) as in App. H (push at SP - 4, value An+d16):
# pead1 prefetch at PC+2 (class t) with the displacement in the ALU; pead2 AU <- AT + ext(ALU) and start of the prefetch;
# pead3 end of the prefetch, AT <- EA, AU <- SP - 4; paal2 AU <- SP - 4.
ATOM_SWAP[0x17d] = ({'acc_class=n', 'acc_type=in', 'au+=-4'}, {'acc_class=t', 'acc_type=ix', 'au+=+2'})
ATOM_SWAP[0x17e] = ({'au+=+2', 'au<-dbe', '*<-(at)', '*e<-alu'}, {'db<-(at)', 'au<-db', 'abe<-alu', 'au<-abe', 'au+=0'})
ATOM_SWAP[0x17f] = ({'acc_class=t', 'au+=0'}, {'acc_class=f', 'au+=-4', 'rp_dst=sp'})
ATOM_SWAP[0x15f] = ({'au+=0'}, {'au+=-4'})
# MOVE SR,Dn: strw1 (3a5) from the die 'bus read; aob = au; pc = au' (App. H trac1: '(pc) -> db -> au; -2 -> au' and no AOB: the read
# would repeat the previous address and the queue would stay one word behind: MOVE #imm,CCR after MOVE SR,D0 would fail)
ATOM_SWAP[0x3a5] = ({'au+=-2', 'au<-db', 'db<-(pc)'}, {'aob<-au', 'pc<-au'})
# dvs20 (049, die-only): the final flags (N of the quotient) must be taken from the NEGATED quotient (alu_r, via AB), not from ALUB,
# which holds the magnitude: otherwise DIVS with dividend + and divisor - would give N=0
ATOM_SWAP[0x049] = ({'alu<-(alub)'}, {'alu<-ab'})
for a, (old, new) in ATOM_SWAP.items():
    assert old <= set(content[a]), (hex(a), old - set(content[a]))
    content[a] = (set(content[a]) - old) | new
# the exception frame writes/reads (trap3-5/7, bser3-6, itlx4) are ALWAYS word-sized, whatever the size of the interrupted
# instruction (Motorola manual): type 'ix' instead of 'op' (byte_acc = is_byte & op). Otherwise ILLEGAL 4AFC (byte size for
# size_si) would push the frame as bytes, and the same holds for interrupts during byte instructions.
for a in (0x3c2, 0x360, 0x0ef, 0x11a, 0x3ca, 0x0a2, 0x3c6, 0x088, 0x0eb):
    if 'acc_type=op' in content[a]: content[a] = (content[a] - {'acc_type=op'}) | {'acc_type=ix'}; src[a] += ':acc_type ix (frame a word)'
allatoms = sorted(set().union(*content.values()))
addrs = sorted(content)
mask = {at: 0 for at in allatoms}
for i, a in enumerate(addrs):
    for at in content[a]: mask[at] |= 1 << i
freq = {at: bin(mask[at]).count('1') for at in allatoms}
maxw = max(len(content[a]) for a in addrs)
def pack(order):
    slots = []   # [atoms], mask
    for at in order:
        for s in slots:
            if s[1] & mask[at] == 0: s[0].append(at); s[1] |= mask[at]; break
        else: slots.append([[at], mask[at]])
    return slots
def width(slots): return sum(math.ceil(math.log2(len(s[0]) + 1)) for s in slots)
# resource classes: CRIT = resources consumed at ph0 or in the index-regfile/bus cone (bus sources, acc*, aob, pointers, rxsrc,
# dbin*, dobh/dobl -> byte_acc at req); NCRIT = resources consumed only at op_ld (ph1: ALU/AU) or at en (register writes,
# at/ath/atl/dob, ftu, psw, irc/ird, dcr, sr, reset pren). 'idle wait' and 'tpend+=' have no consumers in the RTL: they are in
# CRIT. Every resource must be in one of the two lists (assert).
CRIT_RES = {'*e', 'ab*', 'ab', 'abd', 'abe', 'abh', 'abl', 'db*', 'db', 'dbd', 'dbe', 'dbh', 'dbl', 'acc', 'acc_class', 'acc_mode',
            'acc_type', 'aob', 'rp_dst', 'rp_src', 'rxsrc', 'dbin', 'dbinh', 'dbinl', 'dobh', 'dobl', 'idle wait', 'tpend'}
NCRIT_RES = {'alu', 'alub', 'alue', 'alu_byte', 'alu_cc', 'alu_col', 'sr c-alu-alue', 'dcr', 'au', 'at', 'ath', 'atl', 'dob', 'ftu',
             'psw', 'psws', 'irc', 'ird', 'pc', 'pch', 'pcl', 'rx', 'rxa', 'rxdl', 'rxh', 'rxl', 'ry', 'rya', 'rydl', 'ryh', 'ryl',
             'sp', 'sph', 'spl', 'reset pren'}
def res_of(at): return re.split(r'<-|\+=|=', at, 1)[0]
_res = {res_of(a) for a in allatoms}
assert _res <= (CRIT_RES | NCRIT_RES), sorted(_res - CRIT_RES - NCRIT_RES)
assert not (CRIT_RES & NCRIT_RES)
# in the critical part (low bits) only what forms the read index of the register file (head of the cone): bus sources from a
# register + pointers. The rest in the non-critical part (consumers at >= 2 pulses or at ph0 on short paths).
BUS_RES = {'*e', 'ab*', 'ab', 'abd', 'abe', 'abh', 'abl', 'db*', 'db', 'dbd', 'dbe', 'dbh', 'dbl'}
RF_REGS = {'rx', 'rxa', 'rxd', 'rxdl', 'rxh', 'rxl', 'ry', 'rya', 'ryd', 'rydl', 'ryh', 'ryl', 'sp', 'sph', 'spl', 'pc', 'pch', 'pcl'}
def is_crit(a):
    r = res_of(a)
    if r in ('rp_dst', 'rp_src', 'rxsrc'): return True
    m = re.match(r'^[^<]*<-\((\w+)\)$', a)
    return r in BUS_RES and bool(m) and m.group(1) in RF_REGS
crit = [a for a in allatoms if is_crit(a)]; ncrit = [a for a in allatoms if not is_crit(a)]
assert len(crit) == 50, len(crit)
# packing BY WHOLE RESOURCE: the unit to place is the whole enumerated resource (same rule as gen_nano_dec.py: enumerated =
# no word uses two of its values together and it is not in the list of forced one-hot resources), or the single atom of the
# one-hot resources. An enumerated resource spread over several slots would cost the decoder one cell per (bit, slot) plus the
# OR; all in one slot it costs one cell per bit, zero if the slot is its own (the dec field is the slot). Criterion: minimum
# width, at equal width the lower decoding cost; if the M part exceeds 60 bits (3 M10K) the narrowest slots move into the L part
# (M10K read early, 20 bits): any atom can go there.
def _lv(at):
    for op in ('<-', '+=', '='):
        if op in at: return at.split(op, 1)[0] + op
    return at
_multi = set()
for a in addrs:
    per = collections.Counter(_lv(at) for at in content[a])
    _multi |= {L for L, n in per.items() if n > 1}
_multi |= {'ab<-', 'alu<-', 'alub<-', 'aob<-', 'au<-', 'ab*<-', 'abd<-', 'abe<-', 'db<-', 'db*<-', 'dbd<-', 'dbe<-', 'rp_dst=', 'rp_src='}
def units_of(group):
    byres = collections.defaultdict(list)
    for at in group: byres[_lv(at)].append(at)
    us = []
    for L in sorted(byres):
        if L in _multi: us += [[at] for at in sorted(byres[L])]
        else: us.append(sorted(byres[L]))
    return us
def umask(u):
    m = 0
    for at in u: assert m & mask[at] == 0, ('values not exclusive', u); m |= mask[at]
    return m
def pack_u(order):
    sl = []   # [units], mask
    for u in order:
        mu = umask(u)
        for s in sl:
            if s[1] & mu == 0: s[0].append(u); s[1] |= mu; break
        else: sl.append([[u], mu])
    return sl
def width_u(sl): return sum(math.ceil(math.log2(sum(len(u) for u in s[0]) + 1)) for s in sl)
def cost_u(sl):   # decoder cells by model: one-hot 1 per atom (0 if 1-bit slot); enumerated 1 per bit of the field, 0 if the slot is its own
    c = 0
    for s in sl:
        n = sum(len(u) for u in s[0])
        for u in s[0]:
            if _lv(u[0]) in _multi: c += 0 if n == 1 else 1
            else: c += 0 if len(s[0]) == 1 else math.ceil(math.log2(len(u) + 1))
    return c
def best_pack(group, seed):
    us = units_of(group); best = None; rng = random.Random(seed)
    orders = [sorted(us, key=lambda u: -sum(freq[x] for x in u)), sorted(us, key=lambda u: -len(u))]
    for _ in range(3000):
        o = list(us); rng.shuffle(o); orders.append(o)
    for o in orders:
        s = pack_u(o); k = (width_u(s), cost_u(s))
        if best is None or k < best[0]: best = (k, s)
    return [[[at for u in s[0] for at in u], s[1]] for s in best[1]]
for a_ in crit: assert _lv(a_) in _multi or all(is_crit(x) for x in allatoms if _lv(x) == _lv(a_)), ('enumerated resource split between L and M', a_)
# the critical part is packed by ATOM (fixed algorithm and seed: the slots and the codes, hence the bits [14:0] of the nanoword
# and the decoding cells at the head of the q_l -> register index -> read cone, are stable); by whole resource only the
# non-critical part.
def best_pack_atoms(group, seed):
    best = None; orders = [sorted(group, key=lambda x: -freq[x])]; rng = random.Random(seed)
    for _ in range(400):
        o = list(group); rng.shuffle(o); orders.append(o)
    for o in orders:
        s = pack(o)
        if best is None or width(s) < width(best): best = s
    return sorted(best, key=lambda s: -len(s[0]))
slots_l = best_pack_atoms(crit, 1); slots_m = best_pack(ncrit, 2)
slots_m.sort(key=lambda s: (math.ceil(math.log2(len(s[0]) + 1)), len(s[0])))
while width(slots_m) > 60: slots_l.append(slots_m.pop(0))   # the narrowest slots into the L part
assert width(slots_l) <= 20 and width(slots_m) <= 60, (width(slots_l), width(slots_m))
slots_l.sort(key=lambda s: -len(s[0])); slots_m.sort(key=lambda s: -len(s[0]))
slots = slots_l + slots_m
fields = []; off = 0; where = {}
for i, (al, _) in enumerate(slots):
    w = math.ceil(math.log2(len(al) + 1)); al = sorted(al)
    fields.append((f'slot{i}', al, off, w))
    for k, at in enumerate(al): where[at] = (i, off, w, k + 1)
    off += w
NW = off; NWL = width(slots_l); NWM = NW - NWL
assert 0 < NWM <= 60, (NWL, NWM)   # 3 M10K per CPU in 512x20 mode
def encode(A):
    word = 0
    for at in A: _, o, w, c = where[at]; word |= c << o
    return word
def decode(word):
    out = set()
    for name, al, o, w in fields:
        v = (word >> o) & ((1 << w) - 1)
        if v: out.add(al[v - 1])
    return out
# --- our ORIGIN: unique words ---
uniq = {frozenset(): 0}
for a in addrs:
    k = frozenset(content[a])
    if k not in uniq: uniq[k] = len(uniq)
# numbering: nanoword index = {row(addr[9:2]), addr[1:0]}. Every group of 4 consecutive micro addresses is a quadruple of
# unique words (unused address = word 0); equal quadruples share the row. The ORIGIN is a 256 x (NIDX-2) table in logic; the
# nano ROM holds 4 positions per row (a unique word can appear in several positions). Rows numbered in order of first
# appearance (deterministic).
ukey = {i: k for k, i in uniq.items()}
gtup = [tuple(uniq[frozenset(content[4 * g + k])] if (4 * g + k) in content else 0 for k in range(4)) for g in range(256)]
rows = {}
for t in gtup:
    if t not in rows: rows[t] = len(rows)
org_t = [rows[t] for t in gtup]
NROW_W = max(1, math.ceil(math.log2(len(rows)))); NIDX = NROW_W + 2; NDEPTH = 1 << NIDX
origin = {a: 4 * org_t[a >> 2] + (a & 3) for a in range(1024)}
words = [0] * NDEPTH
for t, r in rows.items():
    for k in range(4): words[4 * r + k] = encode(ukey[t[k]])
bad = sum(1 for a in range(1024) if decode(words[origin[a]]) != set(content.get(a, ())))   # lossless encoding, on all 1024 addresses
# --- outputs ---
nl = '\n'
def ident(s):
    s = s.replace('<-', '_from_').replace('+=', '_const_').replace('*', 'star').replace('(', 'p_').replace(')', '')
    s = s.replace('+', 'plus').replace('-', 'minus').replace(',', '_or_')
    return re.sub(r'[^A-Za-z0-9]+', '_', s).strip('_').upper()
assert len({ident(a) for a in allatoms}) == len(allatoms), 'SV identifiers not unique'
import os
if os.environ.get('NROM_LIST'):
    with open(os.environ['NROM_LIST'], 'w', newline=nl) as fl:
        for a in sorted(content): fl.write('%03x' % a + '	' + names.get(a, '?') + '	' + ' '.join(sorted(content[a])) + nl)
with open(f'{R}/rtl/cpu_si/m68k_nrom.mem', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write(f'// nano ROM with our own encoding, critical part: {NDEPTH} x {NWL} bits = bits [{NWL-1}:0] of the nanoword ($readmemb), index = our ORIGIN (word 0 = no action)' + nl)
    f.write('// generated by docs/data/scripts/gen_nrom.py from the App. H blocks (US 4,325,121) - DO NOT edit by hand' + nl)
    for w in words: f.write(format(w & ((1 << NWL) - 1), f'0{NWL}b') + nl)
with open(f'{R}/rtl/cpu_si/m68k_nrom_m.mem', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write(f'// nano ROM with our own encoding, non-critical part: {NDEPTH} x {NWM} bits = bits [{NW-1}:{NWL}] of the nanoword ($readmemb), same ORIGIN index' + nl)
    f.write('// generated by docs/data/scripts/gen_nrom.py from the App. H blocks (US 4,325,121) - DO NOT edit by hand' + nl)
    for w in words: f.write(format(w >> NWL, f'0{NWM}b') + nl)
with open(f'{R}/rtl/cpu_si/m68k_norigin.mem', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write(f'// ORIGIN table: micro group addr[9:2] (256) -> row ({NROW_W} bits, hexadecimal); nanoword index = {{row, addr[1:0]}}; generated by gen_nrom.py' + nl)
    for g in range(256): f.write(format(org_t[g], 'x') + nl)
with open(f'{R}/rtl/cpu_si/m68k_nano_pkg.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// fields of the nano ROM with our own encoding - generated by docs/data/scripts/gen_nrom.py, DO NOT edit by hand' + nl)
    f.write('// active atom <=> nw[NA_<atom>_LSB +: NA_<atom>_W] == NA_<atom>_CODE' + nl)
    f.write('package m68k_nano_pkg;' + nl + '/* verilator lint_off UNUSEDPARAM */' + nl)
    f.write(f'localparam int NROM_W = {NW};' + nl + f'localparam int NROM_WL = {NWL};   // bits [NROM_WL-1:0]: critical part, [NROM_W-1:NROM_WL]: non-critical part; both in M10K, read one clock before en' + nl + f'localparam int NROM_DEPTH = {NDEPTH};' + nl + f'localparam int NROM_IDX_W = {NIDX};' + nl)
    # immediate a0s override in the core: only A_RESET (000) and A_BSER (003): their ORIGIN indices as constants
    _exc = open(f'{R}/rtl/cpu_si/m68k_exc_si.sv', encoding='utf-8').read()
    assert "A_RESET = 10'h000, A_BSER = 10'h003" in _exc and "assign a0s = rst_l ? A_RESET : A_BSER;" in _exc, 'set of a0s values changed in m68k_exc_si'
    assert 0 in origin and 3 in origin, '000/003 without a nanoword'
    f.write(f'localparam int NROM_ORG_000 = {origin[0]};   // ORIGIN index of 000 (A_RESET) and 003 (A_BSER): immediate a0s override in the core' + nl)
    f.write(f'localparam int NROM_ORG_003 = {origin[3]};' + nl)
    # the part [NROM_WL-1:0] of the nanoword is in M10K read one clock before en: for the immediate override (which has no
    # clock of advance) the low bits of the nanowords of 000 and 003 are package constants
    assert NWL <= 31, 'NROM_L_000/003 are localparam int'
    f.write(f'localparam int NROM_L_000 = {words[origin[0]] & ((1 << NWL) - 1)};   // bits [NROM_WL-1:0] of the nanowords of 000 and 003: immediate a0s override in the core' + nl)
    f.write(f'localparam int NROM_L_003 = {words[origin[3]] & ((1 << NWL) - 1)};' + nl)
    # the high part [NROM_W-1:NROM_WL] of the same two words is also a constant, so the override does not need a second read of rom_m
    f.write(f"localparam logic [{NWM - 1}:0] NROM_M_000 = {NWM}'b{words[origin[0]] >> NWL:0{NWM}b};   // bits [NROM_W-1:NROM_WL] of the same two words:" + nl)
    f.write(f"localparam logic [{NWM - 1}:0] NROM_M_003 = {NWM}'b{words[origin[3]] >> NWL:0{NWM}b};   // so rom_m has a single read address and stays in M10K" + nl)
    for name, al, o, w in fields:
        f.write(f'// {name}: bit {o}+:{w}, atoms: {len(al)}' + nl)
        for k, at in enumerate(al):
            f.write(f'localparam int NA_{ident(at)}_LSB = {o}, NA_{ident(at)}_W = {w}, NA_{ident(at)}_CODE = {k + 1};  // {at}' + nl)
    f.write('endpackage' + nl)
with open(f'{D}/mc68000_nano_encoding.tsv', 'w', encoding='utf-8', newline=nl) as f:
    f.write('# our encoding of the nano ROM (gen_nrom.py): slot, lsb, width, atoms (code=atom)' + nl)
    for name, al, o, w in fields:
        f.write(f'{name}\t{o}\t{w}\t' + ' '.join(f'{k + 1}={at}' for k, at in enumerate(al)) + nl)
    f.write(nl + '# die address -> label, nanoword index, origin of the content, atoms' + nl)
    for a in addrs: f.write(f'{a:03x}\t{names.get(a, "")}\t{origin[a]}\t{src[a]}\t' + ' ; '.join(sorted(content[a])) + nl)
print(f'addresses {len(content)} (App.H {len(content) - die_only - len(zzz)}, die-only {die_only} of which partial {die_partial}, empty {len(zzz)}) | label->addr collisions {collisions}')
print('most frequent unresolved by the trace:', unres_atoms.most_common(8))
print(f"bus '*' resolved by exclusion {excl_res}, still ambiguous {excl_unres}")
print(f"AU from the die: words corrected {au_fix} (of which with ambiguous au<-* {au_amb_fix}); residual au<-* {sum(1 for A in content.values() for at in A if at in ('au<-*', 'au<-*e'))}")
print(f"bus '*' resolved {star_res} unresolved {star_unres} | atoms {len(allatoms)} | max atoms/word {maxw} | slots {len(fields)} | NW {NW} bits (L {NWL} + M {NWM}) | unique words {len(uniq)} -> ROM {NDEPTH} x {NW} | ORIGIN 256 x {NROW_W} (rows {len(rows)}, positions {4 * len(rows)}) | encoding loss {bad}")
