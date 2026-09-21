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

# Register file (D0-D7, A0-A6, USP, SSP, PC, DT) of the silicon-structured datapath, driven by the nano atoms.
# SILICON STRUCTURE: the register file talks ONLY to the two buses. Per word at most ONE register source per bus and ONE
# register destination per bus (verified on the 1088 atoms; exceptions: mulm6 writes pch and rxdl from DB -> the PC has its
# own port; unlk4 writes ryh and ryl from AB = same register). Hence:
#   - 2 read ports: rf_ab = r[ri_ab], rf_db = r[ri_db], index chosen by the source atom active on that bus;
#   - 2 write ports (AB, DB) with index, H/L enables and data with the section fallbacks computed ONCE per bus,
#     plus the PC port (pc<-au/ab/db, pch/pcl);
#   - pc_q = PC (for pc_o/dbg_pc); debug ports under the DBG parameter (0 in synthesis).
# App. H names -> physical register (docs/silicon-datapath.md §8): rx/rxh/rxl = register pointed to by RX (IRD 11:9, type D/A from IRD) unless the
# destination pointer overrides it (dt->DT, sp->SP, us->USP, rp->MOVEM encoder, uk->rz from IRC); rxa = same as an address
# register; rxd/rxdl = low word (DATA section) of the pointed register if the pointer is explicit, otherwise D[RX];
# ry/ryh/ryl/rya/ryd/rydl likewise with RY (IRD 2:0; py->PC if the EA is PC-relative); sp = active SP; pc = PC. The REGISTER
# POINTERS cell is already valid in the word that sets it.
# Emits the original forms of rtl/cpu_si/m68k_regfile_si.sv and sim/m68k_regfile_top.sv (testbench top); both files in the
# repository are now maintained by hand and differ from this output.
# Usage: python gen_regfile.py <repo_root>
import re, sys
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
# RETIRED GENERATOR. rtl/cpu_si/m68k_regfile_si.sv and sim/m68k_regfile_top.sv are maintained BY HAND: this script would
# produce the original, simpler register file (and its testbench top) and discard every change made by hand since
# (registered selectors, flat index forms, safety nets, write enables from flops, ...), breaking the core. It therefore always
# refuses to run; the code below is kept only as documentation of how the register file is derived from the nano atoms.
sys.exit('gen_regfile.py is retired: m68k_regfile_si.sv and m68k_regfile_top.sv are maintained by hand; regenerating them '
         'would discard the changes made by hand. Edit the .sv files by hand; the code of this script is kept only as '
         'documentation.')
R = sys.argv[1]; D = f'{R}/docs/data'
dec = {}; codes = {}
for l in open(f'{R}/rtl/cpu_si/m68k_nano_dec_pkg.sv'):
    m = re.match(r'localparam int ND_(\w+)_LSB = (\d+), ND_\w+_W = (\d+), ND_\w+_ONEHOT = (\d);  // (.*)$', l.rstrip('\n'))
    if m: dec[m.group(5)] = (int(m.group(2)), int(m.group(3)), int(m.group(4)))
    m = re.match(r'localparam int ND_(\w+)__(\w+) = (\d+);  // (.*)$', l.rstrip('\n'))
    if m: codes[m.group(4)] = int(m.group(3))
atoms = set(); words = {}
for l in open(f'{D}/mc68000_nano_encoding.tsv', encoding='utf-8'):
    f = l.rstrip('\n').split('\t')
    if re.match(r'^[0-9a-f]{3}\t', l):
        A = {x for x in f[4].split(' ; ') if x}; atoms |= A; words[int(f[0], 16)] = A
REGS = {'rx', 'rxa', 'rxd', 'rxdl', 'rxh', 'rxl', 'ry', 'rya', 'ryd', 'rydl', 'ryh', 'ryl', 'sp', 'sph', 'spl', 'pc', 'pch', 'pcl'}
IDX = {'rx': 'rx_i', 'rxh': 'rx_i', 'rxl': 'rx_i', 'rxa': 'rxa_i', 'rxd': 'rxd_i', 'rxdl': 'rxd_i', 'ry': 'ry_i', 'ryh': 'ry_i', 'ryl': 'ry_i',
       'rya': 'rya_i', 'ryd': 'ryd_i', 'rydl': 'ryd_i', 'sp': 'sp_i', 'sph': 'sp_i', 'spl': 'sp_i', 'pc': "5'd17", 'pch': "5'd17", 'pcl': "5'd17"}
def cond(at):
    L = at.split('<-')[0] + '<-'; lsb, dw, oh = dec[L]; c = codes[at]
    return f'dec[{lsb + c}]' if oh else f"dec[{lsb} +: {dw}] == {dw}'d{c}"
# --- register sources per bus (read)
rd = {'ab': [], 'db': []}
for at in sorted(atoms):
    m = re.match(r'^(ab|db)(h|l|d|e|\*)?<-\((\w+)\)$', at)
    if m and m.group(3) in REGS: rd[m.group(1)].append((at, IDX[m.group(3)]))
# --- writes: atom -> specifications "target_part=source" (source = bus sections with fallback, or au)
def src(b, sec, fb=None): return f'{b}.{sec}' + (f'|{b}.{fb}' if fb else '')
def rule(at):
    m = re.match(r'^(rx|rxa|rxh|rxl|rxdl|ry|rya|rydl|ryh|ryl|sp|sph|spl|pc|pch|pcl)<-(.+)$', at)
    if not m: return None
    dst, X = m.group(1), m.group(2)
    tgt = {'rx': 'rx', 'rxh': 'rx', 'rxl': 'rx', 'rxa': 'rxa', 'rxdl': 'rxd', 'ry': 'ry', 'ryh': 'ry', 'ryl': 'ry', 'rya': 'rya', 'rydl': 'ryd',
           'sp': 'sp', 'sph': 'sp', 'spl': 'sp', 'pc': 'pc', 'pch': 'pc', 'pcl': 'pc'}[dst]
    if X == 'au': return [f'{tgt}_h=au:31:16', f'{tgt}_l=au:15:0']
    mb = re.match(r'^(ab|db|\*)(h|l|d|e|\*)?$', X)
    if not mb: raise Exception(at)
    b, suf = mb.group(1), mb.group(2) or ''
    buses = ['ab', 'db'] if b == '*' else [b]
    def S(sec, fb=None): return '|'.join(src(bb, sec, fb) for bb in buses)
    def SH(): return '|'.join(f'{bb}.h|{bb}.l|{bb}.d' for bb in buses)   # high half: H, fallback L then D (couplers)
    if dst.endswith('h') and dst not in ('rx', 'ry'): return [f'{tgt}_h=' + SH()]
    if dst in ('rxl', 'ryl', 'spl', 'pcl'): return [f'{tgt}_l=' + S('l', 'd')]
    if dst in ('rxdl', 'rydl'): return [f'{tgt}_l=' + (S('d', 'l') if suf in ('', 'd', '*') else S('l', 'd'))]
    if suf == 'h': return [f'{tgt}_h=' + SH()]
    if suf == 'l': return [f'{tgt}_l=' + S('l', 'd')]
    if suf == 'd': return [f'{tgt}_l=' + S('d')]
    # 'e' (abe/dbe): the fabric already extends the H section (sext on H): generic rule H <- h, L <- l|d
    if suf == '*': return [f'{tgt}_h=' + S('h'), f'{tgt}_l=' + S('d', 'l')]
    return [f'{tgt}_h=' + S('h'), f'{tgt}_l=' + S('l', 'd')]
rules = {at: rule(at) for at in sorted(atoms) if rule(at)}
def port_of(at):
    dst, X = at.split('<-')
    if dst in ('pc', 'pch', 'pcl'): return 'pc'
    return 'db' if X.startswith('db') else 'ab'          # ab*, abe, ab, au, '*' -> AB port
TI = {'rx': 'rx_i', 'rxa': 'rxa_i', 'rxd': 'rxd_i', 'ry': 'ry_i', 'rya': 'rya_i', 'ryd': 'ryd_i', 'sp': 'sp_i', 'pc': "5'd17"}
def sv_data(s):   # source of the specification -> (enable, data expression)
    sext = s.startswith('sext:')
    if sext: s = s[5:]
    if ':' in s and '.' not in s:
        p, hi, lo = s.split(':'); return "1'b1", f'{p}[{hi}:{lo}]'
    alts = [(f'{t.split(".")[0]}_{t.split(".")[1]}_drv', f'{t.split(".")[0]}_{t.split(".")[1]}') for t in s.split('|')]
    en = ' | '.join(c for c, _ in alts)
    expr = alts[-1][1]
    for c, e in reversed(alts[:-1]): expr = f'({c} ? {e} : {expr})'
    if sext: expr = 'SEXT'
    return en, expr
# structural check: at most one destination per port per word
multi = 0
for a, A in words.items():
    per = {}
    for at in A:
        if at in rules: per.setdefault(port_of(at), set()).add(at.split('<-')[0].rstrip('hl') if at.split('<-')[0] not in ('rxdl', 'rydl') else at.split('<-')[0][:-2])
    multi += sum(1 for v in per.values() if len(v) > 1)
nl = '\n'
with open(f'{R}/rtl/cpu_si/m68k_regfile_si.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// m68k_regfile_si.sv - register file of the silicon-structured datapath: D0-D7, A0-A6, USP, SSP, PC, DT.' + nl)
    f.write('// GENERATED by docs/data/scripts/gen_regfile.py (rules: docs/silicon-datapath.md section 8), do NOT edit by hand.' + nl)
    f.write('// Silicon structure: 2 read ports (one per bus: rf_ab/rf_db = active source register on that bus), 2 write' + nl)
    f.write('// ports (AB, DB: index + H/L enables + data with the section fallback computed once per bus) + PC port.' + nl)
    f.write('import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;' + nl + 'module m68k_regfile_si #(parameter bit DBG = 1\'b1) (' + nl)
    f.write('  input  logic clk, rst_n, en,' + nl + '  /* verilator lint_off UNUSEDSIGNAL */' + nl + '  input  logic [NDEC_W-1:0] dec,' + nl + '  /* verilator lint_on UNUSEDSIGNAL */' + nl)
    f.write('  input  logic [15:0] ird,          // instruction being executed (fields RX = 11:9, RY = 2:0, modes)' + nl)
    f.write('  input  logic        sup,          // supervisor mode (PSW S): A7 = SSP, otherwise USP' + nl)
    f.write('  input  logic        byte_op,      // byte instruction: writes to the low word of the DATA registers touch only the low byte (Scc/TAS/ADD.B...: the high byte is kept)' + nl)
    f.write('  input  logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,' + nl)
    f.write('  input  logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,' + nl)
    f.write('  input  logic [31:0] au,' + nl)
    f.write("  input  logic [3:0]  rp_reg,       // current register of the MOVEM encoder (0-7 = D, 8-15 = A; 15 = SP) for the rp pointer" + nl)
    f.write("  /* verilator lint_off UNUSEDSIGNAL */" + nl)
    f.write("  input  logic [15:0] irc,          // extension word of indexed addressing: index register rz = IRC[15:12] (uk pointer)" + nl)
    f.write("  input  logic [4:0]  dbg_idx_d, dbg_idx_a,   // debug (testbenches/lockstep): index of the register read (only with DBG)" + nl)
    f.write("  /* verilator lint_on UNUSEDSIGNAL */" + nl)
    f.write('  output logic [31:0] rf_ab, rf_db,  // active source register on bus AB / DB (read ports)' + nl)
    f.write('  output logic        rf_db_sp,      // the register read on DB is A7 (SP): AU uses -2/+2 even for bytes' + nl)
    f.write('  output logic [31:0] pc_q,          // PC' + nl)
    f.write('  output logic [31:0] dbg_data_d, dbg_data_a' + nl + ');' + nl)
    f.write('  // indices: 0-7 D0-D7, 8-14 A0-A6, 15 USP, 16 SSP, 17 PC, 18 DT' + nl)
    # storage in MLAB: asynchronous read (Cyclone V: an unregistered read address is allowed in simple dual-port),
    # one copy per read port, three columns 16/8/8 for the byte write; one write per clock (the queue at en+1..3); no
    # reset of the array (die). pc_q (only pc_o/dbg_pc) from a shadow register written in parallel.
    f.write('  // storage in MLAB: asynchronous read (simple dual-port with unregistered read address, Cyclone V' + nl)
    f.write('  // handbook vol. 1), one copy per read port (AB, DB), three columns 16/8/8 for the byte write; a single write per' + nl)
    f.write('  // clock (queue at en+1..3); forwarding from the queue covers every read of a pending value (no_rw_check). No reset' + nl)
    f.write("  // of the array, like the die. The observed PC (pc_q, debug only) comes from a shadow register written in parallel." + nl)
    for a in ('ra_h [0:18]', 'ra_m [0:18]', 'ra_l [0:18]', 'rb_h [0:18]', 'rb_m [0:18]', 'rb_l [0:18]'):
        f.write('  (* ramstyle = "MLAB, no_rw_check" *) logic ' + ('[15:0] ' if a[3] == 'h' else '[7:0]  ') + a + ';' + nl)
    f.write('  logic [31:0] pc_sh;' + nl)
    f.write('  logic [4:0] sp_i, rx_i, ry_i, rxa_i, rya_i, rxd_i, ryd_i;' + nl)
    f.write('  logic rx_is_a, ry_is_a, pcrel, rx_lo;' + nl)
    f.write("  assign rx_lo = (ird[15:12] == 4'h5) && (ird[7:3] == 5'b11001);   // DBcc: the counter register is IR[2:0] but the microcode reads/writes it as rx (dcnt1/dcnt2, App. H p146)" + nl)
    f.write("  // PERSISTENT pointers: the REGISTER POINTERS cell is already valid in the word that sets it and until it changes;" + nl)
    f.write("  // dx/dy (and uk as source) do not change the pointer; uk as destination = index register rz from IRC" + nl)
    f.write('  logic [ND_RP_DST_W-1:0] pd_w; logic [ND_RP_SRC_W-1:0] ps_w;   // one-hot: dt/dx/rp/rx/sp/uk/us and dt/dy/py/ry/uk' + nl)
    f.write('  assign pd_w = dec[ND_RP_DST_LSB +: ND_RP_DST_W]; assign ps_w = dec[ND_RP_SRC_LSB +: ND_RP_SRC_W];' + nl)
    f.write("  logic new_ird; assign new_ird = (dec[ND_IRD_FROM_LSB +: ND_IRD_FROM_W] == ND_IRD_FROM__P_IR[ND_IRD_FROM_W-1:0]);   // new instruction" + nl)
    # the pointer type is kept only in the registered flags pdr_*/psr_* (no pd/ps registers, no pd_e/ps_e values)
    f.write('  // A7: resolved with sup (S of the word) as soon as it is selected (SSP at index 16, USP at index 15)' + nl)
    f.write('  logic [4:0] sp_r; assign sp_r = sup ? 5\'d16 : 5\'d15;' + nl)
    f.write('  assign sp_i = sp_r;   // active SP already resolved by S (16 SSP / 15 USP), with no final remapping; 15 = explicit USP (US)' + nl)
    f.write('  // type of the generic register: MOVEA (MOVE lines with IRD 8:6 = 001), EXG Ax,Ay / Dx,Ay, LEA; EA mode An (IRD 5:3 = 001)' + nl)
    f.write('  assign rx_is_a = ((ird[15:12] == 4\'h1 || ird[15:12] == 4\'h2 || ird[15:12] == 4\'h3) && ird[8:6] == 3\'b001) | (ird[15:12] == 4\'hc && ird[8:3] == 6\'b101001) | (ird[15:12] == 4\'h4 && ird[8:6] == 3\'b111) | ((ird[15:12] == 4\'h9 || ird[15:12] == 4\'hb || ird[15:12] == 4\'hd) && ird[7:6] == 2\'b11);   // ADDA/SUBA/CMPA: rx = An' + nl)
    # ry = EA register (bits 2:0): An for the modes 001..110 (An, (An), (An)+, -(An), d16(An), d8(An,Xn)); Dn for 000; none for 111.
    # Exception: register shifts (line E, 7:6 != 11): bits 5:3 are i/r + type. Case: popm2 '(ry) -> db -> aob' with (An)+ must read An, not Dn.
    f.write('  assign ry_is_a = ((ird[5:3] != 3\'b000 && ird[5:3] != 3\'b111) && !(ird[15:12] == 4\'he && ird[7:6] != 2\'b11) && !(ird[15:12] == 4\'h5 && ird[7:3] == 5\'b11001)) | (ird[15:12] == 4\'hc && ird[8] && (ird[7:3] == 5\'b01001 || ird[7:3] == 5\'b10001));   // DBcc (mode 001): y is a DATA register (otherwise the exit word roaw2 would write A1)' + nl)
    f.write('  assign pcrel = (ird[5:3] == 3\'b111) && (ird[2:1] == 2\'b01);' + nl)
    # --- pointer decoding in parallel form: candidates from registers only, per word only the mux on the pointer type
    f.write("  // candidates that do not depend on the nanoword (ird, S, MOVEM encoder): outside the critical cone dec -> index -> read" + nl)
    f.write("  logic [4:0] c_x7, c_y7, c_rp, rx_def, rxd_def, ry_def, rz_sel, spv; logic special, rz_sel_on;" + nl)
    f.write("  assign c_x7 = (ird[11:9] == 3'd7) ? sp_i : {2'b01, ird[11:9]};" + nl)
    f.write("  assign c_y7 = (ird[2:0] == 3'd7) ? sp_i : {2'b01, ird[2:0]};" + nl)
    f.write("  assign c_rp = (rp_reg == 4'd15) ? sp_i : {1'b0, rp_reg};   // MOVEM encoder" + nl)
    f.write("  assign rxd_def = rx_lo ? {2'b00, ird[2:0]} : {2'b00, ird[11:9]};   // DBcc: counter in IR[2:0]" + nl)
    f.write("  assign rx_def = rx_is_a ? c_x7 : rxd_def;" + nl)
    f.write("  assign ry_def = ry_is_a ? c_y7 : {2'b00, ird[2:0]};" + nl)
    f.write("  assign rz_sel = (irc[15:12] == 4'd15) ? sp_i : {1'b0, irc[15:12]};   // rz: index register from the extension word (aixw2/aixw4; die: reg[irc >> 12])" + nl)
    f.write("  assign rz_sel_on = (dec[ND_RXSRC_LSB +: ND_RXSRC_W] == ND_RXSRC__RZ[ND_RXSRC_W-1:0]);" + nl)
    f.write("  // special pointer of the word (DT/SP/US/RP): a single mux on pdw_v" + nl)
    f.write("  // (special/spv are assigned only by the always_comb below: a second block assigning them would be a second" + nl)
    f.write("  //  driver, which Quartus rejects.)" + nl)
    # one-hot pointer type: of the word (from dec, one level) and registered (at en); a single mux on the valid bit
    f.write("  logic pdw_v, pdw_dt, pdw_sp, pdw_us, pdw_rp, pdr_dt, pdr_sp, pdr_us, pdr_rp; logic [4:0] spv_w, spv_r;" + nl)
    f.write("  // one-hot field: flag = bit of dec; valid = any except dx/uk (dt, rp, rx, sp, us)" + nl)
    f.write("  assign pdw_v  = pd_w[ND_RP_DST__DT] | pd_w[ND_RP_DST__RP] | pd_w[ND_RP_DST__RX] | pd_w[ND_RP_DST__SP] | pd_w[ND_RP_DST__US];" + nl)
    f.write("  assign pdw_dt = pd_w[ND_RP_DST__DT]; assign pdw_sp = pd_w[ND_RP_DST__SP];" + nl)
    f.write("  assign pdw_us = pd_w[ND_RP_DST__US]; assign pdw_rp = pd_w[ND_RP_DST__RP];" + nl)
    f.write("  always_ff @(posedge clk) begin" + nl)
    f.write("    if (!rst_n) begin pdr_dt <= 1'b0; pdr_sp <= 1'b0; pdr_us <= 1'b0; pdr_rp <= 1'b0; end" + nl)
    f.write("    else if (en) begin" + nl)
    f.write("      if (pdw_v) begin pdr_dt <= pdw_dt; pdr_sp <= pdw_sp; pdr_us <= pdw_us; pdr_rp <= pdw_rp; end" + nl)
    f.write("      else if (new_ird) begin pdr_dt <= 1'b0; pdr_sp <= 1'b0; pdr_us <= 1'b0; pdr_rp <= 1'b0; end" + nl)
    f.write("    end" + nl + "  end" + nl)
    # AND-OR on the exclusive flags (a single rp_dst value per word; the pdr_* come from the same flags): with no flag
    # the value is 0 instead of c_rp, but spv is read only with special = 1
    f.write("  assign spv_w = ({5{pdw_dt}} & 5'd18) | ({5{pdw_sp}} & sp_r) | ({5{pdw_us}} & 5'd15) | ({5{pdw_rp}} & c_rp);" + nl)
    f.write("  assign spv_r = ({5{pdr_dt}} & 5'd18) | ({5{pdr_sp}} & sp_r) | ({5{pdr_us}} & 5'd15) | ({5{pdr_rp}} & c_rp);" + nl)
    f.write("  always_comb begin" + nl)
    f.write("    if (pdw_v) begin special = pdw_dt | pdw_sp | pdw_us | pdw_rp; spv = spv_w; end" + nl)
    f.write("    else begin special = pdr_dt | pdr_sp | pdr_us | pdr_rp; spv = spv_r; end   // dispatch word: pointers of the current instruction" + nl)
    f.write("  end" + nl)
    f.write("  assign rxa_i = special ? spv : c_x7;" + nl)
    f.write("  assign rx_i  = rz_sel_on ? rz_sel : special ? spv : rx_def;" + nl)
    f.write("  assign rxd_i = special ? (rz_sel_on ? rz_sel : spv) : rxd_def;" + nl)
    # one-hot source pointer type (word / registered): a single mux for ry_i, rya_i, ryd_i
    f.write("  logic psw_v, psw_dt, psw_py, psr_dt, psr_py, ps_dt, ps_py;" + nl)
    f.write("  assign psw_v  = ps_w[ND_RP_SRC__DT] | ps_w[ND_RP_SRC__PY] | ps_w[ND_RP_SRC__RY];   // one-hot: valid = except dy/uk" + nl)
    f.write("  assign psw_dt = ps_w[ND_RP_SRC__DT]; assign psw_py = ps_w[ND_RP_SRC__PY];" + nl)
    f.write("  always_ff @(posedge clk) begin" + nl)
    f.write("    if (!rst_n) begin psr_dt <= 1'b0; psr_py <= 1'b0; end" + nl)
    f.write("    else if (en) begin" + nl)
    f.write("      if (psw_v) begin psr_dt <= psw_dt; psr_py <= psw_py; end" + nl)
    f.write("      else if (new_ird) begin psr_dt <= 1'b0; psr_py <= 1'b0; end" + nl)
    f.write("    end" + nl + "  end" + nl)
    f.write("  assign ps_dt = psw_v ? psw_dt : psr_dt; assign ps_py = psw_v ? psw_py : psr_py;" + nl)
    f.write("  assign rya_i = ps_dt ? 5'd18 : (ps_py && pcrel) ? 5'd17 : c_y7;" + nl)
    f.write("  assign ry_i  = ps_dt ? 5'd18 : (ps_py && pcrel) ? 5'd17 : ry_def;" + nl)
    f.write("  assign ryd_i = ps_dt ? 5'd18 : {2'b00, ird[2:0]};" + nl)
    # --- read ports: index = register of the source atom active on that bus (at most one per bus per word)
    f.write("  // read with forwarding of the pending DB write (at en+1); AB writes the MLABs at the en edge: the value read is the one 'after en'. The PC lives in pc_sh" + nl)
    f.write("  function automatic logic [31:0] fwd(input logic [4:0] idx, input logic [31:0] v);" + nl)
    f.write("    logic [31:0] o; o = v;" + nl)
    f.write("    if (q_db_v && q_db_i == idx) begin if (q_db_h) o[31:16] = q_db_wh; if (q_db_l) begin if (q_db_b && idx < 5'd8) o[7:0] = q_db_wl[7:0]; else o[15:0] = q_db_wl; end end" + nl)
    f.write("    return o;" + nl + "  endfunction" + nl)
    # read index as an AND-OR on the one-hot bits: exact because per word there is at most ONE register atom per
    # bus (census over the 547 nanowords: 0 with two registers on the same bus). Sources already remapped and forwarding
    # comparisons per source computed outside the cone dec -> index; the forwarding mux uses the same selects. With no
    # atom active the index is 0 and the forwarding is off.
    def sname(idx): return 'pc' if idx == "5'd17" else idx[:-2]
    srcs = []
    for b in ('ab', 'db'):
        for at, idx in rd[b]:
            if idx not in srcs: srcs.append(idx)
    f.write('  // remapped sources (A7 -> USP/SSP) and forwarding comparisons per source: from ird/pointers/S/queue, not from dec' + nl)
    for idx in srcs:
        n = sname(idx)
        f.write(f'  logic [4:0] s_{n}; assign s_{n} = {idx};' + nl)
        # index 17 (PC) can come only from the pc source and from ry/rya with the py pointer on a PC-relative EA:
        # the other sources (rx*, ryd, sp) are 0-16 or 18 (one-hot pointers: no word has two rp_dst), so hpc = 0
        hpc = f"(s_{n} == 5'd17)" if idx in ("5'd17", 'ry_i', 'rya_i') else "1'b0"
        f.write(f"  logic hdb_{n}, hpc_{n}, lo_{n}, eqsp_{n}; assign hdb_{n} = q_db_v & (q_db_i == s_{n}); "
                f"assign hpc_{n} = {hpc}; assign lo_{n} = (s_{n} < 5'd8); assign eqsp_{n} = (s_{n} == sp_r);" + nl)
    for b in ('ab', 'db'):
        groups = {}
        for at, idx in rd[b]: groups.setdefault(idx, []).append(at)
        for idx, ats in groups.items():
            n = sname(idx)
            f.write(f'  logic sel_{b}_{n}; assign sel_{b}_{n} = ' + ' | '.join(f'({cond(at)})' for at in ats) + ';   // ' + ' '.join(ats) + nl)
        ns = [sname(i) for i in groups]
        f.write(f'  logic [4:0] ri_{b}_f; assign ri_{b}_f = ' + ' | '.join(f'({{5{{sel_{b}_{n}}}}} & s_{n})' for n in ns) + ';' + nl)
        f.write(f'  logic h_db_{b}, h_pc_{b}, lo_{b};' + nl)
        for h in ('hdb', 'hpc', 'lo'):
            lhs = {'hdb': f'h_db_{b}', 'hpc': f'h_pc_{b}', 'lo': f'lo_{b}'}[h]
            f.write(f'  assign {lhs} = ' + ' | '.join(f'(sel_{b}_{n} & {h}_{n})' for n in ns) + ';' + nl)
        RB = 'ra' if b == 'ab' else 'rb'
        f.write(f'  logic [31:0] rr_{b}; assign rr_{b} = {{{RB}_h[ri_{b}_f], {RB}_m[ri_{b}_f], {RB}_l[ri_{b}_f]}};' + nl)
        # free-running registered read: the consumers (operand latches at ph1, S1, S2, en) are >= 2 pulses away
        f.write(f'  logic [31:0] rf_{b}_c;   // combinational read with forwarding; rf_{b} = free-running register (lag 1 clock)' + nl)
        f.write('  always_comb begin' + nl + f'    rf_{b}_c = h_pc_{b} ? pc_sh : rr_{b};   // the PC (index 17) lives in pc_sh' + nl)
        f.write(f'    if (h_db_{b}) begin if (q_db_h) rf_{b}_c[31:16] = q_db_wh; if (q_db_l) begin if (q_db_b && lo_{b}) rf_{b}_c[7:0] = q_db_wl[7:0]; else rf_{b}_c[15:0] = q_db_wl; end end' + nl)
        f.write('  end' + nl)
        f.write(f'  always_ff @(posedge clk) rf_{b} <= rf_{b}_c;   // valid from the clock after en: op_ld (ph1), S1, S2 and en are >= 2 pulses apart' + nl)
        if b == 'db':
            f.write('  logic rf_db_sp_c; assign rf_db_sp_c = ' + ' | '.join(f'(sel_db_{n} & eqsp_{n})' for n in ns) + ';' + nl)
            f.write('  always_ff @(posedge clk) rf_db_sp <= rf_db_sp_c;   // consumed by the AU at op_ld (ph1)' + nl)
    f.write("  assign pc_q = fwd(5'd17, pc_sh);" + nl)
    f.write('  generate if (DBG) begin : g_dbg' + nl + '    assign dbg_data_d = fwd(dbg_idx_d, ' + "(dbg_idx_d == 5'd17) ? pc_sh : " + '{ra_h[dbg_idx_d], ra_m[dbg_idx_d], ra_l[dbg_idx_d]});' + nl)
    f.write('    assign dbg_data_a = fwd(dbg_idx_a, ' + "(dbg_idx_a == 5'd17) ? pc_sh : " + '{ra_h[dbg_idx_a], ra_m[dbg_idx_a], ra_l[dbg_idx_a]});' + nl)
    f.write("  end else begin : g_nodbg" + nl + "    assign dbg_data_d = '0; assign dbg_data_a = '0;" + nl + '  end endgenerate' + nl)
    # --- write ports: per port (ab, db, pc) index, H/L enables and data
    for P in ('ab', 'db', 'pc'):
        f.write(f'  logic we_{P}_h, we_{P}_l; logic [15:0] wd_{P}_h, wd_{P}_l;' + (f' logic [4:0] wi_{P};' if P != 'pc' else '') + nl)
        # PARALLEL form: per port and per half at most one write atom is active per word (assert below),
        # so no priority chain: index = OR of the masked candidates, enable = OR, data = choice among the distinct
        # expressions with the last one as default (with no active atom the enable is 0 and the data is not used)
        idxg = {}; eng = {'l': {}, 'h': {}}; dg = {'l': {}, 'h': {}}; halves_of = {}
        for at, specs in rules.items():
            if port_of(at) != P: continue
            c = cond(at)
            for sp in sorted(specs, key=lambda x: x.split('=')[0].endswith('_h')):
                part, s = sp.split('=', 1); tgt, half = part.rsplit('_', 1)
                en, expr = sv_data(s)
                if expr == 'SEXT': expr = f'{{16{{wd_{P}_l[15]}}}}'
                if P != 'pc' and c not in idxg.setdefault(TI[tgt], []): idxg[TI[tgt]].append(c)
                if c not in eng[half].setdefault(en, []): eng[half][en].append(c)
                if c not in dg[half].setdefault(expr, []): dg[half][expr].append(c)
                halves_of.setdefault(at, set()).add(half)
        for a_, A_ in words.items():
            for half in ('l', 'h'):
                n_ = sum(1 for at in A_ if at in halves_of and half in halves_of[at])
                assert n_ <= 1, ('two write atoms on the same half of the port', P, half, '%03x' % a_)
        def orc(cs): return '(' + ' | '.join(f'({c})' for c in cs) + ')'
        if P != 'pc':
            f.write(f'  assign wi_{P} = ' + ' | '.join(f'({{5{{{orc(cs)}}}}} & {idx})' for idx, cs in idxg.items()) + ';' + nl)
        for half in ('l', 'h'):
            f.write(f'  assign we_{P}_{half} = ' + (' | '.join((orc(cs) if en == "1'b1" else f'({orc(cs)} & ({en}))') for en, cs in eng[half].items()) or "1'b0") + ';' + nl)
            groups = sorted(dg[half].items(), key=lambda kv: len(kv[1]))   # the most used expression last = default
            expr = groups[-1][0] if groups else "'0"
            for e, cs in reversed(groups[:-1]): expr = f'{orc(cs)} ? {e} : ({expr})'
            f.write(f'  assign wd_{P}_{half} = {expr};' + nl)
    # --- a single physical port: AB direct at en, DB queued (en+1), PC port into pc_sh at en; forwarding on the reads
    f.write("  // write: one physical port, shared by the microcode AB and DB ports through the same mux; the read ports forward" + nl)
    f.write("  // the pending write. The PC port writes pc_sh directly at en." + nl)
    f.write("  // The AB port writes the MLABs directly at the en edge; only the DB port is queued (applied at en+1)." + nl)
    f.write("  logic q_db_v, q_db_h, q_db_l, q_db_b; logic [15:0] q_db_wh, q_db_wl; logic [4:0] q_db_i;" + nl)
    f.write("  // write indices registered free-running (used only at en, last clock of the word; dec is valid from the first)" + nl)
    f.write("  logic [4:0] wi_ab_q, wi_db_q; always_ff @(posedge clk) begin wi_ab_q <= wi_ab; wi_db_q <= wi_db; end" + nl)
    f.write("  logic w_h, w_l, w_b; logic [15:0] w_dh, w_dl; logic [4:0] w_i;" + nl)
    f.write("  always_comb begin" + nl)
    f.write("    if (en && (we_ab_h | we_ab_l)) begin w_h = we_ab_h; w_l = we_ab_l; w_dh = wd_ab_h; w_dl = wd_ab_l; w_i = wi_ab_q; w_b = byte_op; end   // AB port direct at en" + nl)
    f.write("    else if (q_db_v) begin w_h = q_db_h; w_l = q_db_l; w_dh = q_db_wh; w_dl = q_db_wl; w_i = q_db_i; w_b = q_db_b; end" + nl)
    f.write("    else             begin w_h = 1'b0; w_l = 1'b0; w_dh = q_db_wh; w_dl = q_db_wl; w_i = q_db_i; w_b = q_db_b; end   // with no write, data and index are not used: no default to zero" + nl)
    f.write("  end" + nl)
    f.write("  always_ff @(posedge clk) begin" + nl)
    f.write("    if (!rst_n) begin q_db_v <= 1'b0; q_db_h <= 1'b0; q_db_l <= 1'b0; q_db_b <= 1'b0; q_db_wh <= '0; q_db_wl <= '0; q_db_i <= '0; end" + nl)
    f.write("    else if (en) begin" + nl)
    f.write("      q_db_v <= we_db_h | we_db_l; q_db_h <= we_db_h; q_db_l <= we_db_l; q_db_wh <= wd_db_h; q_db_wl <= wd_db_l; q_db_i <= wi_db_q; q_db_b <= byte_op;" + nl)
    f.write("    end else if (q_db_v) q_db_v <= 1'b0;" + nl)
    f.write("  end" + nl)
    f.write("  // write: one per clock (w_*); byte write (w_b) only on the data registers D0-D7: the 15:8 column is kept" + nl)
    f.write("  always_ff @(posedge clk) begin" + nl)
    f.write("    if (w_h) begin ra_h[w_i] <= w_dh; rb_h[w_i] <= w_dh; end" + nl)
    f.write("    if (w_l) begin" + nl)
    f.write("      ra_l[w_i] <= w_dl[7:0]; rb_l[w_i] <= w_dl[7:0];" + nl)
    f.write("      if (!(w_b && w_i < 5'd8)) begin ra_m[w_i] <= w_dl[15:8]; rb_m[w_i] <= w_dl[15:8]; end" + nl)
    f.write("    end" + nl + "  end" + nl)
    f.write("  always_ff @(posedge clk) begin" + nl + "    if (!rst_n) pc_sh <= 32'd0;" + nl + "    else begin" + nl)
    f.write("      if (w_h && w_i == 5'd17) pc_sh[31:16] <= w_dh;" + nl)
    f.write("      if (w_l && w_i == 5'd17) pc_sh[15:0]  <= w_dl;" + nl)
    f.write("      if (en && we_pc_h) pc_sh[31:16] <= wd_pc_h;   // direct PC port at en: the PC lives only here" + nl)
    f.write("      if (en && we_pc_l) pc_sh[15:0]  <= wd_pc_l;" + nl)
    f.write("    end" + nl + "  end" + nl + "endmodule" + nl)
with open(f'{R}/sim/m68k_regfile_top.sv', 'w', newline=nl) as f:
    f.write(LICENSE_HEADER)
    f.write('// m68k_regfile_top.sv - register file testbench top: control store + decoder + regfile (bus sections and AU driven by the tb). GENERATED by gen_regfile.py.' + nl)
    f.write('import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;' + nl)
    f.write('module m68k_regfile_top #(parameter MEM_FILE = "m68k_urom.mem", parameter ORIGIN_FILE = "m68k_norigin.mem", parameter NANO_FILE = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem") (' + nl)
    f.write('  input  logic clk, rst_n, en, input logic [9:0] a1, a2, a3, input logic a0_sub, input logic [9:0] a0, input logic a0s_sub, input logic [9:0] a0s, input logic [1:0] c1c0,' + nl)
    f.write('  output logic [9:0] upc, output logic [NDEC_W-1:0] dec,' + nl)
    f.write('  input  logic [15:0] ird, input logic sup,' + nl)
    f.write('  input  logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,' + nl)
    f.write('  input  logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,' + nl)
    f.write('  input  logic [31:0] au,' + nl)
    f.write('  input  logic [15:0] mask_in,     // MOVEM mask word (bus, with ftu<-ab)' + nl)
    f.write('  input  logic [15:0] irc,' + nl)
    f.write('  output logic [3:0]  rp_reg, output logic mvm_end,' + nl)
    f.write('  output logic [31:0] rf_ab, rf_db, pc_q,' + nl)
    f.write('  input  logic [4:0]  dbg_idx_d, dbg_idx_a,' + nl + '  output logic [31:0] dbg_data_d, dbg_data_a' + nl + ');' + nl)
    f.write('  /* verilator lint_off UNUSEDSIGNAL */' + nl)
    f.write('  logic [16:0] uw; logic [NROM_W-1:0] nw; logic [NROM_IDX_W-1:0] nidx; logic [9:0] uaddr; logic ir_load, fmt, bit4; logic [1:0] ty, fc; logic [4:0] cbc;' + nl)
    f.write('  /* verilator lint_on UNUSEDSIGNAL */' + nl)
    f.write('  m68k_nstore_dec #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE)) u_cs (.clk(clk), .rst_n(rst_n), .en(en), .a1(a1), .a2(a2), .a3(a3),' + nl)
    f.write('    .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s), .c1c0(c1c0), .upc(upc), .uw(uw), .nw(nw), .dec(dec), .nidx(nidx), .uaddr(uaddr),' + nl)
    f.write('    .ir_load(ir_load), .fmt(fmt), .bit4(bit4), .ty(ty), .fc(fc), .cbc(cbc));' + nl)
    f.write('  m68k_mvm_si u_mvm (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ird(ird), .mask_in(mask_in), .rp_reg(rp_reg), .mvm_end(mvm_end));' + nl)
    f.write('  m68k_regfile_si #(.DBG(1\'b1)) u_rf (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ird(ird), .sup(sup), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),' + nl)
    f.write('    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv), .au(au), .rp_reg(rp_reg), .irc(irc),' + nl)
    f.write('    .rf_ab(rf_ab), .rf_db(rf_db), .pc_q(pc_q), .dbg_idx_d(dbg_idx_d), .dbg_idx_a(dbg_idx_a), .dbg_data_d(dbg_data_d), .dbg_data_a(dbg_data_a));' + nl)
    f.write('endmodule' + nl)
print(f'register write atoms {len(rules)} | specifications {sum(len(v) for v in rules.values())} | register sources on AB {len(rd["ab"])} / DB {len(rd["db"])} | '
      f'registers 19 x 32 | read ports 2 (+PC) | write ports 2 + PC | words with >1 destination per port {multi}')
