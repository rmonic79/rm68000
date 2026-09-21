// This file is part of rm68000, an MC68000 core in SystemVerilog.
// Copyright (c) 2026 Umberto Parisi (rmonic79)
//
// rm68000 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
// later version. It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details: LICENSE in the root of the repository, or <https://www.gnu.org/licenses/>.
//
// A commercial license, for use without the obligations of the GPL (for example in closed-source products), is
// available from the author, who can also be contacted for other licensing needs.

// m68k_regfile_si.sv - register file of the silicon-structured datapath: D0-D7, A0-A6, USP, SSP, PC, DT (docs/silicon-datapath.md §8).
// Silicon structure: 2 read ports (one per bus: rf_ab/rf_db = active source register on that bus), 2 write ports (AB, DB) + PC port.
// Maintained by hand; derived from the output of docs/data/scripts/gen_regfile.py (retired: it refuses to write), then edited: SSP at
// index 31, read port selectors registered (g_rd), flat index forms, pointer selections and write enables from flops, write data
// from the narrow fabric wires, PC port data from `at`/`au`, safety nets. Constants written by hand: every dec[L +: W] == W'dC decodes
// one nano atom of m68k_nano_dec_pkg.sv (gen_nano_dec.py): L = ND_<field>_LSB, W = ND_<field>_W, C = ND_<field>__<code>; dec[L + C]
// for the one-hot fields; dec[153] | dec[154] = any pc<- atom. If that layout changes, these constants must be rewritten.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_regfile_si #(parameter bit DBG = 1'b1,
                         // 1 = `special`/`spv` come out of a flop (needs dec_p/rp_next/sup_next); 0 = the combinational form.
                         // Kept at 0: `dec_p` arrives late (it comes out of the sequencer + nano ROM override), so a flop fed by it
                         // would itself become the worst path. The chain is cut instead by the flat form of the effective selections
                         // below (e_dt/e_sp/e_us/e_rp), which does not wait for the muxed dec_p.
                         parameter bit PRED_Q = 1'b0,
                         // 1 = write enables from dec alone (the census over the nanowords proves that the drv
                         // flags are always true in those words); 0 = form with the flags (we_*_v, also the
                         // reference of the safety net).
                         parameter bit WE_DEC_ONLY = 1'b1,
                         parameter bit WE_FLOP = 1'b0,   // 1 = the four write enables come from flops (we_*_i, from m68k_nstore_dec); 0 = computed here
                         // 1 = PC port with the data chosen between `at` and `au` and the enables from dec alone (the
                         // census over the 547 nanowords proves that the data is ALWAYS one of the two and that the requested
                         // section is always driven); 0 = form with the bus sections and the drv flags (does not read `at`;
                         // also the reference of the safety net).
                         parameter bit PC_DEC_ONLY = 1'b1,
                         // 1 = register file write data from the narrow fabric wires
                         // (ab_wl/ab_wh/db_wl/db_wh of m68k_bus_sec); 0 = the chains on the drv flags (also the reference of the safety net).
                         parameter bit WD_NARROW = 1'b1,
                         // 1 = pc_sh also accepts the GENERIC write with index 17 (w_i == 17); 0 = only the PC port (in the
                         // 68000 the PC-relative modes are source only, so no generic write ever has index 17; safety net below).
                         parameter bit PC_W17 = 1'b1,
                         parameter bit IR_REG = 1'b1) (   // 1 = the instruction values from `ir`, registered (core); 0 = combinational from `ird`
  input  logic clk, rst_n, en, en_rf,   // en_rf = local copy of en, used only by the register file write
  input  logic we_ab_h_i, we_ab_l_i, we_db_h_i, we_db_l_i,   // write enables from flops (with WE_FLOP)
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [NDEC_W-1:0] dec,
  // inputs to compute `special` and `spv` ONE CYCLE EARLIER and bring them out of a flop (PRED_Q = 1),
  // off the chain sup_e -> spv -> rx_i -> ri_ab_f -> MLAB. `dec_p` is the dec of the next word (it already
  // exists in the control store), `rp_next` and `sup_next` are the next values of the MOVEM index and of the
  // effective supervisor bit, both computed where they originate and with no new logic.
  input  logic [NDEC_W-1:0] dec_p,
  input  logic [NDEC_W-1:0] dec_pm, dec_po,   // the two branches of the dec_p override mux: dec_p = ovr ? dec_po : dec_pm
  input  logic        ovr,          // the selector of that mux
  // NEXT nanoword (one clock before en): kept on the interface but not read here (the effective selections
  // below use dec_pm/dec_po)
  input  logic [NROM_W-1:0] nw_pre,
  input  logic [3:0]  rp_next,
  input  logic        sup_next,
  input  logic [15:0] ir,           // word that ird will take at the next load: candidates decided one clock earlier
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [15:0] ird,          // instruction being executed (fields RX = 11:9, RY = 2:0, modes)
  input  logic [14:0] g_rd,         // the 15 read port selectors, registered at en in m68k_nstore_dec (g[26:12])
  input  logic        sup,          // supervisor mode (PSW S): A7 = SSP, otherwise USP
  input  logic        byte_op,      // byte instruction: writes to the low word of the DATA registers touch only the low byte (Scc/TAS/ADD.B...: the high byte is kept)
  input  logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,
  input  logic [15:0] ab_wl, ab_wh, db_wl, db_wh,   // narrow wires of the write data (not read with WD_NARROW = 0)
  input  logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,
  input  logic [31:0] au,
  input  logic [31:0] at,           // AT register (flop of m68k_regs_bd): PC port data when it is not `au`; not read with PC_DEC_ONLY = 0
  input  logic [3:0]  rp_reg,       // current register of the MOVEM encoder (0-7 = D, 8-15 = A; 15 = SP) for the rp pointer
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [15:0] irc,          // extension word of indexed addressing: index register rz = IRC[15:12] (uk pointer)
  input  logic [4:0]  dbg_idx_d, dbg_idx_a,   // debug (testbenches/lockstep): index of the register read (only with DBG)
  /* verilator lint_on UNUSEDSIGNAL */
  output logic [31:0] rf_ab, rf_db,  // active source register on bus AB / DB (read ports)
  output logic        rf_db_sp,      // the register read on DB is A7 (SP): AU uses -2/+2 even for bytes
  output logic [31:0] pc_q,          // PC
  output logic [31:0] dbg_data_d, dbg_data_a
);
  // indices: 0-7 D0-D7, 8-14 A0-A6, 15 USP, 17 PC, 18 DT, 31 SSP
  // storage in MLAB: asynchronous read (simple dual-port with unregistered read address, Cyclone V
  // handbook vol. 1), one copy per read port (AB, DB), three columns 16/8/8 for the byte write; a single write per
  // clock (AB at en, DB queued to en+1); forwarding from the queue covers every read of a pending value (no_rw_check). No reset
  // of the array, like the die. The observed PC (pc_q, debug only) comes from a shadow register written in parallel.
  (* ramstyle = "MLAB, no_rw_check" *) logic [15:0] ra_h [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [7:0]  ra_m [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [7:0]  ra_l [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [15:0] rb_h [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [7:0]  rb_m [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [7:0]  rb_l [0:31];
  logic [31:0] pc_sh;
  logic [4:0] sp_i, rx_i, ry_i, rxa_i, rya_i, rxd_i, ryd_i;
  logic rx_is_a, ry_is_a, pcrel, rx_lo;
  // with IR_REG = 0 the candidates are decided on ird and used without a register (same values, combinational).
  logic [15:0] w_ir; assign w_ir = IR_REG ? ir : ird;
  function automatic logic f_rx_lo(input logic [15:0] w); f_rx_lo = (w[15:12] == 4'h5) && (w[7:3] == 5'b11001); endfunction   // DBcc: the counter register is IR[2:0] but the microcode reads/writes it as rx (dcnt1/dcnt2, App. H p146)
  logic rx_lo_p; assign rx_lo_p = f_rx_lo(w_ir);
  // PERSISTENT pointers: the REGISTER POINTERS cell is already valid in the word that sets it and until it changes;
  // dx/dy (and uk as source) do not change the pointer; uk as destination = index register rz from IRC
  logic [ND_RP_DST_W-1:0] pd_w; logic [ND_RP_SRC_W-1:0] ps_w;   // one-hot: dt/dx/rp/rx/sp/uk/us and dt/dy/py/ry/uk
  assign pd_w = dec[ND_RP_DST_LSB +: ND_RP_DST_W]; assign ps_w = dec[ND_RP_SRC_LSB +: ND_RP_SRC_W];
  logic new_ird; assign new_ird = (dec[ND_IRD_FROM_LSB +: ND_IRD_FROM_W] == ND_IRD_FROM__P_IR[ND_IRD_FROM_W-1:0]);   // new instruction
  // A7: resolved with sup (S of the word) as soon as it is selected. SSP is at index 31 (11111) rather than 16 (10000):
  // only bit 4 of the index depends on `sup`, so the supervisor bit reaches the read index through a single wire.
  // The arrays have 32 entries for this, and the C++ model of the testbench (sim/tb_m68k_regfile_si.cpp) uses the same
  // numbering (R[32], `sp = dut->sup ? 31 : 15`).
  logic [4:0] sp_r; assign sp_r = sup ? 5'd31 : 5'd15;
  assign sp_i = sp_r;   // active SP already resolved by S (31 SSP / 15 USP), with no final remapping; 15 = explicit USP (US)
  // type of the generic register: MOVEA (MOVE lines with IRD 8:6 = 001), EXG Ax,Ay / Dx,Ay, LEA; EA mode An (IRD 5:3 = 001)
  function automatic logic f_rx_is_a(input logic [15:0] w); f_rx_is_a = ((w[15:12] == 4'h1 || w[15:12] == 4'h2 || w[15:12] == 4'h3) && w[8:6] == 3'b001) | (w[15:12] == 4'hc && w[8:3] == 6'b101001) | (w[15:12] == 4'h4 && w[8:6] == 3'b111) | ((w[15:12] == 4'h9 || w[15:12] == 4'hb || w[15:12] == 4'hd) && w[7:6] == 2'b11); endfunction   // ADDA/SUBA/CMPA: rx = An
  logic rx_is_a_p; assign rx_is_a_p = f_rx_is_a(w_ir);
  function automatic logic f_ry_is_a(input logic [15:0] w); f_ry_is_a = ((w[5:3] != 3'b000 && w[5:3] != 3'b111) && !(w[15:12] == 4'he && w[7:6] != 2'b11) && !(w[15:12] == 4'h5 && w[7:3] == 5'b11001)) | (w[15:12] == 4'hc && w[8] && (w[7:3] == 5'b01001 || w[7:3] == 5'b10001)); endfunction   // DBcc (mode 001): y is a DATA register (otherwise the exit word roaw2 would write A1)
  logic ry_is_a_p; assign ry_is_a_p = f_ry_is_a(w_ir);
  function automatic logic f_pcrel(input logic [15:0] w); f_pcrel = (w[5:3] == 3'b111) && (w[2:1] == 2'b01); endfunction
  logic pcrel_p; assign pcrel_p = f_pcrel(w_ir);
  // candidates that do not depend on the nanoword (ird, S, MOVEM encoder): outside the critical cone dec -> index -> read
  logic [4:0] c_x7, c_y7, c_rp, rx_def, rxd_def, ry_def, rz_sel, spv; logic special, rz_sel_on;
  // part that depends only on the instruction: decided on ir and registered at en & new_ird;
  // the choice on sp_i stays combinational because sp_i changes on its own.
  logic x7_p, y7_p; logic [4:0] xv_p, yv_p, rxd_p, rylo_p;
  assign x7_p   = (w_ir[11:9] == 3'd7);
  assign y7_p   = (w_ir[2:0]  == 3'd7);
  assign xv_p   = {2'b01, w_ir[11:9]};
  assign yv_p   = {2'b01, w_ir[2:0]};
  assign rxd_p  = f_rx_lo(w_ir) ? {2'b00, w_ir[2:0]} : {2'b00, w_ir[11:9]};
  assign rylo_p = {2'b00, w_ir[2:0]};
  logic x7_q, y7_q; logic [4:0] xv_q, yv_q, rxd_q, rylo_q;
  logic rx_is_a_fl, ry_is_a_fl, pcrel_fl, rx_lo_fl, x7_q_fl, y7_q_fl; logic [4:0] xv_q_fl, yv_q_fl, rxd_q_fl, rylo_q_fl;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rx_is_a_fl <= f_rx_is_a(16'd0); ry_is_a_fl <= f_ry_is_a(16'd0); pcrel_fl <= f_pcrel(16'd0); rx_lo_fl <= f_rx_lo(16'd0);
      x7_q_fl <= 1'b0; y7_q_fl <= 1'b0; xv_q_fl <= {2'b01, 3'd0}; yv_q_fl <= {2'b01, 3'd0};
      rxd_q_fl <= f_rx_lo(16'd0) ? {2'b00, 3'd0} : {2'b00, 3'd0}; rylo_q_fl <= {2'b00, 3'd0};
    end else if (en && new_ird) begin
      rx_is_a_fl <= rx_is_a_p; ry_is_a_fl <= ry_is_a_p; pcrel_fl <= pcrel_p; rx_lo_fl <= rx_lo_p;
      x7_q_fl <= x7_p; y7_q_fl <= y7_p; xv_q_fl <= xv_p; yv_q_fl <= yv_p; rxd_q_fl <= rxd_p; rylo_q_fl <= rylo_p;
    end
  end
  assign rx_is_a = IR_REG ? rx_is_a_fl : rx_is_a_p;
  assign ry_is_a = IR_REG ? ry_is_a_fl : ry_is_a_p;
  assign pcrel = IR_REG ? pcrel_fl : pcrel_p;
  assign rx_lo = IR_REG ? rx_lo_fl : rx_lo_p;
  assign x7_q = IR_REG ? x7_q_fl : x7_p;
  assign y7_q = IR_REG ? y7_q_fl : y7_p;
  assign xv_q = IR_REG ? xv_q_fl : xv_p;
  assign yv_q = IR_REG ? yv_q_fl : yv_p;
  assign rxd_q = IR_REG ? rxd_q_fl : rxd_p;
  assign rylo_q = IR_REG ? rylo_q_fl : rylo_p;
  assign c_x7 = x7_q ? sp_i : xv_q;
  assign c_y7 = y7_q ? sp_i : yv_q;
  assign c_rp = (rp_reg == 4'd15) ? sp_i : {1'b0, rp_reg};   // MOVEM encoder
  assign rxd_def = rxd_q;   // DBcc: counter in IR[2:0]
  assign rx_def = rx_is_a ? c_x7 : rxd_def;
  assign ry_def = ry_is_a ? c_y7 : rylo_q;
  assign rz_sel = (irc[15:12] == 4'd15) ? sp_i : {1'b0, irc[15:12]};   // rz: index register from the extension word (aixw2/aixw4; die: reg[irc >> 12])
  assign rz_sel_on = (dec[ND_RXSRC_LSB +: ND_RXSRC_W] == ND_RXSRC__RZ[ND_RXSRC_W-1:0]);
  // special pointer of the word (DT/SP/US/RP): a single mux on pdw_v
  // (special/spv are assigned only by the always_comb below: a second block assigning them would be a second
  //  driver, which Quartus rejects.)
  logic pdw_v, pdw_dt, pdw_sp, pdw_us, pdw_rp, pdr_dt, pdr_sp, pdr_us, pdr_rp; logic [4:0] spv_w, spv_r;
  // one-hot field: flag = bit of dec; valid = any except dx/uk (dt, rp, rx, sp, us)
  assign pdw_v  = pd_w[ND_RP_DST__DT] | pd_w[ND_RP_DST__RP] | pd_w[ND_RP_DST__RX] | pd_w[ND_RP_DST__SP] | pd_w[ND_RP_DST__US];
  assign pdw_dt = pd_w[ND_RP_DST__DT]; assign pdw_sp = pd_w[ND_RP_DST__SP];
  assign pdw_us = pd_w[ND_RP_DST__US]; assign pdw_rp = pd_w[ND_RP_DST__RP];
  always_ff @(posedge clk) begin
    if (!rst_n) begin pdr_dt <= 1'b0; pdr_sp <= 1'b0; pdr_us <= 1'b0; pdr_rp <= 1'b0; end
    else if (en) begin
      if (pdw_v) begin pdr_dt <= pdw_dt; pdr_sp <= pdw_sp; pdr_us <= pdw_us; pdr_rp <= pdw_rp; end
      else if (new_ird) begin pdr_dt <= 1'b0; pdr_sp <= 1'b0; pdr_us <= 1'b0; pdr_rp <= 1'b0; end
    end
  end
  assign spv_w = ({5{pdw_dt}} & 5'd18) | ({5{pdw_sp}} & sp_r) | ({5{pdw_us}} & 5'd15) | ({5{pdw_rp}} & c_rp);
  assign spv_r = ({5{pdr_dt}} & 5'd18) | ({5{pdr_sp}} & sp_r) | ({5{pdr_us}} & 5'd15) | ({5{pdr_rp}} & c_rp);
  // -- PRED_Q form BEGIN (off with PRED_Q = 0: synthesis prunes it, it has no outputs) --
  // the same expressions evaluated on the NEXT values, registered at en: at the edge that opens the word the flop
  // takes special/spv OF THAT word, like the registered selectors g of m68k_nstore_dec. Identical by construction;
  // the assertion below checks it at every en on everything the testbenches exercise.
  logic [ND_RP_DST_W-1:0] pd_wn; assign pd_wn = dec_p[ND_RP_DST_LSB +: ND_RP_DST_W];
  logic pdwn_v, pdwn_dt, pdwn_sp, pdwn_us, pdwn_rp;
  assign pdwn_v  = pd_wn[ND_RP_DST__DT] | pd_wn[ND_RP_DST__RP] | pd_wn[ND_RP_DST__RX] | pd_wn[ND_RP_DST__SP] | pd_wn[ND_RP_DST__US];
  assign pdwn_dt = pd_wn[ND_RP_DST__DT]; assign pdwn_sp = pd_wn[ND_RP_DST__SP];
  assign pdwn_us = pd_wn[ND_RP_DST__US]; assign pdwn_rp = pd_wn[ND_RP_DST__RP];
  // the pdr_* are flops: their next value is the same load rule as the block below
  logic pdrn_dt, pdrn_sp, pdrn_us, pdrn_rp;
  assign pdrn_dt = pdw_v ? pdw_dt : (new_ird ? 1'b0 : pdr_dt);
  assign pdrn_sp = pdw_v ? pdw_sp : (new_ird ? 1'b0 : pdr_sp);
  assign pdrn_us = pdw_v ? pdw_us : (new_ird ? 1'b0 : pdr_us);
  assign pdrn_rp = pdw_v ? pdw_rp : (new_ird ? 1'b0 : pdr_rp);
  logic [4:0] sp_rn; assign sp_rn = sup_next ? 5'd31 : 5'd15;
  logic [4:0] c_rp_n; assign c_rp_n = (rp_next == 4'd15) ? sp_rn : {1'b0, rp_next};   // the same rule as c_rp
  logic [4:0] spv_wn, spv_rn, spv_nx; logic special_nx;
  assign spv_wn = ({5{pdwn_dt}} & 5'd18) | ({5{pdwn_sp}} & sp_rn) | ({5{pdwn_us}} & 5'd15) | ({5{pdwn_rp}} & c_rp_n);
  assign spv_rn = ({5{pdrn_dt}} & 5'd18) | ({5{pdrn_sp}} & sp_rn) | ({5{pdrn_us}} & 5'd15) | ({5{pdrn_rp}} & c_rp_n);
  assign special_nx = pdwn_v ? (pdwn_dt | pdwn_sp | pdwn_us | pdwn_rp) : (pdrn_dt | pdrn_sp | pdrn_us | pdrn_rp);
  assign spv_nx = pdwn_v ? spv_wn : spv_rn;
  logic special_q; logic [4:0] spv_q;
  always_ff @(posedge clk) begin
    if (!rst_n) begin special_q <= 1'b0; spv_q <= 5'd0; end
    else if (en) begin special_q <= special_nx; spv_q <= spv_nx; end
  end
  always_comb begin
    if (pdw_v) begin special = pdw_dt | pdw_sp | pdw_us | pdw_rp; spv = spv_w; end
    else begin special = pdr_dt | pdr_sp | pdr_us | pdr_rp; spv = spv_r; end   // dispatch word: pointers of the current instruction
  end
  // with PRED_Q = 1 `special_e`/`spv_e` come from a flop, not from sup_e -> spv (they are not read: the index uses the flat form below).
  logic special_e; logic [4:0] spv_e;
  assign special_e = PRED_Q ? special_q : special;
  assign spv_e     = PRED_Q ? spv_q     : spv;
`ifdef VERILATOR
  // SAFETY NET, simulation only: at every `en` the flop must equal EXACTLY the combinational form. If the
  // derivation of the next value has a hole, the testbench stops here with the word in hand instead of running a
  // wrong core.
  generate if (PRED_Q) begin : g_special_spv_chk
    always @(posedge clk) if (rst_n && en) begin
      if (special_q !== special || spv_q !== spv) begin
        $display("special/spv MISMATCH t=%0t special q=%b c=%b  spv q=%0d c=%0d", $time, special_q, special, spv_q, spv);
        $fatal(1);
      end
    end
  end endgenerate
`endif
  // -- PRED_Q form END --
  // flat index form: the chain psws_p1 -> set_s_o -> sup_e -> sp_r -> spv_w -> spv -> rx_i -> ri_ab_f -> MLAB would be
  // eight levels from the flop to the MLAB address. Here they become two, with no new flop and the same logic function:
  //  - the four effective selections e_* are flops (loaded with their next value, below);
  //  - the SP index is NOT a mux: 31 = 5'b11111 and 15 = 5'b01111, i.e. {sup, 4'b1111} (sp_w), wires only;
  //  - spv_w/spv/the rx_i mux collapse into ONE single AND-OR, because the e_* are mutually
  //    exclusive by construction (pd_w is one-hot) and pdw_v selects them as a block.
  // Same Boolean function as the mux form on special/spv (still computed above); the register file testbench checks
  // the read ports against its own model.
  logic e_dt, e_sp, e_us, e_rp;
  // the four effective selections come out of a FLOP loaded at en with their NEXT value, like the registered selectors g:
  //   e_k' = pdw_v' ? pdw_k' : pdr_k'   with pdw_*' = rp_dst of the NEXT word
  //                                     and pdr_k' = pdw_v ? pdw_k : (new_ird ? 0 : pdr_k) = the load rule of the pdr_* (current flops).
  // Same value, same cycle; the safety net below re-checks it at every clock against the combinational form.
  logic e_dt_nx, e_sp_nx, e_us_nx, e_rp_nx;
  // the override mux at the END: e_k_nx = ovr ? f(dec_po) : f(dec_pm) with f(D) = D[rp_dst valid] ? D[rp_dst=k] : pdrn_k,
  // = f(dec_p) because dec_p = ovr ? dec_po : dec_pm. The registered branch (dec_pm, from the word read one clock earlier) is ready
  // 2-3 cells from the flops; in the core dec_po is a constant chosen by is3. Same value, same edge. Safety net below.
  logic [ND_RP_DST_W-1:0] pd_wm, pd_wo;
  assign pd_wm = dec_pm[ND_RP_DST_LSB +: ND_RP_DST_W]; assign pd_wo = dec_po[ND_RP_DST_LSB +: ND_RP_DST_W];
  logic pdwm_v, pdwo_v;
  assign pdwm_v = pd_wm[ND_RP_DST__DT] | pd_wm[ND_RP_DST__RP] | pd_wm[ND_RP_DST__RX] | pd_wm[ND_RP_DST__SP] | pd_wm[ND_RP_DST__US];
  assign pdwo_v = pd_wo[ND_RP_DST__DT] | pd_wo[ND_RP_DST__RP] | pd_wo[ND_RP_DST__RX] | pd_wo[ND_RP_DST__SP] | pd_wo[ND_RP_DST__US];
  logic e_dt_m, e_sp_m, e_us_m, e_rp_m, e_dt_o, e_sp_o, e_us_o, e_rp_o;
  assign e_dt_m = pdwm_v ? pd_wm[ND_RP_DST__DT] : pdrn_dt;  assign e_dt_o = pdwo_v ? pd_wo[ND_RP_DST__DT] : pdrn_dt;
  assign e_sp_m = pdwm_v ? pd_wm[ND_RP_DST__SP] : pdrn_sp;  assign e_sp_o = pdwo_v ? pd_wo[ND_RP_DST__SP] : pdrn_sp;
  assign e_us_m = pdwm_v ? pd_wm[ND_RP_DST__US] : pdrn_us;  assign e_us_o = pdwo_v ? pd_wo[ND_RP_DST__US] : pdrn_us;
  assign e_rp_m = pdwm_v ? pd_wm[ND_RP_DST__RP] : pdrn_rp;  assign e_rp_o = pdwo_v ? pd_wo[ND_RP_DST__RP] : pdrn_rp;
  assign e_dt_nx = ovr ? e_dt_o : e_dt_m;  assign e_sp_nx = ovr ? e_sp_o : e_sp_m;
  assign e_us_nx = ovr ? e_us_o : e_us_m;  assign e_rp_nx = ovr ? e_rp_o : e_rp_m;
`ifdef VERILATOR
  // SAFETY NET: the form computed from the already muxed dec_p, compared at every clock
  logic e_dt_nx_v, e_sp_nx_v, e_us_nx_v, e_rp_nx_v;
  assign e_dt_nx_v = pdwn_v ? pdwn_dt : pdrn_dt;  assign e_sp_nx_v = pdwn_v ? pdwn_sp : pdrn_sp;
  assign e_us_nx_v = pdwn_v ? pdwn_us : pdrn_us;  assign e_rp_nx_v = pdwn_v ? pdwn_rp : pdrn_rp;
  always @(posedge clk) if (rst_n && (e_dt_nx !== e_dt_nx_v || e_sp_nx !== e_sp_nx_v || e_us_nx !== e_us_nx_v || e_rp_nx !== e_rp_nx_v)) begin
    $display("e_nx MISMATCH t=%0t ovr=%b nx %b%b%b%b / %b%b%b%b", $time, ovr, e_dt_nx, e_sp_nx, e_us_nx, e_rp_nx, e_dt_nx_v, e_sp_nx_v, e_us_nx_v, e_rp_nx_v); $fatal(1);
  end
`endif
  logic e_dt_q, e_sp_q, e_us_q, e_rp_q;
  always_ff @(posedge clk) begin
    if (!rst_n) begin e_dt_q <= 1'b0; e_sp_q <= 1'b0; e_us_q <= 1'b0; e_rp_q <= 1'b0; end   // at word zero: pdw = 0 and pdr = 0
    else if (en) begin e_dt_q <= e_dt_nx; e_sp_q <= e_sp_nx; e_us_q <= e_us_nx; e_rp_q <= e_rp_nx; end
  end
  assign e_dt = e_dt_q;  assign e_sp = e_sp_q;  assign e_us = e_us_q;  assign e_rp = e_rp_q;
`ifdef VERILATOR
  // SAFETY NET: the combinational form (pdw_v ? pdw_k : pdr_k), compared at every clock.
  logic e_dt_v, e_sp_v, e_us_v, e_rp_v;
  assign e_dt_v = pdw_v ? pdw_dt : pdr_dt;  assign e_sp_v = pdw_v ? pdw_sp : pdr_sp;
  assign e_us_v = pdw_v ? pdw_us : pdr_us;  assign e_rp_v = pdw_v ? pdw_rp : pdr_rp;
  always @(posedge clk) if (rst_n && (e_dt !== e_dt_v || e_sp !== e_sp_v || e_us !== e_us_v || e_rp !== e_rp_v)) begin
    $display("e_sel MISMATCH t=%0t e_dt %b/%b e_sp %b/%b e_us %b/%b e_rp %b/%b", $time, e_dt, e_dt_v, e_sp, e_sp_v, e_us, e_us_v, e_rp, e_rp_v); $fatal(1);
  end
`endif
  logic g_dt, g_sp, g_us, g_rp, g_df;   // the same ones, already intersected with rz_sel_on
  assign g_dt = e_dt & ~rz_sel_on;  assign g_sp = e_sp & ~rz_sel_on;
  assign g_us = e_us & ~rz_sel_on;  assign g_rp = e_rp & ~rz_sel_on;
  assign g_df = ~(e_dt | e_sp | e_us | e_rp) & ~rz_sel_on;
  logic [4:0] sp_w; assign sp_w = {sup, 4'b1111};   // 31 if sup, 15 otherwise: only bit 4 depends on sup
  assign rxa_i = ({5{e_dt}} & 5'd18) | ({5{e_sp}} & sp_w) | ({5{e_us}} & 5'd15) | ({5{e_rp}} & c_rp)
               | ({5{~(e_dt | e_sp | e_us | e_rp)}} & c_x7);
  assign rx_i  = ({5{rz_sel_on}} & rz_sel)
               | ({5{g_dt}} & 5'd18) | ({5{g_sp}} & sp_w) | ({5{g_us}} & 5'd15) | ({5{g_rp}} & c_rp)
               | ({5{g_df}} & rx_def);
  assign rxd_i = ({5{rz_sel_on & (e_dt | e_sp | e_us | e_rp)}} & rz_sel)
               | ({5{g_dt}} & 5'd18) | ({5{g_sp}} & sp_w) | ({5{g_us}} & 5'd15) | ({5{g_rp}} & c_rp)
               | ({5{~(e_dt | e_sp | e_us | e_rp)}} & rxd_def);
  logic psw_v, psw_dt, psw_py, psr_dt, psr_py, ps_dt, ps_py;
  assign psw_v  = ps_w[ND_RP_SRC__DT] | ps_w[ND_RP_SRC__PY] | ps_w[ND_RP_SRC__RY];   // one-hot: valid = except dy/uk
  assign psw_dt = ps_w[ND_RP_SRC__DT]; assign psw_py = ps_w[ND_RP_SRC__PY];
  always_ff @(posedge clk) begin
    if (!rst_n) begin psr_dt <= 1'b0; psr_py <= 1'b0; end
    else if (en) begin
      if (psw_v) begin psr_dt <= psw_dt; psr_py <= psw_py; end
      else if (new_ird) begin psr_dt <= 1'b0; psr_py <= 1'b0; end
    end
  end
  assign ps_dt = psw_v ? psw_dt : psr_dt; assign ps_py = psw_v ? psw_py : psr_py;
  // Y side of the index as a flat AND-OR, like the X side: as a mux chain, synthesis puts ry_i and rya_i IN CHAIN
  // (they share the head `ps_dt ? 18 : …`) on the path dec -> ry_i -> rya_i -> MLAB read address -> rf_db.
  // Same function: the three conditions are exclusive by construction.
  logic y_dt, y_pc, y_df;
  assign y_dt = ps_dt;
  assign y_pc = ~ps_dt & ps_py & pcrel;
  assign y_df = ~ps_dt & ~(ps_py & pcrel);
  assign rya_i = ({5{y_dt}} & 5'd18) | ({5{y_pc}} & 5'd17) | ({5{y_df}} & c_y7);
  assign ry_i  = ({5{y_dt}} & 5'd18) | ({5{y_pc}} & 5'd17) | ({5{y_df}} & ry_def);
  assign ryd_i = ({5{y_dt}} & 5'd18) | ({5{~ps_dt}} & rylo_q);
`ifdef VERILATOR
  logic [4:0] rya_v, ry_v, ryd_v;
  assign rya_v = ps_dt ? 5'd18 : (ps_py && pcrel) ? 5'd17 : c_y7;
  assign ry_v  = ps_dt ? 5'd18 : (ps_py && pcrel) ? 5'd17 : ry_def;
  assign ryd_v = ps_dt ? 5'd18 : rylo_q;
  always @(posedge clk) if (rst_n && (rya_i !== rya_v || ry_i !== ry_v || ryd_i !== ryd_v)) begin
    $display("ry index MISMATCH t=%0t rya %0d/%0d ry %0d/%0d ryd %0d/%0d", $time, rya_i, rya_v, ry_i, ry_v, ryd_i, ryd_v); $fatal(1);
  end
`endif
  // read with forwarding of the pending DB write (at en+1); AB writes the MLABs at the en edge: the value read is the one 'after en'. The PC lives in pc_sh
  function automatic logic [31:0] fwd(input logic [4:0] idx, input logic [31:0] v);
    logic [31:0] o; o = v;
    if (q_db_v && q_db_i == idx) begin if (q_db_h) o[31:16] = q_db_wh; if (q_db_l) begin if (q_db_b && idx < 5'd8) o[7:0] = q_db_wl[7:0]; else o[15:0] = q_db_wl; end end
    return o;
  endfunction
  // remapped sources (A7 -> USP/SSP) and forwarding comparisons per source: from ird/pointers/S/queue, not from dec
  logic [4:0] s_rxd; assign s_rxd = rxd_i;
  logic hdb_rxd, hpc_rxd, lo_rxd, eqsp_rxd; assign hdb_rxd = q_db_v & (q_db_i == s_rxd); assign hpc_rxd = 1'b0; assign lo_rxd = (s_rxd < 5'd8); assign eqsp_rxd = (s_rxd == sp_r);
  logic [4:0] s_rx; assign s_rx = rx_i;
  logic hdb_rx, hpc_rx, lo_rx, eqsp_rx; assign hdb_rx = q_db_v & (q_db_i == s_rx); assign hpc_rx = 1'b0; assign lo_rx = (s_rx < 5'd8); assign eqsp_rx = (s_rx == sp_r);
  logic [4:0] s_ryd; assign s_ryd = ryd_i;
  logic hdb_ryd, hpc_ryd, lo_ryd, eqsp_ryd; assign hdb_ryd = q_db_v & (q_db_i == s_ryd); assign hpc_ryd = 1'b0; assign lo_ryd = (s_ryd < 5'd8); assign eqsp_ryd = (s_ryd == sp_r);
  logic [4:0] s_ry; assign s_ry = ry_i;
  logic hdb_ry, hpc_ry, lo_ry, eqsp_ry; assign hdb_ry = q_db_v & (q_db_i == s_ry); assign hpc_ry = (s_ry == 5'd17); assign lo_ry = (s_ry < 5'd8); assign eqsp_ry = (s_ry == sp_r);
  logic [4:0] s_pc; assign s_pc = 5'd17;
  logic hdb_pc, hpc_pc, lo_pc, eqsp_pc; assign hdb_pc = q_db_v & (q_db_i == s_pc); assign hpc_pc = (s_pc == 5'd17); assign lo_pc = (s_pc < 5'd8); assign eqsp_pc = (s_pc == sp_r);
  logic [4:0] s_rxa; assign s_rxa = rxa_i;
  logic hdb_rxa, hpc_rxa, lo_rxa, eqsp_rxa; assign hdb_rxa = q_db_v & (q_db_i == s_rxa); assign hpc_rxa = 1'b0; assign lo_rxa = (s_rxa < 5'd8); assign eqsp_rxa = (s_rxa == sp_r);
  logic [4:0] s_rya; assign s_rya = rya_i;
  logic hdb_rya, hpc_rya, lo_rya, eqsp_rya; assign hdb_rya = q_db_v & (q_db_i == s_rya); assign hpc_rya = (s_rya == 5'd17); assign lo_rya = (s_rya < 5'd8); assign eqsp_rya = (s_rya == sp_r);
  logic [4:0] s_sp; assign s_sp = sp_i;
  logic hdb_sp, hpc_sp, lo_sp, eqsp_sp; assign hdb_sp = q_db_v & (q_db_i == s_sp); assign hpc_sp = 1'b0; assign lo_sp = (s_sp < 5'd8); assign eqsp_sp = (s_sp == sp_r);
  // the read port selectors come out of `g_rd` (flops of m68k_nstore_dec loaded at en from masks on the slots of the
  // next nanoword). As ORs of 1-5 bits of dec_pq they would add one LUT level and a route at the head of ri_*_f, sel_any_*,
  // h_db/h_pc/lo, rf_db_sp_c. All bits are of class RAW (dec_pq): same value, same cycle. Safety net below.
  logic sel_ab_rxd; assign sel_ab_rxd = g_rd[0];   // ab*<-(rxd) ab*<-(rxdl) abd<-(rxd) abd<-(rxdl)   [dec 2 3 31 32]
  logic sel_ab_rx; assign sel_ab_rx = g_rd[1];   // ab*<-(rxl) ab<-(rx) ab<-(rxh) ab<-(rxl) abe<-(rxl)   [dec 4 16 18 19 35]
  logic sel_ab_ryd; assign sel_ab_ryd = g_rd[2];   // ab*<-(rydl) ab<-(ryd) ab<-(rydl)   [dec 5 22 23]
  logic sel_ab_ry; assign sel_ab_ry = g_rd[3];   // ab*<-(ryl) ab<-(ry) ab<-(ryh) ab<-(ryl) abe<-(ryl)   [dec 6 20 24 25 36]
  logic sel_ab_pc; assign sel_ab_pc = g_rd[4];   // ab<-(pc) ab<-(pch) ab<-(pcl)   [dec 13 14 15]
  logic sel_ab_rxa; assign sel_ab_rxa = g_rd[5];   // ab<-(rxa)   [dec 17]
  logic sel_ab_rya; assign sel_ab_rya = g_rd[6];   // ab<-(rya)   [dec 21]
`ifdef VERILATOR
  // SAFETY NET: the OR of the bits of dec, compared at every clock
  logic sel_ab_rxd_v, sel_ab_rx_v, sel_ab_ryd_v, sel_ab_ry_v, sel_ab_pc_v, sel_ab_rxa_v, sel_ab_rya_v;
  assign sel_ab_rxd_v = (dec[2]) | (dec[3]) | (dec[31]) | (dec[32]);
  assign sel_ab_rx_v = (dec[4]) | (dec[16]) | (dec[18]) | (dec[19]) | (dec[35]);
  assign sel_ab_ryd_v = (dec[5]) | (dec[22]) | (dec[23]);
  assign sel_ab_ry_v = (dec[6]) | (dec[20]) | (dec[24]) | (dec[25]) | (dec[36]);
  assign sel_ab_pc_v = (dec[13]) | (dec[14]) | (dec[15]);
  assign sel_ab_rxa_v = (dec[17]);
  assign sel_ab_rya_v = (dec[21]);
  always @(posedge clk) if (rst_n && {sel_ab_rxd, sel_ab_rx, sel_ab_ryd, sel_ab_ry, sel_ab_pc, sel_ab_rxa, sel_ab_rya} !== {sel_ab_rxd_v, sel_ab_rx_v, sel_ab_ryd_v, sel_ab_ry_v, sel_ab_pc_v, sel_ab_rxa_v, sel_ab_rya_v}) begin
    $display("g_rd MISMATCH ab t=%0t g=%b dec=%b", $time, {sel_ab_rxd, sel_ab_rx, sel_ab_ryd, sel_ab_ry, sel_ab_pc, sel_ab_rxa, sel_ab_rya}, {sel_ab_rxd_v, sel_ab_rx_v, sel_ab_ryd_v, sel_ab_ry_v, sel_ab_pc_v, sel_ab_rxa_v, sel_ab_rya_v}); $fatal(1);
  end
`endif
  // AB port index in FLAT form grouped by source: for each source (18/15/17 constants, SP, RP, RZ,
  // xv_q, rxd_def, yv_q, rylo_q) the OR of the products of registered conditions that select it (shared by the 5 bits), then
  // each bit = OR of (group & source bit) with the constants folded. Same algebra as the chain c_x7/rx_def ->
  // rx_i -> ri_ab_f (OR of masks), fewer levels. The mux form stays in ri_ab_f_v for the safety net.
  logic x1_eany, x1_z15, x1_rp15;
  assign x1_eany = e_dt | e_sp | e_us | e_rp;
  assign x1_z15 = (irc[15:12] == 4'hf);   // rz_sel = SP
  assign x1_rp15 = (rp_reg == 4'd15);     // c_rp = SP
  logic x1_ab_K18; assign x1_ab_K18 = (sel_ab_rxd & e_dt & ~rz_sel_on) | (sel_ab_rx & e_dt & ~rz_sel_on) | (sel_ab_ryd & y_dt) | (sel_ab_ry & y_dt) | (sel_ab_rxa & e_dt) | (sel_ab_rya & y_dt);
  logic x1_ab_K15; assign x1_ab_K15 = (sel_ab_rxd & e_us & ~rz_sel_on) | (sel_ab_rx & e_us & ~rz_sel_on) | (sel_ab_rxa & e_us);
  logic x1_ab_K17; assign x1_ab_K17 = (sel_ab_ry & y_pc) | (sel_ab_pc) | (sel_ab_rya & y_pc);
  logic x1_ab_SP; assign x1_ab_SP = (sel_ab_rxd & e_sp & ~rz_sel_on) | (sel_ab_rx & e_sp & ~rz_sel_on) | (sel_ab_rx & ~x1_eany & ~rz_sel_on & rx_is_a & x7_q) | (sel_ab_ry & y_df & ry_is_a & y7_q) | (sel_ab_rxa & e_sp) | (sel_ab_rxa & ~x1_eany & x7_q) | (sel_ab_rya & y_df & y7_q);
  logic x1_ab_RP; assign x1_ab_RP = (sel_ab_rxd & e_rp & ~rz_sel_on) | (sel_ab_rx & e_rp & ~rz_sel_on) | (sel_ab_rxa & e_rp);
  logic x1_ab_RZ; assign x1_ab_RZ = (sel_ab_rxd & rz_sel_on & x1_eany) | (sel_ab_rx & rz_sel_on);
  logic x1_ab_XV; assign x1_ab_XV = (sel_ab_rx & ~x1_eany & ~rz_sel_on & rx_is_a & ~x7_q) | (sel_ab_rxa & ~x1_eany & ~x7_q);
  logic x1_ab_RXD; assign x1_ab_RXD = (sel_ab_rxd & ~x1_eany) | (sel_ab_rx & ~x1_eany & ~rz_sel_on & ~rx_is_a);
  logic x1_ab_YV; assign x1_ab_YV = (sel_ab_ry & y_df & ry_is_a & ~y7_q) | (sel_ab_rya & y_df & ~y7_q);
  logic x1_ab_RYLO; assign x1_ab_RYLO = (sel_ab_ryd & ~ps_dt) | (sel_ab_ry & y_df & ~ry_is_a);
  logic [4:0] x1_ri_ab;
  assign x1_ri_ab[0] = x1_ab_K15 | x1_ab_K17 | x1_ab_SP | (x1_ab_RP & rp_reg[0]) | (x1_ab_RZ & irc[12]) | (x1_ab_XV & xv_q[0]) | (x1_ab_RXD & rxd_def[0]) | (x1_ab_YV & yv_q[0]) | (x1_ab_RYLO & rylo_q[0]);
  assign x1_ri_ab[1] = x1_ab_K18 | x1_ab_K15 | x1_ab_SP | (x1_ab_RP & rp_reg[1]) | (x1_ab_RZ & irc[13]) | (x1_ab_XV & xv_q[1]) | (x1_ab_RXD & rxd_def[1]) | (x1_ab_YV & yv_q[1]) | (x1_ab_RYLO & rylo_q[1]);
  assign x1_ri_ab[2] = x1_ab_K15 | x1_ab_SP | (x1_ab_RP & rp_reg[2]) | (x1_ab_RZ & irc[14]) | (x1_ab_XV & xv_q[2]) | (x1_ab_RXD & rxd_def[2]) | (x1_ab_YV & yv_q[2]) | (x1_ab_RYLO & rylo_q[2]);
  assign x1_ri_ab[3] = x1_ab_K15 | x1_ab_SP | (x1_ab_RP & rp_reg[3]) | (x1_ab_RZ & irc[15]) | (x1_ab_XV & xv_q[3]) | (x1_ab_RXD & rxd_def[3]) | (x1_ab_YV & yv_q[3]) | (x1_ab_RYLO & rylo_q[3]);
  assign x1_ri_ab[4] = x1_ab_K18 | x1_ab_K17 | (x1_ab_SP & sup) | (x1_ab_RP & x1_rp15 & sup) | (x1_ab_RZ & x1_z15 & sup) | (x1_ab_XV & xv_q[4]) | (x1_ab_RXD & rxd_def[4]) | (x1_ab_YV & yv_q[4]) | (x1_ab_RYLO & rylo_q[4]);
  logic [4:0] ri_ab_f, ri_ab_f_v; assign ri_ab_f_v = ({5{sel_ab_rxd}} & s_rxd) | ({5{sel_ab_rx}} & s_rx) | ({5{sel_ab_ryd}} & s_ryd) | ({5{sel_ab_ry}} & s_ry) | ({5{sel_ab_pc}} & s_pc) | ({5{sel_ab_rxa}} & s_rxa) | ({5{sel_ab_rya}} & s_rya);
  assign ri_ab_f = x1_ri_ab;
`ifdef VERILATOR
  always @(posedge clk) if (rst_n && ri_ab_f !== ri_ab_f_v) begin
    $display("read index MISMATCH ab t=%0t %0d/%0d", $time, ri_ab_f, ri_ab_f_v); $fatal(1);
  end
`endif
  logic h_db_ab, h_pc_ab, lo_ab;
  // the forwarding does not wait for one comparison per source and then the OR with the selections:
  // the selections are exclusive (the design already relies on this for ri_ab_f, the MLAB
  // address), so `OR_X (sel_X & pred(s_X))` = `sel_any & pred(ri_ab_f)`, with fewer levels than the per-source form.
  // No port can be 17 except the PC ones, and none can be < 8 except the data registers:
  // the predicates on the selected index are exact even for the ports that had them at 0.
  logic sel_any_ab; assign sel_any_ab = sel_ab_rxd | sel_ab_rx | sel_ab_ryd | sel_ab_ry | sel_ab_pc | sel_ab_rxa | sel_ab_rya;
  assign h_db_ab = sel_any_ab & q_db_v & (q_db_i == ri_ab_f);
  assign h_pc_ab = sel_any_ab & (ri_ab_f == 5'd17);
  assign lo_ab   = sel_any_ab & (ri_ab_f < 5'd8);
`ifdef VERILATOR
  // safety net: the per-source form, compared at every clock
  logic h_db_ab_v, h_pc_ab_v, lo_ab_v;
  assign h_db_ab_v = (sel_ab_rxd & hdb_rxd) | (sel_ab_rx & hdb_rx) | (sel_ab_ryd & hdb_ryd) | (sel_ab_ry & hdb_ry) | (sel_ab_pc & hdb_pc) | (sel_ab_rxa & hdb_rxa) | (sel_ab_rya & hdb_rya);
  assign h_pc_ab_v = (sel_ab_rxd & hpc_rxd) | (sel_ab_rx & hpc_rx) | (sel_ab_ryd & hpc_ryd) | (sel_ab_ry & hpc_ry) | (sel_ab_pc & hpc_pc) | (sel_ab_rxa & hpc_rxa) | (sel_ab_rya & hpc_rya);
  assign lo_ab_v = (sel_ab_rxd & lo_rxd) | (sel_ab_rx & lo_rx) | (sel_ab_ryd & lo_ryd) | (sel_ab_ry & lo_ry) | (sel_ab_pc & lo_pc) | (sel_ab_rxa & lo_rxa) | (sel_ab_rya & lo_rya);
  always @(posedge clk) if (rst_n && (h_db_ab !== h_db_ab_v || h_pc_ab !== h_pc_ab_v || lo_ab !== lo_ab_v)) begin
    $display("forwarding MISMATCH ab t=%0t h_db %b/%b h_pc %b/%b lo %b/%b", $time, h_db_ab, h_db_ab_v, h_pc_ab, h_pc_ab_v, lo_ab, lo_ab_v); $fatal(1);
  end
`endif
  logic [31:0] rr_ab; assign rr_ab = {ra_h[ri_ab_f], ra_m[ri_ab_f], ra_l[ri_ab_f]};
  // the low-byte predicate of the forwarding does NOT depend on the index read: inside h_db_X, ri_X_f == q_db_i holds,
  // so lo_X (= sel_any & ri_X_f < 8) == (q_db_i < 8), a function of the queue flops only. With this every bit of rf_*_c
  // is ONE function of 5 inputs {f_*, h_pc, queued data, pc_sh, MLAB read}: a single LUT6 after the MLAB on all 32 bits
  // (the direct form of the 15:8 slice has 8: rr, pc_sh, h_pc, h_db, q_db_l, q_db_b, lo, q_db_wl -> two levels). Pure algebra.
  logic q_db_bl, q_db_lm;
  assign q_db_bl = q_db_b & (q_db_i < 5'd8);   // queued: byte write to a data register -> the 15:8 slice is kept
  assign q_db_lm = q_db_l & ~q_db_bl;          // the 15:8 slice takes the queued data
  logic [31:0] rf_ab_c;   // combinational read with forwarding; rf_ab = free-running register (lag 1 clock)
  logic f_ab_h, f_ab_m, f_ab_l;   // exclusive forwarding qualifiers per slice (not read)
  assign f_ab_h = h_db_ab & q_db_h;  assign f_ab_m = h_db_ab & q_db_lm;  assign f_ab_l = h_db_ab & q_db_l;   // kept, not read: the final LUT uses the slice qualifiers directly
  // the slice qualifier (flop) enters the final LUT directly: 6 inputs, one level less after the index than
  // going through f_*.
  assign rf_ab_c[31:16] = (h_db_ab & q_db_h)  ? q_db_wh       : h_pc_ab ? pc_sh[31:16] : rr_ab[31:16];   // the PC (index 17) lives in pc_sh
  assign rf_ab_c[15:8]  = (h_db_ab & q_db_lm) ? q_db_wl[15:8] : h_pc_ab ? pc_sh[15:8]  : rr_ab[15:8];
  assign rf_ab_c[7:0]   = (h_db_ab & q_db_l)  ? q_db_wl[7:0]  : h_pc_ab ? pc_sh[7:0]   : rr_ab[7:0];
`ifdef VERILATOR
  // SAFETY NET: the if/else form, compared at every clock
  logic [31:0] rf_ab_c_v;
  always_comb begin
    rf_ab_c_v = h_pc_ab ? pc_sh : rr_ab;
    if (h_db_ab) begin if (q_db_h) rf_ab_c_v[31:16] = q_db_wh; if (q_db_l) begin if (q_db_b && lo_ab) rf_ab_c_v[7:0] = q_db_wl[7:0]; else rf_ab_c_v[15:0] = q_db_wl; end end
  end
  always @(posedge clk) if (rst_n && rf_ab_c !== rf_ab_c_v) begin
    $display("rf_c MISMATCH ab t=%0t %h / %h", $time, rf_ab_c, rf_ab_c_v); $fatal(1);
  end
`endif
  always_ff @(posedge clk) rf_ab <= rf_ab_c;   // valid from the clock after en: op_ld (ph1), S1, S2 and en are >= 2 pulses apart
  // DB port: as above, from g_rd[14:7]
  logic sel_db_rxd; assign sel_db_rxd = g_rd[7];   // db*<-(rxdl) dbd<-(rxdl) dbe<-(rxdl)   [dec 102 124 129]
  logic sel_db_pc; assign sel_db_pc = g_rd[8];   // db<-(pc)   [dec 108]
  logic sel_db_rx; assign sel_db_rx = g_rd[9];   // db<-(rx) db<-(rxh) db<-(rxl) dbe<-(rxl)   [dec 109 111 112 130]
  logic sel_db_rxa; assign sel_db_rxa = g_rd[10];   // db<-(rxa)   [dec 110]
  logic sel_db_ry; assign sel_db_ry = g_rd[11];   // db<-(ry) db<-(ryh) db<-(ryl)   [dec 113 115 116]
  logic sel_db_rya; assign sel_db_rya = g_rd[12];   // db<-(rya)   [dec 114]
  logic sel_db_sp; assign sel_db_sp = g_rd[13];   // db<-(sp)   [dec 117]
  logic sel_db_ryd; assign sel_db_ryd = g_rd[14];   // dbd<-(rydl)   [dec 125]
`ifdef VERILATOR
  // SAFETY NET: the OR of the bits of dec, compared at every clock
  logic sel_db_rxd_v, sel_db_pc_v, sel_db_rx_v, sel_db_rxa_v, sel_db_ry_v, sel_db_rya_v, sel_db_sp_v, sel_db_ryd_v;
  assign sel_db_rxd_v = (dec[102]) | (dec[124]) | (dec[129]);
  assign sel_db_pc_v = (dec[108]);
  assign sel_db_rx_v = (dec[109]) | (dec[111]) | (dec[112]) | (dec[130]);
  assign sel_db_rxa_v = (dec[110]);
  assign sel_db_ry_v = (dec[113]) | (dec[115]) | (dec[116]);
  assign sel_db_rya_v = (dec[114]);
  assign sel_db_sp_v = (dec[117]);
  assign sel_db_ryd_v = (dec[125]);
  always @(posedge clk) if (rst_n && {sel_db_rxd, sel_db_pc, sel_db_rx, sel_db_rxa, sel_db_ry, sel_db_rya, sel_db_sp, sel_db_ryd} !== {sel_db_rxd_v, sel_db_pc_v, sel_db_rx_v, sel_db_rxa_v, sel_db_ry_v, sel_db_rya_v, sel_db_sp_v, sel_db_ryd_v}) begin
    $display("g_rd MISMATCH db t=%0t g=%b dec=%b", $time, {sel_db_rxd, sel_db_pc, sel_db_rx, sel_db_rxa, sel_db_ry, sel_db_rya, sel_db_sp, sel_db_ryd}, {sel_db_rxd_v, sel_db_pc_v, sel_db_rx_v, sel_db_rxa_v, sel_db_ry_v, sel_db_rya_v, sel_db_sp_v, sel_db_ryd_v}); $fatal(1);
  end
`endif
  // DB port, same as above
  logic x1_db_K18; assign x1_db_K18 = (sel_db_rxd & e_dt & ~rz_sel_on) | (sel_db_rx & e_dt & ~rz_sel_on) | (sel_db_rxa & e_dt) | (sel_db_ry & y_dt) | (sel_db_rya & y_dt) | (sel_db_ryd & y_dt);
  logic x1_db_K15; assign x1_db_K15 = (sel_db_rxd & e_us & ~rz_sel_on) | (sel_db_rx & e_us & ~rz_sel_on) | (sel_db_rxa & e_us);
  logic x1_db_K17; assign x1_db_K17 = (sel_db_pc) | (sel_db_ry & y_pc) | (sel_db_rya & y_pc);
  logic x1_db_SP; assign x1_db_SP = (sel_db_rxd & e_sp & ~rz_sel_on) | (sel_db_rx & e_sp & ~rz_sel_on) | (sel_db_rx & ~x1_eany & ~rz_sel_on & rx_is_a & x7_q) | (sel_db_rxa & e_sp) | (sel_db_rxa & ~x1_eany & x7_q) | (sel_db_ry & y_df & ry_is_a & y7_q) | (sel_db_rya & y_df & y7_q) | (sel_db_sp);
  logic x1_db_RP; assign x1_db_RP = (sel_db_rxd & e_rp & ~rz_sel_on) | (sel_db_rx & e_rp & ~rz_sel_on) | (sel_db_rxa & e_rp);
  logic x1_db_RZ; assign x1_db_RZ = (sel_db_rxd & rz_sel_on & x1_eany) | (sel_db_rx & rz_sel_on);
  logic x1_db_XV; assign x1_db_XV = (sel_db_rx & ~x1_eany & ~rz_sel_on & rx_is_a & ~x7_q) | (sel_db_rxa & ~x1_eany & ~x7_q);
  logic x1_db_RXD; assign x1_db_RXD = (sel_db_rxd & ~x1_eany) | (sel_db_rx & ~x1_eany & ~rz_sel_on & ~rx_is_a);
  logic x1_db_YV; assign x1_db_YV = (sel_db_ry & y_df & ry_is_a & ~y7_q) | (sel_db_rya & y_df & ~y7_q);
  logic x1_db_RYLO; assign x1_db_RYLO = (sel_db_ry & y_df & ~ry_is_a) | (sel_db_ryd & ~ps_dt);
  logic [4:0] x1_ri_db;
  assign x1_ri_db[0] = x1_db_K15 | x1_db_K17 | x1_db_SP | (x1_db_RP & rp_reg[0]) | (x1_db_RZ & irc[12]) | (x1_db_XV & xv_q[0]) | (x1_db_RXD & rxd_def[0]) | (x1_db_YV & yv_q[0]) | (x1_db_RYLO & rylo_q[0]);
  assign x1_ri_db[1] = x1_db_K18 | x1_db_K15 | x1_db_SP | (x1_db_RP & rp_reg[1]) | (x1_db_RZ & irc[13]) | (x1_db_XV & xv_q[1]) | (x1_db_RXD & rxd_def[1]) | (x1_db_YV & yv_q[1]) | (x1_db_RYLO & rylo_q[1]);
  assign x1_ri_db[2] = x1_db_K15 | x1_db_SP | (x1_db_RP & rp_reg[2]) | (x1_db_RZ & irc[14]) | (x1_db_XV & xv_q[2]) | (x1_db_RXD & rxd_def[2]) | (x1_db_YV & yv_q[2]) | (x1_db_RYLO & rylo_q[2]);
  assign x1_ri_db[3] = x1_db_K15 | x1_db_SP | (x1_db_RP & rp_reg[3]) | (x1_db_RZ & irc[15]) | (x1_db_XV & xv_q[3]) | (x1_db_RXD & rxd_def[3]) | (x1_db_YV & yv_q[3]) | (x1_db_RYLO & rylo_q[3]);
  assign x1_ri_db[4] = x1_db_K18 | x1_db_K17 | (x1_db_SP & sup) | (x1_db_RP & x1_rp15 & sup) | (x1_db_RZ & x1_z15 & sup) | (x1_db_XV & xv_q[4]) | (x1_db_RXD & rxd_def[4]) | (x1_db_YV & yv_q[4]) | (x1_db_RYLO & rylo_q[4]);
  logic [4:0] ri_db_f, ri_db_f_v; assign ri_db_f_v = ({5{sel_db_rxd}} & s_rxd) | ({5{sel_db_pc}} & s_pc) | ({5{sel_db_rx}} & s_rx) | ({5{sel_db_rxa}} & s_rxa) | ({5{sel_db_ry}} & s_ry) | ({5{sel_db_rya}} & s_rya) | ({5{sel_db_sp}} & s_sp) | ({5{sel_db_ryd}} & s_ryd);
  assign ri_db_f = x1_ri_db;
`ifdef VERILATOR
  always @(posedge clk) if (rst_n && ri_db_f !== ri_db_f_v) begin
    $display("read index MISMATCH db t=%0t %0d/%0d", $time, ri_db_f, ri_db_f_v); $fatal(1);
  end
`endif
  logic h_db_db, h_pc_db, lo_db;
  // the forwarding does not wait for one comparison per source and then the OR with the selections:
  // the selections are exclusive (the design already relies on this for ri_db_f, the MLAB
  // address), so `OR_X (sel_X & pred(s_X))` = `sel_any & pred(ri_db_f)`, with fewer levels than the per-source form.
  // No port can be 17 except the PC ones, and none can be < 8 except the data registers:
  // the predicates on the selected index are exact even for the ports that had them at 0.
  logic sel_any_db; assign sel_any_db = sel_db_rxd | sel_db_pc | sel_db_rx | sel_db_rxa | sel_db_ry | sel_db_rya | sel_db_sp | sel_db_ryd;
  assign h_db_db = sel_any_db & q_db_v & (q_db_i == ri_db_f);
  assign h_pc_db = sel_any_db & (ri_db_f == 5'd17);
  assign lo_db   = sel_any_db & (ri_db_f < 5'd8);
`ifdef VERILATOR
  // safety net: the per-source form, compared at every clock
  logic h_db_db_v, h_pc_db_v, lo_db_v;
  assign h_db_db_v = (sel_db_rxd & hdb_rxd) | (sel_db_pc & hdb_pc) | (sel_db_rx & hdb_rx) | (sel_db_rxa & hdb_rxa) | (sel_db_ry & hdb_ry) | (sel_db_rya & hdb_rya) | (sel_db_sp & hdb_sp) | (sel_db_ryd & hdb_ryd);
  assign h_pc_db_v = (sel_db_rxd & hpc_rxd) | (sel_db_pc & hpc_pc) | (sel_db_rx & hpc_rx) | (sel_db_rxa & hpc_rxa) | (sel_db_ry & hpc_ry) | (sel_db_rya & hpc_rya) | (sel_db_sp & hpc_sp) | (sel_db_ryd & hpc_ryd);
  assign lo_db_v = (sel_db_rxd & lo_rxd) | (sel_db_pc & lo_pc) | (sel_db_rx & lo_rx) | (sel_db_rxa & lo_rxa) | (sel_db_ry & lo_ry) | (sel_db_rya & lo_rya) | (sel_db_sp & lo_sp) | (sel_db_ryd & lo_ryd);
  always @(posedge clk) if (rst_n && (h_db_db !== h_db_db_v || h_pc_db !== h_pc_db_v || lo_db !== lo_db_v)) begin
    $display("forwarding MISMATCH db t=%0t h_db %b/%b h_pc %b/%b lo %b/%b", $time, h_db_db, h_db_db_v, h_pc_db, h_pc_db_v, lo_db, lo_db_v); $fatal(1);
  end
`endif
  logic [31:0] rr_db; assign rr_db = {rb_h[ri_db_f], rb_m[ri_db_f], rb_l[ri_db_f]};
  logic [31:0] rf_db_c;   // combinational read with forwarding; rf_db = free-running register (lag 1 clock)
  logic f_db_h, f_db_m, f_db_l;   // exclusive forwarding qualifiers per slice (not read)
  assign f_db_h = h_db_db & q_db_h;  assign f_db_m = h_db_db & q_db_lm;  assign f_db_l = h_db_db & q_db_l;   // kept, not read: the final LUT uses the slice qualifiers directly
  // the slice qualifier (flop) enters the final LUT directly: 6 inputs, one level less after the index than
  // going through f_*.
  assign rf_db_c[31:16] = (h_db_db & q_db_h)  ? q_db_wh       : h_pc_db ? pc_sh[31:16] : rr_db[31:16];   // the PC (index 17) lives in pc_sh
  assign rf_db_c[15:8]  = (h_db_db & q_db_lm) ? q_db_wl[15:8] : h_pc_db ? pc_sh[15:8]  : rr_db[15:8];
  assign rf_db_c[7:0]   = (h_db_db & q_db_l)  ? q_db_wl[7:0]  : h_pc_db ? pc_sh[7:0]   : rr_db[7:0];
`ifdef VERILATOR
  // SAFETY NET: the if/else form, compared at every clock
  logic [31:0] rf_db_c_v;
  always_comb begin
    rf_db_c_v = h_pc_db ? pc_sh : rr_db;
    if (h_db_db) begin if (q_db_h) rf_db_c_v[31:16] = q_db_wh; if (q_db_l) begin if (q_db_b && lo_db) rf_db_c_v[7:0] = q_db_wl[7:0]; else rf_db_c_v[15:0] = q_db_wl; end end
  end
  always @(posedge clk) if (rst_n && rf_db_c !== rf_db_c_v) begin
    $display("rf_c MISMATCH db t=%0t %h / %h", $time, rf_db_c, rf_db_c_v); $fatal(1);
  end
`endif
  always_ff @(posedge clk) rf_db <= rf_db_c;   // valid from the clock after en: op_ld (ph1), S1, S2 and en are >= 2 pulses apart
  logic rf_db_sp_c; assign rf_db_sp_c = (sel_db_rxd & eqsp_rxd) | (sel_db_pc & eqsp_pc) | (sel_db_rx & eqsp_rx) | (sel_db_rxa & eqsp_rxa) | (sel_db_ry & eqsp_ry) | (sel_db_rya & eqsp_rya) | (sel_db_sp & eqsp_sp) | (sel_db_ryd & eqsp_ryd);
  always_ff @(posedge clk) rf_db_sp <= rf_db_sp_c;   // consumed by the AU at op_ld (ph1)
  assign pc_q = fwd(5'd17, pc_sh);
  generate if (DBG) begin : g_dbg
    assign dbg_data_d = fwd(dbg_idx_d, (dbg_idx_d == 5'd17) ? pc_sh : {ra_h[dbg_idx_d], ra_m[dbg_idx_d], ra_l[dbg_idx_d]});
    assign dbg_data_a = fwd(dbg_idx_a, (dbg_idx_a == 5'd17) ? pc_sh : {ra_h[dbg_idx_a], ra_m[dbg_idx_a], ra_l[dbg_idx_a]});
  end else begin : g_nodbg
    assign dbg_data_d = '0; assign dbg_data_a = '0;
  end endgenerate
  logic we_ab_h, we_ab_l; logic [15:0] wd_ab_h, wd_ab_l; logic [4:0] wi_ab;
  assign wi_ab = ({5{((dec[173 +: 2] == 2'd1) | (dec[173 +: 2] == 2'd2) | (dec[179 +: 2] == 2'd1) | (dec[181 +: 2] == 2'd1))}} & rx_i) | ({5{((dec[175 +: 2] == 2'd1))}} & rxa_i) | ({5{((dec[177 +: 2] == 2'd1) | (dec[177 +: 2] == 2'd2))}} & rxd_i) | ({5{((dec[184 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd2) | (dec[190 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd2) | (dec[192 +: 2] == 2'd1) | (dec[192 +: 2] == 2'd2))}} & ry_i) | ({5{((dec[186 +: 2] == 2'd1))}} & rya_i) | ({5{((dec[188 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd2))}} & ryd_i) | ({5{((dec[195 +: 1] == 1'd1) | (dec[196 +: 1] == 1'd1))}} & sp_i);
  logic we_ab_l_v, we_ab_h_v, we_db_l_v, we_db_h_v;   // the form with the flags (WE_DEC_ONLY = 0, and the safety net)
  assign we_ab_l_v = (((dec[173 +: 2] == 2'd1) | (dec[173 +: 2] == 2'd2) | (dec[175 +: 2] == 2'd1) | (dec[177 +: 2] == 2'd2) | (dec[181 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd2) | (dec[186 +: 2] == 2'd1) | (dec[192 +: 2] == 2'd1) | (dec[192 +: 2] == 2'd2) | (dec[196 +: 1] == 1'd1)) & (ab_l_drv | ab_d_drv)) | (((dec[177 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd2)) & (ab_d_drv | ab_l_drv));
  assign we_ab_h_v = (((dec[173 +: 2] == 2'd1) | (dec[173 +: 2] == 2'd2) | (dec[175 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd2) | (dec[186 +: 2] == 2'd1)) & (ab_h_drv)) | (((dec[179 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd2) | (dec[195 +: 1] == 1'd1)) & (ab_h_drv | ab_l_drv | ab_d_drv));
  assign we_db_l_v = (((dec[173 +: 2] == 2'd3) | (dec[175 +: 2] == 2'd2) | (dec[181 +: 2] == 2'd2) | (dec[184 +: 2] == 2'd3) | (dec[186 +: 2] == 2'd2) | (dec[192 +: 2] == 2'd3) | (dec[194 +: 1] == 1'd1)) & (db_l_drv | db_d_drv)) | (((dec[177 +: 2] == 2'd3) | (dec[188 +: 2] == 2'd3)) & (db_d_drv | db_l_drv));
  assign we_db_h_v = (((dec[173 +: 2] == 2'd3) | (dec[175 +: 2] == 2'd2) | (dec[184 +: 2] == 2'd3) | (dec[186 +: 2] == 2'd2) | (dec[194 +: 1] == 1'd1)) & (db_h_drv)) | (((dec[179 +: 2] == 2'd2) | (dec[190 +: 2] == 2'd3)) & (db_h_drv | db_l_drv | db_d_drv));
  // the write ENABLES depend on `dec` alone: the factor `& (…_drv)` is redundant.
  // Census over the 547 nanowords: in ALL the words that write a register from AB/DB (143 in eight groups)
  // the requested bus section is driven. Without the flag, two cells and two routes leave the start of the chain
  // dec -> sel_ab_d_2 -> ab_d_drv -> we_ab_h -> w_i -> (w_i == 17) -> pc_sh enable. The data muxes wd_*_v keep
  // the flags: they choose the section, not the enable.
`ifdef VERILATOR
  generate if (WE_DEC_ONLY) begin : g_we_dec_only_chk
  always @(posedge clk) if (rst_n && en && (we_ab_l !== we_ab_l_v || we_ab_h !== we_ab_h_v || we_db_l !== we_db_l_v || we_db_h !== we_db_h_v)) begin
    $display("write enable MISMATCH t=%0t ab_l %b/%b ab_h %b/%b db_l %b/%b db_h %b/%b", $time, we_ab_l, we_ab_l_v, we_ab_h, we_ab_h_v, we_db_l, we_db_l_v, we_db_h, we_db_h_v); $fatal(1);
  end
  end endgenerate
`endif
  logic we_ab_l_c; assign we_ab_l_c = ((((dec[173 +: 2] == 2'd1) | (dec[173 +: 2] == 2'd2) | (dec[175 +: 2] == 2'd1) | (dec[177 +: 2] == 2'd2) | (dec[181 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd2) | (dec[186 +: 2] == 2'd1) | (dec[192 +: 2] == 2'd1) | (dec[192 +: 2] == 2'd2) | (dec[196 +: 1] == 1'd1))) | (((dec[177 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd2))));
  assign we_ab_l = WE_FLOP ? we_ab_l_i : (WE_DEC_ONLY ? we_ab_l_c : we_ab_l_v);   // from the decoder flop with WE_FLOP
  logic [15:0] wd_ab_l_v, wd_ab_h_v, wd_db_l_v, wd_db_h_v;   // the form with the flags (WD_NARROW = 0, and the safety net)
  assign wd_ab_l_v = ((dec[177 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd2)) ? (ab_d_drv ? ab_d : ab_l) : ((ab_l_drv ? ab_l : ab_d));
  logic we_ab_h_c; assign we_ab_h_c = ((((dec[173 +: 2] == 2'd1) | (dec[173 +: 2] == 2'd2) | (dec[175 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd2) | (dec[186 +: 2] == 2'd1))) | (((dec[179 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd2) | (dec[195 +: 1] == 1'd1))));
  assign we_ab_h = WE_FLOP ? we_ab_h_i : (WE_DEC_ONLY ? we_ab_h_c : we_ab_h_v);   // from the decoder flop with WE_FLOP
  assign wd_ab_h_v = ((dec[179 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd2) | (dec[195 +: 1] == 1'd1)) ? (ab_h_drv ? ab_h : (ab_l_drv ? ab_l : ab_d)) : (ab_h);
  logic we_db_h, we_db_l; logic [15:0] wd_db_h, wd_db_l; logic [4:0] wi_db;
  assign wi_db = ({5{((dec[173 +: 2] == 2'd3) | (dec[179 +: 2] == 2'd2) | (dec[181 +: 2] == 2'd2))}} & rx_i) | ({5{((dec[175 +: 2] == 2'd2))}} & rxa_i) | ({5{((dec[177 +: 2] == 2'd3))}} & rxd_i) | ({5{((dec[184 +: 2] == 2'd3) | (dec[190 +: 2] == 2'd3) | (dec[192 +: 2] == 2'd3))}} & ry_i) | ({5{((dec[186 +: 2] == 2'd2))}} & rya_i) | ({5{((dec[188 +: 2] == 2'd3))}} & ryd_i) | ({5{((dec[194 +: 1] == 1'd1))}} & sp_i);
  logic we_db_l_c; assign we_db_l_c = ((((dec[173 +: 2] == 2'd3) | (dec[175 +: 2] == 2'd2) | (dec[181 +: 2] == 2'd2) | (dec[184 +: 2] == 2'd3) | (dec[186 +: 2] == 2'd2) | (dec[192 +: 2] == 2'd3) | (dec[194 +: 1] == 1'd1))) | (((dec[177 +: 2] == 2'd3) | (dec[188 +: 2] == 2'd3))));
  assign we_db_l = WE_FLOP ? we_db_l_i : (WE_DEC_ONLY ? we_db_l_c : we_db_l_v);   // from the decoder flop with WE_FLOP
  assign wd_db_l_v = ((dec[177 +: 2] == 2'd3) | (dec[188 +: 2] == 2'd3)) ? (db_d_drv ? db_d : db_l) : ((db_l_drv ? db_l : db_d));
  logic we_db_h_c; assign we_db_h_c = ((((dec[173 +: 2] == 2'd3) | (dec[175 +: 2] == 2'd2) | (dec[184 +: 2] == 2'd3) | (dec[186 +: 2] == 2'd2) | (dec[194 +: 1] == 1'd1))) | (((dec[179 +: 2] == 2'd2) | (dec[190 +: 2] == 2'd3))));
  assign we_db_h = WE_FLOP ? we_db_h_i : (WE_DEC_ONLY ? we_db_h_c : we_db_h_v);   // from the decoder flop with WE_FLOP
`ifdef VERILATOR
  // SAFETY NET: the decoder flops must equal the combinational form at every clock
  generate if (WE_FLOP) begin : g_we_flop_chk
  always @(posedge clk) if (rst_n && (we_ab_l_i !== we_ab_l_c || we_ab_h_i !== we_ab_h_c || we_db_l_i !== we_db_l_c || we_db_h_i !== we_db_h_c)) begin
    $display("write enable flop MISMATCH t=%0t ab_l %b/%b ab_h %b/%b db_l %b/%b db_h %b/%b", $time, we_ab_l_i, we_ab_l_c, we_ab_h_i, we_ab_h_c, we_db_l_i, we_db_l_c, we_db_h_i, we_db_h_c); $fatal(1);
  end
  end endgenerate
`endif
  assign wd_db_h_v = ((dec[179 +: 2] == 2'd2) | (dec[190 +: 2] == 2'd3)) ? (db_h_drv ? db_h : (db_l_drv ? db_l : db_d)) : (db_h);
  // the register file write DATA arrives on the narrow fabric wires (ab_wl/ab_wh/db_wl/db_wh): flat AND-OR
  // of only the sources that win in some word (census in gen_bus_sec.py, identity assert on the atoms), without the
  // section mux downstream of the bus and without the drv flags. Same function, same en edge. Safety net below.
  assign wd_ab_l = WD_NARROW ? ab_wl : wd_ab_l_v;
  assign wd_ab_h = WD_NARROW ? ab_wh : wd_ab_h_v;
  assign wd_db_l = WD_NARROW ? db_wl : wd_db_l_v;
  assign wd_db_h = WD_NARROW ? db_wh : wd_db_h_v;
`ifdef VERILATOR
  // SAFETY NET: at every en the data on the narrow wires must equal that of the chain form (wd_*_v) when the write is active
  generate if (WD_NARROW) begin : g_wd_narrow_chk
  always @(posedge clk) if (rst_n && en && ((we_ab_l && wd_ab_l !== wd_ab_l_v) || (we_ab_h && wd_ab_h !== wd_ab_h_v) || (we_db_l && wd_db_l !== wd_db_l_v) || (we_db_h && wd_db_h !== wd_db_h_v))) begin
    $display("write data MISMATCH t=%0t ab_l %04x/%04x (we %b) ab_h %04x/%04x (we %b) db_l %04x/%04x (we %b) db_h %04x/%04x (we %b)", $time, wd_ab_l, wd_ab_l_v, we_ab_l, wd_ab_h, wd_ab_h_v, we_ab_h, wd_db_l, wd_db_l_v, we_db_l, wd_db_h, wd_db_h_v, we_db_h); $fatal(1);
  end
  end endgenerate
`endif
  logic we_pc_h, we_pc_l; logic [15:0] wd_pc_h, wd_pc_l;
  // the form with the bus sections, the fallbacks and the drv flags (PC_DEC_ONLY = 0, and the safety net)
  logic we_pc_h_v, we_pc_l_v; logic [15:0] wd_pc_h_v, wd_pc_l_v;
  assign we_pc_l_v = (((dec[153 +: 2] == 2'd1) | (dec[156 +: 1] == 1'd1)) & (ab_l_drv | ab_d_drv)) | ((dec[153 +: 2] == 2'd2)) | (((dec[153 +: 2] == 2'd3)) & (db_l_drv | db_d_drv));
  assign wd_pc_l_v = ((dec[153 +: 2] == 2'd2)) ? au[15:0] : (((dec[153 +: 2] == 2'd3)) ? (db_l_drv ? db_l : db_d) : ((ab_l_drv ? ab_l : ab_d)));
  assign we_pc_h_v = (((dec[153 +: 2] == 2'd1)) & (ab_h_drv)) | ((dec[153 +: 2] == 2'd2)) | (((dec[153 +: 2] == 2'd3)) & (db_h_drv)) | (((dec[155 +: 1] == 1'd1)) & (db_h_drv | db_l_drv | db_d_drv));
  assign wd_pc_h_v = ((dec[153 +: 2] == 2'd1)) ? ab_h : (((dec[153 +: 2] == 2'd2)) ? au[31:16] : (((dec[153 +: 2] == 2'd3)) ? db_h : ((db_h_drv ? db_h : (db_l_drv ? db_l : db_d)))));
  // census over the 547 nanowords: 163 words have a PC port atom and the data is ALWAYS
  // `au` (pc<-au 111; pc<-db 45, all with db<-au; pc<-ab with ab<-au 4: 217 234 3a3 3a7) or `at` (pc<-ab with ab<-(at):
  // 088 bser6; pch<-dbh with dbh<-(ath): 020 mulm6; pcl<-ab with ab<-(atl): 0e0 mulm4), never another fabric source; and the
  // requested section is always driven (0 words without a source), so the `& (…_drv)` factors of the enables are always
  // true (as for the register write enables). The data becomes a two-way at/au mux with the selector from dec alone, QUALIFIED with pc<-ab because
  // 270 jsaw1 has pc<-au with ab<-(at) active. The PC port cone starts from the `at`/`au`/dec flops, not from the fabric.
  logic pc_at_h, pc_at_l;   // the PC port data is `at` (otherwise `au`)
  assign pc_at_h = (dec[155 +: 1] == 1'd1) | ((dec[153 +: 2] == 2'd1) & dec[9]);   // pch<-dbh | pc<-ab & ab<-(at)
  assign pc_at_l = (dec[156 +: 1] == 1'd1) | ((dec[153 +: 2] == 2'd1) & dec[9]);   // pcl<-ab  | pc<-ab & ab<-(at)
  assign we_pc_h = PC_DEC_ONLY ? (dec[153] | dec[154] | dec[155]) : we_pc_h_v;   // pc<-ab/au/db | pch<-dbh
  assign we_pc_l = PC_DEC_ONLY ? (dec[153] | dec[154] | dec[156]) : we_pc_l_v;   // pc<-ab/au/db | pcl<-ab
  assign wd_pc_h = PC_DEC_ONLY ? (pc_at_h ? at[31:16] : au[31:16]) : wd_pc_h_v;
  assign wd_pc_l = PC_DEC_ONLY ? (pc_at_l ? at[15:0] : au[15:0]) : wd_pc_l_v;
`ifdef VERILATOR
  // SAFETY NET, simulation only: at every `en` the enable and the data (when the enable is active) of the census form must
  // equal EXACTLY those of the form with the flags. With PC_DEC_ONLY = 0 it is not instantiated.
  generate if (PC_DEC_ONLY) begin : g_pc_port_chk
  always @(posedge clk) if (rst_n && en && (we_pc_h !== we_pc_h_v || we_pc_l !== we_pc_l_v || (we_pc_h && wd_pc_h !== wd_pc_h_v) || (we_pc_l && wd_pc_l !== wd_pc_l_v))) begin
    $display("PC port MISMATCH t=%0t we_h %b/%b wd_h %h/%h we_l %b/%b wd_l %h/%h", $time, we_pc_h, we_pc_h_v, wd_pc_h, wd_pc_h_v, we_pc_l, we_pc_l_v, wd_pc_l, wd_pc_l_v); $fatal(1);
  end
  end endgenerate
`endif
  // write: one physical port, shared by the microcode AB and DB ports through the same mux; the read ports forward
  // the pending write. The PC port writes pc_sh directly at en.
  // The AB port writes the MLABs directly at the en edge; only the DB port is queued (applied at en+1).
  logic q_db_v, q_db_h, q_db_l, q_db_b; logic [15:0] q_db_wh, q_db_wl; logic [4:0] q_db_i;
  // write indices registered free-running (used only at en, last clock of the word; dec is valid from the first)
  logic [4:0] wi_ab_q, wi_db_q; always_ff @(posedge clk) begin wi_ab_q <= wi_ab; wi_db_q <= wi_db; end
  // register file write ONE LUT from en: the parts that do not depend on en are precomputed (keep) and the final mux
  // "(en & AB) ? AB : DB" fits in a single LUT per wire (5 inputs: en_rf, we_ab_h, we_ab_l, AB, DB). Same function
  // as the if/else form (written below as the reference of the safety net).
  (* keep *) logic ab_any, ab_m, db_h_v, db_l_v, db_m;
  assign ab_any = we_ab_h | we_ab_l;
  assign ab_m   = we_ab_l & ~(byte_op & (wi_ab_q < 5'd8));
  assign db_h_v = q_db_v & q_db_h;
  assign db_l_v = q_db_v & q_db_l;
  assign db_m   = q_db_v & q_db_l & ~(q_db_b & (q_db_i < 5'd8));
  // no shared node `sel_ab = en_rf & ab_any` (synthesis would keep it as a separate LUT, one more level from en):
  // each wire is a 4-5 input LUT with en_rf and ab_any direct. Same function: with en_rf=1 and ab_any=0 the
  // we_ab_* and ab_m are 0, so the DB branch is the right one; with en_rf=0 the DB branch; with both at 1 the AB branch.
  logic w_h, w_l, w_m; logic [15:0] w_dh, w_dl; logic [4:0] w_i;
  assign w_h  = en_rf ? (ab_any ? we_ab_h : db_h_v) : db_h_v;
  assign w_l  = en_rf ? (ab_any ? we_ab_l : db_l_v) : db_l_v;
  assign w_m  = en_rf ? (ab_any ? ab_m    : db_m)   : db_m;
  assign w_dh = en_rf ? (ab_any ? wd_ab_h : q_db_wh) : q_db_wh;
  assign w_dl = en_rf ? (ab_any ? wd_ab_l : q_db_wl) : q_db_wl;
  assign w_i  = en_rf ? (ab_any ? wi_ab_q : q_db_i)  : q_db_i;
`ifdef VERILATOR
  // SAFETY NET: at every clock, enable, data and index must equal those of the if/else form (with en)
  logic o_h, o_l, o_b; logic [15:0] o_dh, o_dl; logic [4:0] o_i;
  always_comb begin
    if (en && (we_ab_h | we_ab_l)) begin o_h = we_ab_h; o_l = we_ab_l; o_dh = wd_ab_h; o_dl = wd_ab_l; o_i = wi_ab_q; o_b = byte_op; end
    else if (q_db_v) begin o_h = q_db_h; o_l = q_db_l; o_dh = q_db_wh; o_dl = q_db_wl; o_i = q_db_i; o_b = q_db_b; end
    else             begin o_h = 1'b0; o_l = 1'b0; o_dh = q_db_wh; o_dl = q_db_wl; o_i = q_db_i; o_b = q_db_b; end
  end
  always @(posedge clk) if (rst_n && (w_h !== o_h || w_l !== o_l || w_m !== (o_l & ~(o_b & (o_i < 5'd8))) ||
      ((o_h | o_l) && (w_i !== o_i)) || (o_h && w_dh !== o_dh) || (o_l && w_dl !== o_dl))) begin
    $display("write mux MISMATCH t=%0t w_h %b/%b w_l %b/%b w_m %b w_i %h/%h", $time, w_h, o_h, w_l, o_l, w_m, w_i, o_i); $fatal(1);
  end
`endif
  always_ff @(posedge clk) begin
    if (!rst_n) begin q_db_v <= 1'b0; q_db_h <= 1'b0; q_db_l <= 1'b0; q_db_b <= 1'b0; q_db_wh <= '0; q_db_wl <= '0; q_db_i <= '0; end
    else if (en) begin
      q_db_v <= we_db_h | we_db_l; q_db_h <= we_db_h; q_db_l <= we_db_l; q_db_wh <= wd_db_h; q_db_wl <= wd_db_l; q_db_i <= wi_db_q; q_db_b <= byte_op;
    end else if (q_db_v) q_db_v <= 1'b0;
  end
  // write: one per clock (w_*); byte write only on the data registers D0-D7 (w_m = 0): the 15:8 column is kept
  always_ff @(posedge clk) begin
    if (w_h) begin ra_h[w_i] <= w_dh; rb_h[w_i] <= w_dh; end
    if (w_l) begin ra_l[w_i] <= w_dl[7:0]; rb_l[w_i] <= w_dl[7:0]; end
    if (w_m) begin ra_m[w_i] <= w_dl[15:8]; rb_m[w_i] <= w_dl[15:8]; end   // w_m = w_l without the byte write on the data registers
  end
  // the GENERIC write of the PC (index 17 from the w_* mux): in the 68000 the PC-relative modes (mode 111) are source only,
  // no instruction writes the EA register in mode 111, -(An)/(An)+ update only in modes 011/100, the exceptions write
  // via sp/dt/rp: no generic write ever has index 17 (flow proof over the 65,536 opcodes and the exception routines).
  // With PC_W17 = 0 the branch disappears and pc_sh has a single data input (wd_pc_*) and a single enable (en & we_pc_*),
  // which takes the comparison w_i == 17 off the path dec -> we_ab_h -> w_i -> pc_sh.
  logic w17_h, w17_l;
  assign w17_h = PC_W17 ? (w_h && w_i == 5'd17) : 1'b0;
  assign w17_l = PC_W17 ? (w_l && w_i == 5'd17) : 1'b0;
`ifdef VERILATOR
  // SAFETY NET: if a generic write ever had index 17, the core stops here
  generate if (!PC_W17) begin : g_pc_generic_wr_chk
  always @(posedge clk) if (rst_n && (w_h || w_l) && w_i == 5'd17) begin
    $display("PC_W17 MISMATCH t=%0t saw a generic write with index 17: w_h %b w_l %b", $time, w_h, w_l); $fatal(1);
  end
  end endgenerate
`endif
  always_ff @(posedge clk) begin
    if (!rst_n) pc_sh <= 32'd0;
    else begin
      if (w17_h) pc_sh[31:16] <= w_dh;
      if (w17_l) pc_sh[15:0]  <= w_dl;
      if (en && we_pc_h) pc_sh[31:16] <= wd_pc_h;   // direct PC port at en: the PC lives only here
      if (en && we_pc_l) pc_sh[15:0]  <= wd_pc_l;
    end
  end
endmodule
