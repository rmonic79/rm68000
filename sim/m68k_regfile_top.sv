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

// m68k_regfile_top.sv - testbench top of the register file in the CORE configuration: control store + decoder + bus section fabric +
// bus-side registers (AT for the PC port; AOB and DBIN as fabric sources) + MOVEM encoder + register file, connected as in
// m68k_cpu_si (register file: DBG = 1, PC_W17 = 0, WE_FLOP = 1 and the default parameters IR_REG, WE_DEC_ONLY, PC_DEC_ONLY, WD_NARROW = 1; encoder:
// IR_REG, RP_REG = 1). The read ports rf_ab/rf_db go back into the fabric as in the core. The tb drives the external sources of the fabric
// (ALU, ALUE, AU, DCR, FTU, EDB), the S bit, IR/IRD/IRC and byte_op, and observes the sections, the "driven" flags, the read ports, the encoder and
// the register state from the debug ports. ORG_OVR_LUT = 0 (core; 1 = testbench configuration, a0s override on any word).
// Maintained by hand: the retired docs/data/scripts/gen_regfile.py does not regenerate it.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_regfile_top #(parameter MEM_FILE = "m68k_urom.mem", parameter ORIGIN_FILE = "m68k_norigin.mem", parameter NANO_FILE = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",
                          parameter bit ORG_OVR_LUT = 1'b0) (
  input  logic clk, rst_n, en, input logic [9:0] a1, a2, a3, input logic a0_sub, input logic [9:0] a0, input logic a0s_sub, input logic [9:0] a0s, input logic [1:0] c1c0,
  output logic [9:0] upc, output logic [NDEC_W-1:0] dec, output logic [16:0] uw,
  input  logic [15:0] src_alu, src_alue, src_dcr, src_ftu, src_edb,
  input  logic [31:0] src_au,
  input  logic [15:0] ir, ird, irc,
  input  logic        sup, sup_next, byte_op,
  input  logic [4:0]  dbg_idx_d, dbg_idx_a,
  output logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,
  output logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,
  output logic [31:0] at, aob,
  output logic [31:0] rf_ab, rf_db, pc_q,
  output logic        rf_db_sp,
  output logic [3:0]  rp_reg, output logic mvm_end,
  output logic [31:0] dbg_data_d, dbg_data_a
);
  /* verilator lint_off UNUSEDSIGNAL */
  logic [NROM_W-1:0] nw, nw_pre; logic [14:0] nwl_pre; logic [NROM_IDX_W-1:0] nidx; logic [9:0] uaddr;
  logic upc_is327, ir_load, fmt, bit4, c_ft_q, psws_p1; logic [1:0] ty, fc; logic [4:0] cbc;
  logic [26:0] g_w; logic [24:0] gq_w; logic [5:0] gd_w; logic we_ab_h_q, we_ab_l_q, we_db_h_q, we_db_l_q;
  logic [NDEC_W-1:0] dec_p, dec_pm, dec_po; logic ovr_p;
  logic [15:0] ab_ld, ab_dob, ab_wl, ab_wh, db_wl, db_wh; logic ab_dob_drv;
  logic [15:0] dob, dbin, dob_n, dbin_n; logic [31:0] aob_n; logic [3:0] rp_next;
  /* verilator lint_on UNUSEDSIGNAL */
  m68k_nstore_dec #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .ORG_OVR_LUT(ORG_OVR_LUT)) u_cs (.clk(clk), .rst_n(rst_n), .en(en), .a1(a1), .a2(a2), .a3(a3),
    .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s), .c1c0(c1c0), .upc(upc), .upc_is327(upc_is327), .uw(uw), .nw(nw), .nwl_pre(nwl_pre), .nw_pre(nw_pre), .dec(dec),
    .g(g_w), .gq(gq_w), .gd(gd_w), .c_ft_q(c_ft_q), .we_ab_h_q(we_ab_h_q), .we_ab_l_q(we_ab_l_q), .we_db_h_q(we_db_h_q), .we_db_l_q(we_db_l_q),
    .dec_p_o(dec_p), .dec_pm_o(dec_pm), .dec_po_o(dec_po), .ovr_p_o(ovr_p), .psws_p1(psws_p1), .nidx(nidx), .uaddr(uaddr),
    .ir_load(ir_load), .fmt(fmt), .bit4(bit4), .ty(ty), .fc(fc), .cbc(cbc));
  // bus section fabric, with the same sources as the core: AOB, AT and DBIN from the bus-side registers, the read ports from the register file
  m68k_bus_sec u_bus (.dec(dec), .g(g_w[11:0]), .gq(gq_w), .gd(gd_w),
    .src_alu(src_alu), .src_alue(src_alue), .src_aob(aob), .src_at(at), .src_ath(at[31:16]), .src_atl(at[15:0]), .src_au(src_au),
    .src_dbin(dbin), .src_dcr(src_dcr), .src_ftu(src_ftu), .src_rf_ab(rf_ab), .src_rf_db(rf_db),
    .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d), .ab_ld(ab_ld),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob), .ab_dob_drv(ab_dob_drv),
    .ab_wl(ab_wl), .ab_wh(ab_wh), .db_wl(db_wl), .db_wh(db_wh));
  m68k_regs_bd u_regs (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob), .ab_dob_drv(ab_dob_drv),
    .src_au(src_au), .src_alu(src_alu), .src_edb(src_edb), .at(at), .aob(aob), .dob(dob), .dbin(dbin), .aob_n(aob_n), .dob_n(dob_n), .dbin_n(dbin_n));
  // MOVEM encoder: the mask enters with ftu<-ab from the L/D coupling of AB (ab_ld), as in the core
  m68k_mvm_si u_mvm (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ird(ird), .ir(ir), .mask_in(ab_ld), .rp_next(rp_next), .rp_reg(rp_reg), .mvm_end(mvm_end));
  m68k_regfile_si #(.DBG(1'b1), .PC_W17(1'b0), .WE_FLOP(1'b1)) u_rf (.clk(clk), .rst_n(rst_n), .en(en), .en_rf(en), .we_ab_h_i(we_ab_h_q), .we_ab_l_i(we_ab_l_q), .we_db_h_i(we_db_h_q), .we_db_l_i(we_db_l_q),
    .dec(dec), .dec_p(dec_p), .dec_pm(dec_pm), .dec_po(dec_po), .ovr(ovr_p), .rp_next(rp_next), .sup_next(sup_next), .ir(ir), .ird(ird), .sup(sup), .byte_op(byte_op),
    .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_wl(ab_wl), .ab_wh(ab_wh), .db_wl(db_wl), .db_wh(db_wh),
    .nw_pre(nw_pre), .g_rd(g_w[26:12]),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .au(src_au), .at(at), .rp_reg(rp_reg), .irc(irc), .dbg_idx_d(dbg_idx_d), .dbg_idx_a(dbg_idx_a), .dbg_data_d(dbg_data_d), .dbg_data_a(dbg_data_a),
    .rf_ab(rf_ab), .rf_db(rf_db), .rf_db_sp(rf_db_sp), .pc_q(pc_q));
endmodule
