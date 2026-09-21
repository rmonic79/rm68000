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

// m68k_bus_top.sv - control store + decoder + bus section fabric (testbench top). Core configuration: ORG_OVR_LUT = 0
// (1 = testbench configuration, a0s override on any word). It exposes the current microword `uw` (the testbench reads its format
// and TY to position itself by walking the microcode up to a dispatch).
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_bus_top #(parameter MEM_FILE = "m68k_urom.mem", parameter ORIGIN_FILE = "m68k_norigin.mem", parameter NANO_FILE = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",
                      parameter bit ORG_OVR_LUT = 1'b0) (
  input  logic clk, rst_n, en, input logic [9:0] a1, a2, a3, input logic a0_sub, input logic [9:0] a0, input logic a0s_sub, input logic [9:0] a0s, input logic [1:0] c1c0,
  output logic [9:0] upc, output logic [NDEC_W-1:0] dec, output logic [16:0] uw,
  input  logic [15:0] src_alu,
  input  logic [15:0] src_alue,
  input  logic [31:0] src_aob,
  input  logic [31:0] src_at,
  input  logic [15:0] src_ath,
  input  logic [15:0] src_atl,
  input  logic [31:0] src_au,
  input  logic [15:0] src_dbin,
  input  logic [15:0] src_dcr,
  input  logic [15:0] src_ftu,
  input  logic [31:0] src_rf_ab,
  input  logic [31:0] src_rf_db,
  output logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,
  output logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,
  output logic [15:0] ab_dob,
  output logic        ab_dob_drv
);
  /* verilator lint_off UNUSEDSIGNAL */
  logic [NROM_W-1:0] nw; logic [NROM_IDX_W-1:0] nidx; logic [9:0] uaddr; logic ir_load, fmt, bit4; logic [1:0] ty, fc; logic [4:0] cbc; logic [26:0] g_w; logic [24:0] gq_w; logic [5:0] gd_w;
  logic [15:0] ab_wl, ab_wh, db_wl, db_wh;   // narrow wires of the register file: here only for the safety net
  /* verilator lint_on UNUSEDSIGNAL */
  m68k_nstore_dec #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .ORG_OVR_LUT(ORG_OVR_LUT)) u_cs (.clk(clk), .rst_n(rst_n), .en(en), .a1(a1), .a2(a2), .a3(a3),
    .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s), .c1c0(c1c0), .upc(upc), .uw(uw), .nw(nw), .dec(dec), .nidx(nidx), .uaddr(uaddr),
    .ir_load(ir_load), .fmt(fmt), .bit4(bit4), .ty(ty), .fc(fc), .cbc(cbc), .g(g_w), .gq(gq_w), .gd(gd_w), .c_ft_q(), .we_ab_h_q(), .we_ab_l_q(), .we_db_h_q(), .we_db_l_q(),
    .nwl_pre(), .nw_pre());   // the early nanoword is not needed by this testbench
  m68k_bus_sec u_bus (.dec(dec), .g(g_w[11:0]), .gq(gq_w), .gd(gd_w), .src_alu(src_alu), .src_alue(src_alue), .src_aob(src_aob), .src_at(src_at), .src_ath(src_ath), .src_atl(src_atl), .src_au(src_au), .src_dbin(src_dbin), .src_dcr(src_dcr), .src_ftu(src_ftu), .src_rf_ab(src_rf_ab), .src_rf_db(src_rf_db),
    .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d), .ab_ld(),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob), .ab_dob_drv(ab_dob_drv),
    .ab_wl(ab_wl), .ab_wh(ab_wh), .db_wl(db_wl), .db_wh(db_wh));
`ifdef VERILATOR
  // SAFETY NET, simulation only: at every en, in every word that writes that half of the register file,
  // the narrow wire must equal EXACTLY the chain on the drv flags that the register file used (the tb drives the sources).
  always @(posedge clk) if (rst_n && en) begin
    if ((dec[173 +: 2] == 2'd1) | (dec[173 +: 2] == 2'd2) | (dec[175 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd2) | (dec[186 +: 2] == 2'd1)) if (ab_wh !== ab_h) begin $display("narrow wire MISMATCH ab_wh t=%0t upc=%03x wire %04x chain %04x", $time, upc, ab_wh, ab_h); $fatal(1); end
    if ((dec[179 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd1) | (dec[190 +: 2] == 2'd2) | (dec[195 +: 1] == 1'd1)) if (ab_wh !== (ab_h_drv ? ab_h : (ab_l_drv ? ab_l : ab_d))) begin $display("narrow wire MISMATCH ab_wh t=%0t upc=%03x wire %04x chain %04x", $time, upc, ab_wh, (ab_h_drv ? ab_h : (ab_l_drv ? ab_l : ab_d))); $fatal(1); end
    if ((dec[173 +: 2] == 2'd1) | (dec[173 +: 2] == 2'd2) | (dec[175 +: 2] == 2'd1) | (dec[177 +: 2] == 2'd2) | (dec[181 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd1) | (dec[184 +: 2] == 2'd2) | (dec[186 +: 2] == 2'd1) | (dec[192 +: 2] == 2'd1) | (dec[192 +: 2] == 2'd2) | (dec[196 +: 1] == 1'd1)) if (ab_wl !== (ab_l_drv ? ab_l : ab_d)) begin $display("narrow wire MISMATCH ab_wl t=%0t upc=%03x wire %04x chain %04x", $time, upc, ab_wl, (ab_l_drv ? ab_l : ab_d)); $fatal(1); end
    if ((dec[177 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd1) | (dec[188 +: 2] == 2'd2)) if (ab_wl !== (ab_d_drv ? ab_d : ab_l)) begin $display("narrow wire MISMATCH ab_wl t=%0t upc=%03x wire %04x chain %04x", $time, upc, ab_wl, (ab_d_drv ? ab_d : ab_l)); $fatal(1); end
    if ((dec[173 +: 2] == 2'd3) | (dec[175 +: 2] == 2'd2) | (dec[184 +: 2] == 2'd3) | (dec[186 +: 2] == 2'd2) | (dec[194 +: 1] == 1'd1)) if (db_wh !== db_h) begin $display("narrow wire MISMATCH db_wh t=%0t upc=%03x wire %04x chain %04x", $time, upc, db_wh, db_h); $fatal(1); end
    if ((dec[179 +: 2] == 2'd2) | (dec[190 +: 2] == 2'd3)) if (db_wh !== (db_h_drv ? db_h : (db_l_drv ? db_l : db_d))) begin $display("narrow wire MISMATCH db_wh t=%0t upc=%03x wire %04x chain %04x", $time, upc, db_wh, (db_h_drv ? db_h : (db_l_drv ? db_l : db_d))); $fatal(1); end
    if ((dec[173 +: 2] == 2'd3) | (dec[175 +: 2] == 2'd2) | (dec[181 +: 2] == 2'd2) | (dec[184 +: 2] == 2'd3) | (dec[186 +: 2] == 2'd2) | (dec[192 +: 2] == 2'd3) | (dec[194 +: 1] == 1'd1)) if (db_wl !== (db_l_drv ? db_l : db_d)) begin $display("narrow wire MISMATCH db_wl t=%0t upc=%03x wire %04x chain %04x", $time, upc, db_wl, (db_l_drv ? db_l : db_d)); $fatal(1); end
    if ((dec[177 +: 2] == 2'd3) | (dec[188 +: 2] == 2'd3)) if (db_wl !== (db_d_drv ? db_d : db_l)) begin $display("narrow wire MISMATCH db_wl t=%0t upc=%03x wire %04x chain %04x", $time, upc, db_wl, (db_d_drv ? db_d : db_l)); $fatal(1); end
  end
`endif
endmodule
