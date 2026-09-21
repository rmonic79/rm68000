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

// m68k_alux_top.sv - testbench top of m68k_alux_si: control store + decoder + ALU with its surrounding registers (bus sections, IR/IRD and PSW from the tb).
// Core configuration: ROW_REG = 1 (matrix row from a flop loaded at en & ld_ird from `ir`, as in m68k_cpu_si) and
// ORG_OVR_LUT as in the core (0; 1 = testbench configuration, a0s override on any word). It exposes ty/fmt/uw of the
// current microword so that the testbench can walk the microcode up to a dispatch and position itself via A1.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_alux_top #(parameter MEM_FILE = "m68k_urom.mem", parameter ORIGIN_FILE = "m68k_norigin.mem", parameter NANO_FILE = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",
                       parameter bit ORG_OVR_LUT = 1'b0) (
  input  logic clk, rst_n, en, input logic [9:0] a1, a2, a3, input logic a0_sub, input logic [9:0] a0, input logic a0s_sub, input logic [9:0] a0s, input logic [1:0] c1c0,
  input  logic op_ld, st_ld, instr_start, is_byte,
  output logic [9:0] upc, output logic [NDEC_W-1:0] dec,
  input  logic [15:0] ird, ir, input logic ld_ird,
  input  logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d, input logic ab_l_drv, ab_d_drv, db_l_drv, db_d_drv, input logic [4:0] cc_in,
  output logic [15:0] src_alu, src_alue, src_dcr, output logic [4:0] cc_alu, output logic locz, d4, output logic [1:0] alue10,
  output logic fmt, output logic [1:0] ty, output logic [16:0] uw
);
  /* verilator lint_off UNUSEDSIGNAL */
  logic [NROM_W-1:0] nw; logic [NROM_IDX_W-1:0] nidx; logic [9:0] uaddr; logic ir_load, bit4; logic [1:0] fc; logic [4:0] cbc;
  /* verilator lint_on UNUSEDSIGNAL */
  m68k_nstore_dec #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .ORG_OVR_LUT(ORG_OVR_LUT)) u_cs (.clk(clk), .rst_n(rst_n), .en(en), .a1(a1), .a2(a2), .a3(a3),
    .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s), .c1c0(c1c0), .upc(upc), .uw(uw), .nw(nw), .dec(dec), .nidx(nidx), .uaddr(uaddr),
    .ir_load(ir_load), .fmt(fmt), .bit4(bit4), .ty(ty), .fc(fc), .cbc(cbc),
    .g(), .gq(), .gd(), .c_ft_q(), .we_ab_h_q(), .we_ab_l_q(), .we_db_h_q(), .we_db_l_q(), .nwl_pre(), .nw_pre());   // outputs not used by this testbench
  m68k_alux_si #(.ROW_REG(1'b1)) u_alux (.clk(clk), .rst_n(rst_n), .en(en), .op_ld(op_ld), .st_ld(st_ld), .instr_start(instr_start), .is_byte(is_byte), .dec(dec), .ird(ird), .ir(ir), .ld_ird(ld_ird), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv), .cc_in(cc_in), .src_alu(src_alu), .src_alue(src_alue), .dbg_a(), .dbg_d(), .dbg_r(), .src_dcr(src_dcr), .cc_alu(cc_alu),
    .locz(locz), .d4(d4), .alue10(alue10));
endmodule
