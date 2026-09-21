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

// m68k_regs_top.sv - control store + decoder + section fabric + AT/AOB/DOB/DBIN registers (testbench top). Core
// configuration: ORG_OVR_LUT = 0 (1 = testbench configuration). It exposes the current microword `uw` (positioning of the testbench along the
// microcode) and the values being loaded aob_n / dob_n / dbin_n.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_regs_top #(parameter MEM_FILE = "m68k_urom.mem", parameter ORIGIN_FILE = "m68k_norigin.mem", parameter NANO_FILE = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",
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
  input  logic [15:0] src_edb,
  output logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,
  output logic        ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv,
  output logic [31:0] at, aob, output logic [15:0] dob, dbin, output logic [31:0] aob_n, output logic [15:0] dob_n, dbin_n
);
  logic [15:0] ab_dob_w; logic ab_dob_drv_w;   // narrow copy of the AB sections for the DOB (m68k_bus_sec)
  m68k_bus_top #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .ORG_OVR_LUT(ORG_OVR_LUT)) u_bus (.clk(clk), .rst_n(rst_n), .en(en), .a1(a1), .a2(a2), .a3(a3),
    .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s), .c1c0(c1c0), .upc(upc), .dec(dec), .uw(uw),
    .src_alu(src_alu), .src_alue(src_alue), .src_aob(src_aob), .src_at(src_at), .src_ath(src_ath), .src_atl(src_atl), .src_au(src_au), .src_dbin(src_dbin), .src_dcr(src_dcr), .src_ftu(src_ftu), .src_rf_ab(src_rf_ab), .src_rf_db(src_rf_db),
    .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob_w), .ab_dob_drv(ab_dob_drv_w));
  m68k_regs_bd u_regs (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob_w), .ab_dob_drv(ab_dob_drv_w),
    .src_au(src_au), .src_alu(src_alu), .src_edb(src_edb), .at(at), .aob(aob), .dob(dob), .dbin(dbin), .aob_n(aob_n), .dob_n(dob_n), .dbin_n(dbin_n));
endmodule
