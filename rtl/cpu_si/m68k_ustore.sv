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

// m68k_ustore.sv - control store of the silicon-structured 68000 core: micro ROM + address selector
// + nano ROM with our own encoding and ORIGIN decoder. Doc: docs/silicon-datapath.md §19 and
// docs/microcode-structure.md §2-5.
// upc = address of the microword currently at the output; uw/nw = current micro/nanowords (read on the same edge);
// uaddr = address of the next one. Reset: forces address 0 (rstp1) - the only intervention on the address outside the silicon.
import m68k_nano_pkg::*;
module m68k_ustore #(
  parameter MEM_FILE    = "m68k_urom.mem",
  parameter ORIGIN_FILE = "m68k_norigin.mem",
  parameter NANO_FILE   = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",
  parameter int NW      = NROM_W,
  parameter int NWL     = NROM_WL,
  parameter int NDEPTH  = NROM_DEPTH,
  parameter int NIDX    = NROM_IDX_W,
  parameter bit ORG_OVR_LUT = 1'b1     // see m68k_nrom: 1 testbenches, 0 core
) (
  input  logic            clk,
  input  logic            rst_n,
  input  logic            en,
  input  logic [9:0]      a1, a2, a3,
  input  logic            a0_sub,
  input  logic [9:0]      a0,
  input  logic            a0s_sub,
  input  logic [9:0]      a0s,
  input  logic [1:0]      c1c0,
  output logic [9:0]      upc,
  output logic            upc_is327,   // = (upc == 10'h327), registered together with upc: STOP executed (word 327)
  output logic [16:0]     uw,
  output logic [NW-1:0]   nw,
  output logic [14:0]     nwl_pre,   // low 15 bits of the next nanoword, one clock before en
  output logic [NW-1:0]   nw_pre,    // whole nanoword one clock before en
  output logic            ovr_o, is3_o,            // selectors of the nw_pre mux, bare (see m68k_nrom)
  output logic [NW-1:0]   nw_pre_m, nw_pre_ovr,    // the two branches of the nw_pre mux
  output logic [NIDX-1:0] nidx,
  output logic [9:0]      uaddr,
  output logic            ir_load, fmt, bit4,
  output logic [1:0]      ty, fc,
  output logic [4:0]      cbc
);
  logic [9:0] addr_eff;
  logic       en_eff;
  assign addr_eff = rst_n ? uaddr : 10'd0;
  assign en_eff   = en | ~rst_n;

  // the micro ROM is in M10K: its data comes out after the clock-to-output delay of the block, too late for the en cone. uw_q is a
  // free-running register: its consumers (useq for the next address, flags I/fmt/ty/bit4/cbc at en) are >= 4 pulses away;
  // only fc, read by the busctl at ph0 (one pulse), stays on the direct output of the memory.
  logic [16:0] uw_raw, uw_q; logic [1:0] fc_seq;
  m68k_urom #(.MEM_FILE(MEM_FILE)) u_rom (.clk(clk), .en(en_eff), .addr(addr_eff), .q(uw_raw));
  always_ff @(posedge clk) uw_q <= uw_raw;
  assign uw = uw_raw;
  assign fc = uw_raw[16:15];
  // the next address without override goes to the ORIGIN table at every clock; immediate a0s override or reset -> ovr
  logic [9:0] base_o, base_pre, base_noa0; logic ovr, a0_sel;   // base_noa0/a0_sel: see m68k_useq
  assign base_pre = base_o;   // already zero in reset inside the sequencer
  // the address WITHOUT a0 goes to the ORIGIN table; a0 has its own lookup in parallel inside nrom
  logic [9:0] base_pre_na; assign base_pre_na = base_noa0;   // already zero in reset, like base_pre
  assign ovr = a0s_sub | ~rst_n;
  m68k_nrom #(.NW(NW), .NWL(NWL), .NDEPTH(NDEPTH), .NIDX(NIDX), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE),
              .ORG_OVR_LUT(ORG_OVR_LUT), .ORG_000(NROM_ORG_000), .ORG_003(NROM_ORG_003), .ORG_L_000(NROM_L_000), .ORG_L_003(NROM_L_003), .ORG_M_000(NROM_M_000), .ORG_M_003(NROM_M_003))
    n_rom (.clk(clk), .en(en_eff), .addr(addr_eff), .addr_pre(base_pre_na), .a0_sel(rst_n & a0_sel), .a0(a0), .ovr(ovr), .is3_i(rst_n & a0s_sub & (a0s == 10'h003)), .nidx(nidx), .q(nw), .q_l_pre(nwl_pre), .q_pre(nw_pre), .ovr_o(ovr_o), .is3_o(is3_o), .q_pre_m(nw_pre_m), .q_pre_ovr(nw_pre_ovr));
  always_ff @(posedge clk) if (en_eff) upc <= addr_eff;
  logic is327_w;   // from m68k_useq: uaddr == 327 computed on the candidates
  always_ff @(posedge clk) if (en_eff) upc_is327 <= rst_n & is327_w;   // same condition and same data as upc; = (addr_eff == 327)
`ifdef VERILATOR
  always @(posedge clk) if ((rst_n & is327_w) !== (addr_eff == 10'h327)) begin
    $display("next address 327 flag MISMATCH t=%0t addr_eff=%0h is327=%b", $time, addr_eff, is327_w); $fatal(1);
  end
`endif

  m68k_useq u_seq (.rst_n(rst_n), .base_noa0(base_noa0), .a0_sel(a0_sel), .uw(uw_q), .a1(a1), .a2(a2), .a3(a3), .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s),
                   .c1c0(c1c0), .uaddr(uaddr), .ir_load(ir_load), .fmt(fmt), .ty(ty), .bit4(bit4), .cbc(cbc), .fc(fc_seq), .base_o(base_o), .is327_o(is327_w));
  /* verilator lint_off UNUSEDSIGNAL */
  logic unused_fc; assign unused_fc = |fc_seq;
  /* verilator lint_on UNUSEDSIGNAL */
endmodule
