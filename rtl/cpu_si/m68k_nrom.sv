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

// m68k_nrom.sv - nano ROM with OUR OWN ENCODING + ORIGIN table. Maintained by hand: no generator writes this file.
// Source: docs/microcode-structure.md §5 (nano ROM addressed by the same micro address, many-to-one) and
// docs/silicon-datapath.md §19.3-19.4. Images written by gen_nrom.py from the App. H blocks: m68k_norigin.mem (micro group
// addr[9:2] -> row; nanoword index = {row, addr[1:0]}), m68k_nrom.mem and m68k_nrom_m.mem (the two parts of the nanowords).
// Constants written by hand: C2_T*_B* (the ORIGIN table, from m68k_norigin.mem, checked against it under VERILATOR). The
// ORG_* parameters (index and nanoword of 000/003) come from m68k_nano_pkg.sv. Output loaded at en (latch 88 of the patent).
module m68k_nrom #(
  parameter int NW     = 73,
  parameter int NWL    = 40,   // bits [NWL-1:0]: critical part, [NW-1:NWL]: non-critical part; both in M10K, read one clock before en
  parameter int NDEPTH = 512,
  parameter int NIDX   = 9,
  parameter ORIGIN_FILE = "m68k_norigin.mem",
  parameter NANO_FILE   = "m68k_nrom.mem",
  parameter NANO_M_FILE = "m68k_nrom_m.mem",
  parameter bit ORG_OVR_LUT = 1'b1,   // 1 = a0s override via full LUT (testbenches: immediate jump anywhere); 0 = constants (core)
  parameter int ORG_000 = 0,           // ORIGIN index of 000 (A_RESET) and 003 (A_BSER), from the pkg (gen_nrom)
  parameter int ORG_003 = 0,
  parameter int ORG_L_000 = 0,         // bits [NWL-1:0] of the nanowords of 000 and 003, from the pkg (gen_nrom): immediate override in the core
  parameter int ORG_L_003 = 0,
  parameter logic [59:0] ORG_M_000 = 60'd0,   // bits [NW-1:NWL] of the nanowords of 000 and 003, from the pkg:
  parameter logic [59:0] ORG_M_003 = 60'd0    // the override of the high half comes from constants, not from a second read of rom_m
) (
  input  logic            clk,
  input  logic            en,
  input  logic [9:0]      addr,
  input  logic [9:0]      addr_pre,
  // a0 (exception input, few constants) and its selector: the ORIGIN of a0 is computed in parallel with that
  // of addr_pre (which here arrives WITHOUT a0) and is chosen AFTER the table, so the path from the exception
  // logic to the M10K address does not cross the table.
  input  logic            a0_sel,
  input  logic [9:0]      a0,   // next address without override: to the ORIGIN table at every clock
  input  logic            ovr,        // immediate override (a0s_sub) or reset: index from constants (core) or from the LUT (testbenches)
  input  logic            is3_i,      // = (addr == 003) when ovr, computed upstream as rst_n & a0s_sub & (a0s == 003): without the mux chain of useq
  output logic [NIDX-1:0] nidx,   // index of the output nanoword (observability)
    output logic [NW-1:0]   q,
    output logic [14:0]     q_l_pre,  // the 15 low bits that q_l will have at en, one clock earlier
    output logic [NW-1:0]   q_pre,    // WHOLE nanoword one clock before en
    // for the precomputed g of the control store: the two branches of the q_pre mux and their selectors, bare
    output logic            ovr_o,     // = ovr
    output logic            is3_o,     // = (addr == 003) in the constants branch; 0 in the LUT branch (testbenches)
    output logic [NW-1:0]   q_pre_m,   // = {qm_pre, ql_pre}: the word without override
    output logic [NW-1:0]   q_pre_ovr  // = {qm_ovr, ql_ovr}: the override word
);
  // ORIGIN = 256 x (NIDX-2) table in logic: the nanoword index is {row(addr[9:2]), addr[1:0]} (nanowords numbered by gen_nrom.py).
  // Both parts of the nanoword are in M10K and are READ ONE CLOCK BEFORE en: the critical part (bits [NWL-1:0]: bus sources,
  // indices, aob, acc*, consumed at ph0) in rom_l, the non-critical part (bits [NW-1:NWL]: ALU/AU at op_ld, writes at en) in rom_m.
  // With the ORIGIN in logic the index is known at the edge before en (from addr_pre), so the M10K address is captured at that
  // edge, the data comes out in the clock that precedes en and q_l/q_m load it at en: whoever consumes q at ph0 sees no memory
  // output delay. Clock count in docs/silicon-datapath.md §19.3 (ORIGIN map and early read).
  (* ramstyle = "logic" *) logic [NIDX-3:0]   org_t  [0:255];
  (* ramstyle = "M10K" *)  logic [NWL-1:0]    rom_l  [0:NDEPTH-1];
  (* ramstyle = "M10K" *)  logic [NW-NWL-1:0] rom_m  [0:NDEPTH-1];
  initial begin
    $readmemh(ORIGIN_FILE, org_t);
    $readmemb(NANO_FILE, rom_l);
    $readmemb(NANO_M_FILE, rom_m);
  end
  // nanoword index: from the address sampled at the clock before (base stable from en-1: uw_q, c1c0, a1-a3, a0 are registers),
  // or, with immediate override or reset, from the constants (core: a0s in {000, 003}) or from the table on the current address (testbenches).
  // The table lies BEFORE the registers: idx_m (flop) and the M10K address registers of rom_l and rom_m capture the same index at
  // every clock, free-running and without enable (in reset addr_pre = 0, so they hold the index of 000 from the first reset clock).
  logic [NIDX-1:0] idx, idx_pre, idx_m, idx_ovr;
  logic [NWL-1:0]  ql_pre, ql_ovr;
  logic [NW-NWL-1:0] qm_ovr;   // high half of the override word
  // Table with EXACTLY TWO LUT levels: four sub-tables of 64 entries (functions of addr_pre[7:2], one 6-input LUT per
  // bit; the 'keep' wires prevent synthesis from merging them back into a deeper form) and a 4-way mux on addr_pre[9:8] (4 data +
  // 2 selectors = one 6-input LUT per bit). It is on the path that reaches the M10K address: it must stay at two levels.
  (* keep *) wire [NIDX-3:0] t_s0;
  (* keep *) wire [NIDX-3:0] t_s1;
  (* keep *) wire [NIDX-3:0] t_s2;
  (* keep *) wire [NIDX-3:0] t_s3;
  // The four sub-tables as EXPLICIT truth tables: one 64-bit constant per (table, bit) and the bit-select with
  // addr_pre[7:2] = ONE LUT6 per bit (on the org_t array loaded with $readmemh, synthesis builds a 64-way mux of
  // 3 LUTs in series). The constants are written by hand from m68k_norigin.mem (generated by gen_nrom.py): bit k of C2_T<t>_B<b>
  // is bit b of the row of micro group 64*t + k, i.e. of the (64*t + k)-th value of the file counting from 0; if gen_nrom.py
  // changes the ORIGIN table they must be rewritten. The safety net below checks them against the array at every clock.
  localparam logic [63:0] C2_T0_B0 = 64'h0a0d05050505050a;
  localparam logic [63:0] C2_T0_B1 = 64'h0c0e06060606060c;
  localparam logic [63:0] C2_T0_B2 = 64'hfff0f7f8f7f8f7f0;
  localparam logic [63:0] C2_T0_B3 = 64'h0f0f0800070f0800;
  localparam logic [63:0] C2_T0_B4 = 64'h0f0f0f0f08000000;
  localparam logic [63:0] C2_T0_B5 = 64'h0000000000000000;
  localparam logic [63:0] C2_T0_B6 = 64'h0000000000000000;
  localparam logic [63:0] C2_T1_B0 = 64'h552b000050500aae;
  localparam logic [63:0] C2_T1_B1 = 64'h6633000090907330;
  localparam logic [63:0] C2_T1_B2 = 64'h78bcffff1fef83c0;
  localparam logic [63:0] C2_T1_B3 = 64'h7f40000010f07c00;
  localparam logic [63:0] C2_T1_B4 = 64'h7f7f0000e0000000;
  localparam logic [63:0] C2_T1_B5 = 64'h7f7f0000f0f07fff;
  localparam logic [63:0] C2_T1_B6 = 64'h8000000000000000;
  localparam logic [63:0] C2_T2_B0 = 64'h20c0a0d050505050;
  localparam logic [63:0] C2_T2_B1 = 64'h3000c0e060606060;
  localparam logic [63:0] C2_T2_B2 = 64'hcf0fff0f7f8f7f8f;
  localparam logic [63:0] C2_T2_B3 = 64'hf0f0000070f08000;
  localparam logic [63:0] C2_T2_B4 = 64'hf0f0f0f080000000;
  localparam logic [63:0] C2_T2_B5 = 64'h0000000000000000;
  localparam logic [63:0] C2_T2_B6 = 64'hf0f0f0f0f0f0f0f0;
  localparam logic [63:0] C2_T3_B0 = 64'h85020a0d05050505;
  localparam logic [63:0] C2_T3_B1 = 64'h89040c0e06060606;
  localparam logic [63:0] C2_T3_B2 = 64'h71fff0fff8f7f8f7;
  localparam logic [63:0] C2_T3_B3 = 64'h8e00000f0f080007;
  localparam logic [63:0] C2_T3_B4 = 64'h8f070f0000000007;
  localparam logic [63:0] C2_T3_B5 = 64'h8f070f0f0f0f0f08;
  localparam logic [63:0] C2_T3_B6 = 64'h8f070f0f0f0f0f0f;
  assign t_s0 = {C2_T0_B6[addr_pre[7:2]], C2_T0_B5[addr_pre[7:2]], C2_T0_B4[addr_pre[7:2]], C2_T0_B3[addr_pre[7:2]], C2_T0_B2[addr_pre[7:2]], C2_T0_B1[addr_pre[7:2]], C2_T0_B0[addr_pre[7:2]]};
  assign t_s1 = {C2_T1_B6[addr_pre[7:2]], C2_T1_B5[addr_pre[7:2]], C2_T1_B4[addr_pre[7:2]], C2_T1_B3[addr_pre[7:2]], C2_T1_B2[addr_pre[7:2]], C2_T1_B1[addr_pre[7:2]], C2_T1_B0[addr_pre[7:2]]};
  assign t_s2 = {C2_T2_B6[addr_pre[7:2]], C2_T2_B5[addr_pre[7:2]], C2_T2_B4[addr_pre[7:2]], C2_T2_B3[addr_pre[7:2]], C2_T2_B2[addr_pre[7:2]], C2_T2_B1[addr_pre[7:2]], C2_T2_B0[addr_pre[7:2]]};
  assign t_s3 = {C2_T3_B6[addr_pre[7:2]], C2_T3_B5[addr_pre[7:2]], C2_T3_B4[addr_pre[7:2]], C2_T3_B3[addr_pre[7:2]], C2_T3_B2[addr_pre[7:2]], C2_T3_B1[addr_pre[7:2]], C2_T3_B0[addr_pre[7:2]]};
`ifdef VERILATOR
  // SAFETY NET: the constants against the array loaded from ORIGIN_FILE, at every clock
  always @(posedge clk) if (t_s0 !== org_t[{2'd0, addr_pre[7:2]}] || t_s1 !== org_t[{2'd1, addr_pre[7:2]}] || t_s2 !== org_t[{2'd2, addr_pre[7:2]}] || t_s3 !== org_t[{2'd3, addr_pre[7:2]}]) begin
    $display("ORIGIN CONSTANTS MISMATCH t=%0t addr_pre=%0h", $time, addr_pre); $fatal(1);
  end
`endif
  logic [NIDX-3:0] row_pre;
  always_comb case (addr_pre[9:8])
    2'd0:    row_pre = t_s0;
    2'd1:    row_pre = t_s1;
    2'd2:    row_pre = t_s2;
    default: row_pre = t_s3;
  endcase
  // The same table on a0: a0 takes few constant values, synthesis reduces it to a function of
  // only the flops that choose a0.
  logic [NIDX-3:0] row_a0; logic [NIDX-1:0] idx_na, idx_a0;
  always_comb case (a0[9:8])
    2'd0:    row_a0 = org_t[{2'd0, a0[7:2]}];
    2'd1:    row_a0 = org_t[{2'd1, a0[7:2]}];
    2'd2:    row_a0 = org_t[{2'd2, a0[7:2]}];
    default: row_a0 = org_t[{2'd3, a0[7:2]}];
  endcase
  assign idx_na  = {row_pre, addr_pre[1:0]};
  assign idx_a0  = {row_a0, a0[1:0]};
  assign idx_pre = a0_sel ? idx_a0 : idx_na;
`ifdef VERILATOR
  // safety net: the direct form, i.e. the table on the already muxed address
  logic [9:0] addr_pre_v; assign addr_pre_v = a0_sel ? a0 : addr_pre;
  logic [NIDX-3:0] row_v;
  always_comb case (addr_pre_v[9:8])
    2'd0:    row_v = org_t[{2'd0, addr_pre_v[7:2]}];
    2'd1:    row_v = org_t[{2'd1, addr_pre_v[7:2]}];
    2'd2:    row_v = org_t[{2'd2, addr_pre_v[7:2]}];
    default: row_v = org_t[{2'd3, addr_pre_v[7:2]}];
  endcase
  always @(posedge clk) if (idx_pre !== {row_v, addr_pre_v[1:0]}) begin
    $display("A0 ORIGIN MISMATCH t=%0t idx_pre=%0d expected=%0d a0_sel=%b", $time, idx_pre, {row_v, addr_pre_v[1:0]}, a0_sel); $fatal(1);
  end
`endif
  always_ff @(posedge clk) idx_m  <= idx_pre;
  always_ff @(posedge clk) ql_pre <= rom_l[idx_pre];   // M10K: address registered at every clock, unregistered output = rom_l[idx_m]
  generate if (ORG_OVR_LUT) begin : g_ovr_lut
    (* ramstyle = "logic" *) logic [NWL-1:0] rom_lb [0:NDEPTH-1];   // testbenches only: immediate override on any address
    (* ramstyle = "logic" *) logic [NW-NWL-1:0] rom_mb [0:NDEPTH-1];   // twin of rom_lb for the high half
    initial $readmemb(NANO_FILE, rom_lb);
    initial $readmemb(NANO_M_FILE, rom_mb);
    assign idx_ovr = {org_t[addr[9:2]], addr[1:0]};
    assign ql_ovr  = rom_lb[idx_ovr];
    assign is3_o = 1'b0;   // in the LUT branch the override g is computed from the word
    assign qm_ovr  = rom_mb[idx_ovr];
  end else begin : g_ovr_const
    localparam logic [NIDX-1:0] K000 = ORG_000[NIDX-1:0], K003 = ORG_003[NIDX-1:0];
    localparam logic [NWL-1:0]  L000 = ORG_L_000[NWL-1:0], L003 = ORG_L_003[NWL-1:0];
    localparam logic [NW-NWL-1:0] M000 = ORG_M_000[NW-NWL-1:0], M003 = ORG_M_003[NW-NWL-1:0];
    assign idx_ovr = is3_i ? K003 : K000;
    assign ql_ovr  = is3_i ? L003 : L000;
    assign is3_o = is3_i;   // the same selector that chooses L003/M003, computed upstream
`ifdef VERILATOR
    // SAFETY NET: under override the selector from upstream must equal the direct compare (addr == 003)
    always @(posedge clk) if (ovr && is3_i !== (addr == 10'h003)) begin $display("OVERRIDE SELECTOR MISMATCH t=%0t addr %0h is3_i %b", $time, addr, is3_i); $fatal(1); end
`endif
    assign qm_ovr  = is3_i ? M003 : M000;
  end endgenerate
  assign idx = ovr ? idx_ovr : idx_m;
  logic [NWL-1:0] q_l; logic [NW-NWL-1:0] q_m;
  always_ff @(posedge clk) if (en) begin
    q_l  <= ovr ? ql_ovr : ql_pre;
    nidx <= idx;
  end
  logic [NW-NWL-1:0] qm_pre;
  always_ff @(posedge clk) qm_pre <= rom_m[idx_pre];   // high part one clock earlier, case without override
  always_ff @(posedge clk) if (en) q_m <= ovr ? qm_ovr : qm_pre;   // the override is sampled at en, like q_l
  assign q = {q_m, q_l};
  assign q_l_pre = ovr ? ql_ovr[14:0] : ql_pre[14:0];
  assign q_pre = {ovr ? qm_ovr : qm_pre, ovr ? ql_ovr : ql_pre};   // whole word that q will take at en
  assign ovr_o = ovr; assign q_pre_m = {qm_pre, ql_pre}; assign q_pre_ovr = {qm_ovr, ql_ovr};
endmodule
