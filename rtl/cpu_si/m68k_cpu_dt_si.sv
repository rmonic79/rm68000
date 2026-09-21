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

// m68k_cpu_dt_si.sv — processor + clock-enable/DTACK generator (m68k_cen_dtack) in ONE hierarchy node.
//
// Why: the three signals that cross the boundary between generator and processor (cpu_cen/cpu_cenb to `en`,
// AS/UDS/LDS to the generator, DTACKn to the processor) are timing-critical; with generator and processor as children
// of the same module the fitter can place them together instead of as two separate blocks. Same logic, same cycles,
// same flops as the two modules side by side; only the placement boundary changes. The surrounding logic receives
// from here the clock enables already gated with the pause, the DTACKn and the latched read data.
//
// Reset: `reset` arrives NOT registered (active high). Here rst_q (power-up at 1) registers it for the generator, and the
// processor registers it in its three copies (RST_REG = 1), all at the same clock.
// Pause: `pause_n` (1 = run) arrives from the top as logic and is registered here (power-up at 1 = not paused)
// before the AND with the clock enables. One clock of delay on the pause, no processor cycle changes.

module m68k_cpu_dt_si #(
  parameter W           = 8,          // width of num/den of the generator
  parameter RECOVERY    = 1,          // recovery of the cycles lost in the stalls
  parameter bit BUSDP_NODE = 1'b0,
  parameter MEM_FILE    = "m68k_urom.mem",
  parameter ORIGIN_FILE = "m68k_norigin.mem",
  parameter NANO_FILE   = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",
  parameter bit DBG     = 1'b0,
  parameter bit RST_REG = 1'b1,       // three registered copies of the reset in the processor
  parameter bit SS_EXT  = 1'b0        // 1 = clock enables closed at whole cycles by the external savestate (cen_ext_i)
) (
  input  logic         clk,
  input  logic         reset,         // system reset NOT registered (active high)
  input  logic [W-2:0] num,           // fractional divider of the generator
  input  logic [W-1:0] den,
  input  logic         bus_cs,        // access to the "special" memory (e.g. SDRAM), from the system memory map
  input  logic         bus_busy,      // memory busy: holds back DTACK
  input  logic         pause_n,       // 1 = the processor runs; 0 = stopped (pause / savestate hold)
  output logic         cen_o, cenb_o, // phases phi1/phi2 gated with the pause, for the surrounding logic
  output logic         dtack_n_o,     // DTACKn of the generator, for the surrounding logic
  output logic         cen_raw_o, cenb_raw_o,   // raw clock enables of the generator (to the external savestate)
  input  logic         cen_ext_i, cenb_ext_i,   // clock enables closed by the savestate (safety net only, with SS_EXT = 1)
  input  logic         gate_q_i, gate_raw_i,    // SS_EXT: closing flop of the savestate and its input (next value)
  input  logic         ss_ovr_i,               // SS_EXT: the savestate replaces the read data
  input  logic [15:0]  ss_word_i,              // replaced word
  input  logic         ss_ovr_c_i,             // combinational form of the command (safety net only)
  input  logic [15:0]  ss_word_c_i,
  // external bus (like m68k_cpu_si)
  output logic [23:1]  addr,
  output logic         as_n, uds_n, lds_n, rw,
  output logic         data_oe,
  input  logic [15:0]  bus_rdata,     // bus data from the surrounding logic: it is latched in here (rd_q)
  output logic [15:0]  data_out,
  output logic [2:0]   fc,
  input  logic [2:0]   ipl_n,
  output logic         iack,
  input  logic         vpa_n,
  output logic         vma_n,
  output logic         e,
  input  logic         berr_n,
  input  logic         halt_n,
  output logic         halted_n,
  input  logic         br_n,
  output logic         bg_n,
  input  logic         bgack_n,
  output logic         bus_tri,
  output logic         reset_o,       // RESET instruction: asserted for its 124 clocks (may be left unconnected)
  output logic [23:0]  dbg_pc
);
  // registered reset for the generator
  logic rst_q = 1'b1;
  always_ff @(posedge clk) rst_q <= reset;
  logic cpu_cen, cpu_cenb, dtack_n, cen_nx, cenb_nx;
  m68k_cen_dtack #(.W(W), .RECOVERY(RECOVERY)) u_dtack (
    .rst(rst_q), .clk(clk), .num(num), .den(den), .cpu_cen(cpu_cen), .cpu_cenb(cpu_cenb),
    .bus_cs(bus_cs), .bus_busy(bus_busy), .bus_legit(1'b0), .bus_ack(1'b0),
    .ASn(as_n), .DSn({uds_n, lds_n}), .VPAn(vpa_n), .wait2(1'b0), .wait3(1'b0),
    .DTACKn(dtack_n), .fave(), .fworst(), .cen_nx(cen_nx), .cenb_nx(cenb_nx));
  // the phases gated with the pause come out of a flop. cpu_cen <= pulse & ~phase (in the generator; its next value is cen_nx)
  // and a registered pause halt_q <= pause_n are both functions of the values at t-1, so cen_q <= pulse & ~phase & pause_n
  // (reset like cpu_cen: 0 under rst_q) equals cpu_cen(t) & halt_q(t) at every clock, with no LUT between the flop of the
  // generator and the logic of `en` or any other consumer of cen/cenb. The debt counter of the generator uses its non-gated
  // flops. Safety net below.
  // With SS_EXT the closing is that of the savestate, computed from its next value: gate_q(t+1) =
  // cpu_cenb(t) ? gate_raw(t) : gate_q(t); without SS_EXT it is the pause.
  wire gate_nx = SS_EXT ? (cpu_cenb ? gate_raw_i : gate_q_i) : pause_n;
  logic cen_q = 1'b0, cenb_q = 1'b0;
  always_ff @(posedge clk) begin
    if (rst_q) begin cen_q <= 1'b0; cenb_q <= 1'b0; end
    else begin cen_q <= cen_nx & gate_nx; cenb_q <= cenb_nx & gate_nx; end
  end
  assign cen_o  = cen_q;
  assign cenb_o = cenb_q;
  assign cen_raw_o = cpu_cen; assign cenb_raw_o = cpu_cenb;
`ifdef VERILATOR
  // SAFETY NET: the equivalent form cpu_cen & halt_q (halt_q <= pause_n), compared at every clock outside reset
  logic halt_q = 1'b1;
  always_ff @(posedge clk) halt_q <= pause_n;
  always @(posedge clk) if (!SS_EXT && !rst_q && (cen_q !== (cpu_cen & halt_q) || cenb_q !== (cpu_cenb & halt_q))) begin
    $display("gated clock enable MISMATCH t=%0t cen %b/%b cenb %b/%b", $time, cen_q, cpu_cen & halt_q, cenb_q, cpu_cenb & halt_q); $fatal(1);
  end
  // SAFETY NET: with SS_EXT the clock enables computed here equal those closed by the savestate
  always @(posedge clk) if (SS_EXT && !rst_q && (cen_q !== cen_ext_i || cenb_q !== cenb_ext_i)) begin
    $display("savestate clock enable MISMATCH t=%0t cen %b/%b cenb %b/%b", $time, cen_q, cen_ext_i, cenb_q, cenb_ext_i); $fatal(1);
  end
`endif
  assign dtack_n_o = dtack_n;
  // latch of the bus data: it loads at every clock in which the cycle is active (AS low with UDS or LDS low),
  // DTACKn is low and it is a read; reset with rst_q.
  // The interrupt acknowledge cycles (FC = 111) load it too: the vector number of a vectored interrupt is the
  // byte read in the IACK cycle that ends with DTACK (MC68000UM 6.3.2 p. 6-12) and it reaches the processor through this latch.
  // With VPA (autovector) the processor does not use the data of the IACK cycle, and no microword reads IRC or DBIN between
  // the IACK (itlx5) and the vector read (trap6): loading the latch in that cycle changes nothing there. The latch has no FC
  // input: every read terminated by DTACK loads it.
  logic        bus_req, cap_rd; logic [15:0] rd_q;
  assign bus_req  = ~as_n & (~uds_n | ~lds_n);
  assign cap_rd   = bus_req & ~dtack_n & rw;
  always_ff @(posedge clk) begin
    if (rst_q) rd_q <= 16'd0;
    else if (cap_rd) rd_q <= (SS_EXT && ss_ovr_i) ? ss_word_i : bus_rdata;   // savestate mux in front of the latch
  end
`ifdef VERILATOR
  // SAFETY NET: shadow latch with the combinational form of the savestate; at every edge in which the CPU
  // advances, the data it sees must be the same
  logic [15:0] rd_c;
  always_ff @(posedge clk) begin
    if (rst_q) rd_c <= 16'd0;
    else if (cap_rd) rd_c <= (SS_EXT && ss_ovr_c_i) ? ss_word_c_i : bus_rdata;
  end
  always @(posedge clk) if (SS_EXT && !rst_q && (cen_o || cenb_o) && rd_q !== rd_c) begin
    $display("savestate read data MISMATCH t=%0t rd_q %h rd_c %h", $time, rd_q, rd_c); $fatal(1);
  end
`endif
  m68k_cpu_si #(.BUSDP_NODE(BUSDP_NODE), .MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE),
                .NANO_M_FILE(NANO_M_FILE), .DBG(DBG), .RST_REG(RST_REG)) u_cpu (
    .clk(clk), .reset(reset), .cen(cen_o), .cen2(cenb_o),
    .addr(addr), .as_n(as_n), .uds_n(uds_n), .lds_n(lds_n), .rw(rw), .data_oe(data_oe), .data_in(rd_q),
    .data_out(data_out), .dtack_n(dtack_n), .fc(fc), .ipl_n(ipl_n), .iack(iack), .vpa_n(vpa_n), .vma_n(vma_n), .e(e),
    .berr_n(berr_n), .halt_n(halt_n), .halted_n(halted_n), .br_n(br_n), .bg_n(bg_n), .bgack_n(bgack_n), .bus_tri(bus_tri),
    .reset_o(reset_o), .dbg_pc(dbg_pc),
    .upc(), .ird_o(), .psw_o(), .pc_o(), .au_o(), .aob_o(), .at_o(), .dbg_alu_a(), .dbg_alu_d(), .dbg_alu_r(),
    .dbg_rd_dnum(3'd0), .dbg_rd_anum(3'd0), .dbg_rd_ddata(), .dbg_rd_adata(), .dbg_instr_done(), .dbg_illegal());
endmodule
