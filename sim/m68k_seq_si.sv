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

// m68k_seq_si.sv - SHIM (simulation only): module `m68k_seq`, the port interface the instruction testbenches (tb_m68k_*.cpp)
// are written against, which instantiates the silicon-structured core m68k_cpu_si inside.
// Declared behaviour of the interface: start is ignored (the core starts by itself when rst_n is released); rst_vector=0 (start
// from pc_init without reading the vectors) is emulated by intercepting the first 4 reads (0,2,4,6): SSP = 0, PC = pc_init;
// reset_out = the core's reset_o (RESET instruction: 124 clocks). The .mem files are resolved relative to the run cwd (sim/).
`default_nettype none
module m68k_seq (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cen,        // phi1
    input  wire        cen2,       // phi2 (tb single-cen: cen2=cen)

    input  wire        start,        // starts execution from pc_init
    input  wire [31:0] pc_init,
    // rst_vector=1: at start it does NOT use pc_init, but performs the RESET-FETCH of the real 68000:
    // reads SSP from 0x000000 (long) and PC from 0x000004 (long), initialises A7=SSP,
    // then executes from PC. Default 0 = starts from pc_init (compatibility with existing tbs).
    input  wire        rst_vector,

    // Hardware interrupt (level IPL2-IPL0, active high here: the core receives ~ipl).
    input  wire [2:0]  ipl,
    // VPAn (autovector / 6800). Keep at 1 if unused.
    input  wire        vpa_n,

    // Bus towards memory (single, arbitrated between fetch and data read/write)
    output wire [23:1] addr,
    output wire        as_n,
    output wire        uds_n,
    output wire        lds_n,
    output wire        rw,
    input  wire [15:0] data_in,
    output wire [15:0] data_out,    // data written by the CPU (write)
    output wire        data_oe,     // 1 = the CPU drives the data bus
    input  wire        dtack_n,
    // Function code (FC2/FC1/FC0).
    output wire [2:0]  fc,
    // M6800 synchronous bus.
    output wire        e,            // Enable: clock free-running clk/10, duty 6/4
    output wire        vma_n,        // Valid Memory Address

    // Bus control: HALT and arbitration (DMA).
    input  wire        halt_n,
    output wire        halted_n,
    input  wire        berr_n,
    input  wire        br_n,
    output wire        bg_n,
    input  wire        bgack_n,
    output wire        bus_tri,

    // Towards the outside
    output wire        reset_out,    // asserted by the RESET instruction for its 124 clocks (core output reset_o)
    output wire [31:0] pc_dbg,       // current PC
    output reg         illegal,      // 1 if an illegal opcode was encountered

    // Register observability (for the testbench)
    input  wire [2:0]  dbg_rd_dnum,
    output wire [31:0] dbg_rd_ddata,
    input  wire [2:0]  dbg_rd_anum,
    output wire [31:0] dbg_rd_adata,
    output wire [15:0] sr_dbg,
    // High at the instruction boundary with the opcode ready (read by tb_m68k_cpu_si.cpp).
    output wire        dbg_instr_done
);
    parameter MEM_FILE    = "../rtl/cpu_si/m68k_urom.mem";
    parameter ORIGIN_FILE = "../rtl/cpu_si/m68k_norigin.mem";
    parameter NANO_FILE   = "../rtl/cpu_si/m68k_nrom.mem";
    parameter NANO_M_FILE   = "../rtl/cpu_si/m68k_nrom_m.mem";
    parameter bit SINGLE_CEN = 1'b0;   // 1: single-phase tb (does not drive cen2): cen2 = cen
    wire cen2_eff = SINGLE_CEN ? cen : cen2;
    parameter bit TRACE = 1'b0;   // 1: prints the bus cycles (debug, no effect on behaviour)
    reg as_t; always @(posedge clk) begin as_t <= as_n; if (TRACE && as_n && !as_t) $display("BUS %s %06x %04x fc%d upc %03x", rw ? "R" : "W", {addr, 1'b0}, rw ? data_in_eff : data_out, fc, u_core.upc); end
    wire unused_start = start;
    // rst_vector=0 emulation: the first four reads after reset (vectors 0/2/4/6) receive SSP=0 and PC=pc_init
    reg [2:0] vec_cnt; reg as_q;
    always @(posedge clk) begin
        if (!rst_n) begin vec_cnt <= 3'd0; as_q <= 1'b1; end
        else begin
            as_q <= as_n;
            if (!as_q && as_n && vec_cnt != 3'd4) vec_cnt <= vec_cnt + 3'd1;   // counts at the END of the access: the emulated data stays valid for the whole cycle (sampled at S6)
        end
    end
    wire [2:0] vec_idx = vec_cnt;
    wire       vec_act = ~rst_vector && (vec_cnt != 3'd4) && !as_n;
    wire [15:0] vec_word = (vec_idx == 3'd2) ? pc_init[31:16] : (vec_idx == 3'd3) ? pc_init[15:0] : 16'd0;
    wire [15:0] data_in_eff = vec_act ? vec_word : data_in;
    wire dbg_instr_done_w, dbg_illegal_w;
    wire [31:0] pc_q_core;   // PC of the silicon core = next word to fetch (X+2 at dispatch); pc_dbg exposes the instruction address
    assign pc_dbg = pc_q_core - 32'd2;
    m68k_cpu_si #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .DBG(1'b1)) u_core (
        .clk(clk), .reset(~rst_n), .cen(cen), .cen2(cen2_eff),
        .addr(addr), .as_n(as_n), .uds_n(uds_n), .lds_n(lds_n), .rw(rw), .data_oe(data_oe),
        .data_in(data_in_eff), .data_out(data_out), .dtack_n(dtack_n), .fc(fc), .ipl_n(~ipl), .iack(),
        .vpa_n(vpa_n), .vma_n(vma_n), .e(e), .berr_n(berr_n), .halt_n(halt_n), .halted_n(halted_n),
        .br_n(br_n), .bg_n(bg_n), .bgack_n(bgack_n), .bus_tri(bus_tri), .reset_o(reset_out), .dbg_pc(),
        .upc(), .ird_o(), .psw_o(sr_dbg), .pc_o(pc_q_core), .au_o(), .aob_o(), .at_o(), .dbg_alu_a(), .dbg_alu_d(), .dbg_alu_r(),
        .dbg_rd_dnum(dbg_rd_dnum), .dbg_rd_anum(dbg_rd_anum), .dbg_rd_ddata(dbg_rd_ddata), .dbg_rd_adata(dbg_rd_adata),
        .dbg_instr_done(dbg_instr_done_w), .dbg_illegal(dbg_illegal_w));
    assign dbg_instr_done = dbg_instr_done_w;
    always @(*) illegal = dbg_illegal_w;
endmodule
`default_nettype wire
