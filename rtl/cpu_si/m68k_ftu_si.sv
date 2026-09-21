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

// m68k_ftu_si.sv - field translation unit FTU (block 106 of the patent). Specification: docs/silicon-datapath.md
// §12. 16-bit latch loaded by the `ftu<-...` atoms (PSW, SSW, IRD, vector x4 from the trap latch, autovector (24+INL) x4, word from the
// AB bus) and consumed by the `X<-ftu` atoms; without a pending latch the output is the implicit field of the instruction in IRD (Bcc.B/BSR.B:
// signed 8-bit displacement; ADDQ/SUBQ and immediate shift: quick value 1..8; TAS: mask 0x0080).
// Parameters: count constant of MUL and DIV (loop counter); implicit value for Scc, STOP and RESET.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_ftu_si #(
  parameter logic [15:0] K_MUL = 16'd15, K_DIV = 16'd15, K_DBCC = 16'd0, K_STOP = 16'd0, K_RESET = 16'd0,
  parameter bit IMP_REG = 1'b1,   // 1 = implicit field registered from `ir` (core); 0 = combinational from ird (testbench)
  // 1 = the ftu_q output comes out of a flop loaded at `en` with the next value of `has_latch ? latch : implicit`
  // (core); 0 = combinational mux (testbench, which does not drive edb).
  parameter bit OUT_REG = 1'b1
) (
  input  logic        clk, rst_n, en,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [NDEC_W-1:0] dec,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [15:0] ird, irc, psw, ssw, ab_l,
  input  logic [15:0] edb,          // data read from the bus: it is the value that irc takes with `irc<-edb` (m68k_ir_si)
  input  logic [15:0] ir,           // instruction at the stage before ird: the implicit field is computed here
  input  logic [7:0]  tvn,          // vector number (trap latch)
  input  logic [2:0]  inl,          // latched interrupt level
  output logic [15:0] ftu_q,
  output logic        ftu_latched   // 1 = the output is the latch (observability)
);
  logic [ND_FTU_FROM_W-1:0] f; assign f = dec[ND_FTU_FROM_LSB +: ND_FTU_FROM_W];
  logic ld_psw, ld_ssw, ld_ird, ld_trap, ld_inl, ld_ab, ld_any;
  assign ld_psw  = (f == ND_FTU_FROM__P_PSW[ND_FTU_FROM_W-1:0]);
  assign ld_ssw  = (f == ND_FTU_FROM__P_SSW[ND_FTU_FROM_W-1:0]);
  assign ld_ird  = (f == ND_FTU_FROM__P_IRD[ND_FTU_FROM_W-1:0]);
  assign ld_trap = (f == ND_FTU_FROM__P_TRAP[ND_FTU_FROM_W-1:0]) | (f == ND_FTU_FROM__TRAP[ND_FTU_FROM_W-1:0]);
  assign ld_inl  = (f == ND_FTU_FROM__P_INL[ND_FTU_FROM_W-1:0]);
  assign ld_ab   = (f == ND_FTU_FROM__AB[ND_FTU_FROM_W-1:0]);
  assign ld_any  = ld_psw | ld_ssw | ld_ird | ld_trap | ld_inl | ld_ab;
  // new instruction: the latch (e.g. next opcode loaded by malw3 "(dbin) -> ab -> ftu") is no longer valid; the output goes back to the implicit
  // field of the IRD (a BRA.S after a not-taken BNE.W must not use the displacement of the BNE)
  logic new_ird; assign new_ird = (dec[ND_IRD_FROM_LSB +: ND_IRD_FROM_W] == ND_IRD_FROM__P_IR[ND_IRD_FROM_W-1:0]);
  // implicit field as a pure function of the instruction (the DBcc branch takes IRC instead: f_imp_irc).
  function automatic logic [15:0] f_imp (input logic [15:0] w);
    f_imp = 16'd0;
    if (w[15:12] == 4'h6)                                     f_imp = {{8{w[7]}}, w[7:0]};   // Bcc.B / BSR.B
    else if (w[15:12] == 4'h7)                                f_imp = {{8{w[7]}}, w[7:0]};   // MOVEQ (rlql1 "ftu -> abe -> alu,rx")
    else if (w[15:12] == 4'h5 && w[7:6] != 2'b11)           f_imp = {12'd0, (w[11:9] == 3'd0) ? 4'd8 : {1'b0, w[11:9]}};   // ADDQ/SUBQ
    else if (w[15:12] == 4'h5 && w[7:3] == 5'b11001)       f_imp = 16'd0;                          // DBcc: displacement = extension word (in IRC at the dispatch); dcnt1 App. H p146
    else if (w[15:12] == 4'h5)                                f_imp = K_DBCC;                       // Scc (no use of the FTU)
    else if (w[15:12] == 4'he && w[7:6] != 2'b11 && !w[5]) f_imp = {12'd0, (w[11:9] == 3'd0) ? 4'd8 : {1'b0, w[11:9]}};   // immediate shift
    else if (w[15:6] == 10'b0100_1010_11)                     f_imp = 16'h0080;                    // TAS
    else if (w[15:12] == 4'hc && w[7:6] == 2'b11)           f_imp = K_MUL;                       // MULU (011) and MULS (111): loop counter
    else if (w[15:12] == 4'h8 && w[7:6] == 2'b11)           f_imp = K_DIV;                       // DIVU (011) and DIVS (111): loop counter
    else if (w == 16'h4e72)                                   f_imp = K_STOP;                      // STOP: K_STOP
    else if (w == 16'h4e70)                                   f_imp = K_RESET;                     // RESET: K_RESET
  endfunction
  // the only branch that is not a function of the instruction: DBcc takes the live extension word from IRC.
  function automatic logic f_imp_irc (input logic [15:0] w);
    f_imp_irc = (w[15:12] == 4'h5) && (w[7:3] == 5'b11001);
  endfunction
  logic [15:0] imp_v; logic imp_i;
  logic [15:0] imp_v_nx; logic imp_i_nx;   // next values of the implicit field (see below)
  generate if (IMP_REG) begin : g_imp_reg
    logic [15:0] imp_q; logic imp_irc_q;
    always_ff @(posedge clk) begin
      if (!rst_n) begin imp_q <= f_imp(16'd0); imp_irc_q <= f_imp_irc(16'd0); end   // ird starts at 0: same value as f_imp(ird) at power-up
      else if (en && new_ird) begin imp_q <= f_imp(ir); imp_irc_q <= f_imp_irc(ir); end
    end
    assign imp_v = imp_q; assign imp_i = imp_irc_q;
    assign imp_v_nx = new_ird ? f_imp(ir) : imp_q; assign imp_i_nx = new_ird ? f_imp_irc(ir) : imp_irc_q;   // the load rule of the flops
  end else begin : g_imp_comb
    assign imp_v = f_imp(ird); assign imp_i = f_imp_irc(ird);
    assign imp_v_nx = f_imp(new_ird ? ir : ird); assign imp_i_nx = f_imp_irc(new_ird ? ir : ird);   // ird' = ird<-(ir) ? ir : ird
  end endgenerate
  // consumption: a destination atom with ftu source (bus or psw)
  logic use_bus, use_psw;
  assign use_bus = (dec[ND_AB_FROM_LSB + ND_AB_FROM__FTU]) | (dec[ND_DB_FROM_LSB + ND_DB_FROM__FTU]) |
                   (dec[ND_ABE_FROM_LSB + ND_ABE_FROM__FTU]) | (dec[ND_DBE_FROM_LSB + ND_DBE_FROM__FTU]) |
                   (dec[ND_ABSTAR_FROM_LSB + ND_ABSTAR_FROM__FTU]) |
                   (dec[ND_STARE_FROM_LSB +: ND_STARE_FROM_W] == ND_STARE_FROM__FTU[ND_STARE_FROM_W-1:0]);
  assign use_psw = (dec[ND_PSW_FROM_LSB +: ND_PSW_FROM_W] == ND_PSW_FROM__FTU[ND_PSW_FROM_W-1:0]);
  logic [15:0] latch, implicit_v, ld_val;
  // load value in FLAT form: the six conditions are different codes of the FTU_FROM field (exclusive), so the
  // priority chain equals the OR of the terms. The five terms without the AB are in a kept wire, ready in parallel with the
  // fabric; ab_l enters the last LUT (3 inputs). Safety net below.
  (* keep *) logic [15:0] ftu_rest;
  assign ftu_rest = ({16{ld_psw}} & psw) | ({16{ld_ssw}} & ssw) | ({16{ld_ird}} & ird) | ({16{ld_trap}} & {6'd0, tvn, 2'b00}) | ({16{ld_inl}} & {12'hfff, inl, 1'b1});
  logic [15:0] ld_val_flat; assign ld_val_flat = ftu_rest | ({16{ld_ab}} & ab_l);
  logic        has_latch;
  always_comb begin
    ld_val = ld_val_flat;   // flat form (see above)
    // implicit field from the executing instruction
    implicit_v = imp_i ? irc : imp_v;   // one mux: the cone starts from the flop of the implicit field
    ftu_q = OUT_REG ? ftu_q_r : ftu_q_c;   // registered output (see below) or combinational mux
    ftu_latched = has_latch;
  end
`ifdef VERILATOR
  // SAFETY NET: the equivalent priority chain, compared at every clock
  logic [15:0] ld_val_v;
  always_comb begin
      ld_val_v = 16'd0;
      if (ld_psw) ld_val_v = psw;
      else if (ld_ssw) ld_val_v = ssw;
      else if (ld_ird) ld_val_v = ird;
      else if (ld_trap) ld_val_v = {6'd0, tvn, 2'b00};
      else if (ld_inl) ld_val_v = {12'hfff, inl, 1'b1};               // address of the IACK cycle: 0xFFFFF1 + 2*level (CPU space; the E section sign-extends)
      else if (ld_ab) ld_val_v = ab_l;
  end
  always @(posedge clk) if (rst_n && ld_val !== ld_val_v) begin
    $display("FTU load value MISMATCH t=%0t %h / %h", $time, ld_val, ld_val_v); $fatal(1);
  end
`endif
  // the output comes out of a flop: `ftu_q = has_latch ? latch : (imp_i ? irc : imp)` depends only on flops that change at
  // `en` (latch/has_latch at en; implicit field at en & new_ird; irc at en & irc<-edb), so `ftu_q_r <= has_latch' ? latch' :
  // (imp_i' ? irc' : imp')` loaded at en, with the next values COPIED from the load rules of the respective flops, equals
  // ftu_q at every clock. It removes the two muxes in front of every fabric source that reads the FTU.
  // Reset: 0 = has_latch 0, imp f(0).
  logic [15:0] ftu_q_c, ftu_q_r; assign ftu_q_c = has_latch ? latch : implicit_v;   // combinational form
  logic ld_irc; assign ld_irc = dec[ND_IRC_FROM_LSB +: ND_IRC_FROM_W] == ND_IRC_FROM__EDB[ND_IRC_FROM_W-1:0];   // like m68k_ir_si
  logic has_latch_nx; logic [15:0] latch_nx, irc_nx;
  assign has_latch_nx = ld_any ? 1'b1 : ((use_bus | use_psw | new_ird) ? 1'b0 : has_latch);   // the load rule of has_latch
  assign latch_nx = ld_any ? ld_val : latch;                                                     // the load rule of latch
  assign irc_nx = ld_irc ? edb : irc;                                                            // the load rule of irc (m68k_ir_si)
  generate if (OUT_REG) begin : g_out_reg
    always_ff @(posedge clk) begin
      if (!rst_n) ftu_q_r <= 16'd0;
      else if (en) ftu_q_r <= has_latch_nx ? latch_nx : (imp_i_nx ? irc_nx : imp_v_nx);
    end
`ifdef VERILATOR
    // SAFETY NET: the flop must equal the combinational mux at every clock
    always @(posedge clk) if (rst_n && ftu_q_r !== ftu_q_c) begin
      $display("FTU registered output MISMATCH t=%0t ftu_q_r=%04x expected=%04x has_latch=%b imp_i=%b", $time, ftu_q_r, ftu_q_c, has_latch, imp_i); $fatal(1);
    end
`endif
  end else begin : g_out_comb
    assign ftu_q_r = ftu_q_c;
    logic unused_nx; assign unused_nx = has_latch_nx | (|latch_nx) | (|irc_nx) | (|imp_v_nx) | imp_i_nx;
  end endgenerate
  always_ff @(posedge clk) begin
    if (!rst_n) begin latch <= 16'd0; has_latch <= 1'b0; end
    else if (en) begin
      if (ld_any) begin latch <= ld_val; has_latch <= 1'b1; end
      else if (use_bus | use_psw | new_ird) has_latch <= 1'b0;
    end
  end
endmodule
