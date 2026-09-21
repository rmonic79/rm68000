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

// m68k_psw_si.sv - PSW status register of the silicon-structured datapath.
// Source: App. B (psw = condition code, interrupt level, trace, supervisor; psws = S bit), App. D, atoms of App. H:
//   psw<-ftu     : whole PSW from the FTU (RTE, MOVE to SR/CCR, stop ...)
//   psw<-(inl)   : interrupt level (I2:0) from the INL latch (interrupt entry)
//   psws+=+1     : S bit to 1 (exception entry)
//   flags X N Z V C: from the ALU (cc_out of m68k_alu_si, already equal to cc_in when the cell does not update) at every microcycle.
// docs/silicon-datapath.md §10.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_psw_si #(
  // 1 = the predicates arrive registered from the control store (core); 0 = computed here from
  // `dec` (testbench, which does not drive those ports).
  parameter bit EXT_PRED = 1'b1
) (
  input  logic        clk, rst_n, en,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [NDEC_W-1:0] dec,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [4:0]  cc_alu,      // X N Z V C from the ALU (cc_out output)
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [15:0] ftu_q,       // output of the FTU (undefined bits of the PSW ignored)
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [2:0]  inl,         // latched interrupt level
  input  logic        ccr_only,    // 1: the write from FTU touches only X N Z V C (byte forms: to CCR, RTR)
  output logic [15:0] psw,         // T . S . . I2 I1 I0 . . . X N Z V C
  output logic        sup,         // S bit
  input  logic        psws_p1,     // `psws+=+1` registered by the control store
  // the value that `sup | set_s_o` will have after this edge.
  output logic        sup_next,
  // only the `en` branch of sup_next: its consumer is a flop enabled by `en` (sup_e_q in the top), so
  // the `~en` branch is never sampled and the mux on `en` is an extra input without function.
  output logic        sup_nx_en,
  output logic        set_s_o,     // '+1 -> psws' in the current word: S already holds for the choice of SP (trap2: (sp) -> db -> au ; +1 -> psws)
  input  logic        g0_entry,    // entry into the bus/address error routine (en & a0s_sub): S <- 1, T <- 0 like trap2;
                                   //   App. H does not write it in the bser words; the die listing does, and the manual requires it (frame on the SSP also from user mode)
  output logic [4:0]  cc           // X N Z V C
);
  logic t, s;
  logic [2:0] il;
  assign psw = {t, 1'b0, s, 2'b00, il, 3'b000, cc};
  assign sup = s;
  logic ld_ftu, ld_inl, set_s;
  assign ld_ftu = dec[ND_PSW_FROM_LSB +: ND_PSW_FROM_W] == ND_PSW_FROM__FTU[ND_PSW_FROM_W-1:0];
  assign ld_inl = dec[ND_PSW_FROM_LSB +: ND_PSW_FROM_W] == ND_PSW_FROM__P_INL[ND_PSW_FROM_W-1:0];
  // set_s arrives registered from the control store: same value and same cycle (see
  // m68k_nstore_dec), so the compare is out of the path towards the bank index.
  assign set_s  = EXT_PRED ? psws_p1
                : (dec[ND_PSWS_CONST_LSB +: ND_PSWS_CONST_W] == ND_PSWS_CONST__PLUS1[ND_PSWS_CONST_W-1:0]);
  // g0_q: microcycle following the entry into the group 0 routine = bser1, the word that captures the SR into the FTU:
  //   S<-1/T<-0 at the same edge, so the previous SR goes into the frame (MC68000UM: internal copy of the SR, then S set, T cleared).
  logic g0_q;
  always_ff @(posedge clk) begin
    if (!rst_n) g0_q <= 1'b0;
    else if (en) g0_q <= g0_entry;
  end
  assign set_s_o = set_s | g0_q;
  // the value that the EFFECTIVE supervisor bit (sup | set_s_o) will have after this edge.
  // The priority is that of the block below: the last assignment wins, so exception and
  // interrupt beat the load from FTU. `set_s` and `g0_q` are atoms/flops, not asynchronous events
  // (set_s = dec[ND_PSWS_CONST] == PLUS1, or its registered copy), so everything can be derived here.
  // It is needed by whoever must register a signal that depends on sup_e: the index of the register bank.
  logic s_nx;
  assign s_nx = (set_s | g0_q | ld_inl) ? 1'b1
              : (ld_ftu & ~ccr_only)    ? ftu_q[13]
                                        : s;
  assign sup_next = en ? (s_nx | set_s | g0_entry) : (s | set_s_o);
  assign sup_nx_en = s_nx | set_s | g0_entry;   // = sup_next when en = 1
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      t <= 1'b0; s <= 1'b1; il <= 3'b111; cc <= 5'd0;       // reset: supervisor, level 7 (Motorola manual)
    end else if (en) begin
      cc <= cc_alu;
      if (ld_ftu) begin
        cc <= ftu_q[4:0];
        if (!ccr_only) begin t <= ftu_q[15]; s <= ftu_q[13]; il <= ftu_q[10:8]; end   // only the word forms (to SR, RTE)
      end
      if (ld_inl) begin il <= inl; s <= 1'b1; t <= 1'b0; end   // interrupt exception: S=1, T=0, I=level (Motorola manual)
      if (set_s | g0_q) begin s <= 1'b1; t <= 1'b0; end         // exception (trap2; group 0 at the end of bser1): S=1, T=0
    end
  end
endmodule
