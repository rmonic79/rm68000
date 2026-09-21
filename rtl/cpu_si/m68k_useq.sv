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

// m68k_useq.sv - address selector of the micro/nano ROM (block 64 of patent US 4,325,121, FIG. 4).
// Source: docs/microcode-structure.md §2 (mux 112/114/117/118) and §3 (microword format).
// Combinational: from the current microword (registered output of the micro ROM) and from the inputs of the decoders
// (A1/A2/A3), of the exception block (A0/A0S) and of the branch control (C1/C0) it produces the address of the next
// word for BOTH ROMs. No runtime decision beyond those of the silicon.
module m68k_useq (
  input  logic [16:0] uw,       // current microword
  input  logic [9:0]  a1,       // decoder A1 (IR): instruction start
  input  logic [9:0]  a2,       // decoder A2 (IR)
  input  logic [9:0]  a3,       // decoder A3 (IR)
  input  logic        a0_sub,   // A0SUB: exception at end of instruction (interrupt/trace) in place of A1
  input  logic [9:0]  a0,
  input  logic        rst_n,    // in reset base/base_na are already 0 here: the mask is in the candidate mux, not in a downstream AND
  input  logic        a0s_sub,  // A0SUBI: immediate override (address error, bus error, reset)
  input  logic [9:0]  a0s,
  input  logic [1:0]  c1c0,     // outcome of the conditional branch (C1 = bit 7, C0 = bit 6 of the address)
  output logic [9:0]  uaddr,    // next address
  output logic        ir_load,  // bit 0 I: IRC -> IR
  output logic        fmt,      // bit 1: 0 direct jump, 1 conditional jump
  output logic [1:0]  ty,       // format I, bits 2-3: 00 NMA, 01 A1, 10 A2, 11 A3
  output logic        bit4,     // format I, bit 4 (with I: db/dbi/dbc/dbl)
  output logic [4:0]  cbc,      // format II, bits 2-6: condition code
  output logic [1:0]  fc,       // bits 15-16: function code (00 n/u, 01 d, 10 i, 11 iack)
  output logic [9:0]  base_o, // next address BEFORE the immediate a0s override: to the ORIGIN in M10K one clock before en
  // the same `base` with a0_sub forced to 0, and the selector: this way the ORIGIN table can
  // look up base_noa0 and a0 IN PARALLEL and choose afterwards. base_o stays for whoever uses it.
  output logic [9:0]  base_noa0,
  output logic        a0_sel,
  output logic        is327_o   // uaddr == 10'h327, computed on the candidates before the mux (same selectors as uaddr)
);
  // 10-bit NMA: physical permutation of the die bits (verified on all 397 jumps):
  // A0<-b11 A1<-b12 A2<-b7 A3<-b8 A4<-b9 A5<-b10 A6<-b5 A7<-b6 A8<-b13 A9<-b14
  logic [9:0] nma;
  assign nma = {uw[14], uw[13], uw[6], uw[5], uw[10], uw[9], uw[8], uw[7], uw[12], uw[11]};
  assign ir_load = uw[0];
  assign fmt     = uw[1];
  assign ty      = uw[3:2];
  assign bit4    = uw[4];
  assign cbc     = uw[6:2];
  assign fc      = uw[16:15];

  logic [9:0] base, base_na;
  always_comb begin
    if (fmt) base = {nma[9:8], c1c0, nma[5:0]};          // NMBA: bit 7/6 <- C1/C0 (mux 117)
    else begin
      unique case (ty)                                    // mux 114 (TY decoded)
        2'd0: base = nma;
        2'd1: base = a0_sub ? a0 : a1;                    // mux 112
        // base_na = base with a0_sub = 0 (see below)
        2'd2: base = a2;
        default: base = a3;
      endcase
    end
    if (!rst_n) base = 10'd0;                              // reset mask inside the mux, not a level of its own downstream
    uaddr = a0s_sub ? a0s : base;                         // mux 118
  end
  assign base_o = base;
  // base_na: identical to base except in the ty=1 branch, where it takes a1 even with a0_sub.
  always_comb begin
    if (fmt) base_na = {nma[9:8], c1c0, nma[5:0]};
    else begin
      unique case (ty)
        2'd0: base_na = nma;
        2'd1: base_na = a1;
        2'd2: base_na = a2;
        default: base_na = a3;
      endcase
    end
    if (!rst_n) base_na = 10'd0;                           // reset mask, as for base
  end
  assign base_noa0 = base_na;
  assign a0_sel = ~fmt & (ty == 2'd1) & a0_sub;   // when base == a0
  // the comparison with 327 distributed over the uaddr mux: comparisons on the candidates (flops or constants), then the same selectors
  logic is327_base;
  always_comb begin
    if (fmt) is327_base = ({nma[9:8], c1c0, nma[5:0]} == 10'h327);
    else begin
      unique case (ty)
        2'd0: is327_base = (nma == 10'h327);
        2'd1: is327_base = a0_sub ? (a0 == 10'h327) : (a1 == 10'h327);
        2'd2: is327_base = (a2 == 10'h327);
        default: is327_base = (a3 == 10'h327);
      endcase
    end
    is327_o = a0s_sub ? (a0s == 10'h327) : is327_base;
  end
endmodule
