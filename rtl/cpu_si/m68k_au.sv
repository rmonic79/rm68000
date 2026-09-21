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

// m68k_au.sv - AU (address arithmetic unit) of the silicon-structured datapath.
// Source: docs/silicon-datapath.md §7 (rules from the die listing: AU = DB + [AB] + constant).
// Inputs from the section fabric (docs/silicon-datapath.md §5): DB always (undriven sections = 0), AB as second operand if the
// microcode asks for it, sign extension on the input when the atom is 'e'. Constant from the au+= field.
// The AU register (32 bits) is loaded at every word with an au<- or au+= atom.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_au (
  input  logic        clk, rst_n, en,
  input  logic        op_ld,     // phase 1 of the microcycle: samples the operands (buses stable from ph0; the sum is needed at en)
  input  logic        st_ld,     // phase 2: registered low sum (two-stage AU, static phases of the microcycle)
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [NDEC_W-1:0] dec,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [15:0] db_h, db_l, ab_h, ab_l,
  input  logic        db_h_drv, db_l_drv,   // driven DB sections: if none, operand A is the AU itself (the bus holds the value)
  input  logic        is_byte,   // byte instruction: the double constants (+1,+2 / -1,-2) are 1
  input  logic        au_sp,     // the register on the DB input is A7: for bytes it stays 2 (Motorola manual, aligned stack)
  output logic [31:0] au
);
  logic [ND_AU_FROM_W-1:0] f;
  assign f = dec[ND_AU_FROM_LSB +: ND_AU_FROM_W];
  logic s_ab, s_abe, s_dbe;
  assign s_ab  = f[ND_AU_FROM__AB];
  assign s_abe = f[ND_AU_FROM__ABE];
  assign s_dbe = f[ND_AU_FROM__DBE];
  logic [ND_AU_CONST_W-1:0] k;
  assign k = dec[ND_AU_CONST_LSB +: ND_AU_CONST_W];
  logic any_src;
  assign any_src = (|f) | (k != '0);
  logic [31:0] a, b; logic [3:0] c;   // constant: -4..+4 on 4 signed bits
  logic [31:0] a_q, b_q; logic [3:0] c_q;   // operands sampled at ph1 (static phase)
  always_ff @(posedge clk) begin
    if (!rst_n) begin a_q <= '0; b_q <= '0; c_q <= 4'd0; end
    else if (op_ld) begin a_q <= a; b_q <= b; c_q <= c; end
  end
  always_comb begin
    a = s_dbe ? {{16{db_l[15]}}, db_l} : {db_h, db_l};                       // operand A: always DB
    if (!db_h_drv && !db_l_drv) a = au;                                       // DB not driven (e.g. rstp6): AU holds the value
    b = (s_ab | s_abe) ? (s_abe ? {{16{ab_l[15]}}, ab_l} : {ab_h, ab_l}) : 32'd0;   // operand B: AB if requested
    c = 4'd0;
    if      (k == ND_AU_CONST__PLUS1 [ND_AU_CONST_W-1:0]) c = 4'd1;
    else if (k == ND_AU_CONST__PLUS2 [ND_AU_CONST_W-1:0]) c = 4'd2;
    else if (k == ND_AU_CONST__PLUS4 [ND_AU_CONST_W-1:0]) c = 4'd4;
    else if (k == ND_AU_CONST__MINUS1[ND_AU_CONST_W-1:0]) c = 4'hf;
    else if (k == ND_AU_CONST__MINUS1_OR_MINUS2[ND_AU_CONST_W-1:0]) c = (is_byte & ~au_sp) ? 4'hf : 4'he;   // -1,-2 -> au
    else if (k == ND_AU_CONST__PLUS1_OR_PLUS2[ND_AU_CONST_W-1:0]) c = (is_byte & ~au_sp) ? 4'd1 : 4'd2;                     // +1,+2 -> au
    else if (k == ND_AU_CONST__MINUS2[ND_AU_CONST_W-1:0]) c = 4'he;
    else if (k == ND_AU_CONST__MINUS4[ND_AU_CONST_W-1:0]) c = 4'hc;
  end
  // stage 1 (ph2): 17-bit low sum; stage 2 (en): high sum with the registered carry
  logic [16:0] lo_s; logic [15:0] hi_s;
  always_ff @(posedge clk) if (st_ld) lo_s <= {1'b0, a_q[15:0]} + {1'b0, b_q[15:0]} + {1'b0, {12{c_q[3]}}, c_q};   // constant extended to 16 bits: lo_s[16] = true carry towards the high half
  assign hi_s = a_q[31:16] + b_q[31:16] + {16{c_q[3]}} + {15'd0, lo_s[16]};
  always_ff @(posedge clk) begin
    if (!rst_n) au <= 32'd0;
    else if (en && any_src) au <= {hi_s, lo_s[15:0]};
  end
endmodule
