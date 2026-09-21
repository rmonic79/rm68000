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

// m68k_ir_si.sv - IRC -> IR -> IRD instruction queue of the silicon-structured core.
// Source: docs/microcode-structure.md §2, docs/silicon-datapath.md §14 (patent FIG. 4/5): IRC = next word from the data bus
// (nano atom `irc<-edb`), IR = instruction being decoded (I bit of the microword: IRC -> IR), IRD = instruction being
// executed (nano atom `ird<-(ir)`). In the same microcycle IR takes the PREVIOUS value of IRC (pipeline).
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_ir_si (
  input  logic        clk, rst_n, en,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [NDEC_W-1:0] dec,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic        ir_load,     // bit 0 (I) of the current microword (m68k_useq)
  input  logic        bit4,        // bit 4: with bit 0 it forms db/dbi/dbc/dbl (patent col. 17): IR is loaded for dbi, dbc, dbl
  input  logic        fmt,         // format II: bit 4 belongs to the CBC and does not count
  input  logic [15:0] edb,         // external data bus (word read)
  output logic [15:0] irc, ir, ird,
  // row class of the decoder, ALREADY registered: computed from irc and loaded at the
  // same edge as `ir <= irc`, so it always equals cls_of(ir[15:6]) in the same cycle (the
  // non-blocking assignments read irc BEFORE the edge, as ir does). It removes the three levels
  // of cls from the head of every ir -> a1/illegal path.
  output logic [6:0]  cls_q
);
  logic ld_irc, ld_ird, ld_ir;
  assign ld_ir = ir_load | (bit4 & ~fmt);
  assign ld_irc = dec[ND_IRC_FROM_LSB +: ND_IRC_FROM_W] == ND_IRC_FROM__EDB[ND_IRC_FROM_W-1:0];
  assign ld_ird = dec[ND_IRD_FROM_LSB +: ND_IRD_FROM_W] == ND_IRD_FROM__P_IR[ND_IRD_FROM_W-1:0];
  logic [6:0] cls_n, cls_zero;
  m68k_a123_cls u_cls  (.hi(irc[15:6]), .cls(cls_n));     // the class of the NEXT ir
  m68k_a123_cls u_cls0 (.hi(10'd0),     .cls(cls_zero));  // the class of ir = 0 (reset): constant
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      irc <= 16'd0; ir <= 16'd0; ird <= 16'd0; cls_q <= cls_zero;
    end else if (en) begin
      if (ld_irc)  irc <= edb;
      if (ld_ir)   begin ir <= irc; cls_q <= cls_n; end
      if (ld_ird)  ird <= ir;
    end
  end
`ifdef VERILATOR
  // safety net: at every clock cls_q must equal the class of ir.
  logic [6:0] cls_ir;
  m68k_a123_cls u_cls_chk (.hi(ir[15:6]), .cls(cls_ir));
  always @(posedge clk) if (rst_n && cls_q !== cls_ir) begin
    $display("IR row class MISMATCH t=%0t cls_q=%0d cls(ir)=%0d ir=%04x", $time, cls_q, cls_ir, ir); $fatal(1);
  end
`endif
endmodule
