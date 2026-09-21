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

// m68k_a123_top.sv - testbench top of the A1/A2/A3 decoder in the CORE configuration: EXT_CLS = 1, with the row class
// computed by m68k_a123_cls on ir[15:6] as m68k_ir_si does (which registers it on the same edge on which it loads ir, with a safety net);
// EXT_CLS = 0 = internal combinational form (second configuration).
module m68k_a123_top #(parameter bit EXT_CLS = 1'b1) (
  input  logic [15:0] ir,
  output logic [9:0]  a1, a2, a3,
  output logic        illegal, priv
);
  logic [6:0] cls;
  m68k_a123_cls u_cls (.hi(ir[15:6]), .cls(cls));
  m68k_a123_si #(.EXT_CLS(EXT_CLS)) u_a123 (.ir(ir), .cls_in(cls), .a1(a1), .a2(a2), .a3(a3), .illegal(illegal), .priv(priv));
endmodule
