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

// m68k_branch_si.sv - conditional branch unit (US 4,325,121 FIG. 13/14, App. C/D/E).
// From the CBC field of the microword (format II) and from the condition variables it produces C1C0 (bits 7:6 of the next
// address, c1c0 input of m68k_useq). Code x value table SI_BRANCH_TABLE in m68k_branch_pkg.sv (App. C, verified against
// the 70 conditional branches of the die listing; docs/silicon-datapath.md §11). cc condition of Bcc/Scc/DBcc: App. E on IRD[11:8] and PSW.
import m68k_branch_pkg::*;
module m68k_branch_si (
  input  logic [4:0]  cbc,
  input  logic        irc11,      // IRC bit 11 (i11: index sign extension)
  input  logic        auz,        // AU == 0
  input  logic        locz,       // local copy of ALU == 0 (ze)
  input  logic        dcr4,       // bit 4 of the bit decoder (d4)
  input  logic        mend,       // END of the multi-register encoder (enl)
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [15:0] ird,        // used: [11:8] cc condition, [8] ms0/m01, [6] enl
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [1:0]  alue10,     // ALUE[1:0]
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [4:0]  cc,         // X N Z V C of the PSW (X does not enter the conditions)
  /* verilator lint_on UNUSEDSIGNAL */
  output logic [1:0]  c1c0
);
  logic n, z, v, c;
  assign {n, z, v, c} = cc[3:0];
  logic cond;   // App. E
  always_comb begin
    case (ird[11:8])
      4'h0: cond = 1'b1;                       // T
      4'h1: cond = 1'b0;                       // F
      4'h2: cond = ~z & ~c;                    // HI
      4'h3: cond = z | c;                      // LS
      4'h4: cond = ~c;                         // CC
      4'h5: cond = c;                          // CS
      4'h6: cond = ~z;                         // NE
      4'h7: cond = z;                          // EQ
      4'h8: cond = ~v;                         // VC
      4'h9: cond = v;                          // VS
      4'ha: cond = ~n;                         // PL
      4'hb: cond = n;                          // MI
      4'hc: cond = (n & v) | (~n & ~v);        // GE
      4'hd: cond = (~n & v) | (n & ~v);        // LT
      4'he: cond = (~z & n & v) | (~z & ~n & ~v);   // GT
      default: cond = z | (~n & v) | (n & ~v); // LE
    endcase
  end
  logic [3:0] val;
  always_comb begin
    case (cbc[3:0])
      4'h0: val = {3'b000, irc11};
      4'h1: val = {3'b000, auz};
      4'h2: val = {3'b000, c};
      4'h3: val = {3'b000, z};
      4'h4: val = {2'b00, n, z};               // nz1
      4'h5: val = {3'b000, n};
      4'h6: val = {2'b00, n, z};               // nz2
      4'h7: val = {2'b00, ird[8], alue10[0]};  // ms0
      4'h8: val = {auz, ird[8], alue10};       // m01
      4'h9: val = {3'b000, cond};              // cc
      4'ha: val = {3'b000, locz};              // ze
      4'hb: val = {2'b00, n | z, v};           // nv (CHK, only use): 'n' also holds for Dn = limit (Dn - ub = 0: no trap; manual)
      4'hc: val = {3'b000, dcr4};              // d4
      4'hd: val = {3'b000, v};
      4'he: val = {2'b00, ird[6], mend};       // enl: bit1 = long (ird[6]), bit0 = end of list (die: popm2 -> 3ac word / 3ec long / 32c end)
      default: val = 4'd0;
    endcase
    c1c0 = SI_BRANCH_TABLE[{cbc, val}];
  end
endmodule
