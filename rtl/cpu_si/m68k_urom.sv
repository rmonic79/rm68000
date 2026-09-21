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

// m68k_urom.sv - micro ROM of the silicon-structured 68000 core.
// Source: docs/microcode-structure.md §2-3 (US 4,325,121, FIG. 4/5/8) and die bits
// (docs/microcode-sources.md §3). Contents: rtl/cpu_si/m68k_urom.mem.
// 1024 x 17 bits, synchronous read with enable (output = output latch 74 of the patent): the word
// read in cycle N provides the address for cycle N+1. Structure inferable as M10K with registered output.
module m68k_urom #(
  parameter MEM_FILE = "m68k_urom.mem"
) (
  input  logic        clk,
  input  logic        en,      // advance by one micro cycle
  input  logic [9:0]  addr,
  output logic [16:0] q
);
  // M10K: 2 blocks per CPU (1024x10), same latency (address registered at en, data after the edge, held until
  // the next en); uw feeds useq and the word flags, never ph0: the M10K tco falls on the en cone. Init: $readmemb -> .mif.
  (* ramstyle = "M10K" *) logic [16:0] rom [0:1023];
  initial $readmemb(MEM_FILE, rom);
  always_ff @(posedge clk) if (en) q <= rom[addr];
endmodule
