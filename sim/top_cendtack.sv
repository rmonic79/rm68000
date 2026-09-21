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

// top_cendtack.sv — the clock-enable and DTACK generator m68k_cen_dtack on its own, for tb_cendtack.cpp
// (docs/clock-dtack.md, section 6.1). Instance parameters #(.W(8), .RECOVERY(1)) and WD at its default, 6: a debt
// counter of 14 bits (section 4.1). The bench drives the processor's strobes and the memory map's bus_cs/bus_busy, and
// reads cpu_cen, cpu_cenb and DTACKn. bus_legit, bus_ack, wait2 and wait3 at 0 (section 4.1); VPAn at 1, inactive: the
// bench runs no VPA cycle; fave, fworst, cen_nx and cenb_nx unconnected.
`default_nettype none
module top_cendtack (
  input  wire       clk,
  input  wire       rst,
  input  wire [6:0] num,
  input  wire [7:0] den,
  input  wire       as_n,
  input  wire [1:0] ds_n,       // {UDSn, LDSn}
  input  wire       bus_cs,
  input  wire       bus_busy,
  output wire       cpu_cen,
  output wire       cpu_cenb,
  output wire       dtack_n
);
  m68k_cen_dtack #(.W(8), .RECOVERY(1)) u_dut (
    .clk(clk), .rst(rst), .num(num), .den(den),
    .cpu_cen(cpu_cen), .cpu_cenb(cpu_cenb),
    .bus_cs(bus_cs), .bus_busy(bus_busy), .bus_legit(1'b0), .bus_ack(1'b0),
    .ASn(as_n), .DSn(ds_n), .VPAn(1'b1), .wait2(1'b0), .wait3(1'b0),
    .DTACKn(dtack_n), .fave(), .fworst(), .cen_nx(), .cenb_nx());
endmodule
`default_nettype wire
