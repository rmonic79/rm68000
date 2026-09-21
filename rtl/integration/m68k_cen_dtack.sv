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

// m68k_cen_dtack.sv — TWO-PHASE clock-enable generator (cpu_cen = phi1, cpu_cenb = phi2) + DTACK with cycle recovery.
//
// OUR module, clean-room, written FROM docs/clock-dtack.md: the section numbers (§N) in the comments
// refer to that document. Sources: MC68000 User's Manual (DTACK sampling at the end of S4, wait states in whole periods,
// TAS with two handshakes), our RTL (rtl/cpu_si) and the AT-THE-PINS behaviour of a reference generator (third party,
// GPL-3), measured by the black-box testbench sim/tb_cendtack.cpp with that module OUTSIDE the repo. It does NOT derive
// from its logic: of the reference module only the port names and the instance parameters are used (§4.1),
// together with the difference classes read at the first differing clock of each testbench run (§2, §6.1).
//
// Target: IDENTICAL to the reference generator at the pins, clock by clock, on cpu_cen, cpu_cenb and DTACKn, at all six
// frequencies (den 12/8/6/4/3/2) and in the standard testbench conditions. Intended differences: rule R2, where a pulse
// counts as lost only while the processor is really waiting (it matters only when the memory looks busy while the
// processor is not waiting: testbench modes MAPLAT=0 and BUSYBACK=1), and the VPAn input of R1 (VPAn = 1 behaves like the
// reference). Same ports as the reference plus VPAn: instance parameters W and RECOVERY (§4.1, §5); bus_legit, bus_ack,
// wait2, wait3 tied to zero and without logic; fave/fworst not connected at the top, here at zero.
//
// Rules (derived at the pins, §2 and §4; R2 adds the condition on the processor waiting, see Target):
//  R0 divider (§4.2): acc += 2*num at every clock; nominal pulse when acc + 2*num > den (STRICT comparison), then acc -= den.
//     Compared with the >= comparison it moves the first pulse by one clock for even den, and at den 3 it gives the alternation 2,1,2,1.
//  R1 REGISTERED DTACKn (§4.4): low at the edge following the first edge that samples the active cycle (AS low with at least one DS
//     low) provided that at that edge bus_cs & bus_busy is low; with bus_busy low, at the edge that samples it low; once
//     low it stays low; high at the edge that samples the rise of AS or of both DS. Never while VPAn is low: an
//     M6800/autovector cycle is ended by the processor on E, "DTACK and VPA should never be simultaneously asserted"
//     (MC68000UM 5.1.4 p. 5-10); the reference generator has no VPA input (tie VPAn to 1 to reproduce it).
//  R2 debt (§4.5): +1 for each pulse (cpu_cen or cpu_cenb, already issued at the previous edge) if in the clock AFTER that edge
//     the processor is really waiting: bus cycle already active at the previous edge, DTACKn not yet given and memory
//     busy (bus_cs & bus_busy). The pulse is issued anyway; no counting of wait states.
//  R3 recovery (§4.6): with debt > 0, an extra pulse at every clock with no nominal pulse due and with AS HIGH (bus
//     free): never inside a bus cycle; cen/cenb alternation always respected; the debt drops by one per pulse.
//  R4 debt counter (§4.7): W+WD bits (14 with W = 8, WD = 6: at most 16383 pulses), count MODULO 2^(W+WD): beyond
//     the maximum it OVERFLOWS (the next lost pulse brings the debt back to 0 and the accumulated amount is lost), no saturation.

`default_nettype none

module m68k_cen_dtack #(
  parameter W           = 8,        // width of num/den
  parameter RECOVERY    = 1,        // 1 = recovery of the cycles lost in stalls; 0 = DTACK only
  parameter WD          = 6         // debt counter bits beyond W: width W+WD, count modulo 2^(W+WD) (R4)
) (
  input  wire          clk,
  input  wire          rst,          // active high
  input  wire [W-2:0]  num,
  input  wire [W-1:0]  den,
  output reg           cpu_cen,      // phi1
  output reg           cpu_cenb,     // phi2
  input  wire          bus_cs,       // access to the "special" memory (e.g. SDRAM) — from the memory map (§4.1)
  input  wire          bus_busy,     // memory busy: holds back DTACK
  input  wire          bus_legit,    // tied to zero: no logic (§4.1)
  input  wire          bus_ack,      // tied to zero: no logic
  input  wire          ASn,
  input  wire [1:0]    DSn,          // {UDSn, LDSn}
  input  wire          VPAn,         // M6800/autovector cycle (VPA low): no DTACK, the processor ends the cycle on E (MC68000UM 5.1.4 p. 5-10); 1 where VPA does not exist
  input  wire          wait2,        // tied to zero: no logic
  input  wire          wait3,
  output reg           DTACKn,       // registered (R1)
  output wire [W-1:0]  fave,         // debug statistics (ports of the reference module): unconnected at the top (§4.1), here at zero
  output wire [W-1:0]  fworst,
  // the D inputs of the cpu_cen/cpu_cenb flops, for whoever wants to register the phases already gated with their own signal
  // (m68k_cpu_dt_si): cpu_cen(t) = cen_nx(t-1). No extra logic, no change at the pins.
  output wire          cen_nx, cenb_nx
);

  assign fave   = '0;
  assign fworst = '0;
  wire unused_ok = bus_legit | bus_ack | wait2 | wait3;

  // --- R0: fractional divider, strict comparison -----------------------------------------------------------------
  wire [W:0] num2        = {1'b0, num, 1'b0};               // 2*num, extended to W+1 bits
  wire [W:0] den_e       = {1'b0, den};
  reg  [W:0] acc;
  wire [W:0] acc_sum     = acc + num2;
  wire       nominal_due = (den != '0) && (acc_sum > den_e);
  wire [W:0] acc_after   = nominal_due ? (acc_sum - den_e) : acc_sum;
  wire [W:0] acc_next    = (acc_after > den_e) ? den_e : acc_after;   // guard: 2*num > den is not a supported setting

  // --- R1: active cycle and registered DTACK --------------------------------------------------------------------
  wire as_low   = ~ASn;
  wire ds_low   = ~(DSn[1] & DSn[0]);
  wire active   = as_low & ds_low;                           // AS low with at least one DS low (current inputs)
  wire hold     = bus_cs & bus_busy;                         // memory busy (current inputs)
  reg  active_q;                                             // active cycle seen at the previous edge
  wire dtack_assert = active_q & active & ~hold & VPAn;      // VPAn: one input more on the next state of DTACKn (6: active, active_q, bus_cs, bus_busy, DTACKn, VPAn), still one LUT6
  // next value: high if the cycle is not active (release at the edge that samples AS or DS high); otherwise low if
  // already low (hold) or if the assert conditions are true in the sampled clock
  wire dtack_n_next = ~active ? 1'b1 : (DTACKn ? ~dtack_assert : 1'b0);

  // --- R2/R3: debt and recovery ---------------------------------------------------------------------------------
  localparam DW = W + WD;                                    // width of the debt counter (R4)
  reg  [DW-1:0] debt;
  reg           phase;                                       // 0 = the next pulse is cpu_cen, 1 = cpu_cenb
  wire supp  = (RECOVERY != 0) & (debt != '0) & ~nominal_due & ~as_low;   // only with the bus free (R3)
  wire pulse = nominal_due | supp;
  assign cen_nx = pulse & ~phase; assign cenb_nx = pulse & phase;   // next values of cpu_cen/cpu_cenb (ports cen_nx/cenb_nx)
  // R2 (§4.5): a pulse is lost only if the processor is really waiting: bus cycle already active at the previous
  // edge (active_q), DTACK not yet given (DTACKn) and memory busy (hold). Memory busy alone is not enough: the
  // map can show it when the processor is not waiting: one clock after it has raised AS, in the same clock in which
  // DS falls (combinational map) or again after DTACK (pulsed external DTACK). Every extra loss counted would come
  // back as an extra recovery pulse and the CPU would run faster than nominal. Testbench: sim/tb_cendtack.cpp,
  // modes MAPLAT=0 and BUSYBACK=1.
  wire miss  = (RECOVERY != 0) & (cpu_cen | cpu_cenb) & hold & active_q & DTACKn;   // pulse of the previous edge while the processor is waiting (R2)
  // count modulo 2^DW: +1 per lost pulse, -1 per recovery pulse (supp implies debt != 0), overflow beyond the maximum
  wire [DW-1:0] debt_next = debt + {{(DW-1){1'b0}}, miss} - {{(DW-1){1'b0}}, supp};

  always_ff @(posedge clk) begin
    if (rst) begin
      acc <= '0; phase <= 1'b0; cpu_cen <= 1'b0; cpu_cenb <= 1'b0;
      active_q <= 1'b0; DTACKn <= 1'b1; debt <= '0;
    end else begin
      acc      <= acc_next;
      phase    <= pulse ? ~phase : phase;
      cpu_cen  <= pulse & ~phase;
      cpu_cenb <= pulse &  phase;
      active_q <= active;
      DTACKn   <= dtack_n_next;
      debt     <= debt_next;
    end
  end

endmodule

`default_nettype wire
