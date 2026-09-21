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

// m68k_mvm_si.sv - MOVEM multi-register encoder. Source: App. B (`reset pren`: "advance encoder to the address of
// the next register to be accessed"; `enl` = {IRD6, END}), App. H (the mask enters with `ftu<-ab` in the blocks with pointer
// `rp`, the cycles write/read `rx` = reg[mvm]), order of the mask bits from the Motorola manual (bit 0 = D0 ... bit 15 = A7;
// with -(An) the order is reversed: bit 0 = A7 ... bit 15 = D0).
// docs/silicon-datapath.md §13; the branch variable `enl` itself is formed in m68k_branch_si.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_mvm_si #(parameter bit IR_REG = 1'b1,
  // 1 = the MOVEM index comes out of a flop (core); 0 = direct form (predec ? ~cur : cur).
  parameter bit RP_REG = 1'b1) (   // 1 = predec from `ir`, registered (core); 0 = combinational from ird (testbench)

  input  logic        clk, rst_n, en,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [NDEC_W-1:0] dec,
  input  logic [15:0] ird,
  input  logic [15:0] ir,           // instruction at the stage before ird: predec is decided here
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [15:0] mask_in,      // mask word from the bus (loaded with ftu<-ab)
  // the NEXT value of the index (already computed for the index flop).
  // It is needed by whoever must register something that depends on rp_reg (the bank index).
  output logic [3:0]  rp_next,
  output logic [3:0]  rp_reg,       // current register: 0-7 = D0-D7, 8-15 = A0-A7
  output logic        mvm_end       // mask exhausted (END)
);
  logic ld_mask, adv;
  assign ld_mask = dec[ND_FTU_FROM_LSB +: ND_FTU_FROM_W] == ND_FTU_FROM__AB[ND_FTU_FROM_W-1:0];
  assign adv     = dec[ND_RESET_PREN_LSB +: ND_RESET_PREN_W] != '0;
  logic [15:0] mask;
  logic [3:0]  low;          // index of the lowest bit at 1
  logic [3:0]  cur;          // register selected by the last 'reset pren' (push2/push6): it is the one that the following words read/write
  logic        predec;
  logic [3:0]  rp_reg_c;     // combinational form: used with IR_REG = 0 or RP_REG = 0 (testbench)
  logic        predec_fl;   // predec registered from ir; with IR_REG = 0 (testbench) predec is combinational from ird
  // predec is a PURE function of the instruction; a compare on ird would start the cone
  //   ird -> predec -> spv -> rxd_i -> raddr -> MLAB address -> rf_ab
  // of the register bank. It is decided on ir and registered at the instant
  // ird takes the value of ir: same value, same cycle, but the cone starts from a flop.
  // At ird = 0 (power-up and reset) (3'b000 == 3'b100) is 0, so the power-up value is 0.
  logic new_ird; assign new_ird = (dec[ND_IRD_FROM_LSB +: ND_IRD_FROM_W] == ND_IRD_FROM__P_IR[ND_IRD_FROM_W-1:0]);
  always_ff @(posedge clk) begin
    if (!rst_n) predec_fl <= 1'b0;
    else if (en && new_ird) predec_fl <= (ir[5:3] == 3'b100);
  end
  assign predec = IR_REG ? predec_fl : (ird[5:3] == 3'b100);
  always_comb begin
    low = 4'd0;
    for (int i = 15; i >= 0; i--) if (mask[i]) low = 4'(i);
    rp_reg_c = predec ? ~cur : cur;    // -(An): bit 0 = A7 (15), bit 15 = D0 (0)
    mvm_end = (mask == 16'd0);   // END: mask exhausted, combinational from the mask register
  end
  // the MOVEM index comes out of a flop instead of from the mux `predec ? ~cur : cur`, which would sit on
  // the cur -> index -> MLAB -> rf_ab cone: the register bank receives the index directly from a flop.
  // No assumption on the microcode: `cur` changes ONLY on (en & ~ld_mask & adv) and `predec_fl` ONLY
  // on (en & new_ird), so it is enough to reload on those events with the NEXT values of
  // both. By induction from reset (cur = 0 and predec = 0, so rp_reg = 0) the flop equals at
  // every instant exactly `predec ? ~cur : cur`: same value, same cycle.
  logic [3:0] cur_nx; logic pd_nx, rp_ld;
  assign cur_nx = (en & ~ld_mask & adv) ? low : cur;
  assign pd_nx  = (en & new_ird) ? (ir[5:3] == 3'b100) : predec_fl;
  assign rp_ld  = en & ((~ld_mask & adv) | new_ird);
  logic [3:0] rp_reg_q;
  always_ff @(posedge clk) begin
    if (!rst_n) rp_reg_q <= 4'd0;
    else if (rp_ld) rp_reg_q <= pd_nx ? ~cur_nx : cur_nx;

  end
  assign rp_reg = (IR_REG & RP_REG) ? rp_reg_q : rp_reg_c;   // flop (core) or combinational form (testbench)
  // the value that rp_reg will have after this edge: the same that is written into rp_reg_q if
  // the load happens, otherwise the current one. It is needed by whoever must register a signal that
  // depends on rp_reg (the index of the register bank), and it costs no new logic: the expression
  // `pd_nx ? ~cur_nx : cur_nx` and the gate `rp_ld` already exist for rp_reg_q.
  assign rp_next = rp_ld ? (pd_nx ? ~cur_nx : cur_nx) : rp_reg;
`ifdef VERILATOR
  // SAFETY NET: `mask & (mask - 1)` must equal `mask & ~(1 << low)` at every clock
  always @(posedge clk) if (rst_n && (mask & (mask - 16'd1)) !== (mask & ~(16'd1 << low))) begin
    $display("MOVEM mask lowest-bit clear MISMATCH t=%0t mask %h low %0d", $time, mask, low); $fatal(1);
  end
`endif
  always_ff @(posedge clk) begin
    if (!rst_n) begin mask <= 16'd0; cur <= 4'd0; end
    else if (en) begin
      if (ld_mask) mask <= mask_in;
      else if (adv) begin cur <= low; mask <= mask & (mask - 16'd1); end   // selects the next register and removes it from the mask. `mask & (mask - 1)` clears the lowest set bit: identical to `mask & ~(1 << low)` for every mask (0 -> 0 in both), without going through the encoder `low` and the decoder `1 << low`. Net below.
    end
  end
endmodule
