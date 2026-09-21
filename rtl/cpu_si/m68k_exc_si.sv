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

// m68k_exc_si.sv - exception block (US 4,325,121 FIG. 4/5 block 66: latches of the special conditions in two groups, decoder of
// A0/A0S, encoder of the vector number TVN). Sources: patent col. 12-14 and 17 (dbi = updates TVN, dbl = partial
// update = group 1 only), Motorola manual (vectors, priority: reset > bus error > address error > trace > interrupt >
// illegal/privilege/line A-F; TRAP #n = 32+n, TRAPV 7, CHK 6, divide by zero 5, autovector 24+level).
// Entries of the die: reset rstp1 000, bus/address error bser1 003, trace trac1 1c0, interrupt itlx1 1c4, trap1 1d0.
// Group 1 (immediate -> A0S, A0SUBI): reset, bus error, address error. Group 2 (at the end of the instruction -> A0 at the A1 dispatch, A0SUB):
// trace pending (T at the start of the instruction), interrupt pending (IPL > mask or level 7), illegal/privilege/line A-F (from the IR).
// docs/silicon-datapath.md §16; behaviour of each exception: docs/exceptions.md.
module m68k_exc_si #(
  // 1 = line A/F and privilege arrive registered from cpu_si (core); 0 = computed here from
  // `ir` (testbench, which does not drive those ports).
  parameter bit EXT_PRED = 1'b1,
  // 1 = the vector of the instruction (TRAP #n / TRAPV / CHK / DIV / 4) comes out of a flop decided on `ir` and loaded
  // at `en & ld_ird` (core); 0 = combinational from `ird` (testbench, which does not drive ld_ird).
  parameter bit TVN_REG = 1'b1
) (
  input  logic        clk, rst_n, en,
  input  logic        en_clk2,      // cen2: phi2 (three-stage IPL synchronizer)
  input  logic        upd_full,     // dbi word (bit0=1, bit4=0): updates both latch groups (start of instruction)
  input  logic        upd_part,     // dbl word (bit0=1, bit4=1): updates group 1 only
  input  logic        dispatch,     // word with TY = A1: it is the moment in which A0 can replace A1
  input  logic        rst_req,      // reset request (from the pin)
  input  logic        berr_ev,      // bus error: done of the cycle terminated by BERR (busctl, once)
  input  logic        aerr_ev,      // address error (word/long access to an odd address)
  input  logic [2:0]  ipl,          // requested interrupt level (pin, active high here)
  input  logic        iack_vec,     // IACK cycle terminated by DTACK: vector number from the bus (vec_num); with VPA the autovector stays
  input  logic        iack_spur,    // IACK cycle terminated by BERR: spurious interrupt, vector 24 (MC68000UM 6.3.2, 6.3.4)
  input  logic [7:0]  vec_num,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [15:0] psw,          // used: T (15), S (13), I2:0 (10:8)
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic        illegal,      // from the A1/A2/A3 decoder: no entry for IR
  input  logic        priv_ir,      // privileged according to the A1/A2/A3 decoder, registered like illegal
  input  logic        linea, linef, // line A / line F, registered like illegal
  input  logic [15:0] ir,           // instruction at the dispatch (line A/F, privilege)
  input  logic [15:0] ird,          // instruction executing (TRAP #n, TRAPV, CHK, DIV for the vector)
  input  logic        ld_ird,       // the current word loads ird <- ir (dec IRD_FROM = (ir)), from the top
  input  logic        stop_w,       // dispatch of word 327 (aaa01) of the executed STOP: entry into the 3fc wait (loop 3fc/3fd)
  output logic        g0_busy,      // bus/address error routine in progress (from the entry to its final dispatch): double fault in the busctl
  output logic        tvn_use,      // 1 while a group 0 or 1 exception is being processed (vector from tvn_r), 0 in an instruction or a group 2 exception: I/N bit of the group 0 frame (MC68000UM 6.3.9.1 p. 6-16)
  output logic        a0s_sub,      // immediate override (A0SUBI)
  output logic [9:0]  a0s,
  output logic        a0_sub,       // end-of-instruction override (A0SUB), valid with dispatch
  output logic [9:0]  a0,
  output logic [7:0]  tvn,          // current vector number (trap latch)
  output logic [2:0]  inl           // latched interrupt level
);
  // A_TRACE = 0e3 like A_TRAP: the trace enters through dvur2 -> dvur3 -> trap3 (content of trac1/trac2 of the die,
  // App. H); the 3a3 entry of this nano ROM is incomplete (frame written at PC-6, S/T not updated) and is not used.
  localparam logic [9:0] A_RESET = 10'h000, A_BSER = 10'h003, A_TRACE = 10'h0e3, A_INT = 10'h1c4, A_TRAP = 10'h0e3, A_STOPW = 10'h3fc;   // 0e3 = dvur2: instruction exceptions via the divide-by-zero path of the die (dvur2 -> dvur3 -> trap3 = content of trac1/trac2), one microword fewer than illg1/illg2 (3ff/3fe, which stay in ROM but are not selected)
  logic t_bit, s_bit; logic [2:0] imask;
  assign t_bit = psw[15]; assign s_bit = psw[13]; assign imask = psw[10:8];
  // group 1: immediate
  logic rst_l, berr_l, aerr_l;
  // berr_ev (busctl: done of the cycle terminated by BERR, once) also enters combinationally: the en of the 't'/'f' word that waits for
  // the cycle coincides with the done and the registered berr_l would not be visible yet.
  assign a0s_sub = rst_l | berr_l | berr_ev | aerr_l;
  assign a0s = rst_l ? A_RESET : A_BSER;
  // group 2: at the end of the instruction
  logic trace_l, int_l;
  logic int_req;
  logic ipl7_q, nmi_pend;   // level 7: pending from the transition to 7 until the recognition
  // IPL synchronizer with ONE stage on phi2 (cen2) + value at the START of the current word (ipl_w, loaded at every en with
  // the ipl_s1 of 4 ticks earlier). The request is sampled at the dbi word (upd_full) with ipl_w: the interrupt is taken at the boundary
  // if the IPL is high at the phi2 tick <= T_start(dbi) - 4. Measured on the reference core: in a BRA loop (dbi = 363) the
  // threshold is 4 ticks before the start of the word; on MOVE.W D1,(A0)+ the dbi word (2FE) lasts 16 ticks because it waits
  // for the write, and an IPL raised 3 ticks before its start is not taken. Sampling at the END of the dbi word with three
  // stages (end - 12) would match only for 8-tick words.
  logic [2:0] ipl_s1, ipl_w;
  always_ff @(posedge clk) begin
    if (!rst_n) begin ipl_s1 <= 3'd0; ipl_w <= 3'd0; end
    else begin
      if (en_clk2) ipl_s1 <= ipl;
      if (en) ipl_w <= ipl_s1;   // same edge as cen2: takes the previous ipl_s1 (4 ticks before the en)
    end
  end
  assign int_req = nmi_pend | (ipl_w > imask);
  always_ff @(posedge clk) begin
    if (!rst_n) begin ipl7_q <= 1'b0; nmi_pend <= 1'b0; end
    else begin
      ipl7_q <= (ipl == 3'd7);
      if ((ipl == 3'd7) && !ipl7_q) nmi_pend <= 1'b1;
      else if (int_take && (inl == 3'd7)) nmi_pend <= 1'b0;   // consumed when the level 7 interrupt is taken
    end
  end
  // bus/address error routine in progress: from the a0s_sub entry to its final dispatch.
  // The busctl decides the double fault (cycle started inside the routine and failed) and stops the core (halted_n), like the die
  // (MC68000UM 5.4.4 p. 5-28: another bus error during the processing of the exception, before the following instruction).
  // berr_ev counts at the entry: the en of the word that waits for the cycle coincides with the done and consumes berr_ev
  // combinationally (a0s_sub) with berr_l not yet high; without berr_ev the routine would not be seen as in progress and a BERR on
  // the first push of the frame would be taken as a new bus error instead of the double fault (double fault case of sim/tb_m68k_berr.cpp).
  // The reset exception processing counts too: "If a bus error occurs during the exception processing for a bus
  // error, an address error, or a reset, the processor halts" (MC68000UM 6.3.9.1 p. 6-16; 5.4.4 p. 5-28: a bus error while the
  // processor is reading the vector table). The reset routine starts from micro address 0 at reset (no a0s_sub entry: rst_req
  // is tied low in the top), so g0_busy is 1 out of reset and stays so until the dispatch of the first instruction; an rst_l
  // entry, like a bus or address error entry, sets it as well.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) g0_busy <= 1'b1;
    else if (en && a0s_sub) g0_busy <= 1'b1;
    else if (en && dispatch && !a0s_sub) g0_busy <= 1'b0;
  end
  logic int_take;   // dispatch that enters the interrupt routine (consumes nmi_pend if of level 7)
  assign int_take = en & ~a0s_sub & dispatch & a0_sub & ~trace_l & int_l;
  logic priv;
  // line A and line F arrive REGISTERED from cpu_si, like `illegal` and `priv_ir`, with which they
  // share this same OR: the path ir -> instr_exc -> a0 -> base -> nano ROM starts from flops.
  // They are PURE functions of the instruction, and the correctness of the OR with illegal (which is
  // f(ir of the previous edge)) ALREADY requires ir to be stable in that window at the dispatch:
  // registering them changes neither value nor cycle.
  // The privileged instructions (STOP, RESET, RTE, MOVE to SR, ANDI/ORI/EORI to SR, MOVE USP)
  // are not decoded here in the core: eight 16-bit compares on `ir` would sit on the cone
  //   ir -> instr_exc -> a0_sub -> useq -> micro ROM address.
  // m68k_a123_si ALREADY decodes them (set 'p' of the die PLA) and the result arrives registered,
  // at the same instant as `illegal`, with which it shares this OR. Exhaustively proven: the two
  // sets match on all the 65,536 opcodes (86 privileged, 0 divergences). The compares below are
  // used only with EXT_PRED = 0 (testbench).
  logic priv_set; assign priv_set = EXT_PRED ? priv_ir
    : ((ir == 16'h4e72) | (ir == 16'h4e70) | (ir == 16'h4e73) | (ir[15:6] == 10'b0100_0110_11)
       | (ir == 16'h027c) | (ir == 16'h007c) | (ir == 16'h0a7c) | (ir[15:4] == 12'h4e6));
  assign priv = ~s_bit & priv_set;
  logic linea_v, linef_v;
  assign linea_v = EXT_PRED ? linea : (ir[15:12] == 4'ha);
  assign linef_v = EXT_PRED ? linef : (ir[15:12] == 4'hf);
  logic instr_exc, stopped;
  assign instr_exc = illegal | linea_v | linef_v | priv;
  always_comb begin
    a0_sub = 1'b0; a0 = A_TRAP;
    if (dispatch) begin
      if (trace_l)        begin a0_sub = 1'b1; a0 = A_TRACE; end
      else if (int_l)     begin a0_sub = 1'b1; a0 = A_INT; end
      // this branch does not assign a0: A_TRAP is ALREADY its default value a few lines above, so
      // a0 does not depend on instr_exc, which stays only in a0_sub (an OR) and in the gate of the
      // STOP branch, out of the priority levels in front of a0. Same semantics by construction.
      else if (instr_exc) begin a0_sub = 1'b1; end
      // STOP (Motorola manual): the CPU stays stopped until an interrupt of sufficient level arrives (or trace/reset).
      // The wait is a loop of two words of our own with an empty nanoword, on the four-clock grid of the die's loop
      // (327 aaa01 with the A1 dispatch, then 3a2 stop1 whose next is 327; docs/data/mc68000_micro_decoded.csv): at the
      // dispatch of word 327 (stop_w = upc == 327) and at every dispatch of 3fd while stopped, the word 3fc (samples the
      // request, no dispatch, next 3fd from the micro ROM); 3fd samples it too (IR-load bit, interrupts.md) and dispatches: the
      // interrupt is seen there, every four clocks, at the instants of the die's 327 dispatch (both are the second word
      // after the STOP word), as measured on the reference core (a loop of 3fd alone took it up to two clocks early). The
      // qualifier is the word, not 'ird == 4e72', which stays true also along the privilege trap of a STOP in user mode: the
      // handler would enter the wait.
      else if (stopped | stop_w) begin a0_sub = 1'b1; a0 = A_STOPW; end
    end
  end
  always_ff @(posedge clk) begin
    if (!rst_n) stopped <= 1'b0;
    else if (en && dispatch) begin
      if (trace_l | int_l) stopped <= 1'b0;
      else if (stop_w) stopped <= 1'b1;
    end
  end
  // vector: exception taken (registered at the entry of the routine) or from the executing instruction
  logic [7:0] tvn_r; logic tvn_use_r;
  logic [7:0] tvn_instr;
  function automatic logic [7:0] f_tvn(input logic [15:0] w);
    if (w[15:4] == 12'h4e4)                          f_tvn = {4'd2, w[3:0]};   // TRAP #n = 32 + n
    else if (w == 16'h4e76)                          f_tvn = 8'd7;             // TRAPV
    else if (w[15:12] == 4'h4 && w[8:6] == 3'b110)   f_tvn = 8'd6;             // CHK
    else if (w[15:12] == 4'h8 && w[7:6] == 2'b11)    f_tvn = 8'd5;             // DIVU/DIVS: divide by zero
    else                                             f_tvn = 8'd4;             // otherwise illegal
  endfunction
  // pure function of the instruction: decided on `ir` and registered at the instant `ird` takes `ir`; at reset it equals
  // f(0) = 4, like ird = 0. Same value, same cycle, and the decode stays out of the path that loads the FTU in the
  // `ftu<-trap` words. Safety net below.
  generate if (TVN_REG) begin : g_tvn_reg
    always_ff @(posedge clk) begin
      if (!rst_n) tvn_instr <= f_tvn(16'd0);
      else if (en && ld_ird) tvn_instr <= f_tvn(ir);
    end
`ifdef VERILATOR
    always @(posedge clk) if (rst_n && tvn_instr !== f_tvn(ird)) begin
      $display("instruction vector flop MISMATCH t=%0t tvn_instr=%0d expected=%0d ird=%04x", $time, tvn_instr, f_tvn(ird), ird); $fatal(1);
    end
`endif
  end else begin : g_tvn_comb
    assign tvn_instr = f_tvn(ird);
  end endgenerate
  // The bus/address error routine takes its vector from tvn_r through g0_busy, not through tvn_use_r: its entry (below) does
  // not set tvn_use_r, which keeps the value it had at the fault until the dbi word of the routine clears it, and is the
  // I/N bit of the group 0 frame (MC68000UM 6.3.9.1 p. 6-16: 1 if the processor was processing a group 0 or 1 exception, 0 in
  // an instruction or a group 2 exception). The frame takes it in bser6 "(ssw) -> ftu", before any dbi word of the routine.
  // Each bit of tvn is a function of four inputs (tvn_use_r, g0_busy, tvn_r, tvn_instr): one LUT per bit.
  always_comb tvn = (tvn_use_r | g0_busy) ? tvn_r : tvn_instr;
  assign tvn_use = tvn_use_r;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      // after reset the rstp1 routine reads the vector from the trap latch (`ftu<-trap`): vector 0 = reset
      // inl = 7 at reset: rstp2 (App. H p139) loads the mask with "(inl) -> psw" -> SR = 2700 after reset (Motorola manual)
      rst_l <= 1'b0; berr_l <= 1'b0; aerr_l <= 1'b0; trace_l <= 1'b0; int_l <= 1'b0; inl <= 3'd7; tvn_r <= 8'd0; tvn_use_r <= 1'b1;
    end else begin
      // group 1: the events are latched immediately (they do not wait for the instruction boundary)
      if (rst_req) rst_l <= 1'b1;
      if (berr_ev) berr_l <= 1'b1;
      if (aerr_ev) aerr_l <= 1'b1;
      if (en) begin
        if (a0s_sub) begin   // entry into the immediate routine (priority reset > address error > bus error: MC68000UM 6.2 p. 6-8)
          // Address error and bus error pending together: only when the system answers BERR to the address phase of the
          // aborted cycle itself (AS without data strobes, ended at its first S5: m68k_bus_fsm nostrobe), since after a
          // termination by BERR the entry is taken at the first en while g0_hold blocks every request for 10 clocks. The
          // address error is taken (MC68000UM 6.2 p. 6-8) and that BERR is discarded with it: "the bus cycle is aborted"
          // (6.3.10 p. 6-19), there is no cycle for a bus error to terminate. Cost: berr_l clears at every entry that is not
          // a reset (one input fewer).
          if (rst_l)       begin rst_l <= 1'b0; tvn_r <= 8'd0; end
          else if (aerr_l) begin aerr_l <= 1'b0; berr_l <= 1'b0; tvn_r <= 8'd3; end
          else             begin berr_l <= 1'b0; tvn_r <= 8'd2; end   // berr_l or berr_ev; the clear at the entry wins over the set of berr_ev (same edge)
          trace_l <= 1'b0; int_l <= 1'b0;   // tvn_use_r is not set here: it keeps the pre-fault value (I/N of the frame), see tvn above
        end else begin
          if (upd_full) begin   // dbi: samples the interrupt (IPL at the start of the word) for the next boundary
            tvn_use_r <= 1'b0;
            int_l <= int_req;   // not sticky: at the dispatch that takes it the clear further below prevails
            if (int_req) inl <= nmi_pend ? 3'd7 : ipl_w;   // NMI: level 7 from the edge on the pin, even if ipl_w lags (sim/tb_m68k_irq.cpp)
          end
          if (iack_vec) tvn_r <= vec_num;   // vectored interrupt: the number read in the IACK replaces the autovector
          else if (iack_spur) tvn_r <= 8'd24;   // IACK terminated by BERR: spurious interrupt, vector 24
          if (dispatch && a0_sub) begin
            tvn_use_r <= 1'b1;
            if (trace_l)      begin trace_l <= 1'b0; tvn_r <= 8'd9; end
            else if (int_l)   begin int_l <= 1'b0; tvn_r <= {3'd0, 2'b11, inl}; end   // autovector 24 + level
            // illegal EA in a privileged encoding (46C8-46CF, 46FD-46FF, 087C): illegal wins, vector 4,
            // like the die PLA (ill and priv together).
            else if (priv & ~illegal) tvn_r <= 8'd8;
            else if (linea)   tvn_r <= 8'd10;
            else if (linef)   tvn_r <= 8'd11;
            else              tvn_r <= 8'd4;
          end
          // T sampled at the start of the instruction, i.e. at the a1 dispatch that starts it (MC68000UM 6.3.9), not at the end of the
          // previous instruction: the instruction that sets T is not traced and the one that clears it is. The decision
          // above uses the value sampled at the previous dispatch. At the entry into an exception (trace, interrupt, instruction
          // exception) the pending is cleared ('0 -> tpend' of trac2); the STOP wait (3fd) is not an exception and samples T.
          if (dispatch) trace_l <= (trace_l | int_l | instr_exc) ? 1'b0 : t_bit;
        end
      end
    end
  end
  /* verilator lint_off UNUSEDSIGNAL */
  logic unused; assign unused = upd_part | (ld_ird & ~TVN_REG);   // dbl: partial update = group 1 only, which here is already continuous; ld_ird not used in the combinational branch
  /* verilator lint_on UNUSEDSIGNAL */
endmodule
