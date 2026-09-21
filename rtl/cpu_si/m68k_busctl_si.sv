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

// m68k_busctl_si.sv - microcycle sequencer + hookup of the bus machine (m68k_bus_fsm) to the access classes
// of the nano ROM. Source: docs/silicon-datapath.md §3.3 and §17; US 4,325,121 (access classes i/f/t/n, App. H).
//
// Timing: one microcycle = 2 clock periods of the 68000 = 4 phases (ph0 = phi1, ph1 = phi2, ph2 = phi1, ph3 = phi2); the control
// store advances (en) on the last phase (ph3, cen2) if there is no stall. One bus cycle = S0..S7 = 2 microcycles without wait states.
//   i (initiate): req to the bus in ph0 (S0 = that cen; AS in S2 = ph2); the sequencer does NOT wait.
//   f (finish)  : the microcycle ends only at phase S7 of the cycle in progress (S4/S5 repeated with the wait states).
//   t (total)   : i + f in the same microcycle (the microcycle is stretched until S7).
//   n / idle wait: no access, 4 phases.
// FC: fc field of the microword (01 d = data, 10 i = program, 11 a = IACK/CPU space) + S bit of the PSW.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_busctl_si (
  input  logic        clk, rst_n,
  input  logic        cen, cen2,          // alternating phases phi1/phi2 (clock-enable generator)
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [NDEC_W-1:0] dec,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [1:0]  fc_micro,           // bits 16:15 of the current microword
  input  logic        sup,                // PSW S
  input  logic        tas_op,             // IRD = TAS (0100 1010 11xx xxxx): the read of the operand opens an indivisible RMW cycle (AS stays low until the write)
  input  logic        pc_rel,             // IRD[5:0] = 111 01x: operand d16(PC)/d8(PC,Xn), read in program space (PC-relative reference, FC = x10)
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [31:0] aob,                // address output buffer (bus-side registers); A23..A1 + A0 for UDS/LDS go out
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [15:0] dob,                // data output buffer
  input  logic        byte_acc,           // byte access (size from the instruction)
  // external bus (like m68k_bus_fsm)
  input  logic        dtack_n, vpa_n, berr_n,
  input  logic        halt_n, br_n, bgack_n,   // HALT and arbitration (Motorola datasheet: docs/bus-control-signals.md)
  input  logic        g0_busy,                 // exc: bus/address error routine in progress (from the entry to its final dispatch)
  output logic        bg_n, bus_tri, halted_n,
  input  logic [15:0] data_in,
  input  logic [4:0]  e_cnt,
  output logic        as_n, uds_n, lds_n, rw, vma_n,
  output logic [15:0] data_out,
  output logic        data_oe,
  output logic [2:0]  fc,
  // the address output register lives in cpu_si: from here only the enable of its 23 flops goes out, so that
  // the register can be placed next to its consumer, outside the CPU.
  output logic addr_ld_o,
  // to the control store and the datapath
  output logic        en,                 // control store advance (one phase per microcycle)
  output logic        en_rf,              // copy of en for the write of the register bank (same function from a copy of the phase register)
  input  logic        c_ft_q,             // c_f | c_t registered at en in m68k_nstore_dec (RAW class: same edge and same source as dec_pq)
  output logic [1:0]  ph,                 // microcycle phase (observability)
  output logic        req,                // request to the bus (observability)
  output logic [15:0] edb,                // read data (for dbin<-edb)
  output logic        iack_vec,           // end of an IACK cycle (FC=7) terminated by DTACK: the vector number is in edb[7:0] (VPA = autovector)
  output logic        iack_spur,          // end of an IACK cycle (FC=7) terminated by BERR: spurious interrupt, vector 24 (MC68000UM 6.3.2, 6.3.4)
  output logic        iack_acc,           // the current word performs the IACK access (acc_type ak): no address error on the odd address 0xFFFFF1+2*level
  output logic        edb_valid,          // rd_capture of the FSM
  output logic        bus_active,
  output logic        stall,              // microcycle stretched waiting for S7
  output logic        ssw_rw,             // SSW of the address/bus error frame: R/W (1 = read) and FC of the aborted cycle
  output logic [2:0]  ssw_fc,
  output logic        berr_ev,            // bus error: once, at the done of the cycle terminated by BERR (entry into bser at the en of the word that waits for it)
  output logic        aerr_ev,            // address error decided at S1 (2 pulses from en: the address from the register comes from rf_q)
  output logic        dfault              // double fault: bus/address error of a cycle started after the entry into the routine (until reset)
);
  logic [ND_ACC_CLASS_W-1:0] cls; logic [ND_ACC_MODE_W-1:0] mode;
  assign cls  = dec[ND_ACC_CLASS_LSB +: ND_ACC_CLASS_W];
  assign mode = dec[ND_ACC_MODE_LSB +: ND_ACC_MODE_W];
  logic c_i, c_f, c_t, wr;
  assign c_i = (cls == ND_ACC_CLASS__I[ND_ACC_CLASS_W-1:0]);
  assign c_f = (cls == ND_ACC_CLASS__F[ND_ACC_CLASS_W-1:0]);
  assign c_t = (cls == ND_ACC_CLASS__T[ND_ACC_CLASS_W-1:0]);
  assign wr  = (mode == ND_ACC_MODE__W[ND_ACC_MODE_W-1:0]);

  // microcycle phase: the EVEN phases (ph0, ph2) advance only on cen (phi1), the ODD ones (ph1, ph3) only on cen2 (phi2), like the
  // states of the bus FSM: the parity of ph is tied to the identity of the phase and self-aligns whatever the first phase after reset is.
  // (A free-running counter on cen|cen2 would start with arbitrary parity and with a cen2 first it would never fire req/en.)
  logic phase_ev;
  assign phase_ev = ph[0] ? cen2 : cen;
  logic started;          // the cycle of this word has already been requested (t: avoids double requests with the stall)
  logic g0_hold;          // wait after an address error (g0_cnt): no request while it is high
  logic [2:0]  st_dbg; logic at_s7;   // at_s7 = (st == S7) registered in the FSM together with st
  logic        bus_done, rmw_hold, rd_cap;
  logic [15:0] rdata;
  // completion: S7 is an odd state (phi2): st_dbg == 7 with the bus active in the cen2 phase. If S7 falls before the
  // f word (e.g. i, n, f) the completion stays latched (done_l) until an f/t word consumes it.
  logic done_now, done_l, wait_done;
  // RMW (TAS): the write half (S10..S13, AS already low) samples DTACK/BERR in S11/S13 and ends through S6/S7 like any cycle
  logic rmw_wr;   // write half of the RMW cycle in progress
  logic s1_tick;  // S1 of the cycle just requested (never in the RMW write half): address error, A0 and SSW are decided here
  assign s1_tick = cen2 & (st_dbg == 3'd1) & ~rmw_wr & ~rmw_hold;   /* st_dbg is st[2:0]: code 9 would alias S1 but is never used (the RMW wait is S8, st_dbg 0), so ~rmw_hold is redundant */
  assign done_now  = at_s7 & cen2;   // at_s7 = (st == S7) registered in the FSM (S7 is not IDLE: bus_active implicit); the RMW write half also ends at S7
  assign iack_vec  = done_now & (fc_r == 3'b111) & vpa_n & ~berr_pend;   // vectored interrupt: IACK terminated by DTACK, vector number on the data bus (docs/interrupt-acknowledge.md §4.4)
  // IACK terminated by BERR = spurious interrupt (MC68000UM 6.3.2 p. 6-12, 6.3.4 p. 6-13, table 6-2 note 3):
  // vector 24 and short frame of the normal exception, no bus error (berr_ev and g0_cnt excluded for FC = 7)
  assign iack_spur = done_now & (fc_r == 3'b111) & berr_pend;
  assign wait_done = c_ft_q & ~((at_s7 & cen2 & ~discard) | done_l);   // c_ft_q = c_f | c_t from a flop; discard: done of the address error cycle of an 'i' word
  logic wait_free;   // i/t word with the previous access still in progress: waits for the free bus before requesting
  logic rmw_ok;   // RMW: the write request starts at the second cen in ph0 (one clock later, like the 68000: S12 after S8-S11)
  assign wait_free = (c_i | c_t) & (ph == 2'd0) & ~started & ((bus_active & ~(rmw_hold & rmw_ok)) | g0_hold);   // while waiting for RMW the write can start; g0_hold: wait after address error
  logic ph3_q;   // (ph == 3) registered from the next value of ph: exact at every clock, one level fewer on en and stall
  assign stall = (ph3_q & wait_done) | wait_free;   // ph3_q = (ph == 3) registered from the next value; the phase stays still on ph3 until S7 arrives
  assign en    = ph3_q & cen2 & ~wait_done;  // one LUT6: ph3_q, cen2, c_ft_q, at_s7, discard, done_l. S7 always falls on a cen2, so the ph3<->cen2 parity is preserved
  // copy of en for the write of the register bank: same function, from ph3_rf, a copy of the phase flag ph3_q
  // (preserve/dont_merge: synthesis does not merge the two), so its LUT can be placed next to the bank and not in the center of
  // the core: en has many loads, and the path halt_n -> cen2 -> en -> bank would otherwise carry the long routing to the bank.
  (* preserve *) (* dont_merge *) logic ph3_rf;   // copy of ph3_q for en_rf
  assign en_rf = ph3_rf & cen2 & ~wait_done;
`ifdef VERILATOR
  // SAFETY NETS at every clock: ph3_q and ph3_rf = (ph == 3); c_ft_q = c_f | c_t; at_s7 = bus_active & (st_dbg == 7) outside a retry; en_rf = en
  always @(posedge clk) if (rst_n && (ph3_q !== (ph == 2'd3) || ph3_rf !== (ph == 2'd3) || c_ft_q !== (c_f | c_t) || at_s7 !== (bus_active & (st_dbg == 3'd7) & ~fsm_retry) || en_rf !== en)) begin
    $display("phase and enable copies MISMATCH t=%0t ph %0d ph3_q %b ph3_rf %b c_ft_q %b cf|ct %b at_s7 %b st_dbg %0d act %b en %b en_rf %b", $time, ph, ph3_q, ph3_rf, c_ft_q, c_f | c_t, at_s7, st_dbg, bus_active, en, en_rf); $fatal(1);
  end
`endif
  // request: in ph0 (cen) of the i/t words, only once per word
  // HALT / arbitration: with the bus idle and HALTn low, BGACKn low or a grant in progress (BGn low) the CPU does not start cycles and the
  // microcycle stays still (frozen); the cycle in progress always ends (bus_active). BGn: asserted at cen with BRn low if the bus is
  // active and the FSM is not in S1 (AS asserted from S2: MC68000UM 5.2.2 p. 5-15 "as soon as possible", also during a cycle,
  // fig. 5-19) or free without a request in that cen (the req cen = cycle decided, AS not yet asserted: BG at the next cen, one
  // clock later than the normal case, fig. 5-21 p. 5-16); removed when the master confirms (BGACKn low) or withdraws the request.
  // With two phases the cens never see S1 (it lies between a cen and the cen2): BG falls together with AS at S2; with a single phase (cen = cen2 every
  // tick) the term st_dbg != 1 (S1 only: code 9 is never used) avoids BG one tick before AS if BR falls in the req cen.
  // A grant with the bus idle also in the req cen would show BG, with AS still high, to a master that would then take the bus
  // while the cycle is starting (BGACK together with the processor's AS). bus_tri: outputs in high impedance while BGACKn is low.
  // Retry of a bus cycle (MC68000UM 5.4.2 p. 5-26, Table 5-1 case 5): BERR with HALT at the termination. The FSM terminates
  // the cycle without a completion (at_s7 stays 0: no done, no bus error, no g0 count), keeps the bus and waits in S7;
  // fsm_retry is high from the decision to the restart: berr_pend is dropped (the BERR of the aborted run must not become a
  // bus error at the completion of the rerun) and the processor counts as halted while HALT is asserted. The restart needs
  // the same conditions as a new cycle (rerun_ok = ~hold_core). With BERR tied high fsm_retry is a constant 0: no cost.
  logic frozen, hold_core, rerun_ok, fsm_retry, fsm_rt_wait;
  // BERR still asserted after the aborted cycle: "After the aborted bus cycle is terminated and BERR is negated, the
  // processor enters exception processing" (MC68000UM 5.4.1 p. 5-24); until then the buses stay released (bus_tri) and no
  // cycle starts (frozen, at bus free only). One input more on hold_core (five, one LUT); with BERR tied high it vanishes.
  assign hold_core = ~halt_n | halt_int | ~bgack_n | ~bg_n | ~berr_n;
  assign frozen   = hold_core & ~bus_active;
  assign rerun_ok = ~hold_core;
  // bus_tri: the address and data buses are in the high-impedance state. Bus granted (BGACK, 5.2: address, data and the
  // control signals); processor halted with the bus idle, by HALT or by the double bus fault ("While the processor is halted,
  // the address bus and the data bus signals are placed in the high-impedance state", 5.4.3 p. 5-27; 5.4.4); retry wait
  // ("puts the address and data lines in the high-impedance state ... until HALT is negated", 5.4.2 p. 5-26, until S0 of the
  // retried cycle); BERR still asserted after the cycle it terminated ("As long as BERR remains asserted, the data and address
  // buses are in the high-impedance state", 5.4.1 p. 5-24). Outside bus arbitration the control signals stay driven: FC holds,
  // AS, UDS, LDS and R/W negated (figures 5-27 and 5-29). Where bus_tri is left unconnected the logic below has no fanout.
  assign bus_tri  = ~bgack_n | ((~halt_n | halt_int) & ~bus_active) | fsm_rt_wait | (~berr_n & ~bus_active);
  assign halted_n = ~((~halt_n | halt_int) & (~bus_active | fsm_retry));
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) bg_n <= 1'b1;
    else if (cen) begin
      if (br_n || !bgack_n) bg_n <= 1'b1;
      else if ((bus_active && (st_dbg != 3'd1)) || (!bus_active && !req)) bg_n <= 1'b0;
    end
  end
  assign req = (c_i | c_t) & (ph == 2'd0) & cen & ~started & ~frozen & ~g0_hold & (~bus_active | (rmw_hold & rmw_ok));
  // address error: word/long access to an odd address (not IACK). The cycle still runs its address phase (AS, address, FC,
  // R/W) without data strobes and without write data, and ends by itself at the first S5 without DTACK (FSM nostrobe,
  // docs/exceptions.md §8). The error is decided at S1 (aerr_ev) and enters the bser routine through exc (aerr_l).
  logic aerr_c; assign aerr_c = ~byte_acc & aob[0] & ~iack_acc;
  assign aerr_ev = s1_tick & aerr_c;   // to exc (aerr_l), at S1
  // the address error cycle runs without strobes (FSM nostrobe); if an 'i' word requests it no 'f' will consume its
  // completion (the sequencer jumps to bser): that done must be discarded, it must not become done_l nor unblock the following 't'
  logic discard, req_was_i, aerr_q;   // aerr_q: registered flag of the address error (request -> en), see below
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin discard <= 1'b0; req_was_i <= 1'b0; end
    else begin
      if (req) req_was_i <= c_i;
      // sum of products (parallel sets, clear at done): an if/else-if chain would add a mux on the AOB cone
      discard <= ~done_now & ((aerr_q & req_was_i) | discard);   // only the address error of an 'i' word (after a bus error the 'f' word consumes its own done)   // the done clears with priority over the set (a set winning at the done would leave the following 't' stalled); registered flags: the AOB cone does not enter here
    end
  end
  // time of entry into the bser routine, as measured on the reference core: the first push of the frame starts 48 ticks after the AS of the
  // aborted cycle = 8 clocks after its end, both for the 'i' words and for the 't' ones. g0_cnt: 8 clocks (cen) from the done of the aborted
  // cycle during which no request starts (the 't' word of the routine stalls in ph0). That is the address error case.
  // Bus error: same mechanism, 10 clocks (first push 56 ticks after the AS of the cycle); not for an IACK cycle (spurious interrupt).
  // Double fault (dfault from the top: berr/aerr inside the bus/address error routine): halt_int 16 ticks after the done of the aborted
  // cycle (halted_n at +16 on the reference core), microcycle still (frozen) and halted_n low until reset; no other bus cycle.
  logic aerr_pend, berr_pend, halt_int, cyc_g0; logic [3:0] g0_cnt;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin aerr_pend <= 1'b0; berr_pend <= 1'b0; g0_cnt <= 4'd0; halt_int <= 1'b0; cyc_g0 <= 1'b0; dfault <= 1'b0; end
    else begin
      if (req) cyc_g0 <= g0_busy;   // the cycle started inside the bus/address error routine
      dfault <= dfault | (g0_busy & aerr_q) | (g0_busy & done_now & berr_pend & cyc_g0);   // double fault (until reset); aerr_q registered: no AOB cone
      if (s1_tick && aerr_c) aerr_pend <= 1'b1; else if (done_now) aerr_pend <= 1'b0;   // at S1
      if (done_now || fsm_retry) berr_pend <= 1'b0; else if (bus_active && !berr_n && !aerr_pend) berr_pend <= 1'b1;   // not in an address error cycle: it is aborted (UM 6.3.10), a BERR to it is no bus error   // the done clears even if BERR is still low in that tick (a berr_pend left high would make the first push a false double fault); retry: dropped until the rerun starts
      if (done_now && aerr_pend) g0_cnt <= 4'd8;
      else if (done_now && berr_pend && fc_r != 3'b111) g0_cnt <= 4'd10;
      else if (cen && g0_cnt != 4'd0) g0_cnt <= g0_cnt - 4'd1;
      if (dfault && g0_cnt == 4'd4 && cen) halt_int <= 1'b1;   // halted_n 16 ticks after the done, as measured on the reference core
    end
  end
  assign g0_hold = (g0_cnt != 4'd0);
  assign berr_ev = done_now & berr_pend & (fc_r != 3'b111);   // once, at the done of the cycle terminated by BERR; in the IACK it is spurious (iack_spur)
  // flag of the address error, set at S1 and cleared at the en of the word: used by the double-fault check and by discard. Being a
  // register, it keeps the AOB cone (aob[0] in aerr_c) out of those flags, which then feed the done and the request logic.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) aerr_q <= 1'b0;
    else if (s1_tick && aerr_c) aerr_q <= 1'b1;   // at S1
    else if (en) aerr_q <= 1'b0;
  end
  always_ff @(posedge clk or negedge rst_n) begin   // asynchronous reset like m68k_bus_fsm (same net)
    // ph = 1 at reset: the first advance waits for a cen2 instead of a cen, one pulse more than with ph = 2 before the reset routine
    // (rstp1, word 000, only ftu<-trap at en), and the first read of the reset vector falls at the first cen from the 50th tick after
    // the release, in every release phase, like the reference core (52/51/50/53 ticks with PHSPACE=2, 26/25 with PHSPACE=1,
    // measured during development). With ph = 2 the read would fall at the first cen from the 48th tick, up to one clock before the
    // reference core depending on the phase. E also starts at reset, so this alignment fixes where every later VPA/IACK cycle
    // falls against E. It is the timing of a cold reset (external and power-on reset together), used for every reset, the
    // restore of the savestate included.
    // Asynchronous reset of ph, ph3_q, ph3_rf and of the handshake flags below, on the same net as the bus FSM.
    if (!rst_n) begin ph <= 2'd1; ph3_q <= 1'b0; ph3_rf <= 1'b0; started <= 1'b0; done_l <= 1'b0; rmw_wr <= 1'b0; rmw_ok <= 1'b0; end
    else begin
      if (req && wr && rmw_hold) rmw_wr <= 1'b1; else if (done_now) rmw_wr <= 1'b0;
      if (cen) rmw_ok <= rmw_hold & (c_i | c_t) & (ph == 2'd0) & ~started;   // true from the cen following the first ph0 waiting for RMW
      if (phase_ev && !stall && !frozen) ph <= ph + 2'd1;
      ph3_q  <= (phase_ev && !stall && !frozen) ? (ph == 2'd2) : (ph == 2'd3);   // flag from the next value
      ph3_rf <= (phase_ev && !stall && !frozen) ? (ph == 2'd2) : (ph == 2'd3);   // copy for en_rf
      if (req) started <= 1'b1;
      if (en) started <= 1'b0;
      if (done_now && !discard && !(en && (c_f | c_t))) done_l <= 1'b1;
      else if (en && (c_f | c_t)) done_l <= 1'b0;
    end
  end
  // FC and address held for the whole cycle (registered at the request)
  // Field fc = 00 ("n/u" in the die listing) = NOT updated: the code stays that of the last access that set it
  // (the completion words and many access words in sequence - MOVEM, EA - have fc = 00). The reset words of the core
  // rstp7/8/9 (2fb/2ff/300, reassigned by the die) have fc = 10 in the field of the microword: the reads of the reset vectors go out
  // with FC = 6 from the explicit field (checked against the reference core), not from the reset value of fc_code
  // (program, 10), which no cycle observes: the first requests after reset take the code from their own field.
  logic [2:0]  fc_r; logic [1:0] fc_code, fc_code_n;
  logic        byte_r; logic [15:0] dob_r; logic a0_r;
  // 'uk' access (operand via d16/abs/MOVEM): data space (the FC bits of the microword are 00 = keep the code, here that of the prefetch)
  logic acc_uk; assign acc_uk = (dec[ND_ACC_TYPE_LSB +: ND_ACC_TYPE_W] == ND_ACC_TYPE__UK[ND_ACC_TYPE_W-1:0]);
  assign iack_acc = (dec[ND_ACC_TYPE_LSB +: ND_ACC_TYPE_W] == ND_ACC_TYPE__AK[ND_ACC_TYPE_W-1:0]);
  logic acc_op; assign acc_op = (dec[ND_ACC_TYPE_LSB +: ND_ACC_TYPE_W] == ND_ACC_TYPE__OP[ND_ACC_TYPE_W-1:0]);   // read/write of the operand
  assign fc_code_n = acc_uk ? (pc_rel ? 2'b10 : 2'b01) : (fc_micro != 2'b00) ? fc_micro : fc_code;   // 'uk' = only operand reads: program if PC-relative
  // address on the pins: at S0 (with req) if the word does not load the AOB or loads it from the AU (aob<-au, aob<-db with
  // db<-au); at S1 (phase after req) if it loads it from AB or from DB with register/AT/PC/dbin. Measured on the reference core over all
  // the 65536 opcodes (word 150 of CHK, the only one with an access and without aob<-, at S0). The incoming AOB
  // (aob_n) is stable for the whole microword: at S1 it equals that of S0. A single enable (addr_ld) for the 23 flops of addr_r (in cpu_si).
  logic aob_dly, addr_ld;
  assign aob_dly = (|dec[ND_AOB_FROM_LSB +: ND_AOB_FROM_W]) & ~(dec[ND_AOB_FROM_LSB + ND_AOB_FROM__AU] | (dec[ND_AOB_FROM_LSB + ND_AOB_FROM__DB] & dec[ND_DB_FROM_LSB + ND_DB_FROM__AU]));
  assign addr_ld = (req & ~aob_dly) | (cen2 & (st_dbg == 3'd1) & aob_dly);
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin fc_r <= 3'd0; fc_code <= 2'b10; byte_r <= 1'b0; dob_r <= '0; a0_r <= 1'b0; end
    else begin
      if (req) begin
        fc_code <= fc_code_n;
        fc_r    <= (fc_code_n == 2'b11) ? 3'b111 : {sup, fc_code_n};
        byte_r <= byte_acc; dob_r <= wdata_q;
      end
      if (s1_tick) a0_r <= aob[0];   // A0 at S1: byte strobes (S2 read, S4 write) and byte lane of the read data (rbyte)
    end
  end
  assign fc = fc_r; assign addr_ld_o = addr_ld;
  logic uds_q, lds_q;
  assign uds_q = byte_acc ? ~a0_r : 1'b1;   // from A0 of the cycle in progress sampled at S1 (a0_r): at S2/S4 it equals aob[0] (safety net below)
  assign lds_q = byte_acc ? a0_r : 1'b1;
`ifdef VERILATOR
  // SAFETY NET: at the ticks in which the FSM samples the strobes (cen in S2 for reads, in S4 for writes) with a byte
  // access, A0 sampled at S1 must equal aob[0] (with a word access the strobes do not depend on A0)
  always @(posedge clk) if (rst_n && cen && byte_acc && (st_dbg == 3'd2 || st_dbg == 3'd4) && a0_r !== aob[0]) begin
    $display("A0 sampled at S1 MISMATCH t=%0t st %0d a0_r %b aob_n[0] %b byte %b", $time, st_dbg, a0_r, aob[0], byte_acc); $fatal(1);
  end
`endif
  logic [15:0] wdata_q;
  // byte: data duplicated on the two bytes (Motorola manual); MOVEP (dobh<-): the byte to write is the high one of DOB
  logic dobh_sel; assign dobh_sel = (dec[ND_DOBH_FROM_LSB +: ND_DOBH_FROM_W] != '0);
  // SSW (address/bus error frame): {IR[15:5], R/W, I/N, FC}, as on the reference core. This block gives R/W and FC of the aborted
  // cycle: address error = values sampled at S1, inside the requesting word; bus error = cycle in progress. I/N is added in m68k_cpu_si
  // (tvn_use of m68k_exc_si: 1 for a fault during the processing of a group 0 or 1 exception, MC68000UM 6.3.9.1 p. 6-16).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin ssw_rw <= 1'b1; ssw_fc <= 3'd0; end
    // address error: sampled once, at S1 (s1_tick), still inside the word that issued the request: after its en, wr belongs to
    // another word. st_dbg = st[2:0]: S1 = 1. Bus error: sampled at every clock of the cycle with BERR low.
    // Direction from the request (~wr), not from the rw pin: with R/W falling at S2 the pin at S1 is still 1 also for writes
    else if ((s1_tick && aerr_c) || (bus_active && !berr_n)) begin ssw_rw <= ~wr; ssw_fc <= fc_r; end   // SSW at S1 with aerr_c (aerr_q is only registered at S1)
  end
  logic [7:0] wbyte; assign wbyte = dobh_sel ? dob[15:8] : dob[7:0];
  assign wdata_q = byte_acc ? {wbyte, wbyte} : dob;
  m68k_bus_fsm u_fsm (
    .clk(clk), .rst_n(rst_n), .cen(cen), .cen2(cen2),
    .req(req), .aerr(aerr_c), .we(wr), .rmw(tas_op & (acc_op | acc_uk)), /* RMW only on the read of the TAS operand (not vectors/fetch with IRD = TAS: ILLEGAL 4AFC, IRQ after TAS) */ .uds_req(uds_q), .lds_req(lds_q), .wdata(wdata_q),
    .dtack_n(dtack_n), .vpa_n(vpa_n), .berr_n(berr_n), .halt(~halt_n), .rerun_ok(rerun_ok), .data_in(data_in), .e_cnt(e_cnt),
    .as_n(as_n), .uds_n(uds_n), .lds_n(lds_n), .rw(rw), .vma_n(vma_n), .data_out(data_out), .data_oe(data_oe),
    .bus_done(bus_done), .rmw_hold(rmw_hold), .rdata(rdata), .bus_active(bus_active), .rd_capture(rd_cap), .state_dbg(st_dbg), .at_s7(at_s7), .retry(fsm_retry), .rt_wait(fsm_rt_wait));
  // byte read: at an even address the byte is on D15-8 (Motorola manual, data bus: UDS -> D15-D8); it moves to the internal low byte
  // (dbin[7:0] = data of the byte operand: ALU, registers). At an odd address (LDS) it is already on D7-0.
  // byte read: the addressed byte (D15-8 at an even address, D7-0 at an odd one) duplicated on both halves (dbinh/dbinl of MOVEP and low byte)
  logic [7:0] rbyte; assign rbyte = a0_r ? rdata[7:0] : rdata[15:8];
  assign edb = byte_r ? {rbyte, rbyte} : rdata; assign edb_valid = rd_cap;
  /* verilator lint_off UNUSEDSIGNAL */
  logic unused; assign unused = bus_done | (|dob_r);
  /* verilator lint_on UNUSEDSIGNAL */
endmodule
