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

// m68k_bus_fsm.sv — state machine of the asynchronous bus cycle of the MC68000.
//
// Implements the S0..S7 state sequence of a read/write transfer as
// documented in docs/bus-interface.md §4-§6 (source: MC68000 User's Manual,
// section 5.1). It does NOT derive from existing implementations.
//
// === TWO-PHASE clock model (cycle-accurate with the real 68000) ===
// The real 68000 is asynchronous: one clock period contains TWO states (one on the
// RISING edge = phi1, one on the FALLING edge = phi2). A minimum bus cycle
// = 4 periods = 8 states S0..S7:
//   period 0: S0 (phi1) , S1 (phi2)
//   period 1: S2 (phi1) , S3 (phi2)
//   period 2: S4 (phi1) , S5 (phi2)
//   period 3: S6 (phi1) , S7 (phi2)
// On the FPGA the double edge is NOT used: single clock + two
// ALTERNATING clock enables. cen = phi1 (rising edge of the 68000), cen2 = phi2
// (falling edge). The clock-enable generator produces exactly these two phases
// (cpu_cen/cpu_cenb). The EVEN states (S0,S2,S4,S6) advance on phi1; the ODD ones
// (S1,S3,S5,S7) on phi2. (The single-cen testbenches pass cen2=cen.)
//
// === Signal timing (transcribed from MC68000UM 5.1.1 / 5.1.2) ===
// READ:
//   S2 (rising): asserts AS + UDS/LDS.
//   S4 (end, FALLING edge = entering S5): samples DTACK/BERR/VPA; if
//      not asserted -> wait state (stays on S4/S5 until it arrives).
//   S6 (end, FALLING edge = entering S7): LATCHES the data; negates AS/DS.
// WRITE:
//   S2 (rising): asserts AS, drives R/W low.
//   S3: the data bus leaves hi-Z, data on the bus.
//   S4 (rising): asserts UDS/LDS. End of S4 (falling): samples DTACK; wait as for read.
//   S7 (entering, falling): negates AS/DS, releases the bus, R/W goes back high.
// MANUAL NOTE: DTACK/BERR/VPA sampled on EVERY falling edge starting from
// S4; read data accepted on the falling edge entering S7 (rdata, see its own
// block). Here: "falling edge of Sx" = transition Sx(phi1) -> Sx+1(phi2), i.e. the
// action is performed in the branch of the ODD state (phi2) that follows.
//
// === What it models ===
// Only the timing of the bus control signals (AS, UDS/LDS, R/W) and
// the termination handshake (DTACK/VPA/BERR) with wait states. The datapath is
// in the sequencer, not here.

`default_nettype none

module m68k_bus_fsm (
    input  wire        clk,        // system clock
    input  wire        rst_n,      // asynchronous reset, active low
    input  wire        cen,        // clock-enable phi1 (RISING edge of the 68000)
    input  wire        cen2,       // clock-enable phi2 (FALLING edge). With the system generator
                                   // cen and cen2 ALTERNATE; single-cen tb: cen2=cen.

    // Request from the CPU sequencer
    input  wire        req,        // 1 = starts a bus cycle
    input  wire        aerr,       // with req: address error (odd address). The cycle runs (AS, R/W, FC normal) but
                                   //   without data strobes and without driving the data, as measured on the reference core (docs/exceptions.md §8)
    input  wire        we,         // 1 = write, 0 = read
    input  wire        rmw,        // (TAS) 1 with req+read: indivisible READ-MODIFY-WRITE cycle: at the end of the
                                   //   read AS STAYS asserted; the following write (req+we) is the 2nd half
    input  wire        uds_req,    // upper byte requested
    input  wire        lds_req,    // lower byte requested
    input  wire [15:0] wdata,      // data to write (for writes)

    // Termination handshake (from the slave)
    input  wire        dtack_n,    // Data Transfer Acknowledge, active low
    input  wire        vpa_n,      // Valid Peripheral Address (6800 / autovector)
    input  wire        berr_n,     // Bus Error: aborts the cycle
    input  wire        halt,       // HALT asserted (active high, from the controller): sampled with BERR at the termination (end
                                   //   of S4): both = retry of the cycle once HALT is negated (MC68000UM 5.4.2 p. 5-26, Table 5-1
                                   //   case 5), not a bus error. Active high so that a bench of this module alone, which leaves
                                   //   it unconnected (0), keeps the plain bus error
    input  wire        rerun_ok,   // 1 = the retried cycle may restart: HALT negated, no double fault, bus neither granted
                                   //   nor taken (from the controller: the same conditions that let a new cycle start)
    input  wire [15:0] data_in,    // input data bus (read)
    input  wire [4:0]  e_cnt,      // phase of the E clock: counts CPU clocks 0..9 on cen2 (phi2),
                                   //   E=1 for e_cnt>=6. The VPA cycle (6800/autovector)
                                   //   synchronizes to E like the 68000 (see S5).

    // Bus control signals (to the slave) — active low as on the 68000
    output reg         as_n,       // Address Strobe
    output reg         uds_n,      // Upper Data Strobe
    output reg         lds_n,      // Lower Data Strobe
    output reg         rw,         // 1 = read, 0 = write
    output reg         vma_n,      // Valid Memory Address (VPA cycle: asserted synchronously to E)
    output reg  [15:0] data_out,   // data bus driven by the CPU (write)
    output reg         data_oe,    // 1 = the CPU drives the data bus (write)

    // Status to the sequencer
    output reg         bus_done,   // pulse: cycle completed (in S7)
    output wire        rmw_hold,   // (TAS) between the two halves of the RMW cycle: AS low, waiting for the write
    output reg  [15:0] rdata,      // read data, accepted at the falling edge entering S7 (follows data_in during S6 and S7)
    output wire        bus_active, // 1 = bus cycle in progress (not IDLE)
    output wire        rd_capture, // 1 in the cen of S6 of a READ: data_in valid (DTACK
                                   //   already honored at S4, wait states included); the internal
                                   //   latch rdata takes it at the next phase tick (entering S7). DEDICATED
                                   //   and unambiguous strobe for the early consume
                                   //   of the sequencer (state_dbg aliases IDLE=15 with S7).
    output reg         at_s7,      // 1 when st == S7, registered together with st: done_now = at_s7 & cen2 in one LUT
    output reg         retry,      // 1 from the retry termination (S5) to the restart of the cycle: the controller sees no
                                   //   completion (at_s7 stays 0) and drops its BERR flag; the processor is halted meanwhile
    output wire        rt_wait,    // 1 while the retry waits with the address and data buses released (5.4.2): from the
                                   //   termination to S0 of the retried cycle (the phi2 that arms the restart, one clock before AS)
    output wire [2:0]  state_dbg   // current state S0..S7, for debug/verification
);

    // S0 of the manual = IDLE (the cycle starts in S0 when req arrives). IDLE is encoded
    // 15 but logically it is an EVEN state (phi1), like S0.
    localparam [3:0] S1 = 4'd1, S2 = 4'd2, S3 = 4'd3,
                     S4 = 4'd4, S5 = 4'd5, S6 = 4'd6, S7 = 4'd7,
                     // second half of the RMW cycle (TAS): S8 wait for the write (AS low,
                     //   strobes negated, like S8-S11 of the 68000: "bus signals unaltered", MC68000UM 5.1.3 p. 5-8),
                     //   req: R/W low + data, S10 data bus + strobes, S11 samples DTACK (wait = S12/S13), end via S6/S7 (AS negated).
                     // S8 is a single even state held until the write req arrives: the controller issues that req only
                     //   once (started), so a wait made of two states alternating on cen/cen2 could let it fall, with cen
                     //   and cen2 in the same tick (single-phase testbenches), in a tick where no state takes it.
                     S8 = 4'd8, S10 = 4'd10, S11 = 4'd11, S12 = 4'd12, S13 = 4'd13,
                     IDLE = 4'd15;
    reg rmw_cyc;   // the current cycle is the read half of an RMW: at S7 do not negate AS
    // rmw_hold looks at st[3:1] only: codes 8 (S8) and 9 (unused, unreachable) both give 100, so it equals (st == S8)
    //   with one input less in the cone of rmw_hold (it enters req).
    assign rmw_hold = (st[3:1] == 3'b100);

    reg [3:0] st;

    // Retry of the bus cycle (MC68000UM 5.4.2 p. 5-26, Table 5-1 case 5): BERR and HALT asserted together at the termination
    // sample (S5) -> the cycle terminates like a bus error (S6, S7: AS and strobes negated) but without a completion to the
    // sequencer, the processor waits in S7 while HALT is asserted, then the SAME cycle runs again from S2 with the function
    // codes and address held by the controller and the R/W, strobes and write data registered by the first run (the microword
    // that issued the request may have moved on: 'i' words). Not retried: read-modify-write cycles (TAS: "the processor does
    // not retry a read-modify-write cycle", note p. 5-27, bus error whether or not HALT is asserted) and address error cycles
    // (no strobes, no data). With BERR tied high the flops below are constant or without fanout: the synthesis removes them.
    reg rerun;          // the current run of the cycle is a retry: S2/S4 take direction, strobes and data from the registered copies
    reg uds_k, lds_k;   // data strobes requested by the first run (captured where the strobes are asserted: S2 read, S4 write)
    reg berr_q;         // the termination decided in S5 was a bus error (BERR): the read half of an RMW ends like any cycle
                        //   (AS negated, no S8: "terminated in S9 for ... the read portion of a read-modify-write cycle", 5.4.1 p. 5-24)
    assign rt_wait = retry & ~rerun;
    // Timing of the restart: the manual fixes no delay between the negation of HALT and S0 of the retried cycle (5.4.2, figure
    // 5-27: "remains in this state until HALT is negated. Then the processor retries the preceding cycle"). Here HALT is sampled
    // at the phi2 of the S7 wait; the next phi2 is S0/S1 of the retried cycle (buses driven again) and AS falls at the following
    // phi1 (S2): 1.5 clocks after the sample, the same S0-to-S2 distance as a cycle started from IDLE.

    assign state_dbg  = st[2:0];
    assign bus_active = (st != IDLE);
    // Early consume: during the cen in which st==S6 (read) the slave data
    // is on the bus; rdata takes it at the edge entering S7 (block below).
    assign rd_capture = (st == S6) & rw;

    // Read data: accepted at the falling edge entering S7 ("On the falling edge of the clock entering S7, the processor
    // accepts data from the device", MC68000UM 5.1.1 p. 5-4). rdata follows data_in at every clock from the tick that
    // enters S6 to the tick that enters S7 (st == S6 or S7: st[3:1] == 011; the RMW write half, S12/S13, is excluded by
    // st[3]) and holds from the S7 edge on. The word that completes the cycle consumes it at that edge (en = done_now) and
    // reads the value sampled one FPGA clock before the edge, in both clock modes: with alternating phases the last load
    // before the edge, with cen = cen2 at every clock the load of the S6 tick (with a load on S7 only, that mode would
    // read the previous cycle). In the systems data_in is the wrapper's register loaded at DTACK (m68k_cpu_dt_si rd_q),
    // stable from the DTACK clock to the end of the cycle: the value one clock before the edge is the value at the edge.
    // Enable from st[3:1] and rw: four inputs. Synchronous reset.
    always @(posedge clk) begin
        if (!rst_n) rdata <= 16'd0;
        else if ((st[3:1] == 3'b011) && rw) rdata <= data_in;   // S6 or S7
    end

    // Termination: DTACK (normal) | BERR (abort) -> S6 immediately. VPA (6800/autovector):
    //   cycle SYNCHRONIZED to E, like the 68000 (MC68000UM 7 "6800 peripheral cycle"; docs/m6800-bus.md §4.2)
    //   and MEASURED on the reference core at all the phases of E:
    //     - VPA recognized at the end of S4 (sampling in S5, like DTACK);
    //     - VMA asserted (on phi2) 2 clocks BEFORE the rise of E, at the first instant
    //       STRICTLY after the recognition (MC68000UM: 6800 cycle from 10 to 19 clocks);
    //     - AS/UDS/LDS negated (S7) EXACTLY on the FALLING edge of E following
    //       the assertion of VMA: AS lasts 34..66 ticks (8.5..16.5 clocks) in the 5 measured phases
    //       (E phase at AS 6/14/22/30/38 -> AS 50/42/34/66/58, VMA at +26/+18/+10/+42/+34).
    //   A VPA cycle terminated like a DTACK cycle (AS 10 ticks) would make an autovector IACK 8-12 clocks
    //   short and shift the whole sequence after the interrupt.
    //   e_cnt (0..9, updated on cen2): E rises when it becomes 6, falls when it becomes 0.
    //   At the S5 ticks (cen2) e_cnt is the value BEFORE the update of the tick itself.
    reg  nostrobe;   // 1 = address error cycle: without data strobes and without data (sampled from aerr at S1)
    // The address error cycle (nostrobe) terminates by itself at the first S5, without DTACK, as measured on the reference core:
    //   a DTACK generator that answers only to cycles with a data strobe (like m68k_cen_dtack) never acknowledges it, and
    //   waiting for DTACK would leave AS low forever.
    wire term_now = (~dtack_n) | (~berr_n) | nostrobe;
    wire retry_set = ~berr_n & halt & ~rmw_cyc & ~nostrobe;   // decided with the termination (S5): BERR with HALT, not RMW, not address error
    wire term_s5 = (st == S5) & term_now;   // termination decided in S5 (the RMW write half ends through S11/S13: never retried)
    reg  vpa_wait;   // 1 = VPA cycle recognized (at a previous S5), waiting for E
    // VMA: at the S5 tick in which e_cnt goes 3->4 (2 clocks before the rise at 6), with VPA recognized at a
    //   PREVIOUS S5 (vpa_wait registered) or at this one (vpa_n low now, see below).
    // Single-phase mode (cen2 = cen every tick): S4/S5 alternate at every tick and e_cnt
    //   advances at every tick -> e_cnt==3/8 can ALWAYS fall on the S4 ticks (parity) and the VPA cycle
    //   would never end. With cen and cen2 both high in the
    //   same tick (never with the two-phase generator, where they alternate) the E conditions are accepted also in S4.
    wire single_cen = cen & cen2;
    // VMA also at the S5 in which VPA is seen for the FIRST time if e_cnt is 3, as measured on the reference core with wait
    //   states: VMA 1.5 clocks after AS (IACK cycle of 30 ticks); waiting for the recognition at a previous S5 would lose
    //   one whole period of E (70 ticks). docs/m6800-bus.md §4.2 compares this with the worst case of the manual.
    wire vma_go = ((st == S5) | (single_cen & (st == S4))) & (vpa_wait | ~vpa_n) & (e_cnt == 5'd3);

    // TWO-PHASE advance: EVEN states (including IDLE=S0) on phi1 (cen); ODD
    // states on phi2 (cen2). st[0]=0 -> even. IDLE has st[0]=1 but it is S0 (even).
    wire is_even   = (st == IDLE) ? 1'b1 : (st[0] == 1'b0);
    wire phase_tick = is_even ? cen : cen2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            begin st       <= IDLE; at_s7 <= ((IDLE) == S7); end
            as_n     <= 1'b1;
            uds_n    <= 1'b1;
            lds_n    <= 1'b1;
            rw       <= 1'b1;   // read by default
            vma_n    <= 1'b1;
            vpa_wait <= 1'b0;
            rmw_cyc  <= 1'b0;
            nostrobe <= 1'b0;
            bus_done <= 1'b0;
            data_out <= 16'd0;
            data_oe  <= 1'b0;
        end else if (phase_tick) begin
            bus_done <= 1'b0;   // 1-phase pulse
            if (vma_go) vma_n <= 1'b0;   // VMA synchronous to E (only in the S4/S5 loop of a VPA cycle)

            case (st)
                // ---- S0 (phi1) : start of the cycle when req arrives ----
                IDLE: begin
                    // R/W goes back high at S0 (every IDLE tick), not at S7, and falls at S2 together with AS (S2 branch), as
                    // measured on the reference core; MC68000UM 5.1.2: R/W driven high in S0, low with AS at the rising edge of S2.
                    rw <= 1'b1;
                    if (req) begin
                        as_n  <= 1'b1;       // AS not yet asserted
                        uds_n <= 1'b1;
                        lds_n <= 1'b1;
                        rmw_cyc <= rmw & ~we;
                        nostrobe <= 1'b0;    // the address error is decided at S1, where the address from the register is ready
                        begin st    <= S1; at_s7 <= ((S1) == S7); end
                    end
                end

                // ---- S1 (phi2) : address valid on the bus (from the sequencer) ----
                S1: begin nostrobe <= aerr; begin st <= S2; at_s7 <= ((S2) == S7); end end   // address error: cycle without data strobes (decided here, before AS at S2)

                // ---- S2 (phi1) : RISING edge -> asserts AS ----
                S2: begin
                    as_n <= 1'b0;
                    // R/W falls here, with AS; the direction is 'we' of the word that issued req (same microword).
                    // Retry (rerun): direction, strobes and data are those registered by the first run of the cycle.
                    rw <= rerun ? rw : ~we;
                    if (rerun ? rw : ~we) begin
                        // READ: AS and UDS/LDS asserted together at S2 (MC68000UM 5.1.1). Address error: strobes negated.
                        uds_n <= ~((rerun ? uds_k : uds_req) & ~nostrobe);
                        lds_n <= ~((rerun ? lds_k : lds_req) & ~nostrobe);
                    end else begin
                        // WRITE: R/W falls here with AS. The data strobes at S4. The data register loads here; the
                        // data bus is driven from S3 (data_oe, S3 branch below: MC68000UM 5.1.2 p. 5-6, STATE 3).
                        data_out <= rerun ? data_out : (nostrobe ? 16'd0 : wdata);   // address error: data not driven; retry: data of the first run
                    end
                    begin st <= S3; at_s7 <= ((S3) == S7); end
                end

                // ---- S3 (phi2) : WRITE -> drives the data bus ("During S3, the data bus is driven out of the high-impedance
                // state as the data to be written is placed on the bus", MC68000UM 5.1.2 p. 5-6). Address error (nostrobe):
                // the data bus is never driven. The next state of data_oe depends on st, rw and nostrobe only: rw already
                // holds the direction of the cycle, also in a retry (S2 branch). ----
                S3: begin
                    if (!rw) data_oe <= ~nostrobe;
                    begin st <= S4; at_s7 <= ((S4) == S7); end
                end

                // ---- S4 (phi1) : WRITE -> asserts UDS/LDS ----
                // The sampling of DTACK happens at the END of S4, on the
                // FALLING edge (phi2, S5 branch): it is the contract of the manual (MC68000UM 5.1:
                // DTACK/BERR/VPA sampled on every falling edge from S4) and
                // it is where the reference core samples.
                // Sampling at S4/phi1 would move the window half a clock
                // earlier: with DTACK arriving between the two instants the count
                // of the wait states would differ by one whole period. The write strobes
                // are asserted here, at the rising edge of S4 (MC68000UM 5.1.2).
                S4: begin
                    if (!rw) begin
                        uds_n <= ~((rerun ? uds_k : uds_req) & ~nostrobe);   // address error: strobes negated; retry: strobes of the first run
                        lds_n <= ~((rerun ? lds_k : lds_req) & ~nostrobe);
                    end
                    // single-cen (see vma_go): VPA termination also from S4
                    if (single_cen & vpa_wait & ~vma_n & (e_cnt == 5'd8)) begin st <= S6; at_s7 <= ((S6) == S7); end
                    else begin st <= S5; at_s7 <= ((S5) == S7); end
                end

                // ---- S5 (phi2) : SAMPLES DTACK (end of S4). Wait state = return
                // to S4: every S4+S5 round is a whole period, like the real 68000.
                //   VPA: stays in the loop as long as VMA is asserted and E is about to fall:
                //   e_cnt 8->9 here, S6 (cen), S7 (cen2) with e_cnt 9->0 = falling edge
                //   of E exactly at the S7 tick (AS negated), like the 68000.
                S5: begin
                    if (term_now)
                        begin st <= S6; at_s7 <= ((S6) == S7); end
                    else if (~vpa_n) begin
                        vpa_wait <= 1'b1;
                        begin st <= (~vma_n & (e_cnt == 5'd8)) ? S6 : S4; at_s7 <= (((~vma_n & (e_cnt == 5'd8)) ? S6 : S4) == S7); end
                    end else
                        begin st <= S4; at_s7 <= ((S4) == S7); end
                end

                // ---- S6 (phi1) : the slave data is on the bus (read); it is accepted at the next edge, the one entering S7
                // (rdata block above). The data strobes stay active until S7.
                S6: begin
                    begin st <= S7; at_s7 <= ((S7) == S7) & ~retry; end   // retry: S7 without completion (done_now = at_s7 & cen2 stays 0)
                end

                // ---- S7 (phi2) : NEGATES AS/UDS/LDS, releases the bus, end of cycle ----
                S7: begin
                    rmw_cyc <= 1'b0;
                    if (rmw_cyc && !berr_q) begin
                        // read half of an RMW (TAS): data delivered (done), strobes negated,
                        // AS STAYS low; the write from the sequencer is awaited in S8.
                        // Terminated by BERR (berr_q): no S8, the whole RMW is aborted and AS is negated like any cycle
                        // (5.4.1); the bus error exception follows (never a retry: note p. 5-27).
                        uds_n    <= 1'b1;
                        lds_n    <= 1'b1;
                        bus_done <= 1'b1;
                        begin st       <= S8; at_s7 <= ((S8) == S7); end
                    end else begin
                        as_n     <= 1'b1;
                        uds_n    <= 1'b1;
                        lds_n    <= 1'b1;
                        vma_n    <= 1'b1;        // VMA negated with AS (VPA cycle)
                        vpa_wait <= 1'b0;
                        data_oe  <= 1'b0;        // releases the data bus
                        bus_done <= ~retry;      // completion pulse; retry: none, the cycle runs again
                        // retry: the processor waits here (AS negated, no completion) while HALT is asserted or the bus is
                        // granted; 'rerun' is armed at the first phi2 with rerun_ok and the cycle restarts from S2 at the
                        // next one (S0/S1 of the retried cycle = this wait), with the parameters of the first run.
                        if (retry)
                            begin st <= rerun ? S2 : S7; at_s7 <= 1'b0; end
                        else
                            begin st       <= IDLE; at_s7 <= ((IDLE) == S7); end
                    end
                end

                // ---- RMW (TAS), write half: S8 (phi1) held until the write req arrives ----
                S8: begin
                    if (req && we) begin
                        rw       <= 1'b0;            // R/W low (S12 of the 68000)
                        data_out <= wdata;
                        begin st       <= S10; at_s7 <= ((S10) == S7); end
                    end else
                        begin st <= S8; at_s7 <= ((S8) == S7); end
                end
                // S10 (phi1): data on the bus ; S11 (phi2) ; S12 (phi1): strobe ; S13 (phi2): DTACK
                S10: begin
                    data_oe <= 1'b1;
                    uds_n   <= ~uds_req;   // RMW: write strobes already from S10 (2 clocks like S16..S19 of the 68000)
                    lds_n   <= ~lds_req;
                    begin st      <= S11; at_s7 <= ((S11) == S7); end
                end
                // S11 (phi2): samples DTACK one phase after the strobe, like S5 for the read; success -> S6/S7 (AS
                //   negated at S7, one clock later), as measured on the reference core: strobe->end 6 ticks with immediate DTACK,
                //   10 with DTACK one clock later. Without DTACK: S12/S13, the wait loop, one clock per round.
                S11: begin st <= term_now ? S6 : S12; at_s7 <= ((term_now ? S6 : S12) == S7); end
                S12: begin
                    uds_n <= ~uds_req;
                    lds_n <= ~lds_req;
                    begin st    <= S13; at_s7 <= ((S13) == S7); end
                end
                S13: begin st <= term_now ? S6 : S12; at_s7 <= ((term_now ? S6 : S12) == S7); end   // wait state: samples every clock; the end goes through S6/S7

                default: begin st <= IDLE; at_s7 <= ((IDLE) == S7); end
            endcase
        end
    end

    // Retry flags: synchronous reset and defined power-up (zero), separate from the state register.
    always @(posedge clk) begin
        if (!rst_n) begin retry <= 1'b0; rerun <= 1'b0; uds_k <= 1'b0; lds_k <= 1'b0; berr_q <= 1'b0; end
        else if (phase_tick) begin
            if (st == S2 && !rerun && ~we) begin uds_k <= uds_req & ~nostrobe; lds_k <= lds_req & ~nostrobe; end   // read: strobes of the first run
            if (st == S4 && !rerun && !rw) begin uds_k <= uds_req & ~nostrobe; lds_k <= lds_req & ~nostrobe; end   // write: idem
            if (term_s5) begin retry <= retry_set; rerun <= 1'b0; berr_q <= ~berr_n; end   // decided with the termination (S5); a retried cycle may be retried again
            if (st == S7) berr_q <= 1'b0;
            if (st == S7 && !rmw_cyc) begin
                if (retry) begin
                    if (rerun) retry <= 1'b0;   // restart (S7 -> S2): the next termination decides afresh
                    else rerun <= rerun_ok;     // HALT negated and bus free: arm the restart one phi2 later
                end else rerun <= 1'b0;
            end
        end
    end

`ifdef VERILATOR
    // SAFETY NET: at_s7 must equal (st == S7) at every clock, except in the S7 of a retry (no completion)
    always @(posedge clk) if (rst_n && at_s7 !== ((st == S7) & ~retry)) begin $display("at_s7 flag MISMATCH t=%0t at_s7 %b st %0d retry %b", $time, at_s7, st, retry); $fatal(1); end
`endif
endmodule

`default_nettype wire
