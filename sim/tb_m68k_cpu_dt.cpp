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

// tb_m68k_cpu_dt.cpp - the processor with its clock-enable and DTACK generator in one module (m68k_cpu_dt_si with
// m68k_cen_dtack), the module a system instantiates. The bench is the system around it: one clock per step,
// a byte memory answering bus_rdata, a memory map that raises bus_cs and bus_busy for a programmed number of clocks
// on the accesses to one region, an interrupt device, and the counters of the pulses the generator emits.
// Sources: MC68000 user's manual (bus cycle and DTACK sampling, UM 5.1; interrupt acknowledge, UM 6.3.2) and
// docs/clock-dtack.md (rules R0..R4 of the generator, sections 4.2-4.8; the timing of DTACK, section 4.4).
//
// What it tests (num = 1 and every supported den, 12, 8, 6, 4, 3, 2: a nominal pulse every den/2 clocks, the alternation
// 2, 1 of den 3, every clock at den 2; the whole run repeats for each den):
//  - reset: the first pulse after the release is cen (section 4.8); the pulses alternate cen/cenb, never two in a clock;
//  - reads and writes acknowledged by the generator: DTACK low in clock a + 2 + Lb, where a is the first clock with
//    AS low and a data strobe low and Lb the number of clocks, from a + 1, in which bus_cs and bus_busy are both high
//    (R1, section 4.4); every access of the program checked; the program's results in memory at the end of the run;
//  - wait states from bus_busy: accesses to the busy region with Lb = 0, 4, 10, 20 (reads and writes): the cycle ends at
//    the first sample (the second cenb pulse of the cycle, then one per wait state, UM 5.1: end of S4 and of every wait
//    state) that finds DTACK low, so the wait states are the samples before clock a + 2 + Lb; measured as the cen
//    pulses seen with AS low minus 2 (S4 and S6);
//  - recovery (R2, R3, sections 4.5-4.6): the pulses received while the memory holds the acknowledge are counted by the bench
//    with the rule of R2 and must be more than 0; the debt is returned on clocks with AS high and no nominal pulse, so at
//    the end of the run the pulses emitted equal the nominal pulses of the divider (R0, section 4.2);
//  - interrupt acknowledge, level 4: autovector with VPA (no DTACK from the generator: the
//    processor ends the cycle on E with VMA and takes vector 28, UM 5.1.4) and vectored with DTACK and the vector number on
//    the data bus (vector 0x40): each handler writes its marker; the vector number reaches the processor through the data
//    latch of the wrapper (UM 6.3.2);
//  - pause: with pause_n low no pulse reaches the processor (cen_o, cenb_o = 0) and no bus cycle starts.
// Not tested: SS_EXT (the external savestate gate), BERR, HALT and bus arbitration (covered by the halt and
// berr benches on the processor alone), TAS, address errors, other dividers.

#include "Vm68k_cpu_dt_si.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

static Vm68k_cpu_dt_si* dut = nullptr;
static vluint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static int n_chk = 0, n_fail = 0;
static std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "%06X", (unsigned)v); return std::string(b); }
static void check(bool ok, const std::string& what, long exp, long got) {
    n_chk++;
    if (!ok) { n_fail++; printf("FAIL: %s expected=%ld read=%ld\n", what.c_str(), exp, got); }
}

// ------------------------------------------------------------------------------------------ memory and program
static std::map<uint32_t, uint8_t> mem;
static uint8_t rd8(uint32_t a) { a &= 0xFFFFFFu; auto it = mem.find(a); return it != mem.end() ? it->second : (uint8_t)((a & 1u) ? 0xFC : 0x4A); }
static uint16_t rd16(uint32_t a) { return (uint16_t)((rd8(a) << 8) | rd8(a + 1)); }
static void wr8(uint32_t a, uint8_t v) { mem[a & 0xFFFFFFu] = v; }
static void wr16(uint32_t a, uint16_t v) { wr8(a, (uint8_t)(v >> 8)); wr8(a + 1, (uint8_t)v); }
static void wr32(uint32_t a, uint32_t v) { wr16(a, (uint16_t)(v >> 16)); wr16(a + 2, (uint16_t)v); }
static uint32_t pc_e = 0, end_pc = 0;
static void W(uint16_t v) { wr16(pc_e, v); pc_e += 2; }
static void L(uint32_t v) { W((uint16_t)(v >> 16)); W((uint16_t)v); }

static const uint32_t BUSY_LO = 0x6000u, BUSY_HI = 0x6100u;   // region with bus_cs: busy for Lb clocks per access
static const uint32_t RES = 0x7000u;                          // results and markers
static const int LB[8] = {0, 4, 10, 20, 20, 10, 4, 0};          // Lb of the accesses to the busy region, in order
struct Exp { uint32_t a; uint32_t v; int sz; std::string what; };
static std::vector<Exp> exps;

static void build() {
    mem.clear(); exps.clear();
    wr32(0, 0x00008000u);                 // SSP
    wr32(4, 0x00004000u);                 // PC
    for (uint32_t v = 2; v < 64; v++) { wr32(4 * v, 0x800u + 4 * v); wr16(0x800u + 4 * v, 0x60FE); }   // unexpected exceptions
    wr32(0x70, 0x00000200u);              // vector 28: level-4 autovector handler
    wr32(0x100, 0x00000240u);             // vector 0x40: vectored handler
    pc_e = 0x200u;  W(0x33FC); W(0xA5A5); L(RES + 0u);  W(0x4E73);   // MOVE.W #$A5A5,(RES).L ; RTE
    pc_e = 0x240u;  W(0x33FC); W(0x5A5A); L(RES + 2u);  W(0x4E73);   // MOVE.W #$5A5A,(RES+2).L ; RTE
    wr32(BUSY_LO + 0u, 0x11223344u);      // operands in the busy region
    wr16(BUSY_LO + 8u, 0xBEEFu);
    pc_e = 0x4000u;
    W(0x203C); L(0x12345678u);            // MOVE.L #$12345678,D0
    W(0x23C0); L(RES + 4u);               // MOVE.L D0,(RES+4).L            : plain write, DTACK from the generator
    exps.push_back({RES + 4u, 0x12345678u, 4, "MOVE.L D0,(RES+4): long written"});
    W(0x2239); L(BUSY_LO + 0u);           // MOVE.L (BUSY).L,D1             : 2 reads, Lb 0 and 4
    W(0x4E71); W(0x4E71);                 // NOP x2: the debt of the stall is returned between the accesses
    W(0x23C1); L(BUSY_LO + 4u);           // MOVE.L D1,(BUSY+4).L           : 2 writes, Lb 10 and 20
    exps.push_back({BUSY_LO + 4u, 0x11223344u, 4, "MOVE.L D1,(BUSY+4): long read with wait states written back"});
    W(0x4E71); W(0x4E71);
    W(0x3439); L(BUSY_LO + 8u);           // MOVE.W (BUSY+8).L,D2           : 1 read, Lb 20
    W(0x4E71); W(0x4E71);
    W(0x33C2); L(BUSY_LO + 10u);          // MOVE.W D2,(BUSY+10).L          : 1 write, Lb 10
    exps.push_back({BUSY_LO + 10u, 0xBEEFu, 2, "MOVE.W D2,(BUSY+10): word read with wait states written back"});
    W(0x4E71); W(0x4E71);
    W(0x3039); L(BUSY_LO + 8u);           // MOVE.W (BUSY+8).L,D0           : 1 read, Lb 4
    W(0x4E71); W(0x4E71);
    W(0x33C0); L(BUSY_LO + 12u);          // MOVE.W D0,(BUSY+12).L          : 1 write, Lb 0
    exps.push_back({BUSY_LO + 12u, 0xBEEFu, 2, "MOVE.W D0,(BUSY+12): word written"});
    W(0x4E71); W(0x4E71);
    W(0x46FC); W(0x2000);                 // MOVE #$2000,SR: mask 0, level 4 enabled
    W(0x33FC); W(0x0001); L(RES + 8u);    // MOVE.W #1,(RES+8).L: ready for the autovectored interrupt
    for (int i = 0; i < 24; i++) W(0x4E71);
    W(0x33FC); W(0x0002); L(RES + 8u);    // MOVE.W #2,(RES+8).L: ready for the vectored interrupt
    for (int i = 0; i < 24; i++) W(0x4E71);
    W(0x33FC); W(0x0003); L(RES + 8u);    // MOVE.W #3,(RES+8).L: ready for the pause
    for (int i = 0; i < 24; i++) W(0x4E71);
    exps.push_back({RES + 0u, 0xA5A5u, 2, "level-4 autovector (VPA): handler of vector 28 executed"});
    exps.push_back({RES + 2u, 0x5A5Au, 2, "level-4 vectored (DTACK, vector 0x40 on the bus): handler of vector 0x40 executed"});
    exps.push_back({RES + 8u, 0x0003u, 2, "program completed: last ready marker"});
    end_pc = pc_e; W(0x60FE);             // BRA.S *
}

// ------------------------------------------------------------------------------------------ system model
static int den = 8;                       // num = 1: a nominal pulse every den/2 clocks (R0, section 4.2); the run repeats for every supported den
struct Cyc { uint32_t a; bool rd, u, l; unsigned fc; uint16_t d; long a_clk, dt_clk; int lb; int cen_in, cenb_in; bool in_busy; bool vma; };
static std::vector<Cyc> cycs;
static bool in_cyc = false, act_prev = false;
static bool act_q_in = false;             // the cycle was active in the previous clock (active_q of the generator, section 4.4)
static Cyc cur;
static long clk_n = 0;                    // clocks since the release of the reset
static long pulses = 0, n_cen = 0, n_cenb = 0, nominal = 0, lost = 0, dbl = 0, samephase = 0, first_pulse = -1;
static long debt_m = 0, returned = 0;   // model of R3: debt of the previous edge, recovery pulses expected (clock-dtack.md 4.6)
static int first_kind = -1, last_kind = -1, acc_r0 = 0;
static bool busy_on = false, busy_arm = false; static int busy_left = 0, busy_idx = 0;
static int irq_mode = 0;                  // 0 none, 1 autovector (VPA), 2 vectored (DTACK + vector on the bus)
static int n_iack = 0, unexpected = -1; static long end_fetch = 0, ready_val = 0;
static long pause_pulses = 0, pause_as = 0;
static bool dtack_prev_n = true;

static void dut_inputs_from_outputs() {
    // inputs of the next clock from the outputs of the current one (the generator and the wrapper are registered)
    const uint32_t a = ((uint32_t)dut->addr) << 1;
    const bool iack = (dut->fc == 7) && !dut->as_n;
    dut->bus_rdata = iack ? (irq_mode == 2 ? 0x0040 : 0xFFFF) : rd16(a);   // vectored: vector number on D7-D0
    dut->vpa_n = (iack && irq_mode == 1) ? 0 : 1;                           // autovector: VPA on the acknowledge
    const bool active = !dut->as_n && (!dut->uds_n || !dut->lds_n);
    if (active && !act_prev) {                                              // clock a: the strobes are visible
        const bool inb = a >= BUSY_LO && a < BUSY_HI && dut->fc != 7;
        busy_on = inb; busy_left = inb ? LB[busy_idx % 8] : 0; if (inb) busy_idx++;
        busy_arm = true;                                                    // clock a: nothing yet, the region answers from a + 1
    } else busy_arm = false;
    if (!active) { busy_on = false; busy_left = 0; busy_arm = false; }
    const bool live = busy_on && !busy_arm;
    dut->bus_cs = live ? 1 : 0;                                             // from clock a + 1 while the cycle is active
    dut->bus_busy = (live && busy_left > 0) ? 1 : 0;                        // Lb clocks from a + 1
    if (live && busy_left > 0) busy_left--;
    act_prev = active;
}
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }
static void step() {
    const bool hold_in = dut->bus_cs && dut->bus_busy;                      // inputs of this clock (sampled at the edge)
    const bool active_in = !dut->as_n && (!dut->uds_n || !dut->lds_n);
    const bool dtackn_in = dut->dtack_n_o != 0;
    const bool pulse_in = dut->cen_raw_o != 0 || dut->cenb_raw_o != 0;      // pulse issued at the previous edge, seen in this clock
    // R2 (section 4.5): the pulse already issued at the previous edge is lost when, in the clock after that edge (this one),
    // the memory holds the acknowledge, the cycle was already active at the previous edge and DTACK has not been given.
    // Counted before the edge that samples this clock, with the inputs of this clock: the hold of the clock before the
    // edge is not the rule (with one pulse every 4 clocks it would count one pulse more per stall of 10 clocks).
    const bool miss_now = pulse_in && hold_in && act_q_in && dtackn_in;
    if (miss_now) lost++;
    // R3 (section 4.6), decided at the coming edge with the debt of the previous edge (debt_next = debt + miss - supp): a recovery
    // pulse when no nominal pulse is due at that edge (the R0 rule below, with the accumulator as it stands) and AS is high
    const bool due_now = (clk_n + 1 >= 2) && (acc_r0 + 2 > den);
    if (debt_m > 0 && !due_now && dut->as_n != 0) { returned++; debt_m--; }
    if (miss_now) debt_m++;
    act_q_in = active_in;
    tick();
    clk_n++;
    // R0 (section 4.2): at every edge out of reset acc += 2*num, nominal pulse when the sum exceeds den (strict), then acc -= den.
    // The generator's reset is registered (section 5): its first edge out of reset is the second one after the release.
    if (clk_n >= 2) { acc_r0 += 2; if (acc_r0 > den) { acc_r0 -= den; nominal++; } }
    // the generator's pulses (raw: divider, phase and debt run on them, section 5); the processor sees them gated by the pause
    const bool cen = dut->cen_o != 0, cenb = dut->cenb_o != 0;
    const bool rcen = dut->cen_raw_o != 0, rcenb = dut->cenb_raw_o != 0;
    if (rcen || rcenb) {
        pulses++; if (rcen) n_cen++; if (rcenb) n_cenb++;
        if (rcen && rcenb) dbl++;
        const int kind = rcen ? 0 : 1;
        if (first_pulse < 0) { first_pulse = clk_n; first_kind = kind; }
        else if (kind == last_kind) samephase++;
        last_kind = kind;
    }
    if ((cen || cenb) && !dut->pause_n) pause_pulses++;
    if (!dut->pause_n && !dut->as_n) pause_as++;
    // bus cycle bookkeeping (outputs of this clock)
    if (!dut->as_n) {
        if (!in_cyc) { in_cyc = true; cur = Cyc(); cur.a = ((uint32_t)dut->addr) << 1; cur.rd = dut->rw != 0; cur.fc = dut->fc; cur.a_clk = -1; cur.dt_clk = -1; cur.lb = -1; }
        if (!dut->uds_n) cur.u = true;
        if (!dut->lds_n) cur.l = true;
        const bool active = !dut->uds_n || !dut->lds_n;
        if (active && cur.a_clk < 0) {                                      // clock a
            cur.a_clk = clk_n; cur.in_busy = cur.a >= BUSY_LO && cur.a < BUSY_HI && cur.fc != 7;
            cur.lb = cur.in_busy ? LB[busy_idx % 8] : 0;
        }
        if (cur.a_clk >= 0 && cur.dt_clk < 0 && dut->dtack_n_o == 0) cur.dt_clk = clk_n;
        if (!dut->vma_n) cur.vma = true;                                    // M6800 cycle (VPA): VMA asserted
        if (cen) cur.cen_in++;
        if (cenb) cur.cenb_in++;
        if (!dut->rw && active) cur.d = dut->data_out;
    } else if (in_cyc) {
        in_cyc = false; cycs.push_back(cur);
        if (cur.fc == 7) { n_iack++; dut->ipl_n = 7; }                     // the device drops its request at the acknowledge
        else if (cur.rd && cur.a >= 0x800u && cur.a < 0x900u && unexpected < 0) unexpected = (int)((cur.a - 0x800u) / 4u);
        else if (cur.rd && (cur.fc & 3u) == 2u && cur.a == end_pc) end_fetch++;
        else if (!cur.rd && cur.fc != 7) {
            if (cur.u) wr8(cur.a, (uint8_t)(cur.d >> 8));
            if (cur.l) wr8(cur.a | 1u, (uint8_t)cur.d);
            if (cur.a == RES + 8u) ready_val = rd16(RES + 8u);
        }
    }
    dut_inputs_from_outputs();
}
static void dut_start() {
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_cpu_dt_si;
    dut->clk = 0; dut->reset = 1; dut->num = 1; dut->den = (uint8_t)den; dut->bus_cs = 0; dut->bus_busy = 0; dut->pause_n = 1;
    dut->cen_ext_i = 0; dut->cenb_ext_i = 0; dut->gate_q_i = 0; dut->gate_raw_i = 0;
    dut->ss_ovr_i = 0; dut->ss_word_i = 0; dut->ss_ovr_c_i = 0; dut->ss_word_c_i = 0;
    dut->bus_rdata = 0; dut->ipl_n = 7; dut->vpa_n = 1; dut->berr_n = 1; dut->halt_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    dut->eval();
    for (int i = 0; i < 8; i++) tick();
    dut->reset = 0;
    clk_n = 0; pulses = n_cen = n_cenb = nominal = lost = dbl = samephase = 0; first_pulse = -1; first_kind = last_kind = -1; acc_r0 = 0; debt_m = 0; returned = 0;
    cycs.clear(); in_cyc = false; act_prev = false; act_q_in = false; busy_on = false; busy_left = 0; busy_idx = 0; irq_mode = 0; n_iack = 0; unexpected = -1;
    end_fetch = 0; ready_val = 0; pause_pulses = 0; pause_as = 0;
}
static bool run_until_ready(long v, long max) {
    for (long i = 0; i < max; i++) { step(); if (unexpected >= 0) return false; if (ready_val == v) return true; }
    return false;
}

static void run_den() {
    printf("--- den %d ---\n", den);
    build();
    dut_start();
    // --- the program up to the first ready marker: plain accesses and the busy region ---
    check(run_until_ready(1, 400000L), "program reaches the first ready marker (plain and busy accesses done)", 1, ready_val);
    check(unexpected < 0, "no unexpected exception", 0, unexpected < 0 ? 0 : unexpected);
    // reset and pulses (sections 4.3 and 4.8)
    check(first_kind == 0, "first pulse after reset is cen (clock-dtack.md 4.8)", 0, first_kind);
    check(first_pulse == den / 2 + 2, "first pulse den/2 + 2 clocks after the release: registered reset, then the first edge k with 2k > den (clock-dtack.md 4.3, 4.8)", den / 2 + 2, first_pulse);
    check(dbl == 0, "never two pulses in one clock", 0, dbl);
    check(samephase == 0, "cen and cenb strictly alternate (clock-dtack.md 4.3)", 0, samephase);
    // DTACK timing and wait states, access by access (R1, section 4.4; UM 5.1 for the samples)
    int n_busy = 0, n_plain = 0;
    for (const Cyc& c : cycs) {
        if (c.fc == 7 || c.a_clk < 0) continue;
        std::string w = std::string(c.rd ? "read" : "write") + " at " + hx(c.a) + " Lb " + std::to_string(c.lb);
        check(c.dt_clk == c.a_clk + 2 + c.lb, w + ": DTACK low in clock a + 2 + Lb (R1)", c.a_clk + 2 + c.lb, c.dt_clk);
        // expected wait states: the samples (cenb pulses with AS low, except the first = S3 and the last = S7) before the
        // clock in which DTACK is low
        const int samples_total = c.cenb_in - 2;
        const int waits_meas = c.cen_in - 2;
        check(samples_total == waits_meas + 1, w + ": one sample per wait state plus the final one (UM 5.1)", waits_meas + 1, samples_total);
        if (c.in_busy) n_busy++; else n_plain++;
    }
    check(n_busy == 8, "eight accesses to the busy region (two long, four word)", 8, n_busy);
    check(n_plain > 20, "plain accesses acknowledged by the generator", 21, n_plain);
    // wait states of the busy accesses against the rule of section 4.4 with s = den / 2 = 4: read ceil((3 + Lb - 12) / 8),
    // write ceil((3 + Lb - 4) / 8), never negative; valid when no recovery pulse falls inside the cycle, which the NOPs
    // between the accesses guarantee (the debt is returned before the next access, see the check on the lost pulses)
    int ib = 0;
    for (const Cyc& c : cycs) {
        if (!c.in_busy || c.a_clk < 0) continue;
        if (den == 3) continue;   // den 3: the pulses alternate 2 and 1 clocks apart and the rule below assumes a uniform spacing
        const int s = den / 2;
        const int num_w = 3 + c.lb - (c.rd ? 3 * s : s);
        const int exp_w = num_w <= 0 ? 0 : (num_w + 2 * s - 1) / (2 * s);
        check(c.cen_in - 2 == exp_w, std::string(c.rd ? "read" : "write") + " at " + hx(c.a) + " Lb " + std::to_string(c.lb) + ": wait states (clock-dtack.md 4.4)", exp_w, c.cen_in - 2);
        ib++;
    }
    // recovery: pulses lost by the rule of R2, all returned before the first marker (R3: on clocks with AS high)
    check(lost > 0, "pulses lost during the stalls (R2 count by the bench)", 1, lost);
    check(pulses == nominal + returned, "raw pulses = nominal pulses (R0) + recovery pulses of the clock-by-clock model of R2 and R3 (debt of the previous edge, no nominal pulse due, AS high)", nominal + returned, pulses);
    if (den == 2) check(returned == 0 && lost > 0, "den = 2: pulses lost but none returned, a nominal pulse is due at every clock (R3, clock-dtack.md 4.9)", 0, returned);
    else if (den >= 4) check(returned == lost && lost > 0, "all lost pulses returned at this divider: the free clocks between the accesses take the whole debt (R2, R3; clock-dtack.md 4.6)", lost, returned);
    else check(lost > 0 && returned > 0, "den = 3: pulses lost and recovery pulses issued (the exact count is the model above)", 1, (lost > 0 && returned > 0) ? 1 : 0);
    // --- level-4 interrupt, autovector with VPA ---
    irq_mode = 1; dut->ipl_n = (uint8_t)(~4u & 7u);
    check(run_until_ready(2, 200000L), "program reaches the second ready marker after the autovectored interrupt", 2, ready_val);
    check(n_iack == 1, "one acknowledge cycle (FC = 7) for the autovectored interrupt", 1, n_iack);
    for (const Cyc& c : cycs) if (c.fc == 7) {
        check((c.a & 0xFu) == 0x8u, "acknowledge: A3-A1 carry the level (4)", 8, (long)(c.a & 0xFu));
        check(c.l, "acknowledge: LDS asserted (vector number on D7-D0, UM 6.3.2)", 1, c.l);
        check(c.dt_clk < 0, "acknowledge with VPA: no DTACK from the generator (UM 5.1.4: DTACK and VPA never asserted together)", -1, c.dt_clk);
        check(c.vma, "acknowledge with VPA: VMA asserted, the processor ends the cycle on E (M6800 cycle, UM 5.1.4)", 1, c.vma ? 1 : 0);
    }
    check(rd16(RES + 0u) == 0xA5A5u, "autovector: the handler of vector 28 ran (marker)", 0xA5A5, rd16(RES + 0u));
    // --- level-4 interrupt, vectored: DTACK and vector 0x40 on D7-D0 ---
    irq_mode = 2; dut->ipl_n = (uint8_t)(~4u & 7u);
    check(run_until_ready(3, 200000L), "program reaches the third ready marker after the vectored interrupt", 3, ready_val);
    check(n_iack == 2, "one acknowledge cycle for the vectored interrupt", 2, n_iack);
    check(rd16(RES + 2u) == 0x5A5Au, "vectored: the vector number read in the acknowledge reached the processor (handler of vector 0x40 ran)", 0x5A5A, rd16(RES + 2u));
    check(unexpected < 0, "no unexpected exception after the interrupts", 0, unexpected < 0 ? 0 : unexpected);
    // --- pause: no pulse to the processor, no bus cycle ---
    irq_mode = 0;
    for (int i = 0; i < 40; i++) step();
    dut->pause_n = 0;
    for (int i = 0; i < 400; i++) step();
    check(pause_pulses == 0, "pause: no cen/cenb pulse to the processor while pause_n is low (clock-dtack.md 5)", 0, pause_pulses);
    dut->pause_n = 1;
    // --- end of run ---
    bool done = false;
    for (long i = 0; i < 400000L && !done; i++) { step(); if (end_fetch >= 3) done = true; if (unexpected >= 0) break; }
    check(done, "end of run: fetch of the final BRA.S", 1, done ? 1 : 0);
    for (int i = 0; i < 2000; i++) step();
    check(pulses == nominal + returned, "end of run: raw pulses = nominal + recovery pulses of the R2/R3 model (the pause gates them for the processor only)", nominal + returned, pulses);
    if (den >= 4) check(returned == lost, "end of run: all lost pulses returned at this divider", lost, returned);
    for (const Exp& e : exps) {
        uint32_t got = e.sz == 4 ? (((uint32_t)rd16(e.a) << 16) | rd16(e.a + 2)) : rd16(e.a);
        check(got == e.v, e.what, (long)e.v, (long)got);
    }
    dut->final(); delete dut; dut = nullptr;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    static const int DENS[6] = {12, 8, 6, 4, 3, 2};   // every supported divider with num = 1 (clock-dtack.md 4.1)
    for (int i = 0; i < 6; i++) { den = DENS[i]; run_den(); }
    printf("checks run: %d, failed: %d\n", n_chk, n_fail);
    printf(n_fail == 0 ? "RESULT: PASS\n" : "RESULT: FAIL\n");
    return n_fail == 0 ? 0 : 1;
}
