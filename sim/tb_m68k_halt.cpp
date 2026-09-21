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

// tb_m68k_halt.cpp - HALT input, bus arbitration (BR/BG/BGACK) and the RESET instruction, on the full processor.
// Sources: MC68000 user's manual (halt, three-wire bus arbitration), PRM 4 RESET; docs/bus-control-signals.md (HALT and
// the halted state, Bus arbitration, RESET output, Retry of a bus cycle, High impedance).
//
// Program: a loop that counts D0 from 1 to 40 and writes each value with MOVE.L D0,(A0)+ to 0x3000..0x309F, then RESET
// and the final saves. The testbench intervenes halfway through the run:
//  - HALT: asserted while AS is low (in the middle of a cycle). The cycle in progress ends normally (AS negated, a single
//    cycle closed), then no AS and no cycle for 600 clocks as long as HALT stays low (the processor stops at the end of
//    the cycle) with the address and data buses in high impedance (bus_tri = 1, UM 5.4.3); once HALT is released the
//    buses are driven again (bus_tri = 0) and the program resumes.
//  - arbitration: BR asserted while AS is low. BG must arrive (within 400 clocks). The manual allows BG also during the
//    cycle in progress: the requester waits for AS and DTACK negated before asserting BGACK, then negates BR; with
//    BGACK low for 600 clocks: outputs in high impedance (bus_tri = 1), no AS, no cycle; BG negated after BGACK.
//    Once BGACK is released: bus_tri = 0 and the program resumes.
//  - RESET (PRM 4 RESET, supervisor mode): the instruction executes and the program continues; the CCR does not change.
//  - at end of run: the 40 longs written once each in the right order (sentinels around them), D0 = 40, A0 = 0x30A0,
//    SR; data FC 5 and program FC 6, no access outside the areas; two runs, DTACK with no wait states and with 3 wait
//    states.
//  - retry (BERR with HALT, UM 5.4.2 and Table 5-1 case 5): a data write of the loop and then an instruction fetch are
//    answered with BERR and HALT asserted together; the cycle terminates without an exception, no AS and buses in high
//    impedance (bus_tri = 1, UM 5.4.2) while HALT is held (80 clocks), and after HALT is negated the same cycle runs
//    again with the buses driven (bus_tri = 0 at its AS: address, R/W, FC, strobes, data as before) and ends with
//    DTACK; the aborted read returned 0x4AFC (ILLEGAL if used), the aborted write is not committed: the final image and
//    the end of the run prove that only the retried cycles count.
//  - RESET output: the RESET instruction asserts the RESET pin (reset_out of the shim) for exactly 124 clocks, once per
//    run (UM 5.5 p. 5-29: "assert RESET for 124 clock periods"; the internal state is not affected, see the SR/D0/A0 saves).
// Not tested: halted_n with external HALT (the manual does not distinguish it from the pin itself); retry of a
// read-modify-write cycle (the manual excludes it: bus error whether or not HALT is asserted, note p. 5-27).

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <map>
#include <set>
#include <vector>
#include <string>

// ============================================================================ common infrastructure
static Vm68k_seq* dut = nullptr;
static vluint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static int n_chk = 0, n_fail = 0;
static std::string tag;
static std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "%06X", (unsigned)v); return std::string(b); }
static void check(bool ok, const std::string& what, uint32_t exp, uint32_t got) {
    n_chk++;
    if (!ok) { n_fail++; printf("FAIL: %s %s expected=%08X read=%08X\n", tag.c_str(), what.c_str(), (unsigned)exp, (unsigned)got); }
}

// Byte-wide memory, 24-bit address. Where the testbench has put nothing, every word reads 0x4AFC (ILLEGAL).
static std::map<uint32_t, uint8_t> mem;
static std::map<uint32_t, uint8_t> img0;                          // initial content of the data areas, rechecked at end of run
static std::map<uint32_t, std::pair<uint8_t, uint8_t> > expb;      // expected byte at end of run: value and mask of the fixed bits
static std::map<uint32_t, std::string> expd;
static std::set<uint32_t> opnd;
static uint8_t rd8(uint32_t a) { a &= 0xFFFFFFu; auto it = mem.find(a); if (it != mem.end()) return it->second; return (a & 1u) ? 0xFC : 0x4A; }
static uint16_t rd16(uint32_t a) { return (uint16_t)((rd8(a) << 8) | rd8(a + 1)); }
static void wr8(uint32_t a, uint8_t v) { mem[a & 0xFFFFFFu] = v; }
static void wr16(uint32_t a, uint16_t v) { wr8(a, (uint8_t)(v >> 8)); wr8(a + 1, (uint8_t)v); }
static void wr32(uint32_t a, uint32_t v) { wr16(a, (uint16_t)(v >> 16)); wr16(a + 2, (uint16_t)v); }

struct Rgn { uint32_t lo, hi; int kind; };   // 0 program (FC 6), 1 data (FC 5), 2 vectors (FC 5), 3 BERR
static std::vector<Rgn> rgns;
static int rgn_of(uint32_t a) { for (size_t i = 0; i < rgns.size(); i++) if (a >= rgns[i].lo && a < rgns[i].hi) return (int)i; return -1; }
static void prog_area(uint32_t lo, uint32_t hi) { rgns.push_back({lo, hi, 0}); }
static void data_area(uint32_t lo, uint32_t hi) {
    rgns.push_back({lo, hi, 1});
    for (uint32_t a = lo; a < hi; a++) { uint8_t s = (uint8_t)(0xA5u ^ (a & 0xFFu) ^ ((a >> 7) & 0xFEu)); wr8(a, s); img0[a & 0xFFFFFFu] = s; }
}
static void expect8(uint32_t a, uint8_t v, uint8_t m, const std::string& d) {
    a &= 0xFFFFFFu;
    auto it = img0.find(a);
    if (it != img0.end() && !opnd.count(a) && ((it->second ^ v) & m) == 0) { it->second = (uint8_t)(it->second ^ 0xFFu); mem[a] = it->second; }
    expb[a] = std::make_pair(v, m); expd[a] = d;
}
static void expect_n(uint32_t a, uint32_t v, uint32_t m, int sz, const std::string& d) {
    for (int i = 0; i < sz; i++) { int sh = 8 * (sz - 1 - i); expect8(a + (uint32_t)i, (uint8_t)(v >> sh), (uint8_t)(m >> sh), d); }
}

static uint32_t pc_e = 0, slot = 0x7000, end_pc = 0;
static void W(uint16_t v) { wr16(pc_e, v); pc_e += 2; }
static void L(uint32_t v) { W((uint16_t)(v >> 16)); W((uint16_t)v); }
static void set_ccr(uint8_t c) { W(0x44FC); W(c); }
static void save_sr(uint16_t e, uint16_t m, const std::string& d) { W(0x40F8); W((uint16_t)slot); expect_n(slot, e, m, 2, d); slot += 2; }
static void save_d(int n, uint32_t e, const std::string& d) { W((uint16_t)(0x21C0 | n)); W((uint16_t)slot); expect_n(slot, e, 0xFFFFFFFFu, 4, d); slot += 4; }
static void save_a(int n, uint32_t e, const std::string& d) { W((uint16_t)(0x21C8 | n)); W((uint16_t)slot); expect_n(slot, e, 0xFFFFFFFFu, 4, d); slot += 4; }
static void ld_a(int n, uint32_t v) { W((uint16_t)(0x207C | (n << 9))); L(v); }
static void end_loop() { end_pc = pc_e; W(0x60FE); }
static void vectors(uint32_t ssp, uint32_t pc0) {
    rgns.push_back({0x000000u, 0x000400u, 2});
    prog_area(0x000800u, 0x000900u);
    wr32(0, ssp); wr32(4, pc0);
    for (uint32_t v = 2; v < 64; v++) { wr32(4 * v, 0x800u + 4 * v); wr16(0x800u + 4 * v, 0x60FE); }
}

struct Cyc { uint32_t a; bool rd, u, l, berr, rt; unsigned fc; uint16_t d; };   // rt: the cycle answered with BERR + HALT (retry)
static std::vector<Cyc> cycs;
static int ws = 0;
static uint32_t berr_lo = 1, berr_hi = 0;
static bool in_cyc = false;
static int n_as = 0;
static Cyc cur;
static int end_fetch = 0, unexpected = -1;
static long n_wr = 0;
static int rt_arm = 0;                   // retry victim armed: 1 = next read, 2 = next write (0 = none)
static uint32_t rt_addr = 0xFFFFFFFFu;   // ... at this address (0xFFFFFFFF = any)
static bool rt_cyc = false;              // the cycle in progress is the victim
static long n_rst = 0;                   // clocks with the RESET output asserted (RESET instruction)
static int rst_edges = 0;                // assertions of the RESET output
static bool rst_prev = false;
static void commit(const Cyc& c) {
    cycs.push_back(c);
    if (c.rd && c.a >= 0x800u && c.a < 0x900u && unexpected < 0) unexpected = (int)((c.a - 0x800u) / 4u);
    if (c.rd && (c.fc & 3u) == 2u && c.a == end_pc) end_fetch++;
    if (c.berr || c.rd) return;
    n_wr++;
    if (!c.u && !c.l) { check(false, "write without data strobe at " + hx(c.a), 1, 0); return; }
    if (c.u) wr8(c.a, (uint8_t)(c.d >> 8));
    if (c.l) wr8(c.a | 1u, (uint8_t)c.d);
}
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }
static void step() {   // n_as: clocks with AS low (S3 = 1 .. S5 = 3); DTACK sampled at end of S4 (UM 5.1), 2*ws wait clocks
    if (!dut->as_n) {
        if (!in_cyc) {
            in_cyc = true; n_as = 0; cur = Cyc();
            // retry victim: the first cycle of the armed direction (and address, when given) is answered with BERR + HALT
            uint32_t a0 = ((uint32_t)dut->addr) << 1;
            rt_cyc = rt_arm != 0 && ((rt_arm == 1) == (dut->rw != 0)) && (rt_addr == 0xFFFFFFFFu || a0 == rt_addr);
            if (rt_cyc) rt_arm = 0;
        }
        n_as++;
        cur.a = ((uint32_t)dut->addr) << 1; cur.rd = dut->rw != 0; cur.fc = dut->fc;
        bool be = (cur.a >= berr_lo && cur.a < berr_hi) || rt_cyc;
        cur.berr = be; cur.rt = rt_cyc;
        if (!dut->uds_n) cur.u = true;
        if (!dut->lds_n) cur.l = true;
        if (dut->rw) dut->data_in = rt_cyc ? 0x4AFC : rd16(cur.a);   // aborted read: data the processor must not use (ILLEGAL if executed)
        else if (!dut->uds_n || !dut->lds_n) cur.d = dut->data_out;
        bool go = n_as >= 3 + 2 * ws;
        dut->dtack_n = (go && !be) ? 0 : 1;
        dut->berr_n = (go && be) ? 0 : 1;
        if (rt_cyc && go) dut->halt_n = 0;   // retry: HALT asserted together with BERR (UM 5.4.2, Table 5-1 case 5); released by the phase
    } else {
        if (in_cyc) { in_cyc = false; commit(cur); rt_cyc = false; }
        dut->dtack_n = 1; dut->berr_n = 1;   // BERR negated with AS: at least one clock before HALT (UM 5.4.2)
    }
    dut->cen = 1; dut->cen2 = 1;
    tick();
    if (dut->reset_out) { n_rst++; if (!rst_prev) rst_edges++; }   // RESET output: counted in steps; single-cen: a 68000 clock is two steps (one per edge)
    rst_prev = dut->reset_out != 0;
}
static bool run_to_end(long max_ticks) {
    for (long i = 0; i < max_ticks; i++) {
        step();
        if (unexpected >= 0) { check(false, "unexpected exception: fetched the handler of vector " + std::to_string(unexpected), 0, (uint32_t)unexpected); return false; }
        if (end_fetch >= 3) { for (int k = 0; k < 40; k++) step(); return true; }
    }
    return false;
}
static void dut_start() {
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    dut->clk = 0; dut->rst_n = 0; dut->cen = 1; dut->cen2 = 1; dut->start = 0; dut->pc_init = 0; dut->rst_vector = 1;
    dut->ipl = 0; dut->vpa_n = 1; dut->dtack_n = 1; dut->halt_n = 1; dut->berr_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    dut->data_in = 0; dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    dut->eval();
    for (int i = 0; i < 8; i++) tick();
    dut->rst_n = 1;
}
static void dut_stop() { dut->final(); delete dut; dut = nullptr; }
static void clear_all() {
    mem.clear(); img0.clear(); expb.clear(); expd.clear(); opnd.clear(); rgns.clear(); cycs.clear();
    in_cyc = false; n_as = 0; end_fetch = 0; unexpected = -1; n_wr = 0; slot = 0x7000; pc_e = 0; end_pc = 0; berr_lo = 1; berr_hi = 0;
    rt_arm = 0; rt_addr = 0xFFFFFFFFu; rt_cyc = false; n_rst = 0; rst_edges = 0; rst_prev = false;
}
static void check_image() {
    for (auto& kv : img0) {
        uint32_t a = kv.first; uint8_t got = rd8(a);
        auto e = expb.find(a);
        if (e != expb.end()) check(((got ^ e->second.first) & e->second.second) == 0, expd[a] + " [byte " + hx(a) + "]", e->second.first, got);
        else check(got == kv.second, "sentinel intact [byte " + hx(a) + "]", kv.second, got);
    }
}
static void check_bus() {
    for (size_t i = 0; i < cycs.size(); i++) {
        const Cyc& c = cycs[i];
        std::string w = std::string(c.rd ? "read" : "write") + " #" + std::to_string(i) + " at " + hx(c.a);
        int r = rgn_of(c.a);
        if (r < 0) { check(false, w + ": outside the testbench areas", 0, c.a); continue; }
        int k = rgns[(size_t)r].kind;
        if (k == 3) continue;
        if (k != 1) check(c.rd, w + ": read-only area", 1, c.rd ? 1u : 0u);
        unsigned fe = (i < 4) ? 6u : (k == 0 ? 6u : 5u);
        check(c.fc == fe, w + ": FC", fe, c.fc);
        if (k != 1 || i < 4) check(c.u && c.l, w + ": word access (UDS and LDS)", 3, (c.u ? 2u : 0u) | (c.l ? 1u : 0u));
    }
}
static int finish() {
    printf("checks run: %d, failed: %d\n", n_chk, n_fail);
    if (n_fail == 0) { printf("RESULT: PASS\n"); return 0; }
    printf("RESULT: FAIL\n"); return 1;
}

// ============================================================================ testbench program
static const int NCNT = 40;
static void build() {
    vectors(0x00002000u, 0x00004000u);
    prog_area(0x4000u, 0x6000u);
    data_area(0x3000u, 0x3100u);          // counter values
    data_area(0x7000u, 0x7100u);          // results
    pc_e = 0x4000u;
    ld_a(0, 0x3000u);
    W(0x7000);                            // MOVEQ #0,D0
    uint32_t lp = pc_e;
    W(0x5280);                            // ADDQ.L #1,D0
    W(0x20C0);                            // MOVE.L D0,(A0)+
    W(0x0C80); L((uint32_t)NCNT);         // CMPI.L #40,D0
    W((uint16_t)(0x6600 | (uint8_t)(lp - (pc_e + 2))));   // BNE.S lp
    for (int i = 0; i < NCNT; i++) expect_n(0x3000u + 4u * (uint32_t)i, (uint32_t)(i + 1), 0xFFFFFFFFu, 4, "counter: long " + std::to_string(i + 1));
    set_ccr(0x15);
    W(0x4E70);                            // RESET
    save_sr(0x2715, 0xFFFF, "RESET: the program continues, SR unchanged (PRM 4 RESET: CCR not touched)");
    save_d(0, (uint32_t)NCNT, "final D0 = 40");
    save_a(0, 0x3000u + 4u * NCNT, "final A0 = 0x30A0");
    end_loop();
}
static bool step_until(bool (*cond)(), long max_ticks) {   // advance until cond() is true (before the clock)
    for (long i = 0; i < max_ticks; i++) { if (cond()) return true; step(); if (unexpected >= 0) return false; }
    return cond();
}
static bool as_low() { return !dut->as_n; }
static bool as_high() { return dut->as_n != 0; }
static bool bg_low() { return !dut->bg_n; }
static long wr_target = 0;
static bool wr_reached() { return n_wr >= wr_target; }
static bool rt_started() { return rt_cyc; }

// Retry of a bus cycle (MC68000UM 5.4.2 p. 5-26, Table 5-1 case 5): the model answers the victim cycle with BERR and asserts
// HALT at the same time. Expected (manual): the cycle terminates (AS negated) without an exception; the processor stays
// stopped while HALT is held (BERR is negated with AS, at least one clock before HALT); once HALT is negated it runs the
// same cycle again (same address, R/W, FC, data strobes and, for a write, data) and that cycle terminates with DTACK. The
// aborted read returned 0x4AFC (ILLEGAL if it were executed or loaded) and the aborted write is not committed by the model:
// only the retried cycle counts.
static void retry_phase(int arm, uint32_t addr, const std::string& what) {
    rt_arm = arm; rt_addr = addr;
    check(step_until(rt_started, 4000L), what + ": the victim cycle starts", 1, 0);
    check(step_until(as_high, 100L), what + ": the cycle terminates (AS negated) with BERR and HALT asserted", 1, 0);
    step();                               // the model closes the cycle (AS high seen) and negates BERR; HALT stays asserted
    size_t nc = cycs.size();
    check(nc > 0 && cycs[nc - 1].rt && cycs[nc - 1].berr, what + ": the terminated cycle is the one answered with BERR", 1, (nc > 0 && cycs[nc - 1].rt) ? 1u : 0u);
    int lows = 0, tri_lo = 0;
    for (int i = 0; i < 80; i++) { step(); if (!dut->as_n) lows++; if (!dut->bus_tri) tri_lo++; }
    check(lows == 0, what + ": HALT held after BERR: no AS for 80 clocks (processor halted, cycle not yet retried)", 0, (uint32_t)lows);
    check(cycs.size() == nc, what + ": HALT held: no bus cycle", (uint32_t)nc, (uint32_t)cycs.size());
    check(tri_lo == 0, what + ": HALT held: address and data buses in high impedance (bus_tri = 1, UM 5.4.2)", 0, (uint32_t)tri_lo);
    check(unexpected < 0, what + ": no exception taken for the aborted cycle", 0, (uint32_t)(unexpected < 0 ? 0 : unexpected));
    dut->halt_n = 1;                      // HALT negated: the processor must run the same cycle again
    check(step_until(as_low, 100L), what + ": HALT negated: a cycle starts", 1, 0);
    check(!dut->bus_tri, what + ": retried cycle started: buses driven again (bus_tri = 0)", 0, dut->bus_tri);
    check(step_until(as_high, 100L), what + ": the retried cycle terminates", 1, 0);
    step();
    check(cycs.size() == nc + 1, what + ": exactly one cycle since the release", (uint32_t)(nc + 1), (uint32_t)cycs.size());
    if (cycs.size() == nc + 1 && nc > 0) {
        const Cyc& v = cycs[nc - 1]; const Cyc& r = cycs[nc];
        check(r.a == v.a, what + ": retried cycle: same address", v.a, r.a);
        check(r.rd == v.rd, what + ": retried cycle: same direction (R/W)", v.rd ? 1u : 0u, r.rd ? 1u : 0u);
        check(r.fc == v.fc, what + ": retried cycle: same function code", v.fc, r.fc);
        check(r.u == v.u && r.l == v.l, what + ": retried cycle: same data strobes", (v.u ? 2u : 0u) | (v.l ? 1u : 0u), (r.u ? 2u : 0u) | (r.l ? 1u : 0u));
        if (!v.rd) check(r.d == v.d, what + ": retried write: same data", v.d, r.d);
        check(!r.berr, what + ": retried cycle terminated by DTACK (no BERR)", 0, r.berr ? 1u : 0u);
    }
}

static void halt_phase() {
    wr_target = 3;
    check(step_until(wr_reached, 200000L), "HALT: the program reaches the third write", 1, 0);
    check(step_until(as_low, 200L), "HALT: a cycle in progress (AS low)", 1, 0);
    size_t nc0 = cycs.size();
    dut->halt_n = 0;
    check(step_until(as_high, 100L), "HALT: the cycle in progress ends (AS negated)", 1, 0);
    step();                               // the model closes the cycle (AS high seen)
    check(cycs.size() == nc0 + 1, "HALT: only the cycle in progress closes", (uint32_t)(nc0 + 1), (uint32_t)cycs.size());
    size_t nc1 = cycs.size();
    int lows = 0, tri_lo = 0;
    for (int i = 0; i < 600; i++) { step(); if (!dut->as_n) lows++; if (!dut->bus_tri) tri_lo++; }
    check(lows == 0, "HALT asserted: no AS for 600 clocks (stopped at end of cycle)", 0, (uint32_t)lows);
    check(cycs.size() == nc1, "HALT asserted: no bus cycle", (uint32_t)nc1, (uint32_t)cycs.size());
    check(tri_lo == 0, "HALT asserted: address and data buses in high impedance while halted (bus_tri = 1, UM 5.4.3)", 0, (uint32_t)tri_lo);
    dut->halt_n = 1;
    step();
    check(!dut->bus_tri, "HALT negated: buses driven again (bus_tri = 0)", 0, dut->bus_tri);
}
static void arb_phase() {
    wr_target = 20;
    check(step_until(wr_reached, 200000L), "BR: the program reaches the twentieth write", 1, 0);
    check(step_until(as_low, 200L), "BR: a cycle in progress (AS low)", 1, 0);
    dut->br_n = 0;
    check(step_until(bg_low, 400L), "BR asserted: the processor asserts BG", 0, dut->bg_n);
    // the requester waits for AS and DTACK negated (end of the processor's cycle), then asserts BGACK and negates BR
    check(step_until(as_high, 100L), "arbitration: the processor's cycle ends", 1, 0);
    step();                               // the model closes the cycle and negates DTACK
    size_t nc = cycs.size();
    dut->bgack_n = 0;
    step();
    dut->br_n = 1;
    int tri_bad = 0, as_bad = 0, bg_off = -1;
    for (int i = 0; i < 600; i++) {
        step();
        if (!dut->bus_tri) tri_bad++;
        if (!dut->as_n) as_bad++;
        if (bg_off < 0 && dut->bg_n) bg_off = i;
    }
    check(tri_bad == 0, "BGACK asserted: processor outputs in high impedance (bus_tri)", 0, (uint32_t)tri_bad);
    check(as_bad == 0, "BGACK asserted: no AS from the processor", 0, (uint32_t)as_bad);
    check(cycs.size() == nc, "BGACK asserted: no processor cycle", (uint32_t)nc, (uint32_t)cycs.size());
    check(bg_off >= 0, "BGACK asserted and BR negated: BG negated", 1, bg_off >= 0 ? 1u : 0u);
    dut->bgack_n = 1;
    step(); step();
    check(!dut->bus_tri, "BGACK negated: the processor takes the bus back (bus_tri = 0)", 0, dut->bus_tri);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    static const int WS[2] = {0, 3};
    for (int k = 0; k < 2; k++) {
        ws = WS[k]; tag = "[ws=" + std::to_string(ws) + "]";
        clear_all();
        build();
        dut_start();
        halt_phase();
        arb_phase();
        retry_phase(2, 0xFFFFFFFFu, "retry (data write)");
        retry_phase(1, 0xFFFFFFFFu, "retry (instruction fetch)");
        bool done = run_to_end(2000000L);
        check(done, "end of run: fetch of the final BRA.S at " + hx(end_pc), 1, done ? 1u : 0u);
        check(!dut->illegal, "no illegal instruction exception", 0, dut->illegal);
        check(n_rst == 2 * 124, "RESET instruction: RESET output asserted for 124 clocks = 248 single-cen steps (UM 5.5 p. 5-29)", 2 * 124, (uint32_t)n_rst);
        check(rst_edges == 1, "RESET instruction: RESET output asserted once", 1, (uint32_t)rst_edges);
        check_bus();
        check_image();
        dut_stop();
    }
    return finish();
}
