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

// tb_m68k_berr.cpp - BERR (bus error, group 0 exception, vector 2) and double fault, on the full processor.
// Sources: MC68000 user's manual (bus error and halt, the bus error exception and its frame, Table 6-2 of the vectors);
// docs/bus-control-signals.md (Bus error, Retry of a bus cycle, Double bus fault, High impedance).
//
// What it tests:
//  - BERR on: data read (MOVE.W (A0),D1), data write (MOVE.W D1,(A0)), instruction fetch (JMP (A0) into the BERR
//    area), .L read with d16 (computed access address), byte write. The 0x8000-0x8FFF area answers with BERR instead
//    of DTACK (same timing as the run's DTACK); the aborted cycle writes nothing.
//  - the cycle that receives BERR is the only one in the BERR area per case (the processor aborts and enters the
//    exception), with the expected R/W and FC; vector 2 is read at address 8 in the supervisor data space (Table 6-2,
//    "SD").
//  - group 0 frame (14 bytes at SSP - 14, cells holding sentinels different from the expected values, a different stack
//    per case): status word (bit 4 R/W: 1 = read; bit 3 I/N: 0 = instruction; bits 2..0 FC; bits 15..5 are not defined
//    by the manual and are not checked), access address (32 bits), IR (first opcode of the instruction; not checked in
//    the fetch case, where the manual does not fix it), SR at the time of the fault (upper byte exact; CCR exact only
//    with JMP, which does not touch the CCR), PC: the manual gives it as "from 2 to 10 bytes beyond the first word of
//    the instruction", checked as a range in the data cases, not checked in the fetch case.
//  - after the exception: vector 2 handler, SR with S = 1, T = 0, mask unchanged (7), SSP = SSP - 14.
//  - double fault: SSP inside the BERR area, BERR on a read, the first push of the frame receives BERR: the processor
//    stops and asserts HALT (halted_n low, the pin exposed by the shim), no other bus cycle for 3000 clocks, the handler
//    is not fetched.
//  - every other cycle: FC and areas; two runs of everything: DTACK/BERR with no wait states and with 3 wait states.
//  - retry (BERR with HALT, UM 5.4.2 and Table 5-1 case 5), third run per wait-state setting: MOVE.W (A0),D1 and
//    MOVE.W D1,(A2) are answered with BERR and HALT asserted together; no exception, no AS while HALT is held (80
//    clocks), after HALT is negated the same cycle runs again (address, R/W, FC, strobes, data) and ends with DTACK;
//    D1 holds the word of the retried read (the aborted one returned 0x4AFC), the destination holds the word of the
//    retried write (the aborted one is not committed), SR unchanged apart from the CCR of the moves.
//  - TAS (A0) in the BERR area with HALT asserted together with BERR (sixth case of the main run): no retry (note p.
//    5-27), the read half ends with AS negated, no AS while HALT is held (40 clocks, the model releases it), then the
//    group 0 frame and the handler like the other cases; one cycle in the BERR area. Double fault and retry wait: buses
//    in high impedance (bus_tri = 1, UM 5.4.3 and 5.4.2), driven again at the retried cycle.
//  - double bus fault during the reset processing (UM 6.3.9.1, 5.4.4): BERR on the first read of the reset vector, and an
//    address error on the first instruction fetch (odd initial PC): the processor halts, no exception, no further cycle.
//  - BERR during a group 1 exception (vector read of an ILLEGAL): frame with I/N = 1; BERR answered to the aborted address
//    phase of an address error: the address error alone (UM 6.2, 6.3.10).
//  - BERR kept asserted after the aborted cycle: no bus cycle and address and data buses released (bus_tri = 1) until
//    BERR is negated, then the exception (UM 5.4.1).

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
static uint32_t rd32(uint32_t a) { return ((uint32_t)rd16(a) << 16) | rd16(a + 2); }
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
static void ld_d(int n, uint32_t v) { W((uint16_t)(0x203C | (n << 9))); L(v); }
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
static int rt_arm = 0;                   // victim armed: 1 = next read, 2 = next write, 3 = next read with HALT released by the model
static uint32_t rt_addr = 0xFFFFFFFFu;   // ... at this address (0xFFFFFFFF = any)
static bool rt_cyc = false;              // the cycle in progress is the victim
static bool rt_auto = false;             // mode 3: HALT released by the model rt_rel steps after the victim cycle ends
static int rt_rel = 0, rt_as_bad = 0;    // steps left with HALT held by the model; AS seen low meanwhile (must stay 0)
static int rt_seen = 0;                  // victims answered
static int bh_arm = 0, bh_rel = 0;       // BERR held: steps the model keeps BERR asserted after the next BERR cycle ends (0 = off), steps left
static int bh_as_bad = 0, bh_tri_bad = 0;   // while BERR is held: AS seen low, bus_tri seen low (both must stay 0)
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
static void step() {   // n_as: clocks with AS low (S3 = 1 .. S5 = 3); DTACK/BERR sampled at end of S4 (UM 5.1), 2*ws wait clocks
    if (rt_rel > 0) {   // mode 3: HALT held by the model after the victim cycle; the processor must stay stopped (no AS)
        if (!dut->as_n) rt_as_bad++;
        if (--rt_rel == 0) dut->halt_n = 1;
    }
    if (!dut->as_n) {
        if (bh_rel > 0) bh_as_bad++;   // no cycle may start while BERR stays asserted
        if (!in_cyc) {
            in_cyc = true; n_as = 0; cur = Cyc();
            // victim: the first cycle of the armed direction (and address, when given) is answered with BERR + HALT
            uint32_t a0 = ((uint32_t)dut->addr) << 1;
            rt_cyc = rt_arm != 0 && ((rt_arm == 2) != (dut->rw != 0)) && (rt_addr == 0xFFFFFFFFu || a0 == rt_addr);
            if (rt_cyc) { rt_auto = rt_arm == 3; rt_arm = 0; rt_seen++; }
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
        if (in_cyc) { in_cyc = false; commit(cur); if (rt_cyc && rt_auto) rt_rel = 40; if (cur.berr && !cur.rt && bh_arm > 0) { bh_rel = bh_arm; bh_arm = 0; } rt_cyc = false; }
        dut->dtack_n = 1;
        if (bh_rel > 0) { if (!dut->bus_tri) bh_tri_bad++; dut->berr_n = 0; bh_rel--; }   // BERR held after the aborted cycle (UM 5.4.1)
        else dut->berr_n = 1;   // BERR negated with AS: at least one clock before HALT (UM 5.4.2)
    }
    dut->cen = 1; dut->cen2 = 1;
    tick();
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
    rt_arm = 0; rt_addr = 0xFFFFFFFFu; rt_cyc = false; rt_auto = false; rt_rel = 0; rt_as_bad = 0; rt_seen = 0;
    bh_arm = 0; bh_rel = 0; bh_as_bad = 0; bh_tri_bad = 0;
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
static const uint32_t HND = 0x5800u;      // vector 2 handler
static const uint32_t BLO = 0x8000u, BHI = 0x9000u;
struct BCase { std::string nm; uint32_t p, f, acc; bool rd, pc_chk; unsigned fc; };
static std::vector<BCase> bcs;
static uint32_t res = 0x7000u;
// one case: own stack at 'top', A0 = a0v, then the instruction ins[0..n-1] that accesses the BERR area; the handler
// returns to cont (A6)
static void berr_case(const char* s, uint32_t top, uint32_t a0v, const uint16_t* ins, int n, uint32_t acc, bool rd, unsigned fc,
                      bool fetch, uint8_t ccr) {
    BCase b; b.nm = std::string("case ") + std::to_string(bcs.size()) + " " + s; b.acc = acc; b.rd = rd; b.fc = fc; b.pc_chk = !fetch;
    ld_a(7, top);
    uint32_t a6at = pc_e; ld_a(6, 0);
    ld_a(0, a0v);
    ld_d(1, 0x13579BDFu);
    set_ccr(ccr);
    b.p = pc_e;
    for (int i = 0; i < n; i++) W(ins[i]);
    W(0x4AFC);                            // if BERR did not interrupt the instruction: ILLEGAL, unexpected exception
    wr32(a6at + 2, pc_e);                 // A6 = resume address
    b.f = top - 14;                       // group 0 frame: 7 words
    uint16_t ssw = (uint16_t)((rd ? 0x10u : 0u) | (fc & 7u));   // I/N = 0: instruction in progress
    expect_n(b.f + 0, ssw, 0x001Fu, 2, b.nm + ": frame status word (R/W, I/N, FC)");
    expect_n(b.f + 2, acc, 0xFFFFFFFFu, 4, b.nm + ": access address in the frame");
    expect_n(b.f + 6, ins[0], fetch ? 0u : 0xFFFFu, 2, b.nm + ": IR in the frame");
    expect_n(b.f + 8, 0x2700u | ccr, fetch ? 0xFFFFu : 0xFFE0u, 2, b.nm + ": SR in the frame");
    expect_n(b.f + 10, 0u, 0u, 4, b.nm + ": PC in the frame (as a range)");
    expect_n(res, 0x2700u | ccr, fetch ? 0xFFFFu : 0xFFE0u, 2, b.nm + ": SR in the handler (S = 1, T = 0, mask 7)");
    expect_n(res + 2, b.f, 0xFFFFFFFFu, 4, b.nm + ": SSP in the handler = SSP - 14");
    res += 6;
    bcs.push_back(b);
}
static void handler() {   // MOVE SR,(A5)+ ; MOVE.L A7,(A5)+ ; JMP (A6)
    pc_e = HND; W(0x40DD); W(0x2ACF); W(0x4ED6);
}
static void build() {
    bcs.clear(); res = 0x7000u;
    vectors(0x00002000u, 0x00004000u);
    wr32(8, HND);
    prog_area(0x4000u, 0x6000u);
    data_area(0x1F00u, 0x2300u);          // stacks of the cases (frames)
    data_area(0x7000u, 0x7100u);          // handler results
    rgns.push_back({BLO, BHI, 3}); berr_lo = BLO; berr_hi = BHI;
    pc_e = 0x4000u;
    ld_a(5, 0x7000u);
    static const uint16_t i_rd[1] = {0x3210};          // MOVE.W (A0),D1
    static const uint16_t i_wr[1] = {0x3081};          // MOVE.W D1,(A0)
    static const uint16_t i_jmp[1] = {0x4ED0};         // JMP (A0)
    static const uint16_t i_rdl[2] = {0x2228, 0x0004}; // MOVE.L 4(A0),D1
    static const uint16_t i_wrb[1] = {0x1081};         // MOVE.B D1,(A0)
    static const uint16_t i_tas[1] = {0x4AD0};         // TAS (A0)
    berr_case("data read MOVE.W (A0),D1", 0x2000u, 0x8000u, i_rd, 1, 0x8000u, true, 5, false, 0x1F);
    berr_case("data write MOVE.W D1,(A0)", 0x2080u, 0x8010u, i_wr, 1, 0x8010u, false, 5, false, 0x00);
    berr_case("instruction fetch after JMP (A0)", 0x2100u, 0x8020u, i_jmp, 1, 0x8020u, true, 6, true, 0x15);
    berr_case("data read MOVE.L 4(A0),D1", 0x2180u, 0x8030u, i_rdl, 2, 0x8034u, true, 5, false, 0x0A);
    berr_case("byte write MOVE.B D1,(A0)", 0x2200u, 0x8040u, i_wrb, 1, 0x8040u, false, 5, false, 0x11);
    // read-modify-write with HALT asserted together with BERR: never retried (UM note p. 5-27), bus error like the others;
    // the read half ends with AS negated (5.4.1), the processor stays halted while HALT is held, then stacks the frame
    berr_case("read-modify-write TAS (A0) with HALT asserted (no retry)", 0x2280u, 0x8050u, i_tas, 1, 0x8050u, true, 5, false, 0x0A);
    rt_arm = 3; rt_addr = 0x8050u;        // the TAS read in the BERR area also gets HALT; the model releases it 40 clocks later
    end_loop();
    handler();
}
static void check_berr_cycles() {
    int n_area = 0;
    for (const Cyc& c : cycs) if (c.a >= BLO && c.a < BHI) n_area++;
    check(n_area == (int)bcs.size(), "a single cycle in the BERR area per case (the aborted cycle, then the exception)", (uint32_t)bcs.size(), (uint32_t)n_area);
    for (const BCase& b : bcs) {
        int n = 0; const Cyc* fc0 = nullptr;
        for (const Cyc& c : cycs) if (c.a == (b.acc & ~1u)) { n++; if (!fc0) fc0 = &c; }
        check(n == 1, b.nm + ": one cycle at the access address", 1, (uint32_t)n);
        if (!fc0) continue;
        check(fc0->berr, b.nm + ": the cycle received BERR", 1, fc0->berr ? 1u : 0u);
        check(fc0->rd == b.rd, b.nm + ": R/W of the aborted cycle", b.rd ? 1u : 0u, fc0->rd ? 1u : 0u);
        check(fc0->fc == b.fc, b.nm + ": FC of the aborted cycle", b.fc, fc0->fc);
        if (b.pc_chk) {
            uint32_t pcv = rd32(b.f + 10);
            check(pcv >= b.p + 2 && pcv <= b.p + 10, b.nm + ": PC in the frame between instruction + 2 and instruction + 10 (manual)", b.p + 2, pcv);
        }
    }
    int nvec = 0;
    for (const Cyc& c : cycs) if (c.rd && (c.a == 8u || c.a == 10u)) nvec++;
    check(nvec == 2 * (int)bcs.size(), "vector 2 read (two words) once per case", 2u * (uint32_t)bcs.size(), (uint32_t)nvec);
}
// double fault: SSP in the BERR area
static void build_df() {
    bcs.clear(); res = 0x7000u;
    vectors(0x00002000u, 0x00004000u);
    wr32(8, HND);
    prog_area(0x4000u, 0x6000u);
    data_area(0x7000u, 0x7100u);          // no write expected
    rgns.push_back({BLO, BHI, 3}); berr_lo = BLO; berr_hi = BHI;
    pc_e = 0x4000u;
    ld_a(5, 0x7000u);
    ld_a(7, 0x8F00u);
    ld_a(0, 0x8000u);
    W(0x3210);                            // MOVE.W (A0),D1: BERR, and the frame goes to 0x8EF2..0x8EFF: BERR during the exception
    W(0x4AFC);
    handler();
}
static void run_df() {
    long i = 0;
    for (; i < 200000L && dut->halted_n; i++) step();
    check(!dut->halted_n, "double fault: the processor asserts HALT (halted_n low)", 0, dut->halted_n);
    size_t nc = cycs.size();
    int as_low = 0, hl = 0, tri_lo = 0;
    for (int k = 0; k < 3000; k++) { step(); if (!dut->as_n) as_low++; if (dut->halted_n) hl++; if (!dut->bus_tri) tri_lo++; }
    check(as_low == 0, "double fault: no AS after the halt", 0, (uint32_t)as_low);
    check(hl == 0, "double fault: HALT stays asserted", 0, (uint32_t)hl);
    check(tri_lo == 0, "double fault: address and data buses in high impedance while halted (bus_tri = 1, UM 5.4.3)", 0, (uint32_t)tri_lo);
    check(cycs.size() == nc, "double fault: no bus cycle after the halt", (uint32_t)nc, (uint32_t)cycs.size());
    int nb = 0, nbw = 0, nh = 0;
    for (const Cyc& c : cycs) {
        if (c.berr) { nb++; if (!c.rd) nbw++; }
        if (c.rd && c.a >= HND && c.a < HND + 8u) nh++;
    }
    check(nb >= 2, "double fault: BERR on the read and on the first push", 2, (uint32_t)nb);
    check(nbw >= 1, "double fault: the second BERR is a frame write", 1, (uint32_t)nbw);
    check(nh == 0, "double fault: the vector 2 handler is not fetched", 0, (uint32_t)nh);
    check(unexpected < 0, "double fault: no other handler fetched", 0, (uint32_t)(unexpected < 0 ? 0 : unexpected));
}

// ---------------------------------------------------------------------------------------------- retry (BERR with HALT)
static bool step_until(bool (*cond)(), long max_ticks) {   // advance until cond() is true (before the clock)
    for (long i = 0; i < max_ticks; i++) { if (cond()) return true; step(); if (unexpected >= 0) return false; }
    return cond();
}
static bool as_low() { return !dut->as_n; }
static bool as_high() { return dut->as_n != 0; }
static bool rt_started() { return rt_cyc; }
static void put_word(uint32_t a, uint16_t v) { wr16(a, v); img0[a] = (uint8_t)(v >> 8); img0[a + 1u] = (uint8_t)v; }   // operand in a data area

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
// program of the retry run: a data read and a data write answered with BERR + HALT, then the proofs that only the retried
// cycles counted. Vector 2 keeps the default handler at 0x808: a bus error would show up as an unexpected exception.
static void build_rt() {
    bcs.clear(); res = 0x7000u;
    vectors(0x00002000u, 0x00004000u);
    prog_area(0x4000u, 0x6000u);
    data_area(0x3000u, 0x3100u);          // operands of the retried cycles
    data_area(0x7000u, 0x7100u);          // results
    put_word(0x3010u, 0x2468u);
    pc_e = 0x4000u;
    ld_a(0, 0x3010u);
    ld_a(2, 0x3020u);
    ld_d(1, 0x13579BDFu);
    set_ccr(0x15);
    W(0x3210);                            // MOVE.W (A0),D1: the read answered with BERR + HALT, then retried
    W(0x3481);                            // MOVE.W D1,(A2): the write answered with BERR + HALT, then retried
    W(0x21C1); W(0x7000);                 // MOVE.L D1,(0x7000).W
    expect_n(0x7000u, 0x13572468u, 0xFFFFFFFFu, 4, "retry: D1 = word read by the retried cycle (0x2468, not the 0x4AFC of the aborted one)");
    expect_n(0x3020u, 0x2468u, 0xFFFFu, 2, "retry: word written once, by the retried cycle");
    W(0x40F8); W(0x7004);                 // MOVE SR,(0x7004).W
    expect_n(0x7004u, 0x2710u, 0xFFFFu, 2, "retry: no exception (S = 1, T = 0, mask 7); CCR from the moves (X kept, N = Z = V = C = 0)");
    end_loop();
}
static void run_rt() {
    retry_phase(1, 0x3010u, "retry: data read MOVE.W (A0),D1");
    retry_phase(2, 0x3020u, "retry: data write MOVE.W D1,(A2)");
    bool done = run_to_end(200000L);
    check(done, "retry: end of run: fetch of the final BRA.S at " + hx(end_pc), 1, done ? 1u : 0u);
    check(!dut->illegal, "retry: no illegal instruction exception", 0, dut->illegal);
    check_bus();
    check_image();
}

// ------------------------------------------------------------------------- double bus fault during the reset processing
// UM 6.3.9.1 p. 6-16: "If a bus error occurs during the exception processing for a bus error, an address error, or a reset,
// the processor halts and all processing ceases"; 5.4.4 p. 5-28: "A double bus fault occurs during a reset operation when a
// bus error occurs while the processor is reading the vector table (before the first instruction is executed)". Two runs: a
// bus error on the first read of the reset vector (address 0) and an address error on the first instruction fetch (odd
// initial PC). Expected in both: halted_n low, no further bus cycle, no handler fetched (vectors 2 and 3 keep the default
// handlers at 0x808 and 0x80C: a fetch there would be an unexpected exception).
static void build_rst_berr() {
    bcs.clear(); res = 0x7000u;
    vectors(0x00002000u, 0x00004000u);
    prog_area(0x4000u, 0x6000u);
    berr_lo = 0u; berr_hi = 8u;           // the reads of the reset vector answer with BERR: the first one (address 0) halts the processor
    pc_e = 0x4000u;
    end_loop();
}
// Bus error during the processing of a group 1 exception (MC68000UM 6.3.9.1 p. 6-16, figure 6-7: "the processor is not
// processing an instruction if it is processing a group 0 or a group 1 exception", I/N = 1): the read of the vector of an
// ILLEGAL instruction exception (vector 4, long at 0x10, supervisor data) receives BERR. Expected: one aborted cycle (read,
// FC = 5, address 0x10), no double fault (ILLEGAL is group 1), the group 0 frame below the 3-word frame already pushed by the
// ILLEGAL exception (SSP - 6 - 14) with status word R/W = 1, I/N = 1, FC = 5, the access address 0x10 and SR with S = 1 and
// T = 0; then the vector 2 handler, which resumes the program after the ILLEGAL.
static uint32_t g1_f = 0;
static void build_g1() {
    bcs.clear(); res = 0x7000u;
    vectors(0x00002000u, 0x00004000u);
    wr32(8, HND);
    prog_area(0x4000u, 0x6000u);
    data_area(0x1F00u, 0x2300u);          // stack (frames)
    data_area(0x7000u, 0x7100u);          // results
    berr_lo = 0x10u; berr_hi = 0x14u;     // vector 4 (ILLEGAL): its read receives BERR
    pc_e = 0x4000u;
    ld_a(5, 0x7000u);
    ld_a(7, 0x2200u);
    uint32_t a6at = pc_e; ld_a(6, 0);
    set_ccr(0x15);
    uint32_t ill_pc = pc_e;
    W(0x4AFC);                            // ILLEGAL: group 1 exception; the read of its vector is the aborted cycle
    wr32(a6at + 2, pc_e);                 // the vector 2 handler resumes here
    end_loop();
    expect_n(0x21FAu, 0x2715u, 0xFFE0u, 2, "group 1 fault: SR in the ILLEGAL frame (pushed before the fault)");
    expect_n(0x21FCu, ill_pc, 0xFFFFFFFFu, 4, "group 1 fault: PC in the ILLEGAL frame = address of the ILLEGAL (UM 6.3.6)");
    g1_f = 0x2200u - 6u - 14u;
    expect_n(g1_f + 0, 0x001Du, 0x001Fu, 2, "group 1 fault: frame status word R/W = 1, I/N = 1, FC = 5 (UM 6.3.9.1, figure 6-7)");
    expect_n(g1_f + 2, 0x10u, 0xFFFFFFFFu, 4, "group 1 fault: access address in the frame = 0x10 (vector 4)");
    expect_n(g1_f + 6, 0u, 0u, 2, "group 1 fault: IR in the frame (not checked)");
    expect_n(g1_f + 8, 0x2715u, 0xFFE0u, 2, "group 1 fault: SR in the frame (S = 1, T = 0, mask 7)");
    expect_n(g1_f + 10, 0u, 0u, 4, "group 1 fault: PC in the frame (not checked)");
    expect_n(res, 0x2715u, 0xFFE0u, 2, "group 1 fault: SR in the vector 2 handler (S = 1, T = 0, mask 7)");
    expect_n(res + 2, g1_f, 0xFFFFFFFFu, 4, "group 1 fault: SSP in the handler = SSP - 6 - 14");
    res += 6;
    handler();
}
static void run_g1() {
    bool done = run_to_end(2000000L);
    check(done, "group 1 fault: end of run (the vector 2 handler resumed the program)", 1, done ? 1u : 0u);
    check(unexpected < 0, "group 1 fault: no other handler fetched (no double fault: ILLEGAL is group 1)", 0, (uint32_t)(unexpected < 0 ? 0 : unexpected));
    check(dut->halted_n != 0, "group 1 fault: the processor is not halted", 1, dut->halted_n);
    int nb = 0; const Cyc* b = nullptr;
    for (const Cyc& c : cycs) if (c.berr) { nb++; if (!b) b = &c; }
    check(nb == 1, "group 1 fault: one cycle answered with BERR", 1, (uint32_t)nb);
    if (b) {
        check(b->a == 0x10u, "group 1 fault: the aborted cycle is the vector 4 read at 0x10", 0x10u, b->a);
        check(b->rd, "group 1 fault: the aborted cycle is a read", 1, b->rd ? 1u : 0u);
        check(b->fc == 5u, "group 1 fault: FC of the aborted cycle = 5 (supervisor data)", 5u, b->fc);
    }
    int nvec = 0;
    for (const Cyc& c : cycs) if (c.rd && (c.a == 8u || c.a == 10u)) nvec++;
    check(nvec == 2, "group 1 fault: vector 2 read (two words) once", 2, (uint32_t)nvec);
}
// Address error whose aborted cycle is answered with BERR (MC68000UM 6.2 p. 6-8: within group 0 the address error comes
// before the bus error; 6.3.10 p. 6-19: "the bus cycle is aborted"): a word read at an odd address runs its address phase
// (AS without data strobes, ended at its first S5) and the model, with ws = 0, answers BERR to it; with ws = 3 the BERR comes
// after the phase has ended and nothing is answered. Expected in both: the address error exception alone (vector 3 read
// once, frame with R/W = 1, I/N = 0, FC = 5 and the odd address, the vector 3 handler resumes the program), vector 2 never
// read, the processor not halted.
// BERR kept asserted after the aborted cycle (MC68000UM 5.4.1 p. 5-24: "As long as BERR remains asserted, the data and
// address buses are in the high-impedance state"; "After the aborted bus cycle is terminated and BERR is negated, the
// processor enters exception processing"): the model answers BERR to a word read in the BERR area and keeps it asserted
// for 80 steps after AS is negated. Expected: no bus cycle and bus_tri = 1 for the whole hold, then the bus error
// exception as in the plain case (frame, vector 2 handler, program resumed), the processor not halted.
static void build_bh() {
    bcs.clear(); res = 0x7000u;
    vectors(0x00002000u, 0x00004000u);
    wr32(8, HND);
    prog_area(0x4000u, 0x6000u);
    data_area(0x1F00u, 0x2300u);          // stack (frames)
    data_area(0x7000u, 0x7100u);          // results
    rgns.push_back({BLO, BHI, 3}); berr_lo = BLO; berr_hi = BHI;
    pc_e = 0x4000u;
    ld_a(5, 0x7000u);
    static const uint16_t ins[1] = {0x3210};   // MOVE.W (A0),D1
    berr_case("word read in the BERR area, BERR held after the cycle", 0x2200u, BLO, ins, 1, BLO, true, 5, false, 0x15);
    end_loop();
    handler();
    bh_arm = 80;
}
static void run_bh() {
    bool done = run_to_end(2000000L);
    check(done, "BERR held: end of run (the vector 2 handler resumed the program)", 1, done ? 1u : 0u);
    check(unexpected < 0, "BERR held: no other handler fetched", 0, (uint32_t)(unexpected < 0 ? 0 : unexpected));
    check(dut->halted_n != 0, "BERR held: the processor is not halted (the stacking waits for the negation of BERR, no double fault)", 1, dut->halted_n);
    check(bh_rel == 0 && bh_arm == 0, "BERR held: the hold was armed and ran out", 0, (uint32_t)(bh_rel + bh_arm));
    check(bh_as_bad == 0, "BERR held: no AS while BERR stays asserted after the aborted cycle (UM 5.4.1)", 0, (uint32_t)bh_as_bad);
    check(bh_tri_bad == 0, "BERR held: address and data buses in high impedance while BERR stays asserted (bus_tri = 1, UM 5.4.1)", 0, (uint32_t)bh_tri_bad);
    check_berr_cycles();
}
static const uint32_t HND3 = 0x5900u;     // vector 3 handler
static void build_aerr_berr() {
    bcs.clear(); res = 0x7000u;
    vectors(0x00002000u, 0x00004000u);
    wr32(8, HND); wr32(12, HND3);
    prog_area(0x4000u, 0x6000u);
    data_area(0x1F00u, 0x2300u);          // stack (frames)
    data_area(0x7000u, 0x7100u);          // results
    rgns.push_back({BLO, BHI, 3}); berr_lo = BLO; berr_hi = BHI;
    pc_e = 0x4000u;
    ld_a(5, 0x7000u);
    static const uint16_t ins[1] = {0x3210};   // MOVE.W (A0),D1 with A0 odd: address error
    berr_case("word read at an odd address in the BERR area", 0x2200u, BLO + 1u, ins, 1, BLO + 1u, true, 5, false, 0x15);
    end_loop();
    handler();
    pc_e = HND3; W(0x40DD); W(0x2ACF); W(0x4ED6);   // vector 3 handler: MOVE SR,(A5)+ ; MOVE.L A7,(A5)+ ; JMP (A6)
}
static void run_aerr_berr() {
    bool done = run_to_end(2000000L);
    check(done, "address error with BERR: end of run (the vector 3 handler resumed the program)", 1, done ? 1u : 0u);
    check(unexpected < 0, "address error with BERR: no other handler fetched", 0, (uint32_t)(unexpected < 0 ? 0 : unexpected));
    check(dut->halted_n != 0, "address error with BERR: the processor is not halted", 1, dut->halted_n);
    int nb = 0; const Cyc* b = nullptr;
    for (const Cyc& c : cycs) if (c.berr) { nb++; if (!b) b = &c; }
    check(nb == 1, "address error with BERR: one cycle answered with BERR (the aborted address phase; with ws = 3 the answer comes after its S5, still inside the cycle)", 1, (uint32_t)nb);
    if (b) {
        check(b->a == BLO, "address error with BERR: the aborted cycle is at the odd address (word address on the bus)", BLO, b->a);
        check(b->rd && b->fc == 5u, "address error with BERR: the aborted cycle is a read with FC = 5", 1, (b->rd && b->fc == 5u) ? 1u : 0u);
        check(!b->u && !b->l, "address error with BERR: no data strobe in the aborted cycle", 0, (b->u ? 2u : 0u) | (b->l ? 1u : 0u));
    }
    int nv3 = 0, nv2 = 0;
    for (const Cyc& c : cycs) if (c.rd) { if (c.a == 12u || c.a == 14u) nv3++; if (c.a == 8u || c.a == 10u) nv2++; }
    check(nv3 == 2, "address error with BERR: vector 3 read (two words) once", 2, (uint32_t)nv3);
    check(nv2 == 0, "address error with BERR: vector 2 never read (the BERR of the aborted cycle is not a second exception)", 0, (uint32_t)nv2);
}
static void build_rst_aerr() {
    bcs.clear(); res = 0x7000u;
    vectors(0x00002000u, 0x00004001u);    // odd initial PC: address error on the first instruction fetch, still reset processing
    prog_area(0x4000u, 0x6000u);
    pc_e = 0x4000u;
    end_loop();
}
static void run_rst_halt(const std::string& what, size_t ncyc) {
    long i = 0;
    for (; i < 2000L && dut->halted_n; i++) step();
    check(!dut->halted_n, what + ": the processor halts (halted_n low) instead of taking the exception", 0, dut->halted_n);
    size_t nc = cycs.size();
    check(nc == ncyc, what + ": bus cycles before the halt (vector reads and the aborted one)", (uint32_t)ncyc, (uint32_t)nc);
    int as_low = 0, hl = 0, tri_lo = 0;
    for (int k = 0; k < 3000; k++) { step(); if (!dut->as_n) as_low++; if (dut->halted_n) hl++; if (!dut->bus_tri) tri_lo++; }
    check(as_low == 0, what + ": no AS after the halt", 0, (uint32_t)as_low);
    check(hl == 0, what + ": HALT stays asserted", 0, (uint32_t)hl);
    check(tri_lo == 0, what + ": buses in high impedance while halted (bus_tri = 1)", 0, (uint32_t)tri_lo);
    check(cycs.size() == nc, what + ": no bus cycle after the halt", (uint32_t)nc, (uint32_t)cycs.size());
    check(unexpected < 0, what + ": no handler fetched", 0, (uint32_t)(unexpected < 0 ? 0 : unexpected));
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    static const int WS[2] = {0, 3};
    for (int k = 0; k < 2; k++) {
        ws = WS[k];
        tag = "[ws=" + std::to_string(ws) + "]";
        clear_all();
        build();
        dut_start();
        bool done = run_to_end(2000000L);
        check(done, "end of run: fetch of the final BRA.S at " + hx(end_pc), 1, done ? 1u : 0u);
        check(!dut->illegal, "no illegal instruction exception", 0, dut->illegal);
        check(rt_seen == 1, "TAS with BERR and HALT: the read half of the RMW was answered with BERR and HALT", 1, (uint32_t)rt_seen);
        check(rt_as_bad == 0, "TAS with BERR and HALT: no AS while HALT is held after the bus error termination (halted, UM 5.4.3)", 0, (uint32_t)rt_as_bad);
        check_berr_cycles();
        check_bus();
        check_image();
        dut_stop();

        tag = "[ws=" + std::to_string(ws) + " double fault]";
        clear_all();
        build_df();
        dut_start();
        run_df();
        check_bus();
        check_image();
        dut_stop();

        tag = "[ws=" + std::to_string(ws) + " retry]";
        clear_all();
        build_rt();
        dut_start();
        run_rt();
        dut_stop();

        tag = "[ws=" + std::to_string(ws) + " reset bus error]";
        clear_all();
        build_rst_berr();
        dut_start();
        run_rst_halt("bus error on the reset vector read", 1);
        dut_stop();

        tag = "[ws=" + std::to_string(ws) + " reset address error]";
        clear_all();
        build_rst_aerr();
        dut_start();
        run_rst_halt("address error on the first fetch after reset (odd PC)", 5);
        dut_stop();

        tag = "[ws=" + std::to_string(ws) + " group 1 fault]";
        clear_all();
        build_g1();
        dut_start();
        run_g1();
        check_bus();
        check_image();
        dut_stop();

        tag = "[ws=" + std::to_string(ws) + " address error with BERR]";
        clear_all();
        build_aerr_berr();
        dut_start();
        run_aerr_berr();
        check_image();
        dut_stop();

        tag = "[ws=" + std::to_string(ws) + " BERR held]";
        clear_all();
        build_bh();
        dut_start();
        run_bh();
        check_bus();
        check_image();
        dut_stop();
    }
    return finish();
}
