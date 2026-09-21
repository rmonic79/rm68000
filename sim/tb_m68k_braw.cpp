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

// tb_m68k_braw.cpp - BRA, BSR, Bcc in the .S and .W forms, and RTS, on the full processor through the m68k_seq shim.
//
// What it tests (expected values written from the manual, none taken from an execution):
//  - Bcc.S and Bcc.W for the 14 conditions HI..LE and for all 16 combinations of N Z V C: 448 cases, taken and not
//    taken. Conditions: PRM sec. 3, condition tests. Branch: PRM 4 Bcc, target = address of the instruction + 2 +
//    displacement.
//    The target is NOT a NOP: the word before the target is ADDQ.B #1,D7 (D7 must stay 0 until the end), the target is
//    MOVEQ #0x11,D0, the not-taken path MOVEQ #0x22,D0, and D0 starts from 0x5A5Axxxx. A base wrong by -2 executes the
//    ADDQ (D7 != 0), one wrong by +2 skips the MOVEQ (D0 stays 0x5A5Axxxx).
//  - BRA.S and BRA.W with positive and negative displacement (the .W also beyond +-128): PRM 4 BRA.
//  - BSR.S and BSR.W forward and backward: return address on the stack (PRM 4 BSR: SP - 4, (SP) = address of the
//    instruction following the BSR), A7 inside the subroutine, RTS (PRM 4 RTS: PC = (SP), SP + 4), A7 after the return
//    and the instruction right after the BSR executed (MOVEQ on D1). Each BSR uses a different stack: all the pushed
//    words stay in memory and are checked.
//  - flags: Bcc, BRA, BSR, RTS do not touch the CCR (PRM 4): the input X must survive (the MOVEQ after the branch
//    rewrites N Z V C according to PRM 4 MOVEQ).
//  - bus: FC of every cycle (program = 6, data = 5, supervisor; the two reset vectors in program space, UM
//    Table 6-2), word reads in the program, no write outside the areas, neighbouring cells holding sentinels.
//    The program runs twice: DTACK with no wait states and DTACK with 3 wait states; same expected values.
// Not tested: odd displacement (address error), DBcc, JMP/JSR, duration in clocks.

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

// Byte-wide memory, 24-bit address. Where the testbench has put nothing, every word reads 0x4AFC (ILLEGAL): a wrong
// branch ends in an exception (unexpected handler), not in a run of NOPs.
static std::map<uint32_t, uint8_t> mem;
static std::map<uint32_t, uint8_t> img0;                          // initial content of the data areas, rechecked at end of run
static std::map<uint32_t, std::pair<uint8_t, uint8_t> > expb;      // expected byte at end of run: value and mask of the fixed bits
static std::map<uint32_t, std::string> expd;                      // what that byte is (for the message)
static std::set<uint32_t> opnd;                                    // preloaded operand bytes (not sentinels)
static uint8_t rd8(uint32_t a) { a &= 0xFFFFFFu; auto it = mem.find(a); if (it != mem.end()) return it->second; return (a & 1u) ? 0xFC : 0x4A; }
static uint16_t rd16(uint32_t a) { return (uint16_t)((rd8(a) << 8) | rd8(a + 1)); }
static void wr8(uint32_t a, uint8_t v) { mem[a & 0xFFFFFFu] = v; }
static void wr16(uint32_t a, uint16_t v) { wr8(a, (uint8_t)(v >> 8)); wr8(a + 1, (uint8_t)v); }
static void wr32(uint32_t a, uint32_t v) { wr16(a, (uint16_t)(v >> 16)); wr16(a + 2, (uint16_t)v); }

// areas: 0 = program (read-only, FC 6), 1 = data (FC 5), 2 = vectors (read-only, FC 5), 3 = answers with BERR
struct Rgn { uint32_t lo, hi; int kind; };
static std::vector<Rgn> rgns;
static int rgn_of(uint32_t a) { for (size_t i = 0; i < rgns.size(); i++) if (a >= rgns[i].lo && a < rgns[i].hi) return (int)i; return -1; }
static void prog_area(uint32_t lo, uint32_t hi) { rgns.push_back({lo, hi, 0}); }
static void data_area(uint32_t lo, uint32_t hi) {   // sentinels different byte by byte
    rgns.push_back({lo, hi, 1});
    for (uint32_t a = lo; a < hi; a++) { uint8_t s = (uint8_t)(0xA5u ^ (a & 0xFFu) ^ ((a >> 7) & 0xFEu)); wr8(a, s); img0[a & 0xFFFFFFu] = s; }
}
static void expect8(uint32_t a, uint8_t v, uint8_t m, const std::string& d) {
    a &= 0xFFFFFFu;
    auto it = img0.find(a);   // if the sentinel equals the expected value I change it: the write must be visible
    if (it != img0.end() && !opnd.count(a) && ((it->second ^ v) & m) == 0) { it->second = (uint8_t)(it->second ^ 0xFFu); mem[a] = it->second; }
    expb[a] = std::make_pair(v, m); expd[a] = d;
}
static void expect_n(uint32_t a, uint32_t v, uint32_t m, int sz, const std::string& d) {
    for (int i = 0; i < sz; i++) { int sh = 8 * (sz - 1 - i); expect8(a + (uint32_t)i, (uint8_t)(v >> sh), (uint8_t)(m >> sh), d); }
}

// program emission and results area (MOVE SR,abs.W and MOVE.L Rn,abs.W after every tested instruction)
static uint32_t pc_e = 0, slot = 0x7000, end_pc = 0;
static void W(uint16_t v) { wr16(pc_e, v); pc_e += 2; }
static void L(uint32_t v) { W((uint16_t)(v >> 16)); W((uint16_t)v); }
static void set_ccr(uint8_t c) { W(0x44FC); W(c); }                                     // MOVE #c,CCR (PRM 4 MOVE to CCR)
static void save_sr(uint16_t e, uint16_t m, const std::string& d) { W(0x40F8); W((uint16_t)slot); expect_n(slot, e, m, 2, d); slot += 2; }   // MOVE SR,abs.W
static void save_d(int n, uint32_t e, const std::string& d) { W((uint16_t)(0x21C0 | n)); W((uint16_t)slot); expect_n(slot, e, 0xFFFFFFFFu, 4, d); slot += 4; }
static void save_a(int n, uint32_t e, const std::string& d) { W((uint16_t)(0x21C8 | n)); W((uint16_t)slot); expect_n(slot, e, 0xFFFFFFFFu, 4, d); slot += 4; }
static void ld_d(int n, uint32_t v) { W((uint16_t)(0x203C | (n << 9))); L(v); }      // MOVE.L #v,Dn
static void ld_a(int n, uint32_t v) { W((uint16_t)(0x207C | (n << 9))); L(v); }      // MOVEA.L #v,An
static void end_loop() { end_pc = pc_e; W(0x60FE); }                                    // BRA.S *
// vectors: initial SSP and PC; every other vector 2..63 to its own handler (BRA.S *) at 0x800 + 4*v
static void vectors(uint32_t ssp, uint32_t pc0) {
    rgns.push_back({0x000000u, 0x000400u, 2});
    prog_area(0x000800u, 0x000900u);
    wr32(0, ssp); wr32(4, pc0);
    for (uint32_t v = 2; v < 64; v++) { wr32(4 * v, 0x800u + 4 * v); wr16(0x800u + 4 * v, 0x60FE); }
}

// bus: one cycle = from AS low to AS high; n_as counts the clocks with AS low (S3 = 1, S4 = 2, S5 = 3: DTACK sampled at
// end of S4, UM 5.1). With ws wait states DTACK arrives 2*ws clocks later (each wait is an S4/S5 pair).
struct Cyc { uint32_t a; bool rd, u, l, berr; unsigned fc; uint16_t d; };
static std::vector<Cyc> cycs;
static int ws = 0;
static uint32_t berr_lo = 1, berr_hi = 0;
static bool in_cyc = false;
static int n_as = 0;
static Cyc cur;
static int end_fetch = 0, unexpected = -1;
static long n_wr = 0;
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
static void step() {
    if (!dut->as_n) {
        if (!in_cyc) { in_cyc = true; n_as = 0; cur = Cyc(); }
        n_as++;
        cur.a = ((uint32_t)dut->addr) << 1; cur.rd = dut->rw != 0; cur.fc = dut->fc;
        bool be = cur.a >= berr_lo && cur.a < berr_hi;
        cur.berr = be;
        if (!dut->uds_n) cur.u = true;
        if (!dut->lds_n) cur.l = true;
        if (dut->rw) dut->data_in = rd16(cur.a);
        else if (!dut->uds_n || !dut->lds_n) cur.d = dut->data_out;
        bool go = n_as >= 3 + 2 * ws;
        dut->dtack_n = (go && !be) ? 0 : 1;
        dut->berr_n = (go && be) ? 0 : 1;
    } else {
        if (in_cyc) { in_cyc = false; commit(cur); }
        dut->dtack_n = 1; dut->berr_n = 1;
    }
    dut->cen = 1; dut->cen2 = 1;
    tick();
}
static bool run_to_end(long max_ticks) {   // until the third fetch of the final BRA.S (prefetch, then two iterations of the loop)
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
}
static void check_image() {   // every byte of the data areas: expected value where specified, sentinel intact elsewhere
    for (auto& kv : img0) {
        uint32_t a = kv.first; uint8_t got = rd8(a);
        auto e = expb.find(a);
        if (e != expb.end()) check(((got ^ e->second.first) & e->second.second) == 0, expd[a] + " [byte " + hx(a) + "]", e->second.first, got);
        else check(got == kv.second, "sentinel intact [byte " + hx(a) + "]", kv.second, got);
    }
}
static void check_bus() {   // FC (UM: function codes; Table 6-2 for the vectors), read-only, word fetches
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
// PRM sec. 3, condition tests: conditions of the Bcc (N Z V C of the CCR)
static bool cond_true(int cc, int nzvc) {
    bool N = (nzvc & 8) != 0, Z = (nzvc & 4) != 0, V = (nzvc & 2) != 0, C = (nzvc & 1) != 0;
    switch (cc) {
        case 0x2: return !C && !Z;            // HI
        case 0x3: return C || Z;              // LS
        case 0x4: return !C;                  // CC
        case 0x5: return C;                   // CS
        case 0x6: return !Z;                  // NE
        case 0x7: return Z;                   // EQ
        case 0x8: return !V;                  // VC
        case 0x9: return V;                   // VS
        case 0xA: return !N;                  // PL
        case 0xB: return N;                   // MI
        case 0xC: return N == V;              // GE
        case 0xD: return N != V;              // LT
        case 0xE: return !Z && (N == V);      // GT
        default:  return Z || (N != V);       // LE
    }
}
static const char* const ccname[16] = {"T", "F", "HI", "LS", "CC", "CS", "NE", "EQ", "VC", "VS", "PL", "MI", "GE", "LT", "GT", "LE"};

// BSR forward or backward, .S or .W; own stack at stk; the subroutine sets D0 = id and saves A7, the caller executes
// MOVEQ #id+1,D1 right after the BSR (a return wrong by +2 would skip it)
static void bsr_case(bool wide, bool back, uint32_t stk, int id) {
    std::string nm = std::string("BSR") + (wide ? ".W" : ".S") + (back ? " backward" : " forward");
    uint32_t sub = 0;
    if (back) {
        uint32_t q = pc_e; W(0x6000); W(0);                      // BRA.W past the subroutine
        W(0x5207); sub = pc_e; W((uint16_t)(0x7000 | id));       // S-2: ADDQ.B #1,D7 ; S: MOVEQ #id,D0
        save_a(7, stk - 4, nm + ": A7 inside the subroutine = SP - 4");
        W(0x4E75);                                               // RTS
        if (wide) for (int i = 0; i < 200; i++) W(0x4AFC);
        wr16(q + 2, (uint16_t)(pc_e - (q + 2)));
    }
    ld_a(7, stk);
    ld_d(0, 0x7C7C0000u | (uint32_t)id);
    ld_d(1, 0x7D7D0000u | (uint32_t)id);
    uint32_t p = pc_e;
    W(0x6100); if (wide) W(0);
    uint32_t ret = pc_e;
    W((uint16_t)(0x7200 | (id + 1)));                            // MOVEQ #id+1,D1
    save_a(7, stk, nm + ": A7 after RTS = initial SP");
    save_d(0, (uint32_t)id, nm + ": subroutine executed (D0)");
    save_d(1, (uint32_t)(id + 1), nm + ": resumed at the instruction after the BSR (D1)");
    if (!back) {
        uint32_t q = pc_e; W(0x6000); W(0);
        if (wide) for (int i = 0; i < 200; i++) W(0x4AFC);
        W(0x5207); sub = pc_e; W((uint16_t)(0x7000 | id));
        save_a(7, stk - 4, nm + ": A7 inside the subroutine = SP - 4");
        W(0x4E75);
        wr16(q + 2, (uint16_t)(pc_e - (q + 2)));
    }
    uint32_t disp = sub - (p + 2);
    if (wide) wr16(p + 2, (uint16_t)disp); else wr8(p + 1, (uint8_t)disp);
    expect_n(stk - 4, ret, 0xFFFFFFFFu, 4, nm + ": pushed return address (PRM 4 BSR)");
}

static void build() {
    vectors(0x00002000u, 0x00010000u);
    prog_area(0x10000u, 0x18000u);
    data_area(0x1C00u, 0x2000u);      // BSR stacks
    data_area(0x7000u, 0x7E00u);      // results
    pc_e = 0x10000u;
    W(0x7E00);                        // MOVEQ #0,D7: counter of entries at target - 2
    int idx = 0;
    for (int wide = 0; wide < 2; wide++)
        for (int cc = 2; cc < 16; cc++)
            for (int f = 0; f < 16; f++, idx++) {
                uint8_t ccr = (uint8_t)(f | (((cc + f) & 1) ? 0x10 : 0));
                bool tk = cond_true(cc, f);
                std::string nm = std::string("B") + ccname[cc] + (wide ? ".W" : ".S") + " CCR=" + hx(ccr);
                ld_d(0, 0x5A5A0000u | (uint32_t)idx);
                set_ccr(ccr);
                uint32_t p = pc_e;
                W((uint16_t)(0x6000 | (cc << 8)));
                if (wide) W(0);
                W(0x7022);                    // not-taken path: MOVEQ #0x22,D0
                W(0x6004);                    // BRA.S +4 (past the ADDQ and MOVEQ of the target)
                W(0x5207);                    // target - 2: ADDQ.B #1,D7
                uint32_t tg = pc_e;
                W(0x7011);                    // target: MOVEQ #0x11,D0
                if (wide) wr16(p + 2, (uint16_t)(tg - (p + 2))); else wr8(p + 1, (uint8_t)(tg - (p + 2)));
                save_sr((uint16_t)(0x2700 | (ccr & 0x10)), 0xFFFF, nm + ": SR after the MOVEQ (input X intact, PRM 4 Bcc and MOVEQ)");
                save_d(0, tk ? 0x11u : 0x22u, nm + (tk ? ": condition true, branch taken" : ": condition false, branch not taken"));
            }
    // BRA.S forward
    { ld_d(0, 0x6B6B0001u); uint32_t p = pc_e; W(0x6000); W(0x4AFC); W(0x4AFC); W(0x5207); uint32_t tg = pc_e; W(0x7031);
      wr8(p + 1, (uint8_t)(tg - (p + 2))); save_d(0, 0x31u, "BRA.S forward: target executed"); }
    // BRA.W forward, beyond +128
    { ld_d(0, 0x6B6B0002u); uint32_t p = pc_e; W(0x6000); W(0); for (int i = 0; i < 200; i++) W(0x4AFC); W(0x5207); uint32_t tg = pc_e; W(0x7032);
      wr16(p + 2, (uint16_t)(tg - (p + 2))); save_d(0, 0x32u, "BRA.W forward: target executed"); }
    // BRA.S backward
    { uint32_t q = pc_e; W(0x6000); W(0);
      W(0x5207); uint32_t tg = pc_e; W(0x7033);
      uint32_t r = pc_e; W(0x6000); W(0);
      wr16(q + 2, (uint16_t)(pc_e - (q + 2)));
      ld_d(0, 0x6B6B0003u);
      uint32_t p = pc_e; W((uint16_t)(0x6000 | (uint8_t)(tg - (p + 2))));
      wr16(r + 2, (uint16_t)(pc_e - (r + 2)));
      save_d(0, 0x33u, "BRA.S backward: target executed"); }
    // BRA.W backward, beyond -128
    { uint32_t q = pc_e; W(0x6000); W(0);
      W(0x5207); uint32_t tg = pc_e; W(0x7034);
      uint32_t r = pc_e; W(0x6000); W(0);
      for (int i = 0; i < 200; i++) W(0x4AFC);
      wr16(q + 2, (uint16_t)(pc_e - (q + 2)));
      ld_d(0, 0x6B6B0004u);
      uint32_t p = pc_e; W(0x6000); W((uint16_t)(tg - (p + 2)));
      wr16(r + 2, (uint16_t)(pc_e - (r + 2)));
      save_d(0, 0x34u, "BRA.W backward: target executed"); }
    bsr_case(false, false, 0x1F00u, 0x41);
    bsr_case(false, true,  0x1E00u, 0x43);
    bsr_case(true,  false, 0x1D00u, 0x45);
    bsr_case(true,  true,  0x1C80u, 0x47);
    save_d(7, 0u, "D7 = 0: no branch landed on the word before the target");
    end_loop();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    static const int WS[2] = {0, 3};
    for (int k = 0; k < 2; k++) {
        ws = WS[k]; tag = "[ws=" + std::to_string(ws) + "]";
        clear_all();
        build();
        dut_start();
        bool done = run_to_end(6000000L);
        check(done, "end of run: fetch of the final BRA.S at " + hx(end_pc), 1, done ? 1u : 0u);
        check(!dut->illegal, "no illegal instruction exception", 0, dut->illegal);
        check_bus();
        check_image();
        dut_stop();
    }
    return finish();
}
