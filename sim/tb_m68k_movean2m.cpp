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

// tb_m68k_movean2m.cpp - (a) MOVE.W/MOVE.L An,<memory> and (b) MOVE.W/MOVE.L #imm,d16(An), on the full processor.
//
// What it tests (expectations from the manual):
//  - (a) direct An source (PRM 2.2.2; MOVE.B An does not exist, PRM 4 MOVE): .W writes the low word of An, .L the whole
//    register; destinations (An), (An)+, -(An), d16(An) (positive and negative), d8(An,Xn) (Dn.W with a high word
//    other than 0, An.L 0x0000FFF0, which taken as .W would be -16), abs.W (positive and negative), abs.L: 14 cases, then -(A7)
//    as word and as long. Source An unchanged.
//  - (b) MOVE.W and MOVE.L #imm,d16(A6) with positive and negative d16.
//  - flags (PRM 4 MOVE): X unchanged, N Z from the data of the size, V = C = 0; a different input CCR for each case.
//  - memory with sentinels, every byte of the data areas compared; data FC 5 and program FC 6, no access outside the
//    areas; two runs, DTACK with no wait states and with 3 wait states.
//  The target is compiled as two-phase (cen and cen2, without SINGLE_CEN): the testbench raises cen and cen2 on every clock.
// Does not test: MOVE.L A7,-(A7) (pushed value not fixed by the manual), word order of a .L write.

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

// Byte-wide memory, 24 address bits. Wherever the testbench has put nothing every word reads 0x4AFC (ILLEGAL).
static std::map<uint32_t, uint8_t> mem;
static std::map<uint32_t, uint8_t> img0;                          // initial content of the data areas, rechecked at the end of the run
static std::map<uint32_t, std::pair<uint8_t, uint8_t> > expb;      // byte expected at the end of the run: value and mask of the fixed bits
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
static void opnd_n(uint32_t a, uint32_t v, int sz) {
    for (int i = 0; i < sz; i++) { uint32_t b = (a + (uint32_t)i) & 0xFFFFFFu; uint8_t x = (uint8_t)(v >> (8 * (sz - 1 - i))); wr8(b, x); img0[b] = x; opnd.insert(b); }
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
static void save_a(int n, uint32_t e, const std::string& d) { W((uint16_t)(0x21C8 | n)); W((uint16_t)slot); expect_n(slot, e, 0xFFFFFFFFu, 4, d); slot += 4; }
static void ld_d(int n, uint32_t v) { W((uint16_t)(0x203C | (n << 9))); L(v); }
static void ld_a(int n, uint32_t v) { W((uint16_t)(0x207C | (n << 9))); L(v); }
static void end_loop() { end_pc = pc_e; W(0x60FE); }
static void vectors(uint32_t ssp, uint32_t pc0) {
    rgns.push_back({0x000000u, 0x000400u, 2});
    prog_area(0x000800u, 0x000900u);
    wr32(0, ssp); wr32(4, pc0);
    for (uint32_t v = 2; v < 64; v++) { wr32(4 * v, 0x800u + 4 * v); wr16(0x800u + 4 * v, 0x60FE); }
}

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
static void step() {   // n_as: clocks with AS low (S3 = 1 .. S5 = 3); DTACK sampled at the end of S4 (UM 5.1), 2*ws wait clocks
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
static uint32_t mk(int sz) { return sz == 1 ? 0xFFu : sz == 2 ? 0xFFFFu : 0xFFFFFFFFu; }
static uint32_t mb(int sz) { return sz == 1 ? 0x80u : sz == 2 ? 0x8000u : 0x80000000u; }
static uint8_t cc_nz(int sz, uint32_t r, uint8_t cin) {   // PRM 4 MOVE: X unchanged, N = msb, Z = zero result, V = C = 0
    r &= mk(sz); return (uint8_t)((cin & 0x10u) | ((r & mb(sz)) ? 8u : 0u) | (r == 0 ? 4u : 0u));
}
static uint32_t sx8(uint32_t v) { return (uint32_t)(int32_t)(int8_t)(uint8_t)v; }
static uint32_t sx16(uint32_t v) { return (uint32_t)(int32_t)(int16_t)(uint16_t)v; }

// ============================================================================== addressing modes (PRM 2.2) and MOVE cases
enum Mode { M_AN, M_AI, M_API, M_APD, M_AD16, M_AIX, M_AW, M_AL, M_IMM };
struct Ea { Mode m; int r; int d; bool xa; int xr; bool xl; uint32_t xv; };   // d = d16 or d8; xa/xr/xl/xv = index A?/number/.L/value
static const char* mname(Mode m) {
    switch (m) {
        case M_AN: return "An"; case M_AI: return "(An)"; case M_API: return "(An)+"; case M_APD: return "-(An)";
        case M_AD16: return "d16(An)"; case M_AIX: return "d8(An,Xn)"; case M_AW: return "abs.W"; case M_AL: return "abs.L"; default: return "#imm";
    }
}
static const char* szs(int sz) { return sz == 1 ? ".B" : sz == 2 ? ".W" : ".L"; }
static int step_of(const Ea& e, int sz) { return (sz == 1 && e.r == 7) ? 2 : sz; }   // PRM 2.2.4/2.2.5: A7 as byte advances by 2
static bool uses_an(const Ea& e) { return e.m == M_AI || e.m == M_API || e.m == M_APD || e.m == M_AD16 || e.m == M_AIX; }
static uint16_t ea_field(const Ea& e) {
    switch (e.m) {
        case M_AN: return (uint16_t)(0x08 | e.r); case M_AI: return (uint16_t)(0x10 | e.r); case M_API: return (uint16_t)(0x18 | e.r);
        case M_APD: return (uint16_t)(0x20 | e.r); case M_AD16: return (uint16_t)(0x28 | e.r); case M_AIX: return (uint16_t)(0x30 | e.r);
        case M_AW: return 0x38; case M_AL: return 0x39; default: return 0x3C;
    }
}
static uint16_t dst_field(const Ea& e) { uint16_t f = ea_field(e); return (uint16_t)(((f & 7u) << 9) | ((f >> 3) << 6)); }
static uint32_t ea_setup(const Ea& e, uint32_t addr, int sz) {
    int st = step_of(e, sz);
    switch (e.m) {
        case M_AI: ld_a(e.r, addr); return addr;
        case M_API: ld_a(e.r, addr); return addr + (uint32_t)st;
        case M_APD: ld_a(e.r, addr + (uint32_t)st); return addr;
        case M_AD16: { uint32_t b = addr - sx16((uint32_t)e.d); ld_a(e.r, b); return b; }
        case M_AIX: {
            uint32_t iv = e.xl ? e.xv : sx16(e.xv);
            if (e.xa) ld_a(e.xr, e.xv); else ld_d(e.xr, e.xv);
            uint32_t b = addr - sx8((uint32_t)e.d) - iv; ld_a(e.r, b); return b;
        }
        default: return 0;
    }
}
static void ea_ext(const Ea& e, uint32_t addr, int sz, uint32_t imm) {
    switch (e.m) {
        case M_AD16: W((uint16_t)e.d); break;
        case M_AIX: W((uint16_t)((e.xa ? 0x8000 : 0) | (e.xr << 12) | (e.xl ? 0x0800 : 0) | (e.d & 0xFF))); break;
        case M_AW: W((uint16_t)addr); break;
        case M_AL: L(addr); break;
        case M_IMM: if (sz == 4) L(imm); else W((uint16_t)(sz == 1 ? (imm & 0xFFu) : imm)); break;
        default: break;
    }
}
static int ncase = 0;
static void move_case(int sz, const Ea& s, uint32_t sa, const Ea& d, uint32_t da, uint32_t val, uint8_t ccr) {
    std::string nm = "case " + std::to_string(ncase++) + " MOVE" + szs(sz) + " " + mname(s.m) + "," + mname(d.m) + " -> " + hx(da);
    uint32_t mv = val & mk(sz);
    if (s.m != M_AN && s.m != M_IMM) opnd_n(sa, mv, sz);
    if (s.m == M_AN) ld_a(s.r, val);
    uint32_t s_after = uses_an(s) ? ea_setup(s, sa, sz) : 0;
    uint32_t d_after = uses_an(d) ? ea_setup(d, da, sz) : 0;
    set_ccr(ccr);
    W((uint16_t)((sz == 1 ? 0x1000 : sz == 2 ? 0x3000 : 0x2000) | dst_field(d) | ea_field(s)));
    ea_ext(s, sa, sz, mv);
    ea_ext(d, da, sz, 0);
    expect_n(da, mv, 0xFFFFFFFFu, sz, nm + ": data in the destination");
    save_sr((uint16_t)(0x2700 | cc_nz(sz, mv, ccr)), 0xFFFF, nm + ": SR (PRM 4 MOVE: X unchanged, N Z from the data, V = C = 0)");
    if (uses_an(s)) save_a(s.r, s_after, nm + ": A" + std::to_string(s.r) + " of the source after the instruction");
    if (s.m == M_AN) save_a(s.r, val, nm + ": source An unchanged");
    if (uses_an(d)) save_a(d.r, d_after, nm + ": A" + std::to_string(d.r) + " of the destination after the instruction");
}

// ============================================================================== testbench program
static void build() {
    ncase = 0;
    vectors(0x00002000u, 0x00004000u);
    prog_area(0x4000u, 0x6000u);
    data_area(0x1E00u, 0x2000u);          // stack for the cases with A7
    data_area(0x2200u, 0x2400u);          // d16(A6) destinations of part (b)
    data_area(0x3400u, 0x3800u);          // destinations
    data_area(0x13000u, 0x13100u);        // abs.L destinations
    data_area(0xFFFF00u, 0x1000000u);     // negative abs.W destinations
    data_area(0x7000u, 0x7400u);          // results
    pc_e = 0x4000u;
    static const Mode dms[7] = {M_AI, M_API, M_APD, M_AD16, M_AIX, M_AW, M_AL};
    static const uint32_t an_vals[2][7] = {
        {0x8000FFFFu, 0x12340000u, 0x00007FFFu, 0xFFFF8000u, 0x00000000u, 0xDEADBEEFu, 0x7FFFFFFFu},
        {0x8000FFFFu, 0x12340000u, 0x00007FFFu, 0xFFFF8000u, 0x00000000u, 0xDEADBEEFu, 0x7FFFFFFFu}};
    static const uint8_t ccrs[7] = {0x1F, 0x00, 0x15, 0x0A, 0x10, 0x0F, 0x1B};
    const Ea a4 = {M_AN, 4, 0, false, 0, false, 0};
    uint32_t dp = 0x3400u, fp = 0x13000u, np = 0xFF20u;
    for (int zi = 0; zi < 2; zi++) {
        int sz = zi == 0 ? 2 : 4;
        for (int j = 0; j < 7; j++) {
            Ea d = {dms[j], 1, 0, false, 0, false, 0};
            uint32_t da;
            if (d.m == M_AW && zi == 1) { da = 0xFFFF0000u | np; np += 16; }
            else if (d.m == M_AL) { da = fp; fp += 16; }
            else { da = dp; dp += 16; }
            if (d.m == M_AD16) d.d = (zi == 0) ? -0x20 : 0x20;
            if (d.m == M_AIX) {
                d.d = zi == 0 ? 4 : -4;
                d.xa = (zi == 1); d.xr = (zi == 1) ? 3 : 2; d.xl = (zi == 1);
                d.xv = zi == 0 ? 0xABCD0006u : 0x0000FFF0u;
            }
            move_case(sz, a4, 0, d, da, an_vals[zi][(j + zi) % 7], ccrs[(j + 3 * zi) % 7]);
        }
    }
    { Ea d = {M_APD, 7, 0, false, 0, false, 0}; move_case(2, a4, 0, d, 0x1F00u, 0x13579BDFu, 0x10); }
    { Ea d = {M_APD, 7, 0, false, 0, false, 0}; move_case(4, a4, 0, d, 0x1F10u, 0x8642FDB9u, 0x0F); }
    // (b) MOVE #imm,d16(A6)
    const Ea imm = {M_IMM, 0, 0, false, 0, false, 0};
    { Ea d = {M_AD16, 6, 0x20, false, 0, false, 0}; move_case(2, imm, 0, d, 0x2220u, 0xCAFEu, 0x01); }
    { Ea d = {M_AD16, 6, -0x30, false, 0, false, 0}; move_case(4, imm, 0, d, 0x2230u, 0x12345678u, 0x1E); }
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
        bool done = run_to_end(2000000L);
        check(done, "end of run: fetch of the final BRA.S at " + hx(end_pc), 1, done ? 1u : 0u);
        check(!dut->illegal, "no illegal instruction exception", 0, dut->illegal);
        check_bus();
        check_image();
        dut_stop();
    }
    return finish();
}
