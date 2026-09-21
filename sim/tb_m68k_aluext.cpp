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

// tb_m68k_aluext.cpp - ADD, SUB, AND, OR <ea>,Dn with a source that uses an extension word, on the whole processor.
//
// What it tests (expected values from the manual):
//  - ADD/SUB/AND/OR .B .W .L <ea>,D3 (PRM 4) with source d16(An) positive and negative, d8(An,Xn) (Dn.W with the high word
//    different from 0, An.L 0x0000FFF0 which taken as .W would be -16), abs.W positive and negative (PRM 2.2.16), abs.L:
//    24 cases, bytes at even and odd addresses. D3 has the bits above the size preloaded (0xA5): they must stay, and the
//    extension word must not get into the result.
//  - flags X N Z V C (PRM sect. 3, condition codes; ADD and SUB: X = C; AND and OR: X unchanged, V = C = 0), input CCR 0x1F or 0x00.
//  - base and index register unchanged; operands with sentinels around them, no write to the data areas except the
//    results; FC data 5 and program 6; two runs, DTACK with no wait states and with 3 wait states.
// It does not test: PC-relative sources, EOR (only Dn,<ea>), ADDA/SUBA/CMP.

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

// Byte memory, 24 address bits. Where the testbench has placed nothing every word reads 0x4AFC (ILLEGAL).
static std::map<uint32_t, uint8_t> mem;
static std::map<uint32_t, uint8_t> img0;                          // initial content of the data areas, checked again at the end of the run
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
static void save_d(int n, uint32_t e, const std::string& d) { W((uint16_t)(0x21C0 | n)); W((uint16_t)slot); expect_n(slot, e, 0xFFFFFFFFu, 4, d); slot += 4; }
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
static uint32_t sx8(uint32_t v) { return (uint32_t)(int32_t)(int8_t)(uint8_t)v; }
static uint32_t sx16(uint32_t v) { return (uint32_t)(int32_t)(int16_t)(uint16_t)v; }

// ============================================================================ addressing modes with extension (PRM 2.2)
enum Mode { M_AD16, M_AIX, M_AW, M_AL };
struct Ea { Mode m; int r; int d; bool xa; int xr; bool xl; uint32_t xv; };   // d = d16 or d8; xa/xr/xl/xv = index A?/number/.L/value
static const char* mname(Mode m) { return m == M_AD16 ? "d16(An)" : m == M_AIX ? "d8(An,Xn)" : m == M_AW ? "abs.W" : "abs.L"; }
static const char* szs(int sz) { return sz == 1 ? ".B" : sz == 2 ? ".W" : ".L"; }
static bool uses_an(const Ea& e) { return e.m == M_AD16 || e.m == M_AIX; }
static uint16_t ea_field(const Ea& e) { return (uint16_t)(e.m == M_AD16 ? (0x28 | e.r) : e.m == M_AIX ? (0x30 | e.r) : e.m == M_AW ? 0x38 : 0x39); }
static uint32_t ea_setup(const Ea& e, uint32_t addr) {   // loads the registers so that the EA equals addr; returns the base
    if (e.m == M_AD16) { uint32_t b = addr - sx16((uint32_t)e.d); ld_a(e.r, b); return b; }
    if (e.m == M_AIX) {
        uint32_t iv = e.xl ? e.xv : sx16(e.xv);
        if (e.xa) ld_a(e.xr, e.xv); else ld_d(e.xr, e.xv);
        uint32_t b = addr - sx8((uint32_t)e.d) - iv; ld_a(e.r, b); return b;
    }
    return 0;
}
static void ea_ext(const Ea& e, uint32_t addr) {
    switch (e.m) {
        case M_AD16: W((uint16_t)e.d); break;
        case M_AIX: W((uint16_t)((e.xa ? 0x8000 : 0) | (e.xr << 12) | (e.xl ? 0x0800 : 0) | (e.d & 0xFF))); break;
        case M_AW: W((uint16_t)addr); break;
        default: L(addr); break;
    }
}
static uint32_t sp_src = 0, sp_far = 0, sp_neg = 0;
static Ea ext_src(int k, int sz, bool odd, uint32_t& a) {   // source number k: base A0, index D1 or A2
    Ea e = {M_AD16, 0, 0, false, 0, false, 0};
    uint32_t ob = (sz == 1 && odd) ? 1u : 0u;
    switch (k % 7) {
        case 0: e.m = M_AD16; e.d = 0x30; a = sp_src + ob; sp_src += 16; break;
        case 1: e.m = M_AD16; e.d = -0x30; a = sp_src + ob; sp_src += 16; break;
        case 2: e.m = M_AIX; e.d = -2; e.xr = 1; e.xv = 0x5A5A000Au; a = sp_src + ob; sp_src += 16; break;
        case 3: e.m = M_AIX; e.d = 0x10; e.xa = true; e.xr = 2; e.xl = true; e.xv = 0x0000FFF0u; a = sp_src + ob; sp_src += 16; break;
        case 4: e.m = M_AW; a = sp_src + ob; sp_src += 16; break;
        case 5: e.m = M_AW; a = 0xFFFF0000u | (sp_neg + ob); sp_neg += 16; break;
        default: e.m = M_AL; a = sp_far + ob; sp_far += 16; break;
    }
    return e;
}

// ============================================================================ ALU operations (PRM 4) and condition codes (PRM sect. 3, condition codes)
enum Op { O_ADD, O_SUB, O_AND, O_OR };
static const char* oname(Op o) { return o == O_ADD ? "ADD" : o == O_SUB ? "SUB" : o == O_AND ? "AND" : "OR"; }
static uint16_t op_base(Op o) { return (uint16_t)(o == O_ADD ? 0xD000 : o == O_SUB ? 0x9000 : o == O_AND ? 0xC000 : 0x8000); }
static uint32_t op_res(Op o, int sz, uint32_t dv, uint32_t sv) {
    switch (o) {
        case O_ADD: return (dv + sv) & mk(sz);
        case O_SUB: return (dv - sv) & mk(sz);
        case O_AND: return dv & sv & mk(sz);
        default:    return (dv | sv) & mk(sz);
    }
}
static uint8_t op_cc(Op o, int sz, uint32_t dv, uint32_t sv, uint32_t r, uint8_t cin) {
    uint32_t m = mb(sz);
    bool Sm = (sv & m) != 0, Dm = (dv & m) != 0, Rm = (r & m) != 0, Z = (r & mk(sz)) == 0;
    if (o == O_ADD || o == O_SUB) {
        bool C, V;
        if (o == O_ADD) { C = (Sm && Dm) || (!Rm && Dm) || (Sm && !Rm); V = (Sm && Dm && !Rm) || (!Sm && !Dm && Rm); }
        else            { C = (Sm && !Dm) || (Rm && !Dm) || (Sm && Rm);  V = (!Sm && Dm && !Rm) || (Sm && !Dm && Rm); }
        return (uint8_t)((C ? 0x11u : 0u) | (Rm ? 8u : 0u) | (Z ? 4u : 0u) | (V ? 2u : 0u));   // X = C
    }
    return (uint8_t)((cin & 0x10u) | (Rm ? 8u : 0u) | (Z ? 4u : 0u));                          // X unchanged, V = C = 0
}
static int ncase = 0;
static void alu_ea_dn(Op o, int sz, const Ea& s, uint32_t sa, uint32_t dv, uint32_t sv, uint8_t ccr) {
    std::string nm = "case " + std::to_string(ncase++) + " " + oname(o) + szs(sz) + " " + mname(s.m) + ",D3 <- " + hx(sa);
    sv &= mk(sz); dv &= mk(sz);
    uint32_t dinit = (0xA5A5A5A5u & ~mk(sz)) | dv;
    opnd_n(sa, sv, sz);
    ld_d(3, dinit);
    uint32_t base = uses_an(s) ? ea_setup(s, sa) : 0;
    set_ccr(ccr);
    W((uint16_t)(op_base(o) | (3 << 9) | ((sz == 1 ? 0 : sz == 2 ? 1 : 2) << 6) | ea_field(s)));
    ea_ext(s, sa);
    uint32_t r = op_res(o, sz, dv, sv);
    save_sr((uint16_t)(0x2700 | op_cc(o, sz, dv, sv, r, ccr)), 0xFFFF, nm + ": SR (PRM sect. 3, condition codes and PRM 4 " + oname(o) + ")");
    save_d(3, (dinit & ~mk(sz)) | r, nm + ": D3, bits above the size unchanged");
    if (uses_an(s)) save_a(s.r, base, nm + ": base register unchanged");
    if (s.m == M_AIX) { if (s.xa) save_a(s.xr, s.xv, nm + ": index unchanged"); else save_d(s.xr, s.xv, nm + ": index unchanged"); }
}

// ============================================================================ testbench program
struct Pair { uint32_t d, s; };
static const Pair tbl[4][3][2] = {   // [ADD SUB AND OR][B W L][case] = {destination, source}
    { { {0x7Fu, 0x01u}, {0xFFu, 0x01u} }, { {0x7FFFu, 0x0001u}, {0xFFFFu, 0x0001u} }, { {0x7FFFFFFFu, 0x00000001u}, {0x80000000u, 0x80000000u} } },
    { { {0x00u, 0x01u}, {0x80u, 0x01u} }, { {0x0000u, 0x0001u}, {0x8000u, 0x0001u} }, { {0x00000005u, 0x00000005u}, {0x80000000u, 0x00000001u} } },
    { { {0xF0u, 0x0Fu}, {0xFFu, 0x80u} }, { {0xF0F0u, 0x0F0Fu}, {0xFFFFu, 0x8001u} }, { {0xFFFF0000u, 0x0000FFFFu}, {0x80000001u, 0xFFFFFFFFu} } },
    { { {0x00u, 0x00u}, {0x01u, 0x80u} }, { {0x0000u, 0x0000u}, {0x0100u, 0x8000u} }, { {0x00000000u, 0x00000000u}, {0x00000001u, 0x80000000u} } }};

static void build() {
    ncase = 0; sp_src = 0x3000u; sp_far = 0x13000u; sp_neg = 0xFF00u;
    vectors(0x00002000u, 0x00004000u);
    prog_area(0x4000u, 0x6000u);
    data_area(0x3000u, 0x3400u);          // operands
    data_area(0x13000u, 0x13100u);        // abs.L operands
    data_area(0xFFFF00u, 0x1000000u);     // negative abs.W operands
    data_area(0x7000u, 0x7400u);          // results
    pc_e = 0x4000u;
    int k = 0;
    for (int oi = 0; oi < 4; oi++)
        for (int zi = 0; zi < 3; zi++)
            for (int p = 0; p < 2; p++, k++) {
                int sz = zi == 0 ? 1 : zi == 1 ? 2 : 4;
                uint32_t sa = 0;
                Ea s = ext_src(k, sz, (k & 1) != 0, sa);
                alu_ea_dn((Op)oi, sz, s, sa, tbl[oi][zi][p].d, tbl[oi][zi][p].s, p == 0 ? 0x1F : 0x00);
            }
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
