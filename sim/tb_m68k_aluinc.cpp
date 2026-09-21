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

// tb_m68k_aluinc.cpp - ADD, SUB, AND, OR <ea>,Dn with source (An)+ and -(An), on the whole processor.
//
// What it tests (expected values from the manual):
//  - ADD/SUB/AND/OR .B .W .L <ea>,D3 (PRM 4 ADD, SUB, AND, OR), source (A0)+ and -(A0): 24 cases; operands with
//    carry, borrow, overflow, zero and negative result. D3 has the bits above the size preloaded (0xA5):
//    they must stay (PRM 4: only the part of the size changes).
//  - byte (A7)+ and -(A7) advance by 2 (PRM 2.2.4/2.2.5).
//  - flags: all five X N Z V C (PRM sect. 3, condition codes for ADD and SUB, X = C; AND and OR: X unchanged, V = C = 0), with
//    input CCR 0x1F or 0x00 so that every bit has to change or stay.
//  - A0/A7 after the instruction; operands in memory with sentinels around them, no expected write to the data areas
//    except the results; FC data 5 and program 6; two runs, DTACK with no wait states and with 3 wait states.
// It does not test: EOR (only Dn,<ea>: tb_m68k_alurmw), other sources (tb_m68k_aluext), ADDA/SUBA, CMP.

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

// ============================================================================ addressing modes (PRM 2.2)
enum Mode { M_AI, M_API, M_APD };
struct Ea { Mode m; int r; };
static const char* mname(Mode m) { return m == M_AI ? "(An)" : m == M_API ? "(An)+" : "-(An)"; }
static const char* szs(int sz) { return sz == 1 ? ".B" : sz == 2 ? ".W" : ".L"; }
static int step_of(const Ea& e, int sz) { return (sz == 1 && e.r == 7) ? 2 : sz; }   // PRM 2.2.4/2.2.5: byte A7 advances by 2
static uint16_t ea_field(const Ea& e) { return (uint16_t)((e.m == M_AI ? 0x10 : e.m == M_API ? 0x18 : 0x20) | e.r); }
static uint32_t ea_setup(const Ea& e, uint32_t addr, int sz) {
    uint32_t st = (uint32_t)step_of(e, sz);
    if (e.m == M_APD) { ld_a(e.r, addr + st); return addr; }
    ld_a(e.r, addr);
    return e.m == M_API ? addr + st : addr;
}

// ============================================================================ ALU operations (PRM 4) and condition codes (PRM sect. 3, condition codes)
enum Op { O_ADD, O_SUB, O_AND, O_OR, O_EOR };
static const char* oname(Op o) { return o == O_ADD ? "ADD" : o == O_SUB ? "SUB" : o == O_AND ? "AND" : o == O_OR ? "OR" : "EOR"; }
static uint16_t op_base(Op o) { return (uint16_t)(o == O_ADD ? 0xD000 : o == O_SUB ? 0x9000 : o == O_AND ? 0xC000 : o == O_OR ? 0x8000 : 0xB000); }
static uint32_t op_res(Op o, int sz, uint32_t dv, uint32_t sv) {   // result = d op s (SUB: d - s)
    switch (o) {
        case O_ADD: return (dv + sv) & mk(sz);
        case O_SUB: return (dv - sv) & mk(sz);
        case O_AND: return dv & sv & mk(sz);
        case O_OR:  return (dv | sv) & mk(sz);
        default:    return (dv ^ sv) & mk(sz);
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
// <op>.<sz> <s>,D3: source operand sv in memory at sa, D3 = dv with the high bits at 0xA5
static void alu_ea_dn(Op o, int sz, const Ea& s, uint32_t sa, uint32_t dv, uint32_t sv, uint8_t ccr) {
    std::string nm = "case " + std::to_string(ncase++) + " " + oname(o) + szs(sz) + " " + mname(s.m) + ",D3 (A" + std::to_string(s.r) + ")";
    sv &= mk(sz); dv &= mk(sz);
    uint32_t dinit = (0xA5A5A5A5u & ~mk(sz)) | dv;
    opnd_n(sa, sv, sz);
    ld_d(3, dinit);
    uint32_t s_after = ea_setup(s, sa, sz);
    set_ccr(ccr);
    W((uint16_t)(op_base(o) | (3 << 9) | ((sz == 1 ? 0 : sz == 2 ? 1 : 2) << 6) | ea_field(s)));
    uint32_t r = op_res(o, sz, dv, sv);
    save_sr((uint16_t)(0x2700 | op_cc(o, sz, dv, sv, r, ccr)), 0xFFFF, nm + ": SR (PRM sect. 3, condition codes and PRM 4 " + oname(o) + ")");
    save_d(3, (dinit & ~mk(sz)) | r, nm + ": D3, bits above the size unchanged");
    save_a(s.r, s_after, nm + ": A" + std::to_string(s.r) + " after the instruction");
}

// ============================================================================ testbench program
struct Pair { uint32_t d, s; };
static const Pair tbl[4][3][2] = {   // [ADD SUB AND OR][B W L][case] = {destination, source}
    { { {0x7Fu, 0x01u}, {0xFFu, 0x01u} }, { {0x7FFFu, 0x0001u}, {0xFFFFu, 0x0001u} }, { {0x7FFFFFFFu, 0x00000001u}, {0x80000000u, 0x80000000u} } },
    { { {0x00u, 0x01u}, {0x80u, 0x01u} }, { {0x0000u, 0x0001u}, {0x8000u, 0x0001u} }, { {0x00000005u, 0x00000005u}, {0x80000000u, 0x00000001u} } },
    { { {0xF0u, 0x0Fu}, {0xFFu, 0x80u} }, { {0xF0F0u, 0x0F0Fu}, {0xFFFFu, 0x8001u} }, { {0xFFFF0000u, 0x0000FFFFu}, {0x80000001u, 0xFFFFFFFFu} } },
    { { {0x00u, 0x00u}, {0x01u, 0x80u} }, { {0x0000u, 0x0000u}, {0x0100u, 0x8000u} }, { {0x00000000u, 0x00000000u}, {0x00000001u, 0x80000000u} } }};

static void build() {
    ncase = 0;
    vectors(0x00002000u, 0x00004000u);
    prog_area(0x4000u, 0x6000u);
    data_area(0x1E00u, 0x2000u);          // stack for the cases with A7
    data_area(0x3000u, 0x3400u);          // operands
    data_area(0x7000u, 0x7400u);          // results
    pc_e = 0x4000u;
    uint32_t sp = 0x3000u;
    for (int oi = 0; oi < 4; oi++)
        for (int zi = 0; zi < 3; zi++)
            for (int p = 0; p < 2; p++) {
                int sz = zi == 0 ? 1 : zi == 1 ? 2 : 4;
                Ea s = {p == 0 ? M_API : M_APD, 0};
                uint32_t sa = sp + ((sz == 1 && p == 1) ? 1u : 0u); sp += 16;
                alu_ea_dn((Op)oi, sz, s, sa, tbl[oi][zi][p].d, tbl[oi][zi][p].s, p == 0 ? 0x1F : 0x00);
            }
    { Ea s = {M_API, 7}; alu_ea_dn(O_ADD, 1, s, 0x1F00u, 0x40u, 0x40u, 0x00); }   // 0x40 + 0x40 = 0x80: N V
    { Ea s = {M_APD, 7}; alu_ea_dn(O_SUB, 1, s, 0x1F10u, 0x10u, 0x20u, 0x1F); }   // 0x10 - 0x20 = 0xF0: X N C
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
