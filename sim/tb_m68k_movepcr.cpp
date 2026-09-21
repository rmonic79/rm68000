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

// tb_m68k_movepcr.cpp - MOVE with a PC-relative source, d16(PC) and d8(PC,Xn), to Dn, on the full processor.
//
// What it tests (expectations from the manual):
//  - d16(PC) (PRM 2.2.11): EA = address of the extension word + sign-extended d16; positive and
//    negative displacement, MOVE.B (even and odd byte), MOVE.W, MOVE.L.
//  - d8(PC,Xn) (PRM 2.2.12): EA = address of the extension word + extended d8 + index; index Dn and An, .W (low
//    word sign-extended: the high word of the register is preloaded to a value other than 0/FFFF, so an index taken as .L goes
//    elsewhere) and .L (value 0x0000FFF8 which taken as .W would be -8), positive, negative and zero d8, negative index.
//  - flags (PRM 4 MOVE): X unchanged, N and Z from the data at the operation size, V = C = 0; input CCR chosen
//    to see every bit change or stay. Dn: only the low byte/word changes, the high bits are preloaded.
//  - bus: the PC-relative operand reads are program references (PRM 2.2.11/2.2.12: "the reference is
//    classified as a program reference"): FC = 6 (supervisor program). The data sit in program areas (islands
//    jumped over by a BRA.S, zones above and below the code, far zone for the .L index). Every other cycle: FC and areas.
//    Two runs: DTACK with no wait states and with 3 wait states.
// Does not test: d16(PC) as destination (not allowed on the 68000), MOVEA/LEA/JMP PC-relative (other testbenches).

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
// program words that contain a byte operand read PC-relative: 0 = even byte (UDS only), 1 = odd (LDS
// only). The same word may also be fetched as an instruction (prefetch, word-sized), so there both apply.
static std::map<uint32_t, int> byte_rd;
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
        if (k != 1 || i < 4) {
            auto b = byte_rd.find(c.a);
            bool okb = b != byte_rd.end() && i >= 4 && (b->second == 0 ? (c.u && !c.l) : (!c.u && c.l));   // UM 5.1: byte = a single strobe
            check((c.u && c.l) || okb, w + ": strobe (word: UDS and LDS; byte operand: the one of its lane)", 3, (c.u ? 2u : 0u) | (c.l ? 1u : 0u));
        }
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

// ============================================================================== testbench program
// program data (program areas, not checked as data areas): written byte-wise, big endian
static void put(uint32_t a, uint32_t v, int sz) {
    for (int i = 0; i < sz; i++) wr8(a + (uint32_t)i, (uint8_t)(v >> (8 * (sz - 1 - i))));
    if (sz == 1) byte_rd[a & ~1u] = (int)(a & 1u);
}
static uint16_t mv_op(int sz, uint16_t eaf) { return (uint16_t)((sz == 1 ? 0x1000 : sz == 2 ? 0x3000 : 0x2000) | eaf); }   // MOVE.<sz> <ea>,D0
static int ncase = 0;
static std::string cname(const char* s) { return "case " + std::to_string(ncase++) + " " + s; }
static void results(const std::string& nm, int sz, uint32_t pre, uint32_t v, uint8_t ccr) {
    save_sr((uint16_t)(0x2700 | cc_nz(sz, v, ccr)), 0xFFFF, nm + ": SR (PRM 4 MOVE)");
    save_d(0, (pre & ~mk(sz)) | (v & mk(sz)), nm + ": D0, only the part of the operation size changes");
}
// MOVE.<sz> d16(PC),D0 with the data at 'at'
static void pc_d16(int sz, uint32_t at, uint32_t v, uint32_t pre, uint8_t ccr, const char* s) {
    std::string nm = cname(s);
    put(at, v, sz);
    ld_d(0, pre);
    set_ccr(ccr);
    W(mv_op(sz, 0x3A));
    W((uint16_t)(at - pc_e));                     // pc_e = address of the extension word
    results(nm, sz, pre, v, ccr);
}
static uint16_t brief(bool xa, int xr, bool xl, int d8) { return (uint16_t)((xa ? 0x8000 : 0) | (xr << 12) | (xl ? 0x0800 : 0) | (d8 & 0xFF)); }
static void ld_x(bool xa, int xr, uint32_t xv) { if (xa) ld_a(xr, xv); else ld_d(xr, xv); }
// MOVE.<sz> d8(PC,Xn),D0; 8-byte island after (before = false) or before (before = true) the instruction, data at 'off'
static void pc_idx(bool before, int sz, bool xa, int xr, bool xl, uint32_t xv, uint32_t off, uint32_t v, uint32_t pre, uint8_t ccr, const char* s) {
    std::string nm = cname(s);
    ld_x(xa, xr, xv);
    ld_d(0, pre);
    set_ccr(ccr);
    uint32_t isl = 0;
    if (before) { W(0x6008); isl = pc_e; for (int i = 0; i < 4; i++) W(0x4AFC); }   // BRA.S +8 past the island
    uint32_t p = pc_e, ext = p + 2;
    if (!before) isl = p + 6;
    uint32_t ea = isl + off, iv = xl ? xv : sx16(xv);
    int32_t d8 = (int32_t)(ea - ext - iv);
    check(d8 >= -128 && d8 <= 127, nm + ": testbench construction, d8 in range", 0, (uint32_t)d8);
    W(mv_op(sz, 0x3B));
    W(brief(xa, xr, xl, (int)d8));
    if (!before) { W(0x6008); for (int i = 0; i < 4; i++) W(0x4AFC); }
    put(ea, v, sz);
    results(nm, sz, pre, v, ccr);
}
// MOVE.<sz> d8(PC,Dxr.L),D0 with a long index: the data sit in the far zone 0x14000
static void pc_idx_far(int sz, int xr, uint32_t xv, int d8, uint32_t v, uint32_t pre, uint8_t ccr, const char* s) {
    std::string nm = cname(s);
    ld_d(xr, xv);
    ld_d(0, pre);
    set_ccr(ccr);
    uint32_t ea = pc_e + 2 + sx8((uint32_t)d8) + xv;
    check(ea >= 0x14000u && ea < 0x15000u, nm + ": testbench construction, data in the far zone", 0x14000u, ea);
    W(mv_op(sz, 0x3B));
    W(brief(false, xr, true, d8));
    put(ea, v, sz);
    results(nm, sz, pre, v, ccr);
}

static void build() {
    ncase = 0; byte_rd.clear();
    vectors(0x00002000u, 0x00004000u);
    prog_area(0x3E00u, 0x6000u);      // code from 0x4000, data at negative d16 in 0x3F00, at positive d16 in 0x5800
    prog_area(0x14000u, 0x15000u);    // far zone for the .L index
    data_area(0x7000u, 0x7400u);      // results
    pc_e = 0x4000u;
    pc_d16(1, 0x5800u, 0x80u, 0xCCCCCCCCu, 0x17, "MOVE.B d16(PC),D0 d16>0, even byte, N");
    pc_d16(1, 0x5809u, 0x00u, 0x33333333u, 0x0B, "MOVE.B d16(PC),D0 d16>0, odd byte, Z");
    pc_d16(1, 0x3F01u, 0x7Fu, 0xCCCCCCCCu, 0x1F, "MOVE.B d16(PC),D0 d16<0, odd byte");
    pc_d16(2, 0x3F10u, 0x8001u, 0x5555AAAAu, 0x1F, "MOVE.W d16(PC),D0 d16<0, N");
    pc_d16(2, 0x5820u, 0x7FFFu, 0xAAAA5555u, 0x0F, "MOVE.W d16(PC),D0 d16>0");
    pc_d16(2, 0x3F20u, 0x0000u, 0x12345678u, 0x00, "MOVE.W d16(PC),D0 d16<0, Z");
    pc_d16(4, 0x3F30u, 0x00000000u, 0xFFFFFFFFu, 0x1B, "MOVE.L d16(PC),D0 d16<0, Z");
    pc_d16(4, 0x5830u, 0x89ABCDEFu, 0x00000000u, 0x00, "MOVE.L d16(PC),D0 d16>0, N");
    pc_idx(false, 2, false, 1, false, 0x5A5A0006u, 0, 0x1234u, 0xF0F0F0F0u, 0x1F, "MOVE.W d8(PC,D1.W),D0 d8<0, high word of the index 5A5A");
    pc_idx_far(4, 2, 0x0000FFF8u, 0x10, 0xFEDCBA98u, 0x11111111u, 0x04, "MOVE.L d8(PC,D2.L),D0 .L index 0x0000FFF8");
    pc_idx(true, 2, false, 3, false, 0x0001FFFCu, 0, 0x0000u, 0x87654321u, 0x13, "MOVE.W d8(PC,D3.W),D0 index -4 and negative d8, Z");
    pc_idx(false, 4, true, 2, true, 0xFFFFFFF8u, 0, 0x00C0FFEEu, 0x99999999u, 0x0E, "MOVE.L d8(PC,A2.L),D0 negative An index");
    pc_idx(false, 1, true, 3, false, 0x12340003u, 1, 0xFFu, 0x0F0F0F0Fu, 0x10, "MOVE.B d8(PC,A3.W),D0 odd byte, N");
    pc_idx(false, 2, false, 4, true, 0x00000004u, 0, 0xA5A5u, 0x00000000u, 0x01, "MOVE.W d8(PC,D4.L),D0 d8 = 0, N");
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
