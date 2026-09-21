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

// tb_m68k_movepd.cpp - MOVE -(An),Dn (PRM 4 MOVE; predecrement source, PRM 2.2 mode 100).
//
// What it tests (36 cases = 3 sizes x 12):
//  - MOVE.B/.W/.L -(An),Dn with An rotating over A0..A7 (A7 in all three sizes) and Dn over D0..D7;
//  - predecrement (PRM 2.2, address register indirect with predecrement): An -= 1/2/4 for .B/.W/.L, but -(A7) as byte subtracts 2
//    (stack pointer kept even); the operand is read at the address AFTER the decrement; An after the instruction checked;
//  - .B at an even (UDS) and odd (LDS) address; -(A7).B reads the byte at the even address A7-2;
//  - values of each size: 0, 1, -1, maximum positive, minimum negative, a pattern: N=1, Z=1 and N=Z=0 in every
//    size; Dn PRELOADED (part within the size = complement of the value, bits beyond = mixed pattern: in .B and .W
//    they stay, PRM 4 MOVE);
//  - input CCR rotating $1F $00 $10 $0F: V=0 and C=0, X intact, N and Z from the value (PRM 4 MOVE, PRM 3.4);
//  - every operand has 4 sentinel bytes before and after it (all different); operand bus: each word read exactly
//    once, no write, UDS/LDS from the size and from A0 (MC68000UM 5.1), FC 101 (supervisor data).
// After each instruction the whole SR (high byte expected $27: S=1 and mask 7 from reset, MC68000UM 6.3), Dn and An are saved.
// Bus (common infrastructure): FC of the fetches 110, no write outside the expected bytes, no read of uninitialised
// memory, no access to the vectors; two runs, DTACK immediate and delayed by 3 clocks, same expectations.
// What it does NOT test: (An)+ and the other source modes (other testbenches), odd addresses for word/long (address error), the
// number of clocks, user mode (A7 here is always the SSP).

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

// =====================================================================================================================
// Common infrastructure of the per-instruction testbenches (copied identically into every testbench: each file is self-contained)
// =====================================================================================================================
static Vm68k_seq* dut = nullptr;
static uint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static constexpr uint32_t ORG     = 0x010000;   // program origin (pc_init)
static constexpr uint32_t RES     = 0x001000;   // results area (reached with positive abs.W)
static constexpr uint32_t RES_END = 0x008000;
static constexpr uint32_t VEC_END = 0x000400;   // vector table: no access expected (no exception)

static std::vector<uint16_t> code;                 // program, word i at address ORG + 2i
static std::map<uint32_t, uint8_t> init_mem;       // initial data (bytes, 24-bit address)
static std::map<uint32_t, uint8_t> exp_wr;         // bytes the program MUST write, with the final value
static std::map<uint32_t, std::string> exp_what;   // description of the expected byte
static uint32_t res_ptr = RES;
static uint32_t end_pc = 0;                        // address of the final BRA.S *-0
static long n_checks = 0, n_fail = 0;
static bool build_ok = true;
static size_t ncases = 0;

static std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "%X", v); return std::string(b); }
static void fail(const std::string& what, uint32_t exp, uint32_t got) {
    n_fail++;
    if (n_fail <= 60) printf("FAIL: %s expected=%X read=%X\n", what.c_str(), exp, got);
}
static void check(bool ok, const std::string& what, uint32_t exp, uint32_t got) { n_checks++; if (!ok) fail(what, exp, got); }

// --- program emission ---
[[maybe_unused]] static uint32_t here() { return ORG + 2u * (uint32_t)code.size(); }
static void w(uint16_t x) { code.push_back(x); }
static void l32(uint32_t x) { w((uint16_t)(x >> 16)); w((uint16_t)x); }
static void set_d(int n, uint32_t v) { w((uint16_t)(0x203C | (n << 9))); l32(v); }    // MOVE.L #v,Dn
[[maybe_unused]] static void set_a(int n, uint32_t v) { w((uint16_t)(0x207C | (n << 9))); l32(v); }    // MOVEA.L #v,An (PRM 4 MOVEA: CCR intact)
static void set_ccr(unsigned c) { w(0x44FC); w((uint16_t)(c & 0x1F)); }                   // MOVE #c,CCR (PRM 4 MOVE to CCR)
static void init_b(uint32_t a, uint8_t v) { init_mem[a & 0xFFFFFFu] = v; }
[[maybe_unused]] static void init_w(uint32_t a, uint16_t v) { init_b(a, (uint8_t)(v >> 8)); init_b(a + 1, (uint8_t)v); }
[[maybe_unused]] static void init_l(uint32_t a, uint32_t v) { init_w(a, (uint16_t)(v >> 16)); init_w(a + 2, (uint16_t)v); }
static void exp_b(uint32_t a, uint8_t v, const std::string& what) { exp_wr[a & 0xFFFFFFu] = v; exp_what[a & 0xFFFFFFu] = what; }
static void exp_w(uint32_t a, uint16_t v, const std::string& what) { exp_b(a, (uint8_t)(v >> 8), what); exp_b(a + 1, (uint8_t)v, what); }
static void exp_l(uint32_t a, uint32_t v, const std::string& what) { exp_w(a, (uint16_t)(v >> 16), what); exp_w(a + 2, (uint16_t)v, what); }
static uint32_t res_alloc(unsigned n) {
    const uint32_t a = res_ptr;
    res_ptr += n;
    if (res_ptr > RES_END) build_ok = false;
    for (unsigned i = 0; i < n; i++) init_b(a + i, 0xEE);   // sentinel: on the 68000 MOVE from SR reads the destination before writing it
    return a;
}
// MOVE SR,abs.W (0x40F8; not privileged on the 68000): expected SR = $2700 | CCR (S=1, mask 7 from reset)
static void save_sr(unsigned ccr, const std::string& what) {
    const uint32_t a = res_alloc(2); w(0x40F8); w((uint16_t)a); exp_w(a, (uint16_t)(0x2700u | (ccr & 0x1Fu)), what + " SR");
}
static void save_d(int n, uint32_t v, const std::string& what) {   // MOVE.L Dn,abs.W
    const uint32_t a = res_alloc(4); w((uint16_t)(0x21C0 | n)); w((uint16_t)a); exp_l(a, v, what + " D" + std::to_string(n));
}
[[maybe_unused]] static void save_a(int n, uint32_t v, const std::string& what) {   // MOVE.L An,abs.W
    const uint32_t a = res_alloc(4); w((uint16_t)(0x21C8 | n)); w((uint16_t)a); exp_l(a, v, what + " A" + std::to_string(n));
}
static void finish_prog() { end_pc = here(); w(0x60FE); for (int i = 0; i < 8; i++) w(0x4E71); }   // BRA.S *-0, then filler NOPs for the prefetch

// memory operand: expected accesses = read of every word, then (rmw) write of every word (UM 8: the EA time
// includes the operand fetch; each word read once); UDS/LDS from the size and from A0 (UM 5.1)
struct Tgt { uint32_t a; int size; bool rmw; std::string what; };
static std::vector<Tgt> tgts;

// --- memory and bus of the run ---
struct Cyc { uint32_t a; bool wr, uds, lds; unsigned fc; uint16_t wd; bool wd_ok; };
static std::map<uint32_t, uint8_t> mem;
static std::set<uint32_t> written;
static std::vector<Cyc> blog;
static uint8_t mem_b(uint32_t a) { auto it = mem.find(a & 0xFFFFFFu); return it == mem.end() ? (uint8_t)0 : it->second; }
static bool mem_def(uint32_t a) { return mem.count(a & 0xFFFFFFu) != 0; }
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }
static uint32_t prog_end() { return ORG + 2u * (uint32_t)code.size(); }

// cycle just closed; false = error that stops the run
static bool end_cycle(const Cyc& c, size_t idx, const std::string& tag, int& end_hits) {
    if (idx < 4) {
        // MC68000UM 6.3 (reset): the first four reads take the initial SSP and PC from addresses 0..7
        // (emulated by the shim sim/m68k_seq_si.sv: SSP = 0, PC = pc_init)
        check(!c.wr && c.a < 8, tag + "read of reset vector no. " + std::to_string(idx) + " (address)", 2u * (uint32_t)idx, c.a);
        return true;
    }
    blog.push_back(c);
    if (c.a < VEC_END) { fail(tag + "access to the vector table (unexpected exception), address", VEC_END, c.a); return false; }
    if (c.a >= ORG && c.a < prog_end()) {
        if (c.wr) { fail(tag + "write into the program area, address", 0, c.a); return false; }
        check(c.fc == 6, tag + "FC of the fetch at " + hx(c.a) + " (110 = supervisor program)", 6, c.fc);
        check(c.uds && c.lds, tag + "UDS and LDS of the fetch at " + hx(c.a) + " (2 = UDS, 1 = LDS)", 3, (c.uds ? 2u : 0u) | (c.lds ? 1u : 0u));
        if (c.a == end_pc) end_hits++;
        return true;
    }
    check(c.fc == 5, tag + "FC of the data access at " + hx(c.a) + " (101 = supervisor data)", 5, c.fc);
    if (!c.uds && !c.lds) { fail(tag + "cycle with neither UDS nor LDS at " + hx(c.a), 3, 0); return true; }
    if (c.wr) {
        if (!c.wd_ok) { fail(tag + "write without driven data at " + hx(c.a), 1, 0); return false; }
        if (c.uds) { mem[c.a] = (uint8_t)(c.wd >> 8); written.insert(c.a); }
        if (c.lds) { mem[c.a + 1] = (uint8_t)c.wd; written.insert(c.a + 1); }
    } else if ((c.uds && !mem_def(c.a)) || (c.lds && !mem_def(c.a + 1))) {
        fail(tag + "read of uninitialised memory, address", 0, c.a); return false;
    }
    return true;
}

// a complete run with DTACK delayed by `wait` clocks; true = reached the final BRA.S *-0
static bool run(int wait, const std::string& tag) {
    mem.clear(); written.clear(); blog.clear();
    for (size_t i = 0; i < code.size(); i++) {
        mem[ORG + 2u * (uint32_t)i] = (uint8_t)(code[i] >> 8);
        mem[ORG + 2u * (uint32_t)i + 1u] = (uint8_t)code[i];
    }
    for (const auto& kv : init_mem) mem[kv.first] = kv.second;
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    dut->clk = 0; dut->rst_n = 0; dut->cen = 0; dut->cen2 = 0; dut->start = 0; dut->pc_init = ORG; dut->rst_vector = 0;
    dut->ipl = 0; dut->vpa_n = 1; dut->data_in = 0; dut->dtack_n = 1; dut->halt_n = 1; dut->berr_n = 1; dut->br_n = 1;
    dut->bgack_n = 1; dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    dut->eval(); tick(); tick();
    dut->rst_n = 1; dut->start = 1; dut->cen = 1; dut->cen2 = 1; tick();
    dut->start = 0;
    const long limit = 200000L + 300L * (long)code.size();
    bool in = false, stop = false, unstable = false;
    Cyc c{};
    int cnt = 0, end_hits = 0;
    size_t ncyc = 0;
    for (long i = 0; i < limit && !stop; i++) {
        if (!dut->as_n) {
            const uint32_t a = ((uint32_t)dut->addr << 1) & 0xFFFFFEu;
            if (!in) { in = true; c = Cyc{}; c.a = a; c.fc = dut->fc; cnt = 0; unstable = false; }
            else { cnt++; if (a != c.a || dut->fc != c.fc) unstable = true; }
            if (!dut->rw) c.wr = true;
            if (!dut->uds_n) c.uds = true;
            if (!dut->lds_n) c.lds = true;
            if (!dut->rw && dut->data_oe && (!dut->uds_n || !dut->lds_n)) { c.wd = dut->data_out; c.wd_ok = true; }
            dut->data_in = (uint16_t)((mem_b(c.a) << 8) | mem_b(c.a + 1));
            dut->dtack_n = (cnt >= wait) ? 0 : 1;
        } else {
            if (in) {
                in = false;
                // MC68000UM 5.1: address and FC valid for the whole cycle (AS asserted)
                if (unstable) { fail(tag + "address or FC changed with AS asserted, cycle at", c.a, 0); stop = true; }
                if (!end_cycle(c, ncyc++, tag, end_hits)) stop = true;
                if (end_hits >= 2) stop = true;   // second fetch of the BRA.S *-0: the branch has executed, everything before it has completed
            }
            dut->dtack_n = 1;
        }
        if (dut->illegal) { fail(tag + "illegal instruction exception taken (illegal signal), PC", 0, dut->pc_dbg); stop = true; }
        dut->cen = 1; dut->cen2 = 1; tick();
    }
    const bool reached = end_hits >= 2;
    check(reached, tag + "arrival at the final BRA.S *-0 at " + hx(end_pc) + " (fetches seen)", 2, (uint32_t)end_hits);
    dut->final(); delete dut; dut = nullptr;
    return reached;
}

static void check_results(const std::string& tag) {
    for (const auto& kv : exp_wr) {
        const uint32_t a = kv.first;
        if (!written.count(a)) { n_checks++; fail(tag + exp_what[a] + ": byte " + hx(a) + " not written", kv.second, mem_b(a)); continue; }
        check(mem_b(a) == kv.second, tag + exp_what[a] + ": byte " + hx(a), kv.second, mem_b(a));
    }
    for (uint32_t a : written)
        if (!exp_wr.count(a)) { n_checks++; fail(tag + "unexpected write at byte " + hx(a), 0, mem_b(a)); }
    for (const auto& g : tgts) {
        const uint32_t w0 = g.a & ~1u, w1 = (g.a + (uint32_t)g.size - 1u) & ~1u;
        const size_t nw = (size_t)((w1 - w0) / 2u + 1u);
        std::vector<const Cyc*> acc;
        for (const auto& c : blog) if (c.a >= w0 && c.a <= w1) acc.push_back(&c);
        const size_t expn = g.rmw ? 2 * nw : nw;
        check(acc.size() == expn, tag + g.what + ": number of cycles on the operand", (uint32_t)expn, (uint32_t)acc.size());
        if (acc.size() != expn) continue;
        const bool eu = g.size > 1 || !(g.a & 1u), el = g.size > 1 || (g.a & 1u) != 0;
        std::set<uint32_t> rd, wr;
        for (size_t k = 0; k < acc.size(); k++) {
            const bool sw = g.rmw && k >= nw;
            check(acc[k]->wr == sw, tag + g.what + ": cycle " + std::to_string(k) + " on the operand (0 = read, 1 = write)", sw ? 1u : 0u, acc[k]->wr ? 1u : 0u);
            check(acc[k]->uds == eu && acc[k]->lds == el, tag + g.what + ": UDS/LDS of cycle " + std::to_string(k) + " (2 = UDS, 1 = LDS)",
                  (eu ? 2u : 0u) | (el ? 1u : 0u), (acc[k]->uds ? 2u : 0u) | (acc[k]->lds ? 1u : 0u));
            (acc[k]->wr ? wr : rd).insert(acc[k]->a);
        }
        check(rd.size() == nw, tag + g.what + ": operand words read", (uint32_t)nw, (uint32_t)rd.size());
        if (g.rmw) check(wr.size() == nw, tag + g.what + ": operand words written", (uint32_t)nw, (uint32_t)wr.size());
    }
}

// =====================================================================================================================
// Testbench: operands and cases
// =====================================================================================================================
static constexpr uint32_t DATA_LO = 0x020000, DATA_HI = 0x030000;   // operands reached by register and abs.L
static constexpr uint32_t LOWW_LO = 0x000400, LOWW_HI = 0x001000;   // positive abs.W operands
static constexpr uint32_t HIGW_LO = 0xFFC000, HIGW_HI = 0xFFF000;   // negative abs.W operands ($C000..$EFFF -> $FFC000..$FFEFFF)
[[maybe_unused]] static uint32_t data_ptr = DATA_LO, loww_ptr = LOWW_LO, higw_ptr = HIGW_LO;
// 16-byte slot: all-different sentinels, operand at base+4 (+1 if odd), big endian
static uint32_t opnd_alloc(uint32_t& ptr, uint32_t hi, int size, bool odd, uint32_t val) {
    const uint32_t base = ptr; ptr += 16; if (ptr > hi) build_ok = false;
    for (uint32_t i = 0; i < 16; i++) init_b(base + i, (uint8_t)(0xA1u + 0x0Bu * i));
    const uint32_t a = base + 4u + (odd ? 1u : 0u);
    for (int i = 0; i < size; i++) init_b(a + (uint32_t)i, (uint8_t)(val >> (8 * (size - 1 - i))));
    return a;
}

// Expectations (PRM 4 MOVE; PRM 2.2 predecrement): An <- An - d (d = 1/2/4; -(A7) as byte: d = 2); Dn <- (An) within the
// size, bits beyond the size intact; X intact; N = msb of the value; Z = zero value; V = 0; C = 0.
static void build() {
    static const uint32_t vals[3][6] = {
        {0x00u, 0x01u, 0xFFu, 0x7Fu, 0x80u, 0x5Au},
        {0x0000u, 0x0001u, 0xFFFFu, 0x7FFFu, 0x8000u, 0xA55Au},
        {0x00000000u, 0x00000001u, 0xFFFFFFFFu, 0x7FFFFFFFu, 0x80000000u, 0x5AA5C33Cu}};
    static const uint32_t pat[4] = {0x69A5C31Eu, 0x965A3CE1u, 0x3C1E69A5u, 0xC3E1965Au};   // high bits of the Dn preload
    static const unsigned ccrs[4] = {0x1Fu, 0x00u, 0x10u, 0x0Fu};
    unsigned k = 0;
    for (int si = 0; si < 3; si++)
    for (int j = 0; j < 12; j++, k++) {
        const int sz = si == 0 ? 1 : si == 1 ? 2 : 4;
        const uint32_t mask = sz == 1 ? 0xFFu : sz == 2 ? 0xFFFFu : 0xFFFFFFFFu;
        const uint32_t msb  = sz == 1 ? 0x80u : sz == 2 ? 0x8000u : 0x80000000u;
        const int an = (int)(k % 8u);
        const int dn = (int)((k * 5u + 3u) % 8u);
        const uint32_t val = vals[si][j / 2];
        const bool odd = (sz == 1) && (j & 1) && an != 7;                         // -(A7).B: address always even
        const uint32_t dec = (sz == 1) ? (an == 7 ? 2u : 1u) : (uint32_t)sz;   // PRM 2.2: A7 as byte moves by 2
        const uint32_t a = opnd_alloc(data_ptr, DATA_HI, sz, odd, val);
        const unsigned ccr_in = ccrs[k % 4u];
        const uint32_t pre = (pat[(k / 4u) % 4u] & ~mask) | (~val & mask);
        const uint32_t res = (pre & ~mask) | (val & mask);
        const unsigned ccr = (ccr_in & 0x10u) | ((val & msb) ? 0x08u : 0u) | (((val & mask) == 0u) ? 0x04u : 0u);
        const uint16_t op = (uint16_t)((sz == 1 ? 0x1020u : sz == 2 ? 0x3020u : 0x2020u) | ((unsigned)dn << 9) | (unsigned)an);
        const std::string what = "case " + std::to_string(k) + " MOVE" + (sz == 1 ? ".B" : sz == 2 ? ".W" : ".L") + " -(A"
            + std::to_string(an) + "=" + hx(a + dec) + "),D" + std::to_string(dn) + " value " + hx(val) + " at " + hx(a)
            + " CCR " + hx(ccr_in) + " [op " + hx(op) + "]";
        set_a(an, a + dec);
        set_d(dn, pre);
        set_ccr(ccr_in);
        w(op);
        save_sr(ccr, what);
        save_d(dn, res, what);
        save_a(an, a, what + " (after the predecrement)");
        tgts.push_back({a, sz, false, what});
    }
    ncases = k;
    finish_prog();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    if (!build_ok) { printf("FAIL: program construction (results area full) expected=1 read=0\nRESULT: FAIL\n"); return 1; }
    printf("tb_m68k_movepd: %zu cases, %zu program words, %zu expected bytes\n", ncases, code.size(), exp_wr.size());
    static const int waits[2] = {0, 3};
    for (int k = 0; k < 2; k++) {
        const std::string tag = "[DTACK+" + std::to_string(waits[k]) + "] ";
        if (run(waits[k], tag)) check_results(tag);
    }
    printf("checks run: %ld, failed: %ld\n", n_checks, n_fail);
    printf(n_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return n_fail ? 1 : 0;
}
