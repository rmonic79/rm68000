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

// tb_m68k_shiftbw.cpp - ASL ASR LSL LSR ROL ROR on the data register, register form (PRM 4: 1110 ccc d ss i tt rrr).
//
// What it tests (648 cases = 6 instructions x 3 sizes x 18 counts x 2 values):
//  - sizes .B .W .L; immediate count 1..8 (field 0 = 8) and from register 0 1 7 8 15 16 31 32 33 63;
//    the count from register is Dx modulo 64 (PRM 4 ASL/LSL/ROL: "shift count ... modulo 64"): in the second value of
//    each pair the count register has bits 6..31 mixed 1/0 (cnt | $A5A5A5C0), which must be ignored;
//  - values with the bits beyond the size PRELOADED (e.g. $5A3C96F1 for .B): an instruction executed at the wrong
//    size changes the high bits or the incoming bit and the 32-bit comparison fails;
//  - after every instruction the whole SR (MOVE SR,abs.W), the result register and the count register
//    (MOVE.L Dn,abs.W) are saved and compared with the expectations. Input CCR $1F in the first value and $00 in the second: X
//    intact with count 0 is visible in both directions, C=0 with count 0 and V=0 start from 1.
// Expectations: step model written from the manual (shift_model): PRM 4 ASL/ASR ("V set if the most significant bit is
// changed at any time during the shift operation"; X and C = last bit shifted out; C = 0 with count 0; X intact with
// count 0), PRM 4 LSL/LSR (V = 0), PRM 4 ROL/ROR (X not touched, C = last bit rotated, 0 with count 0);
// N = msb of the result within the size, Z = zero result within the size (PRM 3, condition codes).
// Expected high byte of the SR $27: S=1, T=0, mask 7 after the reset (MC68000UM 6.3, reset) and no instruction touches it.
// Bus (every cycle): FC 110 for fetches and 101 for data (MC68000UM, function codes), UDS and LDS on fetches,
// no write outside the expected bytes, no read of uninitialized memory, no access to the vectors.
// The program runs twice: DTACK immediate and DTACK delayed by 3 clocks; same expectations.
// What it does NOT test: ROXL/ROXR (tb_m68k_roxl), the memory form (tb_m68k_shiftmem), the number of clocks, user mode.

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
static std::map<uint32_t, uint8_t> exp_wr;         // bytes that the program MUST write, with the final value
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
// MOVE SR,abs.W (0x40F8; not privileged on the 68000): expected SR = $2700 | CCR (S=1, mask 7 from the reset)
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

// operand in memory: expected accesses = read of every word, then (rmw) write of every word (UM 8: the EA time
// includes the operand fetch; every word read once); UDS/LDS from the size and from A0 (UM 5.1)
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
        check(!c.wr && c.a < 8, tag + "read of reset vector no." + std::to_string(idx) + " (address)", 2u * (uint32_t)idx, c.a);
        return true;
    }
    blog.push_back(c);
    if (c.a < VEC_END) { fail(tag + "access to the vector table (unexpected exception), address", VEC_END, c.a); return false; }
    if (c.a >= ORG && c.a < prog_end()) {
        if (c.wr) { fail(tag + "write to the program area, address", 0, c.a); return false; }
        check(c.fc == 6, tag + "FC of the fetch at " + hx(c.a) + " (110 = supervisor program)", 6, c.fc);
        check(c.uds && c.lds, tag + "UDS and LDS of the fetch at " + hx(c.a) + " (2 = UDS, 1 = LDS)", 3, (c.uds ? 2u : 0u) | (c.lds ? 1u : 0u));
        if (c.a == end_pc) end_hits++;
        return true;
    }
    check(c.fc == 5, tag + "FC of the data access at " + hx(c.a) + " (101 = supervisor data)", 5, c.fc);
    if (!c.uds && !c.lds) { fail(tag + "cycle without UDS or LDS at " + hx(c.a), 3, 0); return true; }
    if (c.wr) {
        if (!c.wd_ok) { fail(tag + "write without driven data at " + hx(c.a), 1, 0); return false; }
        if (c.uds) { mem[c.a] = (uint8_t)(c.wd >> 8); written.insert(c.a); }
        if (c.lds) { mem[c.a + 1] = (uint8_t)c.wd; written.insert(c.a + 1); }
    } else if ((c.uds && !mem_def(c.a)) || (c.lds && !mem_def(c.a + 1))) {
        fail(tag + "read of uninitialized memory, address", 0, c.a); return false;
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
                // MC68000UM 5.1: address and FC valid for the whole cycle (AS active)
                if (unstable) { fail(tag + "address or FC changed with AS active, cycle at", c.a, 0); stop = true; }
                if (!end_cycle(c, ncyc++, tag, end_hits)) stop = true;
                if (end_hits >= 2) stop = true;   // second fetch of the BRA.S *-0: the branch has been executed, everything before it has finished
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
        if (!exp_wr.count(a)) { n_checks++; fail(tag + "unexpected write to byte " + hx(a), 0, mem_b(a)); }
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
// Testbench: model and cases
// =====================================================================================================================
enum { T_AS = 0, T_LS = 1, T_ROX = 2, T_RO = 3 };   // type field, bits 4-3 of the register form (PRM 4)
static const char* const kName[4][2] = {{"ASR", "ASL"}, {"LSR", "LSL"}, {"ROXR", "ROXL"}, {"ROR", "ROL"}};
struct ShOut { uint32_t v; unsigned ccr; };

// Step model, one bit per step, as the manual describes the instructions (PRM 4 ASL/ASR, LSL/LSR, ROL/ROR, ROXL/ROXR):
//  left:  the msb goes out; 0 comes in (AS, LS), the bit shifted out (RO), X (ROX). ASL: V=1 if the msb changes in a step.
//  right: bit 0 goes out; the msb comes in (AS), 0 (LS), the bit shifted out (RO), X (ROX).
//  C = last bit shifted out; X = last bit shifted out except RO (X not touched); count 0: X intact, C = 0 (ROX: C = X).
//  N = msb of the result, Z = zero result (within the size); V = 0 except ASL. The bits beyond the size are kept.
static ShOut shift_model(int type, bool left, int sz, uint32_t dst, unsigned cnt, unsigned ccr_in) {
    const unsigned bits = 8u * (unsigned)sz;
    const uint32_t mask = bits == 32 ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    const uint32_t msb = 1u << (bits - 1u);
    uint32_t v = dst & mask;
    unsigned x = (ccr_in >> 4) & 1u, c = 0, vf = 0;
    for (unsigned i = 0; i < cnt; i++) {
        unsigned out;
        if (left) {
            out = (v & msb) ? 1u : 0u;
            const unsigned in = (type == T_RO) ? out : (type == T_ROX) ? x : 0u;
            const uint32_t nv = ((v << 1) | in) & mask;
            if (type == T_AS && (((nv & msb) ? 1u : 0u) != out)) vf = 1;
            v = nv;
        } else {
            out = v & 1u;
            const unsigned in = (type == T_RO) ? out : (type == T_ROX) ? x : (type == T_AS) ? ((v & msb) ? 1u : 0u) : 0u;
            v = (v >> 1) | (in ? msb : 0u);
        }
        c = out;
        if (type != T_RO) x = out;
    }
    if (cnt == 0) c = (type == T_ROX) ? x : 0u;
    const unsigned n = (v & msb) ? 1u : 0u, z = (v == 0) ? 1u : 0u;
    ShOut r;
    r.v = (dst & ~mask) | v;
    r.ccr = (x << 4) | (n << 3) | (z << 2) | (vf << 1) | c;
    return r;
}

static void build() {
    static const int types[3] = {T_AS, T_LS, T_RO};
    static const int sizes[3] = {1, 2, 4};
    // first value: msb 1 with leading ones (ASL: V=0 for the first counts, then 1); second: leading zeros
    static const uint32_t valA[3] = {0x5A3C96F1u, 0x3C5AF1B4u, 0xF1B496A5u};
    static const uint32_t valB[3] = {0xA5C3690Eu, 0xC3A50E4Bu, 0x0E4B695Au};
    static const unsigned regcnt[10] = {0, 1, 7, 8, 15, 16, 31, 32, 33, 63};
    unsigned k = 0;
    for (int ti = 0; ti < 3; ti++)
    for (int left = 0; left < 2; left++)
    for (int si = 0; si < 3; si++)
    for (int ci = 0; ci < 18; ci++)
    for (int vi = 0; vi < 2; vi++, k++) {
        const int type = types[ti], sz = sizes[si];
        const bool reg = ci >= 8;
        const unsigned cnt = reg ? regcnt[ci - 8] : (unsigned)(ci + 1);
        const uint32_t val = vi ? valB[si] : valA[si];
        const unsigned ccr_in = vi ? 0x00u : 0x1Fu;
        const int dn = 2 + (int)(k % 6u);          // D2..D7
        const int cn = (dn + 3) & 7;               // count register, always different from dn
        const uint32_t cval = vi ? (cnt | 0xA5A5A5C0u) : cnt;   // bits 0..5 = count; bits 6..31 to be ignored (modulo 64)
        const ShOut r = shift_model(type, left != 0, sz, val, cnt, ccr_in);
        const unsigned szf = sz == 1 ? 0u : sz == 2 ? 1u : 2u;
        const unsigned cfield = reg ? (unsigned)cn : (cnt & 7u);   // immediate: 8 is encoded as 0
        const uint16_t op = (uint16_t)(0xE000u | (cfield << 9) | ((unsigned)left << 8) | (szf << 6) | (reg ? 0x20u : 0u)
                                       | ((unsigned)type << 3) | (unsigned)dn);
        const std::string what = "case " + std::to_string(k) + " " + kName[type][left] + (sz == 1 ? ".B " : sz == 2 ? ".W " : ".L ")
            + (reg ? "D" + std::to_string(cn) + "(=" + hx(cval) + ")" : "#" + std::to_string(cnt)) + ",D" + std::to_string(dn)
            + " on " + hx(val) + " CCR " + hx(ccr_in) + " [op " + hx(op) + "]";
        set_d(dn, val);
        if (reg) set_d(cn, cval);
        set_ccr(ccr_in);
        w(op);
        save_sr(r.ccr, what);
        save_d(dn, r.v, what);
        if (reg) save_d(cn, cval, what + " (count register intact)");
    }
    ncases = k;
    finish_prog();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    if (!build_ok) { printf("FAIL: program construction (results area full) expected=1 read=0\nRESULT: FAIL\n"); return 1; }
    printf("tb_m68k_shiftbw: %zu cases, %zu program words, %zu expected bytes\n", ncases, code.size(), exp_wr.size());
    static const int waits[2] = {0, 3};
    for (int k = 0; k < 2; k++) {
        const std::string tag = "[DTACK+" + std::to_string(waits[k]) + "] ";
        if (run(waits[k], tag)) check_results(tag);
    }
    printf("checks run: %ld, failed: %ld\n", n_checks, n_fail);
    printf(n_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return n_fail ? 1 : 0;
}
