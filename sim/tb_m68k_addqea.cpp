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

// tb_m68k_addqea.cpp - ADDQ and SUBQ #1..#8 on an address register and on memory (PRM 4: 0101 ddd o ss mmm rrr, o = 0 ADDQ,
// 1 SUBQ; ss = 00 .B, 01 .W, 10 .L; data field 0 = 8). ADDQ.L really exists, alongside .W and .B.
//
// What it tests (286 cases):
//  - 96 cases on An (A0..A7): ADDQ and SUBQ x .W and .L x data 1..8 x 3 values that cross bit 15/16 and bit 31
//    (ADDQ: $0000FFFF-data+1 -> $00010000, $FFFFFFFF-data+1 -> 0, $7FFFFFFF-data+1 -> $80000000; SUBQ: the symmetric ones):
//    32-bit operation also for .W and CCR intact; input CCR rotating through $1F $00 $15 $0A;
//  - 8 cases ADDQ.W and ADDQ.L #1,An with input CCRs chosen so that any update of the flags (at 16 or at 32
//    bits) would change them;
//  - 180 cases on memory: ADDQ and SUBQ x .B .W .L x 10 forms (An), (An)+, -(An), d16(An) positive ($1234) and negative
//    ($FF00), d8(An,Dx.W) (index $5A5AFFC0: low half -64, high bits to be ignored; d8 = +$12), d8(An,Ax.L) (index
//    $FFFFFF80 = -128; d8 = $EE = -18), abs.W positive, abs.W negative ($Cxxx -> $FFCxxx), abs.L x 3 values (carry or
//    borrow with result 0 / -1, overflow with minimum / maximum result, pattern without flags); data 1..8 rotating
//    (#8 included); byte at even and odd address on alternate cases;
//  - 2 A7 byte cases: ADDQ.B #1,(A7)+ and SUBQ.B #1,-(A7) (A7 moves by 2).
//  After every instruction the whole SR, the base register (expected updated for (An)+ and -(An), intact in the other
//  modes) and the index register (intact) are saved. Every memory operand is surrounded by all-different sentinels: the checks cover the
//  final value of the operand bytes, no write elsewhere, the sequence of cycles on the operand (every word read
//  once, then written once: read-modify-write) and UDS/LDS of every cycle (even byte = UDS, odd = LDS).
// Expectations (written by hand from the manual, addq_model): PRM 4 ADDQ/SUBQ and PRM 3 (condition code table, ADD and
// SUB rows): X = C = carry/borrow from the msb of the size, V = signed overflow, N = msb, Z = zero result; on An
// (PRM 4 ADDQ/SUBQ: "the entire destination address register is used regardless of the operation size", condition codes
// not affected): 32 bits and CCR intact. Modes (PRM 2): (An)+ and -(An) by 1/2/4 according to the size, by 2 for A7 with bytes;
// d8(An,Xn) = An + Xn (.W sign-extended or .L) + sign-extended d8; abs.W sign-extended. Cycle sequence:
// MC68000UM 8 (ADDQ/SUBQ timing on memory: one read per operand word in the EA calculation, then the writes);
// UDS/LDS: MC68000UM 5.1. Expected SR high byte $27 (S=1, T=0, mask 7 from reset, MC68000UM 6.3).
// Bus (every cycle): FC 110 for fetches and 101 for data (MC68000UM, function codes), UDS and LDS on fetches,
// no write outside the expected bytes, no read of uninitialised memory, no access to the vectors.
// The program runs twice: DTACK immediate and DTACK delayed by 3 clocks; same expectations.
// What it does NOT test: ADDQ/SUBQ on Dn (tb_m68k_subq), the order of the two words of a .L operand (not fixed by the
// manual), the number of clocks, user mode.

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
// Infrastructure shared by the per-instruction testbenches (copied identically into every testbench: every file is self-contained)
// =====================================================================================================================
static Vm68k_seq* dut = nullptr;
static uint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static constexpr uint32_t ORG     = 0x010000;   // program origin (pc_init)
static constexpr uint32_t RES     = 0x001000;   // result area (reached with positive abs.W)
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
        // MC68000UM 6.3 (reset): the first four reads fetch the initial SSP and PC from addresses 0..7
        // (emulated by the shim sim/m68k_seq_si.sv: SSP = 0, PC = pc_init)
        check(!c.wr && c.a < 8, tag + "read of reset vector no." + std::to_string(idx) + " (address)", 2u * (uint32_t)idx, c.a);
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
                if (end_hits >= 2) stop = true;   // second fetch of the BRA.S *-0: the branch has been executed, everything before is finished
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
static constexpr uint32_t DATA_LO = 0x020000, DATA_HI = 0x030000;   // operands reached by register and abs.L
static constexpr uint32_t LOWW_LO = 0x000400, LOWW_HI = 0x001000;   // positive abs.W operands
static constexpr uint32_t HIGW_LO = 0xFFC000, HIGW_HI = 0xFFF000;   // negative abs.W operands ($C000..$EFFF -> $FFC000..$FFEFFF)
static uint32_t data_ptr = DATA_LO, loww_ptr = LOWW_LO, higw_ptr = HIGW_LO;
// 16-byte slot: all-different sentinels, operand at base+4 (+1 if odd), big endian
static uint32_t opnd_alloc(uint32_t& ptr, uint32_t hi, int size, bool odd, uint32_t val) {
    const uint32_t base = ptr; ptr += 16; if (ptr > hi) build_ok = false;
    for (uint32_t i = 0; i < 16; i++) init_b(base + i, (uint8_t)(0xA1u + 0x0Bu * i));
    const uint32_t a = base + 4u + (odd ? 1u : 0u);
    for (int i = 0; i < size; i++) init_b(a + (uint32_t)i, (uint8_t)(val >> (8 * (size - 1 - i))));
    return a;
}

static const char* szs(int sz) { return sz == 1 ? ".B" : sz == 2 ? ".W" : ".L"; }
struct AqOut { uint32_t v; unsigned ccr; };

// PRM 4 ADDQ and SUBQ, PRM 3 (condition code table, ADD and SUB rows). Sm, Dm, Rm = msb of source,
// destination and result in the size; the data q (1..8) has Sm = 0 in every size.
//  ADDQ: R = D + q;  C = Sm.Dm + !Rm.Dm + Sm.!Rm;  V = Sm.Dm.!Rm + !Sm.!Dm.Rm
//  SUBQ: R = D - q;  C = Sm.!Dm + Rm.!Dm + Sm.Rm;  V = !Sm.Dm.!Rm + Sm.!Dm.Rm
//  X = C; N = Rm; Z = 1 if R = 0 in the size.
static AqOut addq_model(bool sub, int sz, uint32_t dst, unsigned q) {
    const unsigned bits = 8u * (unsigned)sz;
    const uint32_t mask = bits == 32 ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    const uint32_t msb = 1u << (bits - 1u);
    const uint32_t d = dst & mask, s = q;
    const uint32_t r = (sub ? (d - s) : (d + s)) & mask;
    const bool sm = (s & msb) != 0, dm = (d & msb) != 0, rm = (r & msb) != 0;
    unsigned c, v;
    if (!sub) {
        c = ((sm && dm) || (!rm && dm) || (sm && !rm)) ? 1u : 0u;
        v = ((sm && dm && !rm) || (!sm && !dm && rm)) ? 1u : 0u;
    } else {
        c = ((sm && !dm) || (rm && !dm) || (sm && rm)) ? 1u : 0u;
        v = ((!sm && dm && !rm) || (sm && !dm && rm)) ? 1u : 0u;
    }
    const unsigned n = rm ? 1u : 0u, z = (r == 0) ? 1u : 0u;
    AqOut o;
    o.v = (dst & ~mask) | r;
    o.ccr = (c << 4) | (n << 3) | (z << 2) | (v << 1) | c;
    return o;
}

static void build() {
    static const int sizes[3] = {1, 2, 4};
    static const unsigned kCcrIn[4] = {0x1Fu, 0x00u, 0x15u, 0x0Au};
    static const char* const kForm[10] = {"(An)", "(An)+", "-(An)", "d16(An) +$1234", "d16(An) -$100", "d8(An,Dx.W) +$12",
                                          "d8(An,Ax.L) -$12", "abs.W positive", "abs.W negative", "abs.L"};
    unsigned k = 0;
    // --- part 1: ADDQ and SUBQ .W/.L #q,An (mode 001): 32-bit addition/subtraction also for .W, CCR intact ---
    for (int sub = 0; sub < 2; sub++)
    for (int si = 1; si < 3; si++)
    for (unsigned q = 1; q <= 8; q++)
    for (int vi = 0; vi < 3; vi++, k++) {
        const int sz = sizes[si];
        const uint32_t avA[3] = {0x0000FFFFu - (q - 1u), 0xFFFFFFFFu - (q - 1u), 0x7FFFFFFFu - (q - 1u)};   // -> $00010000, 0, $80000000
        const uint32_t avS[3] = {0x00010000u + (q - 1u), q - 1u, 0x80000000u + (q - 1u)};                   // -> $0000FFFF, $FFFFFFFF, $7FFFFFFF
        const uint32_t a = sub ? avS[vi] : avA[vi];
        const uint32_t r = sub ? (a - q) : (a + q);
        const int an = (int)(k % 8u);
        const unsigned ccr_in = kCcrIn[k % 4u];
        const unsigned szf = sz == 2 ? 1u : 2u;
        const uint16_t op = (uint16_t)(0x5000u | ((q & 7u) << 9) | ((unsigned)sub << 8) | (szf << 6) | 0x0008u | (unsigned)an);
        const std::string what = "case " + std::to_string(k) + " " + (sub ? "SUBQ" : "ADDQ") + szs(sz) + " #" + std::to_string(q)
            + ",A" + std::to_string(an) + " on " + hx(a) + " CCR " + hx(ccr_in) + " [op " + hx(op) + "]";
        set_a(an, a);
        set_ccr(ccr_in);
        w(op);
        save_sr(ccr_in, what);
        save_a(an, r, what);
    }
    // --- part 2: ADDQ.W and ADDQ.L #1,An: CCR intact with inputs that a flag update would change ---
    static const uint32_t a1v[4] = {0x0000FFFFu, 0x7FFFFFFFu, 0xFFFFFFFFu, 0x12348000u};
    static const unsigned a1c[4] = {0x00u, 0x1Fu, 0x0Au, 0x15u};
    for (int si = 1; si < 3; si++)
    for (int vi = 0; vi < 4; vi++, k++) {
        const int sz = sizes[si];
        const uint32_t a = a1v[vi], r = a + 1u;
        const int an = (int)(k % 8u);
        const unsigned szf = sz == 2 ? 1u : 2u;
        const uint16_t op = (uint16_t)(0x5000u | (1u << 9) | (szf << 6) | 0x0008u | (unsigned)an);
        const std::string what = "case " + std::to_string(k) + " ADDQ" + szs(sz) + " #1,A" + std::to_string(an) + " on " + hx(a)
            + " CCR " + hx(a1c[vi]) + " [op " + hx(op) + "]";
        set_a(an, a);
        set_ccr(a1c[vi]);
        w(op);
        save_sr(a1c[vi], what);
        save_a(an, r, what);
    }
    // --- part 3: ADDQ and SUBQ .B/.W/.L on memory, 10 EA forms ---
    for (int sub = 0; sub < 2; sub++)
    for (int si = 0; si < 3; si++)
    for (int form = 0; form < 10; form++)
    for (int vi = 0; vi < 3; vi++, k++) {
        const int sz = sizes[si];
        const unsigned bits = 8u * (unsigned)sz;
        const uint32_t mask = bits == 32 ? 0xFFFFFFFFu : ((1u << bits) - 1u);
        const uint32_t minv = 1u << (bits - 1u), maxv = minv - 1u, allv = mask;
        const unsigned q = 1u + (k % 8u);
        const uint32_t pat = sz == 1 ? 0x3Cu : sz == 2 ? 0x3C5Au : 0x3C5A96E1u;
        uint32_t val;
        if (vi == 0) val = sub ? (q - 1u) : ((allv - q + 1u) & mask);              // result -1 (borrow) / 0 (carry, Z)
        else if (vi == 1) val = sub ? (minv + q - 1u) : ((maxv - q + 1u) & mask);  // overflow: maximum / minimum result
        else val = pat;                                                            // neither carry nor overflow
        const bool odd = sz == 1 && (k & 1u) != 0;
        uint32_t a;
        if (form == 7) a = opnd_alloc(loww_ptr, LOWW_HI, sz, odd, val);
        else if (form == 8) a = opnd_alloc(higw_ptr, HIGW_HI, sz, odd, val);
        else a = opnd_alloc(data_ptr, DATA_HI, sz, odd, val);
        const AqOut r = addq_model(sub != 0, sz, val, q);
        const unsigned ccr_in = kCcrIn[k % 4u];
        const int n = (int)(k % 7u);              // base register A0..A6
        const int xd = (int)((k + 3u) % 8u);      // data index
        const int xa = (n + 1) % 7;               // address index, different from the base
        const uint32_t xdv = 0x5A5AFFC0u;         // .W = -64: bits 16..31 must be ignored
        const uint32_t xav = 0xFFFFFF80u;         // .L = -128
        uint32_t an0 = a, an1 = a;                // An before and after
        unsigned mode = 0, reg = (unsigned)n;
        std::vector<uint16_t> ext;
        switch (form) {
            case 0: mode = 2; break;                                                       // (An)
            case 1: mode = 3; an1 = a + (uint32_t)sz; break;                               // (An)+
            case 2: mode = 4; an0 = a + (uint32_t)sz; break;                               // -(An)
            case 3: mode = 5; an0 = a - 0x1234u; an1 = an0; ext.push_back(0x1234); break;  // d16 = +$1234
            case 4: mode = 5; an0 = a + 0x100u; an1 = an0; ext.push_back(0xFF00); break;   // d16 = -$100
            case 5: mode = 6; an0 = a - 0x12u + 64u; an1 = an0;                            // d8 = +$12, Dx.W = -64
                    ext.push_back((uint16_t)(((unsigned)xd << 12) | 0x12u)); break;
            case 6: mode = 6; an0 = a + 0x12u + 128u; an1 = an0;                           // d8 = $EE = -$12, Ax.L = -128
                    ext.push_back((uint16_t)(0x8000u | ((unsigned)xa << 12) | 0x0800u | 0xEEu)); break;
            case 7: case 8: mode = 7; reg = 0; ext.push_back((uint16_t)a); break;          // abs.W (sign-extended)
            default: mode = 7; reg = 1; ext.push_back((uint16_t)(a >> 16)); ext.push_back((uint16_t)a); break;   // abs.L
        }
        const unsigned szf = sz == 1 ? 0u : sz == 2 ? 1u : 2u;
        const uint16_t op = (uint16_t)(0x5000u | ((q & 7u) << 9) | ((unsigned)sub << 8) | (szf << 6) | (mode << 3) | reg);
        const std::string what = "case " + std::to_string(k) + " " + (sub ? "SUBQ" : "ADDQ") + szs(sz) + " #" + std::to_string(q)
            + "," + kForm[form] + " (A" + std::to_string(n) + ") operand at " + hx(a) + " = " + hx(val) + " CCR " + hx(ccr_in)
            + " [op " + hx(op) + "]";
        const bool use_an = form <= 6;
        if (use_an) set_a(n, an0);
        if (form == 5) set_d(xd, xdv);
        if (form == 6) set_a(xa, xav);
        set_ccr(ccr_in);
        w(op);
        for (uint16_t e : ext) w(e);
        save_sr(r.ccr, what);
        if (use_an) save_a(n, an1, what);
        if (form == 5) save_d(xd, xdv, what + " (index intact)");
        if (form == 6) save_a(xa, xav, what + " (index intact)");
        const uint32_t rv = r.v & mask;
        for (int i = 0; i < sz; i++) exp_b(a + (uint32_t)i, (uint8_t)(rv >> (8 * (sz - 1 - i))), what + " operand");
        tgts.push_back({a, sz, true, what});
    }
    // --- part 4: A7 with bytes, (A7)+ and -(A7) move by 2 (PRM 2: word-aligned stack pointer) ---
    {
        const uint32_t a = opnd_alloc(data_ptr, DATA_HI, 1, false, 0x7Fu);   // ADDQ.B #1: $7F + 1 = $80, N=1 V=1 -> CCR $0A
        const std::string what = "case " + std::to_string(k) + " ADDQ.B #1,(A7)+ operand at " + hx(a) + " = 7F CCR 15 [op 521F]";
        set_a(7, a);
        set_ccr(0x15);
        w(0x521F);
        save_sr(0x0Au, what);
        save_a(7, a + 2u, what);
        exp_b(a, 0x80, what + " operand");
        tgts.push_back({a, 1, true, what});
        k++;
    }
    {
        const uint32_t a = opnd_alloc(data_ptr, DATA_HI, 1, false, 0x80u);   // SUBQ.B #1: $80 - 1 = $7F, V=1 -> CCR $02
        const std::string what = "case " + std::to_string(k) + " SUBQ.B #1,-(A7) operand at " + hx(a) + " = 80 CCR 1D [op 5327]";
        set_a(7, a + 2u);
        set_ccr(0x1D);
        w(0x5327);
        save_sr(0x02u, what);
        save_a(7, a, what);
        exp_b(a, 0x7F, what + " operand");
        tgts.push_back({a, 1, true, what});
        k++;
    }
    ncases = k;
    finish_prog();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    if (!build_ok) { printf("FAIL: program construction (result or data area full) expected=1 read=0\nRESULT: FAIL\n"); return 1; }
    printf("tb_m68k_addqea: %zu cases, %zu program words, %zu expected bytes\n", ncases, code.size(), exp_wr.size());
    static const int waits[2] = {0, 3};
    for (int k = 0; k < 2; k++) {
        const std::string tag = "[DTACK+" + std::to_string(waits[k]) + "] ";
        if (run(waits[k], tag)) check_results(tag);
    }
    printf("checks run: %ld, failed: %ld\n", n_checks, n_fail);
    printf(n_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return n_fail ? 1 : 0;
}
