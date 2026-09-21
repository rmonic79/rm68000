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

// tb_m68k_movean.cpp - MOVE An,Dn (.W .L) and MOVEA <ea>,An (.W .L) from all source modes (PRM 4 MOVE, PRM 4 MOVEA).
//
// What it tests (96 cases):
//  1) 28 cases MOVE.W An,Dn and MOVE.L An,Dn (0x3008/0x2008 | Dn<<9 | An), An over A0..A7, Dn over D0..D7, values of An
//     0, $8000, $7FFF, $FFFF0000, $12345678, $80000000, $0001FFFF (N=1 and Z=1 in .W and in .L), Dn PRELOADED (part
//     within the size = complement of the value; in .W the upper 16 bits of Dn must stay); expectations PRM 4 MOVE:
//     N = msb within the size, Z = zero value within the size, V = 0, C = 0, X intact; An intact.
//  2) 64 MOVEA cases (00 ss AAA 001 mmm rrr, ss 11 = .W, 10 = .L) = 16 source forms x 2 sizes x 2 values:
//     Dn, An, (An), (An)+, -(An), d16(An) positive ($124) and negative (-$136), d8(An,Dn.W) (index with non-zero high
//     bits and a negative low part: $7A5BFFF0 -> -16), d8(An,Am.L) (index $00010040: as .W it would be $40),
//     abs.W positive ($0400..$0FFF) and negative ($C000.. -> $FFC000..), abs.L, d16(PC) (negative: the PC-relative data
//     are at the head of the program), d8(PC,Dn.W) (d8 = -6), d8(PC,Am.L) (d8 = +$10), #imm;
//     .W values $8001 $7FFF $0000 $FFFF (sign-extended to 32 bits: PRM 4 MOVEA "word-size source operands are
//     sign-extended"), .L $CAFEBABE $00000000 $7FFFFFFF; destination rotating over A0..A7, preloaded with a different
//     value; input CCR rotating $1F $00 $15 $0A $10 $0F, which must stay IDENTICAL (PRM 4 MOVEA: condition
//     codes not affected); base register after (An)+ (+2/+4) and -(An) (-2/-4), and index registers intact.
//     PRM 2.2: d16(An) = An + sign-extended d16; d8(An,Xn) = An + Xn (.W sign-extended or .L) + sign-extended d8; abs.W sign-extended;
//     d16(PC) and d8(PC,Xn): PC = address of the extension word; the fetches of PC-relative operands are in the
//     program space (FC 110, MC68000UM function codes), the others in the data space (FC 101).
//  3) 4 cases MOVEA.L #imm,A4 and MOVEA.L abs.L,A4 with input CCR $1F and $00.
// Every memory operand has 4 sentinel bytes before and after it; each of its words must be read exactly once and never
// written, with UDS and LDS (MC68000UM 5.1). After each instruction the whole SR (high byte expected $27: S=1 and
// mask 7 from reset, MC68000UM 6.3), the destination and the registers used are saved and compared with the expectations.
// The program starts with BRA.W (PRM 4 BRA: destination = address + 2 + displacement), which skips the PC-relative data.
// Bus (common infrastructure): FC of every cycle, no write outside the expected bytes, no read of uninitialised
// memory, no access to the vectors; two runs, DTACK immediate and delayed by 3 clocks, same expectations.
// What it does NOT test: MOVE.B An (it does not exist), MOVEA with source (A7)+/-(A7), the number of clocks, user mode.

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
static uint32_t data_ptr = DATA_LO, loww_ptr = LOWW_LO, higw_ptr = HIGW_LO;
// 16-byte slot: all-different sentinels, operand at base+4 (+1 if odd), big endian
static uint32_t opnd_alloc(uint32_t& ptr, uint32_t hi, int size, bool odd, uint32_t val) {
    const uint32_t base = ptr; ptr += 16; if (ptr > hi) build_ok = false;
    for (uint32_t i = 0; i < 16; i++) init_b(base + i, (uint8_t)(0xA1u + 0x0Bu * i));
    const uint32_t a = base + 4u + (odd ? 1u : 0u);
    for (int i = 0; i < size; i++) init_b(a + (uint32_t)i, (uint8_t)(val >> (8 * (size - 1 - i))));
    return a;
}

// PC-relative data: at the head of the program, skipped by BRA.W. 16-byte slots with the same sentinels as opnd_alloc,
// operand at base+4. The first slot is 4 words after the BRA.W, far from the prefetch that follows the branch.
static constexpr unsigned PCD_SLOTS = 12;
static constexpr uint32_t PCD = ORG + 4u + 8u;
static unsigned pcd_used = 0;
static void put_code_byte(uint32_t a, uint8_t v) {
    const size_t i = (size_t)((a - ORG) / 2u);
    if (a < ORG || i >= code.size()) { build_ok = false; return; }
    if (a & 1u) code[i] = (uint16_t)((code[i] & 0xFF00u) | v);
    else        code[i] = (uint16_t)((code[i] & 0x00FFu) | ((unsigned)v << 8));
}
static uint32_t pcd_alloc(int size, uint32_t val) {
    if (pcd_used >= PCD_SLOTS) { build_ok = false; return PCD + 4u; }
    const uint32_t a = PCD + 16u * pcd_used + 4u;
    pcd_used++;
    for (int i = 0; i < size; i++) put_code_byte(a + (uint32_t)i, (uint8_t)(val >> (8 * (size - 1 - i))));
    return a;
}
static void emit_head() {
    w(0x6000); w((uint16_t)(2u + 8u + 16u * PCD_SLOTS));   // BRA.W: destination = ORG + 2 + displacement = end of the data
    for (int i = 0; i < 4; i++) w(0x4E71);
    for (unsigned s = 0; s < PCD_SLOTS; s++)
        for (unsigned i = 0; i < 16; i += 2)
            w((uint16_t)(((unsigned)(uint8_t)(0xA1u + 0x0Bu * i) << 8) | (unsigned)(uint8_t)(0xA1u + 0x0Bu * (i + 1u))));
    if (here() != ORG + 2u + 2u + 8u + 16u * PCD_SLOTS) build_ok = false;
}

static const unsigned kCcr4[4] = {0x1Fu, 0x00u, 0x10u, 0x0Fu};
static const unsigned kCcr6[6] = {0x1Fu, 0x00u, 0x15u, 0x0Au, 0x10u, 0x0Fu};
static const uint32_t kPat[4] = {0x69A5C31Eu, 0x965A3CE1u, 0x3C1E69A5u, 0xC3E1965Au};   // high bits of the Dn preload

// 1) MOVE An,Dn. Expectations (PRM 4 MOVE): Dn <- An within the size (in .W the upper 16 bits of Dn stay); An intact;
//    X intact, N = msb within the size, Z = zero value within the size, V = 0, C = 0.
static void part_move_an_dn(unsigned& k) {
    static const uint32_t anv[7] = {0x00000000u, 0x00008000u, 0x00007FFFu, 0xFFFF0000u, 0x12345678u, 0x80000000u, 0x0001FFFFu};
    for (int si = 0; si < 2; si++)
    for (int vi = 0; vi < 7; vi++)
    for (int ci = 0; ci < 2; ci++, k++) {
        const bool wsz = si == 0;
        const uint32_t mask = wsz ? 0xFFFFu : 0xFFFFFFFFu, msb = wsz ? 0x8000u : 0x80000000u;
        const uint32_t v = anv[vi];
        const int an = (int)(k % 8u), dn = (int)((k * 3u + 2u) % 8u);
        const unsigned ccr_in = kCcr4[k % 4u];
        const uint32_t pre = (kPat[(k / 4u) % 4u] & ~mask) | (~v & mask);
        const uint32_t res = (pre & ~mask) | (v & mask);
        const unsigned ccr = (ccr_in & 0x10u) | ((v & msb) ? 0x08u : 0u) | (((v & mask) == 0u) ? 0x04u : 0u);
        const uint16_t op = (uint16_t)((wsz ? 0x3008u : 0x2008u) | ((unsigned)dn << 9) | (unsigned)an);
        const std::string what = "case " + std::to_string(k) + " MOVE" + (wsz ? ".W" : ".L") + " A" + std::to_string(an) + "(=" + hx(v)
            + "),D" + std::to_string(dn) + " (D" + std::to_string(dn) + " before " + hx(pre) + ") CCR " + hx(ccr_in) + " [op " + hx(op) + "]";
        set_a(an, v);
        set_d(dn, pre);
        set_ccr(ccr_in);
        w(op);
        save_sr(ccr, what);
        save_d(dn, res, what);
        save_a(an, v, what + " (source intact)");
    }
}

// 2) MOVEA <ea>,Ad. Expectations (PRM 4 MOVEA, PRM 2.2): Ad <- operand (.W sign-extended to 32 bits); CCR intact; base
//    (An)+ +2/+4, -(An) -2/-4, otherwise intact; indexes intact.
static const char* const kForm[16] = {"Dx", "Ab", "(Ab)", "(Ab)+", "-(Ab)", "$124(Ab)", "-$136(Ab)", "$22(Ab,Dx.W)",
                                      "-$30(Ab,Am.L)", "abs.W+", "abs.W-", "abs.L", "d16(PC)", "-6(PC,Dx.W)",
                                      "$10(PC,Am.L)", "#imm"};
static void movea_case(unsigned k, int f, int sz, uint32_t val, unsigned ccr_in, int d) {
    const bool wsz = sz == 2;
    const uint32_t v = wsz ? (val & 0xFFFFu) : val;
    const uint32_t expv = wsz ? (uint32_t)(int32_t)(int16_t)(uint16_t)v : v;
    int fr[7]; unsigned nfr = 0;
    for (int r = 0; r < 7; r++) if (r != d) fr[nfr++] = r;   // A0..A6 other than the destination
    const int b = fr[(k * 3u) % nfr];                        // base register / source An
    const int m = fr[(k * 3u + 1u) % nfr];                   // index An, other than b and the destination
    const int x = (int)((k * 5u + 1u) % 8u);                 // index / source Dn
    const uint16_t opb = (uint16_t)((wsz ? 0x3040u : 0x2040u) | ((unsigned)d << 9));
    const std::string what = "case " + std::to_string(k) + " MOVEA" + (wsz ? ".W " : ".L ") + kForm[f] + ",A" + std::to_string(d)
        + " value " + hx(v) + " CCR " + hx(ccr_in) + " [Ab=A" + std::to_string(b) + " Am=A" + std::to_string(m) + " Dx=D"
        + std::to_string(x) + "]";
    set_a(d, expv ^ 0x96C3A55Au);   // destination preloaded with a different value
    bool sb = false, sx = false, sm = false, mem_op = true;
    uint32_t a = 0, bv = 0, xv = 0, mv = 0;
    switch (f) {
    case 0:   // Dx (PRM 2.2 mode 000)
        xv = wsz ? (0xA53C0000u | v) : v; set_d(x, xv); sx = true; mem_op = false;
        set_ccr(ccr_in); w((uint16_t)(opb | (0u << 3) | (unsigned)x)); break;
    case 1:   // Ab (mode 001)
        bv = wsz ? (0x3C5A0000u | v) : v; set_a(b, bv); sb = true; mem_op = false;
        set_ccr(ccr_in); w((uint16_t)(opb | (1u << 3) | (unsigned)b)); break;
    case 2:   // (Ab)
        a = opnd_alloc(data_ptr, DATA_HI, sz, false, v); bv = a; set_a(b, a); sb = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (2u << 3) | (unsigned)b)); break;
    case 3:   // (Ab)+: Ab += 2/4 after
        a = opnd_alloc(data_ptr, DATA_HI, sz, false, v); set_a(b, a); bv = a + (uint32_t)sz; sb = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (3u << 3) | (unsigned)b)); break;
    case 4:   // -(Ab): Ab -= 2/4 before
        a = opnd_alloc(data_ptr, DATA_HI, sz, false, v); set_a(b, a + (uint32_t)sz); bv = a; sb = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (4u << 3) | (unsigned)b)); break;
    case 5:   // $124(Ab)
        a = opnd_alloc(data_ptr, DATA_HI, sz, false, v); bv = a - 0x124u; set_a(b, bv); sb = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (5u << 3) | (unsigned)b)); w(0x0124); break;
    case 6:   // -$136(Ab): extension $FECA
        a = opnd_alloc(data_ptr, DATA_HI, sz, false, v); bv = a + 0x136u; set_a(b, bv); sb = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (5u << 3) | (unsigned)b)); w(0xFECA); break;
    case 7:   // $22(Ab,Dx.W), Dx = $7A5BFFF0 -> index -16: EA = Ab - 16 + $22
        a = opnd_alloc(data_ptr, DATA_HI, sz, false, v); xv = 0x7A5BFFF0u; bv = a - 0x12u;
        set_a(b, bv); set_d(x, xv); sb = true; sx = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (6u << 3) | (unsigned)b)); w((uint16_t)(((unsigned)x << 12) | 0x22u)); break;
    case 8:   // -$30(Ab,Am.L), Am = $00010040: EA = Ab + $10040 - $30
        a = opnd_alloc(data_ptr, DATA_HI, sz, false, v); mv = 0x00010040u; bv = a - 0x10010u;
        set_a(b, bv); set_a(m, mv); sb = true; sm = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (6u << 3) | (unsigned)b)); w((uint16_t)(0x8000u | ((unsigned)m << 12) | 0x0800u | 0xD0u)); break;
    case 9:   // positive abs.W (mode 111/000)
        a = opnd_alloc(loww_ptr, LOWW_HI, sz, false, v);
        set_ccr(ccr_in); w((uint16_t)(opb | (7u << 3) | 0u)); w((uint16_t)a); break;
    case 10:  // negative abs.W: $Cxxx sign-extended -> $FFFFCxxx, on the 24-bit bus $FFCxxx
        a = opnd_alloc(higw_ptr, HIGW_HI, sz, false, v);
        set_ccr(ccr_in); w((uint16_t)(opb | (7u << 3) | 0u)); w((uint16_t)(a & 0xFFFFu)); break;
    case 11:  // abs.L (mode 111/001)
        a = opnd_alloc(data_ptr, DATA_HI, sz, false, v);
        set_ccr(ccr_in); w((uint16_t)(opb | (7u << 3) | 1u)); l32(a); break;
    case 12: {  // d16(PC) (mode 111/010): EA = address of the extension word + d16 (negative here)
        a = pcd_alloc(sz, v);
        set_ccr(ccr_in); w((uint16_t)(opb | (7u << 3) | 2u));
        const uint32_t e = here();
        const int32_t dsp = (int32_t)(a - e);
        if (dsp < -32768 || dsp > 32767) build_ok = false;
        w((uint16_t)dsp); break; }
    case 13: {  // -6(PC,Dx.W) (mode 111/011): EA = extension + Dx.W + d8; Dx.W = a - e + 6, high bits $5A5A
        a = pcd_alloc(sz, v);
        const uint32_t e = here() + 6u + 4u + 2u;   // after MOVE.L #,Dx (3 words), MOVE #,CCR (2) and the opcode (1)
        const int32_t ix = (int32_t)(a - e) + 6;
        if (ix < -32768 || ix > 32767) build_ok = false;
        xv = 0x5A5A0000u | ((uint32_t)ix & 0xFFFFu); set_d(x, xv); sx = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (7u << 3) | 3u));
        if (here() != e) build_ok = false;
        w((uint16_t)(((unsigned)x << 12) | 0xFAu)); break; }
    case 14: {  // $10(PC,Am.L): Am = a - e - $10
        a = pcd_alloc(sz, v);
        const uint32_t e = here() + 6u + 4u + 2u;   // after MOVEA.L #,Am (3 words), MOVE #,CCR (2) and the opcode (1)
        mv = a - e - 0x10u; set_a(m, mv); sm = true;
        set_ccr(ccr_in); w((uint16_t)(opb | (7u << 3) | 3u));
        if (here() != e) build_ok = false;
        w((uint16_t)(0x8000u | ((unsigned)m << 12) | 0x0800u | 0x10u)); break; }
    default:  // #imm (mode 111/100)
        mem_op = false;
        set_ccr(ccr_in); w((uint16_t)(opb | (7u << 3) | 4u));
        if (wsz) w((uint16_t)v); else l32(v);
        break;
    }
    save_sr(ccr_in, what + " (CCR intact)");
    save_a(d, expv, what);
    if (sb) save_a(b, bv, what + " (base/source register)");
    if (sx) save_d(x, xv, what + " (index/source register intact)");
    if (sm) save_a(m, mv, what + " (index register intact)");
    if (mem_op) tgts.push_back({a, sz, false, what});
}

static void build() {
    emit_head();
    unsigned k = 0;
    part_move_an_dn(k);
    static const uint32_t valW[4] = {0x8001u, 0x7FFFu, 0x0000u, 0xFFFFu};
    static const uint32_t valL[3] = {0xCAFEBABEu, 0x00000000u, 0x7FFFFFFFu};
    for (int f = 0; f < 16; f++)
    for (int si = 0; si < 2; si++)
    for (int j = 0; j < 2; j++, k++) {
        const int sz = si == 0 ? 2 : 4;
        const uint32_t val = sz == 2 ? valW[(unsigned)(f * 2 + j) % 4u] : valL[(unsigned)(f * 2 + j) % 3u];
        movea_case(k, f, sz, val, kCcr6[k % 6u], (int)(k % 8u));
    }
    // 3) MOVEA.L #imm,A4 and MOVEA.L abs.L,A4 with CCR $1F and $00
    movea_case(k, 15, 4, 0x12345678u, 0x1Fu, 4); k++;
    movea_case(k, 15, 4, 0x80000000u, 0x00u, 4); k++;
    movea_case(k, 11, 4, 0x00000000u, 0x1Fu, 4); k++;
    movea_case(k, 11, 4, 0xFFFFFFFFu, 0x00u, 4); k++;
    if (pcd_used != PCD_SLOTS) build_ok = false;
    ncases = k;
    finish_prog();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    if (!build_ok) { printf("FAIL: program construction (areas full or displacements out of range) expected=1 read=0\nRESULT: FAIL\n"); return 1; }
    printf("tb_m68k_movean: %zu cases, %zu program words, %zu expected bytes\n", ncases, code.size(), exp_wr.size());
    static const int waits[2] = {0, 3};
    for (int k = 0; k < 2; k++) {
        const std::string tag = "[DTACK+" + std::to_string(waits[k]) + "] ";
        if (run(waits[k], tag)) check_results(tag);
    }
    printf("checks run: %ld, failed: %ld\n", n_checks, n_fail);
    printf(n_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return n_fail ? 1 : 0;
}
