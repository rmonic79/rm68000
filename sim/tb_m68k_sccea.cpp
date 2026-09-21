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

// tb_m68k_sccea.cpp - Scc <ea> with destination in memory (PRM 4 Scc: 0101 cccc 11 mmm rrr, byte operand;
// condition true -> $FF, false -> $00; condition codes not touched).
//
// What it tests (800 cases):
//  - (An): all 16 conditions x all 32 CCR values (512 cases, exhaustive), An rotating A0..A7, byte at
//    even and odd address alternately;
//  - (An)+, -(An), d16(An) with positive ($2468) and negative ($9ABC) displacement, d8(An,Dn.W) (index $5A5AFF80: negative
//    low word, non-zero high bits that .W must ignore; d8 = $86, negative), d8(An,Am.L) (index $FFFE8010: the
//    sign-extended low word alone would give another address; d8 = $3C), positive abs.W (area $000400..) and negative
//    ($C000.. extended to $FFFFC000.., bus $FFC000..), abs.L: for each one the 16 conditions with a CCR that makes it true and one
//    that makes it false (T: two true, F: two false), CCRs chosen in rotation among the 32 (X included): 9 x 32 = 288 cases;
//  - -(A7) and (A7)+ on bytes change A7 by 2 (PRM 2.2, predecrement and postincrement modes: with the stack pointer and a
//    byte operand the address changes by two to stay word-aligned);
//  - after every Scc: whole SR saved with MOVE SR,abs.W (expected $2700 | input CCR: Scc does not touch the flags), An saved
//    (updated by (An)+ and -(An), intact in the other modes), index register saved (intact), expected destination
//    byte; the neighbouring bytes and words are preloaded with sentinels and any write to them fails.
// Expectations: table of the conditional tests of PRM 3 (Conditional Tests): T 1, F 0, HI !C&!Z, LS C|Z, CC !C, CS C, NE !Z,
// EQ Z, VC !V, VS V, PL !N, MI N, GE N=V, LT N!=V, GT !Z&(N=V), LE Z|(N!=V); effective address from the modes of PRM 2.2.
// Bus: on the operand one read and then one write (MC68000UM, instruction timing: Scc memory 8(1/1)+, where "+" is
// the EA calculation including the operand fetch), only UDS at an even address and only LDS at an odd one (UM 5.1),
// FC 101 on data and 110 on fetches, no write outside the expected bytes, no read of uninitialized
// memory. The program runs twice: DTACK immediate and DTACK delayed by 3 clocks; same expectations.
// What it does NOT test: Scc Dn (register), DBcc, the number of clocks, user mode.

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
// Testbench: operands in memory, addressing modes, conditions and cases
// =====================================================================================================================
static constexpr uint32_t DATA_LO = 0x020000, DATA_HI = 0x030000;   // operands reached through a register and abs.L
static constexpr uint32_t LOWW_LO = 0x000400, LOWW_HI = 0x001000;   // positive abs.W operands
static constexpr uint32_t HIGW_LO = 0xFFC000, HIGW_HI = 0xFFF000;   // negative abs.W operands ($C000..$EFFF -> $FFC000..$FFEFFF)
static uint32_t data_ptr = DATA_LO, loww_ptr = LOWW_LO, higw_ptr = HIGW_LO;
// 16-byte slot: all sentinels different, operand at base+4 (+1 if odd), big endian
static uint32_t opnd_alloc(uint32_t& ptr, uint32_t hi, int size, bool odd, uint32_t val) {
    const uint32_t base = ptr; ptr += 16; if (ptr > hi) build_ok = false;
    for (uint32_t i = 0; i < 16; i++) init_b(base + i, (uint8_t)(0xA1u + 0x0Bu * i));
    const uint32_t a = base + 4u + (odd ? 1u : 0u);
    for (int i = 0; i < size; i++) init_b(a + (uint32_t)i, (uint8_t)(val >> (8 * (size - 1 - i))));
    return a;
}

// effective address forms (PRM 2.2)
enum { F_IND = 0, F_POST, F_PRE, F_D16P, F_D16N, F_IDXW, F_IDXL, F_ABSWP, F_ABSWN, F_ABSL, F_NUM };
static const char* const kForm[F_NUM] = {"(An)", "(An)+", "-(An)", "d16(An) d16=+$2468", "d16(An) d16=-$6544", "d8(An,Dn.W) d8=-$7A",
                                         "d8(An,Am.L) d8=+$3C", "abs.W positive", "abs.W negative", "abs.L"};
static constexpr uint32_t IDXW_VAL = 0x5A5AFF80u;   // index Dn.W: low word $FF80 = -128, high bits to be ignored
static constexpr uint32_t IDXL_VAL = 0xFFFE8010u;   // index Am.L: -$17FF0 (with .W it would be $FFFF8010)

struct Ea {
    uint32_t a = 0;                 // operand address (24 bits)
    unsigned mode = 0, reg = 0;     // mode/register fields of the opcode
    std::vector<uint16_t> ext;      // extension words after the opcode
    bool has_an = false; int an = 0; uint32_t an_final = 0;   // An used and its expected value after the instruction
    int idx_d = -1, idx_a = -1;     // index register (data or address), to be rechecked intact
};
// Allocates the operand, emits the instructions that prepare the registers (before set_ccr) and returns the EA fields.
static Ea make_ea(int form, int n, int size, bool odd, uint32_t val) {
    Ea e;
    const bool sp_byte = (n == 7 && size == 1);   // PRM 2.2: (A7)+ and -(A7) on bytes change A7 by 2
    switch (form) {
    case F_IND:
        e.a = opnd_alloc(data_ptr, DATA_HI, size, odd, val);
        e.mode = 2; e.reg = (unsigned)n; e.has_an = true; e.an = n; e.an_final = e.a; set_a(n, e.a);
        break;
    case F_POST:
        e.a = opnd_alloc(data_ptr, DATA_HI, size, odd && !sp_byte, val);
        e.mode = 3; e.reg = (unsigned)n; e.has_an = true; e.an = n; set_a(n, e.a);
        e.an_final = e.a + (sp_byte ? 2u : (uint32_t)size);
        break;
    case F_PRE:
        e.a = opnd_alloc(data_ptr, DATA_HI, size, odd && !sp_byte, val);
        e.mode = 4; e.reg = (unsigned)n; e.has_an = true; e.an = n; e.an_final = e.a;
        set_a(n, e.a + (sp_byte ? 2u : (uint32_t)size));
        break;
    case F_D16P:
    case F_D16N: {
        const uint16_t d = form == F_D16P ? (uint16_t)0x2468 : (uint16_t)0x9ABC;   // $9ABC = -$6544
        e.a = opnd_alloc(data_ptr, DATA_HI, size, odd, val);
        const uint32_t an = e.a - (uint32_t)(int32_t)(int16_t)d;
        e.mode = 5; e.reg = (unsigned)n; e.has_an = true; e.an = n; e.an_final = an; e.ext.push_back(d); set_a(n, an);
        break;
    }
    case F_IDXW: {
        const int m = (n + 1) & 7;
        const uint8_t d8 = 0x86;                          // -122
        e.a = opnd_alloc(data_ptr, DATA_HI, size, odd, val);
        const uint32_t an = e.a - (uint32_t)(int32_t)(int16_t)(uint16_t)(IDXW_VAL & 0xFFFFu) - (uint32_t)(int32_t)(int8_t)d8;
        e.mode = 6; e.reg = (unsigned)n; e.has_an = true; e.an = n; e.an_final = an; e.idx_d = m;
        e.ext.push_back((uint16_t)(((unsigned)m << 12) | d8));          // D/A=0 (Dn), W/L=0 (.W)
        set_a(n, an); set_d(m, IDXW_VAL);
        break;
    }
    case F_IDXL: {
        const int m = (n + 3) & 7;
        const uint8_t d8 = 0x3C;                          // +60
        e.a = opnd_alloc(data_ptr, DATA_HI, size, odd, val);
        const uint32_t an = e.a - IDXL_VAL - (uint32_t)(int32_t)(int8_t)d8;
        e.mode = 6; e.reg = (unsigned)n; e.has_an = true; e.an = n; e.an_final = an; e.idx_a = m;
        e.ext.push_back((uint16_t)(0x8000u | ((unsigned)m << 12) | 0x0800u | d8));   // D/A=1 (An), W/L=1 (.L)
        set_a(n, an); set_a(m, IDXL_VAL);
        break;
    }
    case F_ABSWP:
        e.a = opnd_alloc(loww_ptr, LOWW_HI, size, odd, val);
        e.mode = 7; e.reg = 0; e.ext.push_back((uint16_t)(e.a & 0xFFFFu));
        break;
    case F_ABSWN:
        e.a = opnd_alloc(higw_ptr, HIGW_HI, size, odd, val);
        e.mode = 7; e.reg = 0; e.ext.push_back((uint16_t)(e.a & 0xFFFFu));   // $C000..$EFFF: sign-extended to $FFFFC000..
        break;
    default:   // F_ABSL
        e.a = opnd_alloc(data_ptr, DATA_HI, size, odd, val);
        e.mode = 7; e.reg = 1; e.ext.push_back((uint16_t)(e.a >> 16)); e.ext.push_back((uint16_t)(e.a & 0xFFFFu));
        break;
    }
    return e;
}

// PRM 3, conditional tests table (C = bit 0, V = bit 1, Z = bit 2, N = bit 3 of the CCR; X does not take part)
static bool cond_true(unsigned cc, unsigned ccr) {
    const bool c = (ccr & 1u) != 0, v = (ccr & 2u) != 0, z = (ccr & 4u) != 0, n = (ccr & 8u) != 0;
    switch (cc) {
    case 0: return true;               // T
    case 1: return false;              // F
    case 2: return !c && !z;           // HI
    case 3: return c || z;             // LS
    case 4: return !c;                 // CC
    case 5: return c;                  // CS
    case 6: return !z;                 // NE
    case 7: return z;                  // EQ
    case 8: return !v;                 // VC
    case 9: return v;                  // VS
    case 10: return !n;                // PL
    case 11: return n;                 // MI
    case 12: return n == v;            // GE
    case 13: return n != v;            // LT
    case 14: return !z && (n == v);    // GT
    default: return z || (n != v);     // LE
    }
}
static const char* const kCc[16] = {"T", "F", "HI", "LS", "CC", "CS", "NE", "EQ", "VC", "VS", "PL", "MI", "GE", "LT", "GT", "LE"};

static void scc_case(unsigned k, unsigned cc, unsigned ccr, int form, int n, bool odd) {
    const Ea e = make_ea(form, n, 1, odd, 0x5Cu);   // starting byte $5C: neither $00 nor $FF
    const bool cond = cond_true(cc, ccr);
    const uint16_t op = (uint16_t)(0x50C0u | (cc << 8) | (e.mode << 3) | e.reg);
    const std::string what = "case " + std::to_string(k) + " S" + kCc[cc] + " " + kForm[form]
        + (e.has_an ? " A" + std::to_string(n) : std::string()) + " at " + hx(e.a) + " CCR " + hx(ccr) + " [op " + hx(op) + "]";
    set_ccr(ccr);
    w(op);
    for (uint16_t x : e.ext) w(x);
    save_sr(ccr, what);                                        // PRM 4 Scc: condition codes not touched
    if (e.has_an) save_a(e.an, e.an_final, what);
    if (e.idx_d >= 0) save_d(e.idx_d, IDXW_VAL, what + " (index intact)");
    if (e.idx_a >= 0) save_a(e.idx_a, IDXL_VAL, what + " (index intact)");
    exp_b(e.a, cond ? (uint8_t)0xFF : (uint8_t)0x00, what + " destination byte");
    tgts.push_back(Tgt{e.a, 1, true, what});
}

static void build() {
    unsigned k = 0;
    // (An): 16 conditions x 32 CCRs, exhaustive
    for (unsigned cc = 0; cc < 16; cc++)
        for (unsigned ccr = 0; ccr < 32; ccr++, k++)
            scc_case(k, cc, ccr, F_IND, (int)(k % 8u), (k & 1u) != 0);
    // the other modes: for each condition a CCR that makes it true and one that makes it false (T always true, F always false)
    for (int form = F_POST; form < F_NUM; form++)
        for (unsigned cc = 0; cc < 16; cc++)
            for (unsigned j = 0; j < 2; j++, k++) {
                const bool want = (cc == 0) ? true : (cc == 1) ? false : (j == 0);
                unsigned ccr = ((unsigned)form * 11u + cc * 5u + j * 17u) % 32u;
                while (cond_true(cc, ccr) != want) ccr = (ccr + 1u) % 32u;
                scc_case(k, cc, ccr, form, (int)((cc + (unsigned)form + j) % 8u), (k & 1u) != 0);
            }
    ncases = k;
    finish_prog();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    if (!build_ok) { printf("FAIL: program construction (results or data area full) expected=1 read=0\nRESULT: FAIL\n"); return 1; }
    printf("tb_m68k_sccea: %zu cases, %zu program words, %zu expected bytes\n", ncases, code.size(), exp_wr.size());
    static const int waits[2] = {0, 3};
    for (int k = 0; k < 2; k++) {
        const std::string tag = "[DTACK+" + std::to_string(waits[k]) + "] ";
        if (run(waits[k], tag)) check_results(tag);
    }
    printf("checks run: %ld, failed: %ld\n", n_checks, n_fail);
    printf(n_fail ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    return n_fail ? 1 : 0;
}
