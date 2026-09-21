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

// tb_m68k_muldivmem.cpp - MULU.W, MULS.W, DIVU.W, DIVS.W with the source word read from memory or from the immediate.
//
// What it tests (52 cases = 4 instructions x 13 modes, each run twice: DTACK with zero wait states and delayed by 3 clocks):
//  - source modes: (An), (An)+, -(An), d16(An) with positive and negative displacement, d8(An,D7.W) with a negative word
//    index and the high word of the index other than zero, d8(An,A6.L) with a long index (an index taken as a word leads
//    outside the operand), abs.W, negative abs.W (sign-extended: FF8xxx), abs.L above 8000 (reachable only with 32 bits),
//    d16(PC), d8(PC,D7.W), #imm (PRM 2.2);
//  - for each instruction, pairs of edge operands (FFFF*FFFF, -32768*-32768, -32768*32767, zero products; DIVU with
//    maximum remainder and overflow; DIVS with positive and negative overflow and negative remainder), different input CCRs;
//  - result in the data register with the high word preloaded, all five flags, An after (An)+ (+2) and -(An) (-2) and
//    unchanged in the other modes;
//  - bus: exactly one read at the operand address, as a word (UDS and LDS), FC = 5 (supervisor data) for the data
//    modes and FC = 6 (supervisor program) for the PC-relative modes (MC68000UM, function code; PRM 2.2 PC-relative
//    modes: "the reference is classified as a program reference"); no write outside the results area.
// What it does NOT test: division by zero (tb_m68k_div0.cpp), the complete register edge cases (tb_m68k_muldiv.cpp), the clocks.
//
// Expectations: model md_model() from the Motorola semantics (PRM 4 MULU/MULS/DIVU/DIVS, flags PRM 3), compared before the
// run with the hand-computed values of the PAIR_EXP table (self-check).
// Clock: this testbench is compiled without SINGLE_CEN and drives cen and cen2 (both high on every clock).

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

// ======================================================================================================================
// Testbench infrastructure
//  - byte-wide memory: 000000-00FFFF and FF0000-FFFFFF (negative abs.W); even byte = D15-D8 = UDS, odd byte = D7-D0 =
//    LDS (MC68000UM 5.1.1-5.1.2): a write changes only the bytes of the enabled lanes;
//  - every byte the program may write is declared (allow): any other write is a FAIL;
//  - unused memory = ILLEGAL (0x4AFC); every vector 2..255 points to its own BRA.S * (UNEXP_BASE + 4*v): the fetch
//    of that address is an unexpected exception, reported with its vector number;
//  - log of all bus cycles (address, FC, R/W, UDS, LDS, data);
//  - DTACK asserted g_wait clocks after AS (0 or 3);
//  - end: the final BRA.S * fetched 3 times (the first fetch may be the prefetch during the previous
//    instruction; the second and the third belong to the BRA itself, so all the previous instructions have completed).
// ======================================================================================================================
static Vm68k_seq* dut = nullptr;
static uint64_t t = 0;
static uint64_t nclk = 0;
double sc_time_stamp() { return (double)t; }

static int n_checks = 0;
static int n_fail = 0;
static std::string g_ctx;

static std::string hx(uint64_t v) { char b[24]; snprintf(b, sizeof b, "%llX", (unsigned long long)v); return std::string(b); }
static void check_eq(const std::string& what, uint64_t exp, uint64_t got) {
    n_checks++;
    if (exp != got) {
        n_fail++;
        printf("FAIL: %s: %s expected=0x%s read=0x%s\n", g_ctx.c_str(), what.c_str(), hx(exp).c_str(), hx(got).c_str());
    }
}
static void bank_error(const std::string& what) { n_fail++; printf("FAIL: testbench error: %s expected=0 read=1\n", what.c_str()); }

static constexpr uint32_t LO_SIZE    = 0x10000;   // 000000-00FFFF
static constexpr uint32_t HI_BASE    = 0xFF0000;  // FF0000-FFFFFF
static constexpr uint32_t PROG_BASE  = 0x4000;
static constexpr uint32_t PROG_LIMIT = 0x7000;    // the code sits below; above: PC-relative data and unexpected handlers
static constexpr uint32_t UNEXP_BASE = 0x7C00;    // unexpected vector v -> BRA.S * at UNEXP_BASE + 4*v
static constexpr uint32_t RES_BASE   = 0x1000;    // results area (addressable with abs.W)
static std::vector<uint8_t> mem, wr_ok;

static int mem_idx(uint32_t a) {
    a &= 0xFFFFFF;
    if (a < LO_SIZE) return (int)a;
    if (a >= HI_BASE) return (int)(LO_SIZE + (a - HI_BASE));
    return -1;
}
static void put8(uint32_t a, uint8_t v) { int i = mem_idx(a); if (i < 0) { bank_error("put8 outside memory 0x" + hx(a)); return; } mem[i] = v; }
static uint8_t get8(uint32_t a) { int i = mem_idx(a); return i < 0 ? (uint8_t)0xFF : mem[i]; }
static void put16(uint32_t a, uint16_t v) { put8(a, (uint8_t)(v >> 8)); put8(a + 1, (uint8_t)v); }
static void put32(uint32_t a, uint32_t v) { put16(a, (uint16_t)(v >> 16)); put16(a + 2, (uint16_t)v); }
static uint16_t get16(uint32_t a) { return (uint16_t)((get8(a) << 8) | get8(a + 1)); }
static uint32_t get32(uint32_t a) { return ((uint32_t)get16(a) << 16) | get16(a + 2); }
static void allow(uint32_t a, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) { int k = mem_idx(a + i); if (k < 0) bank_error("allow outside memory 0x" + hx(a + i)); else wr_ok[k] = 1; }
}
static void mem_init() {
    mem.assign(2 * LO_SIZE, 0); wr_ok.assign(2 * LO_SIZE, 0);
    for (uint32_t a = 0; a < LO_SIZE; a += 2) { put16(a, 0x4AFC); put16(HI_BASE + a, 0x4AFC); }
    put32(0, 0); put32(4, PROG_BASE);   // reset vectors (the shim emulates SSP = 0 and PC = pc_init)
    for (uint32_t v = 2; v < 256; v++) { put32(4 * v, UNEXP_BASE + 4 * v); put16(UNEXP_BASE + 4 * v, 0x60FE); put16(UNEXP_BASE + 4 * v + 2, 0x60FE); }
}

// --- minimal assembler (PRM 4 encodings; PRM 2.2 modes) ---
static uint32_t apc = 0;
static void org(uint32_t a) { apc = a; }
static void ew(uint16_t v) { put16(apc, v); apc += 2; }
static void el(uint32_t v) { ew((uint16_t)(v >> 16)); ew((uint16_t)v); }
static constexpr int DN = 0, AN = 1, IND = 2, POST = 3, PRE = 4, D16 = 5, IDX = 6, X7 = 7;
static constexpr int ABSW = 0, ABSL = 1, PCD16 = 2, PCIDX = 3, IMM = 4;
static uint16_t ea(int m, int r) { return (uint16_t)((m << 3) | r); }
// MOVE (PRM 4 MOVE): 00 ss RRR MMM mmm rrr, ss = 01 byte, 11 word, 10 long
static uint16_t op_move(int sz, int dm, int dr, int sm, int sr) { int s = sz == 1 ? 1 : (sz == 2 ? 3 : 2); return (uint16_t)((s << 12) | (dr << 9) | (dm << 6) | (sm << 3) | sr); }
static void chk_absw(uint32_t a) { if (a >= 0x8000) bank_error("non-positive abs.W address 0x" + hx(a)); }
static void move_l_imm_d(uint32_t v, int d) { ew(op_move(4, DN, d, X7, IMM)); el(v); }       // MOVE.L #imm,Dn
static void movea_l_imm(uint32_t v, int a) { ew(op_move(4, AN, a, X7, IMM)); el(v); }        // MOVEA.L #imm,An
static void move_ccr(uint8_t c) { ew(0x44FC); ew(c); }                                          // MOVE #imm,CCR
static void move_sr(uint16_t s) { ew(0x46FC); ew(s); }                                          // MOVE #imm,SR
static void save_sr(uint32_t a) { chk_absw(a); ew(0x40F8); ew((uint16_t)a); }                  // MOVE SR,abs.W
static void save_d(int d, uint32_t a) { chk_absw(a); ew(op_move(4, X7, ABSW, DN, d)); ew((uint16_t)a); }   // MOVE.L Dn,abs.W
static void save_a(int n, uint32_t a) { chk_absw(a); ew(op_move(4, X7, ABSW, AN, n)); ew((uint16_t)a); }   // MOVE.L An,abs.W
static void bra_self() { ew(0x60FE); }                                                          // BRA.S *
// brief extension word of the index (PRM 2.2, d8(An,Xn) and d8(PC,Xn)): bit 15 D/A, 14-12 register, 11 W/L, 7-0 d8
static uint16_t ext_idx(int is_a, int reg, int is_long, int d8) { return (uint16_t)((is_a << 15) | (reg << 12) | (is_long << 11) | (d8 & 0xFF)); }

// --- expectations ---
struct Expect { uint32_t addr; int size; uint32_t val; uint32_t mask; std::string what; };
static std::vector<Expect> exps;
static uint32_t rd_sz(uint32_t a, int size) { return size == 1 ? get8(a) : size == 2 ? get16(a) : get32(a); }
static void wr_sz(uint32_t a, int size, uint32_t v) { if (size == 1) put8(a, (uint8_t)v); else if (size == 2) put16(a, (uint16_t)v); else put32(a, v); }
static void expect(uint32_t a, int size, uint32_t v, const std::string& what, uint32_t mask = 0xFFFFFFFFu) { exps.push_back({a, size, v, mask, what}); }
// result cell: writable and preloaded with the complement of the expected value (a missed write cannot pass)
static void result(uint32_t a, int size, uint32_t v, const std::string& what, uint32_t mask = 0xFFFFFFFFu) { allow(a, (uint32_t)size); wr_sz(a, size, ~v); expect(a, size, v, what, mask); }

// --- bus ---
struct Cyc { uint32_t addr; uint8_t fc; bool rw, uds, lds; uint16_t data; uint64_t at; };
static std::vector<Cyc> blog;
static bool cyc_on = false;
static uint32_t cyc_len = 0;
static Cyc cyc{};
static uint32_t g_wait = 0, g_end = 0;
static int g_end_seen = 0, g_unexp = -1, g_bad_wr = 0, g_iack = 0, g_out = 0;

static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; nclk++; }
static void wr_byte(uint32_t a, uint8_t v) {
    int i = mem_idx(a);
    if (i < 0) return;
    if (!wr_ok[i]) { g_bad_wr++; if (g_bad_wr <= 10) printf("FAIL: %s: write outside the expected areas at 0x%06X expected=none read=0x%02X\n", g_ctx.c_str(), (unsigned)a, (unsigned)v); }
    mem[i] = v;
}
static void commit() {
    blog.push_back(cyc);
    if (cyc.fc == 7) return;
    if (mem_idx(cyc.addr) < 0) { g_out++; if (g_out <= 10) printf("FAIL: %s: bus cycle outside memory at 0x%06X expected=none read=1\n", g_ctx.c_str(), (unsigned)cyc.addr); return; }
    if (!cyc.rw) {
        if (!cyc.uds && !cyc.lds) { g_bad_wr++; printf("FAIL: %s: write without data strobe at 0x%06X expected=UDS/LDS read=none\n", g_ctx.c_str(), (unsigned)cyc.addr); }
        if (cyc.uds) wr_byte(cyc.addr, (uint8_t)(cyc.data >> 8));
        if (cyc.lds) wr_byte(cyc.addr + 1, (uint8_t)cyc.data);
    } else if (cyc.fc == 2 || cyc.fc == 6) {
        if (cyc.addr == g_end) g_end_seen++;
        if (g_unexp < 0 && cyc.addr >= UNEXP_BASE && cyc.addr < UNEXP_BASE + 0x400) g_unexp = (int)((cyc.addr - UNEXP_BASE) >> 2);
    }
}
static void bus_step() {
    if (!dut->as_n) {
        if (!cyc_on) {
            cyc_on = true; cyc_len = 0;
            cyc.addr = ((uint32_t)dut->addr << 1) & 0xFFFFFF; cyc.fc = (uint8_t)dut->fc; cyc.rw = dut->rw != 0;
            cyc.uds = false; cyc.lds = false; cyc.data = 0; cyc.at = nclk;
        } else cyc_len++;
        if (!dut->rw) cyc.rw = false;
        if (cyc.fc == 7) {   // no interrupts in this testbench: an IACK is an error (autovector response so as not to stall)
            if (cyc_len == 0) g_iack++;
            dut->vpa_n = 0; dut->dtack_n = 1;
            return;
        }
        if (cyc.rw) {
            uint16_t d = get16(cyc.addr);
            dut->data_in = d; cyc.data = d;
            if (!dut->uds_n) cyc.uds = true;
            if (!dut->lds_n) cyc.lds = true;
        } else if (!dut->uds_n || !dut->lds_n) {
            cyc.uds = !dut->uds_n; cyc.lds = !dut->lds_n; cyc.data = (uint16_t)dut->data_out;
        }
        dut->dtack_n = (cyc_len >= g_wait) ? 0 : 1;
    } else {
        if (cyc_on) { cyc_on = false; commit(); }
        dut->dtack_n = 1; dut->vpa_n = 1; dut->berr_n = 1;
    }
}
static void run_cpu(uint32_t wait, uint32_t end_addr, uint64_t max_clk) {
    g_wait = wait; g_end = end_addr; g_end_seen = 0; g_unexp = -1; g_bad_wr = 0; g_iack = 0; g_out = 0;
    blog.clear(); cyc_on = false;
    dut->rst_n = 0; dut->cen = 0; dut->cen2 = 0; dut->start = 0; dut->rst_vector = 0; dut->pc_init = PROG_BASE;
    dut->ipl = 0; dut->vpa_n = 1; dut->dtack_n = 1; dut->berr_n = 1; dut->halt_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    dut->data_in = 0; dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1; dut->cen = 1; dut->cen2 = 1;   // two phases driven by the testbench (Makefile without SINGLE_CEN): cen and cen2 high on every clock
    for (uint64_t i = 0; i < max_clk && g_end_seen < 3 && g_unexp < 0; i++) { bus_step(); tick(); }
    check_eq("end of program (BRA.S * at 0x" + hx(end_addr) + " fetched 3 times)", 1, g_end_seen >= 3 ? 1 : 0);
    check_eq("unexpected exception (vector number; 0x100 = none)", 0x100, g_unexp < 0 ? 0x100u : (uint64_t)g_unexp);
    check_eq("writes outside the expected areas", 0, (uint64_t)g_bad_wr);
    check_eq("bus cycles outside memory", 0, (uint64_t)g_out);
    check_eq("unexpected IACK cycles", 0, (uint64_t)g_iack);
    for (const Expect& e : exps) {
        uint32_t sm = e.size == 1 ? 0xFFu : e.size == 2 ? 0xFFFFu : 0xFFFFFFFFu;
        check_eq(e.what + " @0x" + hx(e.addr), e.val & e.mask & sm, rd_sz(e.addr, e.size) & e.mask & sm);
    }
}

// ======================================================================================================================
// MUL/DIV: PRM 4 encodings (MULU 1100 ddd 011 ea, MULS 1100 ddd 111 ea, DIVU 1000 ddd 011 ea, DIVS 1000 ddd 111 ea)
// ======================================================================================================================
enum { MULU = 0, MULS = 1, DIVU = 2, DIVS = 3 };
static const char* const OPN[4] = {"MULU.W", "MULS.W", "DIVU.W", "DIVS.W"};
static uint16_t md_op(int op, int dn, uint16_t eaf) { static const uint16_t base[4] = {0xC0C0, 0xC1C0, 0x80C0, 0x81C0}; return (uint16_t)(base[op] | (dn << 9) | eaf); }

struct MdOut { uint32_t d; uint8_t ccr; uint8_t mask; };
// Model (PRM 4 MULU/MULS/DIVU/DIVS, flags from the condition code table of PRM 3):
//  MULU: Dn = Dn[15:0] * src (unsigned, 32 bits); MULS: signed. X unchanged, N = bit 31, Z = (Dn == 0), V = C = 0.
//  DIVU: q = Dn / src, r = Dn % src (unsigned); if q > FFFF: overflow, V = 1, C = 0, Dn unchanged, N Z undefined
//        (not compared: mask 0x13 = X V C); otherwise Dn = r:q, N = q[15], Z = (q == 0), V = C = 0.
//  DIVS: signed, truncation toward zero, remainder with the sign of the dividend (PRM 4 DIVS); overflow if q is outside
//        -32768..32767.
static MdOut md_model(int op, uint32_t dst, uint16_t src, uint8_t ccr_in) {
    uint8_t x = (uint8_t)(ccr_in & 0x10);
    if (op == MULU) { uint32_t r = (dst & 0xFFFFu) * (uint32_t)src; return {r, (uint8_t)(x | ((r >> 31) ? 8 : 0) | (r == 0 ? 4 : 0)), 0x1F}; }
    if (op == MULS) { int32_t r = (int32_t)(int16_t)(uint16_t)(dst & 0xFFFFu) * (int32_t)(int16_t)src; uint32_t u = (uint32_t)r; return {u, (uint8_t)(x | ((u >> 31) ? 8 : 0) | (u == 0 ? 4 : 0)), 0x1F}; }
    if (src == 0) { bank_error("zero divisor in the model (division by zero is in tb_m68k_div0)"); return {dst, 0, 0}; }
    if (op == DIVU) {
        uint32_t q = dst / src, r = dst % src;
        if (q > 0xFFFFu) return {dst, (uint8_t)(x | 0x02), 0x13};
        return {(r << 16) | q, (uint8_t)(x | ((q & 0x8000u) ? 8 : 0) | (q == 0 ? 4 : 0)), 0x1F};
    }
    int64_t a = (int32_t)dst, b = (int16_t)src, q = a / b, r = a % b;
    if (q > 32767 || q < -32768) return {dst, (uint8_t)(x | 0x02), 0x13};
    return {((uint32_t)(uint16_t)r << 16) | (uint16_t)q, (uint8_t)(x | (q < 0 ? 8 : 0) | (q == 0 ? 4 : 0)), 0x1F};
}

// pairs (32-bit dividend or multiplicand, 16-bit source) per instruction, and result computed by hand with X = 0
static const uint32_t PAIRS[4][4][2] = {
    {{0x1234FFFF, 0xFFFF}, {0xFFFF0000, 0x1234}, {0x00008000, 0x8000}, {0x00000003, 0xFFFF}},   // MULU
    {{0x00008000, 0x8000}, {0x00008000, 0x7FFF}, {0xFFFF0005, 0xFFFD}, {0x0000FFFF, 0x0001}},   // MULS
    {{0x00010005, 0x0004}, {0xFFFEFFFF, 0xFFFF}, {0xFFFFFFFF, 0xFFFF}, {0x00000005, 0x0007}},   // DIVU
    {{0xFFFFFFF9, 0xFFFE}, {0x00008000, 0x0001}, {0xFFFF7FFF, 0x0001}, {0xFFFFFF9C, 0x0007}},   // DIVS
};
static const uint32_t PAIR_EXP[4][4][2] = {
    {{0xFFFE0001, 0x08}, {0x00000000, 0x04}, {0x40000000, 0x00}, {0x0002FFFD, 0x00}},   // FFFF*FFFF N; 0 Z; 8000*8000; 3*FFFF
    {{0x40000000, 0x00}, {0xC0008000, 0x08}, {0xFFFFFFF1, 0x08}, {0xFFFFFFFF, 0x08}},   // -32768^2; -32768*32767; 5*-3; -1*1
    {{0x00014001, 0x00}, {0xFFFEFFFF, 0x08}, {0xFFFFFFFF, 0x02}, {0x00050000, 0x04}},   // 65541/4; FFFF r FFFE; overflow; 0 r 5
    {{0xFFFF0003, 0x00}, {0x00008000, 0x02}, {0xFFFF7FFF, 0x02}, {0xFFFEFFF2, 0x08}},   // -7/-2 = 3 r -1; ovf +; ovf -; -14 r -2
};
static const uint8_t CCRS[4] = {0x1F, 0x00, 0x15, 0x0A};

enum { M_IND, M_POST, M_PRE, M_D16P, M_D16N, M_IDXW, M_IDXL, M_ABSW, M_ABSWN, M_ABSL, M_PCD16, M_PCIDX, M_IMM, NMODES };
static const char* const MODEN[NMODES] = {"(An)", "(An)+", "-(An)", "d16(An)+", "d16(An)-", "d8(An,D7.W)", "d8(An,A6.L)",
                                          "abs.W", "abs.W negative", "abs.L", "d16(PC)", "d8(PC,D7.W)", "#imm"};
static constexpr int NTEST = 4 * NMODES;
struct TestInfo { uint32_t opa; uint8_t fc; bool has_mem; std::string name; };
static std::vector<TestInfo> tinfo;

static uint32_t build() {
    mem_init(); exps.clear(); tinfo.clear();
    org(PROG_BASE);
    move_sr(0x2700);          // S=1, mask 7 (PRM 4 MOVE to SR)
    movea_l_imm(0x3F00, 7);   // SSP defined (no exception expected)
    for (int k = 0; k < NTEST; k++) {
        int mi = k / 4, op = k % 4, pi = (mi + op) % 4;
        uint32_t dst = PAIRS[op][pi][0];
        uint16_t src = (uint16_t)PAIRS[op][pi][1];
        uint8_t ccr = CCRS[(k / 3) % 4];
        int dd = k % 6, an = 1 + k % 3;
        uint32_t res = RES_BASE + 16 * (uint32_t)k;
        uint32_t opa = 0x3000 + 0x10 * (uint32_t)k;
        if (mi == M_ABSWN) opa = 0xFF8000 + 0x10 * (uint32_t)k;
        if (mi == M_ABSL) opa = 0x9000 + 0x10 * (uint32_t)k;
        if (mi == M_PCD16 || mi == M_PCIDX) opa = 0x7000 + 0x10 * (uint32_t)k;
        bool has_mem = (mi != M_IMM), uses_an = (mi <= M_IDXL);
        if (has_mem) put16(opa, src);
        uint32_t an_init = 0, an_final = 0;
        move_l_imm_d(dst, dd);
        switch (mi) {
            case M_IND:  an_init = opa; break;
            case M_POST: an_init = opa; break;
            case M_PRE:  an_init = opa + 2; break;
            case M_D16P: an_init = opa - 0x0120; break;
            case M_D16N: an_init = opa + 0x0200; break;
            case M_IDXW: an_init = opa + 0x40; break;
            case M_IDXL: an_init = opa + 6 - 0x10010; break;
            default: break;
        }
        an_final = (mi == M_POST) ? opa + 2 : (mi == M_PRE) ? opa : an_init;
        if (uses_an) movea_l_imm(an_init, an);
        if (mi == M_IDXW) move_l_imm_d(0x5555FF80, 7);   // D7.W = -0x80, high word to be ignored
        if (mi == M_IDXL) movea_l_imm(0x00010010, 6);    // A6.L = +0x10010
        if (mi == M_PCIDX) {
            uint32_t p = apc + 6 + 4;                        // address of the instruction: after MOVE.L #,D7 and MOVE #,CCR
            move_l_imm_d(0xABCD0000u | ((opa - p) & 0xFFFFu), 7);   // EA = (p+2) + D7.W + (-2) = opa
        }
        move_ccr(ccr);
        uint32_t p = apc;   // address of the instruction; the extension is at p+2 (PRM 2.2: PC = address of the extension)
        switch (mi) {
            case M_IND:   ew(md_op(op, dd, ea(IND, an))); break;
            case M_POST:  ew(md_op(op, dd, ea(POST, an))); break;
            case M_PRE:   ew(md_op(op, dd, ea(PRE, an))); break;
            case M_D16P:  ew(md_op(op, dd, ea(D16, an))); ew(0x0120); break;
            case M_D16N:  ew(md_op(op, dd, ea(D16, an))); ew(0xFE00); break;
            case M_IDXW:  ew(md_op(op, dd, ea(IDX, an))); ew(ext_idx(0, 7, 0, 0x40)); break;   // opa+0x40 + (-0x80) + 0x40
            case M_IDXL:  ew(md_op(op, dd, ea(IDX, an))); ew(ext_idx(1, 6, 1, -6)); break;     // opa+6-0x10010 + 0x10010 - 6
            case M_ABSW:  ew(md_op(op, dd, ea(X7, ABSW))); ew((uint16_t)opa); break;
            case M_ABSWN: ew(md_op(op, dd, ea(X7, ABSW))); ew((uint16_t)(opa & 0xFFFF)); break;  // 8xxx sign-extended = FF8xxx
            case M_ABSL:  ew(md_op(op, dd, ea(X7, ABSL))); el(opa); break;
            case M_PCD16: ew(md_op(op, dd, ea(X7, PCD16))); ew((uint16_t)(opa - (p + 2))); break;
            case M_PCIDX: ew(md_op(op, dd, ea(X7, PCIDX))); ew(ext_idx(0, 7, 0, -2)); break;
            case M_IMM:   ew(md_op(op, dd, ea(X7, IMM))); ew(src); break;
            default: break;
        }
        save_sr(res);
        save_d(dd, res + 2);
        if (uses_an) save_a(an, res + 6);
        MdOut m = md_model(op, dst, src, ccr);
        std::string w = "case " + std::to_string(k) + " " + OPN[op] + " " + MODEN[mi] + ",D" + std::to_string(dd)
                      + " (" + hx(dst) + " , " + hx(src) + ")";
        result(res, 2, 0x2700u | m.ccr, w + ": SR after (S=1, mask 7, X N Z V C)", 0xFF00u | m.mask);
        result(res + 2, 4, m.d, w + ": destination register");
        if (uses_an) result(res + 6, 4, an_final, w + ": An after (PRM 2.2: (An)+ +2, -(An) -2, otherwise unchanged)");
        tinfo.push_back({opa, (uint8_t)((mi == M_PCD16 || mi == M_PCIDX) ? 6 : 5), has_mem, w});
    }
    uint32_t end = apc;
    bra_self();
    if (apc > PROG_LIMIT) bank_error("program beyond PROG_LIMIT");
    return end;
}

// bus: a single word read of the operand, in the right space
static void check_operand_reads() {
    for (const TestInfo& ti : tinfo) {
        if (!ti.has_mem) continue;
        int n = 0; bool fc_ok = true, ds_ok = true;
        for (const Cyc& c : blog) {
            if (c.fc == 7 || c.addr != (ti.opa & 0xFFFFFE)) continue;
            if (!c.rw) continue;   // writes are already forbidden by the allowed area
            n++;
            if (c.fc != ti.fc) fc_ok = false;
            if (!(c.uds && c.lds)) ds_ok = false;
        }
        check_eq(ti.name + ": operand reads (one)", 1, (uint64_t)n);
        check_eq(ti.name + ": FC of the operand read = " + std::to_string(ti.fc), 1, fc_ok ? 1 : 0);
        check_eq(ti.name + ": word read (UDS and LDS)", 1, ds_ok ? 1 : 0);
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    g_ctx = "muldivmem model self-check";
    for (int op = 0; op < 4; op++)
        for (int pi = 0; pi < 4; pi++) {
            MdOut m = md_model(op, PAIRS[op][pi][0], (uint16_t)PAIRS[op][pi][1], 0);
            std::string w = std::string(OPN[op]) + " pair " + std::to_string(pi);
            check_eq(w + " model: Dn", PAIR_EXP[op][pi][0], m.d);
            check_eq(w + " model: CCR", PAIR_EXP[op][pi][1] & m.mask, m.ccr & m.mask);
        }
    static const uint32_t WAITS[2] = {0, 3};
    for (int r = 0; r < 2; r++) {
        g_ctx = "muldivmem DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build();
        run_cpu(WAITS[r], end, 600000);
        check_operand_reads();
    }
    dut->final();
    delete dut;
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    return n_fail == 0 ? 0 : 1;
}
