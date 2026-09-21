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

// tb_m68k_bitext.cpp - BTST, BCHG, BCLR, BSET: static form (#n) on a data register and static and dynamic forms in memory
// (the dynamic form on a register is in tb_m68k_bitops.cpp).
//
// What it tests (214 cases, each run twice: DTACK with zero wait states and DTACK delayed by 3 clocks):
//  - #n,Dn (96 cases): 4 instructions x n = 0 1 7 8 15 16 30 31 32 33 63 255 x tested bit at 0 and at 1; long, bit
//    number modulo 32;
//  - memory, byte, bit number modulo 8 (118 cases): all four instructions in static form (#n with bits 3-7 of n
//    preloaded: 0, 08, 38, F8) and dynamic form (Dn with bits 3-31 preloaded: 0, 8, FFFFFFF8, 12345670); all 8 bits for
//    every instruction and form; byte at an even address (UDS) and at an odd address (LDS); modes (An), (An)+ (+1),
//    -(An) (-1), d16(An) with positive and negative displacement, d8(An,D6.W) with a nonzero upper word of the index,
//    d8(An,A4.L), abs.W, abs.L above 8000; BTST also d16(PC), d8(PC,D6.W) (static form: PC = address of the EA
//    extension, which follows that of the bit number; PRM 2.2) and BTST Dn,#imm;
//  - after EVERY instruction: the whole SR (Z = NOT tested bit before, X N V C unchanged with a rotating input CCR),
//    final An in the modes with An, destination register in the #n,Dn cases;
//  - memory: final target byte, the other byte of the same word unchanged (sentinel), no write outside the target byte
//    (BTST: no write);
//  - bus: at the target address exactly one read (and for BCHG/BCLR/BSET one write, after the read), byte-wide on the
//    correct lane only (UDS for an even address, LDS for an odd one; MC68000UM 5.1), FC = 5, or 6 for the PC-relative
//    reads (PRM 2.2: "program reference"); MC68000UM 8, timing: BTST in memory (1/0)+ea, BCHG BCLR BSET (1/1)+ea.
// Not tested: the clocks.
//
// Expected values: bit_model() model from the Motorola semantics (PRM 4 BTST/BCHG/BCLR/BSET: "Z: set if the bit tested
// is zero, cleared otherwise; X N V C: not affected"; destination Dn: long, bit number modulo 32; destination in memory:
// byte, bit number modulo 8).
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
//  - unused memory = ILLEGAL (0x4AFC); every vector 2..255 points to its own BRA.S * (UNEXP_BASE + 4*v): the fetch of
//    that address is an unexpected exception, reported with its vector number;
//  - log of all bus cycles (address, FC, R/W, UDS, LDS, data);
//  - DTACK asserted g_wait clocks after AS (0 or 3);
//  - end: the final BRA.S * fetched 3 times (the first fetch can be the prefetch during the previous instruction; the
//    second and the third belong to the BRA itself, so all the previous instructions have completed).
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
static constexpr uint32_t PROG_LIMIT = 0x7000;    // code lies below; above: PC-relative data and unexpected-exception handlers
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

// --- minimal assembler (encodings PRM 4; modes PRM 2.2) ---
static uint32_t apc = 0;
static void org(uint32_t a) { apc = a; }
static void ew(uint16_t v) { put16(apc, v); apc += 2; }
static void el(uint32_t v) { ew((uint16_t)(v >> 16)); ew((uint16_t)v); }
static constexpr int DN = 0, AN = 1, IND = 2, POST = 3, PRE = 4, D16 = 5, IDX = 6, X7 = 7;
static constexpr int ABSW = 0, ABSL = 1, PCD16 = 2, PCIDX = 3, IMM = 4;
static uint16_t ea(int m, int r) { return (uint16_t)((m << 3) | r); }
// MOVE (PRM 4 MOVE): 00 ss RRR MMM mmm rrr, ss = 01 byte, 11 word, 10 long
static uint16_t op_move(int sz, int dm, int dr, int sm, int sr) { int s = sz == 1 ? 1 : (sz == 2 ? 3 : 2); return (uint16_t)((s << 12) | (dr << 9) | (dm << 6) | (sm << 3) | sr); }
static void chk_absw(uint32_t a) { if (a >= 0x8000) bank_error("abs.W address not positive 0x" + hx(a)); }
static void move_l_imm_d(uint32_t v, int d) { ew(op_move(4, DN, d, X7, IMM)); el(v); }       // MOVE.L #imm,Dn
static void movea_l_imm(uint32_t v, int a) { ew(op_move(4, AN, a, X7, IMM)); el(v); }        // MOVEA.L #imm,An
static void move_ccr(uint8_t c) { ew(0x44FC); ew(c); }                                          // MOVE #imm,CCR
static void move_sr(uint16_t s) { ew(0x46FC); ew(s); }                                          // MOVE #imm,SR
static void save_sr(uint32_t a) { chk_absw(a); ew(0x40F8); ew((uint16_t)a); }                  // MOVE SR,abs.W
static void save_d(int d, uint32_t a) { chk_absw(a); ew(op_move(4, X7, ABSW, DN, d)); ew((uint16_t)a); }   // MOVE.L Dn,abs.W
static void save_a(int n, uint32_t a) { chk_absw(a); ew(op_move(4, X7, ABSW, AN, n)); ew((uint16_t)a); }   // MOVE.L An,abs.W
static void bra_self() { ew(0x60FE); }                                                          // BRA.S *
// brief index extension word (PRM 2.2, d8(An,Xn) and d8(PC,Xn)): bit 15 D/A, 14-12 register, 11 W/L, 7-0 d8
static uint16_t ext_idx(int is_a, int reg, int is_long, int d8) { return (uint16_t)((is_a << 15) | (reg << 12) | (is_long << 11) | (d8 & 0xFF)); }

// --- expected values ---
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
        if (cyc.fc == 7) {   // no interrupts in this testbench: an IACK is an error (autovector response so as not to hang)
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
// Bit operations (PRM 4): static 0000 1000 tt ea + word with the bit number (bits 7-0); dynamic 0000 rrr 1tt ea;
// tt = 00 BTST, 01 BCHG, 10 BCLR, 11 BSET.
// ======================================================================================================================
static const char* const BN[4] = {"BTST", "BCHG", "BCLR", "BSET"};
// Model (PRM 4): n = bit number modulo 32 (Dn, long) or modulo 8 (memory and #imm, byte); Z = NOT bit n before; BTST no
// change, BCHG inverts, BCLR clears, BSET sets bit n; X N V C unchanged.
struct BitOut { uint32_t v; uint8_t ccr; };
static BitOut bit_model(int op, uint32_t v, uint32_t bitnum, uint32_t modulo, uint8_t ccr_in) {
    uint32_t m = 1u << (bitnum & (modulo - 1));
    uint8_t ccr = (uint8_t)((ccr_in & 0x1B) | ((v & m) ? 0 : 0x04));
    uint32_t r = (op == 0) ? v : (op == 1) ? (v ^ m) : (op == 2) ? (v & ~m) : (v | m);
    return {r, ccr};
}
static const uint8_t CCRS[6] = {0x1B, 0x04, 0x1F, 0x00, 0x0A, 0x15};
static const uint32_t NL[12] = {0, 1, 7, 8, 15, 16, 30, 31, 32, 33, 63, 255};
static const uint32_t RAWHI[4] = {0x00, 0x08, 0x38, 0xF8};                                  // bits 3-7 of the static bit number
static const uint32_t DHI[4] = {0x00000000u, 0x00000008u, 0xFFFFFFF8u, 0x12345670u};        // bits 3-31 of the dynamic bit number

enum { MB_IND, MB_POST, MB_PRE, MB_D16, MB_IDXW, MB_IDXL, MB_ABSW, MB_ABSL, MB_PCD16, MB_PCIDX, MB_IMM };
static const char* const MBN[] = {"(An)", "(An)+", "-(An)", "d16(An)", "d8(An,D6.W)", "d8(An,A4.L)", "abs.W", "abs.L", "d16(PC)", "d8(PC,D6.W)", "#imm"};
struct MemT { int op; bool dyn; int mode; uint32_t nb; };
struct MemInfo { std::string name; uint32_t word; bool odd; bool rmw; uint8_t fc; };
static std::vector<MemInfo> minfo;

static std::vector<MemT> mem_tests() {
    std::vector<MemT> v;
    for (int op = 0; op < 4; op++)                   // all 8 bits per instruction and form, (An) and abs.W alternating
        for (int d = 0; d < 2; d++)
            for (uint32_t nb = 0; nb < 8; nb++) v.push_back({op, d != 0, (nb & 1) ? MB_ABSW : MB_IND, nb});
    static const int MODES[6] = {MB_POST, MB_PRE, MB_D16, MB_IDXW, MB_IDXL, MB_ABSL};
    for (int op = 0; op < 4; op++)                   // the other modes per instruction and form
        for (int d = 0; d < 2; d++)
            for (int mi = 0; mi < 6; mi++) v.push_back({op, d != 0, MODES[mi], (uint32_t)((op * 5 + d * 3 + mi) % 8)});
    for (int d = 0; d < 2; d++) { v.push_back({0, d != 0, MB_PCD16, (uint32_t)(5 + d)}); v.push_back({0, d != 0, MB_PCIDX, (uint32_t)(2 + d)}); }
    v.push_back({0, true, MB_IMM, 3}); v.push_back({0, true, MB_IMM, 6});
    return v;
}

static uint32_t build() {
    mem_init(); exps.clear(); minfo.clear();
    org(PROG_BASE);
    move_sr(0x2700);
    movea_l_imm(0x3F00, 7);
    // --- #n,Dn ---
    uint32_t k = 0;
    for (int op = 0; op < 4; op++)
        for (int ni = 0; ni < 12; ni++)
            for (int b = 0; b < 2; b++, k++) {
                uint32_t n = NL[ni];
                int dv = (int)(k % 8);
                uint32_t base = 0x9C3E71A5u * (k + 1) + 0x6B;
                uint32_t m = 1u << (n & 31u);
                uint32_t v = b ? (base | m) : (base & ~m);
                uint8_t ccr = CCRS[k % 6];
                uint32_t res = RES_BASE + 8 * k;
                move_l_imm_d(v, dv);
                move_ccr(ccr);
                ew((uint16_t)(0x0800 | (op << 6) | ea(DN, dv))); ew((uint16_t)n);
                save_sr(res);
                save_d(dv, res + 2);
                BitOut o = bit_model(op, v, n, 32, ccr);
                std::string w = "case " + std::to_string(k) + " " + BN[op] + " #" + std::to_string(n) + ",D" + std::to_string(dv) + " (value " + hx(v) + ")";
                result(res, 2, 0x2700u | o.ccr, w + ": SR after (Z = NOT bit, X N V C unchanged)");
                result(res + 2, 4, o.v, w + ": destination");
            }
    // --- memory (byte) ---
    std::vector<MemT> tl = mem_tests();
    for (size_t j = 0; j < tl.size(); j++, k++) {
        const MemT& tt = tl[j];
        bool odd = ((j >> 1) & 1) != 0;
        bool bitval = ((j ^ (j >> 2)) & 1) != 0;
        uint32_t word = (tt.mode == MB_ABSL) ? 0x9000 + 4 * (uint32_t)j
                      : (tt.mode == MB_PCD16 || tt.mode == MB_PCIDX) ? 0x7000 + 4 * (uint32_t)j
                      : 0x3000 + 4 * (uint32_t)j;
        uint32_t addr = word + (odd ? 1 : 0);
        uint32_t m = 1u << tt.nb;
        uint8_t base8 = (uint8_t)(0x5A + 37 * j);
        uint8_t tv = (uint8_t)(bitval ? (base8 | m) : (base8 & ~m));
        uint8_t other = (uint8_t)(0xC3 ^ j);
        uint32_t raw = tt.nb | RAWHI[j % 4];
        uint32_t dval = tt.nb | DHI[j % 4];
        int an = 1 + (int)(j % 3), db = (int)(j % 6);
        uint8_t ccr = CCRS[j % 6];
        uint32_t res = 0x1400 + 8 * (uint32_t)j;
        bool uses_an = tt.mode <= MB_IDXL;
        int d16 = (j & 1) ? 0x0123 : -0x0144;
        if (tt.mode != MB_IMM) { put8(addr, tv); put8(addr ^ 1, other); }
        uint32_t an_init = 0;
        switch (tt.mode) {
            case MB_IND: case MB_POST: an_init = addr; break;
            case MB_PRE: an_init = addr + 1; break;
            case MB_D16: an_init = addr - (uint32_t)d16; break;
            case MB_IDXW: an_init = addr + 0x30; break;                 // + D6.W (-0x40) + 0x10
            case MB_IDXL: an_init = addr + 4 - 0x20000; break;          // + A4.L (0x20000) - 4
            default: break;
        }
        uint32_t an_final = (tt.mode == MB_POST) ? addr + 1 : (tt.mode == MB_PRE) ? addr : an_init;
        if (tt.dyn) move_l_imm_d(dval, db);
        if (uses_an) movea_l_imm(an_init, an);
        if (tt.mode == MB_IDXW) move_l_imm_d(0x8765FFC0u, 6);
        if (tt.mode == MB_IDXL) movea_l_imm(0x00020000u, 4);
        uint32_t P = apc + (tt.mode == MB_PCIDX ? 6 : 0) + 4;           // address of the instruction
        uint32_t pc_ref = P + (tt.dyn ? 2 : 4);                        // PC of the relative modes: EA extension (PRM 2.2)
        if (tt.mode == MB_PCIDX) move_l_imm_d(0xA5A50000u | ((addr - pc_ref - 2) & 0xFFFFu), 6);   // EA = pc_ref + D6.W + 2
        move_ccr(ccr);
        if (apc != P) bank_error("bitext: address of the instruction");
        uint16_t eaf = 0;
        switch (tt.mode) {
            case MB_IND: eaf = ea(IND, an); break;
            case MB_POST: eaf = ea(POST, an); break;
            case MB_PRE: eaf = ea(PRE, an); break;
            case MB_D16: eaf = ea(D16, an); break;
            case MB_IDXW: case MB_IDXL: eaf = ea(IDX, an); break;
            case MB_ABSW: eaf = ea(X7, ABSW); break;
            case MB_ABSL: eaf = ea(X7, ABSL); break;
            case MB_PCD16: eaf = ea(X7, PCD16); break;
            case MB_PCIDX: eaf = ea(X7, PCIDX); break;
            case MB_IMM: eaf = ea(X7, IMM); break;
            default: break;
        }
        if (tt.dyn) ew((uint16_t)(0x0100 | (db << 9) | (tt.op << 6) | eaf));
        else { ew((uint16_t)(0x0800 | (tt.op << 6) | eaf)); ew((uint16_t)raw); }
        if (apc != pc_ref) bank_error("bitext: address of the EA extension");
        switch (tt.mode) {
            case MB_D16: ew((uint16_t)d16); break;
            case MB_IDXW: ew(ext_idx(0, 6, 0, 0x10)); break;
            case MB_IDXL: ew(ext_idx(1, 4, 1, -4)); break;
            case MB_ABSW: chk_absw(addr); ew((uint16_t)addr); break;
            case MB_ABSL: el(addr); break;
            case MB_PCD16: ew((uint16_t)(addr - pc_ref)); break;
            case MB_PCIDX: ew(ext_idx(0, 6, 0, 2)); break;
            case MB_IMM: tv = (uint8_t)(0x96 ^ (37 * j)); ew((uint16_t)tv); break;   // immediate byte in the low word
            default: break;
        }
        save_sr(res);
        if (uses_an) save_a(an, res + 2);
        uint32_t bitnum = tt.dyn ? dval : raw;
        BitOut o = bit_model(tt.op, tv, bitnum, 8, ccr);
        std::string w = "case " + std::to_string(k) + " " + BN[tt.op] + (tt.dyn ? " D" + std::to_string(db) + "=" + hx(dval) : " #" + hx(raw)) + ","
                      + MBN[tt.mode] + " (byte at 0x" + hx(addr) + " = " + hx(tv) + ", bit " + std::to_string(tt.nb) + ")";
        result(res, 2, 0x2700u | o.ccr, w + ": SR after (Z = NOT bit, X N V C unchanged)");
        if (uses_an) result(res + 2, 4, an_final, w + ": An after (PRM 2.2: (An)+ +1 and -(An) -1 for bytes)");
        if (tt.mode != MB_IMM) {
            if (tt.op != 0) allow(addr, 1);
            expect(addr, 1, o.v & 0xFF, w + ": target byte");
            expect(addr ^ 1, 1, other, w + ": other byte of the word unchanged");
            minfo.push_back({w, word, odd, tt.op != 0, (uint8_t)((tt.mode == MB_PCD16 || tt.mode == MB_PCIDX) ? 6 : 5)});
        }
    }
    uint32_t end = apc;
    bra_self();
    if (apc > PROG_LIMIT) bank_error("program beyond PROG_LIMIT");
    if (RES_BASE + 8 * 96 > 0x1400 || 0x1400 + 8 * tl.size() > 0x3000) bank_error("overlapping result areas");
    return end;
}

// bus: one read (and for BCHG/BCLR/BSET a following write) on the target byte, on the correct lane only
static void check_mem_bus() {
    for (const MemInfo& mi : minfo) {
        int nr = 0, nw = 0, first_r = -1, first_w = -1; bool lane = true, fc_ok = true;
        for (size_t i = 0; i < blog.size(); i++) {
            const Cyc& c = blog[i];
            if (c.fc == 7 || c.addr != mi.word) continue;
            if (c.uds != !mi.odd || c.lds != mi.odd) lane = false;
            if (c.rw) { nr++; if (first_r < 0) first_r = (int)i; if (c.fc != mi.fc) fc_ok = false; }
            else { nw++; if (first_w < 0) first_w = (int)i; if (c.fc != 5) fc_ok = false; }
        }
        check_eq(mi.name + ": reads of the target", 1, (uint64_t)nr);
        check_eq(mi.name + ": writes of the target", mi.rmw ? 1 : 0, (uint64_t)nw);
        check_eq(mi.name + std::string(": byte lane (") + (mi.odd ? "LDS" : "UDS") + " only)", 1, lane ? 1 : 0);
        check_eq(mi.name + ": FC of the reads/writes of the target", 1, fc_ok ? 1 : 0);
        if (mi.rmw) check_eq(mi.name + ": read before the write", 1, (first_r >= 0 && first_w > first_r) ? 1 : 0);
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    // self-check of the model with hand-computed values
    g_ctx = "bitext model self-check";
    { BitOut o = bit_model(3, 0x00000000, 255, 32, 0x00); check_eq("BSET #255,Dn = bit 31: value", 0x80000000, o.v); check_eq("BSET #255,Dn: CCR (Z=1)", 0x04, o.ccr); }
    { BitOut o = bit_model(1, 0x01, 0xF8, 8, 0x1F); check_eq("BCHG #F8 in memory = bit 0 of 01: value", 0x00, o.v); check_eq("BCHG bit 0 = 1: CCR (Z=0, X N V C kept)", 0x1B, o.ccr); }
    { BitOut o = bit_model(2, 0x80, 0xFFFFFFFF, 8, 0x04); check_eq("BCLR Dn=FFFFFFFF in memory = bit 7 of 80: value", 0x00, o.v); check_eq("BCLR bit 7 = 1: CCR (Z=0)", 0x00, o.ccr); }
    { BitOut o = bit_model(0, 0x7F, 0x12345677, 8, 0x0A); check_eq("BTST Dn=12345677 in memory = bit 7 of 7F: value", 0x7F, o.v); check_eq("BTST bit 7 = 0: CCR (Z=1)", 0x0E, o.ccr); }
    static const uint32_t WAITS[2] = {0, 3};
    for (int r = 0; r < 2; r++) {
        g_ctx = "bitext DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build();
        run_cpu(WAITS[r], end, 1500000);
        check_mem_bus();
    }
    dut->final();
    delete dut;
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    return n_fail == 0 ? 0 : 1;
}
