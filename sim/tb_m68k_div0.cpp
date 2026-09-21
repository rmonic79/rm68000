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

// tb_m68k_div0.cpp - DIVU.W and DIVS.W with a zero divisor: divide-by-zero exception (vector 5).
//
// What it tests (13 cases, each executed twice: DTACK with zero wait states and DTACK delayed by 3 clocks):
//  - DIVU and DIVS with the divisor from a register (upper word of the register non-zero: only the low word counts),
//    immediate #0, (An), -(An), abs.W, d16(An), abs.L, d16(PC), d8(An,D7.L); from supervisor (masks 7, 3, 2) and from
//    user (S = 0, frame on the SSP);
//  - frame read in full: SR (T S mask X; N Z V C not compared, see below), PC high and PC low = address of the
//    instruction following the DIV, hence after the extension words (MC68000UM 6.3: instruction traps stack the PC of
//    the next instruction); frame cells preloaded with the complement of the expected value;
//  - the three frame writes: exactly three (MC68000UM 8, exception timing table: x(4/3)), word-sized, FC = 5
//    (supervisor data) even when the exception arises in user mode;
//  - in the handler (separate for vector 5: every other vector leads to a BRA.S * of unexpected exception): SR with
//    S = 1, T = 0, mask unchanged, X unchanged; A7 = SSP - 6;
//  - the instruction after the DIV is not executed (its write is forbidden and the cell stays at the sentinel);
//  - the destination register stays unchanged (PRM 4 DIVU/DIVS: division by zero causes the trap, there is no
//    quotient); USP unchanged in the user cases.
// NOT tested: N Z V C in the frame (PRM 4 DIVU/DIVS: N Z V undefined on division by zero; C "always cleared" is not
// stated to hold in the trap as well, so it is not compared); An after -(An) (not specified by the manual for the trap);
// the clocks.

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
static constexpr uint32_t PROG_LIMIT = 0x6800;    // the code sits below; above: handlers, PC-relative data, unexpected-exception handlers
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
static void move_sr(uint16_t s) { ew(0x46FC); ew(s); }                                          // MOVE #imm,SR
static void save_d(int d, uint32_t a) { chk_absw(a); ew(op_move(4, X7, ABSW, DN, d)); ew((uint16_t)a); }   // MOVE.L Dn,abs.W
static void save_a(int n, uint32_t a) { chk_absw(a); ew(op_move(4, X7, ABSW, AN, n)); ew((uint16_t)a); }   // MOVE.L An,abs.W
static void bra_self() { ew(0x60FE); }                                                          // BRA.S *
static uint16_t ext_idx(int is_a, int reg, int is_long, int d8) { return (uint16_t)((is_a << 15) | (reg << 12) | (is_long << 11) | (d8 & 0xFF)); }   // PRM 2.2, brief extension word

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
    dut->rst_n = 1; dut->cen = 1; dut->cen2 = 1;   // single phase (SINGLE_CEN=1 in the Makefile: cen2 ignored)
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
// writes of the exception frame in [top-6, top): how many, all word-sized and with FC = 5 (supervisor data)
static void check_frame_writes(uint32_t top, int n_exp, const std::string& w) {
    int n = 0; bool ok = true;
    for (const Cyc& c : blog) {
        if (c.fc == 7 || c.rw || c.addr < top - 6 || c.addr >= top) continue;
        n++;
        if (c.fc != 5 || !c.uds || !c.lds) ok = false;
    }
    check_eq(w + ": frame writes", (uint64_t)n_exp, (uint64_t)n);
    check_eq(w + ": word-sized frame writes with FC=5", 1, ok ? 1 : 0);
}

// ======================================================================================================================
// Cases. DIVU.W <ea>,Dn = 1000 ddd 011 ea, DIVS.W = 1000 ddd 111 ea (PRM 4). Vector 5 (MC68000UM 6: vector table).
// Handler of vector 5 (H5): MOVE SR,(A5) ; MOVE.L A7,4(A5) ; JMP (A6). A5 = area of the case, A6 = continuation.
// ======================================================================================================================
static constexpr uint32_t H5 = 0x6800;
enum { S_DN, S_IMM, S_IND, S_PRE, S_ABSW, S_D16, S_ABSL, S_PCD16, S_IDXL };
// ds/sv: source register and its value (S_DN mode only: low word zero, high word non-zero)
struct D0Case { bool divs; int mode; int dd; uint32_t dividend; uint16_t sr_in; bool user; int ds; uint32_t sv; const char* why; };
static const D0Case CASES[] = {
    {false, S_DN,    0, 0x12345678, 0x271F, false, 1, 0xFFFF0000, "DIVU D1,D0, D1 = FFFF0000 (low word 0), X=1"},
    {true,  S_DN,    3, 0x80000000, 0x2700, false, 2, 0x00010000, "DIVS D2,D3, D2 = 00010000"},
    {false, S_IMM,   4, 0xFFFFFFFF, 0x2310, false, 0, 0,          "DIVU #0,D4, mask 3"},
    {true,  S_IMM,   5, 0x00000001, 0x2208, false, 0, 0,          "DIVS #0,D5, mask 2"},
    {false, S_IND,   6, 0x00010000, 0x2715, false, 0, 0,          "DIVU (A0),D6"},
    {true,  S_PRE,   7, 0x7FFFFFFF, 0x2700, false, 0, 0,          "DIVS -(A1),D7"},
    {false, S_ABSW,  0, 0x0000FFFF, 0x2711, false, 0, 0,          "DIVU abs.W,D0"},
    {true,  S_D16,   1, 0xFFFF8000, 0x2704, false, 0, 0,          "DIVS d16(A2),D1"},
    {false, S_ABSL,  2, 0x00000000, 0x2710, false, 0, 0,          "DIVU abs.L,D2"},
    {true,  S_PCD16, 3, 0x00000007, 0x2700, false, 0, 0,          "DIVS d16(PC),D3"},
    {true,  S_DN,    5, 0xFFFFFFF9, 0x0010, true,  4, 0x12340000, "DIVS D4,D5 in user mode, D4 = 12340000, X=1"},
    {false, S_IND,   6, 0x00000005, 0x0000, true,  0, 0,          "DIVU (A3),D6 in user mode"},
    {false, S_IDXL,  0, 0x00012345, 0x2701, false, 0, 0,          "DIVU d8(A4,D7.L),D0"},
};
static constexpr size_t NCASES = sizeof(CASES) / sizeof(CASES[0]);
struct FrameInfo { uint32_t top; std::string name; };
static std::vector<FrameInfo> frames;

static uint32_t build() {
    mem_init(); exps.clear(); frames.clear();
    put32(4 * 5, H5);
    org(H5);
    ew(0x40D5);              // MOVE SR,(A5)
    ew(0x2B4F); ew(0x0004);  // MOVE.L A7,4(A5)
    ew(0x4ED6);              // JMP (A6)
    org(PROG_BASE);
    move_sr(0x2700);
    for (size_t k = 0; k < NCASES; k++) {
        const D0Case& c = CASES[k];
        uint32_t res = RES_BASE + 0x20 * (uint32_t)k;
        uint32_t top = 0x3800 + 0x40 * (uint32_t)k;
        uint32_t usp = 0x3E00 + 0x10 * (uint32_t)k;
        uint32_t opa = 0x3000 + 0x10 * (uint32_t)k;
        if (c.mode == S_ABSL) opa = 0x9000 + 0x10 * (uint32_t)k;
        if (c.mode == S_PCD16) opa = 0x7000 + 0x10 * (uint32_t)k;
        put16(opa, 0x0000);                // zero divisor in memory
        std::string w = "case " + std::to_string(k) + " " + c.why;
        movea_l_imm(top, 7);
        if (c.user) { movea_l_imm(usp, 0); ew(0x4E60); }   // MOVE A0,USP (PRM 4 MOVE USP)
        movea_l_imm(res, 5);
        uint32_t patch = apc + 2;
        movea_l_imm(0, 6);                 // A6 = continuation (written below)
        move_l_imm_d(c.dividend, c.dd);
        uint16_t eaf = 0; int ext = 0;
        switch (c.mode) {
            case S_DN:    move_l_imm_d(c.sv, c.ds); eaf = ea(DN, c.ds); break;
            case S_IMM:   eaf = ea(X7, IMM); ext = 1; break;
            case S_IND:   { int a = c.user ? 3 : 0; movea_l_imm(opa, a); eaf = ea(IND, a); break; }
            case S_PRE:   movea_l_imm(opa + 2, 1); eaf = ea(PRE, 1); break;
            case S_ABSW:  eaf = ea(X7, ABSW); ext = 1; break;
            case S_D16:   movea_l_imm(opa - 0x0100, 2); eaf = ea(D16, 2); ext = 1; break;
            case S_ABSL:  eaf = ea(X7, ABSL); ext = 2; break;
            case S_PCD16: eaf = ea(X7, PCD16); ext = 1; break;
            case S_IDXL:  movea_l_imm(opa - 0x0110, 4); move_l_imm_d(0x00000100, 7); eaf = ea(IDX, 4); ext = 1; break;
            default: break;
        }
        move_sr(c.sr_in);
        uint32_t p = apc;
        ew((uint16_t)((c.divs ? 0x81C0 : 0x80C0) | (c.dd << 9) | eaf));
        switch (c.mode) {
            case S_IMM:   ew(0x0000); break;
            case S_ABSW:  ew((uint16_t)opa); break;
            case S_D16:   ew(0x0100); break;
            case S_ABSL:  el(opa); break;
            case S_PCD16: ew((uint16_t)(opa - (p + 2))); break;
            case S_IDXL:  ew(ext_idx(0, 7, 1, 0x10)); break;
            default: break;
        }
        uint32_t pc_next = p + 2 + 2 * (uint32_t)ext;
        if (apc != pc_next) bank_error(w + ": instruction length");
        // next instruction: must not be executed (write forbidden, sentinel cell)
        ew(0x31FC); ew(0xDEAD); ew((uint16_t)(res + 12));   // MOVE.W #$DEAD,res+12
        ew(0x4ED6);                                          // JMP (A6)
        uint32_t cont = apc;
        put32(patch, cont);
        save_d(c.dd, res + 8);
        if (c.user) { ew(0x4E68); save_a(0, res + 14); }      // MOVE USP,A0 ; MOVE.L A0,res+14
        move_sr(0x2700);
        // expected values
        uint16_t sr_frame = c.sr_in;   // SR before the trap; comparison on T S I2-I0 X (mask A710)
        result(top - 6, 2, sr_frame, w + ": frame SR (T S mask X)", 0xA710);
        result(top - 4, 2, pc_next >> 16, w + ": frame PC high");
        result(top - 2, 2, pc_next & 0xFFFF, w + ": frame PC low = instruction after the DIV");
        result(res, 2, (uint16_t)(0x2000 | (c.sr_in & 0x0710)), w + ": SR in the handler (S=1 T=0 mask and X unchanged)", 0xA710);
        result(res + 4, 4, top - 6, w + ": A7 in the handler = SSP-6");
        result(res + 8, 4, c.dividend, w + ": destination register unchanged");
        put16(res + 12, 0x5A5A); expect(res + 12, 2, 0x5A5A, w + ": instruction after the DIV not executed (sentinel)");
        if (c.user) result(res + 14, 4, usp, w + ": USP unchanged");
        frames.push_back({top, w});
    }
    uint32_t end = apc;
    bra_self();
    if (apc > PROG_LIMIT) bank_error("program beyond PROG_LIMIT");
    return end;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    static const uint32_t WAITS[2] = {0, 3};
    for (int r = 0; r < 2; r++) {
        g_ctx = "div0 DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build();
        run_cpu(WAITS[r], end, 300000);
        for (const FrameInfo& f : frames) check_frame_writes(f.top, 3, f.name);
    }
    dut->final();
    delete dut;
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    return n_fail == 0 ? 0 : 1;
}
