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

// tb_m68k_irq_multi.cpp - hardware interrupts: all levels with autovector, vectored interrupts, uninitialized vector,
// spurious interrupt, return with RTE from supervisor and from user mode, nested interrupts.
//
// Three programs, each run twice (DTACK with zero wait states and DTACK delayed by 3 clocks). In every case the program
// sets SSP, the case area (A5) and the continuation (A6), writes the waiting SR (mask = level - 1 or less, mode, CCR
// different per case) and spins on a BRA.S *; the testbench raises the IPL only after seeing that BRA fetched three
// times and withdraws it at the IACK.
// Program 1 (12 cases):
//  - levels 1..7 with autovector (VPA in the IACK): vector 24 + level (MC68000UM 6: autovectors 25-31);
//  - vectored (DTACK in the IACK, number on D7-D0 and 0xA5 on D15-D8, to be ignored; MC68000UM 6.3.2): vector 0x40 and
//    0xFF;
//  - uninitialized vector: the device answers 15 (MC68000UM 6: "uninitialized interrupt vector"), handler 15;
//  - RTE from supervisor and from user mode (vectors 0x50 and 0x51, a handler that rewrites the stacked PC with the
//    continuation and executes RTE): after the RTE SR = SR of the frame (previous mask and mode, MC68000UM 6.3: RTE
//    restores SR and PC), A7 = previous SSP or USP; the frame stacked by the exception (PC of the interrupted BRA) is
//    read from the write log.
// Program 2 (1 case): spurious interrupt: BERR during the IACK -> vector 24 with a short frame (MC68000UM 6.3.2: "the
//    processor ... fetches the spurious interrupt vector, vector number 24, instead of performing the bus error stack
//    frame"); an entry into the bus error (vector 2) or into an autovector leads to the unexpected-exception BRA.S *.
// Program 3 (2 cases): nested interrupts (MC68000UM 6.3.2: "the priority mask is set to the level of the interrupt being
//    acknowledged"): level 3 with mask 2 taken; inside its handler (mask 3, spinning on a BRA.S *) level 3 held 600 clocks
//    and level 2 held 600 clocks are not taken, then level 5 is taken: second frame at SSP-12 with the handler's SR (mask
//    3), mask 5 in the inner handler; its RTE returns into the level 3 handler with mask 3 and A7 = SSP-6, whose RTE
//    returns to the program with mask 2 and A7 = SSP.
// For every case:
//  - IACK cycle: exactly one; FC = 7, A23-A4 all at 1, A3-A1 = level served, R/W = read, LDS asserted;
//  - vector recognized by a distinct handler per vector that writes its own number;
//  - frame read in full (waiting SR, upper PC, lower PC = interrupted BRA.S *), cells preloaded with the complement of
//    the expected value; exactly three writes from the exception, word-wide, FC = 5 (also from user mode);
//  - in the handler: SR with S = 1, T = 0, mask = level served, CCR unchanged; A7 = SSP - 6.
// T = 1 with an interrupt pending is in tb_m68k_irq.cpp. The manual fixes no clock for the sampling of IPL within an
// instruction (MC68000UM 6.3.2): the level is raised while the processor spins on the BRA.S * and held until the IACK.

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
//  - DTACK asserted g_wait clocks after AS (0 or 3); in the IACK cycles the testbench responds as the current case says;
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
static constexpr uint32_t PROG_LIMIT = 0x6800;    // code lies below; above: handlers and unexpected-exception handlers
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
static constexpr int AN = 1, X7 = 7;
static constexpr int ABSW = 0, IMM = 4;
// MOVE (PRM 4 MOVE): 00 ss RRR MMM mmm rrr, ss = 01 byte, 11 word, 10 long
static uint16_t op_move(int sz, int dm, int dr, int sm, int sr) { int s = sz == 1 ? 1 : (sz == 2 ? 3 : 2); return (uint16_t)((s << 12) | (dr << 9) | (dm << 6) | (sm << 3) | sr); }
static void chk_absw(uint32_t a) { if (a >= 0x8000) bank_error("abs.W address not positive 0x" + hx(a)); }
static void movea_l_imm(uint32_t v, int a) { ew(op_move(4, AN, a, X7, IMM)); el(v); }        // MOVEA.L #imm,An
static void move_sr(uint16_t s) { ew(0x46FC); ew(s); }                                          // MOVE #imm,SR
static void save_sr(uint32_t a) { chk_absw(a); ew(0x40F8); ew((uint16_t)a); }                  // MOVE SR,abs.W
static void save_a(int n, uint32_t a) { chk_absw(a); ew(op_move(4, X7, ABSW, AN, n)); ew((uint16_t)a); }   // MOVE.L An,abs.W
static void bra_self() { ew(0x60FE); }                                                          // BRA.S *

// --- expected values ---
struct Expect { uint32_t addr; int size; uint32_t val; uint32_t mask; std::string what; };
static std::vector<Expect> exps;
static uint32_t rd_sz(uint32_t a, int size) { return size == 1 ? get8(a) : size == 2 ? get16(a) : get32(a); }
static void wr_sz(uint32_t a, int size, uint32_t v) { if (size == 1) put8(a, (uint8_t)v); else if (size == 2) put16(a, (uint16_t)v); else put32(a, v); }
static void expect(uint32_t a, int size, uint32_t v, const std::string& what, uint32_t mask = 0xFFFFFFFFu) { exps.push_back({a, size, v, mask, what}); }
// result cell: writable and preloaded with the complement of the expected value (a missed write cannot pass)
static void result(uint32_t a, int size, uint32_t v, const std::string& what, uint32_t mask = 0xFFFFFFFFu) { allow(a, (uint32_t)size); wr_sz(a, size, ~v); expect(a, size, v, what, mask); }

// --- interrupt cases: IPL script and device response in the IACK ---
enum { R_VPA = 0, R_VEC = 1, R_BERR = 2 };
struct IrqStep { int level; uint64_t clocks; };   // clocks = 0: level held until the IACK
struct IrqCase {
    std::string name; uint16_t sr_wait; bool user; std::vector<IrqStep> steps; int resp; uint8_t vecnum;
    int exp_level; int exp_vec; bool rte; uint64_t hold_after;
    bool inner;          // nested case: its wait point is inside the handler of the previous case (program 3)
    uint32_t wait_pc, cont_pc, res, top, usp;
};
static IrqCase mk(const std::string& name, uint16_t sr_wait, bool user, std::vector<IrqStep> steps, int resp, uint8_t vecnum,
                  int exp_level, int exp_vec, bool rte, uint64_t hold_after) {
    IrqCase c; c.name = name; c.sr_wait = sr_wait; c.user = user; c.steps = steps; c.resp = resp; c.vecnum = vecnum;
    c.exp_level = exp_level; c.exp_vec = exp_vec; c.rte = rte; c.hold_after = hold_after;
    c.inner = false;
    c.wait_pc = 0; c.cont_pc = 0; c.res = 0; c.top = 0; c.usp = 0;
    return c;
}
static std::vector<IrqCase> cases;
static size_t cur = 0;
static int phase = 0;        // 0 waiting for the case's BRA, 1 IPL script, 2 IACK in progress, 3 hold after the IACK
static size_t step = 0;
static uint64_t t0 = 0;

// --- bus ---
struct Cyc { uint32_t addr; uint8_t fc; bool rw, uds, lds; uint16_t data; uint64_t at; };
static std::vector<Cyc> blog;
static bool cyc_on = false;
static uint32_t cyc_len = 0;
static Cyc cyc{};
static uint32_t g_wait = 0, g_end = 0;
static int g_end_seen = 0, g_unexp = -1, g_bad_wr = 0, g_iack = 0, g_out = 0;
static std::vector<int> n_iack, wait_seen;
static std::vector<Cyc> first_iack;

static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; nclk++; }
static void wr_byte(uint32_t a, uint8_t v) {
    int i = mem_idx(a);
    if (i < 0) return;
    if (!wr_ok[i]) { g_bad_wr++; if (g_bad_wr <= 10) printf("FAIL: %s: write outside the expected areas at 0x%06X expected=none read=0x%02X\n", g_ctx.c_str(), (unsigned)a, (unsigned)v); }
    mem[i] = v;
}
static void iack_begin() {
    if (cur >= cases.size()) { g_iack++; return; }   // IACK after the last case: unexpected
    if (phase == 1) phase = 2;
    if (cases[cur].hold_after == 0) dut->ipl = 0;     // request withdrawn at the IACK
}
static void iack_commit() {
    if (cur >= cases.size()) return;
    n_iack[cur]++;
    if (n_iack[cur] == 1) first_iack[cur] = cyc;
    if (phase == 2) { phase = 3; t0 = nclk; }
}
static void commit() {
    blog.push_back(cyc);
    if (cyc.fc == 7) { iack_commit(); return; }
    if (mem_idx(cyc.addr) < 0) { g_out++; if (g_out <= 10) printf("FAIL: %s: bus cycle outside memory at 0x%06X expected=none read=1\n", g_ctx.c_str(), (unsigned)cyc.addr); return; }
    if (!cyc.rw) {
        if (!cyc.uds && !cyc.lds) { g_bad_wr++; printf("FAIL: %s: write without data strobe at 0x%06X expected=UDS/LDS read=none\n", g_ctx.c_str(), (unsigned)cyc.addr); }
        if (cyc.uds) wr_byte(cyc.addr, (uint8_t)(cyc.data >> 8));
        if (cyc.lds) wr_byte(cyc.addr + 1, (uint8_t)cyc.data);
    } else if (cyc.fc == 2 || cyc.fc == 6) {
        if (cyc.addr == g_end) g_end_seen++;
        if (g_unexp < 0 && cyc.addr >= UNEXP_BASE && cyc.addr < UNEXP_BASE + 0x400) g_unexp = (int)((cyc.addr - UNEXP_BASE) >> 2);
        for (size_t k = 0; k < cases.size(); k++) if (cyc.addr == cases[k].wait_pc) wait_seen[k]++;
    }
}
static void bus_step() {
    if (!dut->as_n) {
        if (!cyc_on) {
            cyc_on = true; cyc_len = 0;
            cyc.addr = ((uint32_t)dut->addr << 1) & 0xFFFFFF; cyc.fc = (uint8_t)dut->fc; cyc.rw = dut->rw != 0;
            cyc.uds = false; cyc.lds = false; cyc.data = 0; cyc.at = nclk;
            if (cyc.fc == 7) iack_begin();
        } else cyc_len++;
        if (!dut->rw) cyc.rw = false;
        if (cyc.fc == 7) {
            int resp = (cur < cases.size()) ? cases[cur].resp : R_VPA;
            if (!dut->uds_n) cyc.uds = true;
            if (!dut->lds_n) cyc.lds = true;
            if (resp == R_VPA) { dut->vpa_n = 0; dut->dtack_n = 1; dut->berr_n = 1; }
            else if (resp == R_VEC) {   // vector number on D7-D0; D15-D8 at an arbitrary value (0xA5) that must be ignored
                dut->data_in = (uint16_t)(0xA500u | cases[cur].vecnum); cyc.data = dut->data_in;
                dut->vpa_n = 1; dut->berr_n = 1; dut->dtack_n = (cyc_len >= g_wait) ? 0 : 1;
            } else { dut->vpa_n = 1; dut->dtack_n = 1; dut->berr_n = (cyc_len >= g_wait) ? 0 : 1; }
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
// IPL script of the current case, before every clock
static void irq_hook() {
    if (cur >= cases.size()) { dut->ipl = 0; return; }
    const IrqCase& c = cases[cur];
    if (phase == 0) {
        dut->ipl = 0;
        if (wait_seen[cur] >= 3) { phase = 1; step = 0; t0 = nclk; }
    }
    if (phase == 1) {
        if (c.steps[step].clocks != 0 && nclk - t0 >= c.steps[step].clocks && step + 1 < c.steps.size()) { step++; t0 = nclk; }
        dut->ipl = (uint8_t)c.steps[step].level;
    } else if (phase == 3) {
        if (nclk - t0 >= c.hold_after) { dut->ipl = 0; cur++; phase = 0; }
    }
}
static void run_cpu(uint32_t wait, uint32_t end_addr, uint64_t max_clk) {
    g_wait = wait; g_end = end_addr; g_end_seen = 0; g_unexp = -1; g_bad_wr = 0; g_iack = 0; g_out = 0;
    blog.clear(); cyc_on = false;
    cur = 0; phase = 0; step = 0; t0 = 0;
    n_iack.assign(cases.size(), 0); wait_seen.assign(cases.size(), 0); first_iack.assign(cases.size(), Cyc{});
    dut->rst_n = 0; dut->cen = 0; dut->cen2 = 0; dut->start = 0; dut->rst_vector = 0; dut->pc_init = PROG_BASE;
    dut->ipl = 0; dut->vpa_n = 1; dut->dtack_n = 1; dut->berr_n = 1; dut->halt_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    dut->data_in = 0; dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1; dut->cen = 1; dut->cen2 = 1;   // single phase (SINGLE_CEN=1 in the Makefile: cen2 ignored)
    for (uint64_t i = 0; i < max_clk && g_end_seen < 3 && g_unexp < 0; i++) { irq_hook(); bus_step(); tick(); }
    check_eq("end of program (BRA.S * at 0x" + hx(end_addr) + " fetched 3 times)", 1, g_end_seen >= 3 ? 1 : 0);
    check_eq("unexpected exception (vector number; 0x100 = none)", 0x100, g_unexp < 0 ? 0x100u : (uint64_t)g_unexp);
    check_eq("writes outside the expected areas", 0, (uint64_t)g_bad_wr);
    check_eq("bus cycles outside memory", 0, (uint64_t)g_out);
    check_eq("IACK cycles after the last case", 0, (uint64_t)g_iack);
    for (const Expect& e : exps) {
        uint32_t sm = e.size == 1 ? 0xFFu : e.size == 2 ? 0xFFFFu : 0xFFFFFFFFu;
        check_eq(e.what + " @0x" + hx(e.addr), e.val & e.mask & sm, rd_sz(e.addr, e.size) & e.mask & sm);
    }
}

// ======================================================================================================================
// Program. Handler of vector v (one per vector, at H_BASE + 0x20*i):
//   MOVE SR,(A5) ; MOVE.L A7,4(A5) ; MOVE.W #v,8(A5) ; JMP (A6)
//   (RTE variant: ... ; MOVE.L A6,2(A7) ; RTE: returns to the continuation with the frame's SR)
// Case k: MOVEA.L #top,A7 ; [user: MOVEA.L #usp,A0 ; MOVE A0,USP] ; MOVEA.L #res,A5 ; MOVEA.L #cont,A6 ;
//         MOVE #sr_wait,SR ; wait: BRA.S * ; cont: [RTE: MOVE SR,res+10 ; MOVE.L A7,res+12]
//         [user without RTE: MOVE USP,A0 ; MOVE.L A0,res+16] ; MOVE #$2700,SR (not after the RTE to user mode: it is
//         the last one)
// ======================================================================================================================
static constexpr uint32_t H_BASE = 0x6800;
static void emit_handler(uint32_t at, uint8_t vec, bool rte) {
    org(at);
    ew(0x40D5);                          // MOVE SR,(A5)
    ew(0x2B4F); ew(0x0004);              // MOVE.L A7,4(A5)
    ew(0x3B7C); ew(vec); ew(0x0008);     // MOVE.W #vec,8(A5)
    if (rte) { ew(0x2F4E); ew(0x0002); ew(0x4E73); }   // MOVE.L A6,2(A7) ; RTE
    else ew(0x4ED6);                     // JMP (A6)
    put32(4u * vec, at);
}
static uint32_t emit_cases() {
    org(PROG_BASE);
    move_sr(0x2700);
    for (size_t k = 0; k < cases.size(); k++) {
        IrqCase& c = cases[k];
        if (c.user && c.rte && k + 1 != cases.size()) bank_error("user case with RTE is not the last one");
        c.res = RES_BASE + 0x20 * (uint32_t)k; c.top = 0x3100 + 0x40 * (uint32_t)k; c.usp = 0x3E00 + 0x10 * (uint32_t)k;
        if (c.inner) { c.top = cases[k - 1].top - 6; continue; }   // nested case: frame below the previous case's; code, wait point and expected values in emit_nested_handlers
        movea_l_imm(c.top, 7);
        if (c.user) { movea_l_imm(c.usp, 0); ew(0x4E60); }   // MOVE A0,USP
        movea_l_imm(c.res, 5);
        uint32_t patch = apc + 2;
        movea_l_imm(0, 6);
        move_sr(c.sr_wait);
        c.wait_pc = apc;
        bra_self();
        c.cont_pc = apc;
        put32(patch, c.cont_pc);
        if (c.rte) { save_sr(c.res + 10); save_a(7, c.res + 12); }
        if (c.user && !c.rte) { ew(0x4E68); save_a(0, c.res + 16); }   // MOVE USP,A0 ; MOVE.L A0,res+16
        if (!(c.user && c.rte)) move_sr(0x2700);
        // expected values
        std::string w = "case " + std::to_string(k) + " (" + c.name + ")";
        uint16_t ccr = (uint16_t)(c.sr_wait & 0x1F);
        result(c.res, 2, (uint16_t)(0x2000 | (c.exp_level << 8) | ccr), w + ": SR in the handler (S=1 T=0 mask = level, CCR unchanged)");
        result(c.res + 4, 4, c.top - 6, w + ": A7 in the handler = SSP-6");
        result(c.res + 8, 2, (uint32_t)c.exp_vec, w + ": vector used (handler that ran)");
        result(c.top - 6, 2, c.sr_wait, w + ": frame SR = SR before the interrupt");
        if (c.rte) {
            result(c.top - 4, 2, c.cont_pc >> 16, w + ": frame upper PC after the handler's rewrite");
            result(c.top - 2, 2, c.cont_pc & 0xFFFF, w + ": frame lower PC after the handler's rewrite");
            result(c.res + 10, 2, c.sr_wait, w + ": SR after RTE = SR of the frame");
            result(c.res + 12, 4, c.user ? c.usp : c.top, w + ": A7 after RTE");
        } else {
            result(c.top - 4, 2, c.wait_pc >> 16, w + ": frame PC high");
            result(c.top - 2, 2, c.wait_pc & 0xFFFF, w + ": frame lower PC = interrupted BRA.S *");
        }
        if (c.user && !c.rte) result(c.res + 16, 4, c.usp, w + ": USP unchanged");
    }
    uint32_t end = apc;
    bra_self();
    if (apc > PROG_LIMIT) bank_error("program beyond PROG_LIMIT");
    return end;
}
static void check_cases() {
    for (size_t k = 0; k < cases.size(); k++) {
        const IrqCase& c = cases[k];
        std::string w = "case " + std::to_string(k) + " (" + c.name + ")";
        check_eq(w + ": IACK cycles", 1, (uint64_t)n_iack[k]);
        if (n_iack[k] >= 1) {
            const Cyc& a = first_iack[k];
            check_eq(w + ": IACK address (A23-A4 at 1, A3-A1 = level)", 0xFFFFF0u | ((uint32_t)c.exp_level << 1), a.addr);
            check_eq(w + ": IACK as a read", 1, a.rw ? 1 : 0);
            check_eq(w + ": IACK with LDS asserted", 1, a.lds ? 1 : 0);
        }
        int n = 0; bool ok = true; int pchi = -1, pclo = -1;
        for (const Cyc& b : blog) {
            if (b.fc == 7 || b.rw || b.addr < c.top - 6 || b.addr >= c.top) continue;
            n++;
            if (b.fc != 5 || !b.uds || !b.lds) ok = false;
            if (b.addr == c.top - 4 && pchi < 0) pchi = b.data;
            if (b.addr == c.top - 2 && pclo < 0) pclo = b.data;
        }
        check_eq(w + ": writes in the frame", c.rte ? 5 : 3, (uint64_t)n);   // 3 from the exception (+2 from the MOVE.L A6,2(A7))
        check_eq(w + ": word-wide writes in the frame with FC=5", 1, ok ? 1 : 0);
        check_eq(w + ": upper PC stacked by the exception", c.wait_pc >> 16, (uint64_t)(uint32_t)pchi);
        check_eq(w + ": lower PC stacked by the exception", c.wait_pc & 0xFFFF, (uint64_t)(uint32_t)pclo);
    }
}

// ======================================================================================================================
// Programs
// ======================================================================================================================
static uint32_t build_p1() {
    mem_init(); exps.clear();
    cases.clear();
    for (int L = 1; L <= 7; L++) {
        uint16_t sr = (uint16_t)(0x2000 | ((L - 1) << 8) | ((L * 5) & 0x1F));
        cases.push_back(mk("level " + std::to_string(L) + " autovector, mask " + std::to_string(L - 1), sr, false, {{L, 0}}, R_VPA, 0, L, 24 + L, false, 0));
    }
    cases.push_back(mk("level 3 vectored 0x40", 0x2211, false, {{3, 0}}, R_VEC, 0x40, 3, 0x40, false, 0));
    cases.push_back(mk("level 5 vectored 0xFF", 0x210A, false, {{5, 0}}, R_VEC, 0xFF, 5, 0xFF, false, 0));
    cases.push_back(mk("level 2, uninitialized device: vector 15", 0x2004, false, {{2, 0}}, R_VEC, 0x0F, 2, 0x0F, false, 0));
    cases.push_back(mk("level 4 vectored 0x50 with RTE from supervisor", 0x2113, false, {{4, 0}}, R_VEC, 0x50, 4, 0x50, true, 0));
    cases.push_back(mk("level 6 vectored 0x51 from user mode with RTE to user mode", 0x001D, true, {{6, 0}}, R_VEC, 0x51, 6, 0x51, true, 0));
    uint32_t h = H_BASE;
    for (int v = 25; v <= 31; v++) { emit_handler(h, (uint8_t)v, false); h += 0x20; }
    emit_handler(h, 0x40, false); h += 0x20;
    emit_handler(h, 0xFF, false); h += 0x20;
    emit_handler(h, 0x0F, false); h += 0x20;
    emit_handler(h, 0x50, true);  h += 0x20;
    emit_handler(h, 0x51, true);
    return emit_cases();
}
// spurious interrupt: separate from program 1 because a wrong entry (bus error or autovector) stops the run
static uint32_t build_p2() {
    mem_init(); exps.clear();
    cases.clear();
    cases.push_back(mk("level 3 with BERR in the IACK: spurious interrupt, vector 24", 0x2008, false, {{3, 0}}, R_BERR, 0, 3, 24, false, 0));
    emit_handler(H_BASE, 24, false);
    return emit_cases();
}

// Program 3: nested interrupts. Handler of the level 3 case (vector 27): the standard three writes, then the inner case:
// A5 = its results area, A6 = its continuation, BRA.S * (the inner wait point, where the testbench applies levels 3, 2
// and 5); after the inner RTE: SR and A7 saved, the frame's PC rewritten with the program's continuation (kept in A4), RTE.
// Handler of the level 5 case (vector 29): the standard RTE variant.
static void emit_nested_handlers(size_t ko) {
    IrqCase& i = cases[ko + 1];
    org(H_BASE);
    ew(0x40D5);                          // MOVE SR,(A5)
    ew(0x2B4F); ew(0x0004);              // MOVE.L A7,4(A5)
    ew(0x3B7C); ew(27); ew(0x0008);      // MOVE.W #27,8(A5)
    ew(0x284E);                          // MOVEA.L A6,A4: the program's continuation, kept for the final RTE
    movea_l_imm(i.res, 5);               // A5 = results of the inner case
    uint32_t patch = apc + 2;
    movea_l_imm(0, 6);                   // A6 = continuation of the inner case (patched below)
    i.wait_pc = apc;
    bra_self();                          // inner wait: level 3 (equal) and level 2 (lower) not taken, level 5 taken
    i.cont_pc = apc; put32(patch, i.cont_pc);
    save_sr(i.res + 10); save_a(7, i.res + 12);   // after the inner RTE: SR with mask 3, A7 = SSP-6
    ew(0x2C4C);                          // MOVEA.L A4,A6
    ew(0x2F4E); ew(0x0002); ew(0x4E73);  // MOVE.L A6,2(A7) ; RTE: to the program's continuation with the frame's SR (mask 2)
    put32(4u * 27, H_BASE);
    emit_handler(H_BASE + 0x40, 29, true);
    std::string w = "case " + std::to_string(ko + 1) + " (" + i.name + ")";
    result(i.res, 2, 0x2510, w + ": SR in the inner handler (S=1 T=0 mask 5, X kept)");
    result(i.res + 4, 4, i.top - 6, w + ": A7 in the inner handler = SSP-12 (two frames)");
    result(i.res + 8, 2, 29, w + ": vector used = 29 (autovector level 5)");
    result(i.top - 6, 2, 0x2310, w + ": inner frame SR = SR of the level 3 handler (S=1 T=0 mask 3 X)");
    result(i.top - 4, 2, i.cont_pc >> 16, w + ": inner frame upper PC after the handler's rewrite");
    result(i.top - 2, 2, i.cont_pc & 0xFFFF, w + ": inner frame lower PC after the handler's rewrite");
    result(i.res + 10, 2, 0x2310, w + ": SR after the inner RTE = the level 3 handler's SR (mask back to 3)");
    result(i.res + 12, 4, i.top, w + ": A7 after the inner RTE = SSP-6");
}
static uint32_t build_p3() {
    mem_init(); exps.clear();
    cases.clear();
    cases.push_back(mk("level 3 autovector with mask 2, its handler interrupted by level 5 (nested)", 0x2210, false, {{3, 0}}, R_VPA, 0, 3, 27, true, 0));
    cases.push_back(mk("inside the level 3 handler: level 3 and level 2 held 600 clocks not taken, level 5 taken", 0x2310, false, {{3, 600}, {2, 600}, {5, 0}}, R_VPA, 0, 5, 29, true, 0));
    cases.back().inner = true;
    uint32_t end = emit_cases();
    emit_nested_handlers(0);
    return end;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    static const uint32_t WAITS[2] = {0, 3};
    for (int r = 0; r < 2; r++) {
        g_ctx = "irqmulti program 1 DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build_p1();
        run_cpu(WAITS[r], end, 300000);
        check_cases();
    }
    for (int r = 0; r < 2; r++) {
        g_ctx = "irqmulti program 2 (spurious) DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build_p2();
        run_cpu(WAITS[r], end, 100000);
        check_cases();
    }
    for (int r = 0; r < 2; r++) {
        g_ctx = "irqmulti program 3 (nested) DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build_p3();
        run_cpu(WAITS[r], end, 100000);
        check_cases();
    }
    dut->final();
    delete dut;
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    return n_fail == 0 ? 0 : 1;
}
