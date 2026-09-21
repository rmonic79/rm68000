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

// tb_m68k_irq.cpp - hardware interrupts: mask, level 7 non-maskable and edge-sensitive, IACK cycle, autovector.
//
// What it tests (7 cases, each run twice: DTACK with zero wait states and DTACK delayed by 3 clocks). In every case the
// program sets SSP, the case area (A5) and the continuation (A6), writes the waiting SR (mask, mode, CCR) and spins on
// a BRA.S *; the testbench raises the IPL according to a script only after seeing that BRA fetched three times:
//  - levels 1, 2, 3 with mask 3 held 600 clocks each: ignored (MC68000UM 6.3.2: requests of a level lower than or
//    equal to the mask are not taken); then level 4: taken;
//  - level 7 with mask 7: taken (non-maskable) and HELD high for 3000 clocks: no second entry, not even when the
//    program rewrites the SR (MC68000UM 6.3.2: level 7 generates an interrupt on every transition from a level lower
//    than 7); then released and raised again: taken again;
//  - level 6 from user mode with mask 0: frame on the SSP, S = 1 in the handler, USP unchanged;
//  - level 6 with mask 6 held 600 clocks: ignored; then 7: taken;
//  - level 1 with mask 0: taken;
//  - T = 1 with an interrupt pending (MC68000UM 6.2.3, Multiple Exceptions: "the trace exception has priority and is
//    processed first. Before instruction execution resumes, however, the interrupt exception is also processed, and
//    instruction processing finally commences in the interrupt handler routine"): the traced instruction is a DIVU
//    and the testbench raises level 5 at the read of its divisor, at least 76 clocks before its end. Checked: the trace
//    exception first (frame with T = 1 and the PC after the DIVU, written before the IACK), then the interrupt
//    exception before any instruction of the trace handler (frame below it with S = 1, T = 0, the mask unchanged and
//    PC = the trace handler); the interrupt handler runs first and finds the trace handler's marker cell untouched;
//    its RTE enters the trace handler (SR of the interrupt frame, A7 = SSP-6), which clears T in its frame and
//    returns after the DIVU (SR with T = 0, A7 = SSP).
// For every case:
//  - IACK cycle (MC68000UM 5 and 6.3.2): exactly one; FC = 7 (CPU space), A23-A4 all at 1 and A3-A1 = level served,
//    R/W = read, LDS asserted; testbench response with VPA (autovector);
//  - vector: 24 + level (MC68000UM 6: autovectors 25-31), recognized by a distinct handler per vector that writes its
//    own number; every other vector leads to an unexpected-exception BRA.S *;
//  - frame read in full (waiting SR, upper PC, lower PC = address of the interrupted BRA.S *, which is also its
//    destination: the interrupt is taken at an instruction boundary, MC68000UM 6.3.2), cells preloaded with the
//    complement of the expected value; exactly three writes, word-wide, FC = 5;
//  - in the handler: SR with S = 1, T = 0, mask = level served, CCR unchanged (MC68000UM 6.3.2: "the priority level is
//    set to the level of the interrupt being acknowledged"); A7 = SSP - 6.
// Vectored interrupts, uninitialized vector, spurious interrupt, RTE and nested interrupts are in tb_m68k_irq_multi.cpp.
// The manual fixes no clock for the sampling of IPL within an instruction (MC68000UM 6.3.2): the level is raised while
// the processor spins on the BRA.S * (or during the traced DIVU) and held until the IACK; the boundary at which it is
// taken is checked.

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
    bool trace;          // T = 1 with the interrupt pending: traced DIVU, trace exception then interrupt exception (UM 6.2.3)
    uint32_t trig_rd;    // if not 0: the IPL script starts at the first data read of this address instead of the third fetch of wait_pc
    uint32_t wait_pc, cont_pc, res, top, usp;
};
static IrqCase mk(const std::string& name, uint16_t sr_wait, bool user, std::vector<IrqStep> steps, int resp, uint8_t vecnum,
                  int exp_level, int exp_vec, bool rte, uint64_t hold_after) {
    IrqCase c; c.name = name; c.sr_wait = sr_wait; c.user = user; c.steps = steps; c.resp = resp; c.vecnum = vecnum;
    c.exp_level = exp_level; c.exp_vec = exp_vec; c.rte = rte; c.hold_after = hold_after;
    c.trace = false; c.trig_rd = 0;
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
// acknowledge cycle with VPA: VMA asserted (M6800 cycle, UM App. B). The relations with E are checked in tb_m68k_bus_fsm.cpp with
// alternating phases, where E is the chip's one; in this single-phase bench e_cnt advances at every tick (docs/m6800-bus.md 4.4).
struct VpaObs { bool vma = false; };
static VpaObs vo; static std::vector<VpaObs> first_vpa;

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
    if (n_iack[cur] == 1) { first_iack[cur] = cyc; first_vpa[cur] = vo; }
    vo = VpaObs{};
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
        for (size_t k = 0; k < cases.size(); k++) if (cases[k].trig_rd == 0 && cyc.addr == cases[k].wait_pc) wait_seen[k]++;   // trig_rd: the fetch of the instruction does not start the script
    } else if (cyc.fc == 5 || cyc.fc == 1) {   // data read: trigger of a case with trig_rd (the operand read of its traced instruction)
        for (size_t k = 0; k < cases.size(); k++) if (cases[k].trig_rd != 0 && cyc.addr == cases[k].trig_rd) wait_seen[k]++;
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
            if (resp == R_VPA) {
                dut->vpa_n = 0; dut->dtack_n = 1; dut->berr_n = 1;
                if (!dut->vma_n) vo.vma = true;
            }
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
        if (wait_seen[cur] >= (c.trig_rd != 0 ? 1 : 3)) { phase = 1; step = 0; t0 = nclk; }   // trig_rd: at the first read of the operand
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
    n_iack.assign(cases.size(), 0); wait_seen.assign(cases.size(), 0); first_iack.assign(cases.size(), Cyc{}); first_vpa.assign(cases.size(), VpaObs{}); vo = VpaObs{};
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
//         [user without RTE: MOVE USP,A0 ; MOVE.L A0,res+16] ; MOVE #$2700,SR
// ======================================================================================================================
static constexpr uint32_t H_BASE = 0x6800;
static constexpr uint32_t H_I5 = 0x6900;   // trace case: interrupt handler (vector 29, level 5), returns with RTE into the trace handler
static constexpr uint32_t H_T  = 0x6940;   // trace case: trace handler (vector 9, set by the program), clears T in its frame and returns
static constexpr uint32_t MARK = 0x0FF0;   // trace case: divisor of the traced DIVU; the read of this word starts the IPL script
static void emit_handler(uint32_t at, uint8_t vec, bool rte) {
    org(at);
    ew(0x40D5);                          // MOVE SR,(A5)
    ew(0x2B4F); ew(0x0004);              // MOVE.L A7,4(A5)
    ew(0x3B7C); ew(vec); ew(0x0008);     // MOVE.W #vec,8(A5)
    if (rte) { ew(0x2F4E); ew(0x0002); ew(0x4E73); }   // MOVE.L A6,2(A7) ; RTE
    else ew(0x4ED6);                     // JMP (A6)
    put32(4u * vec, at);
}
// Trace case (UM 6.2.3). Interrupt handler (vector 29): the standard three writes, a copy of the trace handler's marker cell
// (still the preload if that handler has not run), RTE into the trace handler. Trace handler (vector 9): marker, SR and A7
// saved, T cleared in the stacked SR (ANDI.W #$7FFF,(A7)), RTE after the traced DIVU.
static void emit_trace_handlers() {
    org(H_I5);
    ew(0x40D5);                          // MOVE SR,(A5)
    ew(0x2B4F); ew(0x0004);              // MOVE.L A7,4(A5)
    ew(0x3B7C); ew(29); ew(0x0008);      // MOVE.W #29,8(A5)
    ew(0x3B6D); ew(0x0014); ew(0x0016);  // MOVE.W 20(A5),22(A5): the trace handler's marker cell as the interrupt handler finds it
    ew(0x4E73);                          // RTE: into the trace handler (its address is in the interrupt frame)
    put32(4u * 29, H_I5);
    org(H_T);
    ew(0x3B7C); ew(0x5A5A); ew(0x0014);  // MOVE.W #$5A5A,20(A5): marker, the trace handler ran
    ew(0x40ED); ew(0x0018);              // MOVE SR,24(A5)
    ew(0x2B4F); ew(0x001A);              // MOVE.L A7,26(A5)
    ew(0x0257); ew(0x7FFF);              // ANDI.W #$7FFF,(A7): T cleared in the stacked SR
    ew(0x4E73);                          // RTE: after the traced DIVU, with T = 0
}
static uint32_t emit_cases() {
    org(PROG_BASE);
    move_sr(0x2700);
    for (size_t k = 0; k < cases.size(); k++) {
        IrqCase& c = cases[k];
        if (c.user && c.rte && k + 1 != cases.size()) bank_error("user case with RTE is not the last one");
        c.res = RES_BASE + 0x20 * (uint32_t)k; c.top = 0x3100 + 0x40 * (uint32_t)k; c.usp = 0x3E00 + 0x10 * (uint32_t)k;
        if (c.trace) {   // T = 1 with an interrupt pending at the end of the traced instruction (UM 6.2.3)
            if (k + 1 != cases.size()) bank_error("the trace case is not the last one (its program redirects vector 9)");
            movea_l_imm(c.top, 7);
            movea_l_imm(c.res, 5);
            uint32_t patch = apc + 2;
            movea_l_imm(0, 6);
            allow(0x24, 4); ew(0x21FC); el(H_T); ew(0x0024);   // MOVE.L #H_T,($24).W: vector 9 -> the trace handler
            ew(0x203C); el(0x00001000);                        // MOVE.L #$1000,D0: dividend
            put16(MARK, 0x0010); c.trig_rd = MARK;             // divisor; its read starts the IPL script (level 5, held until the IACK)
            move_sr(c.sr_wait);                                // T = 1, S = 1, mask 3, X = 1: not traced itself (T was 0 at its start)
            c.wait_pc = apc;
            ew(0x80F8); ew((uint16_t)MARK);                    // DIVU MARK.W,D0, traced: $1000 / $10 = $100, no overflow, at least 76 clocks after the read
            c.cont_pc = apc;
            put32(patch, c.cont_pc);
            save_sr(c.res + 10); save_a(7, c.res + 12);        // after both returns
            move_sr(0x2700);
            std::string w = "case " + std::to_string(k) + " (" + c.name + ")";
            result(c.res, 2, 0x2510, w + ": SR in the interrupt handler (S=1 T=0 mask 5; X kept by DIVU, NZVC cleared by it)");
            result(c.res + 4, 4, c.top - 12, w + ": A7 in the interrupt handler = SSP-12 (trace frame, then interrupt frame)");
            result(c.res + 8, 2, 29, w + ": vector used = 29 (autovector level 5)");
            result(c.res + 0x16, 2, 0xA5A5, w + ": the trace handler had not run when the interrupt handler started (its marker cell still held the preload)");
            result(c.res + 0x14, 2, 0x5A5A, w + ": the trace handler ran after the interrupt handler's RTE");
            result(c.res + 0x18, 2, 0x2310, w + ": SR in the trace handler = SR of the interrupt frame (S=1 T=0 mask 3 X)");
            result(c.res + 0x1A, 4, c.top - 6, w + ": A7 in the trace handler = SSP-6 (the interrupt frame popped)");
            result(c.res + 10, 2, 0x2310, w + ": SR after both returns (T cleared by the trace handler in its frame)");
            result(c.res + 12, 4, c.top, w + ": A7 after both returns = SSP");
            result(c.top - 6, 2, 0x2310, w + ": trace frame SR at the end (T cleared in place by the trace handler; stacked as 0xA310, checked from the bus log)");
            result(c.top - 4, 2, c.cont_pc >> 16, w + ": trace frame upper PC = instruction after the traced DIVU");
            result(c.top - 2, 2, c.cont_pc & 0xFFFF, w + ": trace frame lower PC = instruction after the traced DIVU");
            result(c.top - 12, 2, 0x2310, w + ": interrupt frame SR = SR at the entry into the trace handler (S=1 T=0, mask 3 unchanged, X)");
            result(c.top - 10, 2, H_T >> 16, w + ": interrupt frame upper PC = the trace handler");
            result(c.top - 8, 2, H_T & 0xFFFF, w + ": interrupt frame lower PC = the trace handler (no instruction of it executed before the interrupt)");
            continue;
        }
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
            if (c.resp == R_VPA) check_eq(w + ": IACK with VPA: VMA asserted (M6800 cycle, UM App. B; its relations with E: tb_m68k_bus_fsm.cpp, two phases)", 1, first_vpa[k].vma ? 1 : 0);
        }
        if (c.trace && getenv("IRQDUMP")) {   // diagnosis: the cycles from the read of the divisor to 40 after the IACK
            size_t i0 = 0; for (size_t i = 0; i < blog.size(); i++) if (blog[i].rw && blog[i].addr == MARK) { i0 = i; break; }
            for (size_t i = i0; i < blog.size() && i < i0 + 60; i++) { const Cyc& b = blog[i];
                printf("DUMP #%zu t%llu a%06X fc%d %s u%d l%d d%04X\n", i, (unsigned long long)b.at, (unsigned)b.addr, b.fc, b.rw ? "R" : "W", b.uds, b.lds, b.data); }
        }
        if (c.trace) {   // two frames: trace at [top-6, top), interrupt at [top-12, top-6); the trace frame first, then the IACK
            int n = 0; bool ok = true; int sr1 = -1, pchi = -1, pclo = -1; int n_tr = 0; uint64_t last_tr = 0, first_int = ~0ull;
            for (const Cyc& b : blog) {
                if (b.fc == 7 || b.rw || b.addr < c.top - 12 || b.addr >= c.top) continue;
                n++;
                if (b.fc != 5 || !b.uds || !b.lds) ok = false;
                if (b.addr >= c.top - 6) { if (n_tr < 3) { n_tr++; last_tr = b.at; } }   // the three writes of the trace exception
                else if (b.at < first_int) first_int = b.at;                             // the interrupt frame
                if (b.addr == c.top - 6 && sr1 < 0) sr1 = b.data;
                if (b.addr == c.top - 4 && pchi < 0) pchi = b.data;
                if (b.addr == c.top - 2 && pclo < 0) pclo = b.data;
            }
            check_eq(w + ": writes in the two frames", 7, (uint64_t)n);   // 3 trace frame + 3 interrupt frame + 1 ANDI.W of the trace handler
            check_eq(w + ": word-wide writes in the frames with FC=5", 1, ok ? 1 : 0);
            check_eq(w + ": SR stacked by the trace exception (T=1 S=1 mask 3 X)", 0xA310, (uint64_t)(uint32_t)sr1);
            check_eq(w + ": upper PC stacked by the trace exception", c.cont_pc >> 16, (uint64_t)(uint32_t)pchi);
            check_eq(w + ": lower PC stacked by the trace exception = instruction after the DIVU", c.cont_pc & 0xFFFF, (uint64_t)(uint32_t)pclo);
            check_eq(w + ": the trace frame is written before the interrupt frame (UM 6.2.3: the trace exception first)", 1, (n_tr == 3 && last_tr < first_int) ? 1 : 0);
            if (n_iack[k] >= 1) check_eq(w + ": the IACK follows the trace frame", 1, (n_tr == 3 && last_tr < first_iack[k].at) ? 1 : 0);
            continue;
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
// Cases
// ======================================================================================================================
static void define_cases() {
    cases.clear();
    cases.push_back(mk("levels 1 2 3 with mask 3 ignored, then 4 taken", 0x2315, false, {{1, 600}, {2, 600}, {3, 600}, {4, 0}}, R_VPA, 0, 4, 28, false, 0));
    cases.push_back(mk("level 7 with mask 7, held: taken only once", 0x270A, false, {{7, 0}}, R_VPA, 0, 7, 31, false, 3000));
    cases.push_back(mk("level 7 released and raised again: taken again", 0x2711, false, {{0, 300}, {7, 0}}, R_VPA, 0, 7, 31, false, 0));
    cases.push_back(mk("level 6 from user mode with mask 0", 0x0004, true, {{6, 0}}, R_VPA, 0, 6, 30, false, 0));
    cases.push_back(mk("level 6 with mask 6 ignored, then 7 taken", 0x261F, false, {{6, 600}, {7, 0}}, R_VPA, 0, 7, 31, false, 0));
    cases.push_back(mk("level 1 with mask 0", 0x2008, false, {{1, 0}}, R_VPA, 0, 1, 25, false, 0));
    cases.push_back(mk("T=1 with level 5 pending at the end of a traced DIVU: trace exception first, then the interrupt (UM 6.2.3)", 0xA310, false, {{5, 0}}, R_VPA, 0, 5, 29, false, 0));
    cases.back().trace = true;
}
static uint32_t build() {
    mem_init(); exps.clear();
    define_cases();
    for (int v = 25; v <= 31; v++) if (v != 29) emit_handler(H_BASE + 0x20 * (uint32_t)(v - 25), (uint8_t)v, false);   // 29: the trace case's handler
    emit_trace_handlers();
    return emit_cases();
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    static const uint32_t WAITS[2] = {0, 3};
    for (int r = 0; r < 2; r++) {
        g_ctx = "irq DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build();
        run_cpu(WAITS[r], end, 300000);
        check_cases();
    }
    dut->final();
    delete dut;
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    return n_fail == 0 ? 0 : 1;
}
