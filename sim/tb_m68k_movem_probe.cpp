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

// tb_m68k_movem_probe.cpp (target movemx) - MOVEM: the corner cases, each checked against the model.
//
// What it tests (16 cases, each run twice: DTACK with zero wait states and DTACK delayed by 3 clocks):
//  - sparse lists with an An inside (D0/D2/A0) to -(A2) and from (A2)+;
//  - base register in the list with -(An), .L and .W, and with the list made of the base register only: the 68000 writes the
//    INITIAL value of the register (PRM 4 MOVEM: "the MC68000 and MC68010 write the initial register value (not
//    decremented)"); final An = address of the last one written;
//  - base register in the list with (An)+, .L and .W, and list made of the base only: the value read is ignored and the
//    register holds the incremented address (PRM 4 MOVEM: "the memory value is ignored and the register is written with
//    the postincremented effective address");
//  - base register in the list with a control mode: to memory its value is written, from memory it is
//    loaded like the others (PRM 4 MOVEM, no exception for the control modes);
//  - A7 in the list: D0/A7 in -(A7) (A7 base: initial value), A7 in -(A7) word-sized, A6/A7 in (A1) (A7 = SSP),
//    (A1)+ in D1/A7 (A7 loaded), mask FFFF in -(A1) (base and A7 included);
//  - .W from memory into the An: sign extension (8xxx -> FFFF8xxx);
//  - after each case SR and all 16 registers saved and compared with the model; target memory preloaded with the
//    complement of the expected value, no write outside the expected cells; bus as in tb_m68k_movem.cpp (number of
//    writes or reads in the area = words transferred, word-sized, FC = 5, order of the registers in memory).
// What it does NOT test: the general MOVEM modes (tb_m68k_movem.cpp), the clocks.
//
// Expectations: model from the Motorola semantics (PRM 4 MOVEM), the same engine as tb_m68k_movem.cpp.

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

// ======================================================================================================================
// MOVEM engine: registers -> memory 0100 1000 1s ea, memory -> registers 0100 1100 1s ea, then the mask, then
// the EA extension (PRM 4 MOVEM). Registers numbered 0-7 = D0-D7, 8-15 = A0-A7; `list` = bit r for register r,
// the emitted mask is reversed with -(An).
// ======================================================================================================================
enum { M_PRE, M_POST, M_IND, M_D16, M_IDX, M_ABSW, M_ABSL, M_PCD16, M_PCIDX };
static const char* const MODEN[] = {"-(An)", "(An)+", "(An)", "d16(An)", "d8(An,Xn)", "abs.W", "abs.L", "d16(PC)", "d8(PC,Xn)"};
static const char* const RN[16] = {"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7"};
// disp = d16 or d8; xreg/xlong/xval = index register (0-15), W/L, value (for d8(PC,Xn) the value is computed by the testbench)
struct MvCase { bool to_reg; bool lng; int mode; int an; uint16_t list; int disp; int xreg; bool xlong; uint32_t xval; const char* why; };
struct MvInfo { std::string name; bool to_reg; bool pre; uint32_t lo, hi; uint32_t sz; int nwords; uint8_t fc; };
static std::vector<MvInfo> minfo;
static uint32_t R[16];

static uint32_t regval(size_t k, int r) { return (0x11111111u * (uint32_t)(r + 1)) ^ (0x00010203u * (uint32_t)(k + 1)) ^ 0x5A00A500u; }
static uint32_t memval(size_t k, int i, uint32_t sz) {
    if (sz == 4) return 0xC0000000u ^ ((uint32_t)k << 20) ^ ((uint32_t)i * 0x01020305u) ^ 0x0F0F0F0Fu;
    return ((i & 1) ? 0x8000u : 0x0000u) | (((uint32_t)k * 0x0137u + (uint32_t)i * 0x00B1u + 0x0101u) & 0x7FFFu);
}
static uint32_t sext16(uint32_t v) { return (uint32_t)(int32_t)(int16_t)(uint16_t)v; }
static uint16_t rev16(uint16_t m) { uint16_t r = 0; for (int i = 0; i < 16; i++) if (m & (1u << i)) r = (uint16_t)(r | (1u << (15 - i))); return r; }
static int popc(uint16_t m) { int n = 0; for (int i = 0; i < 16; i++) n += (m >> i) & 1; return n; }
static void set_reg(int r, uint32_t v) { if (r < 8) move_l_imm_d(v, r); else movea_l_imm(v, r - 8); }
static void save_reg(int r, uint32_t a) { if (r < 8) save_d(r, a); else save_a(r - 8, a); }

static void emit_case(size_t k, const MvCase& c, int& pc_area) {
    uint32_t sz = c.lng ? 4 : 2;
    uint32_t res = RES_BASE + 0x50 * (uint32_t)k;
    uint32_t rb = (c.mode == M_ABSL) ? 0xFF8000 + 0x80 * (uint32_t)k
                : (c.mode == M_PCD16 || c.mode == M_PCIDX) ? 0x7000 + 0x80 * (uint32_t)(pc_area++)
                : 0x2000 + 0x80 * (uint32_t)k;
    uint32_t eaddr = rb + 0x10;                        // EA of the control modes and (An)+
    uint8_t ccr = (uint8_t)((k * 7 + 3) & 0x1F);
    std::string w = "case " + std::to_string(k) + " MOVEM." + (c.lng ? "L " : "W ") + (c.to_reg ? std::string(MODEN[c.mode]) + ",list" : "list," + std::string(MODEN[c.mode]))
                  + " (" + c.why + ")";
    for (int r = 0; r < 16; r++) R[r] = regval(k, r);
    R[15] = 0x3F00;
    uint32_t P = apc + 16 * 6 + 4;    // address of the MOVEM: after 16 loads of 6 bytes and MOVE #,CCR
    uint32_t ext_pc = P + 4;           // address of the EA extension word (after opcode and mask)
    int b = 8 + c.an;
    switch (c.mode) {
        case M_PRE:  R[b] = rb + 0x50; break;
        case M_POST: case M_IND: R[b] = eaddr; break;
        case M_D16:  R[b] = eaddr - (uint32_t)c.disp; break;
        case M_IDX:  { R[c.xreg] = c.xval; uint32_t ix = c.xlong ? c.xval : sext16(c.xval); R[b] = eaddr - ix - (uint32_t)c.disp; break; }
        case M_PCIDX: { uint32_t ix = eaddr - ext_pc - (uint32_t)c.disp; R[c.xreg] = c.xlong ? ix : (0xBEEF0000u | (ix & 0xFFFFu)); break; }
        default: break;
    }
    for (int r = 0; r < 16; r++) set_reg(r, R[r]);
    move_ccr(ccr);
    if (apc != P) bank_error(w + ": address of the MOVEM");
    uint16_t mask = (c.mode == M_PRE) ? rev16(c.list) : c.list;
    uint16_t eaf = 0;
    switch (c.mode) {
        case M_PRE: eaf = ea(PRE, c.an); break;
        case M_POST: eaf = ea(POST, c.an); break;
        case M_IND: eaf = ea(IND, c.an); break;
        case M_D16: eaf = ea(D16, c.an); break;
        case M_IDX: eaf = ea(IDX, c.an); break;
        case M_ABSW: eaf = ea(X7, ABSW); break;
        case M_ABSL: eaf = ea(X7, ABSL); break;
        case M_PCD16: eaf = ea(X7, PCD16); break;
        case M_PCIDX: eaf = ea(X7, PCIDX); break;
        default: break;
    }
    ew((uint16_t)((c.to_reg ? 0x4C80 : 0x4880) | (c.lng ? 0x40 : 0) | eaf));
    ew(mask);
    if (apc != ext_pc) bank_error(w + ": address of the extension");
    switch (c.mode) {
        case M_D16: ew((uint16_t)c.disp); break;
        case M_IDX: case M_PCIDX: ew(ext_idx(c.xreg >= 8 ? 1 : 0, c.xreg & 7, c.xlong ? 1 : 0, c.disp)); break;
        case M_ABSW: chk_absw(eaddr); ew((uint16_t)eaddr); break;
        case M_ABSL: el(eaddr); break;
        case M_PCD16: ew((uint16_t)(eaddr - ext_pc)); break;
        default: break;
    }
    // model (PRM 4 MOVEM)
    int n = popc(c.list);
    uint32_t lo = 0, hi = 0;
    if (!c.to_reg) {
        if (c.mode == M_PRE) {
            uint32_t a = R[b], init = R[b];
            hi = init;
            for (int r = 15; r >= 0; r--) if (c.list & (1u << r)) {
                a -= sz;
                uint32_t v = (r == b) ? init : R[r];   // MC68000: the base register in the list is written with the initial value
                if (sz == 4) result(a, 4, v, w + ": memory of " + RN[r]);
                else result(a, 2, v & 0xFFFFu, w + ": memory of " + RN[r] + " (low word)");
            }
            lo = a; R[b] = a;                          // final An = address of the last one written
        } else {
            uint32_t a = eaddr; lo = a;
            for (int r = 0; r < 16; r++) if (c.list & (1u << r)) {
                if (sz == 4) result(a, 4, R[r], w + ": memory of " + RN[r]);
                else result(a, 2, R[r] & 0xFFFFu, w + ": memory of " + RN[r] + " (low word)");
                a += sz;
            }
            hi = a;
        }
    } else {
        uint32_t a = (c.mode == M_POST) ? R[b] : eaddr; lo = a;
        int i = 0;
        uint32_t nv[16] = {0}; bool ld[16] = {false};
        for (int r = 0; r < 16; r++) if (c.list & (1u << r)) {
            uint32_t v = memval(k, i++, sz);
            if (sz == 4) put32(a, v); else put16(a, (uint16_t)v);
            nv[r] = (sz == 4) ? v : sext16(v); ld[r] = true;   // .W: sign extension to 32 bits, also in the An
            a += sz;
        }
        hi = a;
        for (int r = 0; r < 16; r++) if (ld[r]) R[r] = nv[r];
        if (c.mode == M_POST) R[b] = a;                // final An = address after the last one read (wins over the value read)
    }
    save_sr(res);
    result(res, 2, 0x2700u | ccr, w + ": SR after (CCR not touched)");
    for (int r = 0; r < 16; r++) { save_reg(r, res + 2 + 4 * (uint32_t)r); result(res + 2 + 4 * (uint32_t)r, 4, R[r], w + ": " + RN[r] + " after"); }
    minfo.push_back({w, c.to_reg, c.mode == M_PRE, lo, hi, sz, n * (int)sz / 2, (uint8_t)((c.mode == M_PCD16 || c.mode == M_PCIDX) ? 6 : 5)});
}

static void check_movem_bus() {
    for (const MvInfo& m : minfo) {
        int nw = 0, nr = 0, last = -1; bool wok = true, rok = true, ord = true;
        for (const Cyc& c : blog) {
            if (c.fc == 7 || c.addr < m.lo || c.addr >= m.hi) continue;
            if (!c.rw) {
                nw++;
                if (c.fc != 5 || !c.uds || !c.lds) wok = false;
                int slot = (int)((c.addr - m.lo) / m.sz);
                if (slot != last) { if (last >= 0 && (m.pre ? slot >= last : slot <= last)) ord = false; last = slot; }
            } else {
                nr++;
                if (c.fc != m.fc || !c.uds || !c.lds) rok = false;
            }
        }
        if (!m.to_reg) {
            check_eq(m.name + ": writes in the area (one per word)", (uint64_t)m.nwords, (uint64_t)nw);
            check_eq(m.name + ": word writes with FC=5", 1, wok ? 1 : 0);
            check_eq(m.name + std::string(": order of the registers in memory (") + (m.pre ? "decreasing" : "increasing") + ")", 1, ord ? 1 : 0);
            check_eq(m.name + ": reads in the area", 0, (uint64_t)nr);
        } else {
            check_eq(m.name + ": reads in the area (one per word)", (uint64_t)m.nwords, (uint64_t)nr);
            check_eq(m.name + ": word reads with FC=" + std::to_string(m.fc), 1, rok ? 1 : 0);
            check_eq(m.name + ": writes in the area", 0, (uint64_t)nw);
        }
    }
}

// ======================================================================================================================
// Cases (list: bit r = register r; D0 = bit 0 ... A7 = bit 15)
// ======================================================================================================================
static const MvCase CASES[] = {
    {false, true,  M_PRE,  2, 0x0105, 0, 0, false, 0, "D0/D2/A0 in -(A2), sparse list with an An"},
    {true,  true,  M_POST, 2, 0x0105, 0, 0, false, 0, "(A2)+ in D0/D2/A0"},
    {false, true,  M_PRE,  2, 0x2402, 0, 0, false, 0, "D1/A2/A5 in -(A2): base in the list, initial value written"},
    {false, false, M_PRE,  3, 0x0808, 0, 0, false, 0, "D3/A3 in -(A3) word-sized: base in the list, initial low word"},
    {false, true,  M_PRE,  0, 0x0100, 0, 0, false, 0, "A0 in -(A0): base register only"},
    {true,  true,  M_POST, 3, 0x4810, 0, 0, false, 0, "(A3)+ in D4/A3/A6: base in the list, holds the incremented address"},
    {true,  false, M_POST, 4, 0x1020, 0, 0, false, 0, "(A4)+ in D5/A4 word-sized: base in the list"},
    {true,  false, M_POST, 6, 0x4000, 0, 0, false, 0, "(A6)+ in A6 word-sized: base register only"},
    {false, true,  M_D16,  2, 0x0401, 8, 0, false, 0, "D0/A2 in 8(A2): base in the list, control mode"},
    {true,  true,  M_IND,  4, 0x1004, 0, 0, false, 0, "(A4) in D2/A4: base loaded (control mode)"},
    {false, true,  M_PRE,  7, 0x8001, 0, 0, false, 0, "D0/A7 in -(A7): A7 base in the list, initial value"},
    {false, false, M_PRE,  7, 0x8000, 0, 0, false, 0, "A7 in -(A7) word-sized: A7 only"},
    {false, true,  M_IND,  1, 0xC000, 0, 0, false, 0, "A6/A7 in (A1): A7 = SSP"},
    {true,  true,  M_POST, 1, 0x8002, 0, 0, false, 0, "(A1)+ in D1/A7: A7 loaded"},
    {true,  false, M_POST, 5, 0x0300, 0, 0, false, 0, "(A5)+ in A0/A1 word-sized: sign extension in the An"},
    {false, true,  M_PRE,  1, 0xFFFF, 0, 0, false, 0, "D0-D7/A0-A7 in -(A1): mask FFFF, A1 base and A7 included"},
};
static constexpr size_t NCASES = sizeof(CASES) / sizeof(CASES[0]);

static uint32_t build() {
    mem_init(); exps.clear(); minfo.clear();
    org(PROG_BASE);
    move_sr(0x2700);
    int pc_area = 0;
    for (size_t k = 0; k < NCASES; k++) emit_case(k, CASES[k], pc_area);
    movea_l_imm(0x3F00, 7);
    uint32_t end = apc;
    bra_self();
    if (apc > PROG_LIMIT) bank_error("program beyond PROG_LIMIT");
    return end;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    // self-check, computed by hand: case 2 (D1/A2/A5 in -(A2), initial A2 = 0x2150): -(An) mask = 0x4024,
    // memory 0x214C = A5, 0x2148 = 0x2150 (initial A2, not 0x2148), 0x2144 = D1, final A2 0x2144
    g_ctx = "movemx self-check";
    check_eq("-(An) mask of D1/A2/A5", 0x4024, rev16(0x2402));
    {
        mem_init(); exps.clear(); minfo.clear(); org(PROG_BASE);
        int pa = 0;
        emit_case(2, CASES[2], pa);
        bool found = false;
        for (const Expect& e : exps) if (e.addr == 0x2148 && e.size == 4) { found = true; check_eq("case 2 model: memory of A2 = initial value", 0x2150, e.val); }
        check_eq("case 2 model: cell of A2 expected at 0x2148", 1, found ? 1 : 0);
        check_eq("case 2 model: final A2", 0x2144, R[10]);
    }
    static const uint32_t WAITS[2] = {0, 3};
    for (int r = 0; r < 2; r++) {
        g_ctx = "movemx DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build();
        run_cpu(WAITS[r], end, 400000);
        check_movem_bus();
    }
    dut->final();
    delete dut;
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    return n_fail == 0 ? 0 : 1;
}
