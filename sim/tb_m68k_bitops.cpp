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

// tb_m68k_bitops.cpp - BTST, BCHG, BCLR, BSET in dynamic form on a data register (Dx,Dy): long operand, bit number
// modulo 32 (PRM 4 BTST/BCHG/BCLR/BSET).
//
// What it tests (268 cases, each run twice: DTACK with zero wait states and DTACK delayed by 3 clocks):
//  - exhaustive: the 4 instructions x all 32 bit numbers x tested bit at 0 and at 1 (256 cases); the bit-number register
//    has bits 5 and up preloaded (0, 20, FFFFFFE0, 7ABCDE40: only the low 5 bits count, modulo 32);
//  - same register as bit number and destination (Bxxx Dn,Dn), 12 cases: the bit number is the value of the register
//    before the instruction, modulo 32;
//  - after EVERY instruction: the whole SR (Z = NOT tested bit before the operation; X N V C unchanged, with a rotating
//    input CCR that sets X N V C to 1 and to 0 and Z to the opposite of the expected value where possible), destination
//    register (BTST unchanged, BCHG inverts, BCLR clears, BSET sets only the indicated bit), bit-number register
//    unchanged; no memory write outside the results area.
// Not tested: static forms and memory operands (tb_m68k_bitext.cpp), the clocks.
//
// Expected values: bit_model() model from the Motorola semantics (PRM 4 BTST/BCHG/BCLR/BSET: "Z: set if the bit tested
// is zero, cleared otherwise; X N V C: not affected"; destination Dn: bit number modulo 32).

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
static constexpr uint32_t PROG_LIMIT = 0x7000;    // code lies below; above: unexpected-exception handlers
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
static constexpr int DN = 0, AN = 1, X7 = 7;
static constexpr int ABSW = 0, IMM = 4;
// MOVE (PRM 4 MOVE): 00 ss RRR MMM mmm rrr, ss = 01 byte, 11 word, 10 long
static uint16_t op_move(int sz, int dm, int dr, int sm, int sr) { int s = sz == 1 ? 1 : (sz == 2 ? 3 : 2); return (uint16_t)((s << 12) | (dr << 9) | (dm << 6) | (sm << 3) | sr); }
static void chk_absw(uint32_t a) { if (a >= 0x8000) bank_error("abs.W address not positive 0x" + hx(a)); }
static void move_l_imm_d(uint32_t v, int d) { ew(op_move(4, DN, d, X7, IMM)); el(v); }       // MOVE.L #imm,Dn
static void movea_l_imm(uint32_t v, int a) { ew(op_move(4, AN, a, X7, IMM)); el(v); }        // MOVEA.L #imm,An
static void move_ccr(uint8_t c) { ew(0x44FC); ew(c); }                                          // MOVE #imm,CCR
static void move_sr(uint16_t s) { ew(0x46FC); ew(s); }                                          // MOVE #imm,SR
static void save_sr(uint32_t a) { chk_absw(a); ew(0x40F8); ew((uint16_t)a); }                  // MOVE SR,abs.W
static void save_d(int d, uint32_t a) { chk_absw(a); ew(op_move(4, X7, ABSW, DN, d)); ew((uint16_t)a); }   // MOVE.L Dn,abs.W
static void bra_self() { ew(0x60FE); }                                                          // BRA.S *

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

// ======================================================================================================================
// Bit operations, dynamic form (PRM 4): 0000 rrr 1tt eeeeee, tt = 00 BTST, 01 BCHG, 10 BCLR, 11 BSET; rrr = bit-number
// register.
// ======================================================================================================================
static const char* const BN[4] = {"BTST", "BCHG", "BCLR", "BSET"};
static uint16_t op_bit_dyn(int op, int dbit, int dst) { return (uint16_t)(0x0100 | (dbit << 9) | (op << 6) | dst); }   // dst = Dn (mode 0)
// Model (PRM 4 BTST/BCHG/BCLR/BSET, destination Dn): n = bit number modulo 32; Z = NOT bit n before; then BTST no
// change, BCHG inverts, BCLR clears, BSET sets bit n; X N V C unchanged.
struct BitOut { uint32_t v; uint8_t ccr; };
static BitOut bit_model(int op, uint32_t v, uint32_t bitnum, uint8_t ccr_in) {
    uint32_t m = 1u << (bitnum & 31u);
    uint8_t ccr = (uint8_t)((ccr_in & 0x1B) | ((v & m) ? 0 : 0x04));
    uint32_t r = (op == 0) ? v : (op == 1) ? (v ^ m) : (op == 2) ? (v & ~m) : (v | m);
    return {r, ccr};
}
static const uint32_t HI_BITS[4] = {0x00000000u, 0x00000020u, 0xFFFFFFE0u, 0x7ABCDE40u};   // bits 5-31 of the bit number (ignored)
static const uint8_t CCRS[6] = {0x1B, 0x04, 0x1F, 0x00, 0x0A, 0x15};

static uint32_t build() {
    mem_init(); exps.clear();
    org(PROG_BASE);
    move_sr(0x2700);
    movea_l_imm(0x3F00, 7);
    uint32_t k = 0;
    // exhaustive: 4 instructions x 32 bit numbers x bit at 0 and at 1
    for (int op = 0; op < 4; op++)
        for (int n = 0; n < 32; n++)
            for (int b = 0; b < 2; b++, k++) {
                int dv = (int)(k % 8), db = (int)((k + 3) % 8);
                uint32_t base = 0x9C3E71A5u * (k + 1) + 0x6B;
                uint32_t v = b ? (base | (1u << n)) : (base & ~(1u << n));
                uint32_t bn = (uint32_t)n | HI_BITS[k % 4];
                uint8_t ccr = CCRS[k % 6];
                uint32_t res = RES_BASE + 12 * k;
                move_l_imm_d(v, dv);
                move_l_imm_d(bn, db);
                move_ccr(ccr);
                ew(op_bit_dyn(op, db, dv));
                save_sr(res);
                save_d(dv, res + 2);
                save_d(db, res + 6);
                BitOut o = bit_model(op, v, bn, ccr);
                std::string w = "case " + std::to_string(k) + " " + BN[op] + " D" + std::to_string(db) + ",D" + std::to_string(dv)
                              + " (bit " + std::to_string(n) + ", number " + hx(bn) + ", value " + hx(v) + ")";
                result(res, 2, 0x2700u | o.ccr, w + ": SR after (Z = NOT bit, X N V C unchanged)");
                result(res + 2, 4, o.v, w + ": destination");
                result(res + 6, 4, bn, w + ": bit-number register unchanged");
            }
    // same register: the bit number is the value of the register before the instruction
    static const uint32_t SAME[3] = {0x00000005u, 0x80000021u, 0xFFFFFFFFu};
    for (int op = 0; op < 4; op++)
        for (int s = 0; s < 3; s++, k++) {
            int dv = (int)(k % 8);
            uint8_t ccr = CCRS[k % 6];
            uint32_t res = RES_BASE + 12 * k;
            move_l_imm_d(SAME[s], dv);
            move_ccr(ccr);
            ew(op_bit_dyn(op, dv, dv));
            save_sr(res);
            save_d(dv, res + 2);
            BitOut o = bit_model(op, SAME[s], SAME[s], ccr);
            std::string w = "case " + std::to_string(k) + " " + BN[op] + " D" + std::to_string(dv) + ",D" + std::to_string(dv) + " (value " + hx(SAME[s]) + ")";
            result(res, 2, 0x2700u | o.ccr, w + ": SR after");
            result(res + 2, 4, o.v, w + ": register");
        }
    uint32_t end = apc;
    bra_self();
    if (apc > PROG_LIMIT) bank_error("program beyond PROG_LIMIT");
    if (RES_BASE + 12 * k > 0x3F00) bank_error("results area too large");
    return end;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    // self-check of the model with hand-computed values
    g_ctx = "bitops model self-check";
    { BitOut o = bit_model(0, 0x00000005, 1, 0x1B); check_eq("BTST bit 1 of 5: value", 0x00000005, o.v); check_eq("BTST bit 1 of 5: CCR (Z=1, X N V C kept)", 0x1F, o.ccr); }
    { BitOut o = bit_model(1, 0x80000000, 0xFFFFFFFF, 0x04); check_eq("BCHG bit 31 (number FFFFFFFF): value", 0x00000000, o.v); check_eq("BCHG bit 31: CCR (Z=0)", 0x00, o.ccr); }
    { BitOut o = bit_model(2, 0xFFFFFFFF, 0x00000020, 0x00); check_eq("BCLR number 32 = bit 0: value", 0xFFFFFFFE, o.v); check_eq("BCLR bit 0: CCR", 0x00, o.ccr); }
    { BitOut o = bit_model(3, 0x00000005, 0x00000005, 0x15); check_eq("BSET D,D with D=5: value", 0x00000025, o.v); check_eq("BSET D,D with D=5: CCR (Z=1)", 0x15, o.ccr); }
    { BitOut o = bit_model(1, 0x80000021, 0x80000021, 0x0A); check_eq("BCHG D,D with D=80000021: value (bit 1)", 0x80000023, o.v); check_eq("BCHG D,D: CCR (Z=1)", 0x0E, o.ccr); }
    static const uint32_t WAITS[2] = {0, 3};
    for (int r = 0; r < 2; r++) {
        g_ctx = "bitops DTACK+" + std::to_string(WAITS[r]);
        uint32_t end = build();
        run_cpu(WAITS[r], end, 1500000);
    }
    dut->final();
    delete dut;
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    return n_fail == 0 ? 0 : 1;
}
