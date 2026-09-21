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

// tb_m68k_ea2.cpp - EA with an extension word at byte, word and long size: MOVE with d16(An) (positive and negative
// displacement), abs.W (positive and negative, sign-extended: $8xxx -> $FF8xxx) and abs.L (high word $0034), on the full
// processor (shim m68k_seq). Expected values by hand from M68000PRM ch. 4 (MOVE: destination <- source; N and Z from the
// data, V=0, C=0, X unchanged; byte/word Dn destination: only the low part) and sec. 2.2 (modes; order of the extension
// words: the source ones first) and from MC68000UM (bus: long = two words, the high part at the low address; even byte on
// UDS / D15-D8, odd byte on LDS / D7-D0).
// What it tests (46 cases):
//  - every mode (5) x {even byte, odd byte, word, long} x {Dn -> memory, memory -> Dn}: values with N=1, Z=1, maximum
//    positive and mixed patterns in rotation; byte memory with the neighbouring cells as sentinels and no write allowed
//    outside the target (a word write on a byte, on the wrong lane or at the wrong address fails);
//    Dn read with the upper part preloaded (it must stay); A1 in the absolute modes far from the data;
//  - 6 memory -> memory MOVEs with extensions on both sides (base A1 for the source, A2 for the destination,
//    different values, so a swapped base register is visible);
//  - after EVERY instruction SR saved (X N Z V C with a discriminating input CCR);
//  - bus: supervisor data FC on the operands, no write outside the expected areas; DTACK immediate and delayed by
//    3 clocks.
// NOT tested: MOVE with other modes (tb_m68k_ea, tb_m68k_mem), MOVEA, order of the two writes of a long (not
// specified by the manual for every mode), timing.

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <map>
#include <set>
#include <vector>
#include <string>

static Vm68k_seq* dut = nullptr;
static uint64_t t_ticks = 0;
double sc_time_stamp() { return (double)t_ticks; }

// ---------------------------------------------------------------- result
static int n_checks = 0, n_fails = 0;
static std::string run_tag;
static void check_cond(bool ok, const std::string& what, uint32_t exp, uint32_t got) {
    n_checks++;
    if (ok) return;
    n_fails++;
    if (n_fails <= 150) printf("FAIL: %s%s expected=0x%X read=0x%X\n", run_tag.c_str(), what.c_str(), (unsigned)exp, (unsigned)got);
}
static void check_eq(const std::string& what, uint32_t exp, uint32_t got) { check_cond(exp == got, what, exp, got); }
static std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "%X", (unsigned)v); return std::string(b); }

// ---------------------------------------------------------------- byte memory (24 bits); cells never written = NOP
static std::map<uint32_t, uint8_t> mem;
static uint8_t mrd8(uint32_t a) { auto it = mem.find(a & 0xFFFFFFu); return it != mem.end() ? it->second : (uint8_t)((a & 1u) ? 0x71 : 0x4E); }
static uint16_t mrd16(uint32_t a) { return (uint16_t)((mrd8(a) << 8) | mrd8(a + 1)); }
static uint32_t mrdn(uint32_t a, int n) { uint32_t v = 0; for (int i = 0; i < n; i++) v = (v << 8) | mrd8(a + (uint32_t)i); return v; }
static void mwr8(uint32_t a, uint8_t v) { mem[a & 0xFFFFFFu] = v; }
static void mwrn(uint32_t a, int n, uint32_t v) { for (int i = n - 1; i >= 0; i--) { mwr8(a + (uint32_t)i, (uint8_t)v); v >>= 8; } }
static void mwr16(uint32_t a, uint16_t v) { mwrn(a, 2, v); }
static constexpr uint8_t SENT = 0xEE;
static void fill(uint32_t a, uint32_t n, uint8_t v) { for (uint32_t i = 0; i < n; i++) mwr8(a + i, v); }

// ---------------------------------------------------------------- assembler (encodings from M68000PRM ch. 8)
static uint32_t asm_pc = 0;
static void W(uint16_t v) { mwr16(asm_pc, v); asm_pc += 2; }
static void L(uint32_t v) { W((uint16_t)(v >> 16)); W((uint16_t)v); }

// ---------------------------------------------------------------- allowed writes, areas with allowed FCs, expected values
static std::set<uint32_t> wr_ok;
static void allow(uint32_t a, uint32_t n) { for (uint32_t i = 0; i < n; i++) wr_ok.insert((a + i) & 0xFFFFFFu); }
static constexpr unsigned FC_SD = 1u << 5, FC_SP = 1u << 6;   // UM: FC 5 supervisor data, 6 supervisor program
struct Region { uint32_t lo, hi; unsigned fcmask; std::string name; };
static std::vector<Region> regions;
static void region(uint32_t lo, uint32_t hi, unsigned fcmask, const std::string& name) { regions.push_back(Region{lo, hi, fcmask, name}); }
static const Region* find_region(uint32_t a) { for (const Region& r : regions) if (a >= r.lo && a < r.hi) return &r; return nullptr; }
struct Exp { uint32_t addr; int nbytes; uint32_t val; std::string what; bool chk_init; };
static std::vector<Exp> exps;
static void expect_mem(uint32_t addr, int nbytes, uint32_t val, const std::string& what, bool chk_init = true) { exps.push_back(Exp{addr, nbytes, val, what, chk_init}); }
static uint32_t res_ptr = 0;
static uint32_t res_slot(uint32_t n) { uint32_t a = res_ptr; fill(a, n, SENT); allow(a, n); res_ptr += (n + 1u) & ~1u; return a; }

// ---------------------------------------------------------------- bus: cycle = AS low; DTACK after dtack_lat clocks
struct Cyc { uint32_t addr; int rw, fc, uds, lds; uint16_t wdata; };
static std::vector<Cyc> cyc;
static Cyc cur;
static bool in_cyc = false;
static int wait_n = 0, dtack_lat = 0;
static uint32_t end_addr = 0;
static int end_hits = 0;
static void tick() { dut->clk = 0; dut->eval(); t_ticks++; dut->clk = 1; dut->eval(); t_ticks++; }
static void close_cycle() {
    cyc.push_back(cur); in_cyc = false;
    if (cur.rw && cur.addr == end_addr) end_hits++;
}
static void bus_step() {
    if (!dut->as_n) {
        uint32_t a = ((uint32_t)dut->addr << 1) & 0xFFFFFFu;
        if (!in_cyc) { in_cyc = true; cur = Cyc{a, 1, (int)dut->fc, 0, 0, 0}; wait_n = 0; } else wait_n++;
        if (!dut->rw) cur.rw = 0;
        if (!dut->uds_n) cur.uds = 1;
        if (!dut->lds_n) cur.lds = 1;
        if (dut->rw) dut->data_in = mrd16(a);
        else if (dut->data_oe && (!dut->uds_n || !dut->lds_n)) {   // writes only the enabled lanes
            uint16_t d = (uint16_t)dut->data_out; cur.wdata = d;
            if (!dut->uds_n) mwr8(a, (uint8_t)(d >> 8));
            if (!dut->lds_n) mwr8(a + 1, (uint8_t)d);
        }
        dut->dtack_n = (wait_n >= dtack_lat) ? 0 : 1;
    } else {
        if (in_cyc) close_cycle();
        dut->dtack_n = 1;
    }
    dut->cen = 1; dut->cen2 = 1;
    tick();
}

static constexpr uint32_t PC0 = 0x4000;
static constexpr long MAX_TICKS = 3000000;
static void build();
static void run_once(int lat) {
    mem.clear(); regions.clear(); wr_ok.clear(); exps.clear(); cyc.clear();
    in_cyc = false; end_hits = 0; end_addr = 0; asm_pc = PC0; res_ptr = 0x1000;
    run_tag = "[DTACK+" + std::to_string(lat) + "] ";
    build();
    // testbench invariant: no expected value equals the preloaded content (a missed write must fail)
    for (const Exp& e : exps)
        if (e.chk_init && mrdn(e.addr, e.nbytes) == e.val) check_eq("TESTBENCH: expected value equal to the initial content at 0x" + hx(e.addr) + " (" + e.what + ")", 0, 1);
    dtack_lat = lat;
    t_ticks = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    dut->clk = 0; dut->rst_n = 0; dut->cen = 0; dut->cen2 = 0; dut->start = 0; dut->pc_init = PC0; dut->rst_vector = 0; dut->ipl = 0;
    dut->data_in = 0; dut->dtack_n = 1; dut->vpa_n = 1; dut->halt_n = 1; dut->berr_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    dut->eval(); tick(); tick();
    dut->rst_n = 1;
    long n = 0;
    while (n < MAX_TICKS && end_hits < 3) { bus_step(); n++; }
    check_eq("end of run: fetch of the BRA.S * at 0x" + hx(end_addr) + " seen 3 times within the tick limit", 1, end_hits >= 3 ? 1u : 0u);
    // the first 4 cycles are the reset vector reads emulated by the shim (SSP = 0, PC = pc_init): outside the testbench
    for (size_t i = 4; i < cyc.size(); i++) {
        const Cyc& c = cyc[i];
        std::string where = std::string(c.rw ? "read" : "write") + " at 0x" + hx(c.addr);
        const Region* r = find_region(c.addr);
        if (!r) { check_eq("cycle outside the program areas: " + where, 0, 1); continue; }
        check_cond(((r->fcmask >> c.fc) & 1u) != 0, "FC of the cycle, " + where + " (area " + r->name + ", expected = mask of the allowed FCs)", r->fcmask, (uint32_t)c.fc);
        if (!c.rw) {
            if (c.uds && !wr_ok.count(c.addr)) check_eq("write outside the expected areas (UDS) at 0x" + hx(c.addr), 0, 1);
            if (c.lds && !wr_ok.count((c.addr + 1) & 0xFFFFFFu)) check_eq("write outside the expected areas (LDS) at 0x" + hx(c.addr + 1), 0, 1);
        }
    }
    for (const Exp& e : exps) check_eq(e.what, e.val, mrdn(e.addr, e.nbytes));
    dut->final(); delete dut; dut = nullptr;
}

// ================================================================ testbench program
// modes: 0 positive d16(An), 1 negative d16(An), 2 positive abs.W, 3 negative abs.W, 4 abs.L
static const char* const MODEN[5] = {"d16(An) positive", "d16(An) negative", "abs.W positive", "abs.W negative", "abs.L"};
static const uint32_t POOL[5] = {0x2810, 0x2000, 0x3000, 0xFF8000, 0x345000};   // targets of mode m (8-byte slots)
static constexpr uint32_t A1_BASE = 0x2800, A2_BASE = 0x2C00, A1_FAR = 0x00777770;
static const char* const SVN[4] = {"MOVE.B (even byte)", "MOVE.B (odd byte)", "MOVE.W", "MOVE.L"};
static const uint16_t MSZ[3] = {1, 3, 2};   // size field of MOVE: B = 01, W = 11, L = 10
static uint32_t szmask(int sz) { return sz == 0 ? 0xFFu : sz == 1 ? 0xFFFFu : 0xFFFFFFFFu; }
static uint32_t szmsb(int sz) { return sz == 0 ? 0x80u : sz == 1 ? 0x8000u : 0x80000000u; }
static int szbytes(int sz) { return sz == 0 ? 1 : sz == 1 ? 2 : 4; }
static uint32_t value_of(int sz, int i) {
    static const uint32_t VB[4] = {0x80, 0x00, 0x7F, 0xA5};
    static const uint32_t VW[4] = {0x8001, 0x0000, 0x7FFE, 0x5AC3};
    static const uint32_t VL[4] = {0x80000001u, 0x00000000u, 0x7FFFFFFEu, 0xC3A5965Au};
    return sz == 0 ? VB[i & 3] : sz == 1 ? VW[i & 3] : VL[i & 3];
}
// PRM 4 MOVE: N = msb, Z = (data == 0), V = 0, C = 0, X unchanged
static uint8_t move_ccr(int sz, uint32_t v, uint8_t ccr_in) {
    v &= szmask(sz);
    return (uint8_t)((ccr_in & 0x10) | ((v & szmsb(sz)) ? 0x08 : 0) | (v == 0 ? 0x04 : 0));
}
static int pool_cnt[5];
static uint32_t alloc_target(int m, bool odd) {
    uint32_t slot = POOL[m] + 8u * (uint32_t)pool_cnt[m]++;
    fill(slot, 8, SENT);
    return slot + 2u + (odd ? 1u : 0u);
}
struct EA { int mode, reg; uint32_t addr, base; };
static EA ea_of(int m, int areg, uint32_t addr, uint32_t base) {
    if (m <= 1) return EA{5, areg, addr, base};   // d16(An)
    if (m <= 3) return EA{7, 0, addr, 0};         // abs.W
    return EA{7, 1, addr, 0};                     // abs.L
}
static uint16_t ea_field(const EA& e) { return (uint16_t)((e.mode << 3) | e.reg); }
static void emit_ext(const EA& e) {
    if (e.mode == 5) W((uint16_t)(e.addr - e.base));        // d16 = EA - An
    else if (e.reg == 0) W((uint16_t)(e.addr & 0xFFFF));    // abs.W
    else L(e.addr);                                         // abs.L
}
static uint8_t ccr_for(size_t k, uint8_t e0) { return (k % 3 == 0) ? (uint8_t)(~e0 & 0x1F) : (k % 3 == 1) ? 0x1F : 0x00; }

static void build() {
    for (int m = 0; m < 5; m++) pool_cnt[m] = 0;
    asm_pc = PC0;
    W(0x46FC); W(0x2700);                              // MOVE #$2700,SR: S=1, T=0, I=7 explicit
    size_t k = 0;
    for (int m = 0; m < 5; m++)
        for (int sv = 0; sv < 4; sv++)
            for (int dir = 0; dir < 2; dir++, k++) {
                const int sz = sv <= 1 ? 0 : sv - 1; const int nb = szbytes(sz); const uint32_t M = szmask(sz);
                const uint32_t val = value_of(sz, (int)k);
                const uint32_t T = alloc_target(m, sv == 1);
                const uint32_t a1 = (m <= 1) ? A1_BASE : A1_FAR;
                const EA e = ea_of(m, 1, T, A1_BASE);
                const uint8_t ccr_in = ccr_for(k, move_ccr(sz, val, 0));
                const uint8_t cexp = move_ccr(sz, val, ccr_in);
                const std::string tn = std::string(SVN[sv]) + (dir == 0 ? " D1," : " ") + MODEN[m] + " (EA 0x" + hx(T) + ")" +
                                       (dir == 0 ? "" : ",D2") + " case " + std::to_string(k) + " (data 0x" + hx(val) + "): ";
                if (dir == 0) {
                    const uint32_t d1 = ((k & 1) ? 0xA55AA55Au : 0x5AA55AA5u) & ~M;
                    W(0x223C); L(d1 | val);                // MOVE.L #d1,D1 (high part = pattern, low part = data)
                    W(0x227C); L(a1);                      // MOVEA.L #a1,A1
                    W(0x44FC); W(ccr_in);                  // MOVE #ccr_in,CCR
                    W((uint16_t)((MSZ[sz] << 12) | (ea_field(e) & 7) << 9 | (ea_field(e) >> 3) << 6 | 0x01));   // MOVE D1,<ea>
                    emit_ext(e);
                    allow(T, (uint32_t)nb);
                    expect_mem(T, nb, val, tn + "memory at the target = low part of D1");
                    const uint32_t R = res_slot(2);
                    W(0x40F8); W((uint16_t)R);             // MOVE SR,(R).W
                    expect_mem(R, 2, 0x2700u | cexp, tn + "SR (N Z from the data, V=C=0, X unchanged)");
                } else {
                    const uint32_t d2 = (k & 1) ? 0x96C3A55Au : 0x693C5AA5u;
                    mwrn(T, nb, val);
                    W(0x243C); L(d2);                      // MOVE.L #d2,D2
                    W(0x227C); L(a1);                      // MOVEA.L #a1,A1
                    W(0x44FC); W(ccr_in);                  // MOVE #ccr_in,CCR
                    W((uint16_t)((MSZ[sz] << 12) | (2 << 9) | (0 << 6) | ea_field(e)));   // MOVE <ea>,D2
                    emit_ext(e);
                    const uint32_t R = res_slot(6);
                    W(0x40F8); W((uint16_t)R);             // MOVE SR,(R).W
                    W(0x21C2); W((uint16_t)(R + 2));       // MOVE.L D2,(R+2).W
                    expect_mem(R, 2, 0x2700u | cexp, tn + "SR (N Z from the data, V=C=0, X unchanged)");
                    expect_mem(R + 2, 4, (d2 & ~M) | val, tn + "D2 = high part unchanged, low part = data");
                }
            }
    // memory -> memory: source with base A1, destination with base A2 (different values)
    struct MM { int sm, dm, sv; };
    static const MM MMS[6] = {{0, 4, 1}, {3, 0, 0}, {4, 2, 2}, {1, 3, 2}, {2, 0, 3}, {0, 4, 3}};
    for (int i = 0; i < 6; i++, k++) {
        const MM& c = MMS[i];
        const int sz = c.sv <= 1 ? 0 : c.sv - 1; const int nb = szbytes(sz);
        const uint32_t val = value_of(sz, (int)k + 1);
        const uint32_t Ts = alloc_target(c.sm, c.sv == 1);
        const uint32_t Td = alloc_target(c.dm, c.sv == 0 && sz == 0);   // byte: source and destination of opposite parity
        const EA es = ea_of(c.sm, 1, Ts, A1_BASE);
        const EA ed = ea_of(c.dm, 2, Td, A2_BASE);
        mwrn(Ts, nb, val);
        const uint8_t ccr_in = ccr_for(k, move_ccr(sz, val, 0));
        const uint8_t cexp = move_ccr(sz, val, ccr_in);
        W(0x227C); L(A1_BASE);                         // MOVEA.L #A1_BASE,A1
        W(0x247C); L(A2_BASE);                         // MOVEA.L #A2_BASE,A2
        W(0x44FC); W(ccr_in);                          // MOVE #ccr_in,CCR
        W((uint16_t)((MSZ[sz] << 12) | (ed.reg << 9) | (ed.mode << 6) | ea_field(es)));   // MOVE <es>,<ed>
        emit_ext(es);                                  // the source extension first (PRM sec. 2.2)
        emit_ext(ed);
        allow(Td, (uint32_t)nb);
        static const char* const SRCN[5] = {"d16(A1)", "d16(A1)", "abs.W positive", "abs.W negative", "abs.L"};
        static const char* const DSTN[5] = {"d16(A2)", "d16(A2)", "abs.W positive", "abs.W negative", "abs.L"};
        const std::string tn = std::string(SVN[c.sv]) + " " + SRCN[c.sm] + " (0x" + hx(Ts) + ") -> " + DSTN[c.dm] + " (0x" + hx(Td) +
                               ") case " + std::to_string(k) + " (data 0x" + hx(val) + "): ";
        expect_mem(Td, nb, val, tn + "destination memory = source");
        const uint32_t R = res_slot(2);
        W(0x40F8); W((uint16_t)R);                     // MOVE SR,(R).W
        expect_mem(R, 2, 0x2700u | cexp, tn + "SR (N Z from the data, V=C=0, X unchanged)");
    }
    end_addr = asm_pc; W(0x60FE);                      // BRA.S *
    // allowed areas and FCs (UM, Function Code Outputs)
    region(0x1000, 0x2000, FC_SD, "results");
    region(0x2000, 0x3000, FC_SD, "d16(An) targets");
    region(0x3000, 0x3800, FC_SD, "positive abs.W targets");
    region(0xFF8000, 0xFF8800, FC_SD, "negative abs.W targets");
    region(0x345000, 0x345800, FC_SD, "abs.L targets");
    region(0x4000, 0x5000, FC_SP, "program");
    if (asm_pc > 0x4FF0) check_eq("TESTBENCH: program beyond the code area", 0, asm_pc);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    run_once(0);
    run_once(3);
    printf("checks run: %d, failed: %d\n", n_checks, n_fails);
    puts(n_fails ? "RESULT: FAIL" : "RESULT: PASS");
    return n_fails ? 1 : 0;
}
