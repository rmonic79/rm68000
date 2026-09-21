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

// tb_m68k_alumem2.cpp - ADD, SUB, AND, OR, CMP <ea>,Dn with an EA using an extension word: d16(An) with positive and
// negative displacement, abs.W positive and negative (sign-extended: $8xxx -> $FF8xxx), abs.L (high word different from
// zero), on the whole processor (shim m68k_seq). Expected values come from a hand-written model based on M68000PRM ch. 4
// (ADD, SUB, AND, OR, CMP) and ch. 3 (condition code computation), sect. 2.2 (d16(An): EA = An + sign-extended d16;
// abs.W: address sign-extended to 32 bits; abs.L).
// What it tests (150 cases): every mode (5) x every operation (5) x every size (B, W, L) x 2 operand pairs (the 8 pairs
// of tb_m68k_alumem in rotation: zero, 1 and -1, maximum positive, minimum negative, mixed patterns, equal, carry and
// borrow between the halves of the long). Each case has its own operand (byte at an even and an odd address in turn,
// neighbouring cells set to a sentinel), Dn with the high part preloaded, a discriminating input CCR; after EVERY
// instruction SR, D0 and A1 are saved (A1 unchanged; in the absolute modes A1 holds a far address, so an EA computed
// with A1 shows up). CMP: flags only, with an equal pair that gives Z=1 only if the EA is correct. Bus: supervisor data
// FC on the operands, no writes outside the expected areas; DTACK immediate and delayed by 3 clocks.
// What it does NOT test: (An), (An)+, -(An) (tb_m68k_alumem), index and PC relative (tb_m68k_aluidx), timing.

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

// ---------------------------------------------------------------- verdict
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
        else if (dut->data_oe && (!dut->uds_n || !dut->lds_n)) {   // writes only the enabled byte lanes
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
    // testbench invariant: no expected value matches the preloaded content (a missed write must fail)
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
    check_eq("end of run: fetch of BRA.S * at 0x" + hx(end_addr) + " seen 3 times within the tick limit", 1, end_hits >= 3 ? 1u : 0u);
    // the first 4 cycles are the reset vector reads emulated by the shim (SSP = 0, PC = pc_init): outside the testbench
    for (size_t i = 4; i < cyc.size(); i++) {
        const Cyc& c = cyc[i];
        std::string where = std::string(c.rw ? "read" : "write") + " at 0x" + hx(c.addr);
        const Region* r = find_region(c.addr);
        if (!r) { check_eq("cycle outside the program areas: " + where, 0, 1); continue; }
        check_cond(((r->fcmask >> c.fc) & 1u) != 0, "cycle FC, " + where + " (area " + r->name + ", expected = mask of the allowed FCs)", r->fcmask, (uint32_t)c.fc);
        if (!c.rw) {
            if (c.uds && !wr_ok.count(c.addr)) check_eq("write outside the expected areas (UDS) at 0x" + hx(c.addr), 0, 1);
            if (c.lds && !wr_ok.count((c.addr + 1) & 0xFFFFFFu)) check_eq("write outside the expected areas (LDS) at 0x" + hx(c.addr + 1), 0, 1);
        }
    }
    for (const Exp& e : exps) check_eq(e.what, e.val, mrdn(e.addr, e.nbytes));
    dut->final(); delete dut; dut = nullptr;
}

// ---------------------------------------------------------------- model (M68000PRM ch. 4 and ch. 3, CC computation)
//   ADD (D + S):       V = Sm.Dm.!Rm + !Sm.!Dm.Rm ; C = Sm.Dm + !Rm.Dm + Sm.!Rm ; X = C
//   SUB, CMP (D - S):  V = !Sm.Dm.!Rm + Sm.!Dm.Rm ; C = Sm.!Dm + Rm.!Dm + Sm.Rm ; X = C for SUB, unchanged for CMP
//   AND, OR, EOR:      V = 0, C = 0, X unchanged
//   all:               N = Rm, Z = (R == 0)   (Sm, Dm, Rm = most significant bit of source, destination, result)
enum { OP_ADD = 0, OP_SUB, OP_AND, OP_OR, OP_CMP, OP_EOR };
static const char* const OPN[] = {"ADD", "SUB", "AND", "OR", "CMP", "EOR"};
static const char* const SZN[] = {".B", ".W", ".L"};
static uint32_t szmask(int sz) { return sz == 0 ? 0xFFu : sz == 1 ? 0xFFFFu : 0xFFFFFFFFu; }
static uint32_t szmsb(int sz) { return sz == 0 ? 0x80u : sz == 1 ? 0x8000u : 0x80000000u; }
static int szbytes(int sz) { return sz == 0 ? 1 : sz == 1 ? 2 : 4; }
struct AluRes { uint32_t r; uint8_t ccr; };
static AluRes alu_model(int op, int sz, uint32_t s, uint32_t d, uint8_t ccr_in) {
    const uint32_t M = szmask(sz), H = szmsb(sz);
    s &= M; d &= M;
    uint32_t r = 0;
    bool x = (ccr_in & 0x10) != 0, v = false, c = false;
    const bool Sm = (s & H) != 0, Dm = (d & H) != 0;
    switch (op) {
    case OP_ADD: { r = (d + s) & M; bool Rm = (r & H) != 0; v = (Sm && Dm && !Rm) || (!Sm && !Dm && Rm); c = (Sm && Dm) || (!Rm && Dm) || (Sm && !Rm); x = c; break; }
    case OP_SUB:
    case OP_CMP: { r = (d - s) & M; bool Rm = (r & H) != 0; v = (!Sm && Dm && !Rm) || (Sm && !Dm && Rm); c = (Sm && !Dm) || (Rm && !Dm) || (Sm && Rm); if (op == OP_SUB) x = c; break; }
    case OP_AND: r = d & s; break;
    case OP_OR:  r = d | s; break;
    default:     r = d ^ s; break;
    }
    const bool n = (r & H) != 0, z = (r == 0);
    return AluRes{r, (uint8_t)((x ? 0x10 : 0) | (n ? 0x08 : 0) | (z ? 0x04 : 0) | (v ? 0x02 : 0) | (c ? 0x01 : 0))};
}
// pairs (destination Dn, source in memory) per size
static constexpr int NP = 8;
static void pair_of(int sz, int p, uint32_t& d, uint32_t& s) {
    const uint32_t M = szmask(sz), H = szmsb(sz);
    switch (p) {
    case 0: d = 0; s = 0; break;                                  // zero: Z=1
    case 1: d = 1; s = M; break;                                  // 1 and -1: carry (ADD), borrow (SUB)
    case 2: d = H - 1; s = 1; break;                              // maximum positive + 1: V on addition
    case 3: d = H; s = 1; break;                                  // minimum negative - 1: V on subtraction
    case 4: d = 0xC3A596E1u & M; s = 0x5A3CF00Fu & M; break;      // mixed patterns
    case 5: d = H | 1u; s = H | 1u; break;                        // equal, msb set: CMP Z=1
    case 6: d = 0x0000FFFFu & M; s = 1; break;                    // carry out of the low part (.L: between the two halves)
    default: d = 0x00010000u & M; s = 1; break;                   // borrow across the low part (.L)
    }
}
static constexpr uint16_t OPB[5] = {0xD000, 0x9000, 0xC000, 0x8000, 0xB000};   // ADD SUB AND OR CMP <ea>,Dn (opmode 0ss)

// ================================================================ testbench program
// modes: 0 d16(A1) positive, 1 d16(A1) negative, 2 abs.W positive, 3 abs.W negative, 4 abs.L
static const char* const MODEN[5] = {"d16(A1) positive", "d16(A1) negative", "abs.W positive", "abs.W negative", "abs.L"};
static constexpr uint32_t A1_BASE = 0x2800;                   // base of the d16(A1) modes
static constexpr uint32_t A1_FAR = 0x00777770;                // A1 in the absolute modes (never used as an address)
static const uint32_t POOL[5] = {0x2810, 0x2000, 0x3000, 0xFF8000, 0x12000};   // operands of mode m
struct ACase { int mode, op, sz, p; };
static void build() {
    std::vector<ACase> cs;
    int ci = 0;
    for (int m = 0; m < 5; m++)
        for (int op = 0; op < 5; op++)
            for (int sz = 0; sz < 3; sz++, ci++) { cs.push_back(ACase{m, op, sz, ci % NP}); cs.push_back(ACase{m, op, sz, (ci + 3) % NP}); }
    int per_mode[5] = {0, 0, 0, 0, 0};
    asm_pc = PC0;
    W(0x46FC); W(0x2700);                              // MOVE #$2700,SR: S=1, T=0, I=7 explicit
    for (size_t k = 0; k < cs.size(); k++) {
        const ACase& c = cs[k];
        const uint32_t M = szmask(c.sz); const int nb = szbytes(c.sz);
        uint32_t d, s; pair_of(c.sz, c.p, d, s);
        const uint32_t garb = (k & 1) ? 0xA55AA55Au : 0x5AA55AA5u;
        const uint32_t dinit = (garb & ~M) | d;
        const uint32_t slot = POOL[c.mode] + 8u * (uint32_t)per_mode[c.mode]++;
        fill(slot, 8, SENT);
        const uint32_t ea = slot + 2u + ((c.sz == 0 && (k & 1)) ? 1u : 0u);   // byte: even and odd in turn
        mwrn(ea, nb, s);
        const uint32_t a1 = (c.mode <= 1) ? A1_BASE : A1_FAR;
        const uint8_t r0 = alu_model(c.op, c.sz, s, d, 0).ccr;
        const uint8_t ccr_in = (k % 3 == 0) ? (uint8_t)(~r0 & 0x1F) : (k % 3 == 1) ? 0x1F : 0x00;
        const AluRes e = alu_model(c.op, c.sz, s, d, ccr_in);
        const uint32_t dexp = (c.op == OP_CMP) ? dinit : ((dinit & ~M) | e.r);
        W(0x203C); L(dinit);                           // MOVE.L #dinit,D0
        W(0x227C); L(a1);                              // MOVEA.L #a1,A1
        W(0x44FC); W(ccr_in);                          // MOVE #ccr_in,CCR
        const uint16_t eaf = (c.mode <= 1) ? 0x29 : (c.mode <= 3) ? 0x38 : 0x39;   // d16(A1) / abs.W / abs.L
        W((uint16_t)(OPB[c.op] | (c.sz << 6) | eaf));  // OP.sz <ea>,D0
        if (c.mode <= 1) W((uint16_t)(ea - A1_BASE));  // d16 = EA - A1 (m0 > 0, m1 < 0)
        else if (c.mode <= 3) W((uint16_t)(ea & 0xFFFF));   // abs.W ($8xxx -> $FFFF8xxx, 24 bits $FF8xxx)
        else L(ea);                                    // abs.L
        const uint32_t R = res_slot(10);
        W(0x40F8); W((uint16_t)R);                     // MOVE SR,(R).W
        W(0x21C0); W((uint16_t)(R + 2));               // MOVE.L D0,(R+2).W
        W(0x21C9); W((uint16_t)(R + 6));               // MOVE.L A1,(R+6).W
        const std::string tn = std::string(OPN[c.op]) + SZN[c.sz] + " " + MODEN[c.mode] + " (EA 0x" + hx(ea) + "),D0 case " +
                               std::to_string(k) + " (D0=0x" + hx(dinit) + ", mem=0x" + hx(s) + ", input CCR 0x" + hx(ccr_in) + "): ";
        expect_mem(R, 2, 0x2700u | e.ccr, tn + "SR (X N Z V C from the model, S=1 I=7)");
        expect_mem(R + 2, 4, dexp, tn + (c.op == OP_CMP ? "D0 unchanged (CMP does not write)" : "D0 = high part unchanged, low part = result"));
        expect_mem(R + 6, 4, a1, tn + "A1 unchanged");
    }
    end_addr = asm_pc; W(0x60FE);                      // BRA.S *
    // areas and allowed FCs (UM, Function Code Outputs)
    region(0x1000, 0x2000, FC_SD, "results");
    region(0x2000, 0x3000, FC_SD, "d16(A1) operands");
    region(0x3000, 0x3800, FC_SD, "abs.W positive operands");
    region(0xFF8000, 0xFF8800, FC_SD, "abs.W negative operands");
    region(0x12000, 0x12800, FC_SD, "abs.L operands");
    region(0x4000, 0x6000, FC_SP, "program");
    if (asm_pc > 0x5FF0) check_eq("TESTBENCH: program beyond the code area", 0, asm_pc);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    run_once(0);
    run_once(3);
    printf("checks run: %d, failed: %d\n", n_checks, n_fails);
    puts(n_fails ? "RESULT: FAIL" : "RESULT: PASS");
    return n_fails ? 1 : 0;
}
