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

// tb_m68k_eorbw.cpp - EOR.B / EOR.W / EOR.L Dn,Dn on the full processor (shim m68k_seq).
//
// Semantics (PRM 4 EOR, PRM 3.4 condition code computation): Dd[size] <- Ds[size] ^ Dd[size]; the bits of Dd above the
// size stay unchanged (PRM 1.1.1 data registers: a byte or word operand uses and changes only the low part).
// Flags: N = most significant bit of the result, Z = result zero, V = 0, C = 0, X unchanged.
// Cases (127):
//   A) 10 source/destination pairs chosen so that B, W and L give DIFFERENT results and flags (high bits different in
//      source and destination), x 3 sizes x 2 input CCRs (0x1F: V and C must drop, X must stay at 1;
//      0x00: X must stay at 0), source/destination registers different case by case;
//   B) EOR Dn,Dn with source = destination (zero result, high bits preserved) in the three sizes;
//   C) all 64 register pairs (Ds,Dd), size in rotation, different values in every register.
// After every EOR, SR and D0-D7 are saved in the results area (MOVE SR,abs.W and MOVE.L Dn,abs.W) and everything is
// compared: a write to the wrong register, of the wrong size, or a wrong flag makes the case fail.
// Bus: byte model that honors UDS/LDS; results area preloaded with sentinels and checked again byte by byte;
// every cycle must have the right FC (6 fetch, 5 data, supervisor) and fall in the planned areas. The program runs
// twice: DTACK with no wait states and DTACK after 3 clocks (wait states), with different sentinels; same expected values.
// The clock: cen = cen2 = 1 on every clock (valid both with the two-phase shim and with the single-phase one).
// NOT tested: EOR to memory, EORI, clock timing of the instruction.

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <vector>
#include <string>

static Vm68k_seq* dut = nullptr;
static uint64_t t = 0;
double sc_time_stamp() { return (double)t; }

// ===================================================================== common framework (same in the testbenches of the group)
static int n_checks = 0;
static int n_fail = 0;
static std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "0x%X", v); return std::string(b); }
static void chk(bool ok, const std::string& what, uint32_t exp, uint32_t got) {
    n_checks++;
    if (!ok) {
        n_fail++;
        if (n_fail <= 300) printf("FAIL: %s expected=%s read=%s\n", what.c_str(), hx(exp).c_str(), hx(got).c_str());
    }
}

static const int R_CODE = 1;
static const int R_DATA = 2;
struct Region { uint32_t lo, hi; int kind; };
static std::vector<Region> regions;
static int region_of(uint32_t a) { for (const Region& r : regions) if (a >= r.lo && a < r.hi) return r.kind; return 0; }
static void data_area(uint32_t lo, uint32_t hi) { regions.push_back(Region{lo, hi, R_DATA}); }
static std::map<uint32_t, uint16_t> code;      // program: word per address (24 bits, even)
static std::map<uint32_t, uint8_t> preload;    // initial data (same in both runs)
static std::map<uint32_t, uint8_t> exp_w;      // expected final value of the bytes the program writes
struct Field { int size; std::string name; };
static std::map<uint32_t, Field> fields;
static std::set<uint32_t> fcover;
static std::map<uint32_t, uint8_t> mem;        // data memory during the run
static std::set<uint32_t> byte_rd;             // words read ONLY as bytes: a single strobe expected
static uint8_t sentinel(uint32_t a, int run) {
    uint8_t s = (uint8_t)(0xA5 ^ (a & 0xFF) ^ (((a >> 8) & 0xFF) * 13));
    return run ? (uint8_t)~s : s;
}
static void expect(uint32_t a, int size, uint32_t v, const std::string& name) {
    a &= 0xFFFFFF;
    for (int i = 0; i < size; i++) { exp_w[a + i] = (uint8_t)(v >> (8 * (size - 1 - i))); fcover.insert(a + i); }
    fields[a] = Field{size, name};
}
static void pre8(uint32_t a, uint8_t v) { preload[a & 0xFFFFFF] = v; }

static uint32_t pc_e = 0, blk_lo = 0, sv_pc = 0, sv_lo = 0;
static void org(uint32_t a) { pc_e = a & 0xFFFFFF; blk_lo = pc_e; }
static void endblk() { regions.push_back(Region{blk_lo, pc_e + 8, R_CODE}); }
static void sub_begin(uint32_t a) { sv_pc = pc_e; sv_lo = blk_lo; org(a); }
static void sub_end() { endblk(); pc_e = sv_pc; blk_lo = sv_lo; }
static void w(uint16_t x) { code[pc_e] = x; pc_e = (pc_e + 2) & 0xFFFFFF; }
static void l32(uint32_t x) { w((uint16_t)(x >> 16)); w((uint16_t)x); }

// programmer's model (PRM): registers and CCR
static uint32_t D[8], A[8];
static uint8_t CCR = 0;
static const uint8_t F_X = 0x10, F_N = 0x08, F_Z = 0x04, F_V = 0x02, F_C = 0x01;
static uint32_t szmask(int sz) { return sz == 0 ? 0xFFu : (sz == 1 ? 0xFFFFu : 0xFFFFFFFFu); }
static uint32_t szmsb(int sz) { return sz == 0 ? 0x80u : (sz == 1 ? 0x8000u : 0x80000000u); }
static uint32_t merge(uint32_t old, uint32_t r, int sz) { uint32_t m = szmask(sz); return (old & ~m) | (r & m); }
static const char* szc(int sz) { return sz == 0 ? "B" : (sz == 1 ? "W" : "L"); }
// PRM 4 MOVE/MOVEQ/AND/OR/EOR/NOT: N and Z from the result of the size, V = C = 0, X unchanged
static void flags_nz(uint32_t r, int sz) {
    CCR = (uint8_t)((CCR & F_X) | ((r & szmsb(sz)) ? F_N : 0) | (((r & szmask(sz)) == 0) ? F_Z : 0));
}
static void ld_d(int n, uint32_t v) { w((uint16_t)(0x203C | (n << 9))); l32(v); D[n] = v; flags_nz(v, 2); }   // MOVE.L #v,Dn
static void ld_a(int n, uint32_t v) { w((uint16_t)(0x207C | (n << 9))); l32(v); A[n] = v; }                    // MOVEA.L #v,An (CCs unchanged)
static void set_ccr(uint8_t c) { w(0x44FC); w(c); CCR = (uint8_t)(c & 0x1F); }                                  // MOVE #c,CCR (PRM 4 MOVE to CCR)

static uint32_t res_ptr = 0x1000, res_hi = 0x8000;
// saves SR and the chosen registers into the results area (positive abs.W addresses). MOVE SR,<ea> does not touch the CCs;
// MOVE.L Rn,<ea> sets N and Z, V = C = 0 (the model follows them). Expected SR = 0x2700 | CCR: S = 1 and mask 7 from reset
// (MC68000UM, reset exception), T = 0, no instruction of the testbenches changes them.
static void dump(const std::string& nm, uint8_t dm, uint8_t am) {
    uint32_t r = res_ptr;
    w(0x40F8); w((uint16_t)r); expect(r, 2, 0x2700u | CCR, nm + " SR");
    uint32_t o = r + 4;
    for (int n = 0; n < 8; n++) if (dm & (1 << n)) { w((uint16_t)(0x21C0 | n)); w((uint16_t)o); expect(o, 4, D[n], nm + " D" + std::to_string(n)); flags_nz(D[n], 2); o += 4; }
    for (int n = 0; n < 8; n++) if (am & (1 << n)) { w((uint16_t)(0x21C8 | n)); w((uint16_t)o); expect(o, 4, A[n], nm + " A" + std::to_string(n)); flags_nz(A[n], 2); o += 4; }
    res_ptr = o;
    if (res_ptr + 8 > res_hi) chk(false, "TESTBENCH ERROR: results area full", res_hi, res_ptr);
}

static uint32_t g_pc0 = 0, g_end = 0;
static void prologue(uint32_t pc0, uint32_t sp) {
    g_pc0 = pc0; org(pc0);
    ld_a(7, sp);
    for (int n = 0; n < 8; n++) ld_d(n, (0x11111111u * (uint32_t)(n + 1)) ^ 0x80402010u);
    for (int n = 0; n < 7; n++) ld_a(n, 0x00030000u + 0x1000u * (uint32_t)n + 0x24u);
}
static void finish_program() { g_end = pc_e; w(0x60FE); endblk(); }   // BRA.S * (end of run)

struct BusCyc { bool open, rw, su, sl, wcap, unstable; uint32_t addr, fc; int cnt, idx; uint16_t wd; };
static BusCyc bc;
static int g_delay = 0, g_ncyc = 0, g_end_hits = 0;

static uint8_t rdb(uint32_t a) {
    auto m = mem.find(a);
    if (m != mem.end()) return m->second;
    auto c = code.find(a & 0xFFFFFEu);
    if (c != code.end()) return (a & 1) ? (uint8_t)(c->second & 0xFF) : (uint8_t)(c->second >> 8);
    return (a & 1) ? 0x71 : 0x4E;
}
static void close_cycle() {
    bc.open = false;
    if (bc.idx < 4) return;   // the first four cycles are the reads of the reset vectors (SSP and PC supplied by the shim)
    const std::string where = "[DTACK+" + std::to_string(g_delay) + "] cycle " + std::to_string(bc.idx) + " at " + hx(bc.addr);
    chk(!bc.unstable, where + ": address, FC, R/W, strobes and data stable with AS low", 0, 1);
    int k = region_of(bc.addr);
    uint32_t st = (bc.su ? 2u : 0u) | (bc.sl ? 1u : 0u);
    if (bc.rw) {
        if (k == R_CODE) {
            chk(bc.fc == 6, where + ": FC of the fetch (program, supervisor: UM function code table)", 6, bc.fc);
            chk(st == 3, where + ": UDS and LDS in the word fetch", 3, st);
            if (bc.addr == g_end) g_end_hits++;
        } else if (k == R_DATA) {
            chk(bc.fc == 5, where + ": FC of the data read (supervisor)", 5, bc.fc);
            if (byte_rd.count(bc.addr)) chk(st == 1 || st == 2, where + ": byte read with a single strobe", 1, st);
            else chk(st != 0, where + ": read with strobe", 3, st);
        } else chk(false, where + ": read outside the planned areas", 0, bc.addr);
    } else {
        if (k == R_DATA) {
            chk(bc.fc == 5, where + ": FC of the write (data, supervisor)", 5, bc.fc);
            chk(bc.wcap, where + ": write with at least one strobe", 1, 0);
            if (bc.su) mem[bc.addr] = (uint8_t)(bc.wd >> 8);
            if (bc.sl) mem[bc.addr + 1] = (uint8_t)(bc.wd & 0xFF);
        } else chk(false, where + ": write outside the planned areas", 0, bc.addr);
    }
}
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }
// one clock: memory slave (data valid only on the lanes with a strobe, complement on the others), DTACK after g_delay clocks
static void step() {
    if (dut->as_n == 0) {
        uint32_t a = ((uint32_t)dut->addr) << 1;
        bool u = (dut->uds_n == 0), l = (dut->lds_n == 0);
        if (!bc.open) { bc = BusCyc(); bc.open = true; bc.addr = a; bc.rw = (dut->rw != 0); bc.fc = dut->fc; bc.idx = g_ncyc++; }
        else bc.cnt++;
        if (a != bc.addr || (dut->rw != 0) != bc.rw || (uint32_t)dut->fc != bc.fc) bc.unstable = true;
        if (bc.rw) {
            uint16_t v = (uint16_t)((rdb(a) << 8) | rdb(a + 1));
            uint16_t hi = u ? (uint16_t)(v & 0xFF00) : (uint16_t)(~v & 0xFF00);
            uint16_t lo = l ? (uint16_t)(v & 0x00FF) : (uint16_t)(~v & 0x00FF);
            dut->data_in = (uint16_t)(hi | lo);
            bc.su = bc.su || u; bc.sl = bc.sl || l;
        } else if (u || l) {
            if (!dut->data_oe) bc.unstable = true;
            if (!bc.wcap) { bc.wcap = true; bc.wd = dut->data_out; bc.su = u; bc.sl = l; }
            else if (dut->data_out != bc.wd || u != bc.su || l != bc.sl) bc.unstable = true;
        }
        dut->dtack_n = (bc.cnt >= g_delay) ? 0 : 1;
    } else {
        if (bc.open) close_cycle();
        dut->dtack_n = 1;
    }
    dut->cen = 1; dut->cen2 = 1;
    tick();
}
static void run_once(int run, int delay) {
    g_delay = delay;
    mem.clear();
    for (const Region& r : regions)
        if (r.kind == R_DATA)
            for (uint32_t a = r.lo; a < r.hi; a++) { auto p = preload.find(a); mem[a] = (p != preload.end()) ? p->second : sentinel(a, run); }
    bc = BusCyc(); g_ncyc = 0; g_end_hits = 0;
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    dut->clk = 0; dut->rst_n = 0; dut->cen = 1; dut->cen2 = 1; dut->start = 0; dut->pc_init = g_pc0; dut->rst_vector = 0;
    dut->ipl = 0; dut->vpa_n = 1; dut->dtack_n = 1; dut->halt_n = 1; dut->berr_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    dut->data_in = 0; dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    dut->eval();
    for (int i = 0; i < 8; i++) tick();
    dut->rst_n = 1;
    dut->start = 1; step(); dut->start = 0;
    long n = 0;
    while (g_end_hits < 3 && n < 20000000L) { step(); n++; }
    const std::string tag = "[DTACK+" + std::to_string(delay) + "] ";
    printf("%srun: %d bus cycles, %ld clocks\n", tag.c_str(), g_ncyc, n);
    chk(g_end_hits >= 3, tag + "the program reaches the final BRA.S * at " + hx(g_end), 3, (uint32_t)g_end_hits);
    chk(dut->illegal == 0, tag + "no illegal instruction exception", 0, dut->illegal);
    for (const auto& f : fields) {
        uint32_t e = 0, g = 0;
        for (int i = 0; i < f.second.size; i++) { e = (e << 8) | exp_w[f.first + i]; g = (g << 8) | mem[f.first + i]; }
        chk(e == g, tag + f.second.name + " (" + hx(f.first) + ")", e, g);
    }
    for (const Region& r : regions)
        if (r.kind == R_DATA)
            for (uint32_t a = r.lo; a < r.hi; a++) {
                if (fcover.count(a)) continue;
                auto p = preload.find(a);
                uint8_t e = (p != preload.end()) ? p->second : sentinel(a, run);
                chk(mem[a] == e, tag + "byte that must not be written " + hx(a), e, mem[a]);
            }
    dut->final(); delete dut; dut = nullptr;
}
// ===================================================================== end of the common framework

// EOR.sz Ds,Dd = 1011 sss 1 ss 000 ddd (PRM 4 EOR, form Dn,<ea> with <ea> = Dd)
static void eor_case(const std::string& nm, int s, int d, int sz, uint8_t ccr) {
    set_ccr(ccr);
    w((uint16_t)(0xB100 | (s << 9) | (sz << 6) | d));
    uint32_t r = (D[s] ^ D[d]) & szmask(sz);
    D[d] = merge(D[d], r, sz);
    flags_nz(r, sz);
    dump(nm, 0xFF, 0x00);
}

static void build() {
    prologue(0x010000, 0x0F80);
    // A) pairs with different high bits: the same EOR gives different results in B, W, L. Examples by hand (PRM 4 EOR):
    //    {0x0000000F, 0xFFFFFFFF}: B -> Dd = 0xFFFFFFF0 (N=1); W -> 0xFFFFFFF0 (N=1); L -> 0xFFFFFFF0 (N=1) and the
    //    destination above the size stays 0xFFFFFF.. / 0xFFFF....; {0x00008000, 0}: B -> 0 (Z=1), W -> 0x8000 (N=1),
    //    L -> 0x00008000 (N=0, Z=0); {0x80000000, 1}: B -> 0x01, W -> 0x0001, L -> 0x80000001 (N=1).
    static const uint32_t PS[10][2] = {
        {0x0000000Fu, 0xFFFFFFFFu}, {0x12345678u, 0x9ABCDEF0u}, {0x80808080u, 0x00000000u}, {0xA5A5A5A5u, 0xA5A5A5A5u},
        {0x000000FFu, 0x123456FFu}, {0x00FF0000u, 0x00FF00FFu}, {0x7FFFFFFFu, 0x00000000u}, {0x00008000u, 0x00000000u},
        {0x80000000u, 0x00000001u}, {0x5555AA00u, 0x55550000u}};
    int k = 0;
    for (int p = 0; p < 10; p++)
        for (int sz = 0; sz < 3; sz++)
            for (int c = 0; c < 2; c++) {
                int s = (k * 3 + 1) % 8, d = (k * 5 + 2) % 8;   // 3k+1 and 5k+2 never coincide modulo 8
                ld_d(s, PS[p][0]);
                ld_d(d, PS[p][1]);
                eor_case("A" + std::to_string(k) + " EOR." + szc(sz) + " D" + std::to_string(s) + ",D" + std::to_string(d), s, d, sz, c ? 0x1F : 0x00);
                k++;
            }
    // B) source = destination: result 0 in the size, bits above unchanged (B -> 0x12345600, W -> 0x12340000, L -> 0)
    for (int sz = 0; sz < 3; sz++) {
        ld_d(3, 0x12345678u);
        eor_case("B EOR." + std::string(szc(sz)) + " D3,D3", 3, 3, sz, (uint8_t)(sz == 1 ? 0x1B : 0x0B));
    }
    // C) all register pairs
    for (int s = 0; s < 8; s++)
        for (int d = 0; d < 8; d++) {
            int sz = (s + 2 * d) % 3;
            ld_d(s, 0x9E3779B9u * (uint32_t)(s * 8 + d + 1));
            if (d != s) ld_d(d, 0x7F4A7C15u ^ (0x01010101u * (uint32_t)(s * 8 + d)));
            eor_case("C EOR." + std::string(szc(sz)) + " D" + std::to_string(s) + ",D" + std::to_string(d), s, d, sz, (uint8_t)(((s + d) & 1) ? 0x15 : 0x0A));
        }
    finish_program();
    data_area(0x0E80, 0x0FA0);            // stack (not used: must stay intact)
    data_area(0x1000, res_ptr + 8);       // results
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    run_once(0, 0);
    run_once(1, 3);
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    if (n_fail == 0) { printf("RESULT: PASS\n"); return 0; }
    printf("RESULT: FAIL\n");
    return 1;
}
