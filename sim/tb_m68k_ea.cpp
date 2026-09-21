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

// tb_m68k_ea.cpp - effective addresses with an extension word: d16(An), abs.W, abs.L, on the full processor (shim m68k_seq).
//
// Semantics (PRM 2.2.6, 2.2.12, 2.2.13; PRM 4 MOVE): d16(An): EA = An + sign-extended d16; abs.W: EA = sign-extended word
// (0x8000..0xFFFF -> 0xFFFF8000..: on the 24-bit bus 0xFF8000..); abs.L: EA = two words (high first).
// MOVE: N and Z from the data of the size, V = C = 0, X unchanged; byte and word to Dn do not touch the upper bits (PRM 1.1.1).
// Cases (40): for each of the five modes - positive d16(A2), negative d16(A3), positive abs.W, negative abs.W, abs.L with
// high word 0x0003 - write and read in B (even and odd address), W and L. Every target is different, with neighbouring
// sentinel cells checked again (a write of the wrong size or lane is visible); byte reads want a single strobe and the
// slave puts the complement on the disabled lane; the destination of the reads is preloaded with 0x5A5A5A5A. After each
// case: SR, the data register and the base register in the results area. Input CCR 0x1F or 0x00.
// Two runs (DTACK with no wait states and after 3 clocks), FC and areas of every cycle checked. Clock: cen = cen2 = 1.
// NOT tested: d8(An,Xn) and PC-relative modes in reads (other testbenches), memory-to-memory MOVE, timing in clocks.

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

// ------------------------------------------------------------- MOVE between Dn and a memory operand (PRM 4 MOVE, PRM 2.2)
static const int SZC[3] = {1, 3, 2};   // size field of the MOVE: B = 01, W = 11, L = 10
static int nbytes(int sz) { return sz == 0 ? 1 : (sz == 1 ? 2 : 4); }
static uint32_t sx16(uint32_t v) { return (v & 0x8000u) ? (v | 0xFFFF0000u) : (v & 0xFFFFu); }
static uint32_t step_of(int an, int sz) { return (an == 7 && sz == 0) ? 2u : (uint32_t)nbytes(sz); }   // PRM 2.2.4/2.2.5
// mode: 2 (An), 3 (An)+, 4 -(An), 5 d16(An), 7 absolute (reg 0 = abs.W, reg 1 = abs.L). Applies the effects on An.
static uint32_t ea_eval(int m, int r, int sz, uint32_t ext) {
    switch (m) {
        case 2: return A[r];
        case 3: { uint32_t a = A[r]; A[r] += step_of(r, sz); return a; }
        case 4: { A[r] -= step_of(r, sz); return A[r]; }
        case 5: return A[r] + sx16(ext);
        default: return (r == 0) ? sx16(ext) : ext;
    }
}
static void emit_ext(int m, int r, uint32_t ext) {
    if (m == 5 || (m == 7 && r == 0)) w((uint16_t)ext);
    else if (m == 7 && r == 1) l32(ext);
}
static std::map<uint32_t, uint8_t> wrote;     // bytes already written by the program (to read back what has been written)
static uint32_t rd_now(uint32_t a, int sz) {
    uint32_t v = 0;
    for (int i = 0; i < nbytes(sz); i++) {
        uint32_t b = (a + (uint32_t)i) & 0xFFFFFF;
        auto q = wrote.find(b);
        uint8_t x = 0;
        if (q != wrote.end()) x = q->second;
        else { auto p = preload.find(b); x = (p != preload.end()) ? p->second : 0; }
        v = (v << 8) | x;
    }
    return v;
}
// MOVE.sz Ds,<ea>
static uint32_t mv_store(int s, int m, int r, int sz, uint32_t ext, const std::string& nm) {
    w((uint16_t)((SZC[sz] << 12) | (r << 9) | (m << 6) | s));
    emit_ext(m, r, ext);
    uint32_t ea = ea_eval(m, r, sz, ext) & 0xFFFFFF;
    uint32_t v = D[s] & szmask(sz);
    expect(ea, nbytes(sz), v, nm + " memory " + hx(ea));
    for (int i = 0; i < nbytes(sz); i++) wrote[ea + (uint32_t)i] = (uint8_t)(v >> (8 * (nbytes(sz) - 1 - i)));
    flags_nz(v, sz);
    return ea;
}
// MOVE.sz <ea>,Dd
static uint32_t mv_load(int d, int m, int r, int sz, uint32_t ext) {
    w((uint16_t)((SZC[sz] << 12) | (d << 9) | (m << 3) | r));
    emit_ext(m, r, ext);
    uint32_t ea = ea_eval(m, r, sz, ext) & 0xFFFFFF;
    uint32_t v = rd_now(ea, sz);
    if (sz == 0) byte_rd.insert(ea & 0xFFFFFEu);
    D[d] = merge(D[d], v, sz);
    flags_nz(v, sz);
    return ea;
}

static void build() {
    prologue(0x010000, 0x0F80);
    // five modes: target area (0x100 bytes), mode/register of the MOVE, base register (d16) and its value
    struct V { const char* name; uint32_t area; int m, r; uint32_t base; };
    static const V VS[5] = {
        {"d16(A2) positive", 0x000C00u, 5, 2, 0x0BF0u},     // EA = 0x0BF0 + (off + 0x10)
        {"d16(A3) negative", 0x000D00u, 5, 3, 0x0E00u},     // EA = 0x0E00 + (off - 0x100)
        {"abs.W positive",   0x000B00u, 7, 0, 0},
        {"abs.W negative",   0xFF8000u, 7, 0, 0},           // word 0x8000 + off -> 0xFFFF8000 + off
        {"abs.L high 0x0003", 0x038000u, 7, 1, 0}};
    int k = 0;
    for (int vi = 0; vi < 5; vi++) {
        const V& v = VS[vi];
        if (v.m == 5) ld_a(v.r, v.base);
        for (int rd = 0; rd < 2; rd++) {
            int j = 0;
            for (int sz = 0; sz < 3; sz++)
                for (int par = 0; par < (sz == 0 ? 2 : 1); par++) {
                    uint32_t off = (rd ? 0x80u : 0x00u) + 0x10u * (uint32_t)j + (uint32_t)par;
                    uint32_t tgt = v.area + off;                    // 24-bit EA
                    uint32_t ext;
                    if (v.m == 5) ext = (tgt - v.base) & 0xFFFFu;   // 16-bit displacement
                    else if (v.r == 0) ext = (vi == 3) ? (0x8000u + off) : tgt;
                    else ext = tgt;
                    int dr = (k % 7) + 1;                           // D1..D7
                    std::string nm = std::string(rd ? "R" : "W") + std::to_string(k) + " MOVE." + szc(sz) + " " + v.name + " EA " + hx(tgt);
                    if (rd) {
                        for (int i = 0; i < nbytes(sz); i++) pre8(tgt + (uint32_t)i, (uint8_t)(0x7Fu ^ (uint32_t)(k * 0x35 + i * 0x59)));
                        ld_d(dr, 0x5A5A5A5Au);
                        set_ccr((k & 1) ? 0x1F : 0x00);
                        mv_load(dr, v.m, v.r, sz, ext);
                    } else {
                        ld_d(dr, 0x8C4E2A1Fu ^ (0x01030507u * (uint32_t)k));
                        set_ccr((k & 1) ? 0x00 : 0x1F);
                        mv_store(dr, v.m, v.r, sz, ext, nm);
                    }
                    dump(nm, (uint8_t)(1 << dr), (uint8_t)(v.m == 5 ? (1 << v.r) : 0));
                    j++; k++;
                }
        }
    }
    finish_program();
    data_area(0x000B00, 0x000C00);
    data_area(0x000C00, 0x000D00);
    data_area(0x000D00, 0x000E00);
    data_area(0xFF8000, 0xFF8100);
    data_area(0x038000, 0x038100);
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
