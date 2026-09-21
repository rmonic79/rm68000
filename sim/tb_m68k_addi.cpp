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

// tb_m68k_addi.cpp - immediate and ALU operations on a data register, on the whole processor (shim m68k_seq).
//
// Instructions tested, all with destination Dn, in all sizes B/W/L:
//   1) ORI ANDI SUBI ADDI EORI CMPI #imm,Dn (PRM 4, extension word: byte in the low byte of the word, word, long
//      in two words) - 12 operand pairs x 3 sizes x 2 input CCRs = 432 cases;
//   2) OR SUB AND ADD Ds,Dd (<ea>,Dn form) and EOR Ds,Dd - 12 pairs x 3 sizes x 2 CCRs = 360 cases;
//   3) ASL ASR LSL LSR ROL ROR, immediate count 1, 3, 8 and register count 0, 1, 7, 8, 15, 16, 31, 32, 63 (register with
//      non-zero high bits: the count is modulo 64), two values with high and low bits at 1 - 432 cases (ROL/ROR with wrap-around);
//   4) NEG NOT CLR - 8 values x 3 sizes x 2 CCRs = 144 cases.
// The pairs cover 0, 1, -1, the positive maxima and negative minima of every size, carries, borrows and overflow, with the
// destination having high bits different from the source (a write of the wrong size shows up).
// Expectations from the programmer's model written here from the manual (not from the RTL):
//   ADD/ADDI (PRM 3.4 and 4 ADD): R = S + D; N = Rm; Z = (R == 0); V = Sm.Dm.!Rm + !Sm.!Dm.Rm; C = Sm.Dm + !Rm.Dm + Sm.!Rm; X = C.
//   SUB/SUBI (PRM 3.4 and 4 SUB): R = D - S; V = !Sm.Dm.!Rm + Sm.!Dm.Rm; C = Sm.!Dm + Rm.!Dm + Sm.Rm; X = C.
//   CMPI (PRM 4 CMPI): like SUB without writing the destination, X unchanged.
//   AND/OR/EOR/ANDI/ORI/EORI/NOT (PRM 4): N, Z from the result, V = C = 0, X unchanged.
//   NEG (PRM 4 NEG): R = 0 - D; V = Dm.Rm; C = Dm + Rm; X = C.  CLR (PRM 4 CLR): N=0 Z=1 V=0 C=0, X unchanged.
//   ASL/ASR/LSL/LSR/ROL/ROR (PRM 4): simulated one bit at a time: C = last bit shifted out (0 with count 0); X = C for AS and
//   LS, unchanged with count 0 and always unchanged for RO; V only for ASL = the most significant bit changes at any
//   step; ASR replicates the sign; RO feeds the shifted-out bit back in at the other end.
//   The bits of Dn above the size do not change (PRM 1.1.1).
// After every instruction: MOVE SR,abs.W and MOVE.L of the registers involved into the result area; the whole SR
// (X N Z V C, S, mask, T) and the registers are compared. Byte memory with sentinels, FC and areas of every cycle; two runs (DTACK without
// wait states and after 3 clocks). Clock: cen = cen2 = 1 on every clock.
// Does NOT test: memory destinations, ROXL/ROXR, ADDX/SUBX, timing in clocks (high counts only lengthen the run).

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

// ===================================================================== shared harness (identical in the testbenches of the group)
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
static std::map<uint32_t, uint8_t> preload;    // initial data (identical in the two runs)
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
static void ld_a(int n, uint32_t v) { w((uint16_t)(0x207C | (n << 9))); l32(v); A[n] = v; }                    // MOVEA.L #v,An (CC unchanged)
static void set_ccr(uint8_t c) { w(0x44FC); w(c); CCR = (uint8_t)(c & 0x1F); }                                  // MOVE #c,CCR (PRM 4 MOVE to CCR)

static uint32_t res_ptr = 0x1000, res_hi = 0x8000;
// saves SR and the chosen registers into the result area (positive abs.W addresses). MOVE SR,<ea> does not touch the CCs;
// MOVE.L Rn,<ea> sets N and Z, V = C = 0 (the model follows them). Expected SR = 0x2700 | CCR: S = 1 and mask 7 from reset
// (MC68000UM, reset exception), T = 0, no instruction of the testbenches changes them.
static void dump(const std::string& nm, uint8_t dm, uint8_t am) {
    uint32_t r = res_ptr;
    w(0x40F8); w((uint16_t)r); expect(r, 2, 0x2700u | CCR, nm + " SR");
    uint32_t o = r + 4;
    for (int n = 0; n < 8; n++) if (dm & (1 << n)) { w((uint16_t)(0x21C0 | n)); w((uint16_t)o); expect(o, 4, D[n], nm + " D" + std::to_string(n)); flags_nz(D[n], 2); o += 4; }
    for (int n = 0; n < 8; n++) if (am & (1 << n)) { w((uint16_t)(0x21C8 | n)); w((uint16_t)o); expect(o, 4, A[n], nm + " A" + std::to_string(n)); flags_nz(A[n], 2); o += 4; }
    res_ptr = o;
    if (res_ptr + 8 > res_hi) chk(false, "TESTBENCH ERROR: result area full", res_hi, res_ptr);
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
    chk(!bc.unstable, where + ": address, FC, R/W, strobes and data stable while AS is low", 0, 1);
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
        } else chk(false, where + ": read outside the expected areas", 0, bc.addr);
    } else {
        if (k == R_DATA) {
            chk(bc.fc == 5, where + ": FC of the write (data, supervisor)", 5, bc.fc);
            chk(bc.wcap, where + ": write with at least one strobe", 1, 0);
            if (bc.su) mem[bc.addr] = (uint8_t)(bc.wd >> 8);
            if (bc.sl) mem[bc.addr + 1] = (uint8_t)(bc.wd & 0xFF);
        } else chk(false, where + ": write outside the expected areas", 0, bc.addr);
    }
}
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }
// one clock: memory slave (data valid only on the lanes with strobe, complement on the others), DTACK after g_delay clocks
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
// ===================================================================== end of the shared harness

// ADD (PRM 3.4): V = Sm.Dm.!Rm + !Sm.!Dm.Rm ; C = Sm.Dm + !Rm.Dm + Sm.!Rm ; X = C
static uint32_t m_add(uint32_t s, uint32_t d, int sz) {
    const uint32_t m = szmask(sz), b = szmsb(sz);
    s &= m; d &= m;
    uint32_t r = (s + d) & m;
    bool Sm = (s & b) != 0, Dm = (d & b) != 0, Rm = (r & b) != 0;
    bool v = (Sm && Dm && !Rm) || (!Sm && !Dm && Rm);
    bool c = (Sm && Dm) || (!Rm && Dm) || (Sm && !Rm);
    CCR = (uint8_t)((c ? F_X : 0) | (Rm ? F_N : 0) | (r == 0 ? F_Z : 0) | (v ? F_V : 0) | (c ? F_C : 0));
    return r;
}
// SUB (PRM 3.4), R = D - S: V = !Sm.Dm.!Rm + Sm.!Dm.Rm ; C = Sm.!Dm + Rm.!Dm + Sm.Rm ; X = C (SUB) or unchanged (CMP)
static uint32_t m_sub(uint32_t s, uint32_t d, int sz, bool setx) {
    const uint32_t m = szmask(sz), b = szmsb(sz);
    s &= m; d &= m;
    uint32_t r = (d - s) & m;
    bool Sm = (s & b) != 0, Dm = (d & b) != 0, Rm = (r & b) != 0;
    bool v = (!Sm && Dm && !Rm) || (Sm && !Dm && Rm);
    bool c = (Sm && !Dm) || (Rm && !Dm) || (Sm && Rm);
    uint8_t x = setx ? (c ? F_X : 0) : (uint8_t)(CCR & F_X);
    CCR = (uint8_t)(x | (Rm ? F_N : 0) | (r == 0 ? F_Z : 0) | (v ? F_V : 0) | (c ? F_C : 0));
    return r;
}
// shifts and rotations (PRM 4 ASL/ASR, LSL/LSR, ROL/ROR) simulated one bit at a time. type: 0 AS, 1 LS, 3 RO
static uint32_t m_shift(int type, bool left, int sz, int cnt, uint32_t val) {
    const uint32_t m = szmask(sz), b = szmsb(sz);
    uint32_t v = val & m;
    bool c = false, vf = false, x = (CCR & F_X) != 0;
    for (int i = 0; i < cnt; i++) {
        if (left) {
            bool out = (v & b) != 0;
            uint32_t nv = (v << 1) & m;
            if (type == 3 && out) nv |= 1u;                                            // ROL: the shifted-out bit re-enters at bit 0
            if (type == 0 && (((nv & b) != 0) != ((v & b) != 0))) vf = true;           // ASL: the most significant bit changes
            v = nv; c = out; if (type != 3) x = out;
        } else {
            bool out = (v & 1u) != 0;
            uint32_t nv = v >> 1;
            if (type == 0) nv |= (v & b);                                              // ASR: replicates the sign
            if (type == 3 && out) nv |= b;                                             // ROR: the shifted-out bit re-enters at the top
            v = nv; c = out; if (type != 3) x = out;
        }
    }
    CCR = (uint8_t)((x ? F_X : 0) | ((v & b) ? F_N : 0) | (v == 0 ? F_Z : 0) | (vf ? F_V : 0) | (c ? F_C : 0));
    return v;
}

static const char* IMM_NAME[8] = {"ORI", "ANDI", "SUBI", "ADDI", "?", "EORI", "CMPI", "?"};
// kind = bits 11-9 of the opcode 0000 kkk 0 ss 000 ddd (PRM 4): 0 ORI, 1 ANDI, 2 SUBI, 3 ADDI, 5 EORI, 6 CMPI
static void imm_case(int kind, int sz, int d, uint32_t imm, uint8_t ccr, const std::string& tag) {
    set_ccr(ccr);
    const uint32_t m = szmask(sz);
    uint32_t iv = imm & m;
    w((uint16_t)((kind << 9) | (sz << 6) | d));
    if (sz == 2) l32(iv); else w((uint16_t)iv);   // byte: data in the low byte of the extension word (PRM 2, immediate data)
    uint32_t dv = D[d] & m, r = 0;
    switch (kind) {
        case 0: r = dv | iv; flags_nz(r, sz); break;
        case 1: r = dv & iv; flags_nz(r, sz); break;
        case 2: r = m_sub(iv, dv, sz, true); break;
        case 3: r = m_add(iv, dv, sz); break;
        case 5: r = dv ^ iv; flags_nz(r, sz); break;
        default: m_sub(iv, dv, sz, false); break;   // CMPI
    }
    if (kind != 6) D[d] = merge(D[d], r, sz);
    dump(tag + " " + IMM_NAME[kind] + "." + szc(sz) + " #" + hx(iv) + ",D" + std::to_string(d) + " ccr=" + hx(ccr), (uint8_t)(1 << d), 0);
}
static const char* RR_NAME[5] = {"OR", "SUB", "AND", "ADD", "EOR"};
// 0 OR 1000 ddd 0ss sss ; 1 SUB 1001 ; 2 AND 1100 ; 3 ADD 1101 (<ea>,Dn form) ; 4 EOR 1011 sss 1ss ddd (PRM 4)
static void rr_case(int kind, int sz, int s, int d, uint8_t ccr, const std::string& tag) {
    static const uint16_t base[4] = {0x8000, 0x9000, 0xC000, 0xD000};
    set_ccr(ccr);
    if (kind == 4) w((uint16_t)(0xB100 | (s << 9) | (sz << 6) | d));
    else w((uint16_t)(base[kind] | (d << 9) | (sz << 6) | s));
    const uint32_t m = szmask(sz);
    uint32_t sv = D[s] & m, dv = D[d] & m, r = 0;
    switch (kind) {
        case 0: r = sv | dv; flags_nz(r, sz); break;
        case 1: r = m_sub(sv, dv, sz, true); break;
        case 2: r = sv & dv; flags_nz(r, sz); break;
        case 3: r = m_add(sv, dv, sz); break;
        default: r = sv ^ dv; flags_nz(r, sz); break;
    }
    D[d] = merge(D[d], r, sz);
    dump(tag + " " + RR_NAME[kind] + "." + szc(sz) + " D" + std::to_string(s) + ",D" + std::to_string(d) + " ccr=" + hx(ccr), (uint8_t)((1 << s) | (1 << d)), 0);
}
static const char* SH_NAME[4] = {"AS", "LS", "ROX", "RO"};
// 1110 ccc d ss i tt rrr (PRM 4, register form): ccc = count 1..8 (0 = 8) or register; d = 1 left; i = 1 register
static void sh_case(int type, bool left, int sz, int r, bool isreg, int cr, uint8_t ccr, const std::string& tag) {
    set_ccr(ccr);
    int field = isreg ? cr : (cr & 7);
    w((uint16_t)(0xE000 | (field << 9) | ((left ? 1 : 0) << 8) | (sz << 6) | ((isreg ? 1 : 0) << 5) | (type << 3) | r));
    int cnt = isreg ? (int)(D[cr] & 63u) : cr;   // from register: modulo 64 (PRM 4 ASL/LSL/ROL)
    uint32_t v = m_shift(type, left, sz, cnt, D[r]);
    D[r] = merge(D[r], v, sz);
    std::string nm = tag + " " + SH_NAME[type] + (left ? "L." : "R.") + szc(sz) + (isreg ? (" D" + std::to_string(cr) + "(=" + std::to_string(cnt) + ")") : (" #" + std::to_string(cr))) + ",D" + std::to_string(r) + " ccr=" + hx(ccr);
    dump(nm, (uint8_t)((1 << r) | (isreg ? (1 << cr) : 0)), 0);
}
// NEG 0100 0100 ss ; NOT 0100 0110 ss ; CLR 0100 0010 ss (PRM 4), destination Dn
static void un_case(int kind, int sz, int d, uint8_t ccr, const std::string& tag) {
    static const uint16_t base[3] = {0x4400, 0x4600, 0x4200};
    static const char* nmk[3] = {"NEG", "NOT", "CLR"};
    set_ccr(ccr);
    w((uint16_t)(base[kind] | (sz << 6) | d));
    const uint32_t m = szmask(sz);
    uint32_t dv = D[d] & m, r = 0;
    if (kind == 0) r = m_sub(dv, 0, sz, true);                 // NEG = 0 - D: with S = D, D = 0 the SUB formulas give V = Dm.Rm, C = Dm + Rm
    else if (kind == 1) { r = ~dv & m; flags_nz(r, sz); }
    else { r = 0; CCR = (uint8_t)((CCR & F_X) | F_Z); }
    D[d] = merge(D[d], r, sz);
    dump(tag + " " + nmk[kind] + "." + szc(sz) + " D" + std::to_string(d) + " ccr=" + hx(ccr), (uint8_t)(1 << d), 0);
}

static void build() {
    prologue(0x010000, 0x0F80);
    // pairs (source or immediate, destination). Hand-worked examples (PRM 3.4):
    //   ADDI.B #1,D(0xFFFFFFFF): 0xFF+0x01 = 0x00, C=X=1, Z=1, V=0 -> D = 0xFFFFFF00; in W -> 0xFFFF0000; in L -> 0, C=1.
    //   ADDI.B #0x7F,D(0x5A5A5A01): 0x01+0x7F = 0x80, V=1, N=1, C=0 -> D = 0x5A5A5A80; in W 0x5A01+0x007F = 0x5A80, V=0.
    //   SUBI.L #1,D(0x80000000): 0x7FFFFFFF, V=1, C=0; in W 0x0000-0x0001 = 0xFFFF, C=X=1, N=1 (upper 0x8000 unchanged).
    //   CMPI.B #0x80,D(0x3C3C3C7F): 0x7F-0x80 = 0xFF, V=1, C=1, N=1, X unchanged, D does not change.
    static const uint32_t PR[12][2] = {
        {0x00000000u, 0x00000000u}, {0x00000001u, 0xFFFFFFFFu}, {0xFFFFFFFFu, 0x00000001u}, {0x0000007Fu, 0x5A5A5A01u},
        {0x00007FFFu, 0xA5A50001u}, {0x7FFFFFFFu, 0x00000001u}, {0x00000080u, 0x12345680u}, {0x00008000u, 0x00008000u},
        {0x80000000u, 0x80000000u}, {0x12345678u, 0x9ABCDEF0u}, {0x00000001u, 0x80000000u}, {0x00000080u, 0x3C3C3C7Fu}};
    static const int IMM_K[6] = {0, 1, 2, 3, 5, 6};
    int k = 0;
    for (int ki = 0; ki < 6; ki++)
        for (int sz = 0; sz < 3; sz++)
            for (int p = 0; p < 12; p++)
                for (int c = 0; c < 2; c++) {
                    int d = k % 8;
                    ld_d(d, PR[p][1]);
                    imm_case(IMM_K[ki], sz, d, PR[p][0], c ? 0x1F : 0x00, "I" + std::to_string(k));
                    k++;
                }
    k = 0;
    for (int kind = 0; kind < 5; kind++)
        for (int sz = 0; sz < 3; sz++)
            for (int p = 0; p < 12; p++)
                for (int c = 0; c < 2; c++) {
                    int s = (k * 3 + 1) % 8, d = (k * 5 + 2) % 8;   // never equal (3k+1 - 5k-2 = -2k-1 odd)
                    ld_d(s, PR[p][0]);
                    ld_d(d, PR[p][1]);
                    rr_case(kind, sz, s, d, c ? 0x1F : 0x00, "R" + std::to_string(k));
                    k++;
                }
    // shifts and rotations: values with high and low bit at 1 in every size (0x81 / 0xC381 / 0x9234C381) and with high bit at 0
    static const uint32_t SV[2] = {0x9234C381u, 0x6A5B3C4Eu};
    static const int IMMC[3] = {1, 3, 8};
    static const int REGC[9] = {0, 1, 7, 8, 15, 16, 31, 32, 63};
    static const int TYPES[3] = {0, 1, 3};
    k = 0;
    for (int ti = 0; ti < 3; ti++)
        for (int dir = 0; dir < 2; dir++)
            for (int sz = 0; sz < 3; sz++)
                for (int vi = 0; vi < 2; vi++) {
                    for (int ci = 0; ci < 3; ci++) {
                        int r = k % 8;
                        ld_d(r, SV[vi]);
                        sh_case(TYPES[ti], dir == 1, sz, r, false, IMMC[ci], (k & 1) ? 0x1F : 0x00, "S" + std::to_string(k));
                        k++;
                    }
                    for (int ci = 0; ci < 9; ci++) {
                        int r = k % 8, cr = (k + 3) % 8;
                        ld_d(r, SV[vi]);
                        ld_d(cr, 0x5A5A5A40u | (uint32_t)REGC[ci]);   // bit 6 and above at 1: the count is modulo 64
                        sh_case(TYPES[ti], dir == 1, sz, r, true, cr, (k & 1) ? 0x1F : 0x00, "S" + std::to_string(k));
                        k++;
                    }
                }
    static const uint32_t UV[8] = {0x00000000u, 0x00000001u, 0x5A5A5A80u, 0x3C3C8000u, 0x80000000u, 0x1234567Fu, 0xFFFFFFFFu, 0x12345678u};
    k = 0;
    for (int kind = 0; kind < 3; kind++)
        for (int sz = 0; sz < 3; sz++)
            for (int vi = 0; vi < 8; vi++)
                for (int c = 0; c < 2; c++) {
                    int d = k % 8;
                    ld_d(d, UV[vi]);
                    un_case(kind, sz, d, c ? 0x1F : 0x00, "U" + std::to_string(k));
                    k++;
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
