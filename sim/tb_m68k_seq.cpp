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

// tb_m68k_seq.cpp - sequence of instructions and branches on the whole processor (shim m68k_seq).
//
// Tests:
//   1) forward Bcc.S: the 14 conditions (codes 2..15) x the 16 values of N Z V C (X in rotation) = 224 cases, so every
//      condition taken and not taken with every combination of the flags (PRM 3.6 conditional tests table: HI = !C.!Z,
//      LS = C+Z, CC = !C, CS = C, NE = !Z, EQ = Z, VC = !V, VS = V, PL = !N, MI = N, GE = N.V + !N.!V, LT = N.!V + !N.V,
//      GT = N.V.!Z + !N.!V.!Z, LE = Z + N.!V + !N.V);
//   2) Bcc.W with 16-bit displacement, forward and backward, every condition taken and not taken (56 cases); BRA.S, BRA.W;
//   3) a backward loop with BNE.S (counter from -3, taken twice and not taken the third time);
//   4) BSR.S and BSR.W with RTS: return address on the stack (PRM 4 BSR: address of the instruction after the BSR), A7
//      inside the subroutine (-4) and after the return, CCR unchanged;
//   5) MOVEQ, ADDQ.L, MOVE.L between registers, MOVE.L with (An), (An)+, -(An) and a CMP.L followed by BLT/BGE.
// Whether a branch is taken or not is seen from A0: the not-taken path does ADDQ.W #1,A0, the taken one ADDQ.W #2,A0 (ADDQ to An does not touch
// the CC, PRM 4 ADDQ); SUBA.L A0,A0 clears it beforehand (SUBA does not touch the CC). Bcc, BRA, BSR, RTS do not touch the CC (PRM 4).
// After every case SR and the registers are saved into the results area and everything is compared with the model. Byte memory with
// sentinels, FC and areas of every cycle, two passes (DTACK without wait states and after 3 clocks). Clock: cen = cen2 = 1.
// Does NOT test: DBcc, Scc, Bcc.S with maximum displacement, clock timing of the branches.

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

// ===================================================================== common framework (the same in the testbenches of the group)
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
static std::map<uint32_t, uint8_t> preload;    // initial data (the same in the two passes)
static std::map<uint32_t, uint8_t> exp_w;      // expected final value of the bytes that the program writes
struct Field { int size; std::string name; };
static std::map<uint32_t, Field> fields;
static std::set<uint32_t> fcover;
static std::map<uint32_t, uint8_t> mem;        // data memory during the pass
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
// PRM 4 MOVE/MOVEQ/AND/OR/EOR/NOT: N and Z from the result at the size, V = C = 0, X unchanged
static void flags_nz(uint32_t r, int sz) {
    CCR = (uint8_t)((CCR & F_X) | ((r & szmsb(sz)) ? F_N : 0) | (((r & szmask(sz)) == 0) ? F_Z : 0));
}
static void ld_d(int n, uint32_t v) { w((uint16_t)(0x203C | (n << 9))); l32(v); D[n] = v; flags_nz(v, 2); }   // MOVE.L #v,Dn
static void ld_a(int n, uint32_t v) { w((uint16_t)(0x207C | (n << 9))); l32(v); A[n] = v; }                    // MOVEA.L #v,An (CC unchanged)
static void set_ccr(uint8_t c) { w(0x44FC); w(c); CCR = (uint8_t)(c & 0x1F); }                                  // MOVE #c,CCR (PRM 4 MOVE to CCR)

static uint32_t res_ptr = 0x1000, res_hi = 0x8000;
// saves SR and the chosen registers into the results area (positive abs.W addresses). MOVE SR,<ea> does not touch the CC;
// MOVE.L Rn,<ea> sets N and Z, V = C = 0 (the model follows them). Expected SR = 0x2700 | CCR: S = 1 and mask 7 from the reset
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
    if (bc.idx < 4) return;   // the first four cycles are the reset vector reads (SSP and PC supplied by the shim)
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
    printf("%spass: %d bus cycles, %ld clocks\n", tag.c_str(), g_ncyc, n);
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

// ADD/ADDQ (PRM 3.4): V = Sm.Dm.!Rm + !Sm.!Dm.Rm ; C = Sm.Dm + !Rm.Dm + Sm.!Rm ; X = C
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
// CMP (PRM 3.4), R = D - S: V = !Sm.Dm.!Rm + Sm.!Dm.Rm ; C = Sm.!Dm + Rm.!Dm + Sm.Rm ; X unchanged
static void m_cmp(uint32_t s, uint32_t d, int sz) {
    const uint32_t m = szmask(sz), b = szmsb(sz);
    s &= m; d &= m;
    uint32_t r = (d - s) & m;
    bool Sm = (s & b) != 0, Dm = (d & b) != 0, Rm = (r & b) != 0;
    bool v = (!Sm && Dm && !Rm) || (Sm && !Dm && Rm);
    bool c = (Sm && !Dm) || (Rm && !Dm) || (Sm && Rm);
    CCR = (uint8_t)((CCR & F_X) | (Rm ? F_N : 0) | (r == 0 ? F_Z : 0) | (v ? F_V : 0) | (c ? F_C : 0));
}
// PRM 3.6, conditional tests
static bool cond_true(int c, uint8_t ccr) {
    bool N = (ccr & F_N) != 0, Z = (ccr & F_Z) != 0, V = (ccr & F_V) != 0, C = (ccr & F_C) != 0;
    switch (c) {
        case 0: return true;               case 1: return false;
        case 2: return !C && !Z;           case 3: return C || Z;
        case 4: return !C;                 case 5: return C;
        case 6: return !Z;                 case 7: return Z;
        case 8: return !V;                 case 9: return V;
        case 10: return !N;                case 11: return N;
        case 12: return (N && V) || (!N && !V);
        case 13: return (N && !V) || (!N && V);
        case 14: return (N && V && !Z) || (!N && !V && !Z);
        default: return Z || (N && !V) || (!N && V);
    }
}
static const char* CC_NAME[16] = {"RA", "SR", "HI", "LS", "CC", "CS", "NE", "EQ", "VC", "VS", "PL", "MI", "GE", "LT", "GT", "LE"};

// forward Bcc.S. X: Bcc.S +4 (target X+6) ; X+2: ADDQ.W #1,A0 ; X+4: BRA.S +2 (-> X+8) ; X+6: ADDQ.W #2,A0 ; X+8: continuation
static void bcc_s(int cond, int ccr, const std::string& nm) {
    w(0x91C8); A[0] = 0;                               // SUBA.L A0,A0
    if (ccr >= 0) set_ccr((uint8_t)ccr);
    bool tk = cond_true(cond, CCR);
    w((uint16_t)(0x6004 | (cond << 8)));
    w(0x5248); w(0x6002); w(0x5448);
    A[0] = tk ? 2u : 1u;
    dump(nm + " B" + CC_NAME[cond] + ".S ccr=" + hx(CCR) + (tk ? " taken" : " not taken"), 0, 0x01);
}
// forward Bcc.W. X: Bcc.W +6 (target X+2+6 = X+8) ; X+4: ADDQ.W #1,A0 ; X+6: BRA.S +2 (-> X+10) ; X+8: ADDQ.W #2,A0
static void bcc_w_fwd(int cond, uint8_t ccr, const std::string& nm) {
    w(0x91C8); A[0] = 0;
    set_ccr(ccr);
    bool tk = cond_true(cond, CCR);
    w((uint16_t)(0x6000 | (cond << 8))); w(0x0006);
    w(0x5248); w(0x6002); w(0x5448);
    A[0] = tk ? 2u : 1u;
    dump(nm + " B" + CC_NAME[cond] + ".W forward ccr=" + hx(CCR) + (tk ? " taken" : " not taken"), 0, 0x01);
}
// backward Bcc.W. B: BRA.S +4 (-> B+6) ; B+2: ADDQ.W #2,A0 ; B+4: BRA.S +6 (-> B+12) ;
// B+6: Bcc.W -6 (target B+8-6 = B+2) ; B+10: ADDQ.W #1,A0 ; B+12: continuation
static void bcc_w_back(int cond, uint8_t ccr, const std::string& nm) {
    w(0x91C8); A[0] = 0;
    set_ccr(ccr);
    bool tk = cond_true(cond, CCR);
    w(0x6004); w(0x5448); w(0x6006);
    w((uint16_t)(0x6000 | (cond << 8))); w(0xFFFA);
    w(0x5248);
    A[0] = tk ? 2u : 1u;
    dump(nm + " B" + CC_NAME[cond] + ".W backward ccr=" + hx(CCR) + (tk ? " taken" : " not taken"), 0, 0x01);
}
// MOVE.L (A7),(res).W: saves the top of the stack into the results area
static void dump_top(const std::string& nm, uint32_t v) {
    uint32_t r = res_ptr;
    w(0x21D7); w((uint16_t)r); expect(r, 4, v, nm + " (A7)"); flags_nz(v, 2);
    res_ptr = r + 4;
}
static void moveq(int n, int8_t v) { w((uint16_t)(0x7000 | (n << 9) | (uint8_t)v)); D[n] = (uint32_t)(int32_t)v; flags_nz(D[n], 2); }

static void build() {
    const uint32_t SP = 0x0F80;
    prologue(0x010000, SP);
    // 5) MOVEQ / ADDQ.L / MOVE.L between registers: D3 = 5 + 3 = 8 (C = V = 0, X = 0), D5 = D3
    set_ccr(0x1F);
    moveq(3, 5);
    w(0x5683); D[3] = m_add(3, D[3], 2);               // ADDQ.L #3,D3
    w(0x2A03); D[5] = D[3]; flags_nz(D[5], 2);         // MOVE.L D3,D5
    w(0x4E71);                                         // NOP
    dump("MOVEQ/ADDQ/MOVE", 0x28, 0x00);
    // 1) Bcc.S: every condition with every N Z V C
    for (int cond = 2; cond < 16; cond++)
        for (int nzvc = 0; nzvc < 16; nzvc++)
            bcc_s(cond, (((cond + nzvc) & 1) ? 0x10 : 0x00) | nzvc, "S");
    // 2) Bcc.W forward and backward, taken and not taken; BRA.S / BRA.W
    for (int cond = 2; cond < 16; cond++) {
        int yes = -1, no = -1;
        for (int nzvc = 0; nzvc < 16; nzvc++) {
            if (cond_true(cond, (uint8_t)nzvc)) { if (yes < 0) yes = nzvc; } else { if (no < 0) no = nzvc; }
        }
        // for every condition there is a value that makes it true and one that makes it false (the pairs are complementary)
        bcc_w_fwd(cond, (uint8_t)(0x10 | yes), "W");
        bcc_w_fwd(cond, (uint8_t)no, "W");
        bcc_w_back(cond, (uint8_t)yes, "W");
        bcc_w_back(cond, (uint8_t)(0x10 | no), "W");
    }
    bcc_s(0, 0x0F, "BRA");
    bcc_w_fwd(0, 0x00, "BRA");
    bcc_w_back(0, 0x1F, "BRA");
    // 3) backward loop: D0 = -3, D1 = 0 ; loop: ADDQ.L #1,D1 ; ADDQ.L #1,D0 ; BNE.S loop (-6) -> D0 = 0, D1 = 3, Z = 1, C = X = 1
    moveq(0, -3);
    moveq(1, 0);
    w(0x5281); w(0x5280); w(0x66FA);
    do { D[1] = m_add(1, D[1], 2); D[0] = m_add(1, D[0], 2); } while (!(CCR & F_Z));
    dump("BNE loop", 0x03, 0x00);
    // 4) BSR.S: X: BSR.S +2 (-> X+4, return X+2) ; X+2: BRA.S past the subroutine ; X+4: subroutine ... RTS
    {
        set_ccr(0x0A);
        uint32_t X = pc_e;
        w(0x6102);                                     // BSR.S +2
        uint32_t ret = X + 2;
        uint32_t bra_at = pc_e; w(0x6000);             // BRA.S (displacement written later)
        // subroutine
        A[7] -= 4; expect(A[7], 4, ret, "BSR.S return address on the stack");
        dump("BSR.S inside", 0x00, 0x80);
        dump_top("BSR.S inside", ret);
        w(0x4E75); A[7] += 4;                          // RTS
        uint32_t join = pc_e;
        code[bra_at] = (uint16_t)(0x6000 | ((join - (bra_at + 2)) & 0xFF));
        dump("BSR.S after RTS", 0x00, 0x80);
    }
    // BSR.W: X: BSR.W +6 (-> X+8, return X+4) ; X+4: BRA.W past it ; X+8: subroutine ... RTS
    {
        set_ccr(0x15);
        uint32_t X = pc_e;
        w(0x6100); w(0x0006);
        uint32_t ret = X + 4;
        uint32_t bra_at = pc_e; w(0x6000); w(0x0000);  // BRA.W (displacement written later)
        A[7] -= 4; expect(A[7], 4, ret, "BSR.W return address on the stack");
        dump("BSR.W inside", 0x00, 0x80);
        dump_top("BSR.W inside", ret);
        w(0x4E75); A[7] += 4;
        uint32_t join = pc_e;
        code[bra_at + 2] = (uint16_t)(join - (bra_at + 2));
        dump("BSR.W after RTS", 0x00, 0x80);
    }
    // 5) MOVE.L with (A1), (A1)+, -(A1) on 0x0800 (preloaded with 0xCAFEF00D) and CMP.L + BLT/BGE
    for (int i = 0; i < 4; i++) pre8(0x0800 + (uint32_t)i, (uint8_t)(0xCAFEF00Du >> (24 - 8 * i)));
    ld_a(1, 0x0800);
    set_ccr(0x00);
    w(0x2C11); D[6] = 0xCAFEF00Du; flags_nz(D[6], 2);               // MOVE.L (A1),D6
    dump("MOVE.L (A1),D6", 0x40, 0x02);
    moveq(4, 0x7F);
    w(0x2284); expect(0x0800, 4, D[4], "MOVE.L D4,(A1)"); flags_nz(D[4], 2);   // MOVE.L D4,(A1)
    w(0x2E11); D[7] = D[4]; flags_nz(D[7], 2);                       // MOVE.L (A1),D7 (reads the write back)
    dump("MOVE.L D4,(A1) / (A1),D7", 0x90, 0x02);
    w(0x2A19); D[5] = D[4]; A[1] += 4; flags_nz(D[5], 2);            // MOVE.L (A1)+,D5
    dump("MOVE.L (A1)+,D5", 0x20, 0x02);
    ld_d(3, 0x80000008u);
    w(0x2303); A[1] -= 4; expect(0x0800, 4, D[3], "MOVE.L D3,-(A1)"); flags_nz(D[3], 2);   // MOVE.L D3,-(A1)
    dump("MOVE.L D3,-(A1)", 0x08, 0x02);
    moveq(2, 3);
    moveq(4, 7);
    w(0xB882); m_cmp(D[2], D[4], 2);                                  // CMP.L D2,D4: 7 - 3 = 4, N=Z=V=C=0
    bcc_s(13, -1, "CMP 7,3 then");                                     // BLT: not taken
    moveq(2, 3);
    moveq(4, 7);
    w(0xB882); m_cmp(D[2], D[4], 2);
    bcc_s(12, -1, "CMP 7,3 then");                                     // BGE: taken
    moveq(2, 9);
    w(0xB882); m_cmp(D[2], D[4], 2);                                  // 7 - 9 = -2: N = 1, C = 1
    bcc_s(13, -1, "CMP 7,9 then");                                     // BLT: taken
    finish_program();
    data_area(0x0800, 0x0810);            // data of (A1)
    data_area(0x0E80, 0x0FA0);            // stack
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
