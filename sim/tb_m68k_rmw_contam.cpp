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

// tb_m68k_rmw_contam.cpp - no contamination between a read-modify-write instruction Dn,<ea> and the neighbouring ones, on the
// whole processor (shim m68k_seq): the address, the data and the write of an RMW must not leak into the next
// instruction, and the previous ones must not leak into the RMW. Expectations from a model written by hand from M68000PRM ch. 4 (ADD, SUB, CMP,
// EOR, MOVE) and ch. 3 (condition code computation).
// Sequence, for every size (B, W, L), with all registers loaded explicitly (high parts with different patterns):
//   1 ADD.s D0,(A1)    RMW on X1                 5 SUB.s D4,(A1)    second RMW on X1, after a read and a reg-reg
//   2 MOVE.s D1,(A2)   write to X2               6 CMP.s (A4),D5    read-only access to X4, flags only
//   3 ADD.s (A3),D2    read-only access to X3    7 EOR.s D6,(A5)    RMW on X5
//   4 ADD.s D4,D3      register-register         8 MOVE.s (A3),D7   read-only access to X3
// in two variants: A = the 8 instructions one after the other (no instruction in between), then SR, D2, D3, D7, A1..A5
// saved; B = every instruction preceded by MOVE #ccr,CCR (discriminating input CCR) and followed by MOVE SR,(abs).W,
// so that the flags of each one are checked. X1..X5 are distinct targets with the neighbouring cells at sentinel; the allowed
// writes are ONLY X1, X2, X5 and the results: a write originating from a read (X3, X4) or landing at the address of a previous
// RMW fails. Checks: memory of X1 (two RMWs), X2, X5; D2, D3, D7, A1..A5; SR; FC of every cycle; DTACK
// immediate and delayed by 3 clocks.
// What it does NOT test: the cycles of the single RMW (tb_m68k_rmw), timing.

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

// ---------------------------------------------------------------- outcome
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

// ---------------------------------------------------------------- allowed writes, areas with allowed FCs, expectations
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
    // testbench invariant: no expectation matches the preloaded content (a missing write must fail)
    for (const Exp& e : exps)
        if (e.chk_init && mrdn(e.addr, e.nbytes) == e.val) check_eq("TESTBENCH: expectation equal to the initial content at 0x" + hx(e.addr) + " (" + e.what + ")", 0, 1);
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
//   MOVE:              N, Z from the data, V = 0, C = 0, X unchanged
enum { OP_ADD = 0, OP_SUB, OP_AND, OP_OR, OP_CMP, OP_EOR };
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
static uint8_t move_ccr(int sz, uint32_t v, uint8_t ccr_in) {
    v &= szmask(sz);
    return (uint8_t)((ccr_in & 0x10) | ((v & szmsb(sz)) ? 0x08 : 0) | (v == 0 ? 0x04 : 0));
}

// ================================================================ testbench program
static const char* const SZN[3] = {".B", ".W", ".L"};
static const uint16_t MSZ[3] = {1, 3, 2};   // size field of MOVE
static const char* const STEPN[8] = {"ADD D0,(A1)", "MOVE D1,(A2)", "ADD (A3),D2", "ADD D4,D3", "SUB D4,(A1)", "CMP (A4),D5", "EOR D6,(A5)", "MOVE (A3),D7"};
struct St { uint32_t D[8]; uint32_t mem1, mem2, mem5; uint8_t ccr; };
static uint16_t step_op(int i, int sz) {
    switch (i) {
    case 0: return (uint16_t)(0xD000 | (0 << 9) | ((4 + sz) << 6) | 0x11);   // ADD.s D0,(A1)
    case 1: return (uint16_t)((MSZ[sz] << 12) | (2 << 9) | (2 << 6) | 0x01); // MOVE.s D1,(A2)
    case 2: return (uint16_t)(0xD000 | (2 << 9) | (sz << 6) | 0x13);         // ADD.s (A3),D2
    case 3: return (uint16_t)(0xD000 | (3 << 9) | (sz << 6) | 0x04);         // ADD.s D4,D3
    case 4: return (uint16_t)(0x9000 | (4 << 9) | ((4 + sz) << 6) | 0x11);   // SUB.s D4,(A1)
    case 5: return (uint16_t)(0xB000 | (5 << 9) | (sz << 6) | 0x14);         // CMP.s (A4),D5
    case 6: return (uint16_t)(0xB000 | (6 << 9) | ((4 + sz) << 6) | 0x15);   // EOR.s D6,(A5)
    default: return (uint16_t)((MSZ[sz] << 12) | (7 << 9) | (0 << 6) | 0x13); // MOVE.s (A3),D7
    }
}
// effect of step i on the state (m3, m4: fixed content of X3, X4, never written)
static void step_model(int i, int sz, uint32_t m3, uint32_t m4, St& st) {
    const uint32_t M = szmask(sz);
    AluRes e{0, 0};
    switch (i) {
    case 0: e = alu_model(OP_ADD, sz, st.D[0], st.mem1, st.ccr); st.mem1 = e.r; st.ccr = e.ccr; break;
    case 1: st.mem2 = st.D[1] & M; st.ccr = move_ccr(sz, st.D[1], st.ccr); break;
    case 2: e = alu_model(OP_ADD, sz, m3, st.D[2], st.ccr); st.D[2] = (st.D[2] & ~M) | e.r; st.ccr = e.ccr; break;
    case 3: e = alu_model(OP_ADD, sz, st.D[4], st.D[3], st.ccr); st.D[3] = (st.D[3] & ~M) | e.r; st.ccr = e.ccr; break;
    case 4: e = alu_model(OP_SUB, sz, st.D[4], st.mem1, st.ccr); st.mem1 = e.r; st.ccr = e.ccr; break;
    case 5: e = alu_model(OP_CMP, sz, m4, st.D[5], st.ccr); st.ccr = e.ccr; break;
    case 6: e = alu_model(OP_EOR, sz, st.D[6], st.mem5, st.ccr); st.mem5 = e.r; st.ccr = e.ccr; break;
    default: st.D[7] = (st.D[7] & ~M) | (m3 & M); st.ccr = move_ccr(sz, m3, st.ccr); break;
    }
}
static void build() {
    const uint32_t OPS = 0x2000;   // targets: group g at OPS + 0x40*g, X1..X5 in 8-byte slots
    asm_pc = PC0;
    W(0x46FC); W(0x2700);          // MOVE #$2700,SR: S=1, T=0, I=7 explicit
    int g = 0;
    for (int sz = 0; sz < 3; sz++)
        for (int var = 0; var < 2; var++, g++) {
            const uint32_t M = szmask(sz), H = szmsb(sz); const int nb = szbytes(sz);
            const std::string gn = std::string("variant ") + (var == 0 ? "A (adjacent)" : "B (flags per instruction)") + ", size " + SZN[sz] + ": ";
            uint32_t X[6];
            for (int i = 1; i <= 5; i++) {
                const uint32_t slot = OPS + 0x40u * (uint32_t)g + 8u * (uint32_t)(i - 1);
                fill(slot, 8, SENT);
                X[i] = slot + 2u + ((sz == 0 && (i & 1)) ? 1u : 0u);   // byte: X1, X3, X5 odd, X2, X4 even
            }
            const uint32_t m1 = 1, m3 = 0x10, m4 = 0x33, m5 = 0x5A5A5A5Au & M;
            mwrn(X[1], nb, m1); mwrn(X[3], nb, m3); mwrn(X[4], nb, m4); mwrn(X[5], nb, m5);
            allow(X[1], (uint32_t)nb); allow(X[2], (uint32_t)nb); allow(X[5], (uint32_t)nb);
            // registers: low part = operand, high part = pattern different for each register
            const uint32_t lo[8] = {H - 1, H, M - 0x0Fu, H - 1, 1, 0x33, M, 0x00};
            St st{};
            for (int r = 0; r < 8; r++) st.D[r] = ((0xA5A5A5A5u ^ (0x01010101u * (uint32_t)r)) & ~M) | (lo[r] & M);
            st.D[7] = 0x96969696u;
            st.mem1 = m1; st.mem2 = 0; st.mem5 = m5;
            for (int r = 0; r < 8; r++) { W((uint16_t)(0x203C | (r << 9))); L(st.D[r]); }          // MOVE.L #v,Dr
            for (int a = 1; a <= 5; a++) { W((uint16_t)(0x207C | (a << 9))); L(X[a]); }           // MOVEA.L #Xa,Aa
            if (var == 0) {
                st.ccr = 0x1F;
                W(0x44FC); W(st.ccr);                                                            // MOVE #$1F,CCR
                for (int i = 0; i < 8; i++) { W(step_op(i, sz)); step_model(i, sz, m3, m4, st); }
                const uint32_t R = res_slot(34);
                W(0x40F8); W((uint16_t)R);                                                       // MOVE SR,(R).W
                W(0x21C2); W((uint16_t)(R + 2)); W(0x21C3); W((uint16_t)(R + 6)); W(0x21C7); W((uint16_t)(R + 10));
                for (int a = 1; a <= 5; a++) { W((uint16_t)(0x21C8 | a)); W((uint16_t)(R + 14 + 4 * (a - 1))); }   // MOVE.L Aa,(..).W
                expect_mem(R, 2, 0x2700u | st.ccr, gn + "SR after the last instruction (MOVE (A3),D7; X from the SUB)");
                for (int a = 1; a <= 5; a++) expect_mem(R + 14 + 4 * (uint32_t)(a - 1), 4, X[a], gn + "A" + std::to_string(a) + " unchanged");
            } else {
                for (int i = 0; i < 8; i++) {
                    St probe = st; probe.ccr = 0; step_model(i, sz, m3, m4, probe);
                    const uint8_t ccr_in = (i % 3 == 0) ? (uint8_t)(~probe.ccr & 0x1F) : (i % 3 == 1) ? 0x1F : 0x00;
                    st.ccr = ccr_in;
                    W(0x44FC); W(ccr_in);                                                        // MOVE #ccr_in,CCR
                    W(step_op(i, sz)); step_model(i, sz, m3, m4, st);
                    const uint32_t R = res_slot(2);
                    W(0x40F8); W((uint16_t)R);                                                   // MOVE SR,(R).W
                    expect_mem(R, 2, 0x2700u | st.ccr, gn + STEPN[i] + ": SR (input CCR 0x" + hx(ccr_in) + ")");
                }
                const uint32_t R = res_slot(12);
                W(0x21C2); W((uint16_t)R); W(0x21C3); W((uint16_t)(R + 4)); W(0x21C7); W((uint16_t)(R + 8));
                expect_mem(R, 4, st.D[2], gn + "D2 = D2 + (A3)");
                expect_mem(R + 4, 4, st.D[3], gn + "D3 = D3 + D4");
                expect_mem(R + 8, 4, st.D[7], gn + "D7 = (A3), high part unchanged");
            }
            if (var == 0) {
                const uint32_t R = res_ptr - 34u;   // same results slot as variant A
                expect_mem(R + 2, 4, st.D[2], gn + "D2 = D2 + (A3)");
                expect_mem(R + 6, 4, st.D[3], gn + "D3 = D3 + D4");
                expect_mem(R + 10, 4, st.D[7], gn + "D7 = (A3), high part unchanged");
            }
            expect_mem(X[1], nb, st.mem1, gn + "X1 after ADD and SUB (D0 then D4)");
            expect_mem(X[2], nb, st.mem2, gn + "X2 = D1 (MOVE after the RMW, not at the address of the RMW)");
            expect_mem(X[5], nb, st.mem5, gn + "X5 = X5 EOR D6");
        }
    end_addr = asm_pc; W(0x60FE);                      // BRA.S *
    // areas and allowed FCs (UM, Function Code Outputs)
    region(0x1000, 0x2000, FC_SD, "results");
    region(OPS, OPS + 0x40u * 6u, FC_SD, "targets");
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
