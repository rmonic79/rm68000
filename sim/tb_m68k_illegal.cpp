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

// tb_m68k_illegal.cpp - illegal instruction (vector 4), line A (vector 10) and line F (vector 11) exceptions, on the
// full processor (m68k_seq shim). Expected values by hand from MC68000UM ch. 6 (Illegal and Unimplemented Instructions:
// the stacked PC is the address of the first word of the illegal or unimplemented instruction; 3-word frame at SSP-6;
// S <- 1, T <- 0, mask and CCR unchanged) and M68000PRM ch. 4 (ILLEGAL, MOVEQ, RTE) and ch. 8 (operation code map).
// What it tests (13 cases, each with its own SSP, frame read in full over preloaded cells different from the expected
// values):
//  - supervisor: ILLEGAL $4AFC; codes unassigned on the 68000 $4E7A, $4848 (PEA with An mode), $4E74, $06C0 (ADDI with
//    size 11), $4C00 (group 0100 1100 00, not MOVEM); line A $A000 and $AFFF; line F $F000 and $FFFF; each case with
//    its own mask and CCR;
//  - user (entry via RTE from a preloaded frame, then MOVEQ #-1,D4 which changes the CCR, then the code): ILLEGAL $4AFC,
//    line A $A5A5, line F $F123; the code lies right after a 64 KB boundary, so the stacked upper PC also differs from
//    that of the preloaded frame;
//  - for every case: vector (distinct handlers for vectors 4, 10, 11 that write their own number; every other vector
//    leads to the catch handler), stacked SR, stacked PC = address of the instruction, SR in the handler (S=1, T=0), SSP
//    in the handler; number of entries into the handlers (final A6 pointer);
//  - bus: FC of every cycle against the area, no write outside the expected areas; DTACK immediate and delayed by 3
//    clocks.
// Not tested: every illegal code (the decoder testbenches do that on the 65,536 codes), illegal instructions with T=1,
// group 0 exceptions, timing.

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
static void mwr32(uint32_t a, uint32_t v) { mwrn(a, 4, v); }
static constexpr uint8_t SENT = 0xEE;
static void fill(uint32_t a, uint32_t n, uint8_t v) { for (uint32_t i = 0; i < n; i++) mwr8(a + i, v); }

// ---------------------------------------------------------------- assembler (encodings from M68000PRM ch. 8)
static uint32_t asm_pc = 0;
static void W(uint16_t v) { mwr16(asm_pc, v); asm_pc += 2; }
static void L(uint32_t v) { W((uint16_t)(v >> 16)); W((uint16_t)v); }

// ---------------------------------------------------------------- allowed writes, areas with allowed FCs, expected values
static std::set<uint32_t> wr_ok;
static void allow(uint32_t a, uint32_t n) { for (uint32_t i = 0; i < n; i++) wr_ok.insert((a + i) & 0xFFFFFFu); }
static constexpr unsigned FC_UD = 1u << 1, FC_UP = 1u << 2, FC_SD = 1u << 5, FC_SP = 1u << 6;   // UM: FC 1/2/5/6
struct Region { uint32_t lo, hi; unsigned fcmask; std::string name; };
static std::vector<Region> regions;
static void region(uint32_t lo, uint32_t hi, unsigned fcmask, const std::string& name) { regions.push_back(Region{lo, hi, fcmask, name}); }
static const Region* find_region(uint32_t a) { for (const Region& r : regions) if (a >= r.lo && a < r.hi) return &r; return nullptr; }
struct Exp { uint32_t addr; int nbytes; uint32_t val; std::string what; bool chk_init; };
static std::vector<Exp> exps;
static void expect_mem(uint32_t addr, int nbytes, uint32_t val, const std::string& what, bool chk_init = true) { exps.push_back(Exp{addr, nbytes, val, what, chk_init}); }

// ---------------------------------------------------------------- bus: cycle = AS low; DTACK after dtack_lat clocks
struct Cyc { uint32_t addr; int rw, fc, uds, lds; uint16_t wdata; };
static std::vector<Cyc> cyc;
static Cyc cur;
static bool in_cyc = false;
static int wait_n = 0, dtack_lat = 0;
static uint32_t end_addr = 0, catch_addr = 0xFFFFFFFFu;
static int end_hits = 0;
static bool caught = false;
static void tick() { dut->clk = 0; dut->eval(); t_ticks++; dut->clk = 1; dut->eval(); t_ticks++; }
static void close_cycle() {
    cyc.push_back(cur); in_cyc = false;
    if (cur.rw && cur.addr == end_addr) end_hits++;
    if (cur.rw && cur.addr == catch_addr) caught = true;
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
static void bank_checks();
static void run_once(int lat) {
    mem.clear(); regions.clear(); wr_ok.clear(); exps.clear(); cyc.clear();
    in_cyc = false; end_hits = 0; caught = false; end_addr = 0; catch_addr = 0xFFFFFFFFu; asm_pc = PC0;
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
    while (n < MAX_TICKS && end_hits < 3 && !caught) { bus_step(); n++; }
    check_eq("end of run: fetch of the BRA.S * at 0x" + hx(end_addr) + " seen 3 times within the tick limit", 1, end_hits >= 3 ? 1u : 0u);
    check_eq("no jump to the catch handler (unexpected exception or vector)", 0, caught ? 1u : 0u);
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
    bank_checks();
    dut->final(); delete dut; dut = nullptr;
}

// ================================================================ testbench program
static constexpr uint32_t CATCH = 0x7F00;                     // catch handler: BRA.S * (the testbench stops and fails)
static constexpr uint32_t H4 = 0x5000, H10 = 0x5100, H11 = 0x5200;
static constexpr uint32_t RB = 0x1000;                        // slot of case k at RB + 8k: SR, SSP, vector number
struct ICase { uint16_t op; int vec; bool user; uint16_t sr; const char* name; };
// sr: supervisor = SR set with MOVE to SR before the code; user = SR of the frame preloaded for RTE (S=0)
static const ICase CASES[] = {
    {0x4AFC, 4, false, 0x201F, "ILLEGAL $4AFC"},
    {0x4E7A, 4, false, 0x2100, "$4E7A (unassigned on the 68000)"},
    {0x4848, 4, false, 0x2215, "$4848 (PEA with An mode, not allowed)"},
    {0x4E74, 4, false, 0x230A, "$4E74 (unassigned on the 68000)"},
    {0x06C0, 4, false, 0x2411, "$06C0 (ADDI with size 11)"},
    {0x4C00, 4, false, 0x250E, "$4C00 (0100 1100 00: not MOVEM, unassigned)"},
    {0xA000, 10, false, 0x2613, "line A $A000"},
    {0xAFFF, 10, false, 0x270C, "line A $AFFF"},
    {0xF000, 11, false, 0x2019, "line F $F000"},
    {0xFFFF, 11, false, 0x2106, "line F $FFFF"},
    {0x4AFC, 4, true, 0x0315, "ILLEGAL $4AFC in user mode"},
    {0xA5A5, 10, true, 0x0507, "line A $A5A5 in user mode"},
    {0xF123, 11, true, 0x0614, "line F $F123 in user mode"},
};
static constexpr int NC = (int)(sizeof(CASES) / sizeof(CASES[0]));

static void build() {
    // vectors 2..255 -> catch; 4 (illegal), 10 (line A), 11 (line F) -> distinct handlers (UM ch. 6, vector table)
    for (uint32_t v = 2; v < 256; v++) mwr32(4 * v, CATCH);
    mwr32(4 * 4, H4); mwr32(4 * 10, H10); mwr32(4 * 11, H11);
    asm_pc = CATCH; W(0x60FE);
    catch_addr = CATCH;
    fill(0x2000, 0x400, SENT);                       // stacks filled with sentinels
    fill(RB, 0x100, SENT);                           // results filled with sentinels
    allow(RB, 8u * NC + 4u);
    // handlers: MOVE SR,(A6)+ ; MOVE.L A7,(A6)+ ; MOVE.W #vector,(A6)+ ; JMP (A5)
    const uint32_t HV[3] = {H4, H10, H11};
    const uint16_t VN[3] = {4, 10, 11};
    for (int h = 0; h < 3; h++) {
        asm_pc = HV[h];
        W(0x40DE); W(0x2CCF); W(0x3CFC); W(VN[h]); W(0x4ED5);
    }
    asm_pc = PC0;
    W(0x46FC); W(0x2700);                            // MOVE #$2700,SR: explicit initial state
    W(0x2C7C); L(RB);                                // MOVEA.L #RB,A6: pointer to the slots
    int nu = 0;
    for (int k = 0; k < NC; k++) {
        const ICase& c = CASES[k];
        uint32_t slot = RB + 8u * (uint32_t)k;
        uint32_t frame, pc_exp; uint16_t sr_exp;
        std::string tn = std::string(c.name) + ": ";
        if (!c.user) {
            uint32_t S = 0x2100u + 0x10u * (uint32_t)k;
            W(0x2E7C); L(S);                         // MOVEA.L #S,A7
            uint32_t ia = asm_pc + 10;               // after MOVEA.L #,A5 (6 bytes) and MOVE #,SR (4 bytes)
            W(0x2A7C); L(ia + 2);                    // MOVEA.L #continuation,A5
            W(0x46FC); W(c.sr);                      // MOVE #sr,SR
            W(c.op);                                 // code under test, at ia
            frame = S - 6; pc_exp = ia; sr_exp = c.sr;
            allow(frame, 6);
        } else {
            uint32_t ufr = 0x2300u + 0x10u * (uint32_t)nu;
            uint32_t upc = 0x0001FFFEu + 0x10000u * (uint32_t)nu;
            nu++;
            mwr16(upc, 0x78FF);                      // MOVEQ #-1,D4 (user): N=1 Z=0 V=0 C=0, X unchanged (PRM 4 MOVEQ)
            mwr16(upc + 2, c.op);                    // code under test, first address after the 64 KB boundary
            mwr16(ufr, c.sr); mwr32(ufr + 2, upc);   // frame for RTE
            W(0x2E7C); L(ufr);                       // MOVEA.L #ufr,A7
            uint32_t cont = asm_pc + 8;              // after MOVEA.L #,A5 (6 bytes) and RTE (2 bytes)
            W(0x2A7C); L(cont);                      // MOVEA.L #continuation,A5
            W(0x4E73);                               // RTE -> user mode at upc
            frame = ufr;                             // SSP after RTE = ufr + 6, frame at ufr
            pc_exp = upc + 2;
            sr_exp = (uint16_t)((c.sr & 0xFF00) | (c.sr & 0x10) | 0x08);
            allow(frame, 6);
            region(upc - 0x10, upc + 0x20, FC_SP | FC_UP, std::string("user code of ") + c.name);
        }
        // UM ch. 6: frame = SR before the exception, PC = address of the illegal / line A / line F instruction
        expect_mem(frame, 2, sr_exp, tn + "frame, stacked SR");
        expect_mem(frame + 2, 2, pc_exp >> 16, tn + "frame, upper PC = address of the instruction");
        expect_mem(frame + 4, 2, pc_exp & 0xFFFF, tn + "frame, lower PC = address of the instruction");
        // handler: S <- 1, T <- 0, mask and CCR unchanged; SSP = frame; vector number of the executed handler
        expect_mem(slot, 2, (uint32_t)((sr_exp | 0x2000) & 0x7FFF), tn + "SR in the handler (S=1, T=0)");
        expect_mem(slot + 2, 4, frame, tn + "SSP in the handler");
        expect_mem(slot + 6, 2, (uint32_t)c.vec, tn + "vector taken (number written by the handler)");
    }
    uint32_t rxa6 = RB + 8u * NC;
    W(0x21CE); W((uint16_t)rxa6);                    // MOVE.L A6,(rxa6).W
    end_addr = asm_pc; W(0x60FE);                    // BRA.S *
    expect_mem(rxa6, 4, RB + 8u * NC, "final A6: a single entry into the handlers per case (8 bytes each)");

    // allowed areas and FC (UM, Function Code Outputs; vectors in the supervisor data space)
    region(0x0, 0x400, FC_SD, "vector table");
    region(RB, RB + 0x100, FC_SD, "results");
    region(0x2000, 0x2400, FC_SD, "stacks");
    region(0x4000, 0x4800, FC_SP, "program");
    region(0x5000, 0x5300, FC_SP, "handlers");
    region(CATCH, CATCH + 0x10, FC_SP, "catch");
}

static void bank_checks() {}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    run_once(0);
    run_once(3);
    printf("checks run: %d, failed: %d\n", n_checks, n_fails);
    puts(n_fails ? "RESULT: FAIL" : "RESULT: PASS");
    return n_fails ? 1 : 0;
}
