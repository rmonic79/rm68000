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

// tb_m68k_priv.cpp - privilege violation (vector 8) for EVERY privileged instruction executed in user mode, on the
// whole processor (shim m68k_seq). Expectations written by hand from MC68000UM ch. 6 (Privilege Violations: the instruction is not
// executed, the stacked PC is the address of its first word; 3-word frame at SSP-6; S <- 1, T <- 0, mask and
// CCR unchanged; Function Code Outputs) and M68000PRM ch. 4 (MOVE to SR, ANDI/ORI/EORI to SR, MOVE USP, RTE, STOP, RESET:
// privileged; MOVE, RTE) and ch. 8 (encodings).
// What it tests (10 cases): MOVE #$2700,SR; MOVE D0,SR (D0 = $2700); ANDI #0,SR; ORI #$2700,SR; EORI #$2000,SR; MOVE A0,USP;
// MOVE USP,A1; RTE; STOP #$2700; RESET. Each case enters user mode with RTE from a preloaded frame (its own mask and CCR),
// executes MOVE.W D4,(abs).W in user mode (write with user data FC, CCR changed: N=1 Z=0 V=0 C=0, X unchanged) and then
// the privileged instruction, which sits right after a 64 KB boundary (the stacked high PC differs from that of the
// preloaded frame). For each case: vector 8 (handler distinct from that of vector 4, every other vector leads to the catch
// handler), stacked SR (= user SR: the instruction had no effect), stacked PC = address of the privileged
// instruction (not of its extension words), SR in the handler (S=1, T=0), SSP in the handler. At the end: USP unchanged
// (MOVE A0,USP not executed), A1 unchanged (MOVE USP,A1 not executed), a single entry into the handler per case, SR in
// supervisor mode; STOP and RESET in user mode do not stop the CPU (the program reaches the end).
// Bus: FC of every cycle against the area, no write outside the expected areas; DTACK immediate and delayed by 3 clocks.
// What it does NOT test: the same instructions in supervisor mode (other testbenches), MOVE <ea>,SR with a source in memory, the
// RESET output signal (the shim holds it at 0), timing.

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
static void mwr32(uint32_t a, uint32_t v) { mwrn(a, 4, v); }
static constexpr uint8_t SENT = 0xEE;
static void fill(uint32_t a, uint32_t n, uint8_t v) { for (uint32_t i = 0; i < n; i++) mwr8(a + i, v); }

// ---------------------------------------------------------------- assembler (encodings from M68000PRM ch. 8)
static uint32_t asm_pc = 0;
static void W(uint16_t v) { mwr16(asm_pc, v); asm_pc += 2; }
static void L(uint32_t v) { W((uint16_t)(v >> 16)); W((uint16_t)v); }

// ---------------------------------------------------------------- allowed writes, areas with allowed FCs, expectations
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
    while (n < MAX_TICKS && end_hits < 3 && !caught) { bus_step(); n++; }
    check_eq("end of run: fetch of BRA.S * at 0x" + hx(end_addr) + " seen 3 times within the tick limit", 1, end_hits >= 3 ? 1u : 0u);
    check_eq("no jump to the catch handler (unexpected exception or vector)", 0, caught ? 1u : 0u);
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
    bank_checks();
    dut->final(); delete dut; dut = nullptr;
}

// ================================================================ testbench program
static constexpr uint32_t CATCH = 0x7F00;                 // catch handler: BRA.S * (the testbench stops and fails)
static constexpr uint32_t H8 = 0x5000, H4 = 0x5100;       // handlers of vectors 8 (privilege) and 4 (illegal)
static constexpr uint32_t RB = 0x1000;                    // slot of case j at RB + 8j: SR, SSP, vector number
static constexpr uint32_t RX = 0x1100;                    // final state in supervisor mode
static constexpr uint32_t UW = 0x1800;                    // words written in user mode (one per case)
static constexpr uint32_t USP_V = 0x00C0FFE0;             // USP (never used to access memory)
struct PCase { uint16_t w[2]; int nw; const char* name; };
static const PCase CASES[] = {
    {{0x46FC, 0x2700}, 2, "MOVE #$2700,SR"},
    {{0x46C0, 0x0000}, 1, "MOVE D0,SR"},
    {{0x027C, 0x0000}, 2, "ANDI #$0000,SR"},
    {{0x007C, 0x2700}, 2, "ORI #$2700,SR"},
    {{0x0A7C, 0x2000}, 2, "EORI #$2000,SR"},
    {{0x4E60, 0x0000}, 1, "MOVE A0,USP"},
    {{0x4E69, 0x0000}, 1, "MOVE USP,A1"},
    {{0x4E73, 0x0000}, 1, "RTE"},
    {{0x4E72, 0x2700}, 2, "STOP #$2700"},
    {{0x4E70, 0x0000}, 1, "RESET"},
};
static constexpr int NC = (int)(sizeof(CASES) / sizeof(CASES[0]));
// CCR of the user frame of case j (the mask is j mod 8)
static const uint8_t UCCR[NC] = {0x15, 0x07, 0x1F, 0x00, 0x13, 0x05, 0x1D, 0x02, 0x11, 0x0F};
static uint16_t usr_of(int j) { return (uint16_t)(((j & 7) << 8) | UCCR[j]); }
// stacked SR: MOVE.W D4 ($8000) in user mode sets N=1 Z=0 V=0 C=0 and leaves X (PRM 4 MOVE); T=0, S=0, mask unchanged
static uint16_t pushed_of(int j) { uint16_t u = usr_of(j); return (uint16_t)((u & 0x0700) | (u & 0x10) | 0x08); }

static void build() {
    // vectors 2..255 -> catch; 8 and 4 -> distinct handlers (UM ch. 6, vector table)
    for (uint32_t v = 2; v < 256; v++) mwr32(4 * v, CATCH);
    mwr32(4 * 8, H8); mwr32(4 * 4, H4);
    asm_pc = CATCH; W(0x60FE);
    catch_addr = CATCH;
    fill(0x2000, 0x400, SENT);                       // stacks filled with sentinels
    fill(RB, 0x200, SENT);                           // results filled with sentinels
    fill(UW, 0x20, SENT);
    allow(RB, 8u * NC);
    allow(RX, 14);
    allow(UW, 2u * NC);
    // handlers: MOVE SR,(A6)+ ; MOVE.L A7,(A6)+ ; MOVE.W #vector,(A6)+ ; JMP (A5)
    asm_pc = H8; W(0x40DE); W(0x2CCF); W(0x3CFC); W(8); W(0x4ED5);
    asm_pc = H4; W(0x40DE); W(0x2CCF); W(0x3CFC); W(4); W(0x4ED5);
    asm_pc = PC0;
    W(0x46FC); W(0x2700);                            // MOVE #$2700,SR: explicit initial state
    W(0x2C7C); L(RB);                                // MOVEA.L #RB,A6: pointer to the slots
    W(0x267C); L(USP_V);                             // MOVEA.L #USP_V,A3
    W(0x4E63);                                       // MOVE A3,USP
    W(0x203C); L(0x00002700);                        // MOVE.L #$2700,D0   source of MOVE D0,SR
    W(0x207C); L(0x0BADBAD0);                        // MOVEA.L #$0BADBAD0,A0  value that MOVE A0,USP must not write
    W(0x227C); L(0x11111110);                        // MOVEA.L #$11111110,A1  MOVE USP,A1 must not change it
    W(0x283C); L(0x00008000);                        // MOVE.L #$8000,D4
    for (int j = 0; j < NC; j++) {
        const PCase& c = CASES[j];
        uint32_t ufr = 0x2300u + 0x10u * (uint32_t)j;
        uint32_t upc = 0x0001FFFCu + 0x10000u * (uint32_t)j;   // MOVE.W D4,(abs).W at upc, privileged instruction at upc+4
        uint32_t uw = UW + 2u * (uint32_t)j;
        mwr16(upc, 0x31C4); mwr16(upc + 2, (uint16_t)uw);    // MOVE.W D4,(uw).W
        for (int i = 0; i < c.nw; i++) mwr16(upc + 4 + 2u * (uint32_t)i, c.w[i]);
        mwr16(ufr, usr_of(j)); mwr32(ufr + 2, upc);         // frame for RTE: user SR, PC = upc
        W(0x2E7C); L(ufr);                           // MOVEA.L #ufr,A7
        uint32_t cont = asm_pc + 8;                  // after MOVEA.L #,A5 (6 bytes) and RTE (2 bytes)
        W(0x2A7C); L(cont);                          // MOVEA.L #continuation,A5
        W(0x4E73);                                   // RTE -> user mode at upc
        allow(ufr, 6);
        region(upc - 0x10, upc + 0x20, FC_SP | FC_UP, std::string("user code of ") + c.name);
        std::string tn = std::string(c.name) + " in user mode: ";
        uint16_t ps = pushed_of(j);
        uint32_t pc_exp = upc + 4;
        // UM ch. 6: frame at SSP-6 (SSP after RTE = ufr+6), SR before the violation, PC = address of the instruction
        expect_mem(ufr, 2, ps, tn + "frame, stacked SR (the instruction was not executed)");
        expect_mem(ufr + 2, 2, pc_exp >> 16, tn + "frame, high PC = address of the privileged instruction");
        expect_mem(ufr + 4, 2, pc_exp & 0xFFFF, tn + "frame, low PC = address of the privileged instruction");
        uint32_t slot = RB + 8u * (uint32_t)j;
        expect_mem(slot, 2, (uint32_t)((ps | 0x2000) & 0x7FFF), tn + "SR in the handler (S=1, T=0, mask and CCR unchanged)");
        expect_mem(slot + 2, 4, ufr, tn + "SSP in the handler");
        expect_mem(slot + 6, 2, 8, tn + "vector taken = 8 (number written by the handler)");
        expect_mem(uw, 2, 0x8000, tn + "MOVE.W D4,(abs).W executed in user mode before the violation");
    }
    W(0x40F8); W((uint16_t)(RX + 12));               // MOVE SR,(RX+12).W
    W(0x4E6A);                                       // MOVE USP,A2
    W(0x21CA); W((uint16_t)RX);                      // MOVE.L A2,(RX).W
    W(0x21C9); W((uint16_t)(RX + 4));                // MOVE.L A1,(RX+4).W
    W(0x21CE); W((uint16_t)(RX + 8));                // MOVE.L A6,(RX+8).W
    end_addr = asm_pc; W(0x60FE);                    // BRA.S *
    expect_mem(RX, 4, USP_V, "final USP unchanged (MOVE A0,USP in user mode not executed)");
    expect_mem(RX + 4, 4, 0x11111110, "final A1 unchanged (MOVE USP,A1 in user mode not executed)");
    expect_mem(RX + 8, 4, RB + 8u * NC, "final A6: a single entry into the handler per case (8 bytes each)");
    // SR after the last handler: S=1, mask of the last case; the handler executes last MOVE.L A7,(A6)+ (A7 = $2390)
    // and MOVE.W #8,(A6)+, which give N=0 Z=0 V=0 C=0 and leave X (PRM 4 MOVE); JMP does not change the flags
    {
        uint16_t u = usr_of(NC - 1);
        expect_mem(RX + 12, 2, (uint32_t)(0x2000 | (u & 0x0700) | (u & 0x10)), "SR in supervisor mode after the last handler");
    }

    // areas and allowed FCs (UM, Function Code Outputs; vectors in the supervisor data space)
    region(0x0, 0x400, FC_SD, "vector table");
    region(RB, RB + 0x200, FC_SD, "results");
    region(UW, UW + 0x20, FC_UD, "words written in user mode");
    region(0x2000, 0x2400, FC_SD, "stacks");
    region(0x4000, 0x4800, FC_SP, "program");
    region(0x5000, 0x5200, FC_SP, "handlers");
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
