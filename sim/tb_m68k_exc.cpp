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

// tb_m68k_exc.cpp - instruction exceptions TRAP #0..#15, RTE and TRACE, on the full processor (shim m68k_seq).
// Expected values computed by hand from MC68000UM ch. 6 (Exception Processing: sequence, 3-word frame SR / PC high / PC
// low at SSP-6, vectors, TRAP, Tracing) and from M68000PRM ch. 4 (TRAP, RTE, MOVE to SR, MOVE from SR, MOVE USP, ADD, SUB)
// and sec. 1.3 (status register: bits 14, 12, 11, 7, 6, 5 are not implemented and read as zero).
// What it tests:
//  - TRAP #n for every n = 0..15: vector 32+n (address 0x80+4n) to a DISTINCT handler per vector, which writes into its
//    own slot (a wrong vector leaves a slot at the sentinel); a different SSP for every TRAP, frame read in full
//    (SR, PC high, PC low) on cells preloaded with 0xEEEE; stacked PC = instruction after the TRAP; SR in the handler
//    (S=1, T=0, mask and CCR unchanged: input SR with mask and CCR different for every n); SSP in the handler = SSP-6;
//    RTE: the handler changes mask and CCR before RTE, RTE must restore those of the frame; SSP after RTE = SSP from
//    before. For odd n the handler copies the stacked SR and rewrites it with a new value (n=1: all bits at 1 except
//    T, the unimplemented bits must read as 0): RTE must load that one. TRAP #15 returns in USER mode
//    (A7 = USP after RTE, writes with user data FC, final BRA.S fetched with user program FC).
//  - TRACE (UM ch. 6, Tracing: with T at 1 at the start of an instruction, trace exception after it, vector 9):
//    T set by MOVE to SR in supervisor mode (traces the following ADD.W, not the MOVE) and by RTE to user mode (traces
//    the SUB.L in user mode, whose next PC crosses a 64 KB boundary: 0x0001FFFE -> 0x00020000). Frame (SR with T=1 and
//    CCR of the traced instruction, next PC), SR in the handler (S=1, T=0), SSP, USP unchanged, result.
//  - TRACE of an instruction that raises its own exception (UM 6.3.8): TRAPV with V = 1 under T = 1. The TRAPV exception is
//    processed first (frame with SR T = 1 and the CCR, PC after the TRAPV), then the trace exception (frame below it with the
//    SR of the entry into the TRAPV handler and PC = that handler): the trace handler runs first and returns into the TRAPV
//    handler, which clears T in its frame and returns after the TRAPV. Both frames, both handlers' SR and SSP, the order.
//  - Bus: FC of every cycle against the area (program/data, supervisor/user; vectors = supervisor data), no
//    write outside the expected areas, no jump to the catch handler (all other vectors).
//    The program runs twice: DTACK immediate and DTACK delayed by 3 clocks, same expected values.
// Group 0 exceptions are in tb_m68k_berr.cpp, interrupts in tb_m68k_irq.cpp and tb_m68k_irq_multi.cpp. The order of the
// two frame writes is not specified by the manual and is not checked here; cycle counts are in the timing benches.

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
static constexpr uint32_t CATCH = 0x7F00;                 // catch handler: BRA.S * (the testbench stops and fails)
static constexpr uint32_t H_BASE = 0x5000;                // handler of TRAP #n at H_BASE + 0x40*n
static constexpr uint32_t H_TS = 0x5500, H_TU = 0x5580;   // TRACE handlers (supervisor, user)
static constexpr uint32_t H_TV = 0x5600, H_TS2 = 0x5640; // TRAPV handler and the trace handler that returns (traced TRAPV)
static constexpr uint32_t RTV = 0x1140;                   // results of the traced TRAPV
static constexpr uint32_t SSP_TV = 0x2380;                // SSP for the traced TRAPV (two frames below it)
static constexpr uint32_t RB = 0x1000;                    // slot of TRAP #n at RB + 0x10*n
static constexpr uint32_t RT = 0x1100;                    // TRACE results
static constexpr uint32_t SSP_TS = 0x2200;                // SSP for the TRACE in supervisor mode
static constexpr uint32_t F_TU = 0x22FA;                  // frame preloaded for RTE -> user with T=1
static constexpr uint32_t USP_V = 0x00ABCDE0;             // USP (never used to access memory)
static constexpr uint32_t UT = 0x0001FFFE;                // traced user instruction
static constexpr uint32_t FIN = 0x4800;                   // final block: TRAP #15 and return to user mode
// input CCR of TRAP #n: every bit X N Z V C at 1 and at 0 over several cases
static const uint8_t CCR_TAB[16] = {0x1F, 0x00, 0x15, 0x0A, 0x10, 0x08, 0x04, 0x02, 0x01, 0x1E, 0x1D, 0x1B, 0x17, 0x0F, 0x11, 0x0E};
// SR written by the handler into the frame for the odd TRAPs (RTE must load it): 0x7FFF = unimplemented bits at 1 (T=0);
// 0x0013 = user mode
static const uint16_t NEWSR[16] = {0, 0x7FFF, 0, 0x2015, 0, 0x230A, 0, 0x2410, 0, 0x2108, 0, 0x2604, 0, 0x2502, 0, 0x0013};
static uint32_t ssp_of(int n) { return 0x2100u + 0x10u * (uint32_t)n; }
// SR before TRAP #n: S=1, T=0, mask (3n) mod 8, CCR from the table
static uint16_t sr_in(int n) { return (uint16_t)(0x2000 | (((3 * n) & 7) << 8) | CCR_TAB[n]); }
// SR set by the handler before RTE: mask 7-I and complemented CCR (different from those of the frame)
static uint16_t sr_h(int n) { return (uint16_t)(0x2000 | ((7 - ((3 * n) & 7)) << 8) | (~CCR_TAB[n] & 0x1F)); }
static uint32_t ta[16];      // address of each TRAP
static uint32_t ti_s = 0;    // address of the instruction traced in supervisor mode
static uint32_t tv_at = 0;   // address of the traced TRAPV

static void build() {
    // vectors 2..255 -> catch; then the ones used (UM ch. 6: TRAP #n = vector 32+n, trace = vector 9)
    for (uint32_t v = 2; v < 256; v++) mwr32(4 * v, CATCH);
    asm_pc = CATCH; W(0x60FE);
    catch_addr = CATCH;
    for (int n = 0; n < 16; n++) mwr32(0x80u + 4u * (uint32_t)n, H_BASE + 0x40u * (uint32_t)n);
    mwr32(0x24, H_TS);
    allow(0x24, 4);                                 // the program rewrites vector 9 with H_TU
    // stacks all at sentinel; only the 6 bytes of each frame are allowed
    fill(0x2000, 0x400, SENT);
    for (int n = 0; n < 16; n++) allow(ssp_of(n) - 6, 6);
    allow(SSP_TS - 6, 6);
    allow(SSP_TV - 12, 12); allow(RTV, 22);
    mwr32(0x1C, H_TV);                              // vector 7 (TRAPV) -> H_TV
    allow(F_TU, 6);
    mwr16(F_TU, 0x8200); mwr32(F_TU + 2, UT);       // frame for RTE: SR = T=1, S=0, I=2, CCR=0; PC = UT
    // results at sentinel
    fill(RB, 0x200, SENT);
    for (int n = 0; n < 16; n++) { uint32_t R = RB + 0x10u * (uint32_t)n; allow(R, 12); if (n & 1) allow(R + 12, 2); }
    allow(RT, 10); allow(RT + 0x10, 14);

    // ---- main program (supervisor)
    asm_pc = PC0;
    W(0x46FC); W(0x2700);                            // MOVE #$2700,SR: explicit initial state
    // traced instruction that raises its own exception (UM 6.3.8): TRAPV with V = 1 under T = 1. The TRAPV exception comes
    // first, then the trace exception; the trace handler (H_TS2, which returns) runs first, then the TRAPV handler (H_TV).
    W(0x21FC); L(H_TS2); W(0x0024);                  // MOVE.L #H_TS2,($24).W   vector 9 -> trace handler that returns
    W(0x2E7C); L(SSP_TV);                            // MOVEA.L #SSP_TV,A7
    W(0x46FC); W(0xA702);                            // MOVE #$A702,SR: T=1 S=1 I=7 V=1 (not traced itself: T was 0 at its start)
    tv_at = asm_pc; W(0x4E76);                       // TRAPV (traced; V = 1: TRAPV exception, vector 7)
    W(0x40F8); W((uint16_t)(RTV + 12));              // MOVE SR,(RTV+12).W     SR after both returns (T cleared by H_TV in its frame)
    W(0x21CF); W((uint16_t)(RTV + 14));              // MOVE.L A7,(RTV+14).W   SSP after both returns
    W(0x21FC); L(H_TS); W(0x0024);                   // MOVE.L #H_TS,($24).W    vector 9 -> H_TS for the trace cases below
    for (int n = 0; n < 15; n++) {
        uint32_t R = RB + 0x10u * (uint32_t)n;
        W(0x2E7C); L(ssp_of(n));                     // MOVEA.L #SSP_n,A7
        W(0x46FC); W(sr_in(n));                      // MOVE #sr_in,SR
        ta[n] = asm_pc; W((uint16_t)(0x4E40 | n));   // TRAP #n
        W(0x40F8); W((uint16_t)R);                   // MOVE SR,(R).W       SR after RTE
        W(0x21CF); W((uint16_t)(R + 2));             // MOVE.L A7,(R+2).W   SSP after RTE
    }
    // TRACE in supervisor mode: MOVE to SR sets T (it is not traced itself: T was 0 at its start), the ADD.W is traced
    W(0x2E7C); L(SSP_TS);                            // MOVEA.L #SSP_TS,A7
    W(0x203C); L(0x12347FFF);                        // MOVE.L #$12347FFF,D0
    W(0x223C); L(0x55550001);                        // MOVE.L #$55550001,D1
    W(0x46FC); W(0xA71F);                            // MOVE #$A71F,SR: T=1 S=1 I=7 CCR=1F
    ti_s = asm_pc; W(0xD041);                        // ADD.W D1,D0 (traced)
    W(0x4E71); W(0x4E71);                            // never executed (the handler jumps away)
    uint32_t tu = asm_pc;                            // TRACE in user mode
    W(0x207C); L(USP_V);                             // MOVEA.L #USP_V,A0
    W(0x4E60);                                       // MOVE A0,USP
    W(0x21FC); L(H_TU); W(0x0024);                   // MOVE.L #H_TU,($24).W   vector 9 -> H_TU
    W(0x243C); L(0x00000000);                        // MOVE.L #0,D2
    W(0x263C); L(0x00000001);                        // MOVE.L #1,D3
    W(0x2E7C); L(F_TU);                              // MOVEA.L #F_TU,A7 (preloaded frame)
    W(0x4E73);                                       // RTE -> user, T=1, PC = UT
    asm_pc = UT; W(0x9483);                          // SUB.L D3,D2 (user, traced)
    // final block: TRAP #15, the handler returns to user mode
    asm_pc = FIN;
    {
        uint32_t R = RB + 0xF0;
        W(0x2E7C); L(ssp_of(15));                    // MOVEA.L #SSP_15,A7
        W(0x46FC); W(sr_in(15));                     // MOVE #sr_in,SR
        ta[15] = asm_pc; W(0x4E4F);                  // TRAP #15
        W(0x40F8); W((uint16_t)R);                   // MOVE SR,(R).W       (user)
        W(0x21CF); W((uint16_t)(R + 2));             // MOVE.L A7,(R+2).W   (user: A7 = USP)
        end_addr = asm_pc; W(0x60FE);                // BRA.S *
    }
    // ---- TRAP handlers
    for (int n = 0; n < 16; n++) {
        uint32_t R = RB + 0x10u * (uint32_t)n;
        asm_pc = H_BASE + 0x40u * (uint32_t)n;
        W(0x40F8); W((uint16_t)(R + 6));             // MOVE SR,(R+6).W     SR in the handler
        W(0x21CF); W((uint16_t)(R + 8));             // MOVE.L A7,(R+8).W   SSP in the handler
        if (n & 1) {
            W(0x31D7); W((uint16_t)(R + 12));        // MOVE.W (A7),(R+12).W  copy of the stacked SR
            W(0x3EBC); W(NEWSR[n]);                  // MOVE.W #newsr,(A7)    new SR in the frame
        }
        W(0x46FC); W(sr_h(n));                       // MOVE #sr_h,SR: mask and CCR different from the frame
        W(0x4E73);                                   // RTE
    }
    // ---- traced TRAPV: TRAPV handler (vector 7) and the trace handler that returns (vector 9 during that test)
    asm_pc = H_TV;
    W(0x40F8); W((uint16_t)(RTV + 4));               // MOVE SR,(RTV+4).W            SR in the TRAPV handler
    W(0x21CF); W((uint16_t)(RTV + 6));               // MOVE.L A7,(RTV+6).W          SSP in the TRAPV handler
    W(0x31F8); W((uint16_t)RTV); W((uint16_t)(RTV + 10));   // MOVE.W (RTV).W,(RTV+10).W: the trace handler's marker, if it ran first
    W(0x0257); W(0x7FFF);                            // ANDI.W #$7FFF,(A7): T cleared in the stacked SR
    W(0x4E73);                                       // RTE
    asm_pc = H_TS2;
    W(0x40F8); W((uint16_t)(RTV + 2));               // MOVE SR,(RTV+2).W            SR in the trace handler (before the marker MOVE, which clears NZVC)
    W(0x31FC); W(0x5A5A); W((uint16_t)RTV);          // MOVE.W #$5A5A,(RTV).W        marker
    W(0x21CF); W((uint16_t)(RTV + 18));              // MOVE.L A7,(RTV+18).W         SSP in the trace handler
    W(0x4E73);                                       // RTE
    // ---- TRACE handlers
    asm_pc = H_TS;
    W(0x40F8); W((uint16_t)RT);                      // MOVE SR,(RT).W
    W(0x21CF); W((uint16_t)(RT + 2));                // MOVE.L A7,(RT+2).W
    W(0x21C0); W((uint16_t)(RT + 6));                // MOVE.L D0,(RT+6).W
    W(0x4EF9); L(tu);                                // JMP tu
    asm_pc = H_TU;
    W(0x40F8); W((uint16_t)(RT + 0x10));             // MOVE SR,(RT+$10).W
    W(0x21CF); W((uint16_t)(RT + 0x12));             // MOVE.L A7,(RT+$12).W
    W(0x21C2); W((uint16_t)(RT + 0x16));             // MOVE.L D2,(RT+$16).W
    W(0x4E69);                                       // MOVE USP,A1
    W(0x21C9); W((uint16_t)(RT + 0x1A));             // MOVE.L A1,(RT+$1A).W
    W(0x4EF9); L(FIN);                               // JMP FIN

    // ---- allowed areas and FCs (UM, Function Code Outputs: 1 user data, 2 user program, 5 supervisor data,
    //      6 supervisor program; the exception vectors are in the supervisor data space)
    region(RB + 0xF0, RB + 0xF6, FC_UD, "results written in user mode after TRAP #15");
    region(end_addr, end_addr + 4, FC_UP, "final BRA.S *, executed in user mode");
    region(0x0, 0x400, FC_SD, "vector table");
    region(RB, RB + 0x200, FC_SD, "results");
    region(0x2000, 0x2400, FC_SD, "stacks");
    region(0x4000, 0x4800, FC_SP, "program");
    // return to user mode: the words after TRAP #15 are fetched in supervisor mode (queue) and then in user mode
    region(FIN, FIN + 0x100, FC_SP | FC_UP, "final block");
    region(H_BASE, 0x5700, FC_SP, "handlers");
    // user instruction reached with RTE: the manual does not specify the mode of the queue fetches made by RTE
    region(UT - 0x10, UT + 0x20, FC_SP | FC_UP, "traced user code");
    region(CATCH, CATCH + 0x10, FC_SP, "catch");

    // ---- expected values
    {   // traced TRAPV (UM 6.3.8)
        const uint32_t F1 = SSP_TV - 6, F2 = SSP_TV - 12;
        expect_mem(F1, 2, 0x2702, "traced TRAPV: TRAPV frame, SR (pushed with T = 1, T cleared in place by the TRAPV handler)");
        expect_mem(F1 + 2, 4, tv_at + 2, "traced TRAPV: TRAPV frame, PC = instruction after the TRAPV");
        expect_mem(F2, 2, 0x2702, "traced TRAPV: trace frame below the TRAPV frame, SR of the entry into the TRAPV handler (S = 1, T = 0, mask and CCR kept)");
        expect_mem(F2 + 2, 4, H_TV, "traced TRAPV: trace frame, PC = the TRAPV handler (the trace exception follows the instruction's own exception, UM 6.3.8)");
        expect_mem(RTV + 0, 2, 0x5A5A, "traced TRAPV: the trace handler ran");
        expect_mem(RTV + 2, 2, 0x2702, "traced TRAPV: SR in the trace handler (S = 1, T = 0, mask 7, V = 1)");
        expect_mem(RTV + 18, 4, SSP_TV - 12, "traced TRAPV: SSP in the trace handler = SSP - 12 (two frames)");
        expect_mem(RTV + 10, 2, 0x5A5A, "traced TRAPV: the TRAPV handler found the trace handler's marker: the trace handler ran first");
        expect_mem(RTV + 4, 2, 0x2702, "traced TRAPV: SR in the TRAPV handler, after the trace handler's RTE");
        expect_mem(RTV + 6, 4, SSP_TV - 6, "traced TRAPV: SSP in the TRAPV handler = SSP - 6 (the trace frame popped)");
        expect_mem(RTV + 12, 2, 0x2702, "traced TRAPV: SR after both returns (T cleared by the TRAPV handler)");
        expect_mem(RTV + 14, 4, SSP_TV, "traced TRAPV: SSP after both returns");
    }
    for (int n = 0; n < 16; n++) {
        uint32_t S = ssp_of(n), F = S - 6, R = RB + 0x10u * (uint32_t)n, pcr = ta[n] + 2;
        std::string tn = "TRAP #" + std::to_string(n) + ": ";
        uint16_t srin = sr_in(n);
        // UM ch. 6: the frame contains the SR from before the exception and the PC of the instruction following the TRAP
        expect_mem(F + 2, 2, pcr >> 16, tn + "frame, PC high = instruction after the TRAP");
        expect_mem(F + 4, 2, pcr & 0xFFFF, tn + "frame, PC low = instruction after the TRAP");
        if (n & 1) {
            expect_mem(F, 2, NEWSR[n], tn + "frame, SR rewritten by the handler");
            expect_mem(R + 12, 2, srin, tn + "stacked SR (copied by the handler) = SR before the TRAP");
            // PRM RTE: SR <- (SP); the unimplemented bits read as 0 (PRM 1.3) -> SR = newsr & $A71F
            expect_mem(R, 2, (uint32_t)(NEWSR[n] & 0xA71F), tn + "SR after RTE = SR of the rewritten frame");
        } else {
            expect_mem(F, 2, srin, tn + "frame, SR = SR before the TRAP");
            expect_mem(R, 2, srin, tn + "SR after RTE = SR of the frame (mask and CCR of the handler discarded)");
        }
        // UM ch. 6: internal copy of the SR, S <- 1, T <- 0; mask and CCR do not change
        expect_mem(R + 6, 2, (uint32_t)((srin | 0x2000) & 0x7FFF), tn + "SR in the handler (S=1, T=0, mask and CCR unchanged)");
        expect_mem(R + 8, 4, F, tn + "SSP in the handler = SSP - 6");
        // PRM RTE: SP <- SP + 6; TRAP #15 returns to user mode (A7 = USP)
        expect_mem(R + 2, 4, n == 15 ? USP_V : S, tn + (n == 15 ? "A7 after RTE in user mode = USP" : "SSP after RTE = SSP before the TRAP"));
    }
    // TRACE in supervisor mode. ADD.W D1,D0: $7FFF + $0001 = $8000 (PRM 4 ADD, table 3: N=1, Z=0, V=1 (pos+pos=neg),
    // C=0, X=C=0) -> CCR = $0A; stacked SR = T=1 S=1 I=7 + CCR = $A70A; stacked PC = instruction after the ADD.W
    expect_mem(SSP_TS - 6, 2, 0xA70A, "TRACE supervisor: frame, SR (T=1, CCR of the ADD.W)");
    expect_mem(SSP_TS - 4, 2, (ti_s + 2) >> 16, "TRACE supervisor: frame, PC high");
    expect_mem(SSP_TS - 2, 2, (ti_s + 2) & 0xFFFF, "TRACE supervisor: frame, PC low = instruction after the ADD.W");
    expect_mem(RT, 2, 0x270A, "TRACE supervisor: SR in the handler (S=1, T=0)");
    expect_mem(RT + 2, 4, SSP_TS - 6, "TRACE supervisor: SSP in the handler");
    expect_mem(RT + 6, 4, 0x12348000, "TRACE supervisor: D0 = $1234:$7FFF+$0001 (upper half unchanged)");
    // TRACE in user mode. SUB.L D3,D2: 0 - 1 = $FFFFFFFF (PRM 4 SUB, table 3: N=1, Z=0, V=0, C=1, X=1) -> CCR = $19;
    // stacked SR = T=1 S=0 I=2 + $19 = $8219; stacked PC = $0001FFFE + 2 = $00020000 (64 KB boundary)
    expect_mem(F_TU, 2, 0x8219, "TRACE user: frame, SR (T=1, S=0, I=2, CCR of the SUB.L)");
    expect_mem(F_TU + 2, 2, 0x0002, "TRACE user: frame, PC high (carry past $1FFFE)");
    expect_mem(F_TU + 4, 2, 0x0000, "TRACE user: frame, PC low");
    expect_mem(RT + 0x10, 2, 0x2219, "TRACE user: SR in the handler (S=1, T=0, I=2)");
    expect_mem(RT + 0x12, 4, F_TU, "TRACE user: SSP in the handler (= SSP after RTE - 6)");
    expect_mem(RT + 0x16, 4, 0xFFFFFFFF, "TRACE user: D2 = 0 - 1");
    expect_mem(RT + 0x1A, 4, USP_V, "TRACE user: USP unchanged");
}

static void bank_checks() {
    // final SR: the RTE of TRAP #15 loads $0013 (user, X V C); MOVE SR does not change the flags; MOVE.L A7 (= USP $00ABCDE0)
    // sets N=0 Z=0 V=0 C=0 and leaves X=1 (PRM 4 MOVE) -> $0010
    check_eq("final SR (sr_dbg port) after the return to user mode", 0x0010, dut->sr_dbg);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    run_once(0);
    run_once(3);
    printf("checks run: %d, failed: %d\n", n_checks, n_fails);
    puts(n_fails ? "RESULT: FAIL" : "RESULT: PASS");
    return n_fails ? 1 : 0;
}
