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

// tb_m68k_cpu_si.cpp - testbench of the complete processor (rtl/cpu_si/m68k_cpu_si.sv): 64 KB memory with programmable
// DTACK, reset vectors (SSP = 0x00001000, PC = 0x00000400) and a SELF-CHECKING program hand-assembled from the MC68000UM
// manual (instruction formats), with expected values computed by hand from the semantics of the manual:
//   MOVEQ/MOVE/MOVEA, ADD.L, SUB.W, ADDQ.L with carry into X and C, MOVE SR,Dn, long, word and byte memory writes and
//   reads (even and odd), LEA/SUBA/ADDA, BSR.W/RTS with the stack, MOVEM.L of all 16 registers to memory, STOP.
// Checks: reset sequence (vectors at 0/2/4/6 with supervisor program FC, then fetch at 0x400); expected memory image
// (results, stack, register dump); registers from the debug ports; final PSW and CPU stopped in the STOP wait loop
// (the two own words 3fc and 3fd, docs/exceptions.md STOP); no exception; on EVERY bus cycle: FC = 6 (supervisor program) on fetches and 5 (supervisor data) on data
// accesses, read with both strobes on words, UDS only on the even byte and LDS only on the odd byte (write and read),
// order of the two words of a long (high first), ascending order of the MOVEM, no write outside the expected
// addresses; E period = 20 phases.
// The Makefile runs it twice (PHASE_FIRST=0/1: first phase cen or cen2 after reset). Environment variables:
// CPU_TRACE=<phases>, CPU_DTACK=<clocks>, CPU_SINGLE=1 (cen = cen2 = 1), CPU_IPL_AT/CPU_IPL_LEVEL/CPU_IACK_VEC (interrupt:
// clock of the request, level, vector returned in the IACK cycle; without CPU_IACK_VEC the IACK is answered with VPA).
// Result: RESULT: PASS/FAIL, exit code 0/1.
#include "Vm68k_cpu_si.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <vector>
static Vm68k_cpu_si* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 25) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)
static uint16_t mem[32768];   // 64 KB as words
static int phase = 0; static long n_clk = 0; static int dtack_cnt = -1;
static long ipl_at = -1; static int single_cen = 0; static int dtack_lat = 2; static int ipl_level = 1; static int iack_vec = -1;
struct Cyc { uint32_t addr = 0; int rw = 0, fc = 0, uds = 0, lds = 0, upc = 0; uint16_t data = 0; long clk = 0; };
static std::vector<Cyc> cycles; static Cyc cur; static bool in_cyc = false;
static int trace_clk = 0;
static void prep() {
    dut->cen = single_cen ? 1 : (phase == 0); dut->cen2 = single_cen ? 1 : (phase == 1);
    dut->ipl_n = (ipl_at >= 0 && n_clk >= ipl_at) ? (7 - ipl_level) : 7;
    if (!dut->as_n) dtack_cnt = (dtack_cnt < 0) ? 0 : dtack_cnt + 1; else dtack_cnt = -1;
    dut->dtack_n = !(dtack_cnt >= dtack_lat);
    dut->data_in = mem[dut->addr & 32767];
    if (dut->fc == 7 && !dut->as_n) { if (iack_vec >= 0) { dut->vpa_n = 1; dut->data_in = (uint16_t)iack_vec; } else { dut->vpa_n = 0; dut->dtack_n = 1; } }
    else dut->vpa_n = 1;
    dut->eval();
}
// one clock edge: records the bus cycle (address, FC, R/W at the AS assertion; strobes and data accumulated while AS
// stays low), writes the memory on the strobes
static void edge() {
    if (n_clk < trace_clk) printf("clk %4ld cen%d cen2%d upc %03x as_n%d rw%d addr %06x uds%d lds%d dtack_n%d pc %08x din %04x dout %04x psw %04x\n", n_clk, dut->cen, dut->cen2, dut->upc, dut->as_n, dut->rw, (uint32_t)dut->addr << 1, !dut->uds_n, !dut->lds_n, dut->dtack_n, dut->pc_o, dut->data_in, dut->data_out, dut->psw_o);
    if (!dut->as_n && !dut->reset) {   // during reset the bus pins are not a cycle (AS low with the registers at zero)
        uint32_t wi = dut->addr & 32767;
        if (!in_cyc) { in_cyc = true; cur = Cyc{}; cur.addr = (uint32_t)dut->addr << 1; cur.rw = dut->rw; cur.fc = dut->fc; cur.clk = n_clk; cur.upc = dut->upc; }
        if (!dut->uds_n) cur.uds = 1; if (!dut->lds_n) cur.lds = 1;
        if (dut->rw) cur.data = mem[wi];
        else if (dut->data_oe && (!dut->uds_n || !dut->lds_n)) cur.data = dut->data_out;
        if (!dut->rw && dut->data_oe && !dut->uds_n) mem[wi] = (uint16_t)((mem[wi] & 0x00ff) | (dut->data_out & 0xff00));
        if (!dut->rw && dut->data_oe && !dut->lds_n) mem[wi] = (uint16_t)((mem[wi] & 0xff00) | (dut->data_out & 0x00ff));
    } else if (in_cyc) {
        cycles.push_back(cur); in_cyc = false;
        if (cycles.size() <= 12 || trace_clk) printf("cycle %3zu clk %5ld: %s addr %06x fc %d %s%s data %04x upc %03x\n", cycles.size(), cur.clk, cur.rw ? "R" : "W", cur.addr, cur.fc, cur.uds ? "U" : "-", cur.lds ? "L" : "-", cur.data, cur.upc);
    }
    dut->clk = 1; dut->eval(); dut->clk = 0; dut->eval(); phase ^= 1; n_clk++;
}
// program at 0x400 (MC68000UM manual, instruction formats)
static const uint16_t PROG[] = {
    0x7800, 0x7C00, 0x7E00, 0x7400,            // 400 MOVEQ #0,D4 / D6 / D7 / D2
    0x203C, 0x1234, 0x5678,                    // 408 MOVE.L #$12345678,D0
    0x223C, 0x0000, 0xFFFF,                    // 40E MOVE.L #$0000FFFF,D1
    0x343C, 0x8001,                            // 414 MOVE.W #$8001,D2
    0x76FF,                                    // 418 MOVEQ #-1,D3
    0x207C, 0x0000, 0x2000,                    // 41A MOVEA.L #$2000,A0
    0x227C, 0x0000, 0x2100,                    // 420 MOVEA.L #$2100,A1
    0xD081,                                    // 426 ADD.L D1,D0        D0 = 12355677, X=C=0
    0x9242,                                    // 428 SUB.W D2,D1        D1.w = FFFF - 8001 = 7FFE, X=C=0
    0x5E83,                                    // 42A ADDQ.L #7,D3       D3 = FFFFFFFF + 7 = 00000006, X=C=1, N=Z=V=0
    0x40C4,                                    // 42C MOVE.W SR,D4       D4 = 00002711 (S, I=7, X, C)
    0x2080,                                    // 42E MOVE.L D0,(A0)     2000: 1235, 2002: 5677
    0x3141, 0x0004,                            // 430 MOVE.W D1,4(A0)    2004: 7FFE
    0x1142, 0x0006,                            // 434 MOVE.B D2,6(A0)    even byte 2006 = 01 (UDS)
    0x1143, 0x0007,                            // 438 MOVE.B D3,7(A0)    odd byte 2007 = 06 (LDS)
    0x2A10,                                    // 43C MOVE.L (A0),D5     D5 = 12355677
    0x1C28, 0x0007,                            // 43E MOVE.B 7(A0),D6    D6 = 00000006 (LDS)
    0x3E28, 0x0004,                            // 442 MOVE.W 4(A0),D7    D7 = 00007FFE
    0x45E8, 0x0008,                            // 446 LEA 8(A0),A2       A2 = 2008
    0x95C8,                                    // 44A SUBA.L A0,A2       A2 = 8
    0xD2C1,                                    // 44C ADDA.W D1,A1       A1 = 2100 + 7FFE = A0FE
    0x99CC, 0x9BCD, 0x9DCE,                    // 44E SUBA.L A4,A4 / A5,A5 / A6,A6 = 0
    0x6100, 0x000C,                            // 454 BSR.W sub (456 + C = 462): pushes 00000458 at FFC
    0x48F8, 0xFFFF, 0x2100,                    // 458 MOVEM.L D0-D7/A0-A7,$2100.W
    0x4E72, 0x2700,                            // 45E STOP #$2700
    0x264F,                                    // 462 sub: MOVEA.L A7,A3  A3 = FFC
    0x4E75,                                    // 464 RTS
};
static const int PROG_END = 0x400 + 2 * (int)(sizeof(PROG) / sizeof(PROG[0]));   // 466
static const uint32_t EXP_D[8] = {0x12355677u, 0x00007FFEu, 0x00008001u, 0x00000006u, 0x00002711u, 0x12355677u, 0x00000006u, 0x00007FFEu};
static const uint32_t EXP_A[8] = {0x00002000u, 0x0000A0FEu, 0x00000008u, 0x00000FFCu, 0u, 0u, 0u, 0x00001000u};

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    if (getenv("CPU_TRACE")) trace_clk = atoi(getenv("CPU_TRACE"));
    if (getenv("CPU_IPL_AT")) ipl_at = atol(getenv("CPU_IPL_AT"));
    if (getenv("CPU_IPL_LEVEL")) ipl_level = atoi(getenv("CPU_IPL_LEVEL"));
    if (getenv("CPU_IACK_VEC")) iack_vec = atoi(getenv("CPU_IACK_VEC"));
    if (getenv("CPU_SINGLE")) single_cen = atoi(getenv("CPU_SINGLE"));
    if (getenv("CPU_DTACK")) dtack_lat = atoi(getenv("CPU_DTACK"));
    if (getenv("PHASE_FIRST")) phase = atoi(getenv("PHASE_FIRST")) & 1;
    printf("first phase after reset: %s\n", phase ? "cen2" : "cen");
    dut = new Vm68k_cpu_si;
    for (int i = 0; i < 32768; i++) mem[i] = 0x4e71;               // NOP everywhere
    mem[0] = 0x0000; mem[1] = 0x1000;                                // SSP = 0x00001000
    mem[2] = 0x0000; mem[3] = 0x0400;                                // PC  = 0x00000400
    for (size_t i = 0; i < sizeof(PROG) / sizeof(PROG[0]); i++) mem[(0x400 >> 1) + i] = PROG[i];
    dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0; dut->clk = 0; dut->reset = 1; dut->cen = 0; dut->cen2 = 0; dut->data_in = 0; dut->dtack_n = 1; dut->berr_n = 1; dut->vpa_n = 1; dut->ipl_n = 7; dut->halt_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    prep(); edge(); prep(); edge(); dut->reset = 0;
    // run: until the CPU stays in the STOP wait loop (3fc, 3fd) for 60 phases, with a limit
    long max_clk = 40000; int e_prev = 0, e_per = 0; long e_rise = -1; int stop_cnt = 0; long n_instr = 0; int done_prev = 0; int e_hi = 0, e_lo = 0;
    while (n_clk < max_clk) {
        prep();
        if (dut->cen2) {   // one sample per clock: E period 10 clocks, six low and four high (UM 3, Enable)
            if (dut->e && !e_prev) {
                if (e_rise >= 0) {
                    CHECK(n_clk - e_rise == 20, "E period %ld phases (expected 20)", n_clk - e_rise);
                    CHECK(e_hi == 4 && e_lo == 6, "E duty cycle: %d clocks high, %d low (expected 4 high, 6 low)", e_hi, e_lo);
                    e_per++;
                }
                e_rise = n_clk; e_hi = 0; e_lo = 0;
            }
            if (dut->e) e_hi++; else e_lo++;
            e_prev = dut->e;
        }
        if (dut->dbg_instr_done && !done_prev) n_instr++; done_prev = dut->dbg_instr_done;
        edge();
        if (dut->upc == 0x3fd || dut->upc == 0x3fc) { if (++stop_cnt > 60) break; } else stop_cnt = 0;
    }
    if (in_cyc) { cycles.push_back(cur); in_cyc = false; }
    CHECK(dut->upc == 0x3fd || dut->upc == 0x3fc, "the CPU is not stopped in the STOP wait loop (3fc, 3fd): upc %03x after %ld phases", dut->upc, n_clk);
    CHECK(dut->psw_o == 0x2700, "final PSW %04x expected 2700 (STOP #$2700)", dut->psw_o);
    CHECK(dut->dbg_illegal == 0, "illegal instruction exception taken");
    CHECK(e_per > 20 && dut->iack == 0 && dut->halted_n == 1 && dut->bg_n == 1 && dut->bus_tri == 0 && dut->dbg_pc == (dut->pc_o & 0xffffff), "E (%d periods) / iack %d / halted_n %d / bg_n %d / bus_tri %d / dbg_pc", e_per, dut->iack, dut->halted_n, dut->bg_n, dut->bus_tri);
    // reset: vector reads at 0,2,4,6 then fetch at 0x400 (manual: SSP and PC from vector 0, then the first instruction)
    std::vector<Cyc> rd; for (auto& c : cycles) if (c.rw) rd.push_back(c);
    const uint32_t exp_rst[5] = {0x000000, 0x000002, 0x000004, 0x000006, 0x000400};
    for (int i = 0; i < 5; i++) { CHECK((int)rd.size() > i && rd[i].addr == exp_rst[i], "reset: read %d at %06x expected %06x", i, (int)rd.size() > i ? rd[i].addr : 0u, exp_rst[i]); }
    for (int i = 0; i < 4 && (int)rd.size() > i; i++) CHECK(rd[i].fc == 6 && rd[i].uds && rd[i].lds, "reset: vector at %06x with FC %d strobes %d%d (expected FC 6, word)", rd[i].addr, rd[i].fc, rd[i].uds, rd[i].lds);
    // registers from the debug ports
    for (int i = 0; i < 8; i++) { dut->dbg_rd_dnum = i; dut->dbg_rd_anum = i; dut->eval(); CHECK(dut->dbg_rd_ddata == EXP_D[i], "D%d = %08x expected %08x", i, dut->dbg_rd_ddata, EXP_D[i]); CHECK(dut->dbg_rd_adata == EXP_A[i], "A%d = %08x expected %08x", i, dut->dbg_rd_adata, EXP_A[i]); }
    // memory: results, stack (return address left after RTS), MOVEM dump
    std::map<uint32_t, uint16_t> expm = {{0x2000, 0x1235}, {0x2002, 0x5677}, {0x2004, 0x7FFE}, {0x2006, 0x0106}, {0x0FFC, 0x0000}, {0x0FFE, 0x0458}};
    for (int i = 0; i < 8; i++) { expm[0x2100 + 4 * i] = (uint16_t)(EXP_D[i] >> 16); expm[0x2102 + 4 * i] = (uint16_t)EXP_D[i]; expm[0x2120 + 4 * i] = (uint16_t)(EXP_A[i] >> 16); expm[0x2122 + 4 * i] = (uint16_t)EXP_A[i]; }
    for (auto& kv : expm) CHECK(mem[kv.first >> 1] == kv.second, "memory %06x = %04x expected %04x", kv.first, mem[kv.first >> 1], kv.second);
    // bus cycles: FC, strobes, order
    int n_fetch = 0, n_data = 0, n_vec = 0; std::vector<Cyc> wr, drd;
    for (auto& c : cycles) {
        bool prog = c.addr >= 0x400 && c.addr < (uint32_t)PROG_END + 4, data = (c.addr >= 0x2000 && c.addr < 0x2140) || (c.addr >= 0xFF8 && c.addr < 0x1000), vec = c.addr < 8;   // +4: the prefetch beyond the last instruction (manual: two words ahead)
        if (prog) { n_fetch++; CHECK(c.rw && c.fc == 6 && c.uds && c.lds, "fetch at %06x: rw %d FC %d strobes %d%d (expected read, FC 6, word)", c.addr, c.rw, c.fc, c.uds, c.lds); }
        else if (data) { n_data++; CHECK(c.fc == 5, "data access at %06x: FC %d expected 5 (supervisor data)", c.addr, c.fc); if (c.rw) drd.push_back(c); }
        else if (vec) n_vec++;
        else CHECK(false, "unexpected cycle at %06x (%s, FC %d, upc %03x)", c.addr, c.rw ? "read" : "write", c.fc, c.upc);
        if (!c.rw) wr.push_back(c);
        if (c.addr == 0x2006) { if (c.rw) CHECK(!c.uds && c.lds && (c.data & 0xff) == 0x06, "read of the odd byte 2007: strobes %d%d data %04x (expected LDS only, byte 06)", c.uds, c.lds, c.data); }
        else if (data || prog || vec) CHECK(c.uds && c.lds, "word access at %06x with strobes %d%d", c.addr, c.uds, c.lds);
    }
    // writes: 2000, 2002 (long, high first), 2004, 2006 (UDS, byte 01), 2006 (LDS, byte 06), then the stack (FFC, FFE),
    // then 32 MOVEM words in ascending order
    CHECK(wr.size() == 39, "writes %d expected 39", (int)wr.size());
    if (wr.size() >= 5) {
        CHECK(wr[0].addr == 0x2000 && wr[1].addr == 0x2002 && wr[2].addr == 0x2004, "order of the writes: %06x %06x %06x (expected 2000 2002 2004)", wr[0].addr, wr[1].addr, wr[2].addr);
        CHECK(wr[3].addr == 0x2006 && wr[3].uds && !wr[3].lds && (wr[3].data >> 8) == 0x01, "write of the even byte 2006: addr %06x strobes %d%d data %04x (expected UDS only, byte 01 on D15-8)", wr[3].addr, wr[3].uds, wr[3].lds, wr[3].data);
        CHECK(wr[4].addr == 0x2006 && !wr[4].uds && wr[4].lds && (wr[4].data & 0xff) == 0x06, "write of the odd byte 2007: addr %06x strobes %d%d data %04x (expected LDS only, byte 06 on D7-0)", wr[4].addr, wr[4].uds, wr[4].lds, wr[4].data);
    }
    if (wr.size() >= 7) { std::set<uint32_t> st = {wr[5].addr, wr[6].addr}; CHECK(st.count(0xFFC) && st.count(0xFFE), "BSR stack: writes at %06x and %06x (expected FFC and FFE)", wr[5].addr, wr[6].addr); }
    for (size_t i = 7; i < wr.size() && i < 39; i++) CHECK(wr[i].addr == 0x2100 + 2 * (uint32_t)(i - 7), "MOVEM: write %d at %06x expected %06x", (int)(i - 7), wr[i].addr, 0x2100 + 2 * (uint32_t)(i - 7));
    // data reads: 2000, 2002 (long, high first), 2006 (byte), 2004 (word), then the RTS stack (FFC, FFE)
    CHECK(drd.size() == 6, "data reads %d expected 6", (int)drd.size());
    if (drd.size() >= 4) CHECK(drd[0].addr == 0x2000 && drd[1].addr == 0x2002 && drd[2].addr == 0x2006 && drd[3].addr == 0x2004, "order of the data reads: %06x %06x %06x %06x (expected 2000 2002 2006 2004)", drd[0].addr, drd[1].addr, drd[2].addr, drd[3].addr);
    if (drd.size() >= 6) { std::set<uint32_t> st = {drd[4].addr, drd[5].addr}; CHECK(st.count(0xFFC) && st.count(0xFFE), "RTS stack: reads at %06x and %06x (expected FFC and FFE)", drd[4].addr, drd[5].addr); }
    printf("CPU: phases %ld | instructions %ld | bus cycles %d (fetch %d, data %d, vectors %d; writes %d) | final upc %03x pc %08x psw %04x | E periods %d | check %d | FAIL %d\n",
           n_clk, n_instr, (int)cycles.size(), n_fetch, n_data, n_vec, (int)wr.size(), dut->upc, dut->pc_o, dut->psw_o, e_per, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
