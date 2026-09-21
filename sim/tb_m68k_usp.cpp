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

// tb_m68k_usp.cpp - MOVE An,USP and MOVE USP,An (PRM 4 MOVE USP, privileged), user A7 = USP, privilege violation.
// Tests (expectations by hand, PRM 4 MOVE USP / RTE / MOVE to SR, PRM 2 registers, UM exceptions):
//  - supervisor: two USP round trips with different values (A1 -> USP -> A2, A3 -> USP -> A4); MOVE A7,USP copies the SSP
//    into the USP; MOVE USP,A7 writes the SSP; the SSP does not change for MOVE An,USP; A6 (sentinel) intact; CCR unchanged;
//  - switch to user mode with RTE (stack prepared by hand: SR 0x0015, PC 0x5000) and with MOVE #0x0004,SR: in user mode A7 holds the
//    USP written in supervisor mode, a write of A7 in user mode changes the USP (then seen by the handler with MOVE USP,A5), the
//    user stack (MOVE.L -(A7) and (A7)+) uses the USP with FC 1;
//  - in user mode MOVE USP,A0 and MOVE A1,USP are privilege violations (vector 8): A0 and the USP do not change; 3-word
//    stack frame on the SSP with user SR and PC = address of the instruction (UM: privilege violation), stack cells
//    preloaded with sentinels; in the handler S = 1, T = 0, mask and CCR as before, SSP = SSP - 6; two distinct handlers (vector
//    8 is rewritten between the two violations);
//  - every bus cycle: FC 6 / 2 on fetches in supervisor / user mode, FC 5 / 1 on data; whole memory compared.
//  Two runs: DTACK immediate and after 3 clocks.
// Does not test: the space of the fetches already queued at the mode change made by MOVE to SR (not fixed by the manual: FC not
// checked on those words), the FC of the vector reads, the order of the stack writes, TRACE.
// Exit: "RESULT: PASS" / "RESULT: FAIL", code 0 / 1.

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <string>

static const char* BENCH = "tb_m68k_usp";

// ---- infrastructure shared by the testbenches of the group: byte memory with UDS/LDS, cycle log, two runs, checks ----
static Vm68k_seq* dut = nullptr;
static uint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static int n_checks = 0, n_fails = 0;
static std::string tag;
static std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "0x%X", (unsigned)v); return std::string(b); }
static void chk(bool ok, const std::string& what, uint32_t exp, uint32_t got) {
    n_checks++;
    if (!ok) { n_fails++; if (n_fails <= 150) printf("FAIL: %s%s expected=%s read=%s\n", tag.c_str(), what.c_str(), hx(exp).c_str(), hx(got).c_str()); }
}

static constexpr uint32_t MEMSZ = 0x10000;       // modelled memory: 64 KB
static constexpr uint32_t ORG = 0x4000;          // start of the program (pc_init of the shim)
static constexpr uint32_t TRAPV_ADDR = 0xF000;   // unused vectors: outside every code area (an unexpected exception fails)
static uint8_t mem0[MEMSZ], mem[MEMSZ], isexp[MEMSZ];

// expectations: final bytes (with mask) and data accesses per word and lanes (key = lanes << 24 | address; lanes 3 = word,
// 2 = UDS only (even byte), 1 = LDS only (odd byte))
struct EB { uint32_t a; uint8_t v, m; std::string d; };
static std::vector<EB> expb;
struct DW { int rd = 0, wr = 0, fc = 5, grp = -1; bool any_rd = false; };
static std::map<uint32_t, DW> dexp;
struct Region { uint32_t lo, hi; int fc; };
static std::vector<Region> code;
static std::map<int, std::vector<uint32_t>> seq_exp;   // expected order of the data accesses of a group
static uint32_t key(uint32_t wa, int lanes) { return ((uint32_t)lanes << 24) | (wa & 0xFFFFFEu); }
[[maybe_unused]] static int lane_of(uint32_t a) { return (a & 1) ? 1 : 2; }
static void expect8(uint32_t a, uint8_t v, uint8_t m, const std::string& d) { expb.push_back(EB{a, v, m, d}); isexp[a] = 1; }
static DW& dacc(uint32_t a, int lanes) { return dexp[key(a, lanes)]; }

// minimal assembler (encodings from PRM sec. 4 and 8)
static uint32_t apc = ORG;
static int dfc = 5;                // expected data FC of the code being assembled (UM: 5 supervisor data, 1 user data)
static uint32_t rsv = 0x1000;      // next cell of the results area
static uint32_t end_pc = 0;
static uint32_t dsv = 0x3000;      // next 16-byte block of the data area
[[maybe_unused]] static uint32_t new_slot() { uint32_t s = dsv; dsv += 16; return s; }
static void w(uint16_t v) { mem0[apc] = (uint8_t)(v >> 8); mem0[apc + 1] = (uint8_t)v; apc += 2; }
static void put16(uint32_t a, uint16_t v) { mem0[a] = (uint8_t)(v >> 8); mem0[a + 1] = (uint8_t)v; }
static void put32(uint32_t a, uint32_t v) { put16(a, (uint16_t)(v >> 16)); put16(a + 2, (uint16_t)v); }
static void movea_l(int n, uint32_t v) { w((uint16_t)(0x207C | (n << 9))); w((uint16_t)(v >> 16)); w((uint16_t)v); }   // MOVEA.L #imm,An
[[maybe_unused]] static void move_l_d(int n, uint32_t v) { w((uint16_t)(0x203C | (n << 9))); w((uint16_t)(v >> 16)); w((uint16_t)v); }  // MOVE.L #imm,Dn
static void move_ccr(uint8_t c) { w(0x44FC); w(c); }                                                                      // MOVE #imm,CCR
// result word written once (reads allowed: MOVE from SR reads the destination before writing it)
static void res_word(uint32_t a, uint16_t v, uint16_t m, const std::string& d) {
    DW& x = dacc(a, 3); x.wr += 1; x.any_rd = true; x.fc = dfc;
    expect8(a, (uint8_t)(v >> 8), (uint8_t)(m >> 8), d); expect8(a + 1, (uint8_t)v, (uint8_t)m, d);
}
static void save_sr(uint16_t v, const std::string& d, uint16_t m = 0xFFFF) { w(0x40F8); w((uint16_t)rsv); res_word(rsv, v, m, d + " [SR]"); rsv += 2; }   // MOVE SR,abs.W
[[maybe_unused]] static void save_d(int n, uint32_t v, const std::string& d) {   // MOVE.L Dn,abs.W
    w((uint16_t)(0x21C0 | n)); w((uint16_t)rsv);
    res_word(rsv, (uint16_t)(v >> 16), 0xFFFF, d + " [D" + std::to_string(n) + "]"); res_word(rsv + 2, (uint16_t)v, 0xFFFF, d + " [D" + std::to_string(n) + "]"); rsv += 4;
}
[[maybe_unused]] static void save_a(int n, uint32_t v, const std::string& d) {   // MOVE.L An,abs.W
    w((uint16_t)(0x21C8 | n)); w((uint16_t)rsv);
    res_word(rsv, (uint16_t)(v >> 16), 0xFFFF, d + " [A" + std::to_string(n) + "]"); res_word(rsv + 2, (uint16_t)v, 0xFFFF, d + " [A" + std::to_string(n) + "]"); rsv += 4;
}
static void end_loop() { end_pc = apc; w(0x60FE); }   // BRA.S * : end of run
static void fill_common() {
    for (uint32_t a = 0; a < MEMSZ; a += 2) put16(a, 0x4E71);                                           // NOP everywhere
    for (uint32_t a = 0; a < 0x400; a += 4) put32(a, TRAPV_ADDR);                                       // vectors
    for (uint32_t a = 0x0800; a < ORG; a++) mem0[a] = (uint8_t)((a * 0x3Bu) ^ (a >> 7) ^ 0xA5u);        // sentinels
}

struct Cyc { uint32_t a; int rw, fc, uds, lds; uint16_t d; };
static std::vector<Cyc> cyc;
static bool reached_end = false, oob = false;
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }
static uint16_t rdw(uint32_t a) { return (uint16_t)((mem[a] << 8) | mem[a + 1]); }
// one run: new model, reset, execution up to the second fetch of the final BRA.S (the BRA has been executed, so
// all the previous instructions have finished), then 40 clocks. Writes only the enabled lanes; logs every cycle.
static void run(int waits) {
    memcpy(mem, mem0, MEMSZ); cyc.clear(); reached_end = false; oob = false;
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    dut->clk = 0; dut->rst_n = 0; dut->cen = 1; dut->cen2 = 1; dut->start = 0; dut->pc_init = ORG; dut->rst_vector = 0;
    dut->ipl = 0; dut->vpa_n = 1; dut->data_in = 0; dut->dtack_n = 1; dut->halt_n = 1; dut->berr_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    dut->eval();
    for (int i = 0; i < 10; i++) tick();
    dut->rst_n = 1;
    bool armed = false, in = false; Cyc cur{0, 1, 0, 0, 0, 0}; int cnt = 0, hits = 0, tail = -1;
    for (long i = 0; i < 2000000 && tail != 0; i++) {
        if (dut->as_n) armed = true;
        if (armed && !dut->as_n) {
            uint32_t a = (uint32_t)dut->addr << 1;
            if (!in) { in = true; cnt = 0; cur = Cyc{a, 1, (int)dut->fc, 0, 0, 0}; } else cnt++;
            bool ok = a < MEMSZ; if (!ok) oob = true;
            if (!dut->rw) cur.rw = 0;
            if (!dut->uds_n) cur.uds = 1;
            if (!dut->lds_n) cur.lds = 1;
            if (dut->rw) { dut->data_in = ok ? rdw(a) : 0x4E71; cur.d = dut->data_in; }
            else if (dut->data_oe && (!dut->uds_n || !dut->lds_n)) {
                cur.d = dut->data_out;
                if (ok) { if (!dut->uds_n) mem[a] = (uint8_t)(dut->data_out >> 8); if (!dut->lds_n) mem[a + 1] = (uint8_t)dut->data_out; }
            }
            dut->dtack_n = (cnt >= waits) ? 0 : 1;
        } else {
            if (in) { in = false; cyc.push_back(cur); if (cur.rw && cur.a == end_pc && ++hits == 2) tail = 40; }
            dut->dtack_n = 1;
        }
        tick();
        if (tail > 0) tail--;
    }
    reached_end = hits >= 2;
    dut->final(); delete dut; dut = nullptr;
}
static void check_bus() {
    static const uint32_t rv[5] = {0, 2, 4, 6, ORG};   // UM reset: SSP from the long at 0, PC from the long at 4, then the first fetch
    for (int k = 0; k < 5; k++) {
        uint32_t got = cyc.size() > (size_t)k ? cyc[k].a : 0xFFFFFFFFu;
        chk(cyc.size() > (size_t)k && cyc[k].rw == 1 && got == rv[k], "reset sequence: read " + std::to_string(k), rv[k], got);
    }
    std::map<uint32_t, int> nrd, nwr; std::map<uint32_t, long> last_rd, first_wr; std::map<int, std::vector<uint32_t>> seen;
    for (size_t i = 0; i < cyc.size(); i++) {
        const Cyc& c = cyc[i];
        std::string at = " (cycle " + std::to_string(i) + (c.rw ? " read" : " write") + " at " + hx(c.a) + " lanes " + std::to_string(c.uds * 2 + c.lds) + ")";
        uint32_t k = key(c.a, c.uds * 2 + c.lds);
        auto it = dexp.find(k);
        if (it != dexp.end()) {
            const DW& x = it->second;
            if (x.fc >= 0) chk(c.fc == x.fc, "FC of the data access" + at, (uint32_t)x.fc, (uint32_t)c.fc);
            if (c.rw) { nrd[k]++; if (x.grp >= 0) last_rd[k] = (long)i; }
            else { nwr[k]++; if (x.grp >= 0 && !first_wr.count(k)) first_wr[k] = (long)i; }
            if (x.grp >= 0) seen[x.grp].push_back(k);
            continue;
        }
        int cfc = -2;
        for (const Region& r : code) if (c.a >= r.lo && c.a < r.hi) { cfc = r.fc; break; }
        if (cfc != -2) {
            chk(c.rw == 1 && c.uds == 1 && c.lds == 1, "fetch in the code: word read expected" + at, 1, (uint32_t)c.rw);
            if (cfc >= 0) chk(c.fc == cfc, "FC of the fetch" + at, (uint32_t)cfc, (uint32_t)c.fc);
        } else if (c.a < 0x400) {
            chk(c.rw == 1 && c.uds == 1 && c.lds == 1, "vector table: word read expected" + at, 1, (uint32_t)c.rw);
        } else chk(false, "cycle outside the expected areas" + at, 0, c.a);
    }
    for (auto& kv : dexp) {
        std::string at = " at " + hx(kv.first & 0xFFFFFFu) + " lanes " + std::to_string(kv.first >> 24);
        if (!kv.second.any_rd) chk(nrd[kv.first] == kv.second.rd, "number of reads" + at, (uint32_t)kv.second.rd, (uint32_t)nrd[kv.first]);
        chk(nwr[kv.first] == kv.second.wr, "number of writes" + at, (uint32_t)kv.second.wr, (uint32_t)nwr[kv.first]);
    }
    for (auto& kv : first_wr)
        if (last_rd.count(kv.first)) chk(last_rd[kv.first] < kv.second, "read after the write (cycle read = last read) at " + hx(kv.first & 0xFFFFFFu) + " lanes " + std::to_string(kv.first >> 24), (uint32_t)kv.second, (uint32_t)last_rd[kv.first]);
    for (auto& kv : seq_exp) {
        const std::vector<uint32_t>& s = seen[kv.first];
        chk(s == kv.second, "order or number of the data accesses of case " + std::to_string(kv.first), (uint32_t)kv.second.size(), (uint32_t)s.size());
    }
}
static void check_mem() {
    for (const EB& e : expb) chk((mem[e.a] & e.m) == (e.v & e.m), "memory " + hx(e.a) + " (" + e.d + ")", e.v & e.m, mem[e.a] & e.m);
    int bad = 0;
    for (uint32_t a = 0; a < MEMSZ; a++)
        if (!isexp[a] && mem[a] != mem0[a]) { bad++; if (bad <= 16) chk(false, "byte " + hx(a) + " modified outside the targets", mem0[a], mem[a]); }
    chk(bad == 0, "bytes modified outside the targets", 0, (uint32_t)bad);
}
// ---- end of the shared infrastructure ----

// expected data word (stack, rewritten vector): counted writes, free or counted reads, final value
static void data_word(uint32_t a, uint16_t v, int wr, int rd, bool any_rd, int fc, const std::string& d) {
    DW& x = dacc(a, 3); x.wr = wr; x.rd = rd; x.any_rd = any_rd; x.fc = fc;
    expect8(a, (uint8_t)(v >> 8), 0xFF, d); expect8(a + 1, (uint8_t)v, 0xFF, d);
}

// Encodings (PRM 4): MOVE An,USP = 0100 1110 0110 0 An (0x4E60 | n), MOVE USP,An = 0x4E68 | n, RTE = 0x4E73,
// MOVE #imm,SR = 0x46FC, MOVE.L #imm,-(A7) = 0x2F3C, MOVE.W #imm,-(A7) = 0x3F3C, MOVE.L (A7)+,D5 = 0x2A1F,
// MOVE.L #imm,abs.W = 0x21FC, JMP abs.L = 0x4EF9.
static constexpr uint32_t USER1 = 0x5000, USER2 = 0x5800, HND1 = 0x6000, HND2 = 0x6800;
static constexpr uint32_t SSP0 = 0x0F00;

static void build() {
    fill_common();
    put32(0x20, HND1);   // vector 8 (privilege violation, UM) -> first handler; the program rewrites it to HND2
    data_word(0x20, (uint16_t)(HND2 >> 16), 1, 0, true, -1, "vector 8 rewritten (high word)");   // FC of the vector reads not fixed
    data_word(0x22, (uint16_t)HND2, 1, 0, true, -1, "vector 8 rewritten (low word)");
    uint32_t rs_usr = 0x2000;   // results written in user mode (FC 1), separate from the supervisor ones (0x1000)

    // ---- supervisor, 0x4000 ----
    apc = ORG; dfc = 5;
    movea_l(7, SSP0);          // explicit SSP (the shim starts with SSP = 0)
    movea_l(6, 0x66666666u);   // sentinel: a wrong register index for the USP would touch it
    movea_l(0, 0x0A0A0A0Au);   // destination of the forbidden MOVE USP,A0
    movea_l(1, 0x00ABCDE0u);
    w(0x4E61);                 // MOVE A1,USP
    w(0x4E6A);                 // MOVE USP,A2
    save_a(2, 0x00ABCDE0u, "MOVE A1,USP; MOVE USP,A2");
    save_a(7, SSP0, "SSP after MOVE A1,USP");
    movea_l(3, 0x11223344u);
    w(0x4E63);                 // MOVE A3,USP
    w(0x4E6C);                 // MOVE USP,A4
    save_a(4, 0x11223344u, "MOVE A3,USP; MOVE USP,A4");
    w(0x4E67);                 // MOVE A7,USP: in supervisor mode A7 = SSP -> USP = 0xF00
    w(0x4E6D);                 // MOVE USP,A5
    save_a(5, SSP0, "MOVE A7,USP; MOVE USP,A5");
    save_a(7, SSP0, "SSP after MOVE A7,USP");
    movea_l(1, 0x00445566u);
    w(0x4E61);                 // MOVE A1,USP
    w(0x4E6F);                 // MOVE USP,A7: writes the SSP
    save_a(7, 0x00445566u, "MOVE USP,A7 in supervisor mode writes the SSP");
    movea_l(7, SSP0);
    move_ccr(0x1B);
    w(0x4E6A);                 // MOVE USP,A2
    save_sr(0x271B, "MOVE USP,A2: CCR unchanged");   // PRM 4 MOVE USP: condition codes not affected
    save_a(2, 0x00445566u, "MOVE USP,A2 (second read)");
    movea_l(1, 0x00002800u);
    move_ccr(0x04);
    w(0x4E61);                 // MOVE A1,USP: USP = 0x2800 for the user program
    save_sr(0x2704, "MOVE A1,USP: CCR unchanged");
    save_a(6, 0x66666666u, "A6 intact");
    // stack for RTE (PRM 4 RTE: SR from the stack, then PC): PC = USER1, SR = 0x0015 (user, mask 0, CCR X Z C)
    w(0x2F3C); w((uint16_t)(USER1 >> 16)); w((uint16_t)USER1);   // MOVE.L #USER1,-(A7): 0xEFC/0xEFE, A7 = 0xEFC
    w(0x3F3C); w(0x0015);                                        // MOVE.W #0x0015,-(A7): 0xEFA, A7 = 0xEFA
    w(0x4E73);                                                   // RTE: SR = 0x0015, PC = USER1, SSP = 0xF00
    code.push_back(Region{ORG, apc + 8, 6});                     // UM: FC 6 = supervisor program
    uint32_t rs_sup = rsv;

    // ---- user mode after RTE, 0x5000: A7 = USP ----
    apc = USER1; dfc = 1; rsv = rs_usr;
    save_sr(0x0015, "user mode after RTE");                      // before the MOVE.Ls, which change N Z V C
    save_a(7, 0x00002800u, "user mode after RTE: A7 = USP");
    movea_l(7, 0x00002900u);                                     // write of A7 in user mode = USP
    w(0x2F3C); w(0xCAFE); w(0xBABE);                             // MOVE.L #0xCAFEBABE,-(A7): user stack 0x28FC
    data_word(0x28FC, 0xCAFE, 1, 1, false, 1, "user stack (high word)");   // written, then read by (A7)+, FC 1
    data_word(0x28FE, 0xBABE, 1, 1, false, 1, "user stack (low word)");
    save_a(7, 0x000028FCu, "user mode: A7 after -(A7)");
    w(0x2A1F);                                                   // MOVE.L (A7)+,D5
    save_d(5, 0xCAFEBABEu, "user mode: (A7)+");
    save_a(7, 0x00002900u, "user mode: A7 after (A7)+");
    move_ccr(0x0B);
    uint32_t priv1 = apc;
    w(0x4E68);                                                   // MOVE USP,A0 in user mode: privilege violation
    code.push_back(Region{USER1, apc + 8, 2});                   // UM: FC 2 = user program
    rs_usr = rsv;

    // ---- first handler of vector 8, 0x6000 ----
    apc = HND1; dfc = 5; rsv = rs_sup;
    save_sr(0x200B, "handler 1: S=1 T=0, mask and CCR of the user");   // UM: the exception sets S, clears T
    save_a(7, SSP0 - 6, "handler 1: SSP after the 3-word stack frame");
    w(0x4E6D);                                                   // MOVE USP,A5
    save_a(5, 0x00002900u, "handler 1: USP = A7 written in user mode");
    save_a(0, 0x0A0A0A0Au, "A0 intact (forbidden MOVE USP,A0)");
    w(0x21FC); w((uint16_t)(HND2 >> 16)); w((uint16_t)HND2); w(0x0020);   // MOVE.L #HND2,0x0020.W: vector 8 -> second handler
    uint32_t msr = apc;
    w(0x46FC); w(0x0004);                                        // MOVE #0x0004,SR: user mode, CCR Z
    w(0x4EF9); w((uint16_t)(USER2 >> 16)); w((uint16_t)USER2);   // JMP USER2 (in user mode)
    code.push_back(Region{HND1, msr + 4, 6});
    code.push_back(Region{msr + 4, apc + 8, -1});                // words already queued at the mode change: FC not fixed
    rs_sup = rsv;

    // ---- user mode after MOVE to SR, 0x5800 ----
    apc = USER2; dfc = 1; rsv = rs_usr;
    save_sr(0x0004, "user mode after MOVE to SR");               // before the MOVE.Ls, which change N Z V C
    save_a(7, 0x00002900u, "user mode after MOVE to SR: A7 = USP");
    movea_l(7, 0x00002A00u);                                     // new USP
    movea_l(1, 0x13579BDFu);
    move_ccr(0x11);
    uint32_t priv2 = apc;
    w(0x4E61);                                                   // MOVE A1,USP in user mode: privilege violation
    code.push_back(Region{USER2, apc + 8, 2});

    // ---- second handler of vector 8, 0x6800 ----
    apc = HND2; dfc = 5; rsv = rs_sup;
    save_sr(0x2011, "handler 2: S=1 T=0, mask and CCR of the user");
    save_a(7, SSP0 - 12, "handler 2: SSP after the second stack frame");
    w(0x4E6D);                                                   // MOVE USP,A5
    save_a(5, 0x00002A00u, "handler 2: USP not written by the forbidden MOVE A1,USP");
    save_a(1, 0x13579BDFu, "A1 intact");
    save_a(6, 0x66666666u, "A6 intact at the end");
    end_loop();
    code.push_back(Region{HND2, apc + 8, 6});

    // exception stack frames (UM, group 1/2: SR at SSP-6, high PC at SSP-4, low PC at SSP-2), FC 5
    // first frame 0xEFA..0xEFF: written first by the two MOVE -(A7) for the RTE (read by the RTE), then by the violation
    data_word(0x0EFA, 0x000B, 2, 0, true, 5, "violation frame 1: SR");
    data_word(0x0EFC, (uint16_t)(priv1 >> 16), 2, 0, true, 5, "violation frame 1: high PC");
    data_word(0x0EFE, (uint16_t)priv1, 2, 0, true, 5, "violation frame 1: low PC = MOVE USP,A0");
    data_word(0x0EF4, 0x0011, 1, 0, false, 5, "violation frame 2: SR");
    data_word(0x0EF6, (uint16_t)(priv2 >> 16), 1, 0, false, 5, "violation frame 2: high PC");
    data_word(0x0EF8, (uint16_t)priv2, 1, 0, false, 5, "violation frame 2: low PC = MOVE A1,USP");
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    for (int pass = 0; pass < 2; pass++) {
        int waits = pass ? 3 : 0;
        tag = pass ? "[DTACK after 3 clocks] " : "[DTACK immediate] ";
        run(waits);
        chk(reached_end, "end of run: second fetch of the final BRA.S at " + hx(end_pc), 1, reached_end ? 1u : 0u);
        chk(!oob, "access beyond the modelled memory (64 KB)", 0, oob ? 1u : 0u);
        check_bus();
        check_mem();
    }
    tag = "";
    printf("%s: %d checks, %d failed\n", BENCH, n_checks, n_fails);
    puts(n_fails ? "RESULT: FAIL" : "RESULT: PASS");
    return n_fails ? 1 : 0;
}
