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

// tb_m68k_addxmem.cpp - ADDX and SUBX -(Ay),-(Ax) on memory in .B (even and odd addresses), .W and .L; 64-bit chains with
// two instructions in a row; byte -(A7) (step 2) as source, as destination and as both.
// Tests (expectations worked out by hand, PRM 4 ADDX / SUBX, PRM 3 condition code table):
//  - result in memory in the destination; source intact; neighbouring sentinel bytes intact (whole memory);
//  - flags after each instruction (also between the two of the chain): X = C = carry/borrow, N = Rm, V, Z cleared if the
//    result is not zero and unchanged otherwise (input Z 1 and 0, chains with the first part zero and non-zero);
//  - registers Ax and Ay afterwards: decremented by the size (1, 2, 4; 2 for byte A7, PRM 2.2.5), once per instruction;
//  - bus: every source word (or byte, on its lane) read once, every destination word read and then written once
//    (read before the write, word by word); FC 5 on data, 6 on fetches.
//  Two runs: DTACK immediate and after 3 clocks.
// Does not test: the register form Dy,Dx (tb_m68k_addx), the order between the two words of a long (not fixed by the
// manual), execution times.
// Exit: "RESULT: PASS" / "RESULT: FAIL", code 0 / 1.

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <string>

static const char* BENCH = "tb_m68k_addxmem";

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
static constexpr uint32_t ORG = 0x4000;          // program start (pc_init of the shim)
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
static uint32_t rsv = 0x1000;      // next cell of the result area
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
// all the preceding instructions are finished), then 40 clocks. Writes only the enabled lanes; logs every cycle.
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
        if (last_rd.count(kv.first)) chk(last_rd[kv.first] < kv.second, "read after the write (read cycle = last read) at " + hx(kv.first & 0xFFFFFFu) + " lanes " + std::to_string(kv.first >> 24), (uint32_t)kv.second, (uint32_t)last_rd[kv.first]);
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

// -(Ay),-(Ax) instruction: encoding = base | Rx << 9 | size << 6 | Ry (PRM 4: ADDX 1101 Rx 1 ss 00 1 Ry = 0xD108,
// SUBX 1001 Rx 1 ss 00 1 Ry = 0x9108; ss 00 .B, 01 .W, 10 .L). Ay = A2 or A7, Ax = A1 or A7. ay_off / ax_off = initial
// value of the register within its own 16-byte block (if Rx = Ry = 7 a single register, initial value = block + ax_off).
// n = 1, or 2 for the chains (element 0 = low part, at the higher address; the second instruction uses the X of the first).
// Flags (PRM 4 and PRM 3): ADDX R = D + S + X: C = Sm&Dm | !Rm&Dm | Sm&!Rm, V = Sm&Dm&!Rm | !Sm&!Dm&Rm;
// SUBX R = D - S - X: C = Sm&!Dm | Rm&!Dm | Sm&Rm, V = !Sm&Dm&!Rm | Sm&!Dm&Rm; X = C; N = Rm; Z cleared if R != 0.
struct XC { const char* name; uint16_t base; int sz; int rx, ry; uint32_t ay_off, ax_off; int n; uint32_t src[2], dst[2], res[2]; uint8_t ccr; uint8_t ccr_after[2]; };
static const XC CASES[] = {
    {"ADDX.B -(A2),-(A1) 0x34+0x12", 0xD108, 0, 1, 2, 12, 13, 1, {0x12, 0}, {0x34, 0}, {0x46, 0}, 0x04, {0x00, 0}},          // Z cleared
    {"ADDX.B 0x80+0x80", 0xD108, 0, 1, 2, 13, 12, 1, {0x80, 0}, {0x80, 0}, {0x00, 0}, 0x04, {0x17, 0}},                      // C V, Z unchanged
    {"ADDX.B 0x00+0xFF+X", 0xD108, 0, 1, 2, 12, 12, 1, {0xFF, 0}, {0x00, 0}, {0x00, 0}, 0x10, {0x11, 0}},                    // C (Sm&!Rm), Z stays 0
    {"ADDX.B 0x00+0x7F+X", 0xD108, 0, 1, 2, 13, 13, 1, {0x7F, 0}, {0x00, 0}, {0x80, 0}, 0x14, {0x0A, 0}},                    // V (!Sm&!Dm&Rm), N
    {"ADDX.W 3+2+X", 0xD108, 1, 1, 2, 12, 12, 1, {0x0002, 0}, {0x0003, 0}, {0x0006, 0}, 0x14, {0x00, 0}},
    {"ADDX.W 0x0001+0xFFFF", 0xD108, 1, 1, 2, 12, 12, 1, {0xFFFF, 0}, {0x0001, 0}, {0x0000, 0}, 0x04, {0x15, 0}},            // C, Z unchanged
    {"ADDX.W 0x4000+0x4000", 0xD108, 1, 1, 2, 12, 12, 1, {0x4000, 0}, {0x4000, 0}, {0x8000, 0}, 0x05, {0x0A, 0}},            // V N
    {"ADDX.L 1+0x0000FFFF", 0xD108, 2, 1, 2, 12, 12, 1, {0x0000FFFFu, 0}, {0x00000001u, 0}, {0x00010000u, 0}, 0x05, {0x00, 0}},  // carry between the words
    {"ADDX.L -1+-1+X", 0xD108, 2, 1, 2, 12, 12, 1, {0xFFFFFFFFu, 0}, {0xFFFFFFFFu, 0}, {0xFFFFFFFFu, 0}, 0x14, {0x19, 0}},
    {"ADDX.L 0x80000000+0x80000000", 0xD108, 2, 1, 2, 12, 12, 1, {0x80000000u, 0}, {0x80000000u, 0}, {0x00000000u, 0}, 0x04, {0x17, 0}},
    // 0x00000001_FFFFFFFF + 0x00000000_00000001: low 0 with C (Z stays 1), high 1+0+1 = 2 (Z cleared)
    {"ADDX.L chain 64 bit", 0xD108, 2, 1, 2, 12, 12, 2, {0x00000001u, 0x00000000u}, {0xFFFFFFFFu, 0x00000001u}, {0x00000000u, 0x00000002u}, 0x04, {0x15, 0x00}},
    {"ADDX.B -(A7),-(A1)", 0xD108, 0, 1, 7, 12, 12, 1, {0x11, 0}, {0x22, 0}, {0x33, 0}, 0x04, {0x00, 0}},                    // byte A7: step 2
    {"ADDX.B -(A2),-(A7)", 0xD108, 0, 7, 2, 12, 12, 1, {0x01, 0}, {0xFF, 0}, {0x00, 0}, 0x04, {0x15, 0}},
    {"ADDX.B -(A7),-(A7)", 0xD108, 0, 7, 7, 0, 12, 1, {0x05, 0}, {0x07, 0}, {0x0D, 0}, 0x10, {0x00, 0}},                     // 7+5+X
    {"SUBX.B 0x00-0x01", 0x9108, 0, 1, 2, 12, 13, 1, {0x01, 0}, {0x00, 0}, {0xFF, 0}, 0x04, {0x19, 0}},                      // borrow, N
    {"SUBX.B 0x80-0x01", 0x9108, 0, 1, 2, 13, 12, 1, {0x01, 0}, {0x80, 0}, {0x7F, 0}, 0x00, {0x02, 0}},                      // V (!Sm&Dm&!Rm)
    {"SUBX.W 0-0-X", 0x9108, 1, 1, 2, 12, 12, 1, {0x0000, 0}, {0x0000, 0}, {0xFFFF, 0}, 0x14, {0x19, 0}},
    {"SUBX.W 0x1235-0x1234-X", 0x9108, 1, 1, 2, 12, 12, 1, {0x1234, 0}, {0x1235, 0}, {0x0000, 0}, 0x14, {0x04, 0}},          // zero: Z stays 1
    {"SUBX.W 2-1-X Z=0", 0x9108, 1, 1, 2, 12, 12, 1, {0x0001, 0}, {0x0002, 0}, {0x0000, 0}, 0x10, {0x00, 0}},                // zero: Z stays 0
    {"SUBX.L 0x00010000-1", 0x9108, 2, 1, 2, 12, 12, 1, {0x00000001u, 0}, {0x00010000u, 0}, {0x0000FFFFu, 0}, 0x04, {0x00, 0}},  // borrow between the words
    {"SUBX.L 0x80000000-1", 0x9108, 2, 1, 2, 12, 12, 1, {0x00000001u, 0}, {0x80000000u, 0}, {0x7FFFFFFFu, 0}, 0x00, {0x02, 0}},
    // 0x00000001_00000000 - 0x00000000_00000001: low 0xFFFFFFFF with borrow (Z cleared), high 1-0-1 = 0 (Z stays 0)
    {"SUBX.L chain 64 bit", 0x9108, 2, 1, 2, 12, 12, 2, {0x00000001u, 0x00000000u}, {0x00000000u, 0x00000001u}, {0xFFFFFFFFu, 0x00000000u}, 0x04, {0x19, 0x00}},
    {"SUBX.B -(A7),-(A7)", 0x9108, 0, 7, 7, 0, 12, 1, {0x01, 0}, {0x10, 0}, {0x0E, 0}, 0x14, {0x00, 0}},                     // 0x10-0x01-X
};
static constexpr uint16_t SR_MASK = 0xFFFF;   // all the flags are defined (PRM 4 ADDX/SUBX)

static void put_n(uint32_t a, uint32_t v, uint32_t nb) { for (uint32_t b = 0; b < nb; b++) mem0[a + b] = (uint8_t)(v >> (8 * (nb - 1 - b))); }
static void build() {
    fill_common();
    movea_l(7, 0x00000F00);   // explicit SSP (the shim starts with SSP = 0)
    int idx = 0, grp = 0;
    for (const XC& c : CASES) {
        std::string nm = std::string("case ") + std::to_string(idx) + " " + c.name;
        uint32_t nb = (c.sz == 0) ? 1u : (c.sz == 1) ? 2u : 4u;
        uint32_t sty = (c.sz == 0 && c.ry == 7) ? 2u : nb, stx = (c.sz == 0 && c.rx == 7) ? 2u : nb;   // PRM 2.2.5
        uint32_t ss = new_slot(), ds = (c.rx == c.ry) ? ss : new_slot();
        uint32_t ay = ss + c.ay_off, ax = ds + c.ax_off, r = ss + c.ax_off;
        if (c.rx == c.ry) movea_l(c.rx, r);
        else { movea_l(c.ry, ay); movea_l(c.rx, ax); }
        move_ccr(c.ccr);
        for (int i = 0; i < c.n; i++) {
            uint32_t sa, da;
            if (c.rx == c.ry) { r -= sty; sa = r; r -= stx; da = r; }   // same register: first the source, then the destination
            else { ay -= sty; sa = ay; ax -= stx; da = ax; }
            put_n(sa, c.src[i], nb); put_n(da, c.dst[i], nb);
            for (uint32_t b = 0; b < nb; b++) expect8(da + b, (uint8_t)(c.res[i] >> (8 * (nb - 1 - b))), 0xFF, nm + " element " + std::to_string(i));
            if (nb == 1) {
                DW& xs = dacc(sa, lane_of(sa)); xs.rd += 1; xs.grp = grp;
                DW& xd = dacc(da, lane_of(da)); xd.rd += 1; xd.wr += 1; xd.grp = grp;
            } else {
                for (uint32_t o = 0; o < nb; o += 2) {
                    DW& xs = dacc(sa + o, 3); xs.rd += 1; xs.grp = grp;
                    DW& xd = dacc(da + o, 3); xd.rd += 1; xd.wr += 1; xd.grp = grp;
                }
            }
            w((uint16_t)(c.base | (c.rx << 9) | (c.sz << 6) | c.ry));
            save_sr((uint16_t)(0x2700 | c.ccr_after[i]), nm + " element " + std::to_string(i), SR_MASK);
            grp++;
        }
        if (c.rx == c.ry) save_a(c.rx, r, nm + " register after");
        else { save_a(c.rx, ax, nm + " Ax after"); save_a(c.ry, ay, nm + " Ay after"); }
        idx++;
    }
    movea_l(7, 0x00000F00);
    end_loop();
    code.push_back(Region{ORG, apc + 8, 6});   // UM: FC 6 = supervisor program
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
