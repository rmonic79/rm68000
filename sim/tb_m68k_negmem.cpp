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

// tb_m68k_negmem.cpp - NEG, NOT and NEGX on memory (A1) in .B (even and odd address), .W and .L, with the flags.
// Tests (expectations by hand, PRM 4 NEG / NOT / NEGX, PRM 3 condition code table):
//  - result in memory: NEG of 0, 1, 0x7F, 0x80, 0x8000, 0x80000000, 0xFFFF, 0x0000FFFF (borrow across the word);
//    NOT of 0, 0xFF, mixed values; NEGX with X = 0 and X = 1, zero result with input Z 0 and 1 (Z "cleared if the result
//    is nonzero; unchanged otherwise").
//  - all five flags X N Z V C after each instruction, with an input CCR chosen to discriminate (X at 1 for NOT, which
//    does not touch it).
//  - bus (UM sect. 8, timing: the memory operand is read and then written back, .L = two reads and two writes): every word
//    of the target read once and then written once (read before write, word by word); lanes: even byte only
//    UDS, odd byte only LDS, word and long both; FC 5 on data, 6 on fetches.
//  - neighbouring bytes (same word and adjacent words) set to different sentinels, whole memory compared with the expected image;
//    A1 intact. Two runs: DTACK immediate and after 3 clocks.
// Does not test: addressing modes other than (An) (CLR with the modes is in tb_m68k_clrmem), execution timing.
// Output: "RESULT: PASS" / "RESULT: FAIL", exit code 0 / 1.

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <string>

static const char* BENCH = "tb_m68k_negmem";

// ---- infrastructure common to the testbenches of the group: byte-wide memory with UDS/LDS, cycle log, two runs, checks ----
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
// all the previous instructions have completed), then 40 clocks. Writes only the enabled lanes; logs every cycle.
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
            chk(c.rw == 1 && c.uds == 1 && c.lds == 1, "fetch in the code: expected word read" + at, 1, (uint32_t)c.rw);
            if (cfc >= 0) chk(c.fc == cfc, "FC of the fetch" + at, (uint32_t)cfc, (uint32_t)c.fc);
        } else if (c.a < 0x400) {
            chk(c.rw == 1 && c.uds == 1 && c.lds == 1, "vector table: expected word read" + at, 1, (uint32_t)c.rw);
        } else chk(false, "cycle outside the expected areas" + at, 0, c.a);
    }
    for (auto& kv : dexp) {
        std::string at = " at " + hx(kv.first & 0xFFFFFFu) + " lanes " + std::to_string(kv.first >> 24);
        if (!kv.second.any_rd) chk(nrd[kv.first] == kv.second.rd, "number of reads" + at, (uint32_t)kv.second.rd, (uint32_t)nrd[kv.first]);
        chk(nwr[kv.first] == kv.second.wr, "number of writes" + at, (uint32_t)kv.second.wr, (uint32_t)nwr[kv.first]);
    }
    for (auto& kv : first_wr)
        if (last_rd.count(kv.first)) chk(last_rd[kv.first] < kv.second, "read after the write (read value = cycle of the last read) at " + hx(kv.first & 0xFFFFFFu) + " lanes " + std::to_string(kv.first >> 24), (uint32_t)kv.second, (uint32_t)last_rd[kv.first]);
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
// ---- end of the common infrastructure ----

// one-operand instruction on (A1): encoding = base | size << 6 | 010 001 (PRM 4: NEGX 0x40, NEG 0x44, NOT 0x46;
// size 00 .B, 01 .W, 10 .L). off = position of the target in the 16-byte block (4 even, 5 odd).
// Flags (PRM 4 and PRM 3):
//   NEG:  R = 0 - D; X = C = Dm | Rm (cleared only if R = 0); V = Dm & Rm; N = Rm; Z = (R == 0).
//   NEGX: R = 0 - D - X; X = C = Dm | Rm; V = Dm & Rm; N = Rm; Z cleared if R != 0, otherwise unchanged.
//   NOT:  R = ~D; N = Rm; Z = (R == 0); V = C = 0; X unchanged.
struct SC { const char* name; uint16_t base; int sz; int off; uint32_t init; uint8_t ccr; uint32_t exp; uint8_t exp_ccr; };
static const SC CASES[] = {
    // NEG.B
    {"NEG.B even 0x00", 0x4400, 0, 4, 0x00u, 0x1B, 0x00u, 0x04},            // R=0: all 0, Z1
    {"NEG.B odd 0x01", 0x4400, 0, 5, 0x01u, 0x06, 0xFFu, 0x19},         // Dm0 Rm1: V0, C=X=1, N1
    {"NEG.B even 0x80", 0x4400, 0, 4, 0x80u, 0x04, 0x80u, 0x1B},            // Dm1 Rm1: V1, C=X=1, N1
    {"NEG.B odd 0x7F", 0x4400, 0, 5, 0x7Fu, 0x02, 0x81u, 0x19},         // Dm0 Rm1
    // NEG.W
    {"NEG.W 0x0000", 0x4400, 1, 4, 0x0000u, 0x1F, 0x0000u, 0x04},
    {"NEG.W 0x8000", 0x4400, 1, 4, 0x8000u, 0x00, 0x8000u, 0x1B},
    {"NEG.W 0x0003", 0x4400, 1, 4, 0x0003u, 0x04, 0xFFFDu, 0x19},
    {"NEG.W 0xFFFF", 0x4400, 1, 4, 0xFFFFu, 0x0E, 0x0001u, 0x11},           // Dm1 Rm0: V0, C=X=1, N0
    // NEG.L
    {"NEG.L 0x00000000", 0x4400, 2, 4, 0x00000000u, 0x13, 0x00000000u, 0x04},
    {"NEG.L 0x80000000", 0x4400, 2, 4, 0x80000000u, 0x04, 0x80000000u, 0x1B},
    {"NEG.L 0x00000001", 0x4400, 2, 4, 0x00000001u, 0x06, 0xFFFFFFFFu, 0x19},
    {"NEG.L 0x12345678", 0x4400, 2, 4, 0x12345678u, 0x00, 0xEDCBA988u, 0x19},  // ~0x12345678 + 1
    {"NEG.L 0x0000FFFF", 0x4400, 2, 4, 0x0000FFFFu, 0x04, 0xFFFF0001u, 0x19},  // borrow across the word
    // NOT
    {"NOT.B even 0x0F", 0x4600, 0, 4, 0x0Fu, 0x13, 0xF0u, 0x18},            // N1, V=C=0, X=1 unchanged
    {"NOT.B odd 0xFF", 0x4600, 0, 5, 0xFFu, 0x0B, 0x00u, 0x04},         // Z1, X=0 unchanged
    {"NOT.W 0x0F0F", 0x4600, 1, 4, 0x0F0Fu, 0x17, 0xF0F0u, 0x18},
    {"NOT.W 0xFFFF", 0x4600, 1, 4, 0xFFFFu, 0x1B, 0x0000u, 0x14},
    {"NOT.L 0x00000000", 0x4600, 2, 4, 0x00000000u, 0x07, 0xFFFFFFFFu, 0x08},
    {"NOT.L 0x80FF00FF", 0x4600, 2, 4, 0x80FF00FFu, 0x1F, 0x7F00FF00u, 0x10},
    // NEGX
    {"NEGX.B even 0x00 X=0 Z=1", 0x4000, 0, 4, 0x00u, 0x04, 0x00u, 0x04},      // R=0: Z unchanged (1), C=X=0
    {"NEGX.B odd 0x00 X=1", 0x4000, 0, 5, 0x00u, 0x14, 0xFFu, 0x19},       // R=0-0-1=0xFF: Z cleared, C=X=1, N1
    {"NEGX.B even 0x80 X=0", 0x4000, 0, 4, 0x80u, 0x04, 0x80u, 0x1B},          // V1
    {"NEGX.W 0x0000 X=0 Z=0", 0x4000, 1, 4, 0x0000u, 0x0B, 0x0000u, 0x00},     // R=0: Z unchanged (0)
    {"NEGX.W 0x7FFF X=1", 0x4000, 1, 4, 0x7FFFu, 0x10, 0x8000u, 0x19},         // 0-0x7FFF-1 = 0x8000, Dm0: V0
    {"NEGX.L 0x00000000 X=1", 0x4000, 2, 4, 0x00000000u, 0x14, 0xFFFFFFFFu, 0x19},
    {"NEGX.L 0x00000001 X=0", 0x4000, 2, 4, 0x00000001u, 0x04, 0xFFFFFFFFu, 0x19},
    {"NEGX.L 0x80000000 X=0", 0x4000, 2, 4, 0x80000000u, 0x00, 0x80000000u, 0x1B},
    {"NEGX.L 0xFFFFFFFF X=1 Z=1", 0x4000, 2, 4, 0xFFFFFFFFu, 0x14, 0x00000000u, 0x15},  // 0-(-1)-1 = 0: Z unchanged, Dm1: C=X=1
};

static void build() {
    fill_common();
    movea_l(7, 0x00000F00);   // explicit SSP (the shim starts with SSP = 0)
    int idx = 0;
    for (const SC& c : CASES) {
        std::string nm = std::string("case ") + std::to_string(idx) + " " + c.name;
        uint32_t a = new_slot() + (uint32_t)c.off;
        if (c.sz == 0) {
            mem0[a] = (uint8_t)c.init;
            expect8(a, (uint8_t)c.exp, 0xFF, nm);
            DW& x = dacc(a, lane_of(a)); x.rd += 1; x.wr += 1; x.grp = idx;
        } else if (c.sz == 1) {
            put16(a, (uint16_t)c.init);
            expect8(a, (uint8_t)(c.exp >> 8), 0xFF, nm); expect8(a + 1, (uint8_t)c.exp, 0xFF, nm);
            DW& x = dacc(a, 3); x.rd += 1; x.wr += 1; x.grp = idx;
        } else {
            put32(a, c.init);
            for (int b = 0; b < 4; b++) expect8(a + (uint32_t)b, (uint8_t)(c.exp >> (24 - 8 * b)), 0xFF, nm);
            DW& x0 = dacc(a, 3); x0.rd += 1; x0.wr += 1; x0.grp = idx;
            DW& x1 = dacc(a + 2, 3); x1.rd += 1; x1.wr += 1; x1.grp = idx;
        }
        movea_l(1, a);
        move_ccr(c.ccr);
        w((uint16_t)(c.base | (c.sz << 6) | 0x11));
        save_sr((uint16_t)(0x2700 | c.exp_ccr), nm);   // supervisor, mask 7 from reset (UM reset), expected CCR
        save_a(1, a, nm + " A1 intact");
        idx++;
    }
    end_loop();
    code.push_back(Region{ORG, apc + 8, 6});   // UM: FC 6 = supervisor program
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    for (int pass = 0; pass < 2; pass++) {
        int waits = pass ? 3 : 0;
        tag = pass ? "[DTACK after 3 clocks] " : "[DTACK immediately] ";
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
