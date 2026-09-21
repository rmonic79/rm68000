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

// tb_m68k_adda.cpp - ADDA/SUBA/CMPA <ea>,An with source Dn, An and (An), in .W and .L (the #imm form is in tb_m68k_addai).
// Tests (expectations worked out by hand from the manual, PRM 4 ADDA / SUBA / CMPA, PRM 3 condition code table):
//  - .W: the word source is sign-extended to 32 bits and the operation covers all 32 bits of An (the upper half of the
//    preloaded source is non-zero: it must be ignored); .L: 32 bits, with carries/borrows that cross the word.
//  - ADDA/SUBA do not touch the CCR: non-zero input CCR (0x15 / 0x0A / 0x1F alternated) and SR saved right after.
//  - CMPA: An does not change; X unchanged; N Z V C of the difference An - sext(src) on equal, lower, greater values and with
//    overflow in both directions; input CCR chosen with the flags opposite to the expected ones.
//  - the source does not change (register saved after the instruction); (An) source: one read per word, no write,
//    FC 5 (UM: supervisor data); base An intact.
//  - every bus cycle: FC 6 for fetches and 5 for data, word fetches; no write outside the result area;
//    whole memory compared with the expected image (sentinels intact). Two runs: DTACK immediate and after 3 clocks.
// Does not test: execution times (UM sec. 8) and addressing modes other than Dn, An, (An).
// Exit: "RESULT: PASS" / "RESULT: FAIL", code 0 / 1.

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <string>

static const char* BENCH = "tb_m68k_adda";

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

// cases: destination An = A2; source D3 (src 0), A3 (src 1) or (A4) (src 2, A4 = block holding the operand)
struct AC { const char* name; uint16_t op; int src; int sz; uint32_t dst; uint32_t srcv; uint8_t ccr; uint32_t exp_dst; uint8_t exp_ccr; };
// Encodings (PRM 4): ADDA 1101 An s11 ea, SUBA 1001 An s11 ea, CMPA 1011 An s11 ea (s = 0 .W, 1 .L); ea D3 = 000011,
// A3 = 001011, (A4) = 010100; An = A2. ADDA/SUBA: CCR unchanged (PRM 4 ADDA/SUBA: "Condition codes: not affected").
// CMPA (PRM 4 CMPA, PRM 3 CMP table): R = An - sext(src); N = Rm; Z = R==0; V = !Sm&Dm&!Rm | Sm&!Dm&Rm;
// C = Sm&!Dm | Rm&!Dm | Sm&Rm; X unchanged.
static const AC CASES[] = {
    // ADDA.W: 0x00010000 + sext(0xFFFE) = 0x00010000 - 2 = 0x0000FFFE; upper half of D3 (0x1234) ignored
    {"ADDA.W D3,A2 sext(0xFFFE)", 0xD4C3, 0, 1, 0x00010000u, 0x1234FFFEu, 0x15, 0x0000FFFEu, 0x15},
    // 0x12345678 + 0x00007FFF = 0x1234D677 (0x5678 + 0x7FFF = 0xD677, no carry beyond 16 bits)
    {"ADDA.W D3,A2 sext(0x7FFF)", 0xD4C3, 0, 1, 0x12345678u, 0xFFFF7FFFu, 0x0A, 0x1234D677u, 0x0A},
    // 0xFFFFFFFF + 1 = 0 (carry beyond bit 31: no flag changes)
    {"ADDA.W D3,A2 0xFFFFFFFF+1", 0xD4C3, 0, 1, 0xFFFFFFFFu, 0x00000001u, 0x0A, 0x00000000u, 0x0A},
    // 0 + sext(0x8000) = 0xFFFF8000
    {"ADDA.W D3,A2 sext(0x8000)", 0xD4C3, 0, 1, 0x00000000u, 0x00008000u, 0x15, 0xFFFF8000u, 0x15},
    {"ADDA.L D3,A2", 0xD5C3, 0, 2, 0x00000010u, 0x12340000u, 0x1F, 0x12340010u, 0x1F},
    // 0x80000000 + 0x80000000 = 0 (mod 2^32)
    {"ADDA.L D3,A2 0x80000000+0x80000000", 0xD5C3, 0, 2, 0x80000000u, 0x80000000u, 0x0A, 0x00000000u, 0x0A},
    {"ADDA.L D3,A2 0x7FFFFFFF+1", 0xD5C3, 0, 2, 0x7FFFFFFFu, 0x00000001u, 0x15, 0x80000000u, 0x15},
    // 0x2000 + sext(0xFFF0) = 0x2000 - 16 = 0x1FF0; upper half of A3 (0x5555) ignored
    {"ADDA.W A3,A2 sext(0xFFF0)", 0xD4CB, 1, 1, 0x00002000u, 0x5555FFF0u, 0x0A, 0x00001FF0u, 0x0A},
    {"ADDA.L A3,A2", 0xD5CB, 1, 2, 0x10203040u, 0x01020304u, 0x15, 0x11223344u, 0x15},
    // 0x00010000 + sext(0x8001) = 0x00010000 - 0x7FFF = 0x00008001
    {"ADDA.W (A4),A2 sext(0x8001)", 0xD4D4, 2, 1, 0x00010000u, 0x00008001u, 0x1F, 0x00008001u, 0x1F},
    // 0x01234568 + 0xFEDCBA98 = 0x1_00000000 -> 0 (carry in every byte)
    {"ADDA.L (A4),A2", 0xD5D4, 2, 2, 0x01234568u, 0xFEDCBA98u, 0x0A, 0x00000000u, 0x0A},
    // SUBA.W: 0x00010000 - sext(0x0002) = 0x0000FFFE; upper half of D3 (0xABCD) ignored
    {"SUBA.W D3,A2 sext(0x0002)", 0x94C3, 0, 1, 0x00010000u, 0xABCD0002u, 0x15, 0x0000FFFEu, 0x15},
    // 0 - sext(0xFFFF) = 0 - (-1) = 1
    {"SUBA.W D3,A2 0-sext(0xFFFF)", 0x94C3, 0, 1, 0x00000000u, 0x0000FFFFu, 0x0A, 0x00000001u, 0x0A},
    // 0x1000 - sext(0x8000) = 0x1000 + 0x8000 = 0x9000
    {"SUBA.W D3,A2 sext(0x8000)", 0x94C3, 0, 1, 0x00001000u, 0x00008000u, 0x1F, 0x00009000u, 0x1F},
    {"SUBA.L D3,A2 0-1", 0x95C3, 0, 2, 0x00000000u, 0x00000001u, 0x0A, 0xFFFFFFFFu, 0x0A},
    {"SUBA.L D3,A2 0x80000000-1", 0x95C3, 0, 2, 0x80000000u, 0x00000001u, 0x15, 0x7FFFFFFFu, 0x15},
    // 0x7FFFFFFF - sext(0xFFFF) = 0x7FFFFFFF + 1 = 0x80000000
    {"SUBA.W A3,A2 sext(0xFFFF)", 0x94CB, 1, 1, 0x7FFFFFFFu, 0x1234FFFFu, 0x0A, 0x80000000u, 0x0A},
    {"SUBA.L A3,A2", 0x95CB, 1, 2, 0x33333333u, 0x11111111u, 0x15, 0x22222222u, 0x15},
    // 0x80 - sext(0x0100) = -0x80 = 0xFFFFFF80
    {"SUBA.W (A4),A2", 0x94D4, 2, 1, 0x00000080u, 0x00000100u, 0x1F, 0xFFFFFF80u, 0x1F},
    {"SUBA.L (A4),A2", 0x95D4, 2, 2, 0x00010000u, 0x00010001u, 0x0A, 0xFFFFFFFFu, 0x0A},
    // CMPA.L equal: R = 0 -> N0 Z1 V0 C0; X = 1 from the input CCR 0x1B -> 0x14
    {"CMPA.L D3,A2 equal", 0xB5C3, 0, 2, 0x00000100u, 0x00000100u, 0x1B, 0x00000100u, 0x14},
    // An lower: 0x100 - 0x200 = 0xFFFFFF00 -> N1, V0 (S, D positive), C1 (Rm&!Dm); X = 0 -> 0x09
    {"CMPA.L D3,A2 lower", 0xB5C3, 0, 2, 0x00000100u, 0x00000200u, 0x04, 0x00000100u, 0x09},
    // An greater: 0x300 - 0x200 = 0x100 -> all 0; X = 1 -> 0x10
    {"CMPA.L D3,A2 greater", 0xB5C3, 0, 2, 0x00000300u, 0x00000200u, 0x1F, 0x00000300u, 0x10},
    // 0x80000000 - 1 = 0x7FFFFFFF -> V1 (!Sm&Dm&!Rm), C0, N0, Z0; X = 0 -> 0x02
    {"CMPA.L D3,A2 overflow 0x80000000-1", 0xB5C3, 0, 2, 0x80000000u, 0x00000001u, 0x05, 0x80000000u, 0x02},
    // 0x7FFFFFFF - 0xFFFFFFFF = 0x80000000 -> N1, V1 (Sm&!Dm&Rm), C1 (Sm&!Dm); X = 1 -> 0x1B
    {"CMPA.L D3,A2 overflow 0x7FFFFFFF-(-1)", 0xB5C3, 0, 2, 0x7FFFFFFFu, 0xFFFFFFFFu, 0x10, 0x7FFFFFFFu, 0x1B},
    // CMPA.W: sext(0xFFFF) = 0xFFFFFFFF = An -> Z1 (without extension: 0xFFFF0000, N1); X = 0 -> 0x04
    {"CMPA.W D3,A2 sext(0xFFFF)", 0xB4C3, 0, 1, 0xFFFFFFFFu, 0x0000FFFFu, 0x0B, 0xFFFFFFFFu, 0x04},
    // 0x00008000 - sext(0x8000) = 0x00008000 - 0xFFFF8000 = 0x00010000 -> N0 Z0 V0, C1 (Sm&!Dm); X = 1 -> 0x11
    {"CMPA.W D3,A2 sext(0x8000)", 0xB4C3, 0, 1, 0x00008000u, 0x12348000u, 0x1E, 0x00008000u, 0x11},
    // sext(0x0005) = 5 = An -> Z1; X = 1 -> 0x14 (upper half of A3 = 0x7FFF ignored)
    {"CMPA.W A3,A2 equal", 0xB4CB, 1, 1, 0x00000005u, 0x7FFF0005u, 0x11, 0x00000005u, 0x14},
    // 0 - 0xFFFFFFFF = 1 -> N0 Z0 V0, C1 (Sm&!Dm); X = 0 -> 0x01
    {"CMPA.L A3,A2", 0xB5CB, 1, 2, 0x00000000u, 0xFFFFFFFFu, 0x0E, 0x00000000u, 0x01},
    // -3 - sext(0xFFFE) = -3 - (-2) = 0xFFFFFFFF -> N1, V0, C1 (Sm&Rm); X = 1 -> 0x19
    {"CMPA.W (A4),A2", 0xB4D4, 2, 1, 0xFFFFFFFDu, 0x0000FFFEu, 0x14, 0xFFFFFFFDu, 0x19},
    {"CMPA.L (A4),A2 equal", 0xB5D4, 2, 2, 0x12345678u, 0x12345678u, 0x0B, 0x12345678u, 0x04},
};

static void build() {
    fill_common();
    movea_l(7, 0x00000F00);   // explicit SSP (the shim starts with SSP = 0)
    int idx = 0;
    for (const AC& c : CASES) {
        std::string nm = std::string("case ") + std::to_string(idx) + " " + c.name;
        uint32_t s = 0;
        movea_l(2, c.dst);
        if (c.src == 0) move_l_d(3, c.srcv);
        else if (c.src == 1) movea_l(3, c.srcv);
        else {
            s = new_slot() + 4;
            movea_l(4, s);
            if (c.sz == 1) { put16(s, (uint16_t)c.srcv); dacc(s, 3).rd += 1; }
            else { put32(s, c.srcv); dacc(s, 3).rd += 1; dacc(s + 2, 3).rd += 1; }
        }
        move_ccr(c.ccr);
        w(c.op);
        save_sr((uint16_t)(0x2700 | c.exp_ccr), nm);   // supervisor, mask 7 from reset (UM reset), expected CCR
        save_a(2, c.exp_dst, nm);
        if (c.src == 0) save_d(3, c.srcv, nm + " source intact");
        else if (c.src == 1) save_a(3, c.srcv, nm + " source intact");
        else save_a(4, s, nm + " base intact");
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
