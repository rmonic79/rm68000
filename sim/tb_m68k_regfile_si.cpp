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

// tb_m68k_regfile_si.cpp - verification of the register file (rtl/cpu_si/m68k_regfile_si.sv) and of the MOVEM encoder (m68k_mvm_si.sv) inside
// m68k_regfile_top, in the CORE configuration (real control store, real section fabric, real bus-side registers; parameters of
// m68k_cpu_si: IR_REG, WE_DEC_ONLY, PC_DEC_ONLY, WD_NARROW, WE_FLOP = 1, PC_W17 = 0; ORG_OVR_LUT = 0).
// The testbench is independent of the RTL and of the generators: the commands are read from the dec vector of the control store (pkg fields,
// m68k_ndec_fields.h); the bus sections and the "driven" flags are observed from the real fabric (verified by the bussec testbench); the
// model applies the rules of docs/silicon-datapath.md §8.2-8.6 (register selection: instruction formats of the
// MC68000UM manual, REGISTER POINTERS key of the patent; pointers persistent until the dispatch; rz = IRC[15:12]; writes
// H from X_h and L from X_l with fallback X_d, rxdl/rydl from X_d with fallback X_l, only if the section is driven; pc<-au from the AU; on bytes
// the low word of the DATA registers changes only in the low byte) and §13 (MOVEM encoder: mask with ftu<-ab, `reset pren`
// clears the lowest bit, reversed order with -(An), END = mask at zero). Declared model choices: the atoms of the high
// half only (rxh/ryh/sph/pch) take the driven section of the bus (H, otherwise L, otherwise D: the 16-bit transfer
// goes into the high half); the same half written in the same word by the TWO buses with different values is not expected by the model
// (counted, the register is realigned to the DUT). No generated file is used.
// For EVERY microcode word (positioning along the reset chain + A1): 13 instruction classes (MOVE.L Dy,Dx on all the
// 64 RX/RY pairs; MOVEA.L, EA (Ay), EXG Ax,Ay and Dx,Ay, DBcc, d16(PC), LEA, ADDA.L, ASL Dx,Dy, MOVE.B on RX,RY in {0,7} and (2,5);
// MOVEM -(Ay) and (Ay)+ on RY) x alternating S, index register IRC and external buses from a pattern; on the words without register file atoms only S =
// 0/1. At every microcycle (path included): hold (en = 0: nothing changes) then execution; read ports rf_ab/rf_db/rf_db_sp/pc_q
// at en+1 AND at en+2 (forwarding of the queued DB write), encoder rp_reg/mvm_end, and the WHOLE STATE (D0-D7, A0-A6, USP, SSP, DT, PC)
// from the debug ports. Outcome: RESULT: PASS/FAIL, exit code 0/1. +trace=N prints the first N microcycles.
#include "Vm68k_regfile_top.h"
#include "verilated.h"
#include "m68k_ndec_fields.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
static Vm68k_regfile_top* dut;
static int fails = 0, checks = 0;
static long trace_n = 0;
double sc_time_stamp() { return 0.0; }
static void tick() { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); dut->clk = 0; dut->eval(); }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// --- model state: 0-7 D0-D7, 8-14 A0-A6, 15 USP, 17 PC, 18 DT, 31 SSP (debug port indices)
static uint32_t R[32];
static int cur_ird = 0;                      // instruction being executed (changes at en in the words with ird<-(ir))
static std::string PD, PS; static bool RZ = false, own_pd = false, own_ps = false;   // persistent pointers (§8.3) and rz (§8.4)
static int MASK = 0, CUR = 0, PREDEC = 0;    // MOVEM encoder (§13): mask, current register, -(An) mode of the instruction
static long n_steps = 0, n_hold = 0, n_writes = 0, n_pos = 0, n_rd = 0, n_fwd = 0, n_conf = 0, n_mask0 = 0;
static long n_skip = 0, n_hfb = 0;
static std::map<int, std::pair<int, std::string>> hfb_words;   // word -> (occurrences, atom/section) of the high-half-only atoms served by the L/D fallback
static const char* IDXNAME[32] = {"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "A0", "A1", "A2", "A3", "A4", "A5", "A6", "USP", "-", "PC", "DT", "-", "-", "-", "-", "-", "-", "-", "-", "-", "-", "-", "-", "SSP"};
// REGISTER POINTERS cell of the word: dt/rx/sp/us/rp set the destination pointer (rx = no override), dt/ry/py the source
// one (ry = no override); dx/dy/uk do not change it. The pointer is already valid in the word and stays until the dispatch.
static void pointers(const std::set<std::string>& A) {
    RZ = A.count("rxsrc=rz") > 0; own_pd = false; own_ps = false;
    for (const char* v : {"dt", "rx", "sp", "us", "rp"}) if (A.count(std::string("rp_dst=") + v)) { PD = v; own_pd = true; }
    for (const char* v : {"dt", "ry", "py"}) if (A.count(std::string("rp_src=") + v)) { PS = v; own_ps = true; }
}
static int mvm_reg() { return PREDEC ? (15 - CUR) : CUR; }   // current register of the encoder: with -(An) the order is reversed (manual)
struct Sel { int rx, rxa, rxd, ry, rya, ryd, sp, pc; };
static bool is_dbcc(int ird) { return ((ird >> 12) & 15) == 5 && (((ird >> 3) & 0x1f) == 0x19); }   // DBcc: the counter is IR[2:0], the microcode uses it as rx (App. H dcnt)
// rx of type A: MOVEA (lines 1/2/3 with IRD 8:6 = 001), EXG Ax,Ay, LEA (line 4, 8:6 = 111), ADDA/SUBA/CMPA (lines 9/B/D, opmode x11)
static bool rx_is_addr(int ird) {
    int ln = (ird >> 12) & 15; bool exg = (ln == 0xc) && ((ird >> 8) & 1); int exgm = (ird >> 3) & 0x1f;
    return ((ln == 1 || ln == 2 || ln == 3) && ((ird >> 6) & 7) == 1) || (exg && exgm == 0x09) || (ln == 4 && ((ird >> 6) & 7) == 7) || ((ln == 9 || ln == 0xb || ln == 0xd) && ((ird >> 6) & 3) == 3);
}
// ry of type A: EA mode other than Dn and 111 (not for register shifts, line E with size != 11, nor for DBcc), EXG Ax,Ay and Dx,Ay
static bool ry_is_addr(int ird) {
    int ln = (ird >> 12) & 15, md = (ird >> 3) & 7; bool exg = (ln == 0xc) && ((ird >> 8) & 1); int exgm = (ird >> 3) & 0x1f;
    return ((md != 0 && md != 7) && !(ln == 0xe && ((ird >> 6) & 3) != 3) && !is_dbcc(ird)) || (exg && (exgm == 0x09 || exgm == 0x11));
}
// register selection (§8.2): D/A type from the instruction format (MC68000UM manual), override from the pointers and from rz
static Sel select() {
    int ird = cur_ird; Sel s; int sp = dut->sup ? 31 : 15; s.sp = sp; s.pc = 17;
    int RX = (ird >> 9) & 7, RY = ird & 7, md = (ird >> 3) & 7;
    int RXD = is_dbcc(ird) ? RY : RX;
    bool rx_a = rx_is_addr(ird), ry_a = ry_is_addr(ird);
    bool pcrel = (md == 7) && (((ird >> 1) & 3) == 1);              // modes d16(PC) and d8(PC,Xn)
    int c_x7 = RX == 7 ? sp : 8 + RX, c_y7 = RY == 7 ? sp : 8 + RY;   // A7 = USP/SSP according to S
    int rx_def = rx_a ? c_x7 : RXD, ry_def = ry_a ? c_y7 : RY;
    int pv = -1;   // destination pointer
    if (PD == "dt") pv = 18; else if (PD == "sp") pv = sp; else if (PD == "us") pv = 15; else if (PD == "rp") { int r = mvm_reg(); pv = r == 15 ? sp : r; }
    int rz = -1; if (RZ) { int r = (dut->irc >> 12) & 15; rz = r == 15 ? sp : r; }   // index register of the extension word (§8.4)
    s.rxa = pv >= 0 ? pv : c_x7;
    s.rx  = rz >= 0 ? rz : (pv >= 0 ? pv : rx_def);
    s.rxd = pv >= 0 ? (rz >= 0 ? rz : pv) : RXD;
    if (PS == "dt") { s.ry = 18; s.rya = 18; s.ryd = 18; }
    else if (PS == "py" && pcrel) { s.ry = 17; s.rya = 17; s.ryd = RY; }
    else { s.ry = ry_def; s.rya = c_y7; s.ryd = RY; }
    return s;
}
static int reg_of(const std::string& n, const Sel& s) {   // App. H name -> index
    if (n == "rxa") return s.rxa; if (n == "rxd" || n == "rxdl") return s.rxd; if (n.rfind("rx", 0) == 0) return s.rx;
    if (n == "rya") return s.rya; if (n == "ryd" || n == "rydl") return s.ryd; if (n.rfind("ry", 0) == 0) return s.ry;
    if (n.rfind("sp", 0) == 0) return s.sp; if (n.rfind("pc", 0) == 0) return s.pc;
    return -1;
}
// bus sections observed from the DUT (value and "driven" flag); the AU halves are always valid
static bool sec(const std::string& c, uint16_t& v) {
    if (c == "ab_h") { v = dut->ab_h; return dut->ab_h_drv; } if (c == "ab_l") { v = dut->ab_l; return dut->ab_l_drv; } if (c == "ab_d") { v = dut->ab_d; return dut->ab_d_drv; }
    if (c == "db_h") { v = dut->db_h; return dut->db_h_drv; } if (c == "db_l") { v = dut->db_l; return dut->db_l_drv; } if (c == "db_d") { v = dut->db_d; return dut->db_d_drv; }
    if (c == "au_h") { v = (uint16_t)(dut->src_au >> 16); return true; } if (c == "au_l") { v = (uint16_t)dut->src_au; return true; }
    printf("FAIL: unknown section '%s'\n", c.c_str()); printf("RESULT: FAIL\n"); exit(1);
}
static int chain(const char* ch, uint16_t& v) {   // first driven section of the chain "a|b|c": index (0 = the first), -1 = none
    std::string s(ch); size_t p = 0;
    for (int i = 0;; i++) { size_t b = s.find('|', p); std::string c = s.substr(p, b == std::string::npos ? std::string::npos : b - p); if (sec(c, v)) return i; if (b == std::string::npos) return -1; p = b + 1; }
}
// write rules (§8.6): atom -> register (App. H name), half (h/l), chain of sections (first driven)
struct WRule { const char* atom; const char* reg; char half; const char* chain; };
static const WRule WRULES[] = {
    {"rx<-ab", "rx", 'h', "ab_h"}, {"rx<-ab", "rx", 'l', "ab_l|ab_d"}, {"rx<-abe", "rx", 'h', "ab_h"}, {"rx<-abe", "rx", 'l', "ab_l|ab_d"}, {"rx<-db", "rx", 'h', "db_h"}, {"rx<-db", "rx", 'l', "db_l|db_d"},
    {"rxa<-ab", "rxa", 'h', "ab_h"}, {"rxa<-ab", "rxa", 'l', "ab_l|ab_d"}, {"rxa<-db", "rxa", 'h', "db_h"}, {"rxa<-db", "rxa", 'l', "db_l|db_d"},
    {"rxh<-ab", "rx", 'h', "ab_h|ab_l|ab_d"}, {"rxh<-db", "rx", 'h', "db_h|db_l|db_d"}, {"rxl<-ab", "rx", 'l', "ab_l|ab_d"}, {"rxl<-dbe", "rx", 'l', "db_l|db_d"},
    {"rxdl<-ab*", "rxdl", 'l', "ab_d|ab_l"}, {"rxdl<-abe", "rxdl", 'l', "ab_d|ab_l"}, {"rxdl<-dbd", "rxdl", 'l', "db_d|db_l"},
    {"ry<-ab", "ry", 'h', "ab_h"}, {"ry<-ab", "ry", 'l', "ab_l|ab_d"}, {"ry<-abe", "ry", 'h', "ab_h"}, {"ry<-abe", "ry", 'l', "ab_l|ab_d"}, {"ry<-db", "ry", 'h', "db_h"}, {"ry<-db", "ry", 'l', "db_l|db_d"},
    {"rya<-ab", "rya", 'h', "ab_h"}, {"rya<-ab", "rya", 'l', "ab_l|ab_d"}, {"rya<-db", "rya", 'h', "db_h"}, {"rya<-db", "rya", 'l', "db_l|db_d"},
    {"ryh<-ab", "ry", 'h', "ab_h|ab_l|ab_d"}, {"ryh<-abh", "ry", 'h', "ab_h|ab_l|ab_d"}, {"ryh<-db", "ry", 'h', "db_h|db_l|db_d"}, {"ryl<-ab", "ry", 'l', "ab_l|ab_d"}, {"ryl<-abl", "ry", 'l', "ab_l|ab_d"}, {"ryl<-db", "ry", 'l', "db_l|db_d"},
    {"rydl<-ab", "rydl", 'l', "ab_d|ab_l"}, {"rydl<-ab*", "rydl", 'l', "ab_d|ab_l"}, {"rydl<-dbd", "rydl", 'l', "db_d|db_l"},
    {"sp<-db", "sp", 'h', "db_h"}, {"sp<-db", "sp", 'l', "db_l|db_d"}, {"sph<-ab", "sp", 'h', "ab_h|ab_l|ab_d"}, {"spl<-ab", "sp", 'l', "ab_l|ab_d"},
    {"pc<-au", "pc", 'h', "au_h"}, {"pc<-au", "pc", 'l', "au_l"}, {"pc<-ab", "pc", 'h', "ab_h"}, {"pc<-ab", "pc", 'l', "ab_l|ab_d"}, {"pc<-db", "pc", 'h', "db_h"}, {"pc<-db", "pc", 'l', "db_l|db_d"},
    {"pch<-dbh", "pc", 'h', "db_h|db_l|db_d"}, {"pcl<-ab", "pc", 'l', "ab_l|ab_d"},
};
// reads: atoms "<bus>...<-(name)" with a register name
static bool read_reg(const std::string& atom, std::string& bus, std::string& reg) {
    size_t p = atom.find("<-("); if (p == std::string::npos || atom.back() != ')') return false;
    std::string b = atom.substr(0, p), n = atom.substr(p + 3, atom.size() - p - 4);
    static const std::set<std::string> NAMES = {"rx", "rxh", "rxl", "rxa", "rxd", "rxdl", "ry", "ryh", "ryl", "rya", "ryd", "rydl", "sp", "sph", "spl", "pc", "pch", "pcl"};
    if (!NAMES.count(n)) return false;
    if (b.rfind("ab", 0) == 0) bus = "ab"; else if (b.rfind("db", 0) == 0) bus = "db"; else return false;
    reg = n; return true;
}
static bool writes_reg(const std::set<std::string>& A, const char* pre) {   // the word writes a register of the family pre ("rx"/"ry")
    for (const std::string& a : A) for (const WRule& w : WRULES) if (a == w.atom && std::string(w.reg).rfind(pre, 0) == 0) return true;
    return false;
}
// the word writes both rx and ry and with this instruction the two names fall on the same register (A7 = SP in both cases)
static bool collides(const std::set<std::string>& A, int ird) {
    if (!(writes_reg(A, "rx") && writes_reg(A, "ry"))) return false;
    int RX = (ird >> 9) & 7, RY = ird & 7;
    int rxi = rx_is_addr(ird) ? 8 + RX : (is_dbcc(ird) ? RY : RX), ryi = ry_is_addr(ird) ? 8 + RY : RY;
    return rxi == ryi;
}
static bool regword(const std::set<std::string>& A) {   // the word touches the register file or the encoder
    for (const std::string& a : A) {
        std::string b, r; if (read_reg(a, b, r)) return true;
        for (const WRule& w : WRULES) if (a == w.atom) return true;
        if (a.rfind("rp_dst=", 0) == 0 || a.rfind("rp_src=", 0) == 0 || a == "rxsrc=rz" || a == "ftu<-ab" || a == "reset pren") return true;
    }
    return false;
}
// --- pattern of the external buses and of the index register
static const uint32_t P32[16] = {0x00000000u, 0xffffffffu, 0x80000000u, 0x7fffffffu, 0x00008000u, 0x0000ffffu, 0xffff0000u, 0x12345678u, 0x00000001u, 0x55555555u, 0xaaaaaaaau, 0x0f0f0f0fu, 0x00010000u, 0x80008000u, 0xfffefffeu, 0x7fff8000u};
static const uint16_t P16[16] = {0x0000, 0xffff, 0x8000, 0x7fff, 0x0001, 0x00ff, 0xff00, 0x5555, 0xaaaa, 0x0f0f, 0x1234, 0x8001, 0x7ffe, 0x0100, 0xfffe, 0x4321};
static long pi_g = 0;
static void set_sources(long pi) {
    dut->src_alu = P16[pi % 16]; dut->src_alue = P16[(pi + 5) % 16]; dut->src_au = P32[(pi + 3) % 16]; dut->src_dcr = P16[(pi + 7) % 16]; dut->src_ftu = P16[(pi + 11) % 16]; dut->src_edb = P16[(pi + 13) % 16];
    dut->irc = (uint16_t)((((pi * 5) % 16) << 12) | 0x0123);   // index register rz = IRC[15:12] over all 16 values
    dut->eval();
}
static uint32_t dbg(int idx) { dut->dbg_idx_a = idx; dut->eval(); return dut->dbg_data_a; }
static void check_reads(int w, const std::set<std::string>& A, const Sel& s, const char* when, const std::set<int>* dbw) {
    for (const std::string& a : A) {
        std::string bus, reg; if (!read_reg(a, bus, reg)) continue;
        int idx = reg_of(reg, s); if (idx < 0) continue;
        uint32_t got = bus == "ab" ? dut->rf_ab : dut->rf_db; n_rd++; if (dbw && dbw->count(idx)) n_fwd++;
        CHECK(got == R[idx], "word %03x ird %04x %s: rf_%s %08x expected %08x (%s = %s)", w, cur_ird, when, bus.c_str(), got, R[idx], reg.c_str(), IDXNAME[idx]);
        if (bus == "db") CHECK(dut->rf_db_sp == (idx == s.sp), "word %03x ird %04x %s: rf_db_sp %d (register read %s, SP = %s)", w, cur_ird, when, dut->rf_db_sp, IDXNAME[idx], IDXNAME[s.sp]);
    }
    CHECK(dut->pc_q == R[17], "word %03x %s: pc_q %08x expected %08x", w, when, dut->pc_q, R[17]);
}
static const int STATE_IDX[19] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 18, 31, 17};
static void check_state(int w) {
    for (int i = 0; i < 8; i++) { dut->dbg_idx_d = i; dut->eval(); CHECK(dut->dbg_data_d == R[i], "word %03x: %s = %08x expected %08x", w, IDXNAME[i], dut->dbg_data_d, R[i]); }
    for (int k = 8; k < 18; k++) { int i = STATE_IDX[k]; uint32_t v = dbg(i); CHECK(v == R[i], "word %03x: %s = %08x expected %08x", w, IDXNAME[i], v, R[i]); }
}
static bool is_dispatch() { return ((dut->uw >> 1) & 1) == 0 && ((dut->uw >> 2) & 3) == 1; }
// one microcycle on the current word: capture clock (en = 0: the nano ROM reads the index of the next word one clock before en),
// pattern sources, model, en edge, then en+1 (reads of the next word: forwarding of the queued DB write) and en+2
// (reads from the MLABs) with the state from the debug ports
static void step(bool en, int next_ird) {
    dut->en = 0; dut->eval(); tick();
    set_sources(pi_g++);
    int w = dut->upc; std::set<std::string> A = nd_atoms(&dut->dec[0]);
    bool new_ird = A.count("ird<-(ir)") > 0;
    pointers(A); Sel s = select();
    uint32_t Rn[32]; memcpy(Rn, R, sizeof(R));
    std::map<std::string, std::pair<std::string, char>> who; std::set<int> dbw; std::string wlog;
    // writes: the AB port and the PC port, then the DB port; the same half written by the two buses with different values is not expected
    // (the artificial cases rx = ry with both written are excluded upstream: see the main run) and counts as an error
    if (en) for (int pass = 0; pass < 2; pass++) for (const std::string& a : A) for (const WRule& r : WRULES) {
        if (a != r.atom) continue;
        bool isdb = r.chain[0] == 'd'; if ((pass == 1) != isdb) continue;
        uint16_t v; int ci = chain(r.chain, v); if (ci < 0) continue;   // section not driven: the register holds
        if (r.half == 'h' && ci > 0) { n_hfb++; auto& hw = hfb_words[w]; hw.first++; hw.second = a + " from section " + std::to_string(ci) + " of chain " + r.chain; }
        int idx = reg_of(r.reg, s); if (idx < 0) continue;
        uint32_t nv = Rn[idx];
        if (r.half == 'h') nv = (nv & 0xffffu) | ((uint32_t)v << 16);
        else if (dut->byte_op && idx < 8) nv = (nv & 0xffffff00u) | (v & 0xffu);   // byte: only the low byte of the data registers
        else nv = (nv & 0xffff0000u) | v;
        std::string key = std::string(IDXNAME[idx]) + (r.half == 'h' ? ".h" : ".l");
        auto it = who.find(key);
        if (it != who.end() && Rn[idx] != nv) {
            if (it->second.second != (isdb ? 'D' : 'A')) n_conf++;
            CHECK(false, "word %03x ird %04x: %s written by '%s' and by '%s' (%s) with different values: unexpected double write", w, cur_ird, key.c_str(), it->second.first.c_str(), a.c_str(), it->second.second == (isdb ? 'D' : 'A') ? "same bus" : "buses AB and DB");
        }
        who[key] = {a, isdb ? 'D' : 'A'}; Rn[idx] = nv; if (isdb) dbw.insert(idx);
        if (n_steps < trace_n) wlog += " " + a + ">" + key;
    }
    // MOVEM encoder (§13): mask with ftu<-ab (AB: section L, fallback D; not driven = 0), `reset pren` clears the lowest bit
    int MASKn = MASK, CURn = CUR, PREDECn = PREDEC;
    if (en) {
        if (A.count("ftu<-ab")) { if (dut->ab_l_drv) MASKn = dut->ab_l; else if (dut->ab_d_drv) MASKn = dut->ab_d; else { MASKn = 0; n_mask0++; } }
        else if (A.count("reset pren")) { int low = 0; for (int i = 15; i >= 0; i--) if ((MASK >> i) & 1) low = i; CURn = low; MASKn = MASK & ~(1 << low); }
        if (new_ird) PREDECn = ((next_ird >> 3) & 7) == 4;   // -(An) mode of the incoming instruction
    }
    if (n_steps < trace_n) { printf("trace: word %03x en=%d ird %04x S=%d%s%s%s | sources ab %04x/%04x/%04x drv %d%d%d db %04x/%04x/%04x drv %d%d%d | writes:%s\n", w, en, cur_ird, dut->sup, PD.empty() ? "" : (" pd=" + PD).c_str(), PS.empty() ? "" : (" ps=" + PS).c_str(), RZ ? " rz" : "",
                                   dut->ab_h, dut->ab_l, dut->ab_d, dut->ab_h_drv, dut->ab_l_drv, dut->ab_d_drv, dut->db_h, dut->db_l, dut->db_d, dut->db_h_drv, dut->db_l_drv, dut->db_d_drv, wlog.c_str()); }
    dut->ir = (en && new_ird) ? next_ird : cur_ird;
    dut->en = en; dut->eval(); tick(); dut->en = 0;
    if (en) {
        if (memcmp(Rn, R, sizeof(R)) != 0) n_writes++;
        memcpy(R, Rn, sizeof(R)); MASK = MASKn; CUR = CURn; PREDEC = PREDECn;
        if (new_ird) { cur_ird = next_ird; dut->ird = cur_ird; if (!own_pd) PD = ""; if (!own_ps) PS = ""; }   // dispatch: pointers back to default
    } else n_hold++;
    n_steps++;
    dut->eval(); tick(); dut->eval();   // en+1: next word at the output, reads with the possible forwarding of the queued DB write
    int w2 = dut->upc; std::set<std::string> A2 = nd_atoms(&dut->dec[0]);
    pointers(A2); Sel s2 = select();
    check_reads(w2, A2, s2, "en+1", &dbw);
    CHECK(dut->rp_reg == mvm_reg(), "word %03x: encoder rp_reg %d expected %d (mask %04x current %d predec %d)", w2, dut->rp_reg, mvm_reg(), MASK, CUR, PREDEC);
    CHECK(dut->mvm_end == (MASK == 0), "word %03x: mvm_end %d expected %d (mask %04x)", w2, dut->mvm_end, MASK == 0, MASK);
    dut->eval(); tick(); dut->eval();   // en+2: the DB write is in the MLABs
    check_reads(w2, A2, s2, "en+2", nullptr);
    check_state(w2);
}
static int lut_mode = 0; static const int IRD_WALK = 0x4e71;   // NOP: instruction loaded along the positioning path
static bool goto_word(int W, int irdW) {
    n_pos++;
    if (lut_mode) { dut->a0s_sub = 1; dut->a0s = W; step(true, irdW); dut->a0s_sub = 0; }
    else {
        dut->a0s_sub = 1; dut->a0s = 0; step(true, IRD_WALK); dut->a0s_sub = 0;   // override to word 000 (allowed in the core)
        if (dut->upc != 0) { CHECK(false, "positioning: upc %03x after the override to 000", dut->upc); return false; }
        for (int i = 0; i < 32 && !is_dispatch(); i++) step(true, IRD_WALK);   // reset path up to the dispatch
        if (!is_dispatch()) { CHECK(false, "positioning: no dispatch within 32 words from the reset"); return false; }
        dut->a1 = W; dut->a0_sub = 0; step(true, irdW); dut->a1 = 0;
    }
    CHECK(dut->upc == W, "positioning: upc %03x expected %03x", dut->upc, W);
    return dut->upc == W;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* pkg = "../rtl/cpu_si/m68k_nano_dec_pkg.sv";
    for (int i = 1; i < argc; i++) { if (!strncmp(argv[i], "+pkg=", 5)) pkg = argv[i] + 5; if (!strncmp(argv[i], "+lut=", 5)) lut_mode = atoi(argv[i] + 5); if (!strncmp(argv[i], "+trace=", 7)) trace_n = atol(argv[i] + 7); }
    if (!nd_load_pkg(pkg)) { printf("RESULT: FAIL\n"); return 1; }
    std::set<std::string> known; for (const NdField& f : ND_FIELDS) for (auto& kv : f.atoms) known.insert(kv.second);
    for (const WRule& r : WRULES) CHECK(known.count(r.atom) > 0, "the atom '%s' of the rules does not exist in the pkg", r.atom);
    for (const char* a : {"ird<-(ir)", "rxsrc=rz", "rp_dst=dt", "rp_dst=rp", "rp_src=py", "ftu<-ab", "reset pren"}) CHECK(known.count(a) > 0, "the atom '%s' does not exist in the pkg", a);
    if (fails) { printf("RESULT: FAIL\n"); return 1; }
    // instruction classes (MC68000UM manual, formats): RX = IR[11:9], RY = IR[2:0]; mode 0 = all 64 pairs, 1 = RX,RY in {0,7} and (2,5), 2 = RY only (RX is in the opcode)
    struct Cls { const char* name; int base; int mode; bool byte; };
    static const Cls CLS[] = {{"MOVE.L Dy,Dx", 0x2000, 0, false}, {"MOVEA.L Dy,Ax", 0x2040, 1, false}, {"MOVE.L (Ay),Dx", 0x2010, 1, false},
                              {"EXG Ax,Ay", 0xc148, 1, false}, {"EXG Dx,Ay", 0xc188, 1, false}, {"DBcc Dy", 0x50c8, 1, false}, {"MOVE.L d16(PC),Dx", 0x203a, 1, false},
                              {"LEA (Ay),Ax", 0x41d0, 1, false}, {"ADDA.L Dy,Ax", 0xd1c0, 1, false}, {"ASL.W Dx,Dy", 0xe160, 1, false}, {"MOVE.B Dy,Dx", 0x1000, 1, true},
                              {"MOVEM.W list,-(Ay)", 0x48a0, 2, false}, {"MOVEM.W (Ay)+,list", 0x4c98, 2, false}};
    static const int PAIR1[5][2] = {{0, 0}, {0, 7}, {7, 0}, {7, 7}, {2, 5}}, RY2[3] = {0, 7, 5};
    dut = new Vm68k_regfile_top;
    dut->clk = 0; dut->rst_n = 0; dut->en = 1; dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->a0_sub = 0; dut->a0 = 0; dut->a0s_sub = 0; dut->a0s = 0; dut->c1c0 = 0;
    dut->ird = 0; dut->ir = 0; dut->sup = 1; dut->sup_next = 1; dut->byte_op = 0; dut->dbg_idx_d = 0; dut->dbg_idx_a = 8; set_sources(0);
    tick(); tick(); dut->rst_n = 1; dut->eval();
    // initial model state from the DUT: the register file arrays have no reset (like the die); PC at zero from the reset
    memset(R, 0, sizeof(R)); for (int k = 0; k < 8; k++) { dut->dbg_idx_d = k; dut->eval(); R[k] = dut->dbg_data_d; } for (int k = 8; k < 18; k++) R[STATE_IDX[k]] = dbg(STATE_IDX[k]); R[17] = dut->pc_q;
    cur_ird = 0; PD = ""; PS = ""; MASK = 0; CUR = 0; PREDEC = 0;
    // survey: atoms of every word and words that touch the register file
    std::vector<std::set<std::string>> atomsW(1024); std::vector<bool> isword(1024, false), hasreg(1024, false); std::vector<int> succ(1024, 0); int n_words = 0, n_regw = 0;
    for (int W = 0; W < 1024; W++) {
        if (!goto_word(W, IRD_WALK)) { printf("RESULT: FAIL\n"); return 1; }
        atomsW[W] = nd_atoms(&dut->dec[0]); isword[W] = dut->uw != 0 || !atomsW[W].empty(); hasreg[W] = regword(atomsW[W]);
        if (isword[W]) n_words++; if (hasreg[W]) n_regw++;
        step(true, IRD_WALK); succ[W] = dut->upc;   // successor of the word (c1c0 = 0, a1/a2/a3 = 0, as in the run): it is the word that the following positioning executes when leaving W
        if (fails > 20) break;
    }
    // main run
    long it = 0; const int IRD_PLAIN = 0x2000 | (1 << 9) | 2;   // MOVE.L D2,D1 for the words without register file atoms
    for (int W = 0; W < 1024 && fails <= 20; W++) {
        if (!isword[W]) continue;
        if (!hasreg[W]) {
            for (int s = 0; s < 2; s++) {
                dut->sup = s; dut->sup_next = s; dut->byte_op = 0;
                if (!goto_word(W, IRD_PLAIN)) { printf("RESULT: FAIL\n"); return 1; }
                step(false, IRD_WALK); step(true, IRD_WALK); it++;
            }
            continue;
        }
        for (const Cls& c : CLS) {
            int np = c.mode == 0 ? 64 : (c.mode == 1 ? 5 : 3);
            for (int p = 0; p < np; p++) {
                int RX = c.mode == 0 ? (p >> 3) : (c.mode == 1 ? PAIR1[p][0] : 0), RY = c.mode == 0 ? (p & 7) : (c.mode == 1 ? PAIR1[p][1] : RY2[p]);
                int ird = c.mode == 2 ? (c.base | RY) : (c.base | (RX << 9) | RY);
                // artificial case: the word (or its successor, which the following positioning executes when leaving W with the same
                // instruction) writes both rx and ry and in the instruction of the class the two names fall on the same register. In the processor
                // either the word does not belong to that instruction (e.g. MOVE.L D0,D0 on the second word of EXG, 231 exge2), or the two values
                // coincide by construction (EXG Dx,Dx: AT and AU both hold Dx, App. H exge1/exge2); here the AU comes from a pattern and the values
                // differ: the pair is skipped, and every remaining double write with different values is an error
                if (collides(atomsW[W], ird) || collides(atomsW[succ[W]], ird)) { n_skip++; continue; }
                dut->sup = (it & 1) ? 1 : 0; dut->sup_next = dut->sup; dut->byte_op = c.byte ? 1 : 0;
                if (!goto_word(W, ird)) { printf("RESULT: FAIL\n"); return 1; }
                step(false, IRD_WALK);   // hold: nothing changes
                step(true, IRD_WALK);    // execution of word W with the instruction of the class
                it++;
            }
        }
    }
    printf("REGFILE: words %d (%d touch the register file, %d do not) | instruction classes %d (one on all 64 RX/RY pairs, 10 on 5 pairs, 2 on RY) x S | positionings %ld, skipped pairs %ld (rx = ry with both written: in the processor distinct registers or equal values) | microcycles %ld (hold %ld, with writes %ld) | reads verified %ld (at en+1 and en+2; with forwarding %ld) | AB/DB double writes %ld, L/D fallbacks on high-half atoms %ld, undriven masks %ld | check %d | FAIL %d\n",
           n_words, n_regw, n_words - n_regw, (int)(sizeof(CLS) / sizeof(CLS[0])), n_pos, n_skip, n_steps, n_hold, n_writes, n_rd, n_fwd, n_conf, n_hfb, n_mask0, checks, fails);
    for (auto& kv : hfb_words) printf("fallback on high-half atom: word %03x, %d times: %s\n", kv.first, kv.second.first, kv.second.second.c_str());
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
