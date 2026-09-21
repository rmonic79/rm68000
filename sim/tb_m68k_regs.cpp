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

// tb_m68k_regs.cpp - verification of the bus-side registers AT/AOB/DOB/DBIN (rtl/cpu_si/m68k_regs_bd.sv) inside m68k_regs_top (control
// store + decoder + fabric + registers), core configuration (ORG_OVR_LUT = 0). The testbench is independent of the RTL and of the
// generators: the commands of the word are read from the `dec` vector of the REAL control store decoded with the pkg fields
// (m68k_ndec_fields.h, +pkg=); the bus sections and the "driven" flags are observed from the real fabric (verified by the bussec
// testbench); the model applies the rules of §6 of docs/silicon-datapath.md: every register part loads at en ONLY if the
// source section is driven (fallbacks L<->D, H->L->D for ath), otherwise it holds; AU/ALU/EDB ports always; dob<-ab reads the
// AB word (D, then L, then H); at<-*e takes AB if an AB section is driven, otherwise DB. No generated file is used.
// For EVERY microcode word (positioning: a0s override to 000, reset path up to the dispatch, A1 = word; every
// word traversed is in turn verified) with 6 value patterns for the 13 sources: hold with en = 0 (nothing changes), then
// execution: before the edge aob_n/dob_n/dbin_n = next state, after the edge at/aob/dob/dbin = model state. Two atoms
// that load the same part with different values are a conflict (FAIL). Outcome: RESULT: PASS/FAIL, exit code 0/1.
#include "Vm68k_regs_top.h"
#include "verilated.h"
#include "m68k_ndec_fields.h"
#include <cstring>
static Vm68k_regs_top* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
static void tick() { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); dut->clk = 0; dut->eval(); }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// --- rules of §6: atom -> (part, chain of sources: the first driven one loads; ports always driven)
struct Load { const char* part; const char* chain; };
struct RRule { const char* atom; Load l[2]; };
static const RRule RULES[] = {
    {"at<-ab",    {{"at_h", "ab_h"}, {"at_l", "ab_l|ab_d"}}},   {"at<-abe",   {{"at_h", "ab_h"}, {"at_l", "ab_l|ab_d"}}},
    {"at<-db",    {{"at_h", "db_h"}, {"at_l", "db_l|db_d"}}},   {"at<-dbe",   {{"at_h", "db_h"}, {"at_l", "db_l|db_d"}}},
    {"ath<-ab",   {{"at_h", "ab_h|ab_l|ab_d"}, {nullptr, nullptr}}}, {"ath<-db", {{"at_h", "db_h|db_l|db_d"}, {nullptr, nullptr}}},
    {"atl<-ab",   {{"at_l", "ab_l|ab_d"}, {nullptr, nullptr}}}, {"atl<-db",   {{"at_l", "db_l|db_d"}, {nullptr, nullptr}}},
    {"atl<-abl",  {{"at_l", "ab_l"}, {nullptr, nullptr}}},       {"atl<-dbl",  {{"at_l", "db_l"}, {nullptr, nullptr}}},
    {"atl<-ab*",  {{"at_l", "ab_d|ab_l"}, {nullptr, nullptr}}}, {"atl<-db*",  {{"at_l", "db_d|db_l"}, {nullptr, nullptr}}},
    {"aob<-ab",   {{"aob_h", "ab_h"}, {"aob_l", "ab_l|ab_d"}}}, {"aob<-db",   {{"aob_h", "db_h"}, {"aob_l", "db_l|db_d"}}},
    {"aob<-abh",  {{"aob_h", "ab_h"}, {nullptr, nullptr}}},      {"aob<-dbh",  {{"aob_h", "db_h"}, {nullptr, nullptr}}},
    {"aob<-abl",  {{"aob_l", "ab_l"}, {nullptr, nullptr}}},      {"aob<-dbl",  {{"aob_l", "db_l"}, {nullptr, nullptr}}},
    {"aob<-au",   {{"aob_h", "au_h"}, {"aob_l", "au_l"}}},
    {"dob<-ab",   {{"dob_hi", "ab_d|ab_l|ab_h"}, {"dob_lo", "ab_d|ab_l|ab_h"}}}, {"dob<-ab*", {{"dob_hi", "ab_d|ab_l|ab_h"}, {"dob_lo", "ab_d|ab_l|ab_h"}}},
    {"dobh<-ab",  {{"dob_hi", "ab_d|ab_l|ab_h"}, {nullptr, nullptr}}}, {"dobl<-ab", {{"dob_lo", "ab_d|ab_l|ab_h"}, {nullptr, nullptr}}}, {"dobl<-ab*", {{"dob_lo", "ab_d|ab_l|ab_h"}, {nullptr, nullptr}}},
    {"dob<-dbd",  {{"dob_hi", "db_d|db_l"}, {"dob_lo", "db_d|db_l"}}},
    {"dob<-alu",  {{"dob_hi", "alu"}, {"dob_lo", "alu"}}},
    {"dbin<-edb", {{"dbin_hi", "edb"}, {"dbin_lo", "edb"}}},     {"dbinh<-edb", {{"dbin_hi", "edb"}, {nullptr, nullptr}}}, {"dbinl<-edb", {{"dbin_lo", "edb"}, {nullptr, nullptr}}},
};
// --- model state and sources
struct St { uint16_t at_h = 0, at_l = 0, aob_h = 0, aob_l = 0; uint8_t dob_hi = 0, dob_lo = 0, dbin_hi = 0, dbin_lo = 0; };
static St S;
static const char* SRC[13] = {"alu", "alue", "aob", "at", "ath", "atl", "au", "dbin", "dcr", "ftu", "rf_ab", "rf_db", "edb"};
static std::map<std::string, uint32_t> VAL;
static const uint32_t P32[16] = {0x00000000u, 0xffffffffu, 0x80000000u, 0x7fffffffu, 0x00008000u, 0x0000ffffu, 0xffff0000u, 0x12345678u, 0x00000001u, 0x55555555u, 0xaaaaaaaau, 0x0f0f0f0fu, 0x00010000u, 0x80008000u, 0xfffefffeu, 0x7fff8000u};
static const uint16_t P16[16] = {0x0000, 0xffff, 0x8000, 0x7fff, 0x0001, 0x00ff, 0xff00, 0x5555, 0xaaaa, 0x0f0f, 0x1234, 0x8001, 0x7ffe, 0x0100, 0xfffe, 0x4321};
static void set_sources(long pi) {
    for (int k = 0; k < 13; k++) {
        bool w32 = !strcmp(SRC[k], "aob") || !strcmp(SRC[k], "at") || !strcmp(SRC[k], "au") || !strcmp(SRC[k], "rf_ab") || !strcmp(SRC[k], "rf_db");
        VAL[SRC[k]] = w32 ? P32[(pi + 5 * k) % 16] : P16[(pi + 5 * k) % 16];
    }
    dut->src_alu = VAL["alu"]; dut->src_alue = VAL["alue"]; dut->src_aob = VAL["aob"]; dut->src_at = VAL["at"]; dut->src_ath = VAL["ath"]; dut->src_atl = VAL["atl"];
    dut->src_au = VAL["au"]; dut->src_dbin = VAL["dbin"]; dut->src_dcr = VAL["dcr"]; dut->src_ftu = VAL["ftu"]; dut->src_rf_ab = VAL["rf_ab"]; dut->src_rf_db = VAL["rf_db"]; dut->src_edb = VAL["edb"];
    dut->eval();
}
// 16-bit value of a candidate of the chain, if driven (ports always)
static bool cand(const std::string& c, uint16_t& v) {
    if (c == "ab_h") { v = dut->ab_h; return dut->ab_h_drv; } if (c == "ab_l") { v = dut->ab_l; return dut->ab_l_drv; } if (c == "ab_d") { v = dut->ab_d; return dut->ab_d_drv; }
    if (c == "db_h") { v = dut->db_h; return dut->db_h_drv; } if (c == "db_l") { v = dut->db_l; return dut->db_l_drv; } if (c == "db_d") { v = dut->db_d; return dut->db_d_drv; }
    if (c == "au_h") { v = (uint16_t)(dut->src_au >> 16); return true; } if (c == "au_l") { v = (uint16_t)dut->src_au; return true; }
    if (c == "alu") { v = dut->src_alu; return true; } if (c == "edb") { v = dut->src_edb; return true; }
    printf("FAIL: unknown candidate '%s'\n", c.c_str()); printf("RESULT: FAIL\n"); exit(1);
}
static bool chain_value(const char* chain, uint16_t& v) {
    std::string s(chain); size_t p = 0;
    while (p <= s.size()) { size_t b = s.find('|', p); std::string c = s.substr(p, b == std::string::npos ? std::string::npos : b - p); if (cand(c, v)) return true; if (b == std::string::npos) break; p = b + 1; }
    return false;
}
struct Next { St s; std::map<std::string, std::string> who; bool conflict = false; std::string cstr; };
static void set_part(Next& n, const std::string& part, uint16_t v16, const std::string& atom) {
    uint16_t v = v16; uint8_t hi = (uint8_t)(v16 >> 8), lo = (uint8_t)v16;
    auto chk = [&](const std::string& p, uint32_t nv, uint32_t cur) { auto it = n.who.find(p); if (it != n.who.end() && nv != cur) { n.conflict = true; n.cstr += p + ": " + atom + " versus " + it->second + "; "; } n.who[p] = atom; };
    if (part == "at_h") { chk(part, v, n.s.at_h); n.s.at_h = v; } else if (part == "at_l") { chk(part, v, n.s.at_l); n.s.at_l = v; }
    else if (part == "aob_h") { chk(part, v, n.s.aob_h); n.s.aob_h = v; } else if (part == "aob_l") { chk(part, v, n.s.aob_l); n.s.aob_l = v; }
    else if (part == "dob_hi") { chk(part, hi, n.s.dob_hi); n.s.dob_hi = hi; } else if (part == "dob_lo") { chk(part, lo, n.s.dob_lo); n.s.dob_lo = lo; }
    else if (part == "dbin_hi") { chk(part, hi, n.s.dbin_hi); n.s.dbin_hi = hi; } else if (part == "dbin_lo") { chk(part, lo, n.s.dbin_lo); n.s.dbin_lo = lo; }
}
static Next model(const std::set<std::string>& A) {
    Next n; n.s = S;
    for (const std::string& a : A) {
        if (a == "at<-*e") {   // residual bus: AB if an AB section is driven, otherwise DB (§6)
            bool ab = dut->ab_h_drv || dut->ab_l_drv || dut->ab_d_drv; uint16_t v;
            if (chain_value(ab ? "ab_h" : "db_h", v)) set_part(n, "at_h", v, a);
            if (chain_value(ab ? "ab_l|ab_d" : "db_l|db_d", v)) set_part(n, "at_l", v, a);
            continue;
        }
        for (const RRule& r : RULES) {
            if (a != r.atom) continue;
            for (int k = 0; k < 2; k++) { if (!r.l[k].part) break; uint16_t v; if (chain_value(r.l[k].chain, v)) set_part(n, r.l[k].part, v, a); }
        }
    }
    return n;
}
static uint32_t at_of(const St& s) { return ((uint32_t)s.at_h << 16) | s.at_l; }
static uint32_t aob_of(const St& s) { return ((uint32_t)s.aob_h << 16) | s.aob_l; }
static uint16_t dob_of(const St& s) { return (uint16_t)((s.dob_hi << 8) | s.dob_lo); }
static uint16_t dbin_of(const St& s) { return (uint16_t)((s.dbin_hi << 8) | s.dbin_lo); }
static long n_steps = 0, n_loads = 0, n_holds = 0, n_conf = 0, pi_g = 0;
// one microcycle on the current word (already settled): pattern sources, model, checks before and after the edge
static void step(bool en) {
    dut->en = 0; dut->eval(); tick();   // capture clock: the nano ROM reads the index of the next word (A1 too) at the clock before en
    set_sources(pi_g++);
    int w = dut->upc; std::set<std::string> A = nd_atoms(&dut->dec[0]);
    Next n = model(A);
    if (n.conflict) { n_conf++; CHECK(false, "word %03x: the same part loaded by two atoms with different values: %s", w, n.cstr.c_str()); }
    CHECK(dut->aob_n == aob_of(n.s), "word %03x: aob_n %08x expected %08x", w, dut->aob_n, aob_of(n.s));
    CHECK(dut->dob_n == dob_of(n.s), "word %03x: dob_n %04x expected %04x", w, dut->dob_n, dob_of(n.s));
    CHECK(dut->dbin_n == dbin_of(n.s), "word %03x: dbin_n %04x expected %04x", w, dut->dbin_n, dbin_of(n.s));
    dut->en = en; dut->eval(); tick(); dut->en = 0; dut->eval(); tick(); dut->eval();   // edge (loads if en), then one settling clock
    if (en) { if (memcmp(&n.s, &S, sizeof(St))) n_loads++; else n_holds++; S = n.s; }
    n_steps++;
    CHECK(dut->at == at_of(S), "word %03x en=%d: at %08x expected %08x", w, en, dut->at, at_of(S));
    CHECK(dut->aob == aob_of(S), "word %03x en=%d: aob %08x expected %08x", w, en, dut->aob, aob_of(S));
    CHECK(dut->dob == dob_of(S), "word %03x en=%d: dob %04x expected %04x", w, en, dut->dob, dob_of(S));
    CHECK(dut->dbin == dbin_of(S), "word %03x en=%d: dbin %04x expected %04x", w, en, dut->dbin, dbin_of(S));
}
static bool is_dispatch() { return ((dut->uw >> 1) & 1) == 0 && ((dut->uw >> 2) & 3) == 1; }
static int lut_mode = 0; static long n_pos = 0;
static bool goto_word(int W) {
    n_pos++;
    if (lut_mode) { dut->a0s_sub = 1; dut->a0s = W; step(true); dut->a0s_sub = 0; }
    else {
        dut->a0s_sub = 1; dut->a0s = 0; step(true); dut->a0s_sub = 0;
        if (dut->upc != 0) { CHECK(false, "positioning: upc %03x after the override to 000", dut->upc); return false; }
        for (int i = 0; i < 32 && !is_dispatch(); i++) step(true);
        if (!is_dispatch()) { CHECK(false, "positioning: no dispatch within 32 words from the reset"); return false; }
        dut->a1 = W; dut->a0_sub = 0; step(true); dut->a1 = 0;
    }
    CHECK(dut->upc == W, "positioning: upc %03x expected %03x", dut->upc, W);
    return dut->upc == W;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* pkg = "../rtl/cpu_si/m68k_nano_dec_pkg.sv";
    for (int i = 1; i < argc; i++) { if (!strncmp(argv[i], "+pkg=", 5)) pkg = argv[i] + 5; if (!strncmp(argv[i], "+lut=", 5)) lut_mode = atoi(argv[i] + 5); }
    if (!nd_load_pkg(pkg)) { printf("RESULT: FAIL\n"); return 1; }
    if (ND_W_PKG > 7 * 32) { printf("FAIL: NDEC_W %d beyond the 7 words of the testbench\n", ND_W_PKG); printf("RESULT: FAIL\n"); return 1; }
    // every atom of the rules must exist in the pkg (otherwise the rule is written for a wrong name)
    std::set<std::string> known; for (const NdField& f : ND_FIELDS) for (auto& kv : f.atoms) known.insert(kv.second);
    for (const RRule& r : RULES) CHECK(known.count(r.atom) > 0, "the atom '%s' of the rules does not exist in the pkg", r.atom);
    if (fails) { printf("RESULT: FAIL\n"); return 1; }
    dut = new Vm68k_regs_top;
    dut->clk = 0; dut->rst_n = 0; dut->en = 1; dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->a0_sub = 0; dut->a0 = 0; dut->a0s_sub = 0; dut->a0s = 0; dut->c1c0 = 0;
    set_sources(0);
    tick(); tick(); dut->rst_n = 1; dut->eval(); S = St();
    int n_words = 0, n_empty = 0; const int NP = 6;
    for (int W = 0; W < 1024; W++) {
        bool word = false;
        for (int p = 0; p < NP; p++) {
            if (!goto_word(W)) { printf("RESULT: FAIL\n"); return 1; }
            if (p == 0) { word = dut->uw != 0 || !nd_atoms(&dut->dec[0]).empty(); if (word) n_words++; else n_empty++; }
            step(false);   // hold: with en = 0 no register changes
            step(true);    // execution of word W
            if (!word) break;
        }
        if (fails > 20) break;
    }
    printf("registers AT/AOB/DOB/DBIN: words %d + empty addresses %d | positionings %ld (%s) | microcycles %ld, with loads %ld, without %ld, conflicts %ld | check %d | FAIL %d\n",
           n_words, n_empty, n_pos, lut_mode ? "a0s override" : "reset path", n_steps, n_loads, n_holds, n_conf, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
