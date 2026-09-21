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

// tb_m68k_bus_sec.cpp - verification of the AB/DB bus section fabric (rtl/cpu_si/m68k_bus_sec.sv) inside m68k_bus_top
// (control store + decoder + fabric), core configuration (ORG_OVR_LUT = 0). The testbench is independent of the RTL and
// of the generators: the commands of the word are read from the `dec` vector of the REAL control store, decoded with the
// pkg fields (m68k_ndec_fields.h, +pkg=); the model applies the rule of §5 of docs/silicon-datapath.md to every
// "bus<-source" atom (driven sections and value: 32-bit sources on H+L, 16-bit HIGH/LOW/DATA halves on their own
// section, suffixes h/l/d/e/*), with the optional documented deviations (+dev=: "atom<TAB>H<TAB>L<TAB>D<TAB>source",
// values "src:H" "src:L" "src:L6" "sext:L" or "-").
// For EVERY microcode word (placement: a0s override to 000, reset path up to the dispatch, A1 = word; with +lut=1
// direct override): 16 value patterns for the 12 sources (zeros, ones, boundaries, alternating bits, walking one) and,
// for each, the six sections, the six "driven" flags (undriven section = 0) and, in the words with dob<-ab,
// ab_dob/ab_dob_drv (DOB reads the AB word: section D, then L, then H, §6). Two atoms that drive the same section
// with different values are a conflict (FAIL). No generated file is used. Result: RESULT: PASS/FAIL, exit code 0/1.
#include "Vm68k_bus_top.h"
#include "verilated.h"
#include "m68k_ndec_fields.h"
#include <cstring>
static Vm68k_bus_top* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
static void tick() { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); dut->clk = 0; dut->eval(); }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// --- rule of §5: "bus<-source" atom -> specifications of the sections H, L, D ("" = undriven)
struct Spec { std::string h, l, d; };
static const std::set<std::string> S32 = {"at", "pc", "rx", "rxa", "ry", "rya", "sp", "au", "aob"}, SH = {"ath", "pch", "rxh", "ryh", "sph"},
                                   SL = {"atl", "pcl", "rxl", "ryl", "spl", "ftu"}, SD = {"alu", "alue", "dcr", "dbin", "rxd", "rxdl", "ryd", "rydl"},
                                   REG = {"pc", "pch", "pcl", "rx", "rxa", "rxh", "rxl", "rxd", "rxdl", "ry", "rya", "ryh", "ryl", "ryd", "rydl", "sp", "sph", "spl"};
static bool split_atom(const std::string& atom, std::string& base, std::string& suf, std::string& src) {
    size_t p = atom.find("<-"); if (p == std::string::npos) return false;
    std::string bus = atom.substr(0, p); src = atom.substr(p + 2);
    if (src.size() >= 2 && src.front() == '(' && src.back() == ')') src = src.substr(1, src.size() - 2);
    if (bus.size() < 2 || bus.size() > 3) return false;
    base = bus.substr(0, 2); suf = bus.substr(2);
    return base == "ab" || base == "db";
}
static bool doc_rule(const std::string& atom, Spec& sp) {
    std::string base, suf, src; if (!split_atom(atom, base, suf, src)) return false;
    std::string n = REG.count(src) ? "rf_" + base : src;                                   // the register file arrives on the bus port
    std::string v16 = REG.count(src) ? n + ":" + ((src == "pch" || src == "rxh" || src == "ryh" || src == "sph") ? "H" : "L") : n + ":L";   // natural half
    sp = Spec();
    if (suf == "") { if (S32.count(src)) { sp.h = n + ":H"; sp.l = n + ":L"; } else if (SH.count(src)) sp.h = v16; else if (SL.count(src)) sp.l = v16; else if (SD.count(src)) sp.d = v16; else return false; }
    else if (suf == "h") sp.h = S32.count(src) ? n + ":H" : v16;
    else if (suf == "l") sp.l = S32.count(src) ? n + ":L" : v16;
    else if (suf == "d") sp.d = S32.count(src) ? n + ":L" : v16;
    else if (suf == "e") { sp.l = S32.count(src) ? n + ":L" : v16; sp.h = "sext:L"; }
    else if (suf == "*") { if (S32.count(src)) { sp.h = n + ":H"; sp.l = n + ":L"; sp.d = n + ":L"; } else if (SH.count(src)) sp.h = v16; else if (SL.count(src)) sp.l = v16; else sp.d = v16; }
    else return false;
    return true;
}
static std::map<std::string, Spec> DEV; static int ndev = 0;
static bool load_dev(const char* path) {
    std::ifstream f(path); if (!f) { printf("FAIL: cannot open %s\n", path); return false; }
    std::string line; bool first = true;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (first && line.size() >= 3 && (unsigned char)line[0] == 0xEF) line.erase(0, 3);
        first = false;
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> fl; { std::istringstream is(line); std::string c; while (std::getline(is, c, '\t')) fl.push_back(c); }
        if (fl.size() < 4) { printf("FAIL: incomplete deviation line: '%s'\n", line.c_str()); return false; }
        Spec sp; sp.h = nd_trim(fl[1]) == "-" ? "" : nd_trim(fl[1]); sp.l = nd_trim(fl[2]) == "-" ? "" : nd_trim(fl[2]); sp.d = nd_trim(fl[3]) == "-" ? "" : nd_trim(fl[3]);
        std::string b, s, src; if (!split_atom(nd_trim(fl[0]), b, s, src)) { printf("FAIL: deviation for a non-bus atom: '%s'\n", line.c_str()); return false; }
        DEV[nd_trim(fl[0])] = sp; ndev++;
        printf("deviation: %s = H %s | L %s | D %s (%s)\n", fl[0].c_str(), sp.h.empty() ? "-" : sp.h.c_str(), sp.l.empty() ? "-" : sp.l.c_str(), sp.d.empty() ? "-" : sp.d.c_str(), fl.size() > 4 ? fl[4].c_str() : "no source");
    }
    return true;
}
static bool rule(const std::string& atom, Spec& sp) { auto it = DEV.find(atom); if (it != DEV.end()) { sp = it->second; return true; } return doc_rule(atom, sp); }
// --- source values and model
static const char* SRC[12] = {"alu", "alue", "aob", "at", "ath", "atl", "au", "dbin", "dcr", "ftu", "rf_ab", "rf_db"};
static std::map<std::string, uint32_t> VAL;
static uint16_t spec_value(const std::string& spec, uint16_t lsec) {
    if (spec == "sext:L") return (lsec & 0x8000) ? 0xffff : 0;
    size_t c = spec.find(':'); std::string s = spec.substr(0, c), hl = spec.substr(c + 1);
    if (!VAL.count(s)) { printf("FAIL: unknown source '%s' in specification '%s'\n", s.c_str(), spec.c_str()); printf("RESULT: FAIL\n"); exit(1); }
    uint32_t v = VAL[s];
    if (hl == "H") return (v >> 16) & 0xffff;
    if (hl == "L6") return v & 0x3f;
    return v & 0xffff;
}
struct Exp { uint16_t v[6] = {0, 0, 0, 0, 0, 0}; bool d[6] = {false, false, false, false, false, false}; std::string who[6]; bool conflict = false; std::string cstr; };
static Exp model(const std::set<std::string>& A) {
    Exp e;
    for (const std::string& a : A) {
        Spec sp; if (!rule(a, sp)) continue;   // not a "bus<-source" atom (readers, AU, ...)
        int b = a[0] == 'a' ? 0 : 3;             // ab -> 0..2, db -> 3..5, order H L D
        const std::string* s3[3] = {&sp.h, &sp.l, &sp.d};
        uint16_t lsec = sp.l.empty() ? 0 : spec_value(sp.l, 0);
        for (int k = 0; k < 3; k++) {
            if (s3[k]->empty()) continue;
            uint16_t v = spec_value(*s3[k], lsec);
            if (e.d[b + k] && e.v[b + k] != v) { e.conflict = true; e.cstr += a + " vs " + e.who[b + k] + "; "; }
            e.v[b + k] = v; e.d[b + k] = true; e.who[b + k] = a;
        }
    }
    return e;
}
static const uint32_t P32[16] = {0x00000000u, 0xffffffffu, 0x80000000u, 0x7fffffffu, 0x00008000u, 0x0000ffffu, 0xffff0000u, 0x12345678u, 0x00000001u, 0x55555555u, 0xaaaaaaaau, 0x0f0f0f0fu, 0x00010000u, 0x80008000u, 0xfffefffeu, 0x7fff8000u};
static const uint16_t P16[16] = {0x0000, 0xffff, 0x8000, 0x7fff, 0x0001, 0x00ff, 0xff00, 0x5555, 0xaaaa, 0x0f0f, 0x1234, 0x8001, 0x7ffe, 0x0100, 0xfffe, 0x4321};
static void set_sources(int pi) {
    for (int k = 0; k < 12; k++) {
        bool w32 = !strcmp(SRC[k], "aob") || !strcmp(SRC[k], "at") || !strcmp(SRC[k], "au") || !strcmp(SRC[k], "rf_ab") || !strcmp(SRC[k], "rf_db");
        uint32_t v = w32 ? P32[(pi + 5 * k) % 16] : P16[(pi + 5 * k) % 16];
        VAL[SRC[k]] = v;
    }
    dut->src_alu = VAL["alu"]; dut->src_alue = VAL["alue"]; dut->src_aob = VAL["aob"]; dut->src_at = VAL["at"]; dut->src_ath = VAL["ath"]; dut->src_atl = VAL["atl"];
    dut->src_au = VAL["au"]; dut->src_dbin = VAL["dbin"]; dut->src_dcr = VAL["dcr"]; dut->src_ftu = VAL["ftu"]; dut->src_rf_ab = VAL["rf_ab"]; dut->src_rf_db = VAL["rf_db"];
    dut->eval();
}
// --- placement
static bool is_dispatch() { return ((dut->uw >> 1) & 1) == 0 && ((dut->uw >> 2) & 3) == 1; }
// one word forward at the core's cadence: a capture clock with en = 0 (the nano ROM reads the index of the next word,
// including input A1, on the clock BEFORE en), the en edge, a settling clock (gq/gd and the registered copy of dec)
static void advance() { dut->en = 0; dut->eval(); tick(); dut->en = 1; dut->eval(); tick(); dut->en = 0; dut->eval(); tick(); dut->eval(); }
static int lut_mode = 0; static long n_pos = 0, n_adv = 0; static int trace_n = 0;   // +trace=N: print the first N words
static bool goto_word(int W) {
    n_pos++;
    if (lut_mode) { dut->a0s_sub = 1; dut->a0s = W; advance(); dut->a0s_sub = 0; n_adv++; }
    else {
        dut->a0s_sub = 1; dut->a0s = 0; advance(); dut->a0s_sub = 0; n_adv++;
        if (dut->upc != 0) { CHECK(false, "placement: upc %03x after the override to 000", dut->upc); return false; }
        for (int i = 0; i < 32 && !is_dispatch(); i++) { advance(); n_adv++; }
        if (!is_dispatch()) { CHECK(false, "placement: no dispatch within 32 words of reset"); return false; }
        dut->a1 = W; dut->a0_sub = 0; advance(); dut->a1 = 0; n_adv++;
    }
    CHECK(dut->upc == W, "placement: upc %03x expected %03x", dut->upc, W);
    return dut->upc == W;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* pkg = "../rtl/cpu_si/m68k_nano_dec_pkg.sv"; const char* dev = nullptr;
    for (int i = 1; i < argc; i++) { if (!strncmp(argv[i], "+pkg=", 5)) pkg = argv[i] + 5; if (!strncmp(argv[i], "+dev=", 5)) dev = argv[i] + 5; if (!strncmp(argv[i], "+lut=", 5)) lut_mode = atoi(argv[i] + 5); if (!strncmp(argv[i], "+trace=", 7)) trace_n = atoi(argv[i] + 7); }
    if (!nd_load_pkg(pkg) || (dev && !load_dev(dev))) { printf("RESULT: FAIL\n"); return 1; }
    if (ND_W_PKG > 7 * 32) { printf("FAIL: NDEC_W %d exceeds the 7 words of the testbench\n", ND_W_PKG); printf("RESULT: FAIL\n"); return 1; }
    dut = new Vm68k_bus_top;
    dut->clk = 0; dut->rst_n = 0; dut->en = 1; dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->a0_sub = 0; dut->a0 = 0; dut->a0s_sub = 0; dut->a0s = 0; dut->c1c0 = 0;
    set_sources(0);
    tick(); tick(); dut->rst_n = 1; dut->eval();
    static const char* nm[6] = {"ab_h", "ab_l", "ab_d", "db_h", "db_l", "db_d"};
    int n_words = 0, n_empty = 0, n_dob = 0, n_conflict = 0; long n_vec = 0, n_drv = 0; std::set<std::string> atoms_seen;
    for (int W = 0; W < 1024; W++) {
        if (!goto_word(W)) { printf("RESULT: FAIL\n"); return 1; }
        std::set<std::string> A = nd_atoms(&dut->dec[0]);
        bool empty = dut->uw == 0 && A.empty(); if (empty) n_empty++; else n_words++;
        if (W < trace_n) { printf("trace: word %03x uw %05x dec[0..2] %08x %08x %08x atoms %d:", W, dut->uw, dut->dec[0], dut->dec[1], dut->dec[2], (int)A.size()); for (const std::string& a : A) printf(" %s", a.c_str()); printf(" | drv ab %d%d%d db %d%d%d\n", dut->ab_h_drv, dut->ab_l_drv, dut->ab_d_drv, dut->db_h_drv, dut->db_l_drv, dut->db_d_drv); }
        for (const std::string& a : A) { Spec sp; if (rule(a, sp)) atoms_seen.insert(a); }
        bool has_dob = A.count("dob<-ab") || A.count("dob<-ab*") || A.count("dobh<-ab") || A.count("dobl<-ab") || A.count("dobl<-ab*");
        if (has_dob) n_dob++;
        int np = empty ? 1 : 16;
        for (int pi = 0; pi < np; pi++) {
            set_sources(pi); n_vec++;
            Exp e = model(A);
            if (e.conflict) { n_conflict++; CHECK(false, "word %03x: two sources on the same section with different values: %s", W, e.cstr.c_str()); }
            uint16_t got[6] = {dut->ab_h, dut->ab_l, dut->ab_d, dut->db_h, dut->db_l, dut->db_d};
            bool drv[6] = {(bool)dut->ab_h_drv, (bool)dut->ab_l_drv, (bool)dut->ab_d_drv, (bool)dut->db_h_drv, (bool)dut->db_l_drv, (bool)dut->db_d_drv};
            for (int s = 0; s < 6; s++) {
                CHECK(drv[s] == e.d[s], "word %03x %s: driven %d expected %d (%s)", W, nm[s], drv[s], e.d[s], e.d[s] ? e.who[s].c_str() : "no atom");
                if (e.d[s]) { CHECK(got[s] == e.v[s], "word %03x %s pattern %d: %04x expected %04x (%s)", W, nm[s], pi, got[s], e.v[s], e.who[s].c_str()); if (pi == 0) n_drv++; }
                else CHECK(got[s] == 0, "word %03x %s: undriven but %04x", W, nm[s], got[s]);
            }
            if (has_dob) {
                bool any = e.d[2] || e.d[1] || e.d[0];
                CHECK(any, "word %03x: dob<-ab without any driven AB section", W);
                if (any) { uint16_t exp = e.d[2] ? e.v[2] : e.d[1] ? e.v[1] : e.v[0];
                    CHECK(dut->ab_dob_drv == 1, "word %03x: ab_dob_drv %d expected 1", W, dut->ab_dob_drv);
                    CHECK(dut->ab_dob == exp, "word %03x pattern %d: ab_dob %04x expected %04x", W, pi, dut->ab_dob, exp); }
            }
        }
        if (fails > 20) break;
    }
    printf("bus sections: words %d + empty addresses %d | distinct bus atoms encountered %d | deviations %d | placements %ld (%s), words advanced %ld | patterns %ld, driven sections (per word) %ld, words with dob<-ab %d, conflicts %d | check %d | FAIL %d\n",
           n_words, n_empty, (int)atoms_seen.size(), ndev, n_pos, lut_mode ? "a0s override" : "reset path", n_adv, n_vec, n_drv, n_dob, n_conflict, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
