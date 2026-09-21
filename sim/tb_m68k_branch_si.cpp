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

// tb_m68k_branch_si.cpp - verification of the conditional branch unit (rtl/cpu_si/m68k_branch_si.sv) on ALL 32 CBC codes x
// ALL 131,072 input combinations (irc11, auz, locz, dcr4, mend, ird[11:8], ird[6], alue[1:0], cc) against a model
// INDEPENDENT of the RTL and of the generators: the App. C table read at run time from the literal transcription
// docs/data/us4325121_appC.tsv (+data=<file>; default ../docs/data/us4325121_appC.tsv); the composition of the variables
// from the REMARKS column of App. C, msb first (nz1/nz2 = {n,z}, ms0 = {ird8,alue0}, m01 = {auz,ird8,alue1,alue0},
// nv = {n,v}, enl = {end,ird6}; the others are one bit wide: i11 = irc11, auz, c, z, n, v, ze = locz, d4 = dcr4); the 16
// conditions cc from App. E (manual, Bcc). No generated file is used.
// Documented deviations from the literal data, optional (+dev=<file>): lines "variable<TAB>composition<TAB>source",
// signals msb first among irc11 auz locz dcr4 mend ird8 ird6 alue1 alue0 n z v c nz(=n|z) cond. Without the file the
// model is literal.
// CBC codes absent from App. C: C1C0 expected 00 (convention of m68k_branch_pkg). The internal index {cbc,val} is observed
// (--public-flat-rw): val is checked against the model on every input of the used codes; the table entries (512) that
// are unreachable for the used codes must turn out NEVER selected on all inputs. Shape checks on App. C (FAIL if they
// do not hold): 21 codes, 68 entries, every value < 2^width covered by a single output, composition as wide as the
// values.
// Result: RESULT: PASS/FAIL, exit code 0/1.
#include "Vm68k_branch_si.h"
#include "Vm68k_branch_si___024root.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

struct Ent { std::vector<std::string> pats; int c1c0 = 0; };
struct Code { std::string var; int w = 0; std::vector<Ent> ents; };
static std::map<int, Code> codes;                                  // cbc -> App. C entries
static std::map<std::string, std::vector<std::string>> comp;       // variable -> signals, msb first
static Vm68k_branch_si* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static std::vector<std::string> split(const std::string& s, char d) {
    std::vector<std::string> out; std::string c; std::istringstream is(s);
    while (std::getline(is, c, d)) out.push_back(c);
    return out;
}
static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t");
    return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}
static bool read_lines(const char* path, std::vector<std::string>& lines) {
    std::ifstream f(path); if (!f) { printf("FAIL: cannot open %s\n", path); return false; }
    std::string line; bool first = true;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (first && line.size() >= 3 && (unsigned char)line[0] == 0xEF) line.erase(0, 3);
        first = false;
        if (line.empty() || line[0] == '#') continue;
        lines.push_back(line);
    }
    return true;
}
// App. C: cbc_hex, variable, source, value ("1x,x1"; "0 F": the annotation after the space does not count), C1, C0
static bool load_appC(const char* path) {
    std::vector<std::string> lines; if (!read_lines(path, lines)) return false;
    for (const std::string& line : lines) {
        std::vector<std::string> f = split(line, '\t');
        if (f.size() < 6) { printf("FAIL: App. C line with %d fields: '%s'\n", (int)f.size(), line.c_str()); return false; }
        int cbc = (int)strtol(f[0].c_str(), nullptr, 16);
        std::string v = trim(f[3]); size_t sp = v.find(' '); if (sp != std::string::npos) v = v.substr(0, sp);
        Ent e; e.pats = split(v, ',');
        int w = 0;
        for (const std::string& p : e.pats) {
            if (p.empty() || p.find_first_not_of("01x") != std::string::npos) { printf("FAIL: invalid value '%s' in '%s'\n", p.c_str(), line.c_str()); return false; }
            if (w == 0) w = (int)p.size(); else if (w != (int)p.size()) { printf("FAIL: values of different width in '%s'\n", line.c_str()); return false; }
        }
        std::string c1 = trim(f[4]), c0 = trim(f[5]);
        if (c1.size() != 1 || c0.size() != 1 || (c1[0] != '0' && c1[0] != '1') || (c0[0] != '0' && c0[0] != '1')) { printf("FAIL: invalid C1C0 in '%s'\n", line.c_str()); return false; }
        e.c1c0 = ((c1[0] - '0') << 1) | (c0[0] - '0');
        Code& c = codes[cbc];
        if (c.var.empty()) c.var = trim(f[1]);
        else if (c.var != trim(f[1])) { printf("FAIL: code %02x with two variables (%s, %s)\n", cbc, c.var.c_str(), f[1].c_str()); return false; }
        if (c.w == 0) c.w = w; else if (c.w != w) { printf("FAIL: code %02x with different widths\n", cbc); return false; }
        c.ents.push_back(e);
    }
    return true;
}
static bool pat_match(const std::string& p, int val) {
    int w = (int)p.size();
    for (int i = 0; i < w; i++) { if (p[i] == 'x') continue; if ((p[i] - '0') != ((val >> (w - 1 - i)) & 1)) return false; }
    return true;
}
// C1C0 expected for (cbc, val): -1 no entry, -2 entries with different outputs; code absent from App. C -> 0 (pkg
// convention)
static int lookup(int cbc, int val) {
    auto it = codes.find(cbc); if (it == codes.end()) return 0;
    const Code& c = it->second; if (val >= (1 << c.w)) return -1;
    int out = -1;
    for (const Ent& e : c.ents) for (const std::string& p : e.pats) if (pat_match(p, val)) { if (out >= 0 && out != e.c1c0) return -2; out = e.c1c0; }
    return out;
}
static void comp_literal() {   // REMARKS column of App. C (msb first) and docs/silicon-datapath.md §11
    comp = {{"i11", {"irc11"}}, {"auz", {"auz"}}, {"c", {"c"}}, {"z", {"z"}}, {"n", {"n"}}, {"v", {"v"}}, {"ze", {"locz"}},
            {"d4", {"dcr4"}}, {"cc", {"cond"}}, {"nz1", {"n", "z"}}, {"nz2", {"n", "z"}}, {"ms0", {"ird8", "alue0"}},
            {"m01", {"auz", "ird8", "alue1", "alue0"}}, {"nv", {"n", "v"}}, {"enl", {"mend", "ird6"}}};
}
static int ndev = 0;
static bool load_dev(const char* path) {
    std::vector<std::string> lines; if (!read_lines(path, lines)) return false;
    for (const std::string& line : lines) {
        std::vector<std::string> f = split(line, '\t');
        if (f.size() < 2) { printf("FAIL: deviation line without composition: '%s'\n", line.c_str()); return false; }
        std::string var = trim(f[0]);
        if (!comp.count(var)) { printf("FAIL: deviation for unknown variable '%s'\n", var.c_str()); return false; }
        comp[var] = split(trim(f[1]), ','); ndev++;
        printf("deviation: %s = {%s} (%s)\n", var.c_str(), trim(f[1]).c_str(), f.size() > 2 ? f[2].c_str() : "no source");
    }
    return true;
}
static int appE(int cc4, int n, int z, int v, int c) {   // MC68000UM, conditions of Bcc/Scc/DBcc
    switch (cc4) {
        case 0: return 1; case 1: return 0; case 2: return !z && !c; case 3: return z || c; case 4: return !c; case 5: return c;
        case 6: return !z; case 7: return z; case 8: return !v; case 9: return v; case 10: return !n; case 11: return n;
        case 12: return n == v; case 13: return n != v; case 14: return !z && n == v; default: return z || n != v;
    }
}
struct In { int irc11, auz, locz, dcr4, mend, cc4, ird6, alue, cc; };
static int sig(const std::string& s, const In& in) {
    int n = in.cc >> 3 & 1, z = in.cc >> 2 & 1, v = in.cc >> 1 & 1, c = in.cc & 1;
    if (s == "irc11") return in.irc11;
    if (s == "auz") return in.auz;
    if (s == "locz") return in.locz;
    if (s == "dcr4") return in.dcr4;
    if (s == "mend") return in.mend;
    if (s == "ird8") return in.cc4 & 1;
    if (s == "ird6") return in.ird6;
    if (s == "alue1") return in.alue >> 1 & 1;
    if (s == "alue0") return in.alue & 1;
    if (s == "n") return n;
    if (s == "z") return z;
    if (s == "v") return v;
    if (s == "c") return c;
    if (s == "nz") return n | z;
    if (s == "cond") return appE(in.cc4, n, z, v, c);
    printf("FAIL: unknown signal '%s' in the composition\n", s.c_str()); printf("RESULT: FAIL\n"); exit(1);
}
static int value(const Code& c, const In& in) { int val = 0; for (const std::string& s : comp[c.var]) val = (val << 1) | sig(s, in); return val; }

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* path = "../docs/data/us4325121_appC.tsv"; const char* dev = nullptr;
    for (int i = 1; i < argc; i++) { if (!strncmp(argv[i], "+data=", 6)) path = argv[i] + 6; if (!strncmp(argv[i], "+dev=", 5)) dev = argv[i] + 5; }
    comp_literal();
    if (!load_appC(path) || (dev && !load_dev(dev))) { printf("RESULT: FAIL\n"); return 1; }
    // App. C shape
    int nent = 0;
    for (auto& kv : codes) {
        const Code& c = kv.second; nent += 1 << c.w;
        CHECK(comp.count(c.var) && (int)comp[c.var].size() == c.w, "code %02x %s: composition %d wide, values %d wide", kv.first, c.var.c_str(), comp.count(c.var) ? (int)comp[c.var].size() : -1, c.w);
        for (int val = 0; val < (1 << c.w); val++) { int r = lookup(kv.first, val); CHECK(r >= 0, "code %02x %s value %d: %s", kv.first, c.var.c_str(), val, r == -1 ? "no entry" : "entries with different outputs"); }
    }
    CHECK(codes.size() == 21, "codes read %d, expected 21", (int)codes.size());
    CHECK(nent == 68, "entries (code x value) %d, expected 68", nent);
    if (fails) { printf("RESULT: FAIL\n"); return 1; }
    dut = new Vm68k_branch_si;
    std::vector<int> sel(512, 0); int n_unused = 0;
    for (int cbc = 0; cbc < 32; cbc++) {
        auto it = codes.find(cbc); bool used = it != codes.end();
        if (!used) n_unused++;
        for (int in = 0; in < (1 << 17); in++) {
            In x; x.irc11 = in & 1; x.auz = in >> 1 & 1; x.locz = in >> 2 & 1; x.dcr4 = in >> 3 & 1; x.mend = in >> 4 & 1;
            x.cc4 = in >> 5 & 15; x.ird6 = in >> 9 & 1; x.alue = in >> 10 & 3; x.cc = in >> 12 & 31;
            dut->cbc = cbc; dut->irc11 = x.irc11; dut->auz = x.auz; dut->locz = x.locz; dut->dcr4 = x.dcr4; dut->mend = x.mend;
            dut->ird = (x.cc4 << 8) | (x.ird6 << 6); dut->alue10 = x.alue; dut->cc = x.cc; dut->eval();
            int rtl_val = dut->rootp->m68k_branch_si__DOT__val & 15;
            sel[(cbc << 4) | rtl_val]++;
            if (used) {
                int val = value(it->second, x), exp = lookup(cbc, val);
                CHECK(rtl_val == val, "cbc %02x (%s) in=%05x: val %d expected %d", cbc, it->second.var.c_str(), in, rtl_val, val);
                CHECK(dut->c1c0 == exp, "cbc %02x (%s) in=%05x val=%d: c1c0 %d expected %d", cbc, it->second.var.c_str(), in, val, dut->c1c0, exp);
            } else {
                CHECK(dut->c1c0 == 0, "cbc %02x (not in App. C) in=%05x: c1c0 %d expected 0", cbc, in, dut->c1c0);
            }
        }
    }
    // table entries: the reachable ones (used code, value < 2^width) all selected; the unreachable ones of the used codes
    // NEVER selected; for the unused codes they are only counted (their index is not specified, the output 00 is)
    int n_reach = 0, n_never_used = 0, n_sel_unused = 0, n_never_unused = 0;
    for (int idx = 0; idx < 512; idx++) {
        int cbc = idx >> 4, val = idx & 15; auto it = codes.find(cbc);
        if (it != codes.end()) {
            if (val < (1 << it->second.w)) { n_reach++; CHECK(sel[idx] > 0, "entry %03x (cbc %02x value %d) reachable but never selected", idx, cbc, val); }
            else { CHECK(sel[idx] == 0, "entry %03x (cbc %02x value %d) unreachable but selected %d times", idx, cbc, val, sel[idx]); if (sel[idx] == 0) n_never_used++; }
        } else { if (sel[idx] > 0) n_sel_unused++; else n_never_unused++; }
    }
    printf("branch: App. C %d codes, %d entries from %s | deviations %d | 32 codes x 131072 inputs | table (512 entries): reachable %d all selected, unreachable of the used codes %d never selected, of the %d unused codes %d selected (output 00) and %d never | check %d | FAIL %d\n",
           (int)codes.size(), nent, path, ndev, n_reach, n_never_used, n_unused, n_sel_unused, n_never_unused, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
