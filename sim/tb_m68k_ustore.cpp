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

// tb_m68k_ustore.cpp - verification of the micro control store (rtl/cpu_si/m68k_ustore.sv: micro ROM + address selector m68k_useq)
// against a model INDEPENDENT of the RTL and of the generators: the die bits of the micro ROM read at run time from
// docs/data/mc68000_micro_decoded.csv (+data=<file>; default ../docs/data/mc68000_micro_decoded.csv: column raw17, bit 0
// first, and column nma_or_base = decoded next address, verified 397/397 against the App. A listing), decoded with the
// microword format of docs/microcode-structure.md section 3 (bit 0 I, bit 1 format, bits 2-3 TY and bit 4 in
// format I, bits 2-6 CBC in format II, bits 15-16 FC). No generated file is used.
// Words added to the die or repaired (holes in the dump), optional (+add=<file>): lines "addr<TAB>raw17 (bit 0 first)<TAB>nma
// hex<TAB>source"; an address already in the die replaces the CSV word. Without the file the addresses not in the die must be
// empty words (all bits at 0) and the die holds literally.
// Test A, ALL 1024 addresses (positioning with the immediate a0s override): upc, upc_is327, I, format, FC; format I: TY,
// bit 4 and next address (TY=0: NMA; TY=1: ALL 1024 values of A1 and, with A0SUB, of A0; TY=2/3: all 1024 values of
// A2/A3); format II: CBC and the 4 addresses for C1C0 = 0..3; a0s override on every word (all 1024 values on word 000).
// Test B: random walk of 200,000 microcycles with the real pipeline (fixed seed), hold with en = 0, restarts via a0s where
// the C1C0 combination has no word in the die. Reachability (informative): closure from the entry addresses of the
// exception block (000, 003, 0e3, 1c4, 3fd) along NMA and conditional jumps. Shape checks on the CSV (FAIL if they do not hold):
// 544 words, distinct addresses, raw17 of 17 bits, columns I/fmt/ty/cbc consistent with raw17, bits 7:6 of the base at 0 in format II.
// Configuration: ORG_OVR_LUT as in the core (from the Makefile); it does not affect the outputs checked here. Outcome: RESULT: PASS/FAIL.
#include "Vm68k_ustore.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <fstream>
#include <sstream>

struct Word { std::string raw; int nma = 0; std::string label; bool die = true; };
static std::map<int, Word> words;   // address -> word (die + additions)
static const std::string EMPTY(17, '0');
static const Word EMPTYW{EMPTY, 0, "empty", false};
static Vm68k_ustore* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
static void tick() { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); dut->clk = 0; dut->eval(); }
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
static bool is_bits(const std::string& s) { return s.size() == 17 && s.find_first_not_of("01") == std::string::npos; }
static int bit(const std::string& r, int i) { return r[i] - '0'; }
struct F { int I, fmt, ty, bit4, cbc, fc; };
static F fields(const std::string& r) {   // microword format, microcode-structure.md section 3
    F f; f.I = bit(r, 0); f.fmt = bit(r, 1); f.ty = bit(r, 2) | bit(r, 3) << 1; f.bit4 = bit(r, 4);
    f.cbc = 0; for (int k = 0; k < 5; k++) f.cbc |= bit(r, 2 + k) << k;
    f.fc = bit(r, 15) | bit(r, 16) << 1; return f;
}
static const Word& word_at(int a) { auto it = words.find(a); return it == words.end() ? EMPTYW : it->second; }

// die CSV: addr,label,I,fmt,ty,cbc,cond,nma_or_base,fc,nano_row,nano_slot,raw17 (cbc decimal)
static bool load_csv(const char* path) {
    std::vector<std::string> lines; if (!read_lines(path, lines)) return false;
    std::map<std::string, int> col; int n = 0;
    for (const std::string& line : lines) {
        std::vector<std::string> f = split(line, ',');
        if (col.empty()) { for (size_t i = 0; i < f.size(); i++) col[trim(f[i])] = (int)i;
            for (const char* k : {"addr", "I", "fmt", "ty", "cbc", "nma_or_base", "fc", "raw17", "label"}) if (!col.count(k)) { printf("FAIL: column %s missing from the CSV\n", k); return false; }
            continue; }
        if ((int)f.size() <= col["raw17"]) { printf("FAIL: CSV line with %d fields: '%s'\n", (int)f.size(), line.c_str()); return false; }
        Word w; w.raw = trim(f[col["raw17"]]); w.label = trim(f[col["label"]]); w.die = true;
        int a = (int)strtol(f[col["addr"]].c_str(), nullptr, 16); w.nma = (int)strtol(f[col["nma_or_base"]].c_str(), nullptr, 16);
        CHECK(a >= 0 && a < 1024 && !words.count(a), "CSV %s: address %03x invalid or repeated", w.label.c_str(), a);
        CHECK(is_bits(w.raw), "CSV %s: raw17 '%s' invalid", w.label.c_str(), w.raw.c_str());
        if (!is_bits(w.raw)) return false;
        F x = fields(w.raw);
        CHECK(atoi(f[col["I"]].c_str()) == x.I && atoi(f[col["fmt"]].c_str()) == x.fmt, "CSV %s: columns I/fmt (%s/%s) differ from bits 0/1 of raw17", w.label.c_str(), f[col["I"]].c_str(), f[col["fmt"]].c_str());
        if (x.fmt == 0) CHECK(atoi(f[col["ty"]].c_str()) == x.ty, "CSV %s: column ty %s differs from bits 3:2 (%d)", w.label.c_str(), f[col["ty"]].c_str(), x.ty);
        else { CHECK(atoi(f[col["cbc"]].c_str()) == x.cbc, "CSV %s: column cbc %s differs from bits 6:2 (%d)", w.label.c_str(), f[col["cbc"]].c_str(), x.cbc);
               CHECK(((w.nma >> 6) & 3) == 0, "CSV %s: base %03x with bits 7:6 not at 0", w.label.c_str(), w.nma); }
        CHECK(atoi(f[col["fc"]].c_str()) == x.fc && w.nma >= 0 && w.nma < 1024, "CSV %s: fc %s or nma %03x inconsistent", w.label.c_str(), f[col["fc"]].c_str(), w.nma);
        words[a] = w; n++;
    }
    CHECK(n == 544, "die words read %d, expected 544", n);
    return true;
}
static int nadd = 0, nrep = 0;
static bool load_add(const char* path) {
    std::vector<std::string> lines; if (!read_lines(path, lines)) return false;
    for (const std::string& line : lines) {
        std::vector<std::string> f = split(line, '\t');
        if (f.size() < 3) { printf("FAIL: incomplete added-word line: '%s'\n", line.c_str()); return false; }
        int a = (int)strtol(f[0].c_str(), nullptr, 16); Word w; w.raw = trim(f[1]); w.nma = (int)strtol(f[2].c_str(), nullptr, 16);
        if (a < 0 || a >= 1024 || !is_bits(w.raw) || w.nma < 0 || w.nma >= 1024) { printf("FAIL: invalid added word: '%s'\n", line.c_str()); return false; }
        bool rep = words.count(a) > 0;   // address already in the die: REPAIRED word (e.g. hole in the dump), replaces the CSV one
        w.label = (rep ? "repaired " : "added ") + trim(f[0]); w.die = false;
        words[a] = w; if (rep) nrep++; else nadd++;
        printf("word %s: %03x raw %s nma %03x (%s)\n", rep ? "repaired" : "added", a, w.raw.c_str(), w.nma, f.size() > 3 ? f[3].c_str() : "no source");
    }
    return true;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* path = "../docs/data/mc68000_micro_decoded.csv"; const char* add = nullptr;
    for (int i = 1; i < argc; i++) { if (!strncmp(argv[i], "+data=", 6)) path = argv[i] + 6; if (!strncmp(argv[i], "+add=", 5)) add = argv[i] + 5; }
    if (!load_csv(path) || (add && !load_add(add)) || fails) { printf("RESULT: FAIL\n"); return 1; }
    std::vector<int> addrs; for (auto& kv : words) addrs.push_back(kv.first);
    dut = new Vm68k_ustore;
    dut->clk = 0; dut->rst_n = 0; dut->en = 1; dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->a0_sub = 0; dut->a0 = 0;
    dut->a0s_sub = 0; dut->a0s = 0; dut->c1c0 = 0;
    tick(); tick(); dut->rst_n = 1; dut->eval();
    CHECK(dut->upc == 0, "reset upc=%03x", dut->upc);
    // Test A: all 1024 addresses
    const int M1 = 0x3a5, M2 = 0x35a, M3 = 0x2cc, M0 = 0x1b3;   // distinct markers for A1/A2/A3/A0
    int n_direct = 0, n_disp = 0, n_cond = 0, n_empty = 0;
    for (int a = 0; a < 1024; a++) {
        const Word& w = word_at(a); F f = fields(w.raw); const char* L = w.label.c_str();
        // positioning: immediate a0s override for one clock, then one settling clock with en = 0 (uaddr comes out of the uw_q register
        // one clock after the edge; in the core the next en arrives after >= 4 clocks)
        dut->a0s_sub = 1; dut->a0s = a; dut->eval(); tick(); dut->en = 0; dut->eval(); tick(); dut->en = 1;
        dut->a0s_sub = 0; dut->a1 = M1; dut->a2 = M2; dut->a3 = M3; dut->a0 = M0; dut->a0_sub = 0; dut->c1c0 = 0; dut->eval();
        CHECK(dut->upc == a, "%03x %s: upc %03x", a, L, dut->upc);
        CHECK(dut->upc_is327 == (a == 0x327), "%03x %s: upc_is327 %d", a, L, dut->upc_is327);
        CHECK(dut->ir_load == f.I, "%03x %s: I %d expected %d", a, L, dut->ir_load, f.I);
        CHECK(dut->fmt == f.fmt, "%03x %s: fmt %d expected %d", a, L, dut->fmt, f.fmt);
        CHECK(dut->fc == f.fc, "%03x %s: fc %d expected %d", a, L, dut->fc, f.fc);
        if (f.fmt) {
            CHECK(dut->cbc == f.cbc, "%03x %s: cbc %d expected %d", a, L, dut->cbc, f.cbc);
            for (int c = 0; c < 4; c++) { dut->c1c0 = c; dut->eval(); CHECK(dut->uaddr == (w.nma | c << 6), "%03x %s: C1C0=%d uaddr %03x expected %03x", a, L, c, dut->uaddr, w.nma | c << 6); }
            dut->c1c0 = 0; n_cond++;
        } else {
            CHECK(dut->ty == f.ty, "%03x %s: ty %d expected %d", a, L, dut->ty, f.ty);
            CHECK(dut->bit4 == f.bit4, "%03x %s: bit4 %d expected %d", a, L, dut->bit4, f.bit4);
            if (f.ty == 0) { CHECK(dut->uaddr == w.nma, "%03x %s: uaddr %03x expected NMA %03x", a, L, dut->uaddr, w.nma); if (words.count(a)) n_direct++; }
            else {
                for (int v = 0; v < 1024; v++) {
                    if (f.ty == 1) {
                        dut->a0_sub = 0; dut->a1 = v; dut->eval(); CHECK(dut->uaddr == v, "%03x %s: A1=%03x uaddr %03x", a, L, v, dut->uaddr);
                        dut->a0_sub = 1; dut->a0 = v; dut->eval(); CHECK(dut->uaddr == v, "%03x %s: A0SUB A0=%03x uaddr %03x", a, L, v, dut->uaddr);
                    } else if (f.ty == 2) { dut->a2 = v; dut->eval(); CHECK(dut->uaddr == v, "%03x %s: A2=%03x uaddr %03x", a, L, v, dut->uaddr); }
                    else { dut->a3 = v; dut->eval(); CHECK(dut->uaddr == v, "%03x %s: A3=%03x uaddr %03x", a, L, v, dut->uaddr); }
                }
                dut->a0_sub = 0; dut->a1 = M1; dut->a2 = M2; dut->a3 = M3; dut->a0 = M0; n_disp++;
            }
        }
        if (!words.count(a)) n_empty++;
        // immediate override: all 1024 values on word 000, one on the others
        dut->a0s_sub = 1;
        if (a == 0) { for (int v = 0; v < 1024; v++) { dut->a0s = v; dut->eval(); CHECK(dut->uaddr == v, "000: a0s=%03x uaddr %03x", v, dut->uaddr); } }
        else { dut->a0s = 0x3ff; dut->eval(); CHECK(dut->uaddr == 0x3ff, "%03x %s: a0s override uaddr %03x", a, L, dut->uaddr); }
        dut->a0s_sub = 0;
    }
    // Test B: random walk with the real pipeline (same en/a0s protocol as the core)
    srand(1); dut->a0s_sub = 1; dut->a0s = 0; dut->eval(); dut->en = 0; dut->eval(); tick(); dut->en = 1; dut->eval(); tick(); dut->a0s_sub = 0; dut->en = 0; dut->eval(); tick(); dut->en = 1;
    int cur = 0, steps = 0, holds = 0, deadends = 0;
    for (int s = 0; s < 200000; s++) {
        auto it = words.find(cur);
        CHECK(it != words.end(), "step %d: address %03x without a word", s, cur);
        if (it == words.end()) break;
        const Word& e = it->second; F f = fields(e.raw);
        int nxt; int r = rand();
        if (f.fmt) { int c = r & 3; dut->c1c0 = c; nxt = e.nma | (c << 6); }
        else if (f.ty == 0) nxt = e.nma;
        else {
            int d = addrs[r % addrs.size()];
            if (f.ty == 1) { if (r & 4) { dut->a0_sub = 1; dut->a0 = d; } else { dut->a0_sub = 0; dut->a1 = d; } }
            else if (f.ty == 2) dut->a2 = d; else dut->a3 = d;
            nxt = d;
        }
        if ((r >> 8) % 7 == 0) { dut->a0s_sub = 1; dut->a0s = addrs[(r >> 3) % addrs.size()]; nxt = dut->a0s; }
        if ((r >> 12) % 5 == 0) { dut->en = 0; dut->eval(); tick(); CHECK(dut->upc == cur, "hold en=0 upc %03x expected %03x", dut->upc, cur); dut->en = 1; holds++; }
        dut->eval(); CHECK(dut->uaddr == nxt, "step %d %s: uaddr %03x expected %03x", s, e.label.c_str(), dut->uaddr, nxt);
        if (!words.count(nxt)) {   // C1C0 combination without a word in the die: restart via a0s (immediate override)
            deadends++; dut->a0s_sub = 1; dut->a0s = addrs[(r >> 5) % addrs.size()]; nxt = dut->a0s; dut->eval();
            CHECK(dut->uaddr == nxt, "step %d: restart uaddr %03x expected %03x", s, dut->uaddr, nxt);
        }
        dut->en = 0; dut->eval(); tick(); dut->en = 1; dut->eval(); tick(); dut->a0s_sub = 0; dut->en = 0; dut->eval(); tick(); dut->en = 1; dut->a0_sub = 0;
        CHECK(dut->upc == nxt, "step %d: upc %03x expected %03x", s, dut->upc, nxt);
        cur = nxt; steps++;
    }
    // reachability (informative): from the entry addresses of m68k_exc_si (000, 003, 0e3, 1c4, 3fd) along NMA and the 4 conditional jumps; dispatches stop the chain
    std::set<int> seen, empty_hit; std::vector<int> todo = {0x000, 0x003, 0x0e3, 0x1c4, 0x3fd};
    while (!todo.empty()) {
        int a = todo.back(); todo.pop_back();
        if (!words.count(a)) { empty_hit.insert(a); continue; }
        if (!seen.insert(a).second) continue;
        F f = fields(words[a].raw);
        if (f.fmt) { for (int c = 0; c < 4; c++) todo.push_back(words[a].nma | c << 6); }
        else if (f.ty == 0) todo.push_back(words[a].nma);
    }
    printf("ustore: die words 544 (repaired %d) + added %d from %s | addresses 1024: direct %d, dispatch %d (x 1024 A1 and A0, or A2/A3), conditional %d (x4 C1C0), empty %d | random steps %d (hold %d, C1C0 without a word %d) | reached from the core entry points without dispatch: %d words, %d empty addresses | check %d | FAIL %d\n",
           nrep, nadd, path, n_direct, n_disp, n_cond, n_empty, steps, holds, deadends, (int)seen.size(), (int)empty_hit.size(), checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
