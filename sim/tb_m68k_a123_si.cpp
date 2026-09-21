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

// tb_m68k_a123_si.cpp - verification of the A1/A2/A3 decoder (rtl/cpu_si/m68k_a123_si.sv inside m68k_a123_top, core configuration:
// external row class from m68k_a123_cls) on ALL 65536 IR values against the table of the die PLAs read at run
// time (docs/data/mc68000_a123_die.tsv: op a1 a2 a3 ill priv linea linef; 3ff = none) and the A2 read table
// (docs/data/mc68000_a123_need.tsv: opcode -> 1 if the instruction flow reads A2; A3 is always read). No generated file.
// Expected: illegal = ill | linea | linef of the die; valid opcodes: a1 and a3 equal to the die, a2 equal to the die where the flow
// reads it (elsewhere the decoder may leave it undefined: counted), priv = priv column of the die (on illegal opcodes the illegal
// instruction exception takes precedence over privilege: not compared); illegal opcodes: a1 = 3ff (input illg1, from which the
// exception decides illegal / line A / line F). Documented deviations from the +dev file (one per line, with source): word/long pairs
// of the EA chains (pair), line 4 long in memory -> long chain (a1_line4_long_mem), A2 fixed over a range (a2_set), A3 of the die
// with no word in App. H -> equivalent word in the memory modes and not verified in the register modes (a3_alias), A3 = none
// not verified (a3_none). Verdict: RESULT: PASS/FAIL, exit 0/1.
#include "Vm68k_a123_si.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
static Vm68k_a123_si* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)
struct Die { int a1 = -1, a2 = -1, a3 = -1, ill = 0, priv = 0, linea = 0, linef = 0; };
static std::vector<Die> DIE(65536); static std::vector<int> NEED(65536, -1);
// deviations
static std::map<int, int> PAIR; static bool R_LINE4 = false, R_A3NONE = false, R_LINEAF = false; static std::map<int, int> A3ALIAS; static std::vector<int> A2SET(65536, -1), PRIVSET(65536, -1); static int n_devl = 0;
static bool load(const char* die, const char* need, const char* dev) {
    std::ifstream f(die); if (!f) { printf("FAIL: die table not readable: %s\n", die); return false; }
    std::string l; int n = 0;
    while (std::getline(f, l)) {
        if (l.empty() || l[0] == '#') continue;
        unsigned op, a1, a2, a3; int ill, priv, la, lf;
        if (sscanf(l.c_str(), "%x %x %x %x %d %d %d %d", &op, &a1, &a2, &a3, &ill, &priv, &la, &lf) != 8 || op > 65535) { printf("FAIL: invalid die row: %s\n", l.c_str()); return false; }
        Die& d = DIE[op]; d.a1 = a1; d.a2 = a2; d.a3 = a3; d.ill = ill; d.priv = priv; d.linea = la; d.linef = lf; n++;
    }
    if (n != 65536) { printf("FAIL: die table with %d opcodes (expected 65536)\n", n); return false; }
    std::ifstream g(need); if (!g) { printf("FAIL: A2 read table not readable: %s\n", need); return false; }
    n = 0;
    while (std::getline(g, l)) {
        if (l.empty() || l[0] == '#') continue;
        unsigned op; int r;
        if (sscanf(l.c_str(), "%x %d", &op, &r) != 2 || op > 65535) { printf("FAIL: invalid need row: %s\n", l.c_str()); return false; }
        NEED[op] = r; n++;
    }
    if (n != 65536) { printf("FAIL: A2 read table with %d opcodes (expected 65536)\n", n); return false; }
    if (dev && *dev) {
        std::ifstream h(dev); if (!h) { printf("FAIL: deviation file not readable: %s\n", dev); return false; }
        while (std::getline(h, l)) {
            if (l.empty() || l[0] == '#') continue;
            std::vector<std::string> c; { size_t p = 0; for (;;) { size_t q = l.find('\t', p); c.push_back(l.substr(p, q == std::string::npos ? std::string::npos : q - p)); if (q == std::string::npos) break; p = q + 1; } }
            n_devl++;
            if (c[0] == "pair" && c.size() >= 3) { int a = (int)strtol(c[1].c_str(), nullptr, 16), b = (int)strtol(c[2].c_str(), nullptr, 16); PAIR[a] = b; PAIR[b] = a; }
            else if (c[0] == "a1_line4_long_mem") R_LINE4 = true;
            else if (c[0] == "a3_none") R_A3NONE = true;
            else if (c[0] == "a3_alias" && c.size() >= 3) A3ALIAS[(int)strtol(c[1].c_str(), nullptr, 16)] = (int)strtol(c[2].c_str(), nullptr, 16);
            else if (c[0] == "a2_set" && c.size() >= 4) { int lo = (int)strtol(c[1].c_str(), nullptr, 16), hi = (int)strtol(c[2].c_str(), nullptr, 16), v = (int)strtol(c[3].c_str(), nullptr, 16); for (int o = lo; o <= hi && o < 65536; o++) A2SET[o] = v; }
            else if (c[0] == "priv_set" && c.size() >= 4) { int lo = (int)strtol(c[1].c_str(), nullptr, 16), hi = (int)strtol(c[2].c_str(), nullptr, 16), v = atoi(c[3].c_str()); for (int o = lo; o <= hi && o < 65536; o++) PRIVSET[o] = v; }
            else if (c[0] == "lineaf_dontcare") R_LINEAF = true;
            else { printf("FAIL: invalid deviation row: %s\n", l.c_str()); return false; }
        }
    }
    return true;
}
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* die = "../docs/data/mc68000_a123_die.tsv"; const char* need = "../docs/data/mc68000_a123_need.tsv"; const char* dev = nullptr;
    for (int i = 1; i < argc; i++) { if (!strncmp(argv[i], "+die=", 5)) die = argv[i] + 5; if (!strncmp(argv[i], "+need=", 6)) need = argv[i] + 6; if (!strncmp(argv[i], "+dev=", 5)) dev = argv[i] + 5; }
    if (!load(die, need, dev)) { printf("RESULT: FAIL\n"); return 1; }
    dut = new Vm68k_a123_si;
    int n_valid = 0, n_ill = 0, n_la = 0, n_lf = 0, n_priv = 0, n_a2 = 0, n_a2_dc = 0, n_a3_none = 0, n_a3_alias = 0, n_a3_reg = 0, n_l4 = 0, n_a2set = 0, n_laf = 0, n_privset = 0;
    for (uint32_t ir = 0; ir < 65536; ir++) {
        dut->ir = ir; dut->eval();
        const Die& d = DIE[ir];
        bool ill = d.ill || d.linea || d.linef;
        if (d.ill) n_ill++; if (d.linea) n_la++; if (d.linef) n_lf++;
        if (R_LINEAF && (d.linea || d.linef)) { n_laf++; continue; }   // line A/F: detected by the exception block from ir[15:12], not by the decoder
        CHECK(dut->illegal == ill, "ir %04x: illegal %d expected %d (die: ill %d linea %d linef %d)", ir, dut->illegal, ill, d.ill, d.linea, d.linef);
        if (ill) { CHECK(dut->a1 == 0x3ff, "ir %04x illegal: a1 %03x expected 3ff (illg1)", ir, dut->a1); continue; }
        n_valid++;
        int ep = PRIVSET[ir] >= 0 ? PRIVSET[ir] : d.priv; if (PRIVSET[ir] >= 0) n_privset++; if (ep) n_priv++;
        CHECK(dut->priv == ep, "ir %04x: priv %d expected %d (die %d)", ir, dut->priv, ep, d.priv);
        int line = ir >> 12, sz = (ir >> 6) & 3, mode = (ir >> 3) & 7, hi = (ir >> 8) & 15;
        // A1: deviation line 4 long in memory -> twin chain (long): only the one-operand instructions NEGX/CLR/NEG/NOT/TST
        // (bits 11:8 = 0000/0010/0100/0110/1010 of the manual), not CHK/LEA/MOVEM, which share bits 7:6 = 10
        bool sop = hi == 0 || hi == 2 || hi == 4 || hi == 6 || hi == 10;
        int e1 = d.a1;
        if (R_LINE4 && line == 4 && sop && sz == 2 && mode >= 2 && PAIR.count(e1)) { e1 = PAIR[e1]; n_l4++; }
        CHECK(dut->a1 == e1, "ir %04x: a1 %03x expected %03x (die %03x)", ir, dut->a1, e1, d.a1);
        // A3: none (3ff) not verified; alias of the addresses without a word for the memory modes, not verified for the register modes
        int e3 = d.a3; bool chk3 = true;
        if (e3 == 0x3ff && R_A3NONE) { chk3 = false; n_a3_none++; }
        else if (A3ALIAS.count(e3)) { if (mode >= 2) { e3 = A3ALIAS[e3]; n_a3_alias++; } else { chk3 = false; n_a3_reg++; } }
        if (chk3) CHECK(dut->a3 == e3, "ir %04x: a3 %03x expected %03x (die %03x)", ir, dut->a3, e3, d.a3);
        // A2: only where the flow reads it
        if (NEED[ir] == 1) {
            int e2 = d.a2; if (A2SET[ir] >= 0) { e2 = A2SET[ir]; n_a2set++; }
            n_a2++; CHECK(dut->a2 == e2, "ir %04x: a2 %03x expected %03x (die %03x; read by the flow)", ir, dut->a2, e2, d.a2);
        } else if (dut->a2 != d.a2) n_a2_dc++;
    }
    printf("A123: IR 65536: valid %d (a1 compared on all, of which %d with the long chain of line 4; a3 compared on %d, alias on %d, not verified: none %d, register modes %d; a2 compared on %d opcodes that read it, of which %d fixed, different from the die and not read on %d; priv compared on all valid ones, %d fixed), illegal %d (die ill %d: illegal and a1 = 3ff verified; line A %d, line F %d: %d not verified, detected by the exception block), privileged %d | deviations %d rows | check %d | FAIL %d\n",
           n_valid, n_l4, n_valid - n_a3_none - n_a3_reg, n_a3_alias, n_a3_none, n_a3_reg, n_a2, n_a2set, n_a2_dc, n_privset, n_ill + n_la + n_lf, n_ill, n_la, n_lf, n_laf, n_priv, n_devl, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
