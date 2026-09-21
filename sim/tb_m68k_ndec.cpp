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

// tb_m68k_ndec.cpp - verification of the complete control store (rtl/cpu_si/m68k_nstore_dec.sv: micro ROM + sequencer + nano ROM +
// per-resource decoder m68k_nano_dec.sv) against the INDEPENDENT model of the nano ROM (m68k_apph_model.h: patent App. H + map
// App. H -> die + rules R1-R3 + deviations file with sources). Core configuration (ORG_OVR_LUT = 0: immediate override
// only at 000/003), positioning along the reset chain + A1 as in the processor.
// Test A (exhaustive): for EVERY micro address (1024) the atoms rebuilt from the dec vector (pkg fields, m68k_ndec_fields.h)
//   must match those of the model; undefined addresses (neither App. H block nor deviation) must give an empty dec.
// Test C: the immediate override at 000 and at 003 gives the same atoms as the normal path (A1).
// Test B: random walk of 200,000 microcycles (random c1c0 and A1/A2/A3): at every step the output atoms are those
//   of the current address recorded in test A (consistency of the micro -> nano -> dec pipeline in every sequence).
// Outcome: RESULT: PASS/FAIL, exit code 0/1.
#include "Vm68k_nstore_dec.h"
#include "verilated.h"
#include "m68k_ndec_fields.h"
#include "m68k_apph_model.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <set>
#include <string>
#include <vector>
static Vm68k_nstore_dec* dut;
static int fails = 0, checks = 0;
static long n_steps = 0;
double sc_time_stamp() { return 0.0; }
static void tick() { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); dut->clk = 0; dut->eval(); }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)
// one microcycle: capture clock (en = 0: the nano ROM reads the index of the next word one clock before en), en edge,
// settling clock (the dec bits of non-RAW class come out one clock after en)
static void adv(int en) { dut->en = 0; dut->eval(); tick(); dut->en = en; dut->eval(); tick(); dut->en = 0; dut->eval(); tick(); n_steps++; }
static bool is_dispatch() { return ((dut->uw >> 1) & 1) == 0 && ((dut->uw >> 2) & 3) == 1; }   // format I, TY = 1 (dispatch A1)
static int lut_mode = 0;
static bool goto_word(int W) {
    if (lut_mode) { dut->a0s_sub = 1; dut->a0s = W; adv(1); dut->a0s_sub = 0; }
    else {
        dut->a0s_sub = 1; dut->a0s = 0; adv(1); dut->a0s_sub = 0;   // override to word 000 (allowed in the core)
        if (dut->upc != 0) { CHECK(false, "positioning: upc %03x after the override to 000", dut->upc); return false; }
        for (int i = 0; i < 32 && !is_dispatch(); i++) adv(1);     // reset path up to the dispatch
        if (!is_dispatch()) { CHECK(false, "positioning: no dispatch within 32 words of reset"); return false; }
        dut->a1 = W; adv(1); dut->a1 = 0;
    }
    CHECK(dut->upc == W, "positioning: upc %03x expected %03x", dut->upc, W);
    return dut->upc == W;
}
static std::string join(const std::set<std::string>& s) { return apph_join(s); }
static std::string diff(const std::set<std::string>& a, const std::set<std::string>& b) { std::set<std::string> d; for (auto& x : a) if (!b.count(x)) d.insert(x); return join(d); }

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* pkg = "../rtl/cpu_si/m68k_nano_dec_pkg.sv"; const char* apph = "../docs/data/us4325121_appH.tsv"; const char* diemap = "../docs/data/us4325121_appH_die_map.tsv"; const char* dev = nullptr;
    for (int i = 1; i < argc; i++) {
        if (!strncmp(argv[i], "+pkg=", 5)) pkg = argv[i] + 5; if (!strncmp(argv[i], "+apph=", 6)) apph = argv[i] + 6; if (!strncmp(argv[i], "+diemap=", 8)) diemap = argv[i] + 8;
        if (!strncmp(argv[i], "+dev=", 5)) dev = argv[i] + 5; if (!strncmp(argv[i], "+lut=", 5)) lut_mode = atoi(argv[i] + 5);
    }
    if (!nd_load_pkg(pkg) || !apph_load(apph, diemap, dev)) { printf("RESULT: FAIL\n"); return 1; }
    dut = new Vm68k_nstore_dec;
    dut->clk = 0; dut->rst_n = 0; dut->en = 1; dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->a0_sub = 0; dut->a0 = 0; dut->a0s_sub = 0; dut->a0s = 0; dut->c1c0 = 0;
    tick(); tick(); dut->rst_n = 1; dut->eval();
    // Test A
    std::vector<std::set<std::string>> atomsW(1024); int n_def = 0, n_undef = 0; long n_atoms = 0;
    for (int W = 0; W < 1024; W++) {
        if (!goto_word(W)) { printf("RESULT: FAIL\n"); return 1; }
        std::set<std::string> S = nd_atoms(&dut->dec[0]); atomsW[W] = S;
        std::set<std::string> need; std::vector<std::string> mt;
        if (apph_expected(W, S, need, mt)) {
            n_def++; n_atoms += (long)S.size();
            std::string smt; for (auto& x : mt) smt += (smt.empty() ? "" : ";") + x;
            CHECK(need == S && mt.empty(), "word %03x (%s): DUT atoms differ from the model: missing {%s}, extra {%s}, '*' transfers not found {%s}", W, apph_label(W).c_str(), diff(need, S).c_str(), diff(S, need).c_str(), smt.c_str());
        } else {
            n_undef++;
            CHECK(S.empty(), "address %03x undefined (neither App. H nor deviations) but with atoms: %s", W, join(S).c_str());
        }
        if (fails > 20) break;
    }
    if (!APPH_UNMAPPED.empty()) for (auto& kv : APPH_UNMAPPED) CHECK(false, "App. H form not recognised by the model: %s (%d times)", kv.first.c_str(), kv.second);
    // Test C: immediate override (000 from reset/a0s, 003 = bus error entry) against the normal path
    for (int W : {0, 3}) {
        dut->a0s_sub = 1; dut->a0s = W; adv(1); dut->a0s_sub = 0;
        CHECK(dut->upc == W, "immediate override: upc %03x expected %03x", dut->upc, W);
        std::set<std::string> S = nd_atoms(&dut->dec[0]);
        CHECK(S == atomsW[W], "immediate override at %03x: atoms {%s} differ from the normal path {%s}", W, join(S).c_str(), join(atomsW[W]).c_str());
    }
    // Test B: random walk
    std::mt19937 rng(1); long n_disp = 0; const int NWALK = 200000;
    for (int i = 0; i < NWALK && fails <= 20; i++) {
        dut->c1c0 = rng() & 3; dut->a1 = rng() & 1023; dut->a2 = rng() & 1023; dut->a3 = rng() & 1023;
        if (is_dispatch()) n_disp++;
        adv(1);
        std::set<std::string> S = nd_atoms(&dut->dec[0]);
        if (S != atomsW[dut->upc]) CHECK(false, "random walk step %d: upc %03x with atoms {%s}, in test A {%s}", i, dut->upc, join(S).c_str(), join(atomsW[dut->upc]).c_str()); else checks++;
    }
    dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->c1c0 = 0;
    printf("NDEC: defined addresses %d (App. H blocks with die address %d, deviations %ld over %d addresses), unused %d (empty dec) | atoms verified %ld | '*' transfers resolved on AB %ld, DB %ld, literals %ld | R2 (whole AU bus) %ld, R3 (au+=0) %ld | override 000/003 | random walk %d steps (%ld dispatch) | microcycles %ld | check %d | FAIL %d\n",
           n_def, (int)APPH_WORD.size(), apph_n_devops, (int)APPH_DEVS.size(), n_undef, n_atoms, apph_n_tol_ab, apph_n_tol_db, apph_n_tol_lit, apph_n_r2, apph_n_r3, NWALK, n_disp, n_steps, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
