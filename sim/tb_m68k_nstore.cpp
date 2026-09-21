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

// tb_m68k_nstore.cpp - STRUCTURAL verification of the nano ROM (rtl/cpu_si/m68k_nrom.sv inside m68k_ustore: ORIGIN table in logic,
// nanoword index {row(addr[9:2]), addr[1:0]}, M10K read one clock before en, immediate override), independent of the nanoword
// encoding. The content (atoms) is verified by the ndec testbench; here the App. H model (m68k_apph_model.h) is used only to
// know which addresses are defined and which have the same content. Core configuration (ORG_OVR_LUT = 0), positioning along the
// reset chain.
// Test A (exhaustive): for EVERY micro address (1024): zero nanoword on the undefined addresses; at every en edge the
//   nanoword is the one anticipated one clock earlier (nw_pre) and nwl_pre is its low 15 bits.
// Test S: addresses with the same expected content (same model key) have the same nanoword; equal nanowords with
//   different expected content: counted (informative: equivalent App. H forms can give the same word).
// Test C: immediate override at 000 and 003: same nanoword and same nidx index as the normal path (A1).
// Test B: random walk of 200,000 microcycles: nanoword and index are those of the current address recorded in test A.
// Outcome: RESULT: PASS/FAIL, exit code 0/1.
#include "Vm68k_ustore.h"
#include "verilated.h"
#include "m68k_apph_model.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>
static Vm68k_ustore* dut;
static int fails = 0, checks = 0;
static long n_steps = 0, n_en_checks = 0;
double sc_time_stamp() { return 0.0; }
static void tick() { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); dut->clk = 0; dut->eval(); }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)
typedef std::vector<uint32_t> Word;
template <class T> static Word words(const T& v) { Word w(sizeof(T) / 4); memcpy(w.data(), &v, sizeof(T)); return w; }
static std::string hex(const Word& w) { char b[64]; std::string s; for (int i = (int)w.size() - 1; i >= 0; i--) { snprintf(b, sizeof b, "%08x", w[i]); s += b; } return s; }
static bool zero(const Word& w) { for (auto x : w) if (x) return false; return true; }
// one microcycle: capture clock (the anticipated word nw_pre is valid in the clock before en), en edge, settling clock
static void adv(int en) {
    dut->en = 0; dut->eval(); tick();
    Word pre = words(dut->nw_pre); uint32_t prel = dut->nwl_pre;
    dut->en = en; dut->eval(); tick(); dut->en = 0; dut->eval();
    if (en) {
        n_en_checks++;
        CHECK(words(dut->nw) == pre, "early read: at en the nanoword %s is not the anticipated nw_pre %s (upc %03x)", hex(words(dut->nw)).c_str(), hex(pre).c_str(), dut->upc);
        CHECK(prel == (pre[0] & 0x7fff), "early read: nwl_pre %04x differs from the low 15 bits of nw_pre %04x (upc %03x)", prel, pre[0] & 0x7fff, dut->upc);
    }
    tick(); n_steps++;
}
static bool is_dispatch() { return ((dut->uw >> 1) & 1) == 0 && ((dut->uw >> 2) & 3) == 1; }
static int lut_mode = 0;
static bool goto_word(int W) {
    if (lut_mode) { dut->a0s_sub = 1; dut->a0s = W; adv(1); dut->a0s_sub = 0; }
    else {
        dut->a0s_sub = 1; dut->a0s = 0; adv(1); dut->a0s_sub = 0;
        if (dut->upc != 0) { CHECK(false, "positioning: upc %03x after the override to 000", dut->upc); return false; }
        for (int i = 0; i < 32 && !is_dispatch(); i++) adv(1);
        if (!is_dispatch()) { CHECK(false, "positioning: no dispatch within 32 words of reset"); return false; }
        dut->a1 = W; adv(1); dut->a1 = 0;
    }
    CHECK(dut->upc == W, "positioning: upc %03x expected %03x", dut->upc, W);
    return dut->upc == W;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* apph = "../docs/data/us4325121_appH.tsv"; const char* diemap = "../docs/data/us4325121_appH_die_map.tsv"; const char* dev = nullptr;
    for (int i = 1; i < argc; i++) {
        if (!strncmp(argv[i], "+apph=", 6)) apph = argv[i] + 6; if (!strncmp(argv[i], "+diemap=", 8)) diemap = argv[i] + 8;
        if (!strncmp(argv[i], "+dev=", 5)) dev = argv[i] + 5; if (!strncmp(argv[i], "+lut=", 5)) lut_mode = atoi(argv[i] + 5);
    }
    if (!apph_load(apph, diemap, dev)) { printf("RESULT: FAIL\n"); return 1; }
    dut = new Vm68k_ustore;
    dut->clk = 0; dut->rst_n = 0; dut->en = 1; dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->a0_sub = 0; dut->a0 = 0; dut->a0s_sub = 0; dut->a0s = 0; dut->c1c0 = 0;
    tick(); tick(); dut->rst_n = 1; dut->eval();
    // Test A
    std::vector<Word> nwW(1024); std::vector<int> nidxW(1024, 0); int n_def = 0, n_undef = 0, n_nonzero = 0;
    std::map<std::string, std::vector<int>> groups;
    for (int W = 0; W < 1024; W++) {
        if (!goto_word(W)) { printf("RESULT: FAIL\n"); return 1; }
        nwW[W] = words(dut->nw); nidxW[W] = dut->nidx;
        if (apph_defined(W)) { n_def++; groups[apph_canon(W)].push_back(W); if (!zero(nwW[W])) n_nonzero++; }
        else { n_undef++; CHECK(zero(nwW[W]), "address %03x undefined (neither App. H nor deviations) with non-zero nanoword %s", W, hex(nwW[W]).c_str()); }
        if (fails > 20) break;
    }
    // Test S: same expected content -> same nanoword; equal words with different content: informative
    int n_groups = 0, n_shared = 0, n_same_word_diff = 0;
    for (auto& kv : groups) {
        if (kv.second.size() < 2) continue;
        n_groups++; n_shared += (int)kv.second.size();
        int a0 = kv.second[0];
        for (size_t i = 1; i < kv.second.size(); i++) CHECK(nwW[kv.second[i]] == nwW[a0], "same expected content (%s, %s) but different nanowords: %03x %s / %03x %s", apph_label(a0).c_str(), apph_label(kv.second[i]).c_str(), a0, hex(nwW[a0]).c_str(), kv.second[i], hex(nwW[kv.second[i]]).c_str());
    }
    { std::map<std::string, std::string> seen; for (int W = 0; W < 1024; W++) { if (!apph_defined(W)) continue; std::string k = hex(nwW[W]), c = apph_canon(W); auto it = seen.find(k); if (it == seen.end()) seen[k] = c; else if (it->second != c) n_same_word_diff++; } }
    // Test C: immediate override against the normal path
    for (int W : {0, 3}) {
        dut->a0s_sub = 1; dut->a0s = W; adv(1); dut->a0s_sub = 0;
        CHECK(dut->upc == W, "immediate override: upc %03x expected %03x", dut->upc, W);
        CHECK(words(dut->nw) == nwW[W], "immediate override at %03x: nanoword %s differs from the normal path %s", W, hex(words(dut->nw)).c_str(), hex(nwW[W]).c_str());
        CHECK(dut->nidx == nidxW[W], "immediate override at %03x: nidx %d differs from the normal path %d", W, (int)dut->nidx, nidxW[W]);
    }
    // Test B: random walk
    std::mt19937 rng(1); long n_disp = 0; const int NWALK = 200000;
    for (int i = 0; i < NWALK && fails <= 20; i++) {
        dut->c1c0 = rng() & 3; dut->a1 = rng() & 1023; dut->a2 = rng() & 1023; dut->a3 = rng() & 1023;
        if (is_dispatch()) n_disp++;
        adv(1);
        int u = dut->upc;
        if (words(dut->nw) != nwW[u] || (int)dut->nidx != nidxW[u]) CHECK(false, "random walk step %d: upc %03x nanoword %s nidx %d, in test A %s / %d", i, u, hex(words(dut->nw)).c_str(), (int)dut->nidx, hex(nwW[u]).c_str(), nidxW[u]); else checks++;
    }
    dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->c1c0 = 0;
    printf("NSTORE: defined addresses %d (non-zero nanoword %d), unused %d (zero nanoword) | shared contents: %d groups over %d addresses | equal words with different expected content %d (informative) | early read (nw_pre, nwl_pre) verified at %ld en edges | override 000/003 | random walk %d steps (%ld dispatch) | microcycles %ld | check %d | FAIL %d\n",
           n_def, n_nonzero, n_undef, n_groups, n_shared, n_same_word_diff, n_en_checks, NWALK, n_disp, n_steps, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
