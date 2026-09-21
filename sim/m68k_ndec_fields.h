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

// m68k_ndec_fields.h - fields of the per-resource nano decoder, read at run time from rtl/cpu_si/m68k_nano_dec_pkg.sv
// (the decoder interface): "localparam int ND_X_LSB = n, ND_X_W = w, ND_X_ONEHOT = o;  // prefix" and
// "localparam int ND_X__V = k;  // atom". Enumerated: value k active <=> dec[LSB +: W] == k; one-hot: bit k of dec[LSB +: W].
// From a `dec` vector (32-bit words, bit 0 first) it derives the set of active atoms of the microword. Shared by the
// testbenches that read the commands from the real control store (alux, bussec, regs, regfile). No generated file of expected values is used.
#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <fstream>
#include <sstream>

struct NdField { std::string name, prefix; int lsb = 0, w = 0, onehot = 0; std::map<int, std::string> atoms; };
static std::vector<NdField> ND_FIELDS; static std::map<std::string, int> ND_FIDX; static int ND_W_PKG = 0;
static std::string nd_trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t");
    return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}
static bool nd_load_pkg(const char* path) {
    std::ifstream f(path); if (!f) { printf("FAIL: cannot open %s\n", path); return false; }
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        line = nd_trim(line);
        if (line.rfind("localparam int NDEC_W = ", 0) == 0) { ND_W_PKG = atoi(line.c_str() + 24); continue; }
        if (line.rfind("localparam int ND_", 0) != 0) continue;
        size_t c = line.find("//"); std::string cmt = c == std::string::npos ? "" : nd_trim(line.substr(c + 2)), body = line.substr(0, c);
        if (body.find("_LSB = ") != std::string::npos) {
            size_t a = body.find("ND_") + 3, b = body.find("_LSB"); NdField fd; fd.name = body.substr(a, b - a); fd.prefix = cmt;
            fd.lsb = atoi(body.c_str() + body.find("_LSB = ") + 7); fd.w = atoi(body.c_str() + body.find("_W = ") + 5); fd.onehot = atoi(body.c_str() + body.find("_ONEHOT = ") + 10);
            ND_FIDX[fd.name] = (int)ND_FIELDS.size(); ND_FIELDS.push_back(fd);
        } else if (body.find("__") != std::string::npos) {
            size_t a = body.find("ND_") + 3, b = body.find("__"); std::string name = body.substr(a, b - a);
            auto it = ND_FIDX.find(name); if (it == ND_FIDX.end()) { printf("FAIL: pkg: value of an unknown field: '%s'\n", line.c_str()); return false; }
            ND_FIELDS[it->second].atoms[atoi(body.c_str() + body.find(" = ") + 3)] = cmt;
        }
    }
    if (ND_FIELDS.empty() || ND_W_PKG <= 0) { printf("FAIL: pkg without fields\n"); return false; }
    return true;
}
static uint32_t nd_bits(const uint32_t* dec, int lsb, int w) { uint32_t v = 0; for (int i = 0; i < w; i++) v |= ((dec[(lsb + i) >> 5] >> ((lsb + i) & 31)) & 1u) << i; return v; }
static std::set<std::string> nd_atoms(const uint32_t* dec) {
    std::set<std::string> A;
    for (const NdField& f : ND_FIELDS) {
        uint32_t v = nd_bits(dec, f.lsb, f.w);
        if (f.onehot) { for (int b = 0; b < f.w; b++) if (v >> b & 1) { auto it = f.atoms.find(b); if (it != f.atoms.end()) A.insert(it->second); } }
        else if (v) { auto it = f.atoms.find((int)v); if (it != f.atoms.end()) A.insert(it->second); }
    }
    return A;
}
