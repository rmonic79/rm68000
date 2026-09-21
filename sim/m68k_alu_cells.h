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

// m68k_alu_cells.h - FIG. 17 cells read at run time from the literal transcription docs/data/mc68000_alu_matrix_fig17.tsv
// + hand-written C++ model of the ALU/CC (FIG. 16 operations, CC codes from the text col. 26-32, docs/silicon-datapath.md §9.3-9.4).
// Shared by the ALU testbenches (tb_m68k_alu_si.cpp). No generated file is used.
// Cells of the sheet: "OP CCCCC" (same CC code for initiate and finish), "OP CCCCC / 1 CCCCC" (initiate / finish), "OP" (no
// code: KKKKK), "-" (no operation). Names on the sheet: EDR = EOR (typo), RDL/RDR = ROL/ROR, SLAA_X = SLAA (= LSS, §9.3),
// EXT. Tokens per position X N Z V C: K D 0 1 N Z V C ~C C* ~C* A V' X. Column 6 = column 4 with 1 in the incoming bit (§9.3).
// Row 0 (no row from the decoder) is not on the sheet: literally no operation; the cells the RTL puts there (column 1 AND,
// column 5 EXT) enter as documented deviations.
// Documented deviations from the literal data, optional (file +dev=): lines "row<TAB>column<TAB>OP<TAB>CC init<TAB>CC fin<TAB>source".
#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <fstream>
#include <sstream>

struct Cell { std::string op = "NONE"; std::string ini[5] = {"K", "K", "K", "K", "K"}, fin[5] = {"K", "K", "K", "K", "K"}; };
static Cell CELLS[16][6];   // [row 0..15][column 0..5]; column 0 = no operation

static std::string cells_trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t");
    return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}
static bool cells_read_lines(const char* path, std::vector<std::string>& lines) {
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
// "KNZ0~C" -> 5 tokens (K N Z 0 ~C); false if they are not exactly 5 valid tokens
static bool cc_tokens(const std::string& s, std::string out[5]) {
    int n = 0; size_t i = 0;
    while (i < s.size()) {
        if (n == 5) return false;
        std::string t;
        if (s[i] == '~') { if (i + 1 >= s.size() || s[i + 1] != 'C') return false; t = "~C"; i += 2; if (i < s.size() && s[i] == '*') { t += "*"; i++; } }
        else if (s[i] == 'C') { t = "C"; i++; if (i < s.size() && s[i] == '*') { t = "C*"; i++; } }
        else if (s[i] == 'V') { t = "V"; i++; if (i < s.size() && s[i] == '\'') { t = "V'"; i++; } }
        else if (strchr("KD01NZAX", s[i])) { t = std::string(1, s[i]); i++; }
        else return false;
        out[n++] = t;
    }
    return n == 5;
}
static const std::set<std::string> CELL_OPS = {"AND", "OR", "EOR", "ADD", "ADDX", "ADD1", "SUB", "SUBX", "SUB0", "EXT", "LSS", "LSR", "LSL", "ASR", "ASL", "ROL", "ROR", "ROXL", "ROXR", "SLAA"};
static bool parse_cell(const std::string& txt, Cell& c, std::string& err) {
    c = Cell();
    std::string s = cells_trim(txt);
    if (s == "-") return true;
    std::vector<std::string> p; { std::istringstream is(s); std::string w; while (is >> w) p.push_back(w); }
    if (p.empty()) { err = "empty cell"; return false; }
    static const std::map<std::string, std::string> alias = {{"EDR", "EOR"}, {"RDL", "ROL"}, {"RDR", "ROR"}, {"SLAA_X", "SLAA"}};
    std::string op = p[0]; auto a = alias.find(op); if (a != alias.end()) op = a->second;
    if (!CELL_OPS.count(op)) { err = "unknown operation '" + p[0] + "'"; return false; }
    c.op = op;
    if (p.size() == 1) return true;
    if (!cc_tokens(p[1], c.ini)) { err = "invalid CC code '" + p[1] + "'"; return false; }
    if (p.size() == 2) { for (int k = 0; k < 5; k++) c.fin[k] = c.ini[k]; return true; }
    if (p.size() == 5 && p[2] == "/" && p[3] == "1") { if (!cc_tokens(p[4], c.fin)) { err = "invalid finish CC code '" + p[4] + "'"; return false; } return true; }
    err = "unrecognised cell form"; return false;
}
// literal transcription: row<TAB>col1..col5<TAB>instructions; rows 1..15 once each
static bool load_fig17(const char* path) {
    std::vector<std::string> lines; if (!cells_read_lines(path, lines)) return false;
    std::set<int> rows;
    for (const std::string& line : lines) {
        if (line.rfind("row", 0) == 0) continue;   // header
        std::vector<std::string> f; { std::istringstream is(line); std::string c; while (std::getline(is, c, '\t')) f.push_back(c); }
        if (f.size() < 6) { printf("FAIL: FIG. 17 line with %d fields: '%s'\n", (int)f.size(), line.c_str()); return false; }
        int r = atoi(f[0].c_str());
        if (r < 1 || r > 15 || rows.count(r)) { printf("FAIL: FIG. 17 row %d invalid or repeated\n", r); return false; }
        rows.insert(r);
        for (int c = 1; c <= 5; c++) { std::string err; if (!parse_cell(f[c], CELLS[r][c], err)) { printf("FAIL: row %d column %d: %s\n", r, c, err.c_str()); return false; } }
    }
    if (rows.size() != 15) { printf("FAIL: FIG. 17 rows read %d, expected 15\n", (int)rows.size()); return false; }
    return true;
}
static int ncelldev = 0;
static bool load_cell_dev(const char* path) {
    std::vector<std::string> lines; if (!cells_read_lines(path, lines)) return false;
    for (const std::string& line : lines) {
        std::vector<std::string> f; { std::istringstream is(line); std::string c; while (std::getline(is, c, '\t')) f.push_back(c); }
        if (f.size() < 5) { printf("FAIL: incomplete deviation line: '%s'\n", line.c_str()); return false; }
        int r = atoi(f[0].c_str()), c = atoi(f[1].c_str());
        if (r < 0 || r > 15 || c < 1 || c > 5) { printf("FAIL: deviation outside the table: '%s'\n", line.c_str()); return false; }
        Cell cell; std::string err;
        if (!parse_cell(cells_trim(f[2]) + " " + cells_trim(f[3]) + " / 1 " + cells_trim(f[4]), cell, err)) { printf("FAIL: deviation r%d c%d: %s\n", r, c, err.c_str()); return false; }
        CELLS[r][c] = cell; ncelldev++;
        printf("deviation: r%d c%d = %s %s%s%s%s%s / %s%s%s%s%s (%s)\n", r, c, cell.op.c_str(), cell.ini[0].c_str(), cell.ini[1].c_str(), cell.ini[2].c_str(), cell.ini[3].c_str(), cell.ini[4].c_str(),
               cell.fin[0].c_str(), cell.fin[1].c_str(), cell.fin[2].c_str(), cell.fin[3].c_str(), cell.fin[4].c_str(), f.size() > 5 ? f[5].c_str() : "no source");
    }
    return true;
}
static const Cell& cell_of(int row, int col) {   // column 6 = column 4; column 0 or outside the table = no operation
    static const Cell NONE_CELL;
    int colx = col == 6 ? 4 : col;
    if (row < 0 || row > 15 || colx < 1 || colx > 5) return NONE_CELL;
    return CELLS[row][colx];
}
// encoding of the 45-bit entry (interface of m68k_alu_pkg.sv): {op[4:0], init X N Z V C (5x4), finish X N Z V C (5x4)}
static int op_code(const std::string& op) {
    static const char* OPS[] = {"NONE", "AND", "OR", "EOR", "ADD", "ADDX", "ADD1", "SUB", "SUBX", "SUB0", "EXT", "LSS", "LSR", "LSL", "ASR", "ASL", "ROL", "ROR", "ROXL", "ROXR", "SLAA"};
    for (int i = 0; i < 21; i++) if (op == OPS[i]) return i;
    return 0;
}
static int tok_code(const std::string& t) {
    static const char* TOKS[] = {"K", "D", "0", "N", "Z", "V", "C", "~C", "C*", "~C*", "A", "V'", "1", "X"};
    for (int i = 0; i < 14; i++) if (t == TOKS[i]) return i;
    return 0;
}
static uint64_t cell_entry(const Cell& c) {
    uint64_t e = (uint64_t)op_code(c.op) << 40;
    for (int k = 0; k < 5; k++) { e |= (uint64_t)tok_code(c.ini[k]) << (36 - 4 * k); e |= (uint64_t)tok_code(c.fin[k]) << (16 - 4 * k); }
    return e;
}

// --- ALU/CC model (hand-written; C* = input PSW C OR carry of the operation, patent col. 30)
struct AluOut { uint16_t r, alue; int cc; bool valid; int carry, hc; bool is_sub, adder, pair; int cout; };
static inline AluOut alu_model(int row, int col, bool init, bool fin, bool byte_op, int sz, uint16_t a, uint16_t d, uint16_t alue, int cc, bool c_as_x = false, bool cl = false, bool mul_signed = false, bool shift_op = false) {
    AluOut o{a, alue, cc, false, 0, 0, false, false, false, 0};
    const Cell& c = cell_of(row, col);
    std::string op = c.op; if (op == "NONE") return o;
    o.valid = true;
    o.is_sub = op == "SUB" || op == "SUBX" || op == "SUB0";
    o.adder = o.is_sub || op == "ADD" || op == "ADDX" || op == "ADD1";
    int X = (cc >> 4) & 1, Cin = cc & 1;
    uint32_t mask = byte_op ? 0xff : 0xffff; int msb = byte_op ? 7 : 15;
    bool pair32 = false;
    uint32_t res = 0; int carry = 0, ovf = 0, shc = 0, hc = 0;
    uint16_t alue_o = alue;
    auto bit = [](uint32_t v, int b) { return (int)((v >> b) & 1); };
    if (op == "AND") res = a & d; else if (op == "OR") res = a | d; else if (op == "EOR") res = a ^ d;
    else if (op == "ADD" || op == "ADDX" || op == "ADD1") {
        int Xc = init ? X : (int)cl;   // 'initiate' = X of the PSW, otherwise the carry latch (also in 'n' mode: DIVS)
        int cin = op == "ADDX" ? Xc : op == "ADD1" ? 1 : 0;
        uint32_t s = (a & mask) + (d & mask) + cin; res = s & 0xffff; carry = bit(s, msb + 1);
        ovf = bit(a, msb) == bit(d, msb) && bit(res, msb) != bit(a, msb);
        hc = (((a & 15) + (d & 15) + cin) >> 4) & 1;
    } else if (op == "SUB" || op == "SUBX" || op == "SUB0") {
        int Xc = init ? X : (int)cl;
        int cin = op == "SUB" ? 1 : op == "SUBX" ? !Xc : 0;
        uint32_t s = (d & mask) + ((~a) & mask) + cin; res = s & 0xffff; carry = bit(s, msb + 1);   // carry-out of the adder (the ~C codes negate it)
        ovf = bit(a, msb) != bit(d, msb) && bit(res, msb) != bit(d, msb);
        hc = !((((d & 15) + ((~a) & 15) + cin) >> 4) & 1);   // half-borrow
    } else if (op == "EXT") { res = (a & 0x80) ? (0xff00 | (a & 0xff)) : (a & 0xff); }
    else {
        bool left = op == "LSL" || op == "ASL" || op == "ROL" || op == "ROXL" || op == "LSS" || op == "SLAA";
        int width = sz == 2 ? 32 : sz == 1 ? 16 : 8;
        bool alu_high = c_as_x || (op == "SLAA" && row == 1);   // mul/div: pair {ALU = high, ALUE = low} (App. H mulm6, dvum)
        if (alu_high) width = 32;
        uint64_t v = alu_high ? (((uint32_t)a << 16) | alue) : sz == 2 ? (((uint32_t)alue << 16) | a) : sz == 1 ? a : (a & 0xff);   // 32-bit pair: ALUE = high word, ALU = low word (App. H srrl2/srrl4); mul/div the other way round
        int out = left ? bit(v, width - 1) : bit(v, 0);
        int in = 0;
        int Xsh = c_as_x ? (mul_signed ? (bit(cc, 3) ^ bit(cc, 1)) : Cin) : X;   // MULS: true sign N^V; MULU: carry C
        if (op == "ROL") in = out; else if (op == "ROR") in = out; else if (op == "ROXL" || op == "ROXR") in = Xsh;
        else if (op == "ASR") in = bit(v, width - 1); else if (op == "LSS" || op == "SLAA") in = (col == 6);
        uint64_t nv = left ? ((v << 1) | in) & ((1ull << width) - 1) : ((v >> 1) | ((uint64_t)in << (width - 1)));
        if (alu_high) { res = (nv >> 16) & 0xffff; alue_o = nv & 0xffff; pair32 = true; }
        else if (sz == 2) { res = nv & 0xffff; alue_o = (nv >> 16) & 0xffff; pair32 = true; }
        else if (sz == 1) res = nv & 0xffff;
        else res = (a & 0xff00) | (nv & 0xff);
        shc = out; carry = out;
        if (c_as_x) { shc = 0; carry = 0; }   // C 'A' of the MUL ROXR cell = 0: a shift without an intermediate sum shifts in 0
        if (op == "ASL") ovf = out ^ (sz == 2 ? bit(alue_o, 15) : bit(res, sz == 0 ? 7 : 15));
    }
    uint16_t r = (uint16_t)res;   // 16-bit result even for bytes (die model: the high byte is kept by the regfile)
    o.r = r; o.alue = alue_o; o.carry = carry; o.hc = hc; o.pair = pair32; o.cout = o.adder ? carry : 0;
    if (init || fin) {
        const std::string* tk = (fin && !init) ? c.fin : c.ini;
        bool bcd = tk[4] == "C*" || tk[4] == "~C*";   // decimal cell: Z keeps the previous Z, C*/~C* use X
        bool hi_alu = c_as_x || (op == "SLAA" && row == 1);
        int n = pair32 ? (hi_alu ? bit(r, 15) : bit(alue_o, 15)) : bit(r, msb), z = ((r & mask) == 0) && (!pair32 || (fin && !init) || alue_o == 0), out = cc;   // pair in finish (MUL): Z without the ALUE half; 32-bit shift: N/Z from the pair
        for (int k = 0; k < 5; k++) {
            const std::string& t = tk[k]; int b = 4 - k; int nv = bit(cc, b);
            if (t == "K") nv = bit(cc, b); else if (t == "D") nv = (b == 3) ? n : (b == 1) ? ovf : (b == 0) ? carry : bit(cc, b); else if (t == "0") nv = 0; else if (t == "1") nv = 1; else if (t == "N") nv = n; else if (t == "Z") nv = (z) && (((fin && !init) || bcd || op == "ADDX" || op == "SUBX") ? (((cc >> 2) & 1) != 0) : true);
            else if (t == "V") nv = ovf; else if (t == "C") nv = carry; else if (t == "C*") nv = carry | X; else if (t == "~C") nv = !carry; else if (t == "~C*") nv = !carry | X;
            else if (t == "A") nv = shc; else if (t == "X") nv = shift_op ? X : 0; /* only shift/rotate (ROXd with count 0); EXT.L: 0 */ else if (t == "V'") nv = ((cc >> 1) & 1) | (n ^ carry);
            out = (out & ~(1 << b)) | (nv << b);
        }
        o.cc = out;
    }
    return o;
}
