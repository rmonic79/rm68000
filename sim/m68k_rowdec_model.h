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

// m68k_rowdec_model.h - independent model of the ALU/CC row decoder (FIG. 18) and of the cc_inh rule, shared by the testbenches
// (tb_m68k_rowdec.cpp, tb_m68k_alux_si.cpp). The terms of FIG. 18 are read at run time from the literal transcription
// docs/data/mc68000_alu_rows_fig18.tsv; the cc_inh rule comes from the manual (MC68000UM, MOVEA / ADDA / SUBA / ADDQ / SUBQ with
// destination An: "Condition Codes: Not affected"). No generated file (*_expect.h, *.mem, .sv) is used.
// Rules (docs/silicon-datapath.md §9.1, LITERAL table): a term "N" is active if IRD satisfies its bit product
// (n = bit at 1, n' = bit at 0) and none of the "(k) i" terms that follow it in the table is satisfied (a "(k) i" carries the
// product of term k); an active term inhibits the listed rows; the line "(30) i ... inhibit only" defines term
// 30, which inhibits no rows. row_n[r-1] = 1 for the rows NOT inhibited; row = the remaining row, the lowest if more than one
// remains (RTL convention), 0 if no row is inhibited or if all of them are. ird[2:0] does not appear in FIG. 18.
// Documented deviations from the literal data, optional (file +dev=), one per line:
//   fix<TAB>N<TAB>bit product<TAB>inhibited rows<TAB>source   replaces product and rows of term N
//   add<TAB>bit product<TAB>inhibited rows<TAB>source         adds a term (next index)
//   qual<TAB>N<TAB>bit product<TAB>source                     adds a qualifier to term N
// Shape checks on the literal transcription (false if they fail): 45 terms in sequence 0..44, 13 qualifiers, every
// "(k) i" with the product of term k.
#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

struct RPat { uint32_t mask = 0, val = 0; };
struct RTerm { int idx = 0; RPat p; uint16_t inh = 0; std::vector<RPat> q; std::vector<int> qk; };
static std::vector<RTerm> RTERMS;
static int rowdec_nq = 0, rowdec_ndev = 0;

static bool rowdec_read_lines(const char* path, std::vector<std::string>& lines) {
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
static std::vector<std::string> rowdec_fields(const std::string& line, size_t n) {
    std::vector<std::string> f; std::istringstream is(line); std::string c;
    while (std::getline(is, c, '\t')) f.push_back(c);
    while (f.size() < n) f.push_back("");
    return f;
}
// "15 14 13' 12" -> mask and value; false if it is not a valid bit product (bits 0..15, no repeated bit)
static bool rowdec_parse_pat(const std::string& s, RPat& p) {
    p = RPat(); std::istringstream is(s); std::string tok;
    while (is >> tok) {
        bool neg = false;
        if (tok.back() == '\'') { neg = true; tok.pop_back(); }
        if (tok.empty() || tok.find_first_not_of("0123456789") != std::string::npos) return false;
        int b = atoi(tok.c_str());
        if (b > 15 || (p.mask >> b & 1)) return false;
        p.mask |= 1u << b; if (!neg) p.val |= 1u << b;
    }
    return p.mask != 0;
}
static bool rowdec_parse_inh(const std::string& s, uint16_t& inh) {
    inh = 0; std::istringstream is(s); int r;
    while (is >> r) { if (r < 1 || r > 15) return false; inh |= (uint16_t)(1u << (r - 1)); }
    return true;
}
// literal transcription (fields: index, decoding, instruction, inhibited rows)
static bool rowdec_load(const char* path) {
    std::vector<std::string> lines; if (!rowdec_read_lines(path, lines)) return false;
    RTERMS.clear(); rowdec_nq = 0; rowdec_ndev = 0;
    for (size_t ln = 0; ln < lines.size(); ln++) {
        std::vector<std::string> fld = rowdec_fields(lines[ln], 4);
        RPat p;
        if (fld[0].empty() || !rowdec_parse_pat(fld[1], p)) { printf("FAIL: invalid FIG. 18 line: '%s'\n", lines[ln].c_str()); return false; }
        if (fld[0][0] == '(') {
            int k = atoi(fld[0].c_str() + 1);
            if (fld[2].find("inhibit only") != std::string::npos) {   // "(30) i ... inhibit only": defines term k, with no inhibited rows
                RTerm t; t.idx = k; t.p = p;
                if (t.idx != (int)RTERMS.size()) { printf("FAIL: FIG. 18: term %d out of sequence\n", t.idx); return false; }
                RTERMS.push_back(t);
            } else {                                                  // "(k) i": qualifier of the preceding term
                if (RTERMS.empty()) { printf("FAIL: FIG. 18: qualifier without a term\n"); return false; }
                RTERMS.back().q.push_back(p); RTERMS.back().qk.push_back(k); rowdec_nq++;
            }
        } else {
            RTerm t; t.idx = atoi(fld[0].c_str()); t.p = p;
            if (t.idx != (int)RTERMS.size()) { printf("FAIL: FIG. 18: term %d out of sequence\n", t.idx); return false; }
            if (!rowdec_parse_inh(fld[3], t.inh)) { printf("FAIL: FIG. 18: invalid inhibited rows in '%s'\n", lines[ln].c_str()); return false; }
            RTERMS.push_back(t);
        }
    }
    bool ok = true;
    if (RTERMS.size() != 45) { printf("FAIL: FIG. 18: terms read %d, expected 45\n", (int)RTERMS.size()); ok = false; }
    if (rowdec_nq != 13) { printf("FAIL: FIG. 18: qualifiers read %d, expected 13\n", rowdec_nq); ok = false; }
    for (const RTerm& t : RTERMS)
        for (size_t j = 0; j < t.q.size(); j++) {
            int k = t.qk[j];
            if (k < 0 || k >= (int)RTERMS.size() || RTERMS[k].p.mask != t.q[j].mask || RTERMS[k].p.val != t.q[j].val) { printf("FAIL: FIG. 18: qualifier (%d) i of term %d does not carry the product of term %d\n", k, t.idx, k); ok = false; }
        }
    return ok;
}
static bool rowdec_load_dev(const char* path) {
    std::vector<std::string> lines; if (!rowdec_read_lines(path, lines)) return false;
    for (const std::string& line : lines) {
        std::vector<std::string> f = rowdec_fields(line, 5);
        RPat p; std::string what = f[0];
        if (what == "fix") {
            int n = atoi(f[1].c_str()); uint16_t inh;
            if (n < 0 || n >= (int)RTERMS.size() || !rowdec_parse_pat(f[2], p) || !rowdec_parse_inh(f[3], inh)) { printf("FAIL: invalid fix deviation: '%s'\n", line.c_str()); return false; }
            RTERMS[n].p = p; RTERMS[n].inh = inh;
            printf("deviation: term %d = %s | inhibits %s (%s)\n", n, f[2].c_str(), f[3].c_str(), f[4].c_str());
        } else if (what == "add") {
            RTerm t; uint16_t inh;
            if (!rowdec_parse_pat(f[1], p) || !rowdec_parse_inh(f[2], inh)) { printf("FAIL: invalid add deviation: '%s'\n", line.c_str()); return false; }
            t.idx = (int)RTERMS.size(); t.p = p; t.inh = inh; RTERMS.push_back(t);
            printf("deviation: new term %d = %s | inhibits %s (%s)\n", t.idx, f[1].c_str(), f[2].c_str(), f[3].c_str());
        } else if (what == "qual") {
            int n = atoi(f[1].c_str());
            if (n < 0 || n >= (int)RTERMS.size() || !rowdec_parse_pat(f[2], p)) { printf("FAIL: invalid qual deviation: '%s'\n", line.c_str()); return false; }
            RTERMS[n].q.push_back(p); RTERMS[n].qk.push_back(-1);
            printf("deviation: qualifier of term %d = %s (%s)\n", n, f[2].c_str(), f[3].c_str());
        } else { printf("FAIL: unknown deviation: '%s'\n", line.c_str()); return false; }
        rowdec_ndev++;
    }
    return true;
}
// model: rows inhibited by the active terms; row_n = remaining rows; row = the lowest remaining, 0 if none or all are inhibited
static void rowdec_model(uint32_t ird, int& row_n, int& row, std::vector<int>* act = nullptr) {
    int inh = 0;
    for (size_t i = 0; i < RTERMS.size(); i++) {
        const RTerm& t = RTERMS[i];
        if ((ird & t.p.mask) != t.p.val) continue;
        bool dis = false;
        for (const RPat& q : t.q) if ((ird & q.mask) == q.val) { dis = true; break; }
        if (dis) continue;
        if (act) (*act)[i]++;
        inh |= t.inh;
    }
    row_n = (~inh) & 0x7fff;
    row = 0;
    if (row_n != 0x7fff && row_n != 0) for (int k = 0; k < 15; k++) if (row_n >> k & 1) { row = k + 1; break; }
}
static int rowdec_row(uint32_t ird) { int rn, r; rowdec_model(ird, rn, r); return r; }
// cc_inh: destination An, condition codes not altered (MC68000UM, MOVEA / ADDA / SUBA / ADDQ / SUBQ: "Condition Codes: Not affected")
static bool rowdec_cc_inh(uint32_t ird) {
    uint32_t op = ird >> 12 & 0xf, opmode = ird >> 6 & 7;
    if ((op == 0x2 || op == 0x3) && opmode == 1) return true;                    // MOVEA.L / MOVEA.W: destination mode 001
    if ((op == 0xD || op == 0x9) && (opmode == 3 || opmode == 7)) return true;   // ADDA / SUBA: opmode 011 (word) / 111 (long)
    if (op == 0x5 && (ird >> 6 & 3) != 3 && (ird >> 3 & 7) == 1) return true;   // ADDQ / SUBQ with EA = An (size 11 = Scc/DBcc)
    return false;
}
