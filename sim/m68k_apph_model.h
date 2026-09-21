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

// m68k_apph_model.h - INDEPENDENT model of the nano ROM, from patent US 4,325,121 App. H (docs/data/us4325121_appH.tsv,
// transcription of the microword blocks: access/alu/regptr columns and transfers) and from the App. H -> die map
// (docs/data/us4325121_appH_die_map.tsv: micro address of each block). No generated file is used.
// For each block the atoms (the vocabulary is that of docs/silicon-datapath.md §4.1) are derived as follows:
//  - access (§17.2): 'np' -> acc_class=n acc_mode=p; 'ak' -> acc=ak; 4 letters -> acc_class=c acc_mode=m acc_type=tt;
//  - alu (§9.3): x -> nothing; first character = column ('i' is the spelling of column 1), second = alu_cc (i/f/n), third 'f' -> alu_byte;
//  - regptr: 4 letters -> rp_dst=xx rp_src=yy;
//  - transfers: "src -> d1,d2" -> d<-src (constants: d+=k); "src -> bus -> d1,d2" -> bus<-src and d<-bus; 'idle wait', 'reset pren',
//    'sr c-alu-alue' literal.
// Declared rules (microcode-structure.md section 5; silicon-datapath.md §4.3 and §7):
//  R1 '*' without prefix = AB or DB, decided per word from the die trace: the transfer must exist on ONE of the two buses,
//     with the section form of the driver free ('', '*', 'e', 'd', 'h', 'l') and the destinations on the same bus (the literal form
//     '*e<-src' / 'dst<-*e' is accepted too);
//  R2 the AU reads the whole bus: 'au<-dbh', 'au<-dbl', 'au<-dbe' (and AB) also count as 'au<-db' ('au<-ab');
//  R3 AU loaded without an explicit constant = constant 0 ('au+=0': the AU always adds a constant).
// Deviations (optional file +dev, one per line: addr<TAB>op<TAB>argument<TAB>source): add = extra atoms | del = missing atoms |
// deltr = '*' transfer absent | set = whole word | copy = word equal to the block with that label [+ atoms after '+'].
// An address is DEFINED if it has a block in the die map or a deviation; the others must have an empty nanoword.
#pragma once
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

struct ApphBlk { std::string label, origin, access, type, alu, regptr, marker, nxt, transfers, note; int pat = -1; };
struct ApphTol {
    std::string src, suf; std::vector<std::string> dsts;
    std::string key() const { std::string k = src + "->*" + suf + "->"; for (size_t i = 0; i < dsts.size(); i++) k += (i ? "," : "") + dsts[i]; return k; }
};
struct ApphModel { std::set<std::string> A; std::vector<std::set<std::string>> ALT; std::vector<ApphTol> T; };
struct ApphDev { std::string op, arg, src; };
static std::vector<ApphBlk> APPH_BLKS; static std::map<std::string, int> APPH_BYL; static std::map<int, int> APPH_WORD;   // die address -> block
static std::map<int, std::vector<ApphDev>> APPH_DEVS; static std::map<std::string, int> APPH_UNMAPPED;
static long apph_n_r2 = 0, apph_n_r3 = 0, apph_n_tol_ab = 0, apph_n_tol_db = 0, apph_n_tol_lit = 0, apph_n_devops = 0, apph_n_dup = 0;
static const std::set<std::string> APPH_CONST = {"+1", "+2", "+4", "-1", "-2", "-4", "0", "-1,-2", "+1,+2"};
static const char* const APPH_SUF[6] = {"", "*", "e", "d", "h", "l"};

static std::string apph_trim(const std::string& s) { size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n"); return a == std::string::npos ? "" : s.substr(a, b - a + 1); }
static std::vector<std::string> apph_split(const std::string& s, const std::string& sep) {
    std::vector<std::string> out; size_t p = 0;
    for (;;) { size_t q = s.find(sep, p); if (q == std::string::npos) { out.push_back(s.substr(p)); return out; } out.push_back(s.substr(p, q - p)); p = q + sep.size(); }
}
static std::set<std::string> apph_atomset(const std::string& s) { std::set<std::string> o; for (auto& x : apph_split(s, ";")) { std::string t = apph_trim(x); if (!t.empty()) o.insert(t); } return o; }
static std::string apph_join(const std::set<std::string>& s) { std::string o; for (auto& x : s) { if (!o.empty()) o += ";"; o += x; } return o; }
static bool apph_hex3(const std::string& s0, int& v) { std::string s = apph_trim(s0); if (s.size() != 3) return false; char* e; long x = strtol(s.c_str(), &e, 16); if (*e) return false; v = (int)x; return true; }
static bool apph_defined(int addr) { return APPH_WORD.count(addr) > 0 || APPH_DEVS.count(addr) > 0; }
static std::string apph_label(int addr) { auto it = APPH_WORD.find(addr); return it == APPH_WORD.end() ? std::string("-") : APPH_BLKS[it->second].label; }

static bool apph_load(const char* apph, const char* diemap, const char* dev) {
    std::ifstream f(apph); if (!f) { printf("FAIL: App. H not readable: %s\n", apph); return false; }
    std::string l;
    while (std::getline(f, l)) {
        if (!l.empty() && l.back() == '\r') l.pop_back();
        if (l.empty() || l[0] != 'p') continue;
        std::vector<std::string> c = apph_split(l, "\t");
        if (c.size() < 11) { printf("FAIL: App. H line with %d fields: %s\n", (int)c.size(), l.c_str()); return false; }
        ApphBlk b; b.label = c[2]; b.origin = c[3]; b.access = c[4]; b.type = c[5]; b.alu = c[6]; b.regptr = c[7]; b.marker = c[8]; b.nxt = c[9]; b.transfers = c[10]; b.note = c.size() > 11 ? c[11] : "";
        apph_hex3(c[1], b.pat);
        if (APPH_BYL.count(b.label)) { printf("FAIL: duplicate App. H label: %s\n", b.label.c_str()); return false; }
        APPH_BYL[b.label] = (int)APPH_BLKS.size(); APPH_BLKS.push_back(b);
    }
    if (APPH_BLKS.empty()) { printf("FAIL: App. H empty: %s\n", apph); return false; }
    std::ifstream g(diemap); if (!g) { printf("FAIL: die map not readable: %s\n", diemap); return false; }
    int n_map = 0;
    while (std::getline(g, l)) {
        if (!l.empty() && l.back() == '\r') l.pop_back();
        if (l.empty() || l[0] == '#') continue;
        std::vector<std::string> c = apph_split(l, "\t"); if (c.size() < 3) continue;
        auto it = APPH_BYL.find(c[0]); int a; if (it == APPH_BYL.end() || !apph_hex3(c[2], a)) continue;
        if (APPH_WORD.count(a)) { apph_n_dup++; continue; }
        APPH_WORD[a] = it->second; n_map++;
    }
    if (n_map == 0) { printf("FAIL: die map without usable entries: %s\n", diemap); return false; }
    if (dev && *dev) {
        std::ifstream h(dev); if (!h) { printf("FAIL: deviations file not readable: %s\n", dev); return false; }
        while (std::getline(h, l)) {
            if (!l.empty() && l.back() == '\r') l.pop_back();
            if (l.empty() || l[0] == '#') continue;
            std::vector<std::string> c = apph_split(l, "\t"); int a;
            if (c.size() < 3 || !apph_hex3(c[0], a)) { printf("FAIL: invalid deviations line: %s\n", l.c_str()); return false; }
            ApphDev d; d.op = apph_trim(c[1]); d.arg = apph_trim(c[2]); d.src = c.size() > 3 ? c[3] : "";
            if (d.op != "add" && d.op != "del" && d.op != "deltr" && d.op != "set" && d.op != "copy") { printf("FAIL: deviation %03x: unknown op '%s'\n", a, d.op.c_str()); return false; }
            APPH_DEVS[a].push_back(d); apph_n_devops++;
        }
    }
    printf("App. H: %d blocks, %d with a die address (%ld duplicate addresses ignored), deviations %ld on %d addresses\n", (int)APPH_BLKS.size(), n_map, apph_n_dup, apph_n_devops, (int)APPH_DEVS.size());
    return true;
}

static void apph_acc(const std::string& c, std::set<std::string>& A) {
    if (c == "-" || c.empty()) return;
    if (c == "np") { A.insert("acc_class=n"); A.insert("acc_mode=p"); return; }
    if (c == "ak") { A.insert("acc=ak"); return; }
    if (c.size() == 4) { A.insert("acc_class=" + c.substr(0, 1)); A.insert("acc_mode=" + c.substr(1, 1)); A.insert("acc_type=" + c.substr(2)); return; }
    APPH_UNMAPPED["access:" + c]++;
}
static void apph_alu(const std::string& c, std::set<std::string>& A) {
    if (c == "x" || c == "-" || c.empty()) return;
    A.insert(std::string("alu_col=") + (c[0] == 'i' ? std::string("1") : c.substr(0, 1)));
    if (c.size() > 1) A.insert("alu_cc=" + c.substr(1, 1));
    if (c.size() > 2) A.insert("alu_byte");
}
static void apph_regptr(const std::string& c, std::set<std::string>& A) {
    if (c == "-" || c.empty()) return;
    if (c.size() == 4) { A.insert("rp_dst=" + c.substr(0, 2)); A.insert("rp_src=" + c.substr(2)); return; }
    APPH_UNMAPPED["regptr:" + c]++;
}
static ApphModel apph_block_model(const ApphBlk& b, bool count) {
    ApphModel m; apph_acc(b.access, m.A); apph_alu(b.alu, m.A); apph_regptr(b.regptr, m.A);
    bool au_dst = false, au_const = false;
    for (std::string t : apph_split(b.transfers, " ; ")) {
        t = apph_trim(t);
        if (!t.empty() && t.front() == '\'') t = t.substr(1);
        if (!t.empty() && t.back() == '\'') t.pop_back();
        t = apph_trim(t);
        if (t.empty()) continue;
        if (t == "idle wait" || t == "reset pren" || t == "sr c-alu-alue") { m.A.insert(t); continue; }
        std::vector<std::string> tk = apph_split(t, "->"); for (auto& x : tk) x = apph_trim(x);
        if (tk.size() == 2) {
            for (auto d : apph_split(tk[1], ",")) {
                d = apph_trim(d);
                if (APPH_CONST.count(tk[0])) { m.A.insert(d + "+=" + tk[0]); if (d == "au") au_const = true; }
                else { m.A.insert(d + "<-" + tk[0]); if (d == "au") au_dst = true; }
            }
            continue;
        }
        if (tk.size() == 3) {
            std::string bus = tk[1]; std::vector<std::string> ds; for (auto d : apph_split(tk[2], ",")) ds.push_back(apph_trim(d));
            for (auto& d : ds) if (d == "au") au_dst = true;
            bool ok = (bus == "*" || bus == "*e");
            if (!ok && (bus.rfind("ab", 0) == 0 || bus.rfind("db", 0) == 0)) { std::string sf = bus.substr(2); for (auto s : APPH_SUF) if (sf == s) ok = true; }
            if (!ok) { APPH_UNMAPPED["bus:" + bus]++; continue; }
            if (bus[0] == '*') { ApphTol tl; tl.src = tk[0]; tl.suf = bus.substr(1); tl.dsts = ds; m.T.push_back(tl); continue; }
            m.A.insert(bus + "<-" + tk[0]);
            for (auto& d : ds) { std::string a = d + "<-" + bus; if (d == "au" && bus.size() > 2) m.ALT.push_back({a, d + "<-" + bus.substr(0, 2)}); else m.A.insert(a); }   // R2
            continue;
        }
        APPH_UNMAPPED["form:" + t]++;
    }
    if (au_dst && !au_const) { m.A.insert("au+=0"); if (count) apph_n_r3++; }   // R3
    return m;
}
// R1: 0 = literal form, 1 = on AB, 2 = on DB, -1 = absent; in `used` the DUT atoms that implement the transfer
static int apph_tol_match(const ApphTol& t, const std::set<std::string>& S, std::set<std::string>& used) {
    std::set<std::string> lit; lit.insert("*" + t.suf + "<-" + t.src); for (auto& d : t.dsts) lit.insert(d + "<-*" + t.suf);
    bool all = true; for (auto& x : lit) if (!S.count(x)) { all = false; break; }
    if (all) { used.insert(lit.begin(), lit.end()); return 0; }
    for (int bi = 0; bi < 2; bi++) {
        std::string bus = bi ? "db" : "ab";
        for (auto s1 : APPH_SUF) {
            std::string drv = bus + s1 + "<-" + t.src; if (!S.count(drv)) continue;
            std::set<std::string> u = {drv}; bool ok = true;
            for (auto& d : t.dsts) {
                std::string f; for (auto s2 : APPH_SUF) { std::string c = d + "<-" + bus + s2; if (S.count(c)) { f = c; break; } }
                if (f.empty()) { ok = false; break; } u.insert(f);
            }
            if (ok) { used.insert(u.begin(), u.end()); return bi + 1; }
        }
    }
    return -1;
}
static void apph_resolve(const ApphModel& m, const std::set<std::string>& S, const std::set<std::string>& skip, std::set<std::string>& need, std::vector<std::string>& miss_tol) {
    need.insert(m.A.begin(), m.A.end());
    for (auto& alt : m.ALT) { std::string pick; for (auto& x : alt) if (S.count(x)) { pick = x; break; } if (pick.empty()) pick = *alt.begin(); else apph_n_r2++; need.insert(pick); }
    for (auto& t : m.T) {
        if (skip.count(t.key())) continue;
        std::set<std::string> u; int w = apph_tol_match(t, S, u);
        if (w < 0) miss_tol.push_back(t.key());
        else { need.insert(u.begin(), u.end()); if (w == 0) apph_n_tol_lit++; else if (w == 1) apph_n_tol_ab++; else apph_n_tol_db++; }
    }
}
// expected values for the address, resolved on the DUT atoms S (R2 alternatives and R1 transfers); false if the address is not defined
static bool apph_expected(int addr, const std::set<std::string>& S, std::set<std::string>& need, std::vector<std::string>& miss_tol) {
    auto wi = APPH_WORD.find(addr); auto di = APPH_DEVS.find(addr);
    if (wi == APPH_WORD.end() && di == APPH_DEVS.end()) return false;
    need.clear(); miss_tol.clear();
    const ApphBlk* b = wi != APPH_WORD.end() ? &APPH_BLKS[wi->second] : nullptr;
    std::set<std::string> skip; if (di != APPH_DEVS.end()) for (auto& d : di->second) if (d.op == "deltr") skip.insert(d.arg);
    if (di != APPH_DEVS.end()) for (auto& d : di->second) {
        if (d.op == "set") { need = apph_atomset(d.arg); b = nullptr; }
        else if (d.op == "copy") {
            std::string lab = d.arg, plus; size_t p = lab.find('+'); if (p != std::string::npos) { plus = lab.substr(p + 1); lab = lab.substr(0, p); } lab = apph_trim(lab);
            auto it = APPH_BYL.find(lab); if (it == APPH_BYL.end()) { printf("FAIL: copy deviation %03x: label '%s' missing from App. H\n", addr, lab.c_str()); exit(1); }
            need.clear(); apph_resolve(apph_block_model(APPH_BLKS[it->second], true), S, skip, need, miss_tol);
            for (auto& x : apph_atomset(plus)) need.insert(x); b = nullptr;
        }
    }
    if (b) apph_resolve(apph_block_model(*b, true), S, skip, need, miss_tol);
    if (di != APPH_DEVS.end()) for (auto& d : di->second) {
        if (d.op == "add") for (auto& x : apph_atomset(d.arg)) need.insert(x);
        else if (d.op == "del") for (auto& x : apph_atomset(d.arg)) need.erase(x);
    }
    return true;
}
// canonical key of the expected content (unresolved: atoms, alternatives, '*' transfers): same key => same nanoword
static std::string apph_canon(int addr) {
    auto wi = APPH_WORD.find(addr); auto di = APPH_DEVS.find(addr);
    if (wi == APPH_WORD.end() && di == APPH_DEVS.end()) return "";
    const ApphBlk* b = wi != APPH_WORD.end() ? &APPH_BLKS[wi->second] : nullptr;
    std::set<std::string> skip; if (di != APPH_DEVS.end()) for (auto& d : di->second) if (d.op == "deltr") skip.insert(d.arg);
    ApphModel m; bool have = false;
    if (di != APPH_DEVS.end()) for (auto& d : di->second) {
        if (d.op == "set") { m = ApphModel(); m.A = apph_atomset(d.arg); have = true; b = nullptr; }
        else if (d.op == "copy") {
            std::string lab = d.arg, plus; size_t p = lab.find('+'); if (p != std::string::npos) { plus = lab.substr(p + 1); lab = lab.substr(0, p); } lab = apph_trim(lab);
            auto it = APPH_BYL.find(lab); if (it == APPH_BYL.end()) return "?";
            m = apph_block_model(APPH_BLKS[it->second], false); for (auto& x : apph_atomset(plus)) m.A.insert(x); have = true; b = nullptr;
        }
    }
    if (b) { m = apph_block_model(*b, false); have = true; }
    if (!have) return "";
    if (di != APPH_DEVS.end()) for (auto& d : di->second) {
        if (d.op == "add") for (auto& x : apph_atomset(d.arg)) m.A.insert(x);
        else if (d.op == "del") for (auto& x : apph_atomset(d.arg)) m.A.erase(x);
    }
    std::string k = apph_join(m.A) + "|";
    for (auto& alt : m.ALT) k += "{" + apph_join(alt) + "}";
    k += "|";
    for (auto& t : m.T) if (!skip.count(t.key())) k += t.key() + ";";
    return k;
}
