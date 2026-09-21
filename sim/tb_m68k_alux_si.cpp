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

// tb_m68k_alux_si.cpp - verification of the ALU with its surrounding registers (rtl/cpu_si/m68k_alux_si.sv) inside m68k_alux_top
// (control store + decoder + alux), in the core configuration (ROW_REG = 1, ORG_OVR_LUT = 0). The testbench is independent of the
// RTL and of the generators: the commands of the word are read from the `dec` vector of the REAL control store, decoded with the
// fields of the pkg m68k_nano_dec_pkg.sv (+pkg=<file>, read at run time: it is the decoder interface); the hand-written C++ model
// applies the rules of docs/silicon-datapath.md §9.5 (operands, ALUE, ALUB, DCR, corf, LOCZ, carry latch) with the ALU of
// m68k_alu_cells.h (raw FIG. 17 cells, +fig17=) and the row of m68k_rowdec_model.h (raw FIG. 18, +fig18=); optional documented
// deviations +devalu= / +devrow=. The driven bus sections are derived from the atoms of the word with the rule of
// §5 (as in the processor). No generated file is used.
// The FIG. 17 entry and the row are looked up by the RTL (m68k_alux_si: pkg table, row decoder on the registered `ir`): here the
// real ones are judged. Positioning on a word: a0s override to word 000 (allowed in the core), reset path
// up to the first dispatch (ty = 1), A1 = wanted word (with +lut=1, testbench configuration, direct a0s override).
// For EVERY microcode word: every IRD class (matrix row, cc_inh, shift and its size, IRD[8]) x is_byte
// 0/1, with 8 CC states and six patterned buses that rotate; for the first class all 32 CC states; hold with en = 0.
// Checks at every microcycle: cc_alu (after op_ld and the stage), then src_alu, src_alue, locz, src_dcr, d4, alue10 at the end of the word.
// Verdict: RESULT: PASS/FAIL, exit 0/1.
#include "Vm68k_alux_top.h"
#include "verilated.h"
#include "m68k_alu_cells.h"
#include "m68k_rowdec_model.h"
#include <set>
static Vm68k_alux_top* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
static void tick() { dut->clk = 0; dut->eval(); dut->clk = 1; dut->eval(); dut->clk = 0; dut->eval(); }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// --- nano decoder fields, from the pkg (interface): "localparam int ND_X_LSB = n, ND_X_W = w, ND_X_ONEHOT = o;  // prefix"
//     and "localparam int ND_X__V = k;  // atom". Enumerated: value k active <=> dec[LSB +: W] == k; one-hot: bit k of dec[LSB +: W].
struct Field { std::string name, prefix; int lsb = 0, w = 0, onehot = 0; std::map<int, std::string> atoms; };
static std::vector<Field> FIELDS; static std::map<std::string, int> FIDX; static int NDEC_W_PKG = 0;
static bool load_pkg(const char* path) {
    std::ifstream f(path); if (!f) { printf("FAIL: cannot open %s\n", path); return false; }
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        line = cells_trim(line);
        if (line.rfind("localparam int NDEC_W = ", 0) == 0) { NDEC_W_PKG = atoi(line.c_str() + 24); continue; }
        if (line.rfind("localparam int ND_", 0) != 0) continue;
        size_t c = line.find("//"); std::string cmt = c == std::string::npos ? "" : cells_trim(line.substr(c + 2)), body = line.substr(0, c);
        if (body.find("_LSB = ") != std::string::npos) {
            size_t a = body.find("ND_") + 3, b = body.find("_LSB"); Field fd; fd.name = body.substr(a, b - a); fd.prefix = cmt;
            fd.lsb = atoi(body.c_str() + body.find("_LSB = ") + 7); fd.w = atoi(body.c_str() + body.find("_W = ") + 5); fd.onehot = atoi(body.c_str() + body.find("_ONEHOT = ") + 10);
            FIDX[fd.name] = (int)FIELDS.size(); FIELDS.push_back(fd);
        } else if (body.find("__") != std::string::npos) {
            size_t a = body.find("ND_") + 3, b = body.find("__"); std::string name = body.substr(a, b - a);
            auto it = FIDX.find(name); if (it == FIDX.end()) { printf("FAIL: pkg: value of an unknown field: '%s'\n", line.c_str()); return false; }
            FIELDS[it->second].atoms[atoi(body.c_str() + body.find(" = ") + 3)] = cmt;
        }
    }
    if (FIELDS.empty() || NDEC_W_PKG <= 0) { printf("FAIL: pkg without fields\n"); return false; }
    return true;
}
static uint32_t dec_bits(int lsb, int w) { uint32_t v = 0; for (int i = 0; i < w; i++) v |= ((dut->dec[(lsb + i) >> 5] >> ((lsb + i) & 31)) & 1u) << i; return v; }
static std::set<std::string> atoms_now() {
    std::set<std::string> A;
    for (const Field& f : FIELDS) {
        uint32_t v = dec_bits(f.lsb, f.w);
        if (f.onehot) { for (int b = 0; b < f.w; b++) if (v >> b & 1) { auto it = f.atoms.find(b); if (it != f.atoms.end()) A.insert(it->second); } }
        else if (v) { auto it = f.atoms.find((int)v); if (it != f.atoms.end()) A.insert(it->second); }
    }
    return A;
}
// --- bus sections driven by the atoms of the word (docs/silicon-datapath.md §5)
static void drive_flags(const std::set<std::string>& A, bool& ab_l, bool& ab_d, bool& db_l, bool& db_d) {
    static const std::set<std::string> S32 = {"at", "pc", "rx", "rxa", "ry", "rya", "sp", "au", "aob"}, SH = {"ath", "pch", "rxh", "ryh", "sph"},
                                       SL = {"atl", "pcl", "rxl", "ryl", "spl", "ftu"}, SD = {"alu", "alue", "dcr", "dbin", "rxd", "rxdl", "ryd", "rydl"};
    for (const std::string& a : A) {
        size_t p = a.find("<-"); if (p == std::string::npos) continue;
        std::string bus = a.substr(0, p), src = a.substr(p + 2);
        if (src.size() >= 2 && src.front() == '(' && src.back() == ')') src = src.substr(1, src.size() - 2);
        std::string base; char suf = 0;
        if (bus.rfind("ab", 0) == 0) base = "ab"; else if (bus.rfind("db", 0) == 0) base = "db"; else continue;
        if (bus.size() == 3) suf = bus[2]; else if (bus.size() != 2) continue;
        bool l = false, d = false;
        if (suf == 0) { if (S32.count(src)) l = true; else if (SL.count(src)) l = true; else if (SD.count(src)) d = true; }
        else if (suf == 'l' || suf == 'e') l = true; else if (suf == 'd') d = true;
        else if (suf == '*') { if (S32.count(src)) { l = true; d = true; } else if (SL.count(src)) l = true; else if (!SH.count(src)) d = true; }
        if (base == "ab") { ab_l |= l; ab_d |= d; } else { db_l |= l; db_d |= d; }
    }
}
// --- model of the surrounding registers (§9.5) and microcycle
struct St { uint16_t alu = 0, alue = 0, alub = 0, corf = 0; int dcr = 0, locz = 0, cl = 0, cl_v = 0; } S;
static int cur_ird = 0; static long n_edges = 0, n_ops = 0, n_pos = 0; static long trace_n = 0;   // +trace=N: prints the first N microcycles
static const int CC8[8] = {0x00, 0x10, 0x04, 0x02, 0x01, 0x1f, 0x14, 0x0b};
static const uint16_t PAT[16] = {0x0000, 0xffff, 0x8000, 0x7fff, 0x0001, 0x00ff, 0xff00, 0x5555, 0xaaaa, 0x0f0f, 0x1234, 0x8001, 0x7ffe, 0x0100, 0xfffe, 0x4321};
// the current word is an A1 dispatch (format I, TY = 01): from bits 1 and 3:2 of the current microword `uw` (docs/microcode-structure.md section 3);
// the fmt/ty outputs of the control store come from the registered copy of the word and arrive one clock after upc/uw
static bool is_dispatch() { return ((dut->uw >> 1) & 1) == 0 && ((dut->uw >> 2) & 3) == 1; }
static void edge(bool en, int is_byte, int cc, long pi, int ird_next) {
    // settling (ph0): the non-RAW class bits of `dec` come out of the registered copy one clock after the word fetch:
    // the atoms of the current word are read AFTER this clock (with en = 0 the control store does not advance)
    dut->en = 0; dut->eval(); tick();
    int w = dut->upc; std::set<std::string> A = atoms_now();
    bool disp = is_dispatch();   // instruction start: instr_start at en
    if (n_edges < trace_n) printf("trace: microcycle %ld upc %03x fmt %d ty %d uw %05x en %d a0s_sub %d a0s %03x a1 %03x atoms %d\n", n_edges, w, dut->fmt, dut->ty, dut->uw, en, dut->a0s_sub, dut->a0s, dut->a1, (int)A.size());
    dut->ird = cur_ird; dut->is_byte = is_byte; dut->cc_in = cc;
    dut->ab_h = PAT[pi % 16]; dut->ab_l = PAT[(pi * 3 + 1) % 16]; dut->ab_d = PAT[(pi * 5 + 2) % 16]; dut->db_h = PAT[(pi * 7 + 3) % 16]; dut->db_l = PAT[(pi * 11 + 5) % 16]; dut->db_d = PAT[(pi * 13 + 7) % 16];
    bool abl = false, abd = false, dbl = false, dbd = false; drive_flags(A, abl, abd, dbl, dbd);
    dut->ab_l_drv = abl; dut->ab_d_drv = abd; dut->db_l_drv = dbl; dut->db_d_drv = dbd; dut->eval();
    // model: operands (§9.5: a = AB side or ALUB; d = DB side, corf or constant; the constant goes on the free side)
    int ird = cur_ird;
    bool a_ab = A.count("alu<-ab") || A.count("alu<-ab*") || A.count("alu<-abd") || A.count("alu<-abe");
    bool d_db = A.count("alu<-db") || A.count("alu<-db*") || A.count("alu<-dbd") || A.count("alu<-dbe");
    bool a_alub = A.count("alu<-(alub)"), d_corf = A.count("alu<-corf"), k_m1 = A.count("alu+=-1"), k_0 = A.count("alu+=0");
    uint16_t ab_v = abd ? dut->ab_d : abl ? dut->ab_l : dut->ab_h, db_v = dbd ? dut->db_d : dbl ? dut->db_l : dut->db_h;   // D section, then L, then H
    uint16_t a = 0, d = 0;
    if (d_db) d = db_v; else if (d_corf) d = S.corf; else if (k_m1) d = 0xffff; else if (k_0) d = 0;
    if (a_ab) a = ab_v; else if (a_alub) a = S.alub; else if ((d_db || d_corf) && k_m1) a = 0xffff;
    int col = 0; for (int c = 1; c <= 6; c++) if (A.count("alu_col=" + std::to_string(c))) col = c;
    bool sr = A.count("sr c-alu-alue") > 0;
    bool init = A.count("alu_cc=i") > 0, fin = A.count("alu_cc=f") > 0;
    bool init_l = init;   // the 'initiate' writes the carry latch even when the PSW does not change (ADDA.L/SUBA.L/CMPA.L: the high half uses the carry of the low half; patent col. 30)
    if (rowdec_cc_inh(ird)) { init = false; fin = false; }   // destination An: the ALU computes, the PSW does not change
    int sz = ((ird >> 12) == 0xe && ((ird >> 6) & 3) != 3) ? (ird >> 6) & 3 : 1; if (sr) sz = 2;
    int row = rowdec_row(ird);
    int cl_in = S.cl_v ? S.cl : (cc >> 4 & 1);
    AluOut o = alu_model(row, col, init, fin, is_byte != 0, sz, a, d, S.alue, cc, sr, cl_in != 0, ((ird >> 8) & 1) != 0, (ird >> 12) == 0xe);
    int cc_exp = o.valid ? o.cc : cc;
    // microcycle protocol as in the core, one clock per phase: ph1 op_ld (operands and entry into the alux registers), ph2 st_ld
    // (ALU stage registers), ph3 en (end of word: alu_r, ALUE, ALUB, DCR, corf, LOCZ, carry latch)
    if (n_edges < trace_n) printf("trace:   after settling src_alu %04x cc_alu %02x (model: valid %d r %04x cc %02x col %d row %d a %04x d %04x)\n", dut->src_alu, dut->cc_alu, o.valid, o.r, cc_exp, col, row, a, d);
    dut->op_ld = 1; dut->eval(); tick(); dut->op_ld = 0; dut->st_ld = 1; dut->eval(); tick(); dut->st_ld = 0; dut->eval();
    if (n_edges < trace_n) printf("trace:   after op_ld/st_ld src_alu %04x cc_alu %02x\n", dut->src_alu, dut->cc_alu);
    CHECK(dut->cc_alu == cc_exp, "word %03x ird %04x row %d col %d i%d f%d b%d sz%d a=%04x d=%04x alue=%04x cc=%02x cl=%d: cc_alu %02x expected %02x", w, ird, row, col, init, fin, is_byte, sz, a, d, S.alue, cc, cl_in, dut->cc_alu, cc_exp);
    dut->instr_start = en && disp; dut->ir = ird_next; dut->ld_ird = 1; dut->en = en; dut->eval(); tick();
    dut->ld_ird = 0; dut->instr_start = 0; dut->en = 1;
    if (n_edges < trace_n) printf("trace:   after en=%d        src_alu %04x cc_alu %02x\n", en, dut->src_alu, dut->cc_alu);
    if (en) {
        cur_ird = ird_next; dut->ird = cur_ird;
        if (o.valid) {
            n_ops++; S.alu = o.r; S.locz = is_byte ? (((a ^ d) & 0xff) == 0xff) : ((uint16_t)(a ^ d) == 0xffff);   // LOCZ: all bits propagate
            int dc_c = o.cout ^ (o.is_sub ? 1 : 0);   // carry (addition) / borrow (subtraction) of the adder
            int lo6 = o.hc || (o.r & 15) > 9, hi6 = dc_c || ((o.r >> 4) & 15) > 9 || (((o.r & 15) > 9) && ((o.r >> 4) & 15) == 9);
            S.corf = o.is_sub ? (uint16_t)~(((dc_c ? 6 : 0) << 4) | (o.hc ? 6 : 0)) : (uint16_t)(((hi6 ? 6 : 0) << 4) | (lo6 ? 6 : 0));
            if (init_l || o.adder) S.cl = dc_c;   // carry latch: from the 'initiate' words and from every adder operation
        }
        if (o.valid && init_l) S.cl_v = 1; else if (disp) S.cl_v = 0;
        if (o.valid && o.pair) S.alue = o.alue; else if (A.count("alue<-db*") || A.count("alue<-dbd")) S.alue = db_v;
        if (A.count("alub<-ab") || A.count("alub<-ab*") || A.count("alub<-abd")) S.alub = ab_v;
        else if (A.count("alub<-db") || A.count("alub<-db*") || A.count("alub<-dbd")) S.alub = db_v;
        if (A.count("dcr<-ab") || A.count("dcr<-ab*")) S.dcr = ab_v & 31;
    }
    dut->eval(); n_edges++;
    CHECK(dut->src_alu == S.alu, "word %03x en=%d ird %04x: src_alu %04x expected %04x (model: valid %d row %d col %d i%d f%d b%d a=%04x d=%04x flags ab l%d d%d db l%d d%d; atoms:%s)", w, en, ird, dut->src_alu, S.alu, o.valid, row, col, init, fin, is_byte, a, d, abl, abd, dbl, dbd, [&]{ std::string s; for (auto& x : A) s += " " + x; return s; }().c_str());
    CHECK(dut->src_alue == S.alue, "word %03x en=%d ird %04x: src_alue %04x expected %04x", w, en, ird, dut->src_alue, S.alue);
    CHECK(dut->locz == S.locz, "word %03x en=%d: locz %d expected %d", w, en, dut->locz, S.locz);
    CHECK(dut->src_dcr == (1u << (S.dcr & (is_byte ? 7 : 15))), "word %03x en=%d b%d: src_dcr %04x expected %04x (dcr %d)", w, en, is_byte, dut->src_dcr, 1u << (S.dcr & (is_byte ? 7 : 15)), S.dcr);
    CHECK(dut->d4 == (is_byte ? 0 : ((S.dcr >> 4) & 1)), "word %03x en=%d b%d: d4 %d (dcr %d)", w, en, is_byte, dut->d4, S.dcr);
    CHECK(dut->alue10 == (S.alue & 3), "word %03x en=%d: alue10 %d expected %d", w, en, dut->alue10, S.alue & 3);
}
// instruction loaded during the positioning path and for the word that the path executes when leaving it: MULU.W D0,D0
// (row 7 of FIG. 17), so the `sr c-alu-alue` word of the multiplication, reachable in the processor only from MULU/MULS,
// is always executed with the multiplication row (with another row the RTL and the model have no defined behaviour)
static const int IRD_WALK = 0xc0c0;
static bool is_mul(int ird) { return (ird & 0xf000) == 0xc000 && ((ird >> 6) & 7) == 3; }   // MULU/MULS (manual: 1100 rrr 011/111 eeeeee)
// continuation: after word W the testbench follows the microprogram for another KCONT words with the same instruction, so the
// surrounding state between words (carry latch initiate -> finish, ALUE, ALUB, corf, DCR, LOCZ) is read by the word that uses it
static const int KCONT = 3; static long n_cont = 0;
static int lut_mode = 0;
static bool goto_word(int W, int irdW, long& pi) {
    n_pos++;
    if (lut_mode) { dut->a0s_sub = 1; dut->a0s = W; edge(true, 0, 0, pi++, irdW); dut->a0s_sub = 0; }
    else {
        dut->a0s_sub = 1; dut->a0s = 0; edge(true, 0, 0, pi++, IRD_WALK); dut->a0s_sub = 0;   // override to word 000 (allowed in the core)
        if (dut->upc != 0) { CHECK(false, "positioning: upc %03x after the override to 000", dut->upc); return false; }
        for (int i = 0; i < 32 && !is_dispatch(); i++) edge(true, 0, 0, pi++, IRD_WALK);   // reset path up to the dispatch
        if (!is_dispatch()) { CHECK(false, "positioning: no dispatch within 32 words from reset"); return false; }
        dut->a1 = W; dut->a0_sub = 0; edge(true, 0, 0, pi++, irdW); dut->a1 = 0;
    }
    CHECK(dut->upc == W, "positioning: upc %03x expected %03x", dut->upc, W);
    return dut->upc == W;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* f17 = "../docs/data/mc68000_alu_matrix_fig17.tsv"; const char* f18 = "../docs/data/mc68000_alu_rows_fig18.tsv"; const char* pkg = "../rtl/cpu_si/m68k_nano_dec_pkg.sv";
    const char* devalu = nullptr; const char* devrow = nullptr;
    for (int i = 1; i < argc; i++) {
        if (!strncmp(argv[i], "+fig17=", 7)) f17 = argv[i] + 7; if (!strncmp(argv[i], "+fig18=", 7)) f18 = argv[i] + 7; if (!strncmp(argv[i], "+pkg=", 5)) pkg = argv[i] + 5;
        if (!strncmp(argv[i], "+devalu=", 8)) devalu = argv[i] + 8; if (!strncmp(argv[i], "+devrow=", 8)) devrow = argv[i] + 8; if (!strncmp(argv[i], "+lut=", 5)) lut_mode = atoi(argv[i] + 5);
        if (!strncmp(argv[i], "+trace=", 7)) trace_n = atol(argv[i] + 7);
    }
    if (!load_fig17(f17) || (devalu && !load_cell_dev(devalu)) || !rowdec_load(f18) || (devrow && !rowdec_load_dev(devrow)) || !load_pkg(pkg)) { printf("RESULT: FAIL\n"); return 1; }
    if (NDEC_W_PKG > 7 * 32) { printf("FAIL: NDEC_W %d beyond the 7 words of the testbench\n", NDEC_W_PKG); printf("RESULT: FAIL\n"); return 1; }
    // IRD classes: matrix row, cc_inh, shift and its size, IRD[8] (everything the alux reads from the instruction)
    std::map<int, int> cls;
    for (int ird = 0; ird < 65536; ird++) {
        int sh = (ird >> 12) == 0xe, sz = (sh && ((ird >> 6) & 3) != 3) ? (ird >> 6) & 3 : 1;
        int key = rowdec_row(ird) * 64 + (rowdec_cc_inh(ird) ? 32 : 0) + sh * 16 + sz * 2 + ((ird >> 8) & 1);
        if (!cls.count(key)) cls[key] = ird;
    }
    std::vector<int> CLS; for (auto& kv : cls) CLS.push_back(kv.second);
    dut = new Vm68k_alux_top;
    dut->clk = 0; dut->rst_n = 0; dut->en = 1; dut->a1 = 0; dut->a2 = 0; dut->a3 = 0; dut->a0_sub = 0; dut->a0 = 0; dut->a0s_sub = 0; dut->a0s = 0; dut->c1c0 = 0;
    dut->op_ld = 0; dut->st_ld = 0; dut->instr_start = 0; dut->is_byte = 0; dut->ird = 0; dut->ir = 0; dut->ld_ird = 0;
    dut->ab_h = 0; dut->ab_l = 0; dut->ab_d = 0; dut->db_h = 0; dut->db_l = 0; dut->db_d = 0; dut->ab_l_drv = 0; dut->ab_d_drv = 0; dut->db_l_drv = 0; dut->db_d_drv = 0; dut->cc_in = 0;
    tick(); tick(); dut->rst_n = 1; dut->eval(); S = St(); cur_ird = 0;
    long pi = 0;
    // survey: atoms of every word (for the `sr c-alu-alue` words, reachable only from the multiplication, IRD = MULU/MULS)
    std::vector<std::set<std::string>> atomsW(1024); std::vector<bool> isword(1024, false); std::vector<int> succ(1024, 0); int n_words = 0;
    for (int W = 0; W < 1024; W++) {
        if (!goto_word(W, IRD_WALK, pi)) { printf("RESULT: FAIL\n"); return 1; }
        dut->en = 0; dut->eval(); tick();   // settling: the atoms (non-RAW class of dec) come out one clock after the fetch
        atomsW[W] = atoms_now(); isword[W] = dut->uw != 0 || !atomsW[W].empty(); if (isword[W]) n_words++;
        edge(true, 0, 0, pi++, IRD_WALK); succ[W] = dut->upc;   // successor of the word (c1c0 = 0 and a1/a2/a3 = 0, as in the run)
    }
    // main run
    long it = 0;
    for (int W = 0; W < 1024; W++) {
        if (!isword[W]) continue;
        bool sr = atomsW[W].count("sr c-alu-alue") > 0;
        int ncls = sr ? 2 : (int)CLS.size();
        for (int k = 0; k < ncls; k++) {
            int irdW = sr ? (0xc0c0 | (k << 8)) : CLS[k];   // multiplication: MULU / MULS Dn,Dn
            for (int b = 0; b < 2; b++) {
                int ncc = (k == 0 && b == 0) ? 32 : 1;
                for (int ci = 0; ci < ncc; ci++) {
                    if (!goto_word(W, irdW, pi)) { printf("RESULT: FAIL\n"); return 1; }
                    int cc = ncc == 32 ? ci : CC8[(it + W) % 8];
                    // W and then up to KCONT successors with the same instruction; the last edge loads IRD_WALK, so the word that the
                    // next positioning executes when leaving (the successor) has the multiplication row (see IRD_WALK); in front of
                    // an `sr c-alu-alue` word with an instruction that is not a multiplication the continuation stops
                    int Wc = W;
                    for (int j = 0; j <= KCONT; j++) {
                        int Wn = succ[Wc];
                        bool last = (j == KCONT) || !isword[Wn] || (atomsW[Wn].count("sr c-alu-alue") && !is_mul(irdW));
                        edge(true, b, cc, pi++, last ? IRD_WALK : irdW); if (j > 0) n_cont++;
                        if (j == 0 && it % 7 == 0) edge(false, b, cc, pi++, last ? IRD_WALK : irdW);   // hold with en = 0: nothing changes in the registers
                        if (last) break;
                        Wc = Wn;
                    }
                    it++;
                }
            }
        }
        if (fails > 20) break;
    }
    printf("ALUX: words %d | IRD classes %d (x is_byte 0/1; 32 CC states on the first, 8 in rotation on the others) | positionings %ld (reset path %s) | microcycles %ld, ALU operations %ld, continuation words %ld (up to %d after each word) | ALU deviations %d, row %d | check %d | FAIL %d\n",
           n_words, (int)CLS.size(), n_pos, lut_mode ? "no: a0s override" : "yes", n_edges, n_ops, n_cont, KCONT, ncelldev, rowdec_ndev, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
