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

// tb_m68k_alu_si.cpp - verification of the silicon-structure ALU/CC (rtl/cpu_si/m68k_alu_si.sv) against the hand-written C++
// model of m68k_alu_cells.h, with the cells of FIG. 17 read at run time from the literal transcription
// docs/data/mc68000_alu_matrix_fig17.tsv (+data=<file>; default ../docs/data/mc68000_alu_matrix_fig17.tsv) and the optional
// documented deviations (+dev=<file>). The 45-bit entry (e_in) and the one-hot code (op1h) are built by the testbench from the
// cell read, with the encoding of m68k_alu_pkg.sv (interface); the lookup in the RTL table is tested by the alux_si testbench.
// Each cell (16 rows x columns 0..6) x CC mode (n/i/f):
//  A) byte additions/subtractions and logic: ALL 65,536 pairs a[7:0] x d[7:0] (upper bytes varied with the pair), input X/Z
//     and carry latch at the values that matter;
//  B) word: patterned operands (zeros, ones, walking one/zero, boundaries, carry chains: 48 values, all 2,304 pairs)
//     x 8 CC states x carry latch;
//  C) shift/rotate: byte, word and 32-bit pair, patterned operand and ALUE, column 6, c_as_x with MULU/MULS (row 7), shift_op;
//  D) EXT (column 5): all 65,536 operand values;
//  E) fixed-seed random tail (200 vectors per cell and mode).
// Outputs checked: r, alue_out, cc_out, r_valid, hc, is_sub, pair_o, cout_o, adder_o. Verdict: RESULT: PASS/FAIL, exit 0/1.
#include "Vm68k_alu_si.h"
#include "verilated.h"
#include "m68k_alu_cells.h"
static Vm68k_alu_si* dut;
static int fails = 0, checks = 0; static long n_vec = 0;
double sc_time_stamp() { return 0.0; }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static void vec(int row, int col, int mode, int byte_op, int sz, uint16_t a, uint16_t d, uint16_t alue, int cc, int cx, int cl, int ms, int sh) {
    const Cell& c = cell_of(row, col);
    dut->row = row; dut->col = col; dut->init = mode == 1; dut->fin = mode == 2; dut->byte_op = byte_op; dut->sz = sz;
    dut->e_in = cell_entry(c); dut->op1h = 1u << op_code(c.op);
    dut->shift_op = sh; dut->mul_signed = ms; dut->a = a; dut->d = d; dut->alue_in = alue; dut->cc_in = cc; dut->c_as_x = cx; dut->cl_in = cl; dut->eval();
    dut->st_ld = 1; dut->clk = 1; dut->eval(); dut->clk = 0; dut->st_ld = 0; dut->eval();   // stage 1 -> stage registers (ph2), then the flags (ph3)
    AluOut m = alu_model(row, col, mode == 1, mode == 2, byte_op, sz, a, d, alue, cc, cx, cl, ms, sh);
    n_vec++;
    const char* op = c.op.c_str();
    CHECK(dut->r_valid == m.valid, "r%d c%d %s m%d: r_valid %d expected %d", row, col, op, mode, dut->r_valid, m.valid);
    CHECK(dut->r == m.r, "r%d c%d %s m%d b%d sz%d cx%d cl%d ms%d sh%d a=%04x d=%04x alue=%04x cc=%02x: r %04x expected %04x", row, col, op, mode, byte_op, sz, cx, cl, ms, sh, a, d, alue, cc, dut->r, m.r);
    CHECK(dut->alue_out == m.alue, "r%d c%d %s m%d b%d sz%d cx%d a=%04x alue=%04x: alue %04x expected %04x", row, col, op, mode, byte_op, sz, cx, a, alue, dut->alue_out, m.alue);
    CHECK(dut->cc_out == m.cc, "r%d c%d %s m%d b%d sz%d cx%d cl%d ms%d sh%d a=%04x d=%04x alue=%04x cc=%02x: cc %02x expected %02x", row, col, op, mode, byte_op, sz, cx, cl, ms, sh, a, d, alue, cc, dut->cc_out, m.cc);
    CHECK(dut->hc == m.hc, "r%d c%d %s b%d a=%04x d=%04x cc=%02x cl%d: hc %d expected %d", row, col, op, byte_op, a, d, cc, cl, dut->hc, m.hc);
    CHECK(dut->is_sub == m.is_sub, "r%d c%d %s: is_sub %d expected %d", row, col, op, dut->is_sub, m.is_sub);
    CHECK(dut->pair_o == m.pair, "r%d c%d %s sz%d cx%d: pair_o %d expected %d", row, col, op, sz, cx, dut->pair_o, m.pair);
    CHECK(dut->cout_o == m.cout, "r%d c%d %s b%d a=%04x d=%04x cc=%02x cl%d: cout_o %d expected %d", row, col, op, byte_op, a, d, cc, cl, dut->cout_o, m.cout);
    CHECK(dut->adder_o == m.adder, "r%d c%d %s: adder_o %d expected %d", row, col, op, dut->adder_o, m.adder);
}
static bool is_shift(const std::string& op) { return op == "LSL" || op == "LSR" || op == "ASL" || op == "ASR" || op == "ROL" || op == "ROR" || op == "ROXL" || op == "ROXR" || op == "LSS" || op == "SLAA"; }
static bool is_adder(const std::string& op) { return op == "ADD" || op == "ADDX" || op == "ADD1" || op == "SUB" || op == "SUBX" || op == "SUB0"; }
static bool is_logic(const std::string& op) { return op == "AND" || op == "OR" || op == "EOR"; }

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* path = "../docs/data/mc68000_alu_matrix_fig17.tsv"; const char* dev = nullptr;
    for (int i = 1; i < argc; i++) { if (!strncmp(argv[i], "+data=", 6)) path = argv[i] + 6; if (!strncmp(argv[i], "+dev=", 5)) dev = argv[i] + 5; }
    if (!load_fig17(path) || (dev && !load_cell_dev(dev))) { printf("RESULT: FAIL\n"); return 1; }
    dut = new Vm68k_alu_si; dut->clk = 0; dut->st_ld = 0;
    // patterned operands: zeros, ones, boundaries, carry chains, walking one, walking zero
    std::vector<uint16_t> S = {0x0000, 0xffff, 0x0001, 0x8000, 0x7fff, 0x00ff, 0xff00, 0x0f0f, 0xf0f0, 0x5555, 0xaaaa, 0x0100, 0x0080, 0x007f, 0x8080, 0x7f7f};
    for (int b = 0; b < 16; b++) S.push_back((uint16_t)(1u << b));
    for (int b = 0; b < 16; b++) S.push_back((uint16_t)~(1u << b));
    const std::vector<uint16_t> S8 = {0x0000, 0xffff, 0x8000, 0x0001, 0x7fff, 0x5555, 0x00ff, 0xff00};
    const int CC8[8] = {0x00, 0x10, 0x04, 0x02, 0x01, 0x1f, 0x14, 0x0b};
    long nA = 0, nB = 0, nC = 0, nD = 0, nE = 0; int n_cells = 0, n_none = 0;
    for (int row = 0; row <= 15; row++) for (int col = 0; col <= 6; col++) {
        const Cell& c = cell_of(row, col); const std::string& op = c.op;
        if (op == "NONE") { n_none++; for (int mode = 0; mode < 3; mode++) vec(row, col, mode, 0, 1, 0x1234, 0x5678, 0x9abc, 0x15, 0, 0, 0, 0); continue; }
        n_cells++;
        for (int mode = 0; mode < 3; mode++) {
            bool uses_cl = (op == "ADDX" || op == "SUBX") && mode != 1;
            // A) complete byte
            if (is_adder(op) || is_logic(op)) {
                const int CCA[3] = {0x00, 0x10, 0x04};
                int ncc = (mode == 0) ? (is_adder(op) ? 2 : 1) : 3;
                for (int ci = 0; ci < ncc; ci++) for (int cl = 0; cl <= (uses_cl ? 1 : 0); cl++)
                    for (int al = 0; al < 256; al++) for (int dl = 0; dl < 256; dl++) {
                        uint16_t a = (uint16_t)((dl << 8) | al), d = (uint16_t)((al << 8) | dl);
                        vec(row, col, mode, 1, 1, a, d, 0, CCA[ci], 0, cl, 0, 0); nA++;
                    }
            }
            // B) patterned word (non-shift operations)
            if (!is_shift(op)) {
                for (uint16_t a : S) for (uint16_t d : S) for (int ci = 0; ci < 8; ci++) for (int cl = 0; cl <= (uses_cl ? 1 : 0); cl++) { vec(row, col, mode, 0, 1, a, d, 0x3c3c, CC8[ci], 0, cl, 0, 0); nB++; }
            }
            // C) shifts and rotations: byte/word/pair, patterned ALUE, shift_op, c_as_x (row 7, column 4/6: ROXR of the multiplication)
            if (is_shift(op)) {
                for (int sz = 0; sz < 3; sz++) for (uint16_t a : S) for (uint16_t alue : S8) for (int ci = 0; ci < 8; ci++) for (int sh = 0; sh < 2; sh++) {
                    vec(row, col, mode, sz == 0, sz, a, 0x2468, alue, CC8[ci], 0, 0, 0, sh); nC++;
                    if (row == 7 && (col == 4 || col == 6)) for (int ms = 0; ms < 2; ms++) { vec(row, col, mode, 0, 2, a, 0x2468, alue, CC8[ci], 1, 0, ms, sh); nC++; }
                }
            }
            // D) EXT: all 65,536 operands
            if (op == "EXT") { for (uint32_t a = 0; a < 65536; a++) { vec(row, col, mode, 0, 1, (uint16_t)a, 0x5a5a, 0x0001, (a & 1) ? 0x1f : 0x00, 0, 0, 0, 0); nD++; } }
        }
    }
    // E) fixed-seed random tail
    srand(7);
    for (int row = 0; row <= 15; row++) for (int col = 0; col <= 6; col++) for (int mode = 0; mode < 3; mode++) for (int v = 0; v < 200; v++) {
        uint16_t a = (uint16_t)rand(), d = (uint16_t)rand(), alue = (uint16_t)rand(); int cc = rand() & 31, sz = rand() % 3, byte_op = sz == 0 ? 1 : (rand() & 1);
        int cx = (row == 7 && (col == 4 || col == 6) && (rand() & 1)) ? 1 : 0; if (cx) { sz = 2; byte_op = 0; }
        vec(row, col, mode, byte_op, sz, a, d, alue, cc, cx, rand() & 1, rand() & 1, rand() & 1); nE++;
    }
    printf("ALU/CC: %d cells with an operation + %d empty (16 rows x columns 0..6) from %s | deviations %d | vectors: complete bytes %ld, patterned words %ld, shift %ld, EXT %ld, random %ld, total %ld | checks %d | FAIL %d\n",
           n_cells, n_none, path, ncelldev, nA, nB, nC, nD, nE, n_vec, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
