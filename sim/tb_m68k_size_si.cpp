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

// tb_m68k_size_si.cpp - verification of the size decoder (rtl/cpu_si/m68k_size_si.sv) on ALL 65536 opcodes against a
// C++ reference written independently from the Motorola manual (instruction formats). Opcodes without a data operand
// are "word" by convention in both.
#include "Vm68k_size_si.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
static Vm68k_size_si* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)
static int ref(int op) {
    int line = op >> 12, s = (op >> 6) & 3, mode = (op >> 3) & 7, hi = (op >> 8) & 15, b8 = (op >> 8) & 1;
    switch (line) {
        case 0:
            if (b8 && mode == 1) return (op >> 6) & 1 ? 2 : 1;                       // MOVEP
            if (b8 || hi == 8) return mode == 0 ? 2 : 0;                              // BTST/BCHG/BCLR/BSET
            return s == 3 ? 1 : s;                                                   // immediates
        case 1: return 0; case 2: return 2; case 3: return 1;
        case 4:
            if (((op >> 6) & 63) == 0x2b) return 0;                                  // TAS
            if (((op >> 6) & 63) == 0x20) return 0;                                  // NBCD
            if (hi == 0 || hi == 2 || hi == 4 || hi == 6 || hi == 10) return s == 3 ? 1 : s;   // NEGX CLR NEG NOT TST
            if (((op >> 6) & 63) == 0x23 || ((op >> 6) & 63) == 0x33) return 2;       // MOVEM.L
            return 1;
        case 5:
            if (((op >> 3) & 31) == 0x19) return 1;                                  // DBcc
            return s == 3 ? 0 : s;                                                   // ADDQ/SUBQ, Scc byte
        case 6: case 7: return 1;
        case 8: case 9: case 11: case 13: return s == 3 ? (b8 ? 2 : 1) : s;
        case 12: {
            int f = (op >> 3) & 63;
            if (f == 0x28 || f == 0x29 || f == 0x31) return 2;                       // EXG
            return s == 3 ? 1 : s;
        }
        case 14: return s == 3 ? 1 : s;
        default: return 1;
    }
}
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vm68k_size_si;
    int nb = 0, nw = 0, nl = 0;
    for (uint32_t op = 0; op < 65536; op++) {
        dut->ird = op; dut->eval();
        int r = ref(op); if (r == 0) nb++; else if (r == 1) nw++; else nl++;
        CHECK(dut->sz == r && dut->is_byte == (r == 0), "ird %04x sz %d exp %d", op, dut->sz, r);
    }
    printf("size: IRD 65536 (byte %d, word %d, long %d) | check %d | FAIL %d\n", nb, nw, nl, checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
