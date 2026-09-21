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

// tb_m68k_rowdec.cpp - verification of the ALU/CC row decoder (rtl/cpu_si/m68k_alu_rowdec.sv) on ALL 65536 IRD values
// against the independent model in m68k_rowdec_model.h: FIG. 18 terms read at run time from the literal transcription
// docs/data/mc68000_alu_rows_fig18.tsv (+data=<file>; default ../docs/data/mc68000_alu_rows_fig18.tsv), optional documented
// deviations (+dev=<file>), cc_inh from the manual. No generated file is used.
// Checks: row_n, row and cc_inh on every IRD; every model term active for at least one IRD. Outcome: RESULT: PASS/FAIL, exit code 0/1.
#include "Vm68k_alu_rowdec.h"
#include "verilated.h"
#include "m68k_rowdec_model.h"
#include <cstring>
static Vm68k_alu_rowdec* dut;
static int fails = 0, checks = 0;
double sc_time_stamp() { return 0.0; }
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails <= 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* path = "../docs/data/mc68000_alu_rows_fig18.tsv"; const char* dev = nullptr;
    for (int i = 1; i < argc; i++) { if (!strncmp(argv[i], "+data=", 6)) path = argv[i] + 6; if (!strncmp(argv[i], "+dev=", 5)) dev = argv[i] + 5; }
    if (!rowdec_load(path) || (dev && !rowdec_load_dev(dev))) { printf("RESULT: FAIL\n"); return 1; }
    dut = new Vm68k_alu_rowdec;
    std::vector<int> act(RTERMS.size(), 0);
    int one = 0, none = 0, multi = 0, allinh = 0, ninh = 0, perrow[16] = {0};
    for (uint32_t ird = 0; ird < 65536; ird++) {
        dut->ird = ird; dut->eval();
        int m, r; rowdec_model(ird, m, r, &act);
        CHECK(dut->row_n == m, "ird %04x row_n %04x expected %04x", ird, dut->row_n, m);
        CHECK(dut->row == r, "ird %04x row %d expected %d", ird, dut->row, r);
        bool ci = rowdec_cc_inh(ird);
        CHECK(dut->cc_inh == ci, "ird %04x cc_inh %d expected %d", ird, dut->cc_inh, ci);
        int cnt = __builtin_popcount((unsigned)m);
        if (m == 0x7fff) none++; else if (cnt == 1) one++; else if (cnt == 0) allinh++; else multi++;
        if (ci) ninh++;
        perrow[r]++;
    }
    for (size_t i = 0; i < RTERMS.size(); i++) CHECK(act[i] > 0, "term %d never active", RTERMS[i].idx);
    printf("rowdec: %d terms + %d qualifiers from %s | deviations %d | IRD 65536: one row left %d, none inhibited %d, several rows %d, all inhibited %d | cc_inh=1 %d\n",
           (int)RTERMS.size(), rowdec_nq, path, rowdec_ndev, one, none, multi, allinh, ninh);
    printf("rowdec: IRD per row (0..15):"); for (int k = 0; k < 16; k++) printf(" %d", perrow[k]); printf("\n");
    printf("rowdec: check %d | FAIL %d\n", checks, fails);
    printf(fails ? "RESULT: FAIL\n" : "RESULT: PASS\n");
    delete dut; return fails ? 1 : 0;
}
