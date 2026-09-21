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

// dump_rowdec.cpp: exhaustive table of the ALU row decoder: for each ird it prints row, cc_inh, row_n.
#include "Vm68k_alu_rowdec.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
double sc_time_stamp() { return 0.0; }
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vm68k_alu_rowdec* d = new Vm68k_alu_rowdec;
    FILE* f = fopen(argv[1], "w");
    for (uint32_t ird = 0; ird < 65536; ird++) { d->ird = ird; d->eval(); fprintf(f, "%04x %d %d %04x\n", ird, d->row, d->cc_inh, d->row_n); }
    fclose(f); delete d; return 0;
}
