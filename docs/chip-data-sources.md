# Data taken from the chip

## 1. Purpose

Part of rm68000 is not designed but taken from the MC68000 itself: the contents of its control ROMs, the wiring of
the micro ROM outputs, the addresses and names of its microroutines, the notation of its condition codes. For those
data there was nothing to choose: either they are the chip's, or the processor is not a 68000. This document lists
which data they are, where they are used in the RTL and where each one comes from, so that anyone can check them
against the published source. The rest of the core is our design (§8).

## 2. Sources

Both are public:

- **US 4,325,121**, the Motorola patent of the two-level control store of the processor, with its appendices and
  figures. The parts used are transcribed in `docs/data/`: App. A (`us4325121_appA.csv`), App. C
  (`us4325121_appC.tsv`), App. H (`us4325121_appH.tsv`, with the map to the die addresses
  `us4325121_appH_die_map.tsv`), FIG. 17 (`mc68000_alu_matrix_fig17.tsv`), FIG. 18/19 (`mc68000_alu_rows_fig18.tsv`,
  `mc68000_alu_rows_fig19_check.tsv`), FIG. 21 (`us4325121_fig21_a123.tsv`).
- **The die analysis by Olivier Galibert**: the raw bits of the two ROMs read from the silicon (`microcode.txt`,
  118 × 272 bits = 544 × 17 + 336 × 68), the decoded PLA tables, the label files and a symbolic listing of the
  microcode (`all.txt`). These files are not redistributed here; the repository holds the tables derived from them
  (`mc68000_micro_decoded.csv`, `mc68000_a123_die.tsv`, `mc68000_die_only_words.tsv`).

Where each file comes from and how the raw bits were decoded: [microcode-sources.md](microcode-sources.md). How the
patent describes the control store: [microcode-structure.md](microcode-structure.md).

## 3. Micro ROM contents: 1024 words × 17 bits

File: `rtl/cpu_si/m68k_urom.mem`, read by `rtl/cpu_si/m68k_urom.sv`.

- The 544 words of the chip's micro ROM, bit for bit: the column `raw17` of `docs/data/mc68000_micro_decoded.csv`,
  decoded from the die dump by `docs/data/scripts/decode_micro.py` (which needs Galibert's `microcode.txt`,
  `rommap.txt` and `adr-names.txt`).
- Plus the five entries of `docs/data/deviations/ustore.tsv`: word 1c8 (`rstp2`), all zero in the dump, repaired from
  App. H (next address 002); and four words of our design, at addresses where the die has no word (none between 3f0 and 3ff in the dump):
  3fc and 3fd (STOP wait) and 3ff, 3fe; the 17 bits of 3fe are those of the die word `trap1` (1d0). The core never selects
  3ff and 3fe: instruction exceptions and trace enter at 0e3 (`A_TRAP` and `A_TRACE`
  in `rtl/cpu_si/m68k_exc_si.sv`), the die's path through `dvur2` → `dvur3` → `trap3`, and the A1 value 3ff of the
  illegal opcodes never dispatches, because the exception block replaces it with A0 = 0e3
  ([microcode-structure.md](microcode-structure.md) section 3).
- 548 words in all; 25 of them are zero (unused words of the die), so 523 are non-zero.

The file is written by `docs/data/scripts/gen_urom.py` from the CSV, which is in the repository: running it
reproduces `m68k_urom.mem` exactly, licence header included.

## 4. Nano ROM contents

Files: `rtl/cpu_si/m68k_nrom.mem` (bits 16:0), `rtl/cpu_si/m68k_nrom_m.mem` (bits 76:17),
`rtl/cpu_si/m68k_norigin.mem` (ORIGIN map), and the constants of `rtl/cpu_si/m68k_nano_pkg.sv`; read by
`rtl/cpu_si/m68k_nrom.sv`.

Here the **content** is the chip's and the **encoding** is not:

- content: for each micro address, the operations of the App. H block that the die map places there
  (`us4325121_appH.tsv`, `us4325121_appH_die_map.tsv`), the words the die has and App. H lacks
  (`mc68000_die_only_words.tsv`), and corrections applied by the generator, each listed with its source in
  `docs/data/deviations/nstore.tsv`;
- encoding: the silicon's 68-bit nanoword is not reused. Each operation is an atom, atoms never used together share
  a field, and the result is 512 words of 77 bits, 374 of them distinct, addressed through an ORIGIN map of 124 rows
  ([silicon-datapath.md](silicon-datapath.md) §19.2-§19.3).

The files are written by `docs/data/scripts/gen_nrom.py`, which also reads Galibert's `all.txt` and `adr-names.txt`
(to resolve the buses App. H leaves unnamed and some words App. H lacks): it cannot be rerun from the repository
alone. The `ndec` and `nstore` benches check the content against App. H, the die map and `nstore.tsv` (§10).

## 5. Physical order of the next-address bits

The assignment of `nma` in `rtl/cpu_si/m68k_useq.sv`:

    A0<-b11  A1<-b12  A2<-b7  A3<-b8  A4<-b9  A5<-b10  A6<-b5  A7<-b6  A8<-b13  A9<-b14

The 10 bits of the next micro address (NMA) leave the ROM in this order on the die. It is not a design choice but
the wiring. It was derived by correlation from the dump and checked against the die listing (516 addresses): 399 numeric
jumps out of 399, 47/47 jumps to A1/A2/A3, 70/70 conditional jumps (header of `decode_micro.py`;
[microcode-sources.md](microcode-sources.md)). In format II bits 5-6 belong to the condition code, which is why A6
and A7 are then supplied by the branch unit (C0/C1).

## 6. Microroutine addresses and names

The opcode entries `m` of `rtl/cpu_si/m68k_a123_si.sv` give eight instruction entries directly:

| opcode | routine | address |
|---|---|---|
| BSR.W (6100) | `bsrw1` | 0x0a9 |
| Bcc.W (6x00) | `bbcw1` | 0x068 |
| RESET (4e70) | `rset1` | 0x3a6 |
| NOP (4e71) | `b` | 0x363 |
| STOP (4e72) | `stop1` | 0x3a2 |
| RTE (4e73), RTR (4e77) | `rtr1` | 0x12a |
| RTS (4e75) | `rts1` | 0x126 |
| TRAPV (4e76) | `trpv1` | 0x06d |

The **addresses** are the chip's, from the PLA table of the die (`docs/data/mc68000_a123_die.tsv`: 4e71 → 363,
4e73 and 4e77 → 12a). The entry points of all the other opcodes come from the same table
([silicon-datapath.md](silicon-datapath.md) §15).

The **names** are those of the sources and are kept as printed, also where they look odd: the NOP routine is called
`b` because FIG. 21 prints only "B" for it (`us4325121_fig21_a123.tsv`, note of the NOP cell) and the die labels
give the same name (`mc68000_micro_decoded.csv`, address 363). Renaming them would make the code harder to check
against the patent.

## 7. Condition-code notation and branch-code numbering

Files: `rtl/cpu_si/m68k_alu_pkg.sv`, `rtl/cpu_si/m68k_branch_pkg.sv`, `docs/data/mc68000_alu_matrix_fig17.tsv`,
`docs/data/us4325121_appC.tsv`.

- The condition-code strings of FIG. 17 (`KNZ00`, `KNZKK`, `KKZKK`, `KNZVC`, `ANZ0A`, …; one letter per position
  X N Z V C) and their letters K, D, 0, N, Z, V, C, ~C, C*, ~C*, A, V' (the `ACC_*` constants of `m68k_alu_pkg.sv`).
- The numbering of the conditional-branch codes of App. C:

      0 i11   1 auz   2 c     3 z     4 nz1   5 n     6 nz2   7 ms0
      8 m01   9 cc    a ze    b nv    c d4    d v     e enl

  with the second assignments 11, 12, 15, 19, 1c, 1e and the same C1/C0 values as the appendix.
- The names `SLAA` (FIG. 16/17) and `reset pren` (App. B/H).

They are literal, kept identical to the source so that the code can be read with the patent beside it. Where the
core departs from the letter of these data the departure is a line of `docs/data/deviations/` with its source
([silicon-datapath.md](silicon-datapath.md) §2.3).

## 8. What is our design

Everything else. Where the line falls:

| | |
|---|---|
| nanoword | re-encoded: 512 × 77 bits in mutually exclusive fields, against the die's 336 × 68 raw bits |
| nano decoder | generated vector of 199 bits, one field per resource |
| ORIGIN map | 256 × 7 table in logic, two LUT levels, both nanoword parts read one clock before `en` |
| register file | distributed memory (MLAB), one copy per read port, columns of 16/8/8 bits; AB port written at `en`, DB port queued one clock with forwarding |
| ALU | FIG. 17 cell looked up and registered with the operands, two stages |
| address unit | two-stage sum: low half registered at ph2, high half at `en` |
| internal buses | AND-OR trees on selectors registered from the next nanoword |
| bus cycle machine | 13 states, even states on `cen`, odd states on `cen2` |
| microcycle | 4 phases; a word waiting for its bus cycle holds the phase counter |

[silicon-datapath.md](silicon-datapath.md) describes each of them with its RTL.

## 9. Generated files

13 of the 33 files of `rtl/cpu_si/` say in their header that a script in `docs/data/scripts/` wrote them. Each
generator writes its files complete, licence header included. Their status:

| file | generator | regenerable from the repository |
|---|---|---|
| `m68k_urom.mem` | `gen_urom.py` | yes: reproduced exactly |
| `m68k_nano_dec.sv`, `m68k_nano_dec_pkg.sv` | `gen_nano_dec.py` | yes: reproduced exactly |
| `m68k_bus_sec.sv` | `gen_bus_sec.py` (needs Python 3.12 or later) | yes: reproduced exactly, header included; the generator also writes the bench top `sim/m68k_bus_top.sv` |
| `m68k_regs_bd.sv` | `gen_regs.py` | yes: reproduced exactly; the generator also writes the bench top `sim/m68k_regs_top.sv` |
| `m68k_alu_pkg.sv` | `gen_alu.py` | yes: reproduced exactly, the whole package: the table `SI_ALU_TABLE` and the tables `SI_ALU_OP1H`, `SI_ALU_T2` and `SI_ALU_OP1H2` that the generator derives from it |
| `m68k_alu_rowdec.sv` | `gen_rowdec_case.py`, from `docs/data/mc68000_alu_rowdec_dump.txt`, the dump (`sim/dump_rowdec.cpp`) of the term decoder written by `gen_alu_rowdec.py` | yes, from the dump: reproduced exactly. `gen_alu_rowdec.py` writes the term form only into a folder outside the repository and never overwrites an existing file |
| `m68k_branch_pkg.sv` | `gen_branch.py` | no: needs `all.txt` |
| `m68k_a123_si.sv` | `gen_a123.py` | no: needs `all.txt` and `adr-names.txt` |
| `m68k_nano_pkg.sv`, `m68k_nrom.mem`, `m68k_nrom_m.mem`, `m68k_norigin.mem` | `gen_nrom.py` | no: needs `all.txt` and `adr-names.txt` |

Three files are maintained by hand and carry blocks derived from generated data:

- `m68k_regfile_si.sv`: derived from the output of `gen_regfile.py` and then edited (header of the file).
  `gen_regfile.py` is retired: it refuses to write, and its code remains as the record of how the register file
  follows from the nano atoms.
- `m68k_nrom.sv`: no generator writes it. Its ORIGIN sub-tables are the constants `C2_T*_B*`, written by hand from
  `m68k_norigin.mem` and compared with it at every clock under Verilator.
- `m68k_nstore_dec.sv`: the slot masks with the functions `f_groups` and `f_drv` (the RAW halves of the driven flags,
  registered in `gd`) and the assignments `gq_c` (the selectors of the non-RAW class, registered in `gq`) are the
  blocks that `gen_bus_sec.py` writes into the folder named by `GEN_DIAG_DIR`; `f_groups` is extended by hand with
  the 15 read-port selectors of the register file.

For the files that regenerate exactly, a change goes into the generator and the file is regenerated, not edited.

## 10. Verification

- `ustore` (`sim/tb_m68k_ustore.cpp`): the micro ROM and the sequencer against `mc68000_micro_decoded.csv` and
  `ustore.tsv`, all 1024 addresses.
- `nstore`, `ndec` (`sim/tb_m68k_nstore.cpp`, `sim/tb_m68k_ndec.cpp`): the nano ROM structure and its decoded
  content against App. H, the die map and `nstore.tsv`, all 1024 addresses.
- `a123_si` (`sim/tb_m68k_a123_si.cpp`): the A1/A2/A3 entry points against `mc68000_a123_die.tsv` and `a123.tsv`, all
  65,536 opcodes.
- `alu_si`, `rowdec`, `branch_si` (`sim/tb_m68k_alu_si.cpp`, `sim/tb_m68k_rowdec.cpp`, `sim/tb_m68k_branch_si.cpp`):
  FIG. 17, FIG. 18 and App. C, read from the transcriptions at run time.

All are part of `make -f Makefile.si test` in `sim/` (see `sim/README.md`).
