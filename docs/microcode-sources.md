# MC68000 microcode: sources

## 1. Purpose

The control store of rm68000 is not designed: it is taken from published data
about the MC68000. That covers the micro ROM, the contents of the nano ROM, the
instruction dispatch decoders, the conditional-branch table and the ALU control
matrix. This document lists the sources of those data, says what each one
contains and how it was checked, and names the file in `docs/data/` that holds
its transcription or the data derived from it.

The structure that the data describe, and how the core implements it, is in
[microcode-structure.md](microcode-structure.md). The list of chip data used by
the core, file by file, is in [chip-data-sources.md](chip-data-sources.md).

Only two kinds of source are admitted (top-level `README.md`, "How this was
written"): manufacturer documentation, and published analysis of the die.
Section 6 lists what is excluded.

## 2. Sources

### 2.1 US patent 4,325,121

- *Two-level control store for microprogrammed data processor*. Inventors
  Thomas G. Gunter and Harry L. Tredennick, assignee Motorola, Inc.,
  Appl. No. 41,135, filed May 21, 1979, issued April 13, 1982 (front page of
  the patent). The patent has expired.
- Text of the description, without the appendices:
  <https://www.freepatentsonline.com/4325121.html>. The full document is a
  177-page scan with no text layer. `docs/data/us4325121_google_image_urls.txt`
  lists the address of the full PDF on Google's patent image store, and two
  hosted copies of each scanned page from 52 to 174.
- Layout of the scan: page 1 is the front page. Pages 2–52 are drawing sheets 1–51.
  From page 53 on, each page holds two text columns: page 53+N carries columns 2N+1
  and 2N+2. Citations in these documents use **[P col. N]** for a text column,
  **[P FIG. N]** for a figure and "page N" for a scanned page.

What the patent contains that the core uses:

| part | where | content | transcription in `docs/data/` |
|---|---|---|---|
| Appendix A | col. 33–48 | the 544 microwords: label, function code, next address (NMA), type, CBC, ORIGIN, row, destinations | `us4325121_appA.csv`: columns 33–36 only (136 microwords) |
| Appendix B | col. 49–50 | abbreviations of the nanoword contents | — |
| Appendix C | col. 51–52 | conditional branch choices: CBC code, variable, source, values, C1 C0 | `us4325121_appC.tsv` (21 codes) |
| Appendices D, E | col. 53–54 | condition variables; the conditions of Bcc/Scc/DBcc | applied directly in `m68k_branch_si.sv` ([microcode-structure.md](microcode-structure.md) section 6) |
| Appendix F | col. 55–56 | the macroinstructions | — |
| Appendix G | from col. 57 | opcode formats | — |
| Appendix H | pages 85–175 | one block per microword: label, address, origin, access label, type, ALU function, register pointers, transfers, next label | `us4325121_appH.tsv` (500 blocks) |
| FIG. 17 | sheet 26 | ALU function and condition-code table, 15 rows × 5 columns | `mc68000_alu_matrix_fig17.tsv` |
| FIG. 18 | sheets 27–28 | ALU row decoder: IRD bit patterns and row inhibits | `mc68000_alu_rows_fig18.tsv` |
| FIG. 19A/19B | sheets 29–30 | the PLA that implements FIG. 18 | `mc68000_alu_rows_fig19_check.tsv`, `mc68000_alu_rows_fig19_recheck.tsv` (line-by-line comparison with FIG. 18) |
| FIG. 21A–21U | sheets 31–51 | A1/A2/A3 entry labels per instruction and addressing mode | `us4325121_fig21_a123.tsv` |

Each transcription file states its source page or sheet in its header.
`docs/data/scripts/apph_crops.py` cuts the Appendix H page images into columns
for reading. `docs/data/scripts/apph_check.py` checks the Appendix H transcription
against Appendix A for TYPE and ORIGIN. `docs/data/scripts/validate_appA.py` checks
the Appendix A transcription against the die listing (section 2.3).
`docs/data/ftu_findings.md` collects what the patent says about the field
translation unit.

Structural facts stated by the patent text and used in
[microcode-structure.md](microcode-structure.md):

- "Micro ROM 72 in FIG. 4 contains 544 microwords each having 17 bits. The micro
  ROM is addressed by the 10-bit output of address selection block 64 such that
  up to 1024 microwords could be addressed." [P col. 15–16]
- "each nanoword is 68 bits wide and is decoded to drive approximately 180
  control points within the execution unit. The number of unique nanowords in
  the nano control store is 336" [P col. 17–18].
- "Access to the nano control store is concurrent with access to the micro
  control store. However, while there is a one-to-one mapping in the micro
  control store between addresses and unique microwords, there is a many-to-one
  mapping of control store addresses" to nanowords [P col. 17–18]. The ORIGIN
  column of Appendix A records which nanoword a microword shares.
- "the signals conducted by conductors 120, 122, and 124, and the address
  conducted by line 116 are all derived from the microword addressed during the
  previous micro cycle" [P col. 11–12].
- Microword formats I and II (FIG. 8) [P col. 15–18]: see
  [microcode-structure.md](microcode-structure.md) section 3.
- "The execution unit is a segmented two-bus structure divided into three
  sections by bidirectional bus couplers" [P col. 5–6].

### 2.2 Related applications named by the patent

US 4,325,121 lists four co-pending applications by the same inventors, in its
section "Cross Reference to Related Applications":

- Ser. No. 961,798, *Execution Unit For Data Processor Using Segmented Bus Structure*.
- Ser. No. 041,202, *Instruction Register Sequence Decoder For Microprogrammed Data Processor And Method*.
- Ser. No. 041,203, *Conditional Branch Unit For Microprogrammed Data Processor*.
- Ser. No. 041,201, *ALU And Condition Code Control Unit For Data Processor*.

No data of the core comes from them. Everything the core uses comes from
US 4,325,121 itself and from the die (section 2.3).

### 2.3 Die analysis by Olivier Galibert

Published at <https://og.kervella.org/m68k/>: die photographs, the regenerated
schematic, and listings read from the die. The listings are not redistributed in
this repository, because the page declares no licence for them. They are used as a
source of facts about the chip, not as code. The scripts that need them take their
folder as an argument. These are the files used:

| file | content | read by |
|---|---|---|
| `microcode.txt` | raw bits of the two ROMs as read from the die (section 3) | `decode_micro.py`, `nano_map.py`, `die_only_words.py`, `nano_fields.py`, `nano_fields_greedy.py` (and `nano_corr.py`, `nano_tree.py` through `nano_fields.py`) |
| `rommap.txt` | for each ROM row of `microcode.txt`, the micro addresses it serves | the same scripts |
| `adr-names.txt` | micro address → routine name (544 lines) | `decode_micro.py`, `apph_match.py`, `nano_map.py`, `die_only_words.py`, `gen_a123.py`, `gen_nrom.py` |
| `all.txt` | symbolic listing of the microcode read from the die: 1642 sections (the five states reset, bus error, double fault, trace and interrupt, and 1637 instruction variants), covering 512 distinct micro addresses | `apph_match.py`, `die_only_words.py`, `validate_appA.py`, `nano_fields.py`, `nano_fields_greedy.py`, `gen_a123.py`, `gen_branch.py` (cross-check), `gen_nrom.py` (only to resolve the `*` buses of Appendix H) |
| `a1-decode.txt`, `a23-decode.txt` | the A1 and A2/A3 dispatch PLAs | `die_a123.py` |
| `a1-dump.txt` | decoded entries of the A1 PLA | `apph_match.py`, `gen_a123.py` |
| `m68000.lst` | opcode, mask, mnemonic and modes, 1521 lines | — (reference) |

All scripts named in this document are in `docs/data/scripts/`.

### 2.4 Motorola manuals

- *M68000 8-/16-/32-Bit Microprocessors User's Manual*, Rev. 8 (MC68000UM).
  It is the source of the bus cycles, exception processing and instruction timing
  ([instruction-timing.md](instruction-timing.md)).
- *M68000 Family Programmer's Reference Manual* (M68000PRM), for instruction
  semantics and condition codes.

The manuals describe what the processor does at its pins and in its registers.
They do not describe the microcode. Together with the measurements made during
development against an external reference core, they are the sources of the deviation files in `docs/data/deviations/`, where the
core departs from the literal microcode data (section 5).

## 3. Reading the raw bits

`microcode.txt` holds the complete raw contents of the two ROMs, not a list of
decoded fields. The reading below uses only the bit semantics published in the
patent. It is implemented by `decode_micro.py`, whose header records the physical
layout.

- The file has 118 lines of 272 bits: 32,096 bits, exactly 544 × 17 + 336 × 68,
  the sizes given by the patent.
- Lines for rows 118 down to 85 (34 rows) are the micro ROM. Each row holds 16
  microwords as 17 groups of 16 bits: group g holds patent bit 16 − g, and
  microword w of the row is at the row's base address + w (`rommap.txt`).
  34 × 16 = 544.
- Lines for rows 84 down to 1 (84 rows) are the nano ROM. Each row holds 4
  nanowords as 68 groups of 4 bits; a micro address uses the nanoword at position
  `address & 3` of its row. 84 × 4 = 336. The rows serve address ranges of 4, 8,
  12, 16 or 32 addresses (54, 20, 2, 6 and 2 rows), so one nanoword serves several
  micro addresses, as the patent states. Four ranges lie inside another range;
  there the most specific range applies, which is how `nano_map.py`,
  `nano_fields.py` and `die_only_words.py` resolve it. `decode_micro.py`, which writes the `nano_row` and
  `nano_slot` columns, does the same. No generator or bench reads those two columns.
- In the micro ROM, bit 0 (I), bit 1 (format), bits 2–3 (TY), bits 2–6 (CBC)
  and bits 15–16 (FC) sit at the patent's positions. The ten bits of the next
  micro address are physically permuted with respect to the patent:
  A0←b11, A1←b12, A2←b7, A3←b8, A4←b9, A5←b10, A6←b5, A7←b6, A8←b13, A9←b14
  (`PERM` in `decode_micro.py`; the core applies the same permutation in the
  assignment of `nma` in `rtl/cpu_si/m68k_useq.sv`). This agrees with the patent text: in format
  II, bits 5–6 belong to the CBC, and the two address bits they would carry (A6,
  A7) come from the branch generator as C0/C1.

Checks of this reading:

- `docs/data/mc68000_micro_decoded.csv` is the output of `decode_micro.py`. Its
  `raw17` column equals the bits read from `microcode.txt` for all 544
  microwords.
- Against the listing `all.txt` (516 listed addresses), all 70 conditional
  branches agree, in condition name and in base address with bits 7:6 at 0. So do
  the 399 numeric jumps and the 47 dispatches (`jump a1/a2/a3`, words of format I
  with TY 1, 2 and 3). The listing repeats a word in every section that reaches
  it, always with the same jump (blocks at the pseudo-address `-1d` are not words
  of the ROM and are not counted).
- The format-II words use 21 CBC codes over 15 condition variables. The six codes
  with the top CBC bit set (11, 12, 15, 19, 1c, 1e) are second output assignments
  of the same variable, as Appendix C lists them.
- The nano ROM has 336 physical words with 306 distinct contents.
- The 49 microwords that have an ORIGIN entry in the transcribed part of
  Appendix A all share their nanoword with their origin in the raw bits. For 4 of
  them this holds only after the w/l exchange between the die's routine names and
  the patent's labels (for example the patent's `adsl2` is the die's `adsw2`),
  which `us4325121_appH_die_map.tsv` records for every label.

What the bits do not give is the nanoword decoder: the meaning of the 68 bits,
the patent's "approximately 180 control points". `all.txt` is the die analyst's
rendering of that decoder for the transfers. It does not cover the ALU, condition
code, field translation and trap-vector controls. Appendix H covers all of them in
symbolic form. The core therefore takes the **contents** of its nano ROM from
Appendix H, placed at die addresses through the die map, and uses its own encoding
([microcode-structure.md](microcode-structure.md) section 5). The micro ROM, by
contrast, is the die bits themselves.

## 4. Role of each source

| use | primary source | cross-check |
|---|---|---|
| structure: formats, sequencing, A1/A2/A3, branch, ALU matrix, access classes, execution unit | US 4,325,121 text and figures | die listing `all.txt` |
| micro ROM contents | die raw bits (`microcode.txt` → `mc68000_micro_decoded.csv`) | Appendix A (columns 33–36), `all.txt` jumps |
| nano ROM contents | Appendix H (`us4325121_appH.tsv`), placed by `us4325121_appH_die_map.tsv` | `all.txt` transfers, die raw bits (`mc68000_nano_fields.tsv`) |
| microwords present on the die and absent from Appendix H | die raw bits and `all.txt` (`mc68000_die_only_words.tsv`) | — |
| A1/A2/A3 dispatch and illegal/privileged decode | die PLAs (`mc68000_a123_die.tsv`) | FIG. 21 (`us4325121_fig21_a123.tsv`) |
| conditional-branch table | Appendix C (`us4325121_appC.tsv`) | the 70 conditional branches of the die micro ROM |
| ALU and condition-code control | FIG. 17 and FIG. 18 | FIG. 19A/19B |
| timing and behaviour seen at the pins | MC68000UM, M68000PRM | comparison with an external reference core during development (benches not part of this repository, available from the author on request) |

## 5. Data in `docs/data/` and how they reach the RTL

| file | origin | script | used by |
|---|---|---|---|
| `mc68000_micro_decoded.csv` | die raw bits | `decode_micro.py` | `gen_urom.py` → `rtl/cpu_si/m68k_urom.mem`; bench `ustore` |
| `us4325121_appH.tsv` | Appendix H, transcribed by hand | — (`apph_crops.py`, `apph_check.py`) | `gen_nrom.py`; benches `nstore`, `ndec` |
| `us4325121_appH_die_map.tsv` | Appendix H label → die address, checked against `all.txt` | `apph_match.py` | `gen_nrom.py`; benches `nstore`, `ndec` |
| `mc68000_nano_fields.tsv` | nano bits ↔ Appendix H atoms, by correlation | `nano_map.py` | `die_only_words.py` |
| `mc68000_nano_map_apply.tsv` | the same map applied to the Appendix H blocks whose match with the die listing is a mismatch or ambiguous | `nano_map.py` | — (record) |
| `mc68000_die_only_words.tsv` | the 14 die microwords absent from Appendix H | `die_only_words.py` | `gen_nrom.py`; source of lines of `deviations/nstore.tsv` |
| `mc68000_nano_encoding.tsv` | the core's nano encoding: slots and codes | `gen_nrom.py` | `gen_nano_dec.py`, `gen_regs.py` |
| `mc68000_nrom_reach.tsv` | micro words reachable from reset, exceptions and the dispatch PLAs | `nrom_reach.py` | — (record) |
| `mc68000_a123_die.tsv` | A1/A2/A3, illegal and privileged for all 65,536 opcodes, from the die PLAs | `die_a123.py` | `gen_a123.py` → `m68k_a123_si.sv`; bench `a123_si` |
| `mc68000_a123_need.tsv` | opcodes whose flow actually reads A2 | `a123_need.py` | `gen_a123.py`; bench `a123_si` |
| `us4325121_fig21_a123.tsv` | FIG. 21A–21U | — | `gen_a123.py` → `m68k_a123_si.sv` |
| `us4325121_appC.tsv` | Appendix C | — | `gen_branch.py` → `m68k_branch_pkg.sv`; bench `branch_si` |
| `mc68000_alu_matrix_fig17.tsv` | FIG. 17 | — | `gen_alu.py` → `m68k_alu_pkg.sv`; benches `alu_si`, `alux_si` |
| `mc68000_alu_rows_fig18.tsv` | FIG. 18 | — | `gen_alu_rowdec.py`; benches `rowdec`, `alux_si` |
| `mc68000_alu_rowdec_dump.txt` | the row decoder evaluated on every instruction word | `sim/dump_rowdec.cpp` | `gen_rowdec_case.py` → `m68k_alu_rowdec.sv` |
| `us4325121_appA.csv` | Appendix A, columns 33–36 | — | `apph_check.py`, `validate_appA.py` |
| `mc68000_dec_raw_mask.txt` | the 199-bit mask `DEC_RAW` of `rtl/cpu_si/m68k_nstore_dec.sv` | — | — (record) |

Two more scripts, `gen_bus_sec.py` and `gen_regs.py`, generate the bus fabric
(`m68k_bus_sec.sv`) and the bus-side registers (`m68k_regs_bd.sv`) from the nano
encoding and the rules of [silicon-datapath.md](silicon-datapath.md).
`gen_regfile.py` is no longer used: its header states that
`m68k_regfile_si.sv` is maintained by hand. The generated files are reproduced
exactly by their generators; the files maintained by hand are listed in
[chip-data-sources.md](chip-data-sources.md) section 9. Some generators also read the die listings, which
are not in this repository.

Where the core departs from the literal data, the departure is one line in
`docs/data/deviations/` with its source: patent page, manual section, die listing,
or the comparison case, made during development, that measured it. The micro ROM has five such lines
(`ustore.tsv`) and the nano ROM 100 (`nstore.tsv`).
[microcode-structure.md](microcode-structure.md) section 3 (Contents of the core's micro ROM) and section 5
summarize them.

## 6. Excluded sources

- The source code of any existing 68000 core or emulator, under any licence.
  This includes micro- and nanocode tables embedded in emulator source code, even
  where they were derived from the same die analysis: the core takes its data from
  the patent and from the die listings directly.
- Schematics or service manuals of machines built around the 68000, commercial
  books, and wikis without traceable provenance (top-level `README.md`).

During development an external reference core was used only as a black box, in
comparison benches that are not part of this repository (available from the author
on request). Only its pins were compared, and its source was not read.

## 7. Verification

The block benches read these data files at run time and compare the RTL with
them, never with a generated file (`sim/README.md`). They run with
`make -f Makefile.si test` in `sim/`.

| bench (`sim/Makefile.si` target) | file | data read |
|---|---|---|
| `ustore` | `sim/tb_m68k_ustore.cpp` | `mc68000_micro_decoded.csv` + `deviations/ustore.tsv` |
| `nstore` | `sim/tb_m68k_nstore.cpp` | `us4325121_appH.tsv`, `us4325121_appH_die_map.tsv` + `deviations/nstore.tsv` |
| `ndec` | `sim/tb_m68k_ndec.cpp` | the same, decoded atom by atom (model `sim/m68k_apph_model.h`) |
| `a123_si` | `sim/tb_m68k_a123_si.cpp` | `mc68000_a123_die.tsv`, `mc68000_a123_need.tsv` + `deviations/a123.tsv` |
| `branch_si` | `sim/tb_m68k_branch_si.cpp` | `us4325121_appC.tsv` + `deviations/branch.tsv` |
| `rowdec` | `sim/tb_m68k_rowdec.cpp` | `mc68000_alu_rows_fig18.tsv` + `deviations/rowdec.tsv` |
| `alu_si` | `sim/tb_m68k_alu_si.cpp` | `mc68000_alu_matrix_fig17.tsv` + `deviations/alu.tsv` |
| `alux_si` | `sim/tb_m68k_alux_si.cpp` | the FIG. 17 and FIG. 18 transcriptions, with the real control store |

The checks of the raw-bit reading in section 3 compare the output of
`decode_micro.py` with `microcode.txt` and `all.txt`, which are not in this
repository. They can be repeated with the published files.
