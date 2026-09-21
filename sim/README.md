# sim/ — verification benches

Every bench here builds the RTL with Verilator, runs it, prints a verdict and
exits non-zero on failure. Nothing needs setting up: no ROMs, no external cores,
no configuration.

## Requirements

Verilator 5.x and a C++17 toolchain. On Windows under MSYS2 the make binary is
`mingw32-make` and Verilator comes with the MinGW64 environment of MSYS2.

## Running them

```sh
make -f Makefile.si test    # the block benches, with a summary
make -f Makefile.si all     # the instruction benches
make -f Makefile.si a123_si # any single bench, by name
make -f Makefile.si clean   # remove build artefacts
make -f Makefile.cendtack cendtack   # the clock-enable and DTACK generator bench
```

`test` is the one to start with. It prints:

```
  PASS  ustore
  PASS  nstore
  ...
  PASS  cpu_si

13 PASS, 0 FAIL
```

Build artefacts land in `build_si/` (and `buildcd/` for the DTACK bench), which git
ignores.

## What the benches are

**Block benches** (`test`, 13 of them: twelve on the blocks and `cpu_si` on the whole
CPU). Each block bench takes one block of the core, in the
configuration the core instantiates it with, and compares it against a model that
is independent of the RTL and of the generators: the model is written in C++ from
the published data, which the bench reads at run time from `docs/data/` (the
patent transcriptions, the die tables) or from the manufacturer's manual (register
selection, operand sizes, instruction semantics). No generated file is read: not the
`.mem` images, not the generated SystemVerilog, not any expectation header produced
by a generator. The
input space is covered exhaustively wherever it is finite, and with structured
patterns (zeros, ones, walking bits, boundaries, carry chains) where it is not.
Random stimulus appears only as the 200,000-step pipeline walks of the three store
benches (which check consistency with the exhaustive pass, not expectations) and as
a small tail of extra ALU vectors; every expected value comes from the data.

| bench | what it checks | model from | coverage |
|---|---|---|---|
| `ustore` | micro ROM contents, sequencer (next address, dispatch, conditional branch, override) | die micro ROM dump `mc68000_micro_decoded.csv` (17 raw bits per word) | all 1024 addresses: 430 direct, 48 dispatch (each against all 1024 A1 and A0, or A2/A3), 70 conditional (x 4 C1C0), 476 empty; plus a 200,000-step random walk; 752,841 checks |
| `nstore` | nano ROM structure, encoding-independent: unused addresses empty, equal content ⇒ equal word, word read one clock early (`nw_pre`, `nwl_pre`), immediate override 000/003 = normal path, index | patent App. H + die map (which addresses hold a word, which share content) | all 1024 addresses (522 defined, 502 unused), 107 shared groups over 256 addresses, 215,362 enable edges, 200,000-step random walk; 632,405 checks |
| `ndec` | nano ROM contents decoded into the resource vector `dec`, atom by atom | patent App. H transcription (transfers, access, ALU and pointer columns) placed by the die map, three declared rules, documented deviations | all 1024 addresses, 5,878 atoms; override 000/003; 200,000-step random walk; 202,052 checks |
| `a123_si` | instruction decode to microcode entry points A1/A2/A3, illegal and privileged, with the row class computed outside as in the core | die PLA table `mc68000_a123_die.tsv`, A2-read table `mc68000_a123_need.tsv`, documented deviations (word/long EA chain pairs, two A3 aliases, MOVE USP privilege) | all 65,536 opcodes: 45,815 valid (A1 on all, A3 on 37,578, 378 of them through the two aliases, A2 on the 1,492 that read it, privilege on all), 11,529 illegal (flag and A1 = illegal entry); the 8,192 line A/F opcodes are left to the exception block |
| `size_si` | operand size from the instruction word | manual (instruction formats) | all 65,536 opcodes |
| `rowdec` | ALU row decode from the instruction word: 15 rows, inhibit vector, condition-code inhibit | patent FIG. 18 terms `mc68000_alu_rows_fig18.tsv` (45 terms, 13 qualifiers; 46 terms with the DBcc term of `rowdec.tsv`), documented deviations | all 65,536 opcodes |
| `alu_si` | the 16-bit ALU: every operation and condition-code cell of the matrix, result, extension, carries, flags | patent FIG. 17 matrix `mc68000_alu_matrix_fig17.tsv` (81 cells), documented deviations | 33,513,181 vectors: every byte pair for the adders and logic (25,231,360), word patterns (3,741,696), shifts (1,327,104), every EXT input (3,145,728); 301,618,629 checks |
| `alux_si` | the ALU with its surrounding registers (ALUE, ALUB, carry latch, correction factor, DCR, LOCZ) driven by the real control store | FIG. 17/18 models + the datapath document rules | all 522 microcode words x 56 instruction classes x byte/word x condition codes, each word followed by up to 3 successor words (carry latch initiate → finish); 1,442,409 microcycles, 79,051 ALU operations; 10,172,425 checks |
| `bussec` | the internal bus sections (AB/DB high, low, data) and the "driven" flags, for every driver atom | `docs/silicon-datapath.md` §5 rules, patterned sources, documented deviations | all 522 words (+502 empty addresses), 77 bus atoms, 8,854 patterns; 108,712 checks |
| `regs` | bus-side registers AT, AOB, DOB, DBIN: loads and holds from the sections | `docs/silicon-datapath.md` §6 rules | all 522 words, 61,778 microcycles (29,830 with loads); 436,109 checks |
| `regfile_si` | register file and MOVEM encoder in the core configuration: register selection (types, pointers, index register), writes on the two ports with half-word and byte rules, read ports at en+1 and en+2 (forwarding of the queued write), the whole register state after every microcycle | datapath document §8/§13 + manual (instruction formats) | all 522 words x 13 instruction classes x S (one class on all 64 RX/RY pairs), 895,964 microcycles, 223,154 reads verified; 20,127,147 checks |
| `branch_si` | condition-code unit: every table entry and composition | patent App. C `us4325121_appC.tsv` (21 codes, 68 entries), documented deviations | 32 codes x 131,072 inputs; the 512-entry table: 68 reachable entries all selected, 268 unreachable never selected; 6,947,243 checks |
| `cpu_si` | the whole CPU: reset sequence, a hand-assembled self-checking program (MOVE/MOVEQ/MOVEA, ADD/SUB/ADDQ with carry, MOVE SR, byte/word/long memory accesses, LEA/SUBA/ADDA, BSR/RTS, MOVEM, STOP), memory image, register dump, PSW, function codes, UDS/LDS on byte accesses, cycle order, E clock | manual (encodings and semantics) | 35 instructions, 101 bus cycles (52 fetches, 45 data, 4 vectors, 39 writes), every cycle checked; run twice, with either clock phase first; 350 checks per run |

**Deviations.** Where the core differs from the literal published data, the
difference is not hidden in the model: it is one line in a deviation file under
`docs/data/deviations/` (`rowdec.tsv`, `branch.tsv`, `ustore.tsv`, `alu.tsv`,
`bussec.tsv`, `nstore.tsv`, `a123.tsv`; 153 lines in all), each line carrying its
source — the patent page or transcription note, the manual section, the die trace
file and row, or the cycle-by-cycle comparison, made during development, that
measured it. The bench
applies the file and reports how many lines it used; `make` passes each file through
its own variable (`DEV_ROWDEC`, `DEV_BRANCH`, `ADD_USTORE`, `DEV_ALU`, `DEV_BUSSEC`,
`DEV_NSTORE`, `DEV_A123`), and an empty variable runs the literal data against the
core, which is the point. In `bussec.tsv` 17 of the 19 lines are implementation
choices that no behaviour can observe (a 16-bit register half placed on both halves
of a bus whose readers fall back across sections); they are marked as such, and so
are 2 of the 5 lines of `ustore.tsv` and 2 of the 100 of `nstore.tsv` (the words 3fe
and 3ff, which no path of the core reaches).

**Fault injection.** During development every bench was validated by compiling a
copy of its block with a hand-made fault (a wrong constant, a swapped bus, a missing
term, a timing slip) and checking that the bench fails — 64 faults over the 13
benches. Where the RTL's own simulation-only self-checks would have caught the fault
before the bench, they were disabled in the copy, so that the detection is the
bench's. Faults that the bench cannot distinguish from the real block are declared
as such. The faulty copies are not part of this repository; they and their results
are available from the author on request.

**Instruction benches** (`all`, 84 targets: the instruction benches, `busfsm` and `cpudt`). Each runs a small program on the core
and checks what it produces: registers, condition codes, memory byte by byte (a
write outside the expected areas fails), bus cycles (function codes, data strobes,
order) and exception frames. There is one bench per instruction family: addressing
modes, MOVE and MOVEM, arithmetic and BCD, shifts and rotates, MULU/DIVU including
division by zero, bit operations, LEA/PEA/LINK, branches and DBcc, exceptions,
interrupts and the IACK cycle, tracing, STOP, privilege violation, bus error and
double fault, HALT and bus arbitration. `busfsm` checks the bus state machine on its
own, edge by edge. Expected values are worked out from the Motorola manuals (PRM, UM),
not taken from the core. Each bench prints `FAIL: <what> expected=<expected>
read=<read>` for every failed check and ends with `RESULT: PASS` or `RESULT: FAIL`
(exit code 0 or 1). `make -f Makefile.si all` stops at the first failing bench;
`make -k -f Makefile.si all` runs them all.

**Mutation testing.** During development the instruction benches were validated the
same way as the block benches, on the whole core: 152 copies of the RTL, each with
one hand-made change, each listing the benches that must fail. All 362
mutation-bench pairs fail as required. Where the RTL's own simulation-only
self-checks would have caught a mutation before the bench, they were disabled in the
copy. Mutations that turned out not to change the core's behaviour (the changed line
is not on that instruction's path) were replaced by ones on the real path. The
mutated copies are not part of this repository; they and their results are
available from the author on request.

This work found four deviations from the manual, now corrected (see `docs/`):
BERR during the interrupt acknowledge now takes the spurious interrupt vector 24
(UM 6.3.2, 6.3.4); a bus error during bus-error processing now halts the processor
(UM 5.4.4); BG is no longer asserted in the clock that starts a bus cycle, before AS
(UM 5.2.2); and the TAS read-modify-write no longer loses its write request when
both clock enables are high on every clock.

**DTACK bench** (`Makefile.cendtack`, target `cendtack`). It tests the clock-enable
and DTACK generator `rtl/integration/m68k_cen_dtack.sv` on its own: a phase-accurate
model of the processor's bus cycle and a memory model that holds DTACK drive it,
and the bench checks clock enables, DTACK, wait states and the pulses returned after
a stall against the rules of `docs/clock-dtack.md`, computed at run time.
`run_cendtack.sh` runs it on every supported divider and memory pattern and ends
with `VERDICT: PASS` or `FAIL`. The bench is described in `docs/clock-dtack.md`,
section 6.1.

## How the work was verified during development

During development every change to the core was also compared cycle by cycle, at
the pins, with an external 68000 core used as a black box on the same stimulus.
Those comparison benches rely on third-party code and on programs that cannot be
redistributed, so they are not part of this repository; the benches and their
results are available from the author on request.

## Conventions

- One bench per module or instruction family, `tb_m68k_<name>.cpp`; shared models
  in headers (`m68k_apph_model.h`, `m68k_alu_cells.h`, `m68k_rowdec_model.h`,
  `m68k_ndec_fields.h`).
- Exit code 0 means pass, non-zero means fail — usable from CI.
- A bench checks documented behaviour, from the manufacturer's manual or from the
  die and patent data under `docs/data/`, read at run time. It is written from the
  documentation, not from watching what the RTL happens to do, and it never reads a
  generated file.
