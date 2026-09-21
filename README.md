# rm68000 — an MC68000 in SystemVerilog

![version](https://img.shields.io/badge/version-1.0-blue) [![license](https://img.shields.io/badge/license-GPL--3.0--or--later-green)](LICENSE)

A from-scratch FPGA implementation of the Motorola MC68000, written from the
manufacturer's documentation and from published die analysis, and verified
cycle by cycle — in simulation and on real hardware (Cyclone V, DE10-Nano).

The core reproduces the 68000 **tick for tick**: the same bus accesses, in the
same order, on the same cycles, with the same pacing. It is not a
cycle-approximate interpreter, and it is not a translation of an existing core.

---

## Table of contents

- [Status](#status) · [Design goals](#design-goals) · [How it is built](#how-it-is-built)
- [Using the core](#using-the-core) · [Repository layout](#repository-layout)
- [How this was written](#how-this-was-written) · [Verification](#verification) · [Reproducing the tests](#reproducing-the-tests)
- [What is not here](#what-is-not-here) · [Documentation](#documentation) · [License](#license)

---

## Status

Working and in use. It runs on real hardware in the author's MiSTer arcade
cores, Darius among them, including save and restore of the full machine state.

What is implemented: the complete user and supervisor instruction set,
all addressing modes, the exception model (address error, bus error, double
fault and halt, privilege violation, illegal instruction, TRAP/TRAPV/CHK,
interrupts with autovector and vectored acknowledge, tracing, STOP/RESET),
the M6800 synchronous bus (E, VMA, VPA), and bus arbitration (BR/BG/BGACK).

## Design goals

rm68000 was written to be **tick-accurate** — the bus cycles, their timing and the
wait-state behaviour of the silicon — and, at the same time, to **run at high clock
frequencies**: it is meant to close timing with margin on mid-range FPGAs (the
Cyclone V class found in MiSTer and in the Analogue Pocket, for instance), not to fit
one board, while staying **small** next to the rest of a system.

For speed, logic depth was treated as a resource: the choices in *How it is built*
(two-level ORIGIN read one clock ahead, decoding per enumerated resource, AND-OR
buses over one-hot selectors, tabular ALU control) all exist to keep the paths short.
Around the core, the signals a system drives every clock are taken into flops next
to their users: the reset (in three copies), the pause, and the read data, latched
with DTACK. The clock enables and DTACK come from the registers of the generator,
in the same node as the processor (`m68k_cpu_dt_si`). The paths from the rest of a
design end at those flops instead of running into the core in the same clock.

No figures are given here: size and maximum clock depend on the design around the
core and on the tools, and the core is simple to measure in your own build.

## How it is built

Two non-overlapping phases and two-level microcode, the way the silicon does it:

- **micro ROM** 1024 x 17 bits and **nano ROM** 512 words, both in block memory;
- the **ORIGIN** table that maps a micro address to a nanoword index lives in
  logic, exactly two levels deep, and is read one clock ahead so the memory
  lookup is off the critical path;
- **nanocode decoding** is packed per *enumerated resource* rather than per
  atom, so each field bit depends on a single slot and pure slots collapse to
  wires;
- **register file** in distributed memory, with a parallel-form write side: per
  port and per half at most one write atom is ever active, which is checked with
  an assertion over every microcode word rather than assumed;
- **ALU** driven by two tabular stages — a one-hot row from the instruction,
  then (row x column) to a 5-bit operation code and a condition-code mask;
- a separate **address unit**, and internal buses as AND-OR over one-hot
  selectors instead of nested multiplexers.

The tables under `rtl/cpu_si/` — microcode, decoders, register file, ALU control —
were **generated** by the scripts in `docs/data/scripts/` from the die data. Each
generated file is reproduced exactly by its generator, license header included.
Three files are maintained by hand and carry blocks derived from generated data:
the register file (`m68k_regfile_si.sv`, whose generator `gen_regfile.py` is retired
and refuses to write), the ORIGIN table (`m68k_nrom.sv`, compared with
`m68k_norigin.mem` in simulation) and the bus-section selectors
(`m68k_nstore_dec.sv`). The generators read their data from `docs/data` (some also
read the die trace, which is not in this repository) and need Python 3.12 or later.
A change to a generated file goes into its generator, and the benches are re-run;
`docs/chip-data-sources.md`, section 9, lists each file with its generator.

## Using the core

The CPU is `m68k_cpu_dt_si` (`rtl/cpu_si/m68k_cpu_dt_si.sv`): the processor
`m68k_cpu_si` and its clock-enable and DTACK generator `m68k_cen_dtack`, in one
module. It presents the real 68000 pins — `addr`, `as_n`, `uds_n`, `lds_n`, `rw`,
`data_out`, `fc`, `ipl_n`, `vpa_n`, `vma_n`, `e`, `berr_n`, `halt_n`, `br_n`,
`bg_n`, `bgack_n` — and, towards the system:

- `num` / `den`, the clock divider: the fabric clock stays fast and the CPU is
  paced at the target frequency;
- `bus_cs` / `bus_busy`, the memory handshake from which DTACK is generated: the
  bus cycle is stretched while the memory is busy, and the cycles lost to a stall
  are recovered;
- `bus_rdata`, the read data, latched inside;
- `pause_n`, which stops the CPU between clock enables;
- `reset`, **active high** (the pin of the part is active low).

The module has more ports than these. `cen_o`, `cenb_o` and `dtack_n_o` give the
surroundings the clock enables and DTACK; `data_oe`, `iack`, `halted_n`, `bus_tri`,
`reset_o` (the RESET output, asserted by the RESET instruction) and `dbg_pc` are
outputs of the processor. `cen_raw_o`/`cenb_raw_o`,
`cen_ext_i`/`cenb_ext_i`, `gate_q_i`/`gate_raw_i` and
`ss_ovr_i`/`ss_word_i`/`ss_ovr_c_i`/`ss_word_c_i` serve a savestate path inside the
module, enabled by the parameter `SS_EXT`: with `SS_EXT = 0` (the default), tie
those inputs to 0. The header of `m68k_cpu_dt_si.sv` describes
each port.

Two more files sit outside `rtl/cpu_si/`:

| file | what it is for |
|---|---|
| `rtl/integration/m68k_cen_dtack.sv` | the clock-enable and DTACK generator, instantiated inside `m68k_cpu_dt_si`: part of the CPU |
| `rtl/savestate/m68k_ss_seq.sv` | external savestate, optional: pauses N CPUs, reads out and restores the entire architectural state without a debug port on the core. Its section is 2 words per CPU: the slave that connects it to your system's savestate bus (not part of this repository) must report a size of 2N words for N CPUs. See `docs/external-savestate.md`, section 4.8 |

## Repository layout

```
rtl/cpu_si/        the CPU: SystemVerilog modules and microcode files
rtl/integration/   the clock-enable/DTACK generator used by the CPU
rtl/savestate/     the external savestate, optional
docs/              one document per block: behaviour and timing, with the
                   source of every claim noted
docs/data/         raw data (die tables, ALU matrices, masks) and the
                   generators that turn it into RTL
sim/               Verilator benches
```

## How this was written

The method was fixed before the first line of RTL and was not relaxed afterwards.

### Which sources were allowed

Two classes, and nothing else:

1. **Manufacturer documentation** — the MC68000 user's manual and programmer's
   reference. A datasheet describes a part its maker published; it is the
   intended way to learn what the part does.
2. **Published analysis of the die** — principally the appendices and figures of
   the original Motorola patent (US 4,325,121): the contents of the micro ROM
   and nano ROM, the ALU operation matrices, the register-file address tables,
   the die map. These are published tables of a part shipped since 1979.

And the list of what was deliberately **not** used is the more important half:

- no schematics or service manuals of any machine built around the part;
- no commercial books or wikis documenting undocumented behaviour, because the
  problem is not that someone reverse-engineered it — it is that the claim
  arrives without provenance and cannot be checked;
- no vendor binaries read as a behavioural reference;
- **no source of any existing core or emulator**, under any licence.

### Document first, RTL second

No block was written until there was a document for it in `docs/` describing its
behaviour and timing from the allowed sources — and **each claim carries its
source inline**, naming the manual section, the patent figure or the die table
it comes from. The RTL was then written from the document. Not the other way
round, and never from memory of how somebody else had done it.

That is why `docs/` has one file per block and why it is the bulkier half of the
repository.

### Tables are data, not code

The microcode, the nanocode, the ALU row and column matrices, the register-file
address maps: all of it lives as plain tables in `docs/data/`, transcribed from
the published source and checked against it, and the RTL was emitted from them by
the generators in `docs/data/scripts/`. The 500 microwords of the patent's Appendix H
were transcribed in full and re-checked word by word; our micro-instruction decode
was validated at 100% against the published listing before any of it reached a
simulator.

So `rtl/cpu_si/m68k_nrom.mem` is not something anyone typed: it was generated from
the table. The block benches do not go through the generators at all: each one reads
the published data itself at run time — the patent transcriptions and die tables
under `docs/data/`, or the manual's definitions written out in C++ — and checks the
RTL against them. Where the core deliberately departs from the literal data, the
departure is a line in `docs/data/deviations/` with its source (patent page, manual
section, die trace row, or the cycle-by-cycle comparison, made during development,
that measured it), and the bench
applies that line and says so. Some generators also read the die trace, which is not
in this repository; the three files maintained by hand are listed in "How it is
built".

### The comparison core was a black box, and stayed one

Reaching *tick for tick* means finding the cycle where you differ, and for that
you need something to differ from. So during development an existing 68000 core
ran beside this one in the same testbench: same clock, same reset, same memory
model, same returned data, same interrupts. Every cycle the harness compared the
**pins** — address, `AS`, `UDS`, `LDS`, `R/W`, data out, function codes — and
stopped at the first cycle where they disagreed, naming the signal and the cycle
number.

The discipline around it was explicit:

- **only the outside was looked at.** Its source was not read, not copied, not
  paraphrased. It entered the build as a file path, and what the harness saw were
  its pins.
- **whoever wrote the RTL did not open it.** Where something about it had to
  be established, it was established by someone who was not writing the core,
  and handed over as a description of behaviour.
- **it never entered the repository.** It was referenced through a path
  variable, from outside. It is third-party code: bundling it would put into the
  repository code that is not the author's, and the commercial license under which
  rm68000 is also offered requires all of its code to be the author's. The comparison benches are not part of this
  repository either; they and their results are available from the author on request.

Comparing two boxes is not deriving one from the other. What a 68000 does on its
pins has been published for four decades; this core reproduces that behaviour
from those publications, and the comparison was used only to find out where it
had not got there yet.

### Nothing was accepted on the strength of an argument

Every change had to survive the full chain before it landed, and every
optimisation had to be **measured**, not reasoned about. The ones that did not
pay were measured and discarded.

## Verification

The benches in `sim/`, re-run on this tree before publishing, and the hardware:

1. **Per-block benches** — twelve, one per block, each in the configuration the
   core instantiates it with. Every bench compares the block against a model that
   is independent of the RTL and of the generators: the model reads the published
   data at run time (patent transcriptions and die tables under `docs/data/`) or
   encodes the manual (register selection, operand sizes, instruction semantics),
   and the deliberate departures of the core from the literal data are listed, with
   their sources, in `docs/data/deviations/`. Coverage is exhaustive wherever the
   input space is finite — all 1024 microcode addresses, all 65,536 instruction
   words for the three decoders, every cell of the ALU matrix on every byte pair,
   every microcode word for the bus fabric, the bus-side registers and the register
   file — and structured elsewhere; nothing is sampled at random except where the
   data is. During development each bench was validated by injecting faults into a
   copy of its block and checking that it fails. `sim/README.md` gives the coverage
   of each.
2. **Whole-CPU benches** — `cpu_si`, a hand-assembled self-checking program that
   checks the reset sequence, the memory image and register dump it produces, the
   function codes, the byte strobes and the order of every bus cycle, run with
   either clock phase first; and the 84 benches of `make all`: every addressing mode,
   every exception path, interrupts, STOP, bus arbitration, the bus state machine on
   its own, and the CPU with its clock-enable and DTACK generator (`cpudt`). During
   development these benches were validated by mutation testing: copies of the RTL,
   each with one hand-made change, each caught by the benches that claim it. The
   mutated copies are not part of this repository; they and their results are
   available from the author on request.
3. **DTACK generator** — the clock-enable and DTACK generator on its own, driven by
   a model of the processor's bus cycle and a memory that holds DTACK: at every
   supported divider, clock enables, DTACK, wait states and the pulses returned
   after a stall checked at every clock against the rules of `docs/clock-dtack.md`,
   and no cycle "recovered" when the memory looks busy while the processor is not
   waiting.
4. **Hardware** — the design is built, loaded on a DE10-Nano and played.

Steps 1 to 3 run from this repository alone (see
[Reproducing the tests](#reproducing-the-tests)). Step 4 has been done on the
author's MiSTer arcade cores, Darius among them: in each, only the CPU (with its
DTACK generator inside) and, where present, the savestate driver were exchanged;
the rest of the design is untouched, because rm68000 presents the same pins the
part does.

During development every change was also compared cycle by cycle with an external
68000 core used as a black box (see
[The comparison core was a black box](#the-comparison-core-was-a-black-box-and-stayed-one)):
both cores ran the same programs in
lockstep — the exit from reset, address-error cases, instruction programs, generated
cases for every instruction family, every opcode word, and a real two-CPU game
program with wait states and interrupts — and every bus access was compared:
address, strobes, R/W, function code, data, the length of the cycle and the interval
to the next one. The savestate was checked the same way, by saving and restoring at
several points and comparing each restored run with the same run without savestate.
Those comparison benches need third-party code and programs that cannot be
redistributed, so they are not part of this repository; the benches and their
results are available from the author on request.

## Reproducing the tests

You need **Verilator** (5.x) and a C++ toolchain. Everything in this section runs
with nothing but this repository — no ROMs, no external cores, no setup.

```sh
cd sim
make -f Makefile.si test
```

That runs the **block benches** and prints a summary. Each one checks a block of the
core against a model independent of the RTL and of the generators, built at run time
from the published data under `docs/data/` or from the manual, exhaustively wherever
the input space is finite: the operand-size decoder and the instruction decoder are
each checked on all 65,536 instruction words, the micro and nano stores and their
decoder on all 1024 addresses, the ALU on every pair of bytes for every cell of its
matrix (`sim/README.md` gives the numbers for each). Expected output:

```
  PASS  ustore
  PASS  nstore
  ...
  PASS  cpu_si

13 PASS, 0 FAIL
```

The last one, `cpu_si`, is the whole CPU rather than a block. A single bench can also
be run on its own: `make -f Makefile.si a123_si`.

```sh
make -f Makefile.si all
```

runs the **instruction benches** instead: 84 targets, mostly one per instruction
family — addressing
modes, exceptions, interrupts, tracing, MOVEM, MULU/DIVU, BCD, bus error and double
fault, bus arbitration — plus the bus state machine on its own (`busfsm`) and the
CPU with its clock-enable and DTACK generator (`cpudt`). Each executes a small program on the core and checks
registers, condition codes, memory byte by byte, bus cycles and exception frames
against values worked out from the Motorola manuals, and ends with `RESULT: PASS` or
`RESULT: FAIL`. `make -f Makefile.si all` stops at the first failing bench;
`make -k -f Makefile.si all` runs them all. How the benches were validated during
development (fault injection and mutation testing) is described in `sim/README.md`.

```sh
make -f Makefile.cendtack cendtack
```

runs the bench of the **clock-enable and DTACK generator** on its own
(`docs/clock-dtack.md`, section 6.1): every supported divider, idle and with memory
stalls, checked at every clock against the rules of that document. The last line is
`VERDICT: PASS` or `VERDICT: FAIL`.

All three are expected to be green on a fresh clone; they are re-run here before
anything is published. They are steps 1 to 3 of [Verification](#verification).

## What is not here

- **No system.** This is a CPU, not a machine: no memory controller, no video,
  no sound. `rtl/integration/` and `rtl/savestate/` hold the DTACK generator the
  CPU uses and the optional savestate, not a platform.
- **No MC68010 or later.** No loop mode, no vector base register, no
  instruction-continuation bus-error frame.

## Documentation

`docs/` describes the core as built, one file per subject, each with its sources, the
implementation (file and signal) and the benches that verify it:

| file | what it covers |
|---|---|
| `silicon-datapath.md` | the datapath as built, section by section |
| `microcode-structure.md`, `microcode-sources.md` | the microcode and nanocode of the die, their format and sources |
| `chip-data-sources.md` | the data taken from the chip and the patent, and where each comes from |
| `register-model.md` | the programming model |
| `bus-interface.md`, `function-codes.md`, `m6800-bus.md` | bus cycles, function codes, the 6800 peripheral cycle |
| `bus-control-signals.md` | HALT, BERR, retry, double bus fault, RESET, bus arbitration |
| `exceptions.md`, `interrupts.md`, `interrupt-acknowledge.md` | the exception and interrupt model |
| `mul-div.md`, `instruction-timing.md` | multiply and divide as on the die; clocks per instruction |
| `clock-dtack.md` | the clock-enable and DTACK generator |
| `external-savestate.md` | the external savestate mechanism |

The code and its comments are the normative description; the documents explain the
reasoning.

## License

rm68000 is Copyright (c) 2026 Umberto Parisi (rmonic79). It is released under the
**GNU General Public License, version 3 or (at your option) any later version**: the
full text is in [LICENSE](LICENSE), and every source file carries the same notice.

For use without the obligations of the GPL — for example in a closed-source product —
a **commercial license** is available from the author, who can also be contacted for
other licensing needs: rmonic79 on GitHub.
