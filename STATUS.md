# Status — version 1.0

## What is in this repository

- **The CPU**: `m68k_cpu_dt_si` (`rtl/cpu_si/`), the processor `m68k_cpu_si` with its clock-enable and DTACK
  generator `m68k_cen_dtack` (`rtl/integration/`) in one module.
- **The external savestate**, optional: `m68k_ss_seq` (`rtl/savestate/`), documented in `docs/external-savestate.md`.

## On hardware

In use on real hardware (DE10-Nano) in the author's MiSTer arcade cores, Darius among them, in place of the CPU
they shipped with; save and restore work, sound included.

## Verification of this tree

All of it re-run on the contents of this commit, with nothing outside this repository (from `sim/`):

| bench | result |
|---|---|
| block benches, `make -f Makefile.si test` | 13 PASS, 0 FAIL |
| instruction benches, `make -f Makefile.si all` | 84 targets, 0 failed |
| DTACK generator, `make -f Makefile.cendtack cendtack` (`sim/run_cendtack.sh`) | VERDICT PASS: 18 standard runs (every supported divider, idle and with stalls) pass every check against `docs/clock-dtack.md`; busy-while-not-waiting conditions as expected |

During development every change was also compared cycle by cycle with an external 68000 core used as a black box
(the same programs run in lockstep on both cores, from instruction programs and generated cases to every opcode word
and a real two-CPU game program, and the savestate checked by comparing each restored run with the same run without
savestate), and the benches were validated by fault injection and mutation testing (`sim/README.md`). Those
comparison benches and the mutated copies are not part of this repository; they and their results are available
from the author on request.

## Interface notes

- `reset` of the CPU is active high; the pin of the part is active low.
- A CPU halted by a double bus fault or stopped by STOP at the time of a save: `docs/external-savestate.md`
  section 5.

## Before changing anything

- **A whole-design fit moves on its own** for a barely different netlist: a small change cannot be measured with
  a single build.
- **No state flop without a defined value**, at reset and at power-up. With `ALLOW_POWER_UP_DONT_CARE` enabled, a
  functionally neutral change alters power-up values and can break the system from one build to the next.
- **Speed is not negotiable**: anything that changes the clock-enable pacing is out, even if it saves area.

## Third-party material

`.gitignore` keeps out of the repository the third-party material used during development for the comparison
benches: program ROMs, the external core used for comparison and its data files. None of it is part of the release,
and nothing in this repository needs it.
