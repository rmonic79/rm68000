# Multiply and divide: MULU, MULS, DIVU, DIVS

## 1. Purpose

MULU, MULS, DIVU and DIVS are the longest microroutines of the 68000, and their
execution time depends on the operands. rm68000 runs them as the silicon does: the
microwords of the die micro ROM, with the nanoword contents of US 4,325,121
Appendix H, on the 16-bit ALU with its extension register ALUE. There is no
separate multiplier or divider. This document describes those microroutines, the
ALU behaviour they rely on, and how the core implements both.

## 2. Sources

- US 4,325,121 Appendix H: the blocks `mulr1`, `mulm1`–`mulm6`, `dvur1`,
  `dvum0`–`dvumz`, `dvs01`–`dvs1e`, `leaa2` (`docs/data/us4325121_appH.tsv`). Die addresses
  are those of `docs/data/us4325121_appH_die_map.tsv`. Microword fields (type,
  condition, next address) are from the die micro ROM
  (`docs/data/mc68000_micro_decoded.csv`).
- Microwords present on the die and absent from Appendix H: `dvs0e`, `dvs12`,
  `dvs1f`, `dvs20` (`docs/data/mc68000_die_only_words.tsv`).
- US 4,325,121 FIG. 17, the ALU function and condition-code table
  (`docs/data/mc68000_alu_matrix_fig17.tsv`): row 7 (MULS, MULU) and row 1
  (DIVS, DIVU). FIG. 18 selects the row from `IRD`
  (`docs/data/mc68000_alu_rows_fig18.tsv`).
- US 4,325,121 Appendix C, conditions `ms0`, `m01`, `z`, `c`, `n`, `auz`, `nz2`
  (`docs/data/us4325121_appC.tsv`).
- MC68000UM Rev. 8, Table 8-4 and its notes (execution times), and Table 8-14
  (divide-by-zero exception). M68000PRM section 4, MULU/MULS/DIVU/DIVS (results and
  condition codes).

Notation of the Appendix H blocks, as in [microcode-structure.md](microcode-structure.md):
the type (`db`, `dbi`, `bc`, `a1`, ...) is the next-address type. The ALU field is
the FIG. 17 column (1–6, 6 = column 4 with a 1 shifted in) followed by `i`
(initiate), `f` (finish) or `n` (condition codes untouched); `x` means no ALU
operation.

## 3. Registers

- **ALU** (16 bits) and **ALUE** (16 bits) form a 32-bit pair. In multiply and
  divide the ALU is the **high** word: partial product high and multiplier / low
  product in MULU and MULS (Appendix H `mulm6`: `alu → rxh`, `alue → rxdl`);
  remainder and dividend low / quotient in DIVU and DIVS. This is the opposite
  orientation from the 32-bit shifts (ASL.L and the others), where ALUE is the high
  word. In the core, `alu_high` selects the orientation
  (`rtl/cpu_si/m68k_alu_si.sv`); the pair of the long shifts is `pair`.
- **ALUB**: the second operand: the multiplicand, or the divisor.
- **AT**: prefetch address in the multiply; saved remainder (ATL) in the divide;
  divisor (ATH) and results (ATH remainder, ATL quotient) in DIVS.
- **AU**: the loop counter. It is loaded from the field translation unit with
  `ftu → db → au`. For MULU/MULS and DIVU/DIVS the FTU supplies the constants
  `K_MUL` and `K_DIV`, both 15 (parameters of `rtl/cpu_si/m68k_ftu_si.sv`, selected
  by the opcode in `f_imp`). The branch condition `auz` is AU = 0 on the six low bits
  (input `auz` of `u_br` in `rtl/cpu_si/m68k_cpu_si.sv`).

A conditional branch tests values written by the **previous** microword, because
the next address is computed while the current word executes
([microcode-structure.md](microcode-structure.md) section 6). The routines below
are written for that.

## 4. MULU and MULS

FIG. 17 row 7: column 1 `AND KNZ00`, column 2 `SUB KNZV~C`, column 3
`ADD KNZVC`, column 4 `ROXR KNZ0A`, column 5 `EXT`.

| word (die address) | Appendix H type, ALU | operations (Appendix H) | next |
|---|---|---|---|
| `mulr1` (15b), register source; `mulm1` (15a), memory or immediate source | `dbi`, `1i` | starts the prefetch (`au → ab → aob,at`); ALUE ← source operand (`(rydl) → dbd → alue` in `mulr1`, `(dbin) → dbd → alue` in `mulm1`); AU ← FTU (`K_MUL`) + 0; ALUB ← Dn low word (`(rxdl) → abd → alub`); ALU ← 0 | `mulm2` |
| `mulm2` (3c4) | `bc`, `x` | finishes the prefetch (`edb → dbin,irc`) | condition `ms0` = {IRD[8], ALUE[0]}: `mulm4` if ALUE[0] = 0; `mulm3` if MULU and ALUE[0] = 1; `mulm5` if MULS and ALUE[0] = 1 |
| `mulm3` (060) | `db`, `3f` | ALU ← ALU + ALUB | `mulm4` |
| `mulm5` (0a0) | `db`, `2f` | ALU ← ALU − ALUB | `mulm4` |
| `mulm4` (0e0) | `bc`, `4f` | shift the pair right (`sr c-alu-alue`); AU ← AU − 1; PC low ← ATL | condition `m01` = {AU = 0, IRD[8], ALUE[1], ALUE[0]}: `mulm6` if AU = 0; else `mulm3` (MULU ALUE[1] = 1; MULS ALUE[1:0] = 01), `mulm5` (MULS ALUE[1:0] = 10), `mulm4` (MULU ALUE[1] = 0; MULS 00 or 11) |
| `mulm6` (020) | `a1`, `1f` | Dn high ← ALU (`alu → ab → alu,rxh`); Dn low ← ALUE (`(alue) → dbd → rxdl`); PC high ← ATH and AU ← AT + 2; `IRD ← IR`; ALU column 1 with −1, finish: final condition codes | next instruction (A1) |

The four destinations of `mulm4` are one logical row: 020, 060, 0a0, 0e0, base 020
with address bits 7:6 = 00, 01, 10, 11. US 4,325,121 uses `mulm4` as its example of
a four-way branch [P col. 23–24].

Behaviour:

- The multiplier is examined one bit per iteration. `mulm2` looks at bit 0. Each `mulm4`
  looks at ALUE[1] **before** its own shift, which is the next bit to be consumed.
  MULU adds the multiplicand when the bit is 1. MULS examines the pair
  (b<sub>k</sub>, b<sub>k−1</sub>) with b<sub>−1</sub> = 0, Booth's recoding: 01
  adds, 10 subtracts. This matches the MC68000UM note to Table 8-4, which counts
  "10 or 01 patterns" in the 17-bit source formed with a zero LSB.
- **Shift `sr c-alu-alue`** (column 4, ROXR, condition code `KNZ0A`): the pair moves
  right by one. ALU[0] enters ALUE[15], and ALUE[0], the consumed multiplier bit,
  leaves. The bit entering ALU[15] is:
  - in MULU, C of the previous operation: the carry of the addition, because the
    true high word of the partial product has 17 bits;
  - in MULS, N ⊕ V of the previous operation: the true sign of the signed sum or
    difference (V = 1 when the 16-bit result overflowed, as in 0 − (−32768)). After a
    shift V = 0 and N is the high bit of the pair, so the bit entering a shift that
    follows another shift is the sign.

  The special C code `A` of this cell is 0, so that MULU shifts in 0 after a shift.
- **Counter.** `mulr1` or `mulm1` loads AU = 15. Each `mulm4` decrements AU and tests the value
  written by the previous word, so the sixteenth `mulm4` sees AU = 0 and exits to
  `mulm6`: 16 shifts, 16 multiplier bits.
- **Condition codes** (M68000PRM 4: X unchanged, N = bit 31, Z = result zero,
  V = 0, C = 0). They come from the cells of row 7. The final word `mulm6` passes the
  high word through column 1 with the finish entry, and in a finish entry Z is
  accumulated with the previous Z (case `ACC_Z` of `m68k_alu_si.sv`).

In the core:

- The shift is the `ROXR` branch with `c_as_x` set, in `m68k_alu_si.sv`. The
  entering bit `x_sh` is C for MULU and N ⊕ V for MULS, and the ROXR input `in_r`
  takes it. The C produced is 0 (`sh_c` and `carry` of that branch). For the pair,
  N is ALU[15] (`alu_high`).
- `c_as_x` is the nano atom `sr c-alu-alue` (`sr_mul` in `m68k_alux_si.sv`), registered
  with the operands. It forces the pair size (`sz_eff`).
  `mul_signed` is `IRD[8]` (port `mul_signed` of `u_alu` in `m68k_alux_si.sv`).
- The branch values `ms0` = {IRD[8], ALUE[0]} and `m01` = {AU = 0, IRD[8],
  ALUE[1:0]} are built in the cases `ms0` and `m01` of `val` in `m68k_branch_si.sv`.

**Timing.** With a register source: `mulr1` and `mulm2` are 2 clocks each (the
prefetch started in `mulr1` completes at the end of `mulm2` without wait states).
The 16 `mulm4` take 32 clocks and `mulm6` takes 2; each `mulm3` or `mulm5` adds 2.
That gives 38 + 2n clocks, where n is the number of additions and subtractions,
which is the MC68000UM formula (Table 8-4 note: MULU n = number of ones in the
source; MULS n = number of 01 or 10 pairs). The maximum printed in Table 8-4 is
70(1/0), for n = 16. With a memory or immediate source the effective-address
calculation comes first, the routine enters at `mulm1`, and the effective-address
time of Table 8-1 is added. One microcycle is 2 clocks
([instruction-timing.md](instruction-timing.md) section 3).

## 5. DIVU

FIG. 17 row 1: column 1 `AND KNZ00`, column 2 `SUB KNZ0~C`, column 3 `SUBX`,
column 4 `SLAA_X`, column 5 `EXT`. Column 6 is column 4 with a 1 shifted in.

DIVU is a **restoring** division in 16 iterations. The microwords (Appendix H; die
addresses from the die map):

| word (die address) | type, ALU | operations (Appendix H) | next |
|---|---|---|---|
| `dvur1` (0a6), register source; `dvum1` (0a4), memory or immediate source | `db`, `1i` | `au → pc`; divisor to ATH, ALU and ALUB (`(rydl) → ab → alu,alub,ath` in `dvur1`, `(dbin) → ab → ath,alu,alub` in `dvum1`); ALUE ← Dn low (`(rxdl) → dbd → alue`); ALU column 1 with −1: the flags of the divisor | `dvum2` |
| `dvum2` (213) | `bc`, `2i` | ALU ← Dn high − ALUB | condition `z` (from `dvur1`/`dvum1`): divide by zero if Z; `dvum3` otherwise |
| `dvum3` (023) | `bc`, `1i` | AU ← FTU (`K_DIV`) + 1; ALU and ATL ← Dn high | condition `c` (from `dvum2`): `dvum5` if C (high word < divisor); `dvum4` if not C (overflow) |
| `dvum5` (291) | `bc`, `4n` | shift the pair left, 0 into ALUE[0]; AU ← AU − 1 | condition `n`: `dvum7` if N; `dvum8` otherwise |
| `dvum6` (2d0) | `bc`, `6n` | as `dvum5`, with 1 into ALUE[0] | as `dvum5` |
| `dvum7` (2d2) | `bc`, `2i` | ATL ← ALU; ALU ← ALU − ALUB | `dvum6` if AU ≠ 0; `dvum9` if AU = 0 |
| `dvum8` (252) | `bc`, `2i` | as `dvum7` | `dvumb` if AU ≠ 0; `dvumc` if AU = 0 |
| `dvumb` (3e9) | `bc`, `x` | idle | `dvume` if C (2R < divisor); `dvum6` if not C |
| `dvume` (290) | `db`, `1i` | restore: ALU ← ATL | `dvum5` |
| `dvum9` (250) | `db`, `1n` | Dn high ← ALU | `dvumd` |
| `dvumc` (369) | `bc`, `1n` | Dn high ← ALU | `dvumd` if not C; `dvumf` if C |
| `dvumd` (0c0) | `dbi`, `6n` | last quotient bit 1 into ALUE; starts the prefetch | `dvum0` |
| `dvumf` (080) | `dbi`, `4n` | last quotient bit 0 into ALUE; Dn high ← ATL (restored remainder); starts the prefetch | `dvum0` |
| `dvum0` (212) | `a1`, `2i` | Dn low ← ALUE (quotient); finishes the prefetch; `IRD ← IR`; final condition codes from the column-2 operation | next instruction (A1) |
| `dvum4` (2d1) | `dbi`, `x` | overflow: starts the prefetch | `dvuma` |
| `dvuma` (0e6) | `a1`, `5i` | finishes the prefetch; column 5: V = 1, C = 0; Dn unchanged | next instruction (A1) |

Behaviour:

- The shift (columns 4 and 6 of row 1) moves the pair left by one. ALUE[15] enters
  ALU[0], and the quotient bit (0 in column 4, 1 in column 6) enters ALUE[0]. The bit
  that leaves ALU[15] is the 17th bit of 2R.
- The condition `n` after a shift (columns 4 and 6 use `n`: no condition-code
  update) is the N of the last update: bit 15 of the remainder **before** the
  shift, the bit that left. If it was 1, then 2R ≥ 2<sup>16</sup> > divisor and the
  subtraction is always valid (`dvum7`, next bit 1, no test). Otherwise `dvum8`
  subtracts, and `dvumb` tests the borrow. With a borrow `dvume` restores the
  remainder from ATL and the next bit is 0; without one the next bit is 1.
- **Counter.** `dvum3` loads AU = `K_DIV` + 1 = 16. Each shift decrements it, and
  the subtraction words test the value written by the shift before them, so the loop
  makes 16 iterations, one subtraction each.
- **Overflow** (dividend high word ≥ divisor): `dvum4` and `dvuma`. FIG. 17 row 1
  column 5 has `EXT` and no condition code. The core uses `KKK10` (V = 1, C = 0, X N
  Z unchanged), following M68000PRM 4 ("V set if division overflow, cleared
  otherwise; C always cleared"): `docs/data/deviations/alu.tsv` and the cell
  `cells[(1, 5)]` of `docs/data/scripts/gen_alu.py`.
- **Divide by zero.** `dvum2` branches on the Z of `dvur1`/`dvum1`, the flags of the
  divisor. With Z the die's C1C0 for `z` selects 0e3 (`dvur2`), which continues to `dvur3`
  and `trap3` (`mc68000_micro_decoded.csv`, Appendix C). The vector is 5
  (`f_tvn` in `m68k_exc_si.sv`). MC68000UM Table 8-14 gives 38(4/3) plus the
  effective-address time.

In the core the left shift of the pair is the `SLAA` branch for row 1 in
`m68k_alu_si.sv`. ALU ← {ALU[14:0], ALUE[15]} and ALUE ← {ALUE[14:0], q},
and the bit that leaves is the carry. The quotient bit `q` is 1 for column 6
(`in_l`, `m68k_alu_si.sv`).

## 6. DIVS

DIVS uses the same ALU cells (row 1) and the same restoring loop. The DIVS words
`dvs09`, `dvs0a`, `dvs0c`, `dvs0d`, `dvs0f` and `dvs08` share their nanowords with
`dvum5`, `dvum6`, `dvum7`, `dvumb` and `dvume` (ORIGIN column of Appendix H).
Around the loop:

1. Signs of the operands, then absolute values. `dvs01` (0ae) or `dvs02` (0ac)
   takes the divisor; `dvs03` (0c9) tests it (Z: divide by zero; N: negative);
   `dvs05` (063), `dvs04` (0a3) and `dvs06` (176) to `dvs11` (177) negate where
   needed.
2. The loop, as in DIVU: `dvs09` (295), `dvs0a` (2d4), `dvs0c` (2d6), `dvs0d`
   (3ed), `dvs0f` (294), with `dvs0e` (32d), `dvs12` (214) and `dvs13` (254) at
   the end.
3. Sign correction of quotient and remainder: `dvs14` (210) to `dvs1e` (0ca),
   `dvs1f` (08a), `dvs20` (049).
4. Result `rx ← AT` in `leaa2` (066), with ATH = remainder and ATL = quotient; or
   overflow through `dvum4` (2d1) and `dvuma` (0e6).

Facts the core relies on, each with its source:

- `dvs01` and `dvs02` also save the divisor in **ATH**. Appendix H marks this
  transfer as added by hand in the patent, on their origins `dvur1` and `dvum1`, and
  the die listing shows it. Its sign is tested later, by `dvs15` (211) on the flags
  of `dvs14` (210), which reads ATH. Source: `docs/data/deviations/nstore.tsv`,
  addresses 0ac and 0ae.
- The conditional branches test the flags of the previous word: `dvs03` tests Z and N
  of the divisor produced by `dvs01`/`dvs02`; `dvs15` tests the N produced by
  `dvs14`.
- **Negation of the 32-bit dividend**: `dvs06` (`2n`, 0 − Dn low) then `dvs10`
  (`3n`, SUBX 0 − Dn high − borrow). The carry latch is written by **every** adder
  operation, in `n` mode too, and the extended carry-in of ADDX/SUBX comes from the
  latch in every mode other than initiate. RTL: `xc` in `m68k_alu_si.sv`
  (`xc = init ? x_in : cl_in`) and the latch `cl` of `m68k_alux_si.sv` (written on
  `init | adder_o`).
- `dvs16` and `dvs1d` (041, 0c1) have `(atl) → * → ath` with the bus left open in
  Appendix H. The die listing shows `ath = dbh; dbh <-> dbl`, so the core moves ATL
  onto DB and loads ATH from DB: ATH = remainder (`ATOM_SWAP` of 041 and 0c1 in `gen_nrom.py`).
- `dvs12` (214), present only on the die, is the last shift of the pair with 0
  entering (borrow in the last subtraction). ATL keeps the remainder saved by
  `dvs0c`. `dvs13` (254, `6n`) shifts 1 in and sets ATL ← ALU, the remainder of the
  successful subtraction. ALUE holds the quotient.
- `dvs1f` (08a), present only on the die, negates the quotient (dividend positive,
  divisor negative), as `dvs1a` (0cd) does. `dvs20` (049), also only on the die,
  copies the ALU into ATL and starts the prefetch. It branches on `nz2`, the flags of
  the negation: N or Z goes to `leaa2`; neither (the negated quotient is positive,
  so the quotient was out of range) goes to overflow.
- Appendix H has no block for these four words. The die listing gives only the
  jump for `dvs0e` and `dvs1f`, and the transfers for `dvs12` and `dvs20`; it does
  not give ALU operations (`docs/data/mc68000_die_only_words.tsv`). Their contents
  in the core are lines of `docs/data/deviations/nstore.tsv` (addresses 32d, 049,
  08a, 214). Those for 08a and 214 are marked as confirmed by measurement only: the
  comparison with an external reference core made during development, families
  DIVS and DIVU.
- Overflow (quotient outside −32768..32767): V = 1, C = 0, Dn unchanged, through the
  same `dvuma` as DIVU.

## 7. Timing summary

| instruction | MC68000UM Table 8-4 (register source) | in the core |
|---|---|---|
| MULU | 38 + 2n, n = ones in the source; maximum 70(1/0) | 38 + 2n from the microroutine (section 4) |
| MULS | 38 + 2n, n = 01/10 pairs in the source with a zero LSB; maximum 70(1/0) | same |
| DIVU | maximum 140(1/0); less than 10% between best and worst case | the die microroutine (section 5); the length depends on the path taken at each iteration |
| DIVS | maximum 158(1/0); same note | the die microroutine (section 6) |

With a memory or immediate source, the effective-address time of Table 8-1 is added
to every row (Table 8-4, "+").

## 8. Verification

Instruction benches (`sim/Makefile.si`, `make -f Makefile.si <target>`), with
expected values from M68000PRM 4. Each is run with zero wait states and with DTACK
delayed by 3 clocks:

| target | bench | what it checks |
|---|---|---|
| `muldiv` | `sim/tb_m68k_muldiv.cpp` | MULU, MULS, DIVU, DIVS with a register source, 41 cases: limit operands, overflow at the boundary, DIVS remainder sign, same register as source and destination, all five flags |
| `muls` | `sim/tb_m68k_muls.cpp` | MULS and DIVS, register and immediate source, signed limit cases and overflow |
| `muldivmem` | `sim/tb_m68k_muldivmem.cpp` | the four instructions with the source in memory or immediate, 13 addressing modes, one operand read with the right function code |
| `div0` | `sim/tb_m68k_div0.cpp` | divide by zero: vector 5, frame contents, three frame writes, destination unchanged |

These benches do not check clock counts. The ALU cells are checked by the block
benches `alu_si` (every cell of FIG. 17) and `alux_si` (the ALU with ALUE, ALUB and
the carry latch, driven by the real control store). The branch conditions are
checked by `branch_si`, and the nanoword contents by `nstore` and `ndec`
([microcode-structure.md](microcode-structure.md) section 12).

Clock counts and results were compared during development with an external
reference core, used as a black box: generated cases for the families MULU, MULS,
DIVU and DIVS, and a lockstep comparison of every bus access with its length and the
interval to the next, also in the sweep over all 65,536 opcodes. Those comparison
benches are not part of this repository; they and their results are available from
the author on request.
