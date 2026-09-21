# Register model

## 1. Purpose

The registers a program sees on the MC68000, as the manual defines them, and where each one lives in rm68000. The
internal organisation of the register file (sections, ports, pointers) is in
[silicon-datapath.md](silicon-datapath.md) §8.

## 2. Sources

MC68000 User's Manual, Rev. 8 (MC68000UM, "UM"): §2.1.1 User Programmer's Model (Figure 2-1), §2.1.2 Supervisor
Programmer's Model (Figure 2-2), §2.1.3 Status Register (Figure 2-4), §2.3 Data Organization in Registers, §2.4
Data Organization in Memory, §6.3.1 Reset.

## 3. Registers

- **D0-D7**: eight 32-bit data registers, used for byte (bits 7:0), word (15:0) and long (31:0) operands. When a
  data register is a source or a destination, only the low-order part of the operand size is used or changed; the
  rest is neither used nor changed (UM §2.3.1).
- **A0-A6**: seven 32-bit address registers. They take no byte operands; as a source they give the low word or the
  whole register according to the size; as a destination the whole register is written, word operands being
  sign-extended to 32 bits first (UM §2.3.2).
- **A7**: the stack pointer. It is the user stack pointer **USP** in user mode (S = 0) and the supervisor stack
  pointer **SSP**, written A7', in supervisor mode (S = 1) (UM §2.1.1, §2.1.2). The two are separate registers;
  changing S changes which one A7 names, without copying anything.
- **PC**: 32-bit program counter; the MC68000 drives 24 address bits, A23-A1 plus the data strobes.
- **SR**: 16-bit status register, below.

## 4. Status register

```
 15 14 13 12 11 10  9  8 | 7  6  5  4  3  2  1  0
  T  -  S  -  - I2 I1 I0 | -  -  -  X  N  Z  V  C
 |------ system byte -----|----- user byte (CCR) ---|
```

- **T** (bit 15): trace mode.
- **S** (bit 13): supervisor state (1 = supervisor, 0 = user).
- **I2-I0** (bits 10-8): interrupt priority mask (eight levels).
- **CCR** (low byte, bits 4-0): X extend, N negative, Z zero, V overflow, C carry.
- Bits 5, 6, 7, 11, 12 and 14 are undefined and reserved (UM §2.1.3).

At reset the processor enters the supervisor state with trace off and the interrupt mask at level 7 (UM §6.3.1).

## 5. Memory byte order

The high-order byte of a word has the address of the word; the low-order byte has the next, odd address. Words and
longs sit on even addresses, and the second word of a long at address n is at n + 2 (UM §2.4). On the bus, the byte
at an even address travels on D15-D8 with UDS and the byte at an odd address on D7-D0 with LDS (UM §5.1.1, §5.1.2;
[bus-interface.md](bus-interface.md)).

## 6. Implementation

Paths of RTL files are relative to `rtl/cpu_si/`.

| register | where | notes |
|---|---|---|
| D0-D7 | `m68k_regfile_si.sv`, indices 0-7 | 32 bits, stored as columns of 16, 8 and 8 bits (`ra_h`, `ra_m`, `ra_l` and `rb_h`, `rb_m`, `rb_l` in `m68k_regfile_si.sv`) |
| A0-A6 | `m68k_regfile_si.sv`, indices 8-14 | |
| USP | `m68k_regfile_si.sv`, index 15 | |
| SSP | `m68k_regfile_si.sv`, index 31 | |
| PC | `pc_sh` in `m68k_regfile_si.sv` | a register of its own, with its own write block |
| SR | `m68k_psw_si.sv` | T, S, I2-I0 and X N Z V C (`psw`) |
| DT | `m68k_regfile_si.sv`, index 18 | data temporary of the microcode, not visible to programs |

- **A7.** Every access to A7 resolves to index `{S, 1111}`: 31 (SSP) with S = 1, 15 (USP) with S = 0
  (`sp_r` and `sp_w` in `m68k_regfile_si.sv`). S is the effective value of the current microword, which already includes an S set
  in that same word on exception entry (`set_s_o` in `m68k_psw_si.sv`; `sup_e` and `sup_e_q` in `m68k_cpu_si.sv`).
  MOVE USP reaches
  the USP explicitly through the `us` pointer, set in its two microwords `susp1` and `lusp1`
  (`docs/data/mc68000_nano_encoding.tsv`; `spv_w`, `spv_r`, `rxa_i` and `rx_i` in `m68k_regfile_si.sv`). No value is
  copied when S changes.
- **Byte writes.** For a byte-sized instruction a write of the low word of a data register changes only bits 7:0
  (`ab_m` and `db_m` in `m68k_regfile_si.sv`); the size comes from the instruction (`is_byte` in `m68k_size_si.sv`;
  `u_size`, `sz` and `is_byte` in `m68k_cpu_si.sv`). The register file writes 16-bit halves and never sign-extends; word operands for an
  address register reach it already extended by the microcode's sign-extending bus transfers
  ([silicon-datapath.md](silicon-datapath.md) §5).
- **SR.** The undefined bits are 0 when read and are not stored (`psw` in `m68k_psw_si.sv`). At reset T = 0, S = 1, I = 7
  and the flags are 0 (reset branch of `m68k_psw_si.sv`). Writes from the FTU to the byte forms (to CCR, RTR) change only X N Z V
  C (`ld_ftu` and `ccr_only` in `m68k_psw_si.sv`; `ccr_only` and `f_ccr` in `m68k_cpu_si.sv`). The rules for every SR update are in
  [silicon-datapath.md](silicon-datapath.md) §10.
- **Reset contents.** The register array has no reset value: the manual makes no assumption about register
  contents at reset, and the reset exception loads only SSP and PC from the vector table (UM §6.3.1). The PC register
  and the SR are reset (reset of `pc_sh` in `m68k_regfile_si.sv`; reset branch of `m68k_psw_si.sv`).

## 7. Verification

- Block bench `regfile_si` (`sim/tb_m68k_regfile_si.cpp`, `make -f Makefile.si regfile_si`): register selection,
  USP/SSP by S, writes with the half-word and byte rules, read ports, the whole register state after every
  microcycle.
- Instruction benches: `usp` (MOVE USP, A7 = USP in user mode, privilege), `movean` (MOVEA from every source mode),
  `extswap` (EXT, SWAP, EXG), `movesr`, `mvtsr`, `srlogic` (SR and CCR), `exc` (S and T on exception entry, RTE),
  `rstvec` (reset vectors and initial SR), `cpu_si` (byte, word and long accesses through the whole CPU).
