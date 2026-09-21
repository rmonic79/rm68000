# MC68000 function codes

## 1. Purpose

The function code outputs FC2-FC0 classify every bus cycle: user or supervisor, program or data, or CPU space.
This document gives the classification of the manual, the rules that produce it in rm68000 and the benches that
check it. The interrupt acknowledge cycle, the only CPU space cycle of the MC68000, is described in
`interrupt-acknowledge.md`.

## 2. Sources

- MC68000UM Rev. 8: 3.8 Processor Function Codes and Table 3-3 (pp. 3-8, 3-9); 6.1 Privilege Modes, 6.1.3
  Privilege Mode Changes and 6.1.4 Reference Classification with Table 6-1 (pp. 6-1 to 6-3); 6.2.1 Exception
  Vectors (p. 6-4) and Table 6-2 (p. 6-7) for the space of the vector reads; 5.1.4 CPU Space Cycle (p. 5-9).
- M68000PRM 2.2.11 and 2.2.12: a PC-relative operand reference "is classified as a program reference".
- Function code field of each microword: die micro ROM dump `docs/data/mc68000_micro_decoded.csv` (column `fc`)
  and US 4,325,121 App. A (`docs/data/us4325121_appA.csv`, column `fc`).

RTL: `rtl/cpu_si/m68k_busctl_si.sv` (the function code register), `m68k_useq.sv` and `m68k_ustore.sv` (the
microword field), `m68k_cpu_si.sv`. References give the signal names.

## 3. Behaviour

UM Table 3-3 and Table 6-1:

| FC2 | FC1 | FC0 | Address space |
|---|---|---|---|
| 0 | 0 | 0 | undefined, reserved |
| 0 | 0 | 1 | user data |
| 0 | 1 | 0 | user program |
| 0 | 1 | 1 | undefined, reserved for user definition |
| 1 | 0 | 0 | undefined, reserved |
| 1 | 0 | 1 | supervisor data |
| 1 | 1 | 0 | supervisor program |
| 1 | 1 | 1 | CPU space |

- FC is valid whenever AS is active (UM 3.8).
- FC2 is the S bit: bus cycles of an instruction executed in supervisor mode are supervisor references, in user
  mode user references (UM 6.1.1, 6.1.2).
- Instruction fetches are program references; operand accesses are data references, except PC-relative operands,
  which are program references (PRM 2.2.11, 2.2.12).
- Exception vectors are read in the supervisor data space; the reset vector, in the supervisor program space
  (UM 6.2.1, Table 6-2 note 2).
- CPU space: FC = 111, with A19-A16 giving the type; on the MC68000 only the interrupt acknowledge (UM 5.1.4).
- MOVE to SR, ANDI to SR and EORI to SR fetch their operands in supervisor mode and fetch the next instruction in
  the mode of the new S bit (UM 6.1.3).

## 4. Implementation

### 4.1 The microword field

Every microword has a two-bit function code field, bits 16-15 of the micro ROM word (`fc`, `m68k_useq.sv`).
The busctl reads it in phase 0 of the microword, so it takes it from the memory output directly rather than from
the registered copy of the word (`fc`, `m68k_ustore.sv`). Its values, as in the die listing:

| Field | Meaning |
|---|---|
| 01 | data |
| 10 | program |
| 11 | CPU space; only one microword has it, `itlx5`, the interrupt acknowledge read (die address 236) |
| 00 | not updated: the cycle keeps the code of the last access that set it |

Many access microwords have 00: completion words and the later accesses of a sequence (MOVEM, effective address
chains).

### 4.2 The function code register

On each request the busctl computes the two-bit code and registers the three output bits
(`fc_code`, `fc_r`, `m68k_busctl_si.sv`):

- `fc_code_n`: for an access of type `uk` (`acc_uk`) the code is data, or program if the instruction's source
  operand is PC-relative; otherwise it is the microword field if that is not 00, otherwise the previous code
  `fc_code`. In App. H the `uk` type marks only operand reads, in the displacement, absolute and MOVEM address
  sequences (`adrw2`, `adsw2`, `adsl2`, `adsl3`, `ldm*`).
- `pc_rel` is true when IRD bits 5-0 select d16(PC) or d8(PC,Xn), mode 7 with register 2 or 3
  (`pc_rel` port of the busctl instance, `m68k_cpu_si.sv`).
- At the request (`req`): `fc_code <= fc_code_n`, and `fc_r <= 111` for the CPU space code, otherwise
  `{S, fc_code_n}` with S from the PSW. `fc` is `fc_r`.

Consequences:

- FC changes only with a request: it is valid from S0, and it holds its value between cycles until the next
  request.
- After reset `fc_r` is 000 and `fc_code` 10 (reset branch of the `fc_r` block), values that no cycle shows: the
  reset vector reads take FC 110 from the program code in the field of the reset microwords (die dump: `rstp7` to
  `rstpa`; the comment on the `fc` field above `fc_r` in `m68k_busctl_si.sv`). See `bus-interface.md`.
- The exception routines write the stack and read the vectors with supervisor data FC, also when the exception
  was taken in user mode (the benches below check it).
- The fetches that the processor makes after an instruction changes S use the new S from the first request after
  the change. Prefetched words fetched before the change keep the function code they were fetched with.
- The status word of a bus or address error frame stores the FC of the aborted cycle, `fc_r` (`ssw_fc`,
  `m68k_busctl_si.sv`; `exceptions.md`).

The output `iack` of `m68k_cpu_si` is high while FC = 111 and AS is asserted (`m68k_cpu_si.sv`).

## 5. Verification

| Bench (`sim/`) | Make target | What it checks |
|---|---|---|
| `tb_m68k_cpu_si.cpp` | `cpu_si` (in `test`) | FC on every cycle: 110 on fetches and reset vectors, 101 on data |
| `tb_m68k_usp.cpp` | `usp` | 110 / 010 on fetches and 101 / 001 on data in supervisor and user mode, user stack through the USP with FC 001 |
| `tb_m68k_pcrel.cpp` | `pcrel` | operand reads with d16(PC) and d8(PC,Xn) go out with FC 110 |
| `tb_m68k_rstvec.cpp` | `rstvec` | the four reset vector reads with FC 110 |
| `tb_m68k_exc.cpp`, `tb_m68k_priv.cpp`, `tb_m68k_illegal.cpp` | `exc`, `priv`, `illegal` | FC of every cycle against the memory area, vector reads in supervisor data space, user data writes and user program fetches after RTE to user mode |
| `tb_m68k_trapv.cpp`, `tb_m68k_div0.cpp` | `trapv`, `div0` | frame writes with FC 101 when the exception is taken in user mode |
| `tb_m68k_irq.cpp`, `tb_m68k_irq_multi.cpp` | `irq`, `irqmulti` | FC 111 in the acknowledge cycle, FC 101 on the frame |
| `tb_m68k_berr.cpp` | `berr` | FC of the aborted cycle in the status word; vector 2 read in supervisor data space |
| `tb_m68k_tst.cpp` | `tst` | TAS read and write with supervisor data FC |

The words already in the prefetch queue when MOVE to SR changes the mode were fetched by cycles run in the previous
mode: their function code is the one of those cycles, which the manual does not tie to the new mode (`tb_m68k_usp.cpp`
accepts either program space for those words).
