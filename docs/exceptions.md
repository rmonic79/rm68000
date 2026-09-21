# MC68000 exception processing

## 1. Purpose

This document describes how rm68000 processes exceptions: groups and priorities, vectors, the common sequence
and the two stack frames, and each exception in turn: reset, bus error, address error, double bus fault, trace,
interrupt, illegal instruction, line A and line F, privilege violation, the instruction traps (TRAP, TRAPV, CHK,
divide by zero), the return with RTE and the wake-up from STOP.

Interrupts are covered in two other documents: `interrupts.md` (IPL, mask, level 7, sampling) and
`interrupt-acknowledge.md` (the acknowledge cycle and the vector number). The bus side of BERR and HALT,
including the retry of a cycle with BERR and HALT together, is in `bus-control-signals.md`.

## 2. Sources

- MC68000UM Rev. 8: section 6, Exception Processing: 6.1 Privilege Modes (pp. 6-1 to 6-3), 6.2.1 Exception
  Vectors (p. 6-4), Table 6-2 Exception Vector Assignment (p. 6-7), 6.2.3 Multiple Exceptions (p. 6-8), Table 6-3
  Exception Grouping and Priority (p. 6-9), 6.2.4 Exception Stack Frames and Figure 6-5 (pp. 6-9, 6-10), 6.2.5
  Exception Processing Sequence (p. 6-11), 6.3.1 to 6.3.10 (pp. 6-11 to 6-19), Figure 6-7 Supervisor Stack Order
  for Bus or Address Error Exception (p. 6-17); 5.4.4 Double Bus Fault (p. 5-28).
- M68000PRM: section 1.3 (status register), section 4 (TRAP, TRAPV, CHK, DIVU, DIVS, ILLEGAL, RTE, STOP, RESET,
  MOVE to SR, MOVE USP, ANDI/ORI/EORI to SR).
- US 4,325,121, App. A (list of microwords) and App. H (microword blocks), transcribed in
  `docs/data/us4325121_appA.csv` and `docs/data/us4325121_appH.tsv`: the exception routines `bser*`,
  `trap*`, `itlx*`, `dvur*` named below.
- Where the manual leaves a timing open, the core was matched during development against an external reference
  core, used as a black box, with comparison benches that are not part of this repository (available from the
  author on request); those places are marked "measured on the reference core".

RTL files cited (under `rtl/cpu_si/`): `m68k_exc_si.sv` (exception block), `m68k_useq.sv` (micro address
selection), `m68k_psw_si.sv` (status register), `m68k_ftu_si.sv` (field translation unit: SR copy, vector
address), `m68k_busctl_si.sv` (bus error, address error, status word, double fault), `m68k_bus_fsm.sv`,
`m68k_cpu_si.sv`, `m68k_regfile_si.sv`. References give the signal, block and module names.

## 3. Status register and stack pointers

The status register is `{T, 0, S, 0, 0, I2, I1, I0, 0, 0, 0, X, N, Z, V, C}`: the unimplemented bits read as
zero (`psw`, `m68k_psw_si.sv`; PRM 1.3). S selects the stack pointer: A7 is the SSP with S = 1 and the USP
with S = 0 (`sp_r`, `m68k_regfile_si.sv`). The register file sees the effective S of the microword, which is
already 1 in the microword that sets it, so the first stack access of an exception routine uses the SSP even
when the exception is taken in user mode (`sup_e`, `sup_e_q`, `m68k_cpu_si.sv`). See also
`register-model.md`.

## 4. Groups and priority

UM 6.2.3 (p. 6-8) and Table 6-3 (p. 6-9):

| Group | Exceptions | Processing |
|---|---|---|
| 0 | reset, address error, bus error | the current instruction is aborted |
| 1 | trace, interrupt, illegal, privilege | before the next instruction |
| 2 | TRAP, TRAPV, CHK, zero divide | started by normal instruction execution |

Within group 0 the manual gives reset > address error > bus error; within group 1 trace > interrupt > illegal
and privilege violation.

The core makes these decisions in the exception block at two points of the microcode
(header of `m68k_exc_si.sv`):

- Immediately (A0S, `a0s_sub`): bus error and address error are latched as soon as they occur
  (`berr_l`, `aerr_l`, `m68k_exc_si.sv`) and replace the next micro address at the next microcycle
  boundary with the routine entry `A_BSER` (`a0s_sub`, `a0s`, `m68k_exc_si.sv`; `uaddr`,
  `m68k_useq.sv`). Reset is the core reset: it starts the sequencer at address 000 (reset term of `base`,
  `m68k_useq.sv`), and the `rst_req` input of the block is tied to 0 (instance `u_exc`, `m68k_cpu_si.sv`). On
  entry the vector number is 2 for bus error and 3 for address error, and any pending trace or interrupt is
  dropped (`a0s_sub` entry, `m68k_exc_si.sv`): an aborted instruction is not traced (UM 6.3.8).
- At the dispatch of the next instruction (A0, `a0_sub`), when the microword selects the A1 entry
  (`dispatch`, `m68k_cpu_si.sv`): trace, then interrupt, then the instruction exceptions (illegal, line A,
  line F, privilege), then the STOP wait (selection of `a0_sub` and `a0`, `m68k_exc_si.sv`; `base`,
  `m68k_useq.sv`).

The instruction traps of group 2 are reached by the microcode of the instruction itself, with the vector number
taken from the instruction (section 13).

Within group 0 the entry follows the manual's order: with a bus error and an address error both latched, the
address error is taken first (`a0s_sub` entry, `m68k_exc_si.sv`).

## 5. Vectors

A vector number times four is the address of the vector; every vector is in the supervisor data space except the
reset vector, which is in the supervisor program space (UM 6.2.1 p. 6-4, Table 6-2 p. 6-7). In the core the
exception block produces the number `tvn` (`m68k_exc_si.sv`) and the FTU turns it into the address
`{tvn, 2'b00}` (`ld_trap` term of `ftu_rest`, `m68k_ftu_si.sv`).

| Number | Assignment (Table 6-2) | Where the core sets it |
|---|---|---|
| 0, 1 | reset: initial SSP, initial PC | `tvn_r <= 0` at reset (`m68k_exc_si.sv`); read with FC 110 (`bus-interface.md`) |
| 2 | bus error | `tvn_r <= 2` at the `a0s_sub` entry |
| 3 | address error | `tvn_r <= 3` at the `a0s_sub` entry |
| 4 | illegal instruction | `tvn_r <= 4` at the dispatch (`dispatch && a0_sub`) |
| 5 | zero divide | `f_tvn` |
| 6 | CHK | `f_tvn` |
| 7 | TRAPV | `f_tvn` |
| 8 | privilege violation | `tvn_r <= 8` at the dispatch (`priv & ~illegal`) |
| 9 | trace | `tvn_r <= 9` at the dispatch (`trace_l`) |
| 10 | line 1010 emulator | `tvn_r <= 10` at the dispatch (`linea`) |
| 11 | line 1111 emulator | `tvn_r <= 11` at the dispatch (`linef`) |
| 15 | uninitialized interrupt vector | supplied by the device (`interrupt-acknowledge.md`) |
| 24 | spurious interrupt | `tvn_r <= 24` on `iack_spur` |
| 25-31 | level 1-7 autovectors | `tvn_r <= {3'd0, 2'b11, inl}` at the dispatch (`int_l`) |
| 32-47 | TRAP #0-#15 | `f_tvn` |
| 64-255 | user interrupt vectors | from the acknowledge cycle (`tvn_r <= vec_num` on `iack_vec`) |

`tvn = (tvn_use_r | g0_busy) ? tvn_r : tvn_instr` (`m68k_exc_si.sv`): the latched number `tvn_r` while an
exception routine runs, and the number decided from the instruction (`tvn_instr`) otherwise. `tvn_use_r` is set at
the dispatch that enters a trace, interrupt or instruction exception, or the STOP wait (`dispatch && a0_sub`), and
by reset, and is cleared by the microword that loads the next opcode (dbi, `upd_full`). The entry into the bus or
address error routine leaves `tvn_use_r` unchanged: that routine takes its vector through `g0_busy` (section 9),
and `tvn_use_r` keeps the value it had at the fault, which is the I/N bit of the group 0 frame (section 6).

## 6. Common sequence and stack frames

UM 6.2.5 (p. 6-11): make an internal copy of SR, set S, clear T (and for reset and interrupts update the mask);
obtain the vector number; except for reset, push PC and the SR copy on the supervisor stack; fetch the new PC
from the vector and resume.

In the microcode the first word of each routine copies the SR into the FTU, `(psw) -> ftu` (App. H: `trap1`,
`itlx1`, `bser1`, `dvur2`; `ld_psw`, `m68k_ftu_si.sv`). S is set and T cleared by the atom
`+1 -> psws` (`trap2`, `dvur3`) or, for bus and address error, at the end of `bser1`, where App. H has no such
atom (`g0_q`, and the `set_s | g0_q` update of S and T, `m68k_psw_si.sv`). For an interrupt the atom
`(inl) -> psw` also loads the mask (`ld_inl`, `m68k_psw_si.sv`).

In the die micro ROM (`docs/data/mc68000_micro_decoded.csv`, next-address column) the routines join a common
tail: `trap1` -> `trap2` -> `trap3`, `dvur2` -> `dvur3` -> `trap3`, `bser1` ... `bser6` -> `trap3`, and
`itlx1` ... `itlx7` -> `trap4`; the tail is `trap3` ... `trap8`. The three words of the short frame are written
by `trap3`, `trap4` and `trap5`; for an interrupt `itlx4` takes the place of `trap3`, before the acknowledge
cycle, and the bus error routine writes its four extra words first, in `bser3` to `bser6`. `trap4` loads the
vector address into the FTU (`(trap) -> ftu`), `trap5` moves it to AT, and from `trap6` on the routine reads the
vector, prefetches the handler and dispatches its first instruction.

Short frame (groups 1 and 2, UM 6.2.4 and Figure 6-5): three words, SR at the new SSP, then PC high and PC low;
SSP drops by 6.

Group 0 frame (bus and address error, UM 6.3.9.1 and Figure 6-7): seven words, from the lowest address: the
status word, access address high and low, instruction register, SR, PC high and PC low; SSP drops by 14. In the
core:

- status word `{IRD[15:5], R/W, I/N, FC2-FC0}` (`ssw` port of the FTU instance `u_ftu`, `m68k_cpu_si.sv`), with
  R/W and FC of the aborted cycle (`ssw_rw`, `ssw_fc`, `m68k_busctl_si.sv`) and I/N from `tvn_use` of the
  exception block. I/N is 1 when the fault happens while the processor is processing a group 0 or group 1
  exception, 0 when it happens during an instruction or a group 2 exception (UM 6.3.9.1 p. 6-16, Figure 6-7):
  `tvn_use` is 1 from the dispatch that enters a group 1 exception routine, and from reset, to the dbi word of the
  routine, which loads the handler's first opcode, and 0 during instructions and during group 2 exceptions, whose
  routines are entered by the instruction's own microcode. The entry into the bus or address error routine leaves
  it unchanged, and the routine stacks the status word (`bser6`, `(ssw) -> ftu`) before its own dbi word. A bus
  error on the vector read of an ILLEGAL instruction stacks I/N = 1, R/W = 1, FC = 5 and the access address $10
  (case `group 1 fault` of `sim/tb_m68k_berr.cpp`). Bits 15-5 are not defined by the manual; the core places IRD
  there, as measured on the reference core;
- access address: the address output buffer of the aborted cycle (`bser1`, `aob -> ab -> at`);
- instruction register: IRD (`bser4`, `(ird) -> ftu`).

Stacked PC (UM 6.3.2 to 6.3.10): the next instruction for interrupt, trace and the instruction traps; the
instruction itself for illegal, line A/F and privilege violation; for bus and address error, a value 2 to 10
bytes past the first word of the instruction.

## 7. Bus error

UM 6.3.9 and 6.3.9.1 (pp. 6-16, 6-17): the cycle is aborted and exception processing starts with the group 0
frame and vector 2.

In the core, `berr_ev` pulses once at the end of a cycle terminated by BERR (`berr_ev`, `berr_pend`,
`m68k_busctl_si.sv`). It enters the exception block both latched and combinationally, because
the microword waiting for the cycle ends on that same edge (`a0s_sub`, `m68k_exc_si.sv`). The routine starts at
`A_BSER` = 003 (`m68k_exc_si.sv`). After the end of the aborted cycle no new request goes out for 10 clocks
(`g0_cnt`, `g0_hold`, `m68k_busctl_si.sv`), so the first push of the frame starts at the time
measured on the reference core.

A BERR in an interrupt acknowledge cycle is a spurious interrupt, not a bus error (`interrupt-acknowledge.md`).
A BERR together with HALT is described in `bus-control-signals.md`.

## 8. Address error

UM 6.3.10 (p. 6-19): a word or long access, operand or instruction, at an odd address aborts the cycle and
starts the same processing as a bus error, with vector 3.

In the core:

- The condition is a word access with A0 = 1 that is not the interrupt acknowledge (whose address is odd by
  construction): `aerr_c = ~byte_acc & aob[0] & ~iack_acc` (`m68k_busctl_si.sv`). It is decided on the
  `cen2` of S1 (`s1_tick`, `aerr_ev`, `m68k_busctl_si.sv`).
- The cycle still runs its address phase: AS, address, FC and R/W, but no data strobe and no write data, and it
  ends by itself on the first S5 without waiting for DTACK (`nostrobe` and `term_now`; branches S1 to S4
  of `m68k_bus_fsm.sv`). The manual only says that the cycle is aborted; this shape was measured on
  the reference core.
- `aerr_ev` sets `aerr_l` (`m68k_exc_si.sv`) and the routine starts at `A_BSER`. The status word takes the
  direction of the request and the FC of the cycle (`ssw_rw`, `ssw_fc`, `m68k_busctl_si.sv`). No request goes
  out for 8 clocks after the end of the aborted cycle (`g0_cnt`, `m68k_busctl_si.sv`).
- When an `i` word starts the faulting cycle, no `f` word will consume its end; that end is discarded so that it
  does not release a later word (`discard`, `m68k_busctl_si.sv`).

## 9. Double bus fault

UM 5.4.4 (p. 5-28): if another bus error occurs during exception processing, before the next instruction
starts, the processor halts and asserts HALT, and only an external reset restarts it; a bus error during a retry
does not count; a bus error while reading the reset vectors is also a double bus fault. UM 6.3.9.1 (pp. 6-16,
6-17) and 6.3.10 (p. 6-19): a bus error or an address error during the exception processing for a bus error, an
address error or a reset halts the processor.

In the core:

- `g0_busy` (`m68k_exc_si.sv`) marks the window: it is 1 out of reset, it is set by every immediate entry
  (`a0s_sub`), and it is cleared at the dispatch of the handler's first instruction. The entry counts `berr_ev` as
  well as the latches, because `berr_ev` is consumed combinationally on the entry edge.
- The busctl records whether each cycle started inside the routine (`cyc_g0 <= g0_busy` at every request,
  `m68k_busctl_si.sv`), so the cycle that caused the first fault is not counted again.
- `dfault` (`m68k_busctl_si.sv`) is set by an address error decided while `g0_busy` is high, or by the end
  of a BERR-terminated cycle that started inside the routine. It stays set until reset.
- When the `g0_cnt` counter, loaded at the end of the aborted cycle, reaches 4, `halt_int` is set
  (`m68k_busctl_si.sv`). `halt_int` freezes the microcycle so that no further request goes out (`frozen`, `req`)
  and drives `halted_n` low. Only reset clears it. The time at which `halted_n` falls was measured on the
  reference core.
- `dfault` is used inside the busctl only; `m68k_cpu_si` leaves the port unconnected (`.dfault()`, in both
  the `g_node` and the `g_flat` arrangement of `m68k_cpu_si.sv`).

Reset processing is inside the window. The reset routine starts from micro address 000 without an `a0s_sub`
entry (`rst_req` is tied to 0), and `g0_busy` is 1 from the release of the reset to the dispatch of the first
instruction. A bus error on a reset vector read, or an address error on the first fetch from an odd initial PC,
is therefore a double bus fault: the processor halts, with no exception processing and no further bus cycle
(cases `reset bus error` and `reset address error` of `sim/tb_m68k_berr.cpp`).

## 10. Trace

UM 6.3.8 (pp. 6-15, 6-16): with T set at the start of an instruction, a trace exception (vector 9) follows its
completion; there is none if the instruction was not executed because an interrupt, an illegal or a privileged
instruction was taken instead, or if it was aborted by a group 0 exception. A trace pending together with an
interrupt is processed first; an exception raised by the instruction itself is processed before the trace.

In the core T is sampled at every dispatch, that is at the start of the instruction, into `trace_l`, and cleared
instead when an exception is taken at that dispatch (`trace_l` update on `dispatch`, `m68k_exc_si.sv`). At the
next dispatch a pending `trace_l` wins over everything else (selection of `a0`) and enters `A_TRACE` = 0e3 with
vector 9 (`A_TRACE`, `tvn_r <= 9`, `m68k_exc_si.sv`). `A_TRACE` is the same entry as the instruction exceptions,
the die's `dvur2`, whose word shares the nanoword of the die's own trace entry `trac1` (App. H origin column); the
vector number tells the two apart (`A_TRACE` and `A_TRAP`, `m68k_exc_si.sv`). A latched interrupt is not cleared
by the trace entry; if it is still requested when the trace routine samples it again, it is taken at the dispatch
that ends the trace routine, so the handlers run in the order of UM 6.2.3. For an instruction that raises its own
exception (TRAP, for instance), `trace_l` set at its start is still pending at the dispatch that would start the
handler, so the trace is taken there, after the instruction's exception processing.

## 11. Illegal instruction, line A, line F

UM 6.3.6 (p. 6-14): an opcode that is not a legal first word causes an illegal instruction exception (vector 4);
opcodes with bits 15-12 = 1010 or 1111 go to vectors 10 and 11. The stacked PC is the address of the
instruction.

In the core the illegal flag comes from the A1/A2/A3 decoder, and line A and line F from IR bits 15-12, all
registered one clock after IR (`illegal`, `linea_q`, `linef_q`, `m68k_cpu_si.sv`). At the dispatch they select the
entry `A_TRAP` = 0e3 (`instr_exc`, `m68k_exc_si.sv`) and the vector (`dispatch && a0_sub` block,
`m68k_exc_si.sv`).

## 12. Privilege violation

UM 6.1.2 and 6.3.7 (pp. 6-2, 6-15): in user mode the instructions that change the whole status register, STOP,
RESET, RTE and MOVE USP are privileged; the instruction is not executed, the stacked PC is its address, and the
vector is 8. On the MC68000 MOVE from SR is not privileged (UM 6.3.7 lists it for the MC68010 only), and MOVE to
CCR is not privileged either (PRM 4).

In the core `priv = ~S & priv_set` (`m68k_exc_si.sv`). In the core configuration `priv_set` is the privilege
output of the A1/A2/A3 decoder, registered like the illegal flag (`priv_ir`, `m68k_cpu_si.sv`). The set it decodes
is listed in the fallback form of `priv_set` in `m68k_exc_si.sv`: STOP, RESET, RTE, MOVE to SR, ANDI/ORI/EORI to
SR, MOVE USP. The decoder follows the die PLA table, with one recorded deviation: the PLA does not flag MOVE USP,
and the core makes it privileged as the PRM says (`docs/data/deviations/a123.tsv`, row `priv_set 4e60 4e6f`). An
opcode that is both illegal and in a privileged encoding takes the illegal vector (`priv & ~illegal`,
`m68k_exc_si.sv`).

## 13. Instruction traps: TRAP, TRAPV, CHK, divide by zero

UM 6.3.5 (pp. 6-13, 6-14): TRAP #n always traps through vector 32 + n; TRAPV traps with V set (vector 7); CHK
traps when the register is out of bounds (vector 6); DIVU and DIVS trap on a zero divisor (vector 5). The
stacked PC is the address of the next instruction.

In the core these exceptions are part of the instructions' microcode, which ends in the common trap words. The
vector number is a function of the instruction, `f_tvn` (`m68k_exc_si.sv`), registered when IRD is
loaded (`tvn_instr`).

## 14. RTE

PRM 4, RTE: privileged; SR and then PC are read from the supervisor stack, and the new S may switch to user mode.
In the core the SR word reaches the PSW through the FTU, `ftu -> psw`, which loads every implemented bit
(`ld_ftu`, `m68k_psw_si.sv`). In user mode RTE is a privilege violation (section 12).

## 15. STOP

PRM 4, STOP: privileged; loads SR from the immediate word and stops fetching until a trace, interrupt or reset
exception. In the core the wait is a loop of two words of its own, with the four-clock period of the die's loop (327
`aaa01` with the A1 dispatch, then 3a2 `stop1` whose next word is 327: `docs/data/mc68000_micro_decoded.csv`): the
dispatch of the STOP microword (`stop_w`, micro address 327) and every dispatch of 3fd while `stopped` is set select
`A_STOPW` = 3fc (STOP branch of the selection of `a0`, `m68k_exc_si.sv`); 3fc samples the interrupt request, has no
dispatch and its next word is 3fd, which samples it too and dispatches: the interrupt is taken there, every four clocks, at the instants of
the die's 327 dispatch (both are the second word after the STOP word), as measured on the reference core. `stopped` is
cleared when a trace or an interrupt is taken (`stopped` register, `m68k_exc_si.sv`); reset clears it as well. The die
has no word at 3fc or 3fd: the core adds them (`docs/data/deviations/ustore.tsv`, rows `3fc` and `3fd`), 3fd as a
dispatch word that also samples the interrupt request (`interrupts.md`). The selection at 327 only, rather than on the
STOP opcode, keeps a STOP executed in user mode, which traps, from entering the wait.

## 16. Verification

| Bench (`sim/`) | Make target | What it checks |
|---|---|---|
| `tb_m68k_exc.cpp` | `exc` | TRAP #0-#15 (vector, short frame, SR in the handler), RTE (SR and SSP restored, return to user mode), trace set by MOVE to SR and by RTE; traced TRAPV: the TRAPV exception processed before the trace exception, two frames, the trace handler first (UM 6.3.8) |
| `tb_m68k_illegal.cpp` | `illegal` | ILLEGAL, unassigned opcodes, line A and line F, in supervisor and user mode: vector, stacked PC = instruction, frame |
| `tb_m68k_priv.cpp` | `priv` | every privileged instruction in user mode: vector 8, the instruction has no effect, stacked PC = instruction |
| `tb_m68k_usp.cpp` | `usp` | MOVE USP in both modes, privilege violation in user mode |
| `tb_m68k_trapv.cpp`, `tb_m68k_chk.cpp`, `tb_m68k_chk_neg.cpp`, `tb_m68k_div0.cpp` | `trapv`, `chk`, `chkneg`, `div0` | the instruction traps: vector, stacked PC = next instruction, short frame; TRAPV, CHK with a negative register and DIVU/DIVS also from user mode, with the frame on the SSP |
| `tb_m68k_berr.cpp` | `berr` | BERR on reads, writes and a fetch: group 0 frame (status word R/W, I/N, FC; access address; IR; SR; PC as a range), vector 2 read in supervisor data space; double fault (BERR on the first push): `halted_n` low and no further cycle; double fault during reset (cases `reset bus error`, `reset address error`); BERR on the vector read of an ILLEGAL (case `group 1 fault`): I/N = 1 in the frame, no double fault |
| `tb_m68k_irq.cpp`, `tb_m68k_irq_multi.cpp` | `irq`, `irqmulti` | interrupt entry and RTE, a trace pending together with an interrupt, nested interrupts (`interrupts.md`) |
| `tb_m68k_stop.cpp` | `stop` | STOP waits until an interrupt above the mask; STOP in user mode traps |
| `tb_m68k_a123_si.cpp` | `a123_si` (in `test`) | illegal and privileged flags of the decoder on all 65,536 opcodes, with the documented deviations |

The address error is covered by `tb_m68k_berr.cpp` (case `address error with BERR`: frame
with R/W, I/N = 0, FC and the odd address, vector 3, no second exception for a BERR answered to the aborted cycle;
case `reset address error`: an odd initial PC is a double bus fault, the processor halts with no frame, section 9) and
the trace of an instruction that raises its own exception by `tb_m68k_exc.cpp` (traced TRAPV: the TRAPV exception first,
then the trace exception, the trace handler first; UM 6.3.8). A trace pending together with an interrupt is covered by
`tb_m68k_irq.cpp` (the trace exception first, then the interrupt exception before any instruction of the trace handler;
UM 6.2.3) and nested interrupts by `tb_m68k_irq_multi.cpp` (`interrupts.md`).
