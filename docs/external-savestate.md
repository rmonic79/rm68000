# MC68000 external savestate

`rtl/savestate/m68k_ss_seq.sv` saves and restores the complete programmer-visible state of N MC68000 CPUs without
any port on the core. Each CPU saves itself, running a short handler that the module supplies on its bus, and is
brought back by a reset whose vector the module supplies. The module is optional and is not part of
`m68k_cpu_dt_si`.

Conventions. A name in code font given without a file is a signal, register, state or block of
`rtl/savestate/m68k_ss_seq.sv` in sections 4 and 5, and of the file named last elsewhere. An edge is a rising edge
of `clk`; "clock t" is the interval that begins at edge t. "Boundary B" is the instruction boundary at which a CPU
takes the save interrupt; SR_B and PC_B are the status register and program counter stacked there.

## 1. Purpose

- Save. On a command of the system the module brings each CPU to an instruction boundary with a level-7 interrupt
  and has it store its registers on its own supervisor stack. The one value that cannot be found again in memory,
  the supervisor stack pointer, goes to the module's section (two words per CPU), which the system's savestate
  stream stores together with the RAM.
- Restore. After the system's stream has reloaded the RAM and the section, the module resets each CPU and supplies
  a reset vector that sets the SSP to the saved stack and the PC to the restore part of the handler, which pops the
  registers and returns to boundary B with RTE.
- Nothing is read from inside the core: no access to the register file, the sequencer or the prefetch queue. The
  module observes AS, R/W, FC and the write data of each CPU, and acts on its read data, IPL, reset and clock
  enables through the system's glue. In the bench used during development it served rm68000 and an external
  reference core in one sequence (section 6).

Everything happens inside a pause of the whole system, requested by the module (`ss_pause`) and granted by the system
(`paused`), during which the module lets one CPU at a time run (`cpu_exec`).

## 2. Sources

- M68000 8-/16-/32-Bit Microprocessors User's Manual, M68000UM/AD Rev. 8, cited as [UM section, page]: programmer's
  model (2.1), instruction summary (2.5), privilege modes and stack pointers (6.1), exception processing (6.2),
  reset (5.5, 6.3.1), interrupts and the interrupt acknowledge cycle (6.3.2, 5.1.4), tracing (6.3.8), processing
  states (Section 6), double bus fault (5.4.4), execution times (Section 8).
- M68000 Programmer's Reference Manual, M68000PM/AD, cited as [PRM]: the encodings of the handler's instructions and
  the order in which MOVEM transfers registers. No copy of the PRM is in this repository; these references carry no
  page number. During development a bench ran the handler on rm68000 and on the reference core (section 6).
- The RTL of rm68000 (`rtl/cpu_si/`) for how it implements the facts used.

## 3. The 68000 facts the mechanism relies on

### 3.1 The state of a 68000

- Registers: D0–D7, A0–A6, the user stack pointer (USP), the program counter and the condition codes
  [UM 2.1.1, p. 2-1]; the supervisor stack pointer (SSP) and the status register [UM 2.1.2, p. 2-2], with T, S, the
  interrupt mask and X N Z V C [UM 2.1.3, p. 2-3]. The S bit selects SSP or USP as A7 [UM 6.1, p. 6-1; 6.1.1–6.1.2,
  p. 6-2]. MOVE to and from USP are privileged [UM 6.1.2, p. 6-2]: "If supervisor state then USP → An or An → USP"
  [UM 2.5, Table 2-2, p. 2-13].
- Processing states: "The processor is always in one of three processing states: normal, exception, or halted ...
  A special case of the normal state is the stopped state, resulting from execution of a STOP instruction. In this
  state, no further memory references are made ... Only an external reset can restart a halted processor. Note that
  a processor in the stopped state is not in the halted state, nor vice versa" [UM 6, p. 6-1].
- At an instruction boundary, with the instruction and its bus cycles complete, the program continues from the
  registers and the memory alone. The restore returns there through RTE, "(SP) → SR; SP + 2 → SP; (SP) → PC;
  SP + 4 → SP" [UM 2.5, Table 2-2, p. 2-14], after which the processor fetches its instructions again from the
  restored PC.

### 3.2 The save event: a level-7 interrupt

1. Not maskable, on a transition. "Priority level 7 is a special case. Level 7 interrupts cannot be inhibited by the
   interrupt priority mask, thus providing a 'nonmaskable interrupt' capability. An interrupt is generated each time
   the interrupt request level changes from some lower level to level 7" [UM 6.3.2, p. 6-13]. In rm68000 a change of
   the IPL input to 7, seen at any edge of `clk` and independently of the clock enables, sets `nmi_pend`, cleared when
   the level-7 interrupt is taken (`nmi_pend`, `rtl/cpu_si/m68k_exc_si.sv`); `int_req = nmi_pend | (ipl_w > imask)`.
2. At an instruction boundary. "Interrupt requests arriving at the processor do not force immediate exception
   processing, but the requests are made pending. Pending interrupts are detected between instruction executions"
   [UM 6.3.2, p. 6-12]; "Trace and interrupt exceptions allow the current instruction to execute to completion, but
   pre-empt the execution of the next instruction" [UM 6.2.3, p. 6-8]. rm68000 samples the request at the microword
   that begins an instruction (`upd_full`, `int_l`, `m68k_exc_si.sv`) and takes it at the next dispatch
   (`int_take`).
3. Exception sequence. "In the first step of exception processing, an internal copy is made of the status register.
   After the copy is made, the S bit of the status register is set ... the T bit is cleared ... For the reset and interrupt exceptions, the interrupt priority
   mask is also updated appropriately" [UM 6.2.5, p. 6-11]; "the processor priority level is set to the level of the
   interrupt being acknowledged" [UM 6.3.2, p. 6-12]. The vector number comes from an interrupt acknowledge cycle
   [UM 6.2.5, p. 6-11], which carries FC = 7, the level on A3–A1 and all other address lines high [UM 5.1.4,
   pp. 5-9–5-10]; with VPA instead of DTACK, "the vector number used is $18 plus the interrupt level" [UM 5.1.4,
   p. 5-10], vector 31 at $07C for level 7 [UM 6.2.1, Table 6-2, p. 6-7]. In rm68000: acknowledge address
   $FFFFF1 + 2·level (`ld_inl`, `rtl/cpu_si/m68k_ftu_si.sv`), `iack = (fc == 3'b111) & ~as_n`
   (`rtl/cpu_si/m68k_cpu_si.sv`), autovector 24 + level (`int_l` branch of the dispatch, `m68k_exc_si.sv`), vector
   number from the bus when the cycle ends with DTACK and VPA high (`iack_vec`, `rtl/cpu_si/m68k_busctl_si.sv` and
   `m68k_exc_si.sv`), vector 24 (spurious) when it ends with BERR (`iack_spur`, `m68k_busctl_si.sv` and
   `m68k_exc_si.sv`).
4. Frame and vector. "The current program counter value and the saved copy of the status register are stacked using
   the SSP" [UM 6.2.5, p. 6-11]; "The saved value of the program counter is the address of the instruction that would
   have been executed had the interrupt not been taken" [UM 6.3.2, p. 6-13]. The frame has three words: SR at the
   lowest address, then PC high and PC low [UM 6.2.4, p. 6-9; Figure 6-5, p. 6-10]. Then "the new program counter
   value is fetched from the exception vector" and the first instruction of the handler is fetched [UM 6.2.5,
   p. 6-11]. The sequence makes five reads and three writes, "Interrupt 44(5/3)" [UM 8.12, Table 8-14, p. 8-11].
   Since the vector address depends on the acknowledge and the handler's address on the vector, the two reads that
   follow the acknowledge cycle are the vector fetch.

Inside the handler the mask is 7 and T is 0: no interrupt other than a new transition to level 7 and no trace.

### 3.3 The restore event: reset

- "The processor is forced into the supervisor state, and the trace state is forced off. The interrupt priority mask
  is set at level 7 ... neither the program counter nor the status register is saved. The address in the first two
  words of the reset exception vector is fetched as the initial SSP, and the address in the last two words of the
  reset exception vector is fetched as the initial program counter" [UM 6.3.1, pp. 6-11–6-12]. "No other register is
  affected by the reset sequence" [UM 5.5, p. 5-29]. The reset vector takes four words in the supervisor program
  space [UM 6.2.1, Table 6-2, note 2, p. 6-7]; the reads are SSP high, SSP low, PC high, PC low, then the first
  instruction [UM 5.5, Figure 5-30, p. 5-29].
- "Only an external reset operation can restart a processor halted due to a double bus fault" [UM 5.4.4, p. 5-28].
  The reset is the only event that takes a 68000 from any state to an entry point and a stack chosen from outside.
- "For a subsequent external reset, asserting these signals for 10 clock cycles or longer resets the processor"
  [UM 5.5, p. 5-29]. rm68000 registers its reset input at every edge of `clk`, independently of the clock enables
  (`g_rst_copies`, `m68k_cpu_si.sv`); its reset sets the interrupt level latch `inl` to 7, which the reset microcode loads
  into the mask, giving SR = $2700 (`m68k_exc_si.sv`).

### 3.4 The state of a CPU when the save is requested

| state | what the 68000 does | result |
|---|---|---|
| inside an instruction | completes it, then takes level 7 [UM 6.2.3, p. 6-8; 6.3.2, p. 6-12] | PC_B = next instruction |
| inside a bus cycle, wait states included | the cycle completes normally: the module drives no DTACK, VPA or BERR | as above |
| processing another exception (group 0, 1 or 2) | that exception completes its stacking and vector fetch; the interrupt is processed before the first instruction of its handler [UM 6.2.3, p. 6-8] | PC_B = first instruction of that handler; its frame stays on the stack |
| T = 1 | "If the instruction is executed and an interrupt is pending on completion, the trace exception is processed before the interrupt exception" [UM 6.3.8, p. 6-15]; the trace exception clears T [UM 6.2.5, p. 6-11] | the saved state is the entry of the trace handler, S = 1, T = 0; after a restore the trace handler runs and its RTE returns to the traced program with T = 1 |
| user mode | exception processing sets S and stacks on the SSP [UM 6.1.3, pp. 6-2–6-3]; the USP is untouched | the handler saves the USP; the final RTE restores S = 0 |
| supervisor mode | the frame goes below the current SSP | the 70 bytes below SSP_B must be saved RAM (section 4.4) |
| executing a RESET instruction | an instruction: "The internal state of the processor is not affected" [UM 5.5, p. 5-29] | level 7 taken after it |
| stopped (STOP) | level 7 ends the stopped state; PC_B is the instruction after the STOP [UM 6.3.2, p. 6-13]; in rm68000 `stopped` in `m68k_exc_si.sv` | the CPU continues from PC_B after the save and after the restore, the request pending at the pause taken at the end of the RTE (section 5) |
| halted (double bus fault) | takes no interrupt [UM 6, p. 6-1; 5.4.4, p. 5-28] | served like a CPU in reset: an odd SSP in the section (the CPU's write data, with bit 0 of the low word forced to 1); restored into a running CPU it halts by a double bus fault (section 5) |
| held in reset by the system | does not execute | skipped (`cpu_in_reset`, section 4.6) |

## 4. The module

### 4.1 One sequence, no address comparison

One sequencer serves the N CPUs one after the other (`k`, the CPU served). The module never looks at the address
bus: it recognises the cycles of the CPU served by their type (FC, R/W, AS: `s_fc`, `s_rw`, `s_as_n`) and by their
position in the sequence. The handler is straight-line code, so its program reads come in order, one per word: the n-th
program read is the n-th word of the handler. The word to put on the bus comes from a table indexed by the position
(`rom()`); the SSPs live in a chain of registers read and written in order (section 4.8).

Cycles are seen at their end: the attributes are latched while AS is low and an access ends in the clock in which AS
rises (`q_rw`, `q_fc`, `acc_end`); `rd_end` is a completed read other than an acknowledge, `prog_end` a completed
program read (FC = x10), `wr_end` a completed write. Commands are taken only in the idle state (`IDLE`).

### 4.2 Parameters and ports

| name | dir | meaning |
|---|---|---|
| `N` | parameter | number of CPUs served. Default 2 |
| `WIN_BASE` | parameter | PC given to the handler: an even address whose reads the system acknowledges. Default $FF0000 |
| `H_SCR` | parameter | destination of the SSP write: an even address whose writes the system acknowledges and ignores. Default $FF0040 |
| `clk` | in | system clock |
| `rst` | in | synchronous reset, active high |
| `ss_save`, `ss_load` | in | single-clock commands from the system |
| `paused` | in | level: the system pause has been entered |
| `ss_busy` | in | level: the system's savestate stream is running |
| `slot_empty` | in | valid when `ss_busy` falls at the end of a read stream: the slot was never written |
| `ss_pause` | out | request of the system pause, held until the end of the sequence |
| `ss_mem_write`, `ss_mem_read` | out | single-clock pulses that start the system's write stream (save) or read stream (restore) |
| `sec_word` | out | head of the section chain, registered |
| `sec_rd` | in | pulse: the stream has taken `sec_word` |
| `sec_wr`, `sec_wdata` | in | pulse and data: the stream writes a word |
| `cpu_as_n`, `cpu_rw` | in | AS and R/W of each CPU, bit k for CPU k |
| `cpu_fc` | in | FC2–FC0 of each CPU, 3 bits per CPU |
| `cpu_dout` | in | write data of each CPU, 16 bits per CPU |
| `cpu_in_reset` | in | the CPU is held in reset by the system, not by this module |
| `cpu_exec` | out | the CPU runs during the pause |
| `cpu_ovr` | out | the CPU's read data is replaced |
| `cpu_sspsel` | out | ... with `sec_word`; otherwise with `cpu_word` |
| `cpu_word` | out | replacement word from the handler table, common to all CPUs |
| `cpu_ipl7` | out | force the CPU's IPL to level 7 |
| `cpu_iplhide` | out | hide the system's IPL (no request): `hide_q`, common to all CPUs, or the system pause for a CPU not served (section 4.6) |
| `cpu_soft_reset` | out | reset of the CPU |
| `state_o`, `phase_o`, `cpu_sel_o` | out | debug: sequencer state, phase (0 save head, 1 save tail, 2 restore), CPU served |

`WIN_BASE` and `H_SCR` must be even; an odd value is reported by an `$error` at time 0 of the simulation (the `initial` check). `cpu_ovr`, `cpu_sspsel`
and `cpu_word` are registers; `cpu_exec`, `cpu_ipl7` and `cpu_soft_reset` are a register ANDed with `k == g`;
`cpu_iplhide` is the register `hide_q` ORed with `paused` when `k != g` (generate block `g_cpu`).

### 4.3 Connection to the system and to `m68k_cpu_dt_si`

For CPU k, with `m68k_cpu_dt_si` and `SS_EXT = 0`:

- `pause_n = ~(paused & ~cpu_exec[k])`: the CPU stops while the system is paused, except when the module runs it.
  The wrapper registers `pause_n` and gates the clock enables one clock later, at any point of a bus cycle
  (clock-dtack.md, section 5). rm68000's phase-sequenced logic advances only on a pulse of its own phase
  (clock-dtack.md, section 3.1), so a pulse removed by the pause only delays it, and the module needs no alignment
  of the pause to whole periods.
- `bus_rdata = cpu_ovr[k] ? (cpu_sspsel[k] ? sec_word : cpu_word) : <data from the system memory>`. For a replaced
  read, `cpu_ovr` rises in the clock after AS falls and falls in the clock after AS rises (`ovr_now`, `ovr_q`). The
  wrapper's read latch loads only while DTACK is low, two clocks or more after the strobes fall (clock-dtack.md,
  sections 4.4 and 5), so it always takes the replacement word.
- `ipl_n = cpu_ipl7[k] ? 3'b000 : (cpu_iplhide[k] ? 3'b111 : <system ipl_n>)`; the IPL inputs of
  `m68k_cpu_dt_si` are active low (port `ipl_n` of `rtl/cpu_si/m68k_cpu_dt_si.sv`).
- `reset = <system reset of CPU k> | cpu_soft_reset[k]`, and `cpu_in_reset[k] = <system reset of CPU k> | ~halted_n[k]`: the CPU is
  held in reset by the system, or halted by a double bus fault (section 5).
- `cpu_as_n`, `cpu_rw`, `cpu_fc`, `cpu_dout` from the CPU's `as_n`, `rw`, `fc`, `data_out`.
- `ss_mem_write`, `ss_mem_read` to the system's savestate controller; `ss_busy` and `slot_empty` from it. The module
  registers `ss_busy` and `slot_empty` on input (`busy_q`, `empty_q`).
- The section to the system's stream through a slave (section 4.8).

### 4.4 What is saved, and where

At the end of the save the supervisor stack holds, from `SSP_M` (the SSP after the handler's pushes) upwards:

| offset from `SSP_M` | content | written by |
|---|---|---|
| +0 | USP (long) | `MOVE USP,A0` then `MOVE.L A0,-(SP)` |
| +4 … +63 | D0, …, D7, A0, …, A6 (15 longs, D0 at the lowest address) | `MOVEM.L D0-D7/A0-A6,-(SP)`: in predecrement mode MOVEM stores the registers from A7 down to D0 [PRM, MOVEM] |
| +64 | SR_B (word) | level-7 exception [UM 6.2.4, Figure 6-5, p. 6-10] |
| +66 | PC_B (long, high word first) | level-7 exception |
| +70 | = SSP_B, the program's stack as it was | — |

The 70 bytes below SSP_B must be in RAM that the system saves and restores, for every SSP value the program can
have at an instruction boundary.

The section holds 2N words: high and low half of `SSP_M` for each CPU, in CPU order. Nothing else: PC, SR, USP, D0–D7
and A0–A6 are on the stack. A CPU in reset or halted at the save contributes two words, its write data with bit 0 of the low word forced to 1 (an
odd SSP), which the restore discards.
The module saves no interrupt request and no other system state.

### 4.5 The handler

Index = position among the replaced reads of the CPU served (`pc_i`); the words come from `rom()` through `word_q`.
`R_ENTRY = WIN_BASE + 20` (a `localparam`).

| index | word(s) | instruction or use [PRM] | address at the save | address at the restore |
|---|---|---|---|---|
| 0, 1 | `WIN_BASE` high, low | vector fetch after the level-7 acknowledge | the vector's ($07C, $07E with an autovector) | — |
| 2, 3 | `48E7 FFFE` | `MOVEM.L D0-D7/A0-A6,-(SP)` | `WIN_BASE`+0, `WIN_BASE`+2 | — |
| 4 | `4E68` | `MOVE USP,A0` | `WIN_BASE`+4 | — |
| 5 | `2F08` | `MOVE.L A0,-(SP)` | `WIN_BASE`+6 | — |
| 6 | `4E71` | `NOP`: keeps the writes of `MOVE.L A0,-(SP)` apart from those of the next instruction | `WIN_BASE`+8 | — |
| 7–9 | `23CF`, `H_SCR` high, low | `MOVE.L SP,(H_SCR).L`: its two writes, high word then low, enter the section | `WIN_BASE`+10 … +14 | — |
| 16–19 | `sec_word` ×2, `R_ENTRY` high, low | restore only, before index 10: the reset vector, SSP from the section and PC = `R_ENTRY` | — | $000000 … $000006 |
| 10 | `205F` | `MOVEA.L (SP)+,A0`, start of the restore part | `WIN_BASE`+16 | `R_ENTRY` |
| 11 | `4E60` | `MOVE A0,USP` | `WIN_BASE`+18 | `R_ENTRY`+2 |
| 12, 13 | `4CDF 7FFF` | `MOVEM.L (SP)+,D0-D7/A0-A6` | `WIN_BASE`+20, +22 | `R_ENTRY`+4, +6 |
| 14 | `4E73` | `RTE` | `WIN_BASE`+24 | `R_ENTRY`+8 |
| 15 | `4E71` | fetched by the prefetch before `RTE` empties the queue | `WIN_BASE`+26 | `R_ENTRY`+10 |
| 16 | — | first read after `RTE`, at PC_B: not replaced; the CPU stops when it completes | PC_B | PC_B |

The module does not compare addresses, so the address of the restore entry only has to fall where the system
acknowledges reads: the save fetches index 10 from `WIN_BASE`+16, the restore from `WIN_BASE`+20, and both get the
same words. Altogether the handler reads `WIN_BASE` … `WIN_BASE`+31.

The positions are the module's design (header of the file), based on the 68000's two-word instruction
prefetch: an instruction at index a, n words long, starts when reads 0 … a+1 are complete and makes reads
a+2 … a+n+1, the last after its writes. `MOVE.L A0,-(SP)` (a = 5) writes with the read index at 7 or 8; the `NOP`
makes read 8; `MOVE.L SP,(H_SCR).L` (a = 7) writes with index 9 to 11, before its last read, 11. The SSP writes are
therefore exactly the writes with index ≥ 9 in the save head (`ssp_cap`), and the head ends at read 11 (state
`RUN`). `RTE` (a = 14) starts with 16 reads complete; read 16 is at the restored PC, and the tail ends there (state
`RUN`). The manual gives the number of reads of these instructions — `MOVE.L An,-(An)` 12(1/2) and
`MOVE.L An,(xxx).L` 20(3/2) [UM Table 8-3, p. 8-3], `MOVEM.L` registers to memory 8+8n(2/2n) [UM Table 8-10,
p. 8-8] — but not the prefetch, nor the order of reads and writes. The development bench (section 6) confirmed the
positions on both of its CPUs: its restore cases could not pass if a CPU had put fewer or more than two words in the section, which the two
CPUs share, or had stopped at another point.

### 4.6 The save sequence (sequencer state `st`)

1. In the idle state, `ss_save` raises `ss_pause` and selects the save head (`PH_HEAD`); the module waits for
   `paused` (`PAUSE`).
2. For each CPU k (`NEXT`):
   - held in reset or halted (`cpu_in_reset[k]`): two shifts of the chain (`SHIFT`) push the write data of that CPU
     (`w_in`), with bit 0 of the low word forced to 1, in place of its SSP;
   - otherwise `cpu_ipl7` and `cpu_exec` for CPU k (`ipl7`, `rel`). `ENTER` waits for its acknowledge cycle and then
     drops `cpu_ipl7`. `VEC`: the two reads after the acknowledge, the vector fetch, are replaced with `WIN_BASE`
     (`ovr_now`). `RUN`: the program reads are replaced with the handler words; the writes with index ≥ 9 enter the
     chain (`ssp_cap`). When read 11 completes `cpu_exec` falls (`rel`) and the CPU stops in the pause, at the
     boundary before `MOVEA.L (SP)+,A0`. `ADV` moves to the next CPU.
3. After the last CPU, `ss_mem_write` starts the system's write stream (`ADV`); the module waits for `ss_busy` to rise
   and fall (`STRA`, `STRB`). The stream stores the 2N section words and the system's RAM.
4. The save tail (`PH_TAIL`): for each CPU not in reset, `cpu_exec` again from read index 12 (`NEXT`). The CPU pops
   the USP and the registers and executes `RTE`; the replaced reads are 12 … 15; when read 16, at PC_B, completes,
   `cpu_exec` falls (`rel`). CPUs in reset are skipped.
5. After the last CPU `ss_pause` falls (`ADV`) and the system resumes. Every CPU continues from boundary B with the
   registers it had; the stack below SSP_B holds the data written by the handler.

While the system is paused, in the save and in the restore (section 4.7), `cpu_iplhide` hides the system's IPL from
each CPU until the `RTE` of its handler:

- `hide_q <= paused & (st != IDLE) & ~((st == RUN) & pc_i[4])`. In the save head a CPU sees no request, then the
  level 7 of the module: `hide_q` rises at the edge that leaves `PAUSE` and `cpu_ipl7` one edge later at the
  earliest, so the CPU sees no request for at least one clock before level 7. In rm68000 this is always a new
  transition to level 7 (`nmi_pend`, `m68k_exc_si.sv`), also when the system was requesting level 7 before the pause.
- `hide_q` falls one edge after program read 15 is counted, the `NOP` that fills the prefetch queue before `RTE` (in
  `RUN` `pc_i` is at most 16, so `pc_i[4]` means `pc_i == 16`), and is high again one edge after the sequencer leaves
  `RUN` at read 16. This happens in the save tail and in the restore; the save head stops at read 11. In that window
  the CPU makes the three pops of the handler's `RTE` (SR, PC high, PC low) and its first program read, at PC_B:
  four bus cycles back to back, 16 periods of the 68000 clock without wait states and one period more for each wait
  state. There the CPU sees the system's IPL and samples it: in `m68k_exc_si.sv` `ipl_w` is loaded only when the CPU
  advances (`en`). Stopped after read 16, the CPU keeps the sampled level while its IPL is hidden again, and when the
  pause ends a request that was pending at the pause is taken at the end of the `RTE`, before the instruction at
  PC_B, as in the run without savestate.
- `cpu_iplhide[g] = hide_q | (paused & ~sel)`: the CPUs not served stay hidden also during the `RTE` of the CPU
  served. They are stopped then and sample nothing, and a level-7 request of the system, which the mask cannot block
  [UM 6.3.2, p. 6-13], arms the `nmi_pend` of a CPU only during its own `RTE`, never in the body of its handler. With
  N = 1 the term is constant 0 and `cpu_iplhide` = `hide_q`.

The hiding adds no flop: the next state of `hide_q` has six inputs (`paused`, the four bits of `st`, `pc_i[4]`).

### 4.7 The restore sequence (sequencer state `st`)

1. In the idle state, `ss_load` raises `ss_pause` and selects the restore (`PH_REST`). When `paused` comes,
   `ss_mem_read` starts the system's read stream (`PAUSE`), and the module waits for it (`STRA`, `STRB`). The stream
   writes the 2N section words, in order, and the system's RAM.
2. If `slot_empty` is high at the end of the stream, `ss_pause` falls and nothing else happens (`empty_q`).
3. Otherwise, for each CPU (`NEXT`):
   - held in reset: its two words are dropped (`SHIFT`). The CPU is assumed to be in reset now exactly when
     it was at the save, which holds when whatever keeps it in reset is part of the state the stream has just
     restored;
   - otherwise `cpu_soft_reset` and `cpu_exec` for 64 clocks of `clk` (`srst`, `rel`; `{two, pc_i}` counts up to
     `t_rst`). Then `VEC` with index 16: the four reads of the reset vector are replaced, SSP high and low from the
     chain (`cpu_sspsel`, `ssp_now`; each of the two reads advances the chain, `vec_drop`), PC high and low =
     `R_ENTRY`. `RUN` from index 10: the prefetch at `R_ENTRY` and the restore part of the handler, reads 10 … 15,
     pop the USP and the registers and execute `RTE`; when read 16, at PC_B, completes, `cpu_exec` falls (`rel`).
4. After the last CPU `ss_pause` falls (`ADV`).

During the restore the system's IPL is hidden as during the save, up to the `RTE` of the handler of each CPU
(`hide_q`, section 4.6).

The soft reset lasts 64 clocks of `clk`. rm68000 takes it at `clk` rate (section 3.3). A real part needs at least 10
of its own clock periods [UM 5.5, p. 5-29], which 64 clocks of `clk` do not give at every divider: with
`den/num` = 12 one period is 12 clocks (clock-dtack.md, section 4.2), and 64 clocks are 5⅓ periods.

### 4.8 The section and its slave

The section is a chain of 2N 16-bit registers (`wq`). Each shift moves every word one place towards the head and
puts a new word at the tail; `sec_word` is the head. The chain shifts on `sec_rd`, on `sec_wr`, on an SSP write in the
save head, on each of the two SSP reads of a restore, and in `SHIFT` (`sh`); `rst` clears it. After the save head
it holds, from the head: SSP high and low of CPU 0, of CPU 1, and so on.

The system connects the section to its savestate stream through a slave that is not part of this repository,
because it depends on the system's savestate framework. The slave reports a size of 2N words; on every read request
it gives `sec_word` and pulses `sec_rd`; on every write request it pulses `sec_wr` with the word on `sec_wdata`. The
stream must access the section in order, from the first word to the last, each word once, in the same order at the
save and at the restore. The development bench did the same with one word per clock (section 6.1).

### 4.9 What the system must provide

1. A pause: after `ss_pause` rises, the system enters a state in which nothing that has state moves except the CPUs
   that the module runs, raises `paused`, and stays there while `ss_pause` is high.
2. Acknowledged accesses: the reads of the exception vector table and of the reset vector, the reads of `WIN_BASE` …
   `WIN_BASE`+31, and the two writes at `H_SCR`, which must not change any state the program uses.
3. Stack in saved RAM: the 70 bytes below every SSP value at an instruction boundary (section 4.4).
4. An answer to the level-7 acknowledge, like to any other: a vector with DTACK, VPA, or BERR. Whatever vector
   results, the two reads that follow are replaced.
5. Interrupt requests are system state, and the module saves none. The level-7 acknowledge is a bus cycle the system
   sees (FC = 7, level 7 on A3–A1): the device clears its request only on the acknowledge of its own level (on the
   Darius board the device sees A2 = 0 in the acknowledge cycle, with 4 and 7 the only levels present), so the request pending at the
   save is taken at the end of the handler's RTE, before the instruction at PC_B (section 4.6). A request pending at
   the save comes back after a restore only if the system raises it again, which the bench's pause, entered at the vblank that raises the request, does (section 6.1).
6. Nothing against level-7 requests during a save or a restore: level 7 is not masked [UM 6.3.2, p. 6-13], but the
   module hides the system's IPL from each CPU until the `RTE` of its handler (section 4.6), so such a request is
   never taken inside the handler: the CPU served sees it during its `RTE` and takes it after the `RTE`, before the
   instruction at PC_B.
7. `cpu_in_reset` driven by the system's own reset of each CPU, restored by the stream as section 4.7 assumes, ORed
   with the CPU's `~halted_n` (section 4.3).

## 5. Special states

- A CPU halted at the save (double bus fault) takes no interrupt [UM 6, p. 6-1; 5.4.4, p. 5-28]. The system reports
  it on `cpu_in_reset` (`halted_n` low, section 4.3) and the sequencer serves it like a CPU in reset: at the save its
  two section words are its write data with bit 0 of the low word forced to 1: an odd SSP (a saved SSP of a served
  CPU is even: the handler pushed on it); at the restore into a CPU in reset or halted they are discarded and the CPU stays as it is. Restored into a
  running CPU, the odd SSP makes the first pop of the handler tail an address error and the push of its frame a second
  one: double bus fault, the CPU halts like the chip [UM 6.3.9, p. 6-17] and the sequencer ends its restore on that
  first push (`rest & wr_end` in `RUN`, `m68k_ss_seq.sv`: the restore tail itself has no write). No flop, no new node:
  one input more on bit 0 of the chain input (`two`). Bench: cases `halt` and `halt_live` (section 6.2).
- A CPU in STOP at the save: level 7 wakes it and PC_B is the instruction after the STOP [UM 6.3.2, p. 6-13]. After
  the save and after a restore the CPU continues from PC_B, with the request of the system pending at the pause taken
  at the end of the RTE (section 4.6): the same continuation as the chip woken by that request. Bench: cases `stop_ref`
  and `stop` (section 6.3).

## 6. Verification

During development the module was verified with a bench that is not part of this repository: its second CPU is an
external 68000 core, used as a black box. The bench and its results are available from the author on request. This
section describes how it was made.

### 6.1 The bench

- Setup: CPU 0 was rm68000 (`m68k_cpu_si`), CPU 1 the external core, on the same clock, with one `m68k_ss_seq` with
  N = 2 serving both. The glue was that of section 4.3: read-data multiplexer, IPL, soft reset ORed into the resets,
  `cpu_in_reset` from the bench's own resets and from the halted outputs of the two CPUs. rm68000's clock enables
  were gated by a register of `~(paused & ~cpu_exec)`, at any point of a cycle, the same form as `m68k_cpu_dt_si`
  with `SS_EXT = 0`; the external core's gate changed only on its phi2. The stream read or wrote the four section
  words one per clock.
- The bench drove the clock enables itself, one pulse every two clocks; `m68k_cen_dtack` and `m68k_cpu_dt_si` were
  not used. The memory answered without wait states unless stated otherwise: DTACK as soon as AS and a data strobe
  were low, VPA on every acknowledge.
- The bench played the system: for each CPU a level-4 interrupt device that raised its request at a fixed period of
  system time (a "vblank") and dropped it at the end of the level-4 acknowledge cycle of that CPU only; the pause,
  entered at the vblank after `ss_pause` rose, which stopped system time; a stream that copied the CPU's memory and
  the section into a slot at `ss_mem_write` and put them back at `ss_mem_read`, with `ss_busy` high for 64 ticks.
  The interrupt request was not in the slot: since the pause was entered at the vblank that raised it, the request
  was pending at the save and again at the restore.
- Each test program ran first without savestate, which recorded the reference trace, then with a save and a
  restore. Each command was issued at a chosen vblank and took effect at the next vblank, where the pause was
  entered.
- Programs, vector table at $000000 and code at $400: `irq`, a supervisor loop at mask 3 with a short level-4
  handler (200 delay iterations); `irqh`, the same loop with a long handler (950 iterations, more than one interrupt
  period) and the vblank synchronised to a fetch inside it, so that the save finds the CPU in the handler at mask 4
  with the next request pending; `trace`, T = 1 with a loop of `NOP`s and a counting trace handler; `user`, user
  mode with USP = $7000 and a loop whose `MOVE.W D0,-(SP)` / `MOVE.W (SP)+,D1` make the USP visible on the bus;
  `halt`, an odd SSP ($8001) and `TRAP #0`: the frame write at an odd address gives an address error, and the frame
  of the address error another one, so the CPU halts on a double bus fault [UM 5.4.4, p. 5-28].

### 6.2 Cases and criteria

Each run lasted 1,500,000 ticks, with an interrupt period of 100,003 ticks:

| case | program | what | criterion |
|---|---|---|---|
| `irq_ref`, `irqh_ref`, `trc_ref`, `usr_ref`, `halt_ref` | each | run without savestate | lockstep of the two CPUs over the recorded run: no divergence, more than 1000 accesses |
| `irq_inv` | `irq` | save at vblank 3, restore as soon as the save is over | equivalence |
| `irq_eqv` | `irq` | save at vblank 3, restore at vblank 6 | equivalence |
| `irq_emp` | `irq` | save at 3, load at 6 from an empty slot | no restore on either CPU; both resume after the load pause and make more than 1000 accesses |
| `irqh_eqv` | `irqh` | save inside the level-4 handler | equivalence |
| `trc_eqv` | `trace` | save with T = 1 | equivalence |
| `usr_eqv` | `user` | save in user mode | equivalence |
| `held` | `user` | CPU 1 held in reset from vblank 2 to the end of the restore pause | both CPUs resume after the restore and make more than 1000 accesses |
| `halt` | `halt` | save at vblank 3, restore at vblank 6, both CPUs halted by the double bus fault | the restore runs and its pause ends; neither CPU makes an access from 2000 ticks after the end of that pause |
| `halt_live` | `irq` | save at vblank 3, restore at vblank 6 with the restored section carrying the marker SSP $00000001 of a halted CPU, while both CPUs run | the restore runs into the running CPUs and ends on their halt (double bus fault at the first pop of the odd SSP); neither CPU makes an access from 2000 ticks after the end of the restore pause |

Equivalence: after the restore, the accesses of rm68000 were compared with its own reference run. They were aligned
at the first level-4 acknowledge, within 16 accesses after the restore and within 64 after the save pause of the
reference; failing that, at the longest equal stretch near those two points, at least 64 accesses long. For each
access: address, R/W, UDS/LDS, FC, write data, duration (not that of an acknowledge, which depends on the phase of E)
and distance from the previous access (not right after an acknowledge). The comparison ran up to one interrupt
period after the end of the restore pause, minus 8192 ticks: after a restore the CPU restarts from boundary B while
system time restarts from the vblank, so from the next vblank on the interrupt falls at a different point of the
program. Pass: no divergence over at least 1000 accesses.

The lockstep between rm68000 and the external core and the consistency checks of the module's states were printed
but did not decide the save and restore cases. A run of the bench passed only if every case passed.

### 6.3 Conditions of the verification

- Every restoring case checked the equivalence after the restore (section 6.2) and the saved point: per case, the
  range of PC_B and the mask of SR_B read back from the saved stack.
- `irq_emp` (empty slot) and `held` checked the saved point and that the CPUs resume and run after the load pause or
  after the release from reset; `halt` checked that the halted CPUs are skipped and make no access after the
  restore.
- The STOP case (`stop_ref`, `stop`): the level 7 of the save wakes the stopped CPU and PC_B is the instruction after
  the STOP (section 5).
- The interrupt and user cases ran also with wait states on every access (0 to 3 wait states per address,
  `irq_eqv_w3` and `usr_eqv_w3`), each against its own reference run.
- The sequencer was driven by the test top with fixed clock-enable spacing; the clock-enable generator and the
  wrapper have their own bench in this repository (`clock-dtack.md` section 6.3).
- The equivalence criterion applied to CPU 0, rm68000; CPU 1 shared the sequence and the section (sections 4.1 and 4.8) and
  its lockstep with CPU 0 after the restore was reported.
