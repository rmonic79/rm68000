# MC68000 interrupt requests

## 1. Purpose

This document covers how rm68000 receives and recognizes hardware interrupts: the IPL inputs, the interrupt
mask, the non-maskable level 7, the instant at which a request is sampled and its priority against the other
exceptions. What happens on the bus once an interrupt is taken, the acknowledge cycle, the autovector, the
vectored and the spurious interrupt, is in `interrupt-acknowledge.md`. The exception sequence common to all
exceptions is in `exceptions.md`.

## 2. Sources

- MC68000UM Rev. 8: 3.5 Interrupt Control (p. 3-6); 6.3.2 Interrupts (pp. 6-12, 6-13); 6.2.3 Multiple Exceptions
  and Table 6-3 (pp. 6-8, 6-9); 6.2.5 Exception Processing Sequence (p. 6-11); 6.3.8 Tracing (pp. 6-15, 6-16).
- M68000PRM section 4: STOP, RTE.
- US 4,325,121 App. A (microword types `dbi`, `dbl`, `docs/data/us4325121_appA.csv`) and App. H (the interrupt
  routine `itlx1`-`itlx7`, `docs/data/us4325121_appH.tsv`).

RTL: `rtl/cpu_si/m68k_exc_si.sv` (sampling, level 7, recognition), `m68k_cpu_si.sv`, `m68k_psw_si.sv`,
`m68k_useq.sv`.

## 3. Behaviour

From UM 3.5 and 6.3.2:

- IPL2-IPL0 carry the encoded level of the request, IPL0 the least significant bit; all negated is level 0, no
  request ("all lines negated indicates no interrupt request", UM 6.3.2). The pins are active low.
- The status register holds a three-bit mask; requests at a level less than or equal to the mask are inhibited.
- A request does not force immediate processing: it is made pending and detected between instructions. If its
  level is not above the mask, execution continues and the interrupt waits until the level rises above the mask.
- When the level is above the mask: SR is copied, S is set, T is cleared and the mask is set to the level being
  acknowledged; the vector number is obtained with an interrupt acknowledge cycle; PC and the SR copy are
  stacked, the PC being the address of the instruction that would have executed next; the vector is fetched.
- Level 7 cannot be masked. An interrupt is generated each time the request changes from a lower level to 7; a
  level 7 interrupt can also come from the level comparison if the request is 7 and an instruction lowers the
  mask below 7.
- The request must stay asserted until the processor signals the acknowledge (FC2-FC0 and A19-A16 high) to be sure
  it is recognized (UM 3.5).
- Priority (UM 6.2.3, Table 6-3): interrupts are group 1, below trace and above illegal instruction and privilege
  violation. A trace pending at the end of an instruction is processed before the interrupt, so the interrupt
  handler runs first (UM 6.3.8).

## 4. Implementation

### 4.1 Pins and mask

`ipl_n[2:0]` is inverted at the top into the level `ipl` (`m68k_cpu_si.sv`). The mask is PSW bits 10-8
(`psw`, `m68k_psw_si.sv`; `imask`, `m68k_exc_si.sv`).

### 4.2 Sampling

- `ipl_s1` registers the level on every `cen2`, and `ipl_w` takes `ipl_s1` at the start of every microcycle,
  on `en` (`m68k_exc_si.sv`).
- `int_req = nmi_pend | (ipl_w > imask)` (`m68k_exc_si.sv`).
- The request is sampled once per instruction, in the microword that loads the next opcode into IR with a
  full update of the exception latches: the `dbi` type of App. A, which in the core is a microword with the
  IR-load bit set and bit 4 clear (`upd_full`, `m68k_cpu_si.sv`). There the block latches `int_l <= int_req`
  and, with a request, the level `inl` (`upd_full` branch, `m68k_exc_si.sv`). `int_l` is not sticky: each `dbi`
  word samples it again.
- The comparison therefore uses the level seen at the start of the `dbi` microword. This sampling instant was
  measured during development on an external reference core, used as a black box, with comparison benches that are
  not part of this repository (available from the author on request); the benches in `sim/` check only that a
  request held until the acknowledge is taken.

`upd_part` (`dbl` type, bit 4 set) would update only the immediate group; in the core that group is updated
continuously, so `upd_part` is unused (`unused`, `m68k_exc_si.sv`).

### 4.3 Level 7

- `nmi_pend` is set on a transition of the pin level to 7 (`ipl7_q`, `m68k_exc_si.sv`) and enters
  `int_req` whatever the mask.
- With a pending level 7 the latched level is 7 even if `ipl_w` has not caught up yet
  (`inl <= nmi_pend ? 3'd7 : ipl_w`, `m68k_exc_si.sv`).
- `nmi_pend` is cleared when a level 7 interrupt is taken (`int_take`, `m68k_exc_si.sv`). A
  level 7 held high therefore produces one interrupt; with the mask at 7 it produces no other. With a mask below
  7 the comparison `ipl_w > imask` also requests it, as in the manual.

### 4.4 Recognition

At the dispatch of the next instruction the exception block selects, in this order: a pending trace, a pending
interrupt, an instruction exception, the STOP wait (selection of `a0_sub` and `a0`, `m68k_exc_si.sv`). The
interrupt replaces the A1 entry with `A_INT` = 1c4, the die's `itlx1` (`A_INT`, `m68k_exc_si.sv`; `base`,
`m68k_useq.sv`). On that dispatch `int_l` is cleared and the vector number is preset to the autovector 24 + `inl`
(`dispatch && a0_sub` block, `m68k_exc_si.sv`); the acknowledge cycle may replace it (`interrupt-acknowledge.md`).

A trace taken at the same dispatch leaves `int_l` set; the interrupt is then taken at the dispatch that ends the
trace routine, if the request is still sampled there. A bus or address error clears `int_l`
(`a0s_sub` entry, `m68k_exc_si.sv`); the request is sampled again at the next `dbi` word.

### 4.5 Entry

In the interrupt routine (App. H): `itlx1` copies SR into the FTU; `itlx2` loads the mask from INL and sets S and
clears T through the atom `(inl) -> psw` (`ld_inl`, `m68k_psw_si.sv`); `itlx3` loads INL into the FTU to
form the acknowledge address; `itlx4` writes the first stack word; `itlx5` is the acknowledge cycle. The frame is
the short one, with the PC of the instruction that was not executed (`exceptions.md`).

At reset INL is 7 (`inl <= 3'd7` in the reset branch, `m68k_exc_si.sv`), so the reset routine, which loads the
mask from INL, leaves SR at 2700.

### 4.6 STOP

STOP waits in the loop of the two words `A_STOPW` = 3fc and 3fd, with the four-clock period of the die's loop 327, 3a2,
and leaves it when an interrupt (or a trace) is taken at the dispatch of 3fd (`stopped`, `m68k_exc_si.sv`). Both are
words the core adds to the die micro ROM (`docs/data/deviations/ustore.tsv`, rows `3fc` and `3fd`): both have the IR-load
bit set with bit 4 clear, so they sample the request like a `dbi` word at every word, as the reference core latches it;
only 3fd dispatches: one check per four clocks, at the instants of the die's 327 dispatch, the grid measured on the
reference core (`exceptions.md`, STOP).

## 5. Verification

| Bench (`sim/`) | Make target | What it checks |
|---|---|---|
| `tb_m68k_irq.cpp` | `irq` | levels 1-3 with mask 3 ignored for 600 clocks, then level 4 taken; level 7 with mask 7 taken once while held for 3000 clocks, also when the program rewrites SR, and taken again after release; level 6 from user mode; level 6 with mask 6 ignored, then 7 taken; level 1 with mask 0; in the handler S = 1, T = 0, mask = level, CCR unchanged; stacked PC = interrupted instruction; T = 1 with level 5 pending at the end of a traced DIVU: the trace exception first, then the interrupt exception before any instruction of the trace handler, two frames, the interrupt handler runs first, both returns clean (UM 6.2.3) |
| `tb_m68k_irq_multi.cpp` | `irqmulti` | all levels 1-7, RTE from supervisor and from user mode restoring the previous mask and mode; nested: inside the level 3 handler level 3 and level 2 are not taken and level 5 is, second frame below the first, the mask back to 3 and then to 2 at the two returns (UM 6.3.2) |
| `tb_m68k_stop.cpp` | `stop` | STOP with mask 3: level 3 ignored, level 5 wakes it; STOP with mask 7: level 6 ignored, level 7 wakes it |

Every case runs with DTACK at zero wait states and three clocks late. The manual fixes no clock for the sampling of
IPL within an instruction beyond the synchronisation of the lines (UM 6.3.2): the benches raise the request while the
CPU spins on a BRA.S and check the boundary at which it is taken.
