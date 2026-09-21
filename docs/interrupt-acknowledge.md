# MC68000 interrupt acknowledge cycle

## 1. Purpose

Once an interrupt is taken (`interrupts.md`), the processor reads a vector number from the bus with an interrupt
acknowledge cycle. This document describes that cycle in rm68000: when it runs, its address, function code and
strobes, and how its termination selects the vector: vectored (DTACK), autovector (VPA) or spurious (BERR).

## 2. Sources

- MC68000UM Rev. 8: 5.1.4 CPU Space Cycle with Figure 5-10 (p. 5-9) and Figure 5-11 Interrupt Acknowledge Cycle
  Timing Diagram (p. 5-10); 3.1 Address Bus (p. 3-3) and 3.2 Data Bus (p. 3-4); 6.2.1 Exception Vectors and
  Figure 6-2 Peripheral Vector Number Format (pp. 6-4, 6-5); 6.3.2 Interrupts (pp. 6-12, 6-13); 6.3.3
  Uninitialized Interrupt and 6.3.4 Spurious Interrupt (p. 6-13); Table 6-2 note 3 (p. 6-7); Appendix B.2
  Interrupt Interface Operation with Figure B-6 (pp. B-4, B-5); Table 8-14 (p. 8-11).
- US 4,325,121 App. H, interrupt routine `itlx1`-`itlx7` (`docs/data/us4325121_appH.tsv`), and the die micro ROM
  dump (`docs/data/mc68000_micro_decoded.csv`) for the routine's order and function code fields.

RTL: `rtl/cpu_si/m68k_busctl_si.sv`, `m68k_exc_si.sv`, `m68k_ftu_si.sv`, `m68k_cpu_si.sv`, `m68k_bus_fsm.sv`,
and for the wrapper `m68k_cpu_dt_si.sv`. References give the signal names.

## 3. Behaviour

From the manual:

- Order (Figure 5-11): after the last bus cycle of the instruction the processor stacks the low word of the PC,
  then runs the acknowledge cycle (vector number acquisition), then stacks the rest of the frame and fetches the
  vector.
- The cycle is a read in CPU space: FC2-FC0 = 111, A19-A16 high (the interrupt acknowledge type), A3-A1 = the
  level being acknowledged, all other address lines high (UM 5.1.4, 3.1).
- "Although a vector number is one byte, both data strobes are asserted due to the microcode used for exception
  processing. The processor does not recognize anything on data lines D8 through D15 at this time" (Figure 5-11,
  note). The device places the vector number on D7-D0 (UM 3.2, Figure 6-2).
- Termination:
  - DTACK: vectored interrupt. The number read is the vector number; its address is the number times four
    (UM 5.1.4, 6.2.1).
  - VPA: autovector. The processor generates the vector number $18 + level, vectors 25 to 31 (UM 5.1.4,
    Table 6-2). The cycle is an M6800 read cycle synchronized to E, with VMA asserted (Appendix B.2,
    Figure B-6).
  - BERR: spurious interrupt. The processor takes vector 24 with the short frame of an interrupt instead of the
    bus error frame (UM 6.3.2, 6.3.4, Table 6-2 note 3).
  - "DTACK and VPA (AVEC) should never be simultaneously asserted" (UM 5.1.4).
- A device whose vector register has not been initialized answers with vector 15, the uninitialized interrupt
  vector (UM 6.3.3); for the processor it is an ordinary vector number.
- Table 8-14 gives 44 clocks with 5 reads and 3 writes for an interrupt, assuming a four-clock acknowledge cycle.

## 4. Implementation

### 4.1 Microcode

In the die micro ROM the interrupt routine runs `itlx1` ... `itlx7` and joins the common exception tail at
`trap4` (`exceptions.md`). From App. H:

| Word | Access | Relevant transfers |
|---|---|---|
| `itlx1` | none | `(psw) -> ftu`: copy of SR |
| `itlx2` | none | `(inl) -> psw`: mask = level, S set, T cleared |
| `itlx3` | none | `(inl) -> ftu`: acknowledge address into the FTU |
| `itlx4` | `twop` (total, write, operand) | first stack write; `ftu -> abe -> at`: acknowledge address into AT |
| `itlx5` | `trak` (total, read, acknowledge) | `(at) -> * -> aob`: the acknowledge cycle |
| `itlx6` | none | idle wait |
| `itlx7` | none | `(ir) -> ird` |

`itlx5` is the only microword of the die micro ROM whose function code field is 11 (die address 236). One stack
write precedes the acknowledge cycle, as in Figure 5-11.

### 4.2 Address

The FTU value loaded by `(inl) -> ftu` is `{12'hfff, inl, 1'b1}`, that is $FFF1 + 2 x level (`ld_inl` term of
`ftu_rest`, `m68k_ftu_si.sv`; the same constant appears in the simulation-only check `ld_val_v`).
The extension section sign-extends it on its way to AT, so the cycle runs at $FFFFF1 + 2 x level: A23-A4 all
high, A3-A1 = level, A0 = 1. A0 is internal and selects the strobes; the access type `ak` of `itlx5`
(`iack_acc`, `m68k_busctl_si.sv`) keeps the odd address from raising an address error (`aerr_c`).

### 4.3 Function code, strobes, `iack`

- The field 11 makes `fc_r` 111 (`m68k_busctl_si.sv`).
- The access is a word access (`byte_acc` is only for operand and `uk` accesses, `m68k_cpu_si.sv`), so both
  strobes are asserted, as in the note of Figure 5-11.
- `iack` is high while FC = 111 and AS is asserted (`m68k_cpu_si.sv`).

### 4.4 Termination and vector number

At the dispatch that takes the interrupt the vector number register is preset to the autovector,
`tvn_r <= {3'd0, 2'b11, inl}` = 24 + level (interrupt branch of the dispatch, `m68k_exc_si.sv`). The end of the
acknowledge cycle may then replace it:

- DTACK with VPA high: `iack_vec = done_now & (fc_r == 3'b111) & vpa_n & ~berr_pend` (`m68k_busctl_si.sv`)
  loads `tvn_r <= vec_num` (`m68k_exc_si.sv`), with `vec_num = data_in[7:0]` (instance `u_exc`,
  `m68k_cpu_si.sv`) taken on the edge that ends the cycle (S7). D15-D8 are ignored. The exception block updates
  `tvn_r` only on microcycle edges (`en`, `m68k_exc_si.sv`); the end of the cycle is one, because `itlx5` is a
  `t` word that ends with its cycle.
- VPA: the bus FSM runs the cycle synchronized to E (`m6800-bus.md`). VPA is still asserted at the end of the
  cycle, so `iack_vec` stays low and the autovector is used.
- BERR: `iack_spur = done_now & (fc_r == 3'b111) & berr_pend` (`m68k_busctl_si.sv`) loads
  `tvn_r <= 24` (`m68k_exc_si.sv`). For FC = 111 a BERR raises no bus error (`berr_ev`) and loads
  no group 0 wait (`g0_cnt`), so the interrupt routine goes on and writes the short frame.
- DTACK and VPA together, which the manual forbids: DTACK terminates the cycle (S5 branch of the bus FSM, `term_now`
  tested before VPA, `m68k_bus_fsm.sv`), and because VPA is low at the end the autovector is used, with the
  timing of a DTACK cycle.

The vector number becomes the vector address `{tvn, 2'b00}` in the FTU (`ld_trap` term, `m68k_ftu_si.sv`),
loaded by `trap4`, and the vector is read in the supervisor data space.

The level placed on the address bus is the latched `inl`, the level that was compared with the mask
(`interrupts.md`). A BERR together with HALT in this cycle is described in `bus-control-signals.md`.

### 4.5 Through `m68k_cpu_dt_si`

The read-data latch of the wrapper loads during CPU space cycles too: `cap_rd = bus_req & ~dtack_n & rw` has no
function code term (`rd_q`, `m68k_cpu_dt_si.sv`), so in an acknowledge terminated by DTACK the vector number placed
on D7-D0 reaches the processor through the latch, `rd_q` being the processor's `data_in` (UM 6.3.2; vectored case of
`sim/tb_m68k_cpu_dt.cpp`). With VPA (autovector) the data of the cycle is not used, and the generator gives no DTACK while
VPA is low; otherwise it asserts DTACK on every cycle with AS and a strobe low unless `bus_cs` and `bus_busy` hold it
(`dtack_assert`, `m68k_cen_dtack.sv`). See `bus-interface.md`, section 9.

## 5. Verification

| Bench (`sim/`) | Make target | What it checks |
|---|---|---|
| `tb_m68k_irq.cpp` | `irq` | for every interrupt exactly one acknowledge cycle: FC 111, A23-A4 all high, A3-A1 = level, read, LDS asserted; VPA answer, vector 24 + level; on the VPA answer VMA asserted (M6800 cycle, UM Appendix B) |
| `tb_m68k_irq_multi.cpp` | `irqmulti` | autovector on all levels 1-7; vectored with DTACK, vector $40 and $FF with other data on D15-D8; uninitialized vector 15; spurious (BERR in the acknowledge): vector 24 with the short frame, no bus error entry and no autovector |
| `tb_m68k_stop.cpp` | `stop` | acknowledge address $FFFFFA for level 5 and $FFFFFE for level 7 after STOP, vectors 29 and 31 |

Each runs with DTACK at zero wait states and three clocks late, in single-phase mode, where E is not the chip's one
(`m6800-bus.md` section 4.4): they check that VMA is asserted in the acknowledge cycle answered with VPA. The M6800
cycle of UM Appendix B (VMA asserted while E is low, E high with VMA asserted, AS negated as E goes low) is checked by
`tb_m68k_bus_fsm.cpp` with alternating phases, at every offset of E (`m6800-bus.md` section 5).
