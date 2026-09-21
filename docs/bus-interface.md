# MC68000 bus interface

## 1. Purpose

This document describes the external bus of rm68000 as the processor module `m68k_cpu_si` exposes it: the
signals, the two-phase clock model, the asynchronous read and write cycle S0-S7 with wait states, the
indivisible read-modify-write cycle of TAS, CPU space cycles and the reset vector fetch. It also describes
`m68k_cpu_dt_si`, the wrapper that packages the processor with a clock-enable and DTACK generator.

Related documents:

| Topic | Document |
|---|---|
| BERR, HALT, bus arbitration (BR/BG/BGACK) | `bus-control-signals.md` |
| E, VPA, VMA and the synchronous M6800 cycle | `m6800-bus.md` |
| Function codes | `function-codes.md` |
| Interrupt acknowledge cycle | `interrupt-acknowledge.md` |
| Address error, bus error, double bus fault | `exceptions.md` |
| Clock-enable and DTACK generator | `clock-dtack.md` |

## 2. Sources

- MC68000UM Rev. 8 (M68000 8-/16-/32-Bit Microprocessors User's Manual, 1993):
  - section 3, Signal Description: 3.1 Address Bus (p. 3-3), 3.2 Data Bus and 3.3 Asynchronous Bus Control
    (p. 3-4), Table 3-1 Data Strobe Control of Data Bus (p. 3-5), 3.4 to 3.8 (pp. 3-5 to 3-8), Table 3-4
    Signal Summary (p. 3-10);
  - section 5, 16-Bit Bus Operation: 5.1.1 Read Cycle (pp. 5-1 to 5-4), 5.1.2 Write Cycle (pp. 5-4 to 5-7),
    5.1.3 Read-Modify-Write Cycle (pp. 5-7 to 5-9), 5.1.4 CPU Space Cycle (pp. 5-9, 5-10), 5.5 Reset Operation
    (p. 5-29);
  - 6.3.1 Reset (pp. 6-11, 6-12) and Table 6-2 Exception Vector Assignment (p. 6-7).
- Only the MC68000 in 16-bit operation is covered. The MC68008, MC68010, MC68HC001 and MC68EC000 material of the
  same manual does not apply.
- Where the manual leaves a timing open, the core was matched during development against an external reference
  core, used as a black box, with comparison benches that are not part of this repository (available from the
  author on request). Those places are marked "measured on the reference core" below.

RTL files cited (all under `rtl/`):

| File | Content |
|---|---|
| `cpu_si/m68k_cpu_si.sv` | processor top: ports, E counter, address register on the pins |
| `cpu_si/m68k_busctl_si.sv` | microcycle sequencer, bus requests, function codes, strobes, data lanes |
| `cpu_si/m68k_bus_fsm.sv` | bus cycle state machine S0-S7, RMW states |
| `cpu_si/m68k_cpu_dt_si.sv` | wrapper: processor + clock-enable/DTACK generator |
| `integration/m68k_cen_dtack.sv` | the clock-enable/DTACK generator |

References give the name of the signal, block or state branch they point to, with its file.

## 3. Signals

Ports of `m68k_cpu_si` (port list of `m68k_cpu_si.sv`) against the pins of the chip (UM section 3, Table 3-4). The core
has no bidirectional or three-state pins: every bidirectional chip signal is split into an input and an output.

| Chip signal | Core port(s) | Notes |
|---|---|---|
| CLK | `clk`, `cen`, `cen2` | system clock plus the two phase enables, see section 4 |
| A23-A1 | `addr[23:1]` | registered (`addr_r`, `m68k_cpu_si.sv`); A0 is internal and selects UDS/LDS |
| D15-D0 | `data_in`, `data_out`, `data_oe` | `data_oe` high while the CPU drives the write data |
| AS, UDS, LDS, R/W | `as_n`, `uds_n`, `lds_n`, `rw` | from the bus FSM |
| DTACK | `dtack_n` | sampled in S5, see section 5 |
| FC2-FC0 | `fc[2:0]` | `function-codes.md` |
| IPL2-IPL0 | `ipl_n[2:0]` | active low as on the chip (`interrupts.md`) |
| (none) | `iack` | high while FC = 111 and AS is asserted (`m68k_cpu_si.sv`) |
| E, VPA, VMA | `e`, `vpa_n`, `vma_n` | `m6800-bus.md` |
| BERR | `berr_n` | `bus-control-signals.md` |
| HALT | `halt_n` (in), `halted_n` (out) | `bus-control-signals.md` |
| RESET | `reset` (in, active high); `reset_o` (out) | `reset_o` is asserted for 124 clocks by the RESET instruction (`reset_o`, `m68k_cpu_si.sv`); see section 8 and `bus-control-signals.md` |
| BR, BG, BGACK | `br_n`, `bg_n`, `bgack_n` | `bus-control-signals.md` |
| (none) | `bus_tri` | tells the surrounding logic when the chip would float its address and data buses: bus granted, halted with the bus idle, retry wait, BERR held after its cycle (`bus_tri`, `m68k_busctl_si.sv`); `bus-control-signals.md` |
| (none) | `dbg_pc[23:0]` | low 24 bits of the PC, for debugging (`m68k_cpu_si.sv`) |

The remaining outputs (`upc`, `ird_o`, `psw_o`, `pc_o`, the `dbg_*` register ports of `m68k_cpu_si.sv`)
exist for the benches.

### 3.1 Data strobes and byte lanes

UM Table 3-1 (p. 3-5):

| UDS | LDS | R/W | D15-D8 | D7-D0 |
|---|---|---|---|---|
| high | high | - | no valid data | no valid data |
| low | low | high | valid data bits 15-8 | valid data bits 7-0 |
| high | low | high | no valid data | valid data bits 7-0 |
| low | high | high | valid data bits 15-8 | no valid data |
| low | low | low | valid data bits 15-8 | valid data bits 7-0 |
| high | low | low | valid data bits 7-0 (*) | valid data bits 7-0 |
| low | high | low | valid data bits 15-8 | valid data bits 15-8 (*) |

(*) "These conditions are a result of current implementation and may not appear on future devices." Word
accesses assert both strobes; a byte access asserts UDS for an even address (A0 = 0) and LDS for an odd one
(UM 5.1.1, 5.1.2).

In the core:

- a byte access is an operand or `uk` access of an instruction of byte size, or one half of a MOVEP word
  (`byte_acc`, `m68k_cpu_si.sv`); every other access, including the interrupt acknowledge, is a word
  access with both strobes;
- the strobes of a byte access come from A0 sampled in S1 (`a0_r`, `uds_q`/`lds_q`, `m68k_busctl_si.sv`);
- a byte write places the byte on both halves of the data bus (`wdata_q`, `m68k_busctl_si.sv`), which
  is what the starred rows of Table 3-1 show; MOVEP writes the high byte of the data output buffer
  (`dobh_sel`, `m68k_busctl_si.sv`);
- a byte read takes D15-D8 at an even address and D7-D0 at an odd one, and passes it inside on both halves
  (`rbyte`/`edb`, `m68k_busctl_si.sv`).

## 4. Clock model

The chip runs two bus states per clock period: one starts on the rising edge, the next on the falling edge, so
the minimum cycle S0-S7 takes four clocks (UM 5.1.1, Figure 5-3). The core uses no falling edge. It runs on one
system clock `clk` with two clock enables that alternate: `cen` marks the rising edge of the 68000 clock (phi1)
and `cen2` the falling edge (phi2). The bus FSM advances its even states (S0, S2, S4, S6) on `cen` and its odd
states on `cen2` (`is_even`/`phase_tick`, `m68k_bus_fsm.sv`).

The control store advances in microcycles of four phases, two clocks, `ph` = 0..3 with even phases on `cen`
and odd phases on `cen2` (`phase_ev` and the update of `ph`, `m68k_busctl_si.sv`). A bus cycle
without wait states spans two microcycles.

With `cen` and `cen2` both high on every clock (single-phase mode) every clock advances one bus state. The
instruction benches use this mode; the bus FSM accepts it explicitly (`single_cen`, `m68k_bus_fsm.sv`), but
cycle timing against E is not meaningful there (`m6800-bus.md`).

## 5. Read and write cycle

In the table, "branch Sx" is the case branch of `m68k_bus_fsm.sv` for state Sx. A branch runs on the clock
enable of the edge at which the chip enters state Sx: the S2 branch runs on the rising edge that starts S2,
the S5 branch on the falling edge that ends S4. The idle state `IDLE` (encoded 15) is S0
(the state `localparam`s of `m68k_bus_fsm.sv`).

| State | Manual (UM 5.1.1, 5.1.2) | Core |
|---|---|---|
| S0 | FC valid; R/W high | the cycle starts when `req` arrives in `IDLE` (branch `IDLE`); R/W is set high in every `IDLE` phase (branch `IDLE`); FC is registered with the request (`fc_r`, `m68k_busctl_si.sv`) |
| S1 | address valid | address error decided (`nostrobe <= aerr`, branch S1); address on the pins, see 5.1 |
| S2 | rising edge: AS asserted; read: UDS/LDS asserted; write: R/W low | branch S2: `as_n` low, `rw <= ~we`; read: strobes; write: the data register `data_out` loaded |
| S3 | write: data driven out of high impedance | branch S3: write: data bus driven (`data_oe`) |
| S4 | write: rising edge, UDS/LDS asserted; wait for DTACK, BERR or VPA | branch S4: write strobes |
| S5 | DTACK, BERR, VPA sampled on the falling edge at the end of S4; without them, wait states of whole clocks | branch S5: `term_now` goes to S6, otherwise back to S4 |
| S6 | read: data driven by the device | `rdata` follows `data_in` during S6 and S7 (`st[3:1] == 011`, `m68k_bus_fsm.sv`) |
| S7 | falling edge entering S7: read data latched, AS and strobes negated; rising edge at the end: buses released, R/W high | branch S7: AS and strobes negated, data bus released (`data_oe <= 0`), `bus_done` |

Termination is `term_now = ~dtack_n | ~berr_n | nostrobe` (`m68k_bus_fsm.sv`). Each round S4-S5 without
termination is one wait state of one clock, as on the chip ("wait states (full clock cycles)", UM 5.1.1 p. 5-4).

The UM note on p. 5-4 also says that a cycle with BERR asserted and DTACK absent ends one clock later, in S9.
How the core terminates a cycle on BERR is described in `bus-control-signals.md`; the VPA cycle in `m6800-bus.md`;
the address error cycle (`nostrobe`) in `exceptions.md`.

### 5.1 Differences of edge from the manual

All of these keep the signals valid whenever AS is asserted, which is when a slave samples them (UM 3.3: AS
indicates that the address on the bus is valid; UM 3.8: FC valid whenever AS is active).

- Address: the pins are loaded either with the request, so the address is on the pins from S0, or on the `cen2`
  of S1, from S1 (`aob_dly`/`addr_ld`, `m68k_busctl_si.sv`; `addr_r`, `m68k_cpu_si.sv`). The
  second case is taken when the microword loads the address output buffer from the AB bus, or from the DB bus
  with a register, AT, PC or DBIN source. Both timings were measured on the reference core.
- FC: registered with the request, so valid from S0 like on the chip, and held until the next request.
- Write data: the data register `data_out` is loaded in branch S2, together with AS and R/W; the data bus is driven
  (`data_oe`) from branch S3, as in the UM ("During S3, the data bus is driven out of the high-impedance state as
  the data to be written is placed on the bus", UM 5.1.2 p. 5-6, STATE 3), and released in S7 (the UM releases the
  bus at the end of S7). `tb_m68k_bus_fsm.cpp` checks that `data_oe` is low at the S2 edge of every write and high,
  with the written data, from S3 until the strobes are negated.
- Read data: `rdata` follows `data_in` at every clock from the tick entering S6 to the tick entering S7 (`st[3:1] ==
  011`, `m68k_bus_fsm.sv`) and holds from the S7 edge, where the UM accepts the data (UM 5.1.1, STATE 7). The word
  that completes the cycle consumes `edb` at that edge, so it sees the value sampled one FPGA clock before it, with
  alternating phases as with `cen` = `cen2`. Through `m68k_cpu_dt_si` the data come from the register `rd_q`, loaded
  at the DTACK and stable to the end of the cycle (section 9): the value one clock before the edge is the value at
  the edge.
  `rd_capture` (`m68k_bus_fsm.sv`) flags that same edge to the datapath.

### 5.2 Microcycles and bus requests

Each microword carries an access class (US 4,325,121 App. H, access column; header of `m68k_busctl_si.sv`):

- `i` (initiate): the request goes out in phase 0, the microword does not wait for the end of the cycle;
- `f` (finish): the microword ends only on the S7 of the cycle in progress;
- `t` (total): `i` and `f` in the same microword;
- `n`: no access.

The request `req` goes out on the `cen` of phase 0 of an `i` or `t` word, once per microword (`req`, `started`,
`m68k_busctl_si.sv`). A word that must wait keeps the phase on 3 (`stall`) until the end of the cycle
(`done_now = at_s7 & cen2`, `wait_done`). The control store advances on the `cen2` of phase 3 (`en`). An `i` or `t`
word that finds the previous cycle still running waits in phase 0 (`wait_free`). A cycle that ends before the `f`
word that consumes it is remembered in `done_l`. All of these are in `m68k_busctl_si.sv`.

## 6. Read-modify-write cycle (TAS)

From UM 5.1.3 (pp. 5-7 to 5-9): AS stays asserted for the whole cycle, which makes it indivisible; TAS is the
only instruction that uses it, so it is always a byte cycle. The read half is S0-S7 with the data latched and
UDS/LDS negated in S7 while AS stays low. "The bus signals are unaltered during S8-S11" while the ALU modifies
the data. The write starts in S12 with FC, address, AS and R/W unchanged; R/W goes low on the rising edge of
S14, the data goes on the bus in S15, UDS or LDS are asserted on the rising edge of S16 with wait states as in a
write, and AS and the strobes are negated on the falling edge entering S19.

In the core:

- The cycle is an RMW only for the operand read of TAS: `rmw = tas_op & (acc_op | acc_uk)` (the `rmw` input of
  the bus FSM instance in `m68k_busctl_si.sv`), with `tas_op` = IRD bits 15-6 equal to 0100 1010 11 (the `tas_op`
  connection in `m68k_cpu_si.sv`). Vector
  reads and fetches made while IRD holds TAS are normal cycles.
- Read half: the normal S0-S7 with `rmw_cyc` set (branch `IDLE` of `m68k_bus_fsm.sv`). Branch S7 negates the
  strobes, leaves AS low, reports `bus_done` and goes to S8.
- S8 is held, with every bus signal unchanged, until the write request arrives (branch S8); `rmw_hold` is high
  there. It covers S8-S13 of the manual. Being a single even state,
  it takes the request on whatever `cen` it arrives, including in single-phase mode.
- On the request the FSM drives R/W low and loads the data (manual S14) and goes to S10. Branch S10, one clock
  later, enables the data bus and asserts the strobes (manual S16). Branch S11 samples DTACK half a clock later;
  S12/S13 is the wait loop, one clock per round (branches S12 and S13). The end goes through S6 and S7, where AS and
  the strobes are negated (manual S19).
  The strobe-to-end time of the write half was measured on the reference core.
- The controller issues the write request on the second `cen` of phase 0 of the write microword: `rmw_ok`
  (`m68k_busctl_si.sv`) becomes true one clock after that word reaches phase 0, and `req` and `wait_free` let the
  request through during `rmw_hold`.

## 7. CPU space cycles

A cycle with FC2-FC0 all high is a CPU space cycle; A19-A16 give its type, and on the MC68000 the only type is
the interrupt acknowledge, with A19-A16 high (UM 5.1.4, Figure 5-10, p. 5-9). The core issues a CPU space cycle
only from the interrupt acknowledge microword; the cycle, its address and its termination are described in
`interrupt-acknowledge.md`. The breakpoint acknowledge cycle belongs to the MC68010 and does not exist here.

## 8. Reset

From UM 5.5 (p. 5-29) and 6.3.1 (pp. 6-11, 6-12): the reset sets S, clears T and sets the interrupt mask to 7;
it saves neither PC nor SR; it reads the initial SSP from the long word at address 0 and the initial PC from the
long word at address 4, both in the supervisor program space (Table 6-2, note 2); execution starts at that PC.

In the core:

- `reset` (active high) resets the processor. Unlike the chip, it does not need HALT asserted with it. With
  `RST_REG` = 1 the reset is distributed through three registered copies (block `g_rst_copies` of `m68k_cpu_si.sv`).
- State after reset: PSW T = 0, S = 1, mask 7 (reset branch of `m68k_psw_si.sv`); exception block with vector 0
  selected and INL = 7 (reset branch of `m68k_exc_si.sv`, `tvn_r` and `inl`); micro address 000, the entry of the
  reset routine (`base` forced to 0 under reset, `m68k_useq.sv`).
- The microcycle phase starts at 1 (`ph <= 2'd1`, `m68k_busctl_si.sv`), so the first advance waits for a
  `cen2`. The delay from the release of reset to the first vector read was measured on the reference core during
  development.
- The vectors are read as four word reads at 0, 2, 4, 6 with FC = 110 (supervisor program). The 110 comes from
  the function code field of the reset microwords, not from the reset value of the function code register
  (the `fc_code_n` selection and the reset value of `fc_code`, `m68k_busctl_si.sv`).
- The RESET instruction asserts `reset_o` for 124 clocks and execution continues with the next instruction
  (`bus-control-signals.md`, RESET output).
- A bus error during the reset vector reads, or an address error at the first fetch, is a double bus fault and
  halts the core: see `exceptions.md`, section 9.

## 9. The `m68k_cpu_dt_si` wrapper

`m68k_cpu_dt_si` puts the processor and the generator `m68k_cen_dtack` in one hierarchy node
(header of `m68k_cpu_dt_si.sv`). The generator is described in `clock-dtack.md`; what matters for the bus:

- The phases come from the generator (`u_dtack`, `m68k_cpu_dt_si.sv`), gated by `pause_n` or, with
  `SS_EXT` = 1, by the external savestate, and registered (`cen_q`, `cenb_q`). They are also output
  (`cen_o`, `cenb_o`) for the surrounding logic.
- DTACK comes only from the generator: the wrapper has no DTACK input. The generator asserts DTACK on every cycle
  with AS and at least one data strobe low, unless `bus_cs` and `bus_busy` are both high or VPA is low
  (`dtack_assert`, `dtack_n_next`, `m68k_cen_dtack.sv`; its `VPAn` input is the wrapper's `vpa_n`). A surrounding
  system delays a cycle only through `bus_cs`/`bus_busy`.
- The read data pass through a latch `rd_q` that loads `bus_rdata` on every clock of a read cycle with a strobe
  and DTACK low, interrupt acknowledge cycles included: the latch has no FC input (`cap_rd`,
  `m68k_cpu_dt_si.sv`). The processor reads `rd_q` (its `data_in`, instance `u_cpu`).

Two consequences for the surrounding logic:

- Interrupt acknowledge: an acknowledge terminated by DTACK loads `rd_q` like any read, so the vector number placed
  on `bus_rdata` (D7-D0) by a vectored interrupt reaches the processor through the latch (UM 6.3.2, p. 6-12). An
  autovectored acknowledge is answered with VPA and a spurious one with BERR; neither uses the data
  (`interrupt-acknowledge.md`).
- VPA cycles: DTACK has priority over VPA in the bus FSM (S5 branch), and the generator never asserts DTACK while
  `vpa_n` is low (`VPAn` of `u_dtack`, `clock-dtack.md` section 4.4), so a cycle answered with VPA runs synchronous
  to E with no action of the surrounding logic (`m6800-bus.md`).

## 10. Verification

| Bench (`sim/`) | Make target (`sim/Makefile.si`) | What it checks |
|---|---|---|
| `tb_m68k_bus_fsm.cpp` | `busfsm` | the bus FSM alone, edge by edge, in two-phase and single-phase mode: read and write strobes, R/W, data window (on a write the data bus not driven at S2, driven with the data from S3), DTACK sampling and wait states, the RMW cycle (AS held, strobes negated between the halves, R/W before the strobes, DTACK sampled one edge after the strobes); in two-phase mode also AS asserted on `cen` and the VPA cycle against E (`m6800-bus.md`); not the exact edge of the data latch inside the strobe window; the length of a cycle ended by BERR only as a range |
| `tb_m68k_cpu_si.cpp` | `cpu_si` (in `test`) | the whole processor: reset vector reads at 0/2/4/6 with FC 110, FC on every cycle, both strobes on words and one on bytes, word order of long accesses, E period; run twice, with `cen` or `cen2` first after reset |
| `tb_m68k_rstvec.cpp` | `rstvec` | reset: four word reads with FC 110, SSP and PC with non-zero high words, SR after reset, first fetch at the PC read |
| `tb_m68k_tst.cpp` | `tst` | TAS in memory: read and write inside one AS, read first, byte strobe only, supervisor data FC |
| `tb_m68k_mem.cpp` | `mem` | byte, word and long accesses with (An), (An)+, -(An): a byte read asserts one strobe and must take its own lane while the bench drives the complement on the other |
| `tb_m68k_halt.cpp` | `halt` | HALT, bus arbitration and the RESET instruction (`reset_o` asserted once, for 124 clocks, and the program continues) |
| `tb_m68k_cpu_dt.cpp` | `cpudt` | the wrapper `m68k_cpu_dt_si` of section 9 with its generator: DTACK at a+2+Lb, wait states from `bus_busy`, lost and returned clock-enable pulses, interrupt acknowledge with VPA and vectored with the vector number through `rd_q`, pause (`clock-dtack.md`, section 6.3) |

The instruction benches run every program twice, with DTACK at zero wait states and three clocks late. The
cycle-by-cycle match with the reference core was established during development with comparison benches that are
not part of this repository; they and their results are available from the author on request.
