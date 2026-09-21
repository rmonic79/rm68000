# MC68000 bus control signals: HALT, BERR, RESET, bus arbitration, high impedance

## Purpose

This document specifies the asynchronous bus control signals of rm68000 other than the data transfer handshake
(`bus-interface.md`) and the 6800 peripheral cycle (`m6800-bus.md`): HALT and the halted state, BERR with bus error,
retry and double bus fault, the RESET output, bus arbitration (BR, BG, BGACK) and the high-impedance flag `bus_tri`.

## Sources

MC68000 User's Manual, Rev. 8 (UM): 5.2 Bus Arbitration (5.2.1 to 5.2.3, figures 5-18 to 5-21), 5.4 Bus Error and Halt
Operation (5.4.1 Bus Error Operation, 5.4.2 Retrying the Bus Cycle, 5.4.3 Halt Operation, 5.4.4 Double Bus Fault,
Table 5-1, Table 5-6), 5.5 Reset Operation, 6.2 exception priority, 6.3.9.1 bus error.

## Behavior and implementation

All the logic is in `rtl/cpu_si/m68k_busctl_si.sv` (bus controller) and `rtl/cpu_si/m68k_bus_fsm.sv` (bus cycle state
machine), except the group 0 exception window (`g0_busy`, `rtl/cpu_si/m68k_exc_si.sv`) and the RESET output
(`rtl/cpu_si/m68k_cpu_si.sv`).

### HALT and the halted state

When HALT is asserted the processor completes the current bus cycle and then stops; it stays halted while HALT is
asserted (UM 5.4.3, figure 5-29). Bus arbitration is performed as usual while halted. In the core, with HALT low and the
bus idle no new cycle starts and the microcycle does not advance (`frozen`); the cycle in progress always completes.
The output `halted_n` is low while the processor is halted with the bus idle, by the HALT input or by a double bus
fault (`halt_int`), and while it waits for a retry with HALT asserted.

### Bus error

BERR terminates the current bus cycle. Without HALT the processor takes the bus error exception (vector 2) with the
long group 0 frame: the status word carries R/W and the function code of the aborted cycle, followed by the access
address and the instruction register (UM 5.4.1, 6.3.9.1). In the core the cycle terminated by BERR raises `berr_ev`
once, at its completion; the exception logic enters the group 0 routine. A bus error during the read half of a
read-modify-write cycle (TAS) terminates the whole cycle with AS negated, as for any other cycle (UM 5.4.1).

BERR during an interrupt acknowledge cycle is not a bus error: it is the spurious interrupt, vector 24, with the short
frame of the interrupt (see `interrupt-acknowledge.md`).

Group 0 priority is reset, then address error, then bus error (UM 6.2); the entry logic follows that order. In this
core an address error and a bus error are pending together only when the system answers BERR to the address phase
of the cycle aborted by the address error (AS without data strobes, ended at its first S5), since after a bus error
no request starts before the exception entry. The address error is taken and that BERR is discarded with it: "the
bus cycle is aborted" (UM 6.3.10), there is no cycle for a bus error to terminate. Bench: case `address error with
BERR` of `tb_m68k_berr.cpp` (vector 3 read once, vector 2 never).

The I/N bit of the status word (instruction or not) is 1 when the fault happens while the processor is processing a
group 0 or group 1 exception, and 0 when it happens during an instruction or a group 2 exception (UM 6.3.9.1 p. 6-16,
Figure 6-7). In the core it is `tvn_use` of the exception block (`m68k_exc_si`): set at the dispatch that enters a
group 1 exception routine and by reset, cleared by the routine's microword that loads the handler's first opcode; the
entry into the bus or address error routine leaves it unchanged. A bus error on the vector read of an ILLEGAL
instruction stacks I/N = 1 with R/W = 1, FC = 5 and the access address $10 (case `group 1 fault` of
`sim/tb_m68k_berr.cpp`).

### Retry of a bus cycle (BERR with HALT)

When BERR and HALT are both asserted at the termination sample of a bus cycle (end of S4; Table 5-1 case 5, UM 5.4.2),
the cycle is not a bus error: the processor terminates it (AS and the data strobes are negated in S7, the sequencer
receives no completion), stays halted with the address and data buses in the high-impedance state while HALT is
asserted, and once HALT is negated runs the same cycle again with the same function codes, address, R/W, data strobes
and, for a write, the same data. The retried cycle ends like any other: DTACK gives the normal completion, BERR with HALT
another retry, BERR alone a bus error. BERR must be negated at least one clock before HALT (UM 5.4.2, Table 5-6).

The manual fixes no delay between the negation of HALT and the start of the retried cycle. The core samples HALT at the
phi2 of its S7 wait; the next phi2 is S0/S1 of the retried cycle (buses driven again) and AS falls at the following phi1,
the same S0-to-S2 distance as a cycle started from idle.

Read-modify-write cycles (TAS) are never retried: a bus error during either half is a bus error whether or not HALT is
asserted (UM 5.4.2, note); with HALT asserted the processor stays halted until HALT is negated, then stacks the bus error
frame.

In the core the bus state machine decides at S5 (`retry_set`: BERR, HALT, not a read-modify-write cycle, not an
address-error cycle), suppresses the completion (`at_s7` low in S7), waits in S7 and restarts from S2, using the strobes
and the data registered by the first run (`uds_k`, `lds_k`, `rw`, `data_out`); S1 is skipped because the address and
the address-error decision belong to the first run. The restart needs HALT negated, no double fault and the bus neither
granted nor taken (`rerun_ok`). The bus controller drops `berr_pend` for the whole retry (`fsm_retry`), so that the BERR
of the aborted run does not become a bus error.

### Double bus fault

A bus error, or an address error, that occurs while the processor is processing a group 0 exception halts the
processor: HALT is asserted, the address and data buses are released, and only an external reset restarts it
(UM 6.3.9.1, 5.4.4). The window is the bus error, address error or reset processing, from its entry to the dispatch of
the first instruction of the handler; for reset, from the release of the reset to the first instruction, vector reads
included. A retry (BERR with HALT) is not a fault.

In the core `g0_busy` (`m68k_exc_si`) marks that window; it is 1 out of reset and is set by every immediate exception
entry. A cycle started with `g0_busy` high and terminated by BERR, or an address error with `g0_busy` high, sets
`dfault` in the bus controller; `halt_int` follows, then the processor is frozen with `halted_n` low until reset.

### RESET output

The RESET instruction asserts `reset_o` for 124 clocks (UM 5.5): the two microwords of its delay loop, 31 passes of two
words of two clocks each. The internal state of the processor is not affected and execution continues with the next
instruction.

### Bus arbitration

1. BR (input): a device requests the bus. The processor, at a lower priority, releases it after the current bus cycle
   (UM 5.2.1). If BR is negated before BGACK is asserted, the processor continues to use the bus.
2. BG (output): asserted as soon as possible, also during a bus cycle in progress (figure 5-19), except when the
   processor has decided the next bus cycle but has not yet asserted AS for it: then BG waits until AS is asserted
   (UM 5.2.2, figure 5-21). In the core `bg_n` falls at a phi1 with BR low when the bus is active and the state machine
   is past S1 (AS already asserted), or when the bus is idle and no request starts at that phi1 (`req`). With the
   alternating clock enables of the hardware a phi1 never sees S1, so BG falls together with AS at S2. With BG low and
   the bus idle no cycle starts (`frozen`).
3. BGACK (input): the device confirms that it is the bus master (UM 5.2.3). While BGACK is asserted the processor
   releases the address and data buses and the control signals, and starts no cycle; BG is negated.
4. When BGACK is negated the processor takes the bus back.

### High impedance (`bus_tri`)

`bus_tri` = 1 means that the address and data buses are in the high-impedance state:

- with the bus granted (BGACK asserted, UM 5.2): in this case the control signals AS, UDS, LDS, R/W, FC and VMA are
  released as well;
- while the processor is halted with the bus idle, by HALT or by a double bus fault (UM 5.4.3, 5.4.4);
- during the retry wait, until S0 of the retried cycle (UM 5.4.2);
- as long as BERR stays asserted after the cycle it terminated (UM 5.4.1).

Outside bus arbitration the control signals stay driven: FC holds its value, AS, UDS, LDS and R/W are negated
(figures 5-27 and 5-29). The flag does not tell the two cases apart; BGACK does.

## Verification

- `make -f Makefile.si halt` (`sim/tb_m68k_halt.cpp`): HALT asserted during a cycle (the cycle completes, then no
  cycle while HALT is held), the high-impedance flag while halted and during the retry wait, bus arbitration (BG within 400 clocks of BR, no AS and no cycle while BGACK
  is asserted, BG negated after BGACK), the retry of a write and of an instruction fetch, the RESET output (124 clocks, one assertion).
- `make -f Makefile.si berr` (`sim/tb_m68k_berr.cpp`): bus error on reads and writes with the full frame, the retry of
  a read and of a write without exception, TAS with BERR and HALT (bus error, no retry), double bus fault during bus
  error processing and during reset (bus error on a vector read, address error at the first fetch), with `halted_n` low, a bus error
  during the processing of a group 1 exception (the vector read of an ILLEGAL: I/N = 1, no double fault).
- `make -f Makefile.si cpudt` (`sim/tb_m68k_cpu_dt.cpp`): the processor with its clock-enable and DTACK generator,
  `m68k_cpu_dt_si`, as the cores use it.
