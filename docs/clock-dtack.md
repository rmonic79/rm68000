# MC68000 clock-enable and DTACK generator

`rtl/integration/m68k_cen_dtack.sv` generates the two clock-enable phases that pace rm68000 inside one fast FPGA
clock domain, and the DTACK handshake of its bus. DTACK is held back while the system memory reports itself busy;
the clock-enable pulses that fall into such a stall are counted and returned later, between bus cycles. The module
is part of the CPU: `rtl/cpu_si/m68k_cpu_dt_si.sv` instantiates it next to the processor `m68k_cpu_si`
(section 5).

Conventions. File names refer to this repository; a signal named without a file belongs to
`rtl/integration/m68k_cen_dtack.sv` in section 4, to `rtl/cpu_si/m68k_cpu_dt_si.sv` in section 5, and to the file named
last elsewhere. An edge is a rising
edge of `clk`; "clock t" is the interval that begins at edge t. A register assigned at edge t holds its new value during clock t; a register that samples at
edge t sees the values of clock t−1.

## 1. Purpose

A real 68000 has its own clock input and acts on both of its edges. rm68000 runs on the system clock `clk` and
advances only on two clock-enable inputs: `cen` (phi1, the rising edge of the 68000 clock) and `cen2` (phi2, the
falling edge). The generator:

1. produces `cpu_cen` and `cpu_cenb` (the processor's `cen` and `cen2`): single-clock pulses that alternate and never
   coincide, at an average rate of `clk·num/den` per phase; one 68000 clock period is one `cpu_cen` plus one
   `cpu_cenb`;
2. produces `DTACKn` for every bus cycle that needs an acknowledge, and keeps it high while the system reports
   `bus_cs & bus_busy`;
3. counts the pulses emitted while the processor waits on a held DTACK, and emits as many extra pulses later, in
   clocks where no regular pulse is due and the bus is free.

## 2. Sources

- M68000 8-/16-/32-Bit Microprocessors User's Manual, M68000UM/AD Rev. 8, cited as [UM section, page]: the bus
  cycle states and the sampling of DTACK (5.1.1–5.1.3, 5.6, 5.8), E and VPA (3.7, Appendix B).
- The processor's RTL in `rtl/cpu_si/`, for what rm68000 does with the clock enables and with DTACK (section 3).
- The choices the manual leaves to the system — the divider, the latency of DTACK, which pulses count as lost, when
  they are returned, the width of the counter — follow the behaviour at the pins of an external reference generator
  that is not part of this repository: rules R0–R4 of section 4 are the ones under which the two give identical outputs
  clock by clock in standard conditions, in a comparison that is not part of this repository either (available on
  request). The bench of section 6.1 checks this module against the rules of sections 3 and 4 alone.

## 3. What the processor requires

### 3.1 Two alternating phases

- "The standard M68000 bus cycle consists of four clock periods (eight bus cycle states) and, optionally, an
  integral number of clock cycles inserted as wait states" [UM 5.8, p. 5-35]. In each period one state begins on the
  rising edge of the clock and one on the falling edge [UM 5.1.1, p. 5-4; 5.8, pp. 5-35–5-36].
- In rm68000 the even bus states (S0, S2, S4, S6 and the TAS states S8, S10, S12) advance only on `cen`, the odd ones
  only on `cen2` (`is_even`/`phase_tick`, `rtl/cpu_si/m68k_bus_fsm.sv`). The microcycle phase `ph` advances on `cen`
  from an even phase and on `cen2` from an odd one (`phase_ev` and the update of `ph`, `rtl/cpu_si/m68k_busctl_si.sv`).
  A pulse of the other phase is ignored: the processor waits for the next pulse of its own phase. Nothing in the
  processor counts `cen | cen2`.
- After reset `ph` starts at 1 (`ph <= 2'd1` in the reset branch, `m68k_busctl_si.sv`): the first pulse that moves
  the microcycle is a `cen2`. The generator's first pulse after reset is always `cpu_cen` (section 4.8), so every
  reset starts the same way.
- The microcycle sequencer stays on its last phase until the bus cycle reaches S7 (`stall` and `en`,
  `m68k_busctl_si.sv`): every wait state lengthens the current microword, and the instruction, by
  one full period.
- E comes from the processor: `e_cnt` counts the `cen2` pulses 0..9 (`rtl/cpu_si/m68k_cpu_si.sv`) and
  `e = (e_cnt >= 6)`, six periods low and four high, as in the manual: "A single period of clock E consists
  of 10 MC68000 clock periods (six clocks low, four clocks high)" [UM 3.7, p. 3-8]; "it is low for six system clocks
  and high for four system clocks" [UM B.1, p. B-2]. E therefore follows the `cpu_cenb` pulses that reach the
  processor.

### 3.2 The bus cycle, and when DTACK is looked at

From the manual:

- "If neither termination signal is asserted before the falling edge at the end of S4, the processor inserts wait
  states (full clock cycles) until either DTACK or BERR is asserted" [UM 5.1.1, p. 5-4; the same for writes, 5.1.2,
  p. 5-6]. Wait states appear as whole periods in the timing diagrams ("2 WAIT STATE READ", Figure 5-3, p. 5-3).
- "Until the falling edge of the clock at the end of S4 (beginning of S5), no response from any external device
  except RESET is acknowledged by the processor" [UM 5.8, p. 5-36].
- "If DTACK is asserted with the required setup time before the falling edge of S4, no wait states are incurred, and
  the bus cycle runs at its maximum speed of four clock periods" [UM 5.8, p. 5-39].
- "External logic circuitry should respond to the negation of the AS and UDS, LDS, and/or DS by negating DTACK"
  [UM 5.8, p. 5-37]; "DTACK, BERR, and HALT should be asserted and negated on the rising edge of the processor
  clock" [UM 5.6, p. 5-30].
- Read-modify-write (TAS): "The address strobe (AS) remains asserted throughout the entire cycle, making the cycle
  indivisible ... all read-modify-write cycles are byte operations" [UM 5.1.3, p. 5-7]. The read portion terminates
  like a read (S4, p. 5-8); the write portion asserts the data strobes at the rising edge of S16 and waits for DTACK
  at the falling edge that closes S16, like a write [UM 5.1.3, p. 5-9].

The bus FSM of rm68000 (one case branch of `m68k_bus_fsm.sv` per state):

| state | enable | action |
|---|---|---|
| S0 (`IDLE`) | `cen` | a request starts the cycle; R/W high |
| S1 | `cen2` | address error decided: if so, no data strobes in this cycle |
| S2 | `cen` | AS low; read: UDS/LDS low; write: R/W low, write data register loaded |
| S3 | `cen2` | write: data bus driven |
| S4 | `cen` | write: UDS/LDS low |
| S5 | `cen2` | samples DTACK and BERR (an address-error cycle ends here by itself, `term_now`): if one is asserted, S6; otherwise back to S4, one wait state of a full period. VPA cycles wait for E here |
| S6 | `cen` | read: `rdata` follows `data_in` (during S6 and S7; the value is taken at the edge entering S7) |
| S7 | `cen2` | AS, UDS, LDS high, end of cycle; in the read half of a TAS only UDS/LDS go high and AS stays low |
| S8 | `cen` | TAS: held until the write request, then R/W low |
| S10 | `cen` | TAS write half: data driven, UDS/LDS low |
| S11 | `cen2` | samples DTACK/BERR: S6, then S7 with AS high, or S12 |
| S12, S13 | `cen`, `cen2` | wait states of the write half; sample at S13 |

What the generator takes from this:

- A cycle needs an acknowledge when AS is low and at least one data strobe is low. An address-error cycle never
  lowers a data strobe and ends by itself at S5 (`term_now`, `m68k_bus_fsm.sv`). In a TAS the data strobes rise
  at the end of the read half with AS still low and fall again for the write half: two handshakes under one AS.
- DTACK matters only on the `cen2` pulses that sample it: S5 (S11 in the TAS write half) and the S5 (S13) of every
  wait state. A DTACK that arrives between two samples shortens nothing.
- VPA: "Data transfer acknowledge (DTACK) must not be asserted while VPA is asserted ... If both signals are asserted,
  the operation of the state machine is unpredictable" [UM B.1, p. B-4]. The generator has a `VPAn` input: while VPA
  is low it never asserts DTACK (R1, section 4.4), so a VPA cycle is ended by the processor on E [UM 5.1.4, p. 5-10].
  Pulses of a VPA cycle are counted as lost under the same condition as in any other cycle (section 4.5: memory busy
  while the processor waits). Where VPA does not exist, `VPAn` is tied to 1. In rm68000 a DTACK found at the S5
  sample would end the cycle before VPA is examined (branch S5 of `m68k_bus_fsm.sv`): this is why the generator must
  not give it.

### 3.3 Spacing between pulses

The generator emits pulses in consecutive clocks when `den = 2·num` (a nominal pulse in every clock) and when a
recovery pulse is placed next to a nominal one. The processor relies only on the alternation of the two phases; the
bus FSM bench drives `cen` and `cen2` on alternate clocks (`tick` in `sim/tb_m68k_bus_fsm.cpp`, section 6.2).

## 4. The generator

### 4.1 Ports and parameters

| name | dir | meaning |
|---|---|---|
| `W` | parameter | width of `den`; `num` has `W−1` bits. Default 8 |
| `RECOVERY` | parameter | 1: lost pulses are counted and returned; 0: DTACK only. Default 1 |
| `WD` | parameter | the debt counter has `W+WD` bits. Default 6 |
| `clk` | in | system clock |
| `rst` | in | synchronous reset, active high |
| `num`, `den` | in | divider: average rate `clk·num/den` per phase; intended range `2·num ≤ den` |
| `cpu_cen`, `cpu_cenb` | out | phi1 and phi2 enables, registered |
| `bus_cs`, `bus_busy` | in | from the system: both high = memory busy, DTACK held |
| `ASn`, `DSn[1:0]` | in | AS and {UDS, LDS} of the processor |
| `VPAn` | in | VPA of the processor, active low: while low, DTACK is never asserted (R1); 1 where VPA does not exist |
| `DTACKn` | out | registered |
| `bus_legit`, `bus_ack`, `wait2`, `wait3` | in | no logic (`unused_ok`); `m68k_cpu_dt_si` ties them to 0 |
| `fave`, `fworst` | out | constant 0 |
| `cen_nx`, `cenb_nx` | out | next value of `cpu_cen` / `cpu_cenb` (`pulse & ~phase`, `pulse & phase`): outside reset, `cpu_cen` in clock t equals `cen_nx` in clock t−1 |

### 4.2 R0 — divider (`acc`, `nominal_due`)

At every edge `acc` grows by `2·num`. A nominal pulse is due when `acc + 2·num > den`, with a strict comparison
(`nominal_due`), and `acc` then drops by `den` (`acc_after`). Spacing with `num = 1`:

| `den` | 12 | 8 | 6 | 4 | 3 | 2 |
|---|---|---|---|---|---|---|
| clocks between pulses | 6 | 4 | 3 | 2 | 2, 1, 2, 1, ... | 1 |
| clocks per 68000 period | 12 | 8 | 6 | 4 | 3 | 2 |

Outside the intended range: with `den = 0` no nominal pulse is ever due (`den != 0` in `nominal_due`); with
`2·num > den` one is due in every clock, and `acc` is clamped to `den` (`acc_next`) so that it cannot grow. If `den`
is lowered while running and `acc` is above the new value, a nominal pulse is due at once and at most one more in the
next clock; the regular spacing follows.

### 4.3 Phases (`pulse`, `phase`)

Every pulse, nominal or recovery (`pulse`), goes out as `cpu_cen` when `phase` is 0 and as `cpu_cenb` when it
is 1, and toggles `phase`. There is at most one pulse per clock and the two phases strictly alternate. `phase` is 0
after reset, so the first pulse is `cpu_cen`.

### 4.4 R1 — DTACK (`active`, `active_q`, `dtack_assert`, `dtack_n_next`)

- `active`: AS low and at least one data strobe low, from the current inputs (`as_low`, `ds_low`); `active_q`:
  `active` one clock earlier; `hold = bus_cs & bus_busy`.
- `DTACKn` is a register, loaded with `dtack_n_next`. It goes high at the first edge that samples `active` low, that
  is AS high or both data strobes high. While the cycle stays active it goes low at an edge where `active_q`, `active`,
  `~hold` and `VPAn` are all true (`dtack_assert`), and once low it stays low. With `VPAn` low (a VPA cycle) it stays
  high for the whole cycle.

Timing. Let the cycle become active at edge a: the processor lowers the last strobe it needs at edge a, in response
to a `cpu_cen` in clock a−1 (S2 for a read or the read half of a TAS, S4 for a write, S10 for the write half of a
TAS). Edge a+1 samples the active cycle and loads `active_q`; `DTACKn` goes low at edge a+2+Lb, where Lb is the
number of consecutive clocks, starting with clock a+1, in which `hold` is high (Lb = 0 when the memory is not
busy); `VPAn` is high in this paragraph and in the ones that follow.

A sample of the processor (S5, S11 or S13) on a `cpu_cenb` in clock j reads the value of `DTACKn` in clock j, so it
finds DTACK if and only if j ≥ a+2+Lb. With c = a−1, the clock of the `cpu_cen` that lowered the strobes: the sample
finds DTACK when j − c ≥ 3 + Lb.

With no debt, only nominal pulses occur inside a cycle. For an even `den` the spacing is s = den/(2·num): the first
sample of a read comes 3s clocks after its S2 pulse, the first sample of a write (or of the TAS write half) s clocks
after its S4 (S10) pulse, and each wait state moves the sample 2s clocks later. Wait states per access:

- read: max(0, ⌈(3 + Lb − 3s) / 2s⌉);
- write, TAS write half: max(0, ⌈(3 + Lb − s) / 2s⌉).

Computed from these, with `num = 1`:

| `den` | s | read, Lb = 0 / 1 / 2 | write, Lb = 0 / 1 / 2 |
|---|---|---|---|
| 12 | 6 | 0 / 0 / 0 | 0 / 0 / 0 |
| 8 | 4 | 0 / 0 / 0 | 0 / 0 / 1 |
| 6 | 3 | 0 / 0 / 0 | 0 / 1 / 1 |
| 4 | 2 | 0 / 0 / 0 | 1 / 1 / 1 |
| 2 | 1 | 0 / 1 / 1 | 1 / 2 / 2 |

With a debt, the pulse that follows S2 in a read (S3) can be a recovery pulse in the clock right after S2, because
AS is still high in the clock where it is decided (section 4.6); the first sample of that read then comes earlier.
Writes are not affected: their S4 and S5 pulses are decided with AS low, so both are nominal.

### 4.5 R2 — lost pulses (`miss`)

`miss = RECOVERY & (cpu_cen | cpu_cenb) & hold & active_q & DTACKn`, with `hold = bus_cs & bus_busy`, evaluated at
every edge on the values of the clock before: the pulse emitted at the previous edge t, present in clock t, counts as
lost only when, in that clock t, the memory is busy (`hold`) and the processor is really waiting for this
acknowledge: the cycle was already active at edge t, that is in clock t−1 (`active_q`), and DTACK has not been given
(`DTACKn` high). The pulse itself goes out anyway.

R2 counts pulses of either phase received while the memory holds the acknowledge; it is not a count of wait states.
During a long stall the S3 and S4 pulses of a read count as lost although they cause no wait state; and a failing
sample in the clock right after the last clock of `hold` is not counted (`hold` is already low in that clock while
`DTACKn` is still high), although it costs a wait state. This is the counting under which the module matches the
reference generator at the pins (section 2).

The conditions `active_q` and `DTACKn` restrict the count to pulses during which the processor is in a cycle and
still waiting for this acknowledge. They exclude, for instance, a `hold` raised in the very clock in which the
strobes fall, by a memory map that reacts within the same clock (`active_q` is still 0 there), and a `hold` raised
again after DTACK has been given (`DTACKn` low). The bench runs both cases (section 6.1, `MAPLAT=0` and
`BUSYBACK=1`).

### 4.6 R3 — recovery (`supp`)

`supp = RECOVERY & (debt != 0) & ~nominal_due & ~as_low`: with a non-zero debt, a recovery pulse is emitted in
clock t when, in clock t−1, AS was high and no nominal pulse is due at edge t. Recovery pulses therefore fall only
between bus cycles, never while AS is low, and never displace a nominal pulse. They alternate with the other pulses
like any pulse, and the processor cannot tell them from nominal ones.

Recovery uses only clocks with AS high and no nominal pulse. With `den = 2·num` every clock after the first pulse
carries a nominal pulse, so nothing can be returned and the debt only grows. When the pulses lost over a series of
accesses exceed what the free clocks between them can take, the debt grows until it wraps (R4).

### 4.7 R4 — debt counter (`debt`, `debt_next`)

`debt` has `W+WD` bits: 14 with the defaults, up to 16383 pulses. At every edge `debt ← debt + miss − supp`, modulo
2^(W+WD) (`debt_next`): above the maximum it wraps to 0 and the accumulated debt is lost; there is no saturation. `supp`
requires `debt != 0`, so the counter never goes below zero; a `miss` and a `supp` at the same edge cancel. With
`RECOVERY = 0` both are 0 and the debt stays 0.

### 4.8 Reset and first pulse (`rst`)

While `rst` is sampled high: `acc`, `phase`, `debt` and `active_q` are cleared, `cpu_cen = cpu_cenb = 0`,
`DTACKn = 1`. After the release, `acc + 2·num` is `2·k·num` at the k-th edge with `rst` low, so the first pulse is
due at the first k with `2·k·num > den`, k = ⌊den / (2·num)⌋ + 1, and `cpu_cen` is high in the clock after that edge.
With `num = 1`: k = 7, 5, 4, 3, 2, 2 for `den` = 12, 8, 6, 4, 3, 2. The first pulse is always `cpu_cen`.

### 4.9 Cases at the edges

| case | behaviour |
|---|---|
| debt beyond 2^(W+WD) − 1 | wraps to 0 (R4); no double pulse |
| reset during recovery | debt, accumulator and phase cleared; the first pulse is `cpu_cen` again |
| new stall with a debt already open | the debt keeps growing (R2); recovery resumes at the next clocks with AS high |
| `bus_cs` without `bus_busy` | DTACK as with no stall, nothing counted |
| `bus_busy` falling and rising again within a cycle | DTACK, once low, stays low until the strobes rise (`dtack_n_next`), never withdrawn; nothing counted after it (R2) |
| address-error cycle | never active: `DTACKn` stays high, nothing counted |
| BERR during a stall | the processor ends the cycle on BERR (`term_now`, `m68k_bus_fsm.sv`); the pulses counted during the stall are returned later |
| HALT, bus arbitration | outside the generator: the processor starts no cycle while halted or without the bus (`frozen`, `m68k_busctl_si.sv`) and keeps receiving pulses |
| interrupt acknowledge | an active cycle like any other: the generator does not look at FC |
| `den = 2·num` | no recovery possible (R3) |

### 4.10 Size and paths

28 registers with the defaults (2W + WD + 6: `acc` of W+1 bits, `debt` of W+WD bits, `phase`, `active_q`,
`cpu_cen`, `cpu_cenb`, `DTACKn`), no memory. `cpu_cen`, `cpu_cenb` and `DTACKn` are registered: there is no
combinational path from the inputs to them. `cen_nx` and `cenb_nx` are combinational and depend on `ASn` through
R3 (`supp`, `pulse`).

## 5. Integration in `m68k_cpu_dt_si`

- Reset. `reset` (active high, not registered) is registered in `rst_q`, power-up value 1, for the generator
  (`rtl/cpu_si/m68k_cpu_dt_si.sv`). The processor registers the same input in three copies (parameter `RST_REG = 1`
  of the wrapper and of the instance `u_cpu`; block `g_rst_copies` of `m68k_cpu_si.sv`): generator and processor leave
  reset at the same edge.
- Instance (`u_dtack`): `W` and `RECOVERY` from the wrapper's parameters, `WD` at its default; `ASn` and `DSn` from
  the processor's pins; `VPAn` from the wrapper's `vpa_n` input, the same VPA the processor receives; `bus_legit`, `bus_ack`, `wait2`, `wait3` at 0; `fave`, `fworst` unconnected.
- Clock enables of the processor (`cen_q`, `cenb_q`). `cen_q <= cen_nx & gate_nx` and `cenb_q <= cenb_nx & gate_nx`,
  cleared by `rst_q`; `cen_o` and `cenb_o` drive the processor's `cen` and `cen2` and leave the module. With
  `SS_EXT = 0`, `gate_nx = pause_n`, so `cen_o` in clock t is `cpu_cen(t) & pause_n(t−1)`: the pause acts one clock
  later, at any point of a bus cycle. Under Verilator the module checks this equivalence at every clock (the
  comparison with `cpu_cen & halt_q`). With `SS_EXT = 1` the gate comes from an external savestate block
  (`gate_q_i`, `gate_raw_i`) and can change only in the clock after a `cpu_cenb`, that is at whole periods
  (`gate_nx`); under Verilator `cen_o` is checked against `cen_ext_i`. The savestate of this repository,
  `rtl/savestate/m68k_ss_seq.sv`, works with `SS_EXT = 0` and `pause_n` (external-savestate.md, section 4.3).
- The pause acts after the generator: divider, phase and debt run on the ungated pulses. While the processor is
  paused inside a bus cycle with the memory busy and DTACK not given, the pulses emitted count as lost (R2) and are
  returned after the pause.
- `DTACKn` goes to the processor and out as `dtack_n_o`; `cen_raw_o` and `cenb_raw_o` are the ungated `cpu_cen`
  and `cpu_cenb`.
- Read data (`rd_q`, `cap_rd`). `rd_q`, the processor's `data_in`, loads `bus_rdata` at every edge whose previous
  clock has AS low with a data strobe low, `DTACKn` low and R/W high (`cap_rd = bus_req & ~dtack_n & rw`); the latch
  has no FC input. The system's `bus_rdata` must be valid while `DTACKn` is low in a read. Interrupt acknowledge
  cycles (FC = 7) terminated by DTACK load `rd_q` like any other read: the vector number of a vectored interrupt is
  the byte read in that cycle [UM 6.3.2, p. 6-12], the processor takes it from `data_in[7:0]` (`vec_num` of the
  exception block, `m68k_cpu_si.sv`), so the number placed on `bus_rdata` reaches the processor through the latch.
  Autovectored (VPA) and spurious (BERR) acknowledges do not use the data. With `SS_EXT = 1` and `ss_ovr_i` high,
  `ss_word_i` is loaded instead of `bus_rdata` (the savestate multiplexer in front of `rd_q`).

## 6. Verification

### 6.1 The generator on its own

Files: `sim/Makefile.cendtack`, `sim/top_cendtack.sv`, `sim/tb_cendtack.cpp`, `sim/run_cendtack.sh`. The bench needs
nothing outside this repository. From `sim/`, `make -f Makefile.cendtack cendtack` builds it and runs
`run_cendtack.sh`; `make -f Makefile.cendtack cendtack-lint` lints the module alone. `CENDTACK_SRC` builds the bench
on another copy of the module (mutation testing) and `BUILDCD` sets the build folder (header of `Makefile.cendtack`).

- Setup (instance `u_dut` of `top_cendtack.sv`): the module with `#(.W(8), .RECOVERY(1))` (`WD` = 6 by default, a
  debt counter of 14 bits), `bus_legit`, `bus_ack`, `wait2` and `wait3` at 0, `VPAn` at 1 (the bench runs no VPA
  cycle; the effect of `VPAn`, no DTACK on a VPA cycle, is checked in the package bench of section 6.3,
  `sim/tb_m68k_cpu_dt.cpp`). The bench reads only `cpu_cen`, `cpu_cenb` and `DTACKn`.
- Processor model (`VC::step`), driven by the module's pulses: S0 on a `cpu_cen` after the programmed idle pulses, AS
  (and the strobes of a read or TAS) low at S2, the strobes of a write at S4, DTACK sampled at S5 and at every wait
  state, the TAS write half with strobes falling on a `cpu_cen` and the sample on the next `cpu_cenb`, address-error
  cycles without strobes (section 3.2).
- Order within a clock (the main loop over `clk`): `rst` for the edge; edge; outputs read; the processor model acts
  on the outputs of the previous clock (`VC::step`); the memory model sets `bus_cs` and `bus_busy` (`VC::stalls`);
  inputs applied, evaluation, outputs read again; checks of the clock.
- Memory model (`stall_len`, `VC::stalls`). At every handshake `bus_cs` rises `MAPLAT` clocks after the strobes fall
  (default 1) and `bus_busy` stays high from that point for 1 clock for other targets, 2 for the RAM region, and for
  the ROM region by mode — `idle`: no `bus_cs` and no busy at all; `map`: 2; `short`: 2 + 0..4; `long`: 2 + 0..`LMAX`
  (default 64); `burst`: 2 + 0..64 on five accesses out of eight, 2 on the others; `sat`: 2 + 2000..2499 on one access
  out of four, 2 on the others (the debt passes the counter's maximum and wraps). `BUSYBACK=1` raises `bus_busy` again
  after the stall until the end of the cycle; `MAPLAT=0` raises `bus_cs` and `bus_busy` in the same clock as the
  strobes.
- Access sequence (`build_synth`): 20000 accesses (`NACC`), 10/16 reads, 4/16 writes, 1/16 TAS, 1/16 address errors;
  70 % in the ROM region, 20 % in the RAM region, 10 % elsewhere; mostly back to back, one access in eight preceded by
  0 to 10 idle pulses.
- Checks (`fail`, one `CHECK` line each). The expected values are computed by the bench at run time from this
  document; none comes from a run of the module. Every check counts in the final `RESULT` line:
  - `RESET` (4.8): while `rst` is sampled high, `cpu_cen` = `cpu_cenb` = 0 and `DTACKn` = 1;
  - `FIRST` (4.8): the first pulse after the release is `cpu_cen`, at the k-th edge with `rst` low,
    k = ⌊den / (2·num)⌋ + 1;
  - `PHASES` (4.3): at most one pulse per clock; a pulse is `cpu_cen` when the phase is 0 and `cpu_cenb` when it is
    1, and toggles the phase;
  - `REGISTERED` (4.1, 4.10): `cpu_cen`, `cpu_cenb` and `DTACKn`, read after the edge, do not change when the inputs
    of the clock are applied;
  - `R0` (4.2): a pulse at every edge where the bench's own divider (`acc` grows by 2·num, a nominal pulse is due when
    acc + 2·num > den, then `acc` drops by `den`) has a nominal pulse due;
  - `SPACING` (4.2): in the `idle` mode with `num` = 1, the clocks between consecutive pulses are those of the table
    of 4.2;
  - `R3` (4.5–4.7): a pulse that is not nominal comes at an edge if and only if the debt is not zero, AS was high in
    the clock before and no nominal pulse is due. The debt is the bench's: +1 for every pulse lost under R2 (the
    pulse in clock t, `bus_cs & bus_busy` in clock t, the cycle active in clock t−1, `DTACKn` high in clock t), −1 for
    every recovery pulse, modulo 2^14 (R4);
  - `R1` (4.4): `DTACKn` at every clock is the value R1 gives from the clock before: high if the cycle was not
    active; otherwise low if it was low, or if the cycle was active in the two clocks before and
    `bus_cs & bus_busy` was low in the clock before;
  - `WAITS` (4.4): every sample of the processor model (S5, S11, S13), in clock j, finds DTACK if and only if
    j ≥ a+2+Lb; in the `idle` mode with `den` a multiple of 2·num, the wait states of every access equal the formulas
    of 4.4 with Lb = 0;
  - `COMPLETE` (3.2): the processor model completes the whole sequence, with a second acknowledge in every TAS;
  - `RETURNED` (1, 4.5–4.7): the run ends with a tail in which the bus is free, until the debt is returned (at most
    2^21 clocks). With `den` > 2·num the debt is then 0 and the recovery pulses equal the lost pulses, less 2^14 for
    every wrap of the counter; with `den` = 2·num there is no recovery pulse at all (4.6). The distance from the
    nominal clock (pulses − 2 × wait states − nominal pulses) equals lost pulses − 2 × wait states − the debt left −
    2^14 per wrap; section 4.5 gives no other bound for it: it is above zero when R2 counts pulses that cost no wait
    state (S3 and S4 of a read in a long stall), below zero for wait states with no lost pulse (for example the
    writes of the table of 4.4 at `den` 4 and 2). In the `sat` mode the debt must wrap at least once.
- At the first failure the bench prints the 16 clocks up to it (the `WINDOW` print): pulses, `DTACKn`, the values the
  rules give, strobes, `bus_cs`, `bus_busy`, nominal pulse due, state of the processor model, access and debt. The
  last lines of a run are `FAILED CHECKS`, `SUMMARY` (result, distance from the nominal clock, a hash of `cpu_cen`,
  `cpu_cenb` and `DTACKn` over every clock, and the pulse counts of the two conditions below) and `RESULT`.

`run_cendtack.sh`, run by `make -f Makefile.cendtack cendtack`:

1. eighteen configurations with the synthetic sequence (first loop of `run_cendtack.sh`): every supported `den` (12,
   8, 6, 4, 3 and 2) in the `idle` mode; `map` at 12, 8, 6 and 3; `short` at 12 and 3; `long` at 8, 12 and 4; `burst`
   at 8 and 2; `sat` at 12. Each must give `RESULT PASS`;
2. for six of them, `map` at 8 and 12, `short` at 12, `long` at 8 and 12 and `burst` at 8 (second loop), the two
   conditions of 4.5 where the memory looks busy while the processor is not waiting. With `BUSYBACK=1` the run must
   pass, be identical clock by clock to the same run without it (the same hash) and keep the same distance from the
   nominal clock, because nothing is counted after DTACK and DTACK is never withdrawn (4.5, 4.9); its pulses with
   `bus_busy` high after DTACK must be more than zero. With `MAPLAT=0` the run must pass: the busy clock of the strobes
   is not part of Lb (4.4) and its pulse is not lost (4.5); over the six runs at least one pulse must fall in that
   clock with `bus_busy` high.

The last line is `VERDICT: PASS` or `FAIL`; `STATUS.md` records the result of the latest run.

### 6.2 The processor side of the handshake

`make -f Makefile.si busfsm` (`sim/tb_m68k_bus_fsm.cpp`) tests the bus FSM alone, with expected values from
UM 5.1.1–5.1.3 (header of the bench). It runs in two-phase mode, `cen` and `cen2` on alternate clocks, and in
single-phase mode (`tick` and the `mode` loop of `main`): word and byte reads and writes with DTACK visible from
half-period 1 to 10, reads and writes terminated by BERR, TAS with delays on both halves, and the VPA cycle
synchronised to E (the `cycle`, `rmw_cycle` and `vpa_cycle` calls of `main`).

### 6.3 The processor with its generator

`make -f Makefile.si cpudt` (`sim/tb_m68k_cpu_dt.cpp`, also part of `make -f Makefile.si all`) instantiates
`m68k_cpu_dt_si`, the processor with this generator as section 5 connects them, and plays the system around it: a
byte memory that answers on `bus_rdata`, a memory map that raises `bus_cs` and `bus_busy` for a programmed number of
clocks on the accesses to one region, an interrupt device, and counters of the generator's pulses. With `num` = 1 and
every supported `den` (12, 8, 6, 4, 3 and 2; the whole run repeats for each) it checks:

- reset and phases: the first pulse after the release of `reset` is `cpu_cen`, ⌊den/2⌋ + 2 clocks after the release (the
  registered reset of section 5 plus the first edge k with 2k > den, section 4.8); never two pulses in one clock; `cpu_cen` and `cpu_cenb`
  strictly alternate (section 4.3);
- DTACK (R1): for every access of the program, `DTACKn` low in clock a+2+Lb (section 4.4), and one sample of the
  processor per wait state plus the final one;
- wait states: eight accesses to the busy region, reads and writes, with Lb = 0, 4, 10 and 20; each takes the number
  of wait states given by the formulas of section 4.4 (at den 3 the pulses alternate 2 and 1 clocks apart, the formulas
  assume a uniform spacing and that count is not applied);
- lost and returned pulses (R2, R3): the bench counts the lost pulses with the rule of section 4.5 (more than zero)
  and requires, after the stalls and again at the end of the run, the pulses emitted to equal the nominal pulses of
  the divider (R0) plus the lost ones; at den 2 no pulse can be returned (section 4.9) and the pulses emitted equal
  the nominal ones;
- interrupt acknowledge: a level-4 interrupt answered with VPA (autovector, vector 28) and one answered with DTACK
  and the vector number 0x40 on `bus_rdata`; each handler writes its marker, and the vectored one shows that the
  number read in the acknowledge reaches the processor through `rd_q` (section 5);
- pause: while `pause_n` is low no `cen_o` or `cenb_o` pulse reaches the processor;
- the results that the program writes to memory.

The Verilator check of section 5 that compares `cen_o` and `cenb_o` with `cpu_cen & halt_q` and `cpu_cenb & halt_q`
runs at every clock of this bench.
