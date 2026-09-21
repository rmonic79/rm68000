# MC68000 microcode: structure

## 1. Purpose and sources

This document describes the two-level control store of the MC68000 as published,
and how rm68000 implements it: the control pipeline, the microword formats and the
next-address logic, the nano ROM, conditional branches, instruction dispatch,
exception entry, ALU control, the execution unit that the nanowords drive, and bus
access per microcycle.

Sources, described in [microcode-sources.md](microcode-sources.md):

- **[P col. N]**, **[P FIG. N]**: US patent 4,325,121, text column N or figure N.
  Appendices by letter: Appendix C is in col. 51–52; Appendix H is on scanned pages
  85–175.
- **[G]**: the die listing `all.txt` of Olivier Galibert's die analysis (not in this repository, see
  microcode-sources.md section 2.3).
- the die micro ROM bits, as decoded in `docs/data/mc68000_micro_decoded.csv`.

RTL references are to files in `rtl/cpu_si/` unless stated otherwise. The datapath
blocks are described in detail in [silicon-datapath.md](silicon-datapath.md).

## 2. Control pipeline [P col. 7–14, FIG. 4/5]

As published:

- Instruction queue: `IRC` (the next word, from the data bus), then `IR`
  (instruction being decoded), then `IRD` (instruction being executed). `IR` feeds
  the A1 decoder (58), the A2/A3 decoder (60) and the illegal-instruction decoder
  (62). `IRD` feeds branch control, ALU and condition-code control, register control
  and the field translation unit.
- Address selector (64). Multiplexer 112 chooses between A1 and the exception
  address A0, under control of A0 SUB; A0 is used for "special conditions of the
  type which await the completion of the execution of the current macroinstruction".
  Multiplexer 114 chooses among A1/A0, A2, A3 and the next address from the
  microword. Multiplexer 117 supplies two of the ten bits, which in a conditional
  branch come from branch control 80. Multiplexer 118 joins them to the eight bits
  from multiplexer 114, and selects the second exception address A0S instead when
  A0 SUBI signals "an address error, bus error, or reset condition". The 10-bit
  output addresses **both** ROMs.
- The exception logic (66) holds latches for the special conditions and "a decoder
  which generates the A0 and A0S starting addresses". "The first group of latches
  monitors the special conditions which do not await an instruction boundary before
  diverting control in the micro ROM. The second group of latches monitors the
  remainder of the special conditions." [P col. 17–18]
- Micro ROM 72 feeds output latch 74; nano ROM 73 feeds output latch 88. "The
  signals conducted by conductors 120, 122, and 124, and the address conducted by
  line 116 are all derived from the microword addressed during the previous micro
  cycle" [P col. 11–12]: the address used in cycle N+1 comes from the microword
  latched in cycle N.

In the core:

| element | implementation |
|---|---|
| `IRC`/`IR`/`IRD` | `m68k_ir_si.sv`: `ld_irc`, `ld_ir`, `ld_ird` (load conditions) and the `irc`/`ir`/`ird` registers: `IRC` loads from the data bus on the nano atom `irc<-edb`, `IR` loads from `IRC` on the microword's I bit (or bit 4 in format I), `IRD` loads from `IR` on the nano atom `ird<-(ir)` |
| A1/A2/A3 and illegal decoder | `m68k_a123_si.sv` (section 7); outputs registered in `m68k_cpu_si.sv` (`a1`, `a2`, `a3`, `illegal`, `priv_ir`) |
| address selector | the `base`/`uaddr` block of `m68k_useq.sv`: format II → `{NMA[9:8], C1C0, NMA[5:0]}` (branch `fmt`); TY 0/1/2/3 → NMA, A1 or A0, A2, A3 (`case (ty)`); immediate override `a0s` (multiplexer 118, `uaddr`) |
| exception block (A0, A0S) | `m68k_exc_si.sv` (section 8) |
| micro ROM + latch 74 | `m68k_urom.sv` (`rom`, read into `q`): 1024 × 17, synchronous read at `en`, held until the next `en` |
| nano ROM + latch 88 | `m68k_nrom.sv` (section 5) |
| control store top | `m68k_ustore.sv`; reset forces address 0 (`rstp1`) through `addr_eff` and `en_eff` |
| nanoword decoder | `m68k_nano_dec.sv`, instance `u_dec` inside `m68k_nstore_dec.sv` |

The core advances the control store once per microcycle, on the signal `en`
(`en` in `m68k_busctl_si.sv`, section 11). The micro ROM and the nano ROM are read with
the address selected for the next microcycle, so the microword and nanoword of the
current microcycle stay valid until `en`, as latches 74 and 88 do.

## 3. Microword format [P col. 15–18, FIG. 8]

17 bits, two formats:

| bits | format I (bit 1 = 0, direct branch) | format II (bit 1 = 1, conditional branch) |
|---|---|---|
| 0 | I: load `IR` from `IRC` | same |
| 1 | 0 | 1 |
| 2–3 | TY, next-address source: 00 NMA, 01 A1, 10 A2, 11 A3 | CBC, bits 2–6: condition code |
| 4 | with bit 0: IR load and TVN update (table below) | part of the CBC |
| 5–14 | NMA: next address, 10 bits | bits 7–14: NMBA, 8-bit base; address bits 7 and 6 come from C1/C0 |
| 15–16 | FC: function code field | same |

Format I, bits 4 and 0 [P col. 17–18]:

| bit 4 | bit 0 | name | effect |
|---|---|---|---|
| 0 | 0 | `db` | update neither IR nor TVN |
| 0 | 1 | `dbi` | IRC to IR, update TVN |
| 1 | 0 | `dbc` | IRC to IR, don't update TVN |
| 1 | 1 | `dbl` | IRC to IR, partially update TVN |

Appendix A gives for each label the FC, NMA, TYPE (`db`, `dbi`, `dbc`, `dbl`, `bc`,
`bci`, `a1`, `a2`, `a3`), CBC, ORIGIN (the shared nanoword), row (logical row of a
branch destination) and up to four destinations.

On the die the ten NMA bits are physically permuted with respect to the patent
([microcode-sources.md](microcode-sources.md) section 3).

In the core:

- Fields: the assignments of `ir_load`, `fmt`, `ty`, `bit4`, `cbc` and `fc` in
  `m68k_useq.sv`; NMA permutation: the assignment of `nma` in `m68k_useq.sv`.
- IR load: `ld_ir = I | (bit4 & ~format)` (`m68k_ir_si.sv`).
- TVN update: `dbi` raises `upd_full` and `dbl` raises `upd_part`
  (`m68k_cpu_si.sv`). At a `dbi` word the exception block samples the
  pending interrupt for the next instruction boundary (branch `upd_full` of `m68k_exc_si.sv`: `int_l`, `inl`).
  `upd_part` has no separate effect, because the core latches the events of the patent's Group 1
  (UM group 0: reset, bus error, address error) as soon as they occur (`rst_l`, `berr_l`, `aerr_l` in `m68k_exc_si.sv`;
  comment on `unused`).
- FC: a field value other than 00 sets the function code of the access, 00 keeps
  the previous one, and 11 gives FC = 7 (CPU space). Operand accesses of type `uk`
  use the data space, or the program space for PC-relative operands
  (`fc_code_n` and `fc_r` in `m68k_busctl_si.sv`).

### Contents of the core's micro ROM

`m68k_urom.mem` holds 1024 words, 523 of them non-zero, generated by
`docs/data/scripts/gen_urom.py` from `docs/data/mc68000_micro_decoded.csv`:

- the 544 words of the die micro ROM, bit for bit. 26 of them are all zero: 25
  unused locations named `zzz..` in the die's names, and `1c8` (`rstp2`), which is
  empty in the dump;
- `1c8` repaired: format I, TY 0, I = 0, FC 0, next address 002 (`rstp3`), as
  Appendix H page 139 gives "rstp2 → rstp3" (`HOLES` in `gen_urom.py`;
  `docs/data/deviations/ustore.tsv`);
- four words that do not exist on the die (the words `illg1`, `stopw`, `stopw2` and `illg2` that `gen_urom.py`
  appends; `ustore.tsv`), at addresses where the die has no word: the dump has none between 3f0 and
  3ff, and in the die's A1/A2/A3 tables 3ff means "none" (`docs/data/mc68000_a123_die.tsv`). `3fd` is
  the STOP wait: format I, I = 1, TY = A1; `3fc` is its companion: format I, I = 1, TY 0, next 3fd. The exception
  block sends the sequencer to 3fc from the dispatch of 327 and from every dispatch of 3fd, a loop of two words with
  the period of the die's 327, 3a2, until an exception arrives (`A_STOPW` in `m68k_exc_si.sv`). `3ff` and `3fe` are an entry for instruction
  exceptions (PC − 2 → AU → PC, then `trap2` at 347). The 17 bits of `3fe` are those of the die
  word `trap1` (1d0), from which `gen_urom.py` builds both words; their two nanowords (3ff:
  `au+=-2`, `au<-db`, `db<-(pc)`; 3fe: `ftu<-(psw)`, `pc<-au`) spread over two words the
  transfers that `trap1` makes in one ("(psw) → ftu ; (pc) → db → au ; −2 → au", Appendix H
  page 119), and they do not exist on the die. They remain in the ROM but are
  not selected: the exception block enters instruction exceptions and trace at 0e3
  (`A_TRAP` and `A_TRACE` in `m68k_exc_si.sv`), the die's path through `dvur2` →
  `dvur3` → `trap3`; the A1 value 3ff of the illegal opcodes never dispatches,
  because the exception block replaces it with A0 = 0e3.
  `docs/data/mc68000_nrom_reach.tsv` lists 3fe and 3ff as used but unreachable,
  together with the die's trace words 1c0 and 3a3. Their lines in `ustore.tsv` and
  `nstore.tsv` are labelled IMPLEMENTATION CHOICE (our own word, not reached).

## 4. Next address and the ROM pipeline in the core

`m68k_useq.sv` is combinational: from the current microword and from the A1/A2/A3,
A0/A0S and C1C0 inputs it produces the next address `uaddr` for both ROMs, with no
decision beyond those of the silicon (header of `m68k_useq.sv`). The micro ROM output
feeds a free-running register `uw_q` (`m68k_ustore.sv`), and the sequencer `u_seq`
reads that register. The function-code field `fc` is taken directly from the memory
output.

In reset the address is forced to 0 (the `rst_n` mask of `base` in `m68k_useq.sv`;
`addr_eff` and `en_eff` in `m68k_ustore.sv`):
the word `rstp1` is the first one executed after reset.

## 5. Nano ROM [P col. 17–24, FIG. 9, FIG. 11A–11F]

As published:

- 336 unique nanowords of 68 bits, decoded into about 180 control points, with
  "about three gate levels of decoding" [P col. 17–18].
- Addressed by the same address as the micro ROM, many-to-one (ORIGIN), with
  concurrent access [P col. 17–18].
- A conditional branch selects a logical row of up to four words. Bits 7 and 6 of
  the address select the word, and the destinations of a branch sit in the same
  logical row of both ROMs. The patent's example is `mulm4`, whose destinations
  are `mulm6`, `mulm4`, `mulm3`, `mulm5` [P col. 23–24]. In the die micro ROM they
  are at 020, 0e0, 060, 0a0: base 020 with bits 7:6 = 00, 11, 01, 10
  (`mc68000_micro_decoded.csv`).
- Content: transfers (Appendix B abbreviations; one block per microword in
  Appendix H; the transfer part also in [G] for 512 addresses).

In the core the nano ROM keeps the **content** of the silicon's nanowords but not
their 68-bit encoding:

- **Encoding.** Each "atom" of a word (a transfer, a constant, an access, ALU or
  register-pointer cell) is a boolean action. Atoms that never occur together in
  one word share an enumerated slot of ⌈log2(n+1)⌉ bits, where 0 means no action
  of the slot (`docs/data/scripts/gen_nrom.py`, header). An atom is active when
  `nw[LSB +: W] == CODE` (header of `m68k_nano_pkg.sv`). The slots and codes are listed in
  `docs/data/mc68000_nano_encoding.tsv`. A nanoword is 77 bits (`NROM_W`,
  `m68k_nano_pkg.sv`).
- **ORIGIN.** A table of 256 entries × 7 bits in logic maps the micro address
  group `addr[9:2]` to a row. The nanoword index is `{row, addr[1:0]}`
  (`org_t` and `idx_na` in `m68k_nrom.sv`). The table is built as four 64-entry subtables, one
  6-input LUT per bit, and a 4-way multiplexer on `addr[9:8]`: exactly two LUT
  levels (`t_s0`–`t_s3` and `row_pre` in `m68k_nrom.sv`). Of the 256 entries, 124
  distinct rows are used (`m68k_norigin.mem`). Words with the same set of atoms
  share a nanoword. The 512 slots hold 373 distinct non-empty nanowords
  (`m68k_nrom.mem`, `m68k_nrom_m.mem`), against 306 distinct contents on the die
  ([microcode-sources.md](microcode-sources.md) section 3).
- **Content.** For 500 micro addresses the nanoword is the Appendix H block
  (`docs/data/us4325121_appH.tsv`: transfers and the access, ALU and register-pointer
  columns), placed at the die address given by `docs/data/us4325121_appH_die_map.tsv`.
  Three rules are declared (header of `sim/m68k_apph_model.h`). R1: a `*` bus without
  prefix is AB or DB, decided per word from the die listing. R2: the AU reads the
  whole bus. R3: an AU load without an explicit constant adds 0. On top of that,
  `docs/data/deviations/nstore.tsv` holds 100 corrections on 78 addresses, each with
  its source. They are grouped as follows in the file:
  - 24 indexed-mode words, where the index register `rz` comes from `IRC[15:12]`
    ([G] `dbh = reg[irc >> 12]h`);
  - 8 exception-frame words whose stack accesses are word accesses;
  - the access class taken from the die listing for some words;
  - the tail of the reset sequence (vector reads in 2fb/2ff/300/11e);
  - DBcc as on the die (Appendix H's `dcnt1`–`dcnt5` differ from it);
  - STOP and MOVE SR,Dn;
  - the DIVS words that exist only on the die (`docs/data/mc68000_die_only_words.tsv`);
  - exception words, MOVEM words without an Appendix H block, and per-word corrections.

### Nano ROM in the FPGA

- The 77 bits are split in two block memories with the same index (`m68k_nrom.sv`):
  `rom_l`, 512 × 17 bits, holds the fields consumed at the start of the microcycle
  (the 50 atoms that form the read index of the register file: register sources of the buses, pointers, `rxsrc`;
  plus the narrowest slots of the other part, silicon-datapath.md §19.2); `rom_m`, 512 × 60 bits, holds the rest
  (`NROM_WL = 17`, `m68k_nano_pkg.sv`).
- Both are read **one clock before `en`**. The ORIGIN table works on the next
  address one clock early (`addr_pre`), the memory address register captures the
  index at that clock, and the output register loads the word at `en`
  (`idx_m`, `ql_pre`, `qm_pre`, `q_l` and `q_m` in `m68k_nrom.sv`). Downstream logic sees a register, never a
  memory output.
- The immediate override (reset at 000, bus/address error at 003) does not go
  through the table: the two nanowords are constants generated into
  `m68k_nano_pkg.sv` (`NROM_L_000`, `NROM_L_003`, `NROM_M_000`, `NROM_M_003`) and
  selected in the `g_ovr_const` branch of `m68k_nrom.sv`.
- The decoded vector `dec` has 199 bits (`NDEC_W`, `m68k_nano_dec_pkg.sv`). It
  is generated per resource by `docs/data/scripts/gen_nano_dec.py`: one field per
  resource, enumerated when no word uses two values of it at once, one-hot
  otherwise (header of `m68k_nano_dec_pkg.sv`). In `m68k_nstore_dec.sv` part of `dec`
  is decoded from the word read one clock early and registered at `en`, the rest is
  a registered copy of the decode of the current word (`dec_pq`, `dec_q` and the mask
  `DEC_RAW` in `m68k_nstore_dec.sv`). The value is the decode of the current nanoword in both cases.

## 6. Conditional branch [P col. 23–26, FIG. 13/14, Appendices C/D/E]

As published:

- The CBC (microword bits 6–2) goes to a decoder (162) that decides whether the
  condition is implicit in the microcode or explicit in the macroinstruction:
  the condition field of Bcc, Scc and DBcc (Appendix E: the 16 standard 68000
  conditions). Multiplexer 170 selects the condition signals and address-bit
  generator 178 produces C1/C0. "The address bit generator selects one of two
  possible output combinations for C0 and C1 associated with the conditional
  signals selected by multiplexer 170", under control of CBC bit 6.
- Appendix C lists the codes: variable, source, values, C1 C0. The variables are
  `i11` (IRC bit 11), `auz` (AU = 0), `c`, `z`, `n`, `v` (PSW), `nz1`/`nz2` (n and
  z), `ms0` and `m01` (multiply: IRD bit 8, ALUE bits and AU = 0), `ze` (local copy
  of ALU = 0), `nv` (CHK), `d4` (bit 4 of the bit-number decoder), `enl` (MOVEM:
  end of list and IRD bit 6), `cc` (Bcc/Scc: false/true).
- "Conditional branches present a problem because the accesses to the micro and
  nano control store for the next control word are overlapped with execution of the
  current control word. To compensate for the time delay, a cycle is allowed in
  programming the microroutines." [P col. 23–24]

In the core:

- In `m68k_branch_si.sv`, `cond` evaluates the 16 conditions of `IRD[11:8]`
  (Appendix E) and `val` builds the value of the variable selected by `CBC[3:0]`.
  `C1C0` is looked up in the 512-entry table code × value, `SI_BRANCH_TABLE`,
  generated from Appendix C into `m68k_branch_pkg.sv`.
- The die uses 21 CBC codes over 15 variables
  ([microcode-sources.md](microcode-sources.md) section 3). `gen_branch.py`
  cross-checks the table against the 70 conditional branches of the die listing:
  the targets appear in the order of the variable's values, and bits 7:6 of the
  target are C1 C0.
- Two compositions depart from the literal Appendix C, each with its source in
  `docs/data/deviations/branch.tsv`. `nv` is `{n | z, v}`, so that CHK does not trap
  when Dn equals the bound (case `nv` of `val` in `m68k_branch_si.sv`). `enl` is
  `{IRD[6], END}` (case `enl` of `val`).
- AU = 0 is tested on the six low bits of the AU (input `auz` of `u_br` in `m68k_cpu_si.sv`). The
  RESET instruction's loop counts the AU down from −2, by 2 at a time: with six bits it
  ends after 31 iterations, which gives the 132 clocks of MC68000UM Table 8-12
  ([instruction-timing.md](instruction-timing.md) section 5). The branch unit is
  instantiated as `u_br` in `m68k_cpu_si.sv`, and its output is registered every clock
  (`c1c0`).
- The branch at the end of a microword therefore tests values written at the end
  of the previous microword: the condition sources are registers updated at `en`,
  and `C1C0` is registered. This is the one-cycle allowance of the patent.

## 7. Instruction dispatch: A1, A2, A3 and illegal [P FIG. 5, FIG. 21]

A1 is the first microword of an instruction; A2 is used after an immediate operand
and A3 after the effective-address calculation. In the patent each is a PLA on `IR`,
together with the illegal-instruction decoder.

The A1 PLA of the die has 149 terms, each an opcode mask with its A1 address
(`a1-decode.txt`, not in this repository, see microcode-sources.md section 2.3; the same 149 entries decoded in
`a1-dump.txt`): 129 opcode terms and
20 terms on the effective-address field alone, ten for long and ten for byte/word
operations. `die_a123.py` evaluates them, with the A2/A3 PLA, over all 65,536 opcodes;
the result, `docs/data/mc68000_a123_die.tsv`, is the source of the core's decoder.

In the core, `m68k_a123_si.sv` (generated by `gen_a123.py`) is a two-level decode
like the die's PLA (header of `m68k_a123_si.sv`). Eight of the die's terms, the ones
outside the row/column model, are exact opcode entries that give A1 directly: BSR.W,
Bcc.W, RESET, NOP, STOP, RTE/RTR, RTS and TRAPV (`op_ent` in `gen_a123.py`; `m`, `hit`
in `m68k_a123_si.sv`). Every other A1, and all A2 and A3, come from one table indexed
by a row class (`IR[15:6]`, 100 classes; `cls`) and a column class (EA mode, with the
register for mode 7, 13 classes; `colc`). The table writes 1,298 of its 1,300 cells;
the other two, TRAP #n, take its default, A1 = A3 = TRAP1. A2 is a don't-care in the
cells where no opcode reads it (`docs/data/mc68000_a123_need.tsv`, from `a123_need.py`:
1,556 of the 65,536 opcodes read A2), so it has a value in 119 cells. The row class
of the next `IR` is computed from `IRC` and registered together with `IR` (`u_cls` and
`cls_q` in `m68k_ir_si.sv`). The outputs `a1`, `a2`, `a3`, `illegal` and `priv` are registered
(`m68k_cpu_si.sv`). The bench `a123_si` checks all 65,536 opcodes against the
die PLA table `docs/data/mc68000_a123_die.tsv`. FIG. 21 predates the final opcode
map, so for A1 the die decides (`gen_a123.py` header).

A dispatch is a format-I word with TY = A1 (`dispatch`, `m68k_cpu_si.sv`). There the
exception block can replace A1 with A0.

## 8. Exception entry: A0 and A0S

In the core (`m68k_exc_si.sv`):

- Entry addresses: reset 000, bus and address error 003, interrupt 1c4,
  instruction exceptions and trace 0e3, STOP wait 3fc (`A_RESET`, `A_BSER`, `A_INT`, `A_TRAP`,
  `A_TRACE`, `A_STOPW` in `m68k_exc_si.sv`).
- Group 1 of the patent's latches (UM group 0; immediate, override A0S): reset, bus error, address error
  (`a0s_sub` and `a0s` in `m68k_exc_si.sv`). The override is applied after the branch logic,
  as multiplexer 118 does (`uaddr` in `m68k_useq.sv`).
- Group 2 (UM group 1 and the STOP wait; at the instruction boundary, A0 in place of A1 at a dispatch): trace,
  interrupt, illegal / privileged / line A / line F, and the STOP wait
  (the `a0_sub`/`a0` block of `m68k_exc_si.sv`).
- Vector numbers come from the exception taken or from the instruction in `IRD`:
  TRAP #n, TRAPV, CHK, divide by zero (`f_tvn` and `tvn_r` in `m68k_exc_si.sv`).

The exception model is described in [exceptions.md](exceptions.md).

## 9. ALU and condition codes [P col. 26–32, FIG. 15–20]

As published:

- ALU and condition-code control block 243 "is arranged as a matrix of 15 rows and 5
  columns". Row decoder 241 enables one of the 15 row lines from `IRD`: FIG. 18
  lists the bit patterns, each with the rows it inhibits, and FIG. 19A/19B is the
  PLA. The column comes from three nano bits (NIF0–NIF2). When none is set, no
  column line is enabled, the sixth line (ALURL) is driven, and the ALU's register is
  not updated.
- Each cell of FIG. 17 holds an ALU operation and a five-character condition-code
  code in the order X N Z V C. Column 1 of rows 2–5 and 8–11 has two entries:
  "the first entry is selected when only the initiate bit is set", the second with
  the finish bit (nano field NCC0/NCC1). In columns 2–4 the same code applies to
  initiate and finish.
- Line 336 (INHCC) inhibits the condition codes for macroinstructions that do not
  affect them.

In the core (details in [silicon-datapath.md](silicon-datapath.md) and, for
multiply and divide, [mul-div.md](mul-div.md)):

- FIG. 17 is transcribed in `docs/data/mc68000_alu_matrix_fig17.tsv`, and
  `gen_alu.py` turns it into `SI_ALU_TABLE` of `m68k_alu_pkg.sv` (16 rows × 5
  columns, row 0 = no row) and the direct-index form `SI_ALU_T2`. The departures
  from the literal sheet are in `docs/data/deviations/alu.tsv`.
- The row decoder `m68k_alu_rowdec.sv` is an exhaustive table on `IRD[15:3]`,
  equivalent to the FIG. 18 terms (header of `m68k_alu_rowdec.sv`). The core evaluates it on
  `IR` and registers the result when `IRD` loads (`g_row_reg` in `m68k_alux_si.sv`). It
  also outputs `cc_inh`, which suppresses the condition-code update for An
  destinations (`init_q` and `fin_q` gated by `cc_inh` in `m68k_alux_si.sv`).
- The cell is looked up from (row, column) (`e_c` and `op1h_c` in `m68k_alux_si.sv`). Column 6 is
  column 4 with a 1 shifted in (`colx` and `in_l` in `m68k_alu_si.sv`). The two column-1
  entries are chosen by initiate/finish (`tk` in `m68k_alu_si.sv`). With no column, the
  ALU register keeps its value (`alu_r` loads only on `r_valid`, `m68k_alux_si.sv`).

## 10. Execution unit [P col. 5–8, FIG. 3]

As published, "a segmented two-bus structure divided into three sections by
bidirectional bus couplers":

- **HIGH** section: high words of the address and data registers, with a 16-bit
  arithmetic unit. **LOW** section: low words of the address registers, with a
  16-bit arithmetic unit. **DATA** section: low words of the data registers, with
  the ALU and special units (priority encoder for MOVEM, decoder for bit
  manipulation).
- Two buses (address and data) run through each section, joined across sections by
  the couplers. Temporaries: AT and DT (32 bits), the address output buffer, the
  data bus input and output buffers, PC, the two stack pointers, the field
  translation unit, PSW and SSW (Appendix B).
- [G] renders the transfers with a small vocabulary, for example `au = dbh/l + 2`,
  `aobh = dbh`, `dbin = ebd`, `abd = dbin`, `irc = ebd`, `ir = irc`, `ird = ir`,
  `pch = dbh`, `dth = abh`, `abh <-> abl`, `abh = sign(abl)`,
  `dbh = reg[irc >> 12]h`, `bus access mode instruction word read`, `jump a1`,
  `jumpcc <condition> <targets>`.

In the core: bus sections and couplers `m68k_bus_sec.sv`; AT, AOB, DOB, DBIN
`m68k_regs_bd.sv`; address unit `m68k_au.sv`; register file and MOVEM encoder
`m68k_regfile_si.sv`, `m68k_mvm_si.sv`; ALU and its registers `m68k_alu_si.sv`,
`m68k_alux_si.sv`; field translation unit `m68k_ftu_si.sv`; PSW `m68k_psw_si.sv`.
All are driven by fields of `dec`. [silicon-datapath.md](silicon-datapath.md)
describes them.

## 11. Bus access per microcycle [P col. 19–22]

As published, each microword block carries an access label of four characters:

- class: `i` initiate ("has begun an external access operation during the current
  microcycle but ... need not wait for the external access operation to be
  completed"), `f` finish ("an access was initiated on a previous microcycle and ...
  must be completed during the current microcycle"), `n` no access, `t` total
  ("must both initiate and finish an access ... during the current microcycle").
  "This circuitry inhibits the data processor from proceeding to the next
  microcycle for the finish and total access classes until the access operation has
  been successfully completed."
- mode: `p` process only, `r` read, `w` write;
- type: `ak` interrupt acknowledge, `im` immediate, `in` instruction, `ix`
  immediate or instruction, `op` operand, `uk` unknown.

In the core (`m68k_busctl_si.sv`):

- One microcycle is 2 clock periods of the 68000, that is four phases: ph0 = φ1,
  ph1 = φ2, ph2 = φ1, ph3 = φ2. The control store advances on ph3 unless stalled
  (header of `m68k_busctl_si.sv`; `en`). A bus cycle S0–S7 lasts two microcycles
  without wait states.
- An `i` or `t` word requests the bus cycle in ph0 (`req`). An `f` or `t` word
  holds the microcycle on ph3 until S7 of the cycle (`wait_done`,
  `stall`), so wait states lengthen exactly those words. An `i` or `t`
  word whose previous access is still running waits for the bus in ph0 (`wait_free`).
- The bus state machine `m68k_bus_fsm.sv` generates S0–S7 with φ1/φ2 clock enables
  (header of `m68k_bus_fsm.sv`), including the M6800 cycle synchronized with E
  (the VPA termination in state S5, with `vma_n` and `e_cnt`). [bus-interface.md](bus-interface.md) describes it.

## 12. Verification

Block benches (`sim/Makefile.si`, `make -f Makefile.si test`; coverage in
`sim/README.md`):

| target | bench | what it checks against the data |
|---|---|---|
| `ustore` | `sim/tb_m68k_ustore.cpp` | micro ROM contents and sequencer on all 1024 addresses: direct, dispatch (against all A1 and A0 values), conditional (all four C1C0), empty; model from `mc68000_micro_decoded.csv` + `deviations/ustore.tsv` |
| `nstore` | `sim/tb_m68k_nstore.cpp` | nano ROM structure on all 1024 addresses: empty where undefined, equal content ⇒ equal word, word read one clock early, override at 000/003 |
| `ndec` | `sim/tb_m68k_ndec.cpp` | nanoword decoded atom by atom against Appendix H placed by the die map, rules R1–R3 and `deviations/nstore.tsv` |
| `a123_si` | `sim/tb_m68k_a123_si.cpp` | A1/A2/A3, illegal and privileged on all 65,536 opcodes against the die PLAs |
| `branch_si` | `sim/tb_m68k_branch_si.cpp` | every code × value of the branch table against Appendix C |
| `rowdec`, `alu_si`, `alux_si` | `sim/tb_m68k_rowdec.cpp`, `sim/tb_m68k_alu_si.cpp`, `sim/tb_m68k_alux_si.cpp` | ALU row decode (FIG. 18), ALU cells (FIG. 17), ALU with its registers driven by the real control store |
| `bussec`, `regs`, `regfile_si` | `sim/tb_m68k_bus_sec.cpp`, `sim/tb_m68k_regs.cpp`, `sim/tb_m68k_regfile_si.cpp` | bus sections, bus-side registers and register file for every microcode word |
| `cpu_si` | `sim/tb_m68k_cpu_si.cpp` | the whole CPU on a self-checking program, every bus cycle checked |

The instruction benches (`make -f Makefile.si all`) exercise the microcode
instruction by instruction. During development the bus accesses were also compared
with an external reference core, used as a black box, in lockstep and on generated
cases per instruction family, with comparison benches that are not part of this
repository (available from the author on request).
