# Silicon-structured datapath

## 1. Purpose and scope

rm68000 implements the MC68000 the way the chip is built: an execution unit made of three 16-bit sections joined by
two internal buses, driven by a two-level control store (a micro ROM that sequences and a nano ROM that drives the
datapath). This document describes that execution unit and its control store as the RTL implements them today: for
every block, the rule it follows, the RTL that implements it (file and signal, block or function name, paths relative
to `rtl/cpu_si/`
unless written otherwise) and the source of the rule.

Not covered here, and documented elsewhere:

- the external bus protocol and the bus state machine: [bus-interface.md](bus-interface.md),
  [bus-control-signals.md](bus-control-signals.md), [m6800-bus.md](m6800-bus.md);
- the clock-enable and DTACK generator around the core: [clock-dtack.md](clock-dtack.md);
- exception, interrupt and IACK behaviour seen from outside: [exceptions.md](exceptions.md),
  [interrupts.md](interrupts.md), [interrupt-acknowledge.md](interrupt-acknowledge.md),
  [function-codes.md](function-codes.md);
- multiplication and division: [mul-div.md](mul-div.md);
- where the microcode data come from and how the patent describes the control store:
  [microcode-sources.md](microcode-sources.md), [microcode-structure.md](microcode-structure.md);
- which parts of the core are data taken from the chip: [chip-data-sources.md](chip-data-sources.md);
- the programmer-visible registers: [register-model.md](register-model.md).

## 2. Sources

### 2.1 Published data

| source | what it gives | where it is in the repository |
|---|---|---|
| US 4,325,121 (Motorola, "Two-level control store for microprogrammed data processor"), App. H | the operations of every microword, in symbolic form (transfers, ACCESS, ALU FUNCTION and REGISTER POINTERS cells) | `docs/data/us4325121_appH.tsv` |
| US 4,325,121, App. A + die map | which App. H block sits at which micro address of the die | `docs/data/us4325121_appA.csv`, `docs/data/us4325121_appH_die_map.tsv` |
| US 4,325,121, App. C | conditional-branch codes (CBC) and their C1/C0 outputs | `docs/data/us4325121_appC.tsv` |
| US 4,325,121, FIG. 17 | ALU matrix: operation and condition-code codes per row and column | `docs/data/mc68000_alu_matrix_fig17.tsv` |
| US 4,325,121, FIG. 18/19 | ALU row decode from the instruction word | `docs/data/mc68000_alu_rows_fig18.tsv`, `docs/data/mc68000_alu_rows_fig19_check.tsv` |
| US 4,325,121, FIG. 21 | A1/A2/A3 entry points per instruction class | `docs/data/us4325121_fig21_a123.tsv` |
| die analysis by Olivier Galibert | micro ROM bits, A1/A2/A3 PLA table, symbolic listing of the microcode (`all.txt`) | `docs/data/mc68000_micro_decoded.csv`, `docs/data/mc68000_a123_die.tsv`, `docs/data/mc68000_die_only_words.tsv`; `all.txt` is read by the generators and is not redistributed |
| MC68000 User's Manual, Rev. 8 (MC68000UM, "UM" below) | bus cycles, exception processing, vectors, programmer's model | cited by section |
| M68000 Programmer's Reference Manual (M68000PRM, "PRM" below) | instruction semantics and encodings | cited by section |

The reading of the patent text (pipeline, microword formats, ALU cells, access classes) is in
[microcode-structure.md](microcode-structure.md); the provenance of every data file is in
[microcode-sources.md](microcode-sources.md).

### 2.2 How the sources are combined

- The **sequence** of microwords is the die's: the micro ROM holds the 17 raw bits of the die dump (§19.1).
- The **content** of each microword (what the datapath does in it) is the App. H block placed at the die address by
  the die map, with the corrections listed in `docs/data/deviations/nstore.tsv`; the generator is
  `docs/data/scripts/gen_nrom.py`.
- The **encoding** of the nano ROM is ours: one field per group of mutually exclusive actions, not the 68 raw bits
  of the silicon (§19.2).

### 2.3 Deviations

Where the core departs from the literal published data, the departure is one line of a file under
`docs/data/deviations/` with its source: `bussec.tsv` (bus sections, §5), `rowdec.tsv` (ALU row decode, §9.1),
`alu.tsv` (ALU matrix, §9.2), `branch.tsv` (branch variables, §11), `a123.tsv` (A1/A2/A3, §15), `ustore.tsv` and
`nstore.tsv` (micro and nano ROM content, §19). The block benches read these files at run time (§20).

## 3. Organisation

### 3.1 Sections and buses

The execution unit has three 16-bit sections (US 4,325,121 FIG. 3, see [microcode-structure.md](microcode-structure.md)):

| section | holds | bus sections |
|---|---|---|
| HIGH | high words of all registers, high half of the AU | `ab_h`, `db_h` |
| LOW | low words of address registers, PC, AT, SP; low half of the AU | `ab_l`, `db_l` |
| DATA | low words of data registers; ALU with ALUB, ALUE, DCR, correction factor; DBIN/DOB | `ab_d`, `db_d` |

Each of the two internal buses, AB (address bus) and DB (data bus), is therefore three 16-bit sections, each with
a "driven" flag (`ab_h_drv` ... `db_d_drv`, `m68k_bus_sec.sv`). The bidirectional couplers between sections are not
modelled as wires:
a reader that needs a section which nobody drives in that word takes a neighbouring section instead, in a fixed
order given for each reader in §6, §8.6 and §9.5.

### 3.2 Block map

| silicon element | RTL | section |
|---|---|---|
| micro ROM 544 × 17 (in 1024 addresses) | `m68k_urom.sv`, image `m68k_urom.mem` | §19.1 |
| next-address selector (NMA, A1/A2/A3, A0/A0S, C1/C0) | `m68k_useq.sv` | §19.1 |
| nano ROM (our encoding) and ORIGIN map | `m68k_nrom.sv`, images `m68k_nrom.mem`, `m68k_nrom_m.mem`, `m68k_norigin.mem` | §19.2-§19.4 |
| nano decoder (nanoword → resource vector `dec`) | `m68k_nano_dec.sv`, `m68k_nano_dec_pkg.sv` | §4, §19.5 |
| control store wrappers | `m68k_ustore.sv`, `m68k_nstore_dec.sv` | §19 |
| bus sections AB/DB | `m68k_bus_sec.sv` | §5 |
| AT, AOB, DOB, DBIN | `m68k_regs_bd.sv` | §6 |
| address unit AU | `m68k_au.sv` | §7 |
| register file D0-D7, A0-A6, USP, SSP, PC, DT | `m68k_regfile_si.sv` | §8 |
| ALU, ALUB, ALUE, DCR, correction factor, LOCZ | `m68k_alux_si.sv`, `m68k_alu_si.sv`, `m68k_alu_rowdec.sv`, `m68k_alu_pkg.sv` | §9 |
| status register PSW | `m68k_psw_si.sv` | §10 |
| conditional branch unit | `m68k_branch_si.sv`, `m68k_branch_pkg.sv` | §11 |
| field translation unit FTU | `m68k_ftu_si.sv` | §12 |
| MOVEM priority encoder | `m68k_mvm_si.sv` | §13 |
| instruction pipeline IRC/IR/IRD | `m68k_ir_si.sv` | §14 |
| A1/A2/A3 decoder | `m68k_a123_si.sv` | §15 |
| exception block (A0, A0S, vector number) | `m68k_exc_si.sv` | §16 |
| microcycle sequencer and bus cycle | `m68k_busctl_si.sv`, `m68k_bus_fsm.sv` | §17 |
| operand size | `m68k_size_si.sv` | §18 |
| all of the above, connected | `m68k_cpu_si.sv` | |

### 3.3 The microcycle

One microcycle is two clock periods of the 68000, that is four phases: ph0 and ph2 advance on `cen` (phi1), ph1
and ph3 on `cen2` (phi2) (`phase_ev`, `ph`, `m68k_busctl_si.sv`). The control store advances (`en`) on ph3 unless the word
is waiting for its bus cycle (`stall`, `en`, `m68k_busctl_si.sv`); §17 gives the waiting rules. Inside a word:

- the sources selected by the nanoword drive the bus sections from ph0;
- the AU and the ALU sample their operands at ph1 (`op_ld = (ph == 1) & cen2`) and register a first stage at ph2
  (`st_ld = (ph == 2) & cen`) (instances `u_au` and `u_alux` in `m68k_cpu_si.sv`; `a_q`, `b_q`, `c_q` and `lo_s`
  in `m68k_au.sv`; `a_q`, `d_q`, `e_q` in `m68k_alux_si.sv`; `res_s` and the other stage registers in `m68k_alu_si.sv`);
- every destination register loads at `en`, the last clock of the word.

The transfers of a nanoword happen during the microcycle: a bus cycle started by a word uses the AOB and DOB values
loaded by that same word. `m68k_regs_bd` exports the values being loaded (`aob_n`, `dob_n`)
and the bus controller takes those (`u_bc` in `m68k_cpu_si.sv`); every other reader sees
the registers, updated at `en`.

## 4. Nanoword atoms and the resource vector

### 4.1 Notation

The content of a nanoword is a set of atoms, written as in the transcription of App. H:

- `D<-S`: transfer from S to D; `(r)` means "the contents of register r", e.g. `ab<-(rx)`, `aob<-ab`;
- `D+=k`: constant k into D (`au+=+2`, `alu+=-1`);
- `cell=value`: a cell of the App. H block (`acc_class=t`, `alu_col=2`, `rp_dst=dt`);
- a few named actions (`reset pren`, `sr c-alu-alue`, `idle wait`).

Bus names carry the section: `ab`/`db` (natural sections of the source), `abh`/`abl`/`abd` (one section), `abe`
(low section plus its sign on the high section), `ab*` (at least the data section; same for `db`). App. H also
writes a bare `*` for "AB or DB": §4.3.

### 4.2 The decoded vector

`m68k_nano_dec.sv` turns the nanoword into the vector `dec`, 199 bits (`NDEC_W`, `m68k_nano_dec_pkg.sv`), with one
field per resource (the `ND_*` constants of `m68k_nano_dec_pkg.sv`): the enumerated fields hold one value code (0 =
none), the one-hot
fields (`ab<-`, `db<-`, `ab*<-`, `db*<-`, `abd<-`, `abe<-`, `dbd<-`, `dbe<-`, `alu<-`, `alub<-`, `aob<-`, `au<-`,
`rp_dst=`, `rp_src=`) one bit per value. The resources are:

| group | resources |
|---|---|
| bus sources | `ab<-`, `ab*<-`, `abd<-`, `abe<-`, `abh<-`, `abl<-`, `db<-`, `db*<-`, `dbd<-`, `dbe<-`, `dbh<-`, `dbl<-`, `*e<-` |
| bus-side registers | `aob<-`, `at<-`, `ath<-`, `atl<-`, `dob<-`, `dobh<-`, `dobl<-`, `dbin<-`, `dbinh<-`, `dbinl<-` |
| AU and ALU | `au<-`, `au+=`, `alu<-`, `alu+=`, `alub<-`, `alue<-`, `dcr<-`, `alu_col=`, `alu_cc=`, `alu_byte`, `sr c-alu-alue` |
| registers | `rx<-`, `rxa<-`, `rxdl<-`, `rxh<-`, `rxl<-`, `ry<-`, `rya<-`, `rydl<-`, `ryh<-`, `ryl<-`, `sp<-`, `sph<-`, `spl<-`, `pc<-`, `pch<-`, `pcl<-`, `rp_dst=`, `rp_src=`, `rxsrc=rz` |
| status and units | `psw<-`, `psws+=`, `ftu<-`, `irc<-`, `ird<-`, `reset pren`, `tpend+=` |
| bus access | `acc_class=`, `acc_mode=`, `acc_type=`, `acc=`, `idle wait` |

Four of them are decoded but have no consumer in the RTL: `alu_byte` (the ALU size comes from the instruction word,
`byte_op`, `unused_ab`, `m68k_alux_si.sv`), `idle wait` (a word without access already lasts four phases), `tpend+=0`
(the trace
pending latch lives in the exception block, §16) and `acc=ak` (the IACK cycle is recognised from `acc_type=ak`,
`iack_acc`, `m68k_busctl_si.sv`). `idle wait` and `tpend+=0` are decoded in `m68k_nano_dec.sv` (fields `ND_IDLE_WAIT`,
`ND_TPEND_CONST` of `m68k_nano_dec_pkg.sv`) and read nowhere else.

### 4.3 The bare `*` bus

App. H writes `*` where the bus (AB or DB) is not printed. `gen_nrom.py` resolves it statically, word by word, in
three steps: from the die listing, which names the physical bus of each transfer (step `star_res` of `gen_nrom.py`);
by exclusion, taking the bus that no other atom of the word uses (step `excl_res`); for destinations, from the bus
already resolved for the source of the same App. H transfer (step `pair_res`). Two atoms keep a bare bus:
`*e<-ftu` and `at<-*e`, in word 3a6 (`rset1`, RESET instruction; `docs/data/mc68000_nano_encoding.tsv`).
The fabric drives nothing for them (§5 only handles `ab`/`db` atoms); the one effect of `*e<-ftu` is to consume the FTU latch
(`use_bus`, `m68k_ftu_si.sv`), and `at<-*e` loads nothing because no section is driven in that word
(terms `at_h_u0_*` and `at_l_u0_*` of `m68k_regs_bd.sv`).

## 5. Bus sections (`m68k_bus_sec.sv`)

Generated by `docs/data/scripts/gen_bus_sec.py` (rule function `rule`).

**Rule.** For each source atom `X<-S` the sections driven and their values are:

| source S | no suffix | `h` / `l` / `d` | `e` | `*` |
|---|---|---|---|---|
| 32-bit, not a register: `at`, `au`, `aob` | H = S[31:16], L = S[15:0] | that section only | L = S[15:0], H = 16 copies of S[15] | — (no such atom) |
| 16-bit, not a register: `ath` (HIGH); `atl`, `ftu` (LOW); `alu`, `alue`, `dcr`, `dbin` (DATA) | its own section only | that section only | L = S, H = 16 copies of S[15] | its own section only |
| whole register: `rx`, `rxa`, `ry`, `rya`, `sp`, `pc` | H = R[31:16], L = R[15:0] | that section only | — | — |
| register half: `rxh`, `ryh`, `pch` (high half R[31:16]); `rxl`, `ryl`, `pcl`, `rxd`, `rxdl`, `ryd`, `rydl` (low half R[15:0]) | the half's own section (HIGH for `*h`, LOW for `*l`, DATA for `*d`, `*dl`) | that section only | L = half, H = 16 copies of its bit 15 | the half's own section |

A register source reaches the fabric through the read port of the register file for that bus (`src_rf_ab`,
`src_rf_db`, 32 bits, §8.5). Undriven sections read as zero: each section is an OR of AND terms
(the triples `ab_d_t0` ... `db_h_t3` of `m68k_bus_sec.sv`), and its driven flag is the OR of the selectors that can
drive it (`ab_d_drv` ... `db_h_drv`).

**Deviations** (`docs/data/deviations/bussec.tsv`, 19 lines). For the register halves the RTL does not use the
"own section" column above: a half without suffix is placed on both H and L, with `*` on H, L and D
(`rule`, `gen_bus_sec.py`); e.g. `ab<-(rxh)` drives H and L with R[31:16] (`sel_ab_h_3`, `sel_ab_l_4`,
`m68k_bus_sec.sv`). Two lines
are confirmed by sources independent of the RTL:

- `ab<-(rxh)`: App. H dvs04/dvs05/dvum3 send the high half of RX to `atl`, which reads the LOW section (§6);
- `dbe<-(rxdl)`: L = bits 5:0 of the register, H = 0 (`sel_db_l_8`, `sel_db_h_7`, `m68k_bus_sec.sv`). App. H
  srrw1/srrl1 load the shift
  count from the register into the AU, and the count is the register value modulo 64 (PRM §4, ASd/LSd/ROd/ROXd,
  register form).

The other 17 are implementation choices that no behaviour can observe: the only readers App. H gives those
transfers (ALU, ALUB, DOB, FTU, DCR, ATH) fall back across sections, and no microword drives two different sources
on the same section.

**No double drive.** No word of the microcode drives the same section of the same bus from two atoms: the
generator counts such words and stops if there is one (`conf`, `gen_bus_sec.py`). The sections are therefore
plain AND-OR trees with no priority.

**Derived outputs** (same values, fewer logic levels):

- `ab_ld` = `ab_l` if driven, else `ab_d` (`m68k_bus_sec.sv`), used by the FTU and the MOVEM encoder (`u_ftu`, `u_mvm`
  in `m68k_cpu_si.sv`);
- `ab_dob`: the value the DOB takes from AB. Every word that loads the DOB from AB has a single AB source, so the
  D → L → H choice of §6 always selects that source's value; the generator checks this word by word
  (census `DOB_AB` in `gen_bus_sec.py`; `ab_dob`, `ab_dob_drv` in `m68k_bus_sec.sv`);
- `ab_wl`, `ab_wh`, `db_wl`, `db_wh`: the write data of the register file per port and half, computed from the
  sources that the write rules of §8.6 can select in some word (register-file write-data section of
  `gen_bus_sec.py`; selectors `sel_ab_wh_*` ... `sel_db_wl_*` of `m68k_bus_sec.sv`).

The selectors come from flip-flops of `m68k_nstore_dec.sv` (inputs `g`, `gd` and `gq` of `m68k_bus_sec.sv`), in the
two classes of `dec` (§19.5):

- `g` (12 selectors: the register-file sources and ATH on AB high) and `gd` (the RAW half of the six driven flags)
  are loaded at `en` from the next nanoword (`f_groups` and `f_drv` on `nw_pre`), so they are valid from the first
  clock of the word. `f_groups` also gives the 15 read-port selectors of the register file (`g_rd` of
  `m68k_regfile_si.sv`);
- `gq` (25 bits: the selectors of the sources DBIN, ALU, ALUE, FTU, DCR, ATH and ATL, the non-RAW half of the six
  driven flags, and the write-data selectors `sel_ab_wh_7`, `sel_ab_wl_1` … `sel_ab_wl_4`, `sel_db_wl_0` and
  `sel_db_wl_2`) is loaded at every clock, with no enable, from `gq_c`: the same ORs evaluated on `dec_c`, the decode
  of the current nanoword. It therefore equals those ORs on `dec_q` and is valid one clock after the start of the
  word, like the non-RAW bits of `dec`. In simulation it is compared at every clock with the same ORs on `dec`.

## 6. Bus-side registers AT, AOB, DOB, DBIN (`m68k_regs_bd.sv`)

Generated by `docs/data/scripts/gen_regs.py` (rule function `rule`). Each part of a register loads at
`en` only if the section it reads is driven; otherwise it keeps its value (enables `at_h_en` ... `dbin_lo_en`,
`m68k_regs_bd.sv`). A chain
"X, else Y" means: the first driven section of the list.

| atom | part loaded | source |
|---|---|---|
| `at<-ab`, `at<-abe`, `at<-db`, `at<-dbe` | AT high and low | high ← X_h; low ← X_l, else X_d |
| `ath<-ab`, `ath<-db` | AT high | X_h, else X_l, else X_d |
| `atl<-ab`, `atl<-db` | AT low | X_l, else X_d |
| `atl<-abl`, `atl<-dbl` | AT low | X_l |
| `atl<-ab*`, `atl<-db*` | AT low | X_d, else X_l |
| `at<-*e` | AT high and low | high ← ab_h, else db_h; low ← ab_l, ab_d, db_l, db_d |
| `aob<-ab`, `aob<-db` | AOB high and low | high ← X_h; low ← X_l, else X_d |
| `aob<-abh`, `aob<-dbh` / `aob<-abl`, `aob<-dbl` | AOB high / low | X_h / X_l |
| `aob<-au` | AOB high and low | the AU, 32 bits |
| `dob<-ab`, `dob<-ab*` | DOB both bytes | the AB value, D, else L, else H (`ab_dob`, §5) |
| `dobh<-ab` / `dobl<-ab`, `dobl<-ab*` | DOB high byte / low byte | same value, bits 15:8 / 7:0 |
| `dob<-dbd` | DOB both bytes | db_d |
| `dob<-alu` | DOB both bytes | the ALU result |
| `dbin<-edb`, `dbinh<-edb`, `dbinl<-edb` | DBIN word / high byte / low byte | the data read from the external bus |

The ATH chain is what lets App. H `ablw1` ("(dbin) -> ab -> ath") put the high word of an absolute long address,
read into DBIN (DATA section), into ATH. When AOB loads from DB in a word where `db<-au` is the DB source, the RTL
takes the AU directly (`aob_h_u2_0`, `aob_l_u3_0`, `aob_l_u4_0`, `m68k_regs_bd.sv`): the value is the same, because
`db<-au` drives DB with
the AU (§5).

Where the generator proves that the first section of a chain is driven in every word that uses the atom, the RTL
loads without testing the flag (comment `always driven` in the generated code; `_census` and `emit_part` in
`gen_regs.py`); the loaded value is the same.

## 7. Address unit AU (`m68k_au.sv`)

Source of the rule: the die listing, where the AU appears only in the forms `au = dbh/l`, `au = dbh/l ± N` and
`au = dbh/l + abh/l` (Galibert `all.txt`); App. H gives the atoms `au<-db`, `au<-ab`, `au+=k`.

- Operand A is always DB: `{db_h, db_l}`, or the sign extension of `db_l` with `au<-dbe` (`a`, `m68k_au.sv`). If no
  DB section is driven, operand A is the AU itself, so the AU keeps or increments its own value (word `rstp6`)
  (`a`).
- Operand B is AB with `au<-ab` (`{ab_h, ab_l}`) or `au<-abe` (sign extension of `ab_l`), otherwise 0
  (`b`, `m68k_au.sv`).
- Constant C from `au+=`: +1, +2, +4, −1, −2, −4, 0, and the two size-dependent forms `+1,+2` and `−1,−2`
  (`ND_AU_CONST_*`, `m68k_nano_dec_pkg.sv`): ±1 when the instruction is byte-sized and the register on DB is not A7, ±2
  otherwise (`is_byte`, `au_sp`, `c`, `m68k_au.sv`). This is the address increment of `(An)+` and `-(An)`, where a
  byte access through
  the stack pointer moves by 2 to keep it word-aligned (UM §2.2, Table 2-1, note on N). The "register on DB is A7"
  flag comes from the register file (`rf_db_sp`, `m68k_regfile_si.sv`; input `au_sp` of `u_au`, `m68k_cpu_si.sv`).
- The AU register loads A + B + C at `en` in every word with an `au<-` or `au+=` atom and holds otherwise
  (`any_src`, `au`, `m68k_au.sv`). The sum is split in two stages: the low 17-bit sum is registered at ph2, the high half
  adds its carry at `en` (`lo_s`, `hi_s`, `m68k_au.sv`).

Where App. H and the die listing disagree on an AU atom (operand or constant), the generator takes the die listing
(`au_fix`, `gen_nrom.py`).

## 8. Register file (`m68k_regfile_si.sv`)

### 8.1 Registers and storage

19 registers of 32 bits: D0-D7 (index 0-7), A0-A6 (8-14), USP (15), SSP (31), PC (17), DT (18, data temporary).
The active stack pointer is SSP when the S bit is 1 and USP when it is 0: index `{S, 1111}`
(`sp_r`, `sp_w`, `m68k_regfile_si.sv`); the S used is the effective one of the word (§10). Storage is two copies of a
32-entry array, one per read port, each split in three columns of 16, 8 and 8 bits so that a byte can be written
alone (`ra_h`, `ra_m`, `ra_l`, `rb_h`, `rb_m`, `rb_l`, `m68k_regfile_si.sv`). The PC is a separate register, `pc_sh`.
The arrays have no reset.

### 8.2 Register selection

The App. H names map to physical registers as follows (sources: App. B and the REGISTER POINTERS key of the patent,
see [microcode-structure.md](microcode-structure.md); register fields and types from the instruction formats, PRM
§4 and §8):

| App. H name | register |
|---|---|
| `rx`, `rxh`, `rxl` | register RX = IRD[11:9], data or address register according to the instruction (below) |
| `rxa` | address register A[RX] (A7 = active SP) |
| `rxd`, `rxdl` | data register D[RX], low word |
| `ry`, `ryh`, `ryl` | register RY = IRD[2:0], data or address register according to the instruction |
| `rya` | address register A[RY] |
| `ryd`, `rydl` | data register D[RY], low word |
| `sp`, `sph`, `spl` | active stack pointer |
| `pc`, `pch`, `pcl` | PC |

RX is an address register for MOVEA (lines 1-3 with IRD[8:6] = 001), EXG Ax,Ay, LEA (line 4 with IRD[8:6] = 111)
and ADDA/SUBA/CMPA (lines 9, B, D with IRD[7:6] = 11) (`f_rx_is_a`, `m68k_regfile_si.sv`). RY is an address register when
the effective-address mode IRD[5:3] is one of the address-register modes 001-110, except for register shifts and
DBcc, and for EXG Ax,Ay and EXG Dx,Ay (`f_ry_is_a`). For DBcc the counter register is IRD[2:0], read
and written by the microcode as RX (`f_rx_lo`, `rxd_p`). These choices depend only on the instruction:
they are computed from IR and registered when IRD loads (`rx_is_a_fl`, `ry_is_a_fl` and the other `*_fl` registers).

### 8.3 Pointers

The REGISTER POINTERS cell of a microword overrides the default selection: `rp_dst` for the X side (`dt`, `sp`,
`us`, `rp`, `rx`, `dx`, `uk`) and `rp_src` for the Y side (`dt`, `py`, `ry`, `dy`, `uk`) (`ND_RP_DST_*`,
`ND_RP_SRC_*`, `m68k_nano_dec_pkg.sv`).

- X side: `dt` selects DT, `sp` the active SP, `us` the USP, `rp` the register given by the MOVEM encoder (§13; 15
  = active SP), `rx` goes back to the default (`spv_w`, `spv_r`, `rxa_i`, `rx_i`, `rxd_i`, `m68k_regfile_si.sv`).
- Y side: `dt` selects DT; `py` selects the PC when the instruction's effective address is PC-relative (mode 111,
  register 01x), the default RY otherwise; `ry` goes back to the default (`f_pcrel`, `ry_i`, `ryd_i`).
- `dx`, `dy` and `uk` leave the pointer as it is.

A pointer takes effect in the word that sets it and persists until another pointer value replaces it or a new
instruction is loaded into IRD (`ird<-(ir)`), which clears it (`pdr_*`, `psr_*`). The App. H
word `e#w1` relies on the first property: its cell sets `rp_src=dt` and its transfer `(dbin) -> ab -> rydl` writes
the immediate into DT in the same word. The effective selections enter the read-index logic from flip-flops loaded
with the value of the next word (`e_dt_q`, `e_sp_q`, `e_us_q`, `e_rp_q`).

### 8.4 Index register `rz`

In the indexed modes the index register is named in the extension word: IRC[15] = D/A, IRC[14:12] = number (PRM
§2.2.8, brief extension word format). The die listing reads it as `reg[irc >> 12]` in the words that add the index.
The atom `rxsrc=rz`, added by the generator to the words whose App. H origin is `aixw2`, `aixw4`, `ldmx3`, `ldmx4`,
`jsrx2`, `jsrx3`, `mmxw2` or `mmxw3` (`RZ_ORIG`, `gen_nrom.py`), replaces the RX index with IRC[15:12], 15 being the
active SP (`rz_sel`, `rz_sel_on`, `rx_i`, `m68k_regfile_si.sv`). For `rxd` the index register is used only together with a
`dt`/`sp`/`us`/`rp` pointer (`rxd_i`).

### 8.5 Read ports

Two read ports, one per bus: `rf_ab` and `rf_db` are the register selected by the register source atom active on
that bus (`sel_ab_*`, `sel_db_*`, `m68k_regfile_si.sv`). A word has at most one register source per bus, so the selections
are exclusive (comment above `sel_any_ab`). The read value includes the DB write still in the queue (§8.6) and the
PC (`rf_ab_c`, `rf_db_c`), and is registered on every clock, so it is valid from the clock after
`en`; its consumers sample it at ph1 or later (`rf_ab`, `rf_db`).

### 8.6 Write ports

Two write ports, AB and DB, each with an index, high and low enables and data; the microcode never writes the same
half of the same port twice in one word (check "with two writes" in `gen_bus_sec.py`). The AB port writes the arrays
at `en`; the DB
port is queued and written on the next clock, the read ports forwarding it meanwhile (queue `q_db_*` and the array
write of `m68k_regfile_si.sv`).

Data of a register write `R<-X` (X = AB or DB):

| destination | high half | low half |
|---|---|---|
| `rx`, `rxa`, `ry`, `rya`, `sp` | X_h | X_l, else X_d (with `*`: X_d, else X_l) |
| `rxh`, `ryh`, `sph` | X_h, else X_l, else X_d | — |
| `rxl`, `ryl`, `spl` | — | X_l, else X_d |
| `rxdl`, `rydl` | — | X_d, else X_l (with suffix `e` or `l`: X_l, else X_d) |

(`wrule`, `gen_bus_sec.py`; the same chains in `wd_ab_l_v`, `wd_ab_h_v`, `wd_db_l_v`, `wd_db_h_v` of
`m68k_regfile_si.sv`, which the core replaces with the narrow wires of §5, `WD_NARROW`.) For a byte-sized instruction
a write of the low word of a data
register changes only its low byte (`ab_m`, `db_m`): byte operands occupy bits 7:0 and the rest of a
data register is not changed (UM §2.3.1). The write enables do not test the driven flags: in every word that writes
a register from a bus the section needed is driven (`WE_DEC_ONLY`, `m68k_regfile_si.sv`).

### 8.7 PC port

`pc<-au`, `pc<-ab`, `pc<-db`, `pch<-dbh`, `pcl<-ab` write `pc_sh` at `en`. In the microcode the data is always the AU
(`pc<-au`; `pc<-db` with `db<-au`; `pc<-ab` with `ab<-au`) or AT (`pc<-ab` with `ab<-(at)`, `pch<-dbh` with
`dbh<-(ath)`, `pcl<-ab` with `ab<-(atl)`), and the RTL selects between those two (`pc_at_h`, `pc_at_l`, `wd_pc_h`,
`wd_pc_l` with `PC_DEC_ONLY`).
The generic write ports never address the PC: the PC-relative modes are not alterable, so no instruction writes
through them (PRM §2.2.11-2.2.12), and the microcode reaches DT and SP through pointers. The core is built with
`PC_W17 = 0`
(`u_rf` in `m68k_cpu_si.sv`), which removes that path (`w17_h`, `w17_l`, `m68k_regfile_si.sv`).

## 9. ALU and condition codes

Sources: US 4,325,121 FIG. 16 (operations), FIG. 17 (matrix), FIG. 18/19 (row decode); the cell semantics are read
in [microcode-structure.md](microcode-structure.md). The ALU is 16 bits wide; 32-bit operations are two passes, as in
the silicon.

### 9.1 Row decoder (`m68k_alu_rowdec.sv`)

FIG. 17 has 15 rows; the row comes from the instruction word, the column from the microword. FIG. 18 lists numbered
product terms on the IRD bits; a term followed by "(k) i" lines is disabled when one of those products is true; an
active term inhibits the rows listed next to it, and the row selected is the one no active term inhibits. The
decoder follows the transcription `docs/data/mc68000_alu_rows_fig18.tsv` with the four corrections of
`docs/data/deviations/rowdec.tsv`:

- term 20 (CMPI) with bit 12 = 0, and term 39 (left shift) with bit 12 = 0: PRM §8 opcode map (CMPI = 0000 1100,
  shifts = 1110);
- a new term for DBcc (0101 cccc 11001 rrr) that selects row 2, where FIG. 17 lists DCNT, and a qualifier that
  removes DBcc from line 14 (`scc,movb`).

The RTL is a table on IRD[15:3] (IRD[2:0] do not enter) that gives the row (0 = no row) and `cc_inh`
(header of `m68k_alu_rowdec.sv`). `cc_inh` is true for destinations that are address registers (MOVEA, ADDA/SUBA,
ADDQ/SUBQ to An): the ALU computes, the condition codes are not updated (`cc_inh`, `m68k_alux_si.sv`), as the PRM
specifies for these instructions. The table was produced from the term-based decoder (`gen_alu_rowdec.py`) through
the dump `sim/dump_rowdec.cpp` and `gen_rowdec_case.py`. In the core the row is computed from IR and registered when
IRD loads (`g_row_reg`, `m68k_alux_si.sv`).

### 9.2 The FIG. 17 matrix (`m68k_alu_pkg.sv`)

Each cell holds an operation and two condition-code entries, one for "initiate" and one for "finish"
(`SI_ALU_TABLE`, `m68k_alu_pkg.sv`). The table follows `docs/data/mc68000_alu_matrix_fig17.tsv` except for the nine
cells of
`docs/data/deviations/alu.tsv`:

| row, column | core | source |
|---|---|---|
| 0, 1 and 0, 5 | AND (pass-through) / EXT | row 0 = instructions with no FIG. 18 row (LEA, PEA, JMP, MOVEM, Bcc); App. H uses column 1 as a pass and column 5 to sign-extend the 8-bit displacement of `d8(An,Xn)` (`aixw0`, `aixl0`, `ldmx1`) |
| 1, 5 (DIVx) | EXT with V from the operation and C = 0 | PRM §4 DIVS/DIVU: V set on overflow, C always cleared |
| 8, 1 and 11, 1 (ROXd) | C = X with a count of zero, X kept | PRM §4 ROXd |
| 9, 2 (SBCD/NBCD) | Z "cleared if nonzero, unchanged otherwise" | PRM §4 SBCD/NBCD |
| 11, 2 and 11, 3 (NOT) | C = 0 | PRM §4 NOT |
| 15, 5 (MOVE.B) | EXT where the figure has `-` | App. H `aixw0`/`ldmx1` with a MOVE.B source `d8(An,Xn)` |

The core looks the cell up with the row and the column (column 6 uses column 4) before ph1, and registers it with
the operands (`colx_c`, `e_c`, `op1h_c`, `m68k_alux_si.sv`; tables `SI_ALU_T2`, `SI_ALU_OP1H2` of `m68k_alu_pkg.sv`).

### 9.3 Column, initiate/finish, operations (`m68k_alu_si.sv`)

- Column: nano field `alu_col` 1..6 and `x` (`ND_ALU_COL_*`, `m68k_nano_dec_pkg.sv`). Column 6 is column 4 with a 1
  shifted in (the quotient bit of the division) (`colx`, `in_l`, `m68k_alu_si.sv`); `x` = no operation.
- `alu_cc`: `i` (initiate) and `f` (finish) select the condition-code entry; the second entry of a cell applies
  only with finish; `n` leaves the condition codes alone (`init`, `fin`, `m68k_alux_si.sv`; `tk`, `m68k_alu_si.sv`).
- Operations, with a = the operand on the AB side and d = the operand on the DB side (the operation `case` of
  `m68k_alu_si.sv`):
  AND, OR, EOR; ADD a+d, ADDX a+d+X, ADD1 a+d+1; SUB d−a, SUBX d−a−X, SUB0 d−a−1; EXT (sign of the low byte); shifts
  by one place LSL, LSR, ASL, ASR, ROL, ROR, ROXL, ROXR, LSS (0 or 1 into bit 0) and SLAA.
- The carry-in of ADDX/SUBX is the X bit in an initiate word and the carry latch (§9.5) in a finish or `n` word
  (`xc`, `m68k_alu_si.sv`).
- Shifts act on the byte, the word, or the pair ALU:ALUE (long); the size is IRD[7:6] for register shifts and word
  otherwise (`sz`, `m68k_alux_si.sv`; `pair`, `pl`, `pr`, `m68k_alu_si.sv`). With `sr c-alu-alue` (multiplication) the
  pair is
  used, with the ALU as high word: C (MULU) or the true sign of the partial product N⊕V (MULS) enters the ALU msb and
  the ALU lsb enters the ALUE msb (`sz_eff`, `m68k_alux_si.sv`; `x_sh` and the `c_as_x` branch, `m68k_alu_si.sv`).
  SLAA in row 1
  (division) shifts the pair left with the ALU as remainder, the quotient bit entering ALUE (SLAA branch with
  `row == 1`, `m68k_alu_si.sv`).
  [mul-div.md](mul-div.md) describes both loops.
- Size: byte operations mask the adder operands to 8 bits and take N, V, C at bit 7; the result is always 16 bits
  (`mask`, `msb_a`, `msb_d`, `r`, `m68k_alu_si.sv`). Only the low byte of a data register is written (§8.6).
- The ALU works in two stages: the result and the carries are registered at ph2, the condition codes computed from
  them (`res_s` and the other registers loaded at `st_ld`, `m68k_alu_si.sv`).

### 9.4 Condition-code tokens

Per position X N Z V C (`ACC_*`, `m68k_alu_pkg.sv`; `cc_out`, `m68k_alu_si.sv`):

| token | value |
|---|---|
| K | unchanged |
| D | "don't care" in FIG. 17: N from the msb, V from the overflow, C from the carry; X and Z unchanged |
| 0 / 1 | cleared / set |
| N, V, C | from the operation |
| Z | result zero; with the pair, also ALUE = 0 (except in a finish-only word); "cleared if nonzero, unchanged otherwise" (Z AND old Z) in finish words, in the decimal cells (C*, ~C*) and for ADDX/SUBX |
| ~C | borrow = complement of the adder carry |
| C*, ~C* | carry, or borrow, OR X (decimal operations: carry of the binary addition, carried in X, or of the correction) |
| A | X and C from the shift carry |
| X (position C) | C = X for shifts and rotates (line 1110), with a count of zero; 0 for EXT, which uses the same cell |
| V' | old V OR (N ⊕ C): ASL sets V if the msb changes during any of the one-place shifts |

### 9.5 Operands and surrounding registers (`m68k_alux_si.sv`)

- AB-side value: D, else L, else H (`ab_v`, `m68k_alux_si.sv`); DB-side value: D, else L, else H (`db_v`).
- Operand d = the DB side with an `alu<-db*` atom (`db`, `db*`, `dbd`, `dbe`), else the correction factor
  (`alu<-corf`), else the constant −1 (`alu+=-1`, 0xFFFF: AND with it is a pass-through with flags) or 0 (`alu+=0`:
  SUB gives NEG, SUB0 gives NOT) (`d`, `m68k_alux_si.sv`).
- Operand a = the AB side with an `alu<-ab*` atom, or ALUB with `alu<-(alub)`; the three sources are mutually
  exclusive in the microcode; the constant −1 goes on a when d is already taken (`a`, `a_kc`, `m68k_alux_si.sv`).
- Result: registered at `en` into `src_alu` when an operation was performed (`alu_r`, `m68k_alux_si.sv`).
- LOCZ (condition `ze`): all bits of a XOR d are 1 (byte: the low 8). For a subtraction d + ~a + 1 this is "result
  zero"; for DBcc (a = Dn, d = 0xFFFF, ADD) it is "Dn = 0", so the loop ends when the decremented counter is −1
  (`locz`, `m68k_alux_si.sv`).
- Carry latch: written by initiate words and by every adder operation, read by finish words; it is valid only
  inside the instruction that wrote it, otherwise X is used (`cl`, `cl_v`, `m68k_alux_si.sv`).
- ALUE: loaded from `db_d` by `alue<-dbd` and `alue<-db*` (in every such word the D section is driven), updated by
  pair operations, which win over a load in the same word (`alue_in`, `alue_r`, `m68k_alux_si.sv`).
- ALUB: loaded from the AB side, or from DB (D, else L) (`alub_in`, `alub_r`).
- DCR (bit-number decoder): loads bits 4:0 of the AB side; outputs the mask 1 << n, with n modulo 8 for byte
  operands and modulo 16 otherwise, and `d4` = bit 4 of n (0 for byte operands) (`src_dcr`, `d4`, `dcr_n`,
  `m68k_alux_si.sv`).
  The bit number is taken modulo 8 in memory and modulo 32 in a data register (PRM §4, BTST/BCHG/BCLR/BSET); bit 4
  selects the word through the `d4` branch condition (§11).
- Correction factor (ABCD/SBCD/NBCD), computed after each operation from the binary result (`corf_n`, `lo6`, `hi6`,
  `m68k_alux_si.sv`):
  for an addition +0x06 if the half carry or the low digit > 9, +0x60 if the decimal carry or the high digit > 9 or
  (low digit > 9 and high digit = 9); for a subtraction the complement of (0x60 if borrow, 0x06 if half borrow),
  the following microword adding it with ADD1 (FIG. 17 row 9 column 3). The decimal carry is the adder carry (or
  borrow), not
  the PSW C (`dc_c`).

## 10. Status register PSW (`m68k_psw_si.sv`)

PSW = T (15), S (13), I2:0 (10:8), X N Z V C (4:0); the undefined bits read 0 (`psw`, `m68k_psw_si.sv`; UM §2.1.3).

- Reset: T = 0, S = 1, I = 7, flags 0 (reset branch of `m68k_psw_si.sv`); UM §6.3.1 forces supervisor state, trace off
  and mask 7.
- Flags: every microcycle the flags take the ALU output, which equals the input when the cell does not update them
  (`cc <= cc_alu`).
- `psw<-ftu`: flags from the FTU; T, S and I too, except for the byte forms (to CCR, RTR), which touch only the
  flags (`ld_ftu`, `m68k_psw_si.sv`; `ccr_only`, `m68k_cpu_si.sv`).
- `psw<-(inl)`: I = the latched interrupt level, S = 1, T = 0 (`ld_inl`).
- `psws+=+1` (exception entry): S = 1, T = 0 (`set_s`). Entry into the bus/address-error routine does
  the same one word later, so that the frame receives the status register copied before (`g0_q`).
  Source: UM §6.2.5, exception processing makes an internal copy of the status register, then sets S and clears T.
- Effective S: in the word that sets S the new value already selects the stack pointer (`set_s_o`, `m68k_psw_si.sv`;
  `sup_e`, `sup_e_q`, `m68k_cpu_si.sv`).

## 11. Conditional branch unit (`m68k_branch_si.sv`)

A format II microword holds a 5-bit CBC; the unit gives C1/C0, the bits 7:6 of the next address (§19.1). The table
`SI_BRANCH_TABLE[{cbc, value}]` is App. C: 21 codes, the 15 variables of codes 0-e and six second output
assignments of the same variables with the high CBC bit set (11, 12, 15, 19, 1c, 1e) (`m68k_branch_pkg.sv`;
`c1c0`, `m68k_branch_si.sv`; `docs/data/us4325121_appC.tsv`). The variables (`val`, `m68k_branch_si.sv`):

| code | variable | value |
|---|---|---|
| 0 | `i11` | IRC[11] (index size W/L of the extension word) |
| 1 | `auz` | the six low bits of the AU are zero (input `auz` of `u_br`, `m68k_cpu_si.sv`) |
| 2, 3, 5, d | `c`, `z`, `n`, `v` | PSW flags |
| 4, 6 | `nz1`, `nz2` | {N, Z} |
| 7 | `ms0` | {IRD[8], ALUE[0]} |
| 8 | `m01` | {auz, IRD[8], ALUE[1:0]} |
| 9 | `cc` | condition IRD[11:8] of Bcc/Scc/DBcc on the PSW, App. E (`cond`, `m68k_branch_si.sv`) |
| a | `ze` | LOCZ (§9.5) |
| b | `nv` | {N or Z, V} |
| c | `d4` | DCR bit 4 (§9.5) |
| e | `enl` | {IRD[6], END} (§13) |

Two compositions depart from the letter of App. C (`docs/data/deviations/branch.tsv`): `nv` uses N or Z, so that
CHK with Dn equal to the bound does not trap (PRM §4 CHK: trap if Dn < 0 or Dn > bound); `enl` has IRD[6] as high
bit and END as low bit, matching the App. C values of codes e/1e (00 → 10, 10 → 11, x1 → 00/01). `auz` tests six
bits: the comment on `auz` in `m68k_cpu_si.sv` records the reason (the RESET instruction loop counts the AU down from
−2 and ends only on the six-bit test; the other users start from values ≤ 63, where the two tests agree). The table
output is registered (`c1c0`, `m68k_cpu_si.sv`).

## 12. Field translation unit FTU (`m68k_ftu_si.sv`)

A 16-bit latch loaded by `ftu<-` atoms and read by `X<-ftu` atoms; without a pending load its output is a field
implied by the instruction in IRD. The patent describes the FTU as extracting a field from IRD and does not list the
constants; what the search of the patent found is in `docs/data/ftu_findings.md`.

Loads (`ld_*`, `ftu_rest`, `ld_val_flat`, `m68k_ftu_si.sv`):

| atom | value |
|---|---|
| `ftu<-(psw)`, `ftu<-(ssw)`, `ftu<-(ird)` | PSW, SSW, IRD (exception frames; the PSW also for MOVE from SR, App. H `strw1`, `stmw1`) |
| `ftu<-(trap)`, `ftu<-trap` | vector number × 4, from the vector latch of §16 |
| `ftu<-(inl)` | 0xFFF1 + 2 × level: the low word of the IACK address, sign-extended on the bus to all ones except A3-A1 = level (UM §5.1.4, Fig. 5-10) |
| `ftu<-ab` | the AB value, L else D (`ab_ld`, §5): the MOVEM mask; the new SR or CCR of MOVE to SR/CCR, ANDI/ORI/EORI to SR, RTR and STOP (App. H `mstw1`, `rstw1`, `stiw3`, `rtr2`; word `stop1`) |

The SSW is {IRD[15:5], R/W, I/N, FC} of the faulted cycle (input `ssw` of `u_ftu`, `m68k_cpu_si.sv`; R/W and FC
from `ssw_rw`, `ssw_fc`, `m68k_busctl_si.sv`; I/N is `tvn_use` of `m68k_exc_si.sv`, 1 while a group 0 or group 1
exception is being processed, UM §6.3.9.1, [exceptions.md](exceptions.md) section 6). The latch is released when a word consumes it or a new instruction
enters IRD
(`has_latch`, `m68k_ftu_si.sv`).

Implied field (`f_imp`, `m68k_ftu_si.sv`): Bcc/BSR and MOVEQ, the 8-bit displacement or data sign-extended; ADDQ/SUBQ
and immediate shifts, the quick value 1..8 (0 means 8); DBcc, the extension word in IRC (`f_imp_irc`, `implicit_v`);
TAS, 0x0080; MULU/MULS and DIVU/DIVS, the loop count `K_MUL` = `K_DIV` = 15 (parameters of `m68k_ftu_si.sv`, see
[mul-div.md](mul-div.md)); Scc, STOP and RESET, the parameters `K_DBCC`, `K_STOP`, `K_RESET`, all 0. The output is
registered at `en` (`ftu_q_r`).

## 13. MOVEM register encoder (`m68k_mvm_si.sv`)

- The 16-bit register mask is loaded by `ftu<-ab` from the AB value, L else D (`ld_mask`, `mask`, `m68k_mvm_si.sv`;
  input `mask_in` of `u_mvm`, `m68k_cpu_si.sv`): App. H loads it in the MOVEM blocks with `(dbin) -> ab -> ftu`.
- `reset pren` ("advance encoder to the address of the next register to be accessed", App. B) selects the lowest
  set bit of the mask and clears it (`adv`, `m68k_mvm_si.sv`).
- The selected register is bit 0 = D0 … bit 15 = A7, reversed for `-(An)` (bit 0 = A7) (PRM §4 MOVEM)
  (`predec_fl`, `rp_reg_c`, `rp_reg_q`, `m68k_mvm_si.sv`); it is the register the `rp` pointer selects (§8.3).
- END = mask empty (`mvm_end`, `m68k_mvm_si.sv`), used by the branch variable `enl` = {IRD[6], END} (§11).

## 14. Instruction pipeline IRC, IR, IRD (`m68k_ir_si.sv`)

IRC is the next word from the bus, IR the instruction being decoded, IRD the instruction being executed (US
4,325,121 FIG. 4/5, see [microcode-structure.md](microcode-structure.md)). At `en` (register block of `m68k_ir_si.sv`):

- `irc<-edb` loads IRC from the data just read;
- IR takes the previous IRC when the I bit of the microword is set (codes `dbi`, `dbl`, `bci`) or, in format I,
  when bit 4 is set (`dbc`, `dbl`) (`ld_ir`, `m68k_ir_si.sv`);
- `ird<-(ir)` loads IRD from IR.

The row class of the A1/A2/A3 decoder is computed from IRC and registered together with IR (`cls_n`, `cls_q`,
`m68k_ir_si.sv`).

## 15. A1/A2/A3 decoder (`m68k_a123_si.sv`)

A1 is the entry point of an instruction; A2 and A3 are the continuation points after the immediate operand and after
the effective-address computation (US 4,325,121 FIG. 5 and FIG. 21). Generated by
`docs/data/scripts/gen_a123.py`.

- Source: the PLA table of the die, `docs/data/mc68000_a123_die.tsv` (A1, A2, A3, illegal, privileged, line A/F for
  each of the 65,536 opcodes) (`DIE`, `gen_a123.py`), corrected where the die listing `all.txt` gives a different
  entry (`lst_fix`, `gen_a123.py`); FIG. 21 is used for A2/A3 only where the die table points at an address with no
  word. The corrections that the core applies to the die table, each with its source, are in
  `docs/data/deviations/a123.tsv`: the seven word/long pairs of the effective-address chains whose labels are swapped
  in the listing, so that NEGX/CLR/NEG/NOT/TST .L in memory enter the long chain; BCLR #n,(An) with A2 = 006 (one
  byte access); A3 091 → 299 (EOR to memory) and 288 → 3cb (TST.L in memory); MOVE USP privileged (PRM §4).
- Structure: eight exact opcode entries (BSR.W, Bcc.W, RESET, NOP, STOP, RTE/RTR, RTS, TRAPV) give A1 directly
  (`m`, `hit`, `a1`, `m68k_a123_si.sv`); otherwise A1, A2, A3 come from a table indexed by a row class of IR[15:6]
  (100 classes) and a column class of the effective-address field (13) (`cls`, `colc` and the class tables of
  `m68k_a123_si.sv`).
- Illegal: A1 = 3ff (`illegal`). The value 3ff never dispatches: the exception block replaces it with A0 = 0e3, the
  entry of the instruction exceptions (§16). Privileged: MOVE to SR, RESET, STOP, RTE, ANDI/ORI/EORI to SR, MOVE USP
  (`priv`); UM §6.3.7 lists the MC68000 privileged instructions.
- Line A and line F opcodes are recognised by the exception block from IR[15:12] (§16).
- The outputs are registered (`a1`, `a2`, `a3`, `illegal`, `priv_ir`, `m68k_cpu_si.sv`): IR loads at an `en` and the
  dispatch that uses A1 happens at
  a later `en`.

## 16. Exception block (`m68k_exc_si.sv`)

The block holds the special conditions in two groups of latches, numbered as in the patent: Group 1 holds the
exceptions of UM group 0, Group 2 those of UM group 1 and the STOP wait. It gives the microcode the address to jump
to and the vector number (US 4,325,121 FIG. 4/5, block 66, see [microcode-structure.md](microcode-structure.md)); vectors and
priorities are those of UM §6 and Table 6-2. [exceptions.md](exceptions.md) and [interrupts.md](interrupts.md)
describe the behaviour from outside.

Entry addresses (`A_RESET`, `A_BSER`, `A_INT`, `A_TRACE`, `A_TRAP`, `A_STOPW`, `m68k_exc_si.sv`): reset 000, bus and
address error 003, interrupt 1c4, trace and instruction
exceptions 0e3, STOP wait 3fc. 0e3 (`dvur2` of the die) continues through the trap chain whose content is that of the
patent's `trac1`/`trac2` words.

- **Group 1** (immediate, A0S): reset, bus error, address error, latched when they happen
  (`rst_l`, `berr_l`, `aerr_l`, `a0s_sub`, `a0s`). At the next `en` the micro address is replaced by A0S (000 or 003),
  and the
  vector latch receives 0, 3 or 2 in the order reset > address error > bus error (UM §6.2; `a0s_sub` branch of the
  vector latch).
- **Group 2** (at the instruction boundary, A0 in place of A1 at the dispatch word): trace, interrupt, illegal,
  privilege violation, line A, line F, and the STOP wait (`a0_sub`, `a0`). Priority: trace > interrupt >
  instruction exception > STOP wait.
- **Interrupt**: a request exists when the level is above the mask or a level-7 transition is pending
  (`int_req`, `nmi_pend`); level 7 is edge-sensitive (UM §6.3.2). The request is sampled at the `dbi` word with the
  IPL value at the start of that word, after one synchroniser stage on phi2 (`ipl_s1`, `ipl_w`, `upd_full` branch); the
  level is latched in INL.
- **Trace**: T is sampled at every dispatch, that is at the start of each instruction, and cleared when the dispatch
  takes an exception (`trace_l`); UM §6.3.8 (trace if T is set at the beginning of the instruction;
  no trace for an instruction not executed because of an interrupt, illegal or privileged).
- **Privilege**: privileged opcode with S = 0 (`priv`, `priv_set`); an illegal effective address in a
  privileged encoding gives the illegal vector (`priv & ~illegal` in the vector latch).
- **Vector number** (`tvn`, `tvn_r`): at the entry of an exception, trace 9, interrupt 24 + INL
  (autovector), privilege 8, line A 10, line F 11, illegal 4; the IACK cycle replaces the autovector with the number
  read from the bus, or with 24 (spurious interrupt) when it ends with BERR (`iack_vec`, `iack_spur`; UM Table 6-2
  and note 3, §6.3.4). Between two exceptions the vector comes from the instruction in IRD: TRAP #n = 32 + n,
  TRAPV 7, CHK 6, DIVU/DIVS 5, otherwise 4 (`f_tvn`). After reset the latch holds vector 0 and INL
  = 7 (reset branch of `m68k_exc_si.sv`), which the reset routine copies into the mask (`rstp2`, `(inl) -> psw`).
- **STOP**: the dispatch of the STOP word 327 goes to the wait word 3fc, whose next word 3fd samples the request and
  dispatches back to 3fc, a loop of two words with the period of the die's 327, 3a2, until trace or an interrupt is
  taken at the dispatch of 3fd (`stopped`, `A_STOPW`).
- **Double fault**: `g0_busy` is 1 out of reset and is set at every `a0s_sub` entry, and it stays high until the
  dispatch of the first instruction of the handler, so it covers the reset, bus error and address error routines;
  the bus controller halts the processor if a cycle started inside it fails (`g0_busy`, `m68k_exc_si.sv`;
  `dfault`, `halt_int`, `m68k_busctl_si.sv`; UM §5.4.4).

The partial update `dbl` (Group 1 only) needs no action: Group 1 is latched continuously (`upd_part`, `m68k_exc_si.sv`).

## 17. Microcycle sequencer and bus access (`m68k_busctl_si.sv`, `m68k_bus_fsm.sv`)

### 17.1 Phases

Phase counter and advance as in §3.3. After reset the phase starts at ph1 (reset value of `ph`, `m68k_busctl_si.sv`).
During HALT,
bus arbitration (BG or BGACK asserted) or a double fault, the phase stops at bus idle and no cycle starts
(`hold_core`, `frozen`, `req`, `m68k_busctl_si.sv`); [bus-control-signals.md](bus-control-signals.md) describes these
signals.

### 17.2 Access classes

The ACCESS cell of the microword (US 4,325,121, see [microcode-structure.md](microcode-structure.md)) has a class,
a mode and a type (`ND_ACC_*`, `m68k_nano_dec_pkg.sv`):

| class | meaning | RTL |
|---|---|---|
| `i` (initiate) | starts a bus cycle; the microcode does not wait | request at ph0 (`req`, `m68k_busctl_si.sv`) |
| `f` (finish) | the cycle started earlier must complete in this word | the word stays at ph3 until S7 of that cycle (`wait_done`, `stall`, `en`) |
| `t` (total) | starts and completes in the same word | request at ph0, then as `f` |
| `n` | no access | four phases |

- Mode `r`/`w`/`p`: read, write, no access (`wr`, `m68k_busctl_si.sv`).
- Type `in`, `im`, `ix`, `op`, `uk`, `ak`: instruction, immediate, both, operand, unknown, interrupt acknowledge.
- A cycle completed before its `f` word (for example `i`, `n`, `f`) is remembered until an `f`/`t` word consumes it
  (`done_l`).
- An `i` or `t` word whose predecessor's cycle is still running waits at ph0 for the bus to be free
  (`wait_free`).
- One bus cycle without wait states is S0..S7, four clocks, two microcycles (UM §5.1.1). The bus state machine has
  13 states: idle (S0), S1-S7, and S8, S10-S13 for the write half of the read-modify-write cycle
  (state list of `m68k_bus_fsm.sv`).

### 17.3 Function code, strobes, data

- FC: the FC field of the microword (bits 16:15: 01 data, 10 program, 11 CPU space) with the S bit; field 00 keeps
  the code of the previous access; type `uk` (operand through d16/abs/MOVEM) uses data space, or program space for a
  PC-relative operand (`acc_uk`, `fc_code_n`, `fc_r`, `m68k_busctl_si.sv`). FC = 111 for CPU space. The reset vector
  reads come from
  words whose field is 10, hence FC = 110, supervisor program (comment on `fc_code`, `m68k_busctl_si.sv`;
  `docs/data/mc68000_micro_decoded.csv`, rows 2fb, 2ff, 300, 11e; UM Table 6-2 note 2). Details:
  [function-codes.md](function-codes.md).
- Strobes: word accesses assert UDS and LDS; byte accesses assert UDS when A0 = 0 and LDS when A0 = 1 (UM §5.1.1,
  §5.1.2) (`uds_q`, `lds_q`, `m68k_busctl_si.sv`). Only operand accesses (`op`, `uk`) and the MOVEP half-word
  transfers are
  byte-sized; fetches are always words (`byte_acc`, `movep_half`, `m68k_cpu_si.sv`).
- Write data: a byte is placed on both halves of the data bus; for MOVEP (`dobh<-`) the byte written is the DOB high
  byte (`dobh_sel`, `wbyte`, `wdata_q`, `m68k_busctl_si.sv`).
- Read data: for a byte the addressed byte (D15-8 at an even address, D7-0 at an odd one) is placed on both halves
  of the internal data word (`rbyte`, `edb`, `m68k_busctl_si.sv`), so DBIN low holds the operand byte and MOVEP can take
  either half.
- Address error: a word or long access to an odd address (not IACK) runs as a cycle without data strobes and enters
  the address-error routine (`aerr_c`, `aerr_ev`, `m68k_busctl_si.sv`; `nostrobe`, `term_now`, `m68k_bus_fsm.sv`).

### 17.4 Read-modify-write

The read of the TAS operand opens an indivisible cycle: AS stays asserted after the read and the write follows
(input `rmw` of `u_fsm`, `m68k_busctl_si.sv`; branches S7, S8 and S10-S13 of `m68k_bus_fsm.sv`). UM §5.1.3: TAS is the
only instruction using it.

## 18. Operand size (`m68k_size_si.sv`)

Byte, word or long from IRD, following the instruction formats (PRM §4 and §8) (`sz`, `m68k_size_si.sv`): memory
bit operations byte; MOVEP word or long from bit 6; TAS, NBCD, Scc byte; DBcc word; shifts in memory word; EXG long;
instructions without a data operand word. The core uses only `is_byte`: it is computed from IR and registered
when IRD loads (`u_size`, `is_byte`, `m68k_cpu_si.sv`), and drives the ALU mask, the byte write of the register
file, the AU constants and byte bus accesses.

## 19. Control store

### 19.1 Micro ROM and next-address selector

- `m68k_urom.sv`: 1024 × 17 bits, read at `en` (`rom`, `q`), image `m68k_urom.mem`, the 17 raw bits of the die dump
  for 544 words plus the five entries of `docs/data/deviations/ustore.tsv` (word 1c8 `rstp2` repaired from App. H;
  words 3fc, 3fd, 3fe, 3ff of our design, at addresses where the die has no word: 3fc and 3fd are the STOP wait loop;
  3fe, which
  carries the 17 bits of the die word `trap1` (1d0), and 3ff remain in the ROM but are not selected, because the
  exception block enters the instruction exceptions at 0e3, §16).
  [chip-data-sources.md](chip-data-sources.md) gives the details.
- `m68k_useq.sv` decodes the microword (US 4,325,121 FIG. 8): bit 0 I, bit 1 format, bits 2-3 TY in format I, bits
  2-6 CBC in format II, bits 15-16 FC (`ir_load`, `fmt`, `ty`, `bit4`, `cbc`, `fc`, `m68k_useq.sv`). The 10-bit NMA is
  physically permuted on the die:
  A0←b11, A1←b12, A2←b7, A3←b8, A4←b9, A5←b10, A6←b5, A7←b6, A8←b13, A9←b14 (`nma`).
- Next address (`base`, `uaddr`, `m68k_useq.sv`): format II, NMA with bits 7:6 replaced by C1/C0 (§11); format I, TY
  00 = NMA,
  01 = A1 (or A0 when the exception block substitutes it), 10 = A2, 11 = A3; then A0S overrides everything; in reset
  the address is 000.
- The same address addresses both ROMs: the nano ROM is selected by the micro address through the ORIGIN map
  (§19.3).

### 19.2 Nano ROM encoding (ours)

The silicon's nanoword is 68 bits decoded into about 180 control points; the core does not reuse those bits. Each
atom of §4 is an action; atoms never present together in a word share a field (slot), with value 0 meaning "none"
(`pack`, `width`, `gen_nrom.py`). The nanoword has 77 bits (`NROM_W`, `m68k_nano_pkg.sv`), in two parts:

- bits 16:0 (`NROM_WL = 17`, `m68k_nano_pkg.sv`): the 50 atoms that form the read index of the register file
  (register sources of the buses, pointers, `rxsrc`), packed atom by atom in bits 14:0, plus the narrowest slots of the
  other part, moved here to keep the other part within 60 bits (`is_crit`, `best_pack_atoms`, `slots_l`, `gen_nrom.py`);
- bits 76:17 (60 bits): every other atom, packed by whole resource, so that each enumerated field of `dec` depends
  on one slot (`best_pack`, `gen_nrom.py`).

Words with the same set of atoms share one nanoword. The generator counts the addresses whose decoded nanoword
differs from their set of atoms (`bad`, `gen_nrom.py`), and the `ndec` bench checks every decoded atom against App. H
(§20).

### 19.3 ORIGIN map and early read (`m68k_nrom.sv`)

- Index of the nanoword = {row(addr[9:2]), addr[1:0]}: each group of four consecutive micro addresses is a
  quadruple of words, and equal quadruples share a row (`uniq`, `rows`, `gen_nrom.py`). The ORIGIN table maps the 256
  groups
  to 124 rows (7 bits) and sits in logic, as four explicit 64-entry tables and a 4-way mux, two LUT levels
  (constants `C2_T*_B*`, sub-tables `t_s0` ... `t_s3`, mux `row_pre`, `m68k_nrom.sv`; `m68k_norigin.mem`). The 124
  rows fill 496 of the 512 positions with 374 distinct
  nanowords, the empty word included.
- Both parts of the nanoword are read one clock before `en`: the next address without override is stable in the
  clock before `en`, the table gives its index, the index enters the address registers of the two memories
  (`idx_m`, `ql_pre`, `qm_pre`, `m68k_nrom.sv`), and at `en` the outputs load into `q_l` and `q_m`.
  `q_l` and `q_m` are flip-flops loaded at the same instant as a direct read would load them; the next word is also
  available one clock early (`q_pre`, `q_l_pre`, `m68k_nrom.sv`) for the selectors registered at `en` (§5, §8.3).
- The A0 path has its own table lookup in parallel, selected after the table (`row_a0`, `idx_a0`, `idx_pre`,
  `m68k_nrom.sv`).

### 19.4 Immediate override

With A0S or reset the address is decided in the same clock as `en`, with no clock of lead. In the core the override
addresses are 000 and 003 only (§16), so the index and the two parts of their nanowords are package constants
(`NROM_ORG_*`, `NROM_L_*`, `NROM_M_*`, `m68k_nano_pkg.sv`; `g_ovr_const`, `m68k_nrom.sv`). The benches can configure an
override from any address
(`ORG_OVR_LUT = 1`, `g_ovr_lut`, `m68k_nrom.sv`).

### 19.5 Decoder and timing of `dec`

`m68k_nano_dec.sv` is combinational, nanoword → `dec` (generated by `gen_nano_dec.py`). `m68k_nstore_dec.sv`
delivers `dec` from flip-flops in two classes (`DEC_RAW`, `dec_pq`, `dec_q`, `m68k_nstore_dec.sv`): the bits consumed
early (register
indices, bus selectors, access class) are the decode of the next nanoword registered at `en`, valid from the first
clock of the word; the others are a registered copy of the decode of the current word, valid one clock later.

### 19.6 Memory blocks

On Cyclone V the micro ROM takes two M10K blocks (1024 × 10 each; `rom`, `m68k_urom.sv`) and the nano ROM four:
`rom_l`, 512 × 17, one block in 512 × 20 mode, and `rom_m`, 512 × 60, three blocks (`m68k_nrom.sv`). The
ORIGIN table is logic.

## 20. Verification

Block benches (`make -f Makefile.si test` in `sim/`; models written from the published data, which they read at run
time, with the deviation files of §2.3; see `sim/README.md`):

| block | bench | target |
|---|---|---|
| micro ROM and sequencer | `sim/tb_m68k_ustore.cpp` | `ustore` |
| nano ROM structure, early read, override | `sim/tb_m68k_nstore.cpp` | `nstore` |
| nano decoder (`dec`) against App. H | `sim/tb_m68k_ndec.cpp` | `ndec` |
| bus sections (§5) | `sim/tb_m68k_bus_sec.cpp` | `bussec` |
| AT, AOB, DOB, DBIN (§6) | `sim/tb_m68k_regs.cpp` | `regs` |
| register file and MOVEM encoder (§8, §13) | `sim/tb_m68k_regfile_si.cpp` | `regfile_si` |
| ALU row decoder (§9.1) | `sim/tb_m68k_rowdec.cpp` | `rowdec` |
| ALU and condition codes (§9.2-§9.4) | `sim/tb_m68k_alu_si.cpp` | `alu_si` |
| ALU with surrounding registers (§9.5) | `sim/tb_m68k_alux_si.cpp` | `alux_si` |
| branch unit (§11) | `sim/tb_m68k_branch_si.cpp` | `branch_si` |
| A1/A2/A3 decoder (§15) | `sim/tb_m68k_a123_si.cpp` | `a123_si` |
| operand size (§18) | `sim/tb_m68k_size_si.cpp` | `size_si` |
| whole CPU, self-checking program | `sim/tb_m68k_cpu_si.cpp` | `cpu_si` |

Instruction benches (`make -f Makefile.si all`) that exercise the blocks of this document on the whole core, among
others: `abcd`, `abcdmem` (correction factor), `muldiv`, `div0` (ALUE, loop counts), `shiftbw`, `roxl` (shifts, X
token), `bitops`, `bitext` (DCR), `dbcc` (LOCZ, `cc`), `chk` (`nv`), `movem` (encoder, `enl`), `movep` (byte halves),
`idx`, `pcrel`, `moveaidx` (`rz`, `py`), `usp`, `movean` (register selection), `srlogic`, `mvtsr`, `movesr`, `rtr`
(PSW and `ccr_only`), `tst` (TAS read-modify-write), `exc`, `illegal`, `priv`, `trapv`, `irq`, `irqmulti`, `stop`,
`berr`, `halt`, `rstvec` (exception block), `busfsm` (bus state machine alone). The RTL also carries simulation-only
checks (`ifdef VERILATOR`) that compare each optimised form with the plain form it replaces, on every clock.
