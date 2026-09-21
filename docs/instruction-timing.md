# Instruction execution timing

## 1. Purpose

This document gives the execution time of every MC68000 instruction and exception,
in clock periods, as Motorola publishes it, and explains how rm68000 arrives at
those times. The core has no timing tables: each instruction takes the time of the
die microcode it executes, so the published counts follow from the microcode
sequence and from the bus cycle. Sections 2–4 give the published data and the
mechanism; section 5 covers the instructions whose time depends on the data;
section 6 lists what verifies it.

## 2. Source and notation

**MC68000UM Rev. 8, Section 8, "16-Bit Instruction Execution Times"**, pages
8-1 to 8-11 (Tables 8-1 to 8-14). It applies to the MC68000 and to the other
members in 16-bit mode. Section 7 (8-bit bus) and Section 9 (MC68010) do not apply
to this core.

From the introduction of Section 8:

- Times are in external clock (CLK) periods. "It is assumed that both memory read
  and write cycles consist of four clock periods. A longer memory cycle causes the
  generation of wait states that must be added to the total instruction times."
- The notation is **n(r/w)**: n is the total number of clock periods, r the number
  of read cycles, w the number of write cycles. The manual's example: 18(3/1) is
  18 clocks, of which 12 for three reads and 4 for one write; for the remaining 2
  clocks the bus is idle while the processor completes internal operations.
- "The total number of clock periods (n) includes instruction fetch and all
  applicable operand fetches and stores."
- Where a table entry carries "+", the effective-address calculation time of
  Table 8-1 is added (clocks, reads and writes).

## 3. How the core produces the timing

### 3.1 Clock periods

The core runs on a fast system clock with two clock enables: `cen` (φ1, the rising
edge of the 68000 clock) and `cen2` (φ2, the falling edge). One clock period of the
68000 is one `cen` followed by one `cen2` (header of `rtl/cpu_si/m68k_bus_fsm.sv`). The
enables come from the clock-enable and DTACK generator `rtl/integration/m68k_cen_dtack.sv`,
instantiated as `u_dtack` in `rtl/cpu_si/m68k_cpu_dt_si.sv`, which paces the CPU at the target
frequency and generates DTACK from the memory handshake
([clock-dtack.md](clock-dtack.md)). All counts below are in these clock periods.

### 3.2 Microcycles and bus cycles

- One microcycle, the time of one microword, is **2 clock periods**: four phases,
  ph0 = φ1, ph1 = φ2, ph2 = φ1, ph3 = φ2. The control store advances at ph3 unless
  the microcycle is stalled (header of `rtl/cpu_si/m68k_busctl_si.sv`; `en`).
- A bus cycle S0–S7 is **4 clock periods**, two microcycles, with no wait states.
  States S0, S2, S4, S6 advance on φ1, and S1, S3, S5, S7 on φ2
  (header of `m68k_bus_fsm.sv`).
- Each microword carries an access class from the nano ROM
  ([microcode-structure.md](microcode-structure.md) section 11). `i` (initiate)
  starts a bus cycle in ph0 and does not wait. `f` (finish) holds its microcycle on
  ph3 until S7 of the cycle in progress. `t` (total) starts a cycle and holds until
  its S7, so a `t` word lasts 4 clock periods. `n` has no access and lasts 2. An `i`
  or `t` word whose previous cycle is still running waits for the bus in ph0
  (`wait_done`, `wait_free`, `stall`, `en`, `m68k_busctl_si.sv`).

The time of an instruction is therefore 2 clock periods per microword executed,
plus the clocks during which an `f` or `t` word waits for S7. With zero wait states
and a prefetch started one microword earlier, an `f` word does not wait. The time is
measured, as the manual does, from the fetch of the opcode to the fetch of the next
opcode.

Example, NOP, Table 8-12: 4(1/0). Its A1 entry is the die word 363 (`b`): type
`dbi`, access `irix`, which starts the prefetch of the next word and loads the PC. It
continues to 34c (`mmrw3`): access `frix`, which finishes that read, loads IRC and
dispatches the next instruction (A1). Two microwords and one read: 4 clocks, 1 read
(Appendix H blocks `b` and `mmrw3`; die addresses from
`docs/data/mc68000_micro_decoded.csv`).

### 3.3 Wait states, E-synchronized cycles, stalls

- **Wait states.** DTACK is sampled at the end of S4 and at every falling edge
  after it; each missing sample adds one clock period of S4/S5
  (signal timing in the header of `m68k_bus_fsm.sv`). The `f` or `t` word that waits for the end of the cycle
  is held for those clock periods, and an `i` or `t` word that needs the bus while
  the cycle is still running waits too (`wait_done`, `wait_free`). This is how
  the wait states that the manual says to add enter the instruction time.
- **M6800 cycles.** A cycle terminated by VPA, including the autovector interrupt
  acknowledge, is synchronized with E: AS is negated on the falling edge of E after
  VMA (VPA termination note above `term_now` in `m68k_bus_fsm.sv`). Its length depends on the phase of E, so the
  times of Table 8-14 for an interrupt assume a 4-clock acknowledge cycle (note to
  the table), which an autovectored acknowledge, being an E-synchronized cycle,
  does not have.
- **The generator.** When the memory stretches a cycle through the handshake, the
  generator delays DTACK, and afterwards it recovers the clock periods lost to the
  stall ([clock-dtack.md](clock-dtack.md)). Counted in CPU clock periods, an
  instruction takes the time of the tables plus its wait states and the extra length
  of any E-synchronized cycle.

## 4. The tables of MC68000UM Section 8

Transcribed from MC68000UM Rev. 8 as printed. "—" means the combination does not
exist. Footnote marks are those of the manual.

### Table 8-1. Effective address calculation times (page 8-2)

The total includes fetching any extension words, computing the address and fetching
the memory operand.

| mode | | byte, word | long |
|---|---|---|---|
| Dn | data register direct | 0(0/0) | 0(0/0) |
| An | address register direct | 0(0/0) | 0(0/0) |
| (An) | address register indirect | 4(1/0) | 8(2/0) |
| (An)+ | address register indirect with postincrement | 4(1/0) | 8(2/0) |
| −(An) | address register indirect with predecrement | 6(1/0) | 10(2/0) |
| (d16,An) | address register indirect with displacement | 8(2/0) | 12(3/0) |
| (d8,An,Xn)* | address register indirect with index | 10(2/0) | 14(3/0) |
| (xxx).W | absolute short | 8(2/0) | 12(3/0) |
| (xxx).L | absolute long | 12(3/0) | 16(4/0) |
| (d16,PC) | program counter indirect with displacement | 8(2/0) | 12(3/0) |
| (d8,PC,Xn)* | program counter indirect with index | 10(2/0) | 14(3/0) |
| #&lt;data&gt; | immediate | 4(1/0) | 8(2/0) |

\* The size of the index register (Xn) does not affect execution time. The manual
labels the PC-with-displacement row "(d8, PC)"; it is the (d16,PC) mode, as in
Tables 8-2 and 8-3.

### Table 8-2. Move byte and word (page 8-2)

Rows: source; columns: destination.

| source | Dn | An | (An) | (An)+ | −(An) | (d16,An) | (d8,An,Xn)* | (xxx).W | (xxx).L |
|---|---|---|---|---|---|---|---|---|---|
| Dn | 4(1/0) | 4(1/0) | 8(1/1) | 8(1/1) | 8(1/1) | 12(2/1) | 14(2/1) | 12(2/1) | 16(3/1) |
| An | 4(1/0) | 4(1/0) | 8(1/1) | 8(1/1) | 8(1/1) | 12(2/1) | 14(2/1) | 12(2/1) | 16(3/1) |
| (An) | 8(2/0) | 8(2/0) | 12(2/1) | 12(2/1) | 12(2/1) | 16(3/1) | 18(3/1) | 16(3/1) | 20(4/1) |
| (An)+ | 8(2/0) | 8(2/0) | 12(2/1) | 12(2/1) | 12(2/1) | 16(3/1) | 18(3/1) | 16(3/1) | 20(4/1) |
| −(An) | 10(2/0) | 10(2/0) | 14(2/1) | 14(2/1) | 14(2/1) | 18(3/1) | 20(3/1) | 18(3/1) | 22(4/1) |
| (d16,An) | 12(3/0) | 12(3/0) | 16(3/1) | 16(3/1) | 16(3/1) | 20(4/1) | 22(4/1) | 20(4/1) | 24(5/1) |
| (d8,An,Xn)* | 14(3/0) | 14(3/0) | 18(3/1) | 18(3/1) | 18(3/1) | 22(4/1) | 24(4/1) | 22(4/1) | 26(5/1) |
| (xxx).W | 12(3/0) | 12(3/0) | 16(3/1) | 16(3/1) | 16(3/1) | 20(4/1) | 22(4/1) | 20(4/1) | 24(5/1) |
| (xxx).L | 16(4/0) | 16(4/0) | 20(4/1) | 20(4/1) | 20(4/1) | 24(5/1) | 26(5/1) | 24(5/1) | 28(6/1) |
| (d16,PC) | 12(3/0) | 12(3/0) | 16(3/1) | 16(3/1) | 16(3/1) | 20(4/1) | 22(4/1) | 20(4/1) | 24(5/1) |
| (d8,PC,Xn)* | 14(3/0) | 14(3/0) | 18(3/1) | 18(3/1) | 18(3/1) | 22(4/1) | 24(4/1) | 22(4/1) | 26(5/1) |
| #&lt;data&gt; | 8(2/0) | 8(2/0) | 12(2/1) | 12(2/1) | 12(2/1) | 16(3/1) | 18(3/1) | 16(3/1) | 20(4/1) |

\* The size of the index register (Xn) does not affect execution time.

### Table 8-3. Move long (page 8-3)

| source | Dn | An | (An) | (An)+ | −(An) | (d16,An) | (d8,An,Xn)* | (xxx).W | (xxx).L |
|---|---|---|---|---|---|---|---|---|---|
| Dn | 4(1/0) | 4(1/0) | 12(1/2) | 12(1/2) | 12(1/2) | 16(2/2) | 18(2/2) | 16(2/2) | 20(3/2) |
| An | 4(1/0) | 4(1/0) | 12(1/2) | 12(1/2) | 12(1/2) | 16(2/2) | 18(2/2) | 16(2/2) | 20(3/2) |
| (An) | 12(3/0) | 12(3/0) | 20(3/2) | 20(3/2) | 20(3/2) | 24(4/2) | 26(4/2) | 24(4/2) | 28(5/2) |
| (An)+ | 12(3/0) | 12(3/0) | 20(3/2) | 20(3/2) | 20(3/2) | 24(4/2) | 26(4/2) | 24(4/2) | 28(5/2) |
| −(An) | 14(3/0) | 14(3/0) | 22(3/2) | 22(3/2) | 22(3/2) | 26(4/2) | 28(4/2) | 26(4/2) | 30(5/2) |
| (d16,An) | 16(4/0) | 16(4/0) | 24(4/2) | 24(4/2) | 24(4/2) | 28(5/2) | 30(5/2) | 28(5/2) | 32(6/2) |
| (d8,An,Xn)* | 18(4/0) | 18(4/0) | 26(4/2) | 26(4/2) | 26(4/2) | 30(5/2) | 32(5/2) | 30(5/2) | 34(6/2) |
| (xxx).W | 16(4/0) | 16(4/0) | 24(4/2) | 24(4/2) | 24(4/2) | 28(5/2) | 30(5/2) | 28(5/2) | 32(6/2) |
| (xxx).L | 20(5/0) | 20(5/0) | 28(5/2) | 28(5/2) | 28(5/2) | 32(6/2) | 34(6/2) | 32(6/2) | 36(7/2) |
| (d16,PC) | 16(4/0) | 16(4/0) | 24(4/2) | 24(4/2) | 24(4/2) | 28(5/2) | 30(5/2) | 28(5/2) | 32(5/2) |
| (d8,PC,Xn)* | 18(4/0) | 18(4/0) | 26(4/2) | 26(4/2) | 26(4/2) | 30(5/2) | 32(5/2) | 30(5/2) | 34(6/2) |
| #&lt;data&gt; | 12(3/0) | 12(3/0) | 20(3/2) | 20(3/2) | 20(3/2) | 24(4/2) | 26(4/2) | 24(4/2) | 28(5/2) |

\* The size of the index register (Xn) does not affect execution time. The manual
labels the two PC rows of this table "(d, PC)" and "(d, PC, Xn)".

The entry (d16,PC) → (xxx).L is printed 32(5/2). Every other entry of the (d16,PC) row equals the (d16,An) row,
which gives 32(6/2) in that column: the read count 5 is most likely a misprint.

### Table 8-4. Standard instructions (page 8-4)

The times include the operation, storing the result and reading the next
instruction.

| instruction | size | op&lt;ea&gt;,An† | op&lt;ea&gt;,Dn | op Dn,&lt;M&gt; |
|---|---|---|---|---|
| ADD/ADDA | byte, word | 8(1/0)+ | 4(1/0)+ | 8(1/1)+ |
| | long | 6(1/0)+** | 6(1/0)+** | 12(1/2)+ |
| AND | byte, word | — | 4(1/0)+ | 8(1/1)+ |
| | long | — | 6(1/0)+** | 12(1/2)+ |
| CMP/CMPA | byte, word | 6(1/0)+ | 4(1/0)+ | — |
| | long | 6(1/0)+ | 6(1/0)+ | — |
| DIVS | — | — | 158(1/0)+* | — |
| DIVU | — | — | 140(1/0)+* | — |
| EOR | byte, word | — | 4(1/0)*** | 8(1/1)+ |
| | long | — | 8(1/0)*** | 12(1/2)+ |
| MULS | — | — | 70(1/0)+* | — |
| MULU | — | — | 70(1/0)+* | — |
| OR | byte, word | — | 4(1/0)+ | 8(1/1)+ |
| | long | — | 6(1/0)+** | 12(1/2)+ |
| SUB | byte, word | 8(1/0)+ | 4(1/0)+ | 8(1/1)+ |
| | long | 6(1/0)+** | 6(1/0)+** | 12(1/2)+ |

- \+ Add effective address calculation time.
- † Word or long only.
- \* Indicates maximum basic value added to word effective address time.
- \*\* The base time of six clock periods is increased to eight if the effective
  address mode is register direct or immediate (effective address time should also
  be added).
- \*\*\* Only available effective address mode is data register direct.
- DIVS, DIVU: "The divide algorithm used by the MC68000 provides less than 10%
  difference between the best- and worst-case timings."
- MULS, MULU: "The multiply algorithm requires 38+2n clocks where n is defined as:
  MULU: n = the number of ones in the &lt;ea&gt;; MULS: n = concatenate the &lt;ea&gt;
  with a zero as the LSB; n is the resultant number of 10 or 01 patterns in the
  17-bit source; i.e., worst case happens when the source is $5555."

### Table 8-5. Immediate instructions (page 8-5)

The times include fetching the immediate operands, the operation, storing the
result and reading the next operation.

| instruction | size | op #,Dn | op #,An | op #,M |
|---|---|---|---|---|
| ADDI | byte, word | 8(2/0) | — | 12(2/1)+ |
| | long | 16(3/0) | — | 20(3/2)+ |
| ADDQ | byte, word | 4(1/0) | 4(1/0)* | 8(1/1)+ |
| | long | 8(1/0) | 8(1/0) | 12(1/2)+ |
| ANDI | byte, word | 8(2/0) | — | 12(2/1)+ |
| | long | 14(3/0) | — | 20(3/2)+ |
| CMPI | byte, word | 8(2/0) | — | 8(2/0)+ |
| | long | 14(3/0) | — | 12(3/0)+ |
| EORI | byte, word | 8(2/0) | — | 12(2/1)+ |
| | long | 16(3/0) | — | 20(3/2)+ |
| MOVEQ | long | 4(1/0) | — | — |
| ORI | byte, word | 8(2/0) | — | 12(2/1)+ |
| | long | 16(3/0) | — | 20(3/2)+ |
| SUBI | byte, word | 8(2/0) | — | 12(2/1)+ |
| | long | 16(3/0) | — | 20(3/2)+ |
| SUBQ | byte, word | 4(1/0) | 8(1/0)* | 8(1/1)+ |
| | long | 8(1/0) | 8(1/0) | 12(1/2)+ |

\+ Add effective address calculation time (text of section 8.4). The note for "\*"
is not printed under the table in Rev. 8.

ADDQ byte/word to An is printed 4(1/0)\*, while SUBQ in the same column is 8(1/0)\*. The MC68008 table of the same
manual (Table 7-6) gives ADDQ and SUBQ word to An the same time, 12(2/0), four clocks more than to Dn (8(2/0)): on
the 68000 ADDQ.W #,An takes 8(1/0), like SUBQ, and the 4 is most likely a misprint.

### Table 8-6. Single operand instructions (page 8-6)

| instruction | size | register | memory |
|---|---|---|---|
| CLR | byte, word | 4(1/0) | 8(1/1)+ |
| | long | 6(1/0) | 12(1/2)+ |
| NBCD | byte | 6(1/0) | 8(1/1)+ |
| NEG | byte, word | 4(1/0) | 8(1/1)+ |
| | long | 6(1/0) | 12(1/2)+ |
| NEGX | byte, word | 4(1/0) | 8(1/1)+ |
| | long | 6(1/0) | 12(1/2)+ |
| NOT | byte, word | 4(1/0) | 8(1/1)+ |
| | long | 6(1/0) | 12(1/2)+ |
| Scc | byte, false | 4(1/0) | 8(1/1)+ |
| | byte, true | 6(1/0) | 8(1/1)+ |
| TAS | byte | 4(1/0) | 14(2/1)+ |
| TST | byte, word | 4(1/0) | 4(1/0)+ |
| | long | 4(1/0) | 4(1/0)+ |

\+ Add effective address calculation time.

### Table 8-7. Shift/rotate instructions (page 8-6)

| instruction | size | register | memory |
|---|---|---|---|
| ASR, ASL | byte, word | 6+2n(1/0) | 8(1/1)+ |
| | long | 8+2n(1/0) | — |
| LSR, LSL | byte, word | 6+2n(1/0) | 8(1/1)+ |
| | long | 8+2n(1/0) | — |
| ROR, ROL | byte, word | 6+2n(1/0) | 8(1/1)+ |
| | long | 8+2n(1/0) | — |
| ROXR, ROXL | byte, word | 6+2n(1/0) | 8(1/1)+ |
| | long | 8+2n(1/0) | — |

\+ Add effective address calculation time for word operands. n is the shift count.

### Table 8-8. Bit manipulation instructions (page 8-7)

| instruction | size | dynamic, register | dynamic, memory | static, register | static, memory |
|---|---|---|---|---|---|
| BCHG | byte | — | 8(1/1)+ | — | 12(2/1)+ |
| | long | 8(1/0)* | — | 12(2/0)* | — |
| BCLR | byte | — | 8(1/1)+ | — | 12(2/1)+ |
| | long | 10(1/0)* | — | 14(2/0)* | — |
| BSET | byte | — | 8(1/1)+ | — | 12(2/1)+ |
| | long | 8(1/0)* | — | 12(2/0)* | — |
| BTST | byte | — | 4(1/0)+ | — | 8(2/0)+ |
| | long | 6(1/0) | — | 10(2/0) | — |

\+ Add effective address calculation time. \* Indicates maximum value; data
addressing mode only.

### Table 8-9. Conditional instructions (page 8-7)

| instruction | displacement | branch taken | branch not taken |
|---|---|---|---|
| Bcc | byte | 10(2/0) | 8(1/0) |
| | word | 10(2/0) | 12(2/0) |
| BRA | byte | 10(2/0) | — |
| | word | 10(2/0) | — |
| BSR | byte | 18(2/2) | — |
| | word | 18(2/2) | — |
| DBcc | cc true | — | 12(2/0) |
| | cc false, count not expired | 10(2/0) | — |
| | cc false, counter expired | — | 14(3/0) |

### Table 8-10. JMP, JSR, LEA, PEA and MOVEM (page 8-8)

| instruction | size | (An) | (An)+ | −(An) | (d16,An) | (d8,An,Xn)* | (xxx).W | (xxx).L | (d16,PC) | (d8,PC,Xn)* |
|---|---|---|---|---|---|---|---|---|---|---|
| JMP | — | 8(2/0) | — | — | 10(2/0) | 14(3/0) | 10(2/0) | 12(3/0) | 10(2/0) | 14(3/0) |
| JSR | — | 16(2/2) | — | — | 18(2/2) | 22(2/2) | 18(2/2) | 20(3/2) | 18(2/2) | 22(2/2) |
| LEA | — | 4(1/0) | — | — | 8(2/0) | 12(2/0) | 8(2/0) | 12(3/0) | 8(2/0) | 12(2/0) |
| PEA | — | 12(1/2) | — | — | 16(2/2) | 20(2/2) | 16(2/2) | 20(3/2) | 16(2/2) | 20(2/2) |
| MOVEM M→R | word | 12+4n (3+n/0) | 12+4n (3+n/0) | — | 16+4n (4+n/0) | 18+4n (4+n/0) | 16+4n (4+n/0) | 20+4n (5+n/0) | 16+4n (4n/0) | 18+4n (4+n/0) |
| | long | 12+8n (3+2n/0) | 12+8n (3+2n/0) | — | 16+8n (4+2n/0) | 18+8n (4+2n/0) | 16+8n (4+2n/0) | 20+8n (5+2n/0) | 16+8n (4+2n/0) | 18+8n (4+2n/0) |
| MOVEM R→M | word | 8+4n (2/n) | — | 8+4n (2/n) | 12+4n (3/n) | 14+4n (3/n) | 12+4n (3/n) | 16+4n (4/n) | — | — |
| | long | 8+8n (2/2n) | — | 8+8n (2/2n) | 12+8n (3/2n) | 14+8n (3/2n) | 12+8n (3/2n) | 16+8n (4/2n) | — | — |

n is the number of registers to move. \* The size of the index register (Xn) does
not affect the instruction's execution time. The read count of MOVEM M→R word,
(d16,PC), is printed "(4n/0)".

### Table 8-11. Multiprecision instructions (page 8-9)

| instruction | size | op Dn,Dn | op M,M |
|---|---|---|---|
| ADDX | byte, word | 4(1/0) | 18(3/1) |
| | long | 8(1/0) | 30(5/2) |
| CMPM | byte, word | — | 12(3/0) |
| | long | — | 20(5/0) |
| SUBX | byte, word | 4(1/0) | 18(3/1) |
| | long | 8(1/0) | 30(5/2) |
| ABCD | byte | 6(1/0) | 18(3/1) |
| SBCD | byte | 6(1/0) | 18(3/1) |

### Table 8-12. Miscellaneous instructions (page 8-10)

| instruction | size | register | memory |
|---|---|---|---|
| ANDI to CCR | byte | 20(3/0) | — |
| ANDI to SR | word | 20(3/0) | — |
| CHK (no trap) | — | 10(1/0)+ | — |
| EORI to CCR | byte | 20(3/0) | — |
| EORI to SR | word | 20(3/0) | — |
| ORI to CCR | byte | 20(3/0) | — |
| ORI to SR | word | 20(3/0) | — |
| MOVE from SR | — | 6(1/0) | 8(1/1)+ |
| MOVE to CCR | — | 12(1/0) | 12(1/0)+ |
| MOVE to SR | — | 12(2/0) | 12(2/0)+ |
| EXG | — | 6(1/0) | — |
| EXT | word | 4(1/0) | — |
| | long | 4(1/0) | — |
| LINK | — | 16(2/2) | — |
| MOVE from USP | — | 4(1/0) | — |
| MOVE to USP | — | 4(1/0) | — |
| NOP | — | 4(1/0) | — |
| RESET | — | 132(1/0) | — |
| RTE | — | 20(5/0) | — |
| RTR | — | 20(2/0) | — |
| RTS | — | 16(4/0) | — |
| STOP | — | 4(0/0) | — |
| SWAP | — | 4(1/0) | — |
| TRAPV | — | 4(1/0) | — |
| UNLK | — | 12(3/0) | — |

\+ Add effective address calculation time.

The read count printed for RTR (2) is lower than the number of words RTR must read:
the condition codes and the two words of the return address from the stack
(M68000PRM 4, RTR), plus the prefetch. Table 9-18 of the same manual (MC68010) gives RTR 20(5/0). The bench `rtr` checks that RTR reads the
three stack words once each (`sim/tb_m68k_rtr.cpp`, header).

### Table 8-13. Move peripheral instructions (page 8-10)

| instruction | size | register → memory | memory → register |
|---|---|---|---|
| MOVEP | word | 16(2/2) | 16(4/0) |
| | long | 24(2/4) | 24(6/0) |

### Table 8-14. Exception processing (page 8-11)

The times include all stacking, the vector fetch and the fetch of the first
instruction of the handler.

| exception | periods |
|---|---|
| Address error | 50(4/7) |
| Bus error | 50(4/7) |
| CHK instruction | 40(4/3)+ |
| Divide by zero | 38(4/3)+ |
| Illegal instruction | 34(4/3) |
| Interrupt | 44(5/3)* |
| Privilege violation | 34(4/3) |
| RESET** | 40(6/0) |
| Trace | 34(4/3) |
| TRAP instruction | 34(4/3) |
| TRAPV instruction | 34(5/3) |

\+ Add effective address calculation time. \* The interrupt acknowledge cycle is
assumed to take four clock periods. \*\* Indicates the time from when RESET and
HALT are first sampled as negated to when instruction execution starts.

## 5. Data-dependent times in the microcode

For most instructions the microword sequence is fixed, so the time is fixed. For
the following ones the sequence depends on a count or on the data, through
conditional branches of the microcode
([microcode-structure.md](microcode-structure.md) section 6). A branch at the end
of a word tests the value written by the previous word. Labels are those of
Appendix H (`docs/data/us4325121_appH.tsv`); die addresses are from
`docs/data/us4325121_appH_die_map.tsv` and `docs/data/mc68000_micro_decoded.csv`.

- **Shifts and rotates on a register** (Table 8-7, 6+2n and 8+2n). The word form
  runs `srrw1` (382) and `srrw2` (383), then one `srrw3` (2d3) per bit position,
  then `nbcr3` (253). `srrw1` loads the count into the AU. `srrw2` loads the operand
  and decrements the AU. Each `srrw3` shifts by one position, decrements the AU and
  repeats while AU ≠ 0. That is 2 + 2 + 2n + 2 clocks, with one read, the prefetch
  started in `srrw1`. The long form runs `srrl1`–`srrl5` with one more word: 8 + 2n.
  With an immediate count the instruction enters at `sriw1` (381) or `sril1` (385)
  instead of `srrw1` (382) or `srrl1` (386): the microwords are identical, the
  nanoword differs, and the count 1–8 comes from the instruction through the field
  translation unit (immediate shift entry of `f_imp`, `rtl/cpu_si/m68k_ftu_si.sv`).
  AU = 0 is tested on the six low bits of the AU (input `auz` of `u_br`, `rtl/cpu_si/m68k_cpu_si.sv`),
  so a count taken from a register counts modulo 64, as M68000PRM 4 specifies for the
  register form.
- **MULU, MULS** (Table 8-4, 38+2n): 16 shift iterations and one extra microcycle per
  addition or subtraction; derivation in [mul-div.md](mul-div.md) section 4.
- **DIVU, DIVS** (Table 8-4, maximum 140 and 158): a restoring division in 16 iterations,
  whose length depends on the path taken at each iteration ([mul-div.md](mul-div.md)
  sections 5 and 6).
- **MOVEM** (Table 8-10, n registers): the transfer words (`ldmr*`, `popm*`, `stmr*`,
  `push*`) are of access class `t`, one bus cycle each, and branch on the condition
  `enl` = {IRD[6], END}: the size, and whether the register list is exhausted
  (case `enl` of `val`, `rtl/cpu_si/m68k_branch_si.sv`). END comes from the MOVEM encoder
  `rtl/cpu_si/m68k_mvm_si.sv`. Each register costs 4 clocks for a word and 8 for a
  long.
- **Bcc, BRA, BSR, DBcc, Scc** (Tables 8-6 and 8-9). The die words branch on the
  condition `cc`, the instruction's condition field evaluated on the PSW
  (`cond`, `m68k_branch_si.sv`): `bbci1` (308) for a byte displacement, `bbcw1` (068)
  for a word displacement, `sccr1` (384) and `sccb1` (380) for Scc. DBcc branches
  first on `cc` (`dcnt1`, die `dbcc1`, 06c), then on `ze`, the local zero detector of the count
  (`dcnt3`, die `dbcc3`, 175). Taken and not-taken paths have different lengths, hence the two
  columns of Table 8-9.
- **CHK** (Table 8-12, no trap, and Table 8-14): `chkr2` branches on `nv` and
  `chkr3` on `n` to the trap path or to the end of the instruction.
- **RESET** (Table 8-12, 132(1/0)). `rset1` (3a6) and `rset2` (27b) set the AU to
  −2. Then `rset3` (0e4, AU ← AU − 2) and `rset4` (114, branch on AU = 0) repeat,
  and `rset5` (064) fetches the next instruction. On six bits the AU goes from 62
  to 0 in 31 iterations: 2 + 2 + 31 × 4 + 4 = 132 clocks, with one read.
- **Exceptions** (Table 8-14). The entry routines are those of the die
  ([microcode-structure.md](microcode-structure.md) section 8). The interrupt time
  includes the acknowledge cycle, whose length depends on how it ends: DTACK
  (vectored), VPA (an M6800 cycle synchronized with E), or BERR (spurious
  interrupt) ([interrupt-acknowledge.md](interrupt-acknowledge.md)).

## 6. Verification

The published counts are not stored anywhere in the core. They come from the
microcode, and during development they were verified by comparing the core's bus
activity with an external reference core, clock by clock.

From this repository alone (`sim/Makefile.si`):

| target | bench | what it checks |
|---|---|---|
| `busfsm` | `sim/tb_m68k_bus_fsm.cpp` | the bus cycle edge by edge against MC68000UM 5.1: strobes, DTACK sampling and wait states, the RMW cycle, the M6800 cycle; with two-phase and single-phase clock enables |
| `cpu_si` | `sim/tb_m68k_cpu_si.cpp` | the whole CPU on a self-checking program: order and type of every bus cycle, E period of 20 phases, run with either clock phase first |

The instruction benches of `make -f Makefile.si all` (84 targets, with `busfsm` above
and `cpudt`, the CPU with its DTACK generator, [clock-dtack.md](clock-dtack.md)
section 6.3) check results, condition codes, memory and bus cycles, not clock
counts per instruction; many say so in their headers (for
example `sim/tb_m68k_muldiv.cpp` and `sim/tb_m68k_rtr.cpp`).

During development the timing was also compared with an external reference core,
used as a black box, with comparison benches that are not part of this repository
(they and their results are available from the author on request): clocks per
instruction, measured as the steady-state interval between opcode fetches over
repeated copies, on both cores, at zero wait states also against the nominal values
of MC68000UM Section 8 (NOP, MOVE.W and MOVE.L forms, ADD.W), and repeated with wait
states; clocks between instruction boundaries on both cores, for 58 instruction forms
(arithmetic, logic, shifts by 1, 3, 4 and 8, MOVE, immediate forms, ADDA/SUBA); bus
timing with a memory that returns read data one clock late; a real program on both
cores in lockstep, every bus access with its length and the interval to the next
one, with wait states and interrupts, also used to sweep all 65,536 opcode words;
and the pin transitions after the external reset is released.
