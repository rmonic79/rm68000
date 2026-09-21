# FTU (Field Translation Unit, block 106 of FIG. 4) — what US 4,325,121 actually says

Single source: patent US 4,325,121 (Tredennick/Gunter, Motorola): the patent text and the
Google scans of its pages (page numbers pNN, URLs in `us4325121_google_image_urls.txt`), plus
the existing transcription of App. H in `us4325121_appH.tsv`.

## Summary: everything the patent says about the FTU

1. **Column 10** (scan page p57), the only functional description, 3 sentences:
   > "FIELD TRANSLATION UNIT 106 can extract a bit field from the macroinstruction
   > stored in IRD register 56 for use by the execution unit such as supplying an
   > offset which is to be combined with an index register. FTU 106 can also supply
   > bit fields extracted from PSW register 86 and SSW register 108 to the execution
   > unit 22. FTU 106 can also be used to transfer a result from execution unit 22
   > into PSW register 86."

2. **Appendix B** (col. 49-50, scan p77), entry of the transfer glossary:
   > "ftu       field translation unit"

   (no other detail; the nearby entry "trap" says: "stores vector which can be
   supplied to field translate unit for addressing a trap routine in event of trap
   condition (e.g. divide-by-zero)").

3. **Column 7** (scan p56), FIG. 3:
   > "Although not shown in FIG. 3, a field translate unit (ftu) is also coupled to
   > bus 20' and bus 22' and may be used to transfer digital information between the
   > execution unit and other sections of the data processor."

4. **App. H** (scan p85-p175): the microword blocks show only the
   form of the transfer (`ftu -> db`, `ftu -> abe`, `ftu -> *e`, ...). No block
   annotates the value/field. Checked on the image of page p134 (block mulm1):
   its FTU transfer is exactly `ftu --> db --> au`, with no annotation of the value.

5. **Not present** in the patent: no figure of the FTU (the list of figures runs FIG.1-FIG.21U
   and does not include one), no description of the nanoword control field that
   selects the translation, no list of the generated constants.
   Appendix G = "Word Formats" is the list of the MACROinstruction formats (opcode bits),
   not the FTU formats; Appendix F = "Instructions" (list of mnemonics).

## Case 1 — MULU/MULS, `mulm1` / `mulr1`: loop constant

**Answer: not found.**

- Source: App. H page p134 (SHEET D, blocks `06a mulm1` and `06f mulr1`), checked on the
  image; the text of col. 10 (the only FTU description) does not mention constants.
- Verbatim quote from block `mulm1` (App. H, p134):
  > `au --> ab --> aob,at` / `(dbin) --> dbd --> alue` / `ftu --> db --> au`
  > `(rxdl) --> abd --> alub` / `0 --> alu` / `0 --> au`
- Closest evidence (structural, not a value): the AU is the loop counter.
  `mulm4` (p134) does `au -> db -> au ; -1 -> au` and the branch is `mulm6 if au=0`,
  so the value loaded by `ftu -> db -> au` in mulm1/mulr1 is the initial
  count, but the patent does not write it anywhere.

## Case 2 — DIVU/DIVS, `dvum3` / `dvs04`: loop constant

**Answer: not found.**

- Source: App. H page p114 (`02d dvum3`) and p113 (`0ad dvs04`, origin dvum3).
- Verbatim quote (nanoword content of block dvum3/dvs04):
  > `ftu -> db -> au ; (rxh) -> ab -> alu,atl ; -1 -> alu ; +1 -> au`
- Closest evidence: the loaded count is `ftu + 1` (the AU does `+1` in the same
  microcycle); the decrement is in `dvum5`/`dvum6` (`au -> db -> au ; -1 -> au`); the AU
  test is in `dvum7`/`dvum8` (`dvum6 if au/=0` / `dvumb if au/=0`). Numeric value: never given.

## Case 3 — DBcc (`dcnt1`, of which `bbci1` is an origin alias): displacement or constant?

**Answer: not found** (the patent does not say what the FTU outputs in this microword).

- Source: App. H page p146 (`159 dcnt1`) and p148 (`15d bbci1`, origin = dcnt1 → same nanoword).
- Verbatim quote (block dcnt1):
  > `ftu -> abe -> au ; (pc) -> db -> au ; (rxd) -> abd -> alu ; -1 -> alu`
- What can be stated from the patent: `abe` = "sign extend sign bit onto high
  section of ADDRESS BUS" (App. B, col. 49), so the FTU drives the low section and the
  bus extends the sign onto the high section — the AU computes PC + the sign-extended FTU value.
  Closest text on the content: col. 10, "can extract a bit field from the
  macroinstruction stored in IRD register 56 ... such as supplying an offset".
  App. G (col. 61-62, scan p83) documents the formats: Bcc = `4 | Condition | 8-bit
  Displacement`, DCNT = `Register | Displacement` with the note
  > "DCNT (Displacement is always negative)"

  but does **not** link those fields to the FTU, nor say whether in dcnt1 the FTU gives the
  16-bit displacement or a byte. The fact that dcnt1 and bbci1 share the same nanoword
  (origin dcnt1) is the only hint: the field selection is not in the nanoword.

## Case 4 — STOP (`stop1`) and RESET (`rset1`)

**Answer: not found.**

- Source: App. H page p141 (`083 stop1`) and p133 (`087 rset1`, origin = stop1).
- Verbatim quote (block stop1/rset1):
  > `au -> pc ; ftu -> *e -> at ; 0 -> alu`
- What the patent allows to state: `*e` = "sign extend sign bit onto high section
  of ADDRESS BUS or alternatively onto high section of DATA BUS" (App. B, col. 49-50);
  `at` = "temporary address register". So the FTU loads AT with a 16-bit
  sign-extended value. The value is not specified. App. G (scan p83, col. 61-62) shows
  that RESET/STOP are opcodes without useful fields (`STOP` = fixed word + immediate word),
  so the field cannot be deduced from the format.

## Case 5 — Bcc.B (`bbci1`), BSR.B (`bsri3`), ADDQ/SUBQ (`raqw1`/`raql1`), immediate shift (`sriw1`/`sril1`), TAS (`tasm1`/`tasr1`)

**Answer: not confirmed by the patent (not found).** The patent never states
which field the FTU extracts in these microwords, nor the "quick 0 → 8" translation, nor the
0x80 mask of TAS.

- Sources (App. H, transcription + scans):
  - `bbci1` p148: `ftu -> abe -> au ; (pc) -> db -> au ; (rxdl) -> abd -> alu ; -1 -> alu`
  - `bsri3` p149: `alu -> dob ; au -> aob ; ftu -> *e -> au ; (pc) -> * -> au`
  - `raqw1` p129 / `raql1` p130: `au -> aob,pc ; ftu -> abe -> alu,rx ; (ryl) -> db -> alu`
  - `sriw1` p166 / `sril1` p163: `au -> aob,pc ; ftu -> dbe -> au ; (ryh) -> ab -> alu ; -1 -> alu ; 0 -> au`
  - `tasr1` p161 / `tasm1` p162: `ftu -> ab -> alu` (with `(rydl) -> dbd -> alu,alub` and `(dbin) -> dbd -> alu` respectively)
- What can be supported with a quote:
  1. col. 10: "can extract a bit field from the macroinstruction stored in IRD register 56";
  2. App. B (col. 49-50): `abe`/`dbe`/`*e` = "sign extend sign bit onto high section of
     ADDRESS BUS"/"...DATA BUS" — that is, the sign extension visible in App. H is the
     16→32-bit bus extension, NOT the 8→16 extension of the displacement, which the patent does not describe;
  3. App. G (scan p83, col. 61-62): Bcc has an "8-bit Displacement" field, ADDQ/SUBQ have a
     3-bit "data" field (bits 11-9), DCNT has "Displacement is always negative".
     Shift/rotate (line 1110) **does not appear** in the App. G formats (the appendix stops
     at ADDX, scan p84); TAS appears (scan p82) but only as an opcode layout.
- Conclusion: the quick value 1..8, the count 1..8 and the 0x80 mask are **outside** the patent.

## Method note — inventory of the FTU transfers in App. H (derived data, not a quote)

63 microwords touch the FTU. Forms found (from the transcription `us4325121_appH.tsv`):

- FTU as **destination** (explicit load): `(psw) -> ftu` (9), `(dbin) -> ab -> ftu` (16,
  origin malw3/leaa1/stmr1 — this is the "offset for index register" case of col. 10),
  `(ird) -> ftu` (bser4), `(ssw) -> ftu` (bser6), `(trap) -> ftu` (trap4, rstp1),
  `(inl) -> ftu` (itlx3), `(ryd) -> ab -> ftu` (rstw1), `alu -> ab -> ftu` (stiw3).
- FTU as **source**: `ftu -> psw` (rtr4, stiw4), `ftu -> ab -> alu` (7), `ftu -> abe -> alu,rx` (5),
  `ftu -> db -> au` (5), `ftu -> dbe -> au` (3), `ftu -> abe -> au` (2), `ftu -> abe -> at` (2),
  `ftu -> *e -> at` (2), `ftu -> *e -> au` (1), `ftu -> db -> alub` (1), `ftu -> * -> rydl` (1).
- The five cases examined (mulm1/mulr1, dvum3/dvs04, dcnt1/bbci1, stop1/rset1, raqw1/sriw1/tasm1)
  use the FTU **without** a preceding explicit load: the source is implicit
  (IRD and/or a hard-wired constant). The patent does not document this implicit selection.
