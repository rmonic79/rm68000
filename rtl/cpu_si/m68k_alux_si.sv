// This file is part of rm68000, an MC68000 core in SystemVerilog.
// Copyright (c) 2026 Umberto Parisi (rmonic79)
//
// rm68000 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
// later version. It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details: LICENSE in the root of the repository, or <https://www.gnu.org/licenses/>.
//
// A commercial license, for use without the obligations of the GPL (for example in closed-source products), is
// available from the author, who can also be contacted for other licensing needs.

// m68k_alux_si.sv - ALU with its surrounding registers, driven by the nano atoms: operands a (AB side) and d (DB side,
// ALUB, corf or constant -1/0), registered result (src_alu), ALUE (32-bit shift, `sr c-alu-alue` for multiplication),
// ALUB (latch of the second operand), DCR (bit decoder: mask and d4), corf (decimal correction factor), LOCZ (ze).
// Rules: docs/silicon-datapath.md §9.5. Matrix row from the m68k_alu_rowdec decoder (IRD); column/cc/byte from the nano ROM.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_alux_si #(
  // 1 = matrix row and cc_inh from a flop decided on `ir` and loaded at `en & ld_ird` (core: ONE single instance of the
  // decoder, on ir); 0 = combinational decoder on `ird` (testbench).
  parameter bit ROW_REG = 1'b1
) (
  input  logic        clk, rst_n, en,
  input  logic        op_ld,         // phase 1 of the microcycle (ph1, cen2): samples the ALU operands; the ALU works in the second half
  input  logic        st_ld,         // phase 2 (ph2, cen): ALU stage register
  input  logic        instr_start,   // dispatch of a new instruction (at en): the carry latch is valid only inside the instruction
  input  logic        is_byte,       // byte size from the IRD (m68k_size_si): the ALU masks and evaluates the flags on 8 bits
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [NDEC_W-1:0] dec,
  input  logic [15:0] ird,
  input  logic [15:0] ir,            // instruction at the stage before ird: the row is decided here
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic        ld_ird,        // the current word loads ird <- ir
  input  logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d,
  input  logic        ab_l_drv, ab_d_drv, db_l_drv, db_d_drv,
  input  logic [4:0]  cc_in,         // X N Z V C of the PSW
  output logic [15:0] src_alu,       // ALU result (register)
  output logic [15:0] src_alue,      // ALUE
  output logic [15:0] dbg_a, dbg_d, dbg_r,   // debug: inputs and combinational result of the ALU
  output logic [15:0] src_dcr,       // bit decoder mask
  output logic [4:0]  cc_alu,        // to the PSW (combinational: = cc_in if no update)
  output logic        locz,          // local copy of ALU == 0 (ze condition)
  output logic        d4,            // bit 4 of the bit number (d4 condition)
  output logic [1:0]  alue10         // ALUE[1:0] (ms0/m01 conditions)
);
  // --- decoding of the atoms
  logic [ND_ALU_FROM_W-1:0] af; assign af = dec[ND_ALU_FROM_LSB +: ND_ALU_FROM_W];
  logic a_ab, a_alub, d_db, d_corf;
  assign a_ab   = af[ND_ALU_FROM__AB] | af[ND_ALU_FROM__ABSTAR] | af[ND_ALU_FROM__ABD] | af[ND_ALU_FROM__ABE];
  assign d_db   = af[ND_ALU_FROM__DB] | af[ND_ALU_FROM__DBSTAR] | af[ND_ALU_FROM__DBD] | af[ND_ALU_FROM__DBE];
  assign a_alub = af[ND_ALU_FROM__P_ALUB];
  assign d_corf = af[ND_ALU_FROM__CORF];
  logic [ND_ALU_CONST_W-1:0] ak; assign ak = dec[ND_ALU_CONST_LSB +: ND_ALU_CONST_W];
  logic k_m1, k_0;
  assign k_m1 = (ak == ND_ALU_CONST__MINUS1[ND_ALU_CONST_W-1:0]);
  assign k_0  = (ak == ND_ALU_CONST__0[ND_ALU_CONST_W-1:0]);
  logic [ND_ALU_COL_W-1:0] colv; assign colv = dec[ND_ALU_COL_LSB +: ND_ALU_COL_W];
  logic [2:0] col;
  always_comb begin
    col = 3'd0;
    if (colv == ND_ALU_COL__1[ND_ALU_COL_W-1:0]) col = 3'd1;
    else if (colv == ND_ALU_COL__2[ND_ALU_COL_W-1:0]) col = 3'd2;
    else if (colv == ND_ALU_COL__3[ND_ALU_COL_W-1:0]) col = 3'd3;
    else if (colv == ND_ALU_COL__4[ND_ALU_COL_W-1:0]) col = 3'd4;
    else if (colv == ND_ALU_COL__5[ND_ALU_COL_W-1:0]) col = 3'd5;
    else if (colv == ND_ALU_COL__6[ND_ALU_COL_W-1:0]) col = 3'd6;
    // (the App. H cell 'in' is a printing typo for '1': the nano ROM data carry the corrected value)
  end
  logic [ND_ALU_CC_W-1:0] ccv; assign ccv = dec[ND_ALU_CC_LSB +: ND_ALU_CC_W];
  logic init, fin, byte_op;
  assign init = (ccv == ND_ALU_CC__I[ND_ALU_CC_W-1:0]);
  assign fin  = (ccv == ND_ALU_CC__F[ND_ALU_CC_W-1:0]);
  // the size of the ALU operation comes from the IRD (size decoder), as in the silicon: the 'alu_byte' atom of the nano (heuristic on the
  // alu column of App. H, 8 words) is NOT the size and is not used.
  assign byte_op = is_byte;
  logic unused_ab; assign unused_ab = |dec[ND_ALU_BYTE_LSB +: ND_ALU_BYTE_W];
  logic [ND_ALUB_FROM_W-1:0] bf; assign bf = dec[ND_ALUB_FROM_LSB +: ND_ALUB_FROM_W];
  logic alub_ld_ab, alub_ld_db, alub_ld_star;
  assign alub_ld_ab = bf[ND_ALUB_FROM__AB] | bf[ND_ALUB_FROM__ABSTAR] | bf[ND_ALUB_FROM__ABD];
  assign alub_ld_db = bf[ND_ALUB_FROM__DB] | bf[ND_ALUB_FROM__DBSTAR] | bf[ND_ALUB_FROM__DBD];
  assign alub_ld_star = 1'b0;   // the alub<-* atoms are resolved statically (bus of the source of the same transfer)
  logic alue_ld;
  assign alue_ld = dec[ND_ALUE_FROM_LSB +: ND_ALUE_FROM_W] != '0;
  // the ALUE load takes db_d directly instead of the priority chain
  //   db_v = db_d_drv ? db_d : db_l_drv ? db_l : db_h
  // Exhaustive census: of the 13 words that load the ALUE (atoms alue<-dbd and alue<-db*)
  // ALL drive the D section of the DB bus, so that priority always resolves to db_d and
  // its two mux levels would only lengthen the path. db_v stays as it is for the D input
  // of the ALU, which instead needs the L then H fallback.
  logic [15:0] alue_in; assign alue_in = db_d;
  // the B input of the ALU takes from the DB a TWO-way chain instead of the three of
  //   db_v = db_d_drv ? db_d : db_l_drv ? db_l : db_h
  // Exhaustive census of the 19 words that load ALUB from the DB: 18 drive the D section;
  // the only exception is word 234 (db<-ftu), which drives only L; NONE drives only H.
  // So the fallback to H is never needed here and the third level is not built. db_v stays
  // as it is for the D input, which uses that fallback.
  logic [15:0] alub_in; assign alub_in = db_d_drv ? db_d : db_l;   // same value as db_v in the words that load ALUB
  logic dcr_ld;
  assign dcr_ld = dec[ND_DCR_FROM_LSB +: ND_DCR_FROM_W] != '0;
  logic sr_mul;
  assign sr_mul = dec[ND_SR_CMINUSALUMINUSALUE_LSB +: ND_SR_CMINUSALUMINUSALUE_W] != '0;   // atom 'sr c-alu-alue'
  // --- matrix row and shift size
  logic [3:0] row; logic [14:0] row_n;
  logic cc_inh;   // An destination (MOVEA, ADDA/SUBA, ADDQ/SUBQ An): the ALU computes but the PSW is not updated
  // `row`/`cc_inh` are a pure function of the instruction (exhaustive table on ird[15:3]): in the core the decoder
  // reads `ir` and the result enters a flop at the instant `ird` takes `ir`; at reset it holds the entry
  // of ird = 0 (row 14, cc_inh 0: first row of m68k_alu_rowdec.sv). Same value, same cycle, and the cone
  // row -> FIG. 17 table -> e_q/op1h_q starts from the flop instead of passing through the decoder.
  // The core has a single decoder instance: the one on ird exists only in the testbench (ROW_REG = 0) and in the safety net.
  localparam logic [3:0] ROW_RST = 4'd14; localparam bit CC_INH_RST = 1'b0;   // m68k_alu_rowdec: `13'd0: row = 14, cc_inh = 0`
  generate if (ROW_REG) begin : g_row_reg
    logic [3:0] row_p; logic cc_inh_p;
    m68k_alu_rowdec u_row (.ird(ir), .row_n(row_n), .row(row_p), .cc_inh(cc_inh_p));
    always_ff @(posedge clk) begin
      if (!rst_n) begin row <= ROW_RST; cc_inh <= CC_INH_RST; end
      else if (en && ld_ird) begin row <= row_p; cc_inh <= cc_inh_p; end
    end
`ifdef VERILATOR
    // SAFETY NET: the flop must equal the combinational decoder on ird at every clock
    logic [3:0] row_v; logic cc_inh_v; logic [14:0] row_n_v;
    m68k_alu_rowdec u_row_chk (.ird(ird), .row_n(row_n_v), .row(row_v), .cc_inh(cc_inh_v));
    always @(posedge clk) if (rst_n && (row !== row_v || cc_inh !== cc_inh_v)) begin
      $display("ALU row flop MISMATCH t=%0t row=%0d/%0d cc_inh=%b/%b ird=%04x", $time, row, row_v, cc_inh, cc_inh_v, ird); $fatal(1);
    end
    logic unused_chk; assign unused_chk = |row_n_v;
`endif
  end else begin : g_row_comb
    m68k_alu_rowdec u_row (.ird(ird), .row_n(row_n), .row(row), .cc_inh(cc_inh));
    logic unused_ld; assign unused_ld = ld_ird | (|ir);
  end endgenerate
  logic [1:0] sz;
  assign sz = (ird[15:12] == 4'he && ird[7:6] != 2'b11) ? ird[7:6] : 2'd1;
  // --- operands
  logic [15:0] ab_v, db_v, alub_r, alue_r, alu_r, corf_r, a, d;
  // DATA section, L then H fallback (couplers): "(rxh) -> ab -> alu" (morl2, roal3, ...) drives only ABH
  assign ab_v = ab_d_drv ? ab_d : ab_l_drv ? ab_l : ab_h;
  assign db_v = db_d_drv ? db_d : db_l_drv ? db_l : db_h;
  always_comb begin
    d = 16'd0;
    if (d_db) d = db_v;
    else if (d_corf) d = corf_r;
    else if (k_m1) d = 16'hffff;
    else if (k_0) d = 16'd0;
  end
  // the A input of the ALU without a priority chain. Exhaustive census over the 547 used words
  // of the nano: the three sources of A are MUTUALLY EXCLUSIVE.
  //  - alu<-ab* and alu<-(alub) NEVER coexist (0 words);
  //  - the constant reaches A only with (d_db | d_corf), and the only word that does it is 341
  //    (alu+=-1 ; alu<-db ; alu_col=1), which has neither alu<-ab* nor alu<-(alub);
  //  - the constant 0 ((d_db | d_corf) & k_0) is NEVER used on A, and 0 is anyway the value
  //    with no source selected: it needs no term.
  // So A is ONE single AND-OR (three terms, one LUT per bit) instead of a priority if/else:
  // same value and same cycle by construction.
  logic a_kc; assign a_kc = (d_db | d_corf) & k_m1;   // constant -1 on the free input A (swap1)
  assign a = ({16{a_ab}} & ab_v) | ({16{a_alub}} & alub_r) | ({16{a_kc}} & 16'hffff);
  // --- ALU. `sr c-alu-alue` (multiplication) qualifies the shift of the column: 32-bit ALU:ALUE pair and C (not X) in the msb
  logic [1:0] sz_eff;
  assign sz_eff = sr_mul ? 2'd2 : sz;
  logic [15:0] r, alue_o; logic [4:0] cc_o; logic r_valid, hc, is_sub;
  // FIG. 17 entry looked up before ph1 (row from IRD, column from the nano) and registered at op_ld: outside the ALU -> flag cone
  logic [2:0] colx_c; assign colx_c = (col == 3'd6) ? 3'd4 : col;
  (* ramstyle = "logic" *) logic [44:0] e_c; (* ramstyle = "logic" *) logic [44:0] e_q; assign e_c = SI_ALU_T2[{row, colx_c}];   // direct index {row, column}, without arithmetic; 16 rows: row 0 = only EXT in column 5 (index mode of MOVEA)
  logic [20:0] op1h_c, op1h_q; assign op1h_c = SI_ALU_OP1H2[{row, colx_c}];   // one-hot code of the entry (21'd1 = NONE)
`ifdef VERILATOR
  // SAFETY NET: the direct index must equal the lookup by index row*5+colx-1 with range check, at every clock
  logic [44:0] e_c_v; logic [20:0] op1h_c_v;
  assign e_c_v = (colx_c >= 3'd1 && colx_c <= 3'd5) ? SI_ALU_TABLE[row * 5 + (colx_c - 1)] : 45'd0;
  assign op1h_c_v = (colx_c >= 3'd1 && colx_c <= 3'd5) ? SI_ALU_OP1H[row * 5 + (colx_c - 1)] : 21'd1;
  always @(posedge clk) if (rst_n && (e_c !== e_c_v || op1h_c !== op1h_c_v)) begin
    $display("FIG. 17 entry index MISMATCH t=%0t row=%0d colx=%0d", $time, row, colx_c); $fatal(1);
  end
`endif
  logic [3:0] row_q; logic [2:0] col_q; logic init_q, fin_q, byte_q, srmul_q; logic [1:0] sz_q;   // ALU control sampled at ph1 (dec/ird stable within the microcycle)
  logic [15:0] a_q, d_q, alue_q;   // operands sampled at ph1 (the buses and the registers are stable from ph0; the result is needed at en = ph3)
  always_ff @(posedge clk) begin
    if (!rst_n) begin a_q <= '0; d_q <= '0; alue_q <= '0; row_q <= '0; col_q <= '0; e_q <= '0; op1h_q <= 21'd1; init_q <= 1'b0; fin_q <= 1'b0; byte_q <= 1'b0; sz_q <= '0; srmul_q <= 1'b0; end
    else if (op_ld) begin a_q <= a; d_q <= d; alue_q <= alue_ld ? alue_in : alue_r; row_q <= row; e_q <= e_c; op1h_q <= op1h_c; col_q <= col; init_q <= init & ~cc_inh; fin_q <= fin & ~cc_inh; byte_q <= byte_op; sz_q <= sz_eff; srmul_q <= sr_mul; end
  end
  logic cl, cout_o, cl_v, pair_o, adder_o;   // carry latch: written by the 'initiate' half, read by the 'finish' half (multi-precision); cl_v: latch written by an 'initiate' of THIS instruction, otherwise the 'finish' uses X of the PSW (ABCD, single ADDX)
`ifdef VERILATOR
  // SAFETY NET: the registered one-hot is the decoding of the registered code, at every clock
  always @(posedge clk) if (rst_n && op1h_q !== (21'd1 << e_q[44:40])) begin
    $display("ALU one-hot opcode MISMATCH alux t=%0t op=%0d op1h=%h", $time, e_q[44:40], op1h_q); $fatal(1);
  end
`endif
  m68k_alu_si u_alu (.clk(clk), .st_ld(st_ld), .row(row_q), .e_in(e_q), .op1h(op1h_q), .col(col_q), .init(init_q), .fin(fin_q), .byte_op(byte_q), .sz(sz_q), .a(a_q), .d(d_q), .alue_in(alue_q),
                     .cc_in(cc_in), .c_as_x(srmul_q), .mul_signed(ird[8]), .shift_op(ird[15:12] == 4'he), .cl_in(cl_v ? cl : cc_in[4]), .r(r), .alue_out(alue_o), .cc_out(cc_o), .r_valid(r_valid), .hc(hc), .is_sub(is_sub), .cout_o(cout_o), .adder_o(adder_o), .pair_o(pair_o));
  assign cc_alu = cc_o;
  // --- decimal correction (Motorola manual): ABCD +6/+0x60; SBCD/NBCD: corf = complement of the correction (the step uses ADD1)
  logic lo6, hi6; logic [15:0] corf_n;
  // carry/borrow of the ADDER (cout, negated for subtractions), not the C of the PSW: in row 3 (ADDX CDDDD) C is 'D' = kept,
  // so a C=1 left by a previous instruction would make ABCD in memory add 0x66
  logic dc_c; assign dc_c = cout_o ^ is_sub;
  assign lo6 = hc | (r[3:0] > 4'd9);
  assign hi6 = dc_c | (r[7:4] > 4'd9) | ((r[3:0] > 4'd9) & (r[7:4] == 4'd9));   // the low +6 overflows only with low nibble > 9, not on half-carry (7E+13 = 97, not F7)
  assign corf_n = is_sub ? ~{8'd0, dc_c ? 4'd6 : 4'd0, hc ? 4'd6 : 4'd0} : {8'd0, hi6 ? 4'd6 : 4'd0, lo6 ? 4'd6 : 4'd0};   // SBCD: ~(0x60 with borrow C | 0x06 with half-borrow); the +1 comes from ADD1 (FIG. 17 row 9 col 3)
  // --- registers
  logic [4:0] dcr_n;
  assign src_alu = alu_r; assign src_alue = alue_r; assign alue10 = alue_r[1:0];
  // bit number modulo 8 on byte operands (memory: BTST/BCHG/BCLR/BSET #8,(An) acts on bit 0), modulo 32 on registers
  assign src_dcr = 16'd1 << (byte_op ? {1'b0, dcr_n[2:0]} : dcr_n[3:0]); assign d4 = dcr_n[4] & ~byte_op;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      alu_r <= 16'd0; alue_r <= 16'd0; alub_r <= 16'd0; corf_r <= 16'd0; dcr_n <= 5'd0; locz <= 1'b0; cl <= 1'b0; cl_v <= 1'b0;
    end else if (en) begin
      // LOCZ (ze): local 'all propagate' zero detector (a XOR d = all 1): for d+~a+1 (subtractions) it is result = 0,
      // for DCNT (a = Dn, d = ffff, ADD) it is Dn = 0 -> exit when Dn-1 = -1 (manual); the result Dn+ffff would be zero for Dn = 1
      if (r_valid) begin alu_r <= r; locz <= (byte_q ? &(a_q[7:0] ^ d_q[7:0]) : &(a_q ^ d_q)); corf_r <= corf_n; end
      // carry latch: written by the 'initiate' steps and by EVERY adder operation, also in mode 'n' (DIVS: dvs06 SUB 'n' -> dvs10 SUBX 'n';
      // dvs04 is an AND 'i' that cleared it). The 'finish' steps read it via cl_v.
      if (r_valid && (init || adder_o)) cl <= cout_o ^ is_sub;   // latch = carry (add) / borrow (sub), like the X bit
      if (r_valid && init) cl_v <= 1'b1; else if (instr_start) cl_v <= 1'b0;
      if (r_valid && pair_o) alue_r <= alue_o;   // operation on the pair (long shift, mul, div): wins over the load of the same word (srrl2)
      else if (alue_ld) alue_r <= alue_in;   // db_d: every word that loads the ALUE drives the D section
      if (alub_ld_ab) alub_r <= ab_v;
      else if (alub_ld_db || alub_ld_star) alub_r <= alub_in;   // D section, else L (no word drives only H)
      if (dcr_ld) dcr_n <= ab_v[4:0];
    end
  end
  /* verilator lint_off UNUSEDSIGNAL */
  logic unused; assign unused = |row_n;
  /* verilator lint_on UNUSEDSIGNAL */
  assign dbg_a = a; assign dbg_d = d; assign dbg_r = r;
endmodule
