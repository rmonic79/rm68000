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

// m68k_alu_si.sv - 16-bit ALU + condition code control of the silicon-structured datapath.
// Sources: US 4,325,121 FIG. 16 (operations, docs/silicon-datapath.md §9.3), FIG. 17 (row x column matrix,
// docs/data/mc68000_alu_matrix_fig17.tsv -> m68k_alu_pkg.sv), text col. 26-32 (CC codes, initiate/finish, byte).
// Row = from the m68k_alu_rowdec decoder (IRD); column = alu_col field of the nano ROM (1..6, 6 = column 4 with lss shifting in 1);
// init/fin = alu_cc field (i/f; n = no CC update); byte_op = alu_byte field.
// Operands: a = AB-side input, d = DB-side input (or ALUB); ALU:ALUE pair for 32-bit shifts.
import m68k_alu_pkg::*;
module m68k_alu_si (
  input  logic [3:0]  row,        // 1..15 (0 = no row)
  input  logic [44:0] e_in,       // FIG. 17 table entry for (row, column): {op, cc init, cc finish}
  input  logic [20:0] op1h,       // one-hot operation code, registered with the entry: bit k = (e_in[44:40] == k)
  input  logic        clk,
  input  logic        st_ld,      // phase 2 of the microcycle (ph2, cen): registers the result of stage 1; the flags are computed in phase 3
  input  logic [2:0]  col,        // 0 = no operation, 1..6
  input  logic        init, fin,  // initiate / finish (both 0 = 'n': CC untouched)
  input  logic        byte_op,    // byte operation (low 8 bits only)
  input  logic [1:0]  sz,         // shift size: 0 byte, 1 word, 2 long (ALU:ALUE pair)
  input  logic [15:0] a, d, alue_in,
  input  logic [4:0]  cc_in,      // X N Z V C
  input  logic        c_as_x,     // 1: the shifter uses C instead of X as the incoming bit (`sr c-alu-alue`, multiplication)
  input  logic        shift_op,   // IRD[15:12] = E (shift/rotate): the X token (C <- X, ROXd with count 0) applies only here; EXT.L uses the same cell (row 8 col. 1) with C = 0
  input  logic        mul_signed, // MULS (IRD[8]): the incoming bit of the pair shift is the true sign of the partial product (N^V), not C
  input  logic        cl_in,      // carry latch (carry of the 'initiate' half): carry-in of ADDX/SUBX in the 'finish' half
  output logic [15:0] r, alue_out,
  output logic [4:0]  cc_out,
  output logic        r_valid,    // 1 if an operation was executed (column 1..6 and valid row)
  output logic        hc,         // half carry (add) / half borrow (sub) between bits 3 and 4, for the decimal correction
  output logic        is_sub,     // the operation is a subtraction (for the sign of the decimal correction)
  output logic        pair_o,     // 1: operation on the 32-bit pair (ALUE updated: long shift, multiplication, division)
  output logic        cout_o,     // adder carry (to be latched in the 'initiate' half)
  output logic        adder_o     // adder operation (ADD/ADDX/ADD1/SUB/SUBX/SUB0): writes the carry latch in every CC mode (DIVS dvs06 'n' -> dvs10 SUBX)
);
  logic [44:0] e;
  logic [4:0]  op;
  logic [3:0]  tk [0:4];
  logic [2:0]  colx;
  assign colx = (col == 3'd6) ? 3'd4 : col;
  always_comb begin
    e = e_in;   // FIG. 17 entry already looked up and registered at op_ld by m68k_alux_si (the lookup leaves the op_ld -> flag cone)
    op = e[44:40];
    for (int k = 0; k < 5; k++) tk[k] = (fin & ~init) ? e[16 - 4 * k +: 4] : e[36 - 4 * k +: 4];   // column 1: second entry only with finish
  end
  assign r_valid = ~op1h[AOP_NONE];   // one-hot code: NONE = no operation

  logic x_in, x_sh;
  assign x_in = cc_in[4];
  logic xc;   // extended carry-in: X (PSW) in the 'initiate' steps (single word or low half), carry latch in the 'finish' AND in mode 'n'
              // (DIVS dvs10: the SUBX 'n' of the 32-bit negation of the dividend takes the borrow of dvs06, SUB 'n')
  assign xc = init ? x_in : cl_in;
  // `sr c-alu-alue`: MULU shifts in the carry of the previous sum (C); MULS (Booth: signed add/subtract) shifts in the true
  // sign of the previous result, N^V (V=1 if the 16-bit sum overflowed; after a shift V=0 and N = msb of the pair).
  assign x_sh = c_as_x ? (mul_signed ? (cc_in[3] ^ cc_in[1]) : cc_in[0]) : cc_in[4];
  logic [16:0] sum;
  logic [15:0] res, ext16, sh_r, sh_e;
  logic        cout, carry, ovf, sh_c, hc_c, is_sub_c, is_add_c;
  logic        msb_a, msb_d, msb_r;
  logic [15:0] mask;
  logic pair32;   // shift operation on the 32-bit pair (N/Z flags from the pair)
  logic alu_high; assign alu_high = c_as_x | (op == AOP_SLAA && row == 4'd1);   // pair with ALU = high word (mul, div)
  assign pair_o = pair32;
  assign mask = byte_op ? 16'h00ff : 16'hffff;
  assign msb_a = byte_op ? a[7] : a[15];
  assign msb_d = byte_op ? d[7] : d[15];
  // shifter on the pair [alu:alue] (long), on alu (word) or on the low byte (byte): one position per microcycle
  logic [31:0] pair, pl, pr;
  logic        in_l, in_r, out_l, out_r;
  assign pair = {alue_in, a};   // 32-bit pair: ALUE = high word, ALU = low word (App. H srrl2: alu -> alue, then (rydl) -> alu; srrl4: alu -> rydl, alue -> ryh)
  always_comb begin
    // bit shifted out on the left (msb of the size) and on the right (bit 0 of the ALU word for word/byte, of ALUE for long)
    out_l = (sz == 2'd2) ? alue_in[15] : (sz == 2'd1) ? a[15] : a[7];
    out_r = a[0];
    in_l = 1'b0; in_r = 1'b0;
    unique case (1'b1)   // one-hot selection from flops, without decoding
      op1h[AOP_ROL]:  in_l = out_l;
      op1h[AOP_ROR]:  in_r = out_r;
      op1h[AOP_ROXL]: in_l = x_sh;
      op1h[AOP_ROXR]: in_r = x_sh;
      op1h[AOP_ASR]:  in_r = out_l;
      op1h[AOP_LSS], op1h[AOP_SLAA]: in_l = (col == 3'd6);   // column 6 = column 4 with 1 in the incoming bit (division: quotient bit)
      default: ;
    endcase
    // left shift
    pl = {pair[30:0], 1'b0};
    if (sz == 2'd2)      begin pl[0] = in_l; end
    else if (sz == 2'd1) begin pl = {a[14:0], in_l, alue_in}; end
    else                 begin pl = {a[15:8], a[6:0], in_l, alue_in}; end
    // right shift
    pr = {1'b0, pair[31:1]};
    if (sz == 2'd2)      begin pr[31] = in_r; end
    else if (sz == 2'd1) begin pr = {in_r, a[15:1], alue_in}; end
    else                 begin pr = {a[15:8], in_r, a[7:1], alue_in}; end
  end
  /* verilator lint_off UNUSEDSIGNAL */
  logic [4:0] nib;   // only bit 4 (half carry) is used
  /* verilator lint_on UNUSEDSIGNAL */
  always_comb begin
    sum = 17'd0; res = 16'd0; carry = 1'b0; ovf = 1'b0; sh_r = a; sh_e = alue_in; sh_c = 1'b0; cout = 1'b0; pair32 = 1'b0;
    ext16 = {{8{a[7]}}, a[7:0]}; nib = 5'd0; hc_c = 1'b0; is_sub_c = 1'b0; is_add_c = 1'b0;
    unique case (1'b1)   // one-hot selection from flops, without decoding
      op1h[AOP_AND]:  res = a & d;
      op1h[AOP_OR]:   res = a | d;
      op1h[AOP_EOR]:  res = a ^ d;
      op1h[AOP_ADD], op1h[AOP_ADDX], op1h[AOP_ADD1]: begin
        sum = {1'b0, a & mask} + {1'b0, d & mask} + {16'd0, op1h[AOP_ADDX] ? xc : op1h[AOP_ADD1]};
        res = sum[15:0]; cout = byte_op ? sum[8] : sum[16];
        ovf = (msb_a == msb_d) && ((byte_op ? res[7] : res[15]) != msb_a);
        carry = cout;
        nib = {1'b0, a[3:0]} + {1'b0, d[3:0]} + {4'd0, op1h[AOP_ADDX] ? xc : op1h[AOP_ADD1]}; hc_c = nib[4]; is_add_c = 1'b1;
      end
      op1h[AOP_SUB], op1h[AOP_SUBX], op1h[AOP_SUB0]: begin   // d - a (- X): d + ~a + cin
        sum = {1'b0, d & mask} + {1'b0, (~a) & mask} + {16'd0, op1h[AOP_SUB] ? 1'b1 : op1h[AOP_SUBX] ? ~xc : 1'b0};
        res = sum[15:0]; cout = byte_op ? sum[8] : sum[16];
        ovf = (msb_a != msb_d) && ((byte_op ? res[7] : res[15]) != msb_d);
        carry = cout;                        // adder carry-out, not negated here: the CC code "~C" of FIG. 17 negates it (borrow)
        nib = {1'b0, d[3:0]} + {1'b0, ~a[3:0]} + {4'd0, op1h[AOP_SUB] ? 1'b1 : op1h[AOP_SUBX] ? ~xc : 1'b0}; hc_c = ~nib[4]; is_sub_c = 1'b1;
      end
      op1h[AOP_EXT]:  res = ext16;
      op1h[AOP_LSL], op1h[AOP_ASL], op1h[AOP_ROL], op1h[AOP_ROXL], op1h[AOP_LSS], op1h[AOP_SLAA]: begin
        if (op == AOP_SLAA && row == 4'd1) begin
          // division (FIG. 17 row 1 col. 4/6, App. H dvum5/dvum6 'alu -> ab* -> alu'): left shift of the pair with ALU = remainder
          // (high word) and ALUE = low dividend / quotient (low): alue[15] enters alu[0], the quotient bit enters alue[0].
          // The bit shifted out (alu[15]) is the 17th bit of 2R: it is N of the last CC update (dvum5 'if n' -> dvum7 without compare).
          sh_r = {a[14:0], alue_in[15]}; sh_e = {alue_in[14:0], in_l}; sh_c = a[15]; res = sh_r;
          pair32 = 1'b1;
        end else begin
        sh_r = (sz == 2'd2) ? pl[15:0] : pl[31:16]; sh_e = (sz == 2'd2) ? pl[31:16] : alue_in; sh_c = out_l; res = sh_r;   // long: ALU = low, ALUE = high
        pair32 = (sz == 2'd2);
        end
        ovf = op1h[AOP_ASL] ? (out_l ^ ((sz == 2'd2) ? sh_e[15] : (sz == 2'd0) ? sh_r[7] : sh_r[15])) : 1'b0;
        carry = sh_c;
      end
      op1h[AOP_LSR], op1h[AOP_ASR], op1h[AOP_ROR], op1h[AOP_ROXR]: begin
        if (c_as_x) begin
          // `sr c-alu-alue` (multiplication, FIG. 17 row MUL col. 4 ROXR KNZ0A): C enters the msb of the ALU, the lsb of the ALU enters
          // the msb of ALUE (App. H mulm6: rxh <- alu, rxdl <- alue: ALU = high word, ALUE = low). The bit shifted out is alue[0]
          // (consumed multiplier bit). C 'A' (special) = 0: a shift without an intermediate sum must shift in 0.
          sh_r = {in_r, a[15:1]}; sh_e = {a[0], alue_in[15:1]}; sh_c = 1'b0; res = sh_r;
          pair32 = 1'b1;
          carry = 1'b0;
        end else begin
          sh_r = (sz == 2'd2) ? pr[15:0] : pr[31:16]; sh_e = (sz == 2'd2) ? pr[31:16] : alue_in; sh_c = out_r; res = sh_r;
          pair32 = (sz == 2'd2);
          carry = sh_c;
        end
      end
      default: res = a;
    endcase
  end
  // --- stage registers (ph2): the op_ld -> adder/shifter -> flag cone is split into two static phases of the microcycle
  logic [15:0] res_s, sh_e_s, sh_r_s; logic carry_s, ovf_s, sh_c_s, cout_s, hc_s, is_sub_s, pair32_s, adder_s;
  always_ff @(posedge clk) if (st_ld) begin
    res_s <= res; sh_e_s <= sh_e; sh_r_s <= sh_r; carry_s <= carry; ovf_s <= ovf; sh_c_s <= sh_c; cout_s <= cout; hc_s <= hc_c;
    is_sub_s <= is_sub_c; pair32_s <= pair32; adder_s <= is_sub_c | is_add_c;
  end
  assign hc = hc_s; assign is_sub = is_sub_s; assign cout_o = cout_s; assign adder_o = adder_s;
  always_comb begin
    // result always the whole 16-bit ALU word (die model): for bytes only the flags look at 8 bits; writing only the low byte is
    // done by the regfile (byte_op on the data registers). The whole word is needed when IRD holds a byte instruction but the
    // microcode uses the ALU on words: the exception chain (AND with -1 of PCL/SR/PCH: ILLEGAL 4afc, MOVE #imm,SR, an IRQ taken
    // on MOVE.B) and EXT of a negative d8.
    r = res_s;
    alue_out = sh_e_s;
    // 32-bit shift: N = bit 31 = high word: ALUE in long shifts, ALU in multiplication/division (pair {ALU, ALUE})
    msb_r = pair32_s ? (alu_high ? sh_r_s[15] : sh_e_s[15]) : byte_op ? r[7] : r[15];
    // condition code: token per position X N Z V C
    cc_out = cc_in;
    if (r_valid && (init | fin)) begin
      for (int k = 0; k < 5; k++) begin
        logic nv; nv = cc_in[4 - k];
        case (tk[k])
          ACC_K: nv = cc_in[4 - k];
          // 'D' (don't care, only in the decimal cells): the microcode does not constrain it and the silicon writes the natural value: N from the msb,
          // V from the overflow, C from the carry (reference core: ABCD sets N from the result); Z and X are kept (Z 'unchanged otherwise' in the manual)
          ACC_D: nv = (k == 1) ? msb_r : (k == 3) ? ovf_s : (k == 4) ? carry_s : cc_in[4 - k];
          ACC_0: nv = 1'b0;
          ACC_1: nv = 1'b1;
          ACC_N: nv = msb_r;
          // finish: Z = previous Z AND zero of the high word (patent col. 30); likewise in the decimal cells (C*/~C*: ABCD/SBCD/NBCD, manual:
          // 'Z cleared if nonzero, unchanged otherwise')
          // and in the operations with X (ADDX/SUBX/NEGX: 'Z cleared if nonzero, unchanged otherwise')
          ACC_Z: nv = ((r & mask) == 16'd0) & ((pair32_s & ~(fin & ~init)) ? (sh_e_s == 16'd0) : 1'b1) & (((fin & ~init) | (tk[4] == ACC_CS) | (tk[4] == ACC_NCS) | op1h[AOP_ADDX] | op1h[AOP_SUBX]) ? cc_in[2] : 1'b1);
          ACC_V: nv = ovf_s;
          ACC_C:   nv = carry_s;
          ACC_CS:  nv = carry_s | cc_in[4];     // C* = carry of the ADDX step (in X: the C of the ADDX cell CDDDD is 'D') or carry of the decimal correction
          ACC_NC:  nv = ~carry_s;
          ACC_NCS: nv = ~carry_s | cc_in[4];   // ~C* = borrow of the SUBX step (in X) or borrow of the correction (SBCD 25-48: C=1)
          ACC_A: nv = sh_c_s;                  // X and C from the shift carry
          ACC_X: nv = shift_op & cc_in[4];     // C = X only for ROXd with count 0 (manual); EXT.L (same cell) C = 0
          ACC_VP: nv = cc_in[1] | (msb_r ^ carry_s);   // V' = previous V OR (N xor C): ASL sets V if the MSB changes in any step (manual)
          default: nv = cc_in[4 - k];
        endcase
        cc_out[4 - k] = nv;
      end
    end
  end
endmodule
