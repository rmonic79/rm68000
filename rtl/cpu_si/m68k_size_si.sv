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

// m68k_size_si.sv - operand size from the instruction being executed (IRD). Source: Motorola MC68000 manual
// (instruction formats). Used by the bus sequencer (byte -> UDS/LDS from A0) and by the AU (+1/+2 for byte (An)+/-(An)).
// sz: 0 = byte, 1 = word, 2 = long. Instructions without a data operand (jumps, LEA, JSR, MOVEM ...) -> word.
module m68k_size_si (
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [15:0] ird,
  /* verilator lint_on UNUSEDSIGNAL */
  output logic [1:0]  sz,
  output logic        is_byte
);
  logic [3:0] line; logic [1:0] s;
  assign line = ird[15:12]; assign s = ird[7:6];
  always_comb begin
    sz = 2'd1;
    case (line)
      4'h0: begin
        if (ird[8] || ird[11:8] == 4'b1000) sz = (ird[5:3] == 3'b000) ? 2'd2 : 2'd0;   // dynamic/static bit: Dn = long, memory = byte (MOVEP: word/long from bit 6)
        else sz = (s == 2'b11) ? 2'd1 : s;                                              // ORI/ANDI/SUBI/ADDI/EORI/CMPI
        if (ird[8] && ird[5:3] == 3'b001) sz = ird[6] ? 2'd2 : 2'd1;                    // MOVEP
      end
      4'h1: sz = 2'd0;                                                                  // MOVE.B
      4'h2: sz = 2'd2;                                                                  // MOVE.L
      4'h3: sz = 2'd1;                                                                  // MOVE.W
      4'h4: begin
        if (ird[11:6] == 6'b101011) sz = 2'd0;                                          // TAS
        else if (ird[11:6] == 6'b100000) sz = 2'd0;                                     // NBCD
        else if (ird[11:8] == 4'b0000 || ird[11:8] == 4'b0010 || ird[11:8] == 4'b0100 || ird[11:8] == 4'b0110 || ird[11:8] == 4'b1010)
          sz = (s == 2'b11) ? 2'd1 : s;                                                 // NEGX/CLR/NEG/NOT/TST (11 = MOVE from/to SR/CCR: word)
        else if (ird[11:6] == 6'b100011 || ird[11:6] == 6'b110011) sz = 2'd2;           // MOVEM.L
        else if (ird[8:6] == 3'b110 || ird[11:9] == 3'b100 && ird[8:7] == 2'b11) sz = 2'd1;   // CHK, EXT: word
        else sz = 2'd1;
      end
      4'h5: sz = (s == 2'b11) ? 2'd0 : s;                                               // ADDQ/SUBQ; Scc = byte, DBcc: word (displacement)
      4'h6, 4'h7: sz = 2'd1;                                                            // Bcc (no operand), MOVEQ (long in the register, no access)
      4'h8, 4'h9, 4'hb, 4'hd: sz = (s == 2'b11) ? (ird[8] ? 2'd2 : 2'd1) : s;           // OR/SUB/CMP/ADD (11 = DIV/SUBA/CMPA/ADDA: word or long from bit 8)
      4'hc: sz = (s == 2'b11) ? 2'd1 : s;                                               // AND (11 = MUL word), ABCD = 00 byte; EXG: long (below, registers, no access)
      4'he: sz = (s == 2'b11) ? 2'd1 : s;                                               // shift: memory = word, register = from size
      default: sz = 2'd1;
    endcase
    if (line == 4'h5 && ird[7:3] == 5'b11001) sz = 2'd1;                                // DBcc
    if (line == 4'hc && (ird[8:3] == 6'b101000 || ird[8:3] == 6'b101001 || ird[8:3] == 6'b110001)) sz = 2'd2;   // EXG
    is_byte = (sz == 2'd0);
  end
endmodule
