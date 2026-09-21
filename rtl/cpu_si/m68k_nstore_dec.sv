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

// m68k_nstore_dec.sv - control store (micro + nano ROM) + per-resource nano decoder: the nanoword comes out decoded into one field per
// resource (dec, m68k_nano_dec_pkg.sv). Maintained by hand; slot masks, f_groups, f_drv, gq: diagnostic output of gen_bus_sec.py (GEN_DIAG_DIR), f_groups extended with the 15 read port selectors.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_nstore_dec #(
  parameter MEM_FILE    = "m68k_urom.mem",
  parameter ORIGIN_FILE = "m68k_norigin.mem",
  parameter NANO_FILE   = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",
  parameter bit ORG_OVR_LUT = 1'b1     // see m68k_nrom
) (
  input  logic                  clk,
  input  logic                  rst_n,
  input  logic                  en,
  input  logic [9:0]            a1, a2, a3,
  input  logic                  a0_sub,
  input  logic [9:0]            a0,
  input  logic                  a0s_sub,
  input  logic [9:0]            a0s,
  input  logic [1:0]            c1c0,
  output logic [9:0]            upc,
  output logic                  upc_is327,   // = (upc == 10'h327) from a flop (see m68k_ustore)
  output logic [16:0]           uw,
  output logic [NROM_W-1:0]     nw,
  output logic [14:0]           nwl_pre,   // 15 low bits of the next nanoword, one clock before en
  output logic [NROM_W-1:0]     nw_pre,
  output logic [NDEC_W-1:0] dec,
  output logic [26:0]           g,   // [11:0] selectors of the bus fabric, [26:12] selectors of the read ports of the bank (see f_groups)
  output logic [24:0]           gq,   // selectors of the non-RAW class fabric: flopped at every clock from dec_c (= the same function as dec_q)
  output logic [5:0]           gd,   // RAW half of the driven flags of the fabric: flopped at en from masks on nw_pre, like g
  output logic                  c_ft_q,   // access class f or t (dec[44:42] == 1 or 4), flopped at en from dec_p like dec_pq (RAW bits)
  output logic                  we_ab_h_q, we_ab_l_q, we_db_h_q, we_db_l_q,   // write enables of the bank: flopped at every clock from dec_c (non-RAW bits) = f(dec_q)
  // `psws+=+1` registered: in the decoder the rule is `nw[22 +: 5] == 5'd23 -> dec[159]`,
  // and dec[159] is in the RAW class (DEC_RAW[159] = 1), i.e. it comes from dec_pq, which is dec_c computed
  // on nw = nw_pre registered at en. Computing it here from nw_pre and registering it at en therefore gives the SAME
  // value in the SAME cycle, like g. It is needed because in m68k_psw_si that compare (set_s) would be
  // the first hop of the path dec -> set_s -> set_s_o -> sup_e -> bank index -> rf_db.
  // dec_p_o: the dec of the NEXT word, which already exists here (it is the input of the dec_pq flop).
  // It is needed by whoever must register a signal that is a pure function of dec: it is enough to evaluate its
  // same expression on dec_p and register it at en, without adding any decoding.
  output logic [NDEC_W-1:0] dec_p_o,
  output logic [NDEC_W-1:0] dec_pm_o, dec_po_o,   // the two branches of the override mux of dec_p (registered word / override)
  output logic                  ovr_p_o,   // the selector of that mux (st_ovr)
  output logic psws_p1,     // `psws+=+1` from the early word, registered at en (see above)
  output logic [NROM_IDX_W-1:0] nidx,
  output logic [9:0]            uaddr,
  output logic                  ir_load, fmt, bit4,
  output logic [1:0]            ty, fc,
  output logic [4:0]            cbc
);
  logic st_ovr, st_is3; logic [NROM_W-1:0] nw_pre_m, nw_pre_ovr;   // selectors and branches of the nw_pre mux, from the nano ROM
  m68k_ustore #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .ORG_OVR_LUT(ORG_OVR_LUT)) u_store (
    .clk(clk), .rst_n(rst_n), .en(en), .a1(a1), .a2(a2), .a3(a3), .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s),
    .c1c0(c1c0), .upc(upc), .upc_is327(upc_is327), .uw(uw), .nw(nw), .nwl_pre(nwl_pre), .nw_pre(nw_pre), .ovr_o(st_ovr), .is3_o(st_is3), .nw_pre_m(nw_pre_m), .nw_pre_ovr(nw_pre_ovr), .nidx(nidx), .uaddr(uaddr), .ir_load(ir_load), .fmt(fmt), .bit4(bit4),
    .ty(ty), .fc(fc), .cbc(cbc));
  // the dec bits of class C (DEC_RAW = 0: consumers at >= 2 pulses from en: bank writes, non-register bus sources,
  // ALU/AU, at/dob/ftu/psw/irc/ird) come out of a free-running registered copy (dec_q): the cone of every consumer starts from
  // a flop, not from the Tco of the nano M10K. The 78 bits of class A (indices of the register bank) and of class B (consumed
  // at ph0/S0/req) form the RAW class (DEC_RAW = 1) and come from dec_pq (see below). Mask derived from
  // docs/data/mc68000_nano_encoding.tsv and m68k_nano_dec_pkg.sv.
  localparam logic [NDEC_W-1:0] DEC_RAW = 199'b0000000000000001000000000011111111111101000000001000111111000011100011000110000101111111111000001000000000000000000011111110000000000000000000000000011111111100001100110000011111111111110000001111100;
  logic [NDEC_W-1:0] dec_c, dec_q, dec_p;
  m68k_nano_dec u_dec (.nw(nw), .dec(dec_c));
  always_ff @(posedge clk) dec_q <= dec_c;
  // DEC_PRE: the bits that drive bank indices and bus sections (49 bits, all in the RAW class, so they come from
  // dec_pq like the other RAW bits). It is not used by the logic: it documents that subset.
  localparam logic [NDEC_W-1:0] DEC_PRE = 199'b0000000000000000000000000011111111111100000000000000000000000000000011000110000001111111111000001000000000000000000000000000000000000000000000000000000000000000001100110000011111111111110000001111100;
  // the whole nanoword is known one clock earlier (nw_pre): it is decoded there and registered at en (dec_pq).
  // Same value at the same instant as decoding nw; no cone of the RAW class starts from the memory or from the decoder.
  logic [NDEC_W-1:0] dec_pq;
  localparam logic [NROM_W-1:0] NW_OVR_000 = {NROM_M_000, NROM_L_000[NROM_WL-1:0]};
  localparam logic [NROM_W-1:0] NW_OVR_003 = {NROM_M_003, NROM_L_003[NROM_WL-1:0]};
  // early decoding PRECOMPUTED on the override mux (like g, below): nw_pre = ovr ? (is3 ? NW_OVR_003 : NW_OVR_000) :
  // nw_pre_m, so nano_dec(nw_pre) = ovr ? (is3 ? nano_dec(NW_OVR_003) : nano_dec(NW_OVR_000)) : nano_dec(nw_pre_m). The two constant
  // decodings are folded by synthesis: after the decoding of the registered word ONE LUT per bit remains. Same value, same
  // edge. In the LUT branch of the testbenches (ORG_OVR_LUT) the override comes from a table: the word is decoded. Safety net below.
  logic [NDEC_W-1:0] dec_p_m, dec_p_ovr;
  m68k_nano_dec u_dec_pre (.nw(nw_pre_m), .dec(dec_p_m));
  generate if (ORG_OVR_LUT) begin : g_dec_ovr_lut
    m68k_nano_dec u_dec_ovr (.nw(nw_pre_ovr), .dec(dec_p_ovr));
  end else begin : g_dec_ovr_const
    logic [NDEC_W-1:0] dec_k000, dec_k003;
    m68k_nano_dec u_dec_k000 (.nw(NW_OVR_000), .dec(dec_k000));   // constant
    m68k_nano_dec u_dec_k003 (.nw(NW_OVR_003), .dec(dec_k003));   // constant
    assign dec_p_ovr = st_is3 ? dec_k003 : dec_k000;
  end endgenerate
  assign dec_p = st_ovr ? dec_p_ovr : dec_p_m;
`ifdef VERILATOR
  // SAFETY NET: the direct decoding of nw_pre, compared at every clock
  logic [NDEC_W-1:0] dec_p_chk;
  m68k_nano_dec u_dec_chk (.nw(nw_pre), .dec(dec_p_chk));
  always @(posedge clk) if (rst_n && dec_p !== dec_p_chk) begin
    $display("early decoding dec_p MISMATCH t=%0t ovr=%b is3=%b", $time, st_ovr, st_is3); $fatal(1);
  end
`endif
  assign dec_p_o = dec_p;   // dec of the next word (see the port)
  assign dec_pm_o = dec_p_m; assign dec_po_o = dec_p_ovr; assign ovr_p_o = st_ovr;   // the two branches and the selector of the dec_p mux
  always_ff @(posedge clk) begin
    if (!rst_n) dec_pq <= '0;
    else if (en) dec_pq <= dec_p;
  end
  always_ff @(posedge clk) begin if (!rst_n) c_ft_q <= 1'b0; else if (en) c_ft_q <= (dec_p[42 +: 3] == 3'd1) | (dec_p[42 +: 3] == 3'd4); end   // = (dec_pq[44:42] == f | t): same edge, same source
  assign dec = (dec_pq & DEC_RAW) | (dec_q & ~DEC_RAW);   // RAW class from the early decoding registered at en, the rest from dec_q
  // the wide selectors of m68k_bus_sec, as predicates on the fields of the nanoword:
  // on the dec bits they are ORs of many terms (two LUT levels and a cone that crosses two
  // entities); on the word they are a per-field membership test, i.e. two levels in all,
  // fewer than the three of the decoder. Equivalence proven on all the 512 nanowords.
  // Constant masks: bit k is 1 if the field equal to k belongs to the set.
  // 12 registered selectors in g[11:0]: sel_ab_d_1 sel_ab_h_0 sel_ab_h_2 sel_ab_h_3 sel_ab_h_7 sel_ab_l_0 sel_ab_l_4 sel_db_d_3 sel_db_h_0 sel_db_h_3 sel_db_l_0 sel_db_l_3
  // + 15 selectors of the read ports of the bank in g[26:12] (see below)
  localparam logic [31:0] M_AB_D_1_0 = 32'b00000000000000100000000000011110;   // field nw[0 +: 5] in {1,2,3,4,17}
  localparam logic [15:0] M_AB_D_1_1 = 16'b0000000000000010;   // field nw[5 +: 4] in {1}
  localparam logic [1:0] M_AB_D_1_4 = 2'b10;   // field nw[14 +: 1] in {1}
  localparam logic [31:0] M_AB_H_0_0 = 32'b00000000000000010110010000011110;   // field nw[0 +: 5] in {1,2,3,4,10,13,14,16}
  localparam logic [15:0] M_AB_H_0_1 = 16'b0000000000000110;   // field nw[5 +: 4] in {1,2}
  localparam logic [31:0] M_AB_H_2_7 = 32'b00000000000000000000000000100000;   // field nw[17 +: 5] in {5}
  localparam logic [31:0] M_AB_H_2_9 = 32'b00000000000000000000000000000100;   // field nw[27 +: 5] in {2}
  localparam logic [31:0] M_AB_H_3_0 = 32'b00000000000000001001101111100000;   // field nw[0 +: 5] in {5,6,7,8,9,11,12,15}
  localparam logic [31:0] M_AB_H_7_0 = 32'b00000000000011000000000000000000;   // field nw[0 +: 5] in {18,19}
  localparam logic [31:0] M_AB_L_0_0 = 32'b00000000000011010111110110111110;   // field nw[0 +: 5] in {1,2,3,4,5,7,8,10,11,12,13,14,16,18,19}
  localparam logic [15:0] M_AB_L_0_1 = 16'b0000000000000110;   // field nw[5 +: 4] in {1,2}
  localparam logic [31:0] M_AB_L_4_0 = 32'b00000000000000001000001001000000;   // field nw[0 +: 5] in {6,9,15}
  localparam logic [31:0] M_DB_D_3_0 = 32'b00000001000100000000000000000000;   // field nw[0 +: 5] in {20,24}
  localparam logic [15:0] M_DB_D_3_1 = 16'b0000010000000000;   // field nw[5 +: 4] in {10}
  localparam logic [31:0] M_DB_H_0_0 = 32'b00000000000100000000000000000000;   // field nw[0 +: 5] in {20}
  localparam logic [15:0] M_DB_H_0_1 = 16'b0000001001000000;   // field nw[5 +: 4] in {6,9}
  localparam logic [31:0] M_DB_H_3_0 = 32'b00000000111000000000000000000000;   // field nw[0 +: 5] in {21,22,23}
  localparam logic [15:0] M_DB_H_3_1 = 16'b0000000110111000;   // field nw[5 +: 4] in {3,4,5,7,8}
  localparam logic [31:0] M_DB_L_0_0 = 32'b00000010111100000000000000000000;   // field nw[0 +: 5] in {20,21,22,23,25}
  localparam logic [15:0] M_DB_L_0_1 = 16'b0000001011011000;   // field nw[5 +: 4] in {3,4,6,7,9}
  localparam logic [15:0] M_DB_L_3_1 = 16'b0000000100100000;   // field nw[5 +: 4] in {5,8}
  // the 15 selectors of the read ports of the register bank (sel_ab_rxd/rx/ryd/ry/pc/rxa/rya, sel_db_rxd/pc/rx/rxa/ry/
  // rya/sp/ryd of m68k_regfile_si), same recipe: every dec bit is `nw[slot] == code`, so the OR of bits is a
  // mask per slot. All the bits are of the RAW class (dec_pq), so the flop at en from nw_pre equals the SAME value in the
  // SAME cycle. Masks proven against m68k_nano_dec.sv (identity by construction on every field).
  localparam logic [31:0] M_AB_RXD_0 = 32'b00000000000000100000000000000010;   // field nw[0 +: 5] in {1,17}  (ab*<-(rxdl) abd<-(rxd))
  localparam logic [15:0] M_AB_RXD_1 = 16'b0000000000000010;   // field nw[5 +: 4] in {1}  (ab*<-(rxd))
  localparam logic [1:0] M_AB_RXD_4 = 2'b10;   // field nw[14 +: 1] in {1}  (abd<-(rxdl))
  localparam logic [31:0] M_AB_RX_0 = 32'b00000000000001000000011010000100;   // field nw[0 +: 5] in {2,7,9,10,18}  (ab*<-(rxl) ab<-(rx) ab<-(rxh) ab<-(rxl) abe<-(rxl))
  localparam logic [31:0] M_AB_RYD_0 = 32'b00000000000000000110000000001000;   // field nw[0 +: 5] in {3,13,14}  (ab*<-(rydl) ab<-(ryd) ab<-(rydl))
  localparam logic [31:0] M_AB_RY_0 = 32'b00000000000010011000100000010000;   // field nw[0 +: 5] in {4,11,15,16,19}  (ab*<-(ryl) ab<-(ry) ab<-(ryh) ab<-(ryl) abe<-(ryl))
  localparam logic [31:0] M_AB_PC_0 = 32'b00000000000000000000000001100000;   // field nw[0 +: 5] in {5,6}  (ab<-(pc) ab<-(pch))
  localparam logic [15:0] M_AB_PC_1 = 16'b0000000000000100;   // field nw[5 +: 4] in {2}  (ab<-(pcl))
  localparam logic [31:0] M_AB_RXA_0 = 32'b00000000000000000000000100000000;   // field nw[0 +: 5] in {8}  (ab<-(rxa))
  localparam logic [31:0] M_AB_RYA_0 = 32'b00000000000000000001000000000000;   // field nw[0 +: 5] in {12}  (ab<-(rya))
  localparam logic [31:0] M_DB_RXD_0 = 32'b00000000000100000000000000000000;   // field nw[0 +: 5] in {20}  (db*<-(rxdl))
  localparam logic [15:0] M_DB_RXD_1 = 16'b0000110000000000;   // field nw[5 +: 4] in {10,11}  (dbd<-(rxdl) dbe<-(rxdl))
  localparam logic [15:0] M_DB_PC_1 = 16'b0000000000001000;   // field nw[5 +: 4] in {3}  (db<-(pc))
  localparam logic [31:0] M_DB_RX_0 = 32'b00000010001000000000000000000000;   // field nw[0 +: 5] in {21,25}  (db<-(rx) dbe<-(rxl))
  localparam logic [15:0] M_DB_RX_1 = 16'b0000000001100000;   // field nw[5 +: 4] in {5,6}  (db<-(rxh) db<-(rxl))
  localparam logic [15:0] M_DB_RXA_1 = 16'b0000000000010000;   // field nw[5 +: 4] in {4}  (db<-(rxa))
  localparam logic [15:0] M_DB_RY_1 = 16'b0000001110000000;   // field nw[5 +: 4] in {7,8,9}  (db<-(ry) db<-(ryh) db<-(ryl))
  localparam logic [31:0] M_DB_RYA_0 = 32'b00000000010000000000000000000000;   // field nw[0 +: 5] in {22}  (db<-(rya))
  localparam logic [31:0] M_DB_SP_0 = 32'b00000000100000000000000000000000;   // field nw[0 +: 5] in {23}  (db<-(sp))
  localparam logic [31:0] M_DB_RYD_0 = 32'b00000001000000000000000000000000;   // field nw[0 +: 5] in {24}  (dbd<-(rydl))
  function automatic logic [26:0] f_groups (input logic [NROM_W-1:0] w);
    f_groups[0] = M_AB_D_1_0[w[0 +: 5]] | M_AB_D_1_1[w[5 +: 4]] | M_AB_D_1_4[w[14 +: 1]];   // sel_ab_d_1
    f_groups[1] = M_AB_H_0_0[w[0 +: 5]] | M_AB_H_0_1[w[5 +: 4]];   // sel_ab_h_0
    f_groups[2] = M_AB_H_2_7[w[17 +: 5]] | M_AB_H_2_9[w[27 +: 5]];   // sel_ab_h_2
    f_groups[3] = M_AB_H_3_0[w[0 +: 5]];   // sel_ab_h_3
    f_groups[4] = M_AB_H_7_0[w[0 +: 5]];   // sel_ab_h_7
    f_groups[5] = M_AB_L_0_0[w[0 +: 5]] | M_AB_L_0_1[w[5 +: 4]];   // sel_ab_l_0
    f_groups[6] = M_AB_L_4_0[w[0 +: 5]];   // sel_ab_l_4
    f_groups[7] = M_DB_D_3_0[w[0 +: 5]] | M_DB_D_3_1[w[5 +: 4]];   // sel_db_d_3
    f_groups[8] = M_DB_H_0_0[w[0 +: 5]] | M_DB_H_0_1[w[5 +: 4]];   // sel_db_h_0
    f_groups[9] = M_DB_H_3_0[w[0 +: 5]] | M_DB_H_3_1[w[5 +: 4]];   // sel_db_h_3
    f_groups[10] = M_DB_L_0_0[w[0 +: 5]] | M_DB_L_0_1[w[5 +: 4]];   // sel_db_l_0
    f_groups[11] = M_DB_L_3_1[w[5 +: 4]];   // sel_db_l_3
    f_groups[12] = M_AB_RXD_0[w[0 +: 5]] | M_AB_RXD_1[w[5 +: 4]] | M_AB_RXD_4[w[14 +: 1]];   // sel_ab_rxd: ab*<-(rxd) ab*<-(rxdl) abd<-(rxd) abd<-(rxdl)
    f_groups[13] = M_AB_RX_0[w[0 +: 5]];   // sel_ab_rx: ab*<-(rxl) ab<-(rx) ab<-(rxh) ab<-(rxl) abe<-(rxl)
    f_groups[14] = M_AB_RYD_0[w[0 +: 5]];   // sel_ab_ryd: ab*<-(rydl) ab<-(ryd) ab<-(rydl)
    f_groups[15] = M_AB_RY_0[w[0 +: 5]];   // sel_ab_ry: ab*<-(ryl) ab<-(ry) ab<-(ryh) ab<-(ryl) abe<-(ryl)
    f_groups[16] = M_AB_PC_0[w[0 +: 5]] | M_AB_PC_1[w[5 +: 4]];   // sel_ab_pc: ab<-(pc) ab<-(pch) ab<-(pcl)
    f_groups[17] = M_AB_RXA_0[w[0 +: 5]];   // sel_ab_rxa: ab<-(rxa)
    f_groups[18] = M_AB_RYA_0[w[0 +: 5]];   // sel_ab_rya: ab<-(rya)
    f_groups[19] = M_DB_RXD_0[w[0 +: 5]] | M_DB_RXD_1[w[5 +: 4]];   // sel_db_rxd: db*<-(rxdl) dbd<-(rxdl) dbe<-(rxdl)
    f_groups[20] = M_DB_PC_1[w[5 +: 4]];   // sel_db_pc: db<-(pc)
    f_groups[21] = M_DB_RX_0[w[0 +: 5]] | M_DB_RX_1[w[5 +: 4]];   // sel_db_rx: db<-(rx) db<-(rxh) db<-(rxl) dbe<-(rxl)
    f_groups[22] = M_DB_RXA_1[w[5 +: 4]];   // sel_db_rxa: db<-(rxa)
    f_groups[23] = M_DB_RY_1[w[5 +: 4]];   // sel_db_ry: db<-(ry) db<-(ryh) db<-(ryl)
    f_groups[24] = M_DB_RYA_0[w[0 +: 5]];   // sel_db_rya: db<-(rya)
    f_groups[25] = M_DB_SP_0[w[0 +: 5]];   // sel_db_sp: db<-(sp)
    f_groups[26] = M_DB_RYD_0[w[0 +: 5]];   // sel_db_ryd: dbd<-(rydl)
  endfunction
  // 6 RAW halves of the driven flags (masks on the slots, like f_groups): drv_ab_d_r drv_ab_l_r drv_ab_h_r drv_db_d_r drv_db_l_r drv_db_h_r
  localparam logic [31:0] M_DRV_AB_D_R_0 = 32'b00000000000000100000000000011110;   // field nw[0 +: 5] in {1,2,3,4,17}
  localparam logic [15:0] M_DRV_AB_D_R_1 = 16'b0000000000000010;   // field nw[5 +: 4] in {1}
  localparam logic [1:0] M_DRV_AB_D_R_4 = 2'b10;   // field nw[14 +: 1] in {1}
  localparam logic [31:0] M_DRV_AB_L_R_0 = 32'b00000000000011011111111111111110;   // field nw[0 +: 5] in {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,19}
  localparam logic [15:0] M_DRV_AB_L_R_1 = 16'b0000000000000110;   // field nw[5 +: 4] in {1,2}
  localparam logic [31:0] M_DRV_AB_H_R_0 = 32'b00000000000011011111111111111110;   // field nw[0 +: 5] in {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,19}
  localparam logic [15:0] M_DRV_AB_H_R_1 = 16'b0000000000000110;   // field nw[5 +: 4] in {1,2}
  localparam logic [31:0] M_DRV_AB_H_R_7 = 32'b00000000000000000000000000100000;   // field nw[17 +: 5] in {5}
  localparam logic [31:0] M_DRV_AB_H_R_9 = 32'b00000000000000000000000000000100;   // field nw[27 +: 5] in {2}
  localparam logic [31:0] M_DRV_DB_D_R_0 = 32'b00000001000100000000000000000000;   // field nw[0 +: 5] in {20,24}
  localparam logic [15:0] M_DRV_DB_D_R_1 = 16'b0000010000000000;   // field nw[5 +: 4] in {10}
  localparam logic [31:0] M_DRV_DB_L_R_0 = 32'b00000010111100000000000000000000;   // field nw[0 +: 5] in {20,21,22,23,25}
  localparam logic [15:0] M_DRV_DB_L_R_1 = 16'b0000101111111000;   // field nw[5 +: 4] in {3,4,5,6,7,8,9,11}
  localparam logic [3:0] M_DRV_DB_L_R_20 = 4'b1000;   // field nw[66 +: 2] in {3}
  localparam logic [31:0] M_DRV_DB_H_R_0 = 32'b00000010111100000000000000000000;   // field nw[0 +: 5] in {20,21,22,23,25}
  localparam logic [15:0] M_DRV_DB_H_R_1 = 16'b0000101111111000;   // field nw[5 +: 4] in {3,4,5,6,7,8,9,11}
  localparam logic [3:0] M_DRV_DB_H_R_20 = 4'b1000;   // field nw[66 +: 2] in {3}
  function automatic logic [5:0] f_drv (input logic [NROM_W-1:0] w);
    f_drv[0] = M_DRV_AB_D_R_0[w[0 +: 5]] | M_DRV_AB_D_R_1[w[5 +: 4]] | M_DRV_AB_D_R_4[w[14 +: 1]];   // drv_ab_d_r
    f_drv[1] = M_DRV_AB_L_R_0[w[0 +: 5]] | M_DRV_AB_L_R_1[w[5 +: 4]];   // drv_ab_l_r
    f_drv[2] = M_DRV_AB_H_R_0[w[0 +: 5]] | M_DRV_AB_H_R_1[w[5 +: 4]] | M_DRV_AB_H_R_7[w[17 +: 5]] | M_DRV_AB_H_R_9[w[27 +: 5]];   // drv_ab_h_r
    f_drv[3] = M_DRV_DB_D_R_0[w[0 +: 5]] | M_DRV_DB_D_R_1[w[5 +: 4]];   // drv_db_d_r
    f_drv[4] = M_DRV_DB_L_R_0[w[0 +: 5]] | M_DRV_DB_L_R_1[w[5 +: 4]] | M_DRV_DB_L_R_20[w[66 +: 2]];   // drv_db_l_r
    f_drv[5] = M_DRV_DB_H_R_0[w[0 +: 5]] | M_DRV_DB_H_R_1[w[5 +: 4]] | M_DRV_DB_H_R_20[w[66 +: 2]];   // drv_db_h_r
  endfunction
  // g PRECOMPUTED on the override mux of the nano ROM. g = f_groups(nw_pre) registered at en, with
  //   nw_pre = ovr ? (addr==003 ? NW_OVR_003 : NW_OVR_000) : {qm_pre, ql_pre}  (m68k_nrom, constants branch of the core).
  // A function distributed over a mux is the same function: g <= ovr ? (is3 ? f(NW_OVR_003) : f(NW_OVR_000)) : f({qm_pre, ql_pre}).
  // f(NW_OVR_000) and f(NW_OVR_003) are constants; f({qm_pre, ql_pre}) starts from registers (ROM read one clock earlier). On the
  // path psw.s -> a0 -> effective address -> override compare -> g only the compare and ONE LUT per bit remain after the
  // address. Same value, same en edge. Safety net below.
  logic [26:0] g_000, g_003, g_m, g_ovr;
  assign g_000 = f_groups(NW_OVR_000);   // constant inputs: synthesis folds them to constants
  assign g_003 = f_groups(NW_OVR_003);
  assign g_m   = f_groups(nw_pre_m);
  generate if (ORG_OVR_LUT) begin : g_sel_ovr_lut
    assign g_ovr = f_groups(nw_pre_ovr);   // testbenches: the override comes from a table, g from the word
  end else begin : g_sel_ovr_const
    assign g_ovr = st_is3 ? g_003 : g_000;
  end endgenerate
`ifdef VERILATOR
  // SAFETY NET: f_groups(nw_pre), registered at the same edge, compared at every clock
  logic [26:0] g_chk;
  always_ff @(posedge clk) begin
    if (!rst_n) g_chk <= f_groups('0);
    else if (en) g_chk <= f_groups(nw_pre);
  end
  always @(posedge clk) if (rst_n && g !== g_chk) begin
    $display("registered selectors g MISMATCH t=%0t g=%b expected=%b ovr=%b is3=%b", $time, g, g_chk, st_ovr, st_is3); $fatal(1);
  end
`endif
  always_ff @(posedge clk) begin
    if (!rst_n) g <= f_groups('0);   // at word zero they are 0, like dec_pq at reset
    else if (en) g <= st_ovr ? g_ovr : g_m;   // = f_groups(nw_pre), see above
  end
  // RAW half of the driven flags: same recipe as g (precomputed on the override mux), safety net below
  logic [5:0] gd_000, gd_003, gd_m, gd_ovr;
  assign gd_000 = f_drv(NW_OVR_000); assign gd_003 = f_drv(NW_OVR_003); assign gd_m = f_drv(nw_pre_m);
  generate if (ORG_OVR_LUT) begin : g_drv_ovr_lut
    assign gd_ovr = f_drv(nw_pre_ovr);
  end else begin : g_drv_ovr_const
    assign gd_ovr = st_is3 ? gd_003 : gd_000;
  end endgenerate
  always_ff @(posedge clk) begin
    if (!rst_n) gd <= f_drv('0);
    else if (en) gd <= st_ovr ? gd_ovr : gd_m;
  end
`ifdef VERILATOR
  logic [5:0] gd_chk;
  always_ff @(posedge clk) begin
    if (!rst_n) gd_chk <= f_drv('0);
    else if (en) gd_chk <= f_drv(nw_pre);
  end
  always @(posedge clk) if (rst_n && gd !== gd_chk) begin
    $display("driven flags gd MISMATCH t=%0t gd=%b expected=%b", $time, gd, gd_chk); $fatal(1);
  end
`endif
  // selectors of the NON-RAW class fabric (dec[k] = dec_q[k] = dec_c[k] of one clock earlier): the OR of the
  // conditions computed on dec_c and registered AT EVERY CLOCK equals exactly the OR on dec_q at every clock. Same style as
  // dec_q (no en, no reset: it is the same one-clock pipeline), power-up at 0 like dec_q. Safety net below.
  logic [24:0] gq_c;
  assign gq_c[0] = (dec_c[1]) | (dec_c[12]) | (dec_c[30]);   // sel_ab_d_0
  assign gq_c[1] = (dec_c[7]) | (dec_c[26]) | (dec_c[33]);   // sel_ab_d_2
  assign gq_c[2] = (dec_c[8]) | (dec_c[29]) | (dec_c[38]);   // sel_ab_l_1
  assign gq_c[3] = (dec_c[34]) | (dec_c[40 +: 1] == 1'd1);   // sel_ab_l_7
  assign gq_c[4] = (dec_c[99]) | (dec_c[104]) | (dec_c[121]);   // sel_db_d_0
  assign gq_c[5] = (dec_c[100]) | (dec_c[122]);   // sel_db_d_1
  assign gq_c[6] = (dec_c[101]) | (dec_c[123]);   // sel_db_d_2
  assign gq_c[7] = (dec_c[103]) | (dec_c[118]) | (dec_c[126]);   // sel_db_d_4
  assign gq_c[8] = (dec_c[106]) | (dec_c[133 +: 1] == 1'd1);   // sel_db_h_2
  assign gq_c[9] = (dec_c[107]) | (dec_c[137 +: 2] == 2'd1);   // sel_db_l_2
  assign gq_c[10] = (dec_c[120]) | (dec_c[132]);   // sel_db_l_5
  assign gq_c[11] = (dec_c[128]) | (dec_c[137 +: 2] == 2'd2);   // sel_db_l_7
  assign gq_c[12] = ((dec_c[1]) | (dec_c[12]) | (dec_c[30])) | ((dec_c[7]) | (dec_c[26]) | (dec_c[33]));   // drv_ab_d_q
  assign gq_c[13] = ((dec_c[8]) | (dec_c[29]) | (dec_c[38])) | ((dec_c[9])) | ((dec_c[11])) | ((dec_c[27])) | ((dec_c[28])) | ((dec_c[34]) | (dec_c[40 +: 1] == 1'd1)) | ((dec_c[37]));   // drv_ab_l_q
  assign gq_c[14] = ((dec_c[9])) | ((dec_c[27])) | ((dec_c[28])) | ((dec_c[34])) | ((dec_c[37])) | ((dec_c[38]));   // drv_ab_h_q
  assign gq_c[15] = ((dec_c[99]) | (dec_c[104]) | (dec_c[121])) | ((dec_c[100]) | (dec_c[122])) | ((dec_c[101]) | (dec_c[123])) | ((dec_c[103]) | (dec_c[118]) | (dec_c[126]));   // drv_db_d_q
  assign gq_c[16] = ((dec_c[105])) | ((dec_c[107]) | (dec_c[137 +: 2] == 2'd1)) | ((dec_c[120]) | (dec_c[132])) | ((dec_c[127])) | ((dec_c[128]) | (dec_c[137 +: 2] == 2'd2)) | ((dec_c[131]));   // drv_db_l_q
  assign gq_c[17] = ((dec_c[105])) | ((dec_c[106]) | (dec_c[133 +: 1] == 1'd1)) | ((dec_c[127])) | ((dec_c[128])) | ((dec_c[131])) | ((dec_c[132]));   // drv_db_h_q
  assign gq_c[18] = (dec_c[39 +: 1] == 1'd1) | (dec_c[10]);   // sel_ab_wh_7
  assign gq_c[19] = (dec_c[7]) | (dec_c[26]);   // sel_ab_wl_1
  assign gq_c[20] = (dec_c[12]) | (dec_c[34]) | (dec_c[40 +: 1] == 1'd1);   // sel_ab_wl_2
  assign gq_c[21] = (dec_c[28]) & ~((dec_c[7]));   // sel_ab_wl_3
  assign gq_c[22] = (dec_c[38]) | (dec_c[8]);   // sel_ab_wl_4
  assign gq_c[23] = (dec_c[121]) | (dec_c[127]);   // sel_db_wl_0
  assign gq_c[24] = (dec_c[118]) | (dec_c[126]);   // sel_db_wl_2
  always_ff @(posedge clk) gq <= gq_c;
  // write enables of the register bank from flops: same expressions as the bank (WE_DEC_ONLY form) on dec_c, all non-RAW
  // bits (dec[x] = dec_q[x] = dec_c[x] of the previous clock): exact at every clock like gq. Reset to 0 (= word zero).
  always_ff @(posedge clk) begin
    if (!rst_n) begin we_ab_h_q <= 1'b0; we_ab_l_q <= 1'b0; we_db_h_q <= 1'b0; we_db_l_q <= 1'b0; end
    else begin
      we_ab_l_q <= ((((dec_c[173 +: 2] == 2'd1) | (dec_c[173 +: 2] == 2'd2) | (dec_c[175 +: 2] == 2'd1) | (dec_c[177 +: 2] == 2'd2) | (dec_c[181 +: 2] == 2'd1) | (dec_c[184 +: 2] == 2'd1) | (dec_c[184 +: 2] == 2'd2) | (dec_c[186 +: 2] == 2'd1) | (dec_c[192 +: 2] == 2'd1) | (dec_c[192 +: 2] == 2'd2) | (dec_c[196 +: 1] == 1'd1))) | (((dec_c[177 +: 2] == 2'd1) | (dec_c[188 +: 2] == 2'd1) | (dec_c[188 +: 2] == 2'd2))));
      we_ab_h_q <= ((((dec_c[173 +: 2] == 2'd1) | (dec_c[173 +: 2] == 2'd2) | (dec_c[175 +: 2] == 2'd1) | (dec_c[184 +: 2] == 2'd1) | (dec_c[184 +: 2] == 2'd2) | (dec_c[186 +: 2] == 2'd1))) | (((dec_c[179 +: 2] == 2'd1) | (dec_c[190 +: 2] == 2'd1) | (dec_c[190 +: 2] == 2'd2) | (dec_c[195 +: 1] == 1'd1))));
      we_db_l_q <= ((((dec_c[173 +: 2] == 2'd3) | (dec_c[175 +: 2] == 2'd2) | (dec_c[181 +: 2] == 2'd2) | (dec_c[184 +: 2] == 2'd3) | (dec_c[186 +: 2] == 2'd2) | (dec_c[192 +: 2] == 2'd3) | (dec_c[194 +: 1] == 1'd1))) | (((dec_c[177 +: 2] == 2'd3) | (dec_c[188 +: 2] == 2'd3))));
      we_db_h_q <= ((((dec_c[173 +: 2] == 2'd3) | (dec_c[175 +: 2] == 2'd2) | (dec_c[184 +: 2] == 2'd3) | (dec_c[186 +: 2] == 2'd2) | (dec_c[194 +: 1] == 1'd1))) | (((dec_c[179 +: 2] == 2'd2) | (dec_c[190 +: 2] == 2'd3))));
    end
  end
`ifdef VERILATOR
  // SAFETY NET: the OR of the conditions on dec, compared at every clock
  logic [24:0] gq_v;
  assign gq_v[0] = (dec[1]) | (dec[12]) | (dec[30]);   // sel_ab_d_0
  assign gq_v[1] = (dec[7]) | (dec[26]) | (dec[33]);   // sel_ab_d_2
  assign gq_v[2] = (dec[8]) | (dec[29]) | (dec[38]);   // sel_ab_l_1
  assign gq_v[3] = (dec[34]) | (dec[40 +: 1] == 1'd1);   // sel_ab_l_7
  assign gq_v[4] = (dec[99]) | (dec[104]) | (dec[121]);   // sel_db_d_0
  assign gq_v[5] = (dec[100]) | (dec[122]);   // sel_db_d_1
  assign gq_v[6] = (dec[101]) | (dec[123]);   // sel_db_d_2
  assign gq_v[7] = (dec[103]) | (dec[118]) | (dec[126]);   // sel_db_d_4
  assign gq_v[8] = (dec[106]) | (dec[133 +: 1] == 1'd1);   // sel_db_h_2
  assign gq_v[9] = (dec[107]) | (dec[137 +: 2] == 2'd1);   // sel_db_l_2
  assign gq_v[10] = (dec[120]) | (dec[132]);   // sel_db_l_5
  assign gq_v[11] = (dec[128]) | (dec[137 +: 2] == 2'd2);   // sel_db_l_7
  assign gq_v[12] = ((dec[1]) | (dec[12]) | (dec[30])) | ((dec[7]) | (dec[26]) | (dec[33]));   // drv_ab_d_q
  assign gq_v[13] = ((dec[8]) | (dec[29]) | (dec[38])) | ((dec[9])) | ((dec[11])) | ((dec[27])) | ((dec[28])) | ((dec[34]) | (dec[40 +: 1] == 1'd1)) | ((dec[37]));   // drv_ab_l_q
  assign gq_v[14] = ((dec[9])) | ((dec[27])) | ((dec[28])) | ((dec[34])) | ((dec[37])) | ((dec[38]));   // drv_ab_h_q
  assign gq_v[15] = ((dec[99]) | (dec[104]) | (dec[121])) | ((dec[100]) | (dec[122])) | ((dec[101]) | (dec[123])) | ((dec[103]) | (dec[118]) | (dec[126]));   // drv_db_d_q
  assign gq_v[16] = ((dec[105])) | ((dec[107]) | (dec[137 +: 2] == 2'd1)) | ((dec[120]) | (dec[132])) | ((dec[127])) | ((dec[128]) | (dec[137 +: 2] == 2'd2)) | ((dec[131]));   // drv_db_l_q
  assign gq_v[17] = ((dec[105])) | ((dec[106]) | (dec[133 +: 1] == 1'd1)) | ((dec[127])) | ((dec[128])) | ((dec[131])) | ((dec[132]));   // drv_db_h_q
  assign gq_v[18] = (dec[39 +: 1] == 1'd1) | (dec[10]);   // sel_ab_wh_7
  assign gq_v[19] = (dec[7]) | (dec[26]);   // sel_ab_wl_1
  assign gq_v[20] = (dec[12]) | (dec[34]) | (dec[40 +: 1] == 1'd1);   // sel_ab_wl_2
  assign gq_v[21] = (dec[28]) & ~((dec[7]));   // sel_ab_wl_3
  assign gq_v[22] = (dec[38]) | (dec[8]);   // sel_ab_wl_4
  assign gq_v[23] = (dec[121]) | (dec[127]);   // sel_db_wl_0
  assign gq_v[24] = (dec[118]) | (dec[126]);   // sel_db_wl_2
  always @(posedge clk) if (rst_n && gq !== gq_v) begin
    $display("non-RAW selectors gq MISMATCH t=%0t gq=%b expected=%b", $time, gq, gq_v); $fatal(1);
  end
`endif
  always_ff @(posedge clk) begin
    if (!rst_n) psws_p1 <= 1'b0;   // at word zero it is 0, like dec_pq at reset
    else if (en) psws_p1 <= dec_p[159];   // = (nw_pre[22 +: 5] == 5'd23), from the precomputed decoding dec_p
  end
endmodule
