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

// m68k_cpu_si.sv - silicon-structured MC68000 processor: all the blocks of rtl/cpu_si connected. Only the processor: active-high
// reset, the two clock enables cen/cen2 from outside (the clock-enable/DTACK generator is joined to it in m68k_cpu_dt_si), ipl_n,
// iack, E, HALT/arbitration, dbg_pc. E = CPU clock / 10, counted on the phi2 phases (e_cnt 0..9, E high for e_cnt >= 6, like the bus
// FSM). HALT, BR/BG/BGACK arbitration and the double bus fault (dfault, halt) in the busctl block.
// Structure (patent FIG. 4): control store (micro ROM + selector + nano ROM + decoder) -> microcycle/bus sequencer ->
// sectioned datapath (AB/DB fabric, AT/AOB/DOB/DBIN, AU, register bank + MOVEM encoder, ALU with surrounding logic, FTU, PSW),
// IRC/IR/IRD queue -> A1/A2/A3 decoder, A0/A0S exception block, C1C0 branch unit.
import m68k_nano_pkg::*; import m68k_nano_dec_pkg::*;
module m68k_cpu_si #(
  // 1 = the bus chain in a single hierarchy node (m68k_busdp_si); 0 = the three parts flat.
  parameter bit BUSDP_NODE = 1'b0,
  parameter MEM_FILE    = "m68k_urom.mem",
  parameter ORIGIN_FILE = "m68k_norigin.mem",
  parameter NANO_FILE   = "m68k_nrom.mem", parameter NANO_M_FILE = "m68k_nrom_m.mem",
  parameter bit DBG     = 1'b0          // register read ports for testbenches/lockstep (0 in synthesis)
  , parameter bit RST_REG = 1'b0        // 1 = three registered copies of the reset with local fan-out (the node passes the reset NOT registered); 0 = one wire
) (
  input  logic        clk,
  input  logic        reset,              // system reset (active high)
  input  logic        cen, cen2,          // alternating phi1/phi2 phases (2-phase dtack)
  // external bus
  output logic [23:1] addr,
  output logic        as_n, uds_n, lds_n, rw,
  output logic        data_oe,
  input  logic [15:0] data_in,
  output logic [15:0] data_out,
  input  logic        dtack_n,
  output logic [2:0]  fc,
  input  logic [2:0]  ipl_n,              // IRQ level (active low)
  output logic        iack,               // 1 during the interrupt acknowledge cycle
  input  logic        vpa_n,
  output logic        vma_n,
  output logic        e,                  // Enable clock (clk/10)
  input  logic        berr_n,
  input  logic        halt_n,
  output logic        halted_n,
  input  logic        br_n,
  output logic        bg_n,
  input  logic        bgack_n,
  output logic        bus_tri,
  output logic        reset_o,            // RESET instruction: the RESET pin asserted for 124 clocks (MC68000UM 5.5 p. 5-29)
  output logic [23:0] dbg_pc,
  // observability
  output logic [9:0]  upc,
  output logic [15:0] ird_o, psw_o,
  output logic [31:0] pc_o, au_o, aob_o, at_o,
  output logic [15:0] dbg_alu_a, dbg_alu_d, dbg_alu_r,   // debug (CPU testbench): ALU
  input  logic [2:0]  dbg_rd_dnum, dbg_rd_anum,   // debug: read of Dn / An (A7 = active SP)
  output logic [31:0] dbg_rd_ddata, dbg_rd_adata,
  output logic        dbg_instr_done,     // debug: dispatch of a new instruction (ird<-(ir) at en)
  output logic        dbg_illegal         // debug: sticky, illegal instruction exception taken
);
  // reset distribution: with RST_REG three registered copies (preserve/dont_merge, power-up in reset) with
  // local fan-out: control store (a), register bank + bus + MOVEM (b), rest (c). The wrapper (m68k_cpu_dt_si, RST_REG = 1)
  // then passes the reset NOT registered, so the copies equal exactly its rst_q (no extra clock). With RST_REG = 0
  // (default, testbenches) the reset is one wire, used directly by all the blocks.
  logic rst_n, rst_n_a, rst_n_b;
  generate if (RST_REG) begin : g_rst_copies
    (* preserve *) (* dont_merge *) logic rst_q_a = 1'b1, rst_q_b = 1'b1, rst_q_c = 1'b1;
    always_ff @(posedge clk) begin rst_q_a <= reset; rst_q_b <= reset; rst_q_c <= reset; end
    assign rst_n_a = ~rst_q_a; assign rst_n_b = ~rst_q_b; assign rst_n = ~rst_q_c;
  end else begin : g_rst_wire
    assign rst_n_a = ~reset; assign rst_n_b = ~reset; assign rst_n = ~reset;
  end endgenerate
  logic [2:0] ipl; assign ipl = ~ipl_n;
  logic [4:0] e_cnt;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) e_cnt <= 5'd0;
    else if (cen2) e_cnt <= (e_cnt == 5'd9) ? 5'd0 : e_cnt + 5'd1;
  end
  assign e = (e_cnt >= 5'd6);
  assign iack = (fc == 3'b111) & ~as_n;
  // ---------------- control store
  logic [9:0] a1, a2, a3, a0, a0s, uaddr;
  logic a0_sub, a0s_sub; logic [1:0] c1c0;
  logic [14:0] nwl_pre;   // early low word: from the control store to the register bank
  logic [NROM_W-1:0] nw_pre;   // whole nanoword one clock earlier
  logic [26:0] g_sel; logic [24:0] gq_sel; logic [5:0] gd_sel;   // g: [11:0] bus fabric selectors, [26:12] the 15 read port selectors of the bank, from the early word, registered at en; gq: non-RAW fabric selectors, flopped at every clock; gd: RAW halves of the driven flags, registered at en
  logic [16:0] uw; logic [NROM_W-1:0] nw; logic [NROM_IDX_W-1:0] nidx;
  logic ir_load, fmt, bit4; logic [1:0] ty, fc_micro; logic [4:0] cbc;
  logic [NDEC_W-1:0] dec;
  logic en, en_rf, c_ft_q;   // en_rf: copy of en for the write of the register bank; c_ft_q: class f|t from a flop
  logic we_ab_h_q, we_ab_l_q, we_db_h_q, we_db_l_q;   // write enables of the bank from flops (decoder -> bank)
  logic psws_p1;   // `psws+=+1` registered by the control store
  logic [NDEC_W-1:0] dec_p;   // the dec of the next word, from the control store
  logic [3:0] rp_next; logic sup_next;   // next values of the MOVEM index and of the effective S bit
  logic [NDEC_W-1:0] dec_pm, dec_po; logic ovr_p;   // branches and selector of the override mux of dec_p, for e_* of the bank
  logic upc_is327;   // (upc == 327) from a flop of the control store: keeps the 10-bit compare off the head of the a0 cone
  m68k_nstore_dec #(.MEM_FILE(MEM_FILE), .ORIGIN_FILE(ORIGIN_FILE), .NANO_FILE(NANO_FILE), .NANO_M_FILE(NANO_M_FILE), .ORG_OVR_LUT(1'b0)) u_cs (   /* a0s override from constants */
    .clk(clk), .rst_n(rst_n_a), .en(en), .a1(a1), .a2(a2), .a3(a3), .a0_sub(a0_sub), .a0(a0), .a0s_sub(a0s_sub), .a0s(a0s),
    .c_ft_q(c_ft_q), .we_ab_h_q(we_ab_h_q), .we_ab_l_q(we_ab_l_q), .we_db_h_q(we_db_h_q), .we_db_l_q(we_db_l_q), .c1c0(c1c0), .upc(upc), .upc_is327(upc_is327), .uw(uw), .nw(nw), .nwl_pre(nwl_pre), .nw_pre(nw_pre),  .dec(dec), .g(g_sel), .gq(gq_sel), .gd(gd_sel), .psws_p1(psws_p1), .dec_p_o(dec_p), .dec_pm_o(dec_pm), .dec_po_o(dec_po), .ovr_p_o(ovr_p), .nidx(nidx), .uaddr(uaddr),
    .ir_load(ir_load), .fmt(fmt), .bit4(bit4), .ty(ty), .fc(fc_micro), .cbc(cbc));
  // ---------------- registers and datapath
  // the current word loads ird <- ir: used by the registered operand size (sz), ALU row, ccr_only and instruction vector [declared here because instances that come earlier consume it]
  logic ld_ird_c; assign ld_ird_c = (dec[ND_IRD_FROM_LSB +: ND_IRD_FROM_W] == ND_IRD_FROM__P_IR[ND_IRD_FROM_W-1:0]);
  logic [15:0] psw; logic sup; logic [4:0] cc;
  logic [31:0] at, aob, au, aob_n;
  // the value that the DOB takes from the AB bus: in the flat branch it is born here, in the node branch
  // inside m68k_busdp_si. A single wire, with only the 5 sources that the census of the nanowords allows.
  logic [15:0] ab_dob; logic ab_dob_drv;
  logic [15:0] ab_wl, ab_wh, db_wl, db_wh;   // narrow wires of the write data of the bank, from the fabric to the write ports
  // the 23 flops of the address on the pins (addr). Their only consumer is outside the processor, so they live
  // here, at the top of the processor, and not inside the busctl: the busctl gives only the load enable
  // (addr_ld_o). Data aob_n[23:1], enable addr_ld_o, reset to 0: the address is loaded from the AOB
  // at S0 or S1 of the bus cycle (addr_ld in m68k_busctl_si) and stays on the pins until the next
  // load.
  logic addr_ld_o; logic [23:1] addr_r;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) addr_r <= '0;
    else if (addr_ld_o) addr_r <= aob_n[23:1];
  end
  assign addr = addr_r;
  logic [15:0] dob, dbin, ftu_q, edb, dob_n, dbin_n;
  // DBIN strobe in the die: a 't' word takes the data of the previous access (delayed edb: T3 precedes S6/S7 of its own
  // cycle), except the second reads of the long chains (adrl1/adrl2/adsl2/adsl3/ablw3/pdcl2/pead3/nnrl2, 4 clocks in the die)
  // and the 'f' words, which take the current data. own_strobe and cls_t below describe that rule; they are not used (see below).
  logic [15:0] edb_q, src_edb_eff; logic own_strobe, cls_t;
  always_ff @(posedge clk) edb_q <= edb;
  assign cls_t = (dec[ND_ACC_CLASS_LSB +: ND_ACC_CLASS_W] == ND_ACC_CLASS__T[ND_ACC_CLASS_W-1:0]);
  // the immediate fetch words (e#w1 0ea, e#l1 0a7, o#w1 2b9, o#l1 3e0) consume the immediate (previous access) and start
  // the next prefetch: the DBIN strobe takes the previous data (delayed edb). All the others strobe the current data.
  assign own_strobe = ~((upc == 10'h0ea) | (upc == 10'h0a7) | (upc == 10'h2b9) | (upc == 10'h3e0));
  // RESET instruction: "The RESET instruction causes the processor to assert RESET for 124 clock periods to reset
  // the external devices of the system. The internal state of the processor is not affected" (MC68000UM 5.5 p. 5-29; 6.3.1
  // p. 6-11). In the die the pin is driven by the 'ak' atom of the two words of the delay loop, rset3 (micro 0e4) and rset4
  // (micro 114, App. H p. 137: "access ak", 'idle wait'), an atom that our nano ROM does not resolve: the output is taken from
  // the micro address of the current word instead. The loop runs 31 times (rset2 loads the AU with -2 and each rset3 adds -2
  // until its low six bits are zero: -64), 62 words of 2 clocks = 124 clocks. Left unconnected, it costs no logic.
  assign reset_o = (upc == 10'h0e4) | (upc == 10'h114);
  // In this pipeline every word strobes the data of its own cycle (current): taking the previous data (edb_q) on the 't' words
  // or on the 4 immediate fetch words would break the prefetch chains (reset, LEA, MOVE.L).
  assign src_edb_eff = edb;
  logic unused_strobe; assign unused_strobe = own_strobe | cls_t | (|edb_q);
  // The read data reaches the sections (edb -> dbin, irc, ...) through edb (busctl) at the en of the word that completes the
  // cycle, the falling edge entering S7; the DBIN register lives with the bus-side registers (m68k_regs_bd). edb_q above
  // is only the delayed copy of the die rule and is not consumed (unused_strobe).
  logic [15:0] irc, ir, ird;
  logic [15:0] src_alu, src_alue, src_dcr; logic [4:0] cc_alu; logic locz, d4; logic [1:0] alue10;
  logic [31:0] rf_ab, rf_db, pc_q;   // read ports of the register bank (one per bus) and PC
  logic [3:0] rp_reg; logic mvm_end;
  logic [15:0] ab_h, ab_l, ab_d, db_h, db_l, db_d;
  logic [15:0] ab_ld;   // L/D coupling of the AB (ab_l_drv ? ab_l : ab_d), from the fabric
`ifdef VERILATOR
  always @(posedge clk) if (rst_n && ab_ld !== (ab_l_drv ? ab_l : ab_d)) begin
    $display("AB L/D coupling MISMATCH t=%0t %h / %h", $time, ab_ld, (ab_l_drv ? ab_l : ab_d)); $fatal(1);
  end
`endif
  logic ab_h_drv, ab_l_drv, ab_d_drv, db_h_drv, db_l_drv, db_d_drv;
  // fabric of the bus sections, bus-side registers and bus control
  // The same three parts in two arrangements, chosen by BUSDP_NODE: with 1 they are inside a single
  // hierarchy node (m68k_busdp_si, at the end of this file), so that the fitter, which places per hierarchy
  // node, keeps their internal paths together; with 0 (default) they are instantiated flat here.
  // Logic, cycles and connections are identical in the two branches: only the hierarchy boundary
  // changes.
  generate if (BUSDP_NODE) begin : g_node
  m68k_busdp_si u_busdp (.dec(dec), .g_sel(g_sel[11:0]), .gq_sel(gq_sel), .gd_sel(gd_sel), .src_alu(src_alu), .src_alue(src_alue), .aob(aob), .at(at), .src_ath(at[31:16]), .src_atl(at[15:0]), 
    .au(au), .dbin(dbin), .src_dcr(src_dcr), .ftu_q(ftu_q), .rf_ab(rf_ab), .rf_db(rf_db), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .ab_ld(ab_ld), .db_h(db_h), 
    .db_l(db_l), .db_d(db_d), .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), 
    .db_d_drv(db_d_drv), .ab_wl(ab_wl), .ab_wh(ab_wh), .db_wl(db_wl), .db_wh(db_wh), .clk(clk), .rst_n(rst_n_b), .en(en), .src_edb_eff(src_edb_eff), .dob(dob), .aob_n(aob_n), .dob_n(dob_n), .dbin_n(dbin_n), 
    .cen(cen), .cen2(cen2), .fc_micro(fc_micro), .sup(sup), .tas_op(ird[15:6] == 10'b0100_1010_11), 
    .pc_rel((ird[5:3] == 3'b111) & (ird[2:1] == 2'b01)), .byte_acc(byte_acc), .dtack_n(dtack_n), .vpa_n(vpa_n), .berr_n(berr_n), .data_in(data_in), 
    .e_cnt(e_cnt), .halt_n(halt_n), .g0_busy(g0_busy), .br_n(br_n), .bgack_n(bgack_n), .bg_n(bg_n), .bus_tri(bus_tri), .halted_n(halted_n), 
    .as_n(as_n), .uds_n(uds_n), .lds_n(lds_n), .rw(rw), .vma_n(vma_n), .data_out(data_out), .data_oe(data_oe), .fc(fc), .addr_ld_o(addr_ld_o), .ph(ph), 
    .req(req), .edb(edb), .iack_vec(iack_vec), .iack_spur(iack_spur), .iack_acc(iack_acc), .edb_valid(edb_valid), .bus_active(bus_active), .stall(stall), .ssw_rw(ssw_rw), 
    .ssw_fc(ssw_fc), .berr_ev(berr_ev), .aerr_ev(aerr_ev), .c_ft_q(c_ft_q), .en_rf(en_rf), .dfault());
  end else begin : g_flat
  m68k_bus_sec u_bus (.dec(dec), .g(g_sel[11:0]), .gq(gq_sel), .gd(gd_sel),
    .src_alu(src_alu), .src_alue(src_alue), .src_aob(aob), .src_at(at), .src_ath(at[31:16]), .src_atl(at[15:0]), .src_au(au),
    .src_dbin(dbin), .src_dcr(src_dcr), .src_ftu(ftu_q), .src_rf_ab(rf_ab), .src_rf_db(rf_db),
    .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d), .ab_ld(ab_ld),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob), .ab_dob_drv(ab_dob_drv),
    .ab_wl(ab_wl), .ab_wh(ab_wh), .db_wl(db_wl), .db_wh(db_wh));   // narrow write-data wires of the bank
  m68k_regs_bd u_regs (.clk(clk), .rst_n(rst_n_b), .en(en), .dec(dec), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob), .ab_dob_drv(ab_dob_drv),
    .src_au(au), .src_alu(src_alu), .src_edb(src_edb_eff), .at(at), .aob(aob), .dob(dob), .dbin(dbin), .aob_n(aob_n), .dob_n(dob_n), .dbin_n(dbin_n));
  m68k_busctl_si u_bc (.clk(clk), .rst_n(rst_n), .cen(cen), .cen2(cen2), .dec(dec), .fc_micro(fc_micro), .sup(sup), .tas_op(ird[15:6] == 10'b0100_1010_11), .pc_rel((ird[5:3] == 3'b111) & (ird[2:1] == 2'b01)), .aob(aob_n), .dob(dob_n), .byte_acc(byte_acc),
    .dtack_n(dtack_n), .vpa_n(vpa_n), .berr_n(berr_n), .data_in(data_in), .e_cnt(e_cnt),
    .halt_n(halt_n), .g0_busy(g0_busy), .br_n(br_n), .bgack_n(bgack_n), .bg_n(bg_n), .bus_tri(bus_tri), .halted_n(halted_n),
    .as_n(as_n), .uds_n(uds_n), .lds_n(lds_n), .rw(rw), .vma_n(vma_n), .data_out(data_out), .data_oe(data_oe), .fc(fc), .addr_ld_o(addr_ld_o),
    .en(en), .ph(ph), .req(req), .edb(edb), .iack_vec(iack_vec), .iack_spur(iack_spur), .iack_acc(iack_acc), .edb_valid(edb_valid), .bus_active(bus_active), .stall(stall), .ssw_rw(ssw_rw), .ssw_fc(ssw_fc), .berr_ev(berr_ev), .aerr_ev(aerr_ev), .c_ft_q(c_ft_q), .en_rf(en_rf), .dfault());
  end endgenerate
  // bus-side registers
  // AU (DB not driven = keeps the current value)
  logic is_byte, rf_db_sp, set_s_o, sup_e;
  logic [1:0] ph;
  m68k_au u_au (.clk(clk), .rst_n(rst_n), .en(en), .op_ld((ph == 2'd1) & cen2), .st_ld((ph == 2'd2) & cen), .dec(dec), .db_h(db_h), .db_l(db_l), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .ab_h(ab_h), .ab_l(ab_l), .is_byte(is_byte), .au_sp(rf_db_sp), .au(au));
  // register bank + MOVEM encoder
  // MOVEM mask: same word and same source as ftu<-ab (push1/ldmr1 "(dbin) -> ab -> ftu": DBIN drives ABD, fallback L<-D):
  // ab_ld, not the bare ABL, which is not driven in those words
  m68k_mvm_si u_mvm (.clk(clk), .rst_n(rst_n_b), .en(en), .dec(dec), .ird(ird), .ir(ir), .mask_in(ab_ld), .rp_next(rp_next), .rp_reg(rp_reg), .mvm_end(mvm_end));
  m68k_regfile_si #(.DBG(DBG), .PC_W17(1'b0), .WE_FLOP(1'b1)) u_rf (.clk(clk), .rst_n(rst_n_b), .en(en), .en_rf(en_rf), .we_ab_h_i(we_ab_h_q), .we_ab_l_i(we_ab_l_q), .we_db_h_i(we_db_h_q), .we_db_l_i(we_db_l_q), .dec(dec), .dec_p(dec_p), .dec_pm(dec_pm), .dec_po(dec_po), .ovr(ovr_p), .rp_next(rp_next), .sup_next(sup_next), .ir(ir), .ird(ird), .sup(sup_e_q), .byte_op(is_byte), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_wl(ab_wl), .ab_wh(ab_wh), .db_wl(db_wl), .db_wh(db_wh),   // narrow write-data wires
    .nw_pre(nw_pre), .g_rd(g_sel[26:12]),   // next nanoword; registered read port selectors
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv), .au(au), .at(at), .rp_reg(rp_reg), .irc(irc), .dbg_idx_d({2'b00, dbg_rd_dnum}), .dbg_idx_a(dbg_rd_anum == 3'd7 ? (sup ? 5'd31 : 5'd15) : {2'b01, dbg_rd_anum}), .dbg_data_d(dbg_rd_ddata), .dbg_data_a(dbg_rd_adata),
    .rf_ab(rf_ab), .rf_db(rf_db), .rf_db_sp(rf_db_sp), .pc_q(pc_q));
  // ALU with surrounding logic
  logic dispatch;   // declared before use (Quartus)
  m68k_alux_si u_alux (.clk(clk), .rst_n(rst_n), .en(en), .op_ld((ph == 2'd1) & cen2), .st_ld((ph == 2'd2) & cen), .instr_start(dispatch), .is_byte(is_byte), .dec(dec), .ird(ird), .ir(ir), .ld_ird(ld_ird_c), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv), .cc_in(cc), .src_alu(src_alu), .src_alue(src_alue), .src_dcr(src_dcr), .cc_alu(cc_alu),
    .locz(locz), .d4(d4), .alue10(alue10), .dbg_a(dbg_alu_a), .dbg_d(dbg_alu_d), .dbg_r(dbg_alu_r));
  // exceptions, FTU, PSW
  logic [7:0] tvn; logic [2:0] inl; logic illegal;
  logic upd_full, upd_part, berr_ev, aerr_ev, bus_active, req, byte_acc;
  logic g0_busy;                      // exc -> busctl: bus/address error routine in progress (double fault decided in the busctl)
  logic ssw_rw; logic [2:0] ssw_fc; logic tvn_use;   // SSW of the address/bus error frame: {IRD[15:5], R/W, I/N, FC}. R/W and FC of the aborted cycle
                                      // (busctl); I/N = 1 for a fault during the processing of a group 0 or 1 exception (exc_si tvn_use,
                                      // MC68000UM 6.3.9.1 p. 6-16); IRD[15:5] as on the reference core: IRD = the instruction
                                      // executing at the fault. When the faulting write overlaps the dispatch of the following
                                      // instruction (e.g. MOVE.W Dn,-(An)), IRD, like the IR field of the frame (ftu<-(ird)), is
                                      // already the following opcode.
  logic [1:0] sz;
  // the operand size is a pure function of the instruction: it is decided on `ir`,
  // i.e. one micro-step earlier, and registered at the instant `ird` is loaded, so it
  // equals the size of `ird` in every word without decoding `ird` in the address path.
  logic [1:0] sz_p, sz_0; logic is_byte_p, is_byte_0;
  m68k_size_si u_size  (.ird(ir),    .sz(sz_p), .is_byte(is_byte_p));
  m68k_size_si u_size0 (.ird(16'd0), .sz(sz_0), .is_byte(is_byte_0));   // power-up value: what the module gives for ird = 0
  always_ff @(posedge clk) begin
    if (!rst_n) begin sz <= sz_0; is_byte <= is_byte_0; end
    else if (en && ld_ird_c) begin sz <= sz_p; is_byte <= is_byte_p; end
  end
  assign upd_full = ir_load & ~bit4 & ~fmt;
  assign upd_part = ir_load & bit4 & ~fmt;
  assign dispatch = ~fmt & (ty == 2'd1);
`ifdef VERILATOR
  // SAFETY NET: the flop upc_is327 must equal (upc == 327) at every clock
  always @(posedge clk) if (rst_n && upc_is327 !== (upc == 10'h327)) begin
    $display("upc 327 flag MISMATCH t=%0t upc_is327=%b upc=%03x", $time, upc_is327, upc); $fatal(1);
  end
`endif
  m68k_exc_si u_exc (.clk(clk), .rst_n(rst_n), .en(en), .en_clk2(cen2), .upd_full(upd_full), .upd_part(upd_part), .dispatch(dispatch),
    .rst_req(1'b0), .berr_ev(berr_ev), .aerr_ev(aerr_ev), .ipl(ipl), .psw(psw), .illegal(illegal), .priv_ir(priv_ir), .linea(linea_q), .linef(linef_q), .ir(ir), .ird(ird), .ld_ird(ld_ird_c), .stop_w(upc_is327), .g0_busy(g0_busy), .tvn_use(tvn_use),
    .iack_vec(iack_vec), .iack_spur(iack_spur), .vec_num(data_in[7:0]),
    .a0s_sub(a0s_sub), .a0s(a0s), .a0_sub(a0_sub), .a0(a0), .tvn(tvn), .inl(inl));
  logic ftu_latched; logic iack_vec, iack_spur, iack_acc; logic sup_nx_en;   // sup_nx_en: the en branch of sup_next (m68k_psw_si)
  // ftu<-ab: the FTU lies in the L section; with ABL not driven it equals ABD through the coupler (stiw3 "alu -> ab -> ftu": the ALU
  // drives ABD). Without that fallback the FTU would load 0, and for example ORI #imm,SR would clear the PSW.
  m68k_ftu_si #(.IMP_REG(1'b1)) u_ftu (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ird(ird), .ir(ir), .irc(irc), .psw(psw), .ssw({ird[15:5], ssw_rw, tvn_use, ssw_fc}), .ab_l(ab_ld), .edb(edb), .tvn(tvn), .inl(inl), .ftu_q(ftu_q), .ftu_latched(ftu_latched));
  // `ccr_only` is a pure function of the instruction (byte forms: to CCR, RTR): it is decided on `ir` and registered
  // at the instant `ird` takes `ir` (like `sz` above); at reset it equals f(0), like ird = 0. Same value, same cycle
  // as f_ccr(ird), without three 16-bit compares at the head of the `s_nx` -> `sup_e_q` path.
  logic ccr_only;
  function automatic logic f_ccr(input logic [15:0] w); f_ccr = ((w & 16'hf1ff) == 16'h003c) | (w[15:6] == 10'b0100010011) | (w == 16'h4e77); endfunction
  always_ff @(posedge clk) begin
    if (!rst_n) ccr_only <= f_ccr(16'd0);
    else if (en && ld_ird_c) ccr_only <= f_ccr(ir);
  end
`ifdef VERILATOR
  // SAFETY NET: the flop must equal f_ccr(ird) at every clock
  always @(posedge clk) if (rst_n && ccr_only !== f_ccr(ird)) begin
    $display("ccr_only flop MISMATCH t=%0t ccr_only=%b expected=%b ird=%04x", $time, ccr_only, f_ccr(ird), ird); $fatal(1);
  end
`endif
  m68k_psw_si u_psw (.psws_p1(psws_p1), .clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .cc_alu(cc_alu), .ftu_q(ftu_q), .inl(inl), .ccr_only(ccr_only), .psw(psw), .sup(sup), .sup_next(sup_next), .sup_nx_en(sup_nx_en), .set_s_o(set_s_o), .g0_entry(a0s_sub), .cc(cc));
  assign sup_e = sup | set_s_o;   // effective S in the word that sets it (trap2 reads (sp) as SSP)
  // the effective S bit for the register bank comes out of a flop loaded at en with the value that sup | set_s_o will have
  // after this edge (sup_next of m68k_psw_si, en branch sup_nx_en). sup and set_s_o change only at en, so between two ens
  // sup_e_q == sup_e always (checked below). Same value, same edge: no cycle, and the path psws_p1 -> sup_e -> rx_i[4] ->
  // read index -> rf_db starts from a flop.
  // sup_next alone is NOT enough: at en it uses the current psws_p1, not that of the next word (e.g. the privilege trap entered
  // from MOVE to SR: set_s_o = 1 in the next word). The right value is sup_next | (nw_pre[22+:5] == 23), the psws+=+1 atom.
  logic sup_e_q;
  always_ff @(posedge clk) begin
    if (!rst_n) sup_e_q <= 1'b1;        // reset: S = 1 (Motorola manual), set_s_o = 0
    else if (en) sup_e_q <= sup_nx_en | (nw_pre[22 +: 5] == 5'd23);   // sup_next | psws_p1 of the NEXT word: set_s_o' = psws_p1' | g0_q', and g0_q' = g0_entry is already in sup_next. sup_nx_en = the en branch of sup_next: inside the flop enabled by en the mux on en is not needed
  end
`ifdef VERILATOR
  // SAFETY NET: when en = 1 the en branch sup_nx_en must equal sup_next
  always @(posedge clk) if (rst_n && en && sup_nx_en !== sup_next) begin
    $display("sup_next en branch MISMATCH t=%0t sup_nx_en=%b sup_next=%b", $time, sup_nx_en, sup_next); $fatal(1);
  end
  // SAFETY NET: the flop against the combinational sup | set_s_o, compared at every clock
  always @(posedge clk) if (rst_n && sup_e_q !== sup_e) begin
    $display("effective S flop MISMATCH t=%0t sup_e_q=%b sup_e=%b (sup %b set_s_o %b)", $time, sup_e_q, sup_e, sup, set_s_o); $fatal(1);
  end
`endif
  // instruction queue, A1/A2/A3 decoder, conditional branch
  logic [6:0] cls_q;   // row class of the A1/A2/A3 decoder, registered in m68k_ir_si
  m68k_ir_si u_ir (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ir_load(ir_load), .bit4(bit4), .fmt(fmt), .edb(edb), .irc(irc), .ir(ir), .ird(ird), .cls_q(cls_q));
  // registered A1/A2/A3 decoder: IR is loaded at en and the dispatch (jump a1/a2/a3, illegal exception) happens at a later en
  // (>= 4 clocks later): the decoder -> exc -> sequencer -> nano ROM address cone is split in two (one clock of delay always covered)
  logic [9:0] a1_c, a2_c, a3_c; logic illegal_c, priv_c, priv_ir, linea_q, linef_q;   // priv (from the decoder) and line A/F registered like illegal
  m68k_a123_si #(.EXT_CLS(1'b1)) u_a123 (.ir(ir), .cls_in(cls_q), .a1(a1_c), .a2(a2_c), .a3(a3_c), .illegal(illegal_c), .priv(priv_c));
  // illegal, priv_ir and the line A/F flags are registered together with a1-a3, one clock after IR and always before the
  // dispatch that consumes them. `illegal` comes from its own decoder output (illegal_c), not from the a1 register: the
  // two are equal by construction, but with illegal_c consumed synthesis keeps the decoder PLA in its smaller form and
  // the ir -> a1 path shorter.
  always_ff @(posedge clk) begin a1 <= a1_c; a2 <= a2_c; a3 <= a3_c; illegal <= illegal_c; priv_ir <= priv_c; linea_q <= (ir[15:12] == 4'ha); linef_q <= (ir[15:12] == 4'hf); end
  // auz: zero of the 6 LOW bits of the AU, like the die: the RESET loop starts from -2 (rset2: ALU(0)-2) and goes down by 2
  // until the low six bits are zero -> 31 iterations (RESET takes 132 clocks, MC68000UM Table 8-12). A 32-bit zero test would
  // not end the loop. The other uses (DIV 15/31, shift count, MUL loop) start from values <= 63: identical.
  // c1c0 registered at every clock: its only consumer, useq, uses it at en, at least 4 pulses after its inputs change (all
  // registers written at en, or uw_q valid from en+2), so the flop delays nothing and the ird -> branch table path ends at a flop.
  logic [1:0] c1c0_c;
  m68k_branch_si u_br (.cbc(cbc), .irc11(irc[11]), .auz(au[5:0] == 6'd0), .locz(locz), .dcr4(d4), .mend(mvm_end), .ird(ird), .alue10(alue10), .cc(cc), .c1c0(c1c0_c));
  always_ff @(posedge clk) c1c0 <= c1c0_c;
  // microcycle + bus sequencer
  logic [ND_ACC_TYPE_W-1:0] acc_type; assign acc_type = dec[ND_ACC_TYPE_LSB +: ND_ACC_TYPE_W];
  // byte: operand (op) or 'uk' access (adsw2/adsl2/adsl3/MOVEM: data operand) of byte size; MOVEP: half words of DOB/DBIN
  logic movep_half; assign movep_half = (dec[ND_DOBH_FROM_LSB +: ND_DOBH_FROM_W] != '0) | (dec[ND_DOBL_FROM_LSB +: ND_DOBL_FROM_W] != '0) |
                                        (dec[ND_DBINH_FROM_LSB +: ND_DBINH_FROM_W] != '0) | (dec[ND_DBINL_FROM_LSB +: ND_DBINL_FROM_W] != '0);
  assign byte_acc = (is_byte & (acc_type == ND_ACC_TYPE__OP[ND_ACC_TYPE_W-1:0] || acc_type == ND_ACC_TYPE__UK[ND_ACC_TYPE_W-1:0])) | movep_half;
  logic edb_valid, stall;
  // the bus cycle started in a word uses the AOB/DOB loaded IN THE SAME word (transfers during the microcycle)
  // bus error: berr_ev = done of the cycle terminated by BERR (busctl, once); in exc it also enters a0s_sub combinationally
  // so the 't'/'f' word that waits for the cycle enters bser at its en (= the done). The double fault is decided by the busctl
  // (cyc_g0). berr_ev is an event at the end of the cycle, not the level bus_active & ~berr_n: with the level, a request-only
  // word (i) would enter bser while the cycle is still active, and the level would then signal a false double fault. The
  // routine is entered after the end of the cycle, for reads as for writes.
  // aerr_ev comes from the busctl, decided at S1 (the address from the register is ready at S1, through rf_q): ~byte_acc & A0, except the IACK cycle (CPU space, 0xFFFFF1+2*level, A0 = 1), which is not an address error
  assign ird_o = ird; assign psw_o = psw; assign pc_o = pc_q; assign au_o = au; assign aob_o = aob; assign at_o = at;
  assign dbg_pc = pc_q[23:0];
  /* verilator lint_off UNUSEDSIGNAL */
  logic unused; assign unused = (|uw) | (|nw) | (|nidx) | (|uaddr) | (|ph) | edb_valid | stall | (|sz) | (|cbc) | ftu_latched | (|dob) | iack_acc;
  /* verilator lint_on UNUSEDSIGNAL */
  // debug for the testbenches written for m68k_seq (dbg_instr_done = end of instruction = dispatch of the next one)
  logic instr_done_c;
  assign instr_done_c = en & (dec[ND_IRD_FROM_LSB +: ND_IRD_FROM_W] == ND_IRD_FROM__P_IR[ND_IRD_FROM_W-1:0]);
  always_ff @(posedge clk) dbg_instr_done <= rst_n & instr_done_c;   // registered: visible for one whole clock to the testbenches
  always_ff @(posedge clk) begin
    if (!rst_n) dbg_illegal <= 1'b0;
    else if (instr_done_c && illegal) dbg_illegal <= 1'b1;
  end
endmodule

// m68k_busdp_si - the bus chain in ONE SINGLE hierarchy node (m68k_cpu_si with BUSDP_NODE = 1):
// fabric of the sections (m68k_bus_sec) + bus-side registers (m68k_regs_bd) + bus control
// (m68k_busctl_si, which contains the state machine). The three blocks stay identical and
// the connections between them are the same: no logic and no cycle changes.
// Only the hierarchy boundary changes: the fitter places per hierarchy node, so the paths that
// cross the three blocks are placed together instead of spreading over three separate
// blocks.
module m68k_busdp_si (
  input  logic [NDEC_W-1:0] dec,
  input  logic [11:0] g_sel,
  input  logic [24:0] gq_sel,   // non-RAW fabric selectors, flopped at every clock
  input  logic [5:0] gd_sel,   // RAW halves of the driven flags, registered at en
  input  logic [15:0] src_alu,
  input  logic [15:0] src_alue,
  output logic [31:0] aob,
  output logic [31:0] at,
  input  logic [15:0] src_ath,
  input  logic [15:0] src_atl,
  input  logic [31:0] au,
  output logic [15:0] dbin,
  input  logic [15:0] src_dcr,
  input  logic [15:0] ftu_q,
  input  logic [31:0] rf_ab,
  input  logic [31:0] rf_db,
  output logic [15:0] ab_h,
  output logic [15:0] ab_l,
  output logic [15:0] ab_d,
  output logic [15:0] db_h,
  output logic [15:0] db_l,
  output logic [15:0] db_d,
  output logic [15:0] ab_ld,   // ab_l_drv ? ab_l : ab_d from the fabric
  output logic ab_h_drv,
  output logic ab_l_drv,
  output logic ab_d_drv,
  output logic db_h_drv,
  output logic db_l_drv,
  output logic db_d_drv,
  output logic [15:0] ab_wl, ab_wh, db_wl, db_wh,   // narrow wires of the write data of the bank
  input  logic clk,
  input  logic rst_n,
  output logic en,
  output logic en_rf,   // copy of en for the register bank
  input  logic c_ft_q,   // class f|t from a flop, for busctl
  input  logic [15:0] src_edb_eff,
  output logic [15:0] dob,
  output logic [31:0] aob_n,
  output logic [15:0] dob_n,
  output logic [15:0] dbin_n,
  input  logic cen,
  input  logic cen2,
  input  logic [1:0] fc_micro,
  input  logic sup,
  input  logic tas_op,
  input  logic pc_rel,
  input  logic byte_acc,
  input  logic dtack_n,
  input  logic vpa_n,
  input  logic berr_n,
  input  logic [15:0] data_in,
  input  logic [4:0] e_cnt,
  input  logic halt_n,
  input  logic g0_busy,
  input  logic br_n,
  input  logic bgack_n,
  output logic bg_n,
  output logic bus_tri,
  output logic halted_n,
  output logic as_n,
  output logic uds_n,
  output logic lds_n,
  output logic rw,
  output logic vma_n,
  output logic [15:0] data_out,
  output logic data_oe,
  output logic [2:0] fc,
  output logic addr_ld_o,   // only the enable: the 23 flops of the address are in m68k_cpu_si
  output logic [1:0] ph,
  output logic req,
  output logic [15:0] edb,
  output logic iack_vec,
  output logic iack_spur,
  output logic iack_acc,
  output logic edb_valid,
  output logic bus_active,
  output logic stall,
  output logic ssw_rw,
  output logic [2:0] ssw_fc,
  output logic berr_ev,
  output logic aerr_ev,
  output logic dfault
);
  // the value that the DOB takes from the AB bus: it is born in the fabric and ends in the register,
  // no other consumer uses it, so it stays inside this node.
  logic [15:0] ab_dob; logic ab_dob_drv;
  m68k_bus_sec u_bus (.dec(dec), .g(g_sel), .gq(gq_sel), .gd(gd_sel),
    .src_alu(src_alu), .src_alue(src_alue), .src_aob(aob), .src_at(at), .src_ath(src_ath), .src_atl(src_atl), .src_au(au),
    .src_dbin(dbin), .src_dcr(src_dcr), .src_ftu(ftu_q), .src_rf_ab(rf_ab), .src_rf_db(rf_db),
    .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d), .ab_ld(ab_ld),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob), .ab_dob_drv(ab_dob_drv),
    .ab_wl(ab_wl), .ab_wh(ab_wh), .db_wl(db_wl), .db_wh(db_wh));   // narrow write-data wires of the bank
  m68k_regs_bd u_regs (.clk(clk), .rst_n(rst_n), .en(en), .dec(dec), .ab_h(ab_h), .ab_l(ab_l), .ab_d(ab_d), .db_h(db_h), .db_l(db_l), .db_d(db_d),
    .ab_h_drv(ab_h_drv), .ab_l_drv(ab_l_drv), .ab_d_drv(ab_d_drv), .db_h_drv(db_h_drv), .db_l_drv(db_l_drv), .db_d_drv(db_d_drv),
    .ab_dob(ab_dob), .ab_dob_drv(ab_dob_drv),
    .src_au(au), .src_alu(src_alu), .src_edb(src_edb_eff), .at(at), .aob(aob), .dob(dob), .dbin(dbin), .aob_n(aob_n), .dob_n(dob_n), .dbin_n(dbin_n));
  m68k_busctl_si u_bc (.clk(clk), .rst_n(rst_n), .cen(cen), .cen2(cen2), .dec(dec), .fc_micro(fc_micro), .sup(sup), .tas_op(tas_op), .pc_rel(pc_rel), .aob(aob_n), .dob(dob_n), .byte_acc(byte_acc),
    .dtack_n(dtack_n), .vpa_n(vpa_n), .berr_n(berr_n), .data_in(data_in), .e_cnt(e_cnt),
    .halt_n(halt_n), .g0_busy(g0_busy), .br_n(br_n), .bgack_n(bgack_n), .bg_n(bg_n), .bus_tri(bus_tri), .halted_n(halted_n),
    .as_n(as_n), .uds_n(uds_n), .lds_n(lds_n), .rw(rw), .vma_n(vma_n), .data_out(data_out), .data_oe(data_oe), .fc(fc), .addr_ld_o(addr_ld_o),
    .en(en), .ph(ph), .req(req), .edb(edb), .iack_vec(iack_vec), .iack_spur(iack_spur), .iack_acc(iack_acc), .edb_valid(edb_valid), .bus_active(bus_active), .stall(stall), .ssw_rw(ssw_rw), .ssw_fc(ssw_fc), .berr_ev(berr_ev), .aerr_ev(aerr_ev), .c_ft_q(c_ft_q), .en_rf(en_rf), .dfault(dfault));
endmodule
