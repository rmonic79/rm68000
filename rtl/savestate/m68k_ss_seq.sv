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

// m68k_ss_seq.sv — EXTERNAL savestate of the MC68000 for a system with N CPUs: a single sequencer, CPUs served in series.
//
// SEQUENCE structure (docs/external-savestate.md §4.1). The module does not look at the address: it recognises the cycles of the served CPU by type
// (FC, R/W, AS) and by POSITION in the sequence. The handler is straight-line code, so the program reads
// arrive in order, one per word [UM 68000: two-word prefetch queue]: the k-th program read is word
// k of the handler. No address comparator; the substituted word is a 5-input table per bit (sequence
// index); the section is a chain of 2N 16-bit registers (SSP high and low of each CPU) read and written IN ORDER by the
// system's stream (one word per request, addresses 0..count-1 in order, once each).
// Towards the CPUs (connection: docs/external-savestate.md §4.3), everything from flops: exec (pause_n = ~(paused & ~exec), pause
// registered in m68k_cpu_dt_si), substitution command, substituted word, SSP selection (data = sec_word), IPL 7, hidden
// IPL, soft reset. The data mux is in the top: bus_din = ovr ? (sspsel ? sec_word : word) : map data.
// Closing the clock-enables at any point of the cycle is legitimate for our core: the even phases advance only on cen and
// the odd ones only on cen2 (m68k_busctl_si, phase_ev); a lost pulse only causes a wait.
//
// Save, for each CPU k: in reset -> two arbitrary words in place of the SSP; otherwise IPL 7 and release, IACK, two
// substituted autovector reads (PC = WIN_BASE), handler up to program read 11 (after the two writes
// of the SSP, which entered at the tail of the chain); CPU stopped. Then stream. Then for each CPU not in reset the tail (pop, RTE) up to the
// first program read outside the handler.
// Restore: stream (always); empty slot -> resume only; otherwise for each CPU: in reset -> discard its two words
// (in reset at save time too: the system control register is in the restored state); otherwise soft reset
// for 64 clocks with the CPU clocked, reset vector substituted (SSP from the chain, PC = R_ENTRY), tail of the handler up to the
// first program read outside.
// A CPU halted by a double bus fault takes no interrupt: the system reports it on cpu_in_reset (halted_n low) and it is
// served like a CPU in reset: at the save its two words are the odd marker SSP 00000001; at the restore into a CPU in
// reset or halted they are discarded (it stays as it is); restored into a running CPU the odd SSP halts it by a double
// bus fault at the first pop, like the chip, and the sequencer ends the restore on its first push. A CPU in STOP is woken by
// the level 7 and, after the save and after the restore, continues from the instruction after the STOP (the frame holds
// that PC). See docs/external-savestate.md §5.
//
// Handler (index = position among the substituted reads; [UM 68000 8-x] for encodings and timings):
//   0,1   level 7 autovector ($07C/$07E)            WIN_BASE
//   2,3   48E7 FFFE   MOVEM.L D0-D7/A0-A6,-(SP)
//   4     4E68        MOVE    USP,A0
//   5     2F08        MOVE.L  A0,-(SP)
//   6     4E71        NOP     (separates the writes of MOVE.L A0 from those of the SSP)
//   7-9   23CF H_SCR  MOVE.L  SP,(H_SCR).L          -> writes at the tail of the chain (high word, then low)
//   10    205F        MOVE.L  (SP)+,A0              <- R_ENTRY
//   11    4E60        MOVE    A0,USP
//   12,13 4CDF 7FFF   MOVEM.L (SP)+,D0-D7/A0-A6
//   14    4E73        RTE
//   15    4E71        (read by the prefetch before RTE empties the queue)
//   16,17 SSP high, SSP low (sec_word, cpu_sspsel)    reset vector
//   18,19 R_ENTRY
// An instruction at index a that is n words long starts with reads 0..a+1 completed and reads a+2..a+n+1, the last one after the
// writes. MOVE.L A0,-(SP) (a=5): writes with index <= 8; MOVE.L SP,(H_SCR).L (a=7): writes with index 9..11, then
// read 11 -> SSP writes = those with index >= 9, end of the head at read 11. RTE (a=14) starts with 16
// reads completed; read 16 is at the restored PC -> end of the tail at read 16.
// State on the stack (RAM saved by the system): +0 USP, +4..+63 D0..D7,A0..A6, +64 SR, +66 PC (70 bytes below the SSP).
`default_nettype none

module m68k_ss_seq #(
  parameter        N         = 2,
  parameter [23:0] WIN_BASE  = 24'hFF0000,   // PC of the handler (the program reads are substituted by sequence): even, with DTACK
  parameter [23:0] H_SCR     = 24'hFF0040    // destination of the SSP write: even address with DTACK, not decoded
)(
  input  wire            clk,
  input  wire            rst,            // system reset, active high
  // ---- system savestate controller
  input  wire            ss_save,        // pulse
  input  wire            ss_load,        // pulse
  input  wire            paused,         // level: clean pause entered
  input  wire            ss_busy,        // level: stream in progress
  input  wire            slot_empty,     // valid after the fall of ss_busy on a read: slot never written
  output logic           ss_pause,       // clean pause request
  output logic           ss_mem_write,   // pulse
  output logic           ss_mem_read,    // pulse
  // ---- section: 2N words (SSP high and low of each CPU), read and written in order by the stream
  output wire [15:0]     sec_word,       // from flop: word at the head (stream read; SSP of the reset vector)
  input  wire            sec_rd,         // pulse: the stream has taken sec_word
  input  wire            sec_wr,         // pulse: the stream writes sec_wdata
  input  wire [15:0]     sec_wdata,
  // ---- CPU buses (from the surrounding logic); CPU k in bits k
  input  wire [N-1:0]    cpu_as_n,
  input  wire [N-1:0]    cpu_rw,         // 1 = read
  input  wire [N*3-1:0]  cpu_fc,
  input  wire [N*16-1:0] cpu_dout,
  input  wire [N-1:0]    cpu_in_reset,   // CPU in reset (not by this module) or halted after a double bus fault (halted_n low: no interrupt is taken, MC68000UM 6 p. 6-1): skipped at the save and at the restore
  // ---- towards the CPUs (everything from flops; at most one cell in the top)
  output wire [N-1:0]    cpu_exec,       // the CPU advances during the pause: pause_n = ~(paused & ~exec)
  output wire [N-1:0]    cpu_ovr,        // the read data of this CPU is substituted
  output wire [N-1:0]    cpu_sspsel,     // ... with sec_word (SSP of the reset vector); otherwise with cpu_word
  output wire [15:0]     cpu_word,
  output wire [N-1:0]    cpu_ipl7,       // IPL forced to 7
  output wire [N-1:0]    cpu_iplhide,    // system IPL hidden (0)
  output wire [N-1:0]    cpu_soft_reset,
  // ---- debug
  output wire [3:0]      state_o,
  output wire [1:0]      phase_o,        // 0 head of the save, 1 tail, 2 restore
  output wire [7:0]      cpu_sel_o
);

  localparam KW = (N > 1) ? $clog2(N) : 1;
  localparam NW = 2 * N;                               // words of the section
  localparam [23:0] R_ENTRY = WIN_BASE + 24'd20;       // restore PC = word 10 of the handler (nominal value)

  initial begin
    if (WIN_BASE[0] || H_SCR[0]) $error("m68k_ss_seq: WIN_BASE and H_SCR must be even");
  end

  // ---------------------------------------------------------------- handler table (sequence index only)
  function automatic [15:0] rom(input [4:0] i);
    case (i)
      5'd0:  rom = {8'd0, WIN_BASE[23:16]};
      5'd1:  rom = WIN_BASE[15:0];
      5'd2:  rom = 16'h48E7;
      5'd3:  rom = 16'hFFFE;
      5'd4:  rom = 16'h4E68;
      5'd5:  rom = 16'h2F08;
      5'd7:  rom = 16'h23CF;
      5'd8:  rom = {8'd0, H_SCR[23:16]};
      5'd9:  rom = H_SCR[15:0];
      5'd10: rom = 16'h205F;
      5'd11: rom = 16'h4E60;
      5'd12: rom = 16'h4CDF;
      5'd13: rom = 16'h7FFF;
      5'd14: rom = 16'h4E73;
      5'd18: rom = {8'd0, R_ENTRY[23:16]};
      5'd19: rom = R_ENTRY[15:0];
      default: rom = 16'h4E71;                         // 6, 15: NOP; 16, 17: unused (SSP from sec_word)
    endcase
  endfunction

  // ---------------------------------------------------------------- states
  typedef enum logic [3:0] { IDLE, PAUSE, NEXT, ENTER, VEC, RUN, ADV, SHIFT, STRA, STRB } st_t;
  st_t st = IDLE;
  assign state_o = 4'(st);
  localparam [1:0] PH_HEAD = 2'd0, PH_TAIL = 2'd1, PH_REST = 2'd2;
  logic [1:0] ph = PH_HEAD;                // head of the save, tail of the save, restore
  assign phase_o = ph;
  wire head = (ph == PH_HEAD);
  wire rest = (ph == PH_REST);

  logic [KW-1:0] k = '0;                  // served CPU
  assign cpu_sel_o = 8'(k);
  wire           last = (k == KW'(N-1));
  wire           in_rst_k = cpu_in_reset[k];      // served CPU in reset or halted (one node for NEXT and RUN)

  genvar g;
  wire [2:0] a_fc [N]; wire [15:0] a_dout [N];
  generate for (g = 0; g < N; g = g + 1) begin : g_view
    assign a_fc[g]   = cpu_fc[g*3 +: 3];
    assign a_dout[g] = cpu_dout[g*16 +: 16];
  end endgenerate

  // ---------------------------------------------------------------- bus of the served CPU
  wire        s_as_n = cpu_as_n[k];
  wire        s_rw   = cpu_rw[k];
  wire [2:0]  s_fc   = a_fc[k];
  wire [15:0] s_dout = a_dout[k];
  // Attributes latched while AS is low, pulse at the rising edge of AS; no fake edge when the CPU changes.
  // UDS/LDS are not needed: in the 68000 every cycle with AS has at least one data strobe (read, write, IACK).
  logic       as_q = 1'b1, sel_new = 1'b0, q_rw = 1'b1;
  logic [2:0] q_fc = 3'd0;
  wire acc_end  = s_as_n & ~as_q;
  wire rd_end   = acc_end &  q_rw & (q_fc != 3'b111);           // read completed, IACK excluded
  wire prog_end = rd_end & q_fc[1] & ~q_fc[0];                  // program read (FC = x10) completed
  wire wr_end   = acc_end & ~q_rw;
  wire iack     = ~s_as_n & (s_fc == 3'b111);                   // interrupt acknowledge cycle [UM 5-9]
  wire rd_live  = ~s_as_n & s_rw & (s_fc != 3'b111);            // read in progress, IACK excluded
  wire prg_live = rd_live & s_fc[1] & ~s_fc[0];

  logic         ipl7 = 1'b0, rel = 1'b0, srst = 1'b0, two = 1'b0;
  logic         busy_q = 1'b0, empty_q = 1'b0;   // system ss_busy / slot_empty registered at the input
  logic [4:0]   pc_i = 5'd0;                      // sequence index; during soft reset {two, pc_i} counts 64 clocks
  wire          t_rst = two & (&pc_i);

  // ---------------------------------------------------------------- section chain: wq[0] <- wq[1] <- ... <- w_in
  logic [15:0] wq [NW];
  initial for (int j = 0; j < NW; j++) wq[j] = 16'd0;
  wire        ssp_cap  = (st == RUN) & head & wr_end & (pc_i >= 5'd9);
  wire        vec_drop = (st == VEC) & rest & rd_end & (pc_i[4:1] == 4'b1000);       // after reads 16 and 17
  // Skipped CPU (in reset or halted): its SSP low word enters odd (bit 0 forced by `two`, which is 1 on the second word of
  // SHIFT and 0 in the head, so the SSP words of a served CPU keep their bit 0). A saved SSP of a served CPU is even (the
  // handler pushed on it). Restored into a running CPU, the odd SSP makes the first pop of the handler tail an address
  // error and the push of its frame a second one: double bus fault, the CPU halts like the chip (MC68000UM 6.3.9
  // p. 6-17); RUN ends on that write (the restore tail has none). No flop, no new node: one input more on w_in[0].
  wire        sh       = sec_rd | sec_wr | ssp_cap | vec_drop | (st == SHIFT);
  wire [15:0] w_in     = sec_wr ? sec_wdata : {s_dout[15:1], two | s_dout[0]};
  assign sec_word = wq[0];
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int j = 0; j < NW; j++) wq[j] <= 16'd0;
    end else if (sh) begin
      for (int j = 0; j < NW - 1; j++) wq[j] <= wq[j+1];
      wq[NW-1] <= w_in;
    end
  end

  // ---------------------------------------------------------------- sequencer
  always_ff @(posedge clk) begin
    if (rst) begin
      st <= IDLE; ph <= PH_HEAD; k <= '0; ss_pause <= 1'b0; ss_mem_write <= 1'b0; ss_mem_read <= 1'b0;
      ipl7 <= 1'b0; rel <= 1'b0; srst <= 1'b0; two <= 1'b0; pc_i <= 5'd0; busy_q <= 1'b0; empty_q <= 1'b0;
      as_q <= 1'b1; sel_new <= 1'b0; q_rw <= 1'b1; q_fc <= 3'd0;
    end else begin
      ss_mem_write <= 1'b0; ss_mem_read <= 1'b0;
      busy_q <= ss_busy; empty_q <= slot_empty;
      sel_new <= 1'b0;
      as_q <= sel_new ? 1'b1 : s_as_n;
      if (!s_as_n) begin q_rw <= s_rw; q_fc <= s_fc; end

      case (st)
        IDLE: begin
          k <= '0;
          if (ss_save)      begin ss_pause <= 1'b1; ph <= PH_HEAD; st <= PAUSE; end
          else if (ss_load) begin ss_pause <= 1'b1; ph <= PH_REST; st <= PAUSE; end
        end
        PAUSE: if (paused) begin
          if (head) st <= NEXT;
          else begin ss_mem_read <= 1'b1; st <= STRA; end
        end
        NEXT: begin                                                   // CPU k
          sel_new <= 1'b1;
          if (ph == PH_TAIL) begin
            pc_i <= 5'd12;
            if (in_rst_k) st <= ADV;
            else begin rel <= 1'b1; st <= RUN; end
          end else if (in_rst_k) st <= SHIFT;                         // two words: the marker enters (save) or is discarded
          else if (head) begin ipl7 <= 1'b1; rel <= 1'b1; st <= ENTER; end
          else begin srst <= 1'b1; rel <= 1'b1; pc_i <= 5'd0; two <= 1'b0; st <= ENTER; end
        end
        ENTER: begin
          if (head) begin
            if (iack) begin ipl7 <= 1'b0; pc_i <= 5'd0; st <= VEC; end
          end else if (t_rst) begin srst <= 1'b0; two <= 1'b0; pc_i <= 5'd16; st <= VEC; end
          else begin pc_i <= pc_i + 5'd1; if (&pc_i) two <= 1'b1; end
        end
        VEC: if (rd_end) begin
          pc_i <= pc_i + 5'd1;
          if (head) begin if (pc_i == 5'd1) st <= RUN; end
          else if (pc_i == 5'd19) begin pc_i <= 5'd10; st <= RUN; end
        end
        RUN: if (rest & wr_end) begin rel <= 1'b0; st <= ADV; end       // a write in the restore tail: the CPU is pushing its address error frame on the odd stack (halted slot into a running CPU) and halts
        else if (prog_end) begin
          pc_i <= pc_i + 5'd1;
          if (pc_i == (head ? 5'd11 : 5'd16)) begin rel <= 1'b0; st <= ADV; end
        end
        SHIFT: begin two <= ~two; if (two) st <= ADV; end              // two chain advances
        ADV: begin
          if (!last) begin k <= k + 1'b1; st <= NEXT; end
          else if (head) begin ss_mem_write <= 1'b1; st <= STRA; end
          else begin ss_pause <= 1'b0; st <= IDLE; end
        end
        STRA: if (busy_q) st <= STRB;
        STRB: if (!busy_q) begin
          if (head) begin k <= '0; ph <= PH_TAIL; st <= NEXT; end
          else if (!empty_q) begin k <= '0; st <= NEXT; end
          else begin ss_pause <= 1'b0; st <= IDLE; end
        end
        default: st <= IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------- outputs from flops
  logic [15:0] word_q = 16'd0;
  logic        hide_q = 1'b0;
  always_ff @(posedge clk) begin
    if (rst) begin word_q <= 16'd0; hide_q <= 1'b0; end
    else begin
      word_q <= rom(pc_i);
      // during the save AND the restore the system IPL is hidden (the CPU sees only 0 -> 7 of the module); outside the
      // savestate it passes through. A level-7 request of the system during a handler, not maskable (MC68000UM 6.3.2
      // p. 6-13), would otherwise be taken inside the handler. The hiding ends with the RTE of the handler (RUN with
      // pc_i[4]: program read 15 completed, the NOP that fills the prefetch queue before the RTE; in RUN pc_i is at most
      // 16, so pc_i[4] is pc_i == 16): the request of the system pending at the pause is sampled during the RTE (three
      // pops and the read at PC_B, four bus cycles, 16 periods of the 68000 clock without wait states; m68k_exc_si: ipl_w is loaded only when the CPU advances and keeps the level
      // while the CPU waits for the end of the pause) and taken at the end of the RTE, before the instruction at PC_B,
      // as in the run without savestate (UM 6.3.1 p. 6-8; docs/external-savestate.md §4.6 and §4.9 point 6). Hiding it
      // until the end of the pause would let the level reach the CPU only in the clock of its resume, and the request
      // would be taken one instruction late. Six inputs: paused, st, pc_i[4].
      hide_q <= paused & (st != IDLE) & ~((st == RUN) & pc_i[4]);
    end
  end
  assign cpu_word = word_q;
  wire ovr_now = ((st == VEC) & rd_live) | ((st == RUN) & prg_live & ~pc_i[4]);
  wire ssp_now = (st == VEC) & rest & (pc_i[4:1] == 4'b1000);       // reads 16 and 17: SSP high and low
  generate for (g = 0; g < N; g = g + 1) begin : g_cpu
    wire  sel = (k == KW'(g));
    logic ovr_q = 1'b0, ssp_q = 1'b0;
    always_ff @(posedge clk) begin
      if (rst) begin ovr_q <= 1'b0; ssp_q <= 1'b0; end
      else begin ovr_q <= ovr_now & sel; ssp_q <= ssp_now & sel; end
    end
    assign cpu_ovr[g]        = ovr_q;
    assign cpu_sspsel[g]     = ssp_q;
    assign cpu_exec[g]       = rel & sel;
    assign cpu_ipl7[g]       = ipl7 & sel;
    // the CPUs not served keep the system IPL hidden also during the RTE window of the served one (paused & ~sel): they
    // are frozen then, so ipl_w samples nothing, and a level 7 of the system arms nmi_pend of a CPU only in its own RTE,
    // never in the body of its handler. With N = 1 the term is constant 0 and cpu_iplhide = hide_q.
    assign cpu_iplhide[g]    = hide_q | (paused & ~sel);
    assign cpu_soft_reset[g] = srst & sel;
  end endgenerate

endmodule

`default_nettype wire
