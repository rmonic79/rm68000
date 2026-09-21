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

// tb_m68k_idx.cpp - indexed mode d8(An,Xn) (PRM 2.2.8): EA = An + Xn + d8, Xn.W = sign-extended low word, Xn.L = 32
// bits, d8 sign-extended; brief extension word: bit 15 D/A, 14-12 register, 11 W/L, 10-8 = 000, 7-0 d8.
//
// Tests, with LEA (pure computation, no access) and with MOVE.W d8(An,Xn),Dn (data read):
//  - DISCRIMINATING indices: the upper part of every index differs from the sign extension of its low word, so .W and
//    .L give different addresses and every register is tested in both forms: D2 = 0xFFFF0020 (.W +$20, .L negative),
//    D5 = 0x00018000 (.W -$8000, .L +$18000), A3 = 0x0001FFF0 (.W -$10), D7 = 0x7FFF0004, A7 = 0x0002FF00 (.W -$100),
//    D0 = 0x12340000 / 0x00010000 (.W 0);
//  - index Dn and An (also A7 and D7, D0), index = base (A1,A1.L), index = destination, 32-bit wraparound
//    (0xFFFFFFF0 + ...);
//  - d8 positive, negative, $7F, -$80, 0;
//  - reads: for every read the address of the wrong form (W instead of L and vice versa) also holds a different data
//    value (a wrong form reads the wrong value, not an empty cell); supervisor data FC; the data goes into the low word
//    of Dn (upper part preloaded and intact); MOVE flags (PRM 4 MOVE: N Z from the data, V C at 0, X intact) with a
//    nonzero input CCR; for LEA the CCR is unchanged.
//  - every cycle: FC, UDS/LDS, stability; no read or write outside the areas; two runs (DTACK 0 and +3 clocks).
// Not tested: the PC-relative modes (pcrel), indexed PEA (pea2), timing in clocks.

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <map>
#include <vector>
#include <string>

// =====================================================================================================================
// Common testbench (identical in the per-instruction testbenches of this group): byte memory that honors UDS/LDS, log of
// all bus cycles, DTACK with zero wait states or delayed, program built word by word with the expected values alongside.
// =====================================================================================================================
static Vm68k_seq* dut = nullptr;
static uint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static constexpr uint32_t PROG_BASE  = 0x4000;    // pc_init: first opcode of the program
static constexpr uint32_t RES_BASE   = 0x1000;    // results area: positive abs.W addresses (PRM 2.2.15)
static constexpr uint64_t TICK_LIMIT = 4000000;   // generous tick limit to reach the final loop
static constexpr int FC_UD = 1, FC_UP = 2, FC_SD = 5, FC_SP = 6;   // UM: FC2-FC0 user and supervisor data/program

static int n_checks = 0, n_fail = 0;
static std::string tag;   // DTACK configuration of the current run

static std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "0x%06X", (unsigned)v); return std::string(b); }
static std::string hb(uint32_t v) { char b[8]; snprintf(b, sizeof b, "%02X", (unsigned)(v & 0xFFu)); return std::string(b); }
static void check_eq(const std::string& what, uint32_t exp, uint32_t got) {
    n_checks++;
    if (exp != got) {
        n_fail++;
        if (n_fail <= 300) printf("FAIL: %s %s expected=0x%X read=0x%X\n", tag.c_str(), what.c_str(), (unsigned)exp, (unsigned)got);
    }
}

// --- program and expected values ---
struct ExpWord { uint16_t val; uint16_t mask; std::string what; };
struct ExpWr   { int count; int fc; };
struct Code    { uint32_t lo, hi; unsigned fcmask; };
static std::vector<uint16_t>       prog;
static bool                        user_mode = false;     // mode of the program section being built
static uint16_t                    sys_byte  = 0x2700;    // T S mask expected in SR
static uint32_t                    res_ptr   = RES_BASE;
static uint32_t                    end_pc    = 0;
static std::map<uint32_t, ExpWord> exp_words;   // word (even address, 24 bits) -> expected value at end of run
static std::map<uint32_t, ExpWr>   exp_wr;      // word -> number of expected writes and expected FC
static std::map<uint32_t, int>     rd_ok;       // readable data word -> expected FC of the read
static std::vector<Code>           code;        // program sections with the FCs allowed for the fetch
static uint32_t                    code_lo = PROG_BASE;
static std::map<uint32_t, uint8_t> init_bytes;  // initial data and sentinels (bytes)

static uint32_t here() { return PROG_BASE + 2u * (uint32_t)prog.size(); }
static void w(uint16_t x) { prog.push_back(x); }
static int fc_data() { return user_mode ? FC_UD : FC_SD; }
static unsigned fc_prog_mask() { return user_mode ? (1u << FC_UP) : (1u << FC_SP); }

static void exp_word(uint32_t a, uint16_t v, const std::string& what, uint16_t mask = 0xFFFF) {
    a &= 0xFFFFFEu;
    exp_words[a] = ExpWord{v, mask, what};
    ExpWr& e = exp_wr[a]; e.count++; e.fc = fc_data();
}
static void exp_long(uint32_t a, uint32_t v, const std::string& what) {   // long in memory: high word at the low address
    exp_word(a, (uint16_t)(v >> 16), what + "[31:16]");
    exp_word(a + 2, (uint16_t)v, what + "[15:0]");
}
static void put_w(uint32_t a, uint16_t v) { init_bytes[a & 0xFFFFFFu] = (uint8_t)(v >> 8); init_bytes[(a + 1) & 0xFFFFFFu] = (uint8_t)v; }
static void data_w(uint32_t a, uint16_t v, int fc) {
    a &= 0xFFFFFEu;
    if (rd_ok.count(a)) check_eq("testbench: data already defined at " + hx(a), 0, 1);
    put_w(a, v); rd_ok[a] = fc;
}
static void sentinel(uint32_t lo, uint32_t hi) { for (uint32_t a = lo; a < hi; a++) init_bytes[a & 0xFFFFFFu] = (uint8_t)(0x3Cu ^ (a & 0xFFu)); }

static void set_ccr(uint8_t v) { w(0x44FC); w(v); }                 // MOVE #v,CCR (PRM 4 MOVE to CCR: 0100 0100 11, ea 111 100)
static void set_sr(uint16_t v) {                                     // MOVE #v,SR (PRM 4 MOVE to SR: 0100 0110 11, ea 111 100)
    w(0x46FC); w(v);
    const bool u = (v & 0x2000u) == 0;
    if (u != user_mode) {
        // mode change: the two words after the MOVE can be fetched before or after the SR write (the manual
        // does not specify it): for those two both program FCs are allowed
        const uint32_t sw = here();
        code.push_back(Code{code_lo, sw, fc_prog_mask()});
        code.push_back(Code{sw, sw + 4, (1u << FC_UP) | (1u << FC_SP)});
        code_lo = sw + 4;
        user_mode = u;
    }
    sys_byte = (uint16_t)(v & 0xA700u);
}
static void ld_d(int n, uint32_t v) { w((uint16_t)(0x203C | (n << 9))); w((uint16_t)(v >> 16)); w((uint16_t)v); }   // MOVE.L #v,Dn
static void ld_a(int n, uint32_t v) { w((uint16_t)(0x207C | (n << 9))); w((uint16_t)(v >> 16)); w((uint16_t)v); }   // MOVEA.L #v,An
// MOVE SR,(a).W (PRM 4 MOVE from SR: 0100 0000 11, ea 111 000): expected SR = T S mask | CCR; bits 7-5 are 0
static void expect_sr(uint32_t a, uint8_t ccr, const std::string& what, uint8_t ccr_mask = 0x1F) {
    exp_word(a, (uint16_t)(sys_byte | ccr), what + ": SR", (uint16_t)(0xFFE0u | ccr_mask));
    rd_ok[a] = fc_data();   // the cell may be read before the write (UM 8: MOVE from SR to memory has a read)
}
static void save_sr(uint8_t ccr, const std::string& what, uint8_t ccr_mask = 0x1F) {
    const uint32_t a = res_ptr; res_ptr += 2;
    w(0x40F8); w((uint16_t)a);
    expect_sr(a, ccr, what, ccr_mask);
}
static void save_d(int n, uint32_t v, const std::string& what) {    // MOVE.L Dn,(a).W: 0010 000 111 000 nnn
    const uint32_t a = res_ptr; res_ptr += 4;
    w((uint16_t)(0x21C0 | n)); w((uint16_t)a);
    exp_long(a, v, what + ": D" + std::to_string(n));
}
static void save_a(int n, uint32_t v, const std::string& what) {    // MOVE.L An,(a).W: 0010 000 111 001 nnn
    const uint32_t a = res_ptr; res_ptr += 4;
    w((uint16_t)(0x21C8 | n)); w((uint16_t)a);
    exp_long(a, v, what + ": A" + std::to_string(n));
}
// one register: a = address register; pcrel = v is a displacement from the address of the first extension word
struct Rv { bool a; int r; uint32_t v; bool pcrel = false; };
static void ld_rv(const Rv& x) { if (x.a) ld_a(x.r, x.v); else ld_d(x.r, x.v); }
// one case: loads the registers, the input CCR, the instruction, then SR and the saved registers with their expected value.
// Returns the address of the first extension word of the instruction (reference for the PC-relative modes, PRM 2.2.11).
static uint32_t tcase(const std::string& what, const std::vector<Rv>& loads, uint8_t ccr_in, const std::vector<uint16_t>& words,
                      uint8_t ccr_out, const std::vector<Rv>& expect, uint8_t ccr_mask = 0x1F) {
    for (const Rv& x : loads) ld_rv(x);
    set_ccr(ccr_in);
    const uint32_t ext_at = here() + 2;
    for (uint16_t x : words) w(x);
    save_sr(ccr_out, what, ccr_mask);
    for (const Rv& x : expect) {
        const uint32_t v = x.pcrel ? ext_at + x.v : x.v;
        if (x.a) save_a(x.r, v, what); else save_d(x.r, v, what);
    }
    return ext_at;
}
static void finish_prog() {
    end_pc = here();
    w(0x60FE);                                  // BRA.S * (PRM 4 Bcc: 0110 0000, disp8 = -2): end of run
    for (int i = 0; i < 8; i++) w(0x4E71);      // NOP: prefetches beyond the final loop
    code.push_back(Code{code_lo, here(), fc_prog_mask()});
    for (const auto& r : rd_ok) for (const Code& c : code) if (r.first >= c.lo && r.first < c.hi) check_eq("testbench: readable data inside the program " + hx(r.first), 0, 1);
    for (const auto& r : exp_wr) for (const Code& c : code) if (r.first >= c.lo && r.first < c.hi) check_eq("testbench: expected write inside the program " + hx(r.first), 0, 1);
    for (const auto& b : init_bytes) if (b.first + 16 >= RES_BASE && b.first < res_ptr + 16) check_eq("testbench: data inside the results area " + hx(b.first), 0, 1);
    if (res_ptr > 0x7FF0u) check_eq("testbench: results area beyond the positive abs.W range", 0, 1);
}

// --- run ---
struct Cyc { uint32_t addr = 0; bool rw = true; int fc = 0; bool uds = false, lds = false, unstable = false, wlat = false, wu = false, wl = false; uint16_t wdata = 0; };
static std::map<uint32_t, uint8_t> mem;
static std::map<uint32_t, int>     wr_count;
static int end_fetch = 0, n_cyc = 0;

static uint16_t mem_word(uint32_t a) {
    a &= 0xFFFFFEu;
    auto h = mem.find(a), l = mem.find(a + 1);
    const uint32_t hv = (h != mem.end()) ? h->second : 0xFFu;
    const uint32_t lv = (l != mem.end()) ? l->second : 0xFFu;
    return (uint16_t)((hv << 8) | lv);
}
static void close_cycle(const Cyc& c) {
    const std::string at = std::string(c.rw ? "read" : "write") + " at " + hx(c.addr);
    const int k = n_cyc++;
    check_eq(at + ": address, R/W and FC stable while AS is asserted (UM 5.1)", 0, c.unstable ? 1u : 0u);
    check_eq(at + ": UDS and LDS asserted (word access, UM 5.1)", 3, (c.uds ? 2u : 0u) | (c.lds ? 1u : 0u));
    if (k < 4) {   // the first four cycles after reset: SSP and PC from vectors 0-7 (UM 6.3.1); the data is supplied by the shim
        check_eq(at + ": cycle " + std::to_string(k) + " after reset = read of the reset vector", 1, (c.rw && c.addr < 8) ? 1u : 0u);
        return;
    }
    if (c.rw) {
        for (const Code& r : code) {
            if (c.addr >= r.lo && c.addr < r.hi) {
                unsigned want = (unsigned)c.fc;
                if (!((r.fcmask >> c.fc) & 1u)) { want = 0; while (!((r.fcmask >> want) & 1u)) want++; }
                check_eq(at + ": FC of the fetch", want, (uint32_t)c.fc);
                if (c.addr == end_pc) end_fetch++;
                return;
            }
        }
        auto it = rd_ok.find(c.addr);
        if (it == rd_ok.end()) { check_eq(at + ": read outside the allowed areas", 0, 1); return; }
        check_eq(at + ": FC of the read", (uint32_t)it->second, (uint32_t)c.fc);
    } else {
        auto it = exp_wr.find(c.addr);
        if (it == exp_wr.end()) check_eq(at + ": write outside the expected writes", 0, 1);
        else { check_eq(at + ": FC of the write", (uint32_t)it->second.fc, (uint32_t)c.fc); wr_count[c.addr]++; }
        check_eq(at + ": data driven (data_oe) with the strobes", 1, c.wlat ? 1u : 0u);
        if (c.wlat) {
            check_eq(at + ": write lanes (word)", 3, (c.wu ? 2u : 0u) | (c.wl ? 1u : 0u));
            if (c.wu) mem[c.addr] = (uint8_t)(c.wdata >> 8);
            if (c.wl) mem[c.addr + 1] = (uint8_t)(c.wdata & 0xFFu);
        }
    }
}
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }

static void run(int wait) {
    tag = "[DTACK +" + std::to_string(wait) + "]";
    mem = init_bytes;
    for (uint32_t a = RES_BASE - 16; a < res_ptr + 16; a++) mem[a] = (uint8_t)(0xA5u ^ (a & 0xFFu));   // result sentinels
    for (size_t i = 0; i < prog.size(); i++) {
        const uint32_t a = PROG_BASE + 2u * (uint32_t)i;
        mem[a] = (uint8_t)(prog[i] >> 8); mem[a + 1] = (uint8_t)(prog[i] & 0xFFu);
    }
    wr_count.clear(); end_fetch = 0; n_cyc = 0;
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    dut->clk = 0; dut->rst_n = 0; dut->cen = 1; dut->cen2 = 1; dut->start = 0;
    dut->pc_init = PROG_BASE; dut->rst_vector = 0;
    dut->ipl = 0; dut->vpa_n = 1; dut->data_in = 0; dut->dtack_n = 1;
    dut->halt_n = 1; dut->berr_n = 1; dut->br_n = 1; dut->bgack_n = 1;
    dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    dut->eval();
    for (int i = 0; i < 8; i++) tick();
    dut->rst_n = 1;
    bool in_cyc = false; int age = 0; Cyc cur; uint64_t n = 0;
    while (n < TICK_LIMIT && end_fetch < 3) {
        if (!dut->as_n) {
            const uint32_t a = ((uint32_t)dut->addr) << 1;
            const bool rw = dut->rw != 0;
            const int fc = (int)dut->fc;
            if (!in_cyc) { in_cyc = true; age = 0; cur = Cyc(); cur.addr = a; cur.rw = rw; cur.fc = fc; }
            else { age++; if (a != cur.addr || rw != cur.rw || fc != cur.fc) cur.unstable = true; }
            if (!dut->uds_n) cur.uds = true;
            if (!dut->lds_n) cur.lds = true;
            if (!rw && dut->data_oe && (!dut->uds_n || !dut->lds_n)) {
                cur.wlat = true; cur.wdata = (uint16_t)dut->data_out; cur.wu = !dut->uds_n; cur.wl = !dut->lds_n;
            }
            dut->data_in = mem_word(a);
            dut->dtack_n = (age >= wait) ? 0 : 1;
        } else {
            if (in_cyc) { in_cyc = false; close_cycle(cur); }
            dut->dtack_n = 1;
        }
        tick(); n++;
    }
    check_eq("end of run: fetches of the final BRA.S * at " + hx(end_pc), 3, (uint32_t)end_fetch);
    check_eq("illegal output at end of run", 0, dut->illegal ? 1u : 0u);
    for (const auto& e : exp_words)
        check_eq(e.second.what + " (word " + hx(e.first) + ")", (uint32_t)(e.second.val & e.second.mask), (uint32_t)(mem_word(e.first) & e.second.mask));
    for (const auto& e : exp_wr) {
        auto it = wr_count.find(e.first);
        check_eq("number of writes of word " + hx(e.first), (uint32_t)e.second.count, it == wr_count.end() ? 0u : (uint32_t)it->second);
    }
    dut->final(); delete dut; dut = nullptr;
}

// =====================================================================================================================
// Cases of this testbench
// =====================================================================================================================
static const uint8_t CCRS[] = {0x1F, 0x15, 0x0A, 0x11, 0x04, 0x1B, 0x08, 0x13, 0x0E, 0x01, 0x1C, 0x02, 0x17, 0x09,
                               0x10, 0x06, 0x1D, 0x0F, 0x12, 0x05, 0x19, 0x0C, 0x1E, 0x03, 0x16, 0x07, 0x18, 0x0D};
static int ci = 0;
static uint8_t next_ccr() { const uint8_t c = CCRS[ci % (int)sizeof(CCRS)]; ci++; return c; }   // all nonzero
static uint32_t pre(int i) { return 0xA5000000u | ((uint32_t)(i & 0xFF) << 16) | 0xC3C2u; }      // preload of the destination
static uint16_t lea(int an, int mode, int reg) { return (uint16_t)(0x41C0 | (an << 9) | (mode << 3) | reg); }   // PRM 4 LEA
// brief extension word (PRM 2.2.8): da = 1 index An, l = 1 long index
static uint16_t brief(int da, int reg, int l, int d8) { return (uint16_t)((da << 15) | (reg << 12) | (l << 11) | (d8 & 0xFF)); }
static void lc(const std::string& what, const std::vector<Rv>& loads, const std::vector<uint16_t>& words, const std::vector<Rv>& expect) {
    const uint8_t c = next_ccr();
    tcase(what, loads, c, words, c, expect);   // LEA: CCR unchanged
}
// MOVE.W d8(An,Xn),Dn: 00 11 ddd 000 110 aaa (PRM 4 MOVE)
static uint16_t movew_idx(int dn, int an) { return (uint16_t)(0x3030 | (dn << 9) | an); }

static constexpr uint32_t A1V = 0x00100000u;

static void build() {
    set_sr(0x2700);   // supervisor, T = 0, mask 7
    int i = 0;
    // ---- LEA d8(An,Xn),Am: EA by hand (32-bit sums)
    lc("LEA 4(A1,D2.W),A0", {{true, 1, A1V}, {false, 2, 0xFFFF0020u}, {true, 0, pre(i++)}}, {lea(0, 6, 1), brief(0, 2, 0, 4)},
       {{true, 0, 0x00100024u}, {true, 1, A1V}, {false, 2, 0xFFFF0020u}});            // A1 + 0x20 + 4
    lc("LEA 4(A1,D2.L),A0", {{true, 1, A1V}, {false, 2, 0xFFFF0020u}, {true, 0, pre(i++)}}, {lea(0, 6, 1), brief(0, 2, 1, 4)},
       {{true, 0, 0x000F0024u}, {true, 1, A1V}, {false, 2, 0xFFFF0020u}});            // A1 + 0xFFFF0020 + 4
    lc("LEA $10(A1,D5.W),A2", {{true, 1, A1V}, {false, 5, 0x00018000u}, {true, 2, pre(i++)}}, {lea(2, 6, 1), brief(0, 5, 0, 0x10)},
       {{true, 2, 0x000F8010u}, {true, 1, A1V}, {false, 5, 0x00018000u}});            // A1 + 0xFFFF8000 + 0x10
    lc("LEA $10(A1,D5.L),A2", {{true, 1, A1V}, {false, 5, 0x00018000u}, {true, 2, pre(i++)}}, {lea(2, 6, 1), brief(0, 5, 1, 0x10)},
       {{true, 2, 0x00118010u}, {true, 1, A1V}, {false, 5, 0x00018000u}});            // A1 + 0x18000 + 0x10
    lc("LEA -4(A1,A3.L),A4", {{true, 1, A1V}, {true, 3, 0x0001FFF0u}, {true, 4, pre(i++)}}, {lea(4, 6, 1), brief(1, 3, 1, -4)},
       {{true, 4, 0x0011FFECu}, {true, 1, A1V}, {true, 3, 0x0001FFF0u}});             // A1 + 0x1FFF0 - 4
    lc("LEA -4(A1,A3.W),A4", {{true, 1, A1V}, {true, 3, 0x0001FFF0u}, {true, 4, pre(i++)}}, {lea(4, 6, 1), brief(1, 3, 0, -4)},
       {{true, 4, 0x000FFFECu}, {true, 1, A1V}, {true, 3, 0x0001FFF0u}});             // A1 - 0x10 - 4
    lc("LEA $7F(A1,D7.W),A5", {{true, 1, A1V}, {false, 7, 0x7FFF0004u}, {true, 5, pre(i++)}}, {lea(5, 6, 1), brief(0, 7, 0, 0x7F)},
       {{true, 5, 0x00100083u}, {true, 1, A1V}, {false, 7, 0x7FFF0004u}});            // A1 + 4 + 0x7F
    lc("LEA -$80(A1,D7.L),A5", {{true, 1, A1V}, {false, 7, 0x7FFF0004u}, {true, 5, pre(i++)}}, {lea(5, 6, 1), brief(0, 7, 1, -128)},
       {{true, 5, 0x800EFF84u}, {true, 1, A1V}, {false, 7, 0x7FFF0004u}});            // A1 + 0x7FFF0004 - 0x80
    lc("LEA 0(A1,A7.W),A6", {{true, 1, A1V}, {true, 7, 0x0002FF00u}, {true, 6, pre(i++)}}, {lea(6, 6, 1), brief(1, 7, 0, 0)},
       {{true, 6, 0x000FFF00u}, {true, 1, A1V}, {true, 7, 0x0002FF00u}});             // A1 + 0xFFFFFF00
    lc("LEA 0(A1,A7.L),A6", {{true, 1, A1V}, {true, 7, 0x0002FF00u}, {true, 6, pre(i++)}}, {lea(6, 6, 1), brief(1, 7, 1, 0)},
       {{true, 6, 0x0012FF00u}, {true, 1, A1V}, {true, 7, 0x0002FF00u}});             // A1 + 0x2FF00
    lc("LEA 2(A1,A1.L),A3 (index = base)", {{true, 1, A1V}, {true, 3, pre(i++)}}, {lea(3, 6, 1), brief(1, 1, 1, 2)},
       {{true, 3, 0x00200002u}, {true, 1, A1V}});                                     // 2 * A1 + 2
    lc("LEA $20(A7,D0.W),A0 (base A7)", {{true, 7, 0x0002FF00u}, {false, 0, 0x12340000u}, {true, 0, pre(i++)}}, {lea(0, 6, 7), brief(0, 0, 0, 0x20)},
       {{true, 0, 0x0002FF20u}, {true, 7, 0x0002FF00u}, {false, 0, 0x12340000u}});    // A7 + 0 + 0x20
    lc("LEA $20(A7,D0.L),A0 (base A7)", {{true, 7, 0x0002FF00u}, {false, 0, 0x12340000u}, {true, 0, pre(i++)}}, {lea(0, 6, 7), brief(0, 0, 1, 0x20)},
       {{true, 0, 0x1236FF20u}, {true, 7, 0x0002FF00u}, {false, 0, 0x12340000u}});    // A7 + 0x12340000 + 0x20
    lc("LEA 8(A1,A3.L),A3 (index = destination)", {{true, 1, A1V}, {true, 3, 0x0001FFF0u}}, {lea(3, 6, 1), brief(1, 3, 1, 8)},
       {{true, 3, 0x0011FFF8u}, {true, 1, A1V}});                                     // A1 + 0x1FFF0 + 8
    lc("LEA -1(A4,D6.W),A5 (32-bit wraparound)", {{true, 4, 0xFFFFFFF0u}, {false, 6, 0x00000020u}, {true, 5, pre(i++)}}, {lea(5, 6, 4), brief(0, 6, 0, -1)},
       {{true, 5, 0x0000000Fu}, {true, 4, 0xFFFFFFF0u}, {false, 6, 0x00000020u}});    // 0xFFFFFFF0 + 0x20 - 1 mod 2^32

    // ---- reads MOVE.W d8(An,Xn),Dn: data in the low word of Dn; MOVE flags (N Z from the data, V = C = 0, X intact).
    //      Data: EA of the right form and, as a decoy, EA of the wrong form (W <-> L) with a different value.
    data_w(0x00100024u, 0x8001, FC_SD);   // 4(A1,D2.W)      and decoy for 4(A1,D2.L)
    data_w(0x000F0024u, 0x0000, FC_SD);   // 4(A1,D2.L)      and decoy for 4(A1,D2.W)
    data_w(0x00100000u, 0x7FFE, FC_SD);   // $10(A1,A3.W): A1 - 0x10 + 0x10
    data_w(0x00120000u, 0x2468, FC_SD);   // $10(A1,A3.L): A1 + 0x1FFF0 + 0x10
    data_w(0x000FFEFEu, 0x9ABC, FC_SD);   // -2(A1,A7.W):  A1 - 0x100 - 2
    data_w(0x0012FEFEu, 0xFFFF, FC_SD);   // -2(A1,A7.L):  A1 + 0x2FF00 - 2
    data_w(0x00210080u, 0x0101, FC_SD);   // $7F(A2,D0.L): 0x00200001 + 0x10000 + 0x7F
    data_w(0x00200080u, 0x8080, FC_SD);   //   decoy: $7F(A2,D0.W) = 0x00200001 + 0 + 0x7F
    data_w(0x001FFF80u, 0x00FF, FC_SD);   // -$80(A2,D0.W): 0x00200000 + 0 - 0x80
    data_w(0x0020FF80u, 0x5AA5, FC_SD);   //   decoy: -$80(A2,D0.L) = 0x00200000 + 0x10000 - 0x80
    tcase("MOVE.W 4(A1,D2.W),D6", {{true, 1, A1V}, {false, 2, 0xFFFF0020u}, {false, 6, 0xCAFE5555u}}, 0x1F, {movew_idx(6, 1), brief(0, 2, 0, 4)}, 0x18,
          {{false, 6, 0xCAFE8001u}, {true, 1, A1V}, {false, 2, 0xFFFF0020u}});        // 0x8001: N=1, X=1
    tcase("MOVE.W 4(A1,D2.L),D6", {{true, 1, A1V}, {false, 2, 0xFFFF0020u}, {false, 6, 0xCAFE5555u}}, 0x0B, {movew_idx(6, 1), brief(0, 2, 1, 4)}, 0x04,
          {{false, 6, 0xCAFE0000u}, {true, 1, A1V}, {false, 2, 0xFFFF0020u}});        // 0x0000: Z=1, X=0
    tcase("MOVE.W $10(A1,A3.W),D1", {{true, 1, A1V}, {true, 3, 0x0001FFF0u}, {false, 1, 0x13579BDFu}}, 0x11, {movew_idx(1, 1), brief(1, 3, 0, 0x10)}, 0x10,
          {{false, 1, 0x13577FFEu}, {true, 1, A1V}, {true, 3, 0x0001FFF0u}});         // 0x7FFE: X=1
    tcase("MOVE.W $10(A1,A3.L),D1", {{true, 1, A1V}, {true, 3, 0x0001FFF0u}, {false, 1, 0x13579BDFu}}, 0x0E, {movew_idx(1, 1), brief(1, 3, 1, 0x10)}, 0x00,
          {{false, 1, 0x13572468u}, {true, 1, A1V}, {true, 3, 0x0001FFF0u}});         // 0x2468: all 0
    tcase("MOVE.W -2(A1,A7.W),D3", {{true, 1, A1V}, {true, 7, 0x0002FF00u}, {false, 3, 0x0F0F0F0Fu}}, 0x04, {movew_idx(3, 1), brief(1, 7, 0, -2)}, 0x08,
          {{false, 3, 0x0F0F9ABCu}, {true, 1, A1V}, {true, 7, 0x0002FF00u}});         // 0x9ABC: N=1
    tcase("MOVE.W -2(A1,A7.L),D3", {{true, 1, A1V}, {true, 7, 0x0002FF00u}, {false, 3, 0x0F0F0F0Fu}}, 0x13, {movew_idx(3, 1), brief(1, 7, 1, -2)}, 0x18,
          {{false, 3, 0x0F0FFFFFu}, {true, 1, A1V}, {true, 7, 0x0002FF00u}});         // 0xFFFF: N=1, X=1
    tcase("MOVE.W $7F(A2,D0.L),D4", {{true, 2, 0x00200001u}, {false, 0, 0x00010000u}, {false, 4, 0x44444444u}}, 0x1A, {movew_idx(4, 2), brief(0, 0, 1, 0x7F)}, 0x10,
          {{false, 4, 0x44440101u}, {true, 2, 0x00200001u}, {false, 0, 0x00010000u}}); // 0x0101: X=1
    tcase("MOVE.W -$80(A2,D0.W),D4", {{true, 2, 0x00200000u}, {false, 0, 0x00010000u}, {false, 4, 0x44444444u}}, 0x0C, {movew_idx(4, 2), brief(0, 0, 0, -128)}, 0x00,
          {{false, 4, 0x444400FFu}, {true, 2, 0x00200000u}, {false, 0, 0x00010000u}}); // 0x00FF: all 0
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    build();
    finish_prog();
    run(0);
    run(3);
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    if (n_fail == 0) { printf("RESULT: PASS\n"); return 0; }
    printf("RESULT: FAIL\n");
    return 1;
}
