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

// tb_m68k_extswap.cpp - EXT.W, EXT.L, SWAP, EXG (all three forms), with the flags.
//
// PRM 4 EXT: EXT.W extends bit 7 onto bits 15-8 (bits 31-16 of Dn intact), EXT.L extends bit 15 onto bits 31-16;
//   X untouched, N = msb of the result of the size, Z = result of the size zero, V = C = 0.
// PRM 4 SWAP: swaps the two words of Dn; X untouched, N = bit 31 of the result, Z = 32-bit result zero, V = C = 0.
// PRM 4 EXG: swaps two 32-bit registers (Dx,Dy opmode 01000; Ax,Ay 01001; Dx,Ay 10001); condition codes "not affected".
// Cases: EXT.W on a negative byte, positive $7F, zero with a non-zero high word (Z of the word only), $01 under a high
//   byte $FF, $FF in D7; EXT.L on $8000, $7FFF, $0000 with a non-zero high part, $FFFF; EXT.W then EXT.L on the same
//   register with the intermediate value saved; SWAP with positive, negative, zero result, $0000FFFF (bit 15 at 1 but
//   N = 0), $00010000 (low word zero but Z = 0); EXG Dx,Dy / Ax,Ay / Dx,Ay with registers 0 and 7 in both positions
//   (A7 = SSP), EXG of a register with itself. Input CCR always non-zero, X at 1 and at 0; all registers
//   preloaded; the whole SR and all the registers involved saved after every instruction.
// Every cycle with FC, UDS/LDS and stability checked; two runs (DTACK 0 and +3 clocks).
// Not tested: timing in clocks.

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
static uint16_t extw(int d) { return (uint16_t)(0x4880 | d); }   // EXT.W Dn: 0100 100 010 000 rrr
static uint16_t extl(int d) { return (uint16_t)(0x48C0 | d); }   // EXT.L Dn: 0100 100 011 000 rrr
static uint16_t swp(int d)  { return (uint16_t)(0x4840 | d); }   // SWAP Dn:  0100 1000 0100 0 rrr
static uint16_t exg_dd(int x, int y) { return (uint16_t)(0xC140 | (x << 9) | y); }   // EXG Dx,Dy: 1100 xxx 1 01000 yyy
static uint16_t exg_aa(int x, int y) { return (uint16_t)(0xC148 | (x << 9) | y); }   // EXG Ax,Ay: 1100 xxx 1 01001 yyy
static uint16_t exg_da(int x, int y) { return (uint16_t)(0xC188 | (x << 9) | y); }   // EXG Dx,Ay: 1100 xxx 1 10001 yyy
// an operation on a single Dn: value before, expected value, CCR before and after
static void one(const std::string& what, uint16_t op, int d, uint32_t before, uint8_t ccr_in, uint32_t after, uint8_t ccr_out) {
    tcase(what, {{false, d, before}}, ccr_in, {op}, ccr_out, {{false, d, after}});
}
// EXG: the two registers swapped, CCR unchanged
static void ex(const std::string& what, uint16_t op, bool xa, int x, uint32_t xv, bool ya, int y, uint32_t yv, uint8_t ccr) {
    tcase(what, {{xa, x, xv}, {ya, y, yv}}, ccr, {op}, ccr, {{xa, x, yv}, {ya, y, xv}});
}

static void build() {
    set_sr(0x2700);   // supervisor, T = 0, mask 7
    // ---- EXT.W: bit 7 onto 15-8, 31-16 intact; N Z of the word, V = C = 0, X intact
    one("EXT.W $80",                   extw(0), 0, 0x12345680u, 0x17, 0x1234FF80u, 0x18);   // N=1, X=1
    one("EXT.W $7F",                   extw(1), 1, 0xFFFF007Fu, 0x0F, 0xFFFF007Fu, 0x00);   // positive, X=0
    one("EXT.W $00 (Z of the word)",  extw(2), 2, 0xAAAA5500u, 0x1B, 0xAAAA0000u, 0x14);   // word 0 -> Z=1, X=1
    one("EXT.W $01 under a byte $FF", extw(3), 3, 0x0000FF01u, 0x08, 0x00000001u, 0x00);
    one("EXT.W $FF in D7",             extw(7), 7, 0x800000FFu, 0x01, 0x8000FFFFu, 0x08);   // N=1
    // ---- EXT.L: bit 15 onto 31-16; N Z of the long, V = C = 0, X intact
    one("EXT.L $8000",                 extl(4), 4, 0x00008000u, 0x1E, 0xFFFF8000u, 0x18);   // N=1, X=1
    one("EXT.L $7FFF",                 extl(5), 5, 0xFFFF7FFFu, 0x0D, 0x00007FFFu, 0x00);
    one("EXT.L $0000",                 extl(6), 6, 0x12340000u, 0x12, 0x00000000u, 0x14);   // Z=1, X=1
    one("EXT.L $FFFF in D0",           extl(0), 0, 0x5555FFFFu, 0x06, 0xFFFFFFFFu, 0x08);   // N=1
    // ---- EXT.W then EXT.L on the same register, intermediate value saved
    one("EXT.W (chain) $80",          extw(1), 1, 0x00AA0080u, 0x1F, 0x00AAFF80u, 0x18);
    tcase("EXT.L (chain) $FF80", {}, 0x05, {extl(1)}, 0x08, {{false, 1, 0xFFFFFF80u}});    // D1 from the previous case; N=1, X=0
    // ---- SWAP: N = bit 31, Z = 32 bits zero, V = C = 0, X intact
    one("SWAP $12345678",              swp(1), 1, 0x12345678u, 0x1F, 0x56781234u, 0x10);
    one("SWAP $00008000",              swp(2), 2, 0x00008000u, 0x03, 0x80000000u, 0x08);   // N=1
    one("SWAP $00000000",              swp(3), 3, 0x00000000u, 0x1B, 0x00000000u, 0x14);   // Z=1, X=1
    one("SWAP $FFFF0000 (N from bit 31)", swp(4), 4, 0xFFFF0000u, 0x0C, 0x0000FFFFu, 0x00);
    one("SWAP $80000001 in D7",        swp(7), 7, 0x80000001u, 0x19, 0x00018000u, 0x10);   // X=1
    one("SWAP $00000001 (Z on 32 bits)", swp(0), 0, 0x00000001u, 0x04, 0x00010000u, 0x00);
    // ---- EXG: exchange of the 32 bits, flags intact
    ex("EXG D2,D3", exg_dd(2, 3), false, 2, 0xA1B2C3D4u, false, 3, 0x5E6F7081u, 0x15);
    ex("EXG D0,D7", exg_dd(0, 7), false, 0, 0x01234567u, false, 7, 0x89ABCDEFu, 0x0A);
    ex("EXG D7,D0", exg_dd(7, 0), false, 7, 0x13579BDFu, false, 0, 0x2468ACE0u, 0x1F);
    tcase("EXG D3,D3", {{false, 3, 0xC0FFEE11u}}, 0x09, {exg_dd(3, 3)}, 0x09, {{false, 3, 0xC0FFEE11u}});
    ex("EXG A1,A2", exg_aa(1, 2), true, 1, 0x00A0B0C0u, true, 2, 0xFF000010u, 0x11);
    ex("EXG A7,A0", exg_aa(7, 0), true, 7, 0x00DEAD00u, true, 0, 0x00BEEF00u, 0x0E);   // A7 = SSP
    ex("EXG A0,A7", exg_aa(0, 7), true, 0, 0x00112230u, true, 7, 0x00445560u, 0x13);
    ex("EXG D4,A5", exg_da(4, 5), false, 4, 0xCCCC1234u, true, 5, 0xDDDD5678u, 0x1C);
    ex("EXG D7,A7", exg_da(7, 7), false, 7, 0x7A7A7A7Au, true, 7, 0x00A7A7A6u, 0x07);
    ex("EXG D0,A0", exg_da(0, 0), false, 0, 0xFEDCBA98u, true, 0, 0x76543210u, 0x18);
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
