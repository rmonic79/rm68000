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

// tb_m68k_muls.cpp - MULS.W and DIVS.W (signed), source Dn or #imm, complete flags and edge operands.
//
// PRM 4 MULS: Dn(16) x src(16) -> Dn(32) signed; X not affected, N = bit 31 of the result, Z = zero result, V = 0,
//   C = 0. Only the low words are used: the high parts of the two operands are preloaded with different values.
// PRM 4 DIVS: Dn(32) / src(16) signed -> Dn = remainder(16):quotient(16); quotient truncated toward zero, remainder with the sign
//   of the dividend; X not affected, N = negative quotient, Z = zero quotient, V = overflow, C = 0. Overflow
//   (quotient outside -32768..32767): V = 1, C = 0, "the operands are unaffected" (Dn intact), N and Z undefined (the
//   testbench masks them only in those cases).
// MULS cases: 3 x -2, 0 x $7FFF, $7FFF x $7FFF, -32768 x -32768, -32768 x $7FFF, -1 x -1, -1 x 1, D1 x D1 (same
//   register), #$8000 x 1, $8000 x 0 (negative times zero). DIVS cases: the four signs with remainder, remainder 0, quotient 0 with
//   positive and negative dividend, quotient -32768 exact and with remainder -1, quotient +32767 exact, overflows
//   +32768 (two dividends), $80000000 / -1, $7FFFFFFF / 1, negative #imm. Input CCR always non-zero (X at 1 and at 0).
// Each case: SR saved right after the instruction, then Dn and the source (intact); every cycle with FC, UDS/LDS and
// stability checked; two runs (DTACK 0 and +3 clocks).
// Does not test: division by zero (exception, div0 testbench), memory sources (muldivmem testbench), MULU/DIVU (muldiv testbench),
// timing in clocks.

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <map>
#include <vector>
#include <string>

// =====================================================================================================================
// Common testbench (identical in the per-instruction testbenches of this group): byte-wide memory that honours UDS/LDS, log of
// all bus cycles, DTACK with zero wait states or delayed, program built word by word with the expectations alongside.
// =====================================================================================================================
static Vm68k_seq* dut = nullptr;
static uint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static constexpr uint32_t PROG_BASE  = 0x4000;    // pc_init: first opcode of the program
static constexpr uint32_t RES_BASE   = 0x1000;    // results area: positive abs.W addresses (PRM 2.2.15)
static constexpr uint64_t TICK_LIMIT = 4000000;   // generous tick limit to reach the final cycle
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

// --- program and expectations ---
struct ExpWord { uint16_t val; uint16_t mask; std::string what; };
struct ExpWr   { int count; int fc; };
struct Code    { uint32_t lo, hi; unsigned fcmask; };
static std::vector<uint16_t>       prog;
static bool                        user_mode = false;     // mode of the program stretch under construction
static uint16_t                    sys_byte  = 0x2700;    // T S mask expected in SR
static uint32_t                    res_ptr   = RES_BASE;
static uint32_t                    end_pc    = 0;
static std::map<uint32_t, ExpWord> exp_words;   // word (even address, 24 bits) -> expected value at the end of the run
static std::map<uint32_t, ExpWr>   exp_wr;      // word -> expected number of writes and expected FC
static std::map<uint32_t, int>     rd_ok;       // readable data word -> expected FC of the read
static std::vector<Code>           code;        // program stretches with the FCs allowed for the fetch
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
        // mode change: the two words after the MOVE may be fetched before or after the write of SR (the manual
        // does not fix it): for those two words both program FCs are allowed
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
// a register: a = address register; pcrel = v is a displacement from the address of the first extension word
struct Rv { bool a; int r; uint32_t v; bool pcrel = false; };
static void ld_rv(const Rv& x) { if (x.a) ld_a(x.r, x.v); else ld_d(x.r, x.v); }
// a case: loads the registers, the input CCR, the instruction, then SR and the saved registers with their expected values.
// Returns the address of the first extension word of the instruction (reference of the PC-relative modes, PRM 2.2.11).
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
    for (int i = 0; i < 8; i++) w(0x4E71);      // NOP: prefetches beyond the final cycle
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
    check_eq(at + ": address, R/W and FC stable with AS asserted (UM 5.1)", 0, c.unstable ? 1u : 0u);
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
            check_eq(at + ": lanes of the write (word)", 3, (c.wu ? 2u : 0u) | (c.wl ? 1u : 0u));
            if (c.wu) mem[c.addr] = (uint8_t)(c.wdata >> 8);
            if (c.wl) mem[c.addr + 1] = (uint8_t)(c.wdata & 0xFFu);
        }
    }
}
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }

static void run(int wait) {
    tag = "[DTACK +" + std::to_string(wait) + "]";
    mem = init_bytes;
    for (uint32_t a = RES_BASE - 16; a < res_ptr + 16; a++) mem[a] = (uint8_t)(0xA5u ^ (a & 0xFFu));   // sentinels of the results
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
    check_eq("illegal output at the end of the run", 0, dut->illegal ? 1u : 0u);
    for (const auto& e : exp_words)
        check_eq(e.second.what + " (word " + hx(e.first) + ")", (uint32_t)(e.second.val & e.second.mask), (uint32_t)(mem_word(e.first) & e.second.mask));
    for (const auto& e : exp_wr) {
        auto it = wr_count.find(e.first);
        check_eq("number of writes of the word " + hx(e.first), (uint32_t)e.second.count, it == wr_count.end() ? 0u : (uint32_t)it->second);
    }
    dut->final(); delete dut; dut = nullptr;
}

// =====================================================================================================================
// Cases of this testbench
// =====================================================================================================================
static uint16_t muls(int dn, int src) { return (uint16_t)(0xC1C0 | (dn << 9) | src); }   // MULS.W Ds,Dn: 1100 nnn 111 000 sss
static uint16_t divs(int dn, int src) { return (uint16_t)(0x81C0 | (dn << 9) | src); }   // DIVS.W Ds,Dn: 1000 nnn 111 000 sss
// MULS Ds,Dn: expected result in Dn, source intact
static void mc(const std::string& what, int dn, uint32_t dv, int sn, uint32_t sv, uint8_t ccr_in, uint32_t res, uint8_t ccr_out) {
    tcase(what, {{false, dn, dv}, {false, sn, sv}}, ccr_in, {muls(dn, sn)}, ccr_out, {{false, dn, res}, {false, sn, sv}});
}
static void dc(const std::string& what, int dn, uint32_t dv, int sn, uint32_t sv, uint8_t ccr_in, uint32_t res, uint8_t ccr_out, uint8_t mask = 0x1F) {
    tcase(what, {{false, dn, dv}, {false, sn, sv}}, ccr_in, {divs(dn, sn)}, ccr_out, {{false, dn, res}, {false, sn, sv}}, mask);
}

static void build() {
    set_sr(0x2700);   // supervisor, T = 0, mask 7
    // ---- MULS.W: 32-bit product of the low words; flag X intact, N Z from the result, V = C = 0
    mc("MULS 3 x -2",               1, 0xABCD0003u, 0, 0x1234FFFEu, 0x13, 0xFFFFFFFAu, 0x18);   // -6; N=1, X=1
    mc("MULS 0 x $7FFF",            2, 0xFFFF0000u, 3, 0x00007FFFu, 0x0B, 0x00000000u, 0x04);   // 0; Z=1, X=0
    mc("MULS $7FFF x $7FFF",        4, 0x55557FFFu, 5, 0xAAAA7FFFu, 0x1F, 0x3FFF0001u, 0x10);   // 32767^2; X=1
    mc("MULS -32768 x -32768",      4, 0x00008000u, 5, 0x55558000u, 0x0F, 0x40000000u, 0x00);   // 2^30
    mc("MULS -32768 x $7FFF",       6, 0x7FFF8000u, 7, 0x80007FFFu, 0x04, 0xC0008000u, 0x08);   // -(0x3FFF8000); N=1
    mc("MULS -1 x -1",              6, 0xFFFFFFFFu, 7, 0x0000FFFFu, 0x1A, 0x00000001u, 0x10);   // 1; X=1
    mc("MULS -1 x 1",               0, 0x0000FFFFu, 1, 0xFFFF0001u, 0x05, 0xFFFFFFFFu, 0x08);   // -1; N=1
    mc("MULS $8000 x 0",            2, 0x00008000u, 3, 0xFFFF0000u, 0x19, 0x00000000u, 0x14);   // 0; Z=1, X=1
    tcase("MULS D1,D1 (same register)", {{false, 1, 0x7777FF00u}}, 0x17, {muls(1, 1)}, 0x10, {{false, 1, 0x00010000u}});   // (-256)^2; X=1
    tcase("MULS #$8000,D0", {{false, 0, 0xFFFF0001u}}, 0x02, {(uint16_t)(0xC1FC | (0 << 9)), 0x8000}, 0x08,
          {{false, 0, 0xFFFF8000u}});                                                           // 1 x -32768; N=1

    // ---- DIVS.W: Dn = remainder:quotient; quotient truncated toward zero, remainder with the sign of the dividend
    dc("DIVS -7 / 2",               2, 0xFFFFFFF9u, 3, 0xABCD0002u, 0x17, 0xFFFFFFFDu, 0x18);   // q -3, r -1; N=1, X=1
    dc("DIVS 7 / -2",               2, 0x00000007u, 3, 0x1234FFFEu, 0x02, 0x0001FFFDu, 0x08);   // q -3, r 1; N=1
    dc("DIVS -7 / -2",              4, 0xFFFFFFF9u, 5, 0x0000FFFEu, 0x1D, 0xFFFF0003u, 0x10);   // q 3, r -1; X=1
    dc("DIVS 100 / 10",             4, 0x00000064u, 5, 0xFFFF000Au, 0x0F, 0x0000000Au, 0x00);   // q 10, r 0
    dc("DIVS 5 / 7",                6, 0x00000005u, 7, 0x00000007u, 0x1B, 0x00050000u, 0x14);   // q 0, r 5; Z=1, X=1
    dc("DIVS -5 / 7",               6, 0xFFFFFFFBu, 7, 0x55550007u, 0x08, 0xFFFB0000u, 0x04);   // q 0, r -5; Z=1
    dc("DIVS -32768 / 1",           0, 0xFFFF8000u, 1, 0x00000001u, 0x13, 0x00008000u, 0x18);   // q -32768 (fits), r 0; N=1, X=1
    dc("DIVS $C0007FFF / $7FFF",    0, 0xC0007FFFu, 1, 0x00007FFFu, 0x0A, 0xFFFF8000u, 0x08);   // -1073709057/32767: q -32768, r -1
    dc("DIVS $3FFF0001 / $7FFF",    2, 0x3FFF0001u, 3, 0x00007FFFu, 0x0E, 0x00007FFFu, 0x00);   // q 32767 (maximum), r 0
    dc("DIVS $C0008000 / $7FFF",    2, 0xC0008000u, 3, 0xFFFF7FFFu, 0x15, 0x00008000u, 0x18);   // q -32768 exact; N=1, X=1
    // overflow: V = 1, C = 0, X intact, Dn intact; N and Z not defined by the manual -> mask X V C (0x13)
    dc("DIVS 32768 / 1 (overflows)", 4, 0x00008000u, 5, 0x00000001u, 0x11, 0x00008000u, 0x12, 0x13);
    dc("DIVS $3FFF8000 / $7FFF (overflows)", 4, 0x3FFF8000u, 5, 0x00007FFFu, 0x0C, 0x3FFF8000u, 0x02, 0x13);   // q = 32768
    dc("DIVS $80000000 / -1 (overflows)", 6, 0x80000000u, 7, 0x0000FFFFu, 0x04, 0x80000000u, 0x02, 0x13);
    dc("DIVS $7FFFFFFF / 1 (overflows)", 6, 0x7FFFFFFFu, 7, 0x00000001u, 0x1D, 0x7FFFFFFFu, 0x12, 0x13);
    tcase("DIVS #-3,D4", {{false, 4, 0x0000000Au}}, 0x1F, {(uint16_t)(0x81FC | (4 << 9)), 0xFFFD}, 0x18,
          {{false, 4, 0x0001FFFDu}});                                                           // 10 / -3: q -3, r 1; N=1, X=1
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
