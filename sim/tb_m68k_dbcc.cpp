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

// tb_m68k_dbcc.cpp - DBcc Dn,<disp16> and Scc Dn, exhaustive over the 16 conditions x 16 combinations of N Z V C.
//
// Tests (expected values from the manual, no value taken from an execution):
//  - Scc Dn (PRM 4 Scc): 16 conditions x 16 NZVC values (X alternating): condition true -> low byte = 0xFF, false ->
//    0x00; the upper 24 bits of Dn (preloaded with a value other than 0 and 0xFF) are preserved; CCR untouched (SR saved
//    after every Scc, compared in full: T S mask and XNZVC). The Dn register runs through D0-D7.
//  - DBcc Dn (PRM 4 DBcc): the same 256 combinations. Condition true -> no decrement, execution continues in sequence
//    (counter 0x2468_0000: a wrong decrement would give 0x2468FFFF). Condition false -> only the low word is decremented:
//    for every false case four counters: 0x1234_0000 -> 0x1234FFFF = -1 -> exit in sequence; 0x5678_0005 -> 0004 branch;
//    0x9ABC_8000 -> 7FFF branch (crosses the sign, is not -1); 0xFFFF_0001 -> 0000 branch (zero does not terminate).
//    Upper half always intact. The path taken is seen from which of the two MOVE SR,abs.W writes (only one of the two
//    cells must be written: the other is outside the expected writes). Target: PC of the extension + disp (PRM 4 DBcc:
//    "the current instruction location plus two"), here +8 forward.
//  - a DBF loop with negative displacement (-4) of 4 iterations with ADDQ.L #1,A0 in the body (ADDQ on An does not touch
//    the flags).
//  - every bus cycle: address/RW/FC stable, UDS and LDS for a word, program FC for fetches and data FC for writes
//    (supervisor), no read or write outside the expected areas; exact number of writes per word.
//  - everything twice: DTACK with zero wait states and DTACK delayed by 3 clocks; same expected values.
// Not tested: Scc on memory (testbenches sccmem/sccea), timing in clocks (UM 8), DBcc with backward displacement beyond -4.
// Conditions: PRM 3.6 Table 3-19 (cond_true below, written from the table).

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
static const char* const CCN[16] = {"T", "F", "HI", "LS", "CC", "CS", "NE", "EQ", "VC", "VS", "PL", "MI", "GE", "LT", "GT", "LE"};

// PRM 3.6 Table 3-19 "Conditional Tests" (nzvc: bit 3 N, bit 2 Z, bit 1 V, bit 0 C)
static bool cond_true(int cc, int nzvc) {
    const bool N = ((nzvc >> 3) & 1) != 0, Z = ((nzvc >> 2) & 1) != 0, V = ((nzvc >> 1) & 1) != 0, C = (nzvc & 1) != 0;
    switch (cc) {
        case 0x0: return true;                                    // T  : 1
        case 0x1: return false;                                   // F  : 0
        case 0x2: return !C && !Z;                                // HI : C' . Z'
        case 0x3: return C || Z;                                  // LS : C + Z
        case 0x4: return !C;                                      // CC : C'
        case 0x5: return C;                                       // CS : C
        case 0x6: return !Z;                                      // NE : Z'
        case 0x7: return Z;                                       // EQ : Z
        case 0x8: return !V;                                      // VC : V'
        case 0x9: return V;                                       // VS : V
        case 0xA: return !N;                                      // PL : N'
        case 0xB: return N;                                       // MI : N
        case 0xC: return (N && V) || (!N && !V);                  // GE : N.V + N'.V'
        case 0xD: return (N && !V) || (!N && V);                  // LT : N.V' + N'.V
        case 0xE: return (N && V && !Z) || (!N && !V && !Z);      // GT : N.V.Z' + N'.V'.Z'
        default:  return Z || (N && !V) || (!N && V);             // LE : Z + N.V' + N'.V
    }
}

static void build() {
    set_sr(0x2700);   // supervisor, T = 0, mask 7: explicit state

    // ---- Scc Dn: 0101 cccc 11 000 rrr (PRM 4 Scc). True -> low byte 0xFF, false -> 0x00, rest of Dn and CCR intact.
    for (int cc = 0; cc < 16; cc++) {
        for (int nzvc = 0; nzvc < 16; nzvc++) {
            const int r = (cc + 3 * nzvc) & 7;
            const uint8_t ccr = (uint8_t)((((cc + nzvc) & 1) << 4) | nzvc);
            const uint32_t pre = 0xC6A50000u | ((uint32_t)cc << 12) | ((uint32_t)nzvc << 8) | 0x5Au;   // low byte 0x5A
            const bool tr = cond_true(cc, nzvc);
            const std::string what = std::string("S") + CCN[cc] + " D" + std::to_string(r) + " CCR=" + hb(ccr) + (tr ? " (true)" : " (false)");
            ld_d(r, pre);
            set_ccr(ccr);
            w((uint16_t)(0x50C0 | (cc << 8) | r));
            save_sr(ccr, what);
            save_d(r, (pre & 0xFFFFFF00u) | (tr ? 0xFFu : 0x00u), what);
        }
    }

    // ---- DBcc Dn,+8: 0101 cccc 11001 rrr (PRM 4 DBcc). Layout of each case (X = address of the DBcc):
    //   X    DBcc Dr,X+10        (disp = +8 from the extension X+2)
    //   X+4  MOVE SR,(sf).W      exit in sequence (condition true, or counter reached -1)
    //   X+8  BRA.S X+14          (disp8 = +4)
    //   X+10 MOVE SR,(st).W      branch taken
    //   X+14 MOVE.L Dr,(sd).W
    static const uint32_t KF[4] = {0x12340000u, 0x56780005u, 0x9ABC8000u, 0xFFFF0001u};
    for (int cc = 0; cc < 16; cc++) {
        for (int nzvc = 0; nzvc < 16; nzvc++) {
            const int r = (cc + 5 * nzvc) & 7;
            const uint8_t ccr = (uint8_t)(((((cc >> 1) + nzvc) & 1) << 4) | nzvc);
            const bool tr = cond_true(cc, nzvc);
            const int nk = tr ? 1 : 4;
            for (int j = 0; j < nk; j++) {
                const uint32_t k = tr ? 0x24680000u : KF[j];
                uint32_t res = k; bool fall = true;
                if (!tr) {   // false: low word - 1; if it becomes -1 (0xFFFF) execution exits in sequence, otherwise it branches
                    const uint16_t lo = (uint16_t)((k & 0xFFFFu) - 1u);
                    res = (k & 0xFFFF0000u) | lo;
                    fall = (lo == 0xFFFFu);
                }
                const std::string what = std::string("DB") + CCN[cc] + " D" + std::to_string(r) + " CCR=" + hb(ccr) + " count=" + hx(k) +
                                         (tr ? " (true)" : " (false)") + (fall ? " exit in sequence" : " branch taken");
                ld_d(r, k);
                set_ccr(ccr);
                w((uint16_t)(0x50C8 | (cc << 8) | r)); w(0x0008);
                const uint32_t sf = res_ptr; res_ptr += 2;
                w(0x40F8); w((uint16_t)sf);
                w(0x6004);
                const uint32_t st = res_ptr; res_ptr += 2;
                w(0x40F8); w((uint16_t)st);
                expect_sr(fall ? sf : st, ccr, what);
                save_d(r, res, what);
            }
        }
    }

    // ---- DBF with negative displacement: body ADDQ.L #1,A0 (0101 001 0 10 001 000; PRM 4 ADDQ: on An the flags do not change)
    //   D4 = 0xABCD0003: iterations with D4.W 3->2->1->0->-1 = 4 executions of the body; A0 = 0x00ABC000 + 4; D4 = 0xABCDFFFF.
    ld_a(0, 0x00ABC000u);
    ld_d(4, 0xABCD0003u);
    set_ccr(0x0B);
    const uint32_t L = here();
    w(0x5288);                                   // ADDQ.L #1,A0
    const uint32_t X = here();
    w(0x51CC); w((uint16_t)(L - (X + 2)));       // DBF D4,L: disp = L - (X+2) = -4
    save_sr(0x0B, "DBF D4 loop of 4 backward iterations");
    save_d(4, 0xABCDFFFFu, "DBF D4 loop of 4 backward iterations");
    save_a(0, 0x00ABC004u, "DBF D4 loop of 4 backward iterations (body executed 4 times)");
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
