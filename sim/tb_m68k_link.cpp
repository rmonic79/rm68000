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

// tb_m68k_link.cpp - LINK An,#d16 and UNLK An with An other than A7 (the An = A7 case is in the link_a7 testbench).
//
// Sources: PRM 4 LINK ("SP - 4 -> SP; An -> (SP); SP -> An; SP + d -> SP", signed word displacement; flags not
// touched), PRM 4 UNLK ("An -> SP; (SP) -> An; SP + 4 -> SP"; flags not touched).
// Cases (supervisor, SP = SSP): LINK A6,#-8; LINK A0,#0 (then MOVE.L (A0),D1 reads the stacked value and MOVE.L
// #imm,(A0) changes it: UNLK must reread memory); LINK A5,#+12 with A5 = 0; LINK A1,#-32768; LINK A2,#+32767 (odd SP as
// a value, no access). In user mode (SP = USP, loaded with MOVE A0,USP): LINK A4,#-4, stacked with user data FC.
// After LINK, SR (input CCR different for every case: the flags do not change), An and SP are saved; after UNLK again
// SR, An, SP.
// The stacked long is checked in memory (FC, the two words, surrounding sentinels intact).

#include "Vm68k_seq.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

// ==================================================================================================== common testbench
// (identical in the testbenches addx, movesr, srlogic, mvtsr, stop, tst, link, link_a7, abcd, chk, chk_neg)
// Byte memory that honors UDS/LDS; every strobe phase is recorded (address, R/W, UDS, LDS, FC, data, AS number: the
// same number for read and write = indivisible cycle). Every program runs twice: DTACK without waits and DTACK 3
// clocks after the start of each strobe; same expected values. Clock: cen = cen2 = 1 on every edge (single-phase
// target: cen2 is ignored; two-phase target: cen2 driven equal to cen, the same scheme as the existing testbenches).
// Map: 0x0000-0x03FF vectors (all pointing to TRAP_STUB except the ones used), 0x0400-0x07FF handlers, 0x0800-0x1FFF
// frames and operands, 0x2000-0x3FFF results (the two data areas preloaded with sentinels), 0x4000- program.
// Checks after each run: end reached (third fetch of the final BRA.S *: the first is the prefetch, from the second on
// the BRA has already executed, so all the instructions before it have finished); every written byte belongs to an
// expectation, is written the expected number of times and with the expected FC; final value of every expectation;
// no other byte changed; FC of every read (program in the code regions, data elsewhere, supervisor data for the
// vector table); sequence of the vectors read.
static Vm68k_seq* dut = nullptr;
static uint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static constexpr uint32_t PROG_BASE = 0x4000;
static constexpr uint32_t HAND_BASE = 0x0400;
static constexpr uint32_t TRAP_STUB = 0x07F0;
static constexpr uint32_t DATA_BASE = 0x0800;
static constexpr uint32_t RES_BASE  = 0x2000;
static constexpr uint32_t RES_END   = 0x4000;
static constexpr uint32_t CODE_END  = 0x10000;
static constexpr int      DTACK_WAIT = 3;
static constexpr long     MAX_TICKS = 1000000;

static int  g_fail = 0;
static long g_checks = 0;
static int  g_wait = 0;
static std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "0x%X", (unsigned)v); return std::string(b); }
static void ceq(const std::string& what, uint32_t exp, uint32_t got) {
    g_checks++;
    if (exp != got) { g_fail++; printf("FAIL: [DTACK+%d] %s expected=0x%X read=0x%X\n", g_wait, what.c_str(), (unsigned)exp, (unsigned)got); }
}

static std::map<uint32_t, uint8_t> mem, init_img;
static uint8_t rdb(uint32_t a) { a &= 0xFFFFFFu; auto it = mem.find(a); return it != mem.end() ? it->second : (uint8_t)((a & 1u) ? 0x71 : 0x4E); }
static uint16_t rdw(uint32_t a) { return (uint16_t)((rdb(a) << 8) | rdb(a + 1u)); }
static void put8(uint32_t a, uint8_t v) { init_img[a & 0xFFFFFFu] = v; }
static void put16(uint32_t a, uint16_t v) { put8(a, (uint8_t)(v >> 8)); put8(a + 1u, (uint8_t)(v & 0xFFu)); }
static void put32(uint32_t a, uint32_t v) { put16(a, (uint16_t)(v >> 16)); put16(a + 2u, (uint16_t)(v & 0xFFFFu)); }

struct Item { uint32_t a; int n; uint32_t v; uint32_t m; uint8_t fc; int nw; std::string what; };
static std::vector<Item> items;      // bytes the program must write: address, byte, value, defined bits, FC, writes
static std::vector<int> exp_vec;     // expected vectors, in order
static void ex(uint32_t a, int n, uint32_t v, uint32_t m, uint8_t fc, int nw, const std::string& what) { items.push_back(Item{a, n, v, m, fc, nw, what}); }

static std::vector<uint16_t> code;
static uint32_t end_pc = 0;
static uint8_t  cur_fc = 5;          // FC of the data writes in the program's mode at that point (5 supervisor, 1 user)
static uint32_t res_next = RES_BASE, dat_next = DATA_BASE, hand_next = HAND_BASE;
static uint32_t pc_now() { return PROG_BASE + 2u * (uint32_t)code.size(); }
static void op(uint16_t x) { code.push_back(x); }
static void opl(uint32_t v) { op((uint16_t)(v >> 16)); op((uint16_t)(v & 0xFFFFu)); }
[[maybe_unused]] static void i_movel_d(int n, uint32_t v) { op((uint16_t)(0x203C | (n << 9))); opl(v); }   // MOVE.L #v,Dn
[[maybe_unused]] static void i_movea_l(int n, uint32_t v) { op((uint16_t)(0x207C | (n << 9))); opl(v); }  // MOVEA.L #v,An
[[maybe_unused]] static void i_movea_w(int n, uint32_t v) { op((uint16_t)(0x307C | (n << 9))); op((uint16_t)v); }   // MOVEA.W #v,An (v < 0x8000)
[[maybe_unused]] static void i_ccr(uint32_t v) { op(0x44FC); op((uint16_t)v); }                            // MOVE #v,CCR
[[maybe_unused]] static void i_sr(uint32_t v) { op(0x46FC); op((uint16_t)v); }                             // MOVE #v,SR
// MOVEA.L #?,An with the value to be filled in later (continuation address known only further on)
[[maybe_unused]] static size_t i_movea_l_later(int n) { op((uint16_t)(0x207C | (n << 9))); const size_t at = code.size(); opl(0); return at; }
[[maybe_unused]] static void patch_l(size_t at, uint32_t v) { code[at] = (uint16_t)(v >> 16); code[at + 1] = (uint16_t)(v & 0xFFFFu); }
static uint32_t res_alloc(uint32_t n) { uint32_t a = res_next; res_next += n; return a; }
[[maybe_unused]] static uint32_t dat_alloc(uint32_t n) { uint32_t a = dat_next + 4u; dat_next += ((n + 1u) & ~1u) + 8u; return a; }
[[maybe_unused]] static uint32_t ssp_alloc() { uint32_t s = dat_next + 12u; dat_next += 16u; return s; }   // frame at s-6..s-1
// MOVE SR,abs.W / MOVE.L Dn,abs.W / MOVE.L An,abs.W into the results area (the MOVE from SR goes first: the others
// touch the flags)
[[maybe_unused]] static void rec_sr(uint32_t v, uint32_t m, const std::string& what) { uint32_t a = res_alloc(2); op(0x40F8); op((uint16_t)a); ex(a, 2, v, m, cur_fc, 1, what + ": SR"); }
[[maybe_unused]] static void rec_d(int n, uint32_t v, const std::string& what) { uint32_t a = res_alloc(4); op((uint16_t)(0x21C0 | n)); op((uint16_t)a); ex(a, 4, v, 0xFFFFFFFFu, cur_fc, 1, what + ": D" + std::to_string(n)); }
[[maybe_unused]] static void rec_a(int n, uint32_t v, const std::string& what) { uint32_t a = res_alloc(4); op((uint16_t)(0x21C8 | n)); op((uint16_t)a); ex(a, 4, v, 0xFFFFFFFFu, cur_fc, 1, what + ": A" + std::to_string(n)); }
[[maybe_unused]] static uint32_t handler(const std::vector<uint16_t>& ws) { uint32_t a = hand_next; for (uint16_t x : ws) { put16(hand_next, x); hand_next += 2u; } return a; }
static void set_vec(int v, uint32_t target) { put32((uint32_t)v * 4u, target); }
// short frame (UM 6, exception processing): SR at SSP-6, PC high at SSP-4, PC low at SSP-2, in supervisor data (FC 5)
[[maybe_unused]] static void ex_frame(uint32_t ssp, uint32_t sr, uint32_t srm, uint32_t pc, const std::string& what) {
    ex(ssp - 6u, 2, sr, srm, 5, 1, what + ": SR in the frame");
    ex(ssp - 4u, 2, pc >> 16, 0xFFFFu, 5, 1, what + ": PC high in the frame");
    ex(ssp - 2u, 2, pc & 0xFFFFu, 0xFFFFu, 5, 1, what + ": PC low in the frame");
}
// handler recording: MOVE SR,(A5)+ ; MOVE.L A7,(A5)+ (in supervisor mode)
[[maybe_unused]] static void ex_hrec(uint32_t slot, uint32_t sr, uint32_t srm, uint32_t a7, const std::string& what) {
    ex(slot, 2, sr, srm, 5, 1, what + ": SR in the handler");
    ex(slot + 2u, 4, a7, 0xFFFFFFFFu, 5, 1, what + ": A7 (SSP) in the handler");
}
static void i_end() { end_pc = pc_now(); op(0x60FE); op(0x4E71); op(0x4E71); op(0x4E71); }   // BRA.S * (end of run)
static uint8_t sentinel(uint32_t a) { return (uint8_t)(((a * 0x9Du) >> 3) ^ (a >> 9) ^ 0xA5u); }
static void init_image() {
    for (uint32_t a = DATA_BASE; a < RES_END; a++) put8(a, sentinel(a));
    for (int v = 0; v < 256; v++) set_vec(v, TRAP_STUB);
    put16(TRAP_STUB, 0x60FE);
}
static void finish_image() {
    for (size_t i = 0; i < code.size(); i++) put16(PROG_BASE + 2u * (uint32_t)i, code[i]);
    ceq("testbench: results area within 0x4000", 1u, res_next <= RES_END ? 1u : 0u);
    ceq("testbench: data area within 0x2000", 1u, dat_next <= RES_BASE ? 1u : 0u);
    ceq("testbench: handlers below the trap", 1u, hand_next <= TRAP_STUB ? 1u : 0u);
    ceq("testbench: program within 0x10000", 1u, pc_now() + 8u <= CODE_END ? 1u : 0u);
}

struct Acc { uint32_t a; uint8_t rw, uds, lds, fc; uint16_t d; uint32_t as_id; };
static std::vector<Acc> acc;
static int s_as_prev = 0, s_ds_prev = 0, s_cnt = 0;
static uint32_t s_as_id = 0;
static bool g_abort = false;
static void (*g_hook)() = nullptr;   // IPL driving (STOP testbench only)
static void tick() { dut->clk = 0; dut->eval(); t++; dut->clk = 1; dut->eval(); t++; }
static void bus_service() {
    const int as = dut->as_n ? 0 : 1;
    const int ds = (as && (!dut->uds_n || !dut->lds_n)) ? 1 : 0;
    if (!as) s_cnt = 0;
    else if (!s_as_prev) { s_as_id++; s_cnt = 0; }
    else if (ds && !s_ds_prev) s_cnt = 0;
    else s_cnt++;
    const uint32_t a = ((uint32_t)dut->addr) << 1;
    const uint8_t fc = (uint8_t)dut->fc;
    if (as && fc == 7) { dut->vpa_n = 0; dut->dtack_n = 1; }          // IACK: autovector (VPA)
    else { dut->vpa_n = 1; dut->dtack_n = (as && s_cnt >= g_wait) ? 0 : 1; }
    if (as && dut->rw) dut->data_in = rdw(a);
    if (ds && !s_ds_prev) {
        Acc r;
        r.a = a; r.rw = dut->rw ? 1 : 0; r.uds = dut->uds_n ? 0 : 1; r.lds = dut->lds_n ? 0 : 1; r.fc = fc; r.as_id = s_as_id;
        if (r.rw) r.d = rdw(a);
        else {
            r.d = (uint16_t)dut->data_out;
            if (r.uds) mem[a] = (uint8_t)(r.d >> 8);
            if (r.lds) mem[a + 1u] = (uint8_t)(r.d & 0xFFu);
        }
        acc.push_back(r);
    }
    s_as_prev = as; s_ds_prev = ds;
}
static bool in_code(uint32_t a) { return (a >= HAND_BASE && a < DATA_BASE) || (a >= PROG_BASE && a < CODE_END); }
// 0 = end reached, 1 = tick limit, 2 = trap fetch (unexpected vector), 3 = stopped by the testbench
static int run_prog() {
    mem = init_img; acc.clear(); s_as_prev = 0; s_ds_prev = 0; s_cnt = 0; s_as_id = 0; g_abort = false;
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_seq;
    dut->clk = 0; dut->rst_n = 0; dut->cen = 0; dut->cen2 = 0; dut->start = 0; dut->pc_init = 0; dut->rst_vector = 0;
    dut->ipl = 0; dut->vpa_n = 1; dut->data_in = 0; dut->dtack_n = 1; dut->halt_n = 1; dut->berr_n = 1; dut->br_n = 1;
    dut->bgack_n = 1; dut->dbg_rd_dnum = 0; dut->dbg_rd_anum = 0;
    dut->eval(); tick(); tick();
    dut->rst_n = 1; dut->pc_init = PROG_BASE; dut->start = 1; dut->cen = 1; dut->cen2 = 1; tick();
    dut->start = 0;
    int endf = 0, why = 1;
    for (long n = 0; n < MAX_TICKS; n++) {
        const size_t before = acc.size();
        bus_service();
        if (acc.size() != before) {
            const Acc& r = acc.back();
            if (r.rw && (r.fc == 2 || r.fc == 6)) {
                if (r.a == end_pc && ++endf >= 3) { why = 0; break; }
                if (r.a == TRAP_STUB) { why = 2; break; }
            }
        }
        if (g_hook) g_hook();
        if (g_abort) { why = 3; break; }
        tick();
    }
    dut->final(); delete dut; dut = nullptr;
    return why;
}
static void check_run(int why) {
    ceq("end of run at the BRA.S * at " + hx(end_pc) + " (0 = reached, 1 = tick limit, 2 = unexpected vector, 3 = stopped by the testbench)", 0u, (uint32_t)why);
    std::map<uint32_t, int> owner, nwr;
    for (size_t k = 0; k < items.size(); k++) for (int b = 0; b < items[k].n; b++) owner[items[k].a + (uint32_t)b] = (int)k;
    int outside = 0;
    for (size_t i = 0; i < acc.size(); i++) {
        const Acc& r = acc[i];
        if (r.rw) continue;
        for (int lane = 0; lane < 2; lane++) {
            if (lane == 0 ? !r.uds : !r.lds) continue;
            const uint32_t b = r.a + (uint32_t)lane;
            auto it = owner.find(b);
            if (it == owner.end()) { if (outside++ < 8) ceq("write outside the expected areas at byte " + hx(b) + " (FC " + std::to_string(r.fc) + ")", 0u, 1u); continue; }
            nwr[b]++;
            const Item& e = items[(size_t)it->second];
            if (r.fc != e.fc) ceq(e.what + ": FC of the write at byte " + hx(b), e.fc, r.fc);
        }
    }
    ceq("writes outside the expected areas", 0u, (uint32_t)outside);
    for (size_t k = 0; k < items.size(); k++) {
        const Item& e = items[k];
        uint32_t got = 0; int badn = 0;
        for (int b = 0; b < e.n; b++) { got = (got << 8) | rdb(e.a + (uint32_t)b); if (nwr[e.a + (uint32_t)b] != e.nw) badn++; }
        ceq(e.what + " (" + hx(e.a) + ")", e.v & e.m, got & e.m);
        ceq(e.what + ": bytes written a number of times other than " + std::to_string(e.nw), 0u, (uint32_t)badn);
    }
    int changed = 0;
    for (auto& p : init_img) if (!owner.count(p.first) && rdb(p.first) != p.second) { if (changed++ < 8) ceq("byte not meant to change " + hx(p.first), p.second, rdb(p.first)); }
    ceq("bytes changed outside the expected values", 0u, (uint32_t)changed);
    int badfc = 0; std::vector<uint32_t> vecr;
    for (size_t i = 4; i < acc.size(); i++) {          // the first 4 accesses are the reset vector reads (emulated by the shim)
        const Acc& r = acc[i];
        if (r.fc == 7 || !r.rw) continue;
        const bool code_r = in_code(r.a);
        const bool okfc = code_r ? (r.fc == 2 || r.fc == 6) : (r.a < HAND_BASE ? r.fc == 5 : (r.fc == 1 || r.fc == 5));
        if (!okfc && badfc++ < 8) ceq("FC of the read at " + hx(r.a), code_r ? 6u : 5u, r.fc);
        if (!code_r && r.a < HAND_BASE) vecr.push_back(r.a);
    }
    ceq("reads with wrong FC", 0u, (uint32_t)badfc);
    ceq("vector table reads", (uint32_t)(2 * exp_vec.size()), (uint32_t)vecr.size());
    for (size_t k = 0; k < exp_vec.size() && 2 * k + 1 < vecr.size(); k++) {
        const uint32_t lo = std::min(vecr[2 * k], vecr[2 * k + 1]), hi = std::max(vecr[2 * k], vecr[2 * k + 1]);
        ceq("exception " + std::to_string(k + 1) + ": high word of the vector read", (uint32_t)exp_vec[k] * 4u, lo);
        ceq("exception " + std::to_string(k + 1) + ": low word of the vector read", (uint32_t)exp_vec[k] * 4u + 2u, hi);
    }
}
// =============================================================================================== end of common testbench

// stack with 'below' bytes under s and 'above' bytes above it, plus 4 bytes of sentinels on each side; returns s
static uint32_t stk_alloc(uint32_t below, uint32_t above) { const uint32_t s = dat_next + below + 4u; dat_next += below + above + 8u; return s; }
static void self_test() {}
// LINK An,#d with SP = s, An = anv, then UNLK An; hi = upper SR of the current mode (0x27 supervisor, 0x00 user)
static void link_case(int an, uint32_t anv, uint32_t s, uint16_t d, uint32_t ccr1, uint32_t ccr2, uint32_t hi, const std::string& lab) {
    const uint32_t de = (uint32_t)(int32_t)(int16_t)d;                       // sign-extended displacement
    i_movea_l(an, anv);
    i_ccr(ccr1);
    op((uint16_t)(0x4E50 | an)); op(d);                                      // LINK An,#d
    rec_sr(hi | ccr1, 0xFFFF, "LINK " + lab + ": flags unchanged");
    rec_a(an, s - 4u, "LINK " + lab + ": An = SP - 4");
    rec_a(7, s - 4u + de, "LINK " + lab + ": SP = SP - 4 + d");
    ex(s - 4u, 4, anv, 0xFFFFFFFFu, cur_fc, 1, "LINK " + lab + ": old An stacked at SP - 4");
    i_ccr(ccr2);
    op((uint16_t)(0x4E58 | an));                                             // UNLK An
    rec_sr(hi | ccr2, 0xFFFF, "UNLK " + lab + ": flags unchanged");
    rec_a(an, anv, "UNLK " + lab + ": An restored from the stack");
    rec_a(7, s, "UNLK " + lab + ": SP = An + 4");
}
static void build() {
    uint32_t s;
    s = stk_alloc(4, 0); i_movea_w(7, s); link_case(6, 0x12345678u, s, 0xFFF8, 0x15, 0x0A, 0x2700, "A6,#-8");
    // LINK A0,#0 ; MOVE.L (A0),D1 ; MOVE.L #$0BADF00D,(A0) ; UNLK A0: An from the changed memory
    s = stk_alloc(4, 0); i_movea_w(7, s);
    i_movea_l(0, 0xFEDCBA98u); i_ccr(0x1F);
    op(0x4E50); op(0x0000);
    rec_sr(0x271F, 0xFFFF, "LINK A0,#0: flags unchanged");
    rec_a(0, s - 4u, "LINK A0,#0: A0 = SP - 4");
    rec_a(7, s - 4u, "LINK A0,#0: SP = SP - 4");
    op(0x2210); rec_d(1, 0xFEDCBA98u, "LINK A0,#0: MOVE.L (A0),D1 reads the old stacked A0");
    op(0x20BC); opl(0x0BADF00Du);                                            // MOVE.L #$0BADF00D,(A0)
    ex(s - 4u, 4, 0x0BADF00Du, 0xFFFFFFFFu, 5, 2, "LINK A0,#0 (old A0) then MOVE.L #$0BADF00D,(A0)");
    i_ccr(0x00); op(0x4E58);
    rec_sr(0x2700, 0xFFFF, "UNLK A0: flags unchanged");
    rec_a(0, 0x0BADF00Du, "UNLK A0: A0 reread from memory");
    rec_a(7, s, "UNLK A0: SP = A0 + 4");
    s = stk_alloc(4, 0); i_movea_w(7, s); link_case(5, 0x00000000u, s, 0x000C, 0x04, 0x11, 0x2700, "A5,#+12 (A5 = 0)");
    s = stk_alloc(4, 0); i_movea_w(7, s); link_case(1, 0x89ABCDEFu, s, 0x8000, 0x08, 0x02, 0x2700, "A1,#-32768");
    s = stk_alloc(4, 0); i_movea_w(7, s); link_case(2, 0x13572468u, s, 0x7FFF, 0x13, 0x0C, 0x2700, "A2,#+32767");
    // user mode: A7 = USP
    const uint32_t u = stk_alloc(4, 0);
    i_movea_l(0, u); op(0x4E60);                                             // MOVE A0,USP
    i_sr(0x0000);                                                            // S=0
    cur_fc = 1;
    link_case(4, 0x13579BDFu, u, 0xFFFC, 0x1A, 0x05, 0x0000, "A4,#-4 in user mode (USP)");
    i_end();
}
static void bench_prerun() {}
static void bench_post() {}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    self_test();
    init_image();
    build();
    finish_image();
    for (int k = 0; k < 2; k++) {
        g_wait = k ? DTACK_WAIT : 0;
        bench_prerun();
        const int why = run_prog();
        check_run(why);
        bench_post();
    }
    printf("checks run: %ld, failed: %d\n", g_checks, g_fail);
    if (g_fail == 0) { printf("RESULT: PASS\n"); return 0; }
    printf("RESULT: FAIL\n");
    return 1;
}
