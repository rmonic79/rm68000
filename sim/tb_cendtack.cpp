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

// tb_cendtack.cpp — bench of the clock-enable and DTACK generator m68k_cen_dtack on its own (docs/clock-dtack.md,
// section 6.1). The generator is a black box: the bench reads only its pins (cpu_cen, cpu_cenb, DTACKn).
//
// Stimulus. A phase-accurate model of the processor's bus cycle (VC, section 3.2) runs on the generator's own pulses:
// S0 on a cpu_cen after the programmed idle pulses, AS (and the strobes of a read or TAS) low at S2, the strobes of a
// write at S4, DTACK sampled at S5 and at every wait state, the TAS write half with its strobes falling on a cpu_cen and
// its sample on the next cpu_cenb, address-error cycles without strobes. A memory model (stall_len, VC::stalls) raises
// bus_cs and bus_busy at every handshake. Access sequence: build_synth.
//
// Expected values. All of them are computed by the bench at run time from docs/clock-dtack.md; none comes from a run of
// the generator. Each check has a name, printed in its CHECK line, and every one counts in the verdict:
//   RESET       (4.8)        while rst is sampled high: cpu_cen = cpu_cenb = 0, DTACKn = 1.
//   FIRST       (4.8)        the first pulse after the release is cpu_cen, at the k-th edge with rst low, k = den/(2*num) + 1.
//   PHASES      (4.3)        at most one pulse per clock; each pulse is cpu_cen when the phase is 0 and cpu_cenb when it
//                            is 1, and toggles the phase.
//   REGISTERED  (4.1, 4.10)  cpu_cen, cpu_cenb and DTACKn do not change when the inputs of the clock are applied: they are
//                            registers, with no combinational path from the inputs.
//   R0          (4.2)        a pulse at every edge where the bench's divider (acc += 2*num, nominal pulse when
//                            acc + 2*num > den, then acc -= den) has a nominal pulse due.
//   SPACING     (4.2)        idle mode, num = 1: clocks between consecutive pulses as in the table of 4.2 (6, 4, 3, 2,
//                            2-1-2-1..., 1 for den 12, 8, 6, 4, 3, 2).
//   R3          (4.5-4.7)    a pulse that is not nominal is emitted at an edge if and only if the debt is non-zero, AS
//                            was high in the clock before and no nominal pulse is due. The debt is the bench's own:
//                            +1 for each pulse lost by the rule of 4.5 (pulse in clock t, bus_cs & bus_busy in clock t,
//                            cycle active in clock t-1, DTACKn high in clock t), -1 for each recovery pulse, modulo 2^14
//                            (W+WD = 8+6 bits, 4.7).
//   R1          (4.4)        DTACKn at every clock: high if the cycle was not active in the clock before; otherwise low if it
//                            was low, or if the cycle was active in the two clocks before and bus_cs & bus_busy was low in
//                            the clock before.
//   WAITS       (4.4)        at every sample of the processor model (S5, S11, S13), in clock j: DTACK found if and only if
//                            j >= a+2+Lb (a = clock from which the strobes are low, Lb = consecutive clocks with
//                            bus_cs & bus_busy from clock a+1). Idle mode with den a multiple of 2*num: the wait states of
//                            each access equal the formulas of 4.4 (read max(0, ceil((3-3s)/2s)), write and TAS write half
//                            max(0, ceil((3-s)/2s)), Lb = 0).
//   COMPLETE    (3.2)        the processor model completes the whole sequence, with a second acknowledge in every TAS.
//   RETURNED    (1, 4.5-4.7) at the end of the run, after a tail with the bus free: with den > 2*num every lost pulse has
//                            been returned (debt 0; recovery pulses = lost pulses - 2^14 per wrap of the counter), with
//                            den = 2*num no recovery pulse at all (4.6); the distance from the nominal clock (pulses -
//                            2 x wait states - nominal pulses) equals lost pulses - 2 x wait states - outstanding debt
//                            - 2^14 per wrap, the value 4.5 implies; in MODE=sat the debt wrapped at least once (4.7).
// The two conditions of 4.5 where the memory looks busy while the processor is not waiting (BUSYBACK=1, MAPLAT=0) are
// covered by R1, R3, WAITS and RETURNED in their own runs; the comparison of a BUSYBACK=1 run with the same run without it
// (identical trace, sections 4.5 and 4.9) is done by run_cendtack.sh, from the SUMMARY line.
//
// Order within a clock (main loop): rst for the edge -> edge -> outputs read (REGISTERED) -> the processor model acts on
// the outputs of the previous clock (VC::step) -> the memory model sets bus_cs/bus_busy (VC::stalls) -> inputs applied,
// evaluation, outputs read -> checks of the clock against the rules applied to the previous clock's inputs and outputs.
//
// Environment variables:
//   DEN=12|8|6|4|3|2   NUM=1   MODE=idle|map|short|long|burst|sat   LMAX=64 (clocks)   MAPLAT=1|0 (clocks from the strobes
//     to bus_cs/bus_busy)   RAMBUSY=1   BUSYBACK=0|1   NACC=20000   KINDS=all|rd (rd: reads and address errors only)
//   SEED=1   TRACE=1 (one line per clock for the first TL clocks)   TL=400
//   idle = no bus_cs and no busy; map = busy of the map only (2 clocks on the ROM region); short / long / burst / sat =
//   longer ROM stalls (stall_len). BUSYBACK=1: after the stall (DTACK given) bus_busy rises again until the end of the
//   cycle. MAPLAT=0: bus_cs and bus_busy rise in the same clock as the strobes.
// Output: CHECK lines, "FAILED CHECKS: ...", "SUMMARY <RESULT> <distance> <trace hash> <pulses with busy after DTACK>
// <pulses with busy in the strobe clock> <failed checks>", and last "RESULT: PASS|FAIL".
#include "Vtop_cendtack.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstdarg>
#include <vector>
#include <string>
#include <algorithm>

static Vtop_cendtack* top;
double sc_time_stamp() { return 0.0; }

static int envi(const char* n, int d) { const char* e = getenv(n); return e ? atoi(e) : d; }
static std::string envs(const char* n, const char* d) { const char* e = getenv(n); return e ? std::string(e) : std::string(d); }

// ----------------------------------------------------------------------------------------------- checks
enum Chk { C_RESET, C_FIRST, C_PHASES, C_REGISTERED, C_R0, C_SPACING, C_R3, C_R1, C_WAITS, C_COMPLETE, C_RETURNED, C_N };
static const char* CHK_NAME[C_N] = { "RESET", "FIRST", "PHASES", "REGISTERED", "R0", "SPACING", "R3", "R1", "WAITS", "COMPLETE", "RETURNED" };
static const char* CHK_SEC[C_N]  = { "4.8", "4.8", "4.3", "4.1, 4.10", "4.2", "4.2", "4.5-4.7", "4.4", "4.4", "3.2", "1, 4.5-4.7" };
static long chk_n[C_N];                 // failures per check
static long chk_t[C_N];                 // clock of the first failure
static std::string chk_msg[C_N];        // description of the first failure
static bool chk_na[C_N];                // check not applicable to this run (printed as such)
static bool g_window = false;           // a failure in this clock: print the window of the last clocks (first failure only)
static void fail(int c, long t, const char* fmt, ...) {
    if (chk_n[c]++ == 0) {
        char b[320]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
        chk_t[c] = t; chk_msg[c] = b;
    }
    g_window = true;
}

// ----------------------------------------------------------------------------------------------- access sequence
enum Kind { RD = 0, WR = 1, TAS = 2, AERR = 3 };
static const char* kind_name(int k) { return k == RD ? "read" : k == WR ? "write" : k == TAS ? "TAS" : "address error"; }
struct Acc { uint32_t addr; int kind; int idle_pulses; };   // idle_pulses: pulses between the end of the previous cycle and S0
static std::vector<Acc> seq;
enum Region { R_ROM, R_RAM, R_OTHER };
static Region region(uint32_t a) {
    if (a < 0x060000) return R_ROM;
    if (a >= 0x080000 && a < 0x090000) return R_RAM;
    return R_OTHER;
}
static int KINDS_RD = 0;
// 10/16 reads, 4/16 writes, 1/16 TAS, 1/16 address errors; 70 % ROM region, 20 % RAM region, 10 % elsewhere; mostly back to
// back, one access in eight preceded by 0 to 10 idle pulses
static void build_synth(int nacc, uint32_t seed) {
    uint32_t s = seed * 2654435761u + 12345u;
    for (int i = 0; i < nacc; i++) {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        int k = (s % 16 < 10) ? RD : (s % 16 < 14) ? WR : (s % 16 == 14) ? TAS : AERR;
        if (KINDS_RD && (k == WR || k == TAS)) k = RD;
        int r = (s >> 8) % 10;
        uint32_t a = (r < 7) ? ((s >> 4) % 0x30000) * 2 : (r < 9) ? 0x080000 + ((s >> 4) % 0x8000) * 2 : 0x0B0000 + ((s >> 4) % 0x1000) * 2;
        if (k == AERR) a |= 1;
        int idle = ((s >> 20) % 8 == 0) ? 2 * ((s >> 24) % 6) : 0;
        seq.push_back({a, k, idle});
    }
}

// ----------------------------------------------------------------------------------------------- memory model
static std::string MODE; static int LMAX = 64, MAPLAT = 1, RAMBUSY = 1, BUSYBACK = 0;
static uint32_t hsh(uint32_t i, uint32_t a) { uint32_t h = i * 0x9E3779B1u ^ (a * 0x85EBCA6Bu); h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; return h; }
// clocks of bus_busy for access i, from the clock MAPLAT after the strobes fall: the map raises bus_cs and bus_busy for one
// clock on every access; the ROM region then stays busy by mode, the RAM region one more clock (RAMBUSY), the others not
static int stall_len(int i, const Acc& ac) {
    if (MODE == "idle") return 0;
    Region r = region(ac.addr);
    if (r == R_RAM) return 1 + (RAMBUSY ? 1 : 0);
    if (r == R_OTHER) return 1;
    if (MODE == "map") return 2;
    uint32_t h = hsh(i, ac.addr);
    if (MODE == "short") return 2 + (int)(h % 5);                                          // 2..6
    if (MODE == "long")  return 2 + (int)(h % (uint32_t)(LMAX + 1));                      // 2..LMAX+2
    if (MODE == "burst") return 2 + ((i % 8 < 5) ? (int)(h % (uint32_t)(LMAX + 1)) : 0);  // five stalled accesses out of eight
    if (MODE == "sat")   return 2 + ((i % 4 == 0) ? 2000 + (int)(h % 500) : 0);           // the debt passes 2^14 - 1 and wraps
    return 2;
}

// ----------------------------------------------------------------------------------------------- processor model
struct VC {
    int asn = 1, dsn = 3, cs = 0, busy = 0;               // inputs of the generator for the current clock
    enum St { IDLE, S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12, S13, DONE } st = IDLE;
    int iacc = 0, idle_left = 0, rmw_alu = 0, tas_half = 0;
    long act_vis = -1;                                    // a of section 4.4: first clock with the strobes of the handshake low
    int cur_len = 0;                                      // busy clocks of the current handshake (stall_len)
    std::vector<int> acc_waits; int cur_waits = 0;        // wait states per access (both halves of a TAS)
    long busy_from = -1, busy_to = -1; int cs_on = 0, dsn_prev = 3;
    long waits = 0, cycles = 0, samples = 0, tas_cycles = 0, tas_second_dtack = 0, done_clk = -1;
    long last_progress_clk = 0, last_cycles = 0; int stuck = 0;
    const Acc& cur() const { return seq[iacc]; }
    bool finished() const { return st == DONE; }
    // Lb of section 4.4: consecutive clocks with bus_cs & bus_busy starting with clock a+1. The memory model holds them for
    // cur_len clocks from clock a+MAPLAT: with MAPLAT = 1 Lb = cur_len, with MAPLAT = 0 the first one is clock a itself
    long lb() const { return MAPLAT == 0 ? std::max(0, cur_len - 1) : cur_len; }
    // a sample decided at edge clk reads DTACKn of clock j = clk-1, the clock of its cpu_cenb; section 4.4: found iff j >= a+2+Lb
    bool sample(int dtackn, long clk) {
        samples++;
        bool want = (clk - 1) >= act_vis + 2 + lb();
        bool got = !dtackn;
        if (want != got)
            fail(C_WAITS, clk - 1, "access #%d (%s), sample in clock %ld: DTACK %s, section 4.4 has it %s (a = %ld, Lb = %ld)",
                 iacc, kind_name(cur().kind), clk - 1, got ? "found" : "not found", want ? "found" : "not found", act_vis, lb());
        return got;
    }
    // one edge, with the pulses and DTACKn of the previous clock: even states advance on cpu_cen, odd ones on cpu_cenb
    void step(int cen, int cenb, int dtackn, long clk) {
        if (st == DONE) return;
        if (st == IDLE) {
            if (cen || cenb) { if (idle_left > 0) idle_left--; }
            if (cen && idle_left == 0) st = S1;          // S0: a request starts the cycle on cpu_cen
            return;
        }
        bool even = (st == S0 || st == S2 || st == S4 || st == S6 || st == S8 || st == S10 || st == S12);
        if (even && !cen) return;
        if (!even && !cenb) return;
        const Acc& a = cur();
        switch (st) {
        case S1: st = S2; break;
        case S2: asn = 0; tas_half = 0; if (a.kind == RD || a.kind == TAS) { dsn = 0; act_vis = clk; } cycles++; st = S3; break;
        case S3: st = S4; break;
        case S4: if (a.kind == WR && dsn == 3) { dsn = 0; act_vis = clk; } st = S5; break;
        case S5: if (a.kind == AERR) { st = S6; break; }                       // ends by itself, no data strobes (3.2)
                 if (sample(dtackn, clk)) st = S6; else { waits++; cur_waits++; st = S4; } break;
        case S6: st = S7; break;
        case S7: if (a.kind == TAS && tas_half == 0) { dsn = 3; tas_half = 1; tas_cycles++; rmw_alu = 2; st = S8; }   // AS stays low
                 else { asn = 1; dsn = 3; next_access(); } break;
        case S8: if (rmw_alu == 0) { dsn = 0; act_vis = clk; st = S11; } else { rmw_alu--; st = S9; } break;      // TAS write half
        case S9: st = S8; break;
        case S10: st = S11; break;
        case S11: if (sample(dtackn, clk)) { tas_second_dtack++; st = S6; } else { waits++; cur_waits++; st = S12; } break;
        case S12: st = S13; break;
        case S13: if (sample(dtackn, clk)) { tas_second_dtack++; st = S6; } else { waits++; cur_waits++; st = S12; } break;
        default: break;
        }
    }
    void next_access() {
        acc_waits.push_back(cur_waits); cur_waits = 0;
        iacc++;
        if (iacc >= (int)seq.size()) { st = DONE; return; }
        idle_left = cur().idle_pulses; st = IDLE;
    }
    // memory model: at every falling edge of the data strobes (a new handshake, including the TAS write half) the map arms
    // bus_cs and bus_busy from clock a+MAPLAT; bus_busy lasts cur_len clocks; both low while the cycle is not active
    void stalls(long clk) {
        if (st == DONE) { cs = 0; busy = 0; return; }
        int active = (asn == 0) && (dsn != 3);
        if (active && dsn_prev == 3) {
            cur_len = stall_len(iacc, cur()); cs_on = (MODE == "idle") ? 0 : 1;
            busy_from = act_vis + MAPLAT; busy_to = busy_from + cur_len;
        }
        dsn_prev = dsn;
        if (active && act_vis >= 0) {
            cs   = (cs_on && clk >= act_vis + MAPLAT) ? 1 : 0;
            busy = (busy_from >= 0 && clk >= busy_from && clk < busy_to) ? 1 : 0;
            if (BUSYBACK && cs_on && busy_to >= 0 && clk > busy_to) busy = 1;   // one clock low (DTACK given), then high again
        } else { cs = 0; busy = 0; }
    }
};

// ----------------------------------------------------------------------------------------------- diagnostic window
struct Rec { long t; uint8_t cen, cenb, dt, xcen, xcenb, xdt, asn, dsn, cs, busy, nd, st; int acc; uint32_t debt; };
static Rec ring[32];
static void print_window(long clk) {
    printf("WINDOW (the 16 clocks up to the first failure; x = expected by the rules, debt = the bench's, nd = nominal pulse due)\n");
    for (long t = clk - 15; t <= clk; t++) {
        if (t < 0) continue;
        const Rec& r = ring[t & 31];
        printf("  t=%ld cen%d cenb%d dt%d | x cen%d cenb%d dt%d | as%d ds%d cs%d busy%d nd%d | state %d access #%d debt %u%s\n",
               r.t, r.cen, r.cenb, r.dt, r.xcen, r.xcenb, r.xdt, r.asn, r.dsn, r.cs, r.busy, r.nd, r.st, r.acc, r.debt, t == clk ? "   <==" : "");
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const int DEN = envi("DEN", 12), NUM = envi("NUM", 1), NACC = envi("NACC", 20000), TRACE = envi("TRACE", 0), TL = envi("TL", 400), SEED = envi("SEED", 1);
    MODE = envs("MODE", "idle"); KINDS_RD = (envs("KINDS", "all") == "rd"); LMAX = envi("LMAX", 64); MAPLAT = envi("MAPLAT", 1);
    RAMBUSY = envi("RAMBUSY", 1); BUSYBACK = envi("BUSYBACK", 0);
    if (NUM < 1 || NUM > 127 || DEN > 255 || DEN < 2 * NUM || (MAPLAT != 0 && MAPLAT != 1) || NACC < 1 || LMAX < 0) {
        printf("ERROR: unsupported setting (1 <= NUM <= 127, 2*NUM <= DEN <= 255, MAPLAT 0 or 1, NACC >= 1)\nRESULT: FAIL\n"); return 2;
    }
    build_synth(NACC, (uint32_t)SEED);
    top = new Vtop_cendtack;
    top->clk = 0; top->rst = 1; top->num = (uint8_t)NUM; top->den = (uint8_t)DEN;
    top->as_n = 1; top->ds_n = 3; top->bus_cs = 0; top->bus_busy = 0;
    top->eval();                                          // settle with clk low: the first iteration is a rising edge
    VC v;

    const long RST_CLKS = 20, MAXCLK = 400000000L, TAIL_MAX = 1L << 21;
    const uint32_t DEBT_MOD = 1u << 14;                   // W + WD = 8 + 6 bits (4.1, 4.7)
    const int K_FIRST = DEN / (2 * NUM) + 1;              // 4.8: first k with 2*k*num > den
    const bool idle = (MODE == "idle");
    const bool spacing_on = idle && NUM == 1 && (DEN == 12 || DEN == 8 || DEN == 6 || DEN == 4 || DEN == 3 || DEN == 2);   // the table of 4.2
    // bench state: the registers the rules of section 4 name, kept from the generator's pins and the bench's inputs
    long m_acc = 0; int m_phase = 0, m_aq = 0; uint32_t m_debt = 0;
    int p_cen = 0, p_cenb = 0, p_dt = 1;                  // outputs in the previous clock
    int p_asn = 1, p_active = 0, p_hold = 0;              // inputs in the previous clock
    long pulses = 0, n_cen = 0, n_cenb = 0, nominal = 0, recov = 0, lost = 0, wraps = 0, first_pulse = -1, last_pulse = -1, gap_i = 0;
    int first_is_cen = 0; uint32_t debt_max = 0;
    long n_after = 0, n_strobe = 0;                       // pulses with busy while the processor is not waiting (4.5)
    uint64_t hash = 1469598103934665603ULL;               // FNV-1a over (cpu_cen | cpu_cenb << 1 | DTACKn << 2), every clock
    bool window_done = false;
    long clk;
    for (clk = 0; clk < MAXCLK; clk++) {
        const bool in_reset = clk < RST_CLKS;
        top->rst = in_reset ? 1 : 0;
        top->clk = 1; top->eval(); top->clk = 0; top->eval();
        const int e_cen = top->cpu_cen, e_cenb = top->cpu_cenb, e_dt = top->dtack_n;   // after the edge, previous inputs
        if (!in_reset) v.step(p_cen, p_cenb, p_dt, clk);
        v.stalls(clk);
        top->as_n = v.asn; top->ds_n = v.dsn; top->bus_cs = v.cs; top->bus_busy = v.busy;
        top->eval();
        const int cen = top->cpu_cen, cenb = top->cpu_cenb, dt = top->dtack_n;
        const int active = (v.asn == 0) && (v.dsn != 3), hold = v.cs && v.busy;
        const int pulse = cen || cenb;
        hash = (hash ^ (uint64_t)(cen | (cenb << 1) | (dt << 2))) * 1099511628211ULL;
        if (cen != e_cen || cenb != e_cenb || dt != e_dt)
            fail(C_REGISTERED, clk, "clock %ld: outputs after the edge cen%d cenb%d dt%d, after applying the inputs cen%d cenb%d dt%d", clk, e_cen, e_cenb, e_dt, cen, cenb, dt);
        int nd = 0, xcen = 0, xcenb = 0, xdt = 1;
        if (in_reset) {
            // 4.8: rst sampled high clears acc, phase, debt, active_q; cpu_cen = cpu_cenb = 0, DTACKn = 1
            if (cen || cenb || !dt) fail(C_RESET, clk, "clock %ld with rst: cen%d cenb%d dt%d", clk, cen, cenb, dt);
            m_acc = 0; m_phase = 0; m_debt = 0; m_aq = 0;
        } else {
            // R0 (4.2) at this edge, from acc in the previous clock
            const long sum = m_acc + 2 * NUM;
            nd = (DEN != 0) && (sum > DEN);
            const long after = nd ? sum - DEN : sum;
            m_acc = std::min(after, (long)DEN);
            // R2 (4.5): the pulse of the previous clock is lost if, in that clock, bus_cs & bus_busy, the cycle was active one
            // clock earlier (active_q) and DTACKn was high
            const int miss = (p_cen || p_cenb) && p_hold && m_aq && p_dt;
            // R3 (4.6): recovery pulse when the debt is non-zero, AS was high in the previous clock and no nominal pulse is due
            const int supp_ok = (m_debt != 0) && !nd && p_asn;
            // R1 (4.4): next DTACKn from the previous clock's active, active_q, hold and DTACKn
            xdt = !p_active ? 1 : (p_dt ? !(m_aq && p_active && !p_hold) : 0);
            const int xpulse = nd || supp_ok;
            xcen = xpulse && !m_phase; xcenb = xpulse && m_phase;
            if (cen && cenb) fail(C_PHASES, clk, "clock %ld: cpu_cen and cpu_cenb together", clk);
            else if (pulse && cen != !m_phase) fail(C_PHASES, clk, "clock %ld: %s with phase %d", clk, cen ? "cpu_cen" : "cpu_cenb", m_phase);
            if (nd && !pulse) fail(C_R0, clk, "clock %ld: nominal pulse due, no pulse", clk);
            if (pulse && !nd && !supp_ok) fail(C_R3, clk, "clock %ld: pulse that is not nominal with debt %u and AS %s in the clock before", clk, m_debt, p_asn ? "high" : "low");
            if (supp_ok && !pulse) fail(C_R3, clk, "clock %ld: no recovery pulse with debt %u, AS high in the clock before, no nominal pulse due", clk, m_debt);
            if (dt != xdt) fail(C_R1, clk, "clock %ld: DTACKn %d, the rule of 4.4 gives %d", clk, dt, xdt);
            // update of the bench's registers with what the generator did
            const int supp = pulse && !nd;
            if (pulse) {
                m_phase ^= 1; pulses++; if (cen) n_cen++; else n_cenb++;
                if (first_pulse < 0) { first_pulse = clk; first_is_cen = cen; }
                if (spacing_on && last_pulse >= 0) {                                        // SPACING (4.2 table)
                    gap_i++;
                    const long want = (DEN == 3) ? ((gap_i & 1) ? 2 : 1) : DEN / 2;
                    if (clk - last_pulse != want) fail(C_SPACING, clk, "gap %ld between pulses: %ld clocks, the table of 4.2 gives %ld", gap_i, clk - last_pulse, want);
                }
                last_pulse = clk;
            }
            if (nd) nominal++;
            recov += supp; lost += miss;
            if (m_debt + (uint32_t)miss >= DEBT_MOD && !supp) wraps++;
            m_debt = (m_debt + (uint32_t)miss - (uint32_t)supp) & (DEBT_MOD - 1);
            if (m_debt > debt_max) debt_max = m_debt;
            m_aq = p_active;
            if (pulse && hold && active && !p_active) n_strobe++;   // busy in the very clock where the strobes fall (4.5)
            if (pulse && hold && active && !dt) n_after++;          // busy after DTACK has been given (4.5)
        }
        Rec& r = ring[clk & 31];
        r.t = clk; r.cen = cen; r.cenb = cenb; r.dt = dt; r.xcen = xcen; r.xcenb = xcenb; r.xdt = xdt; r.asn = v.asn; r.dsn = v.dsn;
        r.cs = v.cs; r.busy = v.busy; r.nd = nd; r.st = (uint8_t)v.st; r.acc = v.iacc; r.debt = m_debt;
        if (g_window && !window_done) { print_window(clk); window_done = true; }
        g_window = false;
        if (TRACE && clk < TL)
            printf("clk %6ld cen%d cenb%d dt%d as%d ds%d cs%d busy%d nd%d state %d access #%d debt %u\n", clk, cen, cenb, dt, v.asn, v.dsn, v.cs, v.busy, nd, (int)v.st, v.iacc, m_debt);
        p_cen = cen; p_cenb = cenb; p_dt = dt; p_asn = v.asn; p_active = active; p_hold = hold;
        // processor model stuck (no cycle completed in 200,000 clocks): the run is closed, COMPLETE fails
        if (!v.finished()) {
            if (v.cycles != v.last_cycles) { v.last_cycles = v.cycles; v.last_progress_clk = clk; }
            else if (clk - v.last_progress_clk > 200000) { v.stuck = 1; v.st = VC::DONE; }
        }
        // tail with the bus free: until the debt is returned (den > 2*num), at least 4*den + 64 clocks, at most TAIL_MAX
        if (v.finished()) {
            if (v.done_clk < 0) v.done_clk = clk;
            const bool drained = (DEN <= 2 * NUM) || m_debt == 0;
            if (drained && clk >= v.done_clk + 4 * DEN + 64) break;
            if (clk >= v.done_clk + TAIL_MAX) break;
        }
    }
    const long total = clk;
    // FIRST (4.8): the k-th edge with rst low is the one of clock RST_CLKS - 1 + k
    if (first_pulse != RST_CLKS - 1 + K_FIRST || !first_is_cen)
        fail(C_FIRST, first_pulse, "first pulse %s in clock %ld, section 4.8 gives cpu_cen in clock %ld (k = %d)", first_pulse < 0 ? "none" : first_is_cen ? "cpu_cen" : "cpu_cenb", first_pulse, RST_CLKS - 1 + K_FIRST, K_FIRST);
    // WAITS, idle mode: the formulas of 4.4 per access (Lb = 0, no debt, uniform spacing s = den / (2*num))
    const bool formula = idle && (DEN % (2 * NUM) == 0);
    long formula_n = 0;
    if (formula) {
        const int s = DEN / (2 * NUM);
        auto cdiv = [](int n, int d) { return n <= 0 ? 0 : (n + d - 1) / d; };
        const int w_rd = cdiv(3 - 3 * s, 2 * s), w_wr = cdiv(3 - s, 2 * s);
        for (long i = 0; i < (long)v.acc_waits.size(); i++) {
            const int k = seq[i].kind;
            const int want = (k == RD) ? w_rd : (k == WR) ? w_wr : (k == TAS) ? w_rd + w_wr : 0;
            formula_n++;
            if (v.acc_waits[i] != want) fail(C_WAITS, -1, "access #%ld (%s): %d wait states, the formulas of 4.4 give %d", i, kind_name(k), v.acc_waits[i], want);
        }
    }
    // COMPLETE (3.2)
    if (!v.finished() || v.stuck || v.iacc != (int)seq.size() || v.tas_second_dtack != v.tas_cycles)
        fail(C_COMPLETE, total, "accesses completed %d of %ld%s, TAS %ld with a second acknowledge %ld", v.iacc, (long)seq.size(), v.stuck ? " (stuck)" : "", v.tas_cycles, v.tas_second_dtack);
    // RETURNED (1, 4.5-4.7)
    const long distance = pulses - 2 * v.waits - nominal;
    const long dist_rule = lost - 2 * v.waits - (long)m_debt - (long)DEBT_MOD * wraps;
    if (DEN > 2 * NUM) {
        if (m_debt != 0 || recov != lost - (long)DEBT_MOD * wraps)
            fail(C_RETURNED, total, "at the end debt %u, recovery pulses %ld, lost %ld, wraps %ld", m_debt, recov, lost, wraps);
    } else if (recov != 0) fail(C_RETURNED, total, "den = 2*num: %ld recovery pulses, section 4.6 allows none", recov);
    if (distance != dist_rule) fail(C_RETURNED, total, "distance from the nominal clock %ld, lost - 2 x waits - debt - 2^14 x wraps = %ld", distance, dist_rule);
    if (MODE == "sat" && wraps == 0) fail(C_RETURNED, total, "MODE=sat: the debt never wrapped (4.7 not exercised)");
    if (!spacing_on) chk_na[C_SPACING] = true;

    printf("CENDTACK den=%d num=%d mode=%s lmax=%d maplat=%d busyback=%d kinds=%s accesses=%ld clocks=%ld\n",
           DEN, NUM, MODE.c_str(), LMAX, MAPLAT, BUSYBACK, KINDS_RD ? "rd" : "all", (long)seq.size(), total);
    printf("  pulses %ld (cpu_cen %ld, cpu_cenb %ld), first in clock %ld | nominal %ld, recovery %ld, lost %ld, debt max %u, wraps %ld, debt at the end %u\n",
           pulses, n_cen, n_cenb, first_pulse, nominal, recov, lost, debt_max, wraps, m_debt);
    printf("  processor model: cycles %ld, TAS %ld, samples %ld, wait states %ld, accesses checked against the formulas of 4.4 %ld, end in clock %ld\n",
           v.cycles, v.tas_cycles, v.samples, v.waits, formula_n, v.done_clk);
    printf("  distance from the nominal clock %+ld (pulses - 2 x wait states - nominal pulses) = lost - 2 x wait states - debt - 2^14 x wraps %+ld\n", distance, dist_rule);
    printf("  memory busy while the processor is not waiting (4.5): pulses with busy after DTACK %ld, pulses with busy in the strobe clock %ld\n", n_after, n_strobe);
    std::string failed;
    for (int c = 0; c < C_N; c++) {
        if (chk_na[c]) { printf("CHECK %-10s (%s): not applicable to this run\n", CHK_NAME[c], CHK_SEC[c]); continue; }
        if (chk_n[c] == 0) printf("CHECK %-10s (%s): OK\n", CHK_NAME[c], CHK_SEC[c]);
        else {
            printf("CHECK %-10s (%s): FAIL, %ld failures, first: %s\n", CHK_NAME[c], CHK_SEC[c], chk_n[c], chk_msg[c].c_str());
            failed += (failed.empty() ? "" : ",") + std::string(CHK_NAME[c]);
        }
    }
    const bool pass = failed.empty();
    printf("FAILED CHECKS: %s\n", pass ? "none" : failed.c_str());
    printf("SUMMARY %s %ld %08x%08x %ld %ld %s\n", pass ? "PASS" : "FAIL", distance, (unsigned)(hash >> 32), (unsigned)(hash & 0xFFFFFFFFu), n_after, n_strobe, pass ? "-" : failed.c_str());
    printf(pass ? "RESULT: PASS\n" : "RESULT: FAIL\n");
    delete top; return pass ? 0 : 1;
}
