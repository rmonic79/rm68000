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

// tb_m68k_bus_fsm.cpp - testbench of the bus cycle state machine (rtl/cpu_si/m68k_bus_fsm.sv) tested on its own.
//
// Source of the expected values: MC68000UM 5.1.1 (read), 5.1.2 (write), 5.1.3 (read-modify-write), 5.4 (BERR), cycle
// for 6800 peripherals (VPA/VMA/E). Unit of measure: the EDGE (half period of the 68000 clock). Edge 0 is the one that
// asserts AS (rising edge of S2). From the manual:
//   - read: AS and UDS/LDS asserted together at edge 0 (rising edge of S2); R/W high for the whole cycle;
//   - write: AS asserted and R/W low at edge 0; UDS/LDS asserted at the rising edge of S4 (edge 2); data on the bus
//     from S3 (edge 1) as long as the strobes are asserted; R/W returns high at the rising edge at the end of S7; data
//     bus released then;
//   - DTACK (or BERR) sampled at the falling edge at the end of S4 (edge 3); if absent, wait states of whole clocks
//     (2 edges each), with a new sample at every falling edge (edges 3, 5, 7, ...);
//   - AS, UDS, LDS negated at the falling edge that enters S7: edge 5 + 2n with n wait states.
//   If the slave makes DTACK visible before edge D (D>=1), n = 0 for D<=3, otherwise n = ceil((D-3)/2).
//   - RMW (TAS): AS stays asserted between read and write, strobes negated between the two halves; in the write half
//     R/W low before the strobes, DTACK sampled one edge after the strobes are asserted and then every clock, AS negated
//     three edges after the strobes plus two for each wait state (same structure as the simple write, UM 5.1.3).
//   - 6800 (VPA): VMA asserted with E low, then E rises with VMA asserted; the cycle ends (AS negated) at the falling
//     edge of E.
// Read data: the slave drives a valid value ONLY on the lanes whose strobe is asserted and the complement on the
// others; the latched data (rdata) must be the valid one on the requested lane.
// Module contract (ports, comment at the top of the .sv): req starts the cycle (held while bus_active), bus_done is a
// pulse at the end of the cycle, e_cnt is the E counter (0..9, advances on cen2, E high for e_cnt>=6).
// Clock modes: TWO PHASES (cen and cen2 alternating: cen = rising, cen2 = falling) and SINGLE PHASE (cen = cen2 = 1 on
// every clock, like the full-processor testbenches). With two phases it is also checked that AS is asserted on cen.
// NOT tested: address error (aerr: pin behavior not fixed by the manual), the exact edge of the data latch within the
// strobe window (S6 or entry into S7: a slave that changes the data between the two is not modeled), the exact number
// of clocks of a 6800 cycle, the exact length of a cycle terminated by BERR (checked as a range).

#include "Vm68k_bus_fsm.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <string>

static Vm68k_bus_fsm* dut = nullptr;
static uint64_t t = 0;
double sc_time_stamp() { return (double)t; }

static int n_checks = 0;
static int n_fail = 0;
static void chk(bool ok, const std::string& what, long long exp, long long got) {
    n_checks++;
    if (!ok) {
        n_fail++;
        if (n_fail <= 300) printf("FAIL: %s expected=%lld read=%lld\n", what.c_str(), exp, got);
    }
}

static bool g_two = true;    // true: two phases (cen/cen2 alternating); false: single phase (cen=cen2=1)
static int  g_phase = 0;     // two phases: 0 = the next tick is cen (rising), 1 = cen2 (falling)
static int  g_ecnt = 0;      // E counter driven by the testbench (module contract: advances on cen2)
static bool last_cen = false, last_cen2 = false;
static bool g_late = false;  // read data valid on the pins only for the falling edge entering S7, complement before (UM 5.1.1 STATE 7)

static void tick() {
    if (g_two) { dut->cen = (g_phase == 0) ? 1 : 0; dut->cen2 = (g_phase == 1) ? 1 : 0; }
    else       { dut->cen = 1; dut->cen2 = 1; }
    dut->e_cnt = (uint8_t)g_ecnt;
    last_cen = dut->cen; last_cen2 = dut->cen2;
    dut->clk = 0; dut->eval(); t++;
    dut->clk = 1; dut->eval(); t++;
    if (last_cen2) g_ecnt = (g_ecnt + 1) % 10;
    if (g_two) g_phase ^= 1;
}

// slave: valid data on the lanes whose strobe is asserted (current pins), complement on the others
static void drive_data(uint16_t v) {
    uint16_t hi = (dut->uds_n == 0) ? (uint16_t)(v & 0xFF00) : (uint16_t)(~v & 0xFF00);
    uint16_t lo = (dut->lds_n == 0) ? (uint16_t)(v & 0x00FF) : (uint16_t)(~v & 0x00FF);
    dut->data_in = (uint16_t)(hi | lo);
}

static void idle(int n) {
    dut->req = 0; dut->dtack_n = 1; dut->berr_n = 1; dut->vpa_n = 1;
    for (int i = 0; i < n; i++) {
        drive_data(0);
        tick();
        chk(dut->as_n == 1, "idle: AS negated", 1, dut->as_n);
        chk(dut->uds_n == 1 && dut->lds_n == 1, "idle: UDS/LDS negated", 1, (dut->uds_n & dut->lds_n));
    }
}

static int waits_for(int D) { return (D <= 3) ? 0 : (D - 2) / 2; }   // ceil((D-3)/2) for D>=4 (UM 5.1.1, S4 and wait states)

// Simple cycle. term: 0 = DTACK, 1 = BERR. D = first edge (counted from AS) before which the slave makes the
// termination visible; it stays asserted as long as AS is low.
static void cycle(const std::string& nm0, bool we, bool u, bool l, int D, int term) {
    const std::string nm = std::string(g_two ? "[2 phases] " : "[1 phase] ") + nm0 + " D=" + std::to_string(D);
    const uint16_t wv = (uint16_t)(0x3C5A ^ (D * 0x1111));
    const uint16_t rv = (uint16_t)(0xA5C3 ^ (D * 0x0F0F));
    dut->we = we; dut->uds_req = u; dut->lds_req = l; dut->wdata = wv; dut->rmw = 0; dut->aerr = 0;
    dut->dtack_n = 1; dut->berr_n = 1; dut->vpa_n = 1;
    dut->req = 1;
    const int n = waits_for(D);
    const int e_neg = 5 + 2 * n;              // UM 5.1.1/5.1.2: AS negated at the falling edge that enters S7
    bool as_seen = false; int e = -1; int as_up = -1; int dones = 0; int done_e = -1;
    for (int i = 0; i < 400; i++) {
        int en = as_seen ? e + 1 : -1;
        bool on = as_seen && dut->as_n == 0 && en >= D;
        dut->dtack_n = (term == 0 && on) ? 0 : 1;
        dut->berr_n  = (term == 1 && on) ? 0 : 1;
        // late data: the slave puts the valid data on the bus only for the edge entering S7 ("On the falling edge of the
        // clock entering S7, the processor accepts data from the device", UM 5.1.1 p. 5-4); the complement before it
        drive_data((g_late && !(as_seen && en == e_neg)) ? (uint16_t)~rv : rv);
        tick();
        if (dut->bus_active) dut->req = 0;
        if (!as_seen) {
            // while AS is high; in the tick in which AS falls, the read also asserts the strobes (UM 5.1.1, edge 0)
            if (dut->as_n == 1) chk(dut->uds_n == 1 && dut->lds_n == 1, nm + ": no strobe before AS", 1, (dut->uds_n & dut->lds_n));
            chk(dut->bus_done == 0, nm + ": no bus_done before AS", 0, dut->bus_done);
            if (dut->as_n == 0) {
                as_seen = true; e = 0;
                if (g_two) chk(last_cen, nm + ": AS asserted on cen (rising edge of S2, UM 5.1.1)", 1, last_cen);
                chk(dut->rw == (we ? 0 : 1), nm + ": R/W at the AS edge (UM 5.1.2: low with AS on a write)", we ? 0 : 1, dut->rw);
            } else if (i > 40) { chk(false, nm + ": AS never asserted", 0, 1); return; }
            else continue;
        } else e++;
        if (dut->bus_done) { dones++; done_e = e; }
        if (as_up < 0 && dut->as_n == 1) as_up = e;
        if (term == 1) { if (as_up >= 0 && e >= as_up + 3) break; else continue; }
        // --- DTACK: exact expected values edge by edge ---
        const std::string at = nm + " edge " + std::to_string(e);
        chk(dut->as_n == ((e >= e_neg) ? 1 : 0), at + ": AS", (e >= e_neg) ? 1 : 0, dut->as_n);
        bool strobe_on = we ? (e >= 2 && e < e_neg) : (e < e_neg);
        chk(dut->uds_n == ((u && strobe_on) ? 0 : 1), at + ": UDS", (u && strobe_on) ? 0 : 1, dut->uds_n);
        chk(dut->lds_n == ((l && strobe_on) ? 0 : 1), at + ": LDS", (l && strobe_on) ? 0 : 1, dut->lds_n);
        if (!we) {
            chk(dut->rw == 1, at + ": R/W high on a read", 1, dut->rw);
            chk(dut->data_oe == 0, at + ": data bus not driven on a read", 0, dut->data_oe);
        } else {
            int rw_exp = (e >= e_neg + 1) ? 1 : 0;   // UM 5.1.2: R/W high at the rising edge at the end of S7
            chk(dut->rw == rw_exp, at + ": R/W on a write", rw_exp, dut->rw);
            if (e == 0) chk(dut->data_oe == 0, at + ": data bus not yet driven at S2 (UM 5.1.2 STATE 3: driven during S3)", 0, dut->data_oe);
            if (e >= 1 && e < e_neg) {
                chk(dut->data_oe == 1, at + ": data driven (S3..S7)", 1, dut->data_oe);
                chk(dut->data_out == wv, at + ": written data", wv, dut->data_out);
            }
            if (e >= e_neg + 1) chk(dut->data_oe == 0, at + ": data bus released after S7", 0, dut->data_oe);
        }
        if (e >= e_neg + 2) break;
    }
    chk(as_seen, nm + ": cycle started", 1, as_seen);
    if (term == 0) {
        chk(as_up == e_neg, nm + ": AS negation edge = 5+2n", e_neg, as_up);
        chk(dones == 1 && done_e == e_neg, nm + ": a single bus_done, at the S7 edge", e_neg, (dones == 1) ? done_e : -100 - dones);
    } else {
        // BERR (UM 5.4): sampled like DTACK; the cycle does not end before the sample and ends within one clock
        chk(as_up >= e_neg && as_up <= e_neg + 2, nm + ": BERR terminates the cycle (AS negated between 5+2n and 7+2n)", e_neg, as_up);
        chk(dones == 1, nm + ": a single bus_done with BERR", 1, dones);
    }
    if (!we && term == 0) {   // with BERR the data is not defined (UM 5.4): not checked
        if (u) chk((dut->rdata & 0xFF00) == (rv & 0xFF00), nm + ": read data, high byte", rv & 0xFF00, dut->rdata & 0xFF00);
        if (l) chk((dut->rdata & 0x00FF) == (rv & 0x00FF), nm + ": read data, low byte", rv & 0x00FF, dut->rdata & 0x00FF);
    }
    idle(3);
}

// Read-modify-write (UM 5.1.3). D: termination of the read (from AS); D2: of the write (from the strobes).
static void rmw_cycle(int D, int D2, int gap) {
    const std::string nm = std::string(g_two ? "[2 phases] " : "[1 phase] ") + "RMW D=" + std::to_string(D) + " D2=" + std::to_string(D2) + " gap=" + std::to_string(gap);
    const uint16_t rv = (uint16_t)(0x5AA5 ^ (D * 0x0101) ^ (D2 << 12));
    const uint16_t wv = (uint16_t)(0xC33C ^ (D2 * 0x1010) ^ gap);
    dut->we = 0; dut->rmw = 1; dut->uds_req = 1; dut->lds_req = 1; dut->wdata = 0; dut->aerr = 0;
    dut->dtack_n = 1; dut->berr_n = 1; dut->vpa_n = 1;
    dut->req = 1;
    const int n = waits_for(D);
    const int e_neg = 5 + 2 * n;
    bool as_seen = false; int e = -1; int dones = 0;
    // read half
    for (int i = 0; i < 400; i++) {
        int en = as_seen ? e + 1 : -1;
        bool on = as_seen && (dut->uds_n == 0 || dut->lds_n == 0) && en >= D;
        dut->dtack_n = on ? 0 : 1;
        drive_data(rv);
        tick();
        if (dut->bus_active) dut->req = 0;
        if (!as_seen) {
            if (dut->as_n == 0) { as_seen = true; e = 0; }
            else if (i > 40) { chk(false, nm + ": AS never asserted", 0, 1); return; }
            else continue;
        } else e++;
        const std::string at = nm + " read edge " + std::to_string(e);
        if (dut->bus_done) dones++;
        chk(dut->as_n == 0, at + ": AS stays asserted", 0, dut->as_n);
        chk(dut->rw == 1, at + ": R/W high during the read", 1, dut->rw);
        int s_exp = (e < e_neg) ? 0 : 1;
        chk(dut->uds_n == s_exp && dut->lds_n == s_exp, at + ": UDS/LDS of the read", s_exp, dut->uds_n);
        if (e >= e_neg) break;
    }
    chk(dones == 1, nm + ": bus_done at the end of the read", 1, dones);
    chk((dut->rdata) == rv, nm + ": read data", rv, dut->rdata);
    dut->dtack_n = 1;
    for (int i = 0; i < gap; i++) {
        drive_data(0); tick();
        chk(dut->as_n == 0, nm + ": AS asserted between the two halves", 0, dut->as_n);
        chk(dut->uds_n == 1 && dut->lds_n == 1, nm + ": strobes negated between the two halves", 1, (dut->uds_n & dut->lds_n));
    }
    // write half
    dut->we = 1; dut->rmw = 0; dut->wdata = wv; dut->req = 1;
    int s = -1; int k = -1; int as_up = -1; dones = 0; bool rw_low_before = false; bool prev_rw_low = false;
    const int n2 = (D2 <= 1) ? 0 : D2 / 2;    // ceil((D2-1)/2)
    const int k_neg = 3 + 2 * n2;
    for (int i = 0; i < 400; i++) {
        int kn = (s >= 0) ? k + 1 : -1;
        dut->dtack_n = (s >= 0 && dut->as_n == 0 && kn >= D2) ? 0 : 1;
        drive_data(0);
        prev_rw_low = (dut->rw == 0);
        tick();
        if (dut->rmw_hold == 0) dut->req = 0;
        if (dut->bus_done) dones++;
        if (s < 0) {
            chk(dut->as_n == 0, nm + ": AS asserted before the write strobes", 0, dut->as_n);
            if (dut->uds_n == 0 || dut->lds_n == 0) {
                s = 0; k = 0; rw_low_before = prev_rw_low;
                if (g_two) chk(last_cen, nm + ": write strobes on cen", 1, last_cen);
            } else if (i > 60) { chk(false, nm + ": write strobes never asserted", 0, 1); return; }
            else continue;
        } else k++;
        const std::string at = nm + " write edge " + std::to_string(k);
        if (as_up < 0 && dut->as_n == 1) as_up = k;
        int s_exp = (k < k_neg) ? 0 : 1;
        chk(dut->uds_n == s_exp && dut->lds_n == s_exp, at + ": UDS/LDS of the write", s_exp, dut->uds_n);
        chk(dut->as_n == s_exp, at + ": AS", s_exp, dut->as_n);
        if (k < k_neg) {
            chk(dut->rw == 0, at + ": R/W low", 0, dut->rw);
            chk(dut->data_oe == 1 && dut->data_out == wv, at + ": written data", wv, dut->data_oe ? dut->data_out : -1);
        }
        if (k >= k_neg + 1) break;
    }
    chk(rw_low_before, nm + ": R/W low before the write strobes (UM 5.1.3)", 1, rw_low_before);
    chk(as_up == k_neg, nm + ": AS negated 3+2n edges after the strobes", k_neg, as_up);
    chk(dones == 1, nm + ": one bus_done at the end of the write", 1, dones);
    dut->we = 0;
    idle(3);
}

// 6800 cycle: the slave asserts VPA (never DTACK) from edge 1. Two phases only (E follows the real clocks).
static void vpa_cycle(int pre) {
    const std::string nm = "[2 phases] VPA pre=" + std::to_string(pre);
    idle(pre);
    const uint16_t rv = (uint16_t)(0x6C93 ^ (pre * 0x0505));
    dut->we = 0; dut->rmw = 0; dut->uds_req = 1; dut->lds_req = 1; dut->aerr = 0; dut->req = 1;
    bool as_seen = false; int e = -1; int vma_e = -1; bool e_high_vma = false; int as_up = -1;
    for (int i = 0; i < 600 && as_up < 0; i++) {
        dut->vpa_n = (as_seen && dut->as_n == 0) ? 0 : 1;
        dut->dtack_n = 1; dut->berr_n = 1;
        drive_data(rv);
        int ec_before = g_ecnt; bool is_c2 = g_two && (g_phase == 1);
        tick();
        if (dut->bus_active) dut->req = 0;
        if (!as_seen) {
            chk(dut->vma_n == 1, nm + ": VMA negated before the cycle", 1, dut->vma_n);
            if (dut->as_n == 0) { as_seen = true; e = 0; }
            continue;
        }
        e++;
        if (vma_e < 0 && dut->vma_n == 0) {
            vma_e = e;
            chk(ec_before < 6 && g_ecnt < 6, nm + ": VMA asserted with E low", 1, ec_before);
        }
        if (vma_e >= 0 && dut->vma_n == 0 && g_ecnt >= 6) e_high_vma = true;
        if (dut->as_n == 1) {
            as_up = e;
            chk(is_c2 && ec_before == 9, nm + ": AS negated at the falling edge of E (e_cnt 9->0 on cen2)", 9, ec_before);
            chk(vma_e >= 1, nm + ": VMA asserted during the cycle, after VPA", 1, vma_e);
            chk(e_high_vma, nm + ": E high with VMA asserted before the end", 1, e_high_vma);
        }
    }
    chk(as_up >= 0, nm + ": the VPA cycle ends", 1, as_up >= 0);
    chk(dut->rdata == rv, nm + ": read data", rv, dut->rdata);
    dut->vpa_n = 1;
    idle(2);
    chk(dut->vma_n == 1, nm + ": VMA negated after the cycle", 1, dut->vma_n);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    t = 0;   // the model is created at time 0 (Verilator refuses a model added at a later time)
    dut = new Vm68k_bus_fsm;
    dut->clk = 0; dut->rst_n = 0; dut->cen = 0; dut->cen2 = 0;
    dut->req = 0; dut->aerr = 0; dut->we = 0; dut->rmw = 0; dut->uds_req = 0; dut->lds_req = 0; dut->wdata = 0;
    dut->dtack_n = 1; dut->vpa_n = 1; dut->berr_n = 1; dut->data_in = 0; dut->e_cnt = 0;
    dut->eval();
    g_two = true; g_phase = 0; g_ecnt = 0;
    for (int i = 0; i < 4; i++) tick();
    dut->rst_n = 1;
    chk(dut->as_n == 1 && dut->uds_n == 1 && dut->lds_n == 1, "after reset AS/UDS/LDS negated", 1, dut->as_n);
    chk(dut->rw == 1, "after reset R/W high", 1, dut->rw);
    chk(dut->data_oe == 0, "after reset data bus not driven", 0, dut->data_oe);
    chk(dut->vma_n == 1, "after reset VMA negated", 1, dut->vma_n);
    idle(2);

    for (int mode = 0; mode < 2; mode++) {
        g_two = (mode == 0);
        for (int D = 1; D <= 10; D++) {
            cycle("word read", false, true, true, D, 0);
            cycle("high byte read (UDS)", false, true, false, D, 0);
            cycle("low byte read (LDS)", false, false, true, D, 0);
            cycle("word write", true, true, true, D, 0);
            cycle("high byte write (UDS)", true, true, false, D, 0);
            cycle("low byte write (LDS)", true, false, true, D, 0);
            idle(D & 1);   // changes the starting phase of the next cycle
        }
        for (int D = 1; D <= 7; D += 3) {
            cycle("read terminated by BERR", false, true, true, D, 1);
            cycle("write terminated by BERR", true, true, true, D, 1);
        }
        for (int D = 1; D <= 4; D += 3)
            for (int D2 = 1; D2 <= 5; D2++) rmw_cycle(D, D2, (D2 & 1) ? 1 : 4);
    }
    // read data accepted at the falling edge entering S7 (UM 5.1.1 STATE 7): valid on the pins only for that edge
    g_two = true; g_late = true;
    for (int D = 1; D <= 10; D++) {
        cycle("word read, data valid only at the S7 edge", false, true, true, D, 0);
        cycle("low byte read (LDS), data valid only at the S7 edge", false, false, true, D, 0);
        idle(D & 1);
    }
    g_late = false;
    g_two = true;
    for (int pre = 0; pre < 20; pre++) vpa_cycle(pre);

    dut->final();
    delete dut; dut = nullptr;
    printf("checks run: %d, failed: %d\n", n_checks, n_fail);
    if (n_fail == 0) { printf("RESULT: PASS\n"); return 0; }
    printf("RESULT: FAIL\n");
    return 1;
}
