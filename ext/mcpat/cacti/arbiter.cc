/*****************************************************************************
 *                                McPAT/CACTI
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#include "arbiter.h"

Arbiter::Arbiter(
    double n_req,
    double flit_size_,
    double output_len,
    TechnologyParameter::DeviceType *dt
    ): R(n_req), flit_size(flit_size_),
       o_len (output_len), deviceType(dt) {
    min_w_pmos = deviceType->n_to_p_eff_curr_drv_ratio * g_tp.min_w_nmos_;
    Vdd = dt->Vdd;
    double technology = g_ip->F_sz_um;
    NTn1 = 13.5 * technology / 2;
    PTn1 = 76 * technology / 2;
    NTn2 = 13.5 * technology / 2;
    PTn2 = 76 * technology / 2;
    NTi = 12.5 * technology / 2;
    PTi = 25 * technology / 2;
    NTtr = 10 * technology / 2; /*Transmission gate's nmos tr. length*/
    PTtr = 20 * technology / 2; /* pmos tr. length*/
}

Arbiter::~Arbiter() {}

double
Arbiter::arb_req() {
    double temp = ((R - 1) * (2 * gate_C(NTn1, 0) + gate_C(PTn1, 0)) + 2 *
                   gate_C(NTn2, 0) +
                   gate_C(PTn2, 0) + gate_C(NTi, 0) + gate_C(PTi, 0) +
                   drain_C_(NTi, 0, 1, 1, g_tp.cell_h_def) +
                   drain_C_(PTi, 1, 1, 1, g_tp.cell_h_def));
    return temp;
}

double
Arbiter::arb_pri() {
    /* switching capacitance of flip-flop is ignored */
    double temp = 2 * (2 * gate_C(NTn1, 0) + gate_C(PTn1, 0));
    return temp;
}


double
Arbiter::arb_grant() {
    double temp = drain_C_(NTn1, 0, 1, 1, g_tp.cell_h_def) * 2 +
        drain_C_(PTn1, 1, 1, 1, g_tp.cell_h_def) + crossbar_ctrline();
    return temp;
}

double
Arbiter::arb_int() {
    double temp = (drain_C_(NTn1, 0, 1, 1, g_tp.cell_h_def) * 2 +
                   drain_C_(PTn1, 1, 1, 1, g_tp.cell_h_def) +
                   2 * gate_C(NTn2, 0) + gate_C(PTn2, 0));
    return temp;
}

void
Arbiter::compute_power() {
    power.readOp.dynamic = (R * arb_req() * Vdd * Vdd / 2 + R * arb_pri() *
                            Vdd * Vdd / 2 +
                            arb_grant() * Vdd * Vdd + arb_int() * 0.5 * Vdd *
                            Vdd);
    double nor1_leak = cmos_Isub_leakage(g_tp.min_w_nmos_ * NTn1 * 2,
                                         min_w_pmos * PTn1 * 2, 2, nor);
    double nor2_leak = cmos_Isub_leakage(g_tp.min_w_nmos_ * NTn2 * R,
                                         min_w_pmos * PTn2 * R, 2, nor);
    double not_leak = cmos_Isub_leakage(g_tp.min_w_nmos_ * NTi,
                                        min_w_pmos * PTi, 1, inv);
    double nor1_leak_gate = cmos_Ig_leakage(g_tp.min_w_nmos_ * NTn1 * 2,
                                            min_w_pmos * PTn1 * 2, 2, nor);
    double nor2_leak_gate = cmos_Ig_leakage(g_tp.min_w_nmos_ * NTn2 * R,
                                            min_w_pmos * PTn2 * R, 2, nor);
    double not_leak_gate  = cmos_Ig_leakage(g_tp.min_w_nmos_ * NTi,
                                            min_w_pmos * PTi, 1, inv);
    //FIXME include priority table leakage
    power.readOp.leakage = (nor1_leak + nor2_leak + not_leak) * Vdd;
    power.readOp.gate_leakage = nor1_leak_gate * Vdd + nor2_leak_gate * Vdd +
        not_leak_gate * Vdd;
}

double //wire cap with triple spacing
Arbiter::Cw3(double length) {
    Wire wc(g_ip->wt, length, 1, 3, 3);
    double temp = (wc.wire_cap(length, true));
    return temp;
}

double
Arbiter::crossbar_ctrline() {
    double temp = (Cw3(o_len * 1e-6 /* m */) +
                   drain_C_(NTi, 0, 1, 1, g_tp.cell_h_def) + drain_C_(PTi, 1, 1, 1, g_tp.cell_h_def) +
                   gate_C(NTi, 0) + gate_C(PTi, 0));
    return temp;
}

double
Arbiter::transmission_buf_ctrcap() {
    double temp = gate_C(NTtr, 0) + gate_C(PTtr, 0);
    return temp;
}


void Arbiter::print_arbiter() {
    cout << "\nArbiter Stats ("   << R << " input arbiter" << ")\n\n";
    cout << "Flit size        : " << flit_size << " bits" << endl;
    cout << "Dynamic Power    : " << power.readOp.dynamic*1e9 << " (nJ)" << endl;
    cout << "Leakage Power    : " << power.readOp.leakage*1e3 << " (mW)" << endl;
}


