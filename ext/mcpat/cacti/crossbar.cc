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

#include "crossbar.h"

#define ASPECT_THRESHOLD .8
#define ADJ 1

Crossbar::Crossbar(
    double n_inp_,
    double n_out_,
    double flit_size_,
    TechnologyParameter::DeviceType *dt
): n_inp(n_inp_), n_out(n_out_), flit_size(flit_size_), deviceType(dt) {
    min_w_pmos = deviceType->n_to_p_eff_curr_drv_ratio * g_tp.min_w_nmos_;
    Vdd = dt->Vdd;
    CB_ADJ = 1;
}

Crossbar::~Crossbar() {}

double Crossbar::output_buffer() {

    //Wire winit(4, 4);
    double l_eff = n_inp * flit_size * g_tp.wire_outside_mat.pitch;
    Wire w1(g_ip->wt, l_eff);
    //double s1 = w1.repeater_size *l_eff*ADJ/w1.repeater_spacing;
    double s1 = w1.repeater_size * (l_eff < w1.repeater_spacing ?
                                    l_eff * ADJ / w1.repeater_spacing : ADJ);
    double pton_size = deviceType->n_to_p_eff_curr_drv_ratio;
    // the model assumes input capacitance of the wire driver = input capacitance of nand + nor = input cap of the driver transistor
    TriS1 = s1 * (1 + pton_size) / (2 + pton_size + 1 + 2 * pton_size);
    TriS2 = s1; //driver transistor

    if (TriS1 < 1)
        TriS1 = 1;

    double input_cap = gate_C(TriS1 * (2 * min_w_pmos + g_tp.min_w_nmos_), 0) +
                       gate_C(TriS1 * (min_w_pmos + 2 * g_tp.min_w_nmos_), 0);
//  input_cap += drain_C_(TriS1*g_tp.min_w_nmos_, NCH, 1, 1, g_tp.cell_h_def) +
//    drain_C_(TriS1*min_w_pmos, PCH, 1, 1, g_tp.cell_h_def)*2 +
//    gate_C(TriS2*g_tp.min_w_nmos_, 0)+
//    drain_C_(TriS1*min_w_pmos, NCH, 1, 1, g_tp.cell_h_def)*2 +
//    drain_C_(TriS1*min_w_pmos, PCH, 1, 1, g_tp.cell_h_def) +
//    gate_C(TriS2*min_w_pmos, 0);
    tri_int_cap = drain_C_(TriS1 * g_tp.min_w_nmos_, NCH, 1, 1, g_tp.cell_h_def) +
        drain_C_(TriS1 * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def) * 2 +
        gate_C(TriS2 * g_tp.min_w_nmos_, 0) +
        drain_C_(TriS1 * min_w_pmos, NCH, 1, 1, g_tp.cell_h_def) * 2 +
        drain_C_(TriS1 * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def) +
        gate_C(TriS2 * min_w_pmos, 0);
    double output_cap = drain_C_(TriS2 * g_tp.min_w_nmos_, NCH, 1, 1,
                                 g_tp.cell_h_def) +
        drain_C_(TriS2 * min_w_pmos, PCH, 1, 1, g_tp.cell_h_def);
    double ctr_cap = gate_C(TriS2 * (min_w_pmos + g_tp.min_w_nmos_), 0);

    tri_inp_cap = input_cap;
    tri_out_cap = output_cap;
    tri_ctr_cap = ctr_cap;
    return input_cap + output_cap + ctr_cap;
}

void Crossbar::compute_power() {

    Wire winit(4, 4);
    double tri_cap = output_buffer();
    assert(tri_cap > 0);
    //area of a tristate logic
    double g_area = compute_gate_area(INV, 1, TriS2 * g_tp.min_w_nmos_,
                                      TriS2 * min_w_pmos, g_tp.cell_h_def);
    g_area *= 2; // to model area of output transistors
    g_area += compute_gate_area (NAND, 2, TriS1 * 2 * g_tp.min_w_nmos_,
                                 TriS1 * min_w_pmos, g_tp.cell_h_def);
    g_area += compute_gate_area (NOR, 2, TriS1 * g_tp.min_w_nmos_,
                                 TriS1 * 2 * min_w_pmos, g_tp.cell_h_def);
    double width /*per tristate*/ = g_area / (CB_ADJ * g_tp.cell_h_def);
    // effective no. of tristate buffers that need to be laid side by side
    int ntri = (int)ceil(g_tp.cell_h_def / (g_tp.wire_outside_mat.pitch));
    double wire_len = MAX(width * ntri * n_out,
                          flit_size * g_tp.wire_outside_mat.pitch * n_out);
    Wire w1(g_ip->wt, wire_len);

    area.w = wire_len;
    area.h = g_tp.wire_outside_mat.pitch * n_inp * flit_size * CB_ADJ;
    Wire w2(g_ip->wt, area.h);

    double aspect_ratio_cb = (area.h / area.w) * (n_out / n_inp);
    if (aspect_ratio_cb > 1) aspect_ratio_cb = 1 / aspect_ratio_cb;

    if (aspect_ratio_cb < ASPECT_THRESHOLD) {
        if (n_out > 2 && n_inp > 2) {
            CB_ADJ += 0.2;
            //cout << "CB ADJ " << CB_ADJ << endl;
            if (CB_ADJ < 4) {
                this->compute_power();
            }
        }
    }



    power.readOp.dynamic =
        (w1.power.readOp.dynamic + w2.power.readOp.dynamic +
         (tri_inp_cap * n_out + tri_out_cap * n_inp + tri_ctr_cap +
          tri_int_cap) * Vdd * Vdd) * flit_size;
    power.readOp.leakage = n_inp * n_out * flit_size * (
        cmos_Isub_leakage(g_tp.min_w_nmos_ * TriS2 * 2, min_w_pmos * TriS2 * 2,
                          1, inv) * Vdd +
        cmos_Isub_leakage(g_tp.min_w_nmos_ * TriS1 * 3, min_w_pmos * TriS1 * 3,
                          2, nand) * Vdd +
        cmos_Isub_leakage(g_tp.min_w_nmos_ * TriS1 * 3, min_w_pmos * TriS1 * 3,
                          2, nor) * Vdd +
        w1.power.readOp.leakage + w2.power.readOp.leakage);
    power.readOp.gate_leakage = n_inp * n_out * flit_size * (
        cmos_Ig_leakage(g_tp.min_w_nmos_ * TriS2 * 2, min_w_pmos * TriS2 * 2,
                        1, inv) * Vdd +
        cmos_Ig_leakage(g_tp.min_w_nmos_ * TriS1 * 3, min_w_pmos * TriS1 * 3,
                        2, nand) * Vdd +
        cmos_Ig_leakage(g_tp.min_w_nmos_ * TriS1 * 3, min_w_pmos * TriS1 * 3,
                        2, nor) * Vdd +
        w1.power.readOp.gate_leakage + w2.power.readOp.gate_leakage);

    // delay calculation
    double l_eff = n_inp * flit_size * g_tp.wire_outside_mat.pitch;
    Wire wdriver(g_ip->wt, l_eff);
    double res = g_tp.wire_outside_mat.R_per_um * (area.w + area.h) +
        tr_R_on(g_tp.min_w_nmos_ * wdriver.repeater_size, NCH, 1);
    double cap = g_tp.wire_outside_mat.C_per_um * (area.w + area.h) + n_out *
        tri_inp_cap + n_inp * tri_out_cap;
    delay = horowitz(w1.signal_rise_time(), res * cap, deviceType->Vth /
                     deviceType->Vdd, deviceType->Vth / deviceType->Vdd, RISE);

    Wire wreset();
}

void Crossbar::print_crossbar() {
    cout << "\nCrossbar Stats (" << n_inp << "x" << n_out << ")\n\n";
    cout << "Flit size        : " << flit_size << " bits" << endl;
    cout << "Width            : " << area.w << " u" << endl;
    cout << "Height           : " << area.h << " u" << endl;
    cout << "Dynamic Power    : " << power.readOp.dynamic*1e9 *
        MIN(n_inp, n_out) << " (nJ)" << endl;
    cout << "Leakage Power    : " << power.readOp.leakage*1e3 << " (mW)"
         << endl;
    cout << "Gate Leakage Power    : " << power.readOp.gate_leakage*1e3
         << " (mW)" << endl;
    cout << "Crossbar Delay   : " << delay*1e12 << " ps\n";
}


