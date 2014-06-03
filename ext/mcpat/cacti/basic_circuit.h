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



#ifndef __BASIC_CIRCUIT_H__
#define __BASIC_CIRCUIT_H__

#include "cacti_interface.h"
#include "const.h"

using namespace std;

#define UNI_LEAK_STACK_FACTOR 0.43

int powers (int base, int n);
bool is_pow2(int64_t val);
uint32_t _log2(uint64_t num);
int factorial(int n, int m = 1);
int combination(int n, int m);

//#define DBG
#ifdef DBG
#define PRINTDW(a);\
    a;
#else
#define PRINTDW(a);\

#endif


enum Wire_placement {
    outside_mat,
    inside_mat,
    local_wires
};



enum Htree_type {
    Add_htree,
    Data_in_htree,
    Data_out_htree,
    Search_in_htree,
    Search_out_htree,
};

enum Gate_type {
    nmos,
    pmos,
    inv,
    nand,
    nor,
    tri,
    tg
};

enum Half_net_topology {
    parallel,
    series
};

double logtwo (double x);

double gate_C(
    double width,
    double wirelength,
    bool _is_dram = false,
    bool _is_sram = false,
    bool _is_wl_tr = false);

double gate_C_pass(
    double width,
    double wirelength,
    bool   _is_dram = false,
    bool   _is_sram = false,
    bool   _is_wl_tr = false);

double drain_C_(
    double width,
    int nchannel,
    int stack,
    int next_arg_thresh_folding_width_or_height_cell,
    double fold_dimension,
    bool _is_dram = false,
    bool _is_sram = false,
    bool _is_wl_tr = false);

double tr_R_on(
    double width,
    int nchannel,
    int stack,
    bool _is_dram = false,
    bool _is_sram = false,
    bool _is_wl_tr = false);

double R_to_w(
    double res,
    int nchannel,
    bool _is_dram = false,
    bool _is_sram = false,
    bool _is_wl_tr = false);

double horowitz (
    double inputramptime,
    double tf,
    double vs1,
    double vs2,
    int rise);

double pmos_to_nmos_sz_ratio(
    bool _is_dram = false,
    bool _is_wl_tr = false);

double simplified_nmos_leakage(
    double nwidth,
    bool _is_dram = false,
    bool _is_cell = false,
    bool _is_wl_tr = false);

double simplified_pmos_leakage(
    double pwidth,
    bool _is_dram = false,
    bool _is_cell = false,
    bool _is_wl_tr = false);


double cmos_Ileak(
    double nWidth,
    double pWidth,
    bool _is_dram = false,
    bool _is_cell = false,
    bool _is_wl_tr = false);

double cmos_Ig_n(
    double nWidth,
    bool _is_dram = false,
    bool _is_cell = false,
    bool _is_wl_tr = false);

double cmos_Ig_p(
    double pWidth,
    bool _is_dram = false,
    bool _is_cell = false,
    bool _is_wl_tr = false);


double cmos_Isub_leakage(
    double nWidth,
    double pWidth,
    int    fanin,
    enum Gate_type g_type,
    bool _is_dram = false,
    bool _is_cell = false,
    bool _is_wl_tr = false,
    enum Half_net_topology topo = series);

double cmos_Ig_leakage(
    double nWidth,
    double pWidth,
    int    fanin,
    enum Gate_type g_type,
    bool _is_dram = false,
    bool _is_cell = false,
    bool _is_wl_tr = false,
    enum Half_net_topology topo = series);

double shortcircuit(
    double vt,
    double velocity_index,
    double c_in,
    double c_out,
    double w_nmos,
    double w_pmos,
    double i_on_n,
    double i_on_p,
    double i_on_n_in,
    double i_on_p_in,
    double vdd);

double shortcircuit_simple(
    double vt,
    double velocity_index,
    double c_in,
    double c_out,
    double w_nmos,
    double w_pmos,
    double i_on_n,
    double i_on_p,
    double i_on_n_in,
    double i_on_p_in,
    double vdd);
//set power point product mask; strictly speaking this is not real point product
inline void set_pppm(
    double * pppv,
    double a = 1,
    double b = 1,
    double c = 1,
    double d = 1
) {
    pppv[0] = a;
    pppv[1] = b;
    pppv[2] = c;
    pppv[3] = d;

}

inline void set_sppm(
    double * sppv,
    double a = 1,
    double b = 1,
    double c = 1,
    double d = 1
) {
    sppv[0] = a;
    sppv[1] = b;
    sppv[2] = c;
}

#endif
