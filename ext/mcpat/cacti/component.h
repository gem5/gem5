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



#ifndef __COMPONENT_H__
#define __COMPONENT_H__

#include "area.h"
#include "parameter.h"

using namespace std;

class Crossbar;
class Bank;

class Component {
public:
    Component();
    ~Component();

    Area area;
    // TODO: THERE IS LITTLE AGREEMENT THROUGHOUT THIS CODE ABOUT HOW THESE
    // VARIABLES SHOULD BE USED. PART OF THE PROBLEM IS NAMING. SO THAT THIS
    // MAKES MORE SENSE, ENERGY CALCULATIONS SHOULD BE SPLIT FROM POWER
    // CALCULATIONS. THIS IS THE WORST DESIGN PROBLEM THAT STILL EXISTS
    powerDef power, rt_power;
    double delay;
    double cycle_time;

    double compute_gate_area(int gate_type, int num_inputs, double w_pmos,
            double w_nmos, double h_gate);
    double compute_tr_width_after_folding(double input_width,
            double threshold_folding_width);
    double height_sense_amplifier(double pitch_sense_amp);

protected:
    int logical_effort(int num_gates_min, double g, double F, double * w_n,
            double * w_p, double C_load, double p_to_n_sz_ratio,
            bool is_dram_, bool is_wl_tr_, double max_w_nmos);

private:
    double compute_diffusion_width(int num_stacked_in, int num_folded_tr);
};

#endif

