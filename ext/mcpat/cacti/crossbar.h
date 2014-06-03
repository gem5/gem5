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


#ifndef __CROSSBAR__
#define __CROSSBAR__

#include <assert.h>

#include <iostream>

#include "basic_circuit.h"
#include "cacti_interface.h"
#include "component.h"
#include "mat.h"
#include "parameter.h"
#include "wire.h"

class Crossbar : public Component {
public:
    Crossbar(
        double in,
        double out,
        double flit_sz,
        TechnologyParameter::DeviceType *dt = &(g_tp.peri_global));
    ~Crossbar();

    void print_crossbar();
    double output_buffer();
    void compute_power();

    double n_inp, n_out;
    double flit_size;
    double tri_inp_cap, tri_out_cap, tri_ctr_cap, tri_int_cap;

private:
    double CB_ADJ;
    /*
     * Adjust factor of the height of the cross-point (tri-state buffer) cell (layout) in crossbar
     * buffer is adjusted to get an aspect ratio of whole cross bar close to one;
     * when adjust the ratio, the number of wires route over the tri-state buffers does not change,
     * however, the effective wiring pitch changes. Specifically, since CB_ADJ will increase
     * during the adjust, the tri-state buffer will become taller and thiner, and the effective wiring pitch
     * will increase. As a result, the height of the crossbar (area.h) will increase.
     */

    TechnologyParameter::DeviceType *deviceType;
    double TriS1, TriS2;
    double min_w_pmos, Vdd;

};




#endif
