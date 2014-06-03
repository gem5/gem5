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



#ifndef __ROUTER_H__
#define __ROUTER_H__

#include <assert.h>

#include <iostream>

#include "arbiter.h"
#include "basic_circuit.h"
#include "cacti_interface.h"
#include "component.h"
#include "crossbar.h"
#include "mat.h"
#include "parameter.h"
#include "wire.h"

class Router : public Component {
public:
    Router(
        double flit_size_,
        double vc_buf, /* vc size = vc_buffer_size * flit_size */
        double vc_count,
        TechnologyParameter::DeviceType *dt = &(g_tp.peri_global),
        double I_ = 5,
        double O_ = 5,
        double M_ = 0.6);
    ~Router();


    void print_router();

    Component arbiter, crossbar, buffer;

    double cycle_time, max_cyc;
    double flit_size;
    double vc_count;
    double vc_buffer_size; /* vc size = vc_buffer_size * flit_size */

private:
    TechnologyParameter::DeviceType *deviceType;
    double FREQUENCY; // move this to config file --TODO
    double Cw3(double len);
    double gate_cap(double w);
    double diff_cap(double w, int type /*0 for n-mos and 1 for p-mos*/, double stack);
    enum Wire_type wtype;
    enum Wire_placement wire_placement;
    //corssbar
    double NTtr, PTtr, wt, ht, I, O, NTi, PTi, NTid, PTid, NTod, PTod, TriS1, TriS2;
    double M; //network load
    double transmission_buf_inpcap();
    double transmission_buf_outcap();
    double transmission_buf_ctrcap();
    double crossbar_inpline();
    double crossbar_outline();
    double crossbar_ctrline();
    double tr_crossbar_power();
    void  cb_stats ();
    double arb_power();
    void  arb_stats ();
    double buffer_params();
    void buffer_stats();


    //arbiter

    //buffer

    //router params
    double Vdd;

    void calc_router_parameters();
    void get_router_area();
    void get_router_power();
    void get_router_delay();

    double min_w_pmos;


};

#endif
