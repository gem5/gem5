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


#ifndef __HTREE2_H__
#define __HTREE2_H__

#include "assert.h"
#include "basic_circuit.h"
#include "cacti_interface.h"
#include "component.h"
#include "parameter.h"
#include "subarray.h"
#include "wire.h"

// leakge power includes entire htree in a bank (when uca_tree == false)
// leakge power includes only part to one bank when uca_tree == true

class Htree2 : public Component {
public:
    Htree2(enum Wire_type wire_model,
           double mat_w, double mat_h, int add, int data_in, int search_data_in, int data_out, int search_data_out, int bl, int wl,
           enum Htree_type h_type, bool uca_tree_ = false, bool search_tree_ = false,
           TechnologyParameter::DeviceType *dt = &(g_tp.peri_global));
    ~Htree2() {};

    void in_htree();
    void out_htree();

    // repeaters only at h-tree nodes
    void limited_in_htree();
    void limited_out_htree();
    void input_nand(double s1, double s2, double l);
    void output_buffer(double s1, double s2, double l);

    double in_rise_time, out_rise_time;

    void set_in_rise_time(double rt) {
        in_rise_time = rt;
    }

    double max_unpipelined_link_delay;
    powerDef power_bit;


private:
    double wire_bw;
    double init_wire_bw;  // bus width at root
    enum Htree_type tree_type;
    double htree_hnodes;
    double htree_vnodes;
    double mat_width;
    double mat_height;
    int add_bits;
    int data_in_bits;
    int search_data_in_bits;
    int data_out_bits;
    int search_data_out_bits;
    int ndbl, ndwl;
    bool uca_tree; // should have full bandwidth to access all banks in the array simultaneously
    bool search_tree;

    enum Wire_type wt;
    double min_w_nmos;
    double min_w_pmos;

    TechnologyParameter::DeviceType *deviceType;

};

#endif
