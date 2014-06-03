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



#ifndef __WIRE_H__
#define __WIRE_H__

#include <iostream>
#include <list>

#include "assert.h"
#include "basic_circuit.h"
#include "cacti_interface.h"
#include "component.h"
#include "parameter.h"

class Wire : public Component {
public:
    Wire(enum Wire_type wire_model, double len /* in u*/,
         int nsense = 1/* no. of sense amps connected to the low-swing wire */,
         double width_scaling = 1,
         double spacing_scaling = 1,
         enum Wire_placement wire_placement = outside_mat,
         double resistivity = CU_RESISTIVITY,
         TechnologyParameter::DeviceType *dt = &(g_tp.peri_global));
    ~Wire();

    Wire( double width_scaling = 1,
          double spacing_scaling = 1,
          enum Wire_placement wire_placement = outside_mat,
          double resistivity = CU_RESISTIVITY,
          TechnologyParameter::DeviceType *dt = &(g_tp.peri_global)
        ); // should be used only once for initializing static members
    void init_wire();

    void calculate_wire_stats();
    void delay_optimal_wire();
    double wire_cap(double len, bool call_from_outside = false);
    double wire_res(double len);
    void low_swing_model();
    double signal_fall_time();
    double signal_rise_time();
    double sense_amp_input_cap();

    enum Wire_type wt;
    double wire_spacing;
    double wire_width;
    enum Wire_placement wire_placement;
    double repeater_size;
    double repeater_spacing;
    double wire_length;
    double in_rise_time, out_rise_time;

    void set_in_rise_time(double rt) {
        in_rise_time = rt;
    }
    static Component global;
    static Component global_5;
    static Component global_10;
    static Component global_20;
    static Component global_30;
    static Component low_swing;
    static double wire_width_init;
    static double wire_spacing_init;
    void print_wire();

private:

    int nsense; // no. of sense amps connected to a low-swing wire if it
    // is broadcasting data to multiple destinations
    // width and spacing scaling factor can be used
    // to model low level wires or special
    // fat wires
    double w_scale, s_scale;
    double resistivity;
    powerDef wire_model (double space, double size, double *delay);
    list <Component> repeated_wire;
    void update_fullswing();
    static int initialized;


    //low-swing
    Component transmitter;
    Component l_wire;
    Component sense_amp;

    double min_w_pmos;

    TechnologyParameter::DeviceType *deviceType;

};

#endif
