/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/


#ifndef __INTERCONNECT_H__
#define __INTERCONNECT_H__

#include "assert.h"
#include "basic_circuit.h"
#include "basic_components.h"
#include "cacti_interface.h"
#include "component.h"
#include "parameter.h"
#include "subarray.h"
#include "wire.h"

// leakge power includes entire htree in a bank (when uca_tree == false)
// leakge power includes only part to one bank when uca_tree == true

class interconnect : public Component
{
  public:
    interconnect(
        string  name_,
        enum Device_ty device_ty_,
        double base_w, double base_h, int data_w, double len,
        const InputParameter *configure_interface, int start_wiring_level_,
        bool pipelinable_ = false,
        double route_over_perc_ =0.5,
        bool opt_local_=true,
        enum Core_type core_ty_=Inorder,
        enum Wire_type wire_model=Global,
        double width_s=1.0, double space_s=1.0,
        TechnologyParameter::DeviceType *dt = &(g_tp.peri_global)
                );

    ~interconnect() {};

    void compute();
        string   name;
        enum Device_ty device_ty;
    double in_rise_time, out_rise_time;
        InputParameter l_ip;
        uca_org_t local_result;
    Area no_device_under_wire_area;
    void set_in_rise_time(double rt)
    {
      in_rise_time = rt;
    }

    void leakage_feedback(double temperature);
    double max_unpipelined_link_delay;
    powerDef power_bit;

    double wire_bw;
    double init_wire_bw;  // bus width at root
    double base_width;
    double base_height;
    int data_width;
    enum Wire_type wt;
    double width_scaling, space_scaling;
    int start_wiring_level;
    double length;
    double min_w_nmos;
    double min_w_pmos;
    double latency, throughput;
    bool  latency_overflow;
    bool  throughput_overflow;
    double  interconnect_latency;
    double  interconnect_throughput;
    bool opt_local;
    enum Core_type core_ty;
    bool pipelinable;
    double route_over_perc;
    int  num_pipe_stages;

  private:
    TechnologyParameter::DeviceType *deviceType;

};

#endif

