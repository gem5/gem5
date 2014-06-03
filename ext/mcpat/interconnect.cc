/*****************************************************************************
 *                                McPAT
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


#include <cassert>
#include <iostream>

#include "basic_components.h"
#include "interconnect.h"
#include "wire.h"

double Interconnect::width_scaling_threshold = 3.0;

Interconnect::Interconnect(XMLNode* _xml_data, string name_,
                           enum Device_ty device_ty_, double base_w,
                           double base_h, int data_w,
                           double len,
                           const InputParameter *configure_interface,
                           int start_wiring_level_, double _clockRate,
                           bool pipelinable_, double route_over_perc_,
                           bool opt_local_, enum Core_type core_ty_,
                           enum Wire_type wire_model,
                           double width_s, double space_s,
                           TechnologyParameter::DeviceType *dt)
    : McPATComponent(_xml_data), device_ty(device_ty_), in_rise_time(0),
      out_rise_time(0), base_width(base_w), base_height(base_h),
      data_width(data_w), wt(wire_model), width_scaling(width_s),
      space_scaling(space_s), start_wiring_level(start_wiring_level_),
      length(len), opt_local(opt_local_), core_ty(core_ty_),
      pipelinable(pipelinable_), route_over_perc(route_over_perc_),
      deviceType(dt) {
    name = name_;
    clockRate = _clockRate;
    l_ip = *configure_interface;
    local_result = init_interface(&l_ip, name);

    max_unpipelined_link_delay = 0;
    min_w_nmos = g_tp.min_w_nmos_;
    min_w_pmos = deviceType->n_to_p_eff_curr_drv_ratio * min_w_nmos;



    latency               = l_ip.latency;
    throughput            = l_ip.throughput;
    latency_overflow = false;
    throughput_overflow = false;

    if (pipelinable == false) {
        //Non-pipelinable wires, such as bypass logic, care latency
        calcWireData();
        if (opt_for_clk && opt_local) {
            while (delay > latency &&
                   width_scaling < width_scaling_threshold) {
                width_scaling *= 2;
                space_scaling *= 2;
                Wire winit(width_scaling, space_scaling);
                calcWireData();
            }
            if (delay > latency) {
                latency_overflow = true;
            }
        }
    } else {
        //Pipelinable wires, such as bus, does not care latency but throughput
        calcWireData();
        if (opt_for_clk && opt_local) {
            while (delay > throughput &&
                   width_scaling < width_scaling_threshold) {
                width_scaling *= 2;
                space_scaling *= 2;
                Wire winit(width_scaling, space_scaling);
                calcWireData();
            }
            if (delay > throughput) {
                // insert pipeline stages
                num_pipe_stages = (int)ceil(delay / throughput);
                assert(num_pipe_stages > 0);
                delay = delay / num_pipe_stages + num_pipe_stages * 0.05 * delay;
            }
        }
    }

    power_bit = power;
    power.readOp.dynamic *= data_width;
    power.readOp.leakage *= data_width;
    power.readOp.gate_leakage *= data_width;
    area.set_area(area.get_area()*data_width);
    no_device_under_wire_area.h *= data_width;

    if (latency_overflow == true) {
        cout << "Warning: " << name
             << " wire structure cannot satisfy latency constraint." << endl;
    }

    assert(power.readOp.dynamic > 0);
    assert(power.readOp.leakage > 0);
    assert(power.readOp.gate_leakage > 0);

    double long_channel_device_reduction =
        longer_channel_device_reduction(device_ty, core_ty);

    double sckRation = g_tp.sckt_co_eff;
    power.readOp.dynamic *= sckRation;
    power.writeOp.dynamic *= sckRation;
    power.searchOp.dynamic *= sckRation;

    power.readOp.longer_channel_leakage =
        power.readOp.leakage * long_channel_device_reduction;

    //Only global wires has the option to choose whether routing over or not
    if (pipelinable)
        area.set_area(area.get_area() * route_over_perc +
                      no_device_under_wire_area.get_area() *
                      (1 - route_over_perc));

    Wire wreset();
}



void
Interconnect::calcWireData() {

    Wire *wtemp1 = 0;
    wtemp1 = new Wire(wt, length, 1, width_scaling, space_scaling);
    delay = wtemp1->delay;
    power.readOp.dynamic = wtemp1->power.readOp.dynamic;
    power.readOp.leakage = wtemp1->power.readOp.leakage;
    power.readOp.gate_leakage = wtemp1->power.readOp.gate_leakage;

    area.set_area(wtemp1->area.get_area());
    no_device_under_wire_area.h = (wtemp1->wire_width + wtemp1->wire_spacing);
    no_device_under_wire_area.w = length;

    if (wtemp1)
        delete wtemp1;

}

void
Interconnect::computeEnergy() {
    double pppm_t[4] = {1, 1, 1, 1};

    // Compute TDP
    power_t.reset();
    set_pppm(pppm_t, int_params.active_ports * int_stats.duty_cycle,
            int_params.active_ports, int_params.active_ports,
            int_params.active_ports * int_stats.duty_cycle);
    power_t = power * pppm_t;

    rt_power.reset();
    set_pppm(pppm_t, int_stats.accesses, int_params.active_ports,
             int_params.active_ports, int_stats.accesses);
    rt_power = power * pppm_t;

    output_data.peak_dynamic_power = power_t.readOp.dynamic * clockRate;
    output_data.subthreshold_leakage_power = power_t.readOp.leakage;
    output_data.gate_leakage_power = power_t.readOp.gate_leakage;
    output_data.runtime_dynamic_energy = rt_power.readOp.dynamic;
}

void
Interconnect::computeArea() {
    output_data.area = area.get_area() / 1e6;
}

void
Interconnect::set_params_stats(double active_ports,
                               double duty_cycle, double accesses) {
    int_params.active_ports = active_ports;
    int_stats.duty_cycle = duty_cycle;
    int_stats.accesses = accesses;
}

void Interconnect::leakage_feedback(double temperature) {
  l_ip.temp = (unsigned int)round(temperature/10.0)*10;
  uca_org_t init_result = init_interface(&l_ip, name); // init_result is dummy

  calcWireData();

  power_bit = power;
  power.readOp.dynamic *= data_width;
  power.readOp.leakage *= data_width;
  power.readOp.gate_leakage *= data_width;

  assert(power.readOp.dynamic > 0);
  assert(power.readOp.leakage > 0);
  assert(power.readOp.gate_leakage > 0);

  double long_channel_device_reduction =
      longer_channel_device_reduction(device_ty,core_ty);

  double sckRation = g_tp.sckt_co_eff;
  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;

  power.readOp.longer_channel_leakage =
      power.readOp.leakage*long_channel_device_reduction;
}

