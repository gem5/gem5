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


#include <cassert>
#include <iostream>

#include "globalvar.h"
#include "interconnect.h"
#include "wire.h"

interconnect::interconnect(
    string name_,
    enum Device_ty device_ty_,
        double base_w, double base_h,
    int data_w, double len,const InputParameter *configure_interface,
    int start_wiring_level_,
    bool pipelinable_ ,
    double route_over_perc_ ,
    bool opt_local_,
    enum Core_type core_ty_,
    enum Wire_type wire_model,
    double width_s, double space_s,
    TechnologyParameter::DeviceType *dt
)
 :name(name_),
  device_ty(device_ty_),
  in_rise_time(0),
  out_rise_time(0),
  base_width(base_w),
  base_height(base_h),
  data_width(data_w),
  wt(wire_model),
  width_scaling(width_s),
  space_scaling(space_s),
  start_wiring_level(start_wiring_level_),
  length(len),
  //interconnect_latency(1e-12),
  //interconnect_throughput(1e-12),
  opt_local(opt_local_),
  core_ty(core_ty_),
  pipelinable(pipelinable_),
  route_over_perc(route_over_perc_),
  deviceType(dt)
{

  wt = Global;
  l_ip=*configure_interface;
  local_result = init_interface(&l_ip);


  max_unpipelined_link_delay = 0; //TODO
  min_w_nmos = g_tp.min_w_nmos_;
  min_w_pmos = deviceType->n_to_p_eff_curr_drv_ratio * min_w_nmos;



  latency               = l_ip.latency;
  throughput            = l_ip.throughput;
  latency_overflow=false;
  throughput_overflow=false;

  /*
   * TODO: Add wiring option from semi-global to global automatically
   * And directly jump to global if semi-global cannot satisfy timing
   * Fat wires only available for global wires, thus
   * if signal wiring layer starts from semi-global,
   * the next layer up will be global, i.e., semi-global does
   * not have fat wires.
   */
  if (pipelinable == false)
  //Non-pipelinable wires, such as bypass logic, care latency
  {
          compute();
          if (opt_for_clk && opt_local)
          {
                  while (delay > latency && width_scaling<3.0)
                  {
                          width_scaling *= 2;
                          space_scaling *= 2;
                          Wire winit(width_scaling, space_scaling);
                          compute();
                  }
                  if (delay > latency)
                  {
                          latency_overflow=true;
                  }
          }
  }
  else //Pipelinable wires, such as bus, does not care latency but throughput
  {
          /*
           * TODO: Add pipe regs power, area, and timing;
           * Pipelinable wires optimize latency first.
           */
          compute();
          if (opt_for_clk && opt_local)
          {
                  while (delay > throughput && width_scaling<3.0)
                  {
                          width_scaling *= 2;
                          space_scaling *= 2;
                          Wire winit(width_scaling, space_scaling);
                          compute();
                  }
                  if (delay > throughput)
                          // insert pipeline stages
                  {
                          num_pipe_stages = (int)ceil(delay/throughput);
                          assert(num_pipe_stages>0);
                          delay = delay/num_pipe_stages + num_pipe_stages*0.05*delay;
                  }
          }
  }

  power_bit = power;
  power.readOp.dynamic *= data_width;
  power.readOp.leakage *= data_width;
  power.readOp.gate_leakage *= data_width;
  area.set_area(area.get_area()*data_width);
  no_device_under_wire_area.h *= data_width;

  if (latency_overflow==true)
                cout<< "Warning: "<< name <<" wire structure cannot satisfy latency constraint." << endl;


  assert(power.readOp.dynamic > 0);
  assert(power.readOp.leakage > 0);
  assert(power.readOp.gate_leakage > 0);

  double long_channel_device_reduction = longer_channel_device_reduction(device_ty,core_ty);

  double sckRation = g_tp.sckt_co_eff;
  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;

  power.readOp.longer_channel_leakage =
          power.readOp.leakage*long_channel_device_reduction;

  if (pipelinable)//Only global wires has the option to choose whether routing over or not
          area.set_area(area.get_area()*route_over_perc + no_device_under_wire_area.get_area()*(1-route_over_perc));

  Wire wreset();
}



void
interconnect::compute()
{

  Wire *wtemp1 = 0;
  wtemp1 = new Wire(wt, length, 1, width_scaling, space_scaling);
  delay = wtemp1->delay;
  power.readOp.dynamic = wtemp1->power.readOp.dynamic;
  power.readOp.leakage = wtemp1->power.readOp.leakage;
  power.readOp.gate_leakage = wtemp1->power.readOp.gate_leakage;

  area.set_area(wtemp1->area.get_area());
  no_device_under_wire_area.h =  (wtemp1->wire_width + wtemp1->wire_spacing);
  no_device_under_wire_area.w = length;

  if (wtemp1)
   delete wtemp1;

}

void interconnect::leakage_feedback(double temperature)
{
  l_ip.temp = (unsigned int)round(temperature/10.0)*10;
  uca_org_t init_result = init_interface(&l_ip); // init_result is dummy

  compute();

  power_bit = power;
  power.readOp.dynamic *= data_width;
  power.readOp.leakage *= data_width;
  power.readOp.gate_leakage *= data_width;

  assert(power.readOp.dynamic > 0);
  assert(power.readOp.leakage > 0);
  assert(power.readOp.gate_leakage > 0);

  double long_channel_device_reduction = longer_channel_device_reduction(device_ty,core_ty);

  double sckRation = g_tp.sckt_co_eff;
  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;

  power.readOp.longer_channel_leakage = power.readOp.leakage*long_channel_device_reduction;
}

