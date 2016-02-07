/*
 * Copyright (c) 2008 Princeton University
 * All rights reserved.
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
 *
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
 * Authors: Niket Agarwal
 */

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTER_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTER_D_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/flit_d.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "params/GarnetRouter_d.hh"

class NetworkLink_d;
class CreditLink_d;
class InputUnit_d;
class OutputUnit_d;
class RoutingUnit_d;
class VCallocator_d;
class SWallocator_d;
class Switch_d;
class FaultModel;

class Router_d : public BasicRouter
{
  public:
    typedef GarnetRouter_dParams Params;
    Router_d(const Params *p);

    ~Router_d();

    void init();
    void addInPort(NetworkLink_d *link, CreditLink_d *credit_link);
    void addOutPort(NetworkLink_d *link, const NetDest& routing_table_entry,
                    int link_weight, CreditLink_d *credit_link);

    int get_num_vcs()       { return m_num_vcs; }
    int get_num_vnets()     { return m_virtual_networks; }
    int get_vc_per_vnet()   { return m_vc_per_vnet; }
    int get_num_inports()   { return m_input_unit.size(); }
    int get_num_outports()  { return m_output_unit.size(); }
    int get_id()            { return m_id; }

    void init_net_ptr(GarnetNetwork_d* net_ptr)
    {
        m_network_ptr = net_ptr;
    }

    GarnetNetwork_d* get_net_ptr()                  { return m_network_ptr; }
    std::vector<InputUnit_d *>& get_inputUnit_ref()   { return m_input_unit; }
    std::vector<OutputUnit_d *>& get_outputUnit_ref() { return m_output_unit; }

    void update_sw_winner(int inport, flit_d *t_flit);
    void update_incredit(int in_port, int in_vc, int credit);
    void route_req(flit_d *t_flit, InputUnit_d* in_unit, int invc);
    void vcarb_req();
    void swarb_req();
    void call_sw_alloc();
    void call_switch();

    void printFaultVector(std::ostream& out);
    void printAggregateFaultProbability(std::ostream& out);

    void regStats();
    void collateStats();
    void resetStats();

    bool get_fault_vector(int temperature, float fault_vector[]){
        return m_network_ptr->fault_model->fault_vector(m_id, temperature,
                                                        fault_vector);
    }
    bool get_aggregate_fault_probability(int temperature,
                                         float *aggregate_fault_prob){
        return m_network_ptr->fault_model->fault_prob(m_id, temperature,
                                                      aggregate_fault_prob);
    }

    uint32_t functionalWrite(Packet *);

  private:
    int m_virtual_networks, m_num_vcs, m_vc_per_vnet;
    GarnetNetwork_d *m_network_ptr;

    std::vector<InputUnit_d *> m_input_unit;
    std::vector<OutputUnit_d *> m_output_unit;
    RoutingUnit_d *m_routing_unit;
    VCallocator_d *m_vc_alloc;
    SWallocator_d *m_sw_alloc;
    Switch_d *m_switch;

    // Statistical variables required for power computations
    Stats::Scalar m_buffer_reads;
    Stats::Scalar m_buffer_writes;

    Stats::Scalar m_sw_local_arbiter_activity;
    Stats::Scalar m_sw_global_arbiter_activity;

    Stats::Scalar m_vc_local_arbiter_activity;
    Stats::Scalar m_vc_global_arbiter_activity;

    Stats::Scalar m_crossbar_activity;
};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTER_D_HH__
