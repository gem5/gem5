/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
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
 *          Tushar Krishna
 */


#ifndef __MEM_RUBY_NETWORK_GARNET_ROUTER_HH__
#define __MEM_RUBY_NETWORK_GARNET_ROUTER_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/flit.hh"
#include "params/GarnetRouter.hh"

class NetworkLink;
class CreditLink;
class InputUnit;
class OutputUnit;
class RoutingUnit;
class SwitchAllocator;
class CrossbarSwitch;
class FaultModel;

class Router : public BasicRouter, public Consumer
{
  public:
    typedef GarnetRouterParams Params;
    Router(const Params *p);

    ~Router();

    void wakeup();
    void print(std::ostream& out) const {};

    void init();
    void addInPort(PortDirection inport_dirn, NetworkLink *link,
                   CreditLink *credit_link);
    void addOutPort(PortDirection outport_dirn, NetworkLink *link,
                    const NetDest& routing_table_entry,
                    int link_weight, CreditLink *credit_link);

    Cycles get_pipe_stages(){ return m_latency; }
    int get_num_vcs()       { return m_num_vcs; }
    int get_num_vnets()     { return m_virtual_networks; }
    int get_vc_per_vnet()   { return m_vc_per_vnet; }
    int get_num_inports()   { return m_input_unit.size(); }
    int get_num_outports()  { return m_output_unit.size(); }
    int get_id()            { return m_id; }

    void init_net_ptr(GarnetNetwork* net_ptr)
    {
        m_network_ptr = net_ptr;
    }

    GarnetNetwork* get_net_ptr()                    { return m_network_ptr; }
    std::vector<InputUnit *>& get_inputUnit_ref()   { return m_input_unit; }
    std::vector<OutputUnit *>& get_outputUnit_ref() { return m_output_unit; }
    PortDirection getOutportDirection(int outport);
    PortDirection getInportDirection(int inport);

    int route_compute(RouteInfo route, int inport, PortDirection direction);
    void grant_switch(int inport, flit *t_flit);
    void schedule_wakeup(Cycles time);

    std::string getPortDirectionName(PortDirection direction);
    void printFaultVector(std::ostream& out);
    void printAggregateFaultProbability(std::ostream& out);

    void regStats();
    void collateStats();
    void resetStats();

    // For Fault Model:
    bool get_fault_vector(int temperature, float fault_vector[]) {
        return m_network_ptr->fault_model->fault_vector(m_id, temperature,
                                                        fault_vector);
    }
    bool get_aggregate_fault_probability(int temperature,
                                         float *aggregate_fault_prob) {
        return m_network_ptr->fault_model->fault_prob(m_id, temperature,
                                                      aggregate_fault_prob);
    }

    uint32_t functionalWrite(Packet *);

  private:
    Cycles m_latency;
    int m_virtual_networks, m_num_vcs, m_vc_per_vnet;
    GarnetNetwork *m_network_ptr;

    std::vector<InputUnit *> m_input_unit;
    std::vector<OutputUnit *> m_output_unit;
    RoutingUnit *m_routing_unit;
    SwitchAllocator *m_sw_alloc;
    CrossbarSwitch *m_switch;

    // Statistical variables required for power computations
    Stats::Scalar m_buffer_reads;
    Stats::Scalar m_buffer_writes;

    Stats::Scalar m_sw_input_arbiter_activity;
    Stats::Scalar m_sw_output_arbiter_activity;

    Stats::Scalar m_crossbar_activity;
};

#endif // __MEM_RUBY_NETWORK_GARNET_ROUTER_HH__
