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


#include "mem/ruby/network/garnet2.0/Router.hh"

#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/network/garnet2.0/CrossbarSwitch.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"
#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/network/garnet2.0/OutputUnit.hh"
#include "mem/ruby/network/garnet2.0/RoutingUnit.hh"
#include "mem/ruby/network/garnet2.0/SwitchAllocator.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

Router::Router(const Params *p)
    : BasicRouter(p), Consumer(this)
{
    m_latency = p->latency;
    m_virtual_networks = p->virt_nets;
    m_vc_per_vnet = p->vcs_per_vnet;
    m_num_vcs = m_virtual_networks * m_vc_per_vnet;

    m_routing_unit = new RoutingUnit(this);
    m_sw_alloc = new SwitchAllocator(this);
    m_switch = new CrossbarSwitch(this);

    m_input_unit.clear();
    m_output_unit.clear();
}

Router::~Router()
{
    deletePointers(m_input_unit);
    deletePointers(m_output_unit);
    delete m_routing_unit;
    delete m_sw_alloc;
    delete m_switch;
}

void
Router::init()
{
    BasicRouter::init();

    m_sw_alloc->init();
    m_switch->init();
}

void
Router::wakeup()
{
    DPRINTF(RubyNetwork, "Router %d woke up\n", m_id);

    // check for incoming flits
    for (int inport = 0; inport < m_input_unit.size(); inport++) {
        m_input_unit[inport]->wakeup();
    }

    // check for incoming credits
    // Note: the credit update is happening before SA
    // buffer turnaround time =
    //     credit traversal (1-cycle) + SA (1-cycle) + Link Traversal (1-cycle)
    // if we want the credit update to take place after SA, this loop should
    // be moved after the SA request
    for (int outport = 0; outport < m_output_unit.size(); outport++) {
        m_output_unit[outport]->wakeup();
    }

    // Switch Allocation
    m_sw_alloc->wakeup();

    // Switch Traversal
    m_switch->wakeup();
}

void
Router::addInPort(PortDirection inport_dirn,
                  NetworkLink *in_link, CreditLink *credit_link)
{
    int port_num = m_input_unit.size();
    InputUnit *input_unit = new InputUnit(port_num, inport_dirn, this);

    input_unit->set_in_link(in_link);
    input_unit->set_credit_link(credit_link);
    in_link->setLinkConsumer(this);
    credit_link->setSourceQueue(input_unit->getCreditQueue());

    m_input_unit.push_back(input_unit);

    m_routing_unit->addInDirection(inport_dirn, port_num);
}

void
Router::addOutPort(PortDirection outport_dirn,
                   NetworkLink *out_link,
                   const NetDest& routing_table_entry, int link_weight,
                   CreditLink *credit_link)
{
    int port_num = m_output_unit.size();
    OutputUnit *output_unit = new OutputUnit(port_num, outport_dirn, this);

    output_unit->set_out_link(out_link);
    output_unit->set_credit_link(credit_link);
    credit_link->setLinkConsumer(this);
    out_link->setSourceQueue(output_unit->getOutQueue());

    m_output_unit.push_back(output_unit);

    m_routing_unit->addRoute(routing_table_entry);
    m_routing_unit->addWeight(link_weight);
    m_routing_unit->addOutDirection(outport_dirn, port_num);
}

PortDirection
Router::getOutportDirection(int outport)
{
    return m_output_unit[outport]->get_direction();
}

PortDirection
Router::getInportDirection(int inport)
{
    return m_input_unit[inport]->get_direction();
}

int
Router::route_compute(RouteInfo route, int inport, PortDirection inport_dirn)
{
    return m_routing_unit->outportCompute(route, inport, inport_dirn);
}

void
Router::grant_switch(int inport, flit *t_flit)
{
    m_switch->update_sw_winner(inport, t_flit);
}

void
Router::schedule_wakeup(Cycles time)
{
    // wake up after time cycles
    scheduleEvent(time);
}

std::string
Router::getPortDirectionName(PortDirection direction)
{
    // PortDirection is actually a string
    // If not, then this function should add a switch
    // statement to convert direction to a string
    // that can be printed out
    return direction;
}

void
Router::regStats()
{
    BasicRouter::regStats();

    m_buffer_reads
        .name(name() + ".buffer_reads")
        .flags(Stats::nozero)
    ;

    m_buffer_writes
        .name(name() + ".buffer_writes")
        .flags(Stats::nozero)
    ;

    m_crossbar_activity
        .name(name() + ".crossbar_activity")
        .flags(Stats::nozero)
    ;

    m_sw_input_arbiter_activity
        .name(name() + ".sw_input_arbiter_activity")
        .flags(Stats::nozero)
    ;

    m_sw_output_arbiter_activity
        .name(name() + ".sw_output_arbiter_activity")
        .flags(Stats::nozero)
    ;
}

void
Router::collateStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_buffer_reads += m_input_unit[i]->get_buf_read_activity(j);
            m_buffer_writes += m_input_unit[i]->get_buf_write_activity(j);
        }
    }

    m_sw_input_arbiter_activity = m_sw_alloc->get_input_arbiter_activity();
    m_sw_output_arbiter_activity = m_sw_alloc->get_output_arbiter_activity();
    m_crossbar_activity = m_switch->get_crossbar_activity();
}

void
Router::resetStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_input_unit[i]->resetStats();
        }
    }
}

void
Router::printFaultVector(ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    int num_fault_types = m_network_ptr->fault_model->number_of_fault_types;
    float fault_vector[num_fault_types];
    get_fault_vector(temperature_celcius, fault_vector);
    out << "Router-" << m_id << " fault vector: " << endl;
    for (int fault_type_index = 0; fault_type_index < num_fault_types;
         fault_type_index++) {
        out << " - probability of (";
        out <<
        m_network_ptr->fault_model->fault_type_to_string(fault_type_index);
        out << ") = ";
        out << fault_vector[fault_type_index] << endl;
    }
}

void
Router::printAggregateFaultProbability(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    float aggregate_fault_prob;
    get_aggregate_fault_probability(temperature_celcius,
                                    &aggregate_fault_prob);
    out << "Router-" << m_id << " fault probability: ";
    out << aggregate_fault_prob << endl;
}

uint32_t
Router::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    num_functional_writes += m_switch->functionalWrite(pkt);

    for (uint32_t i = 0; i < m_input_unit.size(); i++) {
        num_functional_writes += m_input_unit[i]->functionalWrite(pkt);
    }

    for (uint32_t i = 0; i < m_output_unit.size(); i++) {
        num_functional_writes += m_output_unit[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

Router *
GarnetRouterParams::create()
{
    return new Router(this);
}
