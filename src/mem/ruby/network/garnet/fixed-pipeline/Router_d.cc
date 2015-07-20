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

#include "base/stl_helpers.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/InputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/OutputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/RoutingUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SWallocator_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Switch_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/VCallocator_d.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

Router_d::Router_d(const Params *p)
    : BasicRouter(p)
{
    m_virtual_networks = p->virt_nets;
    m_vc_per_vnet = p->vcs_per_vnet;
    m_num_vcs = m_virtual_networks * m_vc_per_vnet;

    m_routing_unit = new RoutingUnit_d(this);
    m_vc_alloc = new VCallocator_d(this);
    m_sw_alloc = new SWallocator_d(this);
    m_switch = new Switch_d(this);

    m_input_unit.clear();
    m_output_unit.clear();
}

Router_d::~Router_d()
{
    deletePointers(m_input_unit);
    deletePointers(m_output_unit);
    delete m_routing_unit;
    delete m_vc_alloc;
    delete m_sw_alloc;
    delete m_switch;
}

void
Router_d::init()
{
    BasicRouter::init();

    m_vc_alloc->init();
    m_sw_alloc->init();
    m_switch->init();
}

void
Router_d::addInPort(NetworkLink_d *in_link, CreditLink_d *credit_link)
{
    int port_num = m_input_unit.size();
    InputUnit_d *input_unit = new InputUnit_d(port_num, this);

    input_unit->set_in_link(in_link);
    input_unit->set_credit_link(credit_link);
    in_link->setLinkConsumer(input_unit);
    credit_link->setSourceQueue(input_unit->getCreditQueue());

    m_input_unit.push_back(input_unit);
}

void
Router_d::addOutPort(NetworkLink_d *out_link,
    const NetDest& routing_table_entry, int link_weight,
    CreditLink_d *credit_link)
{
    int port_num = m_output_unit.size();
    OutputUnit_d *output_unit = new OutputUnit_d(port_num, this);

    output_unit->set_out_link(out_link);
    output_unit->set_credit_link(credit_link);
    credit_link->setLinkConsumer(output_unit);
    out_link->setSourceQueue(output_unit->getOutQueue());

    m_output_unit.push_back(output_unit);

    m_routing_unit->addRoute(routing_table_entry);
    m_routing_unit->addWeight(link_weight);
}

void
Router_d::route_req(flit_d *t_flit, InputUnit_d *in_unit, int invc)
{
    m_routing_unit->RC_stage(t_flit, in_unit, invc);
}

void
Router_d::vcarb_req()
{
    m_vc_alloc->scheduleEventAbsolute(clockEdge(Cycles(1)));
}

void
Router_d::swarb_req()
{
    m_sw_alloc->scheduleEventAbsolute(clockEdge(Cycles(1)));
}

void
Router_d::call_sw_alloc()
{
    m_sw_alloc->wakeup();
}

void
Router_d::call_switch()
{
    m_switch->wakeup();
}

void
Router_d::update_incredit(int in_port, int in_vc, int credit)
{
    m_input_unit[in_port]->update_credit(in_vc, credit);
}

void
Router_d::update_sw_winner(int inport, flit_d *t_flit)
{
    m_switch->update_sw_winner(inport, t_flit);
    m_switch->scheduleEventAbsolute(clockEdge(Cycles(1)));
}

void
Router_d::regStats()
{
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

    m_sw_local_arbiter_activity
        .name(name() + ".sw_local_arbiter_activity")
        .flags(Stats::nozero)
    ;

    m_sw_global_arbiter_activity
        .name(name() + ".sw_global_arbiter_activity")
        .flags(Stats::nozero)
    ;

    m_vc_local_arbiter_activity
        .name(name() + ".vc_local_arbiter_activity")
        .flags(Stats::nozero)
    ;

    m_vc_global_arbiter_activity
        .name(name() + ".vc_global_arbiter_activity")
        .flags(Stats::nozero)
    ;
}

void
Router_d::collateStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_buffer_reads += m_input_unit[i]->get_buf_read_count(j);
            m_buffer_writes += m_input_unit[i]->get_buf_write_count(j);
        }

        m_vc_local_arbiter_activity  += m_vc_alloc->get_local_arbit_count(j);
        m_vc_global_arbiter_activity += m_vc_alloc->get_global_arbit_count(j);
    }

    m_sw_local_arbiter_activity = m_sw_alloc->get_local_arbit_count();
    m_sw_global_arbiter_activity = m_sw_alloc->get_global_arbit_count();
    m_crossbar_activity = m_switch->get_crossbar_count();
}

void
Router_d::resetStats()
{
    for (int j = 0; j < m_virtual_networks; j++) {
        for (int i = 0; i < m_input_unit.size(); i++) {
            m_input_unit[i]->resetStats();
        }
    }
}

void
Router_d::printFaultVector(ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    int num_fault_types = m_network_ptr->fault_model->number_of_fault_types;
    float fault_vector[num_fault_types];
    get_fault_vector(temperature_celcius, fault_vector);
    out << "Router-" << m_id << " fault vector: " << endl;
    for (int fault_type_index = 0; fault_type_index < num_fault_types;
         fault_type_index++){
        out << " - probability of (";
        out <<
        m_network_ptr->fault_model->fault_type_to_string(fault_type_index);
        out << ") = ";
        out << fault_vector[fault_type_index] << endl;
    }
}

void
Router_d::printAggregateFaultProbability(std::ostream& out)
{
    int temperature_celcius = BASELINE_TEMPERATURE_CELCIUS;
    float aggregate_fault_prob;
    get_aggregate_fault_probability(temperature_celcius,
                                    &aggregate_fault_prob);
    out << "Router-" << m_id << " fault probability: ";
    out << aggregate_fault_prob << endl;
}

uint32_t
Router_d::functionalWrite(Packet *pkt)
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

Router_d *
GarnetRouter_dParams::create()
{
    return new Router_d(this);
}
