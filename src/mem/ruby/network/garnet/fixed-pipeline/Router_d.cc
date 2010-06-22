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
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/InputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/OutputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/RoutingUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/VCallocator_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/SWallocator_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Switch_d.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

Router_d::Router_d(int id, GarnetNetwork_d *network_ptr)
{
    m_id = id;
    m_network_ptr = network_ptr;
    m_virtual_networks = network_ptr->getNumberOfVirtualNetworks();
    m_vc_per_vnet = m_network_ptr->getVCsPerClass();
    m_num_vcs = m_virtual_networks*m_vc_per_vnet;
    m_flit_width = m_network_ptr->getFlitSize();

    m_routing_unit = new RoutingUnit_d(this);
    m_vc_alloc = new VCallocator_d(this);
    m_sw_alloc = new SWallocator_d(this);
    m_switch = new Switch_d(this);

    m_input_unit.clear();
    m_output_unit.clear();

    buf_read_count = 0;
    buf_write_count = 0;
    crossbar_count = 0;
    vc_local_arbit_count = 0;
    vc_global_arbit_count = 0;
    sw_local_arbit_count = 0;
    sw_global_arbit_count = 0;
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
    g_eventQueue_ptr->scheduleEvent(m_vc_alloc, 1);
}

void
Router_d::swarb_req()
{
    g_eventQueue_ptr->scheduleEvent(m_sw_alloc, 1);
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
    g_eventQueue_ptr->scheduleEvent(m_switch, 1);
}

void
Router_d::calculate_performance_numbers()
{
    for (int i = 0; i < m_input_unit.size(); i++) {
        buf_read_count += m_input_unit[i]->get_buf_read_count();
        buf_write_count += m_input_unit[i]->get_buf_write_count();
    }
    crossbar_count = m_switch->get_crossbar_count();
    vc_local_arbit_count = m_vc_alloc->get_local_arbit_count();
    vc_global_arbit_count = m_vc_alloc->get_global_arbit_count();
    sw_local_arbit_count = m_sw_alloc->get_local_arbit_count();
    sw_global_arbit_count = m_sw_alloc->get_global_arbit_count();
}

void
Router_d::printConfig(ostream& out)
{
    out << "[Router " << m_id << "] :: " << endl;
    out << "[inLink - ";
    for (int i = 0;i < m_input_unit.size(); i++)
        out << m_input_unit[i]->get_inlink_id() << " - ";
    out << "]" << endl;
    out << "[outLink - ";
    for (int i = 0;i < m_output_unit.size(); i++)
        out << m_output_unit[i]->get_outlink_id() << " - ";
    out << "]" << endl;
}
