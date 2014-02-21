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

#include <cassert>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkInterface_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

GarnetNetwork_d::GarnetNetwork_d(const Params *p)
    : BaseGarnetNetwork(p)
{
    m_buffers_per_data_vc = p->buffers_per_data_vc;
    m_buffers_per_ctrl_vc = p->buffers_per_ctrl_vc;

    m_vnet_type.resize(m_virtual_networks);
    for (int i = 0; i < m_vnet_type.size(); i++) {
        m_vnet_type[i] = NULL_VNET_; // default
    }

    // record the routers
    for (vector<BasicRouter*>::const_iterator i =  p->routers.begin();
         i != p->routers.end(); ++i) {
        Router_d* router = safe_cast<Router_d*>(*i);
        m_routers.push_back(router);
    }
}

void
GarnetNetwork_d::init()
{
    BaseGarnetNetwork::init();

    // initialize the router's network pointers
    for (vector<Router_d*>::const_iterator i = m_routers.begin();
         i != m_routers.end(); ++i) {
        Router_d* router = safe_cast<Router_d*>(*i);
        router->init_net_ptr(this);
    }

    // The topology pointer should have already been initialized in the
    // parent network constructor
    assert(m_topology_ptr != NULL);

    for (int i=0; i < m_nodes; i++) {
        NetworkInterface_d *ni = new NetworkInterface_d(i, m_virtual_networks,
                                                        this);
        ni->addNode(m_toNetQueues[i], m_fromNetQueues[i]);
        m_nis.push_back(ni);
    }
    m_topology_ptr->createLinks(this);

    // FaultModel: declare each router to the fault model
    if(isFaultModelEnabled()){
        for (vector<Router_d*>::const_iterator i= m_routers.begin();
             i != m_routers.end(); ++i) {
            Router_d* router = safe_cast<Router_d*>(*i);
            int router_id M5_VAR_USED =
                fault_model->declare_router(router->get_num_inports(),
                                            router->get_num_outports(),
                                            router->get_vc_per_vnet(),
                                            getBuffersPerDataVC(),
                                            getBuffersPerCtrlVC());
            assert(router_id == router->get_id());
            router->printAggregateFaultProbability(cout);
            router->printFaultVector(cout);
        }
    }
}

GarnetNetwork_d::~GarnetNetwork_d()
{
    for (int i = 0; i < m_nodes; i++) {
        deletePointers(m_toNetQueues[i]);
        deletePointers(m_fromNetQueues[i]);
    }
    deletePointers(m_routers);
    deletePointers(m_nis);
    deletePointers(m_links);
    deletePointers(m_creditlinks);
    delete m_topology_ptr;
}

/*
 * This function creates a link from the Network Interface (NI)
 * into the Network.
 * It creates a Network Link from the NI to a Router and a Credit Link from
 * the Router to the NI
*/

void
GarnetNetwork_d::makeInLink(NodeID src, SwitchID dest, BasicLink* link,
                            LinkDirection direction,
                            const NetDest& routing_table_entry)
{
    assert(src < m_nodes);

    GarnetExtLink_d* garnet_link = safe_cast<GarnetExtLink_d*>(link);
    NetworkLink_d* net_link = garnet_link->m_network_links[direction];
    CreditLink_d* credit_link = garnet_link->m_credit_links[direction];

    m_links.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    m_routers[dest]->addInPort(net_link, credit_link);
    m_nis[src]->addOutPort(net_link, credit_link);
}

/*
 * This function creates a link from the Network to a NI.
 * It creates a Network Link from a Router to the NI and
 * a Credit Link from NI to the Router
*/

void
GarnetNetwork_d::makeOutLink(SwitchID src, NodeID dest, BasicLink* link,
                             LinkDirection direction,
                             const NetDest& routing_table_entry)
{
    assert(dest < m_nodes);
    assert(src < m_routers.size());
    assert(m_routers[src] != NULL);

    GarnetExtLink_d* garnet_link = safe_cast<GarnetExtLink_d*>(link);
    NetworkLink_d* net_link = garnet_link->m_network_links[direction];
    CreditLink_d* credit_link = garnet_link->m_credit_links[direction];

    m_links.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    m_routers[src]->addOutPort(net_link, routing_table_entry,
                                         link->m_weight, credit_link);
    m_nis[dest]->addInPort(net_link, credit_link);
}

/*
 * This function creates an internal network link
*/

void
GarnetNetwork_d::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                  LinkDirection direction,
                                  const NetDest& routing_table_entry)
{
    GarnetIntLink_d* garnet_link = safe_cast<GarnetIntLink_d*>(link);
    NetworkLink_d* net_link = garnet_link->m_network_links[direction];
    CreditLink_d* credit_link = garnet_link->m_credit_links[direction];

    m_links.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    m_routers[dest]->addInPort(net_link, credit_link);
    m_routers[src]->addOutPort(net_link, routing_table_entry,
                                         link->m_weight, credit_link);
}

void
GarnetNetwork_d::checkNetworkAllocation(NodeID id, bool ordered,
                                        int network_num,
                                        string vnet_type)
{
    assert(id < m_nodes);
    assert(network_num < m_virtual_networks);

    if (ordered) {
        m_ordered[network_num] = true;
    }
    m_in_use[network_num] = true;

    if (vnet_type == "response")
        m_vnet_type[network_num] = DATA_VNET_; // carries data (and ctrl) packets
    else
        m_vnet_type[network_num] = CTRL_VNET_; // carries only ctrl packets
}

void
GarnetNetwork_d::regStats()
{
    BaseGarnetNetwork::regStats();
    regLinkStats();
    regPowerStats();
}

void
GarnetNetwork_d::regLinkStats()
{
    m_average_link_utilization.name(name() + ".avg_link_utilization");

    m_average_vc_load
        .init(m_virtual_networks * m_vcs_per_vnet)
        .name(name() + ".avg_vc_load")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;
}

void
GarnetNetwork_d::regPowerStats()
{
    m_dynamic_link_power.name(name() + ".link_dynamic_power");
    m_static_link_power.name(name() + ".link_static_power");

    m_total_link_power.name(name() + ".link_total_power");
    m_total_link_power = m_dynamic_link_power + m_static_link_power;

    m_dynamic_router_power.name(name() + ".router_dynamic_power");
    m_static_router_power.name(name() + ".router_static_power");
    m_clk_power.name(name() + ".clk_power");

    m_total_router_power.name(name() + ".router_total_power");
    m_total_router_power = m_dynamic_router_power +
                           m_static_router_power +
                           m_clk_power;
}

void
GarnetNetwork_d::collateStats()
{
    collateLinkStats();
    collatePowerStats();
}

void
GarnetNetwork_d::collateLinkStats()
{
    for (int i = 0; i < m_links.size(); i++) {
        m_average_link_utilization +=
            (double(m_links[i]->getLinkUtilization())) /
            (double(curCycle() - g_ruby_start));

        vector<unsigned int> vc_load = m_links[i]->getVcLoad();
        for (int j = 0; j < vc_load.size(); j++) {
            m_average_vc_load[j] +=
                ((double)vc_load[j] / (double)(curCycle() - g_ruby_start));
        }
    }
}

void
GarnetNetwork_d::collatePowerStats()
{
    double sim_cycles = (double)(curCycle() - g_ruby_start);
    for (int i = 0; i < m_links.size(); i++) {
        m_links[i]->calculate_power(sim_cycles);
        m_dynamic_link_power += m_links[i]->get_dynamic_power();
        m_static_link_power += m_links[i]->get_static_power();
    }

    for (int i = 0; i < m_routers.size(); i++) {
        m_routers[i]->calculate_power();
        m_dynamic_router_power += m_routers[i]->get_dynamic_power();
        m_static_router_power += m_routers[i]->get_static_power();
        m_clk_power += m_routers[i]->get_clk_power();
    }
}

void
GarnetNetwork_d::print(ostream& out) const
{
    out << "[GarnetNetwork_d]";
}

GarnetNetwork_d *
GarnetNetwork_dParams::create()
{
    return new GarnetNetwork_d(this);
}

uint32_t
GarnetNetwork_d::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (unsigned int i = 0; i < m_routers.size(); i++) {
        num_functional_writes += m_routers[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_nis.size(); ++i) {
        num_functional_writes += m_nis[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_links.size(); ++i) {
        num_functional_writes += m_links[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}
