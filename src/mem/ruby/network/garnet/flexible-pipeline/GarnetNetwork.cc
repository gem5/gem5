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
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/GarnetLink.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/NetworkInterface.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/NetworkLink.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/Router.hh"
#include "mem/ruby/system/RubySystem.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

GarnetNetwork::GarnetNetwork(const Params *p)
    : BaseGarnetNetwork(p)
{
    m_buffer_size = p->buffer_size;
    m_number_of_pipe_stages = p->number_of_pipe_stages;

    // record the routers
    for (vector<BasicRouter*>::const_iterator i = p->routers.begin();
         i != p->routers.end(); ++i) {
        Router* router = safe_cast<Router*>(*i);
        m_routers.push_back(router);
    }

    for (int i=0; i < m_nodes; i++) {
        NetworkInterface *ni = safe_cast<NetworkInterface *>(p->netifs[i]);
        m_nis.push_back(ni);
    }
}

void
GarnetNetwork::init()
{
    BaseGarnetNetwork::init();

    // Setup the network switches
    assert (m_topology_ptr!=NULL);

    // initialize the router's network pointers
    for (vector<Router*>::const_iterator i = m_routers.begin();
         i != m_routers.end(); ++i) {
        Router* router = safe_cast<Router*>(*i);
        router->init_net_ptr(this);
    }

    for (int i=0; i < m_nodes; i++) {
        m_nis[i]->init_net_ptr(this);
        m_nis[i]->addNode(m_toNetQueues[i], m_fromNetQueues[i]);
    }

    m_topology_ptr->createLinks(this);
}

GarnetNetwork::~GarnetNetwork()
{
    deletePointers(m_routers);
    deletePointers(m_nis);
    deletePointers(m_links);
}

void
GarnetNetwork::makeInLink(NodeID src, SwitchID dest, BasicLink* link,
                          LinkDirection direction,
                          const NetDest& routing_table_entry)
{
    assert(src < m_nodes);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);
    NetworkLink *net_link = garnet_link->m_network_links[direction];

    net_link->init_net_ptr(this);
    m_links.push_back(net_link);
    m_routers[dest]->addInPort(net_link);
    m_nis[src]->addOutPort(net_link);
}

void
GarnetNetwork::makeOutLink(SwitchID src, NodeID dest, BasicLink* link,
                           LinkDirection direction,
                           const NetDest& routing_table_entry)
{
    assert(dest < m_nodes);
    assert(src < m_routers.size());
    assert(m_routers[src] != NULL);

    GarnetExtLink* garnet_link = safe_cast<GarnetExtLink*>(link);
    NetworkLink *net_link = garnet_link->m_network_links[direction];

    net_link->init_net_ptr(this);
    m_links.push_back(net_link);
    m_routers[src]->addOutPort(net_link, routing_table_entry,
                                         link->m_weight);
    m_nis[dest]->addInPort(net_link);
}

void
GarnetNetwork::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                LinkDirection direction,
                                const NetDest& routing_table_entry)
{
    GarnetIntLink* garnet_link = safe_cast<GarnetIntLink*>(link);
    NetworkLink *net_link = garnet_link->m_network_links[direction];

    net_link->init_net_ptr(this);
    m_links.push_back(net_link);
    m_routers[dest]->addInPort(net_link);
    m_routers[src]->addOutPort(net_link, routing_table_entry,
                                         link->m_weight);
}

/*
 * Go through all the routers, network interfaces and the interconnecting
 * links for reading/writing all the messages.
 */
bool
GarnetNetwork::functionalRead(Packet *pkt)
{
    for (unsigned int i = 0; i < m_routers.size(); i++) {
        if (m_routers[i]->functionalRead(pkt)) {
            return true;
        }
    }

    for (unsigned int i = 0; i < m_nis.size(); ++i) {
        if (m_nis[i]->functionalRead(pkt)) {
            return true;
        }
    }

    for (unsigned int i = 0; i < m_links.size(); ++i) {
        if (m_links[i]->functionalRead(pkt)) {
            return true;
        }
    }

    return false;
}

uint32_t
GarnetNetwork::functionalWrite(Packet *pkt)
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

void
GarnetNetwork::regStats()
{
    BaseGarnetNetwork::regStats();

    m_average_link_utilization.name(name() + ".avg_link_utilization");

    m_average_vc_load
        .init(m_virtual_networks * m_vcs_per_vnet)
        .name(name() + ".avg_vc_load")
        .flags(Stats::pdf | Stats::total | Stats::nozero)
        ;
    for (int i = 0; i < m_virtual_networks * m_vcs_per_vnet; i++) {
        m_average_vc_load
            .subname(i, csprintf(".%i", i))
            .flags(Stats::nozero)
            ;
    }
}

void
GarnetNetwork::collateStats()
{
    RubySystem *rs = params()->ruby_system;
    double time_delta = double(curCycle() - rs->getStartCycle());

    for (int i = 0; i < m_links.size(); i++) {
        m_average_link_utilization +=
            (double(m_links[i]->getLinkUtilization())) / time_delta;

        vector<unsigned int> vc_load = m_links[i]->getVcLoad();
        for (int j = 0; j < vc_load.size(); j++) {
            m_average_vc_load[j] += vc_load[j];
        }
    }
}

void
GarnetNetwork::print(ostream& out) const
{
    out << "[GarnetNetwork]";
}

GarnetNetwork *
GarnetNetworkParams::create()
{
    return new GarnetNetwork(this);
}
