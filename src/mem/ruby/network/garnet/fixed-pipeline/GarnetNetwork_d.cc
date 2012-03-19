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
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/garnet/BaseGarnetNetwork.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkInterface_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/network/Topology.hh"

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
    for (vector<BasicRouter*>::const_iterator i = 
             m_topology_ptr->params()->routers.begin();
         i != m_topology_ptr->params()->routers.end(); ++i) {
        Router_d* router = safe_cast<Router_d*>(*i);
        m_router_ptr_vector.push_back(router);
    }
}

void
GarnetNetwork_d::init()
{
    BaseGarnetNetwork::init();

    // initialize the router's network pointers
    for (vector<Router_d*>::const_iterator i = m_router_ptr_vector.begin();
         i != m_router_ptr_vector.end(); ++i) {
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
        m_ni_ptr_vector.push_back(ni);
    }
    // false because this isn't a reconfiguration
    m_topology_ptr->createLinks(this, false);

    // initialize the link's network pointers
   for (vector<NetworkLink_d*>::const_iterator i = m_link_ptr_vector.begin();
         i != m_link_ptr_vector.end(); ++i) {
        NetworkLink_d* net_link = safe_cast<NetworkLink_d*>(*i);
        net_link->init_net_ptr(this);
    }

    // FaultModel: declare each router to the fault model
    if(isFaultModelEnabled()){
        for (vector<Router_d*>::const_iterator i= m_router_ptr_vector.begin();
             i != m_router_ptr_vector.end(); ++i) {
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
    deletePointers(m_router_ptr_vector);
    deletePointers(m_ni_ptr_vector);
    deletePointers(m_link_ptr_vector);
    deletePointers(m_creditlink_ptr_vector);
    delete m_topology_ptr;
}

void
GarnetNetwork_d::reset()
{
    for (int node = 0; node < m_nodes; node++) {
        for (int j = 0; j < m_virtual_networks; j++) {
            m_toNetQueues[node][j]->clear();
            m_fromNetQueues[node][j]->clear();
        }
    }
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
                            const NetDest& routing_table_entry,
                            bool isReconfiguration)
{
    assert(src < m_nodes);

    GarnetExtLink_d* garnet_link = safe_cast<GarnetExtLink_d*>(link);

    if (!isReconfiguration) {
        NetworkLink_d* net_link = garnet_link->m_network_links[direction];
        CreditLink_d* credit_link = garnet_link->m_credit_links[direction];

        m_link_ptr_vector.push_back(net_link);
        m_creditlink_ptr_vector.push_back(credit_link);

        m_router_ptr_vector[dest]->addInPort(net_link, credit_link);
        m_ni_ptr_vector[src]->addOutPort(net_link, credit_link);
    } else {
        panic("Fatal Error:: Reconfiguration not allowed here");
        // do nothing
    }
}

/*
 * This function creates a link from the Network to a NI.
 * It creates a Network Link from a Router to the NI and
 * a Credit Link from NI to the Router
*/

void
GarnetNetwork_d::makeOutLink(SwitchID src, NodeID dest, BasicLink* link,
                             LinkDirection direction,
                             const NetDest& routing_table_entry,
                             bool isReconfiguration)
{
    assert(dest < m_nodes);
    assert(src < m_router_ptr_vector.size());
    assert(m_router_ptr_vector[src] != NULL);

    GarnetExtLink_d* garnet_link = safe_cast<GarnetExtLink_d*>(link);

    if (!isReconfiguration) {
        NetworkLink_d* net_link = garnet_link->m_network_links[direction];
        CreditLink_d* credit_link = garnet_link->m_credit_links[direction];

        m_link_ptr_vector.push_back(net_link);
        m_creditlink_ptr_vector.push_back(credit_link);

        m_router_ptr_vector[src]->addOutPort(net_link, routing_table_entry,
                                             link->m_weight, 
                                             credit_link);
        m_ni_ptr_vector[dest]->addInPort(net_link, credit_link);
    } else {
        fatal("Fatal Error:: Reconfiguration not allowed here");
        // do nothing
    }
}

/*
 * This function creates an internal network link
*/

void
GarnetNetwork_d::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                  LinkDirection direction,
                                  const NetDest& routing_table_entry,
                                  bool isReconfiguration)
{
    GarnetIntLink_d* garnet_link = safe_cast<GarnetIntLink_d*>(link);

    if (!isReconfiguration) {
        NetworkLink_d* net_link = garnet_link->m_network_links[direction];
        CreditLink_d* credit_link = garnet_link->m_credit_links[direction];

        m_link_ptr_vector.push_back(net_link);
        m_creditlink_ptr_vector.push_back(credit_link);

        m_router_ptr_vector[dest]->addInPort(net_link, credit_link);
        m_router_ptr_vector[src]->addOutPort(net_link, routing_table_entry,
                                             link->m_weight, 
                                             credit_link);
    } else {
        fatal("Fatal Error:: Reconfiguration not allowed here");
        // do nothing
    }
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
GarnetNetwork_d::printLinkStats(ostream& out) const
{
    double average_link_utilization = 0;
    vector<double> average_vc_load;
    average_vc_load.resize(m_virtual_networks*m_vcs_per_vnet);

    for (int i = 0; i < m_virtual_networks*m_vcs_per_vnet; i++) {
        average_vc_load[i] = 0;
    }

    out << endl;
    for (int i = 0; i < m_link_ptr_vector.size(); i++) {
        average_link_utilization +=
            (double(m_link_ptr_vector[i]->getLinkUtilization())) /
            (double(g_eventQueue_ptr->getTime()-m_ruby_start));

        vector<int> vc_load = m_link_ptr_vector[i]->getVcLoad();
        for (int j = 0; j < vc_load.size(); j++) {
            assert(vc_load.size() == m_vcs_per_vnet*m_virtual_networks);
            average_vc_load[j] += vc_load[j];
        }
    }
    average_link_utilization =
        average_link_utilization/m_link_ptr_vector.size();
    out << "Average Link Utilization :: " << average_link_utilization
        << " flits/cycle" << endl;
    out << "-------------" << endl;

    for (int i = 0; i < m_vcs_per_vnet*m_virtual_networks; i++) {
        if (!m_in_use[i/m_vcs_per_vnet])
            continue;

        average_vc_load[i] = (double(average_vc_load[i]) /
            (double(g_eventQueue_ptr->getTime()) - m_ruby_start));
        out << "Average VC Load [" << i << "] = " << average_vc_load[i]
            << " flits/cycle " << endl;
    }
    out << "-------------" << endl;
    out << endl;
}

void
GarnetNetwork_d::printPowerStats(ostream& out) const
{
    out << "Network Power" << endl;
    out << "-------------" << endl;
    double m_total_link_power = 0.0;
    double m_dynamic_link_power = 0.0;
    double m_static_link_power = 0.0;
    double m_total_router_power = 0.0;
    double m_dynamic_router_power = 0.0;
    double m_static_router_power = 0.0;
    double m_clk_power = 0.0;

    for (int i = 0; i < m_link_ptr_vector.size(); i++) {
        m_total_link_power += m_link_ptr_vector[i]->calculate_power();
        m_dynamic_link_power += m_link_ptr_vector[i]->get_dynamic_power();
        m_static_link_power += m_link_ptr_vector[i]->get_static_power();
    }

    for (int i = 0; i < m_router_ptr_vector.size(); i++) {
        m_total_router_power += m_router_ptr_vector[i]->calculate_power();
        m_dynamic_router_power += m_router_ptr_vector[i]->get_dynamic_power();
        m_static_router_power += m_router_ptr_vector[i]->get_static_power();
        m_clk_power += m_router_ptr_vector[i]->get_clk_power();
    }
    out << "Link Dynamic Power = " << m_dynamic_link_power << " W" << endl;
    out << "Link Static Power = " << m_static_link_power << " W" << endl;
    out << "Total Link Power = " << m_total_link_power << " W " << endl;
    out << "Router Dynamic Power = " << m_dynamic_router_power << " W" << endl;
    out << "Router Clock Power = " << m_clk_power << " W" << endl;
    out << "Router Static Power = " << m_static_router_power << " W" << endl;
    out << "Total Router Power = " << m_total_router_power << " W " <<endl;
    out << "-------------" << endl;
    out << endl;
}

void
GarnetNetwork_d::printConfig(ostream& out) const
{
    out << endl;
    out << "Network Configuration" << endl;
    out << "---------------------" << endl;
    out << "network: Garnet Fixed Pipeline" << endl;
    out << "topology: " << m_topology_ptr->getName() << endl;
    out << endl;

    for (int i = 0; i < m_virtual_networks; i++) {
        out << "virtual_net_" << i << ": ";
        if (m_in_use[i]) {
            out << "active, ";
            if (m_ordered[i]) {
                out << "ordered" << endl;
            } else {
                out << "unordered" << endl;
            }
        } else {
            out << "inactive" << endl;
        }
    }
    out << endl;

    for (int i = 0; i < m_ni_ptr_vector.size(); i++) {
        m_ni_ptr_vector[i]->printConfig(out);
    }
    for (int i = 0; i < m_router_ptr_vector.size(); i++) {
        m_router_ptr_vector[i]->printConfig(out);
    }
    m_topology_ptr->printConfig(out);
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
