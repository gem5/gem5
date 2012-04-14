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

#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/Topology.hh"
#include "mem/ruby/network/garnet/BaseGarnetNetwork.hh"

using namespace std;

BaseGarnetNetwork::BaseGarnetNetwork(const Params *p)
    : Network(p)
{
    m_ni_flit_size = p->ni_flit_size;
    m_vcs_per_vnet = p->vcs_per_vnet;
    m_enable_fault_model = p->enable_fault_model;
    if (m_enable_fault_model)
        fault_model = p->fault_model;

    m_ruby_start = 0;

    // Currently Garnet only supports uniform bandwidth for all
    // links and network interfaces.
    for (std::vector<BasicExtLink*>::const_iterator i = 
             m_topology_ptr->params()->ext_links.begin();
         i != m_topology_ptr->params()->ext_links.end(); ++i) {
        BasicExtLink* ext_link = (*i);
        if (ext_link->params()->bandwidth_factor != m_ni_flit_size) {
            fatal("Garnet only supports uniform bw across all links and NIs\n");
        }
    }
    for (std::vector<BasicIntLink*>::const_iterator i = 
             m_topology_ptr->params()->int_links.begin();
         i != m_topology_ptr->params()->int_links.end(); ++i) {
        BasicIntLink* int_link = (*i);
        if (int_link->params()->bandwidth_factor != m_ni_flit_size) {
            fatal("Garnet only supports uniform bw across all links and NIs\n");
        }
    }

    // Allocate to and from queues

    // Queues that are getting messages from protocol
    m_toNetQueues.resize(m_nodes);

    // Queues that are feeding the protocol
    m_fromNetQueues.resize(m_nodes);

    m_in_use.resize(m_virtual_networks);
    m_ordered.resize(m_virtual_networks);
    m_flits_received.resize(m_virtual_networks);
    m_flits_injected.resize(m_virtual_networks);
    m_network_latency.resize(m_virtual_networks);
    m_queueing_latency.resize(m_virtual_networks);
    for (int i = 0; i < m_virtual_networks; i++) {
        m_in_use[i] = false;
        m_ordered[i] = false;
        m_flits_received[i] = 0;
        m_flits_injected[i] = 0;
        m_network_latency[i] = 0.0;
        m_queueing_latency[i] = 0.0;
    }

    for (int node = 0; node < m_nodes; node++) {
        // Setting number of virtual message buffers per Network Queue
        m_toNetQueues[node].resize(m_virtual_networks);
        m_fromNetQueues[node].resize(m_virtual_networks);

        // Instantiating the Message Buffers that
        // interact with the coherence protocol
        for (int j = 0; j < m_virtual_networks; j++) {
            m_toNetQueues[node][j] = new MessageBuffer();
            m_fromNetQueues[node][j] = new MessageBuffer();
        }
    }
}

void
BaseGarnetNetwork::init()
{
    Network::init();
}

MessageBuffer*
BaseGarnetNetwork::getToNetQueue(NodeID id, bool ordered, int network_num,
                                 string vnet_type)
{
    checkNetworkAllocation(id, ordered, network_num, vnet_type);
    return m_toNetQueues[id][network_num];
}

MessageBuffer*
BaseGarnetNetwork::getFromNetQueue(NodeID id, bool ordered, int network_num,  
                                   string vnet_type)
{
    checkNetworkAllocation(id, ordered, network_num, vnet_type);
    return m_fromNetQueues[id][network_num];
}

void
BaseGarnetNetwork::clearStats()
{
    m_ruby_start = g_eventQueue_ptr->getTime();
}

Time
BaseGarnetNetwork::getRubyStartTime()
{
    return m_ruby_start;
}

void
BaseGarnetNetwork::printStats(ostream& out) const
{
    out << endl;
    out << "Network Stats" << endl;
    out << "-------------" << endl;
    out << endl;
    printPerformanceStats(out);
    printLinkStats(out);
    printPowerStats(out);
    m_topology_ptr->printStats(out);
}

void
BaseGarnetNetwork::printPerformanceStats(ostream& out) const
{
    int total_flits_injected = 0;
    int total_flits_received = 0;
    int total_network_latency = 0.0;
    int total_queueing_latency = 0.0;

    for (int i = 0; i < m_virtual_networks; i++) {
        if (!m_in_use[i])
            continue;

        out << "[Vnet " << i << "]: flits injected = "
            << m_flits_injected[i] << endl;
        out << "[Vnet " << i << "]: flits received = "
            << m_flits_received[i] << endl;
        out << "[Vnet " << i << "]: average network latency = "
            << ((double) m_network_latency[i] / (double) m_flits_received[i])
            << endl;
        out << "[Vnet " << i << "]: average queueing (at source NI) latency = "
            << ((double) m_queueing_latency[i] / (double) m_flits_received[i])
            << endl;

        out << endl;
        total_flits_injected += m_flits_injected[i];
        total_flits_received += m_flits_received[i];
        total_network_latency += m_network_latency[i];
        total_queueing_latency += m_queueing_latency[i];
    }
    out << "Total flits injected = " << total_flits_injected << endl;
    out << "Total flits received = " << total_flits_received << endl;
    out << "Average network latency = "
        << ((double) total_network_latency/ (double) total_flits_received) << endl;
    out << "Average queueing (at source NI) latency = "
        << ((double) total_queueing_latency/ (double) total_flits_received) << endl;
    out << "Average latency = "
        << ((double)  (total_queueing_latency + total_network_latency) /
            (double) total_flits_received)<< endl;
    out << "-------------" << endl;
    out << endl;
}

