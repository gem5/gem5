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

#include "mem/ruby/network/garnet/BaseGarnetNetwork.hh"
#include "mem/ruby/network/MessageBuffer.hh"

using namespace std;

BaseGarnetNetwork::BaseGarnetNetwork(const Params *p)
    : Network(p)
{
    m_ni_flit_size = p->ni_flit_size;
    m_vcs_per_vnet = p->vcs_per_vnet;
    m_enable_fault_model = p->enable_fault_model;
    if (m_enable_fault_model)
        fault_model = p->fault_model;

    // Currently Garnet only supports uniform bandwidth for all
    // links and network interfaces.
    for (std::vector<BasicExtLink*>::const_iterator i = p->ext_links.begin();
         i != p->ext_links.end(); ++i) {
        BasicExtLink* ext_link = (*i);
        if (ext_link->params()->bandwidth_factor != m_ni_flit_size) {
            fatal("Garnet only supports uniform bw across all links and NIs\n");
        }
    }
    for (std::vector<BasicIntLink*>::const_iterator i =  p->int_links.begin();
         i != p->int_links.end(); ++i) {
        BasicIntLink* int_link = (*i);
        if (int_link->params()->bandwidth_factor != m_ni_flit_size) {
            fatal("Garnet only supports uniform bw across all links and NIs\n");
        }
    }
}

void
BaseGarnetNetwork::init()
{
    Network::init();
}

void
BaseGarnetNetwork::regStats()
{
    m_flits_received
        .init(m_virtual_networks)
        .name(name() + ".flits_received")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_flits_injected
        .init(m_virtual_networks)
        .name(name() + ".flits_injected")
        .flags(Stats::pdf | Stats::total | Stats::nozero | Stats::oneline)
        ;

    m_network_latency
        .init(m_virtual_networks)
        .name(name() + ".network_latency")
        .flags(Stats::oneline)
        ;

    m_queueing_latency
        .init(m_virtual_networks)
        .name(name() + ".queueing_latency")
        .flags(Stats::oneline)
        ;

    for (int i = 0; i < m_virtual_networks; i++) {
        m_flits_received.subname(i, csprintf("vnet-%i", i));
        m_flits_injected.subname(i, csprintf("vnet-%i", i));
        m_network_latency.subname(i, csprintf("vnet-%i", i));
        m_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_vnet_latency
        .name(name() + ".average_vnet_latency")
        .flags(Stats::oneline);
    m_avg_vnet_latency = m_network_latency / m_flits_received;

    m_avg_vqueue_latency
        .name(name() + ".average_vqueue_latency")
        .flags(Stats::oneline);
    m_avg_vqueue_latency = m_queueing_latency / m_flits_received;

    m_avg_network_latency.name(name() + ".average_network_latency");
    m_avg_network_latency = sum(m_network_latency) / sum(m_flits_received);

    m_avg_queueing_latency.name(name() + ".average_queueing_latency");
    m_avg_queueing_latency = sum(m_queueing_latency) / sum(m_flits_received);

    m_avg_latency.name(name() + ".average_latency");
    m_avg_latency = m_avg_network_latency + m_avg_queueing_latency;
}
