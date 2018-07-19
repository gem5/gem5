/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
 * Copyright (c) 2019 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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
 */

#include "mem/ruby/network/simple/SimpleNetwork.hh"

#include <cassert>
#include <numeric>

#include "base/cast.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/simple/SimpleLink.hh"
#include "mem/ruby/network/simple/Switch.hh"
#include "mem/ruby/network/simple/Throttle.hh"
#include "mem/ruby/profiler/Profiler.hh"

using namespace std;

SimpleNetwork::SimpleNetwork(const Params *p)
    : Network(p), m_buffer_size(p->buffer_size),
      m_endpoint_bandwidth(p->endpoint_bandwidth),
      m_adaptive_routing(p->adaptive_routing)
{
    // record the routers
    for (vector<BasicRouter*>::const_iterator i = p->routers.begin();
         i != p->routers.end(); ++i) {
        Switch* s = safe_cast<Switch*>(*i);
        m_switches.push_back(s);
        s->init_net_ptr(this);
    }

    m_int_link_buffers = p->int_link_buffers;
    m_num_connected_buffers = 0;
}

void
SimpleNetwork::init()
{
    Network::init();

    // The topology pointer should have already been initialized in
    // the parent class network constructor.
    assert(m_topology_ptr != NULL);
    m_topology_ptr->createLinks(this);
}

// From a switch to an endpoint node
void
SimpleNetwork::makeExtOutLink(SwitchID src, NodeID global_dest,
                              BasicLink* link,
                              std::vector<NetDest>& routing_table_entry)
{
    NodeID local_dest = getLocalNodeID(global_dest);
    assert(local_dest < m_nodes);
    assert(src < m_switches.size());
    assert(m_switches[src] != NULL);

    SimpleExtLink *simple_link = safe_cast<SimpleExtLink*>(link);

    m_switches[src]->addOutPort(m_fromNetQueues[local_dest],
                                routing_table_entry[0], simple_link->m_latency,
                                simple_link->m_bw_multiplier);
}

// From an endpoint node to a switch
void
SimpleNetwork::makeExtInLink(NodeID global_src, SwitchID dest, BasicLink* link,
                          std::vector<NetDest>& routing_table_entry)
{
    NodeID local_src = getLocalNodeID(global_src);
    assert(local_src < m_nodes);
    m_switches[dest]->addInPort(m_toNetQueues[local_src]);
}

// From a switch to a switch
void
SimpleNetwork::makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                std::vector<NetDest>& routing_table_entry,
                                PortDirection src_outport,
                                PortDirection dst_inport)
{
    // Create a set of new MessageBuffers
    std::vector<MessageBuffer*> queues(m_virtual_networks);

    for (int i = 0; i < m_virtual_networks; i++) {
        // allocate a buffer
        assert(m_num_connected_buffers < m_int_link_buffers.size());
        MessageBuffer* buffer_ptr = m_int_link_buffers[m_num_connected_buffers];
        m_num_connected_buffers++;
        queues[i] = buffer_ptr;
    }

    // Connect it to the two switches
    SimpleIntLink *simple_link = safe_cast<SimpleIntLink*>(link);

    m_switches[dest]->addInPort(queues);
    m_switches[src]->addOutPort(queues, routing_table_entry[0],
                                simple_link->m_latency,
                                simple_link->m_bw_multiplier);
}

void
SimpleNetwork::regStats()
{
    Network::regStats();

    for (MessageSizeType type = MessageSizeType_FIRST;
         type < MessageSizeType_NUM; ++type) {
        m_msg_counts[(unsigned int) type]
            .name(name() + ".msg_count." + MessageSizeType_to_string(type))
            .flags(Stats::nozero)
            ;
        m_msg_bytes[(unsigned int) type]
            .name(name() + ".msg_byte." + MessageSizeType_to_string(type))
            .flags(Stats::nozero)
            ;

        // Now state what the formula is.
        for (int i = 0; i < m_switches.size(); i++) {
            m_msg_counts[(unsigned int) type] +=
                sum(m_switches[i]->getMsgCount(type));
        }

        m_msg_bytes[(unsigned int) type] =
            m_msg_counts[(unsigned int) type] * Stats::constant(
                    Network::MessageSizeType_to_int(type));
    }
}

void
SimpleNetwork::collateStats()
{
    for (int i = 0; i < m_switches.size(); i++) {
        m_switches[i]->collateStats();
    }
}

void
SimpleNetwork::print(ostream& out) const
{
    out << "[SimpleNetwork]";
}

SimpleNetwork *
SimpleNetworkParams::create()
{
    return new SimpleNetwork(this);
}

/*
 * The simple network has an array of switches. These switches have buffers
 * that need to be accessed for functional reads and writes. Also the links
 * between different switches have buffers that need to be accessed.
 */
bool
SimpleNetwork::functionalRead(Packet *pkt)
{
    for (unsigned int i = 0; i < m_switches.size(); i++) {
        if (m_switches[i]->functionalRead(pkt))
            return true;
    }
    for (unsigned int i = 0; i < m_int_link_buffers.size(); ++i) {
        if (m_int_link_buffers[i]->functionalRead(pkt))
            return true;
    }

    return false;
}

uint32_t
SimpleNetwork::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (unsigned int i = 0; i < m_switches.size(); i++) {
        num_functional_writes += m_switches[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_int_link_buffers.size(); ++i) {
        num_functional_writes += m_int_link_buffers[i]->functionalWrite(pkt);
    }
    return num_functional_writes;
}
