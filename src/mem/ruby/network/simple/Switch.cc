/*
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

#include <numeric>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/simple/PerfectSwitch.hh"
#include "mem/ruby/network/simple/SimpleNetwork.hh"
#include "mem/ruby/network/simple/Switch.hh"
#include "mem/ruby/network/simple/Throttle.hh"

using namespace std;
using m5::stl_helpers::deletePointers;
using m5::stl_helpers::operator<<;

Switch::Switch(const Params *p) : BasicRouter(p)
{
    m_perfect_switch = new PerfectSwitch(m_id, this, p->virt_nets);
    m_port_buffers = p->port_buffers;
    m_num_connected_buffers = 0;
}

Switch::~Switch()
{
    delete m_perfect_switch;

    // Delete throttles (one per output port)
    deletePointers(m_throttles);

    // Delete MessageBuffers
    deletePointers(m_port_buffers);
}

void
Switch::init()
{
    BasicRouter::init();
    m_perfect_switch->init(m_network_ptr);
}

void
Switch::addInPort(const vector<MessageBuffer*>& in)
{
    m_perfect_switch->addInPort(in);
}

void
Switch::addOutPort(const vector<MessageBuffer*>& out,
                   const NetDest& routing_table_entry,
                   Cycles link_latency, int bw_multiplier)
{
    // Create a throttle
    RubySystem *rs = m_network_ptr->params()->ruby_system;
    Throttle* throttle_ptr = new Throttle(m_id, rs, m_throttles.size(),
                                          link_latency, bw_multiplier,
                                          m_network_ptr->getEndpointBandwidth(),
                                          this);

    m_throttles.push_back(throttle_ptr);

    // Create one buffer per vnet (these are intermediaryQueues)
    vector<MessageBuffer*> intermediateBuffers;

    for (int i = 0; i < out.size(); ++i) {
        assert(m_num_connected_buffers < m_port_buffers.size());
        MessageBuffer* buffer_ptr = m_port_buffers[m_num_connected_buffers];
        m_num_connected_buffers++;
        intermediateBuffers.push_back(buffer_ptr);
    }

    // Hook the queues to the PerfectSwitch
    m_perfect_switch->addOutPort(intermediateBuffers, routing_table_entry);

    // Hook the queues to the Throttle
    throttle_ptr->addLinks(intermediateBuffers, out);
}

const Throttle*
Switch::getThrottle(LinkID link_number) const
{
    assert(m_throttles[link_number] != NULL);
    return m_throttles[link_number];
}

void
Switch::regStats()
{
    BasicRouter::regStats();

    for (int link = 0; link < m_throttles.size(); link++) {
        m_throttles[link]->regStats(name());
    }

    m_avg_utilization.name(name() + ".percent_links_utilized");
    for (unsigned int i = 0; i < m_throttles.size(); i++) {
        m_avg_utilization += m_throttles[i]->getUtilization();
    }
    m_avg_utilization /= Stats::constant(m_throttles.size());

    for (unsigned int type = MessageSizeType_FIRST;
         type < MessageSizeType_NUM; ++type) {
        m_msg_counts[type]
            .name(name() + ".msg_count." +
                MessageSizeType_to_string(MessageSizeType(type)))
            .flags(Stats::nozero)
            ;
        m_msg_bytes[type]
            .name(name() + ".msg_bytes." +
                MessageSizeType_to_string(MessageSizeType(type)))
            .flags(Stats::nozero)
            ;

        for (unsigned int i = 0; i < m_throttles.size(); i++) {
            m_msg_counts[type] += m_throttles[i]->getMsgCount(type);
        }
        m_msg_bytes[type] = m_msg_counts[type] * Stats::constant(
                Network::MessageSizeType_to_int(MessageSizeType(type)));
    }
}

void
Switch::resetStats()
{
    m_perfect_switch->clearStats();
    for (int i = 0; i < m_throttles.size(); i++) {
        m_throttles[i]->clearStats();
    }
}

void
Switch::collateStats()
{
    m_perfect_switch->collateStats();
    for (int i = 0; i < m_throttles.size(); i++) {
        m_throttles[i]->collateStats();
    }
}

void
Switch::print(std::ostream& out) const
{
    // FIXME printing
    out << "[Switch]";
}

bool
Switch::functionalRead(Packet *pkt)
{
    return false;
}

uint32_t
Switch::functionalWrite(Packet *pkt)
{
    // Access the buffers in the switch for performing a functional write
    uint32_t num_functional_writes = 0;
    for (unsigned int i = 0; i < m_port_buffers.size(); ++i) {
        num_functional_writes += m_port_buffers[i]->functionalWrite(pkt);
    }
    return num_functional_writes;
}

Switch *
SwitchParams::create()
{
    return new Switch(this);
}
