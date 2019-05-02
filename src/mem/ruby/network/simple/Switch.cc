/*
 * Copyright (c) 2020 Inria
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

#include "mem/ruby/network/simple/Switch.hh"

#include <numeric>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/simple/SimpleNetwork.hh"

using namespace std;
using m5::stl_helpers::operator<<;

Switch::Switch(const Params *p)
  : BasicRouter(p), perfectSwitch(m_id, this, p->virt_nets),
    m_num_connected_buffers(0)
{
    m_port_buffers.reserve(p->port_buffers.size());
    for (auto& buffer : p->port_buffers) {
        m_port_buffers.emplace_back(buffer);
    }
}

void
Switch::init()
{
    BasicRouter::init();
    perfectSwitch.init(m_network_ptr);
}

void
Switch::addInPort(const vector<MessageBuffer*>& in)
{
    perfectSwitch.addInPort(in);
}

void
Switch::addOutPort(const vector<MessageBuffer*>& out,
                   const NetDest& routing_table_entry,
                   Cycles link_latency, int bw_multiplier)
{
    // Create a throttle
    throttles.emplace_back(m_id, m_network_ptr->params()->ruby_system,
        throttles.size(), link_latency, bw_multiplier,
        m_network_ptr->getEndpointBandwidth(), this);

    // Create one buffer per vnet (these are intermediaryQueues)
    vector<MessageBuffer*> intermediateBuffers;

    for (int i = 0; i < out.size(); ++i) {
        assert(m_num_connected_buffers < m_port_buffers.size());
        MessageBuffer* buffer_ptr =
            m_port_buffers[m_num_connected_buffers];
        m_num_connected_buffers++;
        intermediateBuffers.push_back(buffer_ptr);
    }

    // Hook the queues to the PerfectSwitch
    perfectSwitch.addOutPort(intermediateBuffers, routing_table_entry);

    // Hook the queues to the Throttle
    throttles.back().addLinks(intermediateBuffers, out);
}

void
Switch::regStats()
{
    BasicRouter::regStats();

    for (auto& throttle : throttles) {
        throttle.regStats(name());
    }

    m_avg_utilization.name(name() + ".percent_links_utilized");
    for (const auto& throttle : throttles) {
        m_avg_utilization += throttle.getUtilization();
    }
    m_avg_utilization /= Stats::constant(throttles.size());

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

        for (const auto& throttle : throttles) {
            m_msg_counts[type] += throttle.getMsgCount(type);
        }
        m_msg_bytes[type] = m_msg_counts[type] * Stats::constant(
                Network::MessageSizeType_to_int(MessageSizeType(type)));
    }
}

void
Switch::resetStats()
{
    perfectSwitch.clearStats();
    for (auto& throttle : throttles) {
        throttle.clearStats();
    }
}

void
Switch::collateStats()
{
    perfectSwitch.collateStats();
    for (auto& throttle : throttles) {
        throttle.collateStats();
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
    for (unsigned int i = 0; i < m_port_buffers.size(); ++i) {
        if (m_port_buffers[i]->functionalRead(pkt))
            return true;
    }
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
