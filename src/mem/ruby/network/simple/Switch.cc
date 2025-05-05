/*
 * Copyright (c) 2020 Inria
 * Copyright (c) 2019,2021 ARM Limited
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

namespace gem5
{

namespace ruby
{

using stl_helpers::operator<<;

Switch::Switch(const Params &p)
  : BasicRouter(p),
    perfectSwitch(m_id, this, p.virt_nets),
    m_int_routing_latency(p.int_routing_latency),
    m_ext_routing_latency(p.ext_routing_latency),
    m_routing_unit(*p.routing_unit), m_num_connected_buffers(0),
    switchStats(this)
{
    m_port_buffers.reserve(p.port_buffers.size());
    for (auto& buffer : p.port_buffers) {
        m_port_buffers.emplace_back(buffer);
    }
}

void
Switch::init()
{
    BasicRouter::init();
    perfectSwitch.init(m_network_ptr);
    m_routing_unit.init_parent(this);
}

void
Switch::addInPort(const std::vector<MessageBuffer*>& in)
{
    perfectSwitch.addInPort(in);
}

void
Switch::addOutPort(std::string link_name,
                   const std::vector<MessageBuffer*>& out,
                   const NetDest& routing_table_entry,
                   Cycles link_latency, int link_weight,
                   int bw_multiplier,
                   bool is_external,
                   PortDirection dst_inport)
{
    const std::vector<int> &physical_vnets_channels =
        m_network_ptr->params().physical_vnets_channels;

    // Create a throttle
    if (physical_vnets_channels.size() > 0 && !out.empty()) {
        // Assign a different bandwith for each vnet channel if specified by
        // physical_vnets_bandwidth, otherwise all channels use bw_multiplier
        std::vector<int> physical_vnets_bandwidth =
            m_network_ptr->params().physical_vnets_bandwidth;
        physical_vnets_bandwidth.resize(out.size(), bw_multiplier);

        throttles.emplace_back(m_id, m_network_ptr->params().ruby_system,
            throttles.size(), link_latency,
            physical_vnets_channels, physical_vnets_bandwidth,
            m_network_ptr->getEndpointBandwidth(), this, link_name);
    } else {
        throttles.emplace_back(m_id, m_network_ptr->params().ruby_system,
            throttles.size(), link_latency, bw_multiplier,
            m_network_ptr->getEndpointBandwidth(), this, link_name);
    }

    // Create one buffer per vnet (these are intermediaryQueues)
    std::vector<MessageBuffer*> intermediateBuffers;

    for (int i = 0; i < out.size(); ++i) {
        assert(m_num_connected_buffers < m_port_buffers.size());
        MessageBuffer* buffer_ptr =
            m_port_buffers[m_num_connected_buffers];
        m_num_connected_buffers++;
        intermediateBuffers.push_back(buffer_ptr);
    }

    Tick routing_latency = is_external ? cyclesToTicks(m_ext_routing_latency) :
                                         cyclesToTicks(m_int_routing_latency);
    // Hook the queues to the PerfectSwitch
    perfectSwitch.addOutPort(intermediateBuffers, routing_table_entry,
                             dst_inport, routing_latency, link_weight);

    // Hook the queues to the Throttle
    throttles.back().addLinks(intermediateBuffers, out);
}

void
Switch::regStats()
{
    BasicRouter::regStats();

    for (const auto& throttle : throttles) {
        switchStats.m_avg_utilization += throttle.getUtilization();
    }
    switchStats.m_avg_utilization /= statistics::constant(throttles.size());

    for (unsigned int type = MessageSizeType_FIRST;
         type < MessageSizeType_NUM; ++type) {
        switchStats.m_msg_counts[type] = new statistics::Formula(&switchStats,
            csprintf("msg_count.%s",
                MessageSizeType_to_string(MessageSizeType(type))).c_str());
        switchStats.m_msg_counts[type]
            ->flags(statistics::nozero)
            ;

        switchStats.m_msg_bytes[type] = new statistics::Formula(&switchStats,
            csprintf("msg_bytes.%s",
                MessageSizeType_to_string(MessageSizeType(type))).c_str());
        switchStats.m_msg_bytes[type]
            ->flags(statistics::nozero)
            ;

        for (const auto& throttle : throttles) {
            *(switchStats.m_msg_counts[type]) += throttle.getMsgCount(type);
        }
        *(switchStats.m_msg_bytes[type]) =
            *(switchStats.m_msg_counts[type]) * statistics::constant(
                Network::MessageSizeType_to_int(MessageSizeType(type)));
    }
}

void
Switch::resetStats()
{
    perfectSwitch.clearStats();
}

void
Switch::collateStats()
{
    perfectSwitch.collateStats();
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

bool
Switch::functionalRead(Packet *pkt, WriteMask &mask)
{
    bool read = false;
    for (unsigned int i = 0; i < m_port_buffers.size(); ++i) {
        if (m_port_buffers[i]->functionalRead(pkt, mask))
            read = true;
    }
    return read;
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

Switch::
SwitchStats::SwitchStats(statistics::Group *parent)
    : statistics::Group(parent),
      m_avg_utilization(this, "percent_links_utilized")
{

}

} // namespace ruby
} // namespace gem5
