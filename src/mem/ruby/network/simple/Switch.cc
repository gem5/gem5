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
#include "mem/protocol/MessageSizeType.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/network/simple/PerfectSwitch.hh"
#include "mem/ruby/network/simple/SimpleNetwork.hh"
#include "mem/ruby/network/simple/Switch.hh"
#include "mem/ruby/network/simple/Throttle.hh"

using namespace std;
using m5::stl_helpers::deletePointers;
using m5::stl_helpers::operator<<;

Switch::Switch(SwitchID sid, SimpleNetwork* network_ptr)
{
    m_perfect_switch_ptr = new PerfectSwitch(sid, network_ptr);
    m_switch_id = sid;
}

Switch::~Switch()
{
    delete m_perfect_switch_ptr;

    // Delete throttles (one per output port)
    deletePointers(m_throttles);

    // Delete MessageBuffers
    deletePointers(m_buffers_to_free);
}

void
Switch::addInPort(const vector<MessageBuffer*>& in)
{
    m_perfect_switch_ptr->addInPort(in);
}

void
Switch::addOutPort(const vector<MessageBuffer*>& out,
    const NetDest& routing_table_entry, int link_latency, int bw_multiplier)
{
    Throttle* throttle_ptr = NULL;
    SimpleNetwork* net_ptr = 
        safe_cast<SimpleNetwork*>(RubySystem::getNetwork());

    // Create a throttle
    throttle_ptr = new Throttle(m_switch_id, m_throttles.size(), link_latency,
                                bw_multiplier, net_ptr->getEndpointBandwidth());
    m_throttles.push_back(throttle_ptr);

    // Create one buffer per vnet (these are intermediaryQueues)
    vector<MessageBuffer*> intermediateBuffers;
    for (int i = 0; i < out.size(); i++) {
        MessageBuffer* buffer_ptr = new MessageBuffer;
        // Make these queues ordered
        buffer_ptr->setOrdering(true);
        if (net_ptr->getBufferSize() > 0) {
            buffer_ptr->resize(net_ptr->getBufferSize());
        }
        intermediateBuffers.push_back(buffer_ptr);
        m_buffers_to_free.push_back(buffer_ptr);
  }

    // Hook the queues to the PerfectSwitch
    m_perfect_switch_ptr->addOutPort(intermediateBuffers, routing_table_entry);

    // Hook the queues to the Throttle
    throttle_ptr->addLinks(intermediateBuffers, out);
}

void
Switch::clearRoutingTables()
{
    m_perfect_switch_ptr->clearRoutingTables();
}

void
Switch::clearBuffers()
{
    m_perfect_switch_ptr->clearBuffers();
    for (int i = 0; i < m_throttles.size(); i++) {
        if (m_throttles[i] != NULL) {
            m_throttles[i]->clear();
        }
    }
}

void
Switch::reconfigureOutPort(const NetDest& routing_table_entry)
{
    m_perfect_switch_ptr->reconfigureOutPort(routing_table_entry);
}

const Throttle*
Switch::getThrottle(LinkID link_number) const
{
    assert(m_throttles[link_number] != NULL);
    return m_throttles[link_number];
}

const vector<Throttle*>*
Switch::getThrottles() const
{
    return &m_throttles;
}

void
Switch::printStats(std::ostream& out) const
{
    ccprintf(out, "switch_%d_inlinks: %d\n", m_switch_id,
        m_perfect_switch_ptr->getInLinks());
    ccprintf(out, "switch_%d_outlinks: %d\n", m_switch_id,
        m_perfect_switch_ptr->getOutLinks());

    // Average link utilizations
    double average_utilization = 0.0;
    int throttle_count = 0;

    for (int i = 0; i < m_throttles.size(); i++) {
        Throttle* throttle_ptr = m_throttles[i];
        if (throttle_ptr) {
            average_utilization += throttle_ptr->getUtilization();
            throttle_count++;
        }
    }
    average_utilization =
        throttle_count == 0 ? 0 : average_utilization / throttle_count;

    // Individual link utilizations
    out << "links_utilized_percent_switch_" << m_switch_id << ": "
        << average_utilization << endl;

    for (int link = 0; link < m_throttles.size(); link++) {
        Throttle* throttle_ptr = m_throttles[link];
        if (throttle_ptr != NULL) {
            out << "  links_utilized_percent_switch_" << m_switch_id
                << "_link_" << link << ": "
                << throttle_ptr->getUtilization() << " bw: "
                << throttle_ptr->getLinkBandwidth()
                << " base_latency: " << throttle_ptr->getLatency() << endl;
        }
    }
    out << endl;

    // Traffic breakdown
    for (int link = 0; link < m_throttles.size(); link++) {
        Throttle* throttle_ptr = m_throttles[link];
        if (!throttle_ptr)
            continue;

        const vector<vector<int> >& message_counts =
            throttle_ptr->getCounters();
        for (int int_type = 0; int_type < MessageSizeType_NUM; int_type++) {
            MessageSizeType type = MessageSizeType(int_type);
            const vector<int> &mct = message_counts[type];
            int sum = accumulate(mct.begin(), mct.end(), 0);
            if (sum == 0)
                continue;

            out << "  outgoing_messages_switch_" << m_switch_id
                << "_link_" << link << "_" << type << ": " << sum << " "
                << sum * RubySystem::getNetwork()->MessageSizeType_to_int(type)
                << " ";
            out << mct;
            out << " base_latency: "
                << throttle_ptr->getLatency() << endl;
        }
    }
    out << endl;
}

void
Switch::clearStats()
{
    m_perfect_switch_ptr->clearStats();
    for (int i = 0; i < m_throttles.size(); i++) {
        if (m_throttles[i] != NULL)
            m_throttles[i]->clearStats();
    }
}

void
Switch::printConfig(std::ostream& out) const
{
    m_perfect_switch_ptr->printConfig(out);
    for (int i = 0; i < m_throttles.size(); i++) {
        if (m_throttles[i] != NULL)
            m_throttles[i]->printConfig(out);
    }
}

void
Switch::print(std::ostream& out) const
{
    // FIXME printing
    out << "[Switch]";
}

