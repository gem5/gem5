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

#include <cassert>

#include "base/cast.hh"
#include "base/cprintf.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/network/simple/Throttle.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/slicc_interface/NetworkMessage.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

const int HIGH_RANGE = 256;
const int ADJUST_INTERVAL = 50000;
const int MESSAGE_SIZE_MULTIPLIER = 1000;
//const int BROADCAST_SCALING = 4; // Have a 16p system act like a 64p systems
const int BROADCAST_SCALING = 1;
const int PRIORITY_SWITCH_LIMIT = 128;

static int network_message_to_size(NetworkMessage* net_msg_ptr);

Throttle::Throttle(int sID, NodeID node, Cycles link_latency,
                   int link_bandwidth_multiplier, int endpoint_bandwidth,
                   ClockedObject *em)
    : Consumer(em)
{
    init(node, link_latency, link_bandwidth_multiplier, endpoint_bandwidth);
    m_sID = sID;
}

Throttle::Throttle(NodeID node, Cycles link_latency,
                   int link_bandwidth_multiplier, int endpoint_bandwidth,
                   ClockedObject *em)
    : Consumer(em)
{
    init(node, link_latency, link_bandwidth_multiplier, endpoint_bandwidth);
    m_sID = 0;
}

void
Throttle::init(NodeID node, Cycles link_latency,
               int link_bandwidth_multiplier, int endpoint_bandwidth)
{
    m_node = node;
    m_vnets = 0;

    assert(link_bandwidth_multiplier > 0);
    m_link_bandwidth_multiplier = link_bandwidth_multiplier;
    m_link_latency = link_latency;
    m_endpoint_bandwidth = endpoint_bandwidth;

    m_wakeups_wo_switch = 0;

    m_link_utilization_proxy = 0;
}

void
Throttle::addLinks(const std::vector<MessageBuffer*>& in_vec,
    const std::vector<MessageBuffer*>& out_vec)
{
    assert(in_vec.size() == out_vec.size());
    for (int i=0; i<in_vec.size(); i++) {
        addVirtualNetwork(in_vec[i], out_vec[i]);
    }
}

void
Throttle::addVirtualNetwork(MessageBuffer* in_ptr, MessageBuffer* out_ptr)
{
    m_units_remaining.push_back(0);
    m_in.push_back(in_ptr);
    m_out.push_back(out_ptr);

    // Set consumer and description
    m_in[m_vnets]->setConsumer(this);

    string desc = "[Queue to Throttle " + to_string(m_sID) + " " +
        to_string(m_node) + "]";
    m_in[m_vnets]->setDescription(desc);
    m_vnets++;
}

void
Throttle::wakeup()
{
    // Limits the number of message sent to a limited number of bytes/cycle.
    assert(getLinkBandwidth() > 0);
    int bw_remaining = getLinkBandwidth();

    // Give the highest numbered link priority most of the time
    m_wakeups_wo_switch++;
    int highest_prio_vnet = m_vnets-1;
    int lowest_prio_vnet = 0;
    int counter = 1;
    bool schedule_wakeup = false;

    // invert priorities to avoid starvation seen in the component network
    if (m_wakeups_wo_switch > PRIORITY_SWITCH_LIMIT) {
        m_wakeups_wo_switch = 0;
        highest_prio_vnet = 0;
        lowest_prio_vnet = m_vnets-1;
        counter = -1;
    }

    for (int vnet = highest_prio_vnet;
         (vnet * counter) >= (counter * lowest_prio_vnet);
         vnet -= counter) {

        assert(m_out[vnet] != NULL);
        assert(m_in[vnet] != NULL);
        assert(m_units_remaining[vnet] >= 0);

        while (bw_remaining > 0 &&
            (m_in[vnet]->isReady() || m_units_remaining[vnet] > 0) &&
            m_out[vnet]->areNSlotsAvailable(1)) {

            // See if we are done transferring the previous message on
            // this virtual network
            if (m_units_remaining[vnet] == 0 && m_in[vnet]->isReady()) {
                // Find the size of the message we are moving
                MsgPtr msg_ptr = m_in[vnet]->peekMsgPtr();
                NetworkMessage* net_msg_ptr =
                    safe_cast<NetworkMessage*>(msg_ptr.get());
                m_units_remaining[vnet] +=
                    network_message_to_size(net_msg_ptr);

                DPRINTF(RubyNetwork, "throttle: %d my bw %d bw spent "
                        "enqueueing net msg %d time: %lld.\n",
                        m_node, getLinkBandwidth(), m_units_remaining[vnet],
                        g_system_ptr->curCycle());

                // Move the message
                m_out[vnet]->enqueue(m_in[vnet]->peekMsgPtr(), m_link_latency);
                m_in[vnet]->dequeue();

                // Count the message
                m_msg_counts[net_msg_ptr->getMessageSize()][vnet]++;

                DPRINTF(RubyNetwork, "%s\n", *m_out[vnet]);
            }

            // Calculate the amount of bandwidth we spent on this message
            int diff = m_units_remaining[vnet] - bw_remaining;
            m_units_remaining[vnet] = max(0, diff);
            bw_remaining = max(0, -diff);
        }

        if (bw_remaining > 0 &&
            (m_in[vnet]->isReady() || m_units_remaining[vnet] > 0) &&
            !m_out[vnet]->areNSlotsAvailable(1)) {
            DPRINTF(RubyNetwork, "vnet: %d", vnet);
            // schedule me to wakeup again because I'm waiting for my
            // output queue to become available
            schedule_wakeup = true;
        }
    }

    // We should only wake up when we use the bandwidth
    // This is only mostly true
    // assert(bw_remaining != getLinkBandwidth());

    // Record that we used some or all of the link bandwidth this cycle
    double ratio = 1.0 - (double(bw_remaining) / double(getLinkBandwidth()));

    // If ratio = 0, we used no bandwidth, if ratio = 1, we used all
    m_link_utilization_proxy += ratio;

    if (bw_remaining > 0 && !schedule_wakeup) {
        // We have extra bandwidth and our output buffer was
        // available, so we must not have anything else to do until
        // another message arrives.
        DPRINTF(RubyNetwork, "%s not scheduled again\n", *this);
    } else {
        DPRINTF(RubyNetwork, "%s scheduled again\n", *this);

        // We are out of bandwidth for this cycle, so wakeup next
        // cycle and continue
        scheduleEvent(Cycles(1));
    }
}

void
Throttle::regStats(string parent)
{
    m_link_utilization
        .name(parent + csprintf(".throttle%i", m_node) + ".link_utilization");

    for (MessageSizeType type = MessageSizeType_FIRST;
         type < MessageSizeType_NUM; ++type) {
        m_msg_counts[(unsigned int)type]
            .init(m_vnets)
            .name(parent + csprintf(".throttle%i", m_node) + ".msg_count." +
                    MessageSizeType_to_string(type))
            .flags(Stats::nozero)
            ;
        m_msg_bytes[(unsigned int) type]
            .name(parent + csprintf(".throttle%i", m_node) + ".msg_bytes." +
                    MessageSizeType_to_string(type))
            .flags(Stats::nozero)
            ;

        m_msg_bytes[(unsigned int) type] = m_msg_counts[type] * Stats::constant(
                Network::MessageSizeType_to_int(type));
    }
}

void
Throttle::clearStats()
{
    m_link_utilization_proxy = 0;
}

void
Throttle::collateStats()
{
    m_link_utilization = 100.0 * m_link_utilization_proxy
        / (double(g_system_ptr->curCycle() - g_ruby_start));
}

void
Throttle::print(ostream& out) const
{
    ccprintf(out,  "[%i bw: %i]", m_node, getLinkBandwidth());
}

int
network_message_to_size(NetworkMessage* net_msg_ptr)
{
    assert(net_msg_ptr != NULL);

    int size = Network::MessageSizeType_to_int(net_msg_ptr->getMessageSize());
    size *=  MESSAGE_SIZE_MULTIPLIER;

    // Artificially increase the size of broadcast messages
    if (BROADCAST_SCALING > 1 && net_msg_ptr->getDestination().isBroadcast())
        size *= BROADCAST_SCALING;

    return size;
}
