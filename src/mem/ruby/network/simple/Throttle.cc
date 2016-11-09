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

#include "mem/ruby/network/simple/Throttle.hh"

#include <cassert>

#include "base/cast.hh"
#include "base/cprintf.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/network/simple/Switch.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "mem/ruby/system/RubySystem.hh"

using namespace std;

const int MESSAGE_SIZE_MULTIPLIER = 1000;
//const int BROADCAST_SCALING = 4; // Have a 16p system act like a 64p systems
const int BROADCAST_SCALING = 1;
const int PRIORITY_SWITCH_LIMIT = 128;

static int network_message_to_size(Message* net_msg_ptr);

Throttle::Throttle(int sID, RubySystem *rs, NodeID node, Cycles link_latency,
                   int link_bandwidth_multiplier, int endpoint_bandwidth,
                   Switch *em)
    : Consumer(em), m_switch_id(sID), m_switch(em), m_node(node),
      m_ruby_system(rs)
{
    m_vnets = 0;

    assert(link_bandwidth_multiplier > 0);
    m_link_bandwidth_multiplier = link_bandwidth_multiplier;

    m_link_latency = link_latency;
    m_endpoint_bandwidth = endpoint_bandwidth;

    m_wakeups_wo_switch = 0;
    m_link_utilization_proxy = 0;
}

void
Throttle::addLinks(const vector<MessageBuffer*>& in_vec,
                   const vector<MessageBuffer*>& out_vec)
{
    assert(in_vec.size() == out_vec.size());

    for (int vnet = 0; vnet < in_vec.size(); ++vnet) {
        MessageBuffer *in_ptr = in_vec[vnet];
        MessageBuffer *out_ptr = out_vec[vnet];

        m_vnets++;
        m_units_remaining.push_back(0);
        m_in.push_back(in_ptr);
        m_out.push_back(out_ptr);

        // Set consumer and description
        in_ptr->setConsumer(this);
        string desc = "[Queue to Throttle " + to_string(m_switch_id) + " " +
            to_string(m_node) + "]";
    }
}

void
Throttle::operateVnet(int vnet, int &bw_remaining, bool &schedule_wakeup,
                      MessageBuffer *in, MessageBuffer *out)
{
    if (out == nullptr || in == nullptr) {
        return;
    }

    assert(m_units_remaining[vnet] >= 0);
    Tick current_time = m_switch->clockEdge();

    while (bw_remaining > 0 && (in->isReady(current_time) ||
                                m_units_remaining[vnet] > 0) &&
           out->areNSlotsAvailable(1, current_time)) {
        // See if we are done transferring the previous message on
        // this virtual network
        if (m_units_remaining[vnet] == 0 && in->isReady(current_time)) {
            // Find the size of the message we are moving
            MsgPtr msg_ptr = in->peekMsgPtr();
            Message *net_msg_ptr = msg_ptr.get();
            m_units_remaining[vnet] +=
                network_message_to_size(net_msg_ptr);

            DPRINTF(RubyNetwork, "throttle: %d my bw %d bw spent "
                    "enqueueing net msg %d time: %lld.\n",
                    m_node, getLinkBandwidth(), m_units_remaining[vnet],
                    m_ruby_system->curCycle());

            // Move the message
            in->dequeue(current_time);
            out->enqueue(msg_ptr, current_time,
                         m_switch->cyclesToTicks(m_link_latency));

            // Count the message
            m_msg_counts[net_msg_ptr->getMessageSize()][vnet]++;
            DPRINTF(RubyNetwork, "%s\n", *out);
        }

        // Calculate the amount of bandwidth we spent on this message
        int diff = m_units_remaining[vnet] - bw_remaining;
        m_units_remaining[vnet] = max(0, diff);
        bw_remaining = max(0, -diff);
    }

    if (bw_remaining > 0 && (in->isReady(current_time) ||
                             m_units_remaining[vnet] > 0) &&
        !out->areNSlotsAvailable(1, current_time)) {
        DPRINTF(RubyNetwork, "vnet: %d", vnet);

        // schedule me to wakeup again because I'm waiting for my
        // output queue to become available
        schedule_wakeup = true;
    }
}

void
Throttle::wakeup()
{
    // Limits the number of message sent to a limited number of bytes/cycle.
    assert(getLinkBandwidth() > 0);
    int bw_remaining = getLinkBandwidth();

    m_wakeups_wo_switch++;
    bool schedule_wakeup = false;

    // variable for deciding the direction in which to iterate
    bool iteration_direction = false;


    // invert priorities to avoid starvation seen in the component network
    if (m_wakeups_wo_switch > PRIORITY_SWITCH_LIMIT) {
        m_wakeups_wo_switch = 0;
        iteration_direction = true;
    }

    if (iteration_direction) {
        for (int vnet = 0; vnet < m_vnets; ++vnet) {
            operateVnet(vnet, bw_remaining, schedule_wakeup,
                        m_in[vnet], m_out[vnet]);
        }
    } else {
        for (int vnet = m_vnets-1; vnet >= 0; --vnet) {
            operateVnet(vnet, bw_remaining, schedule_wakeup,
                        m_in[vnet], m_out[vnet]);
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
            .init(Network::getNumberOfVirtualNetworks())
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
    double time_delta = double(m_ruby_system->curCycle() -
                               m_ruby_system->getStartCycle());

    m_link_utilization = 100.0 * m_link_utilization_proxy / time_delta;
}

void
Throttle::print(ostream& out) const
{
    ccprintf(out,  "[%i bw: %i]", m_node, getLinkBandwidth());
}

int
network_message_to_size(Message *net_msg_ptr)
{
    assert(net_msg_ptr != NULL);

    int size = Network::MessageSizeType_to_int(net_msg_ptr->getMessageSize());
    size *=  MESSAGE_SIZE_MULTIPLIER;

    // Artificially increase the size of broadcast messages
    if (BROADCAST_SCALING > 1 && net_msg_ptr->getDestination().isBroadcast())
        size *= BROADCAST_SCALING;

    return size;
}
