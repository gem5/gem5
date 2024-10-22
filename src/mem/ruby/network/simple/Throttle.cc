/*
 * Copyright (c) 2021 ARM Limited
 * All rights reserved
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
#include "sim/stats.hh"

namespace gem5
{

namespace ruby
{

const int MESSAGE_SIZE_MULTIPLIER = 1000;
//const int BROADCAST_SCALING = 4; // Have a 16p system act like a 64p systems
const int BROADCAST_SCALING = 1;
const int PRIORITY_SWITCH_LIMIT = 128;

static int network_message_to_size(Message* net_msg_ptr);

Throttle::Throttle(int sID, RubySystem *rs, NodeID node, Cycles link_latency,
                   int endpoint_bandwidth, Switch *em)
    : Consumer(em,  Switch::THROTTLE_EV_PRI),
      m_switch_id(sID), m_switch(em), m_node(node),
      m_physical_vnets(false), m_ruby_system(rs),
      throttleStats(em, node)
{
    m_vnets = 0;

    m_link_latency = link_latency;
    m_endpoint_bandwidth = endpoint_bandwidth;

    m_wakeups_wo_switch = 0;
}

Throttle::Throttle(int sID, RubySystem *rs, NodeID node, Cycles link_latency,
                   int link_bandwidth_multiplier, int endpoint_bandwidth,
                   Switch *em)
    : Throttle(sID, rs, node, link_latency, endpoint_bandwidth, em)
{
    gem5_assert(link_bandwidth_multiplier > 0);
    m_link_bandwidth_multiplier.push_back(link_bandwidth_multiplier);
}

Throttle::Throttle(int sID, RubySystem *rs, NodeID node, Cycles link_latency,
                   const std::vector<int> &vnet_channels,
                   const std::vector<int> &vnet_bandwidth_multiplier,
                   int endpoint_bandwidth, Switch *em)
    : Throttle(sID, rs, node, link_latency, endpoint_bandwidth, em)
{
    m_physical_vnets = true;
    for (auto link_bandwidth_multiplier : vnet_bandwidth_multiplier){
        gem5_assert(link_bandwidth_multiplier > 0);
        m_link_bandwidth_multiplier.push_back(link_bandwidth_multiplier);
    }
    for (auto channels : vnet_channels){
        gem5_assert(channels > 0);
        m_vnet_channels.push_back(channels);
    }
    gem5_assert(m_link_bandwidth_multiplier.size() == m_vnet_channels.size());
}

void
Throttle::addLinks(const std::vector<MessageBuffer*>& in_vec,
                   const std::vector<MessageBuffer*>& out_vec)
{
    assert(in_vec.size() == out_vec.size());

    for (int vnet = 0; vnet < in_vec.size(); ++vnet) {
        MessageBuffer *in_ptr = in_vec[vnet];
        MessageBuffer *out_ptr = out_vec[vnet];

        m_units_remaining.emplace_back(getChannelCnt(vnet),0);
        m_in.push_back(in_ptr);
        m_out.push_back(out_ptr);

        // Set consumer and description
        in_ptr->setConsumer(this);
        std::string desc = "[Queue to Throttle " +
            std::to_string(m_switch_id) + " " + std::to_string(m_node) + "]";
    }

    m_vnets = in_vec.size();

    gem5_assert(m_physical_vnets ?
           (m_link_bandwidth_multiplier.size() == m_vnets) :
           (m_link_bandwidth_multiplier.size() == 1));
}

int
Throttle::getLinkBandwidth(int vnet) const
{
    int bw = m_physical_vnets ?
                m_link_bandwidth_multiplier[vnet] :
                m_link_bandwidth_multiplier[0];
    gem5_assert(bw > 0);
    return m_endpoint_bandwidth * bw;
}

int
Throttle::getTotalLinkBandwidth() const
{
    int sum = getLinkBandwidth(0) * getChannelCnt(0);
    if (m_physical_vnets) {
        for (unsigned i = 1; i < m_vnets; ++i)
            sum += getLinkBandwidth(i) * getChannelCnt(i);
    }
    return sum;
}

int
Throttle::getChannelCnt(int vnet) const
{
    return m_physical_vnets ? m_vnet_channels[vnet] : 1;
}

void
Throttle::operateVnet(int vnet, int channel, int &total_bw_remaining,
                      bool &bw_saturated, bool &output_blocked,
                      MessageBuffer *in, MessageBuffer *out)
{
    if (out == nullptr || in == nullptr) {
        return;
    }

    int &units_remaining = m_units_remaining[vnet][channel];

    gem5_assert(units_remaining >= 0);
    Tick current_time = m_switch->clockEdge();

    int bw_remaining = m_physical_vnets ?
                getLinkBandwidth(vnet) : total_bw_remaining;

    auto hasPendingWork = [&]{ return in->isReady(current_time) ||
                                      units_remaining > 0; };
    while ((bw_remaining > 0) && hasPendingWork() &&
           out->areNSlotsAvailable(1, current_time)) {
        // See if we are done transferring the previous message on
        // this virtual network
        if (units_remaining == 0 && in->isReady(current_time)) {
            // Find the size of the message we are moving
            MsgPtr msg_ptr = in->peekMsgPtr();
            Message *net_msg_ptr = msg_ptr.get();
            Tick msg_enqueue_time = msg_ptr->getLastEnqueueTime();
            units_remaining = network_message_to_size(net_msg_ptr);

            DPRINTF(RubyNetwork, "throttle: %d my bw %d bw spent "
                    "enqueueing net msg %d time: %lld.\n",
                    m_node, getLinkBandwidth(vnet), units_remaining,
                    m_ruby_system->curCycle());

            // Move the message
            in->dequeue(current_time);
            out->enqueue(msg_ptr, current_time,
                         m_switch->cyclesToTicks(m_link_latency),
                         m_ruby_system->getRandomization(),
                         m_ruby_system->getWarmupEnabled());

            // Count the message
            (*(throttleStats.
                msg_counts[net_msg_ptr->getMessageSize()]))[vnet]++;
            throttleStats.total_msg_count += 1;
            uint32_t total_size =
                Network::MessageSizeType_to_int(net_msg_ptr->getMessageSize());
            throttleStats.total_msg_bytes += total_size;
            total_size -=
                Network::MessageSizeType_to_int(MessageSizeType_Control);
            throttleStats.total_data_msg_bytes += total_size;
            throttleStats.total_msg_wait_time +=
                current_time - msg_enqueue_time;
            DPRINTF(RubyNetwork, "%s\n", *out);
        }

        // Calculate the amount of bandwidth we spent on this message
        int spent = std::min(units_remaining, bw_remaining);
        units_remaining -= spent;
        bw_remaining -= spent;
        total_bw_remaining -= spent;
    }

    gem5_assert(units_remaining >= 0);
    gem5_assert(bw_remaining >= 0);
    gem5_assert(total_bw_remaining >= 0);

    // Notify caller if
    //  - we ran out of bandwith and still have stuff to do
    //  - we had something to do but output queue was unavailable
    if (hasPendingWork()) {
        gem5_assert((bw_remaining == 0) ||
                    !out->areNSlotsAvailable(1, current_time));
        bw_saturated = bw_saturated || (bw_remaining == 0);
        output_blocked = output_blocked ||
            !out->areNSlotsAvailable(1, current_time);
    }
}

void
Throttle::wakeup()
{
    // Limits the number of message sent to a limited number of bytes/cycle.
    assert(getTotalLinkBandwidth() > 0);
    int bw_remaining = getTotalLinkBandwidth();

    m_wakeups_wo_switch++;
    bool bw_saturated = false;
    bool output_blocked = false;

    // variable for deciding the direction in which to iterate
    bool iteration_direction = false;


    // invert priorities to avoid starvation seen in the component network
    if (m_wakeups_wo_switch > PRIORITY_SWITCH_LIMIT) {
        m_wakeups_wo_switch = 0;
        iteration_direction = true;
    }

    if (iteration_direction) {
        for (int vnet = 0; vnet < m_vnets; ++vnet) {
            for (int channel = 0; channel < getChannelCnt(vnet); ++channel) {
                operateVnet(vnet, channel, bw_remaining,
                            bw_saturated, output_blocked,
                            m_in[vnet], m_out[vnet]);
            }
        }
    } else {
        for (int vnet = m_vnets-1; vnet >= 0; --vnet) {
            for (int channel = 0; channel < getChannelCnt(vnet); ++channel) {
                operateVnet(vnet, channel, bw_remaining,
                            bw_saturated, output_blocked,
                            m_in[vnet], m_out[vnet]);
            }
        }
    }

    // We should only wake up when we use the bandwidth
    // This is only mostly true
    // assert(bw_remaining != getLinkBandwidth());

    // Record that we used some or all of the link bandwidth this cycle
    double ratio = 1.0 - (double(bw_remaining) /
                         double(getTotalLinkBandwidth()));

    // If ratio = 0, we used no bandwidth, if ratio = 1, we used all
    throttleStats.acc_link_utilization += ratio;

    if (bw_saturated) throttleStats.total_bw_sat_cy += 1;
    if (output_blocked) throttleStats.total_stall_cy += 1;

    if (bw_saturated || output_blocked) {
        // We are out of bandwidth for this cycle, so wakeup next
        // cycle and continue
        DPRINTF(RubyNetwork, "%s scheduled again\n", *this);
        scheduleEvent(Cycles(1));
    }
}

void
Throttle::print(std::ostream& out) const
{
    ccprintf(out,  "[%i bw:", m_node);
    if (m_physical_vnets) {
        for (unsigned i = 0; i < m_vnets; ++i)
            ccprintf(out,  " vnet%d=%i", i, getLinkBandwidth(i));
    } else {
        ccprintf(out,  " %i", getTotalLinkBandwidth());
    }
    ccprintf(out,  "]");
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

Throttle::
ThrottleStats::ThrottleStats(Switch *parent, const NodeID &nodeID)
    : statistics::Group(parent, csprintf("throttle%02i", nodeID).c_str()),
      ADD_STAT(acc_link_utilization, statistics::units::Count::get(),
        "Accumulated link utilization"),
      ADD_STAT(link_utilization, statistics::units::Ratio::get(),
        "Average link utilization"),
      ADD_STAT(total_msg_count, statistics::units::Count::get(),
        "Total number of messages forwarded by this switch"),
      ADD_STAT(total_msg_bytes, statistics::units::Byte::get(),
        "Total number of bytes forwarded by this switch"),
      ADD_STAT(total_data_msg_bytes, statistics::units::Byte::get(),
        "Total number of data bytes forwarded by this switch"),
      ADD_STAT(total_msg_wait_time, statistics::units::Tick::get(),
        "Total time spend forwarding messages"),
      ADD_STAT(total_stall_cy, statistics::units::Cycle::get(),
        "Total time spent blocked on any output link"),
      ADD_STAT(total_bw_sat_cy, statistics::units::Cycle::get(),
        "Total time bandwidth was saturated on any output link"),
      ADD_STAT(avg_msg_wait_time, statistics::units::Ratio::get(),
        "Average time a message took to be forwarded"),
      ADD_STAT(avg_bandwidth, statistics::units::Ratio::get(),
        "Average bandwidth (GB/s)"),
      ADD_STAT(avg_useful_bandwidth, statistics::units::Ratio::get(),
        "Average usefull (only data) bandwidth (GB/s)")
{
    link_utilization = 100 * acc_link_utilization /
                        (simTicks / parent->clockPeriod());

    avg_msg_wait_time = total_msg_wait_time / total_msg_count;

    avg_bandwidth.precision(2);
    avg_bandwidth = (total_msg_bytes / simSeconds) /
                      statistics::constant(1024*1024*1024);

    avg_useful_bandwidth.precision(2);
    avg_useful_bandwidth = (total_data_msg_bytes / simSeconds) /
                             statistics::constant(1024*1024*1024);

    for (MessageSizeType type = MessageSizeType_FIRST;
         type < MessageSizeType_NUM; ++type) {
        msg_counts[(unsigned int)type] =
            new statistics::Vector(this,
            csprintf("msg_count.%s", MessageSizeType_to_string(type)).c_str());
        msg_counts[(unsigned int)type]
            ->init(Network::getNumberOfVirtualNetworks())
            .flags(statistics::nozero)
            ;

        msg_bytes[(unsigned int) type] =
            new statistics::Formula(this,
            csprintf("msg_bytes.%s", MessageSizeType_to_string(type)).c_str());
        msg_bytes[(unsigned int) type]
            ->flags(statistics::nozero)
            ;

        *(msg_bytes[(unsigned int) type]) =
            *(msg_counts[type]) * statistics::constant(
                Network::MessageSizeType_to_int(type));
    }
}

} // namespace ruby
} // namespace gem5
