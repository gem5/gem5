/*
 * Copyright (c) 2020-2021 ARM Limited
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

#include "mem/ruby/network/simple/PerfectSwitch.hh"

#include <algorithm>

#include "base/cast.hh"
#include "base/cprintf.hh"
#include "base/random.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/simple/SimpleNetwork.hh"
#include "mem/ruby/network/simple/Switch.hh"
#include "mem/ruby/slicc_interface/Message.hh"

namespace gem5
{

namespace ruby
{

const int PRIORITY_SWITCH_LIMIT = 128;

PerfectSwitch::PerfectSwitch(SwitchID sid, Switch *sw, uint32_t virt_nets)
    : Consumer(sw, Switch::PERFECTSWITCH_EV_PRI),
      m_switch_id(sid),
      m_switch(sw)
{
    m_wakeups_wo_switch = 0;
    m_virtual_networks = virt_nets;
}

void
PerfectSwitch::init(SimpleNetwork *network_ptr)
{
    m_network_ptr = network_ptr;

    for (int i = 0; i < m_virtual_networks; ++i) {
        m_pending_message_count.push_back(0);
    }
}

void
PerfectSwitch::addInPort(const std::vector<MessageBuffer *> &in)
{
    NodeID port = m_in.size();
    m_in.push_back(in);

    for (int i = 0; i < in.size(); ++i) {
        if (in[i] != nullptr) {
            in[i]->setConsumer(this);
            in[i]->setIncomingLink(port);
            in[i]->setVnet(i);
            updatePriorityGroups(i, in[i]);
        }
    }
}

void
PerfectSwitch::updatePriorityGroups(int vnet, MessageBuffer *in_buf)
{
    while (m_in_prio.size() <= vnet) {
        m_in_prio.emplace_back();
        m_in_prio_groups.emplace_back();
    }

    m_in_prio[vnet].push_back(in_buf);

    std::sort(m_in_prio[vnet].begin(), m_in_prio[vnet].end(),
              [](const MessageBuffer *i, const MessageBuffer *j) {
                  return i->routingPriority() < j->routingPriority();
              });

    // reset groups
    m_in_prio_groups[vnet].clear();
    int cur_prio = m_in_prio[vnet].front()->routingPriority();
    m_in_prio_groups[vnet].emplace_back();
    for (auto buf : m_in_prio[vnet]) {
        if (buf->routingPriority() != cur_prio)
            m_in_prio_groups[vnet].emplace_back();
        m_in_prio_groups[vnet].back().push_back(buf);
    }
}

void
PerfectSwitch::addOutPort(const std::vector<MessageBuffer *> &out,
                          const NetDest &routing_table_entry,
                          const PortDirection &dst_inport,
                          Tick routing_latency, int link_weight)
{
    // Add to routing unit
    m_switch->getRoutingUnit().addOutPort(
        m_out.size(), out, routing_table_entry, dst_inport, link_weight);
    m_out.push_back({ routing_latency, out });
}

PerfectSwitch::~PerfectSwitch() {}

MessageBuffer *
PerfectSwitch::inBuffer(int in_port, int vnet) const
{
    if (m_in[in_port].size() <= vnet) {
        return nullptr;
    } else {
        return m_in[in_port][vnet];
    }
}

void
PerfectSwitch::operateVnet(int vnet)
{
    if (m_pending_message_count[vnet] == 0)
        return;

    for (auto &in : m_in_prio_groups[vnet]) {
        // first check the port with the oldest message
        unsigned start_in_port = 0;
        Tick lowest_tick = MaxTick;
        for (int i = 0; i < in.size(); ++i) {
            MessageBuffer *buffer = in[i];
            if (buffer) {
                Tick ready_time = buffer->readyTime();
                if (ready_time < lowest_tick) {
                    lowest_tick = ready_time;
                    start_in_port = i;
                }
            }
        }
        DPRINTF(RubyNetwork,
                "vnet %d: %d pending msgs. "
                "Checking port %d first\n",
                vnet, m_pending_message_count[vnet], start_in_port);
        // check all ports starting with the one with the oldest message
        for (int i = 0; i < in.size(); ++i) {
            int in_port = (i + start_in_port) % in.size();
            MessageBuffer *buffer = in[in_port];
            if (buffer)
                operateMessageBuffer(buffer, vnet);
        }
    }
}

void
PerfectSwitch::operateMessageBuffer(MessageBuffer *buffer, int vnet)
{
    MsgPtr msg_ptr;
    Message *net_msg_ptr = NULL;

    // temporary vectors to store the routing results
    static thread_local std::vector<BaseRoutingUnit::RouteInfo> output_links;

    Tick current_time = m_switch->clockEdge();

    while (buffer->isReady(current_time)) {
        DPRINTF(RubyNetwork, "incoming: %d\n", buffer->getIncomingLink());

        // Peek at message
        msg_ptr = buffer->peekMsgPtr();
        net_msg_ptr = msg_ptr.get();
        DPRINTF(RubyNetwork, "Message: %s\n", (*net_msg_ptr));

        output_links.clear();
        m_switch->getRoutingUnit().route(*net_msg_ptr, vnet,
                                         m_network_ptr->isVNetOrdered(vnet),
                                         output_links);

        // Check for resources - for all outgoing queues
        bool enough = true;
        for (int i = 0; i < output_links.size(); i++) {
            int outgoing = output_links[i].m_link_id;
            OutputPort &out_port = m_out[outgoing];

            if (!out_port.buffers[vnet]->areNSlotsAvailable(1, current_time))
                enough = false;

            DPRINTF(RubyNetwork,
                    "Checking if node is blocked ..."
                    "outgoing: %d, vnet: %d, enough: %d\n",
                    outgoing, vnet, enough);
        }

        // There were not enough resources
        if (!enough) {
            scheduleEvent(Cycles(1));
            DPRINTF(RubyNetwork, "Can't deliver message since a node "
                                 "is blocked\n");
            DPRINTF(RubyNetwork, "Message: %s\n", (*net_msg_ptr));
            break; // go to next incoming port
        }

        MsgPtr unmodified_msg_ptr;

        if (output_links.size() > 1) {
            // If we are sending this message down more than one link
            // (size>1), we need to make a copy of the message so each
            // branch can have a different internal destination we need
            // to create an unmodified MsgPtr because the MessageBuffer
            // enqueue func will modify the message

            // This magic line creates a private copy of the message
            unmodified_msg_ptr = msg_ptr->clone();
        }

        // Dequeue msg
        buffer->dequeue(current_time);
        m_pending_message_count[vnet]--;

        // Enqueue it - for all outgoing queues
        for (int i = 0; i < output_links.size(); i++) {
            int outgoing = output_links[i].m_link_id;
            OutputPort &out_port = m_out[outgoing];

            if (i > 0) {
                // create a private copy of the unmodified message
                msg_ptr = unmodified_msg_ptr->clone();
            }

            // Change the internal destination set of the message so it
            // knows which destinations this link is responsible for.
            net_msg_ptr = msg_ptr.get();
            net_msg_ptr->getDestination() = output_links[i].m_destinations;

            // Enqeue msg
            DPRINTF(RubyNetwork,
                    "Enqueuing net msg from "
                    "inport[%d][%d] to outport [%d][%d].\n",
                    buffer->getIncomingLink(), vnet, outgoing, vnet);

            out_port.buffers[vnet]->enqueue(msg_ptr, current_time,
                                            out_port.latency);
        }
    }
}

void
PerfectSwitch::wakeup()
{
    // Give the highest numbered link priority most of the time
    m_wakeups_wo_switch++;
    int highest_prio_vnet = m_virtual_networks - 1;
    int lowest_prio_vnet = 0;
    int decrementer = 1;

    // invert priorities to avoid starvation seen in the component network
    if (m_wakeups_wo_switch > PRIORITY_SWITCH_LIMIT) {
        m_wakeups_wo_switch = 0;
        highest_prio_vnet = 0;
        lowest_prio_vnet = m_virtual_networks - 1;
        decrementer = -1;
    }

    // For all components incoming queues
    for (int vnet = highest_prio_vnet;
         (vnet * decrementer) >= (decrementer * lowest_prio_vnet);
         vnet -= decrementer) {
        operateVnet(vnet);
    }
}

void
PerfectSwitch::storeEventInfo(int info)
{
    m_pending_message_count[info]++;
}

void
PerfectSwitch::clearStats()
{}

void
PerfectSwitch::collateStats()
{}

void
PerfectSwitch::print(std::ostream &out) const
{
    out << "[PerfectSwitch " << m_switch_id << "]";
}

} // namespace ruby
} // namespace gem5
