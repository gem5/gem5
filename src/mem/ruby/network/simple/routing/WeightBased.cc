/*
 * Copyright (c) 2021 ARM Limited
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

#include "mem/ruby/network/simple/routing/WeightBased.hh"

#include <algorithm>

#include "base/random.hh"
#include "mem/ruby/network/simple/Switch.hh"

namespace gem5
{

namespace ruby
{

WeightBased::WeightBased(const Params &p)
    :BaseRoutingUnit(p)
{
}

void
WeightBased::addOutPort(LinkID link_id,
                    const std::vector<MessageBuffer*>& m_out_buffer,
                    const NetDest& routing_table_entry,
                    const PortDirection &direction,
                    int link_weight)
{
    gem5_assert(link_id == m_links.size());
    m_links.emplace_back(new LinkInfo{link_id,
                        routing_table_entry,
                        m_out_buffer,
                        0, link_weight});
    sortLinks();
}

void
WeightBased::route(const Message &msg,
                   int vnet,
                   bool deterministic,
                   std::vector<RouteInfo> &out_links)
{
    // Makes sure ordering was reset adaptive option was set
    if (params().adaptive_routing) {
        if (deterministic) {
            // Don't adaptively route
            // Makes sure ordering is reset
            for (auto &link : m_links)
                link->m_order = 0;
        } else {
            // Find how clogged each link is
            for (auto &link : m_links) {
                int out_queue_length = 0;
                Tick current_time = m_parent_switch->clockEdge();
                for (auto buffer : link->m_out_buffers) {
                    out_queue_length += buffer->getSize(current_time);
                }
                // improve load distribution by randomizing order of links
                // with the same queue length
                link->m_order =
                    (out_queue_length << 8) | random_mt.random(0, 0xff);
            }
        }
        sortLinks();
    }

    findRoute(msg, out_links);
}

void
WeightBased::findRoute(const Message &msg,
                       std::vector<RouteInfo> &out_links) const
{
    NetDest msg_dsts = msg.getDestination();
    assert(out_links.size() == 0);
    for (auto &link : m_links) {
        const NetDest &dst = link->m_routing_entry;
        if (msg_dsts.intersectionIsNotEmpty(dst)) {
            // Need to remember which destinations need this message in
            // another vector.  This Set is the intersection of the
            // routing_table entry and the current destination set.
            out_links.emplace_back(msg_dsts.AND(dst), link->m_link_id);

            // Next, we update the msg_destination not to include
            // those nodes that were already handled by this link
            msg_dsts.removeNetDest(dst);
        }
    }

    gem5_assert(msg_dsts.count() == 0);
}

} // namespace ruby
} // namespace gem5
