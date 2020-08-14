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

#ifndef __MEM_RUBY_NETWORK_SIMPLE_WEIGHTBASEDROUTINGUNIT_HH__
#define __MEM_RUBY_NETWORK_SIMPLE_WEIGHTBASEDROUTINGUNIT_HH__

#include "mem/ruby/network/simple/routing/BaseRoutingUnit.hh"
#include "params/WeightBased.hh"

namespace gem5
{

namespace ruby
{

class WeightBased : public BaseRoutingUnit
{
  public:
    PARAMS(WeightBased);

    WeightBased(const Params &p);

    void addOutPort(LinkID link_id,
                    const std::vector<MessageBuffer*>& m_out_buffer,
                    const NetDest& routing_table_entry,
                    const PortDirection &direction,
                    int link_weight) override;

    void route(const Message &msg,
                int vnet,
                bool deterministic,
                std::vector<RouteInfo> &out_links) override;

  private:

    struct LinkInfo {
        const LinkID m_link_id;
        const NetDest m_routing_entry;
        const std::vector<MessageBuffer*> m_out_buffers;
        int m_order;
        int m_weight;
    };

    std::vector<std::unique_ptr<LinkInfo>> m_links;

    void findRoute(const Message &msg,
                   std::vector<RouteInfo> &out_links) const;

    void sortLinks() {
        std::sort(m_links.begin(), m_links.end(),
            [](const auto &a, const auto &b) {
                auto tup = [](const auto &li)
                { return std::make_tuple(li->m_order,
                                         li->m_weight,
                                         li->m_link_id);};
                return tup(a) < tup(b);
            });
    }
};

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_SIMPLE_WEIGHTBASEDROUTINGUNIT_HH__
