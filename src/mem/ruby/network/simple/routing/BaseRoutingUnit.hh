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

#ifndef __MEM_RUBY_NETWORK_SIMPLE_BASEROUTINGUNIT_HH__
#define __MEM_RUBY_NETWORK_SIMPLE_BASEROUTINGUNIT_HH__

#include <vector>

#include "mem/ruby/network/Network.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "params/BaseRoutingUnit.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace ruby
{

class Switch;

class BaseRoutingUnit : public SimObject
{
  public:
    PARAMS(BaseRoutingUnit);

    BaseRoutingUnit(const Params &p)
      :SimObject(p)
    {
    }

    virtual void addOutPort(LinkID link_id,
                           const std::vector<MessageBuffer*>& m_out_buffer,
                           const NetDest& routing_table_entry,
                           const PortDirection &direction,
                           int link_weight) = 0;

    struct RouteInfo
    {
        RouteInfo(const NetDest &dests, const LinkID link_id)
          :m_destinations(dests), m_link_id(link_id)
        {}
        const NetDest m_destinations;
        const LinkID m_link_id;
    };

    virtual void route(const Message &msg,
                       int vnet,
                       bool deterministic,
                       std::vector<RouteInfo> &out_links) = 0;

    void init_parent(Switch *parent_switch)
    { m_parent_switch = parent_switch; }

  protected:

    Switch *m_parent_switch;

};

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_SIMPLE_BASEROUTINGUNIT_HH__
