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
 * Copyright (c) 2020 Inria
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

/*
 * The actual modelled switch. It use the perfect switch and a
 * Throttle object to control and bandwidth and timing *only for the
 * output port*. So here we have un-realistic modelling, since the
 * order of PerfectSwitch and Throttle objects get woke up affect the
 * message timing. A more accurate model would be having two set of
 * system states, one for this cycle, one for next cycle. And on the
 * cycle boundary swap the two set of states.
 */

#ifndef __MEM_RUBY_NETWORK_SIMPLE_SWITCH_HH__
#define __MEM_RUBY_NETWORK_SIMPLE_SWITCH_HH__

#include <iostream>
#include <list>
#include <vector>

#include "mem/packet.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "mem/ruby/network/simple/PerfectSwitch.hh"
#include "mem/ruby/network/simple/Throttle.hh"
#include "mem/ruby/network/simple/routing/BaseRoutingUnit.hh"
#include "mem/ruby/protocol/MessageSizeType.hh"
#include "params/Switch.hh"

namespace gem5
{

namespace ruby
{

class MessageBuffer;
class NetDest;
class SimpleNetwork;

class Switch : public BasicRouter
{
  public:

    // Makes sure throttle sends messages to the links after the switch is
    // done forwarding the messages in the same cycle
    static constexpr Event::Priority PERFECTSWITCH_EV_PRI = Event::Default_Pri;
    static constexpr Event::Priority THROTTLE_EV_PRI = Event::Default_Pri + 1;

    typedef SwitchParams Params;
    Switch(const Params &p);
    ~Switch() = default;
    void init();

    void addInPort(const std::vector<MessageBuffer*>& in);
    void addOutPort(const std::vector<MessageBuffer*>& out,
                    const NetDest& routing_table_entry,
                    Cycles link_latency, int link_weight, int bw_multiplier,
                    bool is_external,
                    PortDirection dst_inport = "");

    void resetStats();
    void collateStats();
    void regStats();
    const statistics::Formula & getMsgCount(unsigned int type) const
    { return *(switchStats.m_msg_counts[type]); }

    void print(std::ostream& out) const;
    void init_net_ptr(SimpleNetwork* net_ptr) { m_network_ptr = net_ptr; }

    bool functionalRead(Packet *);
    bool functionalRead(Packet *, WriteMask&);
    uint32_t functionalWrite(Packet *);

    BaseRoutingUnit& getRoutingUnit() { return m_routing_unit; }

  private:
    // Private copy constructor and assignment operator
    Switch(const Switch& obj);
    Switch& operator=(const Switch& obj);

    PerfectSwitch perfectSwitch;
    SimpleNetwork* m_network_ptr;
    std::list<Throttle> throttles;

    const Cycles m_int_routing_latency;
    const Cycles m_ext_routing_latency;

    BaseRoutingUnit &m_routing_unit;

    unsigned m_num_connected_buffers;
    std::vector<MessageBuffer*> m_port_buffers;


  public:
    struct SwitchStats : public statistics::Group
    {
        SwitchStats(statistics::Group *parent);

        // Statistical variables
        statistics::Formula m_avg_utilization;
        statistics::Formula* m_msg_counts[MessageSizeType_NUM];
        statistics::Formula* m_msg_bytes[MessageSizeType_NUM];
    } switchStats;
};

inline std::ostream&
operator<<(std::ostream& out, const Switch& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_SIMPLE_SWITCH_HH__
