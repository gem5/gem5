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

/*
 * Perfect switch, of course it is perfect and no latency or what so
 * ever. Every cycle it is woke up and perform all the necessary
 * routings that must be done. Note, this switch also has number of
 * input ports/output ports and has a routing table as well.
 */

#ifndef __MEM_RUBY_NETWORK_SIMPLE_PERFECTSWITCH_HH__
#define __MEM_RUBY_NETWORK_SIMPLE_PERFECTSWITCH_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/TypeDefines.hh"

class MessageBuffer;
class NetDest;
class SimpleNetwork;
class Switch;

struct LinkOrder
{
    int m_link;
    int m_value;
};

bool operator<(const LinkOrder& l1, const LinkOrder& l2);

class PerfectSwitch : public Consumer
{
  public:
    PerfectSwitch(SwitchID sid, Switch *, uint32_t);
    ~PerfectSwitch();

    std::string name()
    { return csprintf("PerfectSwitch-%i", m_switch_id); }

    void init(SimpleNetwork *);
    void addInPort(const std::vector<MessageBuffer*>& in);
    void addOutPort(const std::vector<MessageBuffer*>& out,
                    const NetDest& routing_table_entry);

    int getInLinks() const { return m_in.size(); }
    int getOutLinks() const { return m_out.size(); }

    void wakeup();
    void storeEventInfo(int info);

    void clearStats();
    void collateStats();
    void print(std::ostream& out) const;

  private:
    // Private copy constructor and assignment operator
    PerfectSwitch(const PerfectSwitch& obj);
    PerfectSwitch& operator=(const PerfectSwitch& obj);

    void operateVnet(int vnet);
    void operateMessageBuffer(MessageBuffer *b, int incoming, int vnet);

    const SwitchID m_switch_id;
    Switch * const m_switch;

    // vector of queues from the components
    std::vector<std::vector<MessageBuffer*> > m_in;
    std::vector<std::vector<MessageBuffer*> > m_out;

    std::vector<NetDest> m_routing_table;
    std::vector<LinkOrder> m_link_order;

    uint32_t m_virtual_networks;
    int m_round_robin_start;
    int m_wakeups_wo_switch;

    SimpleNetwork* m_network_ptr;
    std::vector<int> m_pending_message_count;
};

inline std::ostream&
operator<<(std::ostream& out, const PerfectSwitch& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_SIMPLE_PERFECTSWITCH_HH__
