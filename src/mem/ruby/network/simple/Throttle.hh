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
 * The class to implement bandwidth and latency throttle. An instance
 * of consumer class that can be woke up. It is only used to control
 * bandwidth at output port of a switch. And the throttle is added
 * *after* the output port, means the message is put in the output
 * port of the PerfectSwitch (a intermediateBuffers) first, then go
 * through the Throttle.
 */

#ifndef __MEM_RUBY_NETWORK_SIMPLE_THROTTLE_HH__
#define __MEM_RUBY_NETWORK_SIMPLE_THROTTLE_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/system/RubySystem.hh"

class MessageBuffer;
class Switch;

class Throttle : public Consumer
{
  public:
    Throttle(int sID, RubySystem *rs, NodeID node, Cycles link_latency,
             int link_bandwidth_multiplier, int endpoint_bandwidth,
             Switch *em);
    ~Throttle() {}

    std::string name()
    { return csprintf("Throttle-%i", m_switch_id); }

    void addLinks(const std::vector<MessageBuffer*>& in_vec,
                  const std::vector<MessageBuffer*>& out_vec);
    void wakeup();

    // The average utilization (a fraction) since last clearStats()
    const Stats::Scalar & getUtilization() const
    { return m_link_utilization; }
    const Stats::Vector & getMsgCount(unsigned int type) const
    { return m_msg_counts[type]; }

    int getLinkBandwidth() const
    { return m_endpoint_bandwidth * m_link_bandwidth_multiplier; }

    Cycles getLatency() const { return m_link_latency; }

    void clearStats();
    void collateStats();
    void regStats(std::string name);
    void print(std::ostream& out) const;

  private:
    void init(NodeID node, Cycles link_latency, int link_bandwidth_multiplier,
              int endpoint_bandwidth);
    void operateVnet(int vnet, int &bw_remainin, bool &schedule_wakeup,
                     MessageBuffer *in, MessageBuffer *out);

    // Private copy constructor and assignment operator
    Throttle(const Throttle& obj);
    Throttle& operator=(const Throttle& obj);

    std::vector<MessageBuffer*> m_in;
    std::vector<MessageBuffer*> m_out;
    unsigned int m_vnets;
    std::vector<int> m_units_remaining;

    const int m_switch_id;
    Switch *m_switch;
    NodeID m_node;

    int m_link_bandwidth_multiplier;
    Cycles m_link_latency;
    int m_wakeups_wo_switch;
    int m_endpoint_bandwidth;
    RubySystem *m_ruby_system;

    // Statistical variables
    Stats::Scalar m_link_utilization;
    Stats::Vector m_msg_counts[MessageSizeType_NUM];
    Stats::Formula m_msg_bytes[MessageSizeType_NUM];

    double m_link_utilization_proxy;
};

inline std::ostream&
operator<<(std::ostream& out, const Throttle& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_SIMPLE_THROTTLE_HH__
