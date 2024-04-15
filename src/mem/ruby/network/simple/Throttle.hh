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

namespace gem5
{

namespace ruby
{

class MessageBuffer;
class Switch;

class Throttle : public Consumer
{
  private:
    Throttle(int sID, RubySystem *rs, NodeID node, Cycles link_latency,
             int endpoint_bandwidth, Switch *em);

  public:
    Throttle(int sID, RubySystem *rs, NodeID node, Cycles link_latency,
             int link_bandwidth_multiplier, int endpoint_bandwidth,
             Switch *em);
    Throttle(int sID, RubySystem *rs, NodeID node, Cycles link_latency,
             const std::vector<int> &vnet_channels,
             const std::vector<int> &vnet_bandwidth_multiplier,
             int endpoint_bandwidth, Switch *em);

    ~Throttle() {}

    std::string
    name()
    {
        return csprintf("Throttle-%i", m_switch_id);
    }

    void addLinks(const std::vector<MessageBuffer *> &in_vec,
                  const std::vector<MessageBuffer *> &out_vec);
    void wakeup();

    // The average utilization (a fraction) since last clearStats()
    const statistics::Formula &
    getUtilization() const
    {
        return throttleStats.link_utilization;
    }

    const statistics::Vector &
    getMsgCount(unsigned int type) const
    {
        return *(throttleStats.msg_counts[type]);
    }

    int getLinkBandwidth(int vnet) const;

    int getTotalLinkBandwidth() const;

    int getChannelCnt(int vnet) const;

    Cycles
    getLatency() const
    {
        return m_link_latency;
    }

    void print(std::ostream &out) const;

  private:
    void init(NodeID node, Cycles link_latency, int link_bandwidth_multiplier,
              int endpoint_bandwidth);
    void operateVnet(int vnet, int channel, int &total_bw_remaining,
                     bool &bw_saturated, bool &output_blocked,
                     MessageBuffer *in, MessageBuffer *out);

    // Private copy constructor and assignment operator
    Throttle(const Throttle &obj);
    Throttle &operator=(const Throttle &obj);

    std::vector<MessageBuffer *> m_in;
    std::vector<MessageBuffer *> m_out;
    unsigned int m_vnets;
    std::vector<std::vector<int>> m_units_remaining;

    const int m_switch_id;
    Switch *m_switch;
    NodeID m_node;

    bool m_physical_vnets;
    std::vector<int> m_link_bandwidth_multiplier;
    std::vector<int> m_vnet_channels;
    Cycles m_link_latency;
    int m_wakeups_wo_switch;
    int m_endpoint_bandwidth;
    RubySystem *m_ruby_system;

    struct ThrottleStats : public statistics::Group
    {
        ThrottleStats(Switch *parent, const NodeID &nodeID);

        // Statistical variables
        statistics::Scalar acc_link_utilization;
        statistics::Formula link_utilization;
        statistics::Vector *msg_counts[MessageSizeType_NUM];
        statistics::Formula *msg_bytes[MessageSizeType_NUM];

        statistics::Scalar total_msg_count;
        statistics::Scalar total_msg_bytes;
        statistics::Scalar total_data_msg_bytes;
        statistics::Scalar total_msg_wait_time;
        statistics::Scalar total_stall_cy;
        statistics::Scalar total_bw_sat_cy;
        statistics::Formula avg_msg_wait_time;
        statistics::Formula avg_bandwidth;
        statistics::Formula avg_useful_bandwidth;
    } throttleStats;
};

inline std::ostream &
operator<<(std::ostream &out, const Throttle &obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_SIMPLE_THROTTLE_HH__
