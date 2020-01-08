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

#ifndef __MEM_RUBY_NETWORK_SIMPLE_SIMPLENETWORK_HH__
#define __MEM_RUBY_NETWORK_SIMPLE_SIMPLENETWORK_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/network/Network.hh"
#include "params/SimpleNetwork.hh"

class NetDest;
class MessageBuffer;
class Throttle;
class Switch;

class SimpleNetwork : public Network
{
  public:
    typedef SimpleNetworkParams Params;
    SimpleNetwork(const Params *p);
    ~SimpleNetwork() = default;

    void init();

    int getBufferSize() { return m_buffer_size; }
    int getEndpointBandwidth() { return m_endpoint_bandwidth; }
    bool getAdaptiveRouting() {return m_adaptive_routing; }

    void collateStats();
    void regStats();

    bool isVNetOrdered(int vnet) const { return m_ordered[vnet]; }

    // Methods used by Topology to setup the network
    void makeExtOutLink(SwitchID src, NodeID dest, BasicLink* link,
                     const NetDest& routing_table_entry);
    void makeExtInLink(NodeID src, SwitchID dest, BasicLink* link,
                    const NetDest& routing_table_entry);
    void makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                          const NetDest& routing_table_entry,
                          PortDirection src_outport,
                          PortDirection dst_inport);

    void print(std::ostream& out) const;

    bool functionalRead(Packet *pkt);
    uint32_t functionalWrite(Packet *pkt);

  private:
    void addLink(SwitchID src, SwitchID dest, int link_latency);
    void makeLink(SwitchID src, SwitchID dest,
        const NetDest& routing_table_entry, int link_latency);
    void makeTopology();

    // Private copy constructor and assignment operator
    SimpleNetwork(const SimpleNetwork& obj);
    SimpleNetwork& operator=(const SimpleNetwork& obj);

    std::vector<Switch*> m_switches;
    std::vector<MessageBuffer*> m_int_link_buffers;
    int m_num_connected_buffers;
    const int m_buffer_size;
    const int m_endpoint_bandwidth;
    const bool m_adaptive_routing;

    //Statistical variables
    Stats::Formula m_msg_counts[MessageSizeType_NUM];
    Stats::Formula m_msg_bytes[MessageSizeType_NUM];
};

inline std::ostream&
operator<<(std::ostream& out, const SimpleNetwork& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_SIMPLE_SIMPLENETWORK_HH__
