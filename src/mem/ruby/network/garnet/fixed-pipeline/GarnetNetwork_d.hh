/*
 * Copyright (c) 2008 Princeton University
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
 *
 * Authors: Niket Agarwal
 */

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_GARNETNETWORK_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_GARNETNETWORK_D_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/network/garnet/BaseGarnetNetwork.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "params/GarnetNetwork_d.hh"

class FaultModel;
class NetworkInterface_d;
class Router_d;
class NetDest;
class NetworkLink_d;
class CreditLink_d;

class GarnetNetwork_d : public BaseGarnetNetwork
{
  public:
    typedef GarnetNetwork_dParams Params;
    GarnetNetwork_d(const Params *p);

    ~GarnetNetwork_d();
    void init();

    int getBuffersPerDataVC() { return m_buffers_per_data_vc; }
    int getBuffersPerCtrlVC() { return m_buffers_per_ctrl_vc; }

    void collateStats();
    void regStats();
    void print(std::ostream& out) const;

    VNET_type
    get_vnet_type(int vc)
    {
        int vnet = vc/getVCsPerVnet();
        return m_vnet_type[vnet];
    }

    // Methods used by Topology to setup the network
    void makeOutLink(SwitchID src, NodeID dest, BasicLink* link,
                     LinkDirection direction,
                     const NetDest& routing_table_entry);
    void makeInLink(NodeID src, SwitchID dest, BasicLink* link,
                    LinkDirection direction,
                    const NetDest& routing_table_entry);
    void makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                          LinkDirection direction,
                          const NetDest& routing_table_entry);

    //! Function for performing a functional write. The return value
    //! indicates the number of messages that were written.
    uint32_t functionalWrite(Packet *pkt);

  private:
    GarnetNetwork_d(const GarnetNetwork_d& obj);
    GarnetNetwork_d& operator=(const GarnetNetwork_d& obj);

    std::vector<VNET_type > m_vnet_type;
    std::vector<Router_d *> m_routers;   // All Routers in Network
    std::vector<NetworkLink_d *> m_links; // All links in the network
    std::vector<CreditLink_d *> m_creditlinks; // All links in net
    std::vector<NetworkInterface_d *> m_nis;   // All NI's in Network

    int m_buffers_per_data_vc;
    int m_buffers_per_ctrl_vc;

    // Statistical variables for performance
    Stats::Scalar m_average_link_utilization;
    Stats::Vector m_average_vc_load;
};

inline std::ostream&
operator<<(std::ostream& out, const GarnetNetwork_d& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_GARNETNETWORK_D_HH__
