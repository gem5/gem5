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

/*
 * This header file is used to define all configuration parameters
 * required by the interconnection network.
 */

#ifndef __MEM_RUBY_NETWORK_GARNET_BASEGARNETNETWORK_HH__
#define __MEM_RUBY_NETWORK_GARNET_BASEGARNETNETWORK_HH__

#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/network/fault_model/FaultModel.hh"
#include "params/BaseGarnetNetwork.hh"

class BaseGarnetNetwork : public Network
{
  public:
    typedef BaseGarnetNetworkParams Params;
    BaseGarnetNetwork(const Params *p);

    void init();
    int getNiFlitSize() const { return m_ni_flit_size; }
    int getVCsPerVnet() const { return m_vcs_per_vnet; }
    bool isFaultModelEnabled() const { return m_enable_fault_model; }
    FaultModel* fault_model;

    void increment_injected_flits(int vnet) { m_flits_injected[vnet]++; }
    void increment_received_flits(int vnet) { m_flits_received[vnet]++; }

    void
    increment_network_latency(Cycles latency, int vnet)
    {
        m_network_latency[vnet] += latency;
    }

    void
    increment_queueing_latency(Cycles latency, int vnet)
    {
        m_queueing_latency[vnet] += latency;
    }

    bool isVNetOrdered(int vnet) const { return m_ordered[vnet]; }

    virtual void regStats();
    virtual void collateStats() {}

  protected:
    int m_ni_flit_size;
    int m_vcs_per_vnet;
    bool m_enable_fault_model;

    // Statistical variables
    Stats::Vector m_flits_received;
    Stats::Vector m_flits_injected;
    Stats::Vector m_network_latency;
    Stats::Vector m_queueing_latency;

    Stats::Formula m_avg_vnet_latency;
    Stats::Formula m_avg_vqueue_latency;
    Stats::Formula m_avg_network_latency;
    Stats::Formula m_avg_queueing_latency;
    Stats::Formula m_avg_latency;
};

#endif // __MEM_RUBY_NETWORK_GARNET_BASEGARNETNETWORK_HH__
