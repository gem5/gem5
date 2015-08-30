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

#ifndef __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_ROUTER_HH__
#define __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_ROUTER_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/FlexibleConsumer.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/InVcState.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/NetworkLink.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/OutVcState.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/flitBuffer.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "params/GarnetRouter.hh"

class VCarbiter;

class Router : public BasicRouter, public FlexibleConsumer
{
  public:
    typedef GarnetRouterParams Params;
    Router(const Params *p);

    ~Router();

    void addInPort(NetworkLink *in_link);
    void addOutPort(NetworkLink *out_link, const NetDest& routing_table_entry,
                    int link_weight);
    void wakeup();
    void request_vc(int in_vc, int in_port, NetDest destination,
                    Cycles request_time);
    bool isBufferNotFull(int vc, int inport);
    void grant_vc(int out_port, int vc, Cycles grant_time);
    void release_vc(int out_port, int vc, Cycles release_time);
    void vc_arbitrate();

    void print(std::ostream& out) const;

    void init_net_ptr(GarnetNetwork* net_ptr) 
    { 
        m_net_ptr = net_ptr; 
    }

    bool functionalRead(Packet *);
    uint32_t functionalWrite(Packet *);

  private:
    int m_virtual_networks, m_num_vcs, m_vc_per_vnet;
    GarnetNetwork *m_net_ptr;
    std::vector<int> m_vc_round_robin; // For scheduling of out source queues
    int m_round_robin_inport, m_round_robin_start; // for vc arbitration
    std::vector<int> m_round_robin_invc; // For vc arbitration of each outport

    // These are essentially output buffers
    std::vector<std::vector<flitBuffer *> > m_router_buffers;

    // These are source queues for the output link
    std::vector<flitBuffer *> m_out_src_queue;

    std::vector<NetworkLink *> m_in_link;
    std::vector<NetworkLink *> m_out_link;
    std::vector<std::vector<InVcState *> > m_in_vc_state;
    std::vector<std::vector<OutVcState *> > m_out_vc_state;
    std::vector<NetDest> m_routing_table;
    std::vector<int> m_link_weights;
    VCarbiter *m_vc_arbiter;

    int getRoute(NetDest destination);
    std::vector<int> get_valid_vcs(int invc);
    void routeCompute(flit *m_flit, int inport);
    void checkReschedule();
    void check_arbiter_reschedule();
    void scheduleOutputLinks();
    int get_vnet(int vc) const;
    int get_next_round_robin_vc(int vc) const;
};

#endif // __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_ROUTER_HH__
