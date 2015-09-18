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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_NETWORK_INTERFACE_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_NETWORK_INTERFACE_D_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/OutVcState_d.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "params/GarnetNetworkInterface_d.hh"

class MessageBuffer;
class flitBuffer_d;

class NetworkInterface_d : public ClockedObject, public Consumer
{
  public:
    typedef GarnetNetworkInterface_dParams Params;
    NetworkInterface_d(const Params *p);
    ~NetworkInterface_d();

    void init();

    void addInPort(NetworkLink_d *in_link, CreditLink_d *credit_link);
    void addOutPort(NetworkLink_d *out_link, CreditLink_d *credit_link);

    void wakeup();
    void addNode(std::vector<MessageBuffer *> &inNode,
                 std::vector<MessageBuffer *> &outNode);

    void print(std::ostream& out) const;
    int get_vnet(int vc);
    void init_net_ptr(GarnetNetwork_d *net_ptr) { m_net_ptr = net_ptr; }

    uint32_t functionalWrite(Packet *);

  private:
    GarnetNetwork_d *m_net_ptr;
    const NodeID m_id;
    const int m_virtual_networks, m_vc_per_vnet, m_num_vcs;
    std::vector<OutVcState_d *> m_out_vc_state;
    std::vector<int> m_vc_allocator;
    int m_vc_round_robin; // For round robin scheduling
    flitBuffer_d *outSrcQueue; // For modelling link contention
    flitBuffer_d *creditQueue;

    NetworkLink_d *inNetLink;
    NetworkLink_d *outNetLink;
    CreditLink_d *m_credit_link;
    CreditLink_d *m_ni_credit_link;

    // Input Flit Buffers
    // The flit buffers which will serve the Consumer
    std::vector<flitBuffer_d *>   m_ni_buffers;
    std::vector<Cycles> m_ni_enqueue_time;

    // The Message buffers that takes messages from the protocol
    std::vector<MessageBuffer *> inNode_ptr;
    // The Message buffers that provides messages to the protocol
    std::vector<MessageBuffer *> outNode_ptr;

    bool flitisizeMessage(MsgPtr msg_ptr, int vnet);
    int calculateVC(int vnet);
    void scheduleOutputLink();
    void checkReschedule();
};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_NETWORK_INTERFACE_D_HH__
