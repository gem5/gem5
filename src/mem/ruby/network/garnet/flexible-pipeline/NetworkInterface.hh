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

#ifndef __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_NETWORK_INTERFACE_HH__
#define __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_NETWORK_INTERFACE_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/network/garnet/flexible-pipeline/FlexibleConsumer.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/GarnetNetwork.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/NetworkLink.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/OutVcState.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "params/GarnetNetworkInterface.hh"

class NetworkMessage;
class MessageBuffer;
class flitBuffer;

class NetworkInterface : public ClockedObject, public FlexibleConsumer
{
  public:
    typedef GarnetNetworkInterfaceParams Params;
    NetworkInterface(const Params *p);

    ~NetworkInterface();

    void addInPort(NetworkLink *in_link);
    void addOutPort(NetworkLink *out_link);

    void wakeup();
    void addNode(std::vector<MessageBuffer *> &inNode,
                 std::vector<MessageBuffer *> &outNode);
    void grant_vc(int out_port, int vc, Cycles grant_time);
    void release_vc(int out_port, int vc, Cycles release_time);

    bool isBufferNotFull(int vc, int inport) { return true; }
    void request_vc(int in_vc, int in_port, NetDest destination,
                    Cycles request_time);

    void print(std::ostream& out) const;

    bool functionalRead(Packet *);
    uint32_t functionalWrite(Packet *);

    void init_net_ptr(GarnetNetwork* net_ptr) { m_net_ptr = net_ptr; }

  private:
    GarnetNetwork *m_net_ptr;
    uint32_t m_virtual_networks, m_num_vcs, m_vc_per_vnet;
    NodeID m_id;

    std::vector<OutVcState *> m_out_vc_state;
    std::vector<int> m_vc_allocator;
    int m_vc_round_robin; // For round robin scheduling
    flitBuffer *outSrcQueue; // For modelling link contention

    NetworkLink *inNetLink;
    NetworkLink *outNetLink;

    // Input Flit Buffers

    // The flit buffers which will serve the Consumer
    std::vector<flitBuffer *>   m_ni_buffers;

    // The Message buffers that takes messages from the protocol
    std::vector<MessageBuffer *> inNode_ptr;

    // The Message buffers that provides messages to the protocol
    std::vector<MessageBuffer *> outNode_ptr;

    bool flitisizeMessage(MsgPtr msg_ptr, int vnet);
    int calculateVC(int vnet);
    void scheduleOutputLink();
    void checkReschedule();
};

#endif // __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_NETWORK_INTERFACE_HH__
