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

#ifndef __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_NETWORK_LINK_HH__
#define __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_NETWORK_LINK_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/FlexibleConsumer.hh"
#include "mem/ruby/network/garnet/flexible-pipeline/flitBuffer.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "params/NetworkLink.hh"
#include "sim/clocked_object.hh"

class GarnetNetwork;

class NetworkLink : public ClockedObject, public Consumer
{
  public:
    typedef NetworkLinkParams Params;
    NetworkLink(const Params *p);
    ~NetworkLink();

    void setLinkConsumer(FlexibleConsumer *consumer);
    void setSourceQueue(flitBuffer *srcQueue);
    flit *peekLink();
    flit *consumeLink();

    void print(std::ostream& out) const {}

    bool is_vc_ready(flit *t_flit);

    int get_id() const { return m_id; }
    void setInPort(int port);
    void setOutPort(int port);
    void wakeup();
    bool isReady();
    void grant_vc_link(int vc, Cycles grant_time);
    void release_vc_link(int vc, Cycles release_time);
    void request_vc_link(int vc, NetDest destination, Cycles request_time);
    bool isBufferNotFull_link(int vc);
    void setSource(FlexibleConsumer *source);
    void init_net_ptr(GarnetNetwork* net_ptr) { m_net_ptr = net_ptr; }

    unsigned int getLinkUtilization() const { return m_link_utilized; }
    const std::vector<unsigned int> & getVcLoad() const { return m_vc_load; }

    bool functionalRead(Packet *);
    uint32_t functionalWrite(Packet *);

  private:
    int m_id;
    Cycles m_latency;
    int m_in_port, m_out_port;
    GarnetNetwork *m_net_ptr;

    flitBuffer *linkBuffer;
    FlexibleConsumer *link_consumer;
    FlexibleConsumer *link_source;
    flitBuffer *link_srcQueue;

    // Statistical variables
    unsigned int m_link_utilized;
    std::vector<unsigned int> m_vc_load;
};

#endif // __MEM_RUBY_NETWORK_GARNET_FLEXIBLE_PIPELINE_NETWORK_LINK_HH__
