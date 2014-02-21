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

#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_NETWORK_LINK_D_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_NETWORK_LINK_D_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/flitBuffer_d.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/network/orion/NetworkPower.hh"
#include "params/NetworkLink_d.hh"
#include "sim/clocked_object.hh"

class GarnetNetwork_d;

class NetworkLink_d : public ClockedObject, public Consumer
{
  public:
    typedef NetworkLink_dParams Params;
    NetworkLink_d(const Params *p);
    ~NetworkLink_d();

    void setLinkConsumer(Consumer *consumer);
    void setSourceQueue(flitBuffer_d *srcQueue);
    void print(std::ostream& out) const{}
    int get_id(){return m_id;}
    void wakeup();

    void calculate_power(double);
    double get_dynamic_power() const { return m_power_dyn; }
    double get_static_power()const { return m_power_sta; }

    unsigned int getLinkUtilization() const { return m_link_utilized; }
    const std::vector<unsigned int> & getVcLoad() const { return m_vc_load; }

    inline bool isReady(Cycles curTime)
    { return linkBuffer->isReady(curTime); }

    inline flit_d* peekLink()       { return linkBuffer->peekTopFlit(); }
    inline flit_d* consumeLink()    { return linkBuffer->getTopFlit(); }

    uint32_t functionalWrite(Packet *);

  private:
    int m_id;
    Cycles m_latency;
    int channel_width;

    flitBuffer_d *linkBuffer;
    Consumer *link_consumer;
    flitBuffer_d *link_srcQueue;
    int m_flit_width;

    // Statistical variables
    unsigned int m_link_utilized;
    std::vector<unsigned int> m_vc_load;

    double m_power_dyn;
    double m_power_sta;
};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_NETWORK_LINK_D_HH__
