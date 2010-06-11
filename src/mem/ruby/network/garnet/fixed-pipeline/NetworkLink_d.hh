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

#ifndef NETWORK_LINK_D_H
#define NETWORK_LINK_D_H

#include <iostream>
#include <vector>

#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/flitBuffer_d.hh"
#include "mem/ruby/network/orion/power_bus.hh"

class GarnetNetwork_d;

class NetworkLink_d : public Consumer {
public:
        //NetworkLink_d(int id);
        ~NetworkLink_d();

        NetworkLink_d(int id, int link_latency, GarnetNetwork_d *net_ptr);
        void setLinkConsumer(Consumer *consumer);
        void setSourceQueue(flitBuffer_d *srcQueue);
        void print(std::ostream& out) const{}
        int getLinkUtilization();
        std::vector<int> getVcLoad();
        int get_id(){return m_id;}
        void wakeup();

    double calculate_offline_power(power_bus*);
    double calculate_power();

        inline bool isReady()
        {
                return linkBuffer->isReady();
        }
        inline flit_d* peekLink()
        {
                return linkBuffer->peekTopFlit();
        }
        inline flit_d* consumeLink()
        {
                return linkBuffer->getTopFlit();
        }

protected:
        int m_id;
        int m_latency;
        GarnetNetwork_d *m_net_ptr;

        flitBuffer_d *linkBuffer;
        Consumer *link_consumer;
        flitBuffer_d *link_srcQueue;
        int m_link_utilized;
        std::vector<int> m_vc_load;
        int m_flit_width;
};

#endif

