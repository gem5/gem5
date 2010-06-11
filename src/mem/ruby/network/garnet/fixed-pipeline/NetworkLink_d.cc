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

#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"

/*
NetworkLink_d::NetworkLink_d(int id)
{
        m_id = id;
        m_latency = 1;
        m_flit_width = NetworkConfig::getFlitSize();

        linkBuffer = new flitBuffer_d();
        m_link_utilized = 0;
        m_vc_load.resize(NetworkConfig::getVCsPerClass()*RubySystem::getNetwork()->getNumberOfVirtualNetworks());

        for(int i = 0; i < NetworkConfig::getVCsPerClass()*RubySystem::getNetwork()->getNumberOfVirtualNetworks(); i++)
                m_vc_load[i] = 0;
}
*/
NetworkLink_d::NetworkLink_d(int id, int link_latency, GarnetNetwork_d *net_ptr)
{
        m_net_ptr = net_ptr;
        m_id = id;
        m_latency = link_latency;
        linkBuffer = new flitBuffer_d();
        m_link_utilized = 0;
        m_vc_load.resize(m_net_ptr->getVCsPerClass()*net_ptr->getNumberOfVirtualNetworks());

        for(int i = 0; i < m_net_ptr->getVCsPerClass()*net_ptr->getNumberOfVirtualNetworks(); i++)
                m_vc_load[i] = 0;
}

NetworkLink_d::~NetworkLink_d()
{
        delete linkBuffer;
}

void NetworkLink_d::setLinkConsumer(Consumer *consumer)
{
        link_consumer = consumer;
}

void NetworkLink_d::setSourceQueue(flitBuffer_d *srcQueue)
{
        link_srcQueue = srcQueue;
}

void NetworkLink_d::wakeup()
{
        if(link_srcQueue->isReady())
        {
                flit_d *t_flit = link_srcQueue->getTopFlit();
                t_flit->set_time(g_eventQueue_ptr->getTime() + m_latency);
                linkBuffer->insert(t_flit);
                g_eventQueue_ptr->scheduleEvent(link_consumer, m_latency);
                m_link_utilized++;
                m_vc_load[t_flit->get_vc()]++;
        }
}

std::vector<int> NetworkLink_d::getVcLoad()
{
        return m_vc_load;
}

int NetworkLink_d::getLinkUtilization()
{
        return m_link_utilized;
}
