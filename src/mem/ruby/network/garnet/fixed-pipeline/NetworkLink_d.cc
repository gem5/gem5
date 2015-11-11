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

#include "mem/ruby/network/garnet/fixed-pipeline/CreditLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"

NetworkLink_d::NetworkLink_d(const Params *p)
    : ClockedObject(p), Consumer(this), m_id(p->link_id),
      m_latency(p->link_latency),
      linkBuffer(new flitBuffer_d()), link_consumer(nullptr),
      link_srcQueue(nullptr), m_link_utilized(0),
      m_vc_load(p->vcs_per_vnet * p->virt_nets)
{
}

NetworkLink_d::~NetworkLink_d()
{
    delete linkBuffer;
}

void
NetworkLink_d::setLinkConsumer(Consumer *consumer)
{
    link_consumer = consumer;
}

void
NetworkLink_d::setSourceQueue(flitBuffer_d *srcQueue)
{
    link_srcQueue = srcQueue;
}

void
NetworkLink_d::wakeup()
{
    if (link_srcQueue->isReady(curCycle())) {
        flit_d *t_flit = link_srcQueue->getTopFlit();
        t_flit->set_time(curCycle() + m_latency);
        linkBuffer->insert(t_flit);
        link_consumer->scheduleEventAbsolute(clockEdge(m_latency));
        m_link_utilized++;
        m_vc_load[t_flit->get_vc()]++;
    }
}

NetworkLink_d *
NetworkLink_dParams::create()
{
    return new NetworkLink_d(this);
}

CreditLink_d *
CreditLink_dParams::create()
{
    return new CreditLink_d(this);
}

uint32_t
NetworkLink_d::functionalWrite(Packet *pkt)
{
    return linkBuffer->functionalWrite(pkt);
}
